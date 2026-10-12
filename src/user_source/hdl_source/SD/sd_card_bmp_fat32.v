`timescale 1ns/1ps

// ============================================================================
// sd_card_bmp_fat32 —— FAT32 图片显示调度器（顶层的"业务大脑"）
// ============================================================================
//
// 【一句话定位】
//   本模块只做"决策和调度"：哪张图、什么时候加载、写进哪个缓冲、何时切换
//   显示。它不碰 SD 卡引脚、不碰 SDRAM、不懂 FAT——这些"体力活"分别封装在
//   u_image_source（读卡+解码）和 frame_read_write（写帧缓存）里。
//
// 【三条事务链（本模块参与的全部工作）】
//   1. scan 盘点：顶层发 scan_start 脉冲 → u_image_source 扫描 FAT32 根目录
//      → 回报资源表（resource_valid 掩码 + 每张图的首簇/大小）
//      → 顶层在 scan_done 脉冲拍把结果锁存进自己的 img_* 寄存器，
//        形成稳定快照（之后加载不再依赖扫描器内部寄存器）。
//   2. load 读文件：顶层发 load_start 脉冲并指定 load_idx → 经 cluster_lut/
//      size_lut 查表得到首簇/大小 → u_image_source 读卡、沿 FAT 簇链取文件、
//      BMP 解码出像素流；中途它用 write_req 拉 frame_read_write 准备写帧。
//   3. write 写帧：像素流 write_en/write_data → frame_read_write 写 FIFO →
//      SDRAM 四个固定帧缓存。整帧写完以 write_finish_toggle 返回。
//   "文件读完"和"整帧写完"两个事件汇合后，标记该图片可显示。
//   首图立即启显；后续图片顺序预加载，按键只切换已完成的缓存。
//
// 【阅读导航（建议按此顺序读本文件）】
//   1. 端口声明：先分清三类信号——事件(单拍脉冲)/电平(持续状态)/直通线；
//   2. 内部变量声明：每个都有注释说明"谁驱动它、谁消费它"；
//   3. assign 组合观察线：纯连线，无状态；
//   4. function：都是纯组合"查表/计算"，不占时钟、不存状态；
//   5. 唯一的 always 块（本模块全部时序逻辑），按小节读：
//      (a) 复位默认值          (b) SD 未初始化时清空业务状态
//      (c) state_code 可视化   (d) scan_done 时锁存资源表
//      (e) 启动一次扫描        (f) 加载超时保护
//      (g) 双事件汇合→标记完成 (h) 按键解析（短按/长按/自动轮播）
//      (i) 顺序预加载与已完成图片的显示选择
//
// 【速记：reg 与 wire 在本文件里的分工】
//   - reg：在时钟沿更新的"记忆"（触发器）。事件类（*_pulse、*_seen）默认
//     每拍清零、条件满足才置 1；状态类（*_busy、*_en、*_idx）条件变化才变。
//   - wire：连续驱动的"连线"。子模块输出（scan_busy、source_load_*）和
//     assign+function 的组合查表（*_lut、*_from_current、auto_tick）都是。
//
// 本层与旧工程的区别：图片索引从"起始物理扇区"改为 FAT32 目录项提供的
// "起始簇 + 文件大小"；扫描/簇链读取的存储细节全部交给 sd_fat32_image_source，
// 显示时序交给 frame_read_write。
// ============================================================================
module sd_card_bmp_fat32 #(
    // ---- 参数：综合期常量，不占运行时硬件 ----
    // 系统时钟频率；下面的长按/超时周期数都由它换算。
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    // 以下三个是旧版"物理扇区扫描"接口的残留参数，FAT32 版不使用，
    // 保留在端口/参数表里只为顶层例化兼容。
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd4,
    // 自动键按住超过该毫秒数判为"长按"（长按触发"上一张"）。
    parameter integer LONG_PRESS_MS     = 1000,
    // 一次加载的最长允许时间；超过则发 load_abort 并报错（防坏卡永久卡死）。
    parameter integer LOAD_TIMEOUT_MS   = 5_000,
    parameter integer PCM_ENABLE = 0
) (
    // ===================== 端口：系统时钟/复位 =====================
    input  wire        clk,            // 系统时钟 100MHz（顶层接 sd_card_clk）
    input  wire        rst,            // 高有效复位
    input wire pixel_clk, pcm_ready,
    output wire signed [15:0] pcm_sample,
    output wire pcm_valid, pcm_active, pcm_end, pcm_underflow,
    output wire [7:0] pcm_error,
    output reg pcm_mode,

    // ===================== 端口：按键（低有效原始输入，内部会同步+去抖） =====================
    input  wire        key_next,       // "下一张"按键
    input  wire        key_auto,       // "自动"键：短按=轮播开/关，长按=上一张
    input  wire        key_audio_mode, // 板上 KEY3：短按切换 PCM/琶音

    // bmp_width/bmp_height 与旧版扫描参数一样仅为顶层兼容保留；FAT32 目录
    // 扫描提供图片列表，bmp24_decoder 内部校验固定的 640x480 格式。
    input  wire [15:0] bmp_width,      // 期望图片宽（不参与逻辑，兼容旧接口）
    input  wire [15:0] bmp_height,     // 期望图片高（不参与逻辑，兼容旧接口）

    // ===================== 端口：显示控制 =====================
    // 第一张图"文件读完+整帧写完"都完成后才置 1；=0 时 HDMI 输出黑色，
    // 防止帧缓存还没准备好就显示随机数据。
    output reg         display_valid,  // 控制 HDMI 输出的开关
    // 数码管/调试用状态码：0 初始化，1 扫描，2 等待写端，3 读文件，4 空闲，F 错误。
    output wire [3:0]  state_code,

    // ===================== 端口：帧写完成事件（跨时钟域） =====================
    // 来自 frame_read_write 所在时钟域（ext_mem_clk 125MHz 域）；每写完一帧
    // 翻转一次 0/1。本模块用三级移位同步后做边沿检测，恢复成单拍事件。
    input  wire        write_finish_toggle,
    // 四个固定帧缓存：逻辑图片索引与 SDRAM 缓冲编号一一对应。
    output reg  [1:0]  write_buf_idx,  // 本次加载写进哪块 SDRAM 缓冲
    output reg  [1:0]  disp_buf_idx,   // 请求 HDMI 在下一帧显示哪块 SDRAM 缓冲

    // ===================== 端口：写帧握手与像素流（对 u_image_source 的直通线） =====================
    // 这四条在 u_image_source 与 frame_read_write 之间传递，本模块只做透明
    // 转发，不改值、不缓存。
    output wire        write_req,      // "我要写一整帧"请求（电平，保持到 ack）
    input  wire        write_req_ack,  // "写 FIFO 已清空、地址/长度已锁存，开始灌"
    output wire        write_en,       // =1 时 write_data 是一个有效像素
    output wire [31:0] write_data,     // {R,G,B,8'd0}，32 位对齐的像素字
    output wire        write_frame_commit, // 最后一个 BMP 像素已输出

    // ===================== 端口：SD 卡 SPI 引脚（直通 u_image_source） =====================
    output wire        SD_nCS,         // SPI 片选（低有效）
    output wire        SD_DCLK,        // SPI 时钟
    output wire        SD_MOSI,        // FPGA→TF 卡 串行数据
    input  wire        SD_MISO,        // TF 卡→FPGA 串行数据

    // ===================== 端口：USB-UART 调试观察线 =====================
    // 都是当前控制器状态的只读镜像，供 50MHz 日志时钟域采样，不参与控制。
    output wire        debug_auto_enabled, // 模式只读输出，供OSD安全跨域使用
    output wire        debug_sd_init_done,
    output wire        debug_scan_busy,
    output wire        debug_scan_done,
    output wire [7:0]  debug_scan_error,
    output wire        debug_source_ready,
    output wire        debug_load_busy,
    output wire [7:0]  debug_load_error,
    // FAT/SD 细粒度诊断观察口，供顶层 UART 周期打印。
    output wire [3:0]  debug_scanner_state,
    output wire [31:0] debug_sector_lba,
    output wire [8:0]  debug_sector_byte_count,
    output wire        debug_sector_active,
    output wire [15:0] debug_sector_done_count,
    output wire [3:0]  debug_resource_valid,
    output wire [4:0]  debug_sd_state,
    output wire        debug_sd_cmd_error,
    output wire        debug_sd_cmd_error_latched
);
    // ========================================================================
    // 内部 wire：按键去抖结果
    // ========================================================================
    // 去抖模块输出的单拍按下事件，以及自动键的稳定电平。
    wire key_next_press;     // "下一张"的单拍按下脉冲（去抖后）
    wire key_next_stable;
    reg next_down_seen, next_release_pulse;
    wire key_audio_mode_press;
    wire key_auto_press;     // "自动"键的单拍按下脉冲（去抖后）
    wire key_auto_stable;    // "自动"键稳定电平（1=松开，0=按住；长按计时用）

    // ========================================================================
    // 内部 wire：来自 u_image_source 的状态与扫描结果（实例输出端口驱动）
    // ========================================================================
    // image_source 的扫描/加载状态和资源结果。
    wire sd_init_done;       // 电平：SD 卡 SPI 初始化完成
    wire source_ready;       // 电平：source 空闲且未在扫描（可接受新事务）
    wire source_load_busy;   // 电平：source 内部正在读文件（粒度比顶层 load_busy 细）
    wire source_load_done;   // 事件：文件读完+解码完（单拍脉冲）
    wire [7:0] source_load_error; // 本次加载的错误码（0=成功）
    wire scan_busy;          // 电平：source 正在扫描 FAT32 目录
    wire scan_done;          // 事件：扫描结束（单拍脉冲，此刻结果寄存器已稳定）
    wire [7:0] scan_error;   // 扫描错误码（0=成功）
    // 扫描结果的资源表（scan_done 脉冲时有效，随后被锁存进顶层 img_* 寄存器）。
    wire [3:0] resource_valid;    // 掩码：bit0~3 分别表示 PIC1~PIC4.BMP 是否找到
    wire [31:0] resource0_cluster, resource1_cluster; // PIC1/PIC2 的首簇号
    wire [31:0] resource2_cluster, resource3_cluster; // PIC3/PIC4 的首簇号
    wire [31:0] resource0_size, resource1_size;       // PIC1/PIC2 的字节数
    wire [31:0] resource2_size, resource3_size;       // PIC3/PIC4 的字节数
    // u_image_source 内部 FAT/SD 诊断线的本层别名。
    wire [3:0]  source_debug_scanner_state;
    wire [31:0] source_debug_sector_lba;
    wire [8:0]  source_debug_sector_byte_count;
    wire        source_debug_sector_active;
    wire [15:0] source_debug_sector_done_count;
    wire [3:0]  source_debug_resource_valid;
    wire [4:0]  source_debug_sd_state;
    wire        source_debug_sd_cmd_error;
    wire        source_debug_sd_cmd_error_latched;

    // ========================================================================
    // 内部 reg：发给 u_image_source 的单拍控制事件
    // ========================================================================
    // 这三个是"事件"型寄存器：默认每拍清零，条件满足的那一拍置 1。
    reg scan_start_pulse;    // 事件："开始扫描"命令（复位后只发一次）
    reg load_start_pulse;    // 事件："开始加载"命令（仲裁链的最终输出）
    reg load_abort_pulse;    // 事件："中止加载"命令（超时保护用）

    // ========================================================================
    // 内部 reg：扫描相关记忆
    // ========================================================================
    reg scan_kicked;         // 电平：已发过扫描启动脉冲（防止重复启动扫描）
    // 电平：扫描已结束（scan_done 脉冲后锁存；日志时钟域采样它是安全的）。
    reg scan_completed;

    // ========================================================================
    // 内部 reg：首图/轮播相关
    // ========================================================================
    // 电平：首图已经同时完成"文件读取 + 帧写入"并提交显示。
    reg [3:0] image_complete; // 每位表示对应图片已解码且完整写入 SDRAM
    reg [2:0] preload_next_idx; // 下一张待预加载图片；4 表示四张均已尝试
    reg preload_halted; // 源文件/写帧失败后暂停，避免复用尚未结束的写通道
    reg auto_play_en;        // 电平：自动轮播开关
    assign debug_auto_enabled = auto_play_en;
    // 轮播定时器（以 clk 周期计数；计满 1 秒产生 auto_tick 后清零）。
    reg [31:0] auto_cnt;

    // ========================================================================
    // 内部 reg：图片清单（扫描结果的稳定快照）
    // ========================================================================
    // 当前有效图片数量（0~4）和当前/待加载图片的逻辑索引。
    // 逻辑索引是"紧凑索引"：资源槽有空洞时按有效槽重新编号（0~count-1）。
    reg [2:0] img_found_count;
    reg [1:0] img_idx;       // 当前显示图片的逻辑索引
    reg [1:0] load_idx;      // 本次加载目标的逻辑索引（查表线 load_*_lut 的输入）
    // 资源表：按扫描结果保存每张图片的首簇（文件在卡上的物理起点）。
    reg [31:0] img_cluster0, img_cluster1, img_cluster2, img_cluster3;
    // 资源表：按扫描结果保存每张图片的字节数。
    reg [31:0] img_size0, img_size1, img_size2, img_size3;
    // 资源槽有效位的锁存副本；cluster_lut/size_lut 靠它跳过无效槽形成紧凑索引。
    reg [3:0] image_resource_valid;

    // ========================================================================
    // 内部 reg：按键请求挂起位
    // ========================================================================
    // 忙时先记住请求，空闲后再执行——防止按键事件直接重入同一个 SD/FAT 事务。
    reg next_req_pending;
    reg prev_req_pending;

    // ========================================================================
    // 内部 reg：自动键的长按/短按判别
    // ========================================================================
    // 电平：本次按住期间已记录过"按下"事件（松开时据此区分短按/长按）。
    reg key_auto_down_seen;
    reg [31:0] long_press_cnt; // 按住计时器（键松开时清零）
    // 电平：本次按住已触发过"长按=上一张"（防止一次长按重复触发）。
    reg long_press_marked;

    // ========================================================================
    // 内部 reg：加载事务状态（业务级）
    // ========================================================================
    // 顶层业务级"加载中"：从发 load_start 起，到"文件读完+整帧写完"双事件
    // 汇合为止。与 image_source 的底层 load_busy 不同：这里还包含等待帧写
    // 完成的时间。
    reg load_busy;
    reg [31:0] load_timeout_cnt; // 超时看门狗计数器（load_busy 期间递增）
    // 两个独立完成事件的记忆位，允许它们任意先后到达。
    reg source_done_seen;    // 已见到"文件读完"事件
    reg write_done_seen;     // 已见到"整帧写完"事件
    reg [7:0] last_error;    // 最近一次事务的错误码（数码管/UART 日志显示用）

    // ========================================================================
    // 内部 reg：跨域 toggle 同步链与状态码
    // ========================================================================
    // 对 write_finish_toggle 做三级移位同步；第 2/1 位异或得到单拍边沿事件。
    reg [2:0] wrfin_tgl_sync;
    // state_code 的寄存器版本；state_code 只是可视化编码，不参与控制。
    reg [3:0] state_code_r;

    // ========================================================================
    // 内部 wire：组合事件/查表结果（全部由 assign 连续驱动，无记忆）
    // ========================================================================
    wire write_finish_pulse;     // 事件：帧写完成（toggle 同步后的单拍边沿）
    wire auto_tick;              // 事件：轮播定时器计满 1 秒
    wire [2:0] next_from_current; // {找到其他已加载图片, 索引}
    wire [2:0] prev_from_current;
    wire [31:0] load_cluster_lut; // 查表：load_idx 对应图片的首簇号
    wire [31:0] load_size_lut;    // 查表：load_idx 对应图片的字节数

    // ========================================================================
    // 综合期常量：毫秒参数换算为时钟周期数
    // ========================================================================
    localparam integer LONG_PRESS_CYCLES =
        (CLK_FREQ_HZ / 1000) * LONG_PRESS_MS;
    localparam integer LOAD_TIMEOUT_CYCLES =
        (CLK_FREQ_HZ / 1000) * LOAD_TIMEOUT_MS;

    // ========================================================================
    // 组合观察线：不额外保存状态，只把内部寄存器/事件映射给端口。
    // ========================================================================
    assign state_code = state_code_r;
    assign debug_sd_init_done = sd_init_done;
    assign debug_scan_busy    = scan_busy;
    // scan_completed 在扫描结束脉冲之后才锁存，所以从 50MHz 日志时钟域采样是安全的。
    assign debug_scan_done    = scan_completed;
    assign debug_scan_error   = last_error;
    assign debug_source_ready = source_ready;
    assign debug_load_busy    = load_busy;
    assign debug_load_error   = source_load_error;
    assign debug_scanner_state = source_debug_scanner_state;
    assign debug_sector_lba = source_debug_sector_lba;
    assign debug_sector_byte_count = source_debug_sector_byte_count;
    assign debug_sector_active = source_debug_sector_active;
    assign debug_sector_done_count = source_debug_sector_done_count;
    assign debug_resource_valid = source_debug_resource_valid;
    assign debug_sd_state = source_debug_sd_state;
    assign debug_sd_cmd_error = source_debug_sd_cmd_error;
    assign debug_sd_cmd_error_latched = source_debug_sd_cmd_error_latched;
    // toggle 经过同步后，连续两级不同表示发生了一个跨域事件（异或=边沿检测）。
    assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];
    // 轮播定时器每计满 CLK_FREQ_HZ 拍（=1 秒）产生一拍 auto_tick。
    assign auto_tick = (auto_cnt == CLK_FREQ_HZ - 1);

    // ========================================================================
    // 实例：按键去抖（key_next）
    // ========================================================================
    // 每个按键先经过两级触发器同步，再用稳定计数器去抖；只在稳定按下时
    // 产生一个 press_pulse，避免机械抖动被解释为多次切图。
    key_press_debounce_fat32 #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .DEBOUNCE_MS(50)
    ) u_key_next (
        .clk(clk), .rst(rst), .button_in(key_next),
        .press_pulse(key_next_press), .button_stable_out(key_next_stable)
    );

    // ========================================================================
    // 实例：按键去抖（key_auto）
    // ========================================================================
    // 与上一个完全相同；多取一个 button_stable_out 稳定电平供长按计时。
    key_press_debounce_fat32 #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .DEBOUNCE_MS(50)
    ) u_key_auto (
        .clk(clk), .rst(rst), .button_in(key_auto),
        .press_pulse(key_auto_press), .button_stable_out(key_auto_stable)
    );

    // 独立音源键：同步并消抖，只在稳定按下时产生一次切换事件。
    key_press_debounce_fat32 #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .DEBOUNCE_MS(50)
    ) u_key_audio_mode (
        .clk(clk), .rst(rst), .button_in(key_audio_mode),
        .press_pulse(key_audio_mode_press), .button_stable_out()
    );

    // 板上 KEY2 松开切图；KEY3 短按切音源，按住不会重复切换。
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pcm_mode <= 0;
            next_down_seen <= 0; next_release_pulse <= 0;
        end else begin
            next_release_pulse <= 0;
            if (PCM_ENABLE && key_audio_mode_press) pcm_mode <= ~pcm_mode;
            if (key_next_press) next_down_seen <= 1;
            if (key_next_stable && next_down_seen) begin
                next_release_pulse <= 1;
                next_down_seen <= 0;
            end
        end
    end

    // ========================================================================
    // 实例：u_image_source —— 存储/解码子系统（本模块的"手"）
    // ========================================================================
    // 顶层只传递"扫描/加载"和"帧写入"握手，存储细节全部在它内部：
    //   sd_card_top(SPI 驱动) → 扇区适配器 → 扫描器/文件流 → BMP 解码。
    sd_fat32_image_source #(.PCM_ENABLE(PCM_ENABLE)) u_image_source (
        .clk(clk), .rst(rst),
        .pixel_clk(pixel_clk), .pcm_ready(pcm_ready), .pcm_sample(pcm_sample),
        .pcm_valid(pcm_valid), .pcm_active(pcm_active), .pcm_end(pcm_end),
        .pcm_underflow(pcm_underflow), .pcm_error(pcm_error),
        .pcm_play_enable(scan_completed && !load_busy &&
                         (preload_halted || preload_next_idx >= img_found_count)),
        // ---- 扫描事务：顶层发脉冲，source 回报 busy/结果 ----
        .scan_start(scan_start_pulse), .scan_busy(scan_busy),
        .scan_done(scan_done), .scan_error(scan_error),
        .resource_valid(resource_valid),
        .resource0_cluster(resource0_cluster), .resource1_cluster(resource1_cluster),
        .resource2_cluster(resource2_cluster), .resource3_cluster(resource3_cluster),
        .resource0_size(resource0_size), .resource1_size(resource1_size),
        .resource2_size(resource2_size), .resource3_size(resource3_size),
        // ---- 加载事务：顶层给"哪张图"（查表结果），source 回报进度 ----
        .load_start(load_start_pulse), .load_abort(load_abort_pulse),
        .load_cluster(load_cluster_lut), .load_size(load_size_lut),
        .source_ready(source_ready), .load_busy(source_load_busy),
        .load_done(source_load_done), .load_error(source_load_error),
        // ---- 写帧握手/像素流（直通 frame_read_write）----
        .write_req(write_req), .write_req_ack(write_req_ack),
        .write_en(write_en), .write_data(write_data),
        .frame_commit(write_frame_commit),
        // ---- SD 卡物理引脚 ----
        .sd_init_done(sd_init_done),
        .SD_nCS(SD_nCS), .SD_DCLK(SD_DCLK), .SD_MOSI(SD_MOSI), .SD_MISO(SD_MISO),
        .debug_scanner_state(source_debug_scanner_state),
        .debug_sector_lba(source_debug_sector_lba),
        .debug_sector_byte_count(source_debug_sector_byte_count),
        .debug_sector_active(source_debug_sector_active),
        .debug_sector_done_count(source_debug_sector_done_count),
        .debug_resource_valid(source_debug_resource_valid),
        .debug_sd_state(source_debug_sd_state),
        .debug_sd_cmd_error(source_debug_sd_cmd_error),
        .debug_sd_cmd_error_latched(source_debug_sd_cmd_error_latched)
    );

    // ========================================================================
    // function 1：数一数有效图片有多少张
    // ========================================================================
    // 把四位资源有效掩码转换成有效图片数量。函数是纯组合逻辑——调用它不占
    // 一个时钟、不会"执行一次后保存结果"，综合后就是一张 4 选 1 译码表。
    function [2:0] resource_count;
        input [3:0] valid;
        begin
            case (valid)
                4'b0000: resource_count = 3'd0;
                4'b0001,4'b0010,4'b0100,4'b1000: resource_count = 3'd1;
                4'b0011,4'b0101,4'b0110,4'b1001,4'b1010,4'b1100:
                    resource_count = 3'd2;
                4'b0111,4'b1011,4'b1101,4'b1110: resource_count = 3'd3;
                default: resource_count = 3'd4;
            endcase
        end
    endfunction

    // 从已加载的紧凑图片索引中寻找另一张，不把尚在写入的帧交给显示端。
    function [2:0] next_loaded;
        input [1:0] cur;
        input [2:0] count;
        input [3:0] complete;
        integer step;
        integer candidate;
        begin
            next_loaded = {1'b0, cur};
            for (step = 1; step < 4; step = step + 1) begin
                candidate = cur + step;
                if (candidate >= count)
                    candidate = candidate - count;
                if (step < count && !next_loaded[2] && complete[candidate])
                    next_loaded = {1'b1, candidate[1:0]};
            end
        end
    endfunction

    function [2:0] prev_loaded;
        input [1:0] cur;
        input [2:0] count;
        input [3:0] complete;
        integer step;
        integer candidate;
        begin
            prev_loaded = {1'b0, cur};
            for (step = 1; step < 4; step = step + 1) begin
                candidate = cur + count - step;
                if (candidate >= count)
                    candidate = candidate - count;
                if (step < count && !prev_loaded[2] && complete[candidate])
                    prev_loaded = {1'b1, candidate[1:0]};
            end
        end
    endfunction

    // ========================================================================
    // function 5：逻辑索引 → 首簇号查表（紧凑映射）
    // ========================================================================
    // 把"紧凑图片索引"映射到扫描器的资源槽首簇；跳过无效目录项。
    // 例：有效掩码 4'b1010（只有 PIC2/PIC4 在卡上）时：
    //   逻辑索引 0 → img_cluster1（PIC2 的首簇）
    //   逻辑索引 1 → img_cluster3（PIC4 的首簇）
    // 硬件上综合成"比较器+多路选择器"树；found 只是组合计算的草稿纸，不生成
    // 任何寄存器。
    function [31:0] cluster_lut;
        input [1:0] index;
        input [3:0] image_resource_valid;
        input [31:0] img_cluster0, img_cluster1, img_cluster2, img_cluster3;
        integer found;
        begin
            cluster_lut = 32'd0;
            found = 0;
            if (image_resource_valid[0]) begin
                if (found == index) cluster_lut = img_cluster0;
                found = found + 1;
            end
            if (image_resource_valid[1]) begin
                if (found == index) cluster_lut = img_cluster1;
                found = found + 1;
            end
            if (image_resource_valid[2]) begin
                if (found == index) cluster_lut = img_cluster2;
                found = found + 1;
            end
            if (image_resource_valid[3]) begin
                if (found == index) cluster_lut = img_cluster3;
                found = found + 1;
            end
        end
    endfunction

    // ========================================================================
    // function 6：逻辑索引 → 文件大小查表（与 cluster_lut 完全相同的压缩映射）
    // ========================================================================
    function [31:0] size_lut;
        input [1:0] index;
        input [3:0] image_resource_valid;
        input [31:0] img_size0, img_size1, img_size2, img_size3;
        integer found;
        begin
            size_lut = 32'd0;
            found = 0;
            if (image_resource_valid[0]) begin
                if (found == index) size_lut = img_size0;
                found = found + 1;
            end
            if (image_resource_valid[1]) begin
                if (found == index) size_lut = img_size1;
                found = found + 1;
            end
            if (image_resource_valid[2]) begin
                if (found == index) size_lut = img_size2;
                found = found + 1;
            end
            if (image_resource_valid[3]) begin
                if (found == index) size_lut = img_size3;
                found = found + 1;
            end
        end
    endfunction

    // ========================================================================
    // 组合查表线的实例化：函数输出 → wire（assign 连续驱动）
    // ========================================================================
    // 函数输出组成的组合查表线，供下一次启动时采样。
    // 这些线无记忆：img_idx/load_idx 一翻新，输出当场跟着变（同拍可用）。
    assign next_from_current = next_loaded(img_idx, img_found_count, image_complete);
    assign prev_from_current = prev_loaded(img_idx, img_found_count, image_complete);
    // Explicit arguments ensure table updates also reevaluate index 0 after
    // scan_done, even when load_idx itself has not changed since reset.
    assign load_cluster_lut = cluster_lut(load_idx, image_resource_valid,
        img_cluster0, img_cluster1, img_cluster2, img_cluster3);
    assign load_size_lut = size_lut(load_idx, image_resource_valid,
        img_size0, img_size1, img_size2, img_size3);

    // ========================================================================
    // 本模块唯一的时序块：顶层业务控制中心
    // ========================================================================
    // 全部状态都集中在这一个 always 块里，避免多个 always 块同时写同一个
    // 寄存器。每拍先产生默认脉冲清零，再根据当前状态/事件设置下一值。
    // 注意：块内所有 <= 都在同一时钟沿"先算后提交"，条件判断读到的都是旧值。
    always @(posedge clk or posedge rst) begin
        // ------------------------------------------------------------------
        // (a) 复位：所有寄存器回到安全默认值
        // ------------------------------------------------------------------
        if (rst) begin
            wrfin_tgl_sync <= 3'b000;        // 同步链清零（无历史事件）
            // 三个单拍事件默认不触发
            scan_start_pulse <= 1'b0;
            load_start_pulse <= 1'b0;
            load_abort_pulse <= 1'b0;
            // 扫描/首图/轮播状态复位
            scan_kicked <= 1'b0;
            scan_completed <= 1'b0;
            image_complete <= 4'b0000;
            preload_next_idx <= 3'd0;
            preload_halted <= 1'b0;
            auto_play_en <= 1'b0;
            auto_cnt <= 32'd0;
            // 图片清单复位：0 张图、索引 0
            img_found_count <= 3'd0;
            img_idx <= 2'd0;
            load_idx <= 2'd0;
            // 四个固定缓冲复位：首图默认使用 Buffer 0
            write_buf_idx <= 2'd0;
            disp_buf_idx <= 2'd0;
            // 资源表复位（首簇/大小全 0、无有效槽）
            img_cluster0 <= 0; img_cluster1 <= 0;
            img_cluster2 <= 0; img_cluster3 <= 0;
            img_size0 <= 0; img_size1 <= 0;
            img_size2 <= 0; img_size3 <= 0;
            image_resource_valid <= 4'd0;
            // 按键请求/长按判别复位
            next_req_pending <= 1'b0;
            prev_req_pending <= 1'b0;
            key_auto_down_seen <= 1'b0;
            long_press_cnt <= 0;
            long_press_marked <= 1'b0;
            // 加载事务/超时/事件记忆复位
            load_busy <= 1'b0;
            load_timeout_cnt <= 0;
            source_done_seen <= 1'b0;
            write_done_seen <= 1'b0;
            last_error <= 8'h00;
            state_code_r <= 4'd0;            // 状态码显示 0（初始化）
            display_valid <= 1'b0;           // 黑屏
        end else begin
            // ------------------------------------------------------------------
            // 每拍开头的固定动作
            // ------------------------------------------------------------------
            // 同步外部 toggle；三级链降低亚稳态传播风险，但会引入几拍延迟。
            wrfin_tgl_sync <= {wrfin_tgl_sync[1:0], write_finish_toggle};
            // 三个单拍事件默认每拍清零；下面条件满足才置 1，因此最多维持一拍。
            scan_start_pulse <= 1'b0;
            load_start_pulse <= 1'b0;
            load_abort_pulse <= 1'b0;

            // ------------------------------------------------------------------
            // (b) SD 卡还没初始化完成：清空全部业务状态，回到"等初始化"
            // ------------------------------------------------------------------
            if (!sd_init_done) begin
                // SD 尚未初始化时，清除所有业务状态。初始化完成后才允许扫描。
                scan_kicked <= 1'b0;
                scan_completed <= 1'b0;
                image_complete <= 4'b0000;
                preload_next_idx <= 3'd0;
                preload_halted <= 1'b0;
                auto_play_en <= 1'b0;
                auto_cnt <= 0;
                img_found_count <= 0;
                image_resource_valid <= 0;
                img_idx <= 0;
                load_idx <= 0;
                write_buf_idx <= 0;
                disp_buf_idx <= 0;
                next_req_pending <= 0;
                prev_req_pending <= 0;
                load_busy <= 0;
                load_timeout_cnt <= 0;
                source_done_seen <= 0;
                write_done_seen <= 0;
                display_valid <= 1'b0;
                state_code_r <= 4'd0;
            end else begin
                // --------------------------------------------------------------
                // (c) state_code：只是对当前并发事务的可视化编码，不参与控制
                // --------------------------------------------------------------
                // 优先级从高到低：扫描中(1) > 读文件中(3) > 等写端(2) > 出错(F) > 空闲(4)
                if (scan_busy)
                    state_code_r <= 4'd1;
                else if (load_busy && source_load_busy)
                    state_code_r <= 4'd3;
                else if (load_busy)
                    state_code_r <= 4'd2;
                else if (last_error != 0)
                    state_code_r <= 4'hf;
                else
                    state_code_r <= 4'd4;

                // --------------------------------------------------------------
                // (d) 扫描结束：把扫描结果锁存成顶层资源表（稳定快照）
                // --------------------------------------------------------------
                if (scan_done) begin
                    // scan_done 是单拍事件；资源数据同时被锁存到顶层资源表，
                    // 后续加载不再直接依赖 scanner 内部寄存器是否改变。
                    scan_completed <= 1'b1;
                    img_cluster0 <= resource0_cluster;
                    img_cluster1 <= resource1_cluster;
                    img_cluster2 <= resource2_cluster;
                    img_cluster3 <= resource3_cluster;
                    img_size0 <= resource0_size;
                    img_size1 <= resource1_size;
                    img_size2 <= resource2_size;
                    img_size3 <= resource3_size;
                    image_resource_valid <= resource_valid;
                    img_found_count <= resource_count(resource_valid);
                    preload_next_idx <= 3'd0;
                    image_complete <= 4'b0000;
                    preload_halted <= 1'b0;
                    last_error <= scan_error;
                end

                // --------------------------------------------------------------
                // (e) 启动一次扫描（复位后只发一次 scan_start_pulse）
                // --------------------------------------------------------------
                if (!scan_kicked && source_ready) begin
                    // source_ready 是扫描/加载共享通道空闲的许可；只发一次启动脉冲。
                    scan_start_pulse <= 1'b1;
                    scan_kicked <= 1'b1;             // 之后不再重复触发
                    scan_completed <= 1'b0;
                    img_found_count <= 0;
                    display_valid <= 1'b0;           // 扫描期间保持黑屏
                    image_complete <= 4'b0000;
                    preload_next_idx <= 3'd0;
                    preload_halted <= 1'b0;
                    last_error <= 0;
                end

                // --------------------------------------------------------------
                // (f) 加载超时保护（看门狗）
                // --------------------------------------------------------------
                if (load_busy) begin
                    // 加载超时保护。硬件状态机必须为"外设永不响应"准备出口，
                    // 否则一个坏卡就可能让系统永久停在某个 busy 状态。
                    if (source_load_done) begin
                        // 文件正常读完：清零看门狗；按错误码分两条路
                        load_timeout_cnt <= 0;
                        if (source_load_error == 0) begin
                            // 读成功：记下"文件完成"事件，继续等帧写完成
                            source_done_seen <= 1'b1;
                        end else begin
                            // 写状态机可能仍在等像素；不再启动下一笔，避免混帧。
                            load_busy <= 1'b0;
                            source_done_seen <= 1'b0;
                            write_done_seen <= 1'b0;
                            preload_halted <= 1'b1;
                            last_error <= source_load_error;
                        end
                    end else if (load_timeout_cnt >= LOAD_TIMEOUT_CYCLES) begin
                        // 超时：强制结束并给 source 发中止脉冲
                        load_busy <= 1'b0;
                        load_timeout_cnt <= 0;
                        source_done_seen <= 0;
                        write_done_seen <= 0;
                        load_abort_pulse <= 1'b1;
                        preload_halted <= 1'b1;
                        last_error <= 8'he0;         // e0 = 超时错误码
                    end else begin
                        load_timeout_cnt <= load_timeout_cnt + 1'b1;
                    end
                end else begin
                    load_timeout_cnt <= 0;           // 不忙时看门狗清零
                end

                // --------------------------------------------------------------
                // (g) 双事件汇合：文件读完 + 帧写完 → 标记固定缓冲可显示
                // --------------------------------------------------------------
                if (load_busy && source_load_done && source_load_error == 0) begin
                    // source_load_done 表示解码器已经把文件字节读完；
                    // 帧写完成事件单独由跨域 toggle 提供。
                    source_done_seen <= 1'b1;
                end
                if (load_busy && write_finish_pulse)
                    write_done_seen <= 1'b1;

                // 两个事件可以任意先后到达，使用"当前值或本拍事件"判断，
                // 避免非阻塞赋值延后一拍造成提交条件漏检。
                // 文件源完成与帧写完成是两个并发事件，任意先后都可以；只有
                // 两者都见到才允许显示该缓存。这是典型的"事件汇合"设计。
                if (load_busy &&
                    (source_done_seen ||
                     (source_load_done && source_load_error == 0)) &&
                    (write_done_seen || write_finish_pulse)) begin
                    // 图片和 buffer 使用同一个紧凑索引，后续不再覆盖它。
                    load_busy <= 1'b0;
                    source_done_seen <= 1'b0;
                    write_done_seen <= 1'b0;
                    load_timeout_cnt <= 0;
                    image_complete[load_idx] <= 1'b1;
                    preload_next_idx <= {1'b0, load_idx} + 3'd1;
                    if (!display_valid) begin
                        // 首张完整图片立即可显示，其余图片继续在后台预加载。
                        disp_buf_idx <= load_idx;
                        img_idx <= load_idx;
                        display_valid <= 1'b1;
                    end
                    last_error <= 0;
                end

                // --------------------------------------------------------------
                // (h) 按键解析 + (i) 加载仲裁链（只有扫描完成且有图才处理）
                // --------------------------------------------------------------
                if (scan_completed && img_found_count != 0) begin
                    // 按键只选择已驻留图片，不再发起 SD 文件读取。

                    // ---- (h1) 按键事件 → 挂起请求 ----
                    if (next_release_pulse)
                        next_req_pending <= 1'b1;    // "下一张"先记着
                    if (key_auto_press)
                        key_auto_down_seen <= 1'b1;  // 记录本次按住有按下事件

                    // ---- (h2) 自动键：按住计时（长按判定）与松开处理 ----
                    if (!key_auto_stable) begin
                        // 自动键稳定为低时计时；达到长按门限后产生一次上一张请求。
                        if (long_press_cnt < LONG_PRESS_CYCLES - 1)
                            long_press_cnt <= long_press_cnt + 1'b1;
                        else if (!long_press_marked) begin
                            // 到达门限且本次还没触发过：长按 = 上一张
                            long_press_marked <= 1'b1;
                            if (img_found_count > 1)
                                prev_req_pending <= 1'b1;
                        end
                    end else begin
                        // 键松开：计时清零
                        long_press_cnt <= 0;
                        if (key_auto_down_seen) begin
                            // 本次按住期间有过按下且没触发长按 → 判为短按：
                            // 切换自动轮播开关
                            if (!long_press_marked && img_found_count > 1) begin
                                auto_play_en <= ~auto_play_en;
                                auto_cnt <= 0;
                            end
                            key_auto_down_seen <= 1'b0;
                        end
                        long_press_marked <= 1'b0;   // 松开后允许下次重新判定
                    end

                    // ---- (h3) 轮播定时器：只在有画面、不忙、开启时计时 ----
                    if (auto_play_en && display_valid && img_found_count > 1) begin
                        if (auto_tick) auto_cnt <= 0;    // 计满 1 秒清零重计
                        else auto_cnt <= auto_cnt + 1'b1;
                    end else begin
                        auto_cnt <= 0;                   // 否则暂停并清零
                    end

                    // 文件读取只有这一处启动：按目录中的紧凑索引依次预加载。
                    if (source_ready && !load_busy && !preload_halted &&
                        preload_next_idx < img_found_count) begin
                        load_idx <= preload_next_idx[1:0];
                        write_buf_idx <= preload_next_idx[1:0];
                        load_start_pulse <= 1'b1;
                        load_busy <= 1'b1;
                        source_done_seen <= 0;
                        write_done_seen <= 0;
                    end

                    // 显示选择和预加载并行；没有其他已完成图片时保留请求。
                    if (display_valid && next_req_pending && next_from_current[2]) begin
                        img_idx <= next_from_current[1:0];
                        disp_buf_idx <= next_from_current[1:0];
                        next_req_pending <= 1'b0;
                        auto_cnt <= 0;
                    end else if (display_valid && prev_req_pending && prev_from_current[2]) begin
                        img_idx <= prev_from_current[1:0];
                        disp_buf_idx <= prev_from_current[1:0];
                        prev_req_pending <= 1'b0;
                        auto_cnt <= 0;
                    end else if (display_valid && auto_play_en && auto_tick &&
                                 next_from_current[2]) begin
                        img_idx <= next_from_current[1:0];
                        disp_buf_idx <= next_from_current[1:0];
                        auto_cnt <= 0;               // 重启 1 秒计时
                    end
                end
            end
        end
    end
endmodule

// ============================================================================
// 子模块：按键去抖（key_press_debounce_fat32）
// ============================================================================
// 功能：把机械按键的抖动变成"干净"的单拍按下脉冲。
//   输入：button_in 原始按键（低有效：按下=0，松开=1，默认上拉）
//   输出：press_pulse      稳定按下瞬间的单拍脉冲（边沿检测）
//         button_stable_out 同步+去抖后的稳定电平（长按计时用）
// 原理：两级触发器同步（异步输入 → clk 域）→ 稳定计数器去抖（连续 N 拍
//       与旧状态不一致才确认改变）→ 下降沿检测产生脉冲。
module key_press_debounce_fat32 #(
    // 输入时钟和去抖时间；按键默认上拉，稳定高=松开、稳定低=按下。
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 50
) (
    input  wire clk,
    input  wire rst,
    input  wire button_in,
    output reg  press_pulse,
    // 对外提供已经同步/去抖的电平，供长按计时使用。
    output wire button_stable_out
);
    // 去抖所需的时钟周期数，由参数在编译期计算。
    localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;
    // 两级同步寄存器：先把异步按键输入带入 clk 域，再供后级使用。
    reg button_sync0, button_sync1, button_stable;
    // 只有连续 DEBOUNCE_CYCLES 个周期与稳定值不一致，才确认状态改变。
    reg [31:0] cnt;

    // 稳定电平是组合观察线；press_pulse 是时序块产生的单周期事件。
    assign button_stable_out = button_stable;

    // 按键输入是异步机械信号，必须先同步，再去抖，再边沿检测。
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // 复位：认为按键松开（高），无脉冲
            button_sync0 <= 1'b1;
            button_sync1 <= 1'b1;
            button_stable <= 1'b1;
            cnt <= 0;
            press_pulse <= 1'b0;
        end else begin
            // 这两级非阻塞赋值在同一时钟沿并行采样，实际形成两个级联触发器。
            button_sync0 <= button_in;
            button_sync1 <= button_sync0;
            press_pulse <= 1'b0;                 // 脉冲默认每拍清零
            if (button_sync1 == button_stable) begin
                // 输入已经稳定：计数清零。
                cnt <= 0;
            end else if (cnt >= DEBOUNCE_CYCLES - 1) begin
                // 输入连续不一致达到门限，正式更新稳定状态；高->低时发按下脉冲。
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;         // 下降沿 = 按下事件
                button_stable <= button_sync1;
                cnt <= 0;
            end else begin
                cnt <= cnt + 1'b1;               // 抖动中：继续数
            end
        end
    end
endmodule
