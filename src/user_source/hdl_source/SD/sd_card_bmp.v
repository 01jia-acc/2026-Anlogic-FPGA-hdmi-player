// ============================================================================
// sd_card_bmp：SD 卡 BMP 图片"扫描 / 加载 / 显示切换"总控模块
//
// 本模块是图片调度的中枢，自己不产生 SPI 时序，也不解析像素。它把上游
// (sd_card_top 的扇区读取 + bmp_read 的 BMP 解析) 和下游 (frame_read_write
// 的 SDRAM 帧缓存) 串起来，负责回答三个问题：
//   * 图在哪里（扫描结果 img_sector0~3）；
//   * 往哪块缓冲写（write_buf_idx / pending_buf_idx 的 ping-pong 选择）；
//   * 什么时候把新图切到显示端（disp_buf_idx 提交切换）。
//
//   整体数据流：
//     SD 卡 ──SPI 扇区字节流──> sd_card_top ──扇区读接口──> bmp_read
//       ──bmp_data/bmp_data_wr_en 像素流──> 写 FIFO(frame_read_write)
//       ──> SDRAM 帧缓存 Buffer0/1 ──> 读 FIFO ──> HDMI
//
//   本模块的核心机制：
//   1. 上电扫描：SD 初始化完成后自动发起一次扫描，顺序找出卡上前 4 张合法
//      的 640x480/24bit BMP（SCAN_TARGET_COUNT=4），锁存各自起始扇区。
//   2. 双缓冲加载：首图固定写 Buffer0；之后的图写"非当前显示"的那块
//      Buffer（ping-pong），整帧写完才切 disp_buf_idx，画面不会撕裂。
//   3. "提交"协议：一次加载必须同时满足两个条件才允许切换显示——
//      * source_done_seen：bmp_ready=1，源像素已全部送入写 FIFO；
//      * write_done_seen ：write_finish_toggle 翻转一次，整帧已写入 SDRAM。
//      两个事件先后顺序不定，各记一个标志位，都到齐后下一拍提交。
//   4. 按键：key_next 手动下一张（加载忙时只挂起 1 次，不连跳）；
//      key_auto 短按开/关自动轮播、长按上一张（长按只触发 1 次）。
//   5. 看门狗：加载超时（约 5s，如坏图/截断文件死锁）强制放弃本次加载并
//      发出 load_abort，保证按键始终有响应。
//
//   时钟域说明：本模块整体运行在 clk（sd_card_clk，约 100 MHz）。唯一跨
//   时钟域的信号是 write_finish_toggle（来自 ext_mem_clk 域，约 125 MHz），
//   用 3 级寄存器同步后做边沿检测。
// ============================================================================
module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000, // 参数：本模块时钟频率，用于按键消抖时长和自动轮播计时
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,       // 参数：扫描起始扇区号（从卡的最开头开始找图）
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,  // 参数：扫描终止扇区号（131072 扇区 x 512B = 64 MB 范围）
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd4,        // 参数：最多记录几张图（4 张，与 2bit 图片编号空间一致）
    parameter integer LONG_PRESS_MS     = 1000         // 参数：KEY2 长按触发时间，单位 ms
)(
    // ------------------------------------------------------------------------
    // 时钟与复位
    // ------------------------------------------------------------------------
    input                       clk,             // 输入：模块时钟，sd_card_clk 域（约 100 MHz）
    input                       rst,             // 输入：高有效复位

    // ------------------------------------------------------------------------
    // 按键输入（低有效，内部消抖模块统一处理）
    // ------------------------------------------------------------------------
    input                       key_next,        // 输入："手动下一张"按键
    input                       key_auto,        // 输入："自动轮播开/关"按键

    // ------------------------------------------------------------------------
    // BMP 尺寸要求（顶层固定为 640x480，bmp_read 用它们与文件头比对校验）
    // ------------------------------------------------------------------------
    input  [15:0]               bmp_width,       // 输入：要求 BMP 宽度 = 640
    input  [15:0]               bmp_height,      // 输入：要求 BMP 高度 = 480

    // ------------------------------------------------------------------------
    // 对顶层的状态输出
    // ------------------------------------------------------------------------
    output reg                  display_valid,   // 输出→top：=1 表示首图已完整写入 SDRAM，允许把帧缓存内容送到 HDMI；之前输出黑屏
    output      [3:0]           state_code,      // 输出→top：bmp_read 的扫描/加载状态码，经七段译码显示在数码管上（仅调试用，不参与图像数据流）

    // ------------------------------------------------------------------------
    // 与 frame_read_write 的"帧写完"握手（跨时钟域）
    // ------------------------------------------------------------------------
    input                       write_finish_toggle, // 输入←top（ext_mem_clk 域）：每写完一整帧就翻转一次 0/1，电平翻转=发生一次"帧写完"事件
    output reg [1:0]            write_buf_idx,       // 输出→top：本次写图使用哪块帧缓冲的基地址选择（0=Buffer0，1=Buffer1），供 frame_read_write 选 write_addr_0/1
    output reg [1:0]            disp_buf_idx,        // 输出→top：当前显示端从哪块帧缓冲读（0=Buffer0，1=Buffer1），供 frame_read_write 选 read_addr_0/1

    // ------------------------------------------------------------------------
    // 与 frame_read_write 的写通道接口（由 bmp_read 直通产生）
    // ------------------------------------------------------------------------
    output                      write_req,           // 输出：帧写请求（电平型，保持到 write_req_ack=1）。bmp_read 收到应答后才开始灌像素
    input                       write_req_ack,       // 输入：帧写请求应答，表示 frame_read_write 已清空写 FIFO、锁存基地址，可以开始送像素
    output                      write_en,            // 输出：逐像素写使能，每拍一个像素（直通 bmp_read 的 bmp_data_wr_en），喂给写 FIFO 的 .we
    output [31:0]               write_data,          // 输出：32bit 像素字 {R,G,B,8'b0}，喂给写 FIFO 的 .di（SDRAM 一个字 = 一个像素）

    // ------------------------------------------------------------------------
    // SD 卡 SPI 接口（直通 sd_card_top）
    // ------------------------------------------------------------------------
    output                      SD_nCS,              // 输出：SD 卡片选（低有效）
    output                      SD_DCLK,             // 输出：SPI 时钟
    output                      SD_MOSI,             // 输出：SPI 主机输出数据（发命令/地址给 SD 卡）
    input                       SD_MISO              // 输入：SPI 主机输入数据（SD 卡返回的数据/响应）
);

// ============================================================================
// 内部连线声明
// ============================================================================

// ---- 按键消抖后的单周期脉冲 ----
wire key_next_press;               // 手动下一张按键"按下"事件的单周期脉冲（50ms 消抖后）
wire key_auto_press;               // 自动轮播按键消抖后的"按下"事件单周期脉冲
wire key_auto_stable;              // KEY2 消抖后的稳定电平：1=松开，0=按下

// ---- 与 sd_card_top 的扇区读接口（bmp_read 直通，本模块只旁观）----
wire             sd_sec_read;            // 读扇区命令（一整个 512B 扇区）
wire [31:0]      sd_sec_read_addr;       // 要读的扇区号
wire [7:0]       sd_sec_read_data;       // 当前收到的 1 个字节
wire             sd_sec_read_data_valid; // 当前字节有效（=1 的拍 sd_sec_read_data 有效）
wire             sd_sec_read_end;        // 当前扇区 512 字节全部读完

// ---- 与 bmp_read 的状态/数据接口 ----
wire [3:0]       state_code_i;      // bmp_read 的当前状态码（扫描/读文件头/等待/出像素），直通给顶层数码管
wire             bmp_data_wr_en;    // 像素有效脉冲：=1 的拍 bmp_data 是一个完整的 RGB 像素
wire [23:0]      bmp_data;          // 24bit 像素 RGB（BMP 文件里是 B,G,R 字节序，bmp_read 已重组为 R,G,B）
wire             sd_init_done;      // SD 卡初始化完成（来自 sd_card_top），之前本模块所有逻辑保持在清零状态
wire             bmp_ready;         // bmp_read 空闲就绪：=1 表示可接收 scan_start/load_start 新任务；扫描/加载期间为 0，源数据送完后回到 1
wire             scan_done;         // 扫描结束（电平）：=1 表示扫描任务完成（找够了目标数或扫到上限）
wire             scan_found_valid;  // 扫描到一张合法 BMP 的单周期脉冲
wire [31:0]      scan_found_sector; // 该脉冲对应的 BMP 起始扇区号
wire [2:0]       scan_found_total;  // bmp_read 报告的扫描到图片总数（本模块未使用，保留作调试观察）

// ============================================================================
// 内部寄存器声明（每个变量的作用）
// ============================================================================
reg              scan_start_pulse;      // 发给 bmp_read 的"开始扫描"单周期脉冲
reg              load_start_pulse;      // 发给 bmp_read 的"开始加载一张图"单周期脉冲
reg [31:0]       load_sector;           // 本次加载图片的起始扇区号（发起加载时从 img_sectorN 取出）
reg              scan_kicked;           // 扫描是否已发起：=1 防止重复扫描（上电后只扫描一次）
reg              first_image_committed; // 首图是否已提交：提交前顶层显示黑屏；也用于判断"首图固定写 Buffer0"
    reg              auto_play_en;          // 自动轮播使能：key_auto 短按松开后翻转一次
reg [31:0]       auto_cnt;              // 自动轮播计时器：计到 CLK_FREQ_HZ-1（约 1s）触发 auto_tick 换下一张
reg [2:0]        img_found_count;       // 扫描到的图片总数（最多记 4 张，饱和于 4）
reg [1:0]        img_idx;               // 当前真正显示中的图片编号（0~3，对应 img_sectorN）
reg [1:0]        load_idx;              // 当前正在加载的图片编号（提交成功时赋给 img_idx）
reg [1:0]        pending_buf_idx;       // 本次加载正在写入的帧缓冲编号（本地留底，提交时赋给 disp_buf_idx）
reg [31:0]       img_sector0;           // 第 0 张图的起始扇区号（扫描时锁存）
reg [31:0]       img_sector1;           // 第 1 张图的起始扇区号
reg [31:0]       img_sector2;           // 第 2 张图的起始扇区号
reg [31:0]       img_sector3;           // 第 3 张图的起始扇区号
reg              next_req_pending;      // 是否挂起一个"下一张"请求：加载忙时按键只记 1 次，不会累积成连跳两张
reg              prev_req_pending;      // 是否挂起一个"上一张"请求：加载忙时按键只记 1 次
reg              key_auto_down_seen;    // 已确认 KEY2 按下，等待松开后判定短按/长按
reg [31:0]       long_press_cnt;        // KEY2 长按计数器
reg              long_press_marked;     // KEY2 长按已经触发，防止按住不放重复触发
reg              load_busy;             // 是否正在加载一张图：=1 时拒绝新的加载请求，直到提交成功或看门狗超时
reg [31:0]       load_timeout_cnt;      // 加载看门狗计数：超过 500M 拍（约 5s）仍未提交则强制放弃本次加载
reg              load_abort;            // 发给 bmp_read 的"放弃本次加载"单周期脉冲（超时恢复用，让 bmp_read 回到空闲态）
reg              source_done_seen;      // 本次加载期间是否已观察到 bmp_ready=1（源像素已全部送入写 FIFO）
reg              write_done_seen;       // 本次加载期间是否已观察到 write_finish_pulse（整帧已写入 SDRAM）
reg [2:0]        wrfin_tgl_sync;        // write_finish_toggle 的 3 级同步器（ext_mem_clk 域 → 本模块 clk 域，消除亚稳态）

// ---- 组合逻辑辅助信号 ----
wire             write_finish_pulse;    // 同步后的 toggle 边沿检测：任一翻转沿 = 一次"帧写完"事件（上升沿、下降沿都算）
wire             auto_tick;             // 自动轮播计时到点（auto_cnt 计满 1s 的当拍为 1）
wire [1:0]       next_from_current;     // 从当前显示图编号推算的"下一张"编号（在已找到的图片范围内回绕）
wire [1:0]       prev_from_current;     // 从当前显示图编号推算的"上一张"编号（在已找到的图片范围内回绕）

localparam integer LONG_PRESS_CYCLES = (CLK_FREQ_HZ / 1000) * LONG_PRESS_MS;

assign write_en   = bmp_data_wr_en;                     // 像素使能直通：bmp_read 每出一个像素，写 FIFO 就收一个
assign write_data = {bmp_data[23:16], bmp_data[15:8], bmp_data[7:0], 8'b0}; // 24bit RGB 高位对齐拼成 32bit 字（低 8bit 填 0），与 SDRAM 32bit 字宽一致
assign auto_tick  = (auto_cnt == (CLK_FREQ_HZ - 1));    // 计满 100M 拍（约 1s）的当拍为 1
assign next_from_current = next_index_limited(img_idx, img_found_count); // 下一张编号：当前编号在 [0, 已找到张数-1] 范围内回绕 +1
assign prev_from_current = prev_index_limited(img_idx, img_found_count); // 上一张编号：当前编号在 [0, 已找到张数-1] 范围内回绕 -1
assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];       // toggle 边沿检测：相邻两拍不同 = 刚发生一次翻转 = 一帧写完成
assign state_code = state_code_i;                        // 状态码直通顶层数码管（仅调试显示）

// ============================================================================
// 按键消抖：把外部按键的电平变化转换成干净的单周期"按下"脉冲
// ============================================================================
// 两个按键各配一个消抖器：按键稳定 50ms 才采信电平变化，且只在 1→0（按下）
// 沿产生一个 clk 周期的 press_pulse，消除机械抖动和长按期间的重复触发。
key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (50)
) u_key_next (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_next),       // 输入：外部"下一张"按键原始电平
    .press_pulse(key_next_press)  // 输出：消抖后的按下脉冲
);

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (50)
) u_key_auto (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_auto),       // 输入：外部"自动轮播"按键原始电平
    .press_pulse(key_auto_press), // 输出：消抖后的按下脉冲
    .button_stable_out(key_auto_stable) // 输出：消抖后的稳定电平
);

// ============================================================================
// 辅助函数（纯组合逻辑）
// ============================================================================

// next_index_limited：在当前显示编号 cur 的基础上，算出"下一张"的图片编号，
// 并且在已找到的图片总数 count 范围内回绕。count 是扫描结果，1~4 张都合法。
function [1:0] next_index_limited;
    input [1:0] cur;    // 当前图片编号
    input [2:0] count;  // 已找到的图片总数
    begin
        case (count)
            3'd0: next_index_limited = 2'd0;  // 0 张：无图可切，恒 0（正常不会走到，作防御）
            3'd1: next_index_limited = 2'd0;  // 1 张：只有第 0 张，无法前进
            3'd2: next_index_limited = (cur == 2'd1) ? 2'd0 : (cur + 2'd1);  // 2 张：0↔1 交替
            3'd3: next_index_limited = (cur == 2'd2) ? 2'd0 : (cur + 2'd1);  // 3 张：0→1→2→0 循环
            default: next_index_limited = (cur == 2'd3) ? 2'd0 : (cur + 2'd1); // 4 张：0→1→2→3→0 循环
        endcase
    end
endfunction

// prev_index_limited：在当前显示编号 cur 的基础上，算出"上一张"的图片编号，
// 并且只在已经扫描到的图片范围内回绕。
function [1:0] prev_index_limited;
    input [1:0] cur;    // 当前图片编号
    input [2:0] count;  // 已找到的图片总数
    begin
        case (count)
            3'd0: prev_index_limited = 2'd0;  // 0 张：无图可切，恒 0
            3'd1: prev_index_limited = 2'd0;  // 1 张：只有第 0 张
            3'd2: prev_index_limited = (cur == 2'd0) ? 2'd1 : (cur - 2'd1);
            3'd3: prev_index_limited = (cur == 2'd0) ? 2'd2 : (cur - 2'd1);
            default: prev_index_limited = (cur == 2'd0) ? 2'd3 : (cur - 2'd1);
        endcase
    end
endfunction

// sector_lut：图片编号 → 起始扇区号查表。扫描阶段把找到的每张图的起始扇区
// 锁存在 img_sector0~3 里，这里按编号取出。
function [31:0] sector_lut;
    input [1:0] idx;  // 图片编号 0~3
    begin
        case (idx)
            2'd0: sector_lut = img_sector0;
            2'd1: sector_lut = img_sector1;
            2'd2: sector_lut = img_sector2;
            2'd3: sector_lut = img_sector3;
            default: sector_lut = img_sector0;  // 防御：非法编号按第 0 张处理
        endcase
    end
endfunction

// next_buf_lut：选出"下一张图该写进哪块帧缓冲"。
//   * 首图（valid_now=0，即还没提交过任何图）：固定写 Buffer0；
//   * 之后：写"非当前显示"的另一块 Buffer，实现 ping-pong 双缓冲，
//     写入不会碰到正在显示的画面，写完切过去才无撕裂。
function [1:0] next_buf_lut;
    input [1:0] cur_disp_buf;  // 当前显示中的帧缓冲编号
    input       valid_now;     // 当前是否已有图在显示（首图为 0）
    begin
        if (!valid_now)
            next_buf_lut = 2'd0;                 // 首图固定写 buffer0
        else if (cur_disp_buf == 2'd0)
            next_buf_lut = 2'd1;                 // 显示在 0，新图写 1
        else
            next_buf_lut = 2'd0;                 // 显示在 1，新图写 0
    end
endfunction

// ============================================================================
// 主调度状态逻辑（单 always 块，无显式状态机，靠一组标志寄存器协同）
// ============================================================================
always @(posedge clk or posedge rst) begin
    if (rst) begin
        // ------------------------------ 复位：全部回到初始值 ------------------------------
        wrfin_tgl_sync        <= 3'b000;   // 同步器清零
        scan_start_pulse      <= 1'b0;     // 不发扫描脉冲
        load_start_pulse      <= 1'b0;     // 不发加载脉冲
        load_sector           <= 32'd0;    // 加载扇区清零
        scan_kicked           <= 1'b0;     // 允许重新发起扫描
        first_image_committed <= 1'b0;     // 首图未提交
        auto_play_en          <= 1'b0;     // 自动轮播关闭
        auto_cnt              <= 32'd0;    // 计时器清零
        img_found_count       <= 3'd0;     // 找到 0 张
        img_idx               <= 2'd0;     // 当前显示第 0 张（尚无显示）
        load_idx              <= 2'd0;     // 当前加载第 0 张（尚无加载）
        pending_buf_idx       <= 2'd0;     // 写入缓冲默认 0
        write_buf_idx         <= 2'd0;     // 对外写缓冲选择默认 0
        disp_buf_idx          <= 2'd0;     // 对外显示缓冲选择默认 0
        img_sector0           <= 32'd0;    // 4 张图的扇区号全部清零
        img_sector1           <= 32'd0;
        img_sector2           <= 32'd0;
        img_sector3           <= 32'd0;
        next_req_pending      <= 1'b0;     // 无挂起的下一张请求
        prev_req_pending      <= 1'b0;     // 无挂起的上一张请求
        key_auto_down_seen    <= 1'b0;     // KEY2 当前未按下
        long_press_cnt        <= 32'd0;    // 长按计数器清零
        long_press_marked     <= 1'b0;     // 长按未触发
        load_busy             <= 1'b0;     // 无加载任务
        load_timeout_cnt      <= 32'd0;    // 看门狗清零
        load_abort            <= 1'b0;     // 不发出放弃脉冲
        source_done_seen      <= 1'b0;     // 未观察到源数据送完
        write_done_seen       <= 1'b0;     // 未观察到帧写完
        display_valid         <= 1'b0;     // 顶层显示黑屏
    end else begin
        // ---------- 每拍默认动作（脉冲类信号只保持 1 拍，这里自动回落） ----------
        wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle}; // 同步器每拍移位一次，把 mem 域的 toggle 采进来
        scan_start_pulse <= 1'b0;      // 扫描脉冲默认清零（需要时下面置 1）
        load_start_pulse <= 1'b0;      // 加载脉冲默认清零
        load_abort       <= 1'b0;      // 放弃脉冲默认清零

        if (!sd_init_done) begin
            // ---------------- SD 卡尚未初始化完成：所有状态保持清零等待 ----------------
            // 注意：这里每拍都重新清零，相当于"初始化完成前本模块一直停在复位态"，
            // sd_init_done 一旦拉高，下一拍才从干净的初值开始工作。
            scan_kicked           <= 1'b0;
            first_image_committed <= 1'b0;
            auto_play_en          <= 1'b0;
            auto_cnt              <= 32'd0;
            img_found_count       <= 3'd0;
            img_idx               <= 2'd0;
            load_idx              <= 2'd0;
            pending_buf_idx       <= 2'd0;
            write_buf_idx         <= 2'd0;
            disp_buf_idx          <= 2'd0;
            img_sector0           <= 32'd0;
            img_sector1           <= 32'd0;
            img_sector2           <= 32'd0;
            img_sector3           <= 32'd0;
            next_req_pending      <= 1'b0;
            prev_req_pending      <= 1'b0;
            key_auto_down_seen    <= 1'b0;
            long_press_cnt        <= 32'd0;
            long_press_marked     <= 1'b0;
            load_busy             <= 1'b0;
            load_timeout_cnt      <= 32'd0;
            load_abort            <= 1'b0;
            source_done_seen      <= 1'b0;
            write_done_seen       <= 1'b0;
            display_valid         <= 1'b0;
        end else begin
            // ======================== SD 已初始化，正常运行阶段 ========================

            // ---- 扫描结果锁存：bmp_read 每找到一张图，scan_found_valid 脉冲一拍 ----
            if (scan_found_valid) begin
                // 按"当前已找到几张"的顺序把扇区号存进 img_sector0~3
                case (img_found_count)
                    3'd0: img_sector0 <= scan_found_sector; // 找到的第 1 张 → img_sector0
                    3'd1: img_sector1 <= scan_found_sector; // 找到的第 2 张 → img_sector1
                    3'd2: img_sector2 <= scan_found_sector; // 找到的第 3 张 → img_sector2
                    3'd3: img_sector3 <= scan_found_sector; // 找到的第 4 张 → img_sector3
                    default: ;                              // 超过 4 张：忽略，只记前 4 张
                endcase

                // 找到张数 +1，饱和在 4（再多也不记）
                if (img_found_count < 3'd4)
                    img_found_count <= img_found_count + 3'd1;
            end

            // ---- 提交条件 1：记住"源图已全部送进写 FIFO" ----
            // bmp_ready 拉高表示 bmp_read 回到空闲、像素流已送完。但此刻还不能切
            // 显示——像素只是进了写 FIFO，还没全部写进 SDRAM，得等帧写完。
            if (load_busy && bmp_ready)
                source_done_seen <= 1'b1;

            // ---- 提交条件 2：记住"整帧已写入 SDRAM" ----
            // 单独用标志位记录而不是直接提交，是为了避免"帧先写完、源后结束"时
            // 脉冲已经过去、条件永远凑不齐而卡死。两个事件谁先谁后都行，各记一笔。
            if (load_busy && write_finish_pulse)
                write_done_seen <= 1'b1;

            // ---- 提交判断：两个条件都满足 → 切换显示缓冲、更新显示图编号 ----
            if (load_busy && source_done_seen && write_done_seen) begin
                load_busy             <= 1'b0;            // 本次加载结束
                load_timeout_cnt      <= 32'd0;            // 看门狗清零
                source_done_seen      <= 1'b0;            // 清标志，供下一次加载使用
                write_done_seen       <= 1'b0;            // 清标志
                disp_buf_idx          <= pending_buf_idx; // 关键切换：把刚写完的那块缓冲交给显示端读
                img_idx               <= load_idx;        // 正在显示的图更新为刚加载完的这张
                display_valid         <= 1'b1;            // 通知顶层：已有图可显示，不再输出黑屏
                first_image_committed <= 1'b1;            // 首图提交完成（后续加载走 ping-pong 分支）
            end else if (load_busy) begin
                // ---- 看门狗：加载中但迟迟无法提交 → 计数 ----
//                if (load_timeout_cnt > 32'd100_000_000) begin
                if (load_timeout_cnt > 32'd500_000_000) begin
                    // 超时（约 5s @100MHz）：例如读到坏图或截断文件导致 bmp_read
                    // 永远送不完数据。强制释放 busy、发 load_abort 让 bmp_read
                    // 也放弃并回到空闲，从而恢复按键响应，不至于整机卡死。
                    load_busy             <= 1'b0;
                    load_timeout_cnt      <= 32'd0;
                    source_done_seen      <= 1'b0;
                    write_done_seen       <= 1'b0;
                    load_abort            <= 1'b1;         // 本拍给出放弃脉冲
                end else begin
                    load_timeout_cnt <= load_timeout_cnt + 32'd1;  // 未超时：继续计时
                end
            end else begin
                load_timeout_cnt <= 32'd0;  // 空闲：看门狗保持清零
            end

            // ---- 上电后自动发起一次扫描（只扫一次）----
            // bmp_ready 表示 bmp_read 已空闲可接任务。同时把运行状态全部复位，
            // 保证扫描结果覆盖任何历史残留。
            if (!scan_kicked && bmp_ready) begin
                scan_start_pulse      <= 1'b1;   // 发"开始扫描"脉冲
                scan_kicked           <= 1'b1;   // 标记已发起，防止重复扫描
                first_image_committed <= 1'b0;
                auto_play_en          <= 1'b0;
                auto_cnt              <= 32'd0;
                img_found_count       <= 3'd0;
                img_idx               <= 2'd0;
                load_idx              <= 2'd0;
                pending_buf_idx       <= 2'd0;
                write_buf_idx         <= 2'd0;
                disp_buf_idx          <= 2'd0;
                next_req_pending      <= 1'b0;
                prev_req_pending      <= 1'b0;
                key_auto_down_seen    <= 1'b0;
                long_press_cnt        <= 32'd0;
                long_press_marked     <= 1'b0;
                display_valid         <= 1'b0;   // 重新扫描期间显示黑屏
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
                write_done_seen       <= 1'b0;
            end else begin
                // ==================== 正常运行阶段：按键与加载调度 ====================

                // ---- 手动下一张请求：只挂起 1 次 ----
                // 即使当前正在加载（load_busy=1）也先记下；等空闲后统一消费。
                // 忙时连按多次不会累积（pending 只有 1bit），避免排队连跳。
                if (key_next_press && scan_done && (img_found_count > 3'd0))
                    next_req_pending <= 1'b1;

                // ---- KEY2 事件识别：短按/长按分离 ----
                // key_auto_press 只表示"已经确认按下"，不能在这里直接切换自动轮播，
                // 否则长按也会先被误判成短按。短按要等松开后再确认。
                if (key_auto_press && scan_done && (img_found_count > 3'd0))
                    key_auto_down_seen <= 1'b1;

                if (!scan_done || (img_found_count == 3'd0)) begin
                    long_press_cnt    <= 32'd0;
                    long_press_marked <= 1'b0;
                end else if (!key_auto_stable) begin
                    // KEY2 保持按下：累计稳定按下时间，达到阈值只触发一次上一张。
                    if (long_press_cnt < (LONG_PRESS_CYCLES - 1)) begin
                        long_press_cnt <= long_press_cnt + 32'd1;
                    end else if (!long_press_marked) begin
                        long_press_marked <= 1'b1;
                        prev_req_pending  <= 1'b1;
                    end
                end else begin
                    // KEY2 已稳定松开：长按不产生短按；否则在松开时确认短按。
                    long_press_cnt <= 32'd0;
                    if (key_auto_down_seen) begin
                        if (!long_press_marked && (img_found_count > 3'd1)) begin
                            auto_play_en <= ~auto_play_en;
                            auto_cnt     <= 32'd0;
                        end
                        key_auto_down_seen <= 1'b0;
                    end
                    long_press_marked <= 1'b0;
                end

                // ---- 自动轮播 1s 计时 ----
                // 只有"扫描完成、轮播开启、已有图显示、当前空闲、首图已提交、
                // 图数≥2"时才计时；任何条件不满足就清零。auto_tick 计满当拍置 1，
                // 下一分支用它触发换图；计满后回绕到 0 继续下一个 1s。
                if (scan_done && auto_play_en && display_valid && !load_busy && first_image_committed && (img_found_count > 3'd1)) begin
                    if (auto_tick)
                        auto_cnt <= 32'd0;
                    else
                        auto_cnt <= auto_cnt + 32'd1;
                end else begin
                    auto_cnt <= 32'd0;
                end

                // ---- 首图自动加载：扫描完成后立即把第 0 张写进 Buffer0 ----
                // 优先级最高（排在最前面的 if 分支）：开机后用户不按键也能出图。
                if (scan_done && !first_image_committed && bmp_ready && !load_busy && (img_found_count != 3'd0)) begin
                    load_idx         <= 2'd0;             // 加载第 0 张
                    load_sector      <= img_sector0;      // 它的起始扇区
                    pending_buf_idx  <= 2'd0;             // 首图固定写 Buffer0
                    write_buf_idx    <= 2'd0;             // 对外同步：本次写用 Buffer0
                    load_start_pulse <= 1'b1;             // 发"开始加载"脉冲
                    load_busy        <= 1'b1;             // 进入加载中
                    source_done_seen <= 1'b0;             // 清两个提交标志
                    write_done_seen  <= 1'b0;
                    next_req_pending <= 1'b0;             // 消费掉可能挂起的请求（首图优先）
                    prev_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;            // 轮播计时归零
                end
                // ---- 手动下一张：写到"非当前显示"的另一块 Buffer ----
                // 优先级次之：手动按键优先于自动轮播。next_req_pending 在此消费。
                else if (scan_done && bmp_ready && display_valid && !load_busy && next_req_pending && (img_found_count != 3'd0)) begin
                    load_idx         <= next_from_current;                           // 下一张的编号（已找到范围内回绕）
                    load_sector      <= sector_lut(next_from_current);               // 查表得到它的起始扇区
                    pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);   // ping-pong：写非显示缓冲
                    write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);   // 对外同步同样的选择
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    next_req_pending <= 1'b0;             // 请求已消费
                    auto_cnt         <= 32'd0;            // 换图后重新计时 1s
                end
                // ---- 手动上一张：下一张请求优先，上一张请求随后消费 ----
                else if (scan_done && bmp_ready && display_valid && !load_busy && prev_req_pending && (img_found_count != 3'd0)) begin
                    load_idx         <= prev_from_current;                            // 上一张的编号（已找到范围内回绕）
                    load_sector      <= sector_lut(prev_from_current);                // 查表得到它的起始扇区
                    pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);    // ping-pong：写非显示缓冲
                    write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);    // 对外同步同样的选择
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    prev_req_pending <= 1'b0;             // 请求已消费
                    auto_cnt         <= 32'd0;            // 换图后重新计时 1s
                end
                // ---- 自动轮播下一张：同样写到"非当前显示"的另一块 Buffer ----
                // 优先级最低：1s 计时到点且无其他任务时触发。条件与手动相同，
                // 只是触发源从按键换成了 auto_tick。
                else if (scan_done && bmp_ready && display_valid && !load_busy && auto_play_en && auto_tick && (img_found_count > 3'd1)) begin
                    load_idx         <= next_from_current;
                    load_sector      <= sector_lut(next_from_current);
                    pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                    write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
            end
        end
    end
end

// ============================================================================
// bmp_read：BMP 扫描与像素流解析（不产生 SPI 时序，也不保存整张图）
// ============================================================================
// 接口分三组：
//   * 扫描组：scan_start 触发，找够 scan_target_count 张图后 scan_done=1，
//     每找到一张 scan_found_valid 脉冲一拍并给出 scan_found_sector；
//   * 加载组：load_start 触发，从 load_sector 开始读文件头校验，等
//     write_req_ack 后再重读整图、按像素输出 bmp_data/bmp_data_wr_en；
//     load_abort 可中途放弃；
//   * 数据组：sd_sec_read* 是给 sd_card_top 的扇区读命令/数据接口，
//     write_req/write_req_ack 是给 frame_read_write 的帧级写握手。
bmp_read bmp_read_m0(
    .clk                    (clk),                   // 输入：模块时钟
    .rst                    (rst),                   // 输入：复位
    .ready                  (bmp_ready),             // 输出：空闲就绪标志，本模块用它判断何时可发新任务

    .scan_start             (scan_start_pulse),      // 输入：开始扫描脉冲（上电后发一次）
    .scan_start_sector      (SCAN_START_SECTOR),     // 输入：扫描起始扇区（参数）
    .scan_max_sector        (SCAN_MAX_SECTOR),       // 输入：扫描终止扇区（参数）
    .scan_target_count      (SCAN_TARGET_COUNT),     // 输入：最多找几张（参数=4）
    .scan_done              (scan_done),             // 输出：扫描完成（电平）
    .scan_found_valid       (scan_found_valid),      // 输出：找到一张图的脉冲
    .scan_found_sector      (scan_found_sector),     // 输出：该图的起始扇区
    .scan_found_total       (scan_found_total),      // 输出：扫描到的总数（未用）

    .load_start             (load_start_pulse),      // 输入：开始加载脉冲
    .load_abort             (load_abort),            // 输入：放弃加载脉冲（看门狗超时用）
    .load_sector            (load_sector),           // 输入：加载哪张图（起始扇区）

    .sd_init_done           (sd_init_done),          // 输入：SD 卡初始化完成
    .state_code             (state_code_i),          // 输出：状态码（→数码管）
    .bmp_width              (bmp_width),             // 输入：要求宽度 640（与文件头比对）
    .bmp_height             (bmp_height),            // 输入：要求高度 480（与文件头比对）
    .write_req              (write_req),             // 输出：帧写请求（→frame_read_write）
    .write_req_ack          (write_req_ack),         // 输入：帧写应答（←frame_read_write）
    .sd_sec_read            (sd_sec_read),           // 输出：读扇区命令（→sd_card_top）
    .sd_sec_read_addr       (sd_sec_read_addr),      // 输出：扇区号（→sd_card_top）
    .sd_sec_read_data       (sd_sec_read_data),      // 输入：扇区字节流（←sd_card_top）
    .sd_sec_read_data_valid (sd_sec_read_data_valid),// 输入：字节有效
    .sd_sec_read_end        (sd_sec_read_end),       // 输入：扇区读完
    .bmp_data_wr_en         (bmp_data_wr_en),        // 输出：像素有效脉冲（→写 FIFO）
    .bmp_data               (bmp_data)               // 输出：24bit RGB 像素（→写 FIFO）
);

// ============================================================================
// sd_card_top：SD 卡 SPI 底层驱动（初始化 + 扇区读）
// ============================================================================
// 本设计只从 SD 卡读 BMP，不写卡，所以 sd_sec_write* 写接口全部固定无效：
//   写命令恒 0、写地址恒 0，写数据/请求/完成信号悬空不接。
sd_card_top sd_card_top_m0(
    .clk                    (clk),                   // 输入：模块时钟
    .rst                    (rst),                   // 输入：复位
    .SD_nCS                 (SD_nCS),                // 输出：SD 片选
    .SD_DCLK                (SD_DCLK),               // 输出：SPI 时钟
    .SD_MOSI                (SD_MOSI),               // 输出：SPI 主机输出
    .SD_MISO                (SD_MISO),               // 输入：SPI 主机输入
    .sd_init_done           (sd_init_done),          // 输出：初始化完成（本模块调度逻辑的总闸门）
    .sd_sec_read            (sd_sec_read),           // 输入：读扇区命令（←bmp_read）
    .sd_sec_read_addr       (sd_sec_read_addr),      // 输入：扇区号（←bmp_read）
    .sd_sec_read_data       (sd_sec_read_data),      // 输出：扇区字节流（→bmp_read）
    .sd_sec_read_data_valid (sd_sec_read_data_valid),// 输出：字节有效
    .sd_sec_read_end        (sd_sec_read_end),       // 输出：扇区读完
    .sd_sec_write           (1'b0),                  // 输入：写命令恒 0（不写卡）
    .sd_sec_write_addr      (32'd0),                 // 输入：写地址恒 0
    .sd_sec_write_data      (),                      // 输出：写数据（未用，悬空）
    .sd_sec_write_data_req  (),                      // 输出：写数据请求（未用）
    .sd_sec_write_end       ()                       // 输出：写完成（未用）
);

endmodule

// ============================================================================
// key_press_debounce：按键消抖模块
//
// 功能：把外部按键的毛刺电平转成干净的单周期"按下"脉冲。
//   * 默认按键电路：松开=1，按下=0（button_in 高电平为常态）；
//   * 电平必须连续稳定 DEBOUNCE_MS 毫秒（换算成 DEBOUNCE_CYCLES 拍）才被采信，
//     期间任何抖动都会让计数重新开始；
//   * 采信电平变化时，只有 1→0 沿（按下）才输出一个 clk 周期的 press_pulse，
//     松开(0→1)不输出；长按也只触发一次，不会连发。
// ============================================================================
module key_press_debounce #(
    parameter integer CLK_FREQ_HZ = 100_000_000,  // 参数：模块时钟频率（Hz）
    parameter integer DEBOUNCE_MS = 20            // 参数：消抖时间（毫秒），本工程实例化时传 50
)(
    input  wire clk,             // 输入：模块时钟
    input  wire rst,             // 输入：高有效复位
    input  wire button_in,       // 输入：按键原始电平（默认：松开=1，按下=0）
    output reg  press_pulse,     // 输出：消抖后的"按下"单周期脉冲
    output wire button_stable_out // 输出：消抖后的稳定电平（1=松开，0=按下）
);

// 消抖时间换算成时钟拍数：CLK_FREQ_HZ/1000 = 每毫秒拍数，再乘毫秒数。
// 例：100MHz、50ms → 5,000,000 拍。
localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

reg button_sync0;   // 按键电平 2 级同步器第 1 级：消除亚稳态（异步按键 → 本时钟域）
reg button_sync1;   // 同步器第 2 级：button_sync1 是真正参与后续判断的稳定电平
reg button_stable;  // 当前被采信的按键电平：只有新电平连续稳定满 DEBOUNCE_CYCLES 拍才更新
reg [31:0] cnt;     // 抖动计数：button_sync1 与 button_stable 不一致期间每拍 +1，一致则清零

assign button_stable_out = button_stable;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        button_sync0  <= 1'b1;   // 复位默认松开
        button_sync1  <= 1'b1;
        button_stable <= 1'b1;
        cnt           <= 32'd0;
        press_pulse   <= 1'b0;
    end else begin
        button_sync0 <= button_in;    // 每拍采样一次按键电平（第 1 级同步）
        button_sync1 <= button_sync0; // 第 2 级同步
        press_pulse  <= 1'b0;         // 脉冲默认清零，只在按下被确认的那一拍置 1

        if (button_sync1 == button_stable) begin
            // 当前电平与已采信电平一致：没有新的按键动作（或动作已完成），计数清零
            cnt <= 32'd0;
        end else begin
            // 电平发生了变化：开始/继续累计稳定时间
            if (cnt >= DEBOUNCE_CYCLES - 1) begin
                // 不一致状态持续满 DEBOUNCE_CYCLES 拍：采信新电平
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;   // 1→0 沿 = 按下：输出单周期脉冲
                button_stable <= button_sync1;  // 更新已采信电平（按下或松开都更新，松开不产生脉冲）
                cnt           <= 32'd0;         // 计数清零，等待下一次变化
            end else begin
                cnt <= cnt + 32'd1;  // 还不满消抖时长：继续计数
            end
        end
    end
end

endmodule
