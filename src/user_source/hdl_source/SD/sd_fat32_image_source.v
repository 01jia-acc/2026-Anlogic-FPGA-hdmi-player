`timescale 1ns/1ps

// ============================================================================
// sd_fat32_image_source —— FAT32 图片源（数据通路外壳/仲裁器，顶层的"手"）
// ============================================================================
//
// 【一句话定位】
//   本模块是唯一拥有 SD 卡访问权的"手"：SPI 驱动、扇区读取、FAT32 扫描、
//   簇链文件流、BMP 解码全在内部。上层（sd_card_bmp_fat32）只看到扫描结果、
//   帧写请求和 RGB 像素流，显示/四缓存逻辑无需知道图片在卡上的物理位置或
//   簇链结构。
//
// 【内部五级结构（阅读导航，按数据流顺序）】
//   ① sd_card_top        ：SPI 初始化和 CMD17 单块读（直接驱动 SD 引脚）；
//   ② sd_sector_adapter  ：把上面的原始驱动包装成统一的 512 字节扇区接口；
//   ③ scanner 和 streamer：共享这条扇区通道，但扫描期间 scanner 优先（仲裁）；
//   ④ streamer → 字节 FIFO → decoder → 双线性 scaler，输出固定屏幕尺寸 RGB；
//   ⑤ SOURCE_ACK 等待帧写端确认 FIFO/帧缓存准备好，再开始读文件。
//
// 【三大事务接口（从本模块往外看）】
//   - 扫描事务（scan_start/busy/done + 资源表输出）：内部 scanner 扫 FAT32
//     根目录，结果经端口输出给上层锁存；本模块自己不保存"文件表"。
//   - 加载事务（load_start/cluster/size + load_busy/done/error）：上层查好
//     表把"这张图的坐标"作为参数发回来，本模块负责照坐标把文件读出来、
//     解成像素流。参数在 load_start 拍锁存，事务期间不再受上层影响。
//   - 写帧握手（write_req/ack/en/data）：加载事务进行到一半时，向
//     frame_read_write 申请写帧，得到 ack 后才开始灌像素。
//
// 【关键设计思想】"模块边界 + 握手"：每个模块只对自己的输入/输出负责，
//   复杂任务通过 req/busy/valid/done 分解成多个时钟阶段。
//   本模块的角色是"执行者"：它不决定加载哪张图（那是上层的事），
//   也不负责把像素存进 SDRAM（那是 frame_read_write 的事）。
// ============================================================================
module sd_fat32_image_source #(
    // ---- 参数：综合期常量 ----
    // SPI 低速/高速分频：初始化阶段用低速（稳），初始化完成后用高速（快）。
    parameter integer SPI_LOW_SPEED_DIV  = 248,
    parameter integer SPI_HIGH_SPEED_DIV = 4,
    // 流式读文件时最多允许访问的簇数（防坏簇链死循环的保护上限）。
    parameter integer MAX_VISITED_CLUSTERS = 8388608,
    parameter integer PCM_ENABLE = 0,
    parameter integer PCM_FIFO_ADDR_WIDTH = 12,
    parameter integer PCM_SECTOR_TIMEOUT = 100_000_000,
    parameter integer DISPLAY_WIDTH = 640,
    parameter integer DISPLAY_HEIGHT = 480
) (
    // 整条 FAT/SD/解码链使用同一 clk；rst 高有效。
    input  wire        clk,            // 系统时钟（100MHz，来自顶层）
    input  wire        rst,            // 高有效复位

    // 允许启动/恢复PCM；让出事务入口后，读图期间仍按整扇区补充已启动的PCM。
    input wire pcm_play_enable,
    input wire pixel_clk,
    // 像素域实际提交的图片；load_start（含后台预读）绝不能触发换音。
    input wire [3:0] pcm_image_id_pix,
    input wire pcm_commit_toggle_pix,
    output reg pcm_commit_ack,
    input wire pcm_ready,
    output wire signed [15:0] pcm_sample,
    output wire pcm_valid,
    output wire pcm_active,
    output wire pcm_end,
    output wire pcm_underflow,
    output reg [7:0] pcm_error,

    // ===================== 端口：FAT 卷扫描事务接口 =====================
    input  wire        scan_start,     // 事件：上层发"开始扫描"单拍脉冲
    output wire        scan_busy,      // 电平：扫描进行中（scanner 状态机非 IDLE）
    output wire        scan_done,      // 事件：扫描结束单拍脉冲（结果此刻有效）
    output wire [7:0]  scan_error,     // 扫描错误码（0=成功）
    // ---- 扫描结果：资源表（scan_done 脉冲拍有效，由上层锁存）----
    output wire [15:0] resource_valid,  // bit0~15 = PIC1~PIC16.BMP 是否找到
    output wire [511:0] resource_clusters, resource_sizes, // 每槽32位，槽0在低32位
    output wire [31:0] resource0_cluster,  // PIC1.BMP 的首簇号（物理起点）
    output wire [31:0] resource1_cluster,  // PIC2.BMP 的首簇号
    output wire [31:0] resource2_cluster,  // PIC3.BMP 的首簇号
    output wire [31:0] resource3_cluster,  // PIC4.BMP 的首簇号
    output wire [31:0] resource0_size,     // PIC1.BMP 的字节数
    output wire [31:0] resource1_size,     // PIC2.BMP 的字节数
    output wire [31:0] resource2_size,     // PIC3.BMP 的字节数
    output wire [31:0] resource3_size,     // PIC4.BMP 的字节数

    // ===================== 端口：文件加载事务接口 =====================
    // 文件加载事务：上层给出资源对应的首簇/大小，并可中止。
    // 注意分工：本模块内部的 scanner 扫出这些值、经端口送给上层存成文件表；
    // 上层按"加载哪张图"查表后，把选中的坐标作为参数发回来。
    input  wire        load_start,     // 事件：上层发"开始加载"单拍脉冲
    input  wire        load_abort,     // 事件：上层发"中止加载"单拍脉冲
    input  wire [31:0] load_cluster,   // 参数：要读的文件首簇号（上层查表结果）
    input  wire [31:0] load_size,      // 参数：要读的文件字节数（上层查表结果）
    output wire        source_ready,   // 电平：空闲且未扫描（可接受新事务的许可）
    output wire        load_busy,      // 电平：加载事务进行中（状态机非 IDLE）
    output reg         load_done,      // 事件：文件读完+解码完（单拍脉冲）
    output reg  [7:0]  load_error,     // 本次加载错误码（保持到下一次 load_start）

    // ===================== 端口：帧写端握手及像素流（连 frame_read_write） =====================
    output wire        write_req,      // 电平："我要写一整帧"（保持到 write_req_ack）
    input  wire        write_req_ack,  // 输入：帧写端已清空 FIFO/锁存地址，可以灌
    output wire        write_en,       // =1 时 write_data 是一个有效像素
    output wire        frame_commit,   // 事件：文件校验成功且缩放像素已完整输出
    // 帧写端握手及 32 位像素数据（RGB888 + 低 8 位空置）。
    output wire [31:0] write_data,     // {scaled_rgb, 8'h00}

    // ===================== 端口：SD 卡状态与 SPI 引脚 =====================
    output wire        sd_init_done,   // 电平：SD 卡 SPI 初始化完成
    output wire        SD_nCS,         // SPI 片选（低有效）
    output wire        SD_DCLK,        // SPI 时钟
    output wire        SD_MOSI,        // FPGA→卡 串行数据
    input  wire        SD_MISO,        // 卡→FPGA 串行数据

    // ===================== 诊断观察口 =====================
    // 这些信号只供 UART 周期日志观察，不参与图片读取控制。
    output wire [3:0]  debug_scanner_state,
    output wire [31:0] debug_sector_lba,
    output wire [8:0]  debug_sector_byte_count,
    output wire        debug_sector_active,
    output wire [15:0] debug_sector_done_count,
    output wire [15:0] debug_resource_valid,
    output wire [31:0] debug_root_crc,
    output wire [15:0] debug_root_bytes,
    output wire debug_pic10_size_bit5,
    output wire [4:0]  debug_sd_state,
    output wire        debug_sd_cmd_error,
    output wire        debug_sd_cmd_error_latched,
    output wire [63:0] debug_load_pipeline,
    output wire [31:0] debug_accepted_cluster,
    output wire [159:0] debug_pcm_status
);
    // ========================================================================
    // 内部 wire：SD 原始驱动的单块读接口（只连接 adapter 与 sd_card_top）
    // ========================================================================
    wire sd_sec_read;              // 请求读一个扇区
    wire [31:0] sd_sec_read_addr;  // 要读的扇区地址（LBA）
    wire [7:0] sd_sec_read_data;   // 返回的字节
    wire sd_sec_read_data_valid;   // 本拍 read_data 有效
    wire sd_sec_read_end;          // 一个扇区读完
    wire [4:0] debug_sd_state_i;
    wire debug_sd_cmd_req_i;
    wire debug_sd_cmd_ack_i;
    wire debug_sd_cmd_error_i;
    wire debug_sd_block_read_req_i;
    wire debug_sd_block_read_ack_i;
    wire debug_sd_cmd_error_latched_i;
    // 写接口本工程未使用（写信号固定接 0），这两条线只占位。
    wire sd_sec_write_data_req;
    wire sd_sec_write_end;

    // ========================================================================
    // 内部 wire：scanner 端的扇区请求与响应线（scanner ↔ 仲裁 mux）
    // ========================================================================
    wire scanner_sector_req;        // scanner 请求读一个扇区
    wire [31:0] scanner_sector_lba; // scanner 要读的扇区地址
    wire scanner_sector_busy;       // 扇区通道正忙
    wire scanner_sector_valid;      // 本拍字节有效
    wire [7:0] scanner_sector_byte; // 扇区字节流（回给 scanner）
    wire [8:0] scanner_sector_byte_index; // 当前字节在扇区内的序号 0~511（9 位）
    wire scanner_sector_done;       // 一个扇区读完（事件）
    wire [7:0] scanner_sector_error;// 扇区读错误码

    // ========================================================================
    // 内部 wire：streamer 端的扇区请求与响应线（streamer ↔ 仲裁 mux）
    // ========================================================================
    // 与 scanner 端完全对称；streamer 读文件时用它读 FAT 表和文件数据扇区。
    wire streamer_sector_req;
    wire [31:0] streamer_sector_lba;
    wire streamer_sector_busy;
    wire streamer_sector_valid;
    wire [7:0] streamer_sector_byte;
    wire [8:0] streamer_sector_byte_index;
    wire streamer_sector_done;
    wire [7:0] streamer_sector_error;
    wire streamer_sector_pending, audio_sector_pending;

    // ========================================================================
    // 内部 wire：仲裁后的共享扇区通道（接到 sd_sector_adapter）
    // ========================================================================
    // 同一时刻只能有一个消费者（scanner、BMP 或 PCM streamer）使用这条通道；
    // 仲裁规则见下面的 assign：scanner 忙时优先。
    wire sector_req;
    wire [31:0] sector_lba;
    wire sector_busy;
    wire sector_valid;
    wire [7:0] sector_byte;
    wire [8:0] sector_byte_index;
    wire sector_done;
    wire [7:0] sector_error;
    wire debug_sector_active_i;
    wire [31:0] debug_sector_lba_i;
    wire [8:0] debug_sector_byte_count_i;

    // ========================================================================
    // 内部 wire：各级状态/输出线，供外壳状态机和上层调试观察
    // ========================================================================
    wire scanner_scan_busy;         // 电平：scanner 正在扫描（仲裁依据）
    wire [3:0] scanner_debug_state;
    wire [15:0] scanner_debug_sector_done_count;
    wire [15:0] scanner_debug_resource_valid;
    // ---- streamer 的文件字节流输出 ----
    wire streamer_file_busy;        // 电平：streamer 正在读文件
    wire streamer_file_done;        // 事件：文件字节全部读完（file_end 给 decoder）
    wire streamer_file_valid;       // 本拍 file_byte 有效
    wire [7:0] streamer_file_byte;  // 文件字节流
    wire [31:0] streamer_file_byte_offset; // 当前字节在文件内的偏移（从 0 开始）
    wire [7:0] streamer_fs_error;   // streamer 的文件系统错误码
    // ---- scanner 解析出的文件系统几何参数（扫描结束后保持，供 streamer 用）----
    wire [31:0] fat_start_lba;      // FAT 区第一个扇区的 LBA
    wire [31:0] data_start_lba;     // 数据区第一个扇区的 LBA（簇 2 从这里开始）
    wire [7:0] sectors_per_cluster; // 每簇扇区数（2 的幂）
    wire [31:0] max_cluster;        // 卷上最大合法簇号（防坏链越界）
    wire [15:0] audio_resource_valid;
    wire [511:0] audio_clusters, audio_sizes;
    reg [31:0] audio_cluster, audio_size;
    reg audio_selected;
    (* ASYNC_REG = "TRUE" *) reg [3:0] audio_image_meta, audio_image_sync;
    (* ASYNC_REG = "TRUE" *) reg [2:0] audio_commit_sync;
    reg audio_commit_seen;
    reg [3:0] audio_image_q, audio_lookup_slot, audio_lookup_index;
    reg [2:0] audio_switch_state;
    reg audio_clear;
    wire audio_clear_ack;
    wire audio_image_changed = PCM_ENABLE && audio_commit_sync[2] != audio_commit_seen;
    always @(posedge clk or posedge rst) begin
        if(rst) begin audio_image_meta<=0; audio_image_sync<=0; audio_commit_sync<=0; end
        else begin
            audio_image_meta<=pcm_image_id_pix; audio_image_sync<=audio_image_meta;
            audio_commit_sync<={audio_commit_sync[1:0],pcm_commit_toggle_pix};
        end
    end
    wire pcm_sector_ready, pcm_overflow;
    wire audio_sector_req, audio_file_busy, audio_file_done, audio_file_valid;
    wire [31:0] audio_sector_lba;
    wire [7:0] audio_file_byte, audio_fs_error;
    reg audio_streamer_start;
    reg audio_running;
    // PCM 使用独立文件流状态，BMP/PCM 在扇区边界交替，保留簇/偏移/FIFO。
    wire audio_channel_busy;
    reg audio_attempted, audio_start, audio_done, audio_abort;
    reg scan_usable;
    wire [31:0] pcm_sample_counter;
    // 只读低频诊断：文件掩码、控制标志、图片/文件槽/错误、首簇、大小、
    // 像素域已消耗样本数和当前样本。跨域计数仅用于观察是否持续变化。
    assign debug_pcm_status = {audio_resource_valid,
        pcm_play_enable, audio_selected, audio_running, audio_attempted,
        audio_switch_state, audio_commit_seen, pcm_commit_ack, audio_commit_sync[2],
        audio_clear, audio_clear_ack, pcm_active, pcm_valid, pcm_ready, pcm_overflow,
        audio_image_q, audio_lookup_slot, pcm_error, audio_cluster, audio_size,
        pcm_sample_counter, pcm_sample};
    reg [31:0] audio_wait_count;
    // ---- decoder 的输出 ----
    wire decoder_header_accepted;   // BMP 头解析通过
    wire decoder_pixel_valid;       // 本拍 pixel_rgb 有效（接 write_en）
    wire [23:0] decoder_pixel_rgb;  // 解码出的 RGB888 像素
    wire decoder_frame_commit;      // 整帧像素全部输出完成
    wire decoder_done;              // 事件：解码结束（加载事务的收尾信号）
    wire [7:0] decoder_error;       // decoder 错误码
    reg [1:0] source_state;
    reg decoder_start;
    localparam SOURCE_IDLE=2'd0, SOURCE_ACK=2'd1, SOURCE_STREAM=2'd2, SOURCE_AUDIO=2'd3;
    wire [15:0] decoder_width, decoder_height, decoder_x, decoder_y;
    wire scaler_input_ready, scaled_valid, scaled_done;
    wire [3:0] scaler_debug_state;
    wire [23:0] scaled_rgb;
    reg decoder_finished, scaler_finished;
    // A complete SD sector cannot be paused. Reserve 512 bytes plus pipeline
    // margin before granting BMP; pause only the buffered decoder byte stream.
    reg [7:0] image_bytes [0:1023];
    reg [9:0] byte_wr_ptr, byte_rd_ptr;
    reg [10:0] byte_count;
    reg [31:0] next_decode_offset, buffered_offset;
    reg [7:0] buffered_byte;
    reg buffered_valid, file_end_pending, file_end_sent;
    wire image_sector_space = byte_count <= 508;
    wire byte_pop = byte_count!=0 && scaler_input_ready && source_state==SOURCE_STREAM;
    wire buffered_end = file_end_pending && byte_count==0 && !buffered_valid && !file_end_sent;

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            byte_wr_ptr<=0; byte_rd_ptr<=0; byte_count<=0;
            next_decode_offset<=0; buffered_offset<=0;
            buffered_valid<=0; file_end_pending<=0; file_end_sent<=0;
            decoder_finished<=0; scaler_finished<=0;
        end else if(decoder_start || load_abort) begin
            byte_wr_ptr<=0; byte_rd_ptr<=0; byte_count<=0;
            next_decode_offset<=0; buffered_valid<=0;
            file_end_pending<=0; file_end_sent<=0;
            decoder_finished<=0; scaler_finished<=0;
        end else begin
            buffered_valid<=byte_pop;
            if(streamer_file_valid) byte_wr_ptr<=byte_wr_ptr+1'b1;
            if(byte_pop) begin
                byte_rd_ptr<=byte_rd_ptr+1'b1;
                buffered_offset<=next_decode_offset;
                next_decode_offset<=next_decode_offset+1'b1;
            end
            case({streamer_file_valid,byte_pop})
                2'b10: byte_count<=byte_count+1'b1;
                2'b01: byte_count<=byte_count-1'b1;
                default: ;
            endcase
            if(streamer_file_done) file_end_pending<=1;
            if(buffered_end) file_end_sent<=1;
            if(decoder_done) decoder_finished<=1;
            if(scaled_done) scaler_finished<=1;
        end
    end
    // Separate synchronous RAM process; do not reset the sector storage.
    always @(posedge clk) begin
        if(streamer_file_valid) image_bytes[byte_wr_ptr]<=streamer_file_byte;
        if(byte_pop) buffered_byte<=image_bytes[byte_rd_ptr];
    end

    // ========================================================================
    // 内部 reg：外壳状态机
    // ========================================================================
    // 外壳级三状态：空闲、等待写端确认、文件字节/像素流传输。
    // 锁存一次 load_start 提供的资源参数，避免上层改变输入影响正在进行的读。
    reg [31:0] load_cluster_q;      // 本次加载事务的首簇号（冻结副本）
    assign debug_accepted_cluster = load_cluster_q;
    reg [31:0] load_size_q;         // 本次加载事务的字节数（冻结副本）
    // 送给 streamer/decoder 的一拍启动脉冲。
    reg streamer_start;             // 事件：启动文件流

    // 状态编码：唯一的状态机，case 里的分支就是全部事务流程。
    localparam CHANNEL_NONE = 2'd0, CHANNEL_BMP = 2'd1, CHANNEL_PCM = 2'd2;
    reg [1:0] sector_owner;
    // High 16 bits: source/scaler/owner, then eight progress flags.
    // Middle 16 bits: five reserved bits + FIFO occupancy. Low 32: byte offset.
    assign debug_load_pipeline = {source_state, scaler_debug_state, sector_owner,
        decoder_header_accepted, scaler_input_ready, decoder_finished, scaler_finished,
        file_end_pending, file_end_sent, streamer_file_busy, write_req_ack,
        5'd0, byte_count, next_decode_offset};
    wire audio_fault;
    wire audio_read_allowed = audio_running && !audio_fault &&
        audio_switch_state==0 && !audio_image_changed &&
        (source_state == SOURCE_STREAM ||
         (source_state == SOURCE_AUDIO && pcm_play_enable));
    assign audio_channel_busy = sector_busy || !pcm_sector_ready ||
                                !audio_read_allowed || sector_owner != CHANNEL_PCM || scanner_scan_busy;
    assign audio_fault = audio_running && (pcm_overflow ||
                       audio_wait_count >= PCM_SECTOR_TIMEOUT ||
                       (audio_file_done && audio_fs_error != 0));

    // ========================================================================
    // 组合观察线：对上层暴露的状态和数据通路（全部 assign 连续驱动，无记忆）
    // ========================================================================
    assign scan_busy = scanner_scan_busy;                     // 直通 scanner 状态
    // source_ready = "可以接新事务"：本外壳空闲 + 扫描器不在扫（扇区通道空）。
    // 上层发 scan_start/load_start 都以它为许可条件。
    assign source_ready = (source_state == SOURCE_IDLE) && !scanner_scan_busy &&
                           !streamer_file_busy && !sector_busy && sector_owner == CHANNEL_NONE;
    // load_busy = 外壳不在空闲 = 加载事务（含等 ack 阶段）正在进行。
    assign load_busy = (source_state == SOURCE_ACK || source_state == SOURCE_STREAM);
    // write_req 只在 SOURCE_ACK 阶段为 1：事务开始读文件前先请求写端准备。
    assign write_req = (source_state == SOURCE_ACK);
    // 只有流传输阶段 + 缩放器输出有效像素时，write_data 才有效。
    assign write_en = (source_state == SOURCE_STREAM) && scaled_valid;
    // 文件成功解码与缩放输出完成必须同时满足，不能只用文件读完来提交。
    // decoder_start is registered in SOURCE_ACK. On the first STREAM cycle
    // the previous file's sticky completion flags have not been cleared yet.
    assign frame_commit = source_state==SOURCE_STREAM && !decoder_start && decoder_finished &&
        scaler_finished && decoder_error==0 && streamer_fs_error==0;
    // 像素打包：RGB888 放高 24 位，低 8 位填 0，凑成 32 位存储字。
    assign write_data = {scaled_rgb, 8'h00};

    // 诊断观察口只做连续映射。日志看到的 S/L/C/Q 可以按层逐级判断：
    // S 是 FAT 扫描状态，L/C 是扇区事务地址和字节进度，Q 是 SD 高层状态。
    assign debug_scanner_state       = scanner_debug_state;
    assign debug_sector_lba          = debug_sector_lba_i;
    assign debug_sector_byte_count   = debug_sector_byte_count_i;
    assign debug_sector_active       = debug_sector_active_i;
    assign debug_sector_done_count   = scanner_debug_sector_done_count;
    assign debug_resource_valid      = scanner_debug_resource_valid;
    assign debug_sd_state            = debug_sd_state_i;
    assign debug_sd_cmd_error        = debug_sd_cmd_error_i;
    assign debug_sd_cmd_error_latched= debug_sd_cmd_error_latched_i;

    // ========================================================================
    // 扇区通道仲裁：先授予访问权，再接收注册请求，完整扇区后才换所有者。
    // ========================================================================
    // PCM FIFO 有整扇区空间时优先补充，满后把带宽让给 BMP。响应只送给所有者，
    // 两个文件读取器可同时保持文件进度，不会把对方的音频/图片/FAT字节混入。
    always @(posedge clk or posedge rst) begin
        if (rst) sector_owner <= CHANNEL_NONE;
        else if (sector_owner == CHANNEL_NONE) begin
            if (!scanner_scan_busy && !sector_busy && !sector_done) begin
                if (audio_read_allowed && pcm_sector_ready && audio_sector_pending)
                    sector_owner <= CHANNEL_PCM;
                else if (source_state == SOURCE_STREAM && streamer_sector_pending && image_sector_space)
                    sector_owner <= CHANNEL_BMP;
            end
        end else if (sector_done) sector_owner <= CHANNEL_NONE;
        else if (!sector_busy && !audio_sector_req && !streamer_sector_req) begin
            // 尚未发出请求时可撤销许可；已经发出的扇区保持所有者直到 done。
            if (sector_owner == CHANNEL_PCM && (!audio_read_allowed || !pcm_sector_ready))
                sector_owner <= CHANNEL_NONE;
            else if (sector_owner == CHANNEL_BMP && load_abort)
                sector_owner <= CHANNEL_NONE;
        end
    end
    assign sector_req = scanner_scan_busy ? scanner_sector_req :
                        sector_owner == CHANNEL_PCM ? audio_sector_req :
                        sector_owner == CHANNEL_BMP ? streamer_sector_req : 1'b0;
    assign sector_lba = scanner_scan_busy ? scanner_sector_lba :
                        sector_owner == CHANNEL_PCM ? audio_sector_lba : streamer_sector_lba;
    assign scanner_sector_busy = sector_busy;
    assign scanner_sector_valid = sector_valid;
    assign scanner_sector_byte = sector_byte;
    assign scanner_sector_byte_index = sector_byte_index;
    assign scanner_sector_done = sector_done;
    assign scanner_sector_error = sector_error;
    // 请求之前预留整个扇区的缓冲；一旦发出请求就必须接收全部字节。
    assign streamer_sector_busy = sector_busy || sector_owner != CHANNEL_BMP || scanner_scan_busy || !image_sector_space;
    assign streamer_sector_valid = sector_valid && sector_owner == CHANNEL_BMP && !scanner_scan_busy;
    assign streamer_sector_byte = sector_byte;
    assign streamer_sector_byte_index = sector_byte_index;
    assign streamer_sector_done = sector_done && sector_owner == CHANNEL_BMP && !scanner_scan_busy;
    assign streamer_sector_error = sector_error;

    // ========================================================================
    // 实例①：sd_card_top —— SD 卡原始 SPI 驱动
    // ========================================================================
    // 职责：上电 SPI 初始化 + CMD17 单块读一个 512 字节扇区。
    // 它不认识"文件/簇"，只知道"读第 N 个扇区"。写接口固定接 0（只读卡）。
    sd_card_top #(
        .SPI_LOW_SPEED_DIV (SPI_LOW_SPEED_DIV),   // 初始化阶段 SPI 分频（低速）
        .SPI_HIGH_SPEED_DIV(SPI_HIGH_SPEED_DIV)   // 正常读阶段 SPI 分频（高速）
    ) u_sd_card_top (
        .clk                  (clk),
        .rst                  (rst),
        // SD 卡 SPI 物理引脚
        .SD_nCS               (SD_nCS),
        .SD_DCLK              (SD_DCLK),
        .SD_MOSI              (SD_MOSI),
        .SD_MISO              (SD_MISO),
        // 状态与单块读接口
        .sd_init_done         (sd_init_done),
        .sd_sec_read          (sd_sec_read),
        .sd_sec_read_addr     (sd_sec_read_addr),
        .sd_sec_read_data     (sd_sec_read_data),
        .sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_end      (sd_sec_read_end),
        // 写接口未使用：全部固定
        .sd_sec_write         (1'b0),
        .sd_sec_write_addr    (32'd0),
        .sd_sec_write_data    (8'd0),
         .sd_sec_write_data_req(sd_sec_write_data_req),
         .sd_sec_write_end     (sd_sec_write_end),
         .debug_state                    (debug_sd_state_i),
         .debug_cmd_req                  (debug_sd_cmd_req_i),
         .debug_cmd_ack                  (debug_sd_cmd_ack_i),
         .debug_cmd_error                (debug_sd_cmd_error_i),
         .debug_block_read_req           (debug_sd_block_read_req_i),
         .debug_block_read_ack           (debug_sd_block_read_ack_i),
         .debug_cmd_error_latched        (debug_sd_cmd_error_latched_i)
     );

    // ========================================================================
    // 实例②：sd_sector_adapter —— 把原始驱动包装成统一扇区接口
    // ========================================================================
    // 职责：把 sd_card_top 的单块读接口转换成"请求一个扇区 → busy →
    // 逐字节 valid（带 0~511 序号）→ done/error"的标准事务，屏蔽 SPI 细节。
    // 上层 scanner/streamer 都只和这个统一接口打交道。
    sd_sector_adapter u_sd_sector_adapter (
        .clk                  (clk),
        .rst                  (rst),
        // 统一扇区事务接口（接仲裁后的共享通道）
        .sector_req           (sector_req),
        .sector_lba           (sector_lba),
        .sector_busy          (sector_busy),
        .sector_valid         (sector_valid),
        .sector_byte          (sector_byte),
         .sector_byte_index    (sector_byte_index),
         .sector_done          (sector_done),
         .sector_error         (sector_error),
         .debug_active         (debug_sector_active_i),
         .debug_lba            (debug_sector_lba_i),
         .debug_byte_count     (debug_sector_byte_count_i),
        // 原始驱动接口
        .sd_sec_read          (sd_sec_read),
        .sd_sec_read_addr     (sd_sec_read_addr),
        .sd_sec_read_data     (sd_sec_read_data),
        .sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_end      (sd_sec_read_end)
    );

    // ========================================================================
    // 实例③：fat32_volume_scanner —— FAT32 卷扫描器（scan 事务的执行者）
    // ========================================================================
    // 职责：读 LBA0/BPB → 沿根目录簇链找 PIC1~PIC16.BMP 和 AUDIO1~16.PCM →
    // 输出资源表（首簇+大小）和文件系统几何参数。
    // 注意 rst_n 接 ~rst：scanner 内部是低有效复位，本模块是高有效。
    fat32_volume_scanner u_volume_scanner (
        .clk                  (clk),
        .rst_n                (~rst),
        // 扫描事务
        .scan_start           (scan_start),
        .scan_busy            (scanner_scan_busy),  // → 仲裁依据 + 端口 scan_busy
        .scan_done            (scan_done),
        .fs_error             (scan_error),
        // 分区/几何参数（partition_lba 未引出，streamer 用不到）
        .partition_lba        (),
        .sectors_per_cluster  (sectors_per_cluster), // 扫描后保持，供 streamer 换算
        .fat_start_lba        (fat_start_lba),       // 扫描后保持，供 streamer 查 FAT
        .data_start_lba       (data_start_lba),      // 扫描后保持，供 streamer 换算簇→扇区
        .max_cluster          (max_cluster),         // 扫描后保持，供 streamer 防越界
        // 扫描结果：资源表（直通输出给上层）
        .resource_valid       (resource_valid),
        .resource_clusters    (resource_clusters),
        .resource_sizes       (resource_sizes),
        .resource0_cluster    (resource0_cluster),
        .resource1_cluster    (resource1_cluster),
        .resource2_cluster    (resource2_cluster),
        .resource3_cluster    (resource3_cluster),
        .resource0_size       (resource0_size),
        .resource1_size       (resource1_size),
        .resource2_size       (resource2_size),
        .resource3_size       (resource3_size),
        .audio_resource_valid (audio_resource_valid),
        .audio_cluster        (audio_clusters),
        .audio_size           (audio_sizes),
        .debug_state          (scanner_debug_state),
        .debug_sector_done_count(scanner_debug_sector_done_count),
        .debug_resource_valid (scanner_debug_resource_valid),
        .debug_pic10_size_bit5(debug_pic10_size_bit5),
        .debug_root_crc(debug_root_crc),
        .debug_root_bytes(debug_root_bytes),
        // 扇区通道（scanner 侧，经仲裁 mux 接共享通道）
        .sector_req           (scanner_sector_req),
        .sector_lba           (scanner_sector_lba),
        .sector_busy          (scanner_sector_busy),
        .sector_valid         (scanner_sector_valid),
        .sector_byte          (scanner_sector_byte),
        .sector_byte_index    (scanner_sector_byte_index),
        .sector_done          (scanner_sector_done),
        .sector_error         (scanner_sector_error)
    );

    // ========================================================================
    // 实例④：fat32_file_streamer —— 沿 FAT 簇链把文件字节流出来
    // ========================================================================
    // 职责：从 first_cluster 出发，读一个簇 → 查 FAT 表找下一簇 → 继续读，
    // 直到凑满 file_size 字节。输出文件字节流（file_valid/file_byte/偏移）。
    fat32_file_streamer #(
        .MAX_VISITED_CLUSTERS(MAX_VISITED_CLUSTERS)  // 防坏簇链死循环
    ) u_file_streamer (
        .clk                  (clk),
        .rst_n                (~rst),
        // 文件事务控制：start 是外壳发的一拍脉冲，abort 直通上层的 load_abort
        .file_start           (streamer_start),
        .file_abort           (load_abort),
        // 文件定位参数：来自外壳锁存的冻结副本（load_cluster_q/load_size_q）
        .first_cluster        (load_cluster_q),
        .file_size            (load_size_q),
        // 文件系统几何参数：来自 scanner 扫描后的保持值
        .sectors_per_cluster  (sectors_per_cluster),
        .fat_start_lba        (fat_start_lba),
        .data_start_lba       (data_start_lba),
        .max_cluster          (max_cluster),
        // 文件字节流输出（接 decoder）
        .file_busy            (streamer_file_busy),
        .file_done            (streamer_file_done),   // 全部字节读完
        .file_valid           (streamer_file_valid),
        .file_byte            (streamer_file_byte),
        .file_byte_offset     (streamer_file_byte_offset),
        .fs_error             (streamer_fs_error),
        // 扇区通道（streamer 侧，经仲裁 mux 接共享通道）
        .sector_req           (streamer_sector_req),
        .sector_pending       (streamer_sector_pending),
        .sector_lba           (streamer_sector_lba),
        .sector_busy          (streamer_sector_busy),
        .sector_valid         (streamer_sector_valid),
        .sector_byte          (streamer_sector_byte),
        .sector_byte_index    (streamer_sector_byte_index),
        .sector_done          (streamer_sector_done),
        .sector_error         (streamer_sector_error)
    );

    fat32_file_streamer #(.MAX_VISITED_CLUSTERS(MAX_VISITED_CLUSTERS)) u_audio_streamer (
        .clk(clk), .rst_n(~rst), .file_start(audio_streamer_start), .file_abort(audio_abort),
        .first_cluster(audio_cluster), .file_size(audio_size),
        .sectors_per_cluster(sectors_per_cluster), .fat_start_lba(fat_start_lba),
        .data_start_lba(data_start_lba), .max_cluster(max_cluster),
        .file_busy(audio_file_busy), .file_done(audio_file_done),
        .file_valid(audio_file_valid), .file_byte(audio_file_byte),
        .file_byte_offset(), .fs_error(audio_fs_error),
        .sector_req(audio_sector_req), .sector_pending(audio_sector_pending),
        .sector_lba(audio_sector_lba),
        .sector_busy(audio_channel_busy),
        .sector_valid(sector_valid && sector_owner == CHANNEL_PCM && !scanner_scan_busy),
        .sector_byte(sector_byte), .sector_byte_index(sector_byte_index),
        .sector_done(sector_done && sector_owner == CHANNEL_PCM && !scanner_scan_busy),
        .sector_error(sector_error)
    );

    // ========================================================================
    // 实例⑤：bmp24_decoder —— BMP 文件字节 → RGB 像素流
    // ========================================================================
    // 职责：校验正高度、无压缩 RGB24 BMP（最大 1920x1080），输出原图像素。
    // 字节 FIFO 在行间暂停解码，为后面的双线性插值留出运算时间。
    bmp24_decoder #(
        .EXPECTED_WIDTH (640),   // ALLOW_RESIZE=1 时按 MAX_WIDTH/HEIGHT 校验
        .EXPECTED_HEIGHT(480),
        .ALLOW_RESIZE(1), .MAX_WIDTH(1920), .MAX_HEIGHT(1080)
    ) u_bmp24_decoder (
        .clk                  (clk),
        .rst_n                (~rst),
        // 控制：start 是外壳发的一拍脉冲；file_size 用于校验/确定像素范围
        .file_start           (decoder_start),
        .file_abort           (load_abort),
        .file_size            (load_size_q),
        // 文件字节流输入（来自可暂停的字节 FIFO）
        .file_valid           (buffered_valid && source_state == SOURCE_STREAM),
        .file_byte            (buffered_byte),
        .file_byte_offset     (buffered_offset),
        .file_end             (buffered_end && source_state == SOURCE_STREAM),
        .upstream_error       (streamer_fs_error),   // 上游 FS 错误透传
        // 输出原尺寸像素，交给 scaler
        .header_accepted      (decoder_header_accepted),
        .pixel_valid          (decoder_pixel_valid),
        .pixel_rgb            (decoder_pixel_rgb),
        .pixel_x              (decoder_x),
        .pixel_y              (decoder_y),
        .frame_commit         (decoder_frame_commit),
        .bmp_done             (decoder_done),        // 解码结束（事务收尾信号）
        .bmp_error            (decoder_error),
        .top_down             (),   // 未使用
        .pixel_offset         (),   // 未使用
        .row_stride           (),
        .image_width          (decoder_width), .image_height(decoder_height)
    );

    image_bilinear_scaler #(.OUT_WIDTH(DISPLAY_WIDTH), .OUT_HEIGHT(DISPLAY_HEIGHT)) u_scaler (
        .clk(clk), .rst(rst), .start(decoder_start),
        .abort(load_abort || decoder_error!=0),
        .header_valid(decoder_header_accepted),
        .source_width(decoder_width), .source_height(decoder_height),
        .pixel_valid(decoder_pixel_valid), .pixel_rgb(decoder_pixel_rgb),
        .pixel_x(decoder_x), .pixel_y(decoder_y), .input_ready(scaler_input_ready),
        .output_valid(scaled_valid), .output_rgb(scaled_rgb),
        .done(scaled_done), .debug_state(scaler_debug_state)
    );
    // synthesis translate_off
    always @(posedge clk) if(!rst && byte_count>1024)
        $fatal(1,"BMP sector FIFO overflow");
    // synthesis translate_on

    // ========================================================================
    // 外壳时序控制中心：唯一的 always 块
    // ========================================================================
    // 它不解析 FAT 字段，只协调三个子模块的启动顺序：
    //   先确认写端（SOURCE_ACK 等 write_req_ack），再同时启动文件流和 BMP
    //   解码（进 SOURCE_STREAM），最后等 decoder_done 收尾。
    pcm_stream_buffer #(.ADDR_WIDTH(PCM_FIFO_ADDR_WIDTH)) u_pcm_buffer (
        .wr_clk(clk), .rd_clk(pixel_clk), .rst(rst),
        .stream_clear(audio_clear), .clear_ack(audio_clear_ack),
        .stream_start(audio_start), .stream_done(audio_done),
        .prefill_done(audio_file_done),
        .byte_valid(audio_file_valid),
        .byte_data(audio_file_byte), .sector_ready(pcm_sector_ready),
        .overflow(pcm_overflow), .sample(pcm_sample), .sample_valid(pcm_valid),
        .sample_ready(pcm_ready), .stream_active(pcm_active),
        .stream_end(pcm_end), .underflow(pcm_underflow), .sample_counter(pcm_sample_counter)
    );

    always @(posedge clk or posedge rst) begin
        // ------------------------------------------------------------------
        // (a) 复位：回空闲、参数清零、无脉冲
        // ------------------------------------------------------------------
        if (rst) begin
            source_state <= SOURCE_IDLE;
            load_cluster_q <= 32'd0;
            load_size_q <= 32'd0;
            // 这两个信号是单周期脉冲，默认每拍清零。
            streamer_start <= 1'b0;
            decoder_start <= 1'b0;
            load_done <= 1'b0;
            load_error <= 8'h00;
            audio_attempted <= 0; audio_running <= 0; audio_streamer_start <= 0;
            audio_start <= 0; audio_done <= 0; audio_abort <= 0;
            audio_wait_count <= 0; pcm_error <= 0; scan_usable <= 0;
            audio_commit_seen<=0; pcm_commit_ack<=0; audio_image_q<=0;
            audio_lookup_slot<=0; audio_lookup_index<=0; audio_switch_state<=0;
            audio_clear<=0; audio_selected<=0; audio_cluster<=0; audio_size<=0;
        end else begin
            // ------------------------------------------------------------------
            // 每拍开头：脉冲类输出默认清零（条件满足才置 1，最多维持一拍）
            // ------------------------------------------------------------------
            streamer_start <= 1'b0;
            decoder_start <= 1'b0;
            load_done <= 1'b0;
            audio_start <= 0; audio_done <= 0; audio_abort <= 0;
            audio_streamer_start <= 0;
            if (scan_done) scan_usable <= scan_error == 0 || scan_error == 8'h22;
            if (sector_owner == CHANNEL_PCM && sector_busy && !sector_done)
                audio_wait_count <= audio_wait_count + 1'b1;
            else audio_wait_count <= 0;

            if (audio_running && audio_switch_state==0 && !audio_image_changed) begin
                if (audio_fault) begin
                    audio_done <= 1;
                    audio_running <= 0;
                    if (pcm_overflow) pcm_error <= 8'h72;
                    else if (audio_wait_count >= PCM_SECTOR_TIMEOUT) pcm_error <= 8'h73;
                    else pcm_error <= audio_fs_error;
                    if (!audio_file_done) audio_abort <= 1;
                end else if (audio_file_done) begin
                    // EOF 连续循环；暂停期间启动只进入请求等待，不抢占 BMP 通道。
                    audio_streamer_start <= 1;
                end
            end

            // 图片已经提交才选音频。读图阶段也允许启动，首图无需等四张全部预载。
            if(PCM_ENABLE && scan_usable && audio_selected && !audio_attempted &&
               audio_switch_state==0 && !audio_image_changed &&
               (pcm_play_enable || source_state==SOURCE_STREAM)) begin
                audio_attempted<=1; audio_streamer_start<=1;
                audio_start<=1; audio_running<=1;
            end

            // ------------------------------------------------------------------
            // (b) 中止处理：优先于一切状态，直接回空闲
            // ------------------------------------------------------------------
            if (load_abort) begin
                // 中止不需要等待 SD 当前扇区完成；子模块也收到同一个 abort，
                // 下一次 load 会从干净的 IDLE 重新开始。
                source_state <= SOURCE_IDLE;
                load_error <= 8'h00;
            end else begin
                case (source_state)
                    // ----------------------------------------------------------
                    // (c) SOURCE_IDLE：等待并接受新加载事务
                    // ----------------------------------------------------------
                    SOURCE_IDLE: begin
                        // 只有空闲且没有扫描占用共享扇区通道时才接受新文件。
                        // 三个动作在同一拍完成：
                        //   ① 锁存参数（冻结副本，之后上层怎么改都影响不了本次读）
                        //   ② 清错误码
                        //   ③ 进入 SOURCE_ACK（开始与帧写端握手）
                        if (load_start && source_ready) begin
                            load_cluster_q <= load_cluster;
                            load_size_q <= load_size;
                            load_error <= 8'h00;
                            source_state <= SOURCE_ACK;
                        end else if (PCM_ENABLE && pcm_play_enable && scan_usable && source_ready &&
                                     audio_running && audio_switch_state==0 && !audio_image_changed && !audio_fault) begin
                            source_state <= SOURCE_AUDIO;
                        end
                    end

                    // ----------------------------------------------------------
                    // (d) SOURCE_ACK：向帧写端请求，等它应答
                    // ----------------------------------------------------------
                    SOURCE_ACK: begin
                        // write_req 保持为 1，直到帧写端回一个 ack；这是一种
                        // 电平握手，避免只给一个脉冲而被慢速下游漏采样。
                        // 先拿到帧写端的应答，再开始发送文件字节，避免
                        // BMP 像素在写 FIFO 尚未清空时被丢弃。
                        if (write_req_ack) begin
                            // 收到应答：同时启动两个并行引擎——
                            // streamer 读文件字节，decoder 解析成像素
                            streamer_start <= 1'b1;
                            decoder_start <= 1'b1;
                            source_state <= SOURCE_STREAM;
                        end
                    end

                    // ----------------------------------------------------------
                    // (e) SOURCE_STREAM：文件字节/像素流传输，等解码结束
                    // ----------------------------------------------------------
                    SOURCE_STREAM: begin
                        // decoder_done 是文件流已经结束的结果事件；错误码在此
                        // 汇总后保持到下一次 load_start。
                        if (!decoder_start && decoder_finished && (decoder_error!=0 || streamer_fs_error!=0 || scaler_finished)) begin
                            // 收尾：报完成事件，汇总错误码（decoder 优先于 streamer），
                            // 回到空闲等待下一次事务。
                            load_done <= 1'b1;
                            // 38只是上游错误的包装；保留原始FAT错误供UART定位。
                            if (decoder_error == 8'h38 && streamer_fs_error != 0)
                                load_error <= streamer_fs_error;
                            else if (decoder_error != 8'h00)
                                load_error <= decoder_error;
                            else if (streamer_fs_error != 8'h00)
                                load_error <= streamer_fs_error;
                            else
                                load_error <= 8'h00;
                            source_state <= SOURCE_IDLE;
                        end
                    end

                    SOURCE_AUDIO: begin
                        // 只在当前扇区完全结束后让出通道；不清 PCM FIFO/簇链位置。
                        if (!pcm_play_enable && !sector_busy && !audio_sector_req &&
                            sector_owner == CHANNEL_NONE)
                            source_state <= SOURCE_IDLE;
                    end

                    // 防御：异常状态兜底回空闲
                    default: source_state <= SOURCE_IDLE;
                endcase
            end
            // 错误退出优先于正常音频状态，避免同拍 case 覆盖退出动作。
            if (audio_fault && source_state == SOURCE_AUDIO)
                source_state <= SOURCE_IDLE;

            // 换音先停止新请求，等旧音频整扇区结束再中止文件流。
            // 双时钟FIFO清空必须完成双向握手，防止旧样本或旧Gray指针混入新文件。
            case(audio_switch_state)
                1: if(sector_owner!=CHANNEL_PCM && !audio_sector_req) begin
                    audio_abort<=1; audio_switch_state<=2;
                    if(source_state==SOURCE_AUDIO) source_state<=SOURCE_IDLE;
                end
                2: if(audio_clear_ack && !audio_file_busy) begin
                    audio_clear<=0; audio_switch_state<=3;
                end
                3: if(!audio_clear_ack) begin
                    audio_lookup_slot<=0; audio_lookup_index<=0; audio_switch_state<=4;
                end
                // 逐拍把播放序号映射回PIC文件名槽，因此PIC1/PIC5/PIC16缺号也匹配正确。
                4: if(scan_usable) begin
                    if(resource_valid[audio_lookup_slot] && audio_lookup_index==audio_image_q) begin
                        audio_cluster<=audio_clusters[audio_lookup_slot*32 +:32];
                        audio_size<=audio_sizes[audio_lookup_slot*32 +:32];
                        audio_selected<=audio_resource_valid[audio_lookup_slot];
                        audio_switch_state<=5;
                    end else if(audio_lookup_slot==15) audio_switch_state<=5;
                    else begin
                        audio_lookup_slot<=audio_lookup_slot+1'b1;
                        if(resource_valid[audio_lookup_slot]) audio_lookup_index<=audio_lookup_index+1'b1;
                    end
                end
                5: begin
                    if(audio_selected && (audio_size==0 || audio_size[0])) begin
                        audio_selected<=0; pcm_error<=8'h71;
                    end
                    pcm_commit_ack<=audio_commit_seen;
                    audio_switch_state<=0;
                end
                default: ;
            endcase
            // 新提交优先，快速切图只保留最近实际显示的图片。
            if(audio_image_changed) begin
                audio_commit_seen<=audio_commit_sync[2]; audio_image_q<=audio_image_sync;
                audio_switch_state<=1; audio_clear<=1;
                audio_running<=0; audio_selected<=0; audio_attempted<=0; pcm_error<=0;
            end
        end
    end
endmodule
