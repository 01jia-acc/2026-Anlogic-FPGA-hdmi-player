`timescale 1ns/1ps
// FAT32多图片调度：16个资源、四bank缓存、帧边界提交、后台预读下一张。
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
    // 连续无图像加载进展的最长时间；正常大图允许加载超过此时长。
    parameter integer LOAD_TIMEOUT_MS   = 5_000,
    parameter integer PCM_ENABLE = 0,
    parameter integer THUMBNAIL_ENABLE = 0,
    parameter integer SWITCH_MODE_ENABLE = 0,
    parameter integer VOLUME_CONTROL_ENABLE = 0
) (
    // ===================== 端口：系统时钟/复位 =====================
    input  wire        clk,            // 系统时钟 100MHz（顶层接 sd_card_clk）
    input  wire        rst,            // 高有效复位
    input wire pixel_clk, pcm_ready,
    input wire [3:0] pcm_image_id_pix,
    input wire pcm_commit_toggle_pix,
    output wire pcm_commit_ack,
    output wire [3:0] pcm_file_number_pix,
    output wire signed [15:0] pcm_sample,
    output wire pcm_valid, pcm_active, pcm_end, pcm_underflow,
    output wire [7:0] pcm_error,
    output reg pcm_mode,
    input wire switch_volume_n, // SW2: ON=volume keys
    output wire volume_mode,
    output wire [4:0] volume_level, // 0..20, 5% per step, reset=15 (75%)
    output reg zoom_mode,
    output reg [1:0] zoom_level, // 0=0.5x, 1=1x, 2=1.5x, 3=2x
    // [23]=menu, [22]=transaction epoch, [21:20]=focus bank,
    // [19:16]=complete banks, [15:0]=logical image ids (not physical banks).
    output wire [31:0] thumbnail_descriptor,
    input wire thumbnail_ack_epoch,

    // ===================== 端口：按键（低有效原始输入，内部会同步+去抖） =====================
    input  wire        key_next,       // "下一张"按键
    input  wire        key_auto,       // "自动"键：短按=轮播开/关，长按=上一张
    input  wire        key_audio_mode, // 板上 KEY3：短按切换 PCM/琶音
    input  wire        switch_pcm_n, switch_zoom_n, // SW3/SW4: closed = low

    // bmp_width/bmp_height 与旧版扫描参数一样仅为顶层兼容保留；FAT32 目录
    // 扫描提供图片列表，bmp24_decoder 内部校验固定的 640x480 格式。
    input  wire [15:0] bmp_width,      // 期望图片宽（不参与逻辑，兼容旧接口）
    input  wire [15:0] bmp_height,     // 期望图片高（不参与逻辑，兼容旧接口）

    // ===================== 端口：显示控制 =====================
    // 第一张图"文件读完+整帧写完"都完成后才置 1；=0 时 HDMI 输出黑色，
    // 防止帧缓存还没准备好就显示随机数据。
    input wire display_ack_valid,
    input wire [1:0] display_ack_bank,
    input wire [3:0] display_ack_image_id,
    output wire [3:0] display_image_id,
    output reg         display_valid,  // 控制 HDMI 输出的开关
    // 数码管/调试用状态码：0 初始化，1 扫描，2 等待写端，3 读文件，4 空闲，F 错误。
    output wire [3:0]  state_code,

    // ===================== 端口：帧写完成事件（跨时钟域） =====================
    // 来自 frame_read_write 所在时钟域（ext_mem_clk 125MHz 域）；每写完一帧
    // 翻转一次 0/1。本模块用三级移位同步后做边沿检测，恢复成单拍事件。
    input  wire        write_finish_toggle,
    // 四个固定帧缓存：bank 与逻辑图片编号分别保存。
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
    output wire [4:0]  debug_load_image_number,
    output wire [31:0] debug_load_cluster,
    output wire [31:0] debug_source_cluster,
    output wire [31:0] debug_load_size,
    // FAT/SD 细粒度诊断观察口，供顶层 UART 周期打印。
    output wire [3:0]  debug_scanner_state,
    output wire [31:0] debug_sector_lba,
    output wire [8:0]  debug_sector_byte_count,
    output wire        debug_sector_active,
    output wire [15:0] debug_sector_done_count,
    output wire [15:0] debug_resource_valid,
    output wire [4:0] debug_image_count,
    output wire [31:0] debug_root_crc,
    output wire [15:0] debug_root_bytes,
    output wire [4:0]  debug_sd_state,
    output wire        debug_sd_cmd_error,
    output wire        debug_sd_cmd_error_latched,
    output wire [63:0] debug_load_pipeline,
    output wire [63:0] debug_playlist,
    output wire [159:0] debug_pcm_status
);

    // 最多16个目录资源，四个bank只作缓存。文件完成与SDRAM写完同时满足才提交。
    wire sd_init_done, source_ready, source_load_busy, source_load_done;
    wire scan_busy, scan_done;
    wire [7:0] scan_error, source_load_error;
    wire [15:0] resource_valid;
    wire [511:0] resource_clusters, resource_sizes;
    wire debug_pic10_size_bit5;
    wire [31:0] resource0_cluster, resource1_cluster, resource2_cluster, resource3_cluster;
    wire [31:0] resource0_size, resource1_size, resource2_size, resource3_size;
    wire [3:0] source_debug_scanner_state;
    wire [15:0] source_debug_resource_valid;
    wire [31:0] source_debug_sector_lba;
    wire [8:0] source_debug_sector_byte_count;
    wire source_debug_sector_active;
    wire [15:0] source_debug_sector_done_count;
    wire [4:0] source_debug_sd_state;
    wire source_debug_sd_cmd_error, source_debug_sd_cmd_error_latched;
    wire [63:0] source_debug_load_pipeline;
    reg [63:0] aborted_load_pipeline;
    reg pipeline_frozen;
    reg scan_start_pulse, load_start_pulse, load_abort_pulse;
    reg scan_kicked, scan_completed, preload_halted;
    reg [15:0] image_resource_valid;
    // 扫描结束后资源表保持不变，直到下一次重新扫描；直接使用唯一参数表。
    // 无需再复制 1024 位，也不再让前四张走独立的参数路径。
    wire [511:0] image_clusters = resource_clusters;
    wire [511:0] image_sizes = resource_sizes;
    reg saved_pic10_size_bit5;
    // 前四槽别名用于原有调试接口。
    wire [31:0] img_cluster1 = image_clusters[63:32];
    reg [4:0] img_found_count;
    reg [3:0] img_idx, load_idx;
    reg [4:0] load_file_number; // Physical PIC number, captured by the resource lookup.
    reg [3:0] image_complete;
    reg [15:0] bank_image_ids;
    reg [2:0] preload_next_idx;
    reg load_is_demand;
    reg load_is_prefetch;
    reg [1:0] victim_cursor;
    reg next_req_pending, prev_req_pending;
    reg auto_play_en;
    reg [31:0] auto_cnt, long_press_cnt, load_timeout_cnt;
    reg key_auto_down_seen, long_press_marked, next_down_seen, next_release_pulse;
    reg load_busy, source_done_seen, write_done_seen;
    wire write_finish_pulse;
    reg [1:0] load_prepare;
    localparam integer LOAD_TIMEOUT_CYCLES = (CLK_FREQ_HZ / 1000) * LOAD_TIMEOUT_MS;
    // The decode offset advances one byte per SD-domain clock at most, so
    // tracking its LSB detects every advance using only one extra register.
    // Do not count unrelated PCM traffic as image progress.
    reg decode_offset_lsb_seen;
    always @(posedge clk or posedge rst) begin
        if(rst) decode_offset_lsb_seen<=0;
        else decode_offset_lsb_seen<=source_debug_load_pipeline[0];
    end
    wire load_progress = source_debug_load_pipeline[0]!=decode_offset_lsb_seen ||
        write_en || (source_load_done && !source_done_seen) || write_finish_pulse;
    wire [63:0] live_load_pipeline = {source_debug_load_pipeline[63:48],
        source_done_seen, write_done_seen, load_prepare, load_busy,
        source_debug_load_pipeline[42:0]};
    assign debug_load_pipeline = pipeline_frozen ? aborted_load_pipeline : live_load_pipeline;
    wire pipeline_timeout = load_busy && load_prepare==0 &&
        !load_progress && !(source_load_done && source_load_error!=0) &&
        load_timeout_cnt>=LOAD_TIMEOUT_CYCLES;
    // Capture at the timeout decision, before the scheduler clears its flags
    // and before the registered abort pulse resets the source on the next edge.
    always @(posedge clk or posedge rst) begin
        if(rst) begin pipeline_frozen<=0; aborted_load_pipeline<=0; end
        else if(load_start_pulse) pipeline_frozen<=0;
        else if(pipeline_timeout) begin
            pipeline_frozen<=1; aborted_load_pipeline<=live_load_pipeline;
        end
    end
    reg [3:0] lookup_slot, lookup_image;
    reg [31:0] load_cluster_lut, load_size_lut;
    // 16 选 1 拆成两级 4 选 1，并将有效槽判断与参数锁存分拍。
    reg [127:0] lookup_cluster_groups, lookup_size_groups;
    genvar lookup_group;
    generate for(lookup_group=0; lookup_group<4; lookup_group=lookup_group+1) begin: metadata_lookup
        always @(posedge clk or posedge rst) begin
            if(rst) begin
                lookup_cluster_groups[lookup_group*32 +:32] <= 0;
                lookup_size_groups[lookup_group*32 +:32] <= 0;
            end else begin
                lookup_cluster_groups[lookup_group*32 +:32] <=
                    image_clusters[lookup_group*128 + lookup_slot[1:0]*32 +:32];
                lookup_size_groups[lookup_group*32 +:32] <=
                    image_sizes[lookup_group*128 + lookup_slot[1:0]*32 +:32];
            end
        end
    end endgenerate
    reg [2:0] wrfin_tgl_sync;
    reg [7:0] last_error;
    wire key_next_press, key_next_stable, key_auto_press, key_auto_stable, key_audio_mode_press;
    wire key_audio_stable;
    wire switch_pcm_stable, switch_zoom_stable;
    reg audio_down_seen, audio_long_marked, zoom_auto_down;
    reg [31:0] audio_hold_count;
    reg [3:0] zoom_image_id;
    reg thumbnail_mode, thumbnail_epoch;
    reg [1:0] thumbnail_focus;
    reg thumbnail_ack1, thumbnail_ack2;
    reg thumbnail_select;
    reg [1:0] thumbnail_selected_bank;
    reg thumbnail_pending;
    reg [3:0] thumbnail_start;
    // Preview slots follow playlist order; SDRAM banks remain freely mapped.
    // Keep the entry window until exit ACK, including selection's commit delay.
    wire thumbnail_hold = THUMBNAIL_ENABLE &&
        (thumbnail_mode || thumbnail_epoch!=thumbnail_ack2);
    wire [2:0] preload_limit = img_found_count > 4 ? 3'd4 : img_found_count[2:0];
    wire [4:0] window_max = img_found_count>4 ? img_found_count-5'd4 : 5'd0;
    wire [4:0] window_preferred = img_idx>2 ? {1'b0,img_idx}-5'd2 : 5'd0;
    wire [3:0] window_start = window_preferred>window_max ? window_max[3:0] : window_preferred[3:0];
    wire [3:0] menu_start = thumbnail_hold ? thumbnail_start : window_start;
    reg [7:0] thumbnail_banks;
    reg [15:0] thumbnail_ids;
    reg [3:0] thumbnail_complete;
    reg window_missing;
    reg [3:0] window_load_idx;
    reg [1:0] window_victim;
    reg window_victim_valid;
    reg slot_found;
    integer slot, bank_scan;
    always @* begin
        thumbnail_banks=0; thumbnail_ids=0; thumbnail_complete=0;
        window_missing=0; window_load_idx=window_start;
        window_victim=0; window_victim_valid=0; slot_found=0;
        for(slot=3;slot>=0;slot=slot-1) begin
            thumbnail_ids[slot*4 +:4]=menu_start+slot;
            slot_found=0;
            for(bank_scan=0;bank_scan<4;bank_scan=bank_scan+1) begin
                if(image_complete[bank_scan] && bank_image_ids[bank_scan*4 +:4]==menu_start+slot && slot<preload_limit) begin
                    thumbnail_banks[slot*2 +:2]=bank_scan;
                    thumbnail_complete[slot]=1;
                end
                if(image_complete[bank_scan] && bank_image_ids[bank_scan*4 +:4]==window_start+slot)
                    slot_found=1;
            end
            if(slot<preload_limit && !slot_found) begin
                window_missing=1; window_load_idx=window_start+slot;
            end
        end
        for(bank_scan=3;bank_scan>=0;bank_scan=bank_scan-1)
            if(bank_scan!=disp_buf_idx && (!display_ack_valid || bank_scan!=display_ack_bank) &&
               (!image_complete[bank_scan] || bank_image_ids[bank_scan*4 +:4]<window_start ||
                {1'b0,bank_image_ids[bank_scan*4 +:4]}>={1'b0,window_start}+preload_limit)) begin
                window_victim=bank_scan; window_victim_valid=1;
            end
    end
    wire thumbnail_ready = thumbnail_epoch==thumbnail_ack2;
    assign thumbnail_descriptor={thumbnail_banks,thumbnail_mode,thumbnail_epoch,thumbnail_focus,
        thumbnail_complete,thumbnail_ids};
    localparam integer LONG_PRESS_CYCLES = (CLK_FREQ_HZ / 1000) * LONG_PRESS_MS;
    // Confirm/cancel owns this cycle. Simultaneous navigation must not change
    // the exit packet's target bank after the selection has already been made.
    wire thumbnail_action = thumbnail_mode && thumbnail_ready &&
        ((SWITCH_MODE_ENABLE && switch_zoom_stable) ||
         (key_audio_stable && audio_down_seen && !audio_long_marked) ||
         (!key_audio_stable && audio_hold_count>=LONG_PRESS_CYCLES-1 && !audio_long_marked));
    assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];
    // 双向邮箱确认像素域已切到所请求的帧之后，才允许复用旧显示bank。
    wire display_settled = display_ack_valid && display_ack_bank == disp_buf_idx &&
                           display_ack_image_id == img_idx;
    wire auto_tick = auto_cnt == CLK_FREQ_HZ - 1;
    wire [3:0] next_idx = {1'b0,img_idx} + 5'd1 >= img_found_count ? 4'd0 : img_idx + 1'b1;
    wire [3:0] prev_idx = img_idx == 0 ? img_found_count - 1'b1 : img_idx - 1'b1;
    // 没有按键请求时也查询下一张，供后台预读使用；手动请求优先。
    wire [3:0] wanted_idx = !next_req_pending && prev_req_pending ? prev_idx : next_idx;
    reg cache_hit;
    reg [1:0] cache_bank, victim_bank;
    integer b, candidate;
    always @* begin
        cache_hit = 0;
        cache_bank = 0;
        victim_bank = victim_cursor;
        for (b=0; b<4; b=b+1)
            if (image_complete[b] && bank_image_ids[b*4 +: 4] == wanted_idx) begin
                cache_hit = 1;
                cache_bank = b;
            end
        // 优先空bank，否则轮换替换；当前实际显示和请求显示bank均禁止覆盖。
        for (b=3; b>=0; b=b-1) begin
            candidate = (victim_cursor + b) & 3;
            if (candidate != disp_buf_idx && (!display_ack_valid || candidate != display_ack_bank))
                victim_bank = candidate;
        end
        for (b=3; b>=0; b=b-1)
            if (!image_complete[b] && b != disp_buf_idx &&
                (!display_ack_valid || b != display_ack_bank))
                victim_bank = b;
    end
    wire preload_finished = preload_next_idx >= preload_limit;
    // 仅真正需要读图时让 PCM 结束当前扇区并释放事务入口；缓存命中不暂停音频。
    wire image_read_needed = !thumbnail_hold && !load_busy && !preload_halted &&
        (!preload_finished || (display_valid && display_settled &&
                              img_found_count > 1 && (THUMBNAIL_ENABLE ?
                               ((next_req_pending || prev_req_pending) ? !cache_hit : window_missing && window_victim_valid) : !cache_hit)));
    function [4:0] resource_count;
        input [15:0] mask;
        integer i;
        begin
            resource_count=0;
            for(i=0;i<10;i=i+1) resource_count=resource_count+mask[i];
        end
    endfunction
    // 每拍查看一个目录槽，播放列表最多十项。
    // 查到后将参数保持在寄存器中，下一阶段才发 load_start。
    assign debug_load_image_number = {1'b0,load_idx} + 5'd1;
    assign debug_load_cluster = load_cluster_lut;
    assign debug_load_size = load_size_lut;
    // Raw scanner mask / saved scheduler mask / count / displayed playlist
    // number / last selected physical PIC number / loader and cache flags.
    // Transfer this whole word through a mailbox before UART sampling.
    assign debug_playlist = {resource_valid, (image_resource_valid & 16'h03ff),
        3'd0, img_found_count,
        (display_valid ? {4'd0,img_idx}+8'd1 : 8'd0),
        saved_pic10_size_bit5, resource_sizes[293], debug_pic10_size_bit5, load_file_number,
        preload_halted, load_busy, load_prepare, image_complete};
    assign display_image_id = img_idx;
    assign debug_auto_enabled = auto_play_en;
    assign state_code = !sd_init_done ? 4'd0 : scan_busy ? 4'd1 :
                        load_busy ? (source_load_busy ? 4'd3 : 4'd2) :
                        last_error != 0 ? 4'hf : 4'd4;
    assign debug_sd_init_done=sd_init_done;
    assign debug_scan_busy=scan_busy;
    assign debug_scan_done=scan_completed;
    assign debug_scan_error=last_error;
    assign debug_source_ready=source_ready;
    assign debug_load_busy=load_busy;
    assign debug_load_error=source_load_error;
    assign debug_scanner_state=source_debug_scanner_state;
    assign debug_sector_lba=source_debug_sector_lba;
    assign debug_sector_byte_count=source_debug_sector_byte_count;
    assign debug_sector_active=source_debug_sector_active;
    assign debug_sector_done_count=source_debug_sector_done_count;
    assign debug_resource_valid=source_debug_resource_valid;
    assign debug_image_count=img_found_count;
    assign debug_sd_state=source_debug_sd_state;
    assign debug_sd_cmd_error=source_debug_sd_cmd_error;
    assign debug_sd_cmd_error_latched=source_debug_sd_cmd_error_latched;

    key_press_debounce_fat32 #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) u_key_next (
        .clk(clk),.rst(rst),.button_in(key_next),.press_pulse(key_next_press),
        .button_stable_out(key_next_stable));
    key_press_debounce_fat32 #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) u_key_auto (
        .clk(clk),.rst(rst),.button_in(key_auto),.press_pulse(key_auto_press),
        .button_stable_out(key_auto_stable));
    key_press_debounce_fat32 #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) u_key_audio_mode (
        .clk(clk),.rst(rst),.button_in(key_audio_mode),.press_pulse(key_audio_mode_press),
        .button_stable_out(key_audio_stable));
    generate if(SWITCH_MODE_ENABLE) begin : g_mode_switches
        key_press_debounce_fat32 #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) pcm_switch (
            .clk(clk), .rst(rst), .button_in(switch_pcm_n),
            .press_pulse(), .button_stable_out(switch_pcm_stable));
        key_press_debounce_fat32 #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) zoom_switch (
            .clk(clk), .rst(rst), .button_in(switch_zoom_n),
            .press_pulse(), .button_stable_out(switch_zoom_stable));
    end else begin : g_key_modes
        assign switch_pcm_stable=1'b1;
        assign switch_zoom_stable=1'b1;
    end endgenerate
    wire volume_switch_stable;
    generate if(VOLUME_CONTROL_ENABLE) begin : g_volume_switch
        key_press_debounce_fat32 #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) volume_switch (
            .clk(clk), .rst(rst), .button_in(switch_volume_n),
            .press_pulse(), .button_stable_out(volume_switch_stable));
    end else begin : g_no_volume_switch
        assign volume_switch_stable=1'b1;
    end endgenerate
    wire navigation_keys_enabled;
    volume_key_control #(.LONG_PRESS_CYCLES(LONG_PRESS_CYCLES)) u_volume_keys (
        .clk(clk), .rst(rst), .mode(!volume_switch_stable),
        .up_press(key_next_press), .down_press(key_auto_press),
        .up_released(key_next_stable), .down_released(key_auto_stable),
        .navigation_enabled(navigation_keys_enabled), .level(volume_level));
    assign volume_mode=!volume_switch_stable;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            pcm_mode<=0; next_down_seen<=0; next_release_pulse<=0;
            zoom_mode<=0; zoom_level<=1; zoom_image_id<=0;
            audio_down_seen<=0; audio_long_marked<=0; audio_hold_count<=0;
            zoom_auto_down<=0;
            thumbnail_mode<=0; thumbnail_epoch<=0; thumbnail_focus<=0;
            thumbnail_ack1<=0; thumbnail_ack2<=0;
            thumbnail_select<=0; thumbnail_selected_bank<=0;
            thumbnail_pending<=0; thumbnail_start<=0;
        end
        else begin
            next_release_pulse<=0;
            thumbnail_select<=0;
            thumbnail_ack1<=thumbnail_ack_epoch; thumbnail_ack2<=thumbnail_ack1;
            if(thumbnail_pending && !thumbnail_hold && display_valid && display_settled &&
               (!SWITCH_MODE_ENABLE || !switch_zoom_stable) &&
               !load_busy && preload_finished && !window_missing &&
               !next_req_pending && !prev_req_pending && !next_release_pulse &&
               key_audio_stable && !audio_down_seen) begin
                thumbnail_pending<=0; thumbnail_mode<=1; thumbnail_epoch<=~thumbnail_epoch;
                thumbnail_start<=window_start; thumbnail_focus<=img_idx-window_start;
            end
            // Defer short KEY3 action until release: a long press must not
            // switch the audio source on its way into/out of zoom mode.
            if(key_audio_mode_press) audio_down_seen<=1;
            if(!key_audio_stable) begin
                if(audio_hold_count<LONG_PRESS_CYCLES-1) audio_hold_count<=audio_hold_count+1'b1;
                else if(!audio_long_marked) begin
                    audio_long_marked<=1;
                    if(thumbnail_mode) begin
                        if(thumbnail_ready) begin
                            thumbnail_mode<=0; thumbnail_epoch<=~thumbnail_epoch;
                            // Exit packet identifies the original image on cancel.
                            thumbnail_focus<=img_idx-thumbnail_start;
                        end
                    end else if(thumbnail_pending) thumbnail_pending<=0;
                    else if(!SWITCH_MODE_ENABLE && !thumbnail_hold && display_valid && display_settled)
                        zoom_mode<=~zoom_mode;
                end
            end else begin
                audio_hold_count<=0; audio_long_marked<=0;
                if(audio_down_seen) begin
                    audio_down_seen<=0;
                    if(!audio_long_marked && (!SWITCH_MODE_ENABLE || !switch_zoom_stable)) begin
                        if(THUMBNAIL_ENABLE && thumbnail_mode) begin
                            if(thumbnail_ready && thumbnail_complete[thumbnail_focus]) begin
                                thumbnail_selected_bank<=thumbnail_banks[thumbnail_focus*2 +:2];
                                thumbnail_select<=1;
                                thumbnail_mode<=0; thumbnail_epoch<=~thumbnail_epoch;
                                zoom_level<=1;
                            end
                        end else if(THUMBNAIL_ENABLE && zoom_mode) begin
                            if(!thumbnail_hold && display_valid && display_settled &&
                               (!load_busy || !load_is_demand)) begin
                                thumbnail_pending<=~thumbnail_pending;
                            end
                        end else if(PCM_ENABLE && !SWITCH_MODE_ENABLE) pcm_mode<=~pcm_mode;
                    end
                end
            end
            if(!navigation_keys_enabled) next_down_seen<=0;
            else if(key_next_press) next_down_seen<=1;
            if(navigation_keys_enabled && key_next_stable && next_down_seen) begin
                next_down_seen<=0;
                if(thumbnail_mode) begin
                    if(thumbnail_ready && !thumbnail_action) thumbnail_focus<=thumbnail_focus+1'b1;
                end else if(zoom_mode) begin if(zoom_level<3) zoom_level<=zoom_level+1'b1; end
                else next_release_pulse<=1;
            end
            if(!navigation_keys_enabled) zoom_auto_down<=0;
            else if(key_auto_press && zoom_mode) zoom_auto_down<=1;
            if(navigation_keys_enabled && key_auto_stable && zoom_auto_down) begin
                zoom_auto_down<=0;
                if(thumbnail_mode) begin
                    if(thumbnail_ready && !thumbnail_action) thumbnail_focus<=thumbnail_focus-1'b1;
                end else if(zoom_mode && zoom_level>0) zoom_level<=zoom_level-1'b1;
            end
            if(img_idx!=zoom_image_id) begin zoom_image_id<=img_idx; zoom_level<=1; end
            if(SWITCH_MODE_ENABLE) begin
                pcm_mode<=PCM_ENABLE && !switch_pcm_stable;
                if(switch_zoom_stable) begin
                    // Cancel queued entry and publish the normal cancel packet.
                    // Keep the original bank locked until the reader ACKs it.
                    thumbnail_pending<=0;
                    if(thumbnail_mode) begin
                        if(thumbnail_ready) begin
                            thumbnail_mode<=0; thumbnail_epoch<=~thumbnail_epoch;
                            thumbnail_focus<=img_idx-thumbnail_start;
                            thumbnail_select<=0;
                        end
                    end else if(!thumbnail_hold) zoom_mode<=0;
                end else if(!zoom_mode && !thumbnail_hold && display_valid && display_settled)
                    zoom_mode<=1;
            end
        end
    end
    sd_fat32_image_source #(.PCM_ENABLE(PCM_ENABLE)) u_image_source (
        .clk(clk), .rst(rst),
        .pixel_clk(pixel_clk), .pcm_ready(pcm_ready), .pcm_sample(pcm_sample),
        .pcm_image_id_pix(pcm_image_id_pix), .pcm_commit_toggle_pix(pcm_commit_toggle_pix),
        .pcm_commit_ack(pcm_commit_ack),
        .pcm_file_number_pix(pcm_file_number_pix),
        .pcm_valid(pcm_valid), .pcm_active(pcm_active), .pcm_end(pcm_end),
        .pcm_underflow(pcm_underflow), .pcm_error(pcm_error),
        .pcm_play_enable(scan_completed && load_prepare == 0 && !image_read_needed &&
                         (preload_halted || preload_finished || thumbnail_hold)),
        // ---- 扫描事务：顶层发脉冲，source 回报 busy/结果 ----
        .scan_start(scan_start_pulse), .scan_busy(scan_busy),
        .scan_done(scan_done), .scan_error(scan_error),
        .resource_valid(resource_valid),
        .resource_clusters(resource_clusters), .resource_sizes(resource_sizes),
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
        .debug_pic10_size_bit5(debug_pic10_size_bit5),
        .debug_root_crc(debug_root_crc),
        .debug_root_bytes(debug_root_bytes),
        .debug_sd_state(source_debug_sd_state),
        .debug_sd_cmd_error(source_debug_sd_cmd_error),
        .debug_sd_cmd_error_latched(source_debug_sd_cmd_error_latched),
        .debug_load_pipeline(source_debug_load_pipeline),
        .debug_accepted_cluster(debug_source_cluster),
        .debug_pcm_status(debug_pcm_status)
    );


    always @(posedge clk or posedge rst) begin
        if(rst) begin
            scan_start_pulse<=0; load_start_pulse<=0; load_abort_pulse<=0;
            scan_kicked<=0; scan_completed<=0; preload_halted<=0;
            image_resource_valid<=0; saved_pic10_size_bit5<=0;
            img_found_count<=0; img_idx<=0; load_idx<=0;
            load_file_number<=0;
            image_complete<=0; bank_image_ids<=0; preload_next_idx<=0;
            load_is_demand<=0; load_is_prefetch<=0; victim_cursor<=0;
            write_buf_idx<=0; disp_buf_idx<=0; display_valid<=0;
            next_req_pending<=0; prev_req_pending<=0;
            auto_play_en<=0; auto_cnt<=0; long_press_cnt<=0;
            key_auto_down_seen<=0; long_press_marked<=0;
            load_busy<=0; source_done_seen<=0; write_done_seen<=0;
            load_prepare<=0; lookup_slot<=0; lookup_image<=0;
            load_cluster_lut<=0; load_size_lut<=0;
            load_timeout_cnt<=0; wrfin_tgl_sync<=0; last_error<=0;
        end else begin
            scan_start_pulse<=0; load_start_pulse<=0; load_abort_pulse<=0;
            wrfin_tgl_sync<={wrfin_tgl_sync[1:0],write_finish_toggle};
            if(!sd_init_done) begin
                scan_kicked<=0; scan_completed<=0; display_valid<=0;
                image_complete<=0; img_found_count<=0; image_resource_valid<=0;
                saved_pic10_size_bit5<=0;
                load_file_number<=0;
                preload_next_idx<=0; preload_halted<=0; load_busy<=0;
                load_prepare<=0;
                source_done_seen<=0; write_done_seen<=0;
                next_req_pending<=0; prev_req_pending<=0; auto_play_en<=0; auto_cnt<=0;
            end else begin
                if(!scan_kicked && source_ready) begin
                    scan_start_pulse<=1; scan_kicked<=1;
                end
                if(scan_done) begin
                    scan_completed<=1;
                    image_resource_valid<=resource_valid;
                    saved_pic10_size_bit5<=resource_sizes[293];
                    img_found_count<=resource_count(resource_valid);
                    last_error<=scan_error;
                end
                if(load_busy) begin
                    if(load_prepare == 1) begin
                        if(image_resource_valid[lookup_slot] && lookup_image == load_idx) begin
                            load_file_number <= {1'b0,lookup_slot}+5'd1;
                            load_prepare <= 3;
                        end else if(lookup_slot == 15) begin
                            load_busy<=0; load_prepare<=0;
                            preload_halted<=1; last_error<=8'h22;
                        end else begin
                            lookup_slot <= lookup_slot + 1'b1;
                            if(image_resource_valid[lookup_slot]) lookup_image<=lookup_image+1'b1;
                        end
                    end else if(load_prepare == 3) begin
                        load_cluster_lut <= lookup_cluster_groups[lookup_slot[3:2]*32 +:32];
                        load_size_lut <= lookup_size_groups[lookup_slot[3:2]*32 +:32];
                        load_prepare <= 2;
                    end else if(load_prepare == 2) begin
                        if(source_ready) begin
                            load_start_pulse<=1; load_prepare<=0;
                        end
                    end else if(source_load_done && source_load_error != 0) begin
                        load_busy<=0; preload_halted<=1; last_error<=source_load_error;
                        source_done_seen<=0; write_done_seen<=0;
                    end else if(pipeline_timeout) begin
                        load_abort_pulse<=1; load_busy<=0; preload_halted<=1; last_error<=8'he0;
                        source_done_seen<=0; write_done_seen<=0;
                    end else begin
                        if(load_progress) load_timeout_cnt<=0;
                        else load_timeout_cnt<=load_timeout_cnt+1'b1;
                        if(source_load_done) source_done_seen<=1;
                        if(write_finish_pulse) write_done_seen<=1;
                        if((source_done_seen || source_load_done) &&
                           (write_done_seen || write_finish_pulse)) begin
                            load_busy<=0; source_done_seen<=0; write_done_seen<=0;
                            image_complete[write_buf_idx]<=1;
                            bank_image_ids[write_buf_idx*4 +: 4]<=load_idx;
                            last_error<=0;
                            if(load_is_demand || !display_valid) begin
                                img_idx<=load_idx; disp_buf_idx<=write_buf_idx; display_valid<=1;
                            end
                            if(!load_is_demand && !load_is_prefetch)
                                preload_next_idx<=preload_next_idx+1'b1;
                            victim_cursor<=write_buf_idx+1'b1;
                        end
                    end
                end else load_timeout_cnt<=0;

                if(scan_completed && img_found_count != 0) begin
                    if(thumbnail_hold) begin next_req_pending<=0; prev_req_pending<=0; end
                    else if(next_release_pulse) next_req_pending<=1;
                    if(key_auto_press && !zoom_mode && navigation_keys_enabled) key_auto_down_seen<=1;
                    if(zoom_mode || !navigation_keys_enabled) begin
                        long_press_cnt<=0; long_press_marked<=0; key_auto_down_seen<=0;
                    end else if(!key_auto_stable && key_auto_down_seen) begin
                        if(long_press_cnt < LONG_PRESS_CYCLES-1) long_press_cnt<=long_press_cnt+1'b1;
                        else if(!long_press_marked) begin
                            long_press_marked<=1;
                            if(img_found_count>1) prev_req_pending<=1;
                        end
                    end else begin
                        long_press_cnt<=0; long_press_marked<=0;
                        if(key_auto_down_seen) begin
                            if(!long_press_marked && img_found_count>1) auto_play_en<=~auto_play_en;
                            key_auto_down_seen<=0;
                        end
                    end
                    if(auto_play_en && !volume_mode && !zoom_mode && display_valid && display_settled &&
                       (!load_busy || load_is_prefetch) &&
                       !next_req_pending && !prev_req_pending && preload_next_idx>=preload_limit &&
                       img_found_count>1) begin
                        if(auto_tick) begin next_req_pending<=1; auto_cnt<=0; end
                        else auto_cnt<=auto_cnt+1'b1;
                    end else auto_cnt<=0;

                    // The menu reads ALL completed banks. Freeze replacement
                    // until the reader acknowledges the exit descriptor, not
                    // merely until the control-side menu bit goes low.
                    if(thumbnail_select) begin
                        disp_buf_idx<=thumbnail_selected_bank;
                        img_idx<=bank_image_ids[thumbnail_selected_bank*4 +:4];
                        next_req_pending<=0; prev_req_pending<=0;
                    end else if(thumbnail_hold) begin
                        // Entry waits for a complete, idle cache window.
                        // All four mapped banks stay protected through exit ACK.
                    end else if(!load_busy && !preload_halted && preload_next_idx<preload_limit) begin
                        if(source_ready) begin
                            load_idx<={1'b0,preload_next_idx};
                            write_buf_idx<=preload_next_idx[1:0];
                            load_is_demand<=0; load_is_prefetch<=0;
                            load_prepare<=1; lookup_slot<=0; lookup_image<=0; load_busy<=1;
                            source_done_seen<=0; write_done_seen<=0; load_timeout_cnt<=0;
                        end
                    end else if((!load_busy || load_is_prefetch) && preload_finished &&
                                display_valid && display_settled &&
                                (next_req_pending || prev_req_pending)) begin
                        if(img_found_count==1) begin
                            next_req_pending<=0; prev_req_pending<=0;
                        end else if(cache_hit) begin
                            disp_buf_idx<=cache_bank; img_idx<=wanted_idx;
                            if(next_req_pending) next_req_pending<=0;
                            else prev_req_pending<=0;
                        end else if(preload_halted) begin
                            // 写端可能未完成错误帧，保持旧画面并停止再次使用写通道。
                            next_req_pending<=0; prev_req_pending<=0;
                        end else if(!load_busy && source_ready) begin
                            load_idx<=wanted_idx; write_buf_idx<=victim_bank;
                            image_complete[victim_bank]<=0;
                            load_is_demand<=1; load_is_prefetch<=0;
                            load_prepare<=1; lookup_slot<=0; lookup_image<=0; load_busy<=1;
                            source_done_seen<=0; write_done_seen<=0; load_timeout_cnt<=0;
                            if(next_req_pending) next_req_pending<=0;
                            else prev_req_pending<=0;
                        end
                    end else if(!load_busy && preload_finished && !preload_halted &&
                                display_valid && display_settled && img_found_count>1 &&
                                !next_req_pending && !prev_req_pending &&
                                (THUMBNAIL_ENABLE ? window_missing && window_victim_valid : !cache_hit) && source_ready) begin
                        // Fill missing window members; legacy mode prefetches next.
                        // Completion publishes cache only, never changes display.
                        load_idx<=THUMBNAIL_ENABLE ? window_load_idx : next_idx;
                        write_buf_idx<=THUMBNAIL_ENABLE ? window_victim : victim_bank;
                        image_complete[THUMBNAIL_ENABLE ? window_victim : victim_bank]<=0;
                        load_is_demand<=0; load_is_prefetch<=1;
                        load_prepare<=1; lookup_slot<=0; lookup_image<=0; load_busy<=1;
                        source_done_seen<=0; write_done_seen<=0; load_timeout_cnt<=0;
                    end
                end
            end
        end
    end
endmodule

// Debounced key ownership. A switch transition cancels held gestures in both
// directions. Only a fresh short press/release changes volume; no auto-repeat.
module volume_key_control #(parameter integer LONG_PRESS_CYCLES=100000000) (
    input wire clk, rst, mode,
    input wire up_press, down_press, up_released, down_released,
    output wire navigation_enabled,
    output reg [4:0] level
);
    localparam integer COUNT_W=LONG_PRESS_CYCLES>1 ? $clog2(LONG_PRESS_CYCLES+1) : 1;
    reg mode_q, blocked, up_seen, down_seen;
    reg [COUNT_W-1:0] up_count, down_count;
    wire changed=mode!=mode_q;
    wire up_short=up_seen && up_released && up_count<LONG_PRESS_CYCLES;
    wire down_short=down_seen && down_released && down_count<LONG_PRESS_CYCLES;
    assign navigation_enabled=!mode && !changed && !blocked;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            level<=15; mode_q<=0; blocked<=1;
            up_seen<=0; down_seen<=0; up_count<=0; down_count<=0;
        end else begin
            mode_q<=mode;
            if(changed) blocked<=1;
            else if(up_released && down_released) blocked<=0;
            if(changed || blocked || !mode) begin
                up_seen<=0; down_seen<=0; up_count<=0; down_count<=0;
            end else begin
                if(up_press) begin up_seen<=1; up_count<=0; end
                else if(up_seen && !up_released && up_count<LONG_PRESS_CYCLES)
                    up_count<=up_count+1'b1;
                if(down_press) begin down_seen<=1; down_count<=0; end
                else if(down_seen && !down_released && down_count<LONG_PRESS_CYCLES)
                    down_count<=down_count+1'b1;
                if(up_released) up_seen<=0;
                if(down_released) down_seen<=0;
                if(up_short && !down_short && level<20) level<=level+1'b1;
                else if(down_short && !up_short && level>0) level<=level-1'b1;
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
    localparam integer DEBOUNCE_WIDTH = DEBOUNCE_CYCLES>1 ? $clog2(DEBOUNCE_CYCLES) : 1;
    // 两级同步寄存器：先把异步按键输入带入 clk 域，再供后级使用。
    reg button_sync0, button_sync1, button_stable;
    // 只有连续 DEBOUNCE_CYCLES 个周期与稳定值不一致，才确认状态改变。
    reg [DEBOUNCE_WIDTH-1:0] cnt;

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
