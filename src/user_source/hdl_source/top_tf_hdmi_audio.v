
// ============================================================================
// top_tf_hdmi_audio.v —— TF 卡 BMP 图片 → SDRAM 帧缓存 → HDMI 显示顶层
// ============================================================================
//
// 【这个文件的定位】
//
// 本文件主要是“系统级连接文件”，不是把所有功能都写在一个大状态机里。
// 它负责：
//   1. 产生并分配多个时钟；
//   2. 等待 PLL 稳定后释放统一复位；
//   3. 连接 TF 卡读取、BMP 解析、FIFO 和 SDRAM 帧缓存；
//   4. 连接视频时序、HDMI 视频发送器和 HDMI PHY。
//
// 【总体数据流】
//
//   TF 卡
//     │ SPI：sd_miso/sd_mosi/sd_dclk/sd_ncs
//     ▼
//   sd_card_bmp_fat32：目录扫描、簇链读取、BMP 解码、预加载调度
//     │ write_en + write_data（32 bit，一个像素一个存储字）
//     ▼
//   media_frame_loader → media_sdram_write_cdc
//     │ 写命令 FIFO：sd_card_clk 域 → ext_mem_clk 域
//     ▼
//   SDRAM 四帧缓存：Buffer 0 / 1 / 2 / 3
//     │ media_frame_reader：ext_mem_clk 域 → video_clk 域
//     ▼
//   video_delay / video_rgb_to_axis_640x480
//     │ AXI-Stream 视频
//     ▼
//   hdmi_1_4b_transmitter_core_wrapper
//     │ 10 bit TMDS 数据
//     ▼
//   hdmi_phy_wrapper → HDMI 差分输出
//
//   media_display_commit 在视频帧请求边界统一提交显示 bank 和琶音编号；
//   media_hdmi_audio_bridge 产生 48 kHz PCM/ACR，送到同一 HDMI 发射核。
//
// 【四个时钟域】
//
//   clk          ：板载输入时钟，复位计数和数码管扫描
//   sd_card_clk  ：SD 卡读取域，约 100 MHz
//   ext_mem_clk  ：SDRAM 控制域，约 125 MHz
//   video_clk    ：640×480 视频像素域，约 25 MHz
//
// 【先只追踪这 10 个关键变量】
//
//   write_req / write_req_ack：请求准备写一帧 / 写侧已准备好
//   write_en / write_data    ：当前像素有效 / 当前 32 bit 像素
//   write_buf_idx            ：当前写入哪个帧缓冲区
//   frame_write_finish       ：SDRAM 写完整帧
//   frame_write_toggle_mem   ：把“写完一帧”变成跨域事件
//   disp_buf_idx             ：当前 HDMI 显示哪个帧缓冲区
//   video_read_en/data       ：HDMI 侧取像素 / 取到的像素
//
// 其余标志位先按“属于哪一条链路、哪个时钟域”归类，不要一次全部记忆。
// ============================================================================

module top(
    input                       clk,            // 【输入】板载系统时钟，约 50 MHz
    input                       rst_n,          // 【输入】外部低有效总复位
    input                       key1,           // 板上 KEY2（B2）：按下并松开切下一张
    input                       key2,           // 板上 KEY4（C1）：短按轮播开/关，长按上一张
    input                       key3,           // 板上 KEY3（B1）：短按切换 PCM/琶音

	output [5:0]                seg_sel,        // 【输出到 seg_scan/数码管】位选
	output [7:0]                seg_data,       // 【输出到数码管】段选数据

    // HDMI TMDS
    output                      HDMI_CLK_P,    // 【输出到 HDMI 接口】TMDS 时钟差分正端
    output                      HDMI_D2_P,     // 【输出到 HDMI 接口】TMDS 数据通道 2
    output                      HDMI_D1_P,     // 【输出到 HDMI 接口】TMDS 数据通道 1
    output                      HDMI_D0_P,     // 【输出到 HDMI 接口】TMDS 数据通道 0

    // HDMI DDC
    output                      HDMI_DDC_SCL,  // 【输出到 HDMI 接口】DDC/I2C 时钟
    inout                       HDMI_DDC_SDA,  // 【双向连接 HDMI】DDC/I2C 数据

    // TF card SPI
    output                      sd_ncs,        // 【输出到 TF 卡】SPI 片选，低有效
    output                      sd_dclk,       // 【输出到 TF 卡】SPI 时钟
    output                      sd_mosi,       // 【输出到 TF 卡】FPGA→TF 卡串行数据
    input                       sd_miso,       // 【输入自 TF 卡】TF 卡→FPGA 串行数据

    // USB-UART（板载 CH340：FPGA RXD=F12，TXD=D12）
    input                       uart_rxd,      // 【输入自 CH340】串口接收，预留给后续命令
    output                      uart_txd       // 【输出到 CH340】串口日志 TX
);

// SDRAM 应用侧数据宽度：每个存储字 32 bit。
// 本工程把 RGB888 扩展为 {R,G,B,8'b0}，因此一个存储字对应一个像素。
parameter MEM_DATA_BITS = 32;             // 参数：SDRAM/FIFO 数据位宽
parameter ADDR_BITS     = 21;             // 参数：SDRAM 应用地址位宽
parameter BUSRT_BITS    = 10;             // 当前顶层未使用，保留原工程参数
parameter FRAME_PIXELS  = 24'd307200;     // 640*480，单位是 32 bit 像素字
// 0: test sine before the first displayed frame, then image-linked arpeggios.
// 1: continuous 3 kHz HDMI test sine. Volume is 0..255 / 256.
parameter AUDIO_TEST_ONLY = 0;
parameter AUDIO_PCM_ENABLE = 1;
parameter [7:0] AUDIO_VOLUME = 8'd192;

// 四个帧缓冲区的首地址。这里是 SDRAM 应用接口的“字地址”，不是字节地址。
//   Buffer 0：0              ~ 307199
//   Buffer 1：307200         ~ 614399
//   Buffer 2：614400         ~ 921599
//   Buffer 3：921600         ~ 1228799
parameter BUF0_ADDR     = 24'd0;          // 参数：Buffer 0 的首字地址
parameter BUF1_ADDR     = FRAME_PIXELS;   // 参数：Buffer 1 的首字地址
parameter BUF2_ADDR     = FRAME_PIXELS * 2;
parameter BUF3_ADDR     = FRAME_PIXELS * 3;

// SDRAM 控制器状态：初始化完成、刷新有效、控制器忙。
wire Sdr_init_done;       // 【SDRAM 输出→top/frame_read_write】初始化完成
wire Sdr_init_ref_vld;    // 【SDRAM 输出→top/frame_read_write】刷新有效提示
wire Sdr_busy;            // 【SDRAM 输出→top/frame_read_write】SDRAM 当前忙

// sys_pll：SD 卡时钟、SDRAM 时钟、SDRAM 相移时钟。
// video_pll：视频像素时钟、HDMI 5 倍串行时钟。
wire sd_card_clk;         // 【sys_pll 输出→SD 链路】SD 卡侧时钟，约 100 MHz
wire ext_mem_clk;         // 【sys_pll 输出→SDRAM 链路】存储器控制时钟，约 125 MHz
wire ext_mem_clk_sft;     // 【sys_pll 输出→sdram】SDRAM 相移时钟
wire video_clk;           // 【video_pll 输出→视频链路】像素时钟，约 25 MHz
wire hdmi_5x_clk;         // 【video_pll 输出→HDMI PHY】高速串行时钟

// 延迟对齐后的视频时序：hs/vs/de 与 vout_data_raw 同步。
wire hs;                  // 【视频流水线输出→video_rgb_to_axis】延迟后的行同步
wire vs;                  // 【视频流水线输出→video_rgb_to_axis】延迟后的场同步
wire de;                  // 【视频流水线输出→video_rgb_to_axis】延迟后的有效显示区

wire [23:0] vout_data_raw;  // 【视频流水线输出→top】已与 de 对齐的 RGB888
wire [23:0] vout_data;      // 【top 输出→video_rgb_to_axis】最终 RGB，未提交时为黑色
wire        display_valid;  // 【sd_card_bmp 输出→top】第一张图完整提交前为 0

// sd_card_bmp 的调试状态码，经过译码后显示在数码管上。
wire [3:0]  state_code;    // 【sd_card_bmp 输出→seg_decoder】当前 SD/加载状态码
wire [6:0]  seg_data_0;    // 【seg_decoder 输出→seg_scan】低有效/编码后的段码

// USB-UART 调试状态：由 SD/FAT32 控制器提供，供 50 MHz 日志时钟域采样。
wire        debug_sd_init_done;
wire osd_auto_sd;                 // SD域模式，经快照邮箱送到像素域
wire        debug_scan_busy;
wire        debug_scan_done;
wire [7:0]  debug_scan_error;
wire        debug_source_ready;
wire        debug_load_busy;
wire [7:0]  debug_load_error;
wire signed [15:0] pcm_stream_sample;
wire pcm_stream_valid, pcm_stream_active, pcm_stream_ready;
wire pcm_mode;
wire [7:0] pcm_stream_error;
// FAT/SD 链路的细粒度诊断观察线，送给 50MHz UART 日志模块。
wire [3:0]  debug_scanner_state;
wire [31:0] debug_sector_lba;
wire [8:0]  debug_sector_byte_count;
wire        debug_sector_active;
wire [15:0] debug_sector_done_count;
wire [3:0]  debug_resource_valid;
wire [4:0]  debug_sd_state;
wire        debug_sd_cmd_error;
wire        debug_sd_cmd_error_latched;
wire [7:0]  uart_rx_data;
wire        uart_rx_valid;

// HDMI 读帧接口，主要在 video_clk 域使用。
// video_read_req 是一帧读请求；video_read_en 是一个像素读使能。
wire        video_read_req;      // 【video_timing 输出→frame_read_write】一帧读请求
wire        video_read_req_ack;  // 【frame_read_write 输出→video_timing】读请求应答
wire        video_read_en;       // 【video_delay 输出→frame_read_write】取一个像素
wire [31:0] video_read_data;     // 【frame_read_write 输出→video_delay】32 bit 像素

// TF 写帧接口，主要由 sd_card_clk 域产生。
// write_req/ack 是“一整帧写入任务”的握手，不是单像素握手。
// write_en=1 时，write_data 才表示一个有效的 32 bit 像素。
wire        sd_card_write_en;         // 【sd_card_bmp 输出→frame_read_write】像素有效
wire [31:0] sd_card_write_data;       // 【sd_card_bmp 输出→frame_read_write】32 bit 像素
wire        sd_card_write_req;        // 【sd_card_bmp 输出→frame_read_write】开始写一帧
wire        sd_card_write_req_ack;    // 【frame_read_write 输出→sd_card_bmp】写侧已准备好
wire        sd_card_write_frame_commit; // 【BMP 解码器→loader】最后一个像素已输出
wire        frame_write_finish;       // 【frame_read_write 输出→top】整帧写入完成脉冲
reg         frame_write_toggle_mem;   // 【top 输出→sd_card_bmp】跨域的整帧完成事件

// 四帧缓存索引：写入哪一块、期望显示哪一块。
wire [1:0]  write_buf_idx;            // 【sd_card_bmp 输出→frame_read_write】写缓冲区编号
wire [1:0]  disp_buf_idx;             // 【sd_card_bmp 输出】期望显示的缓冲区编号
wire [1:0]  video_disp_buf_idx;       // 视频帧请求边界锁存的实际读取缓冲区
wire        video_display_valid;
wire        display_commit_toggle;
reg         video_read_req_d;

wire        audio_valid;
wire [23:0] audio_left_data, audio_right_data;
wire        acr_valid;
wire [19:0] acr_cts, acr_n;

// frame_read_write 与 SDRAM 控制器之间的应用接口。
// App_wr_*：应用侧写 SDRAM；App_rd_*：应用侧读 SDRAM；
// Sdr_rd_*：SDRAM 控制器返回的读数据。
wire App_rd_en;                       // 【frame_read_write 输出→sdram】请求读一个字
wire [ADDR_BITS-1:0] App_rd_addr;     // 【frame_read_write 输出→sdram】读地址
wire Sdr_rd_en;                       // 【sdram 输出→frame_read_write】读数据有效
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;  // 【sdram 输出→frame_read_write】读出的 32 bit 数据
wire App_wr_en;                       // 【frame_read_write 输出→sdram】写一个字使能
wire [ADDR_BITS-1:0] App_wr_addr;     // 【frame_read_write 输出→sdram】写地址
wire [MEM_DATA_BITS-1:0] App_wr_din;  // 【frame_read_write 输出→sdram】写入的 32 bit 数据
wire [3:0] App_wr_dm;                 // 【frame_read_write 输出→sdram】4 个字节的写屏蔽
wire frame_write_busy;                // 【frame_read_write 输出→media_frame_reader】写侧占用 SDRAM

// video_timing_data 产生的原始时序，先进入 video_delay 做数据对齐。
wire hs_0;                            // 【video_timing 输出→video_delay】原始行同步
wire vs_0;                            // 【video_timing 输出→video_delay】原始场同步
wire de_0;                            // 【video_timing 输出→video_delay】原始有效区

// 新读侧的状态和视频域输出。
wire        reader_frame_start;
wire        reader_pixel_consume;
wire [23:0] reader_pixel_rgb;
wire        reader_pixel_valid;
wire        reader_underflow;
wire        reader_busy;
wire        reader_bus_busy;
wire [1:0]  reader_index;
wire [7:0]  reader_error;
reg  [20:0] reader_hs_pipe;
reg  [20:0] reader_vs_pipe;
reg  [20:0] reader_de_pipe;
reg  [23:0] reader_rgb_pipe [0:20];
integer reader_pipe_i;

// 独立写侧：sd_card_clk 域 loader + ext_mem_clk 域写命令 CDC。
wire        writer_load_start;
wire        writer_load_busy;
wire        writer_load_done;
wire [3:0]  writer_image_complete;
wire [7:0]  writer_memory_error;
wire        writer_app_wr_en_src;
wire [ADDR_BITS-1:0] writer_app_wr_addr_src;
wire [3:0]  writer_app_wr_mask_src;
wire [31:0] writer_app_wr_data_src;
wire        writer_src_full;
wire        writer_src_overflow;
wire        writer_dst_app_wr_en;
wire [ADDR_BITS-1:0] writer_dst_app_wr_addr;
wire [3:0]  writer_dst_app_wr_mask;
wire [31:0] writer_dst_app_wr_data;
wire        writer_dst_idle;
wire        writer_dst_allow;
// —— "整帧写完"两级确认事件链的寄存器（工作过程详见 652-682 行注释）——
reg         writer_done_toggle_src;    // 【sd_card_clk 域】loader 每次报"命令全部产生"就翻转一次
reg  [1:0]  writer_done_sync_mem;      // 【ext_mem_clk 域】toggle 的两级同步移位寄存器
reg         writer_done_seen_mem;      // 【ext_mem_clk 域】上次看到的 toggle 值（用于检测"又翻转了"）
reg         writer_commit_pending_mem; // 【ext_mem_clk 域】=1：已收到"命令全部产生"，正在等 CDC 排空
reg  [2:0]  writer_commit_wait_mem;    // 【ext_mem_clk 域】等待倒计时（4 拍，给 dst_idle 稳定时间）
reg         writer_finish_pulse_mem;   // 【ext_mem_clk 域】最终"整帧真正写完"的单拍脉冲

assign writer_load_start = sd_card_write_req && !writer_load_busy;
assign sd_card_write_req_ack = writer_load_busy;
assign writer_dst_allow = Sdr_init_done && !Sdr_busy &&
                          !Sdr_init_ref_vld && !reader_bus_busy;
assign frame_write_busy = !writer_dst_idle || writer_commit_pending_mem;
assign frame_write_finish = writer_finish_pulse_mem;

// HDMI 1.4b 视频 AXI-Stream 输入信号。
// video_rgb_to_axis_640x480 把 RGB + vs/de 转换成这些信号。
wire        axis_s_user;             // 【video_rgb_to_axis 输出→HDMI IP】帧/用户标志
wire        axis_s_valid;            // 【video_rgb_to_axis 输出→HDMI IP】视频数据有效
wire        axis_s_last;             // 【video_rgb_to_axis 输出→HDMI IP】行/帧结束标志
wire [23:0] axis_s_data;             // 【video_rgb_to_axis 输出→HDMI IP】AXIS RGB 数据
wire        axis_s_ready;            // 【HDMI IP 输出】AXIS 接收准备信号，当前未回接

// EDID 读取触发和返回数据，由 HDMI 发射 IP 使用。
wire        edid_trig;               // 【startup_pulse 输出→HDMI IP】触发 EDID 读取
wire        edid_valid;              // 【HDMI IP 输出】EDID 返回数据有效
wire [7:0]  edid_data;               // 【HDMI IP 输出】EDID 返回字节

// HDMI 发射器输出的 10 bit TMDS 并行编码数据。
// 它们还要经过 hdmi_phy_wrapper 才成为差分管脚信号。
wire [9:0]  tmds_ch0_data;           // 【HDMI IP 输出→PHY】TMDS 数据通道 0
wire [9:0]  tmds_ch1_data;           // 【HDMI IP 输出→PHY】TMDS 数据通道 1
wire [9:0]  tmds_ch2_data;           // 【HDMI IP 输出→PHY】TMDS 数据通道 2
wire [9:0]  tmds_clk_data;           // 【HDMI IP 输出→PHY】TMDS 时钟通道

// ============================================================================
// PLL 锁定与统一复位
// ============================================================================
// rst_n 是外部低有效复位；rst_all 是内部高有效复位。
// 两个 PLL 都锁定后，再连续等待约 20 ms 才释放 rst_all。
wire sys_pll_lock;       // 【sys_pll 输出→top】系统 PLL 锁定
wire video_pll_lock;     // 【video_pll 输出→top】视频 PLL 锁定
wire rst_all;            // 【top 输出→所有内部模块】统一高有效复位

reg [19:0] rst_cnt = 20'd0; // clk 域：PLL 锁定后的稳定计数器
reg rst_all_reg = 1'b1;      // clk 域：rst_all 的寄存器版本，1=保持复位

always @(posedge clk) begin
    if (!rst_n) begin
        rst_cnt <= 20'd0;
        rst_all_reg <= 1'b1;
    end else if (!rst_all_reg) begin
        // 一旦复位释放，忽略后续微小的 PLL 抖动，防止系统在使用时随机复位
        rst_all_reg <= 1'b0;
    end else if (sys_pll_lock && video_pll_lock) begin
        // 等待 PLL 连续稳定 20ms @ 50MHz
        if (rst_cnt < 20'd1_000_000) begin
            rst_cnt <= rst_cnt + 1'b1;
            rst_all_reg <= 1'b1;
        end else begin
            rst_all_reg <= 1'b0;
        end
    end else begin
        rst_cnt <= 20'd0;
        rst_all_reg <= 1'b1;
    end
end

assign rst_all = rst_all_reg;

// 图片选择在 SD 时钟域产生；只在下一次视频读帧请求到来时提交。
// read_req 是保持到 ack 的电平，因此只取上升沿，防止一帧内重复改地址。
// 使用握手邮箱整体传递 {显示有效, bank}，在帧请求边界统一提交。
// reader 延迟一拍捕获提交后的索引，音频也从同一提交事件切换。
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        video_read_req_d <= 1'b0;
    end else begin
        video_read_req_d <= video_read_req;
    end
end

media_display_commit u_display_commit (
    .src_clk(sd_card_clk), .rst(rst_all),
    .display_valid_src(display_valid), .display_index_src(disp_buf_idx),
    .pixel_clk(video_clk),
    .frame_request(video_read_req && !video_read_req_d),
    .reader_frame_start(reader_frame_start),
    .display_valid_pix(video_display_valid),
    .display_index_pix(video_disp_buf_idx),
    .audio_commit_toggle(display_commit_toggle)
);

// ============================================================================
// 时钟产生
// ============================================================================
// sys_pll 的输出：
//   clk0_out → SD 卡读取域
//   clk1_out → SDRAM 控制域
//   clk2_out → SDRAM 相移时钟
// video_pll 的输出：
//   clk0_out → 视频像素域
//   clk1_out → HDMI 串行 PHY
sys_pll sys_pll_m0(
    .refclk     (clk),            // 输入：板载参考时钟
    .reset      (1'b0),           // 输入：PLL 不使用外部复位，固定释放
    .extlock    (sys_pll_lock),   // 输出：系统 PLL 锁定标志
    .clk0_out   (sd_card_clk),    // 输出：SD 卡时钟，送入 sd_card_bmp
    .clk1_out   (ext_mem_clk),    // 输出：SDRAM 控制时钟，送入 frame_read_write/sdram
    .clk2_out   (ext_mem_clk_sft) // 输出：SDRAM 相移时钟，送入 sdram
);

video_pll video_pll_m0(
    .refclk     (clk),            // 输入：板载参考时钟
    .reset      (1'b0),           // 输入：PLL 不使用外部复位，固定释放
    .extlock    (video_pll_lock), // 输出：视频 PLL 锁定标志
    .clk0_out   (video_clk),      // 输出：视频像素时钟
    .clk1_out   (hdmi_5x_clk)     // 输出：HDMI PHY 高速串行时钟
);

// ============================================================================
// 跨时钟域传递“整帧写完”事件
// ============================================================================
// frame_write_finish 只在 ext_mem_clk 域保持一个周期，不能直接把这个短
// 脉冲送到 sd_card_clk 域，否则可能被错过。
//
// toggle 方法：每完成一帧就翻转一次 0/1。SD 卡侧同步这个电平，检测到
// 电平变化就知道发生了一次新的“写帧完成”事件。
always @(posedge ext_mem_clk or posedge rst_all) begin
    if (rst_all)
        frame_write_toggle_mem <= 1'b0;
    else if (frame_write_finish)
        frame_write_toggle_mem <= ~frame_write_toggle_mem;
end

// ============================================================================
// TF 多图扫描、BMP 解析、图片切换与写数据产生
// ============================================================================
// sd_card_bmp_fat32 内部连接：
//   FAT32 扫描/簇链读取 → 扇区适配器 → sd_card_top → SPI 驱动
//
// 对本顶层而言，它是一个“TF 图片源”：
//   输入：按键、SD_MISO、写帧完成通知
//   输出：SD SPI 信号、写帧请求、像素写入流、双缓冲索引
sd_card_bmp_fat32 #(
    .PCM_ENABLE(AUDIO_PCM_ENABLE && !AUDIO_TEST_ONLY),
    .CLK_FREQ_HZ       (100_000_000), // 参数：sd_card_bmp 工作时钟频率
    .SCAN_START_SECTOR (32'd0),       // 参数：从扇区 0 开始扫描
    .SCAN_MAX_SECTOR   (32'd131071),  // 参数：最多扫描到的扇区
    .SCAN_TARGET_COUNT (3'd4)         // 参数：最多记录 4 张 BMP
) sd_card_bmp_fat32_m0(
    .pixel_clk(video_clk), .pcm_ready(pcm_stream_ready), .pcm_sample(pcm_stream_sample),
    .pcm_valid(pcm_stream_valid), .pcm_active(pcm_stream_active),
    .pcm_end(), .pcm_underflow(), .pcm_error(pcm_stream_error),
    .pcm_mode(pcm_mode),
    .clk               (sd_card_clk),          // 输入：SD 卡/BMP 控制时钟
    .rst               (rst_all),              // 输入：统一高有效复位
    .key_next          (key1),                 // 输入：手动下一张按键
    .key_auto          (key2),                 // 输入：自动轮播按键
    .key_audio_mode    (key3),                 // 输入：独立音源切换按键
    // state_code 只用于数码管调试，不参与图像数据传输。
    .state_code        (state_code),           // 输出：当前扫描/加载状态码
    .bmp_width         (16'd640),              // 输入：要求 BMP 宽度为 640
    .bmp_height        (16'd480),              // 输入：要求 BMP 高度为 480
    .display_valid     (display_valid),        // 输出→top：首帧提交后允许显示

    .write_finish_toggle(frame_write_toggle_mem), // 输入：SDRAM 写帧完成事件
    .write_buf_idx     (write_buf_idx),         // 输出→frame_read_write：写缓冲区编号
    .disp_buf_idx      (disp_buf_idx),          // 输出→frame_read_write：显示缓冲区编号

    .write_req         (sd_card_write_req),     // 输出→frame_read_write：请求写一帧
    .write_req_ack     (sd_card_write_req_ack), // 输入：写侧准备完成应答
    .write_en          (sd_card_write_en),      // 输出→写 FIFO：当前像素有效
    .write_data        (sd_card_write_data),    // 输出→写 FIFO：32 bit 像素数据
    .write_frame_commit(sd_card_write_frame_commit),
    .SD_nCS            (sd_ncs),                // 输出→TF 卡：SPI 片选
    .SD_DCLK           (sd_dclk),               // 输出→TF 卡：SPI 时钟
    .SD_MOSI           (sd_mosi),               // 输出→TF 卡：SPI 主出从入数据
    .SD_MISO           (sd_miso),               // 输入←TF 卡：SPI 主入从出数据
    .debug_sd_init_done(debug_sd_init_done),
    .debug_auto_enabled(osd_auto_sd),
    .debug_scan_busy   (debug_scan_busy),
    .debug_scan_done   (debug_scan_done),
    .debug_scan_error  (debug_scan_error),
    .debug_source_ready(debug_source_ready),
    .debug_load_busy   (debug_load_busy),
    .debug_load_error  (debug_load_error),
    .debug_scanner_state(debug_scanner_state),
    .debug_sector_lba(debug_sector_lba),
    .debug_sector_byte_count(debug_sector_byte_count),
    .debug_sector_active(debug_sector_active),
    .debug_sector_done_count(debug_sector_done_count),
    .debug_resource_valid(debug_resource_valid),
    .debug_sd_state(debug_sd_state),
    .debug_sd_cmd_error(debug_sd_cmd_error),
    .debug_sd_cmd_error_latched(debug_sd_cmd_error_latched)
 );

// ============================================================================
// USB-UART 调试接口
// ============================================================================
// 115200 baud, 8 data bits, no parity, 1 stop bit (8-N-1).
// 日志从 uart_txd 输出；uart_rxd 已接入接收器，最近收到的字节会在日志
// 中显示，后续可在此基础上增加串口命令控制。
uart_rx #(
    .CLOCK_FREQ_HZ(50_000_000),
    .BAUD_RATE    (115_200)
) u_uart_rx (
    .clk      (clk),
    .rst      (~rst_n),
    .rx       (uart_rxd),
    .rx_data  (uart_rx_data),
    .rx_valid (uart_rx_valid)
);

uart_debug_logger #(
    .CLOCK_FREQ_HZ   (50_000_000),
    .BAUD_RATE       (115_200),
    .LOG_INTERVAL_MS (1000)
) u_uart_debug_logger (
    .clk            (clk),
    .rst            (~rst_n),
    .pll_ready      (sys_pll_lock && video_pll_lock),
    .sd_init_done   (debug_sd_init_done),
    .state_code     (state_code),
    .scan_busy      (debug_scan_busy),
    .scan_done      (debug_scan_done),
    .scan_error     (pcm_stream_error != 0 ? pcm_stream_error :
                     debug_load_error != 0 ? debug_load_error : debug_scan_error),
    .display_valid  (display_valid),
    .rx_data        (uart_rx_data),
    .scanner_state  (debug_scanner_state),
    .sector_active  (debug_sector_active),
    .sector_lba     (debug_sector_lba),
    .sector_byte_count(debug_sector_byte_count),
    .sd_state       (debug_sd_state),
    .sd_cmd_error_latched(debug_sd_cmd_error_latched),
    .resource_valid (debug_resource_valid),
    .sector_done_count(debug_sector_done_count),
    .tx             (uart_txd)
);

// ============================================================================
// 数码管调试输出
// ============================================================================
// state_code → 七段译码 → 动态扫描。它与图像主数据流无关。
seg_decoder seg_decoder_m0(
    .bin_data          (state_code),     // 输入：4 bit 状态码
    .seg_data          (seg_data_0)      // 输出→seg_scan：七段编码
);

seg_scan seg_scan_m0(
    .clk               (clk),             // 输入：数码管扫描时钟
    .rst_n             (rst_n),           // 输入：外部低有效复位
    .seg_sel           (seg_sel),         // 输出→顶层管脚：数码管位选
    .seg_data          (seg_data),        // 输出→顶层管脚：数码管段选
    .seg_data_0        ({1'b1,7'b1111_111}), // 输入：第 0 位关闭
    .seg_data_1        ({1'b1,7'b1111_111}), // 输入：第 1 位关闭
    .seg_data_2        ({1'b1,7'b1111_111}), // 输入：第 2 位关闭
    .seg_data_3        ({1'b1,7'b1111_111}), // 输入：第 3 位关闭
    .seg_data_4        ({1'b1,7'b1111_111}), // 输入：第 4 位关闭
    .seg_data_5        ({1'b1,seg_data_0})   // 输入：第 5 位显示状态码
);

// ============================================================================
// 视频时序发生器
// ============================================================================
// video_timing_data 产生 640×480 的原始 hs_0/vs_0/de_0，
// 并在每帧开始时发出 video_read_req，要求帧缓存准备读出当前显示帧。
video_timing_data video_timing_data_m0(
    .video_clk         (video_clk),          // 输入：视频像素时钟
    .rst               (rst_all),            // 输入：统一高有效复位
    .read_req          (video_read_req),     // 输出→frame_read_write：请求读一帧
    .read_req_ack      (video_read_req_ack), // 输入：帧缓存接受读请求
    .hs                (hs_0),               // 输出→video_delay：原始行同步
    .vs                (vs_0),               // 输出→video_delay：原始场同步
    .de                (de_0)                // 输出→video_delay：原始有效显示区
);

// 读请求由 media_frame_reader 接管；它把 read_req 的上升沿转换为帧加载事件。
// 第一张图片尚未完成 SDRAM 写入前，不能读取 Buffer 0。否则 reader_busy
// 会暂停写侧，BMP 加载可能因长期没有写进展而超时并使 state_code=F。
assign video_read_req_ack = 1'b1;

// ============================================================================
// 新读侧：SDRAM -> 异步 FIFO -> 两级预取 -> video_clk
// ============================================================================
// video_timing_data 的 read_req 会保持到应答。上升沿转换为新 reader 的
// 单周期 frame_start，reader 在下一拍锁存已经同步到 video_clk 的 buffer 索引。
assign reader_pixel_consume = de_0;

media_frame_reader #(
    .ADDR_WIDTH          (ADDR_BITS),
    .IMAGE_WORDS         (FRAME_PIXELS),
    .FIFO_DEPTH          (512),
    .FIFO_MARGIN         (8),
    .BURST_SIZE          (128),
    .BURST_GAP_CYCLES    (12),
    .MAX_INFLIGHT_BURSTS (2)
) u_media_frame_reader (
    .mem_clk             (ext_mem_clk),
    .mem_rst_n           (~rst_all),
    .controller_init_done(Sdr_init_done),
    .controller_busy     (Sdr_busy),
    .refresh_active      (Sdr_init_ref_vld),
    .external_write_busy (frame_write_busy),
    .app_rd_en           (App_rd_en),
    .app_rd_addr         (App_rd_addr),
    .sdram_rd_valid      (Sdr_rd_en),
    .sdram_rd_data       (Sdr_rd_dout),
    .pix_clk             (video_clk),
    .pix_rst_n           (~rst_all),
    .frame_start         (reader_frame_start),
    .active_index        (video_disp_buf_idx),
    .pixel_consume       (reader_pixel_consume),
    .pixel_rgb           (reader_pixel_rgb),
    .pixel_valid         (reader_pixel_valid),
    .underflow           (reader_underflow),
    .reader_busy         (reader_busy),
    .reader_bus_busy     (reader_bus_busy),
    .frame_read_done     (),
    .reader_index        (reader_index),
    .reader_error        (reader_error)
);

// reader_pixel_rgb 已经位于 video_clk 域，但 FIFO 可能在帧开始后暂时没有
// 数据。无效周期送黑色，避免把 FIFO 上一次残留值显示成雪花。
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        reader_hs_pipe  <= 21'd0;
        reader_vs_pipe  <= 21'd0;
        reader_de_pipe  <= 21'd0;
        for (reader_pipe_i = 0; reader_pipe_i <= 20; reader_pipe_i = reader_pipe_i + 1)
            reader_rgb_pipe[reader_pipe_i] <= 24'd0;
    end else begin
        reader_hs_pipe <= {reader_hs_pipe[19:0], hs_0};
        reader_vs_pipe <= {reader_vs_pipe[19:0], vs_0};
        reader_de_pipe <= {reader_de_pipe[19:0], de_0};
        reader_rgb_pipe[0] <= reader_pixel_valid ? reader_pixel_rgb : 24'd0;
        for (reader_pipe_i = 1; reader_pipe_i <= 20; reader_pipe_i = reader_pipe_i + 1)
            reader_rgb_pipe[reader_pipe_i] <= reader_rgb_pipe[reader_pipe_i-1];
    end
end

assign hs = reader_hs_pipe[20];
assign vs = reader_vs_pipe[20];
assign de = reader_de_pipe[20];
assign vout_data_raw = reader_rgb_pipe[20];

// 第一张图完成“源数据读取 + SDRAM 整帧写入”之前输出黑色，
// 防止 HDMI 在帧缓存尚未准备好时显示随机数据。
assign vout_data = video_display_valid ? vout_data_raw : 24'd0;

// OSD只替换最终像素颜色，不改SDRAM/FIFO协议。
wire osd_sd_ready, osd_mem_ready; // 两个源时钟域各自的邮箱空闲反馈
wire [95:0] osd_sd_snapshot;      // [8]=模式，[7:0]=SD/写侧错误
wire [95:0] osd_mem_snapshot;     // [7:0]=读侧错误
wire [23:0] osd_rgb;             // 叠字后的完整RGB888
wire osd_de;                     // DE透传，与输出颜色同拍
wire osd_frame_start;
wire [11:0] osd_x, osd_y;
wire [23:0] timestamp_rgb;        // 状态OSD后再叠运行时长，共用像素域坐标
// 明确优先级，不把多个错误码按位OR，避免产生不存在的错误码。
wire [7:0] osd_sd_error = (writer_memory_error != 0) ? writer_memory_error :
                         (pcm_stream_error != 0) ? pcm_stream_error :
                         (debug_load_error != 0) ? debug_load_error : debug_scan_error;
wire [7:0] osd_error = (osd_mem_snapshot[7:0] != 0) ?
                       osd_mem_snapshot[7:0] : osd_sd_snapshot[7:0];
// 邮箱复用于低速状态：空闲即发送最新快照，中间变化允许合并。
// 此处frame_start接消隐安全窗口；字形核心仍只在VS下降沿更新显示状态。
subtitle_frame_commit u_osd_sd_commit (
    .src_clk(sd_card_clk), .src_rst_n(~rst_all),
    .src_commit(osd_sd_ready), .src_ready(osd_sd_ready),
    .src_text({87'd0,osd_auto_sd,osd_sd_error}), .src_length(6'd0), .src_enable(1'b1),
    .dst_clk(video_clk), .dst_rst_n(~rst_all), .frame_start(!de),
    .dst_text(osd_sd_snapshot), .dst_length(), .dst_enable()
);
subtitle_frame_commit u_osd_mem_commit (
    .src_clk(ext_mem_clk), .src_rst_n(~rst_all),
    .src_commit(osd_mem_ready), .src_ready(osd_mem_ready),
    .src_text({88'd0,reader_error}), .src_length(6'd0), .src_enable(1'b1),
    .dst_clk(video_clk), .dst_rst_n(~rst_all), .frame_start(!de),
    .dst_text(osd_mem_snapshot), .dst_length(), .dst_enable()
);
osd_overlay_24 #(.OSD_X(8), .OSD_Y(8)) u_osd_overlay (
    .clk(video_clk), .rst_n(~rst_all), .vsync(vs), .de(de), .rgb_in(vout_data),
    .active_index(video_disp_buf_idx), .auto_enabled(osd_sd_snapshot[8]),
    .error_code(osd_error), .rgb_out(osd_rgb), .de_out(osd_de),
    .osd_active(), .frame_start(osd_frame_start),
    .pixel_x_out(osd_x), .pixel_y_out(osd_y)
);
timestamp_osd #(.CLOCK_FREQ_HZ(25_000_000)) u_timestamp_osd (
    .clk(video_clk), .rst_n(~rst_all), .frame_start(osd_frame_start),
    .de(osd_de), .pixel_x(osd_x), .pixel_y(osd_y),
    .rgb_in(osd_rgb), .rgb_out(timestamp_rgb)
);

// ============================================================================
// 帧缓存总控：两个异步 FIFO + SDRAM 读写突发控制
// ============================================================================
// 写方向：
//   sd_card_clk/write_en/write_data
//       → 写异步 FIFO
//       → ext_mem_clk/App_wr_*
//       → SDRAM
//
// 读方向：
//   SDRAM/App_rd_* / Sdr_rd_*
//       → 读异步 FIFO
//       → video_clk/read_en/read_data
//
// WRITE_V_FLIP=1 用来修正 BMP 常见的 bottom-up 行顺序。
// ============================================================================
// 独立写侧：loader（sd_card_clk）+ 命令 CDC（ext_mem_clk）
// ============================================================================
// loader 只接收 FAT/BMP 产生的 RGB 流，并按固定 bank 地址生成写命令。
// media_sdram_write_cdc 把 {地址、掩码、数据} 作为一个整体跨到 SDRAM 域，
// 避免直接跨时钟采样三组彼此可能错位的总线。
media_frame_loader #(
    .ADDR_WIDTH   (ADDR_BITS),
    .IMAGE_WORDS  (FRAME_PIXELS),
    .FIFO_DEPTH   (512),
    .BURST_SIZE   (128),
    .STALL_TIMEOUT(32'd5_000_000)
) u_frame_loader (
    .clk                  (sd_card_clk),
    .rst_n                (~rst_all),
    .load_start           (writer_load_start),
    .image_index          (write_buf_idx),
    .pixel_valid          (sd_card_write_en),
    .pixel_rgb            (sd_card_write_data[31:8]),
    .frame_commit         (sd_card_write_frame_commit),
    .decode_error         (debug_load_error),
    .command_ready        (!writer_src_full),
    .load_busy            (writer_load_busy),
    .load_done            (writer_load_done),
    .image_complete       (writer_image_complete),
    .memory_error         (writer_memory_error),
    .controller_init_done (Sdr_init_done),
    .controller_busy      (Sdr_busy),
    .refresh_active       (Sdr_init_ref_vld),
    .app_wr_en            (writer_app_wr_en_src),
    .app_wr_addr          (writer_app_wr_addr_src),
    .app_wr_mask          (writer_app_wr_mask_src),
    .app_wr_data          (writer_app_wr_data_src)
);

media_sdram_write_cdc #(
    .ADDR_WIDTH      (ADDR_BITS),
    .FIFO_ADDR_WIDTH (9),
    .BURST_SIZE      (128)
) u_frame_write_cdc (
    .src_clk         (sd_card_clk),
    .src_rst_n       (~rst_all),
    .src_wr_en       (writer_app_wr_en_src),
    .src_wr_addr     (writer_app_wr_addr_src),
    .src_wr_mask     (writer_app_wr_mask_src),
    .src_wr_data     (writer_app_wr_data_src),
    .src_full        (writer_src_full),
    .src_overflow    (writer_src_overflow),
    .dst_clk         (ext_mem_clk),
    .dst_rst_n       (~rst_all),
    .dst_allow       (writer_dst_allow),
    .dst_app_wr_en   (writer_dst_app_wr_en),
    .dst_app_wr_addr (writer_dst_app_wr_addr),
    .dst_app_wr_mask (writer_dst_app_wr_mask),
    .dst_app_wr_data (writer_dst_app_wr_data),
    .dst_idle        (writer_dst_idle)
);

assign App_wr_en   = writer_dst_app_wr_en;
assign App_wr_addr = writer_dst_app_wr_addr;
assign App_wr_dm   = writer_dst_app_wr_mask;
assign App_wr_din  = writer_dst_app_wr_data;

// ============================================================================
// "整帧写完"的两级确认：先等命令全部产生，再等 CDC FIFO 真正排空
// ============================================================================
// 为什么需要两级？loader 的 load_done（writer_load_done）只说明"写命令
// 已全部产生并送进了 CDC FIFO"；这些命令可能还堵在 FIFO 里没吐给 SDRAM。
// 必须等 writer_dst_idle = 1（CDC FIFO 排空、无突发在途）才算"数据真的
// 全部写进了 SDRAM"，此时才能把"整帧写完"事件通知 FAT32 控制器。
//
// 两个 always 块的分工：
//   第一个（sd_card_clk 域）：把 load_done 脉冲变成 toggle 电平翻转。
//     为什么不直接跨域传脉冲？短脉冲跨时钟域可能被对面漏采；toggle
//     翻转一次代表一次事件，对面只要比较"和上次看到的值一不一样"
//     就能捕捉到，永远不漏（这是跨域传事件的通用手法）。
//   第二个（ext_mem_clk 域）：同步 toggle → 检测翻转 → 等 4 拍 →
//     等 writer_dst_idle → 输出 writer_finish_pulse_mem 单拍脉冲。
// ============================================================================

// —— 第一级：loader 报"命令全部产生" → 翻转 toggle ——
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all)
        writer_done_toggle_src <= 1'b0;                      // 复位：toggle 归零
    // load_done also pulses on errors: only a complete, lossless frame may
    // become visible. Failure leaves the scheduler to abort/timeout this load.
    else if (writer_load_done && writer_memory_error == 0 &&
             writer_image_complete[write_buf_idx] && !writer_src_overflow)
        writer_done_toggle_src <= ~writer_done_toggle_src;   // 翻转一次 = 发生一次事件
end

// —— 第二级：等 CDC 排空，产出最终"整帧写完"脉冲 ——
always @(posedge ext_mem_clk or posedge rst_all) begin
    if (rst_all) begin
        writer_done_sync_mem <= 2'b00;      // 同步链清零
        writer_done_seen_mem <= 1'b0;       // 清零"上次看到的值"
        writer_commit_pending_mem <= 1'b0;  // 无待确认事件
        writer_commit_wait_mem <= 3'd0;     // 倒计时清零
        writer_finish_pulse_mem <= 1'b0;    // 无完成脉冲
    end else begin
        // 同步链：把 sd_card_clk 域的 toggle 打两拍采进来（消除亚稳态）。
        // [0] 位直接采 toggle，[1] 位接收 [0] 的旧值——判断用 [1]，
        // 因为它在判断当拍已经稳定了一拍，时序更收敛。
        writer_done_sync_mem <= {writer_done_sync_mem[0], writer_done_toggle_src};
        // 完成脉冲默认拉低：只在确认瞬间拉高 1 拍
        writer_finish_pulse_mem <= 1'b0;
        if (writer_done_sync_mem[1] != writer_done_seen_mem) begin
            // —— 检测到 toggle 翻转：发生了一次新的"命令全部产生"事件 ——
            writer_done_seen_mem <= writer_done_sync_mem[1]; // 更新"上次看到的值"，为下次检测做准备
            writer_commit_pending_mem <= 1'b1;    // 挂起确认：开始等 CDC 排空
            writer_commit_wait_mem <= 3'd4;       // 先倒计时 4 拍：给 writer_dst_idle 稳定时间
                                                  //（dst_idle 由空标志等组合逻辑拼出，
                                                  //  等几拍吸收延迟是安全习惯）
        end else if (writer_commit_pending_mem) begin
            // —— 有事件待确认：先走完倒计时，再每拍等排空 ——
            if (writer_commit_wait_mem != 0)
                writer_commit_wait_mem <= writer_commit_wait_mem - 1'b1; // 倒计时 4→0
            else if (writer_dst_idle) begin
                // —— CDC FIFO 已排空且无突发在途：数据真正全部写进 SDRAM ——
                writer_commit_pending_mem <= 1'b0; // 确认完成，撤销挂起
                writer_finish_pulse_mem <= 1'b1;   // 输出"整帧写完"单拍脉冲
                                                   // → 经 241 行成为 frame_write_finish
                                                   // → 355-359 行再转成 toggle 通知 FAT32 控制器
            end
            // 注：倒计时结束但 dst_idle 还没来，本分支什么都不做，
            //     pending 保持 1，下一拍继续等（每拍重复检查）。
        end
    end
end

// ============================================================================
// 外部 SDRAM 控制器
// ============================================================================
// sdram 把应用侧读写接口交给安路 SDRAM IP，负责具体的行地址、列地址、
// 刷新、预充电和外部 SDRAM 管脚时序。本顶层只关心 App_* 和 Sdr_* 接口。
sdram U3(
    .Clk               (ext_mem_clk),       // 输入：SDRAM 主控制时钟
    .Clk_sft           (ext_mem_clk_sft),   // 输入：SDRAM 相移时钟
    .Rst               (rst_all),           // 输入：统一高有效复位
    .Sdr_init_done     (Sdr_init_done),     // 输出→frame_read_write：初始化完成
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),  // 输出→frame_read_write：刷新有效
    .Sdr_busy          (Sdr_busy),          // 输出→frame_read_write：控制器忙
    .App_wr_en         (App_wr_en),         // 输入←frame_read_write：写使能
    .App_wr_addr       (App_wr_addr),       // 输入←frame_read_write：写地址
    .App_wr_dm         (App_wr_dm),         // 输入←frame_read_write：字节写屏蔽
    .App_wr_din        (App_wr_din),        // 输入←frame_read_write：写入数据
    .App_rd_en         (App_rd_en),         // 输入←frame_read_write：读使能
    .App_rd_addr       (App_rd_addr),       // 输入←frame_read_write：读地址
    .Sdr_rd_en         (Sdr_rd_en),         // 输出→frame_read_write：读数据有效
    .Sdr_rd_dout       (Sdr_rd_dout)        // 输出→frame_read_write：读出数据
);

// ============================================================================
// RGB + 视频时序 → AXI-Stream
// ============================================================================
// HDMI 1.4b 发射 IP 不直接接收 RGB/de，而是接收 AXI-Stream 视频。
video_rgb_to_axis_640x480 u_video_rgb_to_axis_640x480(
    .I_clk         (video_clk),       // 输入：视频像素时钟
    .I_rst         (rst_all),         // 输入：统一高有效复位
    .I_vs          (vs),              // 输入：延迟后的场同步
    .I_de          (osd_de),          // OSD不增加流水拍数
    .I_rgb         (timestamp_rgb),   // 状态条与运行时长叠加后的RGB888
    .O_video_user  (axis_s_user),     // 输出→HDMI IP：帧起始/用户标志
    .O_video_valid (axis_s_valid),    // 输出→HDMI IP：AXIS 数据有效
    .O_video_last  (axis_s_last),     // 输出→HDMI IP：数据包/行结束标志
    .O_video_data  (axis_s_data)      // 输出→HDMI IP：AXIS 视频数据
);

// 上电后在 video_clk 域产生一个单周期脉冲，触发 HDMI IP 读取显示器 EDID。
startup_pulse #(
    .CNT_MAX(20'd100000)              // 参数：EDID 触发前的 video_clk 延时计数
) u_startup_pulse (
    .I_clk   (video_clk),             // 输入：视频像素时钟
    .I_rst   (rst_all),               // 输入：统一高有效复位
    .O_pulse (edid_trig)              // 输出→HDMI IP：一次性 EDID 触发脉冲
);

// ============================================================================
// HDMI 1.4b 发射器
// ============================================================================
// 视频路径：AXI-Stream → HDMI 视频包 → 10 bit TMDS 数据。
// PCM/ACR are generated in the pixel domain from the actually selected bank.
// The bridge's existing input synchronizers also accept same-clock signals.
media_hdmi_audio_bridge #(
    .PCM_ENABLE(AUDIO_PCM_ENABLE),
    .CLOCK_HZ(25_000_000), .TEST_ONLY(AUDIO_TEST_ONLY), .VOLUME(AUDIO_VOLUME)
) u_media_hdmi_audio_bridge (
    .pcm_active(pcm_stream_active), .pcm_valid(pcm_stream_valid),
    .pcm_mode_sd(pcm_mode),
    .stream_sample(pcm_stream_sample), .pcm_ready(pcm_stream_ready),
    .pixel_clk(video_clk), .rst(rst_all),
    .display_valid_sd(video_display_valid), .media_id_sd(video_disp_buf_idx),
    .commit_toggle_sd(display_commit_toggle),
    .audio_valid(audio_valid), .audio_left_data(audio_left_data),
    .audio_right_data(audio_right_data), .acr_valid(acr_valid),
    .acr_cts(acr_cts), .acr_n(acr_n)
);

hdmi_1_4b_transmitter_core_wrapper #(
    .DEVICE                 ( "EG"       ), // 参数：安路 FPGA 器件类型
    .HTOTAL                 ( 800        ), // 参数：一行总像素时钟数
    .HSA                    ( 96         ), // 参数：水平同步宽度
    .HFP                    ( 16         ), // 参数：水平前肩
    .HBP                    ( 48         ), // 参数：水平后肩
    .HACTIVE                ( 640        ), // 参数：水平有效区
    .VTOTAL                 ( 525        ), // 参数：一帧总行数
    .VSA                    ( 2          ), // 参数：垂直同步宽度
    .VFP                    ( 10         ), // 参数：垂直前肩
    .VBP                    ( 33         ), // 参数：垂直后肩
    .VACTIVE                ( 480        ), // 参数：垂直有效区
    .VIDEO_VIC              ( 1          ), // 参数：640×480 视频模式编号
    .VIDEO_TPG              ( "Disable"  ), // 参数：关闭 HDMI 内部测试图
    .VIDEO_FORMAT           ( "RGB"      ), // 参数：RGB 视频格式
    .AUDIO_SAMPLE_RATE      ( "48K"      ), // 参数：声明音频采样率为 48 kHz
    .IIC_SCL_DIV            ( 250        )  // 参数：DDC/I2C 时钟分频
) u_hdmi_1_4b_transmitter_core_wrapper(
    .I_pixel_clk        (video_clk),       // 输入：视频像素时钟
    .I_rst              (rst_all),         // 输入：统一高有效复位
    .I_edid_read_trig   (edid_trig),       // 输入：触发 EDID 读取
    .O_edid_read_valid  (edid_valid),      // 输出：EDID 数据有效
    .O_edid_read_data   (edid_data),       // 输出：EDID 当前返回字节

    .I_axis_s_user      (axis_s_user),     // 输入：AXIS 用户/帧标志
    .I_axis_s_valid     (axis_s_valid),    // 输入：AXIS 视频数据有效
    .I_axis_s_last      (axis_s_last),     // 输入：AXIS 数据结束标志
    .I_axis_s_data      (axis_s_data),     // 输入：AXIS RGB 视频数据
    .O_axis_s_ready     (axis_s_ready),    // 输出：HDMI IP 接收准备状态

    .I_audio_valid      (audio_valid),
    .I_audio_left_data  (audio_left_data),
    .I_audio_right_data (audio_right_data),
    .I_acr_valid        (acr_valid),
    .I_acr_cts          (acr_cts),
    .I_acr_n            (acr_n),

    .O_video_locked     (),                 // 输出：视频锁定状态，本工程未使用
    .O_ddc_scl          (HDMI_DDC_SCL),    // 输出→顶层管脚：DDC 时钟
    .IO_ddc_sda         (HDMI_DDC_SDA),    // 双向→顶层管脚：DDC 数据

    .O_ch0_tmds_data    (tmds_ch0_data),   // 输出→HDMI PHY：TMDS 通道 0
    .O_ch1_tmds_data    (tmds_ch1_data),   // 输出→HDMI PHY：TMDS 通道 1
    .O_ch2_tmds_data    (tmds_ch2_data),   // 输出→HDMI PHY：TMDS 通道 2
    .O_clk_tmds_data    (tmds_clk_data)    // 输出→HDMI PHY：TMDS 时钟通道
);

// ============================================================================
// HDMI PHY
// ============================================================================
// 发射器输出 10 bit TMDS 并行数据；PHY 使用 video_clk 和 5 倍串行时钟，
// 将其转换为 HDMI 差分输出。
hdmi_phy_wrapper #(
    .DEVICE ( "EG" ) // 参数：安路 FPGA 器件类型
) u_hdmi2phy_wrapper(
    .I_pixel_clk        (video_clk),        // 输入：像素时钟
    .I_serial_clk       (hdmi_5x_clk),      // 输入：高速串行时钟
    .I_rst              (rst_all),          // 输入：统一高有效复位
    .I_tmds_channel_0   (tmds_ch0_data),    // 输入：TMDS 通道 0 并行数据
    .I_tmds_channel_1   (tmds_ch1_data),    // 输入：TMDS 通道 1 并行数据
    .I_tmds_channel_2   (tmds_ch2_data),    // 输入：TMDS 通道 2 并行数据
    .I_tmds_channel_clk (tmds_clk_data),    // 输入：TMDS 时钟并行数据
    .O_tmds_ch0_p       (HDMI_D0_P),        // 输出→顶层管脚：HDMI D0 差分正端
    .O_tmds_ch1_p       (HDMI_D1_P),        // 输出→顶层管脚：HDMI D1 差分正端
    .O_tmds_ch2_p       (HDMI_D2_P),        // 输出→顶层管脚：HDMI D2 差分正端
    .O_tmds_clk_p       (HDMI_CLK_P)        // 输出→顶层管脚：HDMI 时钟差分正端
);

endmodule
