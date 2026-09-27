// ============================================================================
// bmp_read：BMP 文件扫描与像素流解析模块
//
// 本模块不负责产生 SPI 时钟，也不保存整张图片。它的上下游关系是：
//
//   SD 卡底层模块
//       └─ sd_sec_read_data[7:0]       当前收到的一个 SD 字节
//          sd_sec_read_data_valid      当前字节有效
//          sd_sec_read_end             当前 512 字节扇区读完
//              ↓
//   bmp_read
//       ├─ 扫描模式：判断当前扇区是不是合法的 640x480/24bit BMP
//       └─ 加载模式：从 pixel_offset 开始，每 3 个字节组装一个 RGB 像素
//              ↓
//       bmp_data[23:0] + bmp_data_wr_en
//              ↓
//       后级 FIFO / SDRAM 帧缓存
//
// 时序阅读提示：本文件中的多个 always @(posedge clk) 是并行硬件。
// 它们在同一个 clk 上升沿同时触发；<= 的新值在该时钟沿结束后统一生效。
// state 决定当前是哪一个阶段，扫描和加载不会同时进行。
// ============================================================================
module bmp_read(
    input                       clk,
    input                       rst,
    output                      ready,

    // 上电扫描：从 scan_start_sector 开始，顺序寻找前 scan_target_count 张 BMP。
    // 找到一张时，scan_found_valid 只产生一个时钟周期的脉冲，
    // scan_found_sector 给出该 BMP 的起始扇区。
    input                       scan_start,
    input  [31:0]               scan_start_sector,
    input  [31:0]               scan_max_sector,
    input  [2:0]                scan_target_count,
    output reg                  scan_done,
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_sector,
    output reg [2:0]            scan_found_total,

    // 按指定扇区加载一张图到后级 FIFO/SDRAM。
    // load_start 到来后，本模块先读一次文件头，再等待 write_req_ack，
    // 最后进入 ST_LOAD_DATA 输出像素流。
    input                       load_start,
    input                       load_abort,
    input  [31:0]               load_sector,

    input                       sd_init_done,
    output reg [3:0]            state_code,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,

    output reg                  write_req,
    input                       write_req_ack,

    output reg                  sd_sec_read,
    output reg [31:0]           sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,

    output reg                  bmp_data_wr_en,
    output reg [23:0]           bmp_data
);

 // 一个状态寄存器 state 只可能处于其中一个状态：
 // ST_SCAN 与 ST_LOAD_DATA 是先后关系，不是并发关系。
 localparam ST_IDLE      = 3'd0; // 空闲，等待 scan_start 或 load_start
 localparam ST_SCAN      = 3'd1; // 逐扇区扫描，解析每个扇区的 BMP 头
 localparam ST_LOAD_HDR  = 3'd2; // 读取目标图片起始扇区，确认文件头
 localparam ST_LOAD_WAIT = 3'd3; // 等待后级写 FIFO/帧缓存准备好
 localparam ST_LOAD_DATA = 3'd4; // 重新读取整个 BMP，输出像素数据

 reg [2:0]  state;

 // 当前 512 字节扇区内的字节位置。
 // 只在 ST_SCAN/ST_LOAD_HDR 中使用，用于定位 BMP 头字段的偏移。
 reg [9:0]  rd_cnt;

reg [7:0]  header_0;
reg [7:0]  header_1;
reg [31:0] file_len;
reg [31:0] pixel_offset;
reg [31:0] width;
reg [31:0] height;
reg [15:0] bit_count;
reg [31:0] compression;

 // 当前扫描到的扇区地址。
 reg [31:0] scan_sector;

 // 锁存本次要加载的 BMP 起始扇区；后面从文件头重新开始读整张图时使用。
 reg [31:0] load_sector_latched;

 // 当前已经从 BMP 文件开头读过的字节数。
 // 与 rd_cnt 不同：它跨越多个 SD 扇区，不在扇区切换时清零。
 reg [31:0] bmp_len_cnt;

 // 当前像素内已经收到几个字节：0=B，1=G，2=R。
 // 收到第 3 个字节（R）时，bmp_data 才完整。
 reg [1:0]  bmp_byte_idx;

wire header_match;
wire bmp_data_valid;
wire [31:0] file_sector_count;
wire [31:0] next_scan_sector_if_match;
wire [31:0] next_scan_sector_if_miss;

 // ready 是组合信号：只要状态机处于空闲，就表示可以接受新任务。
 assign ready = (state == ST_IDLE);
assign header_match = (header_0 == "B") &&
                      (header_1 == "M") &&
                      (width[15:0]  == bmp_width) &&
                      (height[15:0] == bmp_height) &&
                      (bit_count    == 16'd24) &&
                      (compression  == 32'd0);
 // sd_sec_read_data_valid：SD 层给了一个有效字节；
 // bmp_data_valid：这个有效字节还必须位于 BMP 像素区域，才参与组像素。
 assign bmp_data_valid = (sd_sec_read_data_valid == 1'b1) &&
                        (bmp_len_cnt >= pixel_offset) &&
                        (bmp_len_cnt <  file_len);
 // ceil(file_len / 512)：一张 BMP 占用的扇区数。
 // BMP 文件合法时，扫描找到它后可以直接跳过整张图，而不是逐扇区扫描。
 assign file_sector_count = (file_len == 32'd0) ? 32'd1 : ((file_len + 32'd511) >> 9);
assign next_scan_sector_if_match = scan_sector + file_sector_count;
assign next_scan_sector_if_miss  = scan_sector + 32'd1;

 // --------------------------------------------------------------------------
 // always 1：rd_cnt —— 当前 SD 扇区内的字节计数器。
 //
 // 注意：rd_cnt 只在“解析文件头”的两个状态工作；进入 ST_LOAD_DATA 后
 // 会被清零。它不是整张 BMP 的进度计数器，整张 BMP 的进度由 bmp_len_cnt
 // 负责。
 // --------------------------------------------------------------------------
 always @(posedge clk or posedge rst) begin
    if (rst) begin
        rd_cnt <= 10'd0;
    end else if ((state == ST_SCAN) || (state == ST_LOAD_HDR)) begin
        if (sd_sec_read_data_valid)
            rd_cnt <= rd_cnt + 10'd1;
        else if (sd_sec_read_end)
            rd_cnt <= 10'd0;
    end else begin
        rd_cnt <= 10'd0;
    end
end

 // --------------------------------------------------------------------------
 // always 2：BMP 文件头解析。
 // 当 sd_sec_read_data_valid=1 时，rd_cnt 表示当前字节在扇区内的位置，
 // 因而可以按 BMP 规定的偏移，把字节写入 file_len、width 等寄存器。
 // rd_cnt 和这些寄存器在同一个时钟沿并行更新；case 使用的是更新前的
 // rd_cnt，这正好对应“当前到来的这个字节”的位置。
 // --------------------------------------------------------------------------
 always @(posedge clk or posedge rst) begin
    if (rst) begin
        header_0     <= 8'd0;
        header_1     <= 8'd0;
        file_len     <= 32'd0;
        pixel_offset <= 32'd54;
        width        <= 32'd0;
        height       <= 32'd0;
        bit_count    <= 16'd0;
        compression  <= 32'd0;
    end else if (((state == ST_SCAN) || (state == ST_LOAD_HDR)) && sd_sec_read_data_valid) begin
        case (rd_cnt)
            10'd0 : header_0 <= sd_sec_read_data;
            10'd1 : header_1 <= sd_sec_read_data;

            10'd2 : file_len[7:0] <= sd_sec_read_data;
            10'd3 : file_len[15:8] <= sd_sec_read_data;
            10'd4 : file_len[23:16] <= sd_sec_read_data;
            10'd5 : file_len[31:24] <= sd_sec_read_data;

            10'd10: pixel_offset[7:0] <= sd_sec_read_data;
            10'd11: pixel_offset[15:8] <= sd_sec_read_data;
            10'd12: pixel_offset[23:16] <= sd_sec_read_data;
            10'd13: pixel_offset[31:24] <= sd_sec_read_data;

            10'd18: width[7:0] <= sd_sec_read_data;
            10'd19: width[15:8] <= sd_sec_read_data;
            10'd20: width[23:16] <= sd_sec_read_data;
            10'd21: width[31:24] <= sd_sec_read_data;

            10'd22: height[7:0] <= sd_sec_read_data;
            10'd23: height[15:8] <= sd_sec_read_data;
            10'd24: height[23:16] <= sd_sec_read_data;
            10'd25: height[31:24] <= sd_sec_read_data;

            10'd28: bit_count[7:0] <= sd_sec_read_data;
            10'd29: bit_count[15:8] <= sd_sec_read_data;

            10'd30: compression[7:0] <= sd_sec_read_data;
            10'd31: compression[15:8] <= sd_sec_read_data;
            10'd32: compression[23:16] <= sd_sec_read_data;
            10'd33: compression[31:24] <= sd_sec_read_data;
            default: ;
        endcase
    end
end

 // --------------------------------------------------------------------------
 // always 3：bmp_len_cnt —— 整个 BMP 文件的字节进度。
 // 在 ST_LOAD_DATA 中，每收到一个有效 SD 字节加 1；跨越扇区时不清零。
 // 因此 bmp_len_cnt 可以和 pixel_offset、file_len 比较，判断当前字节
 // 是文件头、像素区，还是已经超出文件末尾。
 // --------------------------------------------------------------------------
 always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_len_cnt <= 32'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (sd_sec_read_data_valid)
            bmp_len_cnt <= bmp_len_cnt + 32'd1;
    end else begin
        bmp_len_cnt <= 32'd0;
    end
end

 // --------------------------------------------------------------------------
 // always 4：bmp_byte_idx —— 当前 24 bit 像素内的字节位置。
 // 0：等待/接收 B；1：接收 G；2：接收 R。收到 R 后回到 0。
 // --------------------------------------------------------------------------
 always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_byte_idx <= 2'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (bmp_data_valid)
            bmp_byte_idx <= (bmp_byte_idx == 2'd2) ? 2'd0 : (bmp_byte_idx + 2'd1);
    end else begin
        bmp_byte_idx <= 2'd0;
    end
end

 // --------------------------------------------------------------------------
 // always 5：像素组装与输出握手。
 // BMP 24 bit 像素在文件中按 B、G、R 顺序保存：
 //   第1字节 -> bmp_data[7:0]
 //   第2字节 -> bmp_data[15:8]
 //   第3字节 -> bmp_data[23:16]，同时 bmp_data_wr_en=1
 //
 // 使用 <= 时，第3个字节所在的时钟沿结束后，bmp_data 才完整，
 // bmp_data_wr_en 才变成 1，并在下一个时钟沿被后级 FIFO 看到。
 // --------------------------------------------------------------------------
 always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_data_wr_en <= 1'b0;
        bmp_data       <= 24'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (bmp_data_valid) begin
            case (bmp_byte_idx)
                2'd0: begin
                    bmp_data_wr_en <= 1'b0;
                    bmp_data[7:0]  <= sd_sec_read_data;
                end
                2'd1: begin
                    bmp_data_wr_en <= 1'b0;
                    bmp_data[15:8] <= sd_sec_read_data;
                end
                2'd2: begin
                    bmp_data_wr_en  <= 1'b1;
                    bmp_data[23:16] <= sd_sec_read_data;
                end
                default: begin
                    bmp_data_wr_en <= 1'b0;
                end
            endcase
        end else begin
            bmp_data_wr_en <= 1'b0;
        end
    end else begin
        bmp_data_wr_en <= 1'b0;
    end
end

 // --------------------------------------------------------------------------
 // always 6：主状态机。
 //
 // 状态之间的关系：
 //   ST_IDLE --scan_start--> ST_SCAN --扫描完成--> ST_IDLE
 //   ST_IDLE --load_start--> ST_LOAD_HDR --> ST_LOAD_WAIT --> ST_LOAD_DATA
 //                         --整张 BMP 读完--------------------------> ST_IDLE
 //
 // ST_SCAN 与 ST_LOAD_DATA 不会同时成立；它们只是由同一个 state 寄存器
 // 控制的两个先后阶段。
 // --------------------------------------------------------------------------
 always @(posedge clk or posedge rst) begin
    if (rst) begin
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;
        scan_sector       <= 32'd0;
        load_sector_latched <= 32'd0;
    end else if (!sd_init_done || load_abort) begin
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;
        scan_sector       <= 32'd0;
        load_sector_latched <= 32'd0;
    end else begin
        scan_found_valid <= 1'b0;

        case (state)
             // 空闲状态：只接受一个新任务。
             // scan_start 与 load_start 同时有效时，扫描优先。
             ST_IDLE: begin
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;

                if (scan_start) begin
                    scan_done        <= 1'b0;
                    scan_found_total <= 3'd0;
                    scan_sector      <= scan_start_sector;
                    sd_sec_read_addr <= scan_start_sector;
                    state            <= ST_SCAN;
                end else if (load_start) begin
                    load_sector_latched <= load_sector;
                    sd_sec_read_addr    <= load_sector;
                    state               <= ST_LOAD_HDR;
                end
            end

             // 扫描状态：读当前扇区，并在扇区结束时检查 header_match。
             // 找到 BMP 后记录起始扇区；没找到则地址加 1 继续扫描。
             ST_SCAN: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;

                    if (header_match) begin
                        scan_found_valid  <= 1'b1;
                        scan_found_sector <= scan_sector;
                        scan_found_total  <= scan_found_total + 3'd1;

                        if ((scan_found_total + 3'd1 >= scan_target_count) || (next_scan_sector_if_match > scan_max_sector)) begin
                            scan_done        <= 1'b1;
                            state            <= ST_IDLE;
                            sd_sec_read_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;
                        end else begin
                            sd_sec_read_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;
                        end
                    end else begin
                        if (scan_sector >= scan_max_sector) begin
                            scan_done <= 1'b1;
                            state     <= ST_IDLE;
                        end else begin
                            sd_sec_read_addr <= next_scan_sector_if_miss;
                            scan_sector      <= next_scan_sector_if_miss;
                        end
                    end
                end
            end

             // 加载阶段第一步：重新读取目标 BMP 的起始扇区。
             // 这里再次解析文件头，是为了获得 pixel_offset/file_len 等参数，
             // 并确认即将加载的文件仍然符合要求。
             ST_LOAD_HDR: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (header_match) begin
                        write_req        <= 1'b1;
                        sd_sec_read_addr <= load_sector_latched;
                        state            <= ST_LOAD_WAIT;
                    end else begin
                        state <= ST_IDLE;
                    end
                end
            end

             // 加载阶段第二步：等待后级帧缓存写入模块应答。
             // write_req_ack 到来后，才开始真正读取并输出像素。
             ST_LOAD_WAIT: begin
                state_code <= 4'd3;
                if (write_req_ack) begin
                    write_req <= 1'b0;
                    state     <= ST_LOAD_DATA;
                end
            end

             // 加载阶段第三步：重新从 BMP 起始扇区读取整个文件。
             // bmp_len_cnt 跟踪文件进度；只有到达 pixel_offset 后，
             // bmp_data_wr_en 才会随着每3个像素字节产生脉冲。
             ST_LOAD_DATA: begin
                state_code  <= 4'd4;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (bmp_len_cnt >= file_len) begin
                        state <= ST_IDLE;
                    end else begin
                        sd_sec_read_addr <= sd_sec_read_addr + 32'd1;
                    end
                end
            end

            default: begin
                state       <= ST_IDLE;
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;
            end
        endcase
    end
end

endmodule
