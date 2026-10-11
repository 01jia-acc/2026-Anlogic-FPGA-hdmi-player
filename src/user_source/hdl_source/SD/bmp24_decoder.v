`timescale 1ns/1ps

// 24 bit、无压缩 BMP 解码器。文件字节流来自 FAT32 文件读取器；本模块
// 只缓存 54 字节头和当前像素，不再假设图片连续占用 SD 扇区。
//
// 阅读导航：先收取文件偏移 0~53 的 BMP/DIB 头；偏移 53 到达时校验格式；
// 校验成功后，从 pixel_offset 开始按 B、G、R 三字节组成 RGB888 像素；
// 每行还要跳过 BMP 的 4 字节对齐填充，最终输出 pixel_valid 脉冲。
//
// 本模块只做“字节流 -> 像素流”，不负责 SD、FAT、帧缓存或 HDMI。这个边界
// 让文件在 SD 上是否连续不再影响 BMP 解码，也让显示链路不必理解 FAT 簇链。
module bmp24_decoder #(
    parameter integer EXPECTED_WIDTH  = 640,
    parameter integer EXPECTED_HEIGHT = 480
) (
    // 文件流与解码器使用同一个时钟域，复位低有效。
    input  wire        clk,
    input  wire        rst_n,
    // 一拍开始/中止事件。
    input  wire        file_start,
    input  wire        file_abort,
    // FAT 目录项记录的文件长度，用于防止头部声明超出实际文件。
    input  wire [31:0] file_size,
    // 文件流当前字节及其文件内偏移。
    input  wire        file_valid,
    input  wire [7:0]  file_byte,
    input  wire [31:0] file_byte_offset,
    // 文件流结束和上游错误码。
    input  wire        file_end,
    input  wire [7:0]  upstream_error,
    // 头部合法标志；在第 53 字节处理后锁存。
    output reg         header_accepted,
    // 当前拍产生一个完整 RGB 像素。
    output reg         pixel_valid,
    // RGB888，顺序为 {R,G,B}。
    output reg  [23:0] pixel_rgb,
    // 当前像素坐标；本模块输出文件中的物理行顺序。
    output reg  [15:0] pixel_x,
    output reg  [15:0] pixel_y,
    // 整帧像素完整且无错误时的提交脉冲。
    output reg         frame_commit,
    // 文件流结束后的完成脉冲，不等同于成功提交。
    output reg         bmp_done,
    // BMP 格式或上游错误码。
    output reg  [7:0]  bmp_error,
    // 负高度（top-down）标志；当前后级统一处理垂直翻转。
    output reg         top_down,
    // 文件头给出的像素数据起始偏移。
    output reg  [31:0] pixel_offset,
    // 每行字节数，包含 4 字节对齐填充。
    output reg  [31:0] row_stride
);
    // 错误码：31~36 表示 BMP 自身格式问题，37/38 表示数据不完整或上游失败。
    localparam ERR_NONE        = 8'h00;
    localparam ERR_SIGNATURE   = 8'h31;
    localparam ERR_DIMENSIONS  = 8'h32;
    localparam ERR_PLANES      = 8'h33;
    localparam ERR_BPP         = 8'h34;
    localparam ERR_COMPRESSION = 8'h35;
    localparam ERR_OFFSET      = 8'h36;
    localparam ERR_TRUNCATED   = 8'h37;
    localparam ERR_UPSTREAM    = 8'h38;

    // 24 位 BMP 每像素 3 字节，但每行必须按 4 字节对齐；640*3=1920 已对齐。
    localparam integer ROW_BYTES = ((EXPECTED_WIDTH * 3 + 3) / 4) * 4;
    // 完整像素区应有的字节数（不含 54 字节头）。
    localparam integer IMAGE_BYTES = ROW_BYTES * EXPECTED_HEIGHT;

    // 54 字节 BMP 文件头 + BITMAPINFOHEADER 的缓存；磁盘小端字段通过 le16/le32 读取。
    reg [7:0] header [0:53];
    // 头部验证成功后才允许像素解码。
    reg decode_enabled;
    // 当前行已经消费的字节数，包含行尾 padding。
    reg [31:0] byte_in_row;
    // 文件中的物理行号；此处先保留文件顺序，后级可完成统一垂直翻转。
    reg [31:0] physical_row;
    // 已产生的像素数，用于 file_end 时确认完整输出。
    reg [31:0] pixels_received;
    // 当前像素通道阶段：0 等待 B，1 等待 G，2 等待 R。
    reg [1:0]  channel_phase;
    // 暂存一个像素的 B/G 通道；收到 R 时组合成 {R,G,B}。
    reg [7:0]  blue_hold;
    reg [7:0]  green_hold;
    // 当前行的像素横坐标。
    reg [15:0] x_count;
    // 头部解析出的字段寄存器，便于观察/调试。
    reg [31:0] parsed_width;
    reg signed [31:0] parsed_height;
    reg [31:0] header_file_size;
    reg [15:0] planes;
    reg [15:0] bits_per_pixel;
    reg [31:0] compression;

    integer i;

    // 磁盘格式字段为 little-endian；这些函数是组合读取缓存的工具函数。
    function [15:0] le16;
        input integer offset;
        begin
            le16 = {header[offset+1], header[offset]};
        end
    endfunction

    function [31:0] le32;
        input integer offset;
        begin
            le32 = {header[offset+3], header[offset+2],
                    header[offset+1], header[offset]};
        end
    endfunction

    // 解码器的时序控制中心：接收文件字节、推进像素组装、输出事件。
    // file_valid 和 file_end 都是上游单周期事件，不能当作持续电平。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            header_accepted <= 1'b0;
            pixel_valid     <= 1'b0;
            pixel_rgb       <= 24'd0;
            pixel_x         <= 16'd0;
            pixel_y         <= 16'd0;
            frame_commit    <= 1'b0;
            bmp_done        <= 1'b0;
            bmp_error       <= ERR_NONE;
            top_down        <= 1'b0;
            pixel_offset    <= 32'd0;
            row_stride      <= 32'd0;
            decode_enabled  <= 1'b0;
            byte_in_row     <= 32'd0;
            physical_row    <= 32'd0;
            pixels_received <= 32'd0;
            channel_phase   <= 2'd0;
            blue_hold       <= 8'd0;
            green_hold      <= 8'd0;
            x_count         <= 16'd0;
            parsed_width    <= 32'd0;
            parsed_height   <= 32'sd0;
            header_file_size<= 32'd0;
            planes          <= 16'd0;
            bits_per_pixel  <= 16'd0;
            compression     <= 32'd0;
            for (i = 0; i < 54; i = i + 1)
                header[i] <= 8'd0;
        end else if (file_abort) begin
            // 中止优先级高于普通数据处理：清掉进行中的头部/像素状态，防止
            // 下一张图继承旧的 channel_phase 或 pixels_received。
            header_accepted <= 1'b0;
            pixel_valid     <= 1'b0;
            frame_commit    <= 1'b0;
            bmp_done        <= 1'b0;
            bmp_error       <= ERR_NONE;
            decode_enabled  <= 1'b0;
            byte_in_row     <= 32'd0;
            physical_row    <= 32'd0;
            pixels_received <= 32'd0;
            channel_phase   <= 2'd0;
            x_count         <= 16'd0;
        end else begin
            pixel_valid  <= 1'b0;
            frame_commit <= 1'b0;
            bmp_done     <= 1'b0;

            if (file_start) begin
                // file_start 只做一次事务初始化；真正的字节由 file_valid 驱动。
                header_accepted <= 1'b0;
                bmp_error       <= ERR_NONE;
                top_down        <= 1'b0;
                pixel_offset    <= 32'd0;
                row_stride      <= 32'd0;
                decode_enabled  <= 1'b0;
                byte_in_row     <= 32'd0;
                physical_row    <= 32'd0;
                pixels_received <= 32'd0;
                channel_phase   <= 2'd0;
                x_count         <= 16'd0;
                parsed_width    <= 32'd0;
                parsed_height   <= 32'sd0;
                header_file_size<= 32'd0;
                planes          <= 16'd0;
                bits_per_pixel  <= 16'd0;
                compression     <= 32'd0;
            end

            // 头部字节按文件偏移写入缓存。因为 file_byte_offset 与 valid 同拍，
            // 不要求文件在扇区边界对齐。
            if (file_valid && file_byte_offset < 54)
                header[file_byte_offset] <= file_byte;

            // Offset 53 is the first cycle at which all fields needed for
            // validation have already been captured in the header array.
            if (file_valid && file_byte_offset == 53) begin
                // 第 53 字节到来时，前 53 个字节已在此前时钟沿写入；因此读取
                // header[0..52] 是安全的，header[53] 本身不是校验所需字段。
                parsed_width     <= le32(18);
                parsed_height    <= le32(22);
                header_file_size <= le32(2);
                planes           <= le16(26);
                bits_per_pixel   <= le16(28);
                compression      <= le32(30);

                if (header[0] != "B" || header[1] != "M") begin
                    bmp_error <= ERR_SIGNATURE;
                end else if (le32(14) < 40 ||
                             le32(18) != EXPECTED_WIDTH ||
                             le32(22) != EXPECTED_HEIGHT) begin
                    bmp_error <= ERR_DIMENSIONS;
                end else if (le16(26) != 16'd1) begin
                    bmp_error <= ERR_PLANES;
                end else if (le16(28) != 16'd24) begin
                    bmp_error <= ERR_BPP;
                end else if (le32(30) != 0) begin
                    bmp_error <= ERR_COMPRESSION;
                end else if (le32(10) < 54 || le32(10) >= file_size) begin
                    bmp_error <= ERR_OFFSET;
                end else if (le32(10) + IMAGE_BYTES > file_size ||
                             (le32(2) != 0 &&
                              (le32(2) > file_size ||
                               le32(2) < le32(10) + IMAGE_BYTES))) begin
                    bmp_error <= ERR_TRUNCATED;
                end else begin
                    header_accepted <= 1'b1;
                    decode_enabled  <= 1'b1;
                    top_down        <= (le32(22) & 32'h80000000) != 0;
                    pixel_offset    <= le32(10);
                    row_stride      <= ROW_BYTES;
                end
            end

            // 头校验成功后才进入像素流。一个像素跨三个 file_valid 周期，
            // 第三个字节（R）到达的周期产生 pixel_valid。
            if (file_valid && decode_enabled &&
                file_byte_offset >= pixel_offset &&
                physical_row < EXPECTED_HEIGHT) begin
                // 行前 EXPECTED_WIDTH*3 字节是 BGR 像素；其后的字节是 padding，
                // 只推进行计数，不产生像素。
                if (byte_in_row < EXPECTED_WIDTH * 3) begin
                    case (channel_phase)
                        2'd0: begin
                            blue_hold <= file_byte;
                            channel_phase <= 2'd1;
                        end
                        2'd1: begin
                            green_hold <= file_byte;
                            channel_phase <= 2'd2;
                        end
                        default: begin
                            pixel_valid <= 1'b1;
                            pixel_rgb <= {file_byte, green_hold, blue_hold};
                            pixel_x <= x_count;
                            // frame_fifo_write performs the project-wide
                            // vertical flip, so keep file order here.
                            pixel_y <= physical_row[15:0];
                            pixels_received <= pixels_received + 1'b1;
                            x_count <= x_count + 1'b1;
                            channel_phase <= 2'd0;
                        end
                    endcase
                end

                // 到达行尾（包括 padding）后换行并重置通道状态。
                if (byte_in_row == row_stride - 1'b1) begin
                    byte_in_row <= 32'd0;
                    physical_row <= physical_row + 1'b1;
                    channel_phase <= 2'd0;
                    x_count <= 16'd0;
                end else begin
                    byte_in_row <= byte_in_row + 1'b1;
                end
            end

            // file_end 时错误优先传播；只有头部合法且像素计数完整才提交整帧。
            if (file_end) begin
                bmp_done <= 1'b1;
                decode_enabled <= 1'b0;
                if (upstream_error != ERR_NONE)
                    bmp_error <= ERR_UPSTREAM;
                else if (bmp_error == ERR_NONE && header_accepted &&
                         pixels_received == EXPECTED_WIDTH * EXPECTED_HEIGHT)
                    frame_commit <= 1'b1;
                else if (bmp_error == ERR_NONE)
                    bmp_error <= ERR_TRUNCATED;
            end
        end
    end
endmodule
