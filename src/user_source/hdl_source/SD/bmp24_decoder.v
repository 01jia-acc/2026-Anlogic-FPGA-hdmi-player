`timescale 1ns/1ps

// 24 bit、无压缩 BMP 解码器。文件字节流来自 FAT32 文件读取器；本模块
// 只缓存 54 字节头和当前像素，不再假设图片连续占用 SD 扇区。
module bmp24_decoder #(
    parameter integer EXPECTED_WIDTH  = 640,
    parameter integer EXPECTED_HEIGHT = 480
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        file_start,
    input  wire        file_abort,
    input  wire [31:0] file_size,
    input  wire        file_valid,
    input  wire [7:0]  file_byte,
    input  wire [31:0] file_byte_offset,
    input  wire        file_end,
    input  wire [7:0]  upstream_error,
    output reg         header_accepted,
    output reg         pixel_valid,
    output reg  [23:0] pixel_rgb,
    output reg  [15:0] pixel_x,
    output reg  [15:0] pixel_y,
    output reg         frame_commit,
    output reg         bmp_done,
    output reg  [7:0]  bmp_error,
    output reg         top_down,
    output reg  [31:0] pixel_offset,
    output reg  [31:0] row_stride
);
    localparam ERR_NONE        = 8'h00;
    localparam ERR_SIGNATURE   = 8'h31;
    localparam ERR_DIMENSIONS  = 8'h32;
    localparam ERR_PLANES      = 8'h33;
    localparam ERR_BPP         = 8'h34;
    localparam ERR_COMPRESSION = 8'h35;
    localparam ERR_OFFSET      = 8'h36;
    localparam ERR_TRUNCATED   = 8'h37;
    localparam ERR_UPSTREAM    = 8'h38;

    localparam integer ROW_BYTES = ((EXPECTED_WIDTH * 3 + 3) / 4) * 4;
    localparam integer IMAGE_BYTES = ROW_BYTES * EXPECTED_HEIGHT;

    reg [7:0] header [0:53];
    reg decode_enabled;
    reg [31:0] byte_in_row;
    reg [31:0] physical_row;
    reg [31:0] pixels_received;
    reg [1:0]  channel_phase;
    reg [7:0]  blue_hold;
    reg [7:0]  green_hold;
    reg [15:0] x_count;
    reg [31:0] parsed_width;
    reg signed [31:0] parsed_height;
    reg [31:0] header_file_size;
    reg [15:0] planes;
    reg [15:0] bits_per_pixel;
    reg [31:0] compression;

    integer i;

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

            if (file_valid && file_byte_offset < 54)
                header[file_byte_offset] <= file_byte;

            // Offset 53 is the first cycle at which all fields needed for
            // validation have already been captured in the header array.
            if (file_valid && file_byte_offset == 53) begin
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

            if (file_valid && decode_enabled &&
                file_byte_offset >= pixel_offset &&
                physical_row < EXPECTED_HEIGHT) begin
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

                if (byte_in_row == row_stride - 1'b1) begin
                    byte_in_row <= 32'd0;
                    physical_row <= physical_row + 1'b1;
                    channel_phase <= 2'd0;
                    x_count <= 16'd0;
                end else begin
                    byte_in_row <= byte_in_row + 1'b1;
                end
            end

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
