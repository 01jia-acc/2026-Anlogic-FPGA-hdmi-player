`timescale 1ns/1ps

// FAT32 图片源：SD 原驱动 + 扇区适配 + 卷扫描 + FAT 文件流 + BMP 解码。
// 上层只看到扫描结果、帧写请求和 RGB 像素流，显示/双缓冲逻辑无需知道
// 图片在卡上的物理位置或簇链结构。
module sd_fat32_image_source #(
    parameter integer SPI_LOW_SPEED_DIV  = 248,
    parameter integer SPI_HIGH_SPEED_DIV = 4,
    parameter integer MAX_VISITED_CLUSTERS = 8192
) (
    input  wire        clk,
    input  wire        rst,

    input  wire        scan_start,
    output wire        scan_busy,
    output wire        scan_done,
    output wire [7:0]  scan_error,
    output wire [3:0]  resource_valid,
    output wire [31:0] resource0_cluster,
    output wire [31:0] resource1_cluster,
    output wire [31:0] resource2_cluster,
    output wire [31:0] resource3_cluster,
    output wire [31:0] resource0_size,
    output wire [31:0] resource1_size,
    output wire [31:0] resource2_size,
    output wire [31:0] resource3_size,

    input  wire        load_start,
    input  wire        load_abort,
    input  wire [31:0] load_cluster,
    input  wire [31:0] load_size,
    output wire        source_ready,
    output wire        load_busy,
    output reg         load_done,
    output reg  [7:0]  load_error,

    output wire        write_req,
    input  wire        write_req_ack,
    output wire        write_en,
    output wire [31:0] write_data,

    output wire        sd_init_done,
    output wire        SD_nCS,
    output wire        SD_DCLK,
    output wire        SD_MOSI,
    input  wire        SD_MISO
);
    wire sd_sec_read;
    wire [31:0] sd_sec_read_addr;
    wire [7:0] sd_sec_read_data;
    wire sd_sec_read_data_valid;
    wire sd_sec_read_end;
    wire sd_sec_write_data_req;
    wire sd_sec_write_end;

    wire scanner_sector_req;
    wire [31:0] scanner_sector_lba;
    wire scanner_sector_busy;
    wire scanner_sector_valid;
    wire [7:0] scanner_sector_byte;
    wire [8:0] scanner_sector_byte_index;
    wire scanner_sector_done;
    wire [7:0] scanner_sector_error;

    wire streamer_sector_req;
    wire [31:0] streamer_sector_lba;
    wire streamer_sector_busy;
    wire streamer_sector_valid;
    wire [7:0] streamer_sector_byte;
    wire [8:0] streamer_sector_byte_index;
    wire streamer_sector_done;
    wire [7:0] streamer_sector_error;

    wire sector_req;
    wire [31:0] sector_lba;
    wire sector_busy;
    wire sector_valid;
    wire [7:0] sector_byte;
    wire [8:0] sector_byte_index;
    wire sector_done;
    wire [7:0] sector_error;

    wire scanner_scan_busy;
    wire streamer_file_busy;
    wire streamer_file_done;
    wire streamer_file_valid;
    wire [7:0] streamer_file_byte;
    wire [31:0] streamer_file_byte_offset;
    wire [7:0] streamer_fs_error;
    wire [31:0] fat_start_lba;
    wire [31:0] data_start_lba;
    wire [7:0] sectors_per_cluster;
    wire [31:0] max_cluster;

    wire decoder_header_accepted;
    wire decoder_pixel_valid;
    wire [23:0] decoder_pixel_rgb;
    wire decoder_frame_commit;
    wire decoder_done;
    wire [7:0] decoder_error;

    reg [1:0] source_state;
    reg [31:0] load_cluster_q;
    reg [31:0] load_size_q;
    reg streamer_start;
    reg decoder_start;

    localparam SOURCE_IDLE  = 2'd0;
    localparam SOURCE_ACK   = 2'd1;
    localparam SOURCE_STREAM= 2'd2;

    assign scan_busy = scanner_scan_busy;
    assign source_ready = (source_state == SOURCE_IDLE) && !scanner_scan_busy;
    assign load_busy = (source_state != SOURCE_IDLE);
    assign write_req = (source_state == SOURCE_ACK);
    assign write_en = (source_state == SOURCE_STREAM) && decoder_pixel_valid;
    assign write_data = {decoder_pixel_rgb, 8'h00};

    // 扫描和文件读取是互斥事务；用扫描器 busy 作为仲裁优先级。
    assign sector_req = scanner_scan_busy ? scanner_sector_req : streamer_sector_req;
    assign sector_lba = scanner_scan_busy ? scanner_sector_lba : streamer_sector_lba;
    assign scanner_sector_busy = sector_busy;
    assign scanner_sector_valid = sector_valid;
    assign scanner_sector_byte = sector_byte;
    assign scanner_sector_byte_index = sector_byte_index;
    assign scanner_sector_done = sector_done;
    assign scanner_sector_error = sector_error;
    assign streamer_sector_busy = sector_busy;
    assign streamer_sector_valid = sector_valid;
    assign streamer_sector_byte = sector_byte;
    assign streamer_sector_byte_index = sector_byte_index;
    assign streamer_sector_done = sector_done;
    assign streamer_sector_error = sector_error;

    sd_card_top #(
        .SPI_LOW_SPEED_DIV (SPI_LOW_SPEED_DIV),
        .SPI_HIGH_SPEED_DIV(SPI_HIGH_SPEED_DIV)
    ) u_sd_card_top (
        .clk                  (clk),
        .rst                  (rst),
        .SD_nCS               (SD_nCS),
        .SD_DCLK              (SD_DCLK),
        .SD_MOSI              (SD_MOSI),
        .SD_MISO              (SD_MISO),
        .sd_init_done         (sd_init_done),
        .sd_sec_read          (sd_sec_read),
        .sd_sec_read_addr     (sd_sec_read_addr),
        .sd_sec_read_data     (sd_sec_read_data),
        .sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_end      (sd_sec_read_end),
        .sd_sec_write         (1'b0),
        .sd_sec_write_addr    (32'd0),
        .sd_sec_write_data    (8'd0),
        .sd_sec_write_data_req(sd_sec_write_data_req),
        .sd_sec_write_end     (sd_sec_write_end)
    );

    sd_sector_adapter u_sd_sector_adapter (
        .clk                  (clk),
        .rst                  (rst),
        .sector_req           (sector_req),
        .sector_lba           (sector_lba),
        .sector_busy          (sector_busy),
        .sector_valid         (sector_valid),
        .sector_byte          (sector_byte),
        .sector_byte_index    (sector_byte_index),
        .sector_done          (sector_done),
        .sector_error         (sector_error),
        .sd_sec_read          (sd_sec_read),
        .sd_sec_read_addr     (sd_sec_read_addr),
        .sd_sec_read_data     (sd_sec_read_data),
        .sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_end      (sd_sec_read_end)
    );

    fat32_volume_scanner u_volume_scanner (
        .clk                  (clk),
        .rst_n                (~rst),
        .scan_start           (scan_start),
        .scan_busy            (scanner_scan_busy),
        .scan_done            (scan_done),
        .fs_error             (scan_error),
        .partition_lba        (),
        .sectors_per_cluster  (sectors_per_cluster),
        .fat_start_lba        (fat_start_lba),
        .data_start_lba       (data_start_lba),
        .max_cluster          (max_cluster),
        .resource_valid       (resource_valid),
        .resource0_cluster    (resource0_cluster),
        .resource1_cluster    (resource1_cluster),
        .resource2_cluster    (resource2_cluster),
        .resource3_cluster    (resource3_cluster),
        .resource0_size       (resource0_size),
        .resource1_size       (resource1_size),
        .resource2_size       (resource2_size),
        .resource3_size       (resource3_size),
        .sector_req           (scanner_sector_req),
        .sector_lba           (scanner_sector_lba),
        .sector_busy          (scanner_sector_busy),
        .sector_valid         (scanner_sector_valid),
        .sector_byte          (scanner_sector_byte),
        .sector_byte_index    (scanner_sector_byte_index),
        .sector_done          (scanner_sector_done),
        .sector_error         (scanner_sector_error)
    );

    fat32_file_streamer #(
        .MAX_VISITED_CLUSTERS(MAX_VISITED_CLUSTERS)
    ) u_file_streamer (
        .clk                  (clk),
        .rst_n                (~rst),
        .file_start           (streamer_start),
        .file_abort           (load_abort),
        .first_cluster        (load_cluster_q),
        .file_size            (load_size_q),
        .sectors_per_cluster  (sectors_per_cluster),
        .fat_start_lba        (fat_start_lba),
        .data_start_lba       (data_start_lba),
        .max_cluster          (max_cluster),
        .file_busy            (streamer_file_busy),
        .file_done            (streamer_file_done),
        .file_valid           (streamer_file_valid),
        .file_byte            (streamer_file_byte),
        .file_byte_offset     (streamer_file_byte_offset),
        .fs_error             (streamer_fs_error),
        .sector_req           (streamer_sector_req),
        .sector_lba           (streamer_sector_lba),
        .sector_busy          (streamer_sector_busy),
        .sector_valid         (streamer_sector_valid),
        .sector_byte          (streamer_sector_byte),
        .sector_byte_index    (streamer_sector_byte_index),
        .sector_done          (streamer_sector_done),
        .sector_error         (streamer_sector_error)
    );

    bmp24_decoder #(
        .EXPECTED_WIDTH (640),
        .EXPECTED_HEIGHT(480)
    ) u_bmp24_decoder (
        .clk                  (clk),
        .rst_n                (~rst),
        .file_start           (decoder_start),
        .file_abort           (load_abort),
        .file_size            (load_size_q),
        .file_valid           (streamer_file_valid),
        .file_byte            (streamer_file_byte),
        .file_byte_offset     (streamer_file_byte_offset),
        .file_end             (streamer_file_done),
        .upstream_error       (streamer_fs_error),
        .header_accepted      (decoder_header_accepted),
        .pixel_valid          (decoder_pixel_valid),
        .pixel_rgb            (decoder_pixel_rgb),
        .pixel_x              (),
        .pixel_y              (),
        .frame_commit         (decoder_frame_commit),
        .bmp_done             (decoder_done),
        .bmp_error            (decoder_error),
        .top_down             (),
        .pixel_offset         (),
        .row_stride           ()
    );

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            source_state <= SOURCE_IDLE;
            load_cluster_q <= 32'd0;
            load_size_q <= 32'd0;
            streamer_start <= 1'b0;
            decoder_start <= 1'b0;
            load_done <= 1'b0;
            load_error <= 8'h00;
        end else begin
            streamer_start <= 1'b0;
            decoder_start <= 1'b0;
            load_done <= 1'b0;

            if (load_abort) begin
                source_state <= SOURCE_IDLE;
                load_error <= 8'h00;
            end else begin
                case (source_state)
                    SOURCE_IDLE: begin
                        if (load_start && source_ready) begin
                            load_cluster_q <= load_cluster;
                            load_size_q <= load_size;
                            load_error <= 8'h00;
                            source_state <= SOURCE_ACK;
                        end
                    end

                    SOURCE_ACK: begin
                        // 先拿到帧写端的应答，再开始发送文件字节，避免
                        // BMP 像素在写 FIFO 尚未清空时被丢弃。
                        if (write_req_ack) begin
                            streamer_start <= 1'b1;
                            decoder_start <= 1'b1;
                            source_state <= SOURCE_STREAM;
                        end
                    end

                    SOURCE_STREAM: begin
                        if (decoder_done) begin
                            load_done <= 1'b1;
                            if (decoder_error != 8'h00)
                                load_error <= decoder_error;
                            else if (streamer_fs_error != 8'h00)
                                load_error <= streamer_fs_error;
                            else
                                load_error <= 8'h00;
                            source_state <= SOURCE_IDLE;
                        end
                    end

                    default: source_state <= SOURCE_IDLE;
                endcase
            end
        end
    end
endmodule
