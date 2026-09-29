`timescale 1ns/1ps

// FAT32 版本的图片调度器。按键、双缓冲、自动轮播和帧提交协议沿用原工程，
// 图片索引从“起始物理扇区”改为 FAT32 目录项提供的“起始簇 + 文件大小”。
module sd_card_bmp_fat32 #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd4,
    parameter integer LONG_PRESS_MS     = 1000,
    parameter integer LOAD_TIMEOUT_MS   = 5_000
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        key_next,
    input  wire        key_auto,
    input  wire [15:0] bmp_width,
    input  wire [15:0] bmp_height,
    output reg         display_valid,
    output wire [3:0]  state_code,
    input  wire        write_finish_toggle,
    output reg  [1:0]  write_buf_idx,
    output reg  [1:0]  disp_buf_idx,
    output wire        write_req,
    input  wire        write_req_ack,
    output wire        write_en,
    output wire [31:0] write_data,
    output wire        SD_nCS,
    output wire        SD_DCLK,
    output wire        SD_MOSI,
    input  wire        SD_MISO
);
    // bmp_width/bmp_height and the legacy scan parameters remain in the port
    // list for top-level compatibility; FAT32 directory scanning supplies the
    // image list and bmp24_decoder validates the fixed 640x480 format.
    wire key_next_press;
    wire key_auto_press;
    wire key_auto_stable;

    wire sd_init_done;
    wire source_ready;
    wire source_load_busy;
    wire source_load_done;
    wire [7:0] source_load_error;
    wire scan_busy;
    wire scan_done;
    wire [7:0] scan_error;
    wire [3:0] resource_valid;
    wire [31:0] resource0_cluster, resource1_cluster;
    wire [31:0] resource2_cluster, resource3_cluster;
    wire [31:0] resource0_size, resource1_size;
    wire [31:0] resource2_size, resource3_size;

    reg scan_start_pulse;
    reg load_start_pulse;
    reg load_abort_pulse;
    reg scan_kicked;
    reg scan_completed;
    reg first_image_committed;
    reg auto_play_en;
    reg [31:0] auto_cnt;
    reg [2:0] img_found_count;
    reg [1:0] img_idx;
    reg [1:0] load_idx;
    reg [1:0] pending_buf_idx;
    reg [31:0] img_cluster0, img_cluster1, img_cluster2, img_cluster3;
    reg [31:0] img_size0, img_size1, img_size2, img_size3;
    reg [3:0] image_resource_valid;
    reg next_req_pending;
    reg prev_req_pending;
    reg key_auto_down_seen;
    reg [31:0] long_press_cnt;
    reg long_press_marked;
    reg load_busy;
    reg [31:0] load_timeout_cnt;
    reg source_done_seen;
    reg write_done_seen;
    reg [7:0] last_error;
    reg [2:0] wrfin_tgl_sync;
    reg [3:0] state_code_r;

    wire write_finish_pulse;
    wire auto_tick;
    wire [1:0] next_from_current;
    wire [1:0] prev_from_current;
    wire [31:0] load_cluster_lut;
    wire [31:0] load_size_lut;

    localparam integer LONG_PRESS_CYCLES =
        (CLK_FREQ_HZ / 1000) * LONG_PRESS_MS;
    localparam integer LOAD_TIMEOUT_CYCLES =
        (CLK_FREQ_HZ / 1000) * LOAD_TIMEOUT_MS;

    assign state_code = state_code_r;
    assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];
    assign auto_tick = (auto_cnt == CLK_FREQ_HZ - 1);

    key_press_debounce_fat32 #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .DEBOUNCE_MS(50)
    ) u_key_next (
        .clk(clk), .rst(rst), .button_in(key_next),
        .press_pulse(key_next_press), .button_stable_out()
    );

    key_press_debounce_fat32 #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .DEBOUNCE_MS(50)
    ) u_key_auto (
        .clk(clk), .rst(rst), .button_in(key_auto),
        .press_pulse(key_auto_press), .button_stable_out(key_auto_stable)
    );

    sd_fat32_image_source u_image_source (
        .clk(clk), .rst(rst),
        .scan_start(scan_start_pulse), .scan_busy(scan_busy),
        .scan_done(scan_done), .scan_error(scan_error),
        .resource_valid(resource_valid),
        .resource0_cluster(resource0_cluster), .resource1_cluster(resource1_cluster),
        .resource2_cluster(resource2_cluster), .resource3_cluster(resource3_cluster),
        .resource0_size(resource0_size), .resource1_size(resource1_size),
        .resource2_size(resource2_size), .resource3_size(resource3_size),
        .load_start(load_start_pulse), .load_abort(load_abort_pulse),
        .load_cluster(load_cluster_lut), .load_size(load_size_lut),
        .source_ready(source_ready), .load_busy(source_load_busy),
        .load_done(source_load_done), .load_error(source_load_error),
        .write_req(write_req), .write_req_ack(write_req_ack),
        .write_en(write_en), .write_data(write_data),
        .sd_init_done(sd_init_done),
        .SD_nCS(SD_nCS), .SD_DCLK(SD_DCLK), .SD_MOSI(SD_MOSI), .SD_MISO(SD_MISO)
    );

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

    function [1:0] next_index_limited;
        input [1:0] cur;
        input [2:0] count;
        begin
            case (count)
                3'd0,3'd1: next_index_limited = 2'd0;
                3'd2: next_index_limited = (cur == 2'd1) ? 2'd0 : cur + 1'b1;
                3'd3: next_index_limited = (cur == 2'd2) ? 2'd0 : cur + 1'b1;
                default: next_index_limited = (cur == 2'd3) ? 2'd0 : cur + 1'b1;
            endcase
        end
    endfunction

    function [1:0] prev_index_limited;
        input [1:0] cur;
        input [2:0] count;
        begin
            case (count)
                3'd0,3'd1: prev_index_limited = 2'd0;
                3'd2: prev_index_limited = (cur == 0) ? 2'd1 : cur - 1'b1;
                3'd3: prev_index_limited = (cur == 0) ? 2'd2 : cur - 1'b1;
                default: prev_index_limited = (cur == 0) ? 2'd3 : cur - 1'b1;
            endcase
        end
    endfunction

    function [1:0] next_buffer;
        input [1:0] current;
        input valid;
        begin
            if (!valid) next_buffer = 2'd0;
            else if (current == 2'd0) next_buffer = 2'd1;
            else next_buffer = 2'd0;
        end
    endfunction

    function [31:0] cluster_lut;
        input [1:0] index;
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

    function [31:0] size_lut;
        input [1:0] index;
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

    assign next_from_current = next_index_limited(img_idx, img_found_count);
    assign prev_from_current = prev_index_limited(img_idx, img_found_count);
    assign load_cluster_lut = cluster_lut(load_idx);
    assign load_size_lut = size_lut(load_idx);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            wrfin_tgl_sync <= 3'b000;
            scan_start_pulse <= 1'b0;
            load_start_pulse <= 1'b0;
            load_abort_pulse <= 1'b0;
            scan_kicked <= 1'b0;
            scan_completed <= 1'b0;
            first_image_committed <= 1'b0;
            auto_play_en <= 1'b0;
            auto_cnt <= 32'd0;
            img_found_count <= 3'd0;
            img_idx <= 2'd0;
            load_idx <= 2'd0;
            pending_buf_idx <= 2'd0;
            write_buf_idx <= 2'd0;
            disp_buf_idx <= 2'd0;
            img_cluster0 <= 0; img_cluster1 <= 0;
            img_cluster2 <= 0; img_cluster3 <= 0;
            img_size0 <= 0; img_size1 <= 0;
            img_size2 <= 0; img_size3 <= 0;
            image_resource_valid <= 4'd0;
            next_req_pending <= 1'b0;
            prev_req_pending <= 1'b0;
            key_auto_down_seen <= 1'b0;
            long_press_cnt <= 0;
            long_press_marked <= 1'b0;
            load_busy <= 1'b0;
            load_timeout_cnt <= 0;
            source_done_seen <= 1'b0;
            write_done_seen <= 1'b0;
            last_error <= 8'h00;
            state_code_r <= 4'd0;
            display_valid <= 1'b0;
        end else begin
            wrfin_tgl_sync <= {wrfin_tgl_sync[1:0], write_finish_toggle};
            scan_start_pulse <= 1'b0;
            load_start_pulse <= 1'b0;
            load_abort_pulse <= 1'b0;

            if (!sd_init_done) begin
                scan_kicked <= 1'b0;
                scan_completed <= 1'b0;
                first_image_committed <= 1'b0;
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

                if (scan_done) begin
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
                    last_error <= scan_error;
                end

                if (!scan_kicked && source_ready) begin
                    scan_start_pulse <= 1'b1;
                    scan_kicked <= 1'b1;
                    scan_completed <= 1'b0;
                    img_found_count <= 0;
                    display_valid <= 1'b0;
                    first_image_committed <= 1'b0;
                    last_error <= 0;
                end

                if (load_busy) begin
                    if (source_load_done) begin
                        load_timeout_cnt <= 0;
                        if (source_load_error == 0) begin
                            source_done_seen <= 1'b1;
                        end else begin
                            load_busy <= 1'b0;
                            source_done_seen <= 1'b0;
                            write_done_seen <= 1'b0;
                            last_error <= source_load_error;
                        end
                    end else if (load_timeout_cnt >= LOAD_TIMEOUT_CYCLES) begin
                        load_busy <= 1'b0;
                        load_timeout_cnt <= 0;
                        source_done_seen <= 0;
                        write_done_seen <= 0;
                        load_abort_pulse <= 1'b1;
                        last_error <= 8'he0;
                    end else begin
                        load_timeout_cnt <= load_timeout_cnt + 1'b1;
                    end
                end else begin
                    load_timeout_cnt <= 0;
                end

                if (load_busy && source_load_done && source_load_error == 0) begin
                    // source_load_done 表示解码器已经把文件字节读完；
                    // 帧写完成事件单独由跨域 toggle 提供。
                    source_done_seen <= 1'b1;
                end
                if (load_busy && write_finish_pulse)
                    write_done_seen <= 1'b1;

                // 两个事件可以任意先后到达，使用“当前值或本拍事件”判断，
                // 避免非阻塞赋值延后一拍造成提交条件漏检。
                if (load_busy &&
                    (source_done_seen ||
                     (source_load_done && source_load_error == 0)) &&
                    (write_done_seen || write_finish_pulse)) begin
                    load_busy <= 1'b0;
                    source_done_seen <= 1'b0;
                    write_done_seen <= 1'b0;
                    load_timeout_cnt <= 0;
                    disp_buf_idx <= pending_buf_idx;
                    img_idx <= load_idx;
                    display_valid <= 1'b1;
                    first_image_committed <= 1'b1;
                    last_error <= 0;
                end

                if (scan_completed && img_found_count != 0) begin
                    if (key_next_press)
                        next_req_pending <= 1'b1;
                    if (key_auto_press)
                        key_auto_down_seen <= 1'b1;

                    if (!key_auto_stable) begin
                        if (long_press_cnt < LONG_PRESS_CYCLES - 1)
                            long_press_cnt <= long_press_cnt + 1'b1;
                        else if (!long_press_marked) begin
                            long_press_marked <= 1'b1;
                            if (img_found_count > 1)
                                prev_req_pending <= 1'b1;
                        end
                    end else begin
                        long_press_cnt <= 0;
                        if (key_auto_down_seen) begin
                            if (!long_press_marked && img_found_count > 1) begin
                                auto_play_en <= ~auto_play_en;
                                auto_cnt <= 0;
                            end
                            key_auto_down_seen <= 1'b0;
                        end
                        long_press_marked <= 1'b0;
                    end

                    if (auto_play_en && display_valid && !load_busy && img_found_count > 1) begin
                        if (auto_tick) auto_cnt <= 0;
                        else auto_cnt <= auto_cnt + 1'b1;
                    end else begin
                        auto_cnt <= 0;
                    end

                    // 首图、下一张、上一张、自动轮播依次仲裁。
                    if (!first_image_committed && source_ready && !load_busy) begin
                        load_idx <= 0;
                        pending_buf_idx <= 0;
                        write_buf_idx <= 0;
                        load_start_pulse <= 1'b1;
                        load_busy <= 1'b1;
                        source_done_seen <= 0;
                        write_done_seen <= 0;
                    end else if (display_valid && source_ready && !load_busy &&
                                 next_req_pending) begin
                        load_idx <= next_from_current;
                        pending_buf_idx <= next_buffer(disp_buf_idx, display_valid);
                        write_buf_idx <= next_buffer(disp_buf_idx, display_valid);
                        load_start_pulse <= 1'b1;
                        load_busy <= 1'b1;
                        next_req_pending <= 0;
                        source_done_seen <= 0;
                        write_done_seen <= 0;
                    end else if (display_valid && source_ready && !load_busy &&
                                 prev_req_pending) begin
                        load_idx <= prev_from_current;
                        pending_buf_idx <= next_buffer(disp_buf_idx, display_valid);
                        write_buf_idx <= next_buffer(disp_buf_idx, display_valid);
                        load_start_pulse <= 1'b1;
                        load_busy <= 1'b1;
                        prev_req_pending <= 0;
                        source_done_seen <= 0;
                        write_done_seen <= 0;
                    end else if (display_valid && source_ready && !load_busy &&
                                 auto_play_en && auto_tick && img_found_count > 1) begin
                        load_idx <= next_from_current;
                        pending_buf_idx <= next_buffer(disp_buf_idx, display_valid);
                        write_buf_idx <= next_buffer(disp_buf_idx, display_valid);
                        load_start_pulse <= 1'b1;
                        load_busy <= 1'b1;
                        source_done_seen <= 0;
                        write_done_seen <= 0;
                        auto_cnt <= 0;
                    end
                end
            end
        end
    end
endmodule

module key_press_debounce_fat32 #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 50
) (
    input  wire clk,
    input  wire rst,
    input  wire button_in,
    output reg  press_pulse,
    output wire button_stable_out
);
    localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;
    reg button_sync0, button_sync1, button_stable;
    reg [31:0] cnt;

    assign button_stable_out = button_stable;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            button_sync0 <= 1'b1;
            button_sync1 <= 1'b1;
            button_stable <= 1'b1;
            cnt <= 0;
            press_pulse <= 1'b0;
        end else begin
            button_sync0 <= button_in;
            button_sync1 <= button_sync0;
            press_pulse <= 1'b0;
            if (button_sync1 == button_stable) begin
                cnt <= 0;
            end else if (cnt >= DEBOUNCE_CYCLES - 1) begin
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;
                button_stable <= button_sync1;
                cnt <= 0;
            end else begin
                cnt <= cnt + 1'b1;
            end
        end
    end
endmodule
