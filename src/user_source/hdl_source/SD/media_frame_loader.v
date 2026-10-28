`timescale 1ns/1ps

// 独立帧写入器：像素流 -> 本地 FIFO -> 连续写命令。
// 本模块只使用 clk 域；跨到 SDRAM 的 ext_mem_clk 由 media_sdram_write_cdc 完成。
module media_frame_loader #(
    parameter integer ADDR_WIDTH = 21,
    parameter integer IMAGE_WORDS = 307200,
    parameter integer FIFO_DEPTH = 512,
    parameter integer BURST_SIZE = 128,
    parameter integer STALL_TIMEOUT = 65535,
    parameter integer FRAME_WIDTH = 640,
    parameter integer FRAME_HEIGHT = 480,
    parameter integer WRITE_V_FLIP = 1
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  load_start,
    input  wire [1:0]            image_index,
    input  wire                  pixel_valid,
    input  wire [23:0]           pixel_rgb,
    input  wire                  frame_commit,
    input  wire [7:0]            decode_error,
    input  wire                  command_ready,
    output wire                  load_busy,
    output reg                   load_done,
    output reg  [3:0]            image_complete,
    output reg  [7:0]            memory_error,
    output wire [47:0]           debug_writer_status, // error / received / written
    input  wire                  controller_init_done,
    input  wire                  controller_busy,
    input  wire                  refresh_active,
    output reg                   app_wr_en,
    output reg  [ADDR_WIDTH-1:0] app_wr_addr,
    output reg  [3:0]            app_wr_mask,
    output reg  [31:0]           app_wr_data
);
    localparam [7:0] ERR_NONE          = 8'h00;
    localparam [7:0] ERR_NOT_ACTIVE    = 8'h41;
    localparam [7:0] ERR_FIFO_OVERFLOW = 8'h42;
    localparam [7:0] ERR_PIXEL_COUNT   = 8'h43;
    localparam [7:0] ERR_DECODE        = 8'h44;
    localparam [7:0] ERR_SHORT_FRAME   = 8'h45;
    localparam [7:0] ERR_WRITE_STALL   = 8'h46;

    localparam integer PTR_WIDTH = (FIFO_DEPTH <= 2) ? 1 : $clog2(FIFO_DEPTH);
    localparam integer EFFECTIVE_BURST = (BURST_SIZE < FIFO_DEPTH) ? BURST_SIZE : FIFO_DEPTH;
    localparam [1:0] LOADER_IDLE = 2'd0;
    localparam [1:0] LOADER_LOAD = 2'd1;

    // 用显式状态编码表示空闲/加载，避免 TD 将名为 active 的寄存器
    // 展开成异常的 active_reg/active_n 内部网络。
    reg [1:0] loader_state;
    reg [1:0] active_image;
    reg commit_seen;
    reg [31:0] pixels_received;
    reg [31:0] pixels_written;
    // 640x480=307200=20'h4B000. Counts stop on error until the next load.
    assign debug_writer_status={memory_error,pixels_received[19:0],pixels_written[19:0]};
    // BMP bottom-up: stream row 0 belongs to the last display row.
    // Advance only when a command is generated, so stalls cannot shift pixels.
    reg [ADDR_WIDTH-1:0] write_offset;
    reg [31:0] write_x;
    reg [31:0] stall_count;
    reg [23:0] fifo_mem [0:FIFO_DEPTH-1]; /* fehdl force_ram=1, ram_style="bram" */
    reg [23:0] fifo_read_q;
    reg [PTR_WIDTH-1:0] wr_ptr;
    reg [PTR_WIDTH-1:0] rd_ptr;
    reg [PTR_WIDTH:0] fifo_count;
    reg burst_active;
    reg [PTR_WIDTH:0] burst_remaining;

    wire loader_active = (loader_state == LOADER_LOAD);
    wire pop_allowed = loader_active && command_ready && burst_active && fifo_count != 0;
    wire push_allowed = loader_active && pixel_valid &&
                         ((fifo_count < FIFO_DEPTH) || pop_allowed);
    wire start_burst = loader_active && command_ready && !burst_active && fifo_count != 0 &&
                       ((fifo_count >= EFFECTIVE_BURST) || commit_seen);
    wire [PTR_WIDTH:0] next_burst_words =
        (fifo_count >= EFFECTIVE_BURST) ? EFFECTIVE_BURST : fifo_count;

    // Keep the synchronous RAM ports outside reset/control logic. Data is
    // qualified by app_wr_en; stale data while idle/reset is never consumed.
    wire fifo_transaction = rst_n && loader_active && decode_error==0 &&
        !(pixel_valid && pixels_received>=IMAGE_WORDS) &&
        !(pixel_valid && !push_allowed);
    always @(posedge clk) begin
        if(fifo_transaction && push_allowed) fifo_mem[wr_ptr]<=pixel_rgb;
        if(fifo_transaction && pop_allowed) fifo_read_q<=fifo_mem[rd_ptr];
    end
    always @* app_wr_data={fifo_read_q,8'd0};

    function [ADDR_WIDTH-1:0] bank_base;
        input [1:0] index;
        begin
            case (index)
                2'd0: bank_base = 0;
                2'd1: bank_base = IMAGE_WORDS;
                2'd2: bank_base = IMAGE_WORDS * 2;
                default: bank_base = IMAGE_WORDS * 3;
            endcase
        end
    endfunction

    assign load_busy = loader_active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            loader_state <= LOADER_IDLE;
            active_image <= 2'd0;
            commit_seen <= 1'b0;
            pixels_received <= 0;
            pixels_written <= 0;
            write_offset <= 0;
            write_x <= 0;
            stall_count <= 0;
            wr_ptr <= 0;
            rd_ptr <= 0;
            fifo_count <= 0;
            burst_active <= 1'b0;
            burst_remaining <= 0;
            load_done <= 1'b0;
            image_complete <= 4'b0;
            memory_error <= ERR_NONE;
            app_wr_en <= 1'b0;
            app_wr_addr <= 0;
            app_wr_mask <= 4'b0000;
        end else begin
            load_done <= 1'b0;
            app_wr_en <= 1'b0;

            if (load_start && !loader_active) begin
                loader_state <= LOADER_LOAD;
                active_image <= image_index;
                commit_seen <= 1'b0;
                pixels_received <= 0;
                pixels_written <= 0;
                write_offset <= WRITE_V_FLIP ? (FRAME_HEIGHT-1)*FRAME_WIDTH : 0;
                write_x <= 0;
                stall_count <= 0;
                wr_ptr <= 0;
                rd_ptr <= 0;
                fifo_count <= 0;
                burst_active <= 1'b0;
                burst_remaining <= 0;
                memory_error <= ERR_NONE;
                image_complete[image_index] <= 1'b0;
            end

            if (loader_active) begin
                // 错误收口全部写在这个 always 块内。
                // 不使用 task 修改 active，避免 TD 把 active 的下一状态
                // 展开成无法保留的内部 active_n 网络。
                if (decode_error != 0) begin
                    memory_error <= ERR_DECODE;
                    loader_state <= LOADER_IDLE;
                    burst_active <= 1'b0;
                    burst_remaining <= 0;
                    load_done <= 1'b1;
                end else if (pixel_valid && pixels_received >= IMAGE_WORDS) begin
                    memory_error <= ERR_PIXEL_COUNT;
                    loader_state <= LOADER_IDLE;
                    burst_active <= 1'b0;
                    burst_remaining <= 0;
                    load_done <= 1'b1;
                end else if (pixel_valid && !push_allowed) begin
                    memory_error <= ERR_FIFO_OVERFLOW;
                    loader_state <= LOADER_IDLE;
                    burst_active <= 1'b0;
                    burst_remaining <= 0;
                    load_done <= 1'b1;
                end
                else begin
                    if (start_burst) begin
                        burst_active <= 1'b1;
                        burst_remaining <= next_burst_words;
                    end

                    if (push_allowed) begin
                        // 与原写侧一致：SDRAM 一个字为 {R,G,B,8'h00}。
                        wr_ptr <= wr_ptr + 1'b1;
                        pixels_received <= pixels_received + 1'b1;
                    end

                    if (pop_allowed) begin
                        app_wr_en <= 1'b1;
                        app_wr_addr <= bank_base(active_image) + write_offset;
                        if (WRITE_V_FLIP && write_x == FRAME_WIDTH-1) begin
                            write_x <= 0;
                            write_offset <= write_offset - (2*FRAME_WIDTH-1);
                        end else begin
                            write_x <= write_x + 1'b1;
                            write_offset <= write_offset + 1'b1;
                        end
                        app_wr_mask <= 4'b0000;
                        rd_ptr <= rd_ptr + 1'b1;
                        pixels_written <= pixels_written + 1'b1;
                        stall_count <= 0;
                        if (burst_remaining == 1) begin
                            burst_active <= 1'b0;
                            burst_remaining <= 0;
                        end else begin
                            burst_remaining <= burst_remaining - 1'b1;
                        end
                    end else if (fifo_count != 0 &&
                                 (commit_seen || fifo_count >= EFFECTIVE_BURST)) begin
                        if (stall_count == STALL_TIMEOUT) begin
                            memory_error <= ERR_WRITE_STALL;
                            loader_state <= LOADER_IDLE;
                            burst_active <= 1'b0;
                            burst_remaining <= 0;
                            load_done <= 1'b1;
                        end else
                            stall_count <= stall_count + 1'b1;
                    end else begin
                        stall_count <= 0;
                    end

                    case ({push_allowed, pop_allowed})
                        2'b10: fifo_count <= fifo_count + 1'b1;
                        2'b01: fifo_count <= fifo_count - 1'b1;
                        default: fifo_count <= fifo_count;
                    endcase

                    if (frame_commit)
                        commit_seen <= 1'b1;

                    if (commit_seen && fifo_count == 0 && !pop_allowed && !burst_active) begin
                        if (pixels_received != IMAGE_WORDS || pixels_written != IMAGE_WORDS) begin
                            memory_error <= ERR_SHORT_FRAME;
                            loader_state <= LOADER_IDLE;
                            burst_active <= 1'b0;
                            burst_remaining <= 0;
                            load_done <= 1'b1;
                        end else begin
                            image_complete[active_image] <= 1'b1;
                            loader_state <= LOADER_IDLE;
                            load_done <= 1'b1;
                        end
                    end
                end
            end else if (pixel_valid) begin
                memory_error <= ERR_NOT_ACTIVE;
            end
        end
    end
endmodule
