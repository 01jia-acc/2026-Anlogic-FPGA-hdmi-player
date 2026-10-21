`timescale 1ns/1ps

module media_frame_reader #(
    parameter integer ADDR_WIDTH = 21,
    parameter integer IMAGE_WORDS = 307200,
    parameter integer FIFO_DEPTH = 512,
    parameter integer FIFO_MARGIN = 8,
    parameter integer BURST_SIZE = 128,
    parameter integer BURST_GAP_CYCLES = 12,
    parameter integer MAX_INFLIGHT_BURSTS = 2,
    parameter integer DIAG_RETURN_PATTERN = 0,
    parameter integer DIAG_SYNTH_STREAM = 0,
    parameter integer REGION_READ_ENABLE = 0,
    parameter integer ZOOM_ENABLE = 0,
    parameter integer THUMBNAIL_ENABLE = 0,
    parameter integer FRAME_WIDTH = 640,
    parameter integer FRAME_HEIGHT = 480
) (
    input  wire                  mem_clk,
    input  wire                  mem_rst_n,
    input  wire                  controller_init_done,
    input  wire                  controller_busy,
    input  wire                  refresh_active,
    input  wire                  external_write_busy,
    output reg                   app_rd_en,
    output reg  [ADDR_WIDTH-1:0] app_rd_addr,
    input  wire                  sdram_rd_valid,
    input  wire [31:0]           sdram_rd_data,

    input  wire                  pix_clk,
    input  wire                  pix_rst_n,
    input  wire                  frame_start,
    input  wire [1:0]            active_index,
    input  wire                  pixel_consume,
    output wire [23:0]           pixel_rgb,
    output wire                  pixel_valid,
    output reg                   underflow,

    output reg                   reader_busy,
    output wire                  reader_bus_busy,
    output reg                   frame_read_done,
    output reg  [1:0]            reader_index,
    output reg  [7:0]            reader_error,
    // Frame-latched prefix from active_index; suffix from old_index. Every
    // boundary must be a multiple of BURST_SIZE. Disabled ports may be omitted.
    input  wire [1:0]            old_index,
    input  wire [ADDR_WIDTH:0]   new_words,
    input  wire [1:0]            zoom_level,
    output wire [1:0]            display_zoom_level,
    input wire [31:0]            thumbnail_descriptor,
    output wire [31:0]           display_thumbnail
);
    localparam [7:0] ERR_NONE          = 8'h00;
    localparam [7:0] ERR_FRAME_OVERRUN = 8'h61;
    localparam [7:0] ERR_FIFO_OVERFLOW = 8'h62;
    localparam [7:0] ERR_UNDERFLOW     = 8'h63;
    localparam [7:0] ERR_SHORT_FRAME   = 8'h64;
    localparam integer FIFO_COUNT_W = (FIFO_DEPTH <= 2) ? 1 : $clog2(FIFO_DEPTH);
    localparam integer BURST_COUNT_W =
        (BURST_SIZE < 2) ? 1 : $clog2(BURST_SIZE + 1);
    localparam integer INFLIGHT_COUNT_W =
        (MAX_INFLIGHT_BURSTS * BURST_SIZE < 2) ? 1 :
        $clog2(MAX_INFLIGHT_BURSTS * BURST_SIZE + 1);

    reg request_pending_pix;
    reg request_toggle_pix;
    reg [1:0] request_index_pix;
    reg [1:0] request_old_index_pix;
    reg [ADDR_WIDTH:0] request_new_words_pix;
    reg frame_armed_pix;
    reg [ADDR_WIDTH:0] consumed_count_pix;

    reg request_sync1_mem;
    reg request_sync2_mem;
    reg request_seen_mem;
    reg [1:0] index_sync1_mem;
    reg [1:0] index_sync2_mem;
    reg reload_pending_mem;
    reg [1:0] reload_index_mem;
    reg [1:0] old_index_sync1_mem, old_index_sync2_mem, reload_old_index_mem;
    reg [ADDR_WIDTH:0] new_words_sync1_mem, new_words_sync2_mem, reload_new_words_mem;
    reg [1:0] frame_old_index_mem;
    reg [ADDR_WIDTH:0] frame_new_words_mem;
    reg [ADDR_WIDTH-1:0] burst_base_mem;
    reg [1:0] request_zoom_pix, zoom_sync1_mem, zoom_sync2_mem, reload_zoom_mem, frame_zoom_mem;
    assign display_zoom_level = request_zoom_pix;
    reg [31:0] thumbnail_pix;
    reg [12:0] thumbnail_sync1, thumbnail_sync2, thumbnail_reload, thumbnail_mem;
    assign display_thumbnail=thumbnail_pix;
    wire thumbnail_active=THUMBNAIL_ENABLE && thumbnail_mem[4];
    localparam integer LINE_AW = FRAME_WIDTH<=2 ? 1 : $clog2(FRAME_WIDTH);
    // Reuse the existing output FIFO. One synchronous row RAM is enough:
    // read a contiguous source row, then sample it into one output row.
    reg [23:0] zoom_line [(1<<LINE_AW)-1:0]; /* fehdl force_ram=1, ram_style="bram" */
    reg [23:0] zoom_line_q;
    localparam Z_READ=0, Z_ADDRESS=1, Z_PUSH=2;
    reg [1:0] zoom_state, zoom_x_phase, zoom_y_phase;
    reg [15:0] zoom_row_issued, zoom_row_returned, zoom_x, zoom_y, zoom_source_x;
    reg [ADDR_WIDTH-1:0] zoom_row_base;
    wire zoom_active = (ZOOM_ENABLE && frame_zoom_mem!=1) || thumbnail_active;
    // Four 1/4-size previews centered as a 2x2 block. Reuse ONE row RAM:
    // fetch left source row -> emit left half, fetch right -> emit right half.
    wire [1:0] thumbnail_slot={zoom_y>=FRAME_HEIGHT/2,zoom_x>=FRAME_WIDTH/2};
    wire [1:0] thumbnail_bank=thumbnail_mem[5+thumbnail_slot*2 +:2];
    wire thumbnail_row_valid=thumbnail_mem[thumbnail_slot] &&
        zoom_y>=FRAME_HEIGHT/4 && zoom_y<FRAME_HEIGHT*3/4;
    wire thumbnail_inside=thumbnail_row_valid &&
        zoom_x>=FRAME_WIDTH/4 && zoom_x<FRAME_WIDTH*3/4;
    wire thumbnail_window=zoom_y>=FRAME_HEIGHT/4 && zoom_y<FRAME_HEIGHT*3/4 &&
        zoom_x>=FRAME_WIDTH/4 && zoom_x<FRAME_WIDTH*3/4;
    wire [23:0] thumbnail_rgb=thumbnail_inside ? zoom_line_q :
        thumbnail_window ? 24'h202830 : 24'h080c10;
    wire zoom_border = frame_zoom_mem==0 &&
        (zoom_x<FRAME_WIDTH/4 || zoom_x>=FRAME_WIDTH*3/4 ||
         zoom_y<FRAME_HEIGHT/4 || zoom_y>=FRAME_HEIGHT*3/4);
    wire zoom_room, zoom_write;
    // 行 RAM 同步读一拍：输出当前像素时预取下一像素。FIFO 反压时
    // 同时保持地址、行 RAM 输出和坐标，恢复后不会重复或跳过像素。
    wire [15:0] zoom_next_source_x = thumbnail_active ?
        (zoom_x==FRAME_WIDTH/2-1 ? 16'd0 :
         (zoom_x>=FRAME_WIDTH/4 && zoom_x<FRAME_WIDTH*3/4-1) ?
             zoom_source_x+16'd4 : zoom_source_x) : frame_zoom_mem==0 ?
        ((zoom_x>=FRAME_WIDTH/4 && zoom_x<FRAME_WIDTH*3/4-1) ?
            zoom_source_x+16'd2 : zoom_source_x) :
        frame_zoom_mem==2 ? (zoom_x_phase>=1 ? zoom_source_x+16'd1 : zoom_source_x) :
        frame_zoom_mem==3 ? (zoom_x[0] ? zoom_source_x+16'd1 : zoom_source_x) : zoom_source_x;
    wire [LINE_AW-1:0] zoom_ram_read_address = zoom_state==Z_ADDRESS ?
        zoom_source_x[LINE_AW-1:0] : zoom_next_source_x[LINE_AW-1:0];
    wire zoom_ram_read_enable = (zoom_active && zoom_state==Z_ADDRESS && zoom_room) ||
        (zoom_write && zoom_x!=FRAME_WIDTH-1);
    always @(posedge mem_clk) begin
        if(zoom_active && sdram_rd_valid && reader_busy && !reload_pending_mem)
            zoom_line[zoom_row_returned[LINE_AW-1:0]]<=sdram_rd_data[31:8];
        // 单一 RAM 读口；把地址选择放在端口外，保证 TD 推断为 BRAM。
        if(zoom_ram_read_enable)
            zoom_line_q<=zoom_line[zoom_ram_read_address];
    end
    // synthesis translate_off
    always @(posedge pix_clk) begin
        if (pix_rst_n && REGION_READ_ENABLE && request_pending_pix && !frame_start &&
            (new_words > IMAGE_WORDS || new_words % BURST_SIZE != 0))
            $fatal(1, "Region boundary must be within the image and burst aligned");
    end
    // synthesis translate_on
    reg [2:0] fifo_reset_count_mem;
    reg [ADDR_WIDTH:0] issued_count_mem;
    reg [ADDR_WIDTH:0] returned_count_mem;
    reg [INFLIGHT_COUNT_W-1:0] inflight_count_mem;
    reg inflight_empty_mem /* fehdl keep="true" */;
    reg burst_active_mem;
    reg [BURST_COUNT_W-1:0] burst_issued_mem;
    reg [BURST_COUNT_W-1:0] burst_target_mem;
    reg [7:0] burst_gap_count_mem;
    reg [ADDR_WIDTH:0] diag_written_count_mem;
    reg [9:0] diag_column_mem;
    reg [23:0] diag_rgb_mem;
    reg [1:0] fifo_release_mem;
    reg [1:0] fifo_release_pix;

    wire fifo_reset_request = !mem_rst_n || !pix_rst_n ||
                              (fifo_reset_count_mem != 0);
    wire fifo_reset = fifo_reset_request ||
                      !fifo_release_mem[1] || !fifo_release_pix[1];
    // The FIFO also synchronizes reset release internally. Do not launch
    // SDRAM reads while its write port is still held in reset: early return
    // words would be counted by this reader but silently dropped by the FIFO.
    reg [1:0] fifo_ready_mem;
    always @(posedge mem_clk or posedge fifo_reset) begin
        if (fifo_reset)
            fifo_ready_mem <= 2'b00;
        else
            fifo_ready_mem <= {fifo_ready_mem[0], 1'b1};
    end
    wire fifo_full;
    wire fifo_empty;
    wire fifo_almost_full;
    wire [FIFO_COUNT_W-1:0] fifo_wrusedw;
    assign zoom_room = !fifo_full && fifo_wrusedw<FIFO_DEPTH-FIFO_MARGIN;
    // Stream thumbnail samples while the rest of the source row is arriving.
    // Waiting for both complete rows before emitting costs > one VGA line at
    // 50 MHz. Keep contiguous SDRAM bursts and reuse the same single row RAM.
    wire thumbnail_sample_ready = !thumbnail_active || !thumbnail_row_valid ||
        ((zoom_row_returned > zoom_source_x) &&
         (zoom_x==FRAME_WIDTH/2-1 || zoom_x==FRAME_WIDTH-1 ?
             zoom_row_returned==FRAME_WIDTH :
             zoom_row_returned > zoom_next_source_x));
    assign zoom_write = zoom_active && reader_busy && zoom_state==Z_PUSH && zoom_room &&
                        !fifo_reset && thumbnail_sample_ready;
    wire [31:0] fifo_raw_data;
    wire fifo_raw_valid;
    wire fifo_raw_read_en;
    wire prefetch_valid;
    wire [31:0] prefetch_data;
    wire pixel_pop = pixel_consume && prefetch_valid;
    wire [ADDR_WIDTH:0] inflight_words =
        {{(ADDR_WIDTH+1-INFLIGHT_COUNT_W){1'b0}}, inflight_count_mem};
    wire final_return = returned_count_mem == IMAGE_WORDS - 1;
    wire [ADDR_WIDTH+1:0] reserved_words =
        {{(ADDR_WIDTH+2-FIFO_COUNT_W){1'b0}}, fifo_wrusedw} +
        {{1{1'b0}}, inflight_words};
    localparam integer FULL_BURSTS_ONLY = (IMAGE_WORDS % BURST_SIZE) == 0;
    wire [ADDR_WIDTH:0] remaining_words = IMAGE_WORDS - issued_count_mem;
    wire [ADDR_WIDTH:0] next_burst_words = zoom_active ?
        ((FRAME_WIDTH-zoom_row_issued>BURST_SIZE) ? BURST_SIZE : FRAME_WIDTH-zoom_row_issued) :
        FULL_BURSTS_ONLY ? BURST_SIZE :
        ((remaining_words > BURST_SIZE) ? BURST_SIZE : remaining_words);
    wire [ADDR_WIDTH+2:0] reserved_after_burst =
        {{1{1'b0}}, reserved_words} + {{2{1'b0}}, next_burst_words};
    wire inflight_window_ok = (MAX_INFLIGHT_BURSTS == 1) ?
        inflight_empty_mem :
        ((inflight_words + next_burst_words) <=
         (MAX_INFLIGHT_BURSTS * BURST_SIZE));
    wire fifo_reservation_ok =
        reserved_after_burst <= (FIFO_DEPTH - FIFO_MARGIN);
    reg inflight_window_ok_q;
    reg fifo_reservation_ok_q;
    wire start_burst = !DIAG_SYNTH_STREAM && reader_busy && controller_init_done &&
                       !controller_busy && !refresh_active &&
                       !external_write_busy &&
                       fifo_reset_count_mem == 0 && fifo_ready_mem[1] && !burst_active_mem &&
                       burst_gap_count_mem == 0 &&
                       (thumbnail_active || issued_count_mem != IMAGE_WORDS) &&
                       inflight_window_ok_q && (zoom_active ?
                           ((zoom_state==Z_READ || thumbnail_active) && zoom_row_issued<FRAME_WIDTH &&
                            (!thumbnail_active || thumbnail_row_valid)) : fifo_reservation_ok_q);
    wire issue_allowed = !DIAG_SYNTH_STREAM && reader_busy && burst_active_mem;
    // reader_busy 表示“整帧读取事务仍未完成”，不能直接拿来做 SDRAM
    // 仲裁；否则写侧会被锁住整个读帧，读侧又会等待写侧，形成死锁。
    // 只有正在发起突发或仍有返回数据在途时，才真正占用 SDRAM 总线。
    assign reader_bus_busy = burst_active_mem || !inflight_empty_mem;
    wire diag_stream_write = DIAG_SYNTH_STREAM && reader_busy &&
                             fifo_ready_mem[1] && !fifo_reset && !fifo_full;
    wire fifo_write_enable = zoom_active ? zoom_write : DIAG_SYNTH_STREAM ? diag_stream_write :
                             (sdram_rd_valid && !fifo_reset && !fifo_full);
    wire [31:0] fifo_write_data = thumbnail_active ? {thumbnail_rgb,8'd0} :
                                  zoom_active ? {zoom_border ? 24'd0 : zoom_line_q,8'd0} :
                                  (DIAG_RETURN_PATTERN || DIAG_SYNTH_STREAM) ?
                                  {8'h00, diag_rgb_mem} : sdram_rd_data;

    always @* begin
        if (diag_column_mem < 10'd80)
            diag_rgb_mem = 24'hffffff;
        else if (diag_column_mem < 10'd160)
            diag_rgb_mem = 24'hffff00;
        else if (diag_column_mem < 10'd240)
            diag_rgb_mem = 24'h00ffff;
        else if (diag_column_mem < 10'd320)
            diag_rgb_mem = 24'h00ff00;
        else if (diag_column_mem < 10'd400)
            diag_rgb_mem = 24'hff00ff;
        else if (diag_column_mem < 10'd480)
            diag_rgb_mem = 24'hff0000;
        else if (diag_column_mem < 10'd560)
            diag_rgb_mem = 24'h0000ff;
        else
            diag_rgb_mem = 24'h000000;
    end

    function [ADDR_WIDTH-1:0] bank_base;
        input [1:0] index;
        begin
            case (index)
                2'd0: bank_base = {ADDR_WIDTH{1'b0}};
                2'd1: bank_base = IMAGE_WORDS;
                2'd2: bank_base = IMAGE_WORDS * 2;
                default: bank_base = IMAGE_WORDS * 3;
            endcase
        end
    endfunction

    // Delay the request by one pixel clock so media_player_control can commit
    // active_index on the frame_start edge before this module captures it.
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            request_pending_pix <= 1'b0;
            request_toggle_pix <= 1'b0;
            request_index_pix <= 2'd0;
            request_old_index_pix <= 2'd0;
            request_new_words_pix <= IMAGE_WORDS;
            request_zoom_pix <= 1;
            thumbnail_pix<=0;
            frame_armed_pix <= 1'b0;
            consumed_count_pix <= 0;
            underflow <= 1'b0;
            fifo_release_pix <= 2'b00;
        end else begin
            if (fifo_reset_request)
                fifo_release_pix <= 2'b00;
            else
                fifo_release_pix <= {fifo_release_pix[0], 1'b1};

            if (frame_start) begin
                if (frame_armed_pix && consumed_count_pix != IMAGE_WORDS) begin
                    underflow <= 1'b1;
                end
                consumed_count_pix <= 0;
                request_pending_pix <= 1'b1;
            end else if (request_pending_pix) begin
                request_index_pix <= active_index;
                request_old_index_pix <= REGION_READ_ENABLE ? old_index : active_index;
                request_new_words_pix <= REGION_READ_ENABLE ? new_words : IMAGE_WORDS;
                request_zoom_pix <= ZOOM_ENABLE ? zoom_level : 2'd1;
                thumbnail_pix<=THUMBNAIL_ENABLE ? thumbnail_descriptor : 32'd0;
                request_toggle_pix <= ~request_toggle_pix;
                request_pending_pix <= 1'b0;
                frame_armed_pix <= 1'b1;
            end

            if (pixel_consume) begin
                if (prefetch_valid)
                    consumed_count_pix <= consumed_count_pix + 1'b1;
                else if (frame_armed_pix)
                    underflow <= 1'b1;
            end
        end
    end

    always @(posedge mem_clk or negedge mem_rst_n) begin
        if (!mem_rst_n) begin
            request_sync1_mem <= 1'b0;
            request_sync2_mem <= 1'b0;
            request_seen_mem <= 1'b0;
            index_sync1_mem <= 2'd0;
            index_sync2_mem <= 2'd0;
            reload_pending_mem <= 1'b0;
            reload_index_mem <= 2'd0;
            old_index_sync1_mem <= 0; old_index_sync2_mem <= 0; reload_old_index_mem <= 0;
            new_words_sync1_mem <= IMAGE_WORDS; new_words_sync2_mem <= IMAGE_WORDS;
            reload_new_words_mem <= IMAGE_WORDS;
            frame_old_index_mem <= 0; frame_new_words_mem <= IMAGE_WORDS;
            zoom_sync1_mem<=1; zoom_sync2_mem<=1; reload_zoom_mem<=1; frame_zoom_mem<=1;
            thumbnail_sync1<=0; thumbnail_sync2<=0; thumbnail_reload<=0; thumbnail_mem<=0;
            zoom_state<=Z_READ; zoom_x_phase<=0; zoom_y_phase<=0;
            zoom_row_issued<=0; zoom_row_returned<=0;
            zoom_x<=0; zoom_y<=0; zoom_source_x<=0; zoom_row_base<=0;
            burst_base_mem <= 0;
            fifo_reset_count_mem <= 3'd0;
            issued_count_mem <= 0;
            returned_count_mem <= 0;
            inflight_count_mem <= 0;
            inflight_empty_mem <= 1'b1;
            burst_active_mem <= 1'b0;
            burst_issued_mem <= 0;
            burst_target_mem <= 0;
            burst_gap_count_mem <= 0;
            diag_written_count_mem <= 0;
            diag_column_mem <= 0;
            fifo_release_mem <= 2'b00;
            inflight_window_ok_q <= 1'b1;
            fifo_reservation_ok_q <= 1'b1;
            app_rd_en <= 1'b0;
            app_rd_addr <= 0;
            reader_busy <= 1'b0;
            frame_read_done <= 1'b0;
            reader_index <= 2'd0;
            reader_error <= ERR_NONE;
        end else begin
            if (fifo_reset_request)
                fifo_release_mem <= 2'b00;
            else
                fifo_release_mem <= {fifo_release_mem[0], 1'b1};

            request_sync1_mem <= request_toggle_pix;
            request_sync2_mem <= request_sync1_mem;
            index_sync1_mem <= request_index_pix;
            index_sync2_mem <= index_sync1_mem;
            // Bundled frame descriptor: held with request_index_pix until the
            // next video frame; synchronize/capture alongside the same toggle.
            old_index_sync1_mem <= request_old_index_pix;
            old_index_sync2_mem <= old_index_sync1_mem;
            new_words_sync1_mem <= request_new_words_pix;
            new_words_sync2_mem <= new_words_sync1_mem;
            zoom_sync1_mem<=request_zoom_pix; zoom_sync2_mem<=zoom_sync1_mem;
            thumbnail_sync1<={thumbnail_pix[31:24],thumbnail_pix[23],thumbnail_pix[19:16]};
            thumbnail_sync2<=thumbnail_sync1;
            app_rd_en <= 1'b0;
            frame_read_done <= 1'b0;
            inflight_window_ok_q <= inflight_window_ok;
            fifo_reservation_ok_q <= fifo_reservation_ok;

            if (fifo_reset_count_mem != 0)
                fifo_reset_count_mem <= fifo_reset_count_mem - 1'b1;
            if (burst_gap_count_mem != 0)
                burst_gap_count_mem <= burst_gap_count_mem - 1'b1;

            case ({issue_allowed, sdram_rd_valid})
                2'b10: begin
                    inflight_count_mem <= inflight_count_mem + 1'b1;
                    inflight_empty_mem <= 1'b0;
                end
                2'b01: begin
                    if (!inflight_empty_mem) begin
                        inflight_count_mem <= inflight_count_mem - 1'b1;
                        if (inflight_count_mem == 1)
                            inflight_empty_mem <= 1'b1;
                    end
                end
                default: begin
                    inflight_count_mem <= inflight_count_mem;
                    inflight_empty_mem <= inflight_empty_mem;
                end
            endcase

            if (request_sync2_mem != request_seen_mem) begin
                request_seen_mem <= request_sync2_mem;
                reload_pending_mem <= 1'b1;
                reload_index_mem <= index_sync2_mem;
                reload_old_index_mem <= old_index_sync2_mem;
                reload_new_words_mem <= new_words_sync2_mem;
                reload_zoom_mem<=zoom_sync2_mem;
                thumbnail_reload<=thumbnail_sync2;
                if (reader_busy || !inflight_empty_mem) begin
                    reader_busy <= 1'b0;
                    burst_active_mem <= 1'b0;
                    reader_error <= ERR_FRAME_OVERRUN;
                end
            end

            if (reload_pending_mem && inflight_empty_mem) begin
                reload_pending_mem <= 1'b0;
                reader_index <= reload_index_mem;
                frame_old_index_mem <= reload_old_index_mem;
                frame_new_words_mem <= reload_new_words_mem;
                frame_zoom_mem<=reload_zoom_mem;
                thumbnail_mem<=thumbnail_reload;
                zoom_state<=Z_READ; zoom_x_phase<=0; zoom_y_phase<=0;
                zoom_row_issued<=0; zoom_row_returned<=0; zoom_x<=0; zoom_y<=0;
                zoom_source_x<=thumbnail_reload[4] ? 0 : reload_zoom_mem==2 ? FRAME_WIDTH/6 :
                    reload_zoom_mem==3 ? FRAME_WIDTH/4 : 0;
                zoom_row_base<=thumbnail_reload[4] ? 0 : reload_zoom_mem==2 ? (FRAME_HEIGHT/6)*FRAME_WIDTH :
                    reload_zoom_mem==3 ? (FRAME_HEIGHT/4)*FRAME_WIDTH : 0;
                issued_count_mem <= 0;
                returned_count_mem <= 0;
                inflight_count_mem <= 0;
                inflight_empty_mem <= 1'b1;
                burst_active_mem <= 1'b0;
                burst_issued_mem <= 0;
                burst_target_mem <= 0;
                burst_gap_count_mem <= 0;
                diag_written_count_mem <= 0;
                diag_column_mem <= 0;
                fifo_reset_count_mem <= 3'd4;
                reader_busy <= 1'b1;
                reader_error <= ERR_NONE;
            end

            if (start_burst) begin
                // Choose only between bursts; no bank jump inside a burst.
                burst_base_mem <= bank_base(thumbnail_active ? thumbnail_bank :
                    (REGION_READ_ENABLE && issued_count_mem >= frame_new_words_mem) ?
                    frame_old_index_mem : reader_index);
                burst_active_mem <= 1'b1;
                burst_issued_mem <= 0;
                burst_target_mem <= next_burst_words[BURST_COUNT_W-1:0];
            end

            if (issue_allowed) begin
                app_rd_en <= 1'b1;
                app_rd_addr <= burst_base_mem + (zoom_active ?
                    zoom_row_base+zoom_row_issued : issued_count_mem[ADDR_WIDTH-1:0]);
                if(zoom_active) zoom_row_issued<=zoom_row_issued+1'b1;
                issued_count_mem <= issued_count_mem + 1'b1;
                burst_issued_mem <= burst_issued_mem + 1'b1;
                if (burst_issued_mem + 1'b1 == burst_target_mem) begin
                    burst_active_mem <= 1'b0;
                    // At 50 MHz a thumbnail consumes two source words per
                    // 25 MHz output pixel. The thumbnail path therefore has
                    // no throughput margin for the normal inter-burst gap.
                    // Keep the gap for full-frame reads, but run adjacent
                    // bursts for the thumbnail stream.
                    burst_gap_count_mem <= thumbnail_active ? 0 : BURST_GAP_CYCLES;
                end
            end

            if (!DIAG_SYNTH_STREAM && sdram_rd_valid) begin
                returned_count_mem <= returned_count_mem + 1'b1;
                if(zoom_active) begin
                    zoom_row_returned<=zoom_row_returned+1'b1;
                    if(thumbnail_active) begin
                        if(zoom_state==Z_READ && zoom_row_returned==1)
                            zoom_state<=Z_ADDRESS;
                    end else if(zoom_row_returned==FRAME_WIDTH-1) zoom_state<=Z_ADDRESS;
                end else if (fifo_full) begin
                    reader_busy <= 1'b0;
                    burst_active_mem <= 1'b0;
                    reader_error <= ERR_FIFO_OVERFLOW;
                end else if (final_return) begin
                    reader_busy <= 1'b0;
                    burst_active_mem <= 1'b0;
                    frame_read_done <= 1'b1;
                end
            end

            if(zoom_active && reader_busy && !reload_pending_mem &&
               request_sync2_mem==request_seen_mem) begin
                if(thumbnail_active && zoom_state==Z_READ && !thumbnail_row_valid)
                    zoom_state<=Z_ADDRESS;
                // Empty thumbnail rows can emit immediately (no SDRAM delay).
                // Respect the FIFO's internal reset-release pipeline as well.
                if(zoom_state==Z_ADDRESS && zoom_room && !fifo_reset && fifo_ready_mem[1])
                    zoom_state<=Z_PUSH;
                if(zoom_write) begin
                    if(zoom_x==FRAME_WIDTH-1) begin
                        zoom_x<=0; zoom_x_phase<=0;
                        zoom_source_x<=thumbnail_active ? 0 : frame_zoom_mem==2 ? FRAME_WIDTH/6 :
                            frame_zoom_mem==3 ? FRAME_WIDTH/4 : 0;
                        if(zoom_y==FRAME_HEIGHT-1) begin
                            reader_busy<=0; frame_read_done<=1;
                        end else begin
                            zoom_y<=zoom_y+1'b1; zoom_state<=Z_READ;
                            zoom_row_issued<=0; zoom_row_returned<=0;
                            if(thumbnail_active) begin
                                if(zoom_y==FRAME_HEIGHT/2-1) zoom_row_base<=0;
                                else if(zoom_y>=FRAME_HEIGHT/4 && zoom_y<FRAME_HEIGHT*3/4-1)
                                    zoom_row_base<=zoom_row_base+4*FRAME_WIDTH;
                            end else case(frame_zoom_mem)
                                0: if(zoom_y>=FRAME_HEIGHT/4 && zoom_y<FRAME_HEIGHT*3/4-1)
                                    zoom_row_base<=zoom_row_base+2*FRAME_WIDTH;
                                2: if(zoom_y_phase>=1) begin
                                    zoom_y_phase<=zoom_y_phase-1'b1;
                                    zoom_row_base<=zoom_row_base+FRAME_WIDTH;
                                end else zoom_y_phase<=zoom_y_phase+2'd2;
                                3: if(zoom_y[0]) zoom_row_base<=zoom_row_base+FRAME_WIDTH;
                            endcase
                        end
                    end else begin
                        zoom_x<=zoom_x+1'b1;
                        if(thumbnail_active) begin
                            if(zoom_x==FRAME_WIDTH/2-1) begin
                                zoom_state<=Z_READ; zoom_row_issued<=0; zoom_row_returned<=0;
                                zoom_source_x<=0;
                            end else if(zoom_x>=FRAME_WIDTH/4 && zoom_x<FRAME_WIDTH*3/4-1)
                                zoom_source_x<=zoom_source_x+3'd4;
                        end else case(frame_zoom_mem)
                            0: if(zoom_x>=FRAME_WIDTH/4 && zoom_x<FRAME_WIDTH*3/4-1)
                                zoom_source_x<=zoom_source_x+2'd2;
                            2: if(zoom_x_phase>=1) begin
                                zoom_x_phase<=zoom_x_phase-1'b1;
                                zoom_source_x<=zoom_source_x+1'b1;
                            end else zoom_x_phase<=zoom_x_phase+2'd2;
                            3: if(zoom_x[0]) zoom_source_x<=zoom_source_x+1'b1;
                        endcase
                    end
                end
            end

            if (DIAG_SYNTH_STREAM && diag_stream_write) begin
                diag_written_count_mem <= diag_written_count_mem + 1'b1;
                diag_column_mem <= (diag_column_mem == 10'd639) ?
                                   10'd0 : diag_column_mem + 1'b1;
                if (diag_written_count_mem == IMAGE_WORDS - 1) begin
                    reader_busy <= 1'b0;
                    frame_read_done <= 1'b1;
                end
            end else if (!DIAG_SYNTH_STREAM && DIAG_RETURN_PATTERN &&
                         sdram_rd_valid) begin
                diag_column_mem <= (diag_column_mem == 10'd639) ?
                                   10'd0 : diag_column_mem + 1'b1;
            end
        end
    end

    rfifo_32_32_512 #(
        .DATA_WIDTH_W(32), .DATA_WIDTH_R(32),
        .ADDR_WIDTH_W(FIFO_COUNT_W), .ADDR_WIDTH_R(FIFO_COUNT_W),
        .AL_FULL_NUM(FIFO_DEPTH-FIFO_MARGIN), .AL_EMPTY_NUM(2),
        .SHOW_AHEAD_EN(1'b0), .OUTREG_EN("NOREG")
    ) u_read_fifo (
        .rst(fifo_reset), .clkw(mem_clk), .clkr(pix_clk),
        .we(fifo_write_enable), .di(fifo_write_data),
        .re(fifo_raw_read_en), .dout(fifo_raw_data), .valid(fifo_raw_valid),
        .full_flag(fifo_full), .empty_flag(fifo_empty),
        .afull(fifo_almost_full), .aempty(),
        .wrusedw(fifo_wrusedw), .rdusedw()
    );

    fifo_prefetch_2 #(.DATA_WIDTH(32)) u_prefetch (
        .clk(pix_clk), .rst(fifo_reset), .raw_empty(fifo_empty),
        .raw_valid(fifo_raw_valid), .raw_data(fifo_raw_data),
        .raw_read_en(fifo_raw_read_en), .consume(pixel_pop),
        .out_valid(prefetch_valid), .out_empty(), .out_data(prefetch_data)
    );

    // assign pixel_rgb = prefetch_data[23:0];
    // SDRAM 中每个像素按 {R,G,B,8'h00} 存储，低 8 位为填充字节。
    assign pixel_rgb = prefetch_data[31:8];

    assign pixel_valid = prefetch_valid;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            // The memory-domain error remains authoritative for controller faults.
        end else if (underflow) begin
            // underflow is exported separately; the Top may merge it as 8'h63.
        end
    end

endmodule

// The menu descriptor and existing image commit use separate CDC mailboxes.
// Do not acknowledge menu exit until the selected image has actually committed.
// Cancel packets focus the slot containing the original image, so cancel does not
// need an image switch. Feed the result into the reader's frame-latched input.
module thumbnail_exit_gate (
    input wire [31:0] requested, displayed,
    input wire [3:0] committed_image,
    output wire [31:0] descriptor
);
    wire [3:0] target_image=requested[requested[21:20]*4 +:4];
    assign descriptor=displayed[23] && !requested[23] &&
        committed_image!=target_image ? displayed : requested;
endmodule
