`timescale 1ns/1ps

// Transfer a coherent {valid, image, bank} snapshot, then select video and audio
// together at the video read-request boundary. Background loads do not commit.
module media_display_commit #(
    parameter integer ADDR_WIDTH = 21,
    parameter integer IMAGE_WORDS = 307200,
    parameter integer FRAME_WIDTH = 640,
    // 0 preserves the original hard commit. Nonzero: top-to-bottom wipe.
    parameter integer WIPE_FRAMES = 0
) (
    input wire src_clk,
    input wire rst,
    input wire display_valid_src,
    input wire [1:0] display_index_src,
    input wire [3:0] display_image_id_src,
    input wire pixel_clk,
    input wire frame_request,
    output wire reader_frame_start,
    output reg display_valid_pix,
    output reg [1:0] display_index_pix,
    output reg [3:0] display_image_id_pix,
    output reg display_changed_pix,
    output wire display_ack_valid_src,
    output wire [1:0] display_ack_bank_src,
    output wire [3:0] display_ack_image_id_src,
    output reg audio_commit_toggle,
    output reg [1:0] reader_old_index_pix,
    output reg [ADDR_WIDTH:0] reader_new_words_pix,
    output reg wipe_active_pix
);
    localparam integer STEP_W = (WIPE_FRAMES < 2) ? 1 : $clog2(WIPE_FRAMES + 1);
    localparam integer FRAME_HEIGHT = IMAGE_WORDS / FRAME_WIDTH;
    localparam integer SAFE_FRAMES = WIPE_FRAMES > 0 ? WIPE_FRAMES : 1;
    reg [STEP_W-1:0] wipe_step;
    reg [3:0] old_image_id_pix;
    // Select complete rows. For 640x480 / 16 frames: 30 rows = 19200 words.
    // The reader requires each boundary to align to its SDRAM burst size.
    wire [ADDR_WIDTH:0] next_new_words =
        ((FRAME_HEIGHT * (wipe_step + 1'b1)) / SAFE_FRAMES) * FRAME_WIDTH;
    // synthesis translate_off
    initial begin
        if (FRAME_WIDTH < 1 || IMAGE_WORDS % FRAME_WIDTH != 0 ||
            WIPE_FRAMES < 0 || (WIPE_FRAMES > 0 && FRAME_HEIGHT % SAFE_FRAMES != 0))
            $fatal(1, "Wipe requires whole rows and a frame count dividing image height");
    end
    // synthesis translate_on
    wire mailbox_ready;
    wire [95:0] snapshot;
    wire snapshot_valid;

    // Hold the bank and validity together until the receiver acknowledges.
    // Deliver snapshots continuously; only the block below changes the frame.
    subtitle_frame_commit u_snapshot (
        .src_clk(src_clk), .src_rst_n(~rst),
        .src_commit(mailbox_ready), .src_ready(mailbox_ready),
        .src_text({90'd0, display_image_id_src, display_index_src}), .src_length(6'd0),
        .src_enable(display_valid_src),
        .dst_clk(pixel_clk), .dst_rst_n(~rst), .frame_start(1'b1),
        .dst_text(snapshot), .dst_length(), .dst_enable(snapshot_valid)
    );

    // Keep ACK on the old bank throughout mixed frames. The scheduler already
    // protects both its requested bank and this ACK bank, and waits for a match
    // before navigating/prefetching again. Never drop ack_valid during a wipe.
    // Release at the request for the first all-new frame: the preceding mixed
    // frame has been consumed and no more old-bank commands will be issued.
    wire ack_ready;
    wire [95:0] ack_snapshot;
    subtitle_frame_commit u_ack_snapshot (
        .src_clk(pixel_clk), .src_rst_n(~rst),
        .src_commit(ack_ready), .src_ready(ack_ready),
        .src_text({90'd0,
            (wipe_active_pix ? old_image_id_pix : display_image_id_pix),
            (wipe_active_pix ? reader_old_index_pix : display_index_pix)}), .src_length(6'd0),
        .src_enable(display_valid_pix),
        .dst_clk(src_clk), .dst_rst_n(~rst), .frame_start(1'b1),
        .dst_text(ack_snapshot), .dst_length(), .dst_enable(display_ack_valid_src)
    );
    assign display_ack_bank_src=ack_snapshot[1:0];
    assign display_ack_image_id_src=ack_snapshot[5:2];

    // media_frame_reader captures the committed index one pixel clock later.
    assign reader_frame_start = frame_request && (snapshot_valid || wipe_active_pix);
    always @(posedge pixel_clk or posedge rst) begin
        if (rst) begin
            display_valid_pix <= 1'b0;
            display_index_pix <= 2'd0;
            display_image_id_pix <= 4'd0;
            display_changed_pix <= 1'b0;
            audio_commit_toggle <= 1'b0;
            reader_old_index_pix <= 2'd0;
            old_image_id_pix <= 4'd0;
            reader_new_words_pix <= IMAGE_WORDS;
            wipe_active_pix <= 1'b0;
            wipe_step <= 0;
        end else if (frame_request) begin
            display_changed_pix <= 1'b0;
            if (wipe_active_pix) begin
                // Freeze both images even if another snapshot arrives early.
                reader_new_words_pix <= next_new_words;
                wipe_step <= wipe_step + 1'b1;
                if (wipe_step >= WIPE_FRAMES - 1) begin
                    reader_new_words_pix <= IMAGE_WORDS;
                    wipe_active_pix <= 1'b0;
                end
            end else begin
                display_valid_pix <= snapshot_valid;
                if (snapshot_valid) begin
                    display_index_pix <= snapshot[1:0];
                    display_image_id_pix <= snapshot[5:2];
                    reader_new_words_pix <= IMAGE_WORDS;
                    if (!display_valid_pix || display_image_id_pix != snapshot[5:2])
                        audio_commit_toggle <= ~audio_commit_toggle;
                    if (display_valid_pix && display_image_id_pix != snapshot[5:2])
                        display_changed_pix <= 1'b1;
                    if (WIPE_FRAMES > 1 && display_valid_pix &&
                        display_image_id_pix != snapshot[5:2]) begin
                        reader_old_index_pix <= display_index_pix;
                        old_image_id_pix <= display_image_id_pix;
                        reader_new_words_pix <= (FRAME_HEIGHT / SAFE_FRAMES) * FRAME_WIDTH;
                        wipe_step <= 1;
                        wipe_active_pix <= 1'b1;
                    end
                end
            end
        end else begin
            display_changed_pix <= 1'b0;
        end
    end
endmodule
