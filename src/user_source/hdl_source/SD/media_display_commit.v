`timescale 1ns/1ps

// Transfer a coherent {valid, image, bank} snapshot, then select video and audio
// together at the video read-request boundary. Background loads do not commit.
module media_display_commit (
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
    output wire display_ack_valid_src,
    output wire [1:0] display_ack_bank_src,
    output wire [3:0] display_ack_image_id_src,
    output reg audio_commit_toggle
);
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

    // Return the actual pixel-domain selection, coherently, before cache reuse.
    wire ack_ready;
    wire [95:0] ack_snapshot;
    subtitle_frame_commit u_ack_snapshot (
        .src_clk(pixel_clk), .src_rst_n(~rst),
        .src_commit(ack_ready), .src_ready(ack_ready),
        .src_text({90'd0,display_image_id_pix,display_index_pix}), .src_length(6'd0),
        .src_enable(display_valid_pix),
        .dst_clk(src_clk), .dst_rst_n(~rst), .frame_start(1'b1),
        .dst_text(ack_snapshot), .dst_length(), .dst_enable(display_ack_valid_src)
    );
    assign display_ack_bank_src=ack_snapshot[1:0];
    assign display_ack_image_id_src=ack_snapshot[5:2];

    // media_frame_reader captures the committed index one pixel clock later.
    assign reader_frame_start = frame_request && snapshot_valid;
    always @(posedge pixel_clk or posedge rst) begin
        if (rst) begin
            display_valid_pix <= 1'b0;
            display_index_pix <= 2'd0;
            display_image_id_pix <= 4'd0;
            audio_commit_toggle <= 1'b0;
        end else if (frame_request) begin
            display_valid_pix <= snapshot_valid;
            if (snapshot_valid) begin
                display_index_pix <= snapshot[1:0];
                display_image_id_pix <= snapshot[5:2];
                if (!display_valid_pix || display_image_id_pix != snapshot[5:2])
                    audio_commit_toggle <= ~audio_commit_toggle;
            end
        end
    end
endmodule
