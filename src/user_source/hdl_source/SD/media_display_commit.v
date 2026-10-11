`timescale 1ns/1ps

// Transfer a coherent {valid, bank} snapshot, then select video and audio
// together at the video read-request boundary. Background loads do not commit.
module media_display_commit (
    input wire src_clk,
    input wire rst,
    input wire display_valid_src,
    input wire [1:0] display_index_src,
    input wire pixel_clk,
    input wire frame_request,
    output wire reader_frame_start,
    output reg display_valid_pix,
    output reg [1:0] display_index_pix,
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
        .src_text({94'd0, display_index_src}), .src_length(6'd0),
        .src_enable(display_valid_src),
        .dst_clk(pixel_clk), .dst_rst_n(~rst), .frame_start(1'b1),
        .dst_text(snapshot), .dst_length(), .dst_enable(snapshot_valid)
    );

    // media_frame_reader captures the committed index one pixel clock later.
    assign reader_frame_start = frame_request && snapshot_valid;
    always @(posedge pixel_clk or posedge rst) begin
        if (rst) begin
            display_valid_pix <= 1'b0;
            display_index_pix <= 2'd0;
            audio_commit_toggle <= 1'b0;
        end else if (frame_request) begin
            display_valid_pix <= snapshot_valid;
            if (snapshot_valid) begin
                display_index_pix <= snapshot[1:0];
                if (!display_valid_pix || display_index_pix != snapshot[1:0])
                    audio_commit_toggle <= ~audio_commit_toggle;
            end
        end
    end
endmodule
