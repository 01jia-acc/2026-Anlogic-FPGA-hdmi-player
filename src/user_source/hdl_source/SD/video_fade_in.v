`timescale 1ns/1ps

// Single-stream transition for the current four-bank player.
// A display commit is already synchronized into video_clk by
// media_display_commit, so this module only needs a pixel-domain event.
// The new frame starts black and ramps to full brightness at frame boundaries.
// The current reader supplies one SDRAM frame stream at a time, so this module
// intentionally does not pretend to implement a two-frame wipe/crossfade.
module video_fade_in #(
    parameter integer FADE_FRAMES = 16
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       frame_start,
    input  wire       transition_start,
    input  wire       player_active,
    output wire [7:0] alpha,
    output wire       active
);
    localparam integer CNT_W = (FADE_FRAMES <= 1) ? 1 : $clog2(FADE_FRAMES + 1);
    reg [CNT_W-1:0] ramp;
    reg             running;

    assign active = running;
    assign alpha = !player_active ? 8'd0 :
                   !running ? 8'd255 :
                   (ramp >= FADE_FRAMES) ? 8'd255 :
                   (ramp * (256 / FADE_FRAMES));

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ramp    <= FADE_FRAMES[CNT_W-1:0];
            running <= 1'b0;
        end else begin
            if (transition_start && player_active) begin
                ramp    <= {CNT_W{1'b0}};
                running <= 1'b1;
            end else if (running && frame_start) begin
                if (ramp >= FADE_FRAMES - 1) begin
                    ramp    <= FADE_FRAMES[CNT_W-1:0];
                    running <= 1'b0;
                end else begin
                    ramp <= ramp + 1'b1;
                end
            end
        end
    end
endmodule
