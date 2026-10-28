`timescale 1ns/1ps
// Normal playback follows actual video commits. PCM slideshow locks to the
// first logical image, so changing pictures cannot restart its soundtrack.
module media_pcm_selection (
    input wire pixel_clk, rst,
    input wire pcm_mode_sd, auto_play_sd,
    input wire display_valid_pix, display_commit_toggle,
    input wire [3:0] display_image_id_pix,
    output reg [3:0] pcm_image_id_pix,
    output reg pcm_commit_toggle
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] pcm_sync, auto_sync;
    reg selected, locked_q, display_seen;
    wire locked=pcm_sync[1] && auto_sync[1];
    always @(posedge pixel_clk or posedge rst) begin
        if(rst) begin
            pcm_sync<=0; auto_sync<=0;
            selected<=0; locked_q<=0; display_seen<=0;
            pcm_image_id_pix<=0; pcm_commit_toggle<=0;
        end else begin
            pcm_sync<={pcm_sync[0],pcm_mode_sd};
            auto_sync<={auto_sync[0],auto_play_sd};
            display_seen<=display_commit_toggle;
            locked_q<=locked;
            if(!display_valid_pix) selected<=0;
            else if(!selected || locked!=locked_q ||
                    (!locked && display_commit_toggle!=display_seen)) begin
                selected<=1;
                // Logical zero maps to the first scanned PIC, even with gaps.
                pcm_image_id_pix<=locked ? 4'd0 : display_image_id_pix;
                pcm_commit_toggle<=!pcm_commit_toggle;
            end
        end
    end
endmodule
