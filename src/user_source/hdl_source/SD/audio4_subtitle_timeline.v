`timescale 1ns/1ps
// Timeline advances only when a real PCM sample is accepted by the HDMI bridge.
// Underflow and arpeggio mode pause it; SD prefetch/EOF never advance it.
module audio4_subtitle_timeline (
    input wire clk, rst, frame_start, hide,
    input wire selected, show, commit_toggle, sample_accept,
    input wire [5:0] character_position,
    output wire [7:0] glyph,
    output reg visible,
    output reg [5:0] visible_length
);
    reg epoch;
    reg [21:0] sample_position;
    reg [2:0] cue, visible_cue;
    wire [21:0] start_sample, end_sample, total_samples;
    wire [5:0] cue_length;
    // Cue metadata and character ROM share one data instance. Metadata is
    // selected with the live cue, text lookup uses the frame-latched cue base.
    audio4_subtitle_data data(.clk(clk), .cue(cue), .text_cue(visible_cue), .position(character_position),
        .glyph(glyph), .start_sample(start_sample), .end_sample(end_sample),
        .length(cue_length), .total_samples(total_samples));
    wire in_cue=sample_position>=start_sample && sample_position<end_sample;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            epoch<=0; sample_position<=0; cue<=0;
            visible_cue<=0; visible<=0; visible_length<=0;
        end else begin
            epoch<=commit_toggle;
            if(!selected || epoch!=commit_toggle) begin
                sample_position<=0; cue<=0;
            end else if(sample_accept) begin
                if(sample_position==total_samples-1'b1) begin sample_position<=0; cue<=0; end
                else begin
                    sample_position<=sample_position+1'b1;
                    if(sample_position>=end_sample && cue!=7) cue<=cue+1'b1;
                end
            end
            if(frame_start) begin
                visible<=selected && show && !hide && epoch==commit_toggle && in_cue;
                visible_cue<=cue;
                visible_length<=cue_length;
            end
        end
    end
endmodule
