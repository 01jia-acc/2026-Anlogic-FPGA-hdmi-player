`timescale 1ns/1ps
// 32 chronological envelope columns: oldest left, newest right.
// Tap the HDMI bridge AFTER gain/source selection, in the pixel clock domain.
// Two registered video stages keep the synchronous sample RAM aligned with RGB.
module promax_audio_waveform #(
    parameter integer WAVE_X=64,
    parameter integer WAVE_Y=400,
    parameter integer WAVE_WIDTH=512,
    parameter integer WAVE_HEIGHT=64,
    parameter integer ATTACK_SHIFT=8,
    parameter integer RELEASE_SHIFT=11
)(
    input wire clk, rst_n,
    input wire vsync, de,
    input wire [23:0] rgb_in,
    input wire [11:0] pixel_x, pixel_y,
    input wire enable,
    input wire pcm_tap_valid,
    input wire signed [15:0] pcm_tap_left,
    output reg [23:0] rgb_out,
    output reg out_vs, out_de,
    output reg [11:0] out_x, out_y,
    output reg frame_start_out,
    output reg waveform_active
);
    localparam integer SAMPLE_COUNT=32;
    localparam integer SAMPLE_SPACING=WAVE_WIDTH/SAMPLE_COUNT;

    reg vs_q;
    wire frame_start=vs_q && !vsync;
    // Append only during vertical blanking so every visible frame is stable.
    // At ~60 Hz, one column is ~16.7 ms and 32 columns cover ~0.53 seconds.
    reg [4:0] history_head;
    reg [5:0] history_count;
    reg frame_has_audio;
    reg [15:0] envelope;
    // Unsigned magnitude represents -32768 correctly as +32768.
    wire [15:0] sample_abs=pcm_tap_left[15] ? (~pcm_tap_left+16'd1) : pcm_tap_left;
    // Every actual HDMI sample updates the envelope between history commits.
    // Rise ~5.3ms, fall ~42.7ms at 48kHz. Round UP the step so quiet audio
    // eventually reaches zero rather than sticking at a small nonzero value.
    wire rising=sample_abs>envelope;
    wire [15:0] difference=rising ? sample_abs-envelope : envelope-sample_abs;
    wire [16:0] rise_rounded={1'b0,difference}+((1<<ATTACK_SHIFT)-1);
    wire [16:0] fall_rounded={1'b0,difference}+((1<<RELEASE_SHIFT)-1);
    wire [15:0] rise_step=rise_rounded>>ATTACK_SHIFT;
    wire [15:0] fall_step=fall_rounded>>RELEASE_SHIFT;
    wire [15:0] next_envelope=rising ? envelope+rise_step : envelope-fall_step;

    // Single write/read ports without array reset preserve block RAM inference.
    reg [7:0] samples [0:31]; /* fehdl force_ram=1, ram_style="bram" */
    // Include an audio sample coincident with VS; never drop a boundary sample.
    wire [7:0] history_sample=pcm_tap_valid ? next_envelope[15:8] :
        frame_has_audio ? envelope[15:8] : 8'd0;
    wire [11:0] relative_x=pixel_x-WAVE_X;
    wire [4:0] sample_index=relative_x/SAMPLE_SPACING;
    wire [4:0] history_address=history_head+sample_index;
    // Unwritten history stays on the left and displays only the baseline.
    wire sample_valid={1'b0,sample_index}>=(6'd32-history_count);
    reg [7:0] sample_q;
    always @(posedge clk) begin
        if(rst_n && frame_start) samples[history_head]<=history_sample;
        sample_q<=samples[history_address];
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            vs_q<=1; history_head<=0; history_count<=0;
            frame_has_audio<=0; envelope<=0;
        end else begin
            vs_q<=vsync;
            if(pcm_tap_valid) envelope<=next_envelope;
            else if(frame_start && !frame_has_audio) envelope<=0;
            if(frame_start) begin
                history_head<=history_head+1'b1;
                if(history_count<SAMPLE_COUNT) history_count<=history_count+1'b1;
                frame_has_audio<=0;
            end else if(pcm_tap_valid) frame_has_audio<=1;
        end
    end

    reg [23:0] rgb1;
    reg [11:0] x1,y1;
    reg vs1,de1,enabled1,valid1,frame1;
    wire [11:0] column_x=(x1-WAVE_X)%SAMPLE_SPACING;
    wire [11:0] from_bottom=WAVE_Y+WAVE_HEIGHT-1-y1;
    // One quiet baseline block, up to five blocks. 4px lit + 4px gap.
    // At the default 16px column pitch, blocks are 12px wide with 4px gaps.
    // Compressed thresholds keep ordinary speech/arpeggios visible without
    // normalizing quiet files to full height: 768 / 2304 / 6144 in PCM units.
    // These thresholds affect visualization only, never the HDMI audio gain.
    wire [2:0] blocks=!valid1 ? 3'd1 : sample_q>=48 ? 3'd5 : sample_q>=24 ? 3'd4 :
        sample_q>=9 ? 3'd3 : sample_q>=3 ? 3'd2 : 3'd1;
    wire in_wave=enabled1 && de1 && x1>=WAVE_X && x1<WAVE_X+WAVE_WIDTH &&
        y1>=WAVE_Y && y1<WAVE_Y+WAVE_HEIGHT;
    wire bar_pixel=in_wave && column_x>=2 && column_x<SAMPLE_SPACING-2 &&
        from_bottom[11:3]<blocks && from_bottom[2:0]<4;
    wire [23:0] bar_color=from_bottom[2:0]==3 ? 24'hb8f438 : 24'h20ee70;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            rgb1<=0; x1<=0; y1<=0; vs1<=1; de1<=0;
            enabled1<=0; valid1<=0; frame1<=0;
            rgb_out<=0; out_vs<=1; out_de<=0; out_x<=0; out_y<=0;
            waveform_active<=0; frame_start_out<=0;
        end else begin
            rgb1<=rgb_in; x1<=pixel_x; y1<=pixel_y; vs1<=vsync; de1<=de;
            enabled1<=enable; valid1<=sample_valid; frame1<=frame_start;
            rgb_out<=bar_pixel ? bar_color : rgb1;
            out_vs<=vs1; out_de<=de1; out_x<=x1; out_y<=y1;
            frame_start_out<=frame1;
            waveform_active<=bar_pixel;
        end
    end
endmodule
