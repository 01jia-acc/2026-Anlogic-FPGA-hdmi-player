`timescale 1ns/1ps
// Adapted from lesson 11: 32 decimated samples, frame-swapped banks and peak bar.
// Tap the HDMI bridge AFTER gain/source selection, in the pixel clock domain.
// Two registered video stages keep the synchronous sample RAM aligned with RGB.
module promax_audio_waveform #(
    parameter integer WAVE_X=64,
    parameter integer WAVE_Y=400,
    parameter integer WAVE_WIDTH=512,
    parameter integer WAVE_HEIGHT=64,
    parameter integer DECIMATE=12
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
    localparam integer HALF_HEIGHT=WAVE_HEIGHT/2;
    localparam [11:0] CENTER_Y=WAVE_Y+HALF_HEIGHT;
    localparam integer DEC_BITS=(DECIMATE>1) ? $clog2(DECIMATE) : 1;

    reg vs_q;
    wire frame_start=vs_q && !vsync;
    reg write_bank, display_bank, display_valid;
    reg [5:0] capture_count;
    reg [DEC_BITS-1:0] decimate_count;
    reg [15:0] capture_peak, display_peak;
    // Unsigned magnitude represents -32768 correctly as +32768.
    wire [15:0] sample_abs=pcm_tap_left[15] ? (~pcm_tap_left+16'd1) : pcm_tap_left;

    // Single write/read ports without array reset preserve block RAM inference.
    reg [7:0] samples [0:63]; /* fehdl force_ram=1, ram_style="bram" */
    wire sample_write=rst_n && !frame_start && pcm_tap_valid &&
        capture_count<SAMPLE_COUNT && decimate_count==0;
    wire [11:0] relative_x=pixel_x-WAVE_X;
    wire [4:0] sample_index=relative_x/SAMPLE_SPACING;
    reg [7:0] sample_q;
    always @(posedge clk) begin
        if(sample_write) samples[{write_bank,capture_count[4:0]}]<=pcm_tap_left[15:8];
        sample_q<=samples[{display_bank,sample_index}];
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            vs_q<=1; write_bank<=0; display_bank<=1; display_valid<=0;
            capture_count<=0; decimate_count<=0; capture_peak<=0; display_peak<=0;
        end else begin
            vs_q<=vsync;
            if(frame_start) begin
                if(capture_count==SAMPLE_COUNT) begin
                    display_bank<=write_bank;
                    write_bank<=~write_bank;
                    display_valid<=1;
                    display_peak<=capture_peak;
                end else begin
                    // An interrupted stream must not leave a stale waveform/meter.
                    display_valid<=0;
                    display_peak<=0;
                end
                capture_count<=0; decimate_count<=0; capture_peak<=0;
            end else if(pcm_tap_valid) begin
                // Peak uses ALL samples in the frame, including decimated-away
                // transients and samples after the 32-point window is complete.
                if(sample_abs>capture_peak) capture_peak<=sample_abs;
                if(capture_count<SAMPLE_COUNT) begin
                    if(decimate_count==0) begin
                        capture_count<=capture_count+1'b1;
                        decimate_count<=DECIMATE-1;
                    end else decimate_count<=decimate_count-1'b1;
                end
            end
        end
    end

    reg [23:0] rgb1;
    reg [11:0] x1,y1;
    reg vs1,de1,enabled1,valid1,frame1;
    reg [15:0] peak1;
    wire [7:0] magnitude=sample_q[7] ? (~sample_q+8'd1) : sample_q;
    // Constant geometry: default 64px height synthesizes as shifts, no DSP.
    wire [19:0] amplitude_product=magnitude*HALF_HEIGHT;
    wire [11:0] amplitude_raw=amplitude_product>>7;
    wire [11:0] amplitude=amplitude_raw>=HALF_HEIGHT ? HALF_HEIGHT-1 : amplitude_raw;
    wire [11:0] sample_y=sample_q[7] ? CENTER_Y+amplitude : CENTER_Y-amplitude;
    wire [27:0] peak_product=peak1*HALF_HEIGHT;
    wire [11:0] peak_height=peak_product>>15;
    wire in_wave=enabled1 && de1 && x1>=WAVE_X && x1<WAVE_X+WAVE_WIDTH &&
        y1>=WAVE_Y && y1<WAVE_Y+WAVE_HEIGHT;
    wire wave_pixel=in_wave && valid1 && y1+12'd1>=sample_y && y1<=sample_y+12'd1;
    wire axis_pixel=in_wave && y1==CENTER_Y;
    wire peak_pixel=enabled1 && de1 && valid1 &&
        x1>=WAVE_X+WAVE_WIDTH+8 && x1<WAVE_X+WAVE_WIDTH+16 &&
        y1>=WAVE_Y+WAVE_HEIGHT-peak_height && y1<WAVE_Y+WAVE_HEIGHT;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            rgb1<=0; x1<=0; y1<=0; vs1<=1; de1<=0;
            enabled1<=0; valid1<=0; peak1<=0; frame1<=0;
            rgb_out<=0; out_vs<=1; out_de<=0; out_x<=0; out_y<=0;
            waveform_active<=0; frame_start_out<=0;
        end else begin
            rgb1<=rgb_in; x1<=pixel_x; y1<=pixel_y; vs1<=vsync; de1<=de;
            enabled1<=enable; valid1<=display_valid; peak1<=display_peak; frame1<=frame_start;
            rgb_out<=wave_pixel ? 24'h20ff90 : peak_pixel ? 24'hffb020 :
                axis_pixel ? 24'h406060 : rgb1;
            out_vs<=vs1; out_de<=de1; out_x<=x1; out_y<=y1;
            frame_start_out<=frame1;
            waveform_active<=wave_pixel || axis_pixel || peak_pixel;
        end
    end
endmodule
