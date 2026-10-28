`timescale 1ns/1ps
// PCM: 32 chronological envelope columns, oldest left and newest right.
// Arpeggio: triggered fixed-time waveform, higher pitch gives denser cycles.
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
    input wire arpeggio_mode,
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

    wire scope_mode;
    wire [23:0] scope_rgb;
    wire scope_hit;
    // Same one-clock RAM read as the envelope, then share the output register.
    arpeggio_scope #(.WAVE_X(WAVE_X), .WAVE_Y(WAVE_Y),
        .WAVE_WIDTH(WAVE_WIDTH), .WAVE_HEIGHT(WAVE_HEIGHT)) u_scope (
        .clk(clk), .rst_n(rst_n), .frame_start(frame_start),
        .arpeggio_mode(arpeggio_mode), .sample_valid(pcm_tap_valid), .sample(pcm_tap_left),
        .pixel_x(pixel_x), .x1(x1), .y1(y1), .in_wave(in_wave),
        .mode_frame(scope_mode), .rgb(scope_rgb), .trace_pixel(scope_hit));

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            rgb1<=0; x1<=0; y1<=0; vs1<=1; de1<=0;
            enabled1<=0; valid1<=0; frame1<=0;
            rgb_out<=0; out_vs<=1; out_de<=0; out_x<=0; out_y<=0;
            waveform_active<=0; frame_start_out<=0;
        end else begin
            rgb1<=rgb_in; x1<=pixel_x; y1<=pixel_y; vs1<=vsync; de1<=de;
            enabled1<=enable; valid1<=sample_valid; frame1<=frame_start;
            rgb_out<=scope_mode && in_wave ? scope_rgb : bar_pixel && !scope_mode ? bar_color : rgb1;
            out_vs<=vs1; out_de<=de1; out_x<=x1; out_y<=y1;
            frame_start_out<=frame1;
            waveform_active<=scope_mode ? scope_hit : bar_pixel;
        end
    end
endmodule

// Triggered oscilloscope: 128 line segments, two 48kHz samples per segment.
// The fixed 5.33ms window shows ~2..6 cycles of the image-linked arpeggios.
// Packed adjacent endpoints need only one synchronous RAM read per pixel.
// Capture into the hidden bank; swap only at VS, never during active video.
module arpeggio_scope #(
    parameter integer WAVE_X=64, WAVE_Y=368, WAVE_WIDTH=512, WAVE_HEIGHT=64
)(
    input wire clk, rst_n, frame_start, arpeggio_mode, sample_valid,
    input wire signed [15:0] sample,
    input wire [11:0] pixel_x, x1, y1,
    input wire in_wave,
    output reg mode_frame,
    output wire [23:0] rgb,
    output wire trace_pixel
);
    localparam integer SPACING=WAVE_WIDTH/128;
    localparam integer CENTER=WAVE_HEIGHT/2;
    localparam integer LIMIT=CENTER-4;
    reg [15:0] points[0:255]; /* fehdl force_ram=1, ram_style="bram" */
    reg display_bank, display_valid;
    reg capturing, pending, decimate, previous_negative, mode_q;
    reg [6:0] write_index;
    reg [7:0] previous_y;
    reg frame_signal;
    wire mode_changed=arpeggio_mode!=mode_q;
    wire crossing=previous_negative && !sample[15];
    // Fixed scale: volume changes height, never the horizontal frequency scale.
    wire signed [15:0] scaled=sample >>> 9;
    wire [7:0] sample_y=scaled>LIMIT ? CENTER-LIMIT :
        scaled< -LIMIT ? CENTER+LIMIT : CENTER-scaled;
    wire write_point=arpeggio_mode && !mode_changed && sample_valid && capturing && decimate;
    wire [6:0] read_index=(pixel_x-WAVE_X)/SPACING;
    reg [15:0] endpoints;
    always @(posedge clk) begin
        if(rst_n && write_point)
            points[{!display_bank,write_index}]<={previous_y,sample_y};
        endpoints<=points[{display_bank,read_index}];
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            display_bank<=0; display_valid<=0; mode_frame<=0;
            capturing<=0; pending<=0; decimate<=0; previous_negative<=0;
            write_index<=0; previous_y<=CENTER; frame_signal<=0; mode_q<=0;
        end else begin
            mode_q<=arpeggio_mode;
            if(sample_valid && (sample>256 || sample< -256)) frame_signal<=1;
            if(!arpeggio_mode || mode_changed) begin
                capturing<=0; pending<=0; previous_negative<=0; decimate<=0;
            end else if(sample_valid) begin
                previous_negative<=sample[15];
                if(!capturing && !pending && crossing) begin
                    capturing<=1; write_index<=0; previous_y<=sample_y; decimate<=0;
                end else if(capturing) begin
                    decimate<=!decimate;
                    if(decimate) begin
                        previous_y<=sample_y;
                        if(write_index==127) begin capturing<=0; pending<=1; end
                        else write_index<=write_index+1'b1;
                    end
                end
            end
            if(frame_start) begin
                mode_frame<=arpeggio_mode;
                frame_signal<=sample_valid && (sample>256 || sample< -256);
                if(!arpeggio_mode || mode_changed || !frame_signal) display_valid<=0;
                else if(pending) begin display_bank<=!display_bank; display_valid<=1; end
                if(pending) pending<=0;
            end
        end
    end
    localparam integer FRACTION_BITS=(SPACING>1) ? $clog2(SPACING) : 1;
    wire [FRACTION_BITS-1:0] fraction=(x1-WAVE_X)%SPACING;
    wire signed [9:0] delta=$signed({1'b0,endpoints[7:0]})-$signed({1'b0,endpoints[15:8]});
    wire signed [22:0] interpolation=delta*$signed({1'b0,fraction});
    wire signed [22:0] interpolation_next=interpolation+delta;
    wire signed [23:0] line_y=$signed({1'b0,endpoints[15:8]})+interpolation/SPACING;
    wire signed [23:0] line_next=$signed({1'b0,endpoints[15:8]})+interpolation_next/SPACING;
    wire [11:0] relative_y=y1-WAVE_Y;
    wire [11:0] trace_y=display_valid ? line_y[11:0] : CENTER;
    wire [11:0] trace_next=display_valid ? line_next[11:0] : CENTER;
    wire [11:0] trace_min=trace_y<trace_next ? trace_y : trace_next;
    wire [11:0] trace_max=trace_y>trace_next ? trace_y : trace_next;
    assign trace_pixel=in_wave && relative_y>=trace_min && relative_y<=trace_max+1'b1;
    assign rgb=trace_pixel ? 24'h40efb0 : relative_y==CENTER ? 24'h465864 : 24'h101c28;
endmodule
