`timescale 1ns/1ps

// Lesson pcm_media_tone adapted to the vendor core's unthrottled PCM input.
// Timing comes from the bridge's 48 kHz enable, entirely in the pixel domain.
module pcm_media_tone #(
    parameter integer BLIP_SAMPLES = 4800,
    parameter integer NOTE_SAMPLES = 12000
) (
    input  wire              clk,
    input  wire              rst,
    input  wire              enable,
    input  wire              sample_tick,
    input  wire [1:0]        media_id,
    output wire signed [15:0] sample,
    output wire              blip_active
);
    localparam integer BLIP_WIDTH = (BLIP_SAMPLES < 2) ? 1 : $clog2(BLIP_SAMPLES + 1);
    localparam integer NOTE_WIDTH = (NOTE_SAMPLES < 2) ? 1 : $clog2(NOTE_SAMPLES);
    localparam [19:0] BLIP_INC = 20'd32768; // 1500 Hz at 48 kHz

    reg [19:0] phase;
    reg [1:0] media_id_q;
    reg enable_q;
    reg [BLIP_WIDTH-1:0] blip_countdown;
    reg [2:0] note_index;
    reg [NOTE_WIDTH-1:0] note_sample_count;

    wire restart = enable && (!enable_q || media_id != media_id_q);
    assign blip_active = enable && (restart || blip_countdown != 0);

    // round(frequency * 2**20 / 48000), using equal-tempered note frequencies.
    function [19:0] melody_inc;
        input [1:0] melody;
        input [2:0] note;
        begin
            case ({melody, note})
                // Image 0: C major, C5 E5 G5 C6 G5 E5.
                5'b00_000: melody_inc = 20'd11431;
                5'b00_001: melody_inc = 20'd14402;
                5'b00_010: melody_inc = 20'd17127;
                5'b00_011: melody_inc = 20'd22861;
                5'b00_100: melody_inc = 20'd17127;
                5'b00_101: melody_inc = 20'd14402;
                // Image 1: A minor, A4 C5 E5 A5 E5 C5.
                5'b01_000: melody_inc = 20'd9612;
                5'b01_001: melody_inc = 20'd11431;
                5'b01_010: melody_inc = 20'd14402;
                5'b01_011: melody_inc = 20'd19224;
                5'b01_100: melody_inc = 20'd14402;
                5'b01_101: melody_inc = 20'd11431;
                // Image 2: pentatonic, C5 D5 E5 G5 A5 C6.
                5'b10_000: melody_inc = 20'd11431;
                5'b10_001: melody_inc = 20'd12830;
                5'b10_010: melody_inc = 20'd14402;
                5'b10_011: melody_inc = 20'd17127;
                5'b10_100: melody_inc = 20'd19224;
                5'b10_101: melody_inc = 20'd22861;
                // Image 3: low/high swing, C5 G4 E5 C5 G4 E5.
                5'b11_000: melody_inc = 20'd11431;
                5'b11_001: melody_inc = 20'd8563;
                5'b11_010: melody_inc = 20'd14402;
                5'b11_011: melody_inc = 20'd11431;
                5'b11_100: melody_inc = 20'd8563;
                5'b11_101: melody_inc = 20'd14402;
                default: melody_inc = 20'd11431;
            endcase
        end
    endfunction

    wire [19:0] phase_inc = blip_active ? BLIP_INC : melody_inc(media_id, note_index);

    function signed [15:0] sine_sample;
        input [3:0] index;
        begin
            case (index)
                4'd0:  sine_sample = 16'sd0;
                4'd1:  sine_sample = 16'sd4592;
                4'd2:  sine_sample = 16'sd8485;
                4'd3:  sine_sample = 16'sd11087;
                4'd4:  sine_sample = 16'sd12000;
                4'd5:  sine_sample = 16'sd11087;
                4'd6:  sine_sample = 16'sd8485;
                4'd7:  sine_sample = 16'sd4592;
                4'd8:  sine_sample = 16'sd0;
                4'd9:  sine_sample = -16'sd4592;
                4'd10: sine_sample = -16'sd8485;
                4'd11: sine_sample = -16'sd11087;
                4'd12: sine_sample = -16'sd12000;
                4'd13: sine_sample = -16'sd11087;
                4'd14: sine_sample = -16'sd8485;
                default: sine_sample = -16'sd4592;
            endcase
        end
    endfunction

    assign sample = sine_sample(phase[19:16]);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            phase <= 20'd0;
            media_id_q <= 2'd0;
            enable_q <= 1'b0;
            blip_countdown <= 0;
            note_index <= 3'd0;
            note_sample_count <= 0;
        end else begin
            media_id_q <= media_id;
            enable_q <= enable;
            if (!enable) begin
                phase <= 20'd0;
                blip_countdown <= 0;
                note_index <= 3'd0;
                note_sample_count <= 0;
            end else if (restart) begin
                // Restart only the melody timer; keep DDS phase continuous.
                blip_countdown <= BLIP_SAMPLES;
                note_index <= 3'd0;
                note_sample_count <= 0;
                if (sample_tick) begin
                    phase <= phase + BLIP_INC;
                    blip_countdown <= BLIP_SAMPLES - 1;
                end
            end else if (sample_tick) begin
                phase <= phase + phase_inc;
                if (blip_countdown != 0) begin
                    blip_countdown <= blip_countdown - 1'b1;
                end else if (note_sample_count == NOTE_SAMPLES - 1) begin
                    note_sample_count <= 0;
                    note_index <= (note_index == 3'd5) ? 3'd0 : note_index + 1'b1;
                end else begin
                    note_sample_count <= note_sample_count + 1'b1;
                end
            end
        end
    end
endmodule
