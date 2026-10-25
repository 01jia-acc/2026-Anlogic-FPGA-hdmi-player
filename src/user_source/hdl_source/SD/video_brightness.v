`timescale 1ns/1ps
// Lesson 13: frame-held RGB gain, folded into the existing fade coefficient.
// Input level is already in the pixel domain via a bundled-data mailbox.
// No RGB/timing registers: the existing OSD/video pipeline latency is unchanged.
module video_brightness (
    input wire clk, rst_n, frame_start,
    input wire [4:0] level,
    input wire [7:0] fade_alpha, // 255=steady picture, 0=black transition
    input wire [23:0] rgb_in,
    output wire [23:0] rgb_out
);
    reg [8:0] gain_q;
    reg [8:0] gain_next;
    // Case LUT is synthesis-safe in TD; explicit ninth bit preserves 256.
    always @* begin
        case(level)
            0:gain_next=0;     1:gain_next=13;    2:gain_next=26;
            3:gain_next=38;    4:gain_next=51;    5:gain_next=64;
            6:gain_next=77;    7:gain_next=90;    8:gain_next=102;
            9:gain_next=115;  10:gain_next=128;  11:gain_next=141;
            12:gain_next=154; 13:gain_next=166;  14:gain_next=179;
            15:gain_next=192; 16:gain_next=205;  17:gain_next=218;
            18:gain_next=230; 19:gain_next=243;
            default:gain_next=256;
        endcase
    end
    always @(posedge clk or negedge rst_n)
        if(!rst_n) gain_q<=9'd256;
        else if(frame_start) gain_q<=gain_next;

    // Rounded alpha folding; only one RGB multiply per channel even in fade mode.
    wire [16:0] alpha_product=fade_alpha * gain_q + 17'd128;
    wire [7:0] effective_alpha=alpha_product[15:8];
    // Retain the original fade's ceil rounding. At full brightness alpha is
    // unchanged, and ceil(channel*255/256)==channel for all 256 channel values.
    // At zero brightness all three channels are exactly zero.
    wire [15:0] r_mix=rgb_in[23:16]*effective_alpha + 16'd255;
    wire [15:0] g_mix=rgb_in[15:8] *effective_alpha + 16'd255;
    wire [15:0] b_mix=rgb_in[7:0]  *effective_alpha + 16'd255;
    assign rgb_out={r_mix[15:8],g_mix[15:8],b_mix[15:8]};
endmodule
