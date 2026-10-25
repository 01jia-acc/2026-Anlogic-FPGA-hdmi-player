`timescale 1ns/1ps
// Frame-held midpoint contrast/clipping followed by brightness/fade gain.
// Both levels arrive in the pixel domain through a bundled-data mailbox.
// No RGB/timing registers: the existing OSD/video pipeline latency is unchanged.
module video_brightness (
    input wire clk, rst_n, frame_start,
    input wire [4:0] level,
    input wire [5:0] contrast_level, // 0..40 is 0..200%, 20 is exact original
    input wire [7:0] fade_alpha, // 255=steady picture, 0=black transition
    input wire [23:0] rgb_in,
    output wire [23:0] rgb_out
);
    reg [8:0] gain_q;
    reg [8:0] gain_next;
    reg [8:0] contrast_q;
    reg [8:0] contrast_next;
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
    always @* begin
        case(contrast_level)
             0:contrast_next=0;    1:contrast_next=6;    2:contrast_next=13;
             3:contrast_next=19;   4:contrast_next=26;   5:contrast_next=32;
             6:contrast_next=38;   7:contrast_next=45;   8:contrast_next=51;
             9:contrast_next=58;  10:contrast_next=64;  11:contrast_next=70;
            12:contrast_next=77;  13:contrast_next=83;  14:contrast_next=90;
            15:contrast_next=96;  16:contrast_next=102; 17:contrast_next=109;
            18:contrast_next=115; 19:contrast_next=122; 20:contrast_next=128;
            21:contrast_next=134; 22:contrast_next=141; 23:contrast_next=147;
            24:contrast_next=154; 25:contrast_next=160; 26:contrast_next=166;
            27:contrast_next=173; 28:contrast_next=179; 29:contrast_next=186;
            30:contrast_next=192; 31:contrast_next=198; 32:contrast_next=205;
            33:contrast_next=211; 34:contrast_next=218; 35:contrast_next=224;
            36:contrast_next=230; 37:contrast_next=237; 38:contrast_next=243;
            39:contrast_next=250; default:contrast_next=256;
        endcase
    end
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin gain_q<=9'd256; contrast_q<=9'd128; end
        else if(frame_start) begin gain_q<=gain_next; contrast_q<=contrast_next; end

    // Rounded alpha folding; only one RGB multiply per channel even in fade mode.
    wire [16:0] alpha_product=fade_alpha * gain_q + 17'd128;
    wire [7:0] effective_alpha=alpha_product[15:8];
    // Retain the original fade's ceil rounding. At full brightness alpha is
    // unchanged, and ceil(channel*255/256)==channel for all 256 channel values.
    // At zero brightness all three channels are exactly zero.
    // Contrast must precede fade: applying a midpoint offset after fading
    // would turn a black transition gray whenever contrast is below 100%.
    wire [23:0] contrast_rgb={contrast_channel(rgb_in[23:16],contrast_q),
                             contrast_channel(rgb_in[15:8],contrast_q),
                             contrast_channel(rgb_in[7:0],contrast_q)};
    wire [15:0] r_mix=contrast_rgb[23:16]*effective_alpha + 16'd255;
    wire [15:0] g_mix=contrast_rgb[15:8] *effective_alpha + 16'd255;
    wire [15:0] b_mix=contrast_rgb[7:0]  *effective_alpha + 16'd255;
    function [7:0] contrast_channel;
        input [7:0] channel;
        input [8:0] contrast;
        reg signed [9:0] centered;
        reg signed [18:0] product;
        reg signed [10:0] adjusted;
        begin
            centered=$signed({2'b00,channel})-10'sd128;
            product=centered*$signed({1'b0,contrast});
            adjusted=(product >>> 7)+11'sd128;
            if(adjusted<0) contrast_channel=0;
            else if(adjusted>255) contrast_channel=255;
            else contrast_channel=adjusted[7:0];
        end
    endfunction
    assign rgb_out={r_mix[15:8],g_mix[15:8],b_mix[15:8]};
endmodule
