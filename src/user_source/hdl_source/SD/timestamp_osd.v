`timescale 1ns/1ps
// 上电运行时长：像素时钟分频计秒，30:00饱和保持，系统复位重新计时。
// 当前video_clk=25MHz，因此25,000,000拍是一秒，不依赖DE、帧率或图片切换。
// 计时精度取决于板载时钟误差；显示在帧边界更新，最多晚一帧看到新秒数。
module timestamp_osd #(
    parameter integer CLOCK_FREQ_HZ = 25_000_000,
    parameter integer OSD_X = 552,
    parameter integer OSD_Y = 8
) (
    input wire clk, rst_n,          // video_clk与低有效复位
    input wire frame_start,         // 与坐标同源的帧事件
    input wire de,                  // 有效区标签，消隐时间也继续计秒
    input wire [11:0] pixel_x, pixel_y, // 共用现有OSD坐标，避免重复计数错拍
    input wire [23:0] rgb_in,       // 已经叠好状态条的RGB888
    output wire [23:0] rgb_out      // 时间文字白色；非笔画像素完整透传
);
    localparam integer DIV_WIDTH = (CLOCK_FREQ_HZ<=1) ? 1 : $clog2(CLOCK_FREQ_HZ);
    reg [DIV_WIDTH-1:0] subsecond;  // 一秒内第几拍，0到频率-1，25MHz需25位
    reg [10:0] elapsed_seconds;    // 总秒数0..1800；11位可表示0..2047
    reg [3:0] mt, mo, st, so;     // 分十位/分个位/秒十位/秒个位：BCD数字
    reg [15:0] frame_digits;       // 本帧冻结的4个数字，避免09:59变10:00时混字
    wire tick = (subsecond == CLOCK_FREQ_HZ-1) && (elapsed_seconds < 1800);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            subsecond<=0; elapsed_seconds<=0;
            mt<=0; mo<=0; st<=0; so<=0; frame_digits<=0;
        end else begin
            if(elapsed_seconds < 1800) begin
                if(tick) begin
                    subsecond<=0;
                    elapsed_seconds<=elapsed_seconds+1'b1;
                    // 用十进制进位代替每像素计算/60、/10的除法。
                    if(so!=9) so<=so+1'b1;
                    else begin
                        so<=0;
                        if(st!=5) st<=st+1'b1;
                        else begin
                            st<=0;
                            if(mo!=9) mo<=mo+1'b1;
                            else begin mo<=0; mt<=mt+1'b1; end
                        end
                    end
                end else subsecond<=subsecond+1'b1;
            end
            // 与tick同拍时采样旧数字，至多下一帧显示新数字，不影响计秒。
            if(frame_start) frame_digits<={mt,mo,st,so};
        end
    end

    // 5x7点阵数字，放大2倍为10x14；每个字符座位16x16，5字共80x16。
    function [4:0] digit_row;
        input [3:0] digit;
        input [2:0] row;
        reg [34:0] glyph;           // 从高到低依次存第0..6行
        begin
            case(digit)
                0:glyph={5'b01110,5'b10001,5'b10011,5'b10101,5'b11001,5'b10001,5'b01110};
                1:glyph={5'b00100,5'b01100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
                2:glyph={5'b01110,5'b10001,5'b00001,5'b00010,5'b00100,5'b01000,5'b11111};
                3:glyph={5'b11110,5'b00001,5'b00001,5'b01110,5'b00001,5'b00001,5'b11110};
                4:glyph={5'b00010,5'b00110,5'b01010,5'b10010,5'b11111,5'b00010,5'b00010};
                5:glyph={5'b11111,5'b10000,5'b10000,5'b11110,5'b00001,5'b00001,5'b11110};
                6:glyph={5'b01110,5'b10000,5'b10000,5'b11110,5'b10001,5'b10001,5'b01110};
                7:glyph={5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b01000,5'b01000};
                8:glyph={5'b01110,5'b10001,5'b10001,5'b01110,5'b10001,5'b10001,5'b01110};
                9:glyph={5'b01110,5'b10001,5'b10001,5'b01111,5'b00001,5'b00001,5'b01110};
                default:glyph=0;
            endcase
            digit_row=(row<7) ? (glyph >> ((6-row)*5)) : 5'd0;
        end
    endfunction
    wire [11:0] dx=pixel_x-OSD_X, dy=pixel_y-OSD_Y;
    wire [2:0] col=dx[3:1];         // 字符内部x/2，每个点水平占2个像素
    wire [2:0] row=dy[3:1];         // y/2，每个点垂直占2个像素
    reg [3:0] digit;                // 当前字符数字
    reg [4:0] bits_row;             // 当前字符点阵的一行
    reg hit;                       // 真正命中笔画，空隙仍显示背景
    always @* begin
        hit=0; digit=0; bits_row=0;
        if(de && pixel_x>=OSD_X && pixel_x<OSD_X+80 &&
           pixel_y>=OSD_Y && pixel_y<OSD_Y+16) begin
            case(dx[6:4])
                0:digit=frame_digits[15:12];
                1:digit=frame_digits[11:8];
                3:digit=frame_digits[7:4];
                4:digit=frame_digits[3:0];
                default:digit=0;
            endcase
            bits_row=digit_row(digit,row);
            if(dx[6:4]==2) hit=(col==2 && (row==2 || row==4));
            else if(col<5 && row<7) hit=bits_row[4-col];
        end
    end
    assign rgb_out=hit ? 24'h000000 : rgb_in;
endmodule
