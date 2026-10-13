`timescale 1ns/1ps
// 字形核心：只决定当前像素是否落在字的笔画上，并选择 RGB444 颜色。
// 所有输入均须已在 clk 像素域；frame_start 锁存不是 CDC 同步器。
// 输出是组合逻辑，零整拍延迟；组合传播时间仍须通过时序检查。
module status_osd_overlay #(
    parameter integer OSD_X=8, OSD_Y=8
)(
    input wire clk, rst_n,             // 像素时钟、低有效复位
    input wire frame_start,            // 消隐区内一拍的帧更新事件
    input wire [3:0] active_index,      // 已提交逻辑图片编号，0～15
    input wire auto_enabled,           // 已同步的自动模式：1=A，0=E（讲义约定）
    input wire [7:0] error_code,        // 已同步的错误快照，00 为正常
    input wire in_de,                  // 当前输入 RGB 是否属于有效显示区域
    input wire [11:0] in_rgb,          // RGB444，每个通道 4 位，非笔画像素透传
    input wire [11:0] pixel_x,pixel_y, // 必须与 in_rgb/in_de 同一个采样时刻
    output reg [11:0] out_rgb,         // 笔画时黄/红，否则 in_rgb
    output wire out_de, osd_pixel      // DE 不增加延迟；osd_pixel=有效笔画命中
);
    reg [3:0] index_q;                 // 一帧内固定的图片编号，避免数字画到一半变化
    reg auto_q;                        // 本帧模式
    reg [7:0] error_q;                 // 本帧错误码，同时决定文字颜色
    reg hit;                           // 当前格子是否是字的一笔（不是整个文字矩形）
    reg [3:0] value;                   // 当前字符的十六进制值，非 ASCII 编码
    reg [4:0] bits_row;                // 当前字形的一行，bit4 在左、bit0 在右
    wire [11:0] dx=pixel_x-OSD_X;      // 相对于状态条左上角的横向距离
    wire [11:0] dy=pixel_y-OSD_Y;      // 相对于状态条左上角的纵向距离
    wire [4:0] image_number={1'b0,index_q}+5'd1;
    // dx[5:3] = dx/8，dx[2:0] = dx%8。每字占8列，前5列画字。
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin index_q<=0; auto_q<=0; error_q<=0; end
        else if(frame_start) begin
            index_q<=active_index; auto_q<=auto_enabled; error_q<=error_code;
        end

    // 字模查表：输入数字与行号，返回5个横向点。case 通常综合为查表逻辑；
    // 它不是软件调用，也不保证使用块 RAM。第7行由下方 dy<7 屏蔽。
    function [4:0] font_row;
        input [3:0] value;
        input [2:0] row;
        begin
            case (value)
                4'h0: case(row) 0:font_row=5'b11111;1:font_row=5'b10001;2:font_row=5'b10001;3:font_row=5'b10001;4:font_row=5'b10001;5:font_row=5'b10001;default:font_row=5'b11111; endcase
                4'h1: case(row) 0:font_row=5'b00100;1:font_row=5'b01100;2:font_row=5'b00100;3:font_row=5'b00100;4:font_row=5'b00100;5:font_row=5'b00100;default:font_row=5'b11111; endcase
                4'h2: case(row) 0:font_row=5'b11111;1:font_row=5'b00001;2:font_row=5'b00001;3:font_row=5'b11111;4:font_row=5'b10000;5:font_row=5'b10000;default:font_row=5'b11111; endcase
                4'h3: case(row) 0:font_row=5'b11111;1:font_row=5'b00001;2:font_row=5'b00001;3:font_row=5'b11111;4:font_row=5'b00001;5:font_row=5'b00001;default:font_row=5'b11111; endcase
                4'h4: case(row) 0:font_row=5'b10001;1:font_row=5'b10001;2:font_row=5'b10001;3:font_row=5'b11111;4:font_row=5'b00001;5:font_row=5'b00001;default:font_row=5'b00001; endcase
                4'h5: case(row) 0:font_row=5'b11111;1:font_row=5'b10000;2:font_row=5'b10000;3:font_row=5'b11111;4:font_row=5'b00001;5:font_row=5'b00001;default:font_row=5'b11111; endcase
                4'h6: case(row) 0:font_row=5'b11111;1:font_row=5'b10000;2:font_row=5'b10000;3:font_row=5'b11111;4:font_row=5'b10001;5:font_row=5'b10001;default:font_row=5'b11111; endcase
                4'h7: case(row) 0:font_row=5'b11111;1:font_row=5'b00001;2:font_row=5'b00010;3:font_row=5'b00100;4:font_row=5'b01000;5:font_row=5'b01000;default:font_row=5'b01000; endcase
                4'h8: case(row) 0:font_row=5'b11111;1:font_row=5'b10001;2:font_row=5'b10001;3:font_row=5'b11111;4:font_row=5'b10001;5:font_row=5'b10001;default:font_row=5'b11111; endcase
                4'h9: case(row) 0:font_row=5'b11111;1:font_row=5'b10001;2:font_row=5'b10001;3:font_row=5'b11111;4:font_row=5'b00001;5:font_row=5'b00001;default:font_row=5'b11111; endcase
                4'ha: case(row) 0:font_row=5'b01110;1:font_row=5'b10001;2:font_row=5'b10001;3:font_row=5'b11111;4:font_row=5'b10001;5:font_row=5'b10001;default:font_row=5'b10001; endcase
                4'he: case(row) 0:font_row=5'b11111;1:font_row=5'b10000;2:font_row=5'b10000;3:font_row=5'b11110;4:font_row=5'b10000;5:font_row=5'b10000;default:font_row=5'b11111; endcase
                4'hf: case(row) 0:font_row=5'b11111;1:font_row=5'b10000;2:font_row=5'b10000;3:font_row=5'b11110;4:font_row=5'b10000;5:font_row=5'b10000;default:font_row=5'b10000; endcase
                // 补齐 B/C/D，避免包含这些十六进制数字的错误码显示空白。
                4'hb: case(row) 0:font_row=5'b11110;3:font_row=5'b11110;6:font_row=5'b11110;default:font_row=5'b10001;endcase
                4'hc: case(row) 0:font_row=5'b01111;6:font_row=5'b01111;default:font_row=5'b10000;endcase
                4'hd: case(row) 0:font_row=5'b11110;6:font_row=5'b11110;default:font_row=5'b10001;endcase
                default: font_row = 5'b00000;
            endcase
        end
    endfunction

    always @* begin
        // 所有组合输出先给默认值，保证不推断锁存器。
        hit=0; value=0; bits_row=0;
        if(pixel_x>=OSD_X && pixel_x<OSD_X+40 &&
           pixel_y>=OSD_Y && pixel_y<OSD_Y+8) begin
            case(dx[5:3])
                0:value=image_number >= 10 ? 4'd1 : 4'd0;
                1:value=image_number >= 10 ? image_number-5'd10 : image_number;
                2:value=auto_q ? 4'ha : 4'he;
                3:value=error_q[7:4];
                4:value=error_q[3:0];
            endcase
            bits_row=font_row(value,dy[2:0]);
            if(dx[2:0]<5 && dy<7) hit=bits_row[4-dx[2:0]];
        end
        out_rgb=in_rgb;
        if(in_de && hit) out_rgb=(error_q==0) ? 12'hf80 : 12'hf44;
    end
    assign out_de=in_de;
    assign osd_pixel=in_de && hit;
endmodule
