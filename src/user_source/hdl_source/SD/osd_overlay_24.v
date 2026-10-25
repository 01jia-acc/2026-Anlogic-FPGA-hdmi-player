`timescale 1ns/1ps
// RGB888 包装层：用同一组最终 DE/VS 重建坐标，再调用 RGB444 字形核心。
// 本工程适配：讲义外接 pixel_x；这里同时重建 x/y，避免前级坐标与后级RGB错拍。
// 不改原图缓存。只在输出时把字形笔画像素替换为文字色。
module osd_overlay_24 #(
    parameter integer OSD_X=8, OSD_Y=8
)(
    input wire clk,rst_n,             // video_clk 与像素域复位
    input wire vsync,de,              // 与 rgb_in 同一流水级；VS低有效，DE高有效
    input wire [23:0] rgb_in,         // 原始RGB888，R[23:16]/G[15:8]/B[7:0]
    input wire [3:0] active_index,    // 像素域已提交的逻辑图片编号（0～15）
    input wire auto_enabled,          // 已安全跨域的模式
    input wire [7:0] error_code,      // 已安全跨域的完整错误快照
    output wire [23:0] rgb_out,       // 完整保留非文字像素的24位精度
    output wire de_out,osd_active,    // 有效标签与笔画命中标志
    output wire frame_start,          // VS下降沿事件，供状态快照提交使用
    output wire [11:0] pixel_x_out, pixel_y_out // 同拍坐标供时间戳共享
);
    reg [11:0] pixel_x;               // 当前输入有效像素的列；行消隐时准备为0
    reg [11:0] line_y;                // 当前有效行；只在DE下降沿加1，不数消隐行
    reg de_q;                         // 上一拍DE，用于识别有效行结束
    reg vs_q;                         // 上一拍VS，用于识别场同步开始
    assign frame_start=vs_q && !vsync;
    assign pixel_x_out=pixel_x;
    assign pixel_y_out=line_y;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin pixel_x<=0; line_y<=0; de_q<=0; vs_q<=1; end
        else begin
            de_q<=de; vs_q<=vsync;
            // 下游在时钟沿采样的是更新前坐标：首个DE像素见x=0，下一像素见x=1。
            if(!de) pixel_x<=0;
            else pixel_x<=pixel_x+1'b1;
            if(!vsync) line_y<=0;
            else if(de_q && !de) line_y<=line_y+1'b1;
        end
    end
    wire [11:0] osd_rgb12;            // 核心产生的4位/通道文字颜色
    wire hit;                         // 只有笔画为1，字符间距与空心部分均为0
    status_osd_overlay #(.OSD_X(OSD_X),.OSD_Y(OSD_Y)) core(
        .clk(clk),.rst_n(rst_n),.frame_start(frame_start),
        .active_index(active_index),.auto_enabled(auto_enabled),.error_code(error_code),
        .in_de(de),.in_rgb({rgb_in[23:20],rgb_in[15:12],rgb_in[7:4]}),
        .pixel_x(pixel_x),.pixel_y(line_y),.out_rgb(osd_rgb12),
        .out_de(),.osd_pixel(hit)
    );
    // 4位n扩成8位 {n,n} = 17*n：F变FF、4变44，而不是只补四个0。
    // 核心虽接收到截断背景，但非命中时走 rgb_in 旁路，整张图不会降色深。
    assign rgb_out=hit ? {osd_rgb12[11:8],osd_rgb12[11:8],
                          osd_rgb12[7:4],osd_rgb12[7:4],
                          osd_rgb12[3:0],osd_rgb12[3:0]} : rgb_in;
    assign de_out=de;
    assign osd_active=hit;
endmodule

// The descriptor is the SAME frame-latched packet used by the reader, so
// focus, logical image labels and completeness cannot disagree with its RGB.
module thumbnail_status_osd #(
    parameter integer THUMBNAIL_DIV = 4
) (
    input wire de,
    input wire [11:0] pixel_x, pixel_y,
    input wire [31:0] descriptor,
    input wire [23:0] rgb_in,
    output reg [23:0] rgb_out
);
    wire enabled=descriptor[23];
    localparam integer TILE_W=640/THUMBNAIL_DIV, TILE_H=480/THUMBNAIL_DIV;
    localparam integer LEFT=320-TILE_W, TOP=240-TILE_H;
    localparam integer TITLE_X=THUMBNAIL_DIV==2 ? 104 : 160;
    localparam integer TITLE_Y=THUMBNAIL_DIV==2 ? 4 : 96;
    localparam integer HELP_X=THUMBNAIL_DIV==2 ? 8 : 160;
    localparam integer HELP_Y=THUMBNAIL_DIV==2 ? 468 : 384;
    localparam integer BACK_X=THUMBNAIL_DIV==2 ? 552 : 160;
    localparam integer BACK_Y=THUMBNAIL_DIV==2 ? 468 : 400;
    wire [1:0] slot={pixel_y>=240,pixel_x>=320};
    wire ready=descriptor[16+slot];
    wire focused=descriptor[21:20]==slot;
    wire [4:0] number={1'b0,descriptor[slot*4 +:4]}+5'd1;
    wire [11:0] lx=pixel_x-(pixel_x>=320 ? 320 : LEFT);
    wire [11:0] ly=pixel_y-(pixel_y>=240 ? 240 : TOP);
    wire inside=pixel_x>=LEFT && pixel_x<320+TILE_W && pixel_y>=TOP && pixel_y<240+TILE_H;
    reg [11:0] tx, ty;
    reg [7:0] ch;
    reg [34:0] glyph;
    reg [4:0] bits_row;
    reg text_area;
    // 5x7, eight-pixel character seats. Constant shifts select ROM characters.
    localparam [8*13-1:0] TITLE="CACHE PREVIEW";
    localparam [8*19-1:0] HELP="2 NEXT 4 PREV 3 OK ";
    localparam [8*10-1:0] BACK="HOLD3 BACK";
    always @* begin
        tx=0; ty=0; ch=" "; text_area=0;
        if(pixel_x>=TITLE_X && pixel_x<TITLE_X+104 && pixel_y>=TITLE_Y && pixel_y<TITLE_Y+7) begin
            tx=pixel_x-TITLE_X; ty=pixel_y-TITLE_Y; text_area=1;
            ch=TITLE >> ((12-tx[6:3])*8);
        end else if(pixel_x>=HELP_X && pixel_x<HELP_X+152 && pixel_y>=HELP_Y && pixel_y<HELP_Y+7) begin
            tx=pixel_x-HELP_X; ty=pixel_y-HELP_Y; text_area=1;
            ch=HELP >> ((18-tx[7:3])*8);
        end else if(pixel_x>=BACK_X && pixel_x<BACK_X+80 && pixel_y>=BACK_Y && pixel_y<BACK_Y+7) begin
            tx=pixel_x-BACK_X; ty=pixel_y-BACK_Y; text_area=1;
            ch=BACK >> ((9-tx[6:3])*8);
        end else if(inside && lx>=8 && lx<24 && ly>=4 && ly<11) begin
            tx=lx-8; ty=ly-4; text_area=1;
            if(!ready) ch="-";
            else if(tx<8) ch=number>=10 ? "1" : "0";
            else ch="0"+(number>=10 ? number-10 : number);
        end
        case(ch)
            "0":glyph={5'b01110,5'b10001,5'b10011,5'b10101,5'b11001,5'b10001,5'b01110};
            "1":glyph={5'b00100,5'b01100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
            "2":glyph={5'b01110,5'b10001,5'b00001,5'b00010,5'b00100,5'b01000,5'b11111};
            "3":glyph={5'b11110,5'b00001,5'b00001,5'b01110,5'b00001,5'b00001,5'b11110};
            "4":glyph={5'b00010,5'b00110,5'b01010,5'b10010,5'b11111,5'b00010,5'b00010};
            "5":glyph={5'b11111,5'b10000,5'b10000,5'b11110,5'b00001,5'b00001,5'b11110};
            "6":glyph={5'b01110,5'b10000,5'b10000,5'b11110,5'b10001,5'b10001,5'b01110};
            "7":glyph={5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b01000,5'b01000};
            "8":glyph={5'b01110,5'b10001,5'b10001,5'b01110,5'b10001,5'b10001,5'b01110};
            "9":glyph={5'b01110,5'b10001,5'b10001,5'b01111,5'b00001,5'b00001,5'b01110};
            "A":glyph={5'b01110,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
            "B":glyph={5'b11110,5'b10001,5'b10001,5'b11110,5'b10001,5'b10001,5'b11110};
            "C":glyph={5'b01111,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b01111};
            "D":glyph={5'b11110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b11110};
            "E":glyph={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b11111};
            "H":glyph={5'b10001,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
            "I":glyph={5'b01110,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
            "K":glyph={5'b10001,5'b10010,5'b10100,5'b11000,5'b10100,5'b10010,5'b10001};
            "L":glyph={5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b11111};
            "N":glyph={5'b10001,5'b11001,5'b10101,5'b10011,5'b10001,5'b10001,5'b10001};
            "O":glyph={5'b01110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
            "P":glyph={5'b11110,5'b10001,5'b10001,5'b11110,5'b10000,5'b10000,5'b10000};
            "R":glyph={5'b11110,5'b10001,5'b10001,5'b11110,5'b10100,5'b10010,5'b10001};
            "T":glyph={5'b11111,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100};
            "V":glyph={5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01010,5'b00100};
            "W":glyph={5'b10001,5'b10001,5'b10001,5'b10101,5'b10101,5'b10101,5'b01010};
            "X":glyph={5'b10001,5'b10001,5'b01010,5'b00100,5'b01010,5'b10001,5'b10001};
            "-":glyph={5'b00000,5'b00000,5'b00000,5'b11111,5'b00000,5'b00000,5'b00000};
            default:glyph=0;
        endcase
        bits_row=ty<7 ? glyph >> ((6-ty[2:0])*5) : 0;
        rgb_out=rgb_in;
        if(de && enabled) begin
            if(inside && ly<14) rgb_out=24'h101820;
            if(THUMBNAIL_DIV==2 && inside && pixel_y>=464) rgb_out=24'h101820;
            if(inside && (lx<2 || lx>=TILE_W-2 || ly<2 || ly>=TILE_H-2))
                rgb_out=focused ? (ready ? 24'hffff00 : 24'hff8000) : 24'h607080;
            if(text_area && tx[2:0]<5 && bits_row[4-tx[2:0]]) rgb_out=24'hffffff;
        end
    end
endmodule

// Shared output coordinates; no additional video pipeline latency.
// Keep this module in the registered OSD source to avoid a new project entry.
// Top-center VOL  75%, frozen for a whole frame. Pure RGB overlay adds no delay.
module volume_status_osd #(
    parameter integer OSD_X=288,
    parameter [23:0] LABEL="VOL",
    parameter integer LEVEL_BITS=5,
    parameter [LEVEL_BITS-1:0] DEFAULT_LEVEL=15
) (
    input wire clk, rst_n, frame_start, de,
    input wire [11:0] pixel_x, pixel_y,
    input wire [LEVEL_BITS-1:0] level,
    input wire adjust_mode,
    input wire [23:0] rgb_in,
    output wire [23:0] rgb_out
);
    reg [LEVEL_BITS-1:0] level_q;
    reg mode_q;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin level_q<=DEFAULT_LEVEL; mode_q<=0; end
        else if(frame_start) begin level_q<=level; mode_q<=adjust_mode; end
    // 5% steps: decimal tens = level/2, units = odd ? 5 : 0.
    wire [11:0] dx=pixel_x-OSD_X, dy=pixel_y-8;
    wire [5:0] tens=level_q >> 1;
    wire [5:0] tens_digit=tens-(level_q>=40 ? 6'd20 : level_q>=20 ? 6'd10 : 6'd0);
    reg [7:0] ch;
    reg [34:0] glyph;
    reg [4:0] row;
    always @* begin
        case(dx[5:3])
            0:ch=LABEL[23:16]; 1:ch=LABEL[15:8]; 2:ch=LABEL[7:0];
            4:ch=level_q>=40 ? "2" : level_q>=20 ? "1" : " ";
            5:ch=level_q<2 ? " " : 8'd48+tens_digit;
            6:ch=level_q[0] ? "5" : "0";
            7:ch="%";
            default:ch=" ";
        endcase
        case(ch)
            "B":glyph={5'b11110,5'b10001,5'b10001,5'b11110,5'b10001,5'b10001,5'b11110};
            "C":glyph={5'b01111,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b01111};
            "N":glyph={5'b10001,5'b11001,5'b10101,5'b10011,5'b10001,5'b10001,5'b10001};
            "R":glyph={5'b11110,5'b10001,5'b10001,5'b11110,5'b10100,5'b10010,5'b10001};
            "I":glyph={5'b11111,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b11111};
            "V":glyph={5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01010,5'b00100};
            "O","0":glyph={5'b01110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
            "L":glyph={5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b11111};
            "1":glyph={5'b00100,5'b01100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
            "2":glyph={5'b01110,5'b10001,5'b00001,5'b00010,5'b00100,5'b01000,5'b11111};
            "3":glyph={5'b11110,5'b00001,5'b00001,5'b01110,5'b00001,5'b00001,5'b11110};
            "4":glyph={5'b00010,5'b00110,5'b01010,5'b10010,5'b11111,5'b00010,5'b00010};
            "5":glyph={5'b11111,5'b10000,5'b10000,5'b11110,5'b00001,5'b00001,5'b11110};
            "6":glyph={5'b01110,5'b10000,5'b10000,5'b11110,5'b10001,5'b10001,5'b01110};
            "7":glyph={5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b01000,5'b01000};
            "8":glyph={5'b01110,5'b10001,5'b10001,5'b01110,5'b10001,5'b10001,5'b01110};
            "9":glyph={5'b01110,5'b10001,5'b10001,5'b01111,5'b00001,5'b00001,5'b01110};
            "%":glyph={5'b11001,5'b11010,5'b00010,5'b00100,5'b01000,5'b01011,5'b10011};
            default:glyph=0;
        endcase
        row=dy<7 ? (glyph >> ((6-dy[2:0])*5)) : 0;
    end
    wire hit=de && pixel_x>=OSD_X && pixel_x<OSD_X+64 && pixel_y>=8 && pixel_y<15 &&
        dx[2:0]<5 && row[4-dx[2:0]];
    assign rgb_out=hit ? (mode_q ? 24'hffff00 : 24'hffffff) : rgb_in;
endmodule

module zoom_status_osd (
    input wire clk, rst_n, de,
    input wire [11:0] pixel_x, pixel_y,
    input wire zoom_mode,
    input wire [1:0] zoom_level,
    input wire [23:0] rgb_in,
    output wire [23:0] rgb_out
);
    reg mode_q;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) mode_q<=0;
        else if(!de) mode_q<=zoom_mode;
    wire [11:0] dx=pixel_x-80, dy=pixel_y-8;
    reg [34:0] glyph;
    reg [7:0] ch;
    reg [4:0] row;
    always @* begin
        case(dx[6:3])
            0:ch="Z"; 1,2:ch="O"; 3:ch="M";
            5:ch=zoom_level==0 ? "0" : zoom_level==3 ? "2" : "1";
            6:ch=".";
            7:ch=(zoom_level==0 || zoom_level==2) ? "5" : "0";
            8:ch="X";
            default:ch=" ";
        endcase
        case(ch)
            "Z":glyph={5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b10000,5'b11111};
            "O","0":glyph={5'b01110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
            "M":glyph={5'b10001,5'b11011,5'b10101,5'b10101,5'b10001,5'b10001,5'b10001};
            "1":glyph={5'b00100,5'b01100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
            "2":glyph={5'b01110,5'b10001,5'b00001,5'b00010,5'b00100,5'b01000,5'b11111};
            "5":glyph={5'b11111,5'b10000,5'b10000,5'b11110,5'b00001,5'b00001,5'b11110};
            ".":glyph={5'b00000,5'b00000,5'b00000,5'b00000,5'b00000,5'b00100,5'b00100};
            "X":glyph={5'b10001,5'b10001,5'b01010,5'b00100,5'b01010,5'b10001,5'b10001};
            default:glyph=0;
        endcase
        row=dy<7 ? (glyph >> ((6-dy[2:0])*5)) : 0;
    end
    wire hit=de && (mode_q || zoom_level!=1) && pixel_x>=80 && pixel_x<152 &&
        pixel_y>=8 && pixel_y<15 && dx[2:0]<5 && row[4-dx[2:0]];
    assign rgb_out=hit ? (mode_q ? 24'hffff00 : 24'hffffff) : rgb_in;
endmodule
