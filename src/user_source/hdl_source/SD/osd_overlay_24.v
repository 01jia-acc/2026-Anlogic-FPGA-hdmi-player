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

// Shared output coordinates; no additional video pipeline latency.
// Keep this module in the registered OSD source to avoid a new project entry.
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
