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
