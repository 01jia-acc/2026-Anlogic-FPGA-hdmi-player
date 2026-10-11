`timescale 1ns/1ps
// 一包数据的跨域邮箱，基于讲义增加源域冻结快照和ACK，防止重复toggle丢事件。
// 96位通常容纳12个8位字符；本次也复用来传低速状态（不是画字模块）。
// 协议：仅在src_ready=1时提交；一包未确认时不能覆盖hold寄存器。
// 两域须一起复位；独立复位某一端需要额外恢复协议，本模块不支持。
module subtitle_frame_commit(
    input wire src_clk,src_rst_n,    // 发送域时钟与低有效复位
    input wire src_commit,           // 当src_ready为1时，本拍要求提交输入包
    input wire [95:0] src_text,      // 完整文本或不超过96位的状态数据
    input wire [5:0] src_length,     // 字符数元数据，与text一起保持/提交
    input wire src_enable,          // 使能元数据，也不能单独跨域
    output wire src_ready,          // 1=邮箱空闲，可接下一包；不是目标端显示使能
    input wire dst_clk,dst_rst_n,    // 接收域，此处为像素时钟
    input wire frame_start,         // 允许整包更新的安全时刻；文本用帧边界
    output reg [95:0] dst_text,     // 已提交的接收域完整快照
    output reg [5:0] dst_length,    // 已提交长度
    output reg dst_enable           // 已提交使能
);
    reg [95:0] text_hold;           // 源域冻结数据：直到ACK回来绝不改
    reg [5:0] length_hold;          // 源域冻结长度
    reg enable_hold;               // 源域冻结开关
    reg request_toggle;            // 每接收一包翻转一次，比短脉冲易于跨域观察
    reg req_sync1,req_sync2;        // 请求事件进入目标域的两级同步
    reg acknowledge;               // 目标域最后已提交的请求编号（0或1）
    reg ack_sync1,ack_sync2;        // 确认事件返回源域的两级同步
    assign src_ready=(request_toggle==ack_sync2);
    always @(posedge src_clk or negedge src_rst_n) begin
        if(!src_rst_n) begin
            text_hold<=0; length_hold<=0; enable_hold<=0;
            request_toggle<=0; ack_sync1<=0; ack_sync2<=0;
        end else begin
            ack_sync1<=acknowledge; ack_sync2<=ack_sync1;
            if(src_commit && src_ready) begin
                text_hold<=src_text; length_hold<=src_length; enable_hold<=src_enable;
                request_toggle<=~request_toggle;
            end
        end
    end
    always @(posedge dst_clk or negedge dst_rst_n) begin
        if(!dst_rst_n) begin
            req_sync1<=0; req_sync2<=0; acknowledge<=0;
            dst_text<=0; dst_length<=0; dst_enable<=0;
        end else begin
            req_sync1<=request_toggle; req_sync2<=req_sync1;
            if(frame_start && req_sync2!=acknowledge) begin
                // 数据总线不逐位同步：源域一直保持不变，控制请求先经两级同步。
                // 这是bundled-data握手；实现仍需保证保持总线在采样前稳定的时序。
                dst_text<=text_hold; dst_length<=length_hold; dst_enable<=enable_hold;
                acknowledge<=req_sync2; // 读完才签收，允许源端以后重写hold
            end
        end
    end
endmodule
