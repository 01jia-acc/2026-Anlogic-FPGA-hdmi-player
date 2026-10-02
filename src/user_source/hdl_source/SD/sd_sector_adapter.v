`timescale 1ns/1ps

// 将原工程 sd_card_top 的“读请求/数据有效/扇区结束”接口，适配为
// FAT32 模块使用的单扇区事务接口。原驱动保持不变，便于定位问题。
//
// 阅读导航：
// 1. sector_req/sector_lba 是上层发起一次“读一个 LBA 扇区”的请求；
// 2. 本模块把请求转换成 sd_sec_read，并在 SD 驱动返回字节时逐拍转发；
// 3. sector_valid 表示当前拍 sector_byte 有效，sector_byte_index 给出 0~511；
// 4. sd_sec_read_end 到来时产生一个时钟宽度的 sector_done，事务结束。
//
// 这里没有缓存整个扇区。原因是 FAT 扫描器/文件流读取器本身已经在接收端
// 按字节保存或消费数据；适配层只负责“握手翻译”和字节编号，避免复制一份
// 512 字节 RAM，也把原有 SD 驱动与新的 FAT 逻辑隔离开。
module sd_sector_adapter (
    // 所有接口都在同一个系统时钟域中；本工程的 rst 为高有效异步复位。
    input  wire        clk,
    input  wire        rst,
    // 上层单周期请求脉冲；只有 adapter 空闲时才会接受。
    input  wire        sector_req,
    // 要读取的逻辑块地址（LBA），单位是 512 字节扇区。
    input  wire [31:0] sector_lba,
    // 当前扇区事务正在进行；高电平期间上层不得发起下一次请求。
    output wire        sector_busy,
    // 当前拍 sector_byte/sector_byte_index 有效的脉冲。
    output reg         sector_valid,
    // SD 驱动返回的一个字节。
    output reg  [7:0]  sector_byte,
    // 当前字节在扇区中的偏移，范围 0~511。
    output reg  [8:0]  sector_byte_index,
    // 扇区所有字节已经传出后的单周期完成脉冲。
    output reg         sector_done,
    // 适配层错误码；当前原 SD 驱动没有向上提供具体错误，所以保留为 0。
    output reg  [7:0]  sector_error,
    // 调试观察口：active、当前 LBA、已转发字节数。
    output wire        debug_active,
    output wire [31:0] debug_lba,
    output wire [8:0]  debug_byte_count,

    // 下游原始 SD 扇区读接口。
    // sd_sec_read 是启动脉冲，sd_sec_read_addr 是被锁存的 LBA。
    output reg         sd_sec_read,
    output reg  [31:0] sd_sec_read_addr,
    // 原 SD 驱动返回的字节、字节有效和扇区结束信号。
    input  wire [7:0]  sd_sec_read_data,
    input  wire        sd_sec_read_data_valid,
    input  wire        sd_sec_read_end
);
    // 适配层目前没有定义本地错误来源；真正的 SD 错误由上游状态机通过
    // “没有正常结束/外部超时”等机制处理。保留统一错误字段便于以后扩展。
    localparam ERR_NONE = 8'h00;

    // active 是本模块唯一的事务状态寄存器：1 表示已经接受 sector_req，
    // 正在等待/转发这一扇区；它同时驱动 sector_busy。
    reg active;
    // 已接收/转发的字节数。9 位可以表示 0~511；到 511 后保持不再溢出。
    reg [8:0] byte_count;

    // 连续赋值表示 busy 只是 active 的组合观察值，并不额外存储状态。
    assign sector_busy = active;
    assign debug_active = active;
    assign debug_lba = sd_sec_read_addr;
    assign debug_byte_count = byte_count;

    // 时序逻辑：所有输出和内部寄存器只在 clk 上升沿改变。
    // 非阻塞赋值 <= 描述触发器并行采样，不能按软件语句顺序理解为串行执行。
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            active           <= 1'b0;
            byte_count       <= 9'd0;
            sector_valid     <= 1'b0;
            sector_byte      <= 8'd0;
            sector_byte_index<= 9'd0;
            sector_done      <= 1'b0;
            sector_error     <= ERR_NONE;
            sd_sec_read      <= 1'b0;
            sd_sec_read_addr <= 32'd0;
        end else begin
            // valid/done/read 都是脉冲型握手信号，默认每拍清零；只有满足
            // 条件的那个时钟周期重新置 1。这是“事件脉冲”而非电平状态。
            sector_valid <= 1'b0;
            sector_done  <= 1'b0;
            sd_sec_read  <= 1'b0;

            // 空闲时接受一个扇区请求。请求地址在此拍送入 SD 驱动，active
            // 从下一拍开始为 1；因此调用方应保持请求至少一个时钟周期。
            if (!active && sector_req) begin
                active           <= 1'b1;
                byte_count       <= 9'd0;
                sector_error     <= ERR_NONE;
                sd_sec_read_addr <= sector_lba;
                sd_sec_read      <= 1'b1;
            end

            // SD 驱动每返回一个有效字节，本模块给它编号并转发。byte_count
            // 和数据的关系是：本拍输出的是当前 byte_count，然后计数器准备
            // 指向下一个字节。
            if (active && sd_sec_read_data_valid) begin
                sector_valid      <= 1'b1;
                sector_byte       <= sd_sec_read_data;
                sector_byte_index <= byte_count;
                if (byte_count != 9'd511)
                    byte_count <= byte_count + 1'b1;
            end

            // 原驱动的结束事件只关闭本次事务并生成上层看到的 done 脉冲。
            if (active && sd_sec_read_end) begin
                active      <= 1'b0;
                sector_done <= 1'b1;
            end
        end
    end
endmodule
