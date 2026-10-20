`timescale 1ns/1ps

// ============================================================================
// media_sdram_write_cdc：写命令跨时钟域搬运器（手写异步 FIFO）
//
// 用途：把 loader 时钟域（src_clk = sd_card_clk，100MHz）产生的"写 SDRAM
//       命令"打包，安全搬运到 SDRAM 控制器时钟域（dst_clk = ext_mem_clk，
//       50MHz）。一条命令 = {地址, 字节掩码, 数据} = 57 位，三组总线作为
//       一个整体跨时钟域，避免分别打拍采样造成的"地址与数据错位"。
//
// 结构：手写异步 FIFO，原理与工程里 IP 生成的 wfifo/rfifo 完全相同——
//   * 存储体：mem[DEPTH]，每格存一条 57 位命令
//   * 写侧（src_clk 域）：wr_bin/wr_gray 写指针；对方读指针格雷码两级同步回来
//   * 读侧（dst_clk 域）：rd_bin/rd_gray 读指针；对方写指针格雷码两级同步回来
//   * 满/空：格雷码比较。指针位宽 = 地址位宽 + 1（多 1 位区分空与满）
//   * 读出口带输出寄存器（rd_data/rd_valid）：读出命令有 1 拍延迟
//
// 两个特色设计：
//   1. 提前一格反压：src_full 输出的是 almost_full_reg（差 1 格才满）。
//      因为 loader 的命令输出是寄存器输出，它看到"满"时已有一条命令在途、
//      下一拍必然到达；提前一格喊满，让这条在途命令仍能进来，
//      真正的 full_reg 做最后兜底拒绝（配合 src_overflow 报警）。
//   2. dst 侧突发：攒齐 BURST_SIZE=128 条命令且 dst_allow 放行后才启动。
//      一旦启动就连续发送整块，不能用 busy/refresh 逐字暂停。
//      吐完后等待收尾间隔，再检查是否允许下一轮。帧大小须为整块倍数。
//
// 数据流：
//   media_frame_loader（src_clk 域）
//     --src_wr_en/src_wr_addr/src_wr_mask/src_wr_data--> 本模块 FIFO
//     --dst_app_wr_en/addr/mask/data--> SDRAM 控制器 App_wr_*
//     dst_idle=1 表示所有命令已真正交给 SDRAM，顶层据此确认"整帧写完"。
// ============================================================================
module media_sdram_write_cdc #(
    parameter integer ADDR_WIDTH       = 21,  // SDRAM 地址位宽（4 bank × 307200 字 < 2^21）
    parameter integer FIFO_ADDR_WIDTH  = 9,   // FIFO 深度指数：DEPTH = 2^9 = 512
    parameter integer BURST_SIZE       = 128, // 帧字数必须是 BURST_SIZE 的整数倍
    parameter integer BURST_GAP_CYCLES = 12  // 最后一字输出后再留出控制器收尾时间
) (
    // ------------------------- src 域（loader 时钟域）-------------------------
    input  wire                  src_clk,        // 【输入】源时钟：接 sd_card_clk（100MHz）
    input  wire                  src_rst_n,      // 【输入】源域复位，低有效
    input  wire                  src_wr_en,      // 【输入】写命令有效：=1 表示这一拍来一条命令
    input  wire [ADDR_WIDTH-1:0] src_wr_addr,    // 【输入】命令①：SDRAM 写地址 [20:0]
    input  wire [3:0]            src_wr_mask,    // 【输入】命令②：字节掩码 [3:0]（全 0 = 4 字节全写）
    input  wire [31:0]           src_wr_data,    // 【输入】命令③：写数据 [31:0]（32 位像素字）
    output wire                  src_full,       // 【输出】对外满标志：提前一格反压给 loader
    output reg                   src_overflow,   // 【输出】真溢出报警：满的时候还来命令
    // ------------------------- dst 域（SDRAM 控制器时钟域）--------------------
    input  wire                  dst_clk,        // 【输入】目的时钟：接 ext_mem_clk（50MHz）
    input  wire                  dst_rst_n,      // 【输入】目的域复位，低有效
    input  wire                  dst_allow,      // 【输入】总线放行：顶层互斥仲裁允许写时才为 1
    output wire                  dst_app_wr_en,  // 【输出】写使能 → SDRAM 控制器 App_wr_en
    output wire [ADDR_WIDTH-1:0] dst_app_wr_addr,// 【输出】写地址 → SDRAM 控制器 App_wr_addr
    output wire [3:0]            dst_app_wr_mask,// 【输出】字节掩码 → SDRAM 控制器 App_wr_dm
    output wire [31:0]           dst_app_wr_data,// 【输出】写数据 → SDRAM 控制器 App_wr_din
    output wire                  dst_idle        // 【输出】FIFO 已排空且无突发在途（整帧写完依据）
);
    // ------------------------------------------------------------------------
    // 参数与常量
    // ------------------------------------------------------------------------
    // 命令打包位宽：{地址[20:0], 掩码[3:0], 数据[31:0]} = 21 + 4 + 32 = 57
    localparam integer COMMAND_WIDTH = ADDR_WIDTH + 36;
    // 指针位宽 = FIFO 地址位宽 + 1。多出的最高位是"绕圈计数位"，用来区分
    // 指针完全相等时的两种含义：空（差 0 个字）还是满（差 512 个字）。
    localparam integer PTR_WIDTH = FIFO_ADDR_WIDTH + 1;
    // FIFO 深度：2^9 = 512 格
    localparam integer DEPTH = 1 << FIFO_ADDR_WIDTH;
    // 突发计数器位宽：clog2(128) = 7；burst_count 声明为 8 位，多 1 位存 128 本身
    localparam integer BURST_COUNT_WIDTH = (BURST_SIZE <= 2) ? 1 : $clog2(BURST_SIZE);

    // ------------------------------------------------------------------------
    // 存储体与指针
    // ------------------------------------------------------------------------
    reg [COMMAND_WIDTH-1:0] mem [0:DEPTH-1];   // 命令存储体：每格存一条 57 位命令
    reg [PTR_WIDTH-1:0] wr_bin, wr_gray;       // 写指针：二进制（写 RAM 用）+ 格雷码（跨域用），src 域
    reg [PTR_WIDTH-1:0] rd_bin, rd_gray;       // 读指针：二进制（读 RAM 用）+ 格雷码（跨域用），dst 域
    reg [PTR_WIDTH-1:0] rd_gray_s1, rd_gray_s2; // 读指针格雷码在 src 域的两级同步（判满用）
    reg [PTR_WIDTH-1:0] wr_gray_s1, wr_gray_s2; // 写指针格雷码在 dst 域的两级同步（判空用）

    // ------------------------------------------------------------------------
    // 满 / 空 / 水位状态
    // ------------------------------------------------------------------------
    // 满/空标志必须寄存在各自的时钟域内。
    // 不能用"下一写指针 -> full -> 写允许 -> 下一写指针"的组合表达式，
    // 否则会形成组合环；日志中的内部网络错误发生在此警告之后。
    reg full_reg;                              // 满标志（真值，src 域）：FIFO 一格都不剩
    reg almost_full_reg;                       // 差一格满（src 域）：对外当 src_full 提前反压
    reg empty_reg;                             // 空标志（真值，dst 域）：FIFO 一格都没有
    reg burst_active;                          // dst 侧突发进行中：=1 期间每拍吐一条命令
    reg [BURST_COUNT_WIDTH:0] burst_count;     // 突发剩余命令条数：从 128 数到 0
    reg [COMMAND_WIDTH-1:0] rd_data;           // 读出的命令寄存器（读延迟 1 拍后送出）
    reg rd_valid;                              // 输出命令有效脉冲（随 rd_data 延迟 1 拍）
    reg [31:0] burst_gap_count;

    // ------------------------------------------------------------------------
    // 组合逻辑：读写握手与指针推进
    // ------------------------------------------------------------------------
    wire wr_accept = src_wr_en && !full_reg;   // 本拍真的接收一条命令（有命令且没满）
    // 启动 burst 的一拍不读 RAM，预测空标志也不能提前推进指针。
    // dst_allow 只决定能否开始，不能逐字打断已启动的 SDRAM 突发。
    // 启动前攒齐整块，因此活动期间一定有 BURST_SIZE 个字可连续读取。
    wire rd_accept = burst_active;
    wire [PTR_WIDTH-1:0] wr_bin_next = wr_bin + wr_accept;   // 下一拍写指针：接收了才 +1
    wire [PTR_WIDTH-1:0] wr_gray_next = ((wr_bin + wr_accept) >> 1) ^ (wr_bin + wr_accept); // 下一拍写指针格雷码
    wire [PTR_WIDTH-1:0] rd_gray_next = ((rd_bin + rd_accept) >> 1) ^ (rd_bin + rd_accept); // 下一拍读指针格雷码

    // 满判断（src 域）：写指针格雷码与同步来的读指针格雷码比较。
    // 标准异步 FIFO 判满式：两者最高 2 位相反、其余位相同 ⇔ 写指针绕了整圈 = 满。
    wire full_next = wr_gray_next ==
                      {~rd_gray_s2[PTR_WIDTH-1:PTR_WIDTH-2],
                       rd_gray_s2[PTR_WIDTH-3:0]};
    // 空判断（dst 域）：读指针格雷码与同步来的写指针格雷码完全相同 ⇔ 空。
    wire empty_next = rd_gray_next == wr_gray_s2;
    // 有命令待发（dst 域）：同步来的写指针 != 本地读指针。
    // 突发启动条件之一：有货可吐才开始。
    wire block_pending = wr_gray_s2 != rd_gray;

    // 格雷码转二进制函数：src 域算"FIFO 里存了几个字"（水位）用。
    // 转换规则：最高位不变，其余位 = 已还原的高位二进制 ^ 本位格雷码。
    function [PTR_WIDTH-1:0] gray_to_bin;
        input [PTR_WIDTH-1:0] gray;
        integer i;
        begin
            gray_to_bin[PTR_WIDTH-1] = gray[PTR_WIDTH-1];
            for (i = PTR_WIDTH-2; i >= 0; i = i-1)
                gray_to_bin[i] = gray_to_bin[i+1] ^ gray[i];
        end
    endfunction
    // 下一拍 FIFO 已存字数（src 域视角）= 下一拍写指针 − 同步来的读指针。
    // 因为读指针是旧值，算出的水位偏保守（看起来更多），方向安全。
    wire [PTR_WIDTH-1:0] used_next = wr_bin_next - gray_to_bin(rd_gray_s2);
    // 只转换经过两级同步的 Gray 指针。这个水位偏少，适合安全判定整块就绪。
    wire [PTR_WIDTH-1:0] dst_level = gray_to_bin(wr_gray_s2) - rd_bin;

    // ------------------------------------------------------------------------
    // 对外输出
    // ------------------------------------------------------------------------
    // 对 loader 提前一格反压：它的 app_wr_en/data 是寄存输出，
    // 看到 src_full 后仍有一条已生成的命令在下一拍到达。
    // 内部用真正的 full_reg 拒绝溢出，允许这条在途命令占用最后一格。
    assign src_full = almost_full_reg;
    // 读出的 57 位命令拆回三组总线，直接送 SDRAM 控制器（寄存器输出，延迟 1 拍）
    assign dst_app_wr_en   = rd_valid;
    assign dst_app_wr_addr = rd_data[COMMAND_WIDTH-1:36];
    assign dst_app_wr_mask = rd_data[35:32];
    assign dst_app_wr_data = rd_data[31:0];
    // 空闲条件：FIFO 空 + 无待发命令 + 无突发在途 + 本拍无输出。
    // 顶层用它确认"写命令已全部交给 SDRAM"（loader 报完成只是命令都产生了）。
    assign dst_idle = empty_reg && !block_pending && !burst_active && !rd_valid &&
                      (burst_gap_count == 0);

    // =========================================================================
    // 写侧（src_clk 域）：接收命令、写 RAM、推进写指针、判满
    // =========================================================================
    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            // —— 复位：指针归零、清空标志、清报警 ——
            wr_bin <= 0; wr_gray <= 0;
            rd_gray_s1 <= 0; rd_gray_s2 <= 0;
            full_reg <= 1'b0;
            almost_full_reg <= 1'b0;
            src_overflow <= 1'b0;
        end else begin
            // 同步链：读指针格雷码两级打拍，消除亚稳态（对方域指针的唯一入口）
            rd_gray_s1 <= rd_gray;
            rd_gray_s2 <= rd_gray_s1;
            // 真溢出报警：full_reg 已经为 1 时还有命令来 → 这条命令被丢弃，
            // 报警置 1 并保持（正常流控下由 almost_full 提前反压，不该发生）
            if (src_wr_en && full_reg)
                src_overflow <= 1'b1;
            // 写 RAM：接收的命令按写指针写入（打包三组总线为一条 57 位命令）
            if (wr_accept)
                mem[wr_bin[FIFO_ADDR_WIDTH-1:0]] <= {src_wr_addr, src_wr_mask, src_wr_data};
            // 写指针推进：接收了才 +1
            wr_bin <= wr_bin + wr_accept;
            wr_gray <= wr_gray_next;
            // 满标志更新
            full_reg <= full_next;
            // 差一格满更新：下一拍水位 >= 511 时置 1（留 1 格给"在途命令"）
            almost_full_reg <= (used_next >= DEPTH-1);
        end
    end

    // =========================================================================
    // 读侧（dst_clk 域）：突发吐出命令、推进读指针、判空
    // =========================================================================
    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            // —— 复位：指针归零、无输出、空标志置 1、突发停止 ——
            rd_bin <= 0; rd_gray <= 0;
            wr_gray_s1 <= 0; wr_gray_s2 <= 0;
            rd_data <= 0; rd_valid <= 1'b0;
            empty_reg <= 1'b1;
            burst_active <= 1'b0; burst_count <= 0;
            burst_gap_count <= 0;
        end else begin
            // 同步链：写指针格雷码两级打拍（对方域指针的唯一入口）
            wr_gray_s1 <= wr_gray;
            wr_gray_s2 <= wr_gray_s1;
            // 输出命令有效默认拉低：rd_valid 是单拍脉冲
            rd_valid <= 1'b0;
            if (burst_gap_count != 0)
                burst_gap_count <= burst_gap_count - 1'b1;

            if (!burst_active && !rd_valid && burst_gap_count == 0 &&
                dst_allow && dst_level >= BURST_SIZE) begin
                // —— 完整块到齐且总线空闲才启动，禁止边到货边发送 ——
                // 注意：这一拍只置突发标志，不读 RAM（rd_accept 用的还是旧
                // burst_active=0），相当于"启动拍"占 1 拍不出数。
                burst_active <= 1'b1;
                burst_count <= BURST_SIZE;      // 目标：连吐 128 条命令
            end else if (rd_accept) begin
                // —— 突发进行中：每拍吐一条命令 ——
                rd_data <= mem[rd_bin[FIFO_ADDR_WIDTH-1:0]]; // 读 RAM（寄存器输出，下一拍才到 rd_data）
                rd_valid <= 1'b1;               // 下一拍 dst_app_wr_en=1，与 rd_data 同拍送出
                rd_bin <= rd_bin + 1'b1;        // 读指针 +1
                rd_gray <= rd_gray_next;
                if (burst_count == 1) begin
                    // 吐完最后一条：突发结束，等下一轮启动
                    burst_active <= 1'b0;
                    burst_count <= 0;
                    // 本拍读出的最后一字在下一拍才被控制器采样。
                    burst_gap_count <= BURST_GAP_CYCLES + 1;
                end else begin
                    // 剩余条数 −1（128 → 127 → … → 1，共吐 128 条）
                    burst_count <= burst_count - 1'b1;
                end
            end
            // 空标志更新（用同步来的写指针判断）
            empty_reg <= empty_next;
        end
    end
endmodule
