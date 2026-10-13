`timescale 1ns/1ps

// 按 FAT32 簇链顺序输出文件字节。数据簇可以不连续，文件大小决定输出
// 字节数，FAT 表只负责找到下一个簇；遍历计数和范围检查用于防止坏卡死循环。
//
// 阅读导航：
//   ST_DATA_REQ/WAIT：读取当前簇的当前扇区，并把 512 个字节直接变成文件流；
//   当前簇/扇区读完后，计算 FAT[current_cluster] 所在的 FAT 扇区与字节偏移；
//   ST_FAT_REQ/WAIT/PARSE：读 FAT 项、取出下一个簇，回到 ST_DATA_REQ。
//
// 文件大小是“输出多少字节”的权威来源，FAT 链是“下一个数据簇在哪里”的
// 地址来源。二者分工很重要：不能把 EOC 当作文件长度，也不能假定簇连续。
// 本模块逐字节输出，没有 FIFO；上游 decoder 必须能在每个 file_valid 周期接收。
module fat32_file_streamer #(
    // 用于限制坏 FAT 链的最大遍历次数，防止环链让硬件永久运行。
    parameter integer MAX_VISITED_CLUSTERS = 8192
) (
    // 文件流状态机的时钟、低有效异步复位。
    input  wire        clk,
    input  wire        rst_n,
    // 一拍启动事件和立即中止事件。
    input  wire        file_start,
    input  wire        file_abort,
    // 目录项给出的首簇和文件字节数。
    input  wire [31:0] first_cluster,
    input  wire [31:0] file_size,
    // 来自 BPB 的布局参数。
    input  wire [7:0]  sectors_per_cluster,
    input  wire [31:0] fat_start_lba,
    input  wire [31:0] data_start_lba,
    input  wire [31:0] max_cluster,
    // 文件事务正在运行；空闲时才可接收新的 file_start。
    output wire        file_busy,
    // 文件字节全部输出或发生错误后的单周期完成脉冲。
    output reg         file_done,
    // 当前拍 file_byte/file_byte_offset 有效的脉冲。
    output reg         file_valid,
    // 当前文件字节。
    output reg  [7:0]  file_byte,
    // 当前字节在文件中的绝对偏移，从 0 递增到 file_size-1。
    output reg  [31:0] file_byte_offset,
    // FAT/扇区/链错误码，0 表示没有错误。
    output reg  [7:0]  fs_error,
    // 给扇区适配层的单扇区请求。
    output reg         sector_req,
    // 请求意图先于注册的 sector_req，供共享通道先授予整扇区访问权。
    output wire        sector_pending,
    output reg  [31:0] sector_lba,
    input  wire        sector_busy,
    input  wire        sector_valid,
    input  wire [7:0]  sector_byte,
    input  wire [8:0]  sector_byte_index,
    input  wire        sector_done,
    input  wire [7:0]  sector_error
);
    // 本模块的错误码。EOC 只允许在文件最后一个有效字节已经输出后出现；
    // 如果提前遇到 EOC，说明目录项长度与 FAT 链不一致。
    localparam ERR_NONE              = 8'h00;
    localparam ERR_SECTOR            = 8'h10;
    localparam ERR_FAT_LOOP          = 8'h23;
    localparam ERR_CLUSTER_RANGE     = 8'h24;
    localparam ERR_EARLY_EOF         = 8'h25;
    localparam ERR_VISITED_EXHAUSTED = 8'h26;

    // 每一个状态都是“正在等待某种事件”的硬件阶段；REQ 发起扇区事务，
    // WAIT 接收字节并等待 done，PARSE 在扇区缓冲/寄存器准备好后计算下一步。
    localparam ST_IDLE      = 3'd0;
    localparam ST_DATA_REQ  = 3'd1;
    localparam ST_DATA_WAIT = 3'd2;
    localparam ST_FAT_REQ   = 3'd3;
    localparam ST_FAT_WAIT  = 3'd4;
    localparam ST_FAT_PARSE = 3'd5;
    localparam ST_FINISH    = 3'd6;

    localparam integer CHAIN_COUNT_WIDTH =
        (MAX_VISITED_CLUSTERS < 2) ? 1 : $clog2(MAX_VISITED_CLUSTERS + 1);

    // 主状态寄存器。
    reg [2:0] state;
    // 当前正在输出的 FAT32 数据簇号。
    reg [31:0] current_cluster;
    // 尚未输出的文件字节数；每个有效数据字节减一。
    reg [31:0] remaining_bytes;
    // 当前簇内正在读取的扇区编号 0~SPC-1。
    reg [7:0] sector_in_cluster;
    // 当前簇对应 FAT 项在一个 512 字节 FAT 扇区中的起始字节偏移。
    reg [8:0] fat_entry_offset;
    // FAT32 一个簇项占 4 字节；四个寄存器跨多个 valid 周期拼成完整项。
    reg [7:0] fat_byte0, fat_byte1, fat_byte2, fat_byte3;
    // 已走过的簇数，用于限制异常链长度。
    reg [CHAIN_COUNT_WIDTH-1:0] chain_count;
    // 当前数据扇区结束后是否需要验证“当前簇的 FAT 项确实为 EOC”。
    // 文件恰好在扇区/簇边界结束时，用这个标志补做最终链检查。
    reg final_chain_check;
    // 下一个待输出文件字节的偏移计数器。
    //
    // file_byte_offset 是与 file_valid/file_byte 配对输出的“当前字节偏移”，
    // 而不是内部的“下一个偏移”。由于这些信号都是寄存器，若直接写成
    // file_byte_offset <= file_byte_offset + 1'b1，实际第 0 个字节会被标记为
    // 偏移 1，导致 BMP 的 42 4D 文件头错位。此寄存器专门保存下一字节位置，
    // 输出时先把它送给 file_byte_offset，再自增。
    reg [31:0] next_file_byte_offset;

    // FAT32 只使用 FAT 项的低 28 位，高 4 位保留；EOC 的比较也在 28 位域中做。
    wire [27:0] fat_value = {fat_byte3[3:0], fat_byte2,
                              fat_byte1, fat_byte0};

    // 同样的状态观察值：只要不在 IDLE，就占用底层扇区通道。
    assign file_busy = (state != ST_IDLE);
    assign sector_pending = state == ST_DATA_REQ || state == ST_FAT_REQ;

    // 统一的错误收尾入口；任务展开后仍是当前 always 块中的寄存器赋值。
    task finish_with_error;
        input [7:0] code;
        begin
            fs_error <= code;
            state <= ST_FINISH;
        end
    endtask

    // 文件流时序控制中心。注意本 always 中的几个 if 与 case 在同一时钟拍
    // 并行计算；阅读时应先看“当前状态 + 当前输入事件”，再看寄存器下一值。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= ST_IDLE;
            file_done         <= 1'b0;
            file_valid        <= 1'b0;
            file_byte         <= 8'd0;
            file_byte_offset  <= 32'd0;
            next_file_byte_offset <= 32'd0;
            fs_error          <= ERR_NONE;
            sector_req        <= 1'b0;
            sector_lba        <= 32'd0;
            current_cluster   <= 32'd0;
            remaining_bytes   <= 32'd0;
            sector_in_cluster <= 8'd0;
            fat_entry_offset  <= 9'd0;
            fat_byte0         <= 8'd0;
            fat_byte1         <= 8'd0;
            fat_byte2         <= 8'd0;
            fat_byte3         <= 8'd0;
            chain_count       <= 0;
            final_chain_check <= 1'b0;
        end else if (file_abort) begin
            state            <= ST_IDLE;
            file_done        <= 1'b0;
            file_valid       <= 1'b0;
            sector_req       <= 1'b0;
            fs_error         <= ERR_NONE;
            remaining_bytes  <= 32'd0;
            file_byte_offset <= 32'd0;
            next_file_byte_offset <= 32'd0;
            final_chain_check<= 1'b0;
        end else begin
            file_done  <= 1'b0;
            file_valid <= 1'b0;
            sector_req <= 1'b0;

            // 数据扇区字节来一拍就输出一拍，不等待整个扇区；这是流式结构，
            // 因此 decoder 可以和 SD 读取重叠工作。
            if (state == ST_DATA_WAIT && sector_valid &&
                remaining_bytes != 0) begin
                file_valid       <= 1'b1;
                file_byte        <= sector_byte;
                // 当前输出字节使用当前计数值；计数器自增只用于下一字节。
                file_byte_offset <= next_file_byte_offset;
                next_file_byte_offset <= next_file_byte_offset + 1'b1;
                remaining_bytes  <= remaining_bytes - 1'b1;
            end

            // FAT 项可能位于当前 FAT 扇区任意 4 字节边界；只抓取所需四字节。
            if (state == ST_FAT_WAIT && sector_valid) begin
                if (sector_byte_index == fat_entry_offset)
                    fat_byte0 <= sector_byte;
                else if (sector_byte_index == fat_entry_offset + 1'b1)
                    fat_byte1 <= sector_byte;
                else if (sector_byte_index == fat_entry_offset + 2'd2)
                    fat_byte2 <= sector_byte;
                else if (sector_byte_index == fat_entry_offset + 2'd3)
                    fat_byte3 <= sector_byte;
            end

            case (state)
                ST_IDLE: begin
                    // 启动时锁存本次文件的运行参数。首簇地址立即换算为数据区 LBA，
                    // 但真正的 sector_req 要到 ST_DATA_REQ 且底层空闲时才发出。
                    if (file_start) begin
                        fs_error         <= ERR_NONE;
                        file_byte_offset <= 32'd0;
                        next_file_byte_offset <= 32'd0;
                        remaining_bytes  <= file_size;
                        sector_in_cluster<= 8'd0;
                        chain_count      <= 1;
                        final_chain_check<= 1'b0;
                        if (file_size == 0) begin
                            state <= ST_FINISH;
                        end else if (first_cluster < 2 ||
                                     first_cluster > max_cluster ||
                                     sectors_per_cluster == 0) begin
                            finish_with_error(ERR_CLUSTER_RANGE);
                        end else begin
                            current_cluster <= first_cluster;
                            sector_lba <= data_start_lba +
                                          (first_cluster - 2) * sectors_per_cluster;
                            state <= ST_DATA_REQ;
                        end
                    end
                end

                ST_DATA_REQ: begin
                    // 请求与“等待数据”分开，避免同一拍既发请求又误判旧的 valid/done。
                    if (!sector_busy) begin
                        sector_req <= 1'b1;
                        state <= ST_DATA_WAIT;
                    end
                end

                ST_DATA_WAIT: begin
                    // sector_valid 在这里形成文件字节流；sector_done 只负责决定
                    // 当前簇/扇区结束后的下一步，不代表本拍没有有效数据。
                    if (sector_done) begin
                        if (sector_error != ERR_NONE) begin
                            finish_with_error(ERR_SECTOR);
                        end else if (remaining_bytes == 0 ||
                                     (remaining_bytes == 1 && sector_valid)) begin
                            // 文件最后一个有效字节已在此前的 sector_valid
                            // 周期输出，此处仍验证当前簇确实以 EOC 结束。
                            final_chain_check <= 1'b1;
                            sector_in_cluster <= 8'd0;
                            fat_entry_offset <= {current_cluster[6:0], 2'b00};
                            sector_lba <= fat_start_lba + (current_cluster >> 7);
                            state <= ST_FAT_REQ;
                        end else if (sector_in_cluster + 1'b1 < sectors_per_cluster) begin
                            sector_in_cluster <= sector_in_cluster + 1'b1;
                            sector_lba <= sector_lba + 1'b1;
                            state <= ST_DATA_REQ;
                        end else begin
                            sector_in_cluster <= 8'd0;
                            fat_entry_offset <= {current_cluster[6:0], 2'b00};
                            sector_lba <= fat_start_lba + (current_cluster >> 7);
                            state <= ST_FAT_REQ;
                        end
                    end
                end

                ST_FAT_REQ: begin
                    // FAT 项和普通数据一样通过单扇区适配器读取。
                    if (!sector_busy) begin
                        sector_req <= 1'b1;
                        state <= ST_FAT_WAIT;
                    end
                end

                ST_FAT_WAIT: begin
                    // 先接收 FAT 项所处扇区的四字节，直到整个扇区 done。
                    if (sector_done) begin
                        if (sector_error != ERR_NONE)
                            finish_with_error(ERR_SECTOR);
                        else
                            state <= ST_FAT_PARSE;
                    end
                end

                ST_FAT_PARSE: begin
                    // fat_value 是当前簇的“后继簇号”。EOC、越界、环链和正常
                    // 后继分别走不同分支；这就是 FAT 链遍历的安全边界。
                    if (final_chain_check) begin
                        final_chain_check <= 1'b0;
                        if (fat_value >= 28'hffffff8)
                            state <= ST_FINISH;
                        else if (fat_value < 2 || fat_value > max_cluster)
                            finish_with_error(ERR_CLUSTER_RANGE);
                        else
                            finish_with_error(ERR_FAT_LOOP);
                    end else if (fat_value >= 28'hffffff8) begin
                        finish_with_error(ERR_EARLY_EOF);
                    end else if (fat_value < 2 || fat_value > max_cluster) begin
                        finish_with_error(ERR_CLUSTER_RANGE);
                    end else if (chain_count >= MAX_VISITED_CLUSTERS) begin
                        finish_with_error(ERR_VISITED_EXHAUSTED);
                    end else if (fat_value == current_cluster) begin
                        finish_with_error(ERR_FAT_LOOP);
                    end else begin
                        chain_count <= chain_count + 1'b1;
                        current_cluster <= {4'd0, fat_value};
                        sector_lba <= data_start_lba +
                                      ({4'd0, fat_value} - 32'd2) *
                                      {24'd0, sectors_per_cluster};
                        state <= ST_DATA_REQ;
                    end
                end

                ST_FINISH: begin
                    // 完成脉冲只保持一拍，下一拍回到空闲。
                    file_done <= 1'b1;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
