`timescale 1ns/1ps

// FAT32 根目录扫描器：支持 PIC1~PIC16.BMP 和 AUDIO1~AUDIO16.PCM。
// 支持 MBR/无分区卷，沿根目录 FAT 簇链查找，忽略删除项、目录、卷标和长名项。
// 资源表按文件编号排列，位 0 对应 PIC1；旧四槽输出保留用于诊断/兼容。
module fat32_volume_scanner (
    // 扫描器自己的时钟和低有效异步复位；外部 source 用 ~rst 连接。
    input  wire        clk,
    input  wire        rst_n,
    // 启动脉冲。扫描器空闲时接受，完成后通过 scan_done 脉冲报告。
    input  wire        scan_start,
    // state != ST_IDLE 的组合状态指示。
    output wire        scan_busy,
    // 扫描结束的单周期脉冲；结果寄存器在该脉冲时已经更新。
    output reg         scan_done,
    // 0 表示成功；非 0 表示扇区、签名、BPB 或资源缺失错误。
    output reg  [7:0]  fs_error,
    // 分区起始 LBA；输出给上层保存/观察，计算时也作为 BPB 的基准。
    output reg  [31:0] partition_lba,
    // 一个簇包含多少个 512 字节扇区，必须是 2 的幂。
    output reg  [7:0]  sectors_per_cluster,
    // FAT 区第一个扇区的 LBA。
    output reg  [31:0] fat_start_lba,
    // 数据区第一个扇区的 LBA；簇 2 就从这里开始。
    output reg  [31:0] data_start_lba,
    // 根据卷大小估算的最大合法簇号，用于后续防止坏链越界。
    output reg  [31:0] max_cluster,
    // 16个资源槽是否找到对应文件；bit0~bit15 对应 PIC1~PIC16。
    output reg  [15:0] resource_valid,
    output reg [511:0] resource_clusters,
    output reg [511:0] resource_sizes,
    // 四个文件的首簇。
    output reg  [31:0] resource0_cluster,
    output reg  [31:0] resource1_cluster,
    output reg  [31:0] resource2_cluster,
    output reg  [31:0] resource3_cluster,
    // 四个文件的目录项文件大小，单位为字节。
    output reg  [31:0] resource0_size,
    output reg  [31:0] resource1_size,
    output reg  [31:0] resource2_size,
    output reg  [31:0] resource3_size,
    // 按文件名编号保存，bit0/低32位对应 AUDIO1.PCM。
    output reg  [15:0] audio_resource_valid,
    output reg [511:0] audio_cluster,
    output reg [511:0] audio_size,
    // 调试观察口：只把扫描器内部状态照出，不参与控制逻辑。
    output wire [3:0]  debug_state,
    output wire [15:0] debug_sector_done_count,
    output wire [15:0] debug_resource_valid,
    output wire [31:0] debug_root_crc,
    output reg [15:0] debug_root_bytes,
    // 给 sd_sector_adapter 的单扇区请求接口。
    output reg         sector_req,
    output reg  [31:0] sector_lba,
    // 来自扇区适配层的事务状态和字节流。
    input  wire        sector_busy,
    input  wire        sector_valid,
    input  wire [7:0]  sector_byte,
    input  wire [8:0]  sector_byte_index,
    input  wire        sector_done,
    input  wire [7:0]  sector_error
);
    // 扫描错误码按模块分组，便于数码管/UART 日志定位错误发生在哪一层。
    localparam ERR_NONE             = 8'h00;
    localparam ERR_SECTOR           = 8'h10;
    localparam ERR_BOOT_SIGNATURE   = 8'h20;
    localparam ERR_UNSUPPORTED_BPB  = 8'h21;
    localparam ERR_RESOURCE_MISSING = 8'h22;

    // 状态机不是“软件函数调用”，而是一组触发器保存当前阶段；每个状态
    // 通常至少占用一个或多个时钟，等待外部扇区事务完成后再进入下一阶段。
    localparam ST_IDLE        = 4'd0;
    localparam ST_READ0_REQ   = 4'd1;
    localparam ST_READ0_WAIT  = 4'd2;
    localparam ST_BOOT_PARSE  = 4'd3;
    localparam ST_BOOT_REQ    = 4'd4;
    localparam ST_BOOT_WAIT   = 4'd5;
    localparam ST_LAYOUT      = 4'd6;
    localparam ST_ROOT_INIT   = 4'd7;
    localparam ST_ROOT_REQ    = 4'd8;
    localparam ST_ROOT_WAIT   = 4'd9;
    localparam ST_ROOT_PARSE  = 4'd10;
    localparam ST_ROOT_CHECK  = 4'd11;
    localparam ST_FINISH      = 4'd12;
    localparam ST_ROOT_FAT_REQ = 4'd13;
    localparam ST_ROOT_FAT_WAIT = 4'd14;
    localparam ST_ROOT_FAT_PARSE = 4'd15;

    // 主控制状态寄存器；case(state) 是当前扫描事务的唯一控制中心。
    reg [3:0] state;
    // 512 字节扇区缓冲。接收期间按 sector_byte_index 写入，解析状态中读取。
    reg [7:0] sector_buffer [0:511];
    // BPB 中的总扇区数和每个 FAT 占用的扇区数。
    reg [31:0] total_sectors;
    reg [31:0] fat_sectors;
    // BPB 保留区长度，通常包含引导扇区以及其他保留扇区。
    reg [15:0] reserved_sectors;
    // 根目录起始簇号；FAT32 根目录不是固定位置，而是普通簇链。
    reg [31:0] root_cluster;
    // 数据区总扇区数；用于估算合法簇号上限。
    reg [31:0] data_sector_count;
    // FAT 区总跨度。FAT32 常见有 2 份 FAT，因此可能是 fat_sectors*2。
    reg [31:0] fat_span;
    // sectors_per_cluster 的 log2。簇转扇区时用移位代替乘法。
    reg [2:0]  spc_shift;
    // 当前根目录簇起始 LBA。
    reg [31:0] root_lba;
    // 当前正在读取根目录簇的第几个扇区。
    reg [7:0]  root_sector_index;

    // 当前根目录项的流式解析寄存器。根目录一个扇区恰好包含 16 个 32 字节
    // 项，且 512 能被 32 整除，所以每个目录项都不会跨扇区。
    reg [7:0] dir_first_byte;
    reg [7:0] dir_attr;
    // Parse the common prefix/extension once, then decode the numeric slot.
    // Avoid 16 feedback comparators per name and overlapping valid-bit writes.
    reg dir_pic_match;
    reg [3:0] dir_pic_slot, dir_audio_slot;
    integer name_slot;
    reg [31:0] root_next_cluster;
    reg [31:0] root_clusters_visited;
    reg dir_audio_match;
    reg directory_ended;
    reg [7:0] dir_cluster_31_24;
    reg [7:0] dir_cluster_23_16;
    reg [7:0] dir_cluster_15_8;
    reg [7:0] dir_cluster_7_0;
    reg [7:0] dir_size_31_24;
    reg [7:0] dir_size_23_16;
    reg [7:0] dir_size_15_8;
    reg [7:0] dir_size_7_0;
    // 先锁存完整目录项，再用寄存过的单槽写使能发布资源表。
    // 避免最后一个 SD 字节、属性判断、编号比较和宽表写入挤在同一拍。
    reg [31:0] entry_cluster, entry_size;
    reg [15:0] entry_pic_write, entry_audio_write;
    // 已完整结束的扇区数。把 sector_done 单拍事件变成可被 UART 采样的进度值。
    reg [15:0] sector_done_count;
    reg [31:0] root_crc;

    // 空闲是唯一可接受 scan_start 的状态；其余状态都属于同一笔扫描事务。
    assign scan_busy = (state != ST_IDLE);
    assign debug_state = state;
    assign debug_sector_done_count = sector_done_count;
    assign debug_resource_valid = resource_valid;
    assign debug_root_crc = ~root_crc;

    // Reflected CRC-32/ISO-HDLC of all received root-directory bytes.
    // Observation only: no influence on parsing, requests or file selection.
    function [31:0] crc32_byte;
        input [31:0] previous;
        input [7:0] value;
        reg [31:0] work;
        integer bit_no;
        begin
            work = previous ^ {24'd0,value};
            for(bit_no=0;bit_no<8;bit_no=bit_no+1)
                work = (work >> 1) ^ (work[0] ? 32'hedb88320 : 32'd0);
            crc32_byte = work;
        end
    endfunction

    // FAT32/BMP 等磁盘格式使用 little-endian；FPGA 总线按高位在左表示，
    // 所以要把低地址字节放到结果的低位。函数只是组合取数，不保存状态。
    function [15:0] le16;
        input integer offset;
        begin
            le16 = {sector_buffer[offset+1], sector_buffer[offset]};
        end
    endfunction

    function [31:0] le32;
        input integer offset;
        begin
            le32 = {sector_buffer[offset+3], sector_buffer[offset+2],
                    sector_buffer[offset+1], sector_buffer[offset]};
        end
    endfunction

    // FAT32 规范要求每簇扇区数为非零 2 的幂，便于用移位完成簇地址计算。
    function valid_spc;
        input [7:0] value;
        begin
            valid_spc = (value != 0) && ((value & (value - 1'b1)) == 0);
        end
    endfunction

    // 把 1/2/4/.../128 转换成对应的 log2 值。
    function [2:0] get_spc_shift;
        input [7:0] value;
        begin
            case (value)
                8'd1:   get_spc_shift = 3'd0;
                8'd2:   get_spc_shift = 3'd1;
                8'd4:   get_spc_shift = 3'd2;
                8'd8:   get_spc_shift = 3'd3;
                8'd16:  get_spc_shift = 3'd4;
                8'd32:  get_spc_shift = 3'd5;
                8'd64:  get_spc_shift = 3'd6;
                8'd128: get_spc_shift = 3'd7;
                default:get_spc_shift = 3'd0;
            endcase
        end
    endfunction

    // task 只是在当前时序块中复用两次非阻塞赋值；它不代表软件子程序，
    // 综合后仍然只是 state/fs_error 的普通寄存器赋值。
    task finish_with_error;
        input [7:0] code;
        begin
            fs_error <= code;
            state <= ST_FINISH;
        end
    endtask

    // 扫描器的时序控制中心。复位后回到空闲；正常工作每拍：
    // ①收集扇区字节；②按当前状态推进一次事务；③生成请求/完成脉冲。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state              <= ST_IDLE;
            scan_done          <= 1'b0;
            fs_error           <= ERR_NONE;
            partition_lba      <= 32'd0;
            sectors_per_cluster<= 8'd0;
            fat_start_lba      <= 32'd0;
            data_start_lba     <= 32'd0;
            max_cluster        <= 32'd0;
            resource_valid     <= 16'd0;
            resource_clusters <= 0;
            resource_sizes <= 0;
            entry_cluster <= 0; entry_size <= 0;
            entry_pic_write <= 0; entry_audio_write <= 0;
            root_next_cluster <= 0;
            root_clusters_visited <= 0;
            audio_resource_valid <= 1'b0;
            audio_cluster <= 0;
            audio_size <= 0;
            dir_audio_match <= 0;
            dir_pic_match <= 0;
            dir_pic_slot <= 0;
            dir_audio_slot <= 0;
            directory_ended <= 0;
            resource0_cluster  <= 32'd0;
            resource1_cluster  <= 32'd0;
            resource2_cluster  <= 32'd0;
            resource3_cluster  <= 32'd0;
            resource0_size     <= 32'd0;
            resource1_size     <= 32'd0;
            resource2_size     <= 32'd0;
            resource3_size     <= 32'd0;
            sector_req         <= 1'b0;
            sector_lba         <= 32'd0;
            total_sectors      <= 32'd0;
            fat_sectors        <= 32'd0;
            reserved_sectors   <= 16'd0;
            root_cluster       <= 32'd0;
            data_sector_count  <= 32'd0;
            fat_span           <= 32'd0;
            spc_shift          <= 3'd0;
            root_lba           <= 32'd0;
            root_sector_index  <= 8'd0;
            dir_first_byte     <= 8'd0;
            dir_attr           <= 8'd0;
            dir_cluster_31_24  <= 8'd0;
            dir_cluster_23_16  <= 8'd0;
            dir_cluster_15_8   <= 8'd0;
            dir_cluster_7_0    <= 8'd0;
            dir_size_31_24     <= 8'd0;
            dir_size_23_16     <= 8'd0;
            dir_size_15_8      <= 8'd0;
            dir_size_7_0       <= 8'd0;
            sector_done_count  <= 16'd0;
            root_crc <= 32'hffffffff;
            debug_root_bytes <= 0;
        end else begin
            scan_done <= 1'b0;
            sector_req <= 1'b0;
            entry_pic_write <= 0;
            entry_audio_write <= 0;
            for (name_slot=0; name_slot<16; name_slot=name_slot+1) begin
                if(entry_pic_write[name_slot]) begin
                    resource_clusters[name_slot*32 +:32] <= entry_cluster;
                    resource_sizes[name_slot*32 +:32] <= entry_size;
                    resource_valid[name_slot] <= 1'b1;
                end
                if(entry_audio_write[name_slot]) begin
                    audio_cluster[name_slot*32 +:32] <= entry_cluster;
                    audio_size[name_slot*32 +:32] <= entry_size;
                    audio_resource_valid[name_slot] <= 1'b1;
                end
            end
            if(entry_pic_write[0]) begin resource0_cluster<=entry_cluster; resource0_size<=entry_size; end
            if(entry_pic_write[1]) begin resource1_cluster<=entry_cluster; resource1_size<=entry_size; end
            if(entry_pic_write[2]) begin resource2_cluster<=entry_cluster; resource2_size<=entry_size; end
            if(entry_pic_write[3]) begin resource3_cluster<=entry_cluster; resource3_size<=entry_size; end
            if(state == ST_ROOT_WAIT && sector_valid) begin
                root_crc <= crc32_byte(root_crc, sector_byte);
                debug_root_bytes <= debug_root_bytes + 1'b1;
            end

            // sector_done 只有一个时钟宽度；累计后即使 UART 没采到那个瞬间，
            // 仍可判断扫描是否真的完成过扇区。计数到满后饱和，避免回卷。
            if (sector_done && sector_done_count != 16'hffff)
                sector_done_count <= sector_done_count + 1'b1;

            // 适配器保证 sector_byte_index 在 0~511 且与 sector_valid 同拍有效。
            // BPB/MBR 阶段仍缓存一个扇区，因为这些字段需要在 sector_done 后
            // 统一解析；根目录阶段改为下面的实时解析，不再写入无用的缓存。
            if (sector_valid &&
                (state == ST_READ0_WAIT || state == ST_BOOT_WAIT))
                sector_buffer[sector_byte_index] <= sector_byte;

            case (state)
                ST_IDLE: begin
                    // scan_start 是启动事件；清除上一轮的资源有效位和错误码。
                    if (scan_start) begin
                        fs_error <= ERR_NONE;
                        resource_valid <= 16'd0;
                        resource_clusters <= 0;
                        resource_sizes <= 0;
                        root_clusters_visited <= 1;
                        audio_resource_valid <= 1'b0;
                        audio_cluster <= 0;
                        audio_size <= 0;
                        directory_ended <= 0;
                        sector_done_count <= 16'd0;
                        root_crc <= 32'hffffffff;
                        debug_root_bytes <= 0;
                        state <= ST_READ0_REQ;
                    end
                end

                ST_READ0_REQ: begin
                    // 首先读 LBA0。只有底层空闲才发请求，防止请求脉冲被吞掉。
                    if (!sector_busy) begin
                        sector_lba <= 32'd0;
                        sector_req <= 1'b1;
                        state <= ST_READ0_WAIT;
                    end
                end

                ST_READ0_WAIT: begin
                    // 等待一个扇区完整到达后，判断它是直接 FAT32 引导扇区还是 MBR。
                    if (sector_done) begin
                        if (sector_error != ERR_NONE)
                            finish_with_error(ERR_SECTOR);
                        else if (sector_buffer[510] != 8'h55 ||
                                 sector_buffer[511] != 8'haa)
                            finish_with_error(ERR_BOOT_SIGNATURE);
                        else if (sector_buffer[0] == 8'heb ||
                                 sector_buffer[0] == 8'he9) begin
                            partition_lba <= 32'd0;
                            state <= ST_BOOT_PARSE;
                        end else if (sector_buffer[450] == 8'h0b ||
                                     sector_buffer[450] == 8'h0c) begin
                            partition_lba <= le32(454);
                            sector_lba <= le32(454);
                            state <= ST_BOOT_REQ;
                        end else begin
                            finish_with_error(ERR_UNSUPPORTED_BPB);
                        end
                    end
                end

                ST_BOOT_REQ: begin
                    // MBR 情况下 sector_lba 已被设置为分区起始 LBA。
                    if (!sector_busy) begin
                        sector_req <= 1'b1;
                        state <= ST_BOOT_WAIT;
                    end
                end

                ST_BOOT_WAIT: begin
                    // 等待分区引导扇区；后续 BPB 字段均以此扇区为基准解析。
                    if (sector_done) begin
                        if (sector_error != ERR_NONE)
                            finish_with_error(ERR_SECTOR);
                        else if (sector_buffer[510] != 8'h55 ||
                                 sector_buffer[511] != 8'haa)
                            finish_with_error(ERR_BOOT_SIGNATURE);
                        else
                            state <= ST_BOOT_PARSE;
                    end
                end

                ST_BOOT_PARSE: begin
                    // BPB 偏移：11 字节/扇区，13 每簇扇区，14 保留区，16 FAT 数，
                    // 32 总扇区，36 每 FAT 扇区，44 根目录首簇。所有字段是小端。
                    if (le16(11) != 16'd512 || !valid_spc(sector_buffer[13]) ||
                        le16(14) == 0 ||
                        (sector_buffer[16] != 1 && sector_buffer[16] != 2) ||
                        le16(17) != 0 || le32(32) == 0 ||
                        le32(36) == 0 || le32(44) < 2) begin
                        finish_with_error(ERR_UNSUPPORTED_BPB);
                    end else begin
                        sectors_per_cluster <= sector_buffer[13];
                        reserved_sectors <= le16(14);
                        total_sectors <= le32(32);
                        fat_sectors <= le32(36);
                        root_cluster <= le32(44);
                        spc_shift <= get_spc_shift(sector_buffer[13]);
                        fat_span <= (sector_buffer[16] == 2) ?
                                    (le32(36) << 1) : le32(36);
                        fat_start_lba <= partition_lba + le16(14);
                        state <= ST_LAYOUT;
                    end
                end

                ST_LAYOUT: begin
                    // 数据区 = 分区起点 + 保留区 + 所有 FAT 区；FAT32 簇 2 映射到
                    // data_start_lba，簇 N 映射到 data_start+(N-2)*SPC。
                    if (total_sectors <= reserved_sectors + fat_span)
                        finish_with_error(ERR_UNSUPPORTED_BPB);
                    else begin
                        data_start_lba <= partition_lba + reserved_sectors + fat_span;
                        data_sector_count <= total_sectors - reserved_sectors - fat_span;
                        state <= ST_ROOT_INIT;
                    end
                end

                ST_ROOT_INIT: begin
                    // 根目录首簇位置；后续簇通过FAT项继续遍历。
                    max_cluster <= (data_sector_count >> spc_shift) + 1'b1;
                    if (root_cluster < 2 ||
                        root_cluster > ((data_sector_count >> spc_shift) + 1'b1)) begin
                        finish_with_error(ERR_UNSUPPORTED_BPB);
                    end else begin
                        root_lba <= data_start_lba +
                                    ((root_cluster - 2) << spc_shift);
                        root_sector_index <= 8'd0;
                        state <= ST_ROOT_REQ;
                    end
                end

                ST_ROOT_REQ: begin
                    // 逐扇区读取根目录簇；root_sector_index 在解析后递增。
                    if (!sector_busy) begin
                        sector_lba <= root_lba + root_sector_index;
                        sector_req <= 1'b1;
                        state <= ST_ROOT_WAIT;
                    end
                end

                ST_ROOT_WAIT: begin
                    // 根目录不再等待整个扇区后展开 16 项比较，而是在字节到达时
                    // 逐项收集：文件名使用类型标志和数字编号，属性/首簇/大小用少量
                    // 寄存器保存。这样把大规模组合多路选择改成 512 个时钟的
                    // 小逻辑重复使用，显著降低 LUT 和布线拥塞。
                    if (sector_valid) begin
                        case (sector_byte_index[4:0])
                            5'd0: begin
                                dir_first_byte <= sector_byte;
                                dir_audio_match <= sector_byte == "A";
                                dir_pic_match <= sector_byte == "P";
                                dir_pic_slot <= 0;
                                dir_audio_slot <= 0;
                                if (sector_byte == 0) directory_ended <= 1'b1;
                            end
                            5'd1: begin
                                dir_pic_match <= dir_pic_match && sector_byte == "I";
                                dir_audio_match <= dir_audio_match && sector_byte == "U";
                            end
                            5'd2: begin
                                dir_pic_match <= dir_pic_match && sector_byte == "C";
                                dir_audio_match <= dir_audio_match && sector_byte == "D";
                            end
                            5'd3: begin
                                dir_pic_match <= dir_pic_match && sector_byte >= "1" && sector_byte <= "9";
                                dir_pic_slot <= sector_byte[3:0] - 4'd1;
                                dir_audio_match <= dir_audio_match && sector_byte == "I";
                            end
                            5'd4: begin
                                if(sector_byte != " ") begin
                                    dir_pic_match <= dir_pic_match && dir_pic_slot == 0 &&
                                                     sector_byte >= "0" && sector_byte <= "6";
                                    dir_pic_slot <= 4'd9 + sector_byte[3:0];
                                end
                                dir_audio_match <= dir_audio_match && sector_byte == "O";
                            end
                            5'd5: begin
                                dir_pic_match <= dir_pic_match && sector_byte == " ";
                                dir_audio_match <= dir_audio_match && sector_byte >= "1" && sector_byte <= "9";
                                dir_audio_slot <= sector_byte[3:0] - 4'd1;
                            end
                            5'd6: begin
                                dir_pic_match <= dir_pic_match && sector_byte == " ";
                                if(sector_byte != " ") begin
                                    dir_audio_match <= dir_audio_match && dir_audio_slot == 0 &&
                                                       sector_byte >= "0" && sector_byte <= "6";
                                    dir_audio_slot <= 4'd9 + sector_byte[3:0];
                                end
                            end
                            5'd7: begin
                                dir_pic_match <= dir_pic_match && sector_byte == " ";
                                dir_audio_match <= dir_audio_match && sector_byte == " ";
                            end
                            5'd8: begin
                                dir_pic_match <= dir_pic_match && sector_byte == "B";
                                dir_audio_match <= dir_audio_match && sector_byte == "P";
                            end
                            5'd9: begin
                                dir_pic_match <= dir_pic_match && sector_byte == "M";
                                dir_audio_match <= dir_audio_match && sector_byte == "C";
                            end
                            5'd10: begin
                                dir_pic_match <= dir_pic_match && sector_byte == "P";
                                dir_audio_match <= dir_audio_match && sector_byte == "M";
                            end
                            5'd11: dir_attr <= sector_byte;
                            5'd20: dir_cluster_23_16 <= sector_byte;
                            5'd21: dir_cluster_31_24 <= sector_byte;
                            5'd26: dir_cluster_7_0  <= sector_byte;
                            5'd27: dir_cluster_15_8 <= sector_byte;
                            5'd28: dir_size_7_0      <= sector_byte;
                            5'd29: dir_size_15_8     <= sector_byte;
                            5'd30: dir_size_23_16    <= sector_byte;
                            5'd31: begin
                                dir_size_31_24 <= sector_byte;
                                entry_cluster <= {dir_cluster_31_24, dir_cluster_23_16,
                                                  dir_cluster_15_8, dir_cluster_7_0};
                                entry_size <= {sector_byte, dir_size_23_16, dir_size_15_8, dir_size_7_0};

                                // 目录项在本拍已经收齐。最后一个文件大小字节
                                // 直接使用 sector_byte，避免非阻塞赋值尚未更新
                                // dir_size_31_24 的时序陷阱。
                                if (!directory_ended && dir_first_byte != 8'h00 &&
                                    dir_first_byte != 8'he5 &&
                                    (dir_attr & 8'h0f) != 8'h0f &&
                                    (dir_attr & 8'h18) == 8'h00) begin
                                    for (name_slot=0; name_slot<16; name_slot=name_slot+1) begin
                                        if (dir_audio_match && dir_audio_slot == name_slot && !audio_resource_valid[name_slot]) begin
                                            entry_audio_write[name_slot] <= 1'b1;
                                        end
                                        if (dir_pic_match && dir_pic_slot == name_slot && !resource_valid[name_slot]) begin
                                            entry_pic_write[name_slot] <= 1'b1;
                                        end
                                    end
                                end
                            end
                            default: begin
                            end
                        endcase
                    end

                    if (sector_done) begin
                        if (sector_error != ERR_NONE)
                            finish_with_error(ERR_SECTOR);
                        else
                            state <= ST_ROOT_PARSE;
                    end
                end

                ST_ROOT_PARSE: begin
                    // 本状态现在只负责根目录簇内的扇区计数；目录项已经在
                    // ST_ROOT_WAIT 接收期间解析完成。
                    if (directory_ended)
                        state <= ST_ROOT_CHECK;
                    else if (root_sector_index + 1'b1 >= sectors_per_cluster)
                        state <= ST_ROOT_FAT_REQ;
                    else begin
                        root_sector_index <= root_sector_index + 1'b1;
                        state <= ST_ROOT_REQ;
                    end
                end

                // 根目录也沿 FAT 簇链遍历；限制 1024 簇，避免坏链循环。
                ST_ROOT_FAT_REQ: begin
                    if (!sector_busy) begin
                        sector_lba <= fat_start_lba + (root_cluster >> 7);
                        sector_req <= 1'b1;
                        root_next_cluster <= 0;
                        state <= ST_ROOT_FAT_WAIT;
                    end
                end
                ST_ROOT_FAT_WAIT: begin
                    if (sector_valid && sector_byte_index[8:2] == root_cluster[6:0]) begin
                        case (sector_byte_index[1:0])
                            0: root_next_cluster[7:0] <= sector_byte;
                            1: root_next_cluster[15:8] <= sector_byte;
                            2: root_next_cluster[23:16] <= sector_byte;
                            3: root_next_cluster[31:24] <= {4'd0, sector_byte[3:0]};
                        endcase
                    end
                    if (sector_done) begin
                        if (sector_error != 0) finish_with_error(ERR_SECTOR);
                        else state <= ST_ROOT_FAT_PARSE;
                    end
                end
                ST_ROOT_FAT_PARSE: begin
                    // 下一拍检查，兼容 FAT 项末字节与 sector_done 同拍到达。
                    if (root_next_cluster >= 32'h0ffffff8) state <= ST_ROOT_CHECK;
                    else if (root_next_cluster < 2 || root_next_cluster > max_cluster ||
                             root_next_cluster == root_cluster || root_clusters_visited >= 1024)
                        finish_with_error(8'h23);
                    else begin
                        root_cluster <= root_next_cluster;
                        root_clusters_visited <= root_clusters_visited + 1'b1;
                        root_lba <= data_start_lba + ((root_next_cluster-2) << spc_shift);
                        root_sector_index <= 0;
                        state <= ST_ROOT_REQ;
                    end
                end

                ST_ROOT_CHECK: begin
                    // 至少一张图片即可播放；缺号由调度器跳过。
                    if (resource_valid == 0)
                        fs_error <= ERR_RESOURCE_MISSING;
                    state <= ST_FINISH;
                end

                ST_FINISH: begin
                    // 产生一个周期的 scan_done；下一拍回空闲，允许重新扫描。
                    scan_done <= 1'b1;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
