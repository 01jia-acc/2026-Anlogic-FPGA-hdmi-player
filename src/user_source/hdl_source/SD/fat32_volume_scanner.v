`timescale 1ns/1ps

// FAT32 卷扫描器：读取 BPB，定位根目录，并在根目录首个簇的全部扇区中
// 查找 PIC1.BMP~PIC4.BMP。文件系统参数和文件起始簇/大小在扫描结束后保持。
// 这里支持 MBR 分区和“直接从 LBA0 开始的 FAT32”两种常见格式。
module fat32_volume_scanner (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        scan_start,
    output wire        scan_busy,
    output reg         scan_done,
    output reg  [7:0]  fs_error,
    output reg  [31:0] partition_lba,
    output reg  [7:0]  sectors_per_cluster,
    output reg  [31:0] fat_start_lba,
    output reg  [31:0] data_start_lba,
    output reg  [31:0] max_cluster,
    output reg  [3:0]  resource_valid,
    output reg  [31:0] resource0_cluster,
    output reg  [31:0] resource1_cluster,
    output reg  [31:0] resource2_cluster,
    output reg  [31:0] resource3_cluster,
    output reg  [31:0] resource0_size,
    output reg  [31:0] resource1_size,
    output reg  [31:0] resource2_size,
    output reg  [31:0] resource3_size,
    output reg         sector_req,
    output reg  [31:0] sector_lba,
    input  wire        sector_busy,
    input  wire        sector_valid,
    input  wire [7:0]  sector_byte,
    input  wire [8:0]  sector_byte_index,
    input  wire        sector_done,
    input  wire [7:0]  sector_error
);
    localparam ERR_NONE             = 8'h00;
    localparam ERR_SECTOR           = 8'h10;
    localparam ERR_BOOT_SIGNATURE   = 8'h20;
    localparam ERR_UNSUPPORTED_BPB  = 8'h21;
    localparam ERR_RESOURCE_MISSING = 8'h22;

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

    reg [3:0] state;
    reg [7:0] sector_buffer [0:511];
    reg [31:0] total_sectors;
    reg [31:0] fat_sectors;
    reg [15:0] reserved_sectors;
    reg [31:0] root_cluster;
    reg [31:0] data_sector_count;
    reg [31:0] fat_span;
    reg [2:0]  spc_shift;
    reg [31:0] root_lba;
    reg [7:0]  root_sector_index;

    integer i;
    integer base;

    assign scan_busy = (state != ST_IDLE);

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

    function valid_spc;
        input [7:0] value;
        begin
            valid_spc = (value != 0) && ((value & (value - 1'b1)) == 0);
        end
    endfunction

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

    function entry_is_pic;
        input integer offset;
        input [1:0] number;
        begin
            entry_is_pic = (sector_buffer[offset+0] == "P") &&
                           (sector_buffer[offset+1] == "I") &&
                           (sector_buffer[offset+2] == "C") &&
                           (sector_buffer[offset+3] == ("1" + number)) &&
                           (sector_buffer[offset+4] == " ") &&
                           (sector_buffer[offset+5] == " ") &&
                           (sector_buffer[offset+6] == " ") &&
                           (sector_buffer[offset+7] == " ") &&
                           (sector_buffer[offset+8] == "B") &&
                           (sector_buffer[offset+9] == "M") &&
                           (sector_buffer[offset+10] == "P");
        end
    endfunction

    task finish_with_error;
        input [7:0] code;
        begin
            fs_error <= code;
            state <= ST_FINISH;
        end
    endtask

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
            resource_valid     <= 4'd0;
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
        end else begin
            scan_done <= 1'b0;
            sector_req <= 1'b0;

            if (sector_valid)
                sector_buffer[sector_byte_index] <= sector_byte;

            case (state)
                ST_IDLE: begin
                    if (scan_start) begin
                        fs_error <= ERR_NONE;
                        resource_valid <= 4'd0;
                        state <= ST_READ0_REQ;
                    end
                end

                ST_READ0_REQ: begin
                    if (!sector_busy) begin
                        sector_lba <= 32'd0;
                        sector_req <= 1'b1;
                        state <= ST_READ0_WAIT;
                    end
                end

                ST_READ0_WAIT: begin
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
                    if (!sector_busy) begin
                        sector_req <= 1'b1;
                        state <= ST_BOOT_WAIT;
                    end
                end

                ST_BOOT_WAIT: begin
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
                    if (total_sectors <= reserved_sectors + fat_span)
                        finish_with_error(ERR_UNSUPPORTED_BPB);
                    else begin
                        data_start_lba <= partition_lba + reserved_sectors + fat_span;
                        data_sector_count <= total_sectors - reserved_sectors - fat_span;
                        state <= ST_ROOT_INIT;
                    end
                end

                ST_ROOT_INIT: begin
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
                    if (!sector_busy) begin
                        sector_lba <= root_lba + root_sector_index;
                        sector_req <= 1'b1;
                        state <= ST_ROOT_WAIT;
                    end
                end

                ST_ROOT_WAIT: begin
                    if (sector_done) begin
                        if (sector_error != ERR_NONE)
                            finish_with_error(ERR_SECTOR);
                        else
                            state <= ST_ROOT_PARSE;
                    end
                end

                ST_ROOT_PARSE: begin
                    // 每个目录项 32 字节，每个扇区检查 16 项。跳过删除项、
                    // LFN、卷标和子目录，只接受固定的 8.3 文件名。
                    for (i = 0; i < 16; i = i + 1) begin
                        base = i * 32;
                        if (sector_buffer[base] != 8'h00 &&
                            sector_buffer[base] != 8'he5 &&
                            (sector_buffer[base+11] & 8'h0f) != 8'h0f &&
                            (sector_buffer[base+11] & 8'h10) == 8'h00) begin
                            if (entry_is_pic(base, 2'd0)) begin
                                resource0_cluster <= {sector_buffer[base+21], sector_buffer[base+20],
                                                       sector_buffer[base+27], sector_buffer[base+26]};
                                resource0_size <= le32(base+28);
                                resource_valid[0] <= 1'b1;
                            end else if (entry_is_pic(base, 2'd1)) begin
                                resource1_cluster <= {sector_buffer[base+21], sector_buffer[base+20],
                                                       sector_buffer[base+27], sector_buffer[base+26]};
                                resource1_size <= le32(base+28);
                                resource_valid[1] <= 1'b1;
                            end else if (entry_is_pic(base, 2'd2)) begin
                                resource2_cluster <= {sector_buffer[base+21], sector_buffer[base+20],
                                                       sector_buffer[base+27], sector_buffer[base+26]};
                                resource2_size <= le32(base+28);
                                resource_valid[2] <= 1'b1;
                            end else if (entry_is_pic(base, 2'd3)) begin
                                resource3_cluster <= {sector_buffer[base+21], sector_buffer[base+20],
                                                       sector_buffer[base+27], sector_buffer[base+26]};
                                resource3_size <= le32(base+28);
                                resource_valid[3] <= 1'b1;
                            end
                        end
                    end

                    if (root_sector_index + 1'b1 >= sectors_per_cluster)
                        state <= ST_ROOT_CHECK;
                    else begin
                        root_sector_index <= root_sector_index + 1'b1;
                        state <= ST_ROOT_REQ;
                    end
                end

                ST_ROOT_CHECK: begin
                    if (resource_valid != 4'hf)
                        fs_error <= ERR_RESOURCE_MISSING;
                    state <= ST_FINISH;
                end

                ST_FINISH: begin
                    scan_done <= 1'b1;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
