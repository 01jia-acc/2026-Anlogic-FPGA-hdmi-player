`timescale 1ns/1ps

// 按 FAT32 簇链顺序输出文件字节。数据簇可以不连续，文件大小决定输出
// 字节数，FAT 表只负责找到下一个簇；遍历计数和范围检查用于防止坏卡死循环。
module fat32_file_streamer #(
    parameter integer MAX_VISITED_CLUSTERS = 8192
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        file_start,
    input  wire        file_abort,
    input  wire [31:0] first_cluster,
    input  wire [31:0] file_size,
    input  wire [7:0]  sectors_per_cluster,
    input  wire [31:0] fat_start_lba,
    input  wire [31:0] data_start_lba,
    input  wire [31:0] max_cluster,
    output wire        file_busy,
    output reg         file_done,
    output reg         file_valid,
    output reg  [7:0]  file_byte,
    output reg  [31:0] file_byte_offset,
    output reg  [7:0]  fs_error,
    output reg         sector_req,
    output reg  [31:0] sector_lba,
    input  wire        sector_busy,
    input  wire        sector_valid,
    input  wire [7:0]  sector_byte,
    input  wire [8:0]  sector_byte_index,
    input  wire        sector_done,
    input  wire [7:0]  sector_error
);
    localparam ERR_NONE              = 8'h00;
    localparam ERR_SECTOR            = 8'h10;
    localparam ERR_FAT_LOOP          = 8'h23;
    localparam ERR_CLUSTER_RANGE     = 8'h24;
    localparam ERR_EARLY_EOF         = 8'h25;
    localparam ERR_VISITED_EXHAUSTED = 8'h26;

    localparam ST_IDLE      = 3'd0;
    localparam ST_DATA_REQ  = 3'd1;
    localparam ST_DATA_WAIT = 3'd2;
    localparam ST_FAT_REQ   = 3'd3;
    localparam ST_FAT_WAIT  = 3'd4;
    localparam ST_FAT_PARSE = 3'd5;
    localparam ST_FINISH    = 3'd6;

    localparam integer CHAIN_COUNT_WIDTH =
        (MAX_VISITED_CLUSTERS < 2) ? 1 : $clog2(MAX_VISITED_CLUSTERS + 1);

    reg [2:0] state;
    reg [31:0] current_cluster;
    reg [31:0] remaining_bytes;
    reg [7:0] sector_in_cluster;
    reg [8:0] fat_entry_offset;
    reg [7:0] fat_byte0, fat_byte1, fat_byte2, fat_byte3;
    reg [CHAIN_COUNT_WIDTH-1:0] chain_count;
    reg final_chain_check;

    wire [27:0] fat_value = {fat_byte3[3:0], fat_byte2,
                              fat_byte1, fat_byte0};

    assign file_busy = (state != ST_IDLE);

    task finish_with_error;
        input [7:0] code;
        begin
            fs_error <= code;
            state <= ST_FINISH;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= ST_IDLE;
            file_done         <= 1'b0;
            file_valid        <= 1'b0;
            file_byte         <= 8'd0;
            file_byte_offset  <= 32'd0;
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
            final_chain_check<= 1'b0;
        end else begin
            file_done  <= 1'b0;
            file_valid <= 1'b0;
            sector_req <= 1'b0;

            if (state == ST_DATA_WAIT && sector_valid &&
                remaining_bytes != 0) begin
                file_valid       <= 1'b1;
                file_byte        <= sector_byte;
                remaining_bytes  <= remaining_bytes - 1'b1;
                file_byte_offset <= file_byte_offset + 1'b1;
            end

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
                    if (file_start) begin
                        fs_error         <= ERR_NONE;
                        file_byte_offset <= 32'd0;
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
                    if (!sector_busy) begin
                        sector_req <= 1'b1;
                        state <= ST_DATA_WAIT;
                    end
                end

                ST_DATA_WAIT: begin
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
                    if (!sector_busy) begin
                        sector_req <= 1'b1;
                        state <= ST_FAT_WAIT;
                    end
                end

                ST_FAT_WAIT: begin
                    if (sector_done) begin
                        if (sector_error != ERR_NONE)
                            finish_with_error(ERR_SECTOR);
                        else
                            state <= ST_FAT_PARSE;
                    end
                end

                ST_FAT_PARSE: begin
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
                    file_done <= 1'b1;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
