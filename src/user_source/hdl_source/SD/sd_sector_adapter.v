`timescale 1ns/1ps

// 将原工程 sd_card_top 的“读请求/数据有效/扇区结束”接口，适配为
// FAT32 模块使用的单扇区事务接口。原驱动保持不变，便于定位问题。
module sd_sector_adapter (
    input  wire        clk,
    input  wire        rst,
    input  wire        sector_req,
    input  wire [31:0] sector_lba,
    output wire        sector_busy,
    output reg         sector_valid,
    output reg  [7:0]  sector_byte,
    output reg  [8:0]  sector_byte_index,
    output reg         sector_done,
    output reg  [7:0]  sector_error,

    output reg         sd_sec_read,
    output reg  [31:0] sd_sec_read_addr,
    input  wire [7:0]  sd_sec_read_data,
    input  wire        sd_sec_read_data_valid,
    input  wire        sd_sec_read_end
);
    localparam ERR_NONE = 8'h00;

    reg active;
    reg [8:0] byte_count;

    assign sector_busy = active;

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
            sector_valid <= 1'b0;
            sector_done  <= 1'b0;
            sd_sec_read  <= 1'b0;

            if (!active && sector_req) begin
                active           <= 1'b1;
                byte_count       <= 9'd0;
                sector_error     <= ERR_NONE;
                sd_sec_read_addr <= sector_lba;
                sd_sec_read      <= 1'b1;
            end

            if (active && sd_sec_read_data_valid) begin
                sector_valid      <= 1'b1;
                sector_byte       <= sd_sec_read_data;
                sector_byte_index <= byte_count;
                if (byte_count != 9'd511)
                    byte_count <= byte_count + 1'b1;
            end

            if (active && sd_sec_read_end) begin
                active      <= 1'b0;
                sector_done <= 1'b1;
            end
        end
    end
endmodule
