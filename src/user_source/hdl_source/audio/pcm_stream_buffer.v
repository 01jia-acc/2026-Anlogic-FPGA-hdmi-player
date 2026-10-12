`timescale 1ns/1ps

// 讲义格式：48 kHz、16 位有符号小端序、单声道，无 WAV 头。
// start/done 在写域，valid/ready 在像素域；循环播放由上游连续拼接文件样本，
// 不在每轮 EOF 清空指针或置 done，done 仅在错误停止时收尾。
// 扇区请求前必须检查 sector_ready，预留整扇区 256 样本及流水线余量。
// 指针含回卷位，Gray 编码在所属时钟域生成再跨域，不能跨域采二进制指针。
module pcm_stream_buffer #(
    parameter integer ADDR_WIDTH = 12,
    parameter integer PREFILL_SAMPLES = 256
) (
    input wire wr_clk, rd_clk, rst,
    input wire stream_start, byte_valid,
    input wire [7:0] byte_data,
    input wire stream_done,
    input wire prefill_done, // 一轮文件已读完：短文件也允许开始播放，保持循环活动。
    output wire sector_ready,
    output reg overflow,
    output wire signed [15:0] sample,
    output wire sample_valid,
    input wire sample_ready,
    output wire stream_active,
    output wire stream_end,
    output reg underflow,
    output reg [31:0] sample_counter
);
    localparam integer DEPTH = 1 << ADDR_WIDTH;
    reg [15:0] buffer [0:DEPTH-1];
    reg [ADDR_WIDTH:0] wr_ptr, rd_ptr, wr_gray, rd_gray;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_WIDTH:0] rd_sync1, rd_sync2;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_WIDTH:0] wr_sync1, wr_sync2;
    reg started_wr, ended_wr, prefilled_wr, low_pending;
    reg [7:0] low_byte;
    (* ASYNC_REG = "TRUE" *) reg [2:0] start_sync;
    (* ASYNC_REG = "TRUE" *) reg [3:0] end_sync;
    (* ASYNC_REG = "TRUE" *) reg [3:0] prefill_sync;
    reg primed;
    reg out_valid;
    reg [15:0] out_sample;
    function [ADDR_WIDTH:0] from_gray;
        input [ADDR_WIDTH:0] g;
        integer i;
        begin
            from_gray[ADDR_WIDTH] = g[ADDR_WIDTH];
            for (i=ADDR_WIDTH-1; i>=0; i=i-1)
                from_gray[i] = from_gray[i+1] ^ g[i];
        end
    endfunction
    wire [ADDR_WIDTH:0] used_wr = wr_ptr - from_gray(rd_sync2);
    wire [ADDR_WIDTH:0] used_rd = from_gray(wr_sync2) - rd_ptr;
    wire [ADDR_WIDTH:0] wr_next = wr_ptr + 1'b1;
    wire [ADDR_WIDTH:0] rd_next = rd_ptr + 1'b1;
    // Two extra slots cover the registered file-byte output and sample assembly.
    assign sector_ready = used_wr <= DEPTH - 258;
    assign sample = out_sample;
    assign sample_valid = out_valid;
    // EOF is delayed beyond the pointer synchronizer and includes the held sample.
    assign stream_end = end_sync[3] && used_rd == 0 && !out_valid;
    assign stream_active = start_sync[2] && !stream_end;

    always @(posedge wr_clk or posedge rst) begin
        if (rst) begin
            wr_ptr <= 0; wr_gray <= 0; rd_sync1 <= 0; rd_sync2 <= 0;
            started_wr <= 0; ended_wr <= 0; prefilled_wr <= 0; low_pending <= 0;
            low_byte <= 0; overflow <= 0;
        end else begin
            rd_sync1 <= rd_gray; rd_sync2 <= rd_sync1;
            if (stream_start) started_wr <= 1'b1;
            if (stream_done) ended_wr <= 1'b1;
            if (prefill_done) prefilled_wr <= 1'b1;
            if (byte_valid && started_wr && !ended_wr) begin
                if (!low_pending) begin
                    low_byte <= byte_data;
                    low_pending <= 1'b1;
                end else begin
                    low_pending <= 1'b0;
                    if (used_wr < DEPTH) begin
                        buffer[wr_ptr[ADDR_WIDTH-1:0]] <= {byte_data, low_byte};
                        wr_ptr <= wr_next;
                        wr_gray <= wr_next ^ (wr_next >> 1);
                    end else overflow <= 1'b1;
                end
            end
        end
    end

    // Synchronous RAM read with one held output sample. The consumer only
    // accepts on a 48 kHz tick; it receives zero from the bridge when empty.
    always @(posedge rd_clk or posedge rst) begin
        if (rst) begin
            rd_ptr <= 0; rd_gray <= 0; wr_sync1 <= 0; wr_sync2 <= 0;
            start_sync <= 0; end_sync <= 0; prefill_sync <= 0; primed <= 0;
            out_valid <= 0; out_sample <= 0; underflow <= 0; sample_counter <= 0;
        end else begin
            wr_sync1 <= wr_gray; wr_sync2 <= wr_sync1;
            start_sync <= {start_sync[1:0], started_wr};
            end_sync <= {end_sync[2:0], ended_wr};
            prefill_sync <= {prefill_sync[2:0], prefilled_wr};
            if (used_rd >= PREFILL_SAMPLES || end_sync[3] || prefill_sync[3])
                primed <= 1'b1;
            if (sample_ready && out_valid) begin
                out_valid <= 1'b0;
                sample_counter <= sample_counter + 1'b1;
            end
            if (primed && sample_ready && !out_valid && stream_active)
                underflow <= 1'b1;
            if (primed && used_rd != 0 && (!out_valid || sample_ready)) begin
                out_sample <= buffer[rd_ptr[ADDR_WIDTH-1:0]];
                out_valid <= 1'b1;
                rd_ptr <= rd_next;
                rd_gray <= rd_next ^ (rd_next >> 1);
            end
        end
    end
endmodule
