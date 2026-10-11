`timescale 1ns/1ps

// UART transmitter, 8-N-1.
// The input clock is intentionally parameterized because this module is also
// useful for later diagnostic messages from other clock domains.
module uart_tx #(
    parameter integer CLOCK_FREQ_HZ = 50_000_000,
    parameter integer BAUD_RATE     = 115_200
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       tx_start,
    input  wire [7:0] tx_data,
    output reg        tx_busy,
    output reg        tx
);
    localparam integer CLKS_PER_BIT =
        (CLOCK_FREQ_HZ + (BAUD_RATE / 2)) / BAUD_RATE;

    reg [31:0] baud_count;
    reg [3:0]  bit_index;
    reg [9:0]  shift_reg;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            baud_count <= 32'd0;
            bit_index  <= 4'd0;
            shift_reg  <= 10'b11_1111_1111;
            tx_busy    <= 1'b0;
            tx         <= 1'b1;
        end else if (!tx_busy) begin
            baud_count <= 32'd0;
            tx         <= 1'b1;
            if (tx_start) begin
                // Frame order on the wire: start, data[0]..data[7], stop.
                shift_reg  <= {1'b1, tx_data, 1'b0};
                bit_index  <= 4'd0;
                tx_busy    <= 1'b1;
                tx         <= 1'b0;
            end
        end else if (baud_count >= CLKS_PER_BIT - 1) begin
            baud_count <= 32'd0;
            if (bit_index == 4'd9) begin
                tx_busy <= 1'b0;
                tx      <= 1'b1;
            end else begin
                bit_index <= bit_index + 1'b1;
                shift_reg <= {1'b1, shift_reg[9:1]};
                tx        <= shift_reg[1];
            end
        end else begin
            baud_count <= baud_count + 1'b1;
        end
    end
endmodule
