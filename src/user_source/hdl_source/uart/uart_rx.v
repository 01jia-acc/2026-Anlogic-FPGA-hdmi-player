`timescale 1ns/1ps

// UART receiver, 8-N-1.  rx_valid is a one-clock pulse for each received byte.
module uart_rx #(
    parameter integer CLOCK_FREQ_HZ = 50_000_000,
    parameter integer BAUD_RATE     = 115_200
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       rx,
    output reg  [7:0] rx_data,
    output reg        rx_valid
);
    localparam integer CLKS_PER_BIT =
        (CLOCK_FREQ_HZ + (BAUD_RATE / 2)) / BAUD_RATE;
    localparam integer HALF_BIT_CLKS = CLKS_PER_BIT / 2;

    localparam [1:0] ST_IDLE  = 2'd0;
    localparam [1:0] ST_START = 2'd1;
    localparam [1:0] ST_DATA  = 2'd2;
    localparam [1:0] ST_STOP  = 2'd3;

    reg       rx_meta;
    reg       rx_sync;
    reg [1:0] state;
    reg [31:0] baud_count;
    reg [2:0] bit_index;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state      <= ST_IDLE;
            baud_count <= 32'd0;
            bit_index  <= 3'd0;
            rx_data    <= 8'd0;
            rx_valid   <= 1'b0;
        end else begin
            rx_valid <= 1'b0;

            case (state)
                ST_IDLE: begin
                    baud_count <= 32'd0;
                    if (!rx_sync) begin
                        state      <= ST_START;
                        baud_count <= HALF_BIT_CLKS - 1;
                    end
                end

                ST_START: begin
                    if (baud_count == 0) begin
                        if (!rx_sync) begin
                            state      <= ST_DATA;
                            bit_index  <= 3'd0;
                            baud_count <= CLKS_PER_BIT - 1;
                        end else begin
                            state <= ST_IDLE;
                        end
                    end else begin
                        baud_count <= baud_count - 1'b1;
                    end
                end

                ST_DATA: begin
                    if (baud_count == 0) begin
                        rx_data[bit_index] <= rx_sync;
                        baud_count <= CLKS_PER_BIT - 1;
                        if (bit_index == 3'd7)
                            state <= ST_STOP;
                        else
                            bit_index <= bit_index + 1'b1;
                    end else begin
                        baud_count <= baud_count - 1'b1;
                    end
                end

                ST_STOP: begin
                    if (baud_count == 0) begin
                        if (rx_sync) begin
                            rx_valid <= 1'b1;
                        end
                        state <= ST_IDLE;
                    end else begin
                        baud_count <= baud_count - 1'b1;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
