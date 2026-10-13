`timescale 1ns/1ps
module tb_uart_diagnostics;
    reg clk=0, rst=1;
    always #5 clk=~clk;
    wire tx, valid;
    wire [7:0] received_byte;
    integer received=0;
    reg [855:0] expected={"P=1 I=1 T=F B=0 D=1 E=25 V=1 S=0 A=0 L=0000358E C=1FF Q=11 X=0 F=F N=077B R=00 J=02 K=00000102 Z=000E1036",8'h0d,8'h0a};
    reg [31:0] cluster=32'h102;
    uart_debug_logger #(.CLOCK_FREQ_HZ(1_000_000), .BAUD_RATE(100_000)) logger (
        .clk(clk), .rst(rst), .pll_ready(1'b1), .sd_init_done(1'b1),
        .state_code(4'hf), .scan_busy(1'b0), .scan_done(1'b1), .scan_error(8'h25),
        .display_valid(1'b1), .rx_data(8'd0), .scanner_state(4'd0), .sector_active(1'b0),
        .sector_lba(32'h358e), .sector_byte_count(9'h1ff), .sd_state(5'h11),
        .sd_cmd_error_latched(1'b0), .resource_valid(4'hf), .sector_done_count(16'h77b),
        .load_image_number(5'd2), .load_cluster(cluster), .load_size(32'd921654), .tx(tx)
    );
    uart_rx #(.CLOCK_FREQ_HZ(1_000_000), .BAUD_RATE(100_000)) receiver (
        .clk(clk), .rst(rst), .rx(tx), .rx_data(received_byte), .rx_valid(valid)
    );
    always @(posedge clk) if(valid) begin
        if(received_byte !== expected[(106-received)*8 +:8])
            $fatal(1,"UART byte %0d incorrect: %h",received,received_byte);
        received=received+1;
        if(received==10) cluster=32'hdeadbeef; // must not alter the current snapshot
        if(received==107) begin
            $display("PASS: UART 8N1 complete diagnostic line, raw error, image/cluster/size and snapshot");
            $finish;
        end
    end
    initial begin repeat(10) @(negedge clk); rst=0; end
    initial begin #1000000; $fatal(1,"UART test timeout"); end
endmodule
