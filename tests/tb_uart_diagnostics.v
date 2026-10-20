`timescale 1ns/1ps
module tb_uart_diagnostics;
    reg clk=0, rst=1;
    always #5 clk=~clk;
    wire tx, valid;
    wire [7:0] received_byte;
    integer received=0;
    reg [1679:0] expected={"P=1 I=1 T=F B=0 D=1 E=25 V=1 S=0 A=0 L=0000358E C=1FF Q=11 X=0 F=F N=077B R=00 J=02 K=00000102 Z=000E1036 H=0123456789ABCDEF M=03FF U=0A W=CBF43926 Y=0400 O=0003F18C0000000001CA002BF2000000BB807FFF G=0000032A",8'h0d,8'h0a};
    reg [31:0] scanned_cluster=32'h32a;
    reg [159:0] pcm_status=160'h0003F18C0000000001CA002BF2000000BB807FFF;
    reg [31:0] cluster=32'h102;
    reg [63:0] pipeline=64'h0123456789abcdef;
    reg [15:0] picture_mask=16'h03ff;
    reg [4:0] picture_count=10;
    reg [31:0] root_crc=32'hcbf43926;
    reg [15:0] root_bytes=16'h0400;
    uart_debug_logger #(.CLOCK_FREQ_HZ(1_000_000), .BAUD_RATE(100_000)) logger (
        .clk(clk), .rst(rst), .pll_ready(1'b1), .sd_init_done(1'b1),
        .state_code(4'hf), .scan_busy(1'b0), .scan_done(1'b1), .scan_error(8'h25),
        .display_valid(1'b1), .rx_data(8'd0), .scanner_state(4'd0), .sector_active(1'b0),
        .sector_lba(32'h358e), .sector_byte_count(9'h1ff), .sd_state(5'h11),
        .sd_cmd_error_latched(1'b0), .resource_valid(picture_mask), .image_count(picture_count),
        .root_crc(root_crc), .root_bytes(root_bytes),
        .sector_done_count(16'h77b),
        .load_image_number(5'd2), .load_cluster(cluster), .load_size(32'd921654),
        .accepted_cluster(scanned_cluster),
        .load_pipeline(pipeline), .pcm_status(pcm_status), .tx(tx)
    );
    uart_rx #(.CLOCK_FREQ_HZ(1_000_000), .BAUD_RATE(100_000)) receiver (
        .clk(clk), .rst(rst), .rx(tx), .rx_data(received_byte), .rx_valid(valid)
    );
    always @(posedge clk) if(valid) begin
        if(received_byte !== expected[(209-received)*8 +:8])
            $fatal(1,"UART byte %0d incorrect: %h",received,received_byte);
        received=received+1;
        if(received==10) begin cluster=32'hdeadbeef; pipeline=0; picture_mask=0; picture_count=0; root_crc=0; root_bytes=0; pcm_status=0; scanned_cluster=0; end
        if(received==210) begin
            $display("PASS: UART 8N1 complete 210-byte diagnostic line, PIC/PCM status, accepted cluster and snapshot");
            $finish;
        end
    end
    initial begin repeat(10) @(negedge clk); rst=0; end
    initial begin #1000000; $fatal(1,"UART test timeout"); end
endmodule
