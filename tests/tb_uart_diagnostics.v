`timescale 1ns/1ps
module tb_uart_diagnostics;
    reg clk=0, src_clk=0, rst=1, logger_rst=1;
    always #5 clk=~clk;
    always #3 src_clk=~src_clk;
    wire tx, valid;
    wire [7:0] received_byte;
    integer received=0, lines=0;
    reg [1279:0] expected={"P=1 I=1 T=F B=0 D=1 E=25 V=1 S=0 A=0 L=0000358E C=1FF Q=11 X=0 F=F N=077B R=00 J=02 K=00000102 Z=000E1036 H=0123456789ABCDEF M=03FF U=03FF G=0A O=06 Y=07 W=0F",8'h0d,8'h0a};
    reg [31:0] cluster=32'h102;
    reg [63:0] pipeline=64'h0123456789abcdef;
    reg [63:0] playlist=64'h03ff03ff0a06070f;
    wire [95:0] mailbox;
    wire ready;
    subtitle_frame_commit commit (
        .src_clk(src_clk), .src_rst_n(~rst), .src_commit(ready), .src_ready(ready),
        .src_text({32'd0,playlist}), .src_length(6'd0), .src_enable(1'b1),
        .dst_clk(clk), .dst_rst_n(~rst), .frame_start(1'b1),
        .dst_text(mailbox), .dst_length(), .dst_enable()
    );
    uart_debug_logger #(.CLOCK_FREQ_HZ(1_000_000), .BAUD_RATE(100_000), .LOG_INTERVAL_MS(2)) logger (
        .clk(clk), .rst(logger_rst), .pll_ready(1'b1), .sd_init_done(1'b1),
        .state_code(4'hf), .scan_busy(1'b0), .scan_done(1'b1), .scan_error(8'h25),
        .display_valid(1'b1), .rx_data(8'd0), .scanner_state(4'd0), .sector_active(1'b0),
        .sector_lba(32'h358e), .sector_byte_count(9'h1ff), .sd_state(5'h11),
        .sd_cmd_error_latched(1'b0), .resource_valid(4'hf), .sector_done_count(16'h77b),
        .load_image_number(5'd2), .load_cluster(cluster), .load_size(32'd921654),
        .load_pipeline(pipeline), .playlist_snapshot(mailbox[63:0]), .tx(tx)
    );
    uart_rx #(.CLOCK_FREQ_HZ(1_000_000), .BAUD_RATE(100_000)) receiver (
        .clk(clk), .rst(logger_rst), .rx(tx), .rx_data(received_byte), .rx_valid(valid)
    );
    always @(posedge clk) if(valid) begin
        if(received_byte !== expected[(159-received)*8 +:8])
            $fatal(1,"UART byte %0d incorrect: %h",received,received_byte);
        received=received+1;
        if(received==10 && lines==0) begin
            cluster=32'hdeadbeef; pipeline=0; playlist=64'hfffefffe0f10e1a5;
        end // Current UART line must remain unchanged while source data updates.
        if(received==160) begin
            lines=lines+1;
            if(lines==2) begin
                $display("PASS: two full 160-byte UART lines, mailbox CDC, file masks/count/mapping and frozen per-line snapshot");
                $finish;
            end
            received=0;
            expected={"P=1 I=1 T=F B=0 D=1 E=25 V=1 S=0 A=0 L=0000358E C=1FF Q=11 X=0 F=F N=077B R=00 J=02 K=DEADBEEF Z=000E1036 H=0000000000000000 M=FFFE U=FFFE G=0F O=10 Y=E1 W=A5",8'h0d,8'h0a};
        end
    end
    initial begin
        repeat(10) @(negedge clk); rst=0;
        repeat(20) @(negedge clk); logger_rst=0;
    end
    initial begin #1000000; $fatal(1,"UART test timeout"); end
endmodule
