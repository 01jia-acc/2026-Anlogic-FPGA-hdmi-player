`timescale 1ns/1ps
module tb_sw1_key1_reset;
    reg clk=0, key1_n=1, sw1_n=1;
    always #5 clk=~clk;
    top dut(.clk(clk), .rst_n(key1_n), .key1(1'b1), .key2(1'b1), .key3(1'b1),
        .switch_brightness_n(sw1_n), .switch_volume_n(1'b1),
        .switch_pcm_n(1'b1), .switch_zoom_n(1'b1),
        .sd_miso(1'b1), .uart_rxd(1'b1), .HDMI_DDC_SDA());
    initial begin
        #1;if(dut.system_reset_n!==1) $fatal(1,"SW1 OFF idle");
        key1_n=0;#1;if(dut.system_reset_n!==0) $fatal(1,"SW1 OFF KEY1 must reset");
        sw1_n=0;#1;if(dut.system_reset_n!==1) $fatal(1,"SW1 ON KEY1 must adjust only");
        key1_n=1;#1;if(dut.system_reset_n!==1) $fatal(1,"SW1 ON released");
        sw1_n=1;key1_n=0;#1;
        if(dut.system_reset_n!==0) $fatal(1,"SW1 OFF must restore KEY1 reset");
        $display("PASS: physical KEY1 resets only when SW1 is OFF");
        $finish;
    end
endmodule
