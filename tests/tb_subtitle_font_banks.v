`timescale 1ns/1ps
module tb_subtitle_font_banks;
    reg clk=0;
    always #5 clk=~clk;
    reg [12:0] address=0;
    wire [15:0] row;
    subtitle_font_rom dut(.clk(clk),.address(address),.row(row));
    integer i;
    reg [15:0] expected;
    task check;
        input integer target;
        begin
            @(negedge clk); address=target;
            if(target<4096) expected=dut.pixels0[target];
            else if(target<6144) expected=dut.pixels1[target-4096];
            else expected=dut.pixels2[target-6144];
            @(posedge clk); #1;
            if(row!==expected) $fatal(1,"Font bank data/latency at %d",target);
        end
    endtask
    initial begin
        // Alternate banks every clock as well as reading every allocated row.
        for(i=0;i<7168;i=i+1) begin
            check(i);
            check(6144+i%1024);
            check(i%4096);
            check(4096+i%2048);
        end
        $display("PASS: banked font all 7168 rows and per-clock bank changes, one-clock read latency");
        $finish;
    end
    initial begin #1000000; $fatal(1,"Font timeout"); end
endmodule
