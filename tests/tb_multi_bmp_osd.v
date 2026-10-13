`timescale 1ns/1ps
module tb_multi_bmp_osd;
    reg clk=0, rst_n=0, frame=0;
    always #5 clk=~clk;
    reg [3:0] index=0;
    reg [11:0] x=8, y=8;
    wire [11:0] rgb;
    wire de, hit;
    integer i, slot;
    status_osd_overlay dut (.clk(clk), .rst_n(rst_n), .frame_start(frame),
        .active_index(index), .auto_enabled(1'b1), .error_code(8'hbc),
        .in_de(1'b1), .in_rgb(12'h123), .pixel_x(x), .pixel_y(y),
        .out_rgb(rgb), .out_de(de), .osd_pixel(hit));
    initial begin
        repeat(2) @(negedge clk); rst_n=1;
        for(i=0;i<16;i=i+1) begin
            @(negedge clk); index=i; frame=1; @(negedge clk); frame=0;
            for(slot=0;slot<5;slot=slot+1) begin
                x=8+slot*8; #1;
                case(slot)
                    0: if(dut.value!=(i+1)/10) $fatal(1,"OSD tens incorrect");
                    1: if(dut.value!=(i+1)%10) $fatal(1,"OSD units incorrect");
                    2: if(dut.value!=4'ha) $fatal(1,"OSD auto incorrect");
                    3: if(dut.value!=4'hb) $fatal(1,"OSD high error incorrect");
                    4: if(dut.value!=4'hc) $fatal(1,"OSD low error incorrect");
                endcase
                if(!de || (hit && rgb!=12'hf44)) $fatal(1,"OSD color/DE incorrect");
            end
            // Changes during a frame must not change the displayed digits.
            index=0; x=8; #1;
            if(dut.index_q!=i) $fatal(1,"OSD changed mid-frame");
            x=48; #1;
            if(hit || rgb!=12'h123) $fatal(1,"OSD leaked beyond five characters");
        end
        $display("PASS: OSD decimal 01..16, frame latch, auto/error columns and clipping");
        $finish;
    end
endmodule
