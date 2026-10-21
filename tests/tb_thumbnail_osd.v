`timescale 1ns/1ps
module tb_thumbnail_osd;
    reg de=1;
    reg [11:0] x=0,y=0;
    reg [31:0] descriptor={8'he4,1'b1,1'b0,2'd0,4'b1101,16'hf970};
    wire [23:0] rgb;
    reg [31:0] requested=0;
    reg [3:0] committed=0;
    wire [31:0] gated;
    thumbnail_exit_gate gate(.requested(requested),.displayed(descriptor),
        .committed_image(committed),.descriptor(gated));
    thumbnail_status_osd dut(.de(de),.pixel_x(x),.pixel_y(y),
        .descriptor(descriptor),.rgb_in(24'h123456),.rgb_out(rgb));
    task probe;
        input integer px, py;
        input [23:0] expected;
        begin x=px; y=py; #1; if(rgb!==expected) $fatal(1,"Thumbnail OSD x=%0d y=%0d got=%h want=%h",x,y,rgb,expected); end
    endtask
    initial begin
        probe(160,120,24'hffff00); // ready focus
        probe(320,120,24'h607080); // non-focused, unavailable
        descriptor[21:20]=1;
        probe(320,120,24'hff8000); // unavailable focus stays visible
        probe(180,150,24'h123456); // background unchanged
        probe(328,127,24'hffffff); // unavailable label '--', middle stroke
        probe(330,124,24'h101820); // label background
        probe(338,244,24'hffffff); // bank3 logical id15 -> label '16', '6' top row
        probe(170,244,24'hffffff); // bank2 logical id9 -> label '10', '1' top center
        probe(162,96,24'hffffff); // title 'C'
        probe(162,384,24'hffffff); // help '2'
        probe(160,400,24'hffffff); // back hint 'H'
        de=0; probe(320,120,24'h123456);
        de=1;
        requested={1'b0,1'b1,2'd3,4'b1101,16'hf970};
        #1; if(gated!==descriptor) $fatal(1,"Menu exit before selected image commits");
        committed=15;
        #1; if(gated!==requested) $fatal(1,"Menu exit not released after image commits");
        descriptor[23]=0; probe(320,120,24'h123456);
        $display("PASS: thumbnail OSD ready/empty focus borders, 01..16 logical labels, title/help, transparent background, DE and mode bypass");
        $finish;
    end
endmodule
