`timescale 1ns/1ps
module tb_volume_osd #(parameter BRIGHTNESS_TEST=0, parameter CONTRAST_TEST=0);
    localparam X=CONTRAST_TEST ? 376 : (BRIGHTNESS_TEST ? 200 : 288);
    localparam LEVEL_BITS=CONTRAST_TEST ? 6 : 5;
    reg clk=0,rst_n=0,frame=0,de=1,mode=0;
    always #5 clk=~clk;
    reg [LEVEL_BITS-1:0] level=(BRIGHTNESS_TEST || CONTRAST_TEST) ? 20 : 15;
    reg [11:0] x=0,y=8;
    wire [23:0] rgb;
    volume_status_osd #(.OSD_X(X), .LABEL(CONTRAST_TEST ? "CON" : BRIGHTNESS_TEST ? "BRI" : "VOL"),
        .LEVEL_BITS(LEVEL_BITS),
        .DEFAULT_LEVEL((BRIGHTNESS_TEST || CONTRAST_TEST) ? 20 : 15)) dut(.clk(clk),.rst_n(rst_n),.frame_start(frame),.de(de),
        .pixel_x(x),.pixel_y(y),.level(level),.adjust_mode(mode),
        .rgb_in(24'h123456),.rgb_out(rgb));
    task commit_frame;
        begin @(negedge clk);frame=1;@(negedge clk);frame=0;end
    endtask
    // Top rows of the decimal font. Verify rendered digits, including 0/100.
    function [4:0] top_row;
        input integer digit;
        begin
            case(digit)
                0,2,6,8,9:top_row=5'b01110;
                1:top_row=5'b00100;
                3:top_row=5'b11110;
                4:top_row=5'b00010;
                5,7:top_row=5'b11111;
                default:top_row=0;
            endcase
        end
    endfunction
    integer k,col,bitno,digit;
    reg [4:0] pattern;
    reg [23:0] expected;
    initial begin
        repeat(4) @(negedge clk);rst_n=1;
        x=X+(CONTRAST_TEST ? 1 : 0);#1;if(rgb!==24'hffffff) $fatal(1,"Default white label");
        mode=1;#1;if(rgb!==24'hffffff) $fatal(1,"Midframe mode changed");
        commit_frame;#1;if(rgb!==24'hffff00) $fatal(1,"Adjust mode yellow");
        for(k=0;k<=(CONTRAST_TEST ? 40 : 20);k=k+1) begin
            level=k;commit_frame;
            for(col=4;col<=6;col=col+1) begin
                if(col==4) digit=k>=40 ? 2 : k>=20 ? 1 : -1;
                else if(col==5) digit=k<2 ? -1 : ((k/2)%10);
                else digit=(k%2)*5;
                pattern=top_row(digit);
                for(bitno=0;bitno<8;bitno=bitno+1) begin
                    x=X+col*8+bitno;#1;
                    expected=(bitno<5 && pattern[4-bitno]) ? 24'hffff00 : 24'h123456;
                    if(rgb!==expected) $fatal(1,"OSD percentage %0d digit %0d pixel %0d",k*5,col,bitno);
                end
            end
        end
        level=0; x=X+32+2;#1;
        if(rgb!==24'hffff00) $fatal(1,"Midframe digits changed");
        commit_frame;#1;if(rgb!==24'h123456) $fatal(1,"Next-frame digits not updated");
        x=X;de=0;#1;if(rgb!==24'h123456) $fatal(1,"Blanking overlay");
        de=1;y=15;#1;if(rgb!==24'h123456) $fatal(1,"Outside rectangle");
        $display("PASS: OSD all percentages 0..%0d, white/yellow, frame-stable text, blanking and background passthrough",
            CONTRAST_TEST ? 200 : 100);
        $finish;
    end
endmodule

module tb_brightness_osd;
    tb_volume_osd #(.BRIGHTNESS_TEST(1)) test();
endmodule

module tb_contrast_osd;
    tb_volume_osd #(.CONTRAST_TEST(1)) test();
endmodule
