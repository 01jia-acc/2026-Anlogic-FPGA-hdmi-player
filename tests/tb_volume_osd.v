`timescale 1ns/1ps
module tb_volume_osd;
    reg clk=0,rst_n=0,frame=0,de=1,mode=0;
    always #5 clk=~clk;
    reg [4:0] level=15;
    reg [11:0] x=0,y=8;
    wire [23:0] rgb;
    volume_status_osd dut(.clk(clk),.rst_n(rst_n),.frame_start(frame),.de(de),
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
        x=288;#1;if(rgb!==24'hffffff) $fatal(1,"Default white VOL");
        mode=1;#1;if(rgb!==24'hffffff) $fatal(1,"Midframe mode changed");
        commit_frame;#1;if(rgb!==24'hffff00) $fatal(1,"Adjust mode yellow");
        for(k=0;k<=20;k=k+1) begin
            level=k;commit_frame;
            for(col=4;col<=6;col=col+1) begin
                if(col==4) digit=k==20 ? 1 : -1;
                else if(col==5) digit=k==20 ? 0 : (k<2 ? -1 : k/2);
                else digit=(k%2)*5;
                pattern=top_row(digit);
                for(bitno=0;bitno<8;bitno=bitno+1) begin
                    x=288+col*8+bitno;#1;
                    expected=(bitno<5 && pattern[4-bitno]) ? 24'hffff00 : 24'h123456;
                    if(rgb!==expected) $fatal(1,"OSD percentage %0d digit %0d pixel %0d",k*5,col,bitno);
                end
            end
        end
        level=0; x=320+2;#1;
        if(rgb!==24'hffff00) $fatal(1,"Midframe digits changed");
        commit_frame;#1;if(rgb!==24'h123456) $fatal(1,"Next-frame digits not updated");
        x=288;de=0;#1;if(rgb!==24'h123456) $fatal(1,"Blanking overlay");
        de=1;y=15;#1;if(rgb!==24'h123456) $fatal(1,"Outside rectangle");
        $display("PASS: OSD all percentages 0..100, white/yellow, frame-stable text, blanking and background passthrough");
        $finish;
    end
endmodule
