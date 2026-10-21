`timescale 1ns/1ps
module tb_audio_waveform;
    reg clk=0, rst_n=0, vs=1, de=0, enable=1, tap_valid=0;
    always #20 clk=~clk;
    reg [11:0] x=0,y=0;
    reg [23:0] rgb=24'h123456;
    reg signed [15:0] sample=0;
    wire [23:0] out_rgb;
    wire out_vs,out_de,hit,frame;
    wire [11:0] ox,oy;
    promax_audio_waveform #(.WAVE_Y(368)) dut(
        .clk(clk),.rst_n(rst_n),.vsync(vs),.de(de),.rgb_in(rgb),
        .pixel_x(x),.pixel_y(y),.enable(enable),.pcm_tap_valid(tap_valid),
        .pcm_tap_left(sample),.rgb_out(out_rgb),.out_vs(out_vs),.out_de(out_de),
        .out_x(ox),.out_y(oy),.waveform_active(hit),.frame_start_out(frame));
    task boundary;
        begin
            @(negedge clk); de=0;vs=0;tap_valid=0;
            repeat(4) @(negedge clk);
            vs=1;
            repeat(3) @(negedge clk);
        end
    endtask
    task capture;
        input integer level;
        input integer spike;
        integer n;
        begin
            for(n=0;n<384;n=n+1) begin
                @(negedge clk);tap_valid=1;
                sample=((n/12)%2==0) ? level : -level;
                if(level==32768 && (n/12)%2==0) sample=32767;
                // A transient AFTER point 32 must still appear on the meter.
                if(spike && n==383) sample=-32768;
                @(negedge clk);tap_valid=0;
            end
        end
    endtask
    task pixel;
        input integer px,py;
        input [23:0] expected;
        input expected_hit;
        begin
            @(negedge clk);x=px;y=py;de=1;
            repeat(2) @(posedge clk);
            #1;
            if(out_rgb!==expected || hit!==expected_hit || ox!==px || oy!==py || !out_de || !out_vs)
                $fatal(1,"pixel %0d,%0d got %h hit=%b expected %h hit=%b",px,py,out_rgb,hit,expected,expected_hit);
        end
    endtask
    integer xx,yy,sy,amp,idx;
    reg [23:0] expected;
    reg expected_hit;
    task scan;
        input integer amplitude, peak;
        begin
            // Fill the OTHER bank with zeros while scanning: displayed data
            // must remain unchanged until the next frame boundary.
            tap_valid=1; sample=0;
            for(yy=366;yy<435;yy=yy+1) begin
                for(xx=62;xx<595;xx=xx+1) begin
                    idx=(xx-64)/16;
                    sy=(idx%2==0) ? 400-amplitude : 400+amplitude;
                    expected=rgb;expected_hit=0;
                    if(xx>=64 && xx<576 && yy>=368 && yy<432) begin
                        if(yy==400) begin expected=24'h406060;expected_hit=1;end
                        if(yy>=sy-1 && yy<=sy+1) begin expected=24'h20ff90;expected_hit=1;end
                    end
                    if(xx>=584 && xx<592 && yy>=432-peak && yy<432) begin
                        expected=24'hffb020;expected_hit=1;
                    end
                    pixel(xx,yy,expected,expected_hit);
                end
            end
            @(negedge clk);tap_valid=0;de=0;
        end
    endtask
    initial begin
        repeat(4) @(negedge clk); rst_n=1;
        pixel(80,400,24'h406060,1); // Never read uninitialized RAM as a waveform.
        boundary; capture(16384,1); boundary; scan(16,32);
        // Quiet bank captured during scan becomes a flat line, not an old peak.
        boundary; pixel(80,400,24'h20ff90,1);pixel(584,431,rgb,0);
        boundary;capture(8192,0);boundary;scan(8,8);
        boundary;capture(32768,0);boundary;scan(31,32);
        enable=0; pixel(80,400,rgb,0);pixel(584,431,rgb,0);
        enable=1;
        // Incomplete collection must invalidate the prior waveform and meter.
        boundary;capture(0,0);boundary;
        pixel(80,400,24'h20ff90,1);pixel(584,431,rgb,0);
        boundary;
        repeat(7) begin @(negedge clk);tap_valid=1;sample=32767; end
        boundary;pixel(80,400,24'h406060,1);pixel(584,431,rgb,0);
        // Recovery after an incomplete frame must clear the correct peak bank.
        capture(4096,0);boundary;scan(4,4);
        @(negedge clk);rst_n=0;
        repeat(3) @(negedge clk);rst_n=1;
        pixel(80,400,24'h406060,1);pixel(584,431,rgb,0);
        $display("PASS: waveform amplitude, signed extremes, all-sample peaks, silence, ping-pong stability, hide, incomplete capture and reset");
        $finish;
    end
    initial begin #100000000; $fatal(1,"waveform timeout"); end
endmodule
