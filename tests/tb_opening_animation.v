`timescale 1ns/1ps
module tb_opening_animation;
    reg clk=0;
    always #5 clk=~clk;
    reg rst=1, selected=0, frame_start=0, de=0, vs=1;
    reg [11:0] x=0,y=0;
    reg [23:0] rgb=24'hc4d5e6;
    wire active, out_de,out_vs;
    wire [23:0] out_rgb;
    scrolling_subtitle #(.INTRO_ENABLE(1),.FILE_SUBTITLE_ENABLE(1)) dut(
        .control_clk(clk),.pixel_clk(clk),.rst(rst),.rx_data(8'd0),.rx_valid(1'b0),
        .frame_start(frame_start),.hide(1'b0),.intro_selected(selected),.intro_active(active),
        .file_selected(1'b0),.file_show(1'b0),.file_commit_toggle(1'b0),.file_sample_accept(1'b0),
        .in_vs(vs),.in_de(de),.pixel_x(x),.pixel_y(y),.rgb_in(rgb),
        .out_vs(out_vs),.out_de(out_de),.rgb_out(out_rgb),.busy(),.rejected_lines());
    task pulse_frame;
        begin
            @(negedge clk); frame_start=1;
            @(negedge clk); frame_start=0;
        end
    endtask
    function [15:0] font_row;
        input integer address;
        begin
            if(address<4096) font_row=dut.font.pixels0[address];
            else if(address<6144) font_row=dut.font.pixels1[address-4096];
            else font_row=dut.font.pixels2[address-6144];
        end
    endfunction
    integer i,j,tick,line,left,top,scale,dx,dy,char_id,bits;
    reg [23:0] expected, prev_rgb=0;
    reg prev_de=0,prev_vs=1;
    task check_pixel;
        begin
            expected=rgb;
            if(active && de) begin
                if((y>=190 && y<192 && x>=40 && x<(tick<60 ? 40+tick*4 : 280)) ||
                   (y>=452 && y<455 && x>=40 && x<40+tick)) expected=24'hb18539;
                for(line=0;line<6;line=line+1) begin
                    case(line)
                        0: begin top=96; scale=2; j=60; end
                        1: begin top=152; scale=1; j=120; end
                        2: begin top=304; scale=1; j=180; end
                        3: begin top=336; scale=1; j=240; end
                        4: begin top=368; scale=1; j=300; end
                        5: begin top=416; scale=1; j=390; end
                    endcase
                    left=line==0 && tick<124 ? 8+(tick-60)/2 : 40;
                    dx=x-left; dy=y-top;
                    if(tick>=j && dx>=0 && dx<(line==0 ? 128 : 512) && dy>=0 && dy<16*scale) begin
                        char_id=dut.g_file_subtitle.timeline.data.text_rom[628+line*32+dx/(16*scale)];
                        bits=font_row(char_id*16+dy/scale);
                        if(bits & (32768>>(dx/scale%16))) expected=24'h39281c;
                    end
                end
            end
            @(posedge clk); #1;
            if(out_de!==prev_de || out_vs!==prev_vs || out_rgb!==prev_rgb)
                $fatal(1,"RGB/timing mismatch tick=%0d x=%0d y=%0d got=%h expected=%h",tick,x,y,out_rgb,prev_rgb);
            prev_rgb=expected; prev_de=de; prev_vs=vs;
        end
    endtask
    task scan_frame;
        integer px,py;
        begin
            // Prime scoreboard across blanking; avoid old state from frame pulses.
            @(negedge clk); de=0; vs=1; x=0;y=0;
            repeat(3) @(negedge clk);
            prev_rgb=rgb;prev_de=0;prev_vs=1;
            for(py=0;py<480;py=py+1) begin
                for(px=0;px<640;px=px+1) begin
                    x=px;y=py;de=1;vs=0;
                    check_pixel;
                    @(negedge clk);
                end
                de=0;check_pixel;@(negedge clk);
            end
            de=0;check_pixel;@(negedge clk);check_pixel;
        end
    endtask
    initial begin
        repeat(4) @(negedge clk);rst=0;
        repeat(4) pulse_frame;
        if(active) $fatal(1,"Started before image valid");
        selected=1;pulse_frame;
        if(!active) $fatal(1,"No startup");
        tick=0;scan_frame;
        for(i=1;i<=479;i=i+1) begin
            pulse_frame;tick=i;
            if(i==90 || i==210 || i==330 || i==450) scan_frame;
        end
        pulse_frame;
        if(!active || dut.g_intro.opening.tick!=479) $fatal(1,"Final text did not hold");
        tick=479;scan_frame;
        repeat(520) pulse_frame;
        if(!active || dut.g_intro.opening.tick!=479) $fatal(1,"Final frame wrapped or vanished");
        selected=0;pulse_frame;selected=1;pulse_frame;
        if(!active || dut.g_intro.opening.tick!=0) $fatal(1,"Returning to PIC1 must restart");
        tick=0;scan_frame;
        @(negedge clk);rst=1;repeat(3) @(negedge clk);rst=0;
        pulse_frame;if(!active) $fatal(1,"Reset must rearm intro");
        selected=0;pulse_frame;if(active) $fatal(1,"Navigation did not cancel intro");
        selected=1;pulse_frame;
        if(!active || dut.g_intro.opening.tick!=0) $fatal(1,"Cancelled intro must replay on return");
        $display("PASS: opening RGB/VS/DE at 640x480, staged text, final frame hold/no wrap, reentry/reset/cancel/replay");
        $finish;
    end
    initial begin #50000000;$fatal(1,"Timeout");end
endmodule
