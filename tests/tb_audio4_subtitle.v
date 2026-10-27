`timescale 1ns/1ps
module tb_audio4_subtitle #(parameter TRACK=0);
    reg clk=0, rst=1;
    always #5 clk=~clk;
    reg selected=0, show=1, epoch=0, accept=0, frame=0, hide=0;
    reg [1:0] selected_track=TRACK;
    reg de=0, vs=1;
    reg [11:0] x=0,y=0;
    reg [23:0] rgb=24'h80c0fe;
    wire ovs,ode;
    wire [23:0] out;
    scrolling_subtitle #(.FILE_SUBTITLE_ENABLE(1)) dut(
        .control_clk(clk),.pixel_clk(clk),.rst(rst),.rx_data(8'd0),.rx_valid(1'b0),
        .frame_start(frame),.hide(hide),.file_selected(selected),.file_show(show),
        .file_track(selected_track),
        .file_commit_toggle(epoch),.file_sample_accept(accept),
        .in_vs(vs),.in_de(de),.pixel_x(x),.pixel_y(y),.rgb_in(rgb),
        .out_vs(ovs),.out_de(ode),.rgb_out(out),.busy(),.rejected_lines());
    function [15:0] font_row;
        input integer address;
        begin
            if(address<4096) font_row=dut.font.pixels0[address];
            else if(address<6144) font_row=dut.font.pixels1[address-4096];
            else font_row=dut.font.pixels2[address-6144];
        end
    endfunction
    integer starts[0:7],ends[0:7],lengths[0:7];
    integer i,j,pass,position=0,saved,xx,yy,pixels=0;
    reg [23:0] expect0,expect1;
    reg [1:0] deq=0,vsq=3;
    integer sx,glyph,row;
    // Independent pixel reference uses the compiled font/text contents, while
    // host tests independently compare those contents against UTF-8 SRT.
    always @(posedge clk) begin
        if(rst) begin deq=0;vsq=3;expect0=0;expect1=0; end
        else begin
            expect1=expect0;
            expect0=rgb;
            if(de && dut.timed_visible && selected && y>=438 && y<462) begin
                expect0={1'b0,rgb[23:17],1'b0,rgb[15:9],1'b0,rgb[7:1]};
                sx=x-(640-lengths[0]*16)/2;
                if(y>=442 && y<458 && sx>=0 && sx<lengths[0]*16) begin
                    glyph=dut.g_file_subtitle.timeline.data.text_rom[TRACK*256+sx/16];
                    row=font_row(glyph*16+y-442);
                    if(row & (16'h8000>>(sx%16))) expect0=24'hffffff;
                end
            end
            deq={deq[0],de}; vsq={vsq[0],vs};
            #1;
            if(ode!==deq[1] || ovs!==vsq[1]) $fatal(1,"Video timing alignment");
            if(ode) begin
                pixels=pixels+1;
                if(out!==expect1) $fatal(1,"Timed subtitle RGB x=%d y=%d got=%h expected=%h",x,y,out,expect1);
            end
        end
    end
    task boundary;
        begin
            @(negedge clk); frame=1;
            @(negedge clk); frame=0;
        end
    endtask
    task advance_to;
        input integer target;
        begin
            // Fast simulated sample strobes retain exact 48kHz sample indices.
            @(negedge clk);accept=1;
            while(position<target) begin @(negedge clk);position=position+1;end
            accept=0;
            if(dut.g_file_subtitle.timeline.sample_position!=position)
                $fatal(1,"Sample timeline drift target=%d actual=%d",position,dut.g_file_subtitle.timeline.sample_position);
        end
    endtask
    integer total_samples=3079722;
    initial begin
        starts[0]=12000;ends[0]=383376;lengths[0]=26;
        starts[1]=392016;ends[1]=812880;lengths[1]=29;
        starts[2]=821520;ends[2]=1147824;lengths[2]=24;
        starts[3]=1156464;ends[3]=1513392;lengths[3]=25;
        starts[4]=1522032;ends[4]=1895136;lengths[4]=28;
        starts[5]=1903776;ends[5]=2257536;lengths[5]=26;
        starts[6]=2266176;ends[6]=2642016;lengths[6]=26;
        starts[7]=2650656;ends[7]=3055728;lengths[7]=31;
        if(TRACK==1) begin
            total_samples=3118884;
            starts[0]=12000;ends[0]=370368;lengths[0]=26;
            starts[1]=379008;ends[1]=755856;lengths[1]=28;
            starts[2]=764496;ends[2]=1147872;lengths[2]=28;
            starts[3]=1156512;ends[3]=1486992;lengths[3]=24;
            starts[4]=1495632;ends[4]=1879200;lengths[4]=29;
            starts[5]=1887840;ends[5]=2289936;lengths[5]=28;
            starts[6]=2298576;ends[6]=2664720;lengths[6]=27;
            starts[7]=2673360;ends[7]=3094896;lengths[7]=31;
        end
        if(TRACK==2) begin
            total_samples=3242676;
            starts[0]=12000;ends[0]=377568;lengths[0]=27;
            starts[1]=386208;ends[1]=735744;lengths[1]=26;
            starts[2]=744384;ends[2]=1116576;lengths[2]=24;
            starts[3]=1125216;ends[3]=1534752;lengths[3]=29;
            starts[4]=1543392;ends[4]=1944096;lengths[4]=27;
            starts[5]=1952736;ends[5]=2331648;lengths[5]=27;
            starts[6]=2340288;ends[6]=2753376;lengths[6]=30;
            starts[7]=2762016;ends[7]=3218688;lengths[7]=35;
        end
        repeat(5) @(negedge clk);rst=0;selected=1;epoch=1;
        repeat(5) @(negedge clk);
        boundary;
        if(dut.timed_visible) $fatal(1,"Subtitle before 250ms lead-in");
        // Two entire narration loops: no accumulated frame/timer drift.
        for(pass=0;pass<2;pass=pass+1) begin
            for(i=0;i<8;i=i+1) begin
                advance_to(starts[i]-1);boundary;
                if(dut.timed_visible) $fatal(1,"Cue early %d",i);
                advance_to(starts[i]);boundary;
                if(!dut.timed_visible || dut.timed_length!=lengths[i]) $fatal(1,"Cue start %d",i);
                if(i==0 && pass==0) begin
                    // Pause/no PCM/underflow all mean no accepted samples.
                    saved=position;repeat(100) @(negedge clk);
                    show=0;boundary;
                    if(dut.timed_visible || dut.g_file_subtitle.timeline.sample_position!=saved) $fatal(1,"Pause");
                    show=1;hide=1;boundary;
                    advance_to(saved+200);boundary;
                    if(dut.timed_visible) $fatal(1,"Preview not hidden");
                    hide=0;boundary;
                    if(!dut.timed_visible) $fatal(1,"Preview resume");
                    for(yy=0;yy<480;yy=yy+1) begin
                        for(xx=0;xx<640;xx=xx+1) begin
                            @(negedge clk);de=1;x=xx;y=yy;
                        end
                        @(negedge clk);de=0;
                    end
                    repeat(4) @(negedge clk);
                end
                advance_to(ends[i]-1);boundary;
                if(!dut.timed_visible) $fatal(1,"Cue ended early");
                advance_to(ends[i]);boundary;
                if(dut.timed_visible) $fatal(1,"Cue not cleared at end");
            end
            advance_to(total_samples-1);
            @(negedge clk);accept=1;
            @(negedge clk);accept=0;position=0;
            boundary;
            if(dut.g_file_subtitle.timeline.sample_position!=0 || dut.timed_visible) $fatal(1,"Loop boundary");
        end
        advance_to(20000);boundary;
        selected=0;epoch=~epoch;repeat(4) @(negedge clk);boundary;
        if(dut.timed_visible || dut.g_file_subtitle.timeline.sample_position!=0) $fatal(1,"Leave image4");
        selected=1;epoch=~epoch;repeat(4) @(negedge clk);boundary;
        position=0;
        if(dut.timed_visible) $fatal(1,"Revisit did not restart lead-in");
        // Consecutive subtitle-enabled images must reset even if selected stays high.
        for(j=0;j<3;j=j+1) begin
            advance_to(20000);boundary;
            selected_track=selected_track==2 ? 0 : selected_track+1'b1;
            epoch=~epoch;
            repeat(4) @(negedge clk);position=0;boundary;
            if(dut.g_file_subtitle.timeline.sample_position!=0 || dut.timed_visible ||
               dut.g_file_subtitle.timeline.active_track!=selected_track)
                $fatal(1,"Direct track change did not restart");
            advance_to(12000);boundary;
            if(!dut.timed_visible || dut.g_file_subtitle.timeline.visible_track!=selected_track)
                $fatal(1,"Direct track change selected wrong text bank");
        end
        if(pixels!=640*480) $fatal(1,"Full-frame coverage");
        $display("PASS: AUDIO%0d eight SRT cues, exact sample boundaries, two full loops, pause/underflow, preview, revisit, 640x480 RGB/DE/VS",TRACK+4);
        $finish;
    end
    initial begin #200000000; $fatal(1,"AUDIO4 test timeout"); end
endmodule
