`timescale 1ns/1ps
module tb_audio4_subtitle #(parameter TRACK=0);
    reg clk=0, rst=1;
    always #5 clk=~clk;
    reg selected=0, show=1, epoch=0, accept=0, frame=0, hide=0;
    reg [3:0] selected_track=TRACK;
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
    integer sx,glyph,row, text_base;
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
                    glyph=dut.g_file_subtitle.timeline.data.text_rom[text_base+sx/16];
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
    integer total_samples;
    initial begin
        case(TRACK)
            0: begin total_samples=914580; text_base=0;
                starts[0]=7200;ends[0]=307920;lengths[0]=19;
                starts[1]=315600;ends[1]=615936;lengths[1]=21;
                starts[2]=623616;ends[2]=897792;lengths[2]=19;
            end
            1: begin total_samples=1070289; text_base=59;
                starts[0]=7200;ends[0]=328176;lengths[0]=23;
                starts[1]=335856;ends[1]=753264;lengths[1]=28;
                starts[2]=760944;ends[2]=1053504;lengths[2]=21;
            end
            2: begin total_samples=1031409; text_base=131;
                starts[0]=7200;ends[0]=305376;lengths[0]=20;
                starts[1]=313056;ends[1]=679392;lengths[1]=26;
                starts[2]=687072;ends[2]=1014624;lengths[2]=24;
            end
            3: begin total_samples=1066659; text_base=201;
                starts[0]=7200;ends[0]=297024;lengths[0]=20;
                starts[1]=304704;ends[1]=620208;lengths[1]=22;
                starts[2]=627888;ends[2]=1049856;lengths[2]=29;
            end
            4: begin total_samples=1027932; text_base=272;
                starts[0]=7200;ends[0]=305664;lengths[0]=22;
                starts[1]=313344;ends[1]=667152;lengths[1]=26;
                starts[2]=674832;ends[2]=1011120;lengths[2]=24;
            end
            5: begin total_samples=1055028; text_base=344;
                starts[0]=7200;ends[0]=366336;lengths[0]=26;
                starts[1]=374016;ends[1]=685824;lengths[1]=23;
                starts[2]=693504;ends[2]=1038240;lengths[2]=25;
            end
            6: begin total_samples=1064760; text_base=418;
                starts[0]=7200;ends[0]=381216;lengths[0]=26;
                starts[1]=388896;ends[1]=752208;lengths[1]=26;
                starts[2]=759888;ends[2]=1047936;lengths[2]=20;
            end
            7: begin total_samples=1025925; text_base=490;
                starts[0]=7200;ends[0]=363936;lengths[0]=25;
                starts[1]=371616;ends[1]=654720;lengths[1]=20;
                starts[2]=662400;ends[2]=1009104;lengths[2]=24;
            end
            8: begin total_samples=997788; text_base=559;
                starts[0]=7200;ends[0]=342336;lengths[0]=24;
                starts[1]=350016;ends[1]=654864;lengths[1]=22;
                starts[2]=662544;ends[2]=980976;lengths[2]=23;
            end
            default: $fatal(1,"Invalid test track");
        endcase
        repeat(5) @(negedge clk);rst=0;selected=1;epoch=1;
        repeat(5) @(negedge clk);
        boundary;
        if(dut.timed_visible) $fatal(1,"Subtitle before 150ms lead-in");
        // Two entire narration loops: no accumulated frame/timer drift.
        for(pass=0;pass<2;pass=pass+1) begin
            for(i=0;i<3;i=i+1) begin
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
        for(j=0;j<9;j=j+1) begin
            advance_to(20000);boundary;
            selected_track=selected_track==8 ? 0 : selected_track+1'b1;
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
        $display("PASS: AUDIO%0d three SRT cues, exact sample boundaries, two full loops, pause/underflow, preview, revisit, 640x480 RGB/DE/VS",TRACK+2);
        $finish;
    end
    initial begin #200000000; $fatal(1,"AUDIO4 test timeout"); end
endmodule
