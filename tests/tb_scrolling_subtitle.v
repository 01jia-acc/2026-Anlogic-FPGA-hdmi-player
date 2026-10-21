`timescale 1ns/1ps
// Real UART receiver, independently clocked text RAM, publication and renderer.
module tb_scrolling_subtitle;
    parameter integer WIDTH=64, HEIGHT=64, TEXT_Y=24;
    reg control_clk=0, pixel_clk=0, rst=1, serial=1;
    always #5 control_clk=~control_clk;
    always #7 pixel_clk=~pixel_clk;
    wire [7:0] rx_data;
    wire rx_valid, out_vs, out_de, busy;
    wire [23:0] rgb_out;
    wire [15:0] rejected;
    reg frame_start=0, hide=0, in_vs=1, in_de=0;
    reg [11:0] x=0,y=0;
    reg [23:0] rgb=24'h81c3f5;
    uart_rx #(.CLOCK_FREQ_HZ(1000),.BAUD_RATE(100)) receiver(
        .clk(control_clk),.rst(rst),.rx(serial),.rx_data(rx_data),.rx_valid(rx_valid));
    scrolling_subtitle #(.VIEW_WIDTH(WIDTH),.TEXT_Y(TEXT_Y)) dut(
        .control_clk(control_clk),.pixel_clk(pixel_clk),.rst(rst),
        .rx_data(rx_data),.rx_valid(rx_valid),.frame_start(frame_start),.hide(hide),
        .in_vs(in_vs),.in_de(in_de),.pixel_x(x),.pixel_y(y),.rgb_in(rgb),
        .out_vs(out_vs),.out_de(out_de),.rgb_out(rgb_out),.busy(busy),.rejected_lines(rejected));

    task send;
        input [7:0] value;
        integer bit_no;
        begin
            @(negedge control_clk); serial=0;
            repeat(10) @(negedge control_clk);
            for(bit_no=0;bit_no<8;bit_no=bit_no+1) begin
                serial=value[bit_no]; repeat(10) @(negedge control_clk);
            end
            serial=1; repeat(12) @(negedge control_clk);
        end
    endtask
    task frame;
        begin
            @(negedge pixel_clk); in_de=0; in_vs=0; frame_start=1;
            @(negedge pixel_clk); frame_start=0; in_vs=1;
            repeat(5) @(negedge pixel_clk);
        end
    endtask
    task commit_line;
        input integer length;
        begin
            send(13); send(10);
            if(!busy) $fatal(1,"Line not waiting for frame commit");
            frame;
            repeat(6) @(negedge control_clk);
            if(busy || !dut.runtime_text || dut.active_length!=length || dut.scroll_offset!=0)
                $fatal(1,"Publication length=%0d actual=%0d busy=%b",length,dut.active_length,busy);
        end
    endtask

    // RGB/DE/VS scoreboard. It uses explicit chosen text/offset, not renderer
    // coordinates or its hit/band pipeline. ROM bits are checked separately.
    reg check_pixels=0;
    reg model_enabled=0;
    integer model_length=0, model_offset=0;
    reg [7:0] model_text [0:255];
    reg [23:0] expected_prev=0, expected_now;
    reg expected_de=0, expected_vs=1, prev_valid=0;
    integer local_x, glyph_id, address;
    reg [15:0] reference_row;
    always @(posedge pixel_clk) begin
        if(rst || !check_pixels) prev_valid=0;
        else begin
            expected_now=rgb;
            local_x=x+model_offset-WIDTH;
            if(in_de && model_enabled && x<WIDTH && y>=TEXT_Y-4 && y<TEXT_Y+20) begin
                expected_now={1'b0,rgb[23:17],1'b0,rgb[15:9],1'b0,rgb[7:1]};
                if(y>=TEXT_Y && y<TEXT_Y+16 && local_x>=0 && local_x<model_length*16) begin
                    glyph_id=model_text[local_x/16]; address=glyph_id*16+(y-TEXT_Y);
                    reference_row=dut.font.pixels[address];
                    if(reference_row[15-local_x%16]) expected_now=24'hffffff;
                end
            end
            #1;
            if(prev_valid && (rgb_out!==expected_prev || out_de!==expected_de || out_vs!==expected_vs))
                $fatal(1,"Subtitle pipeline x=%0d y=%0d rgb=%h want=%h DE=%b/%b VS=%b/%b",x,y,rgb_out,expected_prev,out_de,expected_de,out_vs,expected_vs);
            expected_prev=expected_now; expected_de=in_de; expected_vs=in_vs; prev_valid=1;
        end
    end
    task raster;
        integer xx,yy;
        begin
            check_pixels=1;
            for(yy=0;yy<HEIGHT;yy=yy+1) begin
                for(xx=0;xx<WIDTH;xx=xx+1) begin
                    @(negedge pixel_clk); x=xx; y=yy; in_de=1;
                    rgb={xx[7:0],yy[7:0],8'hf5};
                end
                @(negedge pixel_clk); in_de=0;
                repeat(4) @(negedge pixel_clk);
            end
            repeat(4) @(negedge pixel_clk); check_pixels=0;
        end
    endtask
    integer i, old_length, old_bank, saved_offset, rejects;
    integer boot_length;
    wire [7:0] huan_glyph, ying_glyph;
    subtitle_unicode_map map_huan(.codepoint(16'h6b22),.glyph(huan_glyph));
    subtitle_unicode_map map_ying(.codepoint(16'h8fce),.glyph(ying_glyph));
    initial begin
        repeat(12) @(negedge control_clk); rst=0;
        frame;
        boot_length=dut.default_length;
        if(boot_length==0 || dut.text_length!=boot_length || !dut.visible) $fatal(1,"Default paragraph missing");
        // Complete default paragraph leaves the viewport before wrap, per-pixel
        // scrolling changes only at a frame boundary, never at line boundaries.
        for(i=2;i<WIDTH+boot_length*16+64;i=i+1) begin
            frame;
            if(dut.scroll_offset!=i) $fatal(1,"Scroll step/wrap duration %0d",i);
        end
        frame;
        if(dut.scroll_offset!=0) $fatal(1,"Default paragraph loop");

        send("A"); send("b"); send("9");
        old_bank=dut.active_bank;
        if(dut.runtime_text) $fatal(1,"Uncommitted text became visible");
        commit_line(3);
        if(dut.active_bank==old_bank) $fatal(1,"No spare-bank publication");
        model_text[0]="A"; model_text[1]="b"; model_text[2]="9";
        model_enabled=1; model_length=3;
        for(i=0;i<WIDTH+16;i=i+1) frame;
        model_offset=WIDTH+16;
        saved_offset=dut.scroll_offset;
        raster;
        if(dut.scroll_offset!=saved_offset) $fatal(1,"Scroll changed within a frame");
        hide=1; frame; model_enabled=0; raster;
        repeat(8) frame;
        if(dut.scroll_offset!=saved_offset) $fatal(1,"Hidden preview did not pause");
        hide=0; frame; model_enabled=1; model_offset=saved_offset+1; raster;

        // UTF-8 Chinese from the generated font, unknown CJK and an emoji.
        send(8'he6); send(8'hac); send(8'ha2); // 欢
        send(8'he8); send(8'hbf); send(8'h8e); // 迎
        send(8'he9); send(8'hbe); send(8'h98); // 龘, not in subset
        send(8'hf0); send(8'h9f); send(8'h98); send(8'h80); // emoji -> one '?'
        commit_line(4);
        if(dut.text_ram[{dut.active_bank,8'd0}]!=huan_glyph ||
           dut.text_ram[{dut.active_bank,8'd1}]!=ying_glyph ||
           dut.text_ram[{dut.active_bank,8'd2}]!="?" || dut.text_ram[{dut.active_bank,8'd3}]!="?")
            $fatal(1,"UTF-8 decode/subset fallback");
        model_text[0]=huan_glyph; model_text[1]=ying_glyph; model_text[2]="?"; model_text[3]="?";
        model_length=4;
        for(i=0;i<WIDTH+32;i=i+1) frame;
        model_offset=WIDTH+32; raster;

        // Active bank cannot be modified by UART, even when a whole new line
        // arrives before the pending update has received a frame ACK.
        send("X"); send(10); send("Y"); send(10);
        rejects=rejected;
        if(rejects!=1) $fatal(1,"Busy line not rejected atomically");
        frame; repeat(6) @(negedge control_clk);
        if(dut.active_length!=1 || dut.text_ram[{dut.active_bank,8'd0}]!="X")
            $fatal(1,"Pending line overwritten");
        old_bank=dut.active_bank;
        for(i=0;i<257;i=i+1) send("Z");
        send(10); frame;
        if(rejected!=rejects+1 || dut.active_bank!=old_bank || dut.active_length!=1)
            $fatal(1,"Overlong line must preserve live text");
        // Maximum-sized line uses all 256 cells (no 8-bit length wrap).
        for(i=0;i<256;i=i+1) send(i==255 ? "!" : "A");
        commit_line(256);
        if(dut.text_ram[{dut.active_bank,8'd255}]!="!") $fatal(1,"Last cell missing");
        for(i=0;i<256;i=i+1) model_text[i]=i==255 ? "!" : "A";
        for(i=0;i<WIDTH+4096-16;i=i+1) frame;
        model_length=256; model_offset=WIDTH+4096-16; raster;
        for(i=0;i<16+64;i=i+1) frame;
        if(dut.scroll_offset!=0) $fatal(1,"Maximum paragraph loop overflow");
        send(8'he6); send(10); frame;
        if(dut.active_length!=256 || rejected!=rejects+2) $fatal(1,"Partial UTF-8 must reject line");
        send(8'he0); send(8'h80); send(8'h80); commit_line(1);
        if(dut.text_ram[{dut.active_bank,8'd0}]!="?") $fatal(1,"Overlong UTF-8 not replaced");
        // Empty CRLF disables at a frame boundary; reset restores boot text.
        commit_line(0);
        model_enabled=0; raster;
        rst=1; repeat(8) @(negedge control_clk); rst=0; frame;
        if(dut.runtime_text || dut.text_length!=boot_length || busy) $fatal(1,"Reset/default recovery");
        $display("PASS: scrolling subtitle %0dx%0d real UART, UTF-8 Chinese/ASCII/fallback, 256 cells, atomic dual-clock bank swap, busy/overflow/partial UTF-8 rejection, CRLF/clear/reset, complete loop and preview pause, every RGB/DE/VS aligned",WIDTH,HEIGHT);
        $finish;
    end
    initial begin #1000000000; $fatal(1,"Subtitle timeout"); end
endmodule
