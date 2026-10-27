`timescale 1ns/1ps
// UART line receiver + two text RAM banks + frame-boundary publication.
// UART 115200/8N1 bytes are supplied by the existing receiver in the 50MHz domain.
// The spare bank is written only while no publication awaits pixel-domain ACK.
module scrolling_subtitle #(
    parameter integer VIEW_WIDTH=640,
    parameter integer TEXT_Y=442,
    parameter integer FRAME_DIV=1,
    parameter integer LOOP_GAP=64,
    parameter integer FILE_SUBTITLE_ENABLE=0,
    parameter integer INTRO_ENABLE=0
)(
    input wire control_clk, pixel_clk, rst,
    input wire [7:0] rx_data,
    input wire rx_valid,
    input wire frame_start, hide,
    input wire intro_selected,
    output wire intro_active,
    input wire file_selected, file_show, file_commit_toggle, file_sample_accept,
    input wire [3:0] file_track, // 0..8 = AUDIO2..AUDIO10
    input wire in_vs, in_de,
    input wire [11:0] pixel_x, pixel_y,
    input wire [23:0] rgb_in,
    output wire out_vs, out_de,
    output wire [23:0] rgb_out,
    output wire busy,
    output reg [15:0] rejected_lines
);
    reg [8:0] text_ram [0:511]; /* fehdl force_ram=1, ram_style="bram" */
    reg write_bank, publish_bank, request_toggle;
    reg ack_toggle, ack_sync1, ack_sync2;
    reg [8:0] receive_length, publish_length;
    reg overflow, dropped_line, skip_lf;
    reg [1:0] utf_remaining;
    reg [20:0] utf_value;
    reg [20:0] utf_minimum;
    wire [20:0] completed_unicode={utf_value[14:0],rx_data[5:0]};
    wire unicode_valid=completed_unicode>=utf_minimum && completed_unicode<=21'hffff &&
        !(completed_unicode>=21'hd800 && completed_unicode<=21'hdfff);
    wire [8:0] mapped_glyph;
    wire lookup_busy, lookup_done;
    reg [8:0] lookup_address;
    wire lookup_start;
    subtitle_unicode_lookup map(.clk(control_clk),.rst(rst),.start(lookup_start),
        .codepoint(completed_unicode[15:0]),.busy(lookup_busy),.done(lookup_done),.glyph(mapped_glyph));
    assign busy=request_toggle!=ack_sync2;

    // RAM write has its own clocked process and no reset on the RAM array.
    // Exactly one glyph is produced for each complete UTF-8 sequence.
    wire line_end=rx_data==8'h0d || rx_data==8'h0a;
    wire continuation=rx_data[7:6]==2'b10;
    wire emit_glyph=(utf_remaining==0 && rx_data>=32 && rx_data<=126) ||
        (utf_remaining!=0 && continuation && utf_remaining==1) ||
        (utf_remaining==0 && rx_data>=128 && !(rx_data>=8'hc2 && rx_data<=8'hf4)) ||
        (utf_remaining!=0 && !continuation);
    wire [8:0] incoming_glyph=utf_remaining!=0 ?
        (continuation && unicode_valid ? mapped_glyph : 8'h3f) :
        (rx_data<128 ? rx_data : 8'h3f);
    wire ram_write=!rst && rx_valid && !line_end && !busy && !dropped_line &&
        !overflow && receive_length<256 && emit_glyph;
    assign lookup_start=ram_write && utf_remaining!=0 && continuation && unicode_valid;
    // At 50 MHz / 115200 baud, adjacent UART bytes are >4300 clocks apart.
    // The binary search completes within 11 clocks; hold the write address
    // until its result arrives. Glyph RAM retains a single physical write port.
    wire text_write=lookup_done || (ram_write && !lookup_start);
    wire [8:0] text_write_address=lookup_done ? lookup_address : {write_bank,receive_length[7:0]};
    wire [8:0] text_write_glyph=lookup_done ? mapped_glyph : incoming_glyph;
    always @(posedge control_clk) begin
        if(lookup_start) lookup_address<={write_bank,receive_length[7:0]};
        if(text_write) text_ram[text_write_address]<=text_write_glyph;
    end

    always @(posedge control_clk or posedge rst) begin
        if(rst) begin
            write_bank<=1; publish_bank<=0; request_toggle<=0;
            ack_sync1<=0; ack_sync2<=0;
            receive_length<=0; publish_length<=0;
            overflow<=0; dropped_line<=0; skip_lf<=0;
            utf_remaining<=0; utf_value<=0; utf_minimum<=0; rejected_lines<=0;
        end else begin
            ack_sync1<=ack_toggle; ack_sync2<=ack_sync1;
            if(rx_valid) begin
                if(skip_lf && rx_data==8'h0a) skip_lf<=0;
                else if(line_end) begin
                    skip_lf<=rx_data==8'h0d;
                    if(!busy && !overflow && !dropped_line && utf_remaining==0) begin
                        publish_bank<=write_bank; publish_length<=receive_length;
                        request_toggle<=~request_toggle; write_bank<=~write_bank;
                    end else rejected_lines<=rejected_lines+1'b1;
                    receive_length<=0; overflow<=0; dropped_line<=0; utf_remaining<=0;
                end else begin
                    skip_lf<=0;
                    if(busy) dropped_line<=1;
                    else if(!dropped_line && !overflow) begin
                        if(emit_glyph) begin
                            utf_remaining<=0;
                            if(receive_length==256) overflow<=1;
                            else receive_length<=receive_length+1'b1;
                        end else if(utf_remaining!=0 && continuation) begin
                            utf_value<=completed_unicode; utf_remaining<=utf_remaining-1'b1;
                        end else if(utf_remaining==0) begin
                            if(rx_data>=8'hc2 && rx_data<=8'hdf) begin
                                utf_remaining<=1; utf_value<={16'd0,rx_data[4:0]}; utf_minimum<=128;
                            end else if(rx_data>=8'he0 && rx_data<=8'hef) begin
                                utf_remaining<=2; utf_value<={17'd0,rx_data[3:0]}; utf_minimum<=2048;
                            end else if(rx_data>=8'hf0 && rx_data<=8'hf4) begin
                                utf_remaining<=3; utf_value<={18'd0,rx_data[2:0]}; utf_minimum<=65536;
                            end
                            // Other ASCII control bytes are ignored. Send UTF-8
                            // without BOM; unsupported Unicode maps to '?'.
                        end
                    end
                end
            end
        end
    end

    reg request_sync1, request_sync2;
    reg active_bank, runtime_text, visible;
    reg [8:0] active_length;
    reg [12:0] scroll_offset;
    reg [15:0] frame_count;
    wire [8:0] default_length;
    wire [8:0] text_length=runtime_text ? active_length : default_length;
    wire [12:0] text_width={text_length,4'b0};
    always @(posedge pixel_clk or posedge rst) begin
        if(rst) begin
            request_sync1<=0; request_sync2<=0; ack_toggle<=0;
            active_bank<=0; runtime_text<=0; active_length<=0;
            visible<=0; scroll_offset<=0; frame_count<=0;
        end else begin
            request_sync1<=request_toggle; request_sync2<=request_sync1;
            if(frame_start) begin
                visible<=!hide;
                if(request_sync2!=ack_toggle) begin
                    // Bundled bank/length held stable from request through ACK.
                    active_bank<=publish_bank; active_length<=publish_length;
                    runtime_text<=1; ack_toggle<=request_sync2;
                    scroll_offset<=0; frame_count<=0;
                end else if(!hide && text_length!=0) begin
                    if(frame_count==FRAME_DIV-1) begin
                        frame_count<=0;
                        scroll_offset<=scroll_offset>=VIEW_WIDTH+text_width+LOOP_GAP-1 ?
                            13'd0 : scroll_offset+1'b1;
                    end else frame_count<=frame_count+1'b1;
                end
            end
        end
    end

    // Two synchronous ROM/RAM reads, with RGB/VS/DE delayed by the same clocks.
    // No wide packed-string multiplexer and no pixel-rate divider/multiplier.
    wire [13:0] sum_x={2'd0,pixel_x}+{1'b0,scroll_offset};
    wire [8:0] timed_glyph;
    wire [7:0] intro_address;
    wire timed_visible;
    wire [5:0] timed_length;
    wire [11:0] timed_left=(VIEW_WIDTH-({6'd0,timed_length}<<4))>>1;
    // Production boot paragraph is replaced by per-file SRT. UART text still
    // works on other images; no unrelated boot paragraph accompanies narration.
    wire timed_mode=FILE_SUBTITLE_ENABLE && file_selected;
    wire scrolling_visible=visible && (!FILE_SUBTITLE_ENABLE || runtime_text) && !timed_mode;
    wire [13:0] source_x=timed_mode ? {2'd0,pixel_x}-{2'd0,timed_left} : sum_x-VIEW_WIDTH;
    generate if(FILE_SUBTITLE_ENABLE) begin : g_file_subtitle
        audio4_subtitle_timeline timeline(.clk(pixel_clk), .rst(rst), .frame_start(frame_start),
            .track(file_track), .opening(intro_active), .opening_address(intro_address),
            .hide(hide), .selected(file_selected), .show(file_show),
            .commit_toggle(file_commit_toggle), .sample_accept(file_sample_accept),
            .character_position(source_x[9:4]), .glyph(timed_glyph),
            .visible(timed_visible), .visible_length(timed_length));
    end else begin : g_no_file_subtitle
        assign timed_glyph=0;
        assign timed_visible=0;
        assign timed_length=0;
    end endgenerate
    wire timed_hit=timed_mode && timed_visible && pixel_x>=timed_left &&
        pixel_x<timed_left+({6'd0,timed_length}<<4);
    wire in_text=(timed_hit || (scrolling_visible && text_length!=0 &&
        sum_x>=VIEW_WIDTH && source_x<text_width)) && pixel_x<VIEW_WIDTH &&
        pixel_y>=TEXT_Y && pixel_y<TEXT_Y+16 &&
        (!FILE_SUBTITLE_ENABLE || !hide);
    wire in_band=((timed_mode && timed_visible) || (scrolling_visible && text_length!=0)) && pixel_x<VIEW_WIDTH &&
        pixel_y>=TEXT_Y-4 && pixel_y<TEXT_Y+20;
    wire [8:0] default_glyph;
    reg [8:0] ram_glyph;
    generate if(FILE_SUBTITLE_ENABLE) begin : g_no_boot_text
        // The SRT replaces boot scrolling text. Eliminate its unused ROM and
        // length-dependent scroll logic from the production configuration.
        assign default_glyph=8'h20;
        assign default_length=0;
    end else begin : g_boot_text
        subtitle_default_text default_text(.clk(pixel_clk),.address(source_x[11:4]),
            .glyph(default_glyph),.length(default_length));
    end endgenerate
    always @(posedge pixel_clk) ram_glyph<=text_ram[{active_bank,source_x[11:4]}];
    wire [3:0] intro_row, intro_col;
    wire [8:0] intro_glyph;
    wire intro_hit, intro_gold;
    generate if(INTRO_ENABLE) begin : g_intro
        opening_animation opening(.clk(pixel_clk),.rst(rst),.frame_start(frame_start),
            .selected(intro_selected),.x(pixel_x),.y(pixel_y),.active(intro_active),
            .row(intro_row),.col(intro_col),.text_address(intro_address),.hit(intro_hit),.gold(intro_gold));
        if(FILE_SUBTITLE_ENABLE) begin : g_shared_text
            assign intro_glyph=timed_glyph;
        end else begin : g_opening_only
            opening_text_rom text(.clk(pixel_clk),.address(intro_address),.glyph(intro_glyph));
        end
    end else begin : g_no_intro
        assign intro_active=0;
        assign intro_row=0;
        assign intro_col=0;
        assign intro_glyph=0;
        assign intro_address=0;
        assign intro_hit=0;
        assign intro_gold=0;
    end endgenerate
    reg intro1, intro2, gold1, gold2;
    reg [3:0] row1, col1, col2;
    reg hit1, hit2, band1, band2;
    reg timed1;
    reg [23:0] rgb1, rgb2;
    reg [1:0] vs_pipe, de_pipe;
    wire [15:0] glyph_row;
    subtitle_font_rom font(.clk(pixel_clk),
        .address({intro1 ? intro_glyph : timed1 ? timed_glyph : runtime_text ? ram_glyph : default_glyph,row1}),.row(glyph_row));
    always @(posedge pixel_clk or posedge rst) begin
        if(rst) begin
            row1<=0; col1<=0; col2<=0; hit1<=0; hit2<=0;
            band1<=0; band2<=0; rgb1<=0; rgb2<=0;
            timed1<=0;
            intro1<=0; intro2<=0; gold1<=0; gold2<=0;
            vs_pipe<=3; de_pipe<=0;
        end else begin
            row1<=intro_active ? intro_row : pixel_y-TEXT_Y;
            col1<=intro_active ? intro_col : source_x[3:0]; col2<=col1;
            timed1<=timed_mode;
            intro1<=intro_active; intro2<=intro1; gold1<=intro_gold; gold2<=gold1;
            hit1<=intro_active ? intro_hit : in_text; hit2<=hit1;
            band1<=!intro_active && in_band; band2<=band1;
            rgb1<=rgb_in; rgb2<=rgb1;
            vs_pipe<={vs_pipe[0],in_vs}; de_pipe<={de_pipe[0],in_de};
        end
    end
    assign out_vs=vs_pipe[1];
    assign out_de=de_pipe[1];
    // Half-black background uses shifts only, retaining full precision elsewhere.
    assign rgb_out=de_pipe[1] && hit2 && glyph_row[15-col2] ? (intro2 ? 24'h39281c : 24'hffffff) :
        de_pipe[1] && gold2 ? 24'hb18539 :
        de_pipe[1] && band2 ? {1'b0,rgb2[23:17],1'b0,rgb2[15:9],1'b0,rgb2[7:1]} : rgb2;
endmodule

// Replay on each PIC1 entry, then hold the final text until leaving PIC1.
// State changes only in blanking; the final tick saturates instead of wrapping.
// Frame timebase avoids a pixel-clock seconds counter; 480 frames ~= 8 seconds.
module opening_animation(input wire clk, rst, frame_start, selected,
    input wire [11:0] x,y, output reg active,
    output wire [3:0] row,col, output wire [7:0] text_address,
    output reg hit, output wire gold);
    reg [8:0] tick;
    always @(posedge clk or posedge rst) begin
        if(rst) begin active<=0; tick<=0; end
        else if(frame_start) begin
            if(!selected) begin active<=0; tick<=0; end
            else if(!active) begin active<=1; tick<=0; end
            else if(tick<479) tick<=tick+1'b1;
        end
    end
    // Fixed rows use bit decodes. Do not build a 12-bit variable top/length
    // rectangle followed by adders, subtractors and four wide comparisons.
    wire title_band=y[11:5]==7'd3;
    wire sub_band=y[11:3]==9'd19 || y[11:3]==9'd20;
    wire row2=y[11:4]==8'd19, row3=y[11:4]==8'd21;
    wire row4=y[11:4]==8'd23, row5=y[11:4]==8'd26;
    wire [5:0] title_phase=tick[5:0]-6'd60;
    wire [5:0] title_left=tick>=60 && tick<124 ? 6'd8+{1'b0,title_phase[5:1]} : 6'd40;
    wire [7:0] title_dx=x[7:0]-{2'd0,title_left};
    wire [8:0] body_dx=x[8:0]-9'd40;
    reg [2:0] line;
    always @* begin
        line=0;
        if(sub_band) line=1;
        else if(row2) line=2;
        else if(row3) line=3;
        else if(row4) line=4;
        else if(row5) line=5;
        hit=active && ((title_band && tick>=60 && x[11:8]==0 && title_dx<128) ||
            (x[11:9]==0 && ((sub_band && tick>=120 && body_dx<160) ||
            (row2 && tick>=180 && body_dx<176) || (row3 && tick>=240 && body_dx<176) ||
            (row4 && tick>=300 && body_dx<224) || (row5 && tick>=390 && body_dx<144))));
    end
    assign row=title_band ? y[4:1] : y[3:0] ^ (sub_band ? 4'd8 : 4'd0);
    assign col=title_band ? title_dx[4:1] : body_dx[3:0];
    wire [4:0] position=title_band ? {2'd0,title_dx[7:5]} : body_dx[8:4];
    assign text_address={line,position};
    wire [11:0] line_end=tick<60 ? 12'd40+{tick[5:0],2'd0} : 12'd280;
    assign gold=active && ((y>=190 && y<192 && x>=40 && x<line_end) ||
        (y>=452 && y<455 && x>=40 && x<12'd40+{3'd0,tick}));
endmodule
