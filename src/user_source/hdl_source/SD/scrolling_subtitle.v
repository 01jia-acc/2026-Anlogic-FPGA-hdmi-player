`timescale 1ns/1ps
// UART line receiver + two text RAM banks + frame-boundary publication.
// UART 115200/8N1 bytes are supplied by the existing receiver in the 50MHz domain.
// The spare bank is written only while no publication awaits pixel-domain ACK.
module scrolling_subtitle #(
    parameter integer VIEW_WIDTH=640,
    parameter integer TEXT_Y=442,
    parameter integer FRAME_DIV=1,
    parameter integer LOOP_GAP=64,
    parameter integer FILE_SUBTITLE_ENABLE=0
)(
    input wire control_clk, pixel_clk, rst,
    input wire [7:0] rx_data,
    input wire rx_valid,
    input wire frame_start, hide,
    input wire file_selected, file_show, file_commit_toggle, file_sample_accept,
    input wire in_vs, in_de,
    input wire [11:0] pixel_x, pixel_y,
    input wire [23:0] rgb_in,
    output wire out_vs, out_de,
    output wire [23:0] rgb_out,
    output wire busy,
    output reg [15:0] rejected_lines
);
    reg [7:0] text_ram [0:511]; /* fehdl force_ram=1, ram_style="bram" */
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
    wire [7:0] mapped_glyph;
    subtitle_unicode_map map(.codepoint(completed_unicode[15:0]),.glyph(mapped_glyph));
    assign busy=request_toggle!=ack_sync2;

    // RAM write has its own clocked process and no reset on the RAM array.
    // Exactly one glyph is produced for each complete UTF-8 sequence.
    wire line_end=rx_data==8'h0d || rx_data==8'h0a;
    wire continuation=rx_data[7:6]==2'b10;
    wire emit_glyph=(utf_remaining==0 && rx_data>=32 && rx_data<=126) ||
        (utf_remaining!=0 && continuation && utf_remaining==1) ||
        (utf_remaining==0 && rx_data>=128 && !(rx_data>=8'hc2 && rx_data<=8'hf4)) ||
        (utf_remaining!=0 && !continuation);
    wire [7:0] incoming_glyph=utf_remaining!=0 ?
        (continuation && unicode_valid ? mapped_glyph : 8'h3f) :
        (rx_data<128 ? rx_data : 8'h3f);
    wire ram_write=!rst && rx_valid && !line_end && !busy && !dropped_line &&
        !overflow && receive_length<256 && emit_glyph;
    always @(posedge control_clk)
        if(ram_write) text_ram[{write_bank,receive_length[7:0]}]<=incoming_glyph;

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
    wire [7:0] timed_glyph;
    wire timed_visible;
    wire [5:0] timed_length;
    wire [11:0] timed_left=(VIEW_WIDTH-({6'd0,timed_length}<<4))>>1;
    // Production boot paragraph is replaced by the file-4 SRT. UART text still
    // works on other images; no unrelated boot paragraph accompanies narration.
    wire timed_mode=FILE_SUBTITLE_ENABLE && file_selected;
    wire scrolling_visible=visible && (!FILE_SUBTITLE_ENABLE || runtime_text) && !timed_mode;
    wire [13:0] source_x=timed_mode ? {2'd0,pixel_x}-{2'd0,timed_left} : sum_x-VIEW_WIDTH;
    generate if(FILE_SUBTITLE_ENABLE) begin : g_file_subtitle
        audio4_subtitle_timeline timeline(.clk(pixel_clk), .rst(rst), .frame_start(frame_start),
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
    wire [7:0] default_glyph;
    reg [7:0] ram_glyph;
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
    reg [3:0] row1, col1, col2;
    reg hit1, hit2, band1, band2;
    reg timed1;
    reg [23:0] rgb1, rgb2;
    reg [1:0] vs_pipe, de_pipe;
    wire [15:0] glyph_row;
    subtitle_font_rom font(.clk(pixel_clk),
        .address({timed1 ? timed_glyph : runtime_text ? ram_glyph : default_glyph,row1}),.row(glyph_row));
    always @(posedge pixel_clk or posedge rst) begin
        if(rst) begin
            row1<=0; col1<=0; col2<=0; hit1<=0; hit2<=0;
            band1<=0; band2<=0; rgb1<=0; rgb2<=0;
            timed1<=0;
            vs_pipe<=3; de_pipe<=0;
        end else begin
            row1<=pixel_y-TEXT_Y; col1<=source_x[3:0]; col2<=col1;
            timed1<=timed_mode;
            hit1<=in_text; hit2<=hit1; band1<=in_band; band2<=band1;
            rgb1<=rgb_in; rgb2<=rgb1;
            vs_pipe<={vs_pipe[0],in_vs}; de_pipe<={de_pipe[0],in_de};
        end
    end
    assign out_vs=vs_pipe[1];
    assign out_de=de_pipe[1];
    // Half-black background uses shifts only, retaining full precision elsewhere.
    assign rgb_out=de_pipe[1] && hit2 && glyph_row[15-col2] ? 24'hffffff :
        de_pipe[1] && band2 ? {1'b0,rgb2[23:17],1'b0,rgb2[15:9],1'b0,rgb2[7:1]} : rgb2;
endmodule
