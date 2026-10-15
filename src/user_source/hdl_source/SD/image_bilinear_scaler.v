`timescale 1ns/1ps
// Load-time resampling in BMP physical row order. Two synchronous line RAMs;
// no frame RAM and no extra SDRAM reads. Caller pauses file bytes between rows.
// Q16 align-corners coordinates, Q8 weights, round only after both axes.
module image_bilinear_scaler #(
    parameter integer OUT_WIDTH=640, OUT_HEIGHT=480, MAX_WIDTH=1920
) (
    input wire clk, rst, start, abort,
    input wire header_valid,
    input wire [15:0] source_width, source_height,
    input wire pixel_valid,
    input wire [23:0] pixel_rgb,
    input wire [15:0] pixel_x, pixel_y,
    output wire input_ready,
    output reg output_valid,
    output reg [23:0] output_rgb,
    output reg done,
    output wire [3:0] debug_state
);
    localparam WAIT_HEADER=0, DIV_X=1, DIV_Y=2, COLLECT=3, CHECK_ROW=4,
        READ_LEFT=5, SAVE_LEFT=6, READ_RIGHT=7, SAVE_RIGHT=8,
        MIX_TOP=9, MIX_BOTTOM=10, MIX_VERTICAL=11, EMIT=12, FINISHED=13, PASSTHROUGH=14;
    reg [3:0] state;
    assign debug_state = state;
    reg configured;
    reg [15:0] width_q, height_q, latest_row, out_x, out_y;
    reg [31:0] x_step, y_step, x_phase, y_phase;
    reg [31:0] dividend, quotient, remainder;
    reg [5:0] div_count;
    wire [31:0] divisor = state==DIV_X ? OUT_WIDTH-1 : OUT_HEIGHT-1;
    wire [31:0] shifted_remainder = {remainder[30:0],dividend[31]};
    wire div_bit = shifted_remainder >= divisor;
    wire [31:0] next_quotient = {quotient[30:0], div_bit};
    wire [15:0] sx = out_x==OUT_WIDTH-1 ? width_q-1'b1 : x_phase[31:16];
    wire [15:0] sy = out_y==OUT_HEIGHT-1 ? height_q-1'b1 : y_phase[31:16];
    wire [15:0] sx1 = sx < width_q-1 ? sx+1'b1 : sx;
    wire [15:0] sy1 = sy < height_q-1 ? sy+1'b1 : sy;
    wire [7:0] wx = out_x==OUT_WIDTH-1 ? 8'd0 : x_phase[15:8];
    wire [7:0] wy = out_y==OUT_HEIGHT-1 ? 8'd0 : y_phase[15:8];
    // TD otherwise selects distributed RAM for these lines (14k+ LUTs in
    // the failing gate.area). Use the same native BRAM directive as the
    // generated Anlogic FIFO IP, power-of-two depth and bounded addresses.
    localparam integer LINE_ADDR_WIDTH = MAX_WIDTH<=1 ? 1 : $clog2(MAX_WIDTH);
    localparam integer LINE_DEPTH = 1 << LINE_ADDR_WIDTH;
    reg [23:0] even_line [LINE_DEPTH-1:0]; /* fehdl force_ram=1, ram_style="bram" */
    reg [23:0] odd_line [LINE_DEPTH-1:0];  /* fehdl force_ram=1, ram_style="bram" */
    reg [23:0] even_q, odd_q, p00, p01, p10, p11;
    wire [LINE_ADDR_WIDTH-1:0] write_x = pixel_x[LINE_ADDR_WIDTH-1:0];
    wire [LINE_ADDR_WIDTH-1:0] read_x = state==READ_LEFT ?
        sx[LINE_ADDR_WIDTH-1:0] : sx1[LINE_ADDR_WIDTH-1:0];
    wire line_write = pixel_valid && input_ready && state==COLLECT;
    wire line_read = state==READ_LEFT || state==READ_RIGHT;
    // Separate synchronous write/read ports; never reset the RAM or its
    // read registers. PASSTHROUGH does not need to write either line.
    always @(posedge clk) begin
        if(line_write && !pixel_y[0]) even_line[write_x] <= pixel_rgb;
    end
    always @(posedge clk) begin
        if(line_write && pixel_y[0]) odd_line[write_x] <= pixel_rgb;
    end
    always @(posedge clk) begin
        if(line_read) even_q <= even_line[read_x];
    end
    always @(posedge clk) begin
        if(line_read) odd_q <= odd_line[read_x];
    end
    reg [15:0] row_top [0:2];
    reg [15:0] row_bottom [0:2];
    wire [7:0] weight = state==MIX_VERTICAL ? wy : wx;
    wire [23:0] mix_left = state==MIX_TOP ? p00 : p01;
    wire [23:0] mix_right = state==MIX_TOP ? p10 : p11;
    wire [25:0] mixed [0:2];
    genvar c;
    generate for(c=0;c<3;c=c+1) begin : channel
        // Shared multiplier per channel: top, bottom, then vertical.
        wire signed [16:0] a = state==MIX_VERTICAL ?
            $signed({1'b0,row_top[c]}) : $signed({1'b0,mix_left[c*8 +:8],8'd0});
        wire signed [16:0] b = state==MIX_VERTICAL ?
            $signed({1'b0,row_bottom[c]}) : $signed({1'b0,mix_right[c*8 +:8],8'd0});
        wire signed [16:0] delta = b-a;
        wire signed [25:0] product = delta * $signed({1'b0,weight});
        assign mixed[c] = $signed({a,8'd0}) + product;
    end endgenerate
    integer k;
    assign input_ready = !rst && (state==COLLECT || state==FINISHED || state==PASSTHROUGH ||
        (state==WAIT_HEADER && !header_valid));
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=WAIT_HEADER; configured<=0; width_q<=0; height_q<=0;
            latest_row<=0; out_x<=0; out_y<=0;
            x_step<=0; y_step<=0; x_phase<=0; y_phase<=0;
            dividend<=0; quotient<=0; remainder<=0; div_count<=0;
            p00<=0; p01<=0; p10<=0; p11<=0;
            output_valid<=0; output_rgb<=0; done<=0;
            for(k=0;k<3;k=k+1) begin row_top[k]<=0; row_bottom[k]<=0; end
        end else if(start || abort) begin
            state<=WAIT_HEADER; configured<=0;
            out_x<=0; out_y<=0; x_phase<=0; y_phase<=0;
            output_valid<=0; done<=0;
        end else begin
            output_valid<=0; done<=0;
            case(state)
                WAIT_HEADER: if(header_valid && !configured) begin
                    configured<=1; width_q<=source_width; height_q<=source_height;
                    dividend<=({16'd0,source_width}-1'b1)<<16;
                    quotient<=0; remainder<=0; div_count<=0;
                    state <= source_width==OUT_WIDTH && source_height==OUT_HEIGHT ? PASSTHROUGH : DIV_X;
                end
                DIV_X, DIV_Y: begin
                    dividend<=dividend<<1;
                    remainder<=div_bit ? shifted_remainder-divisor : shifted_remainder;
                    quotient<=next_quotient;
                    div_count<=div_count+1'b1;
                    if(div_count==31) begin
                        if(state==DIV_X) begin
                            x_step<=next_quotient;
                            dividend<=({16'd0,height_q}-1'b1)<<16;
                            quotient<=0; remainder<=0; div_count<=0; state<=DIV_Y;
                        end else begin y_step<=next_quotient; state<=COLLECT; end
                    end
                end
                COLLECT: if(pixel_valid && pixel_x==width_q-1) begin
                    latest_row<=pixel_y; state<=CHECK_ROW;
                end
                PASSTHROUGH: if(pixel_valid) begin
                    output_rgb<=pixel_rgb; output_valid<=1;
                    if(pixel_x==width_q-1 && pixel_y==height_q-1) begin
                        done<=1; state<=FINISHED;
                    end
                end
                CHECK_ROW: begin
                    if(latest_row>=sy1) begin out_x<=0; x_phase<=0; state<=READ_LEFT; end
                    else state<=COLLECT;
                end
                READ_LEFT: state<=SAVE_LEFT;
                SAVE_LEFT: begin
                    p00<=sy[0] ? odd_q : even_q;
                    p01<=sy1[0] ? odd_q : even_q;
                    state<=READ_RIGHT;
                end
                READ_RIGHT: state<=SAVE_RIGHT;
                SAVE_RIGHT: begin
                    p10<=sy[0] ? odd_q : even_q;
                    p11<=sy1[0] ? odd_q : even_q;
                    state<=MIX_TOP;
                end
                MIX_TOP: begin
                    for(k=0;k<3;k=k+1) row_top[k]<=mixed[k][23:8];
                    state<=MIX_BOTTOM;
                end
                MIX_BOTTOM: begin
                    for(k=0;k<3;k=k+1) row_bottom[k]<=mixed[k][23:8];
                    state<=MIX_VERTICAL;
                end
                MIX_VERTICAL: begin
                    for(k=0;k<3;k=k+1)
                        output_rgb[k*8 +:8]<=(mixed[k]+26'd32768)>>16;
                    state<=EMIT;
                end
                EMIT: begin
                    output_valid<=1;
                    if(out_x==OUT_WIDTH-1) begin
                        if(out_y==OUT_HEIGHT-1) begin state<=FINISHED; done<=1; end
                        else begin out_y<=out_y+1'b1; y_phase<=y_phase+y_step; state<=CHECK_ROW; end
                    end else begin
                        out_x<=out_x+1'b1; x_phase<=x_phase+x_step; state<=READ_LEFT;
                    end
                end
                FINISHED: ;
                default: state<=WAIT_HEADER;
            endcase
        end
    end
    // synthesis translate_off
    initial if(OUT_WIDTH<2 || OUT_HEIGHT<2 || MAX_WIDTH<1)
        $fatal(1,"Scaler requires output dimensions >=2");
    always @(posedge clk) if(!rst && !start && !abort && pixel_valid &&
        (!input_ready || pixel_x>=MAX_WIDTH))
        $fatal(1,"Scaler input overrun / invalid source coordinate");
    // synthesis translate_on
endmodule
