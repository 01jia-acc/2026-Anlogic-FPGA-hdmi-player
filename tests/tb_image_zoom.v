`timescale 1ns/1ps
// Real reader, row RAM, async FIFO and prefetch; memory returns ordered data.
module tb_image_zoom #(
    parameter integer WIDTH=24, HEIGHT=12, BURST=4, FIFO_DEPTH=64
);
    localparam integer WORDS=WIDTH*HEIGHT, AW=$clog2(WORDS*4);
    reg mem_clk=0, pix_clk=0, rst=1;
    always #4 mem_clk=~mem_clk;
    always #20 pix_clk=~pix_clk;
    reg start=0, consume=0;
    reg [1:0] level=1, bank=3, old_bank=0;
    reg [AW:0] prefix=WORDS;
    wire rd_en, valid, underflow, busy, done;
    wire [AW-1:0] addr;
    wire [23:0] rgb;
    wire [7:0] error;
    wire [1:0] actual_level;
    reg [7:0] returns=0;
    reg [31:0] data_pipe[0:7];
    media_frame_reader #(.ADDR_WIDTH(AW), .IMAGE_WORDS(WORDS),
        .FRAME_WIDTH(WIDTH), .FRAME_HEIGHT(HEIGHT), .ZOOM_ENABLE(1),
        .REGION_READ_ENABLE(1), .FIFO_DEPTH(FIFO_DEPTH), .FIFO_MARGIN(4),
        .BURST_SIZE(BURST), .BURST_GAP_CYCLES(12)) dut (
        .mem_clk(mem_clk), .mem_rst_n(~rst), .controller_init_done(1'b1),
        .controller_busy(1'b0), .refresh_active(1'b0), .external_write_busy(1'b0),
        .app_rd_en(rd_en), .app_rd_addr(addr), .sdram_rd_valid(returns[7]),
        .sdram_rd_data(data_pipe[7]), .pix_clk(pix_clk), .pix_rst_n(~rst),
        .frame_start(start), .active_index(bank), .old_index(old_bank), .new_words(prefix),
        .zoom_level(level), .display_zoom_level(actual_level),
        .pixel_consume(consume), .pixel_rgb(rgb), .pixel_valid(valid),
        .underflow(underflow), .reader_busy(busy), .reader_bus_busy(),
        .frame_read_done(done), .reader_index(), .reader_error(error)
    );
    integer issued=0, frame_level=1, frame_bank=3, frame_old=0, frame_prefix=WORDS;
    integer i, x, y, sx, sy, selected, offset, frames=0, dones=0;
    reg [23:0] expected;
    reg check_reads=1;
    function [23:0] pattern;
        input integer b, o;
        begin pattern=24'h200000+b*24'h100000+o; end
    endfunction
    function integer source_y;
        input integer row, z;
        begin
            case(z)
                0: source_y=row<HEIGHT/4 ? 0 : row>=HEIGHT*3/4 ? HEIGHT-2 : 2*(row-HEIGHT/4);
                2: source_y=HEIGHT/6+(row*2)/3;
                3: source_y=HEIGHT/4+row/2;
                default:source_y=row;
            endcase
        end
    endfunction
    integer j, expected_addr;
    always @(posedge mem_clk) begin
        returns<=rst ? 0 : {returns[6:0],rd_en};
        for(j=1;j<8;j=j+1) data_pipe[j]<=data_pipe[j-1];
        if(!rst && rd_en) begin
            expected_addr=(issued<frame_prefix ? frame_bank : frame_old)*WORDS+
                source_y(issued/WIDTH,frame_level)*WIDTH+issued%WIDTH;
            if(check_reads && (issued>=WORDS || addr!==expected_addr[AW-1:0]))
                $fatal(1,"Zoom address level=%0d issued=%0d got=%0d expected=%0d",frame_level,issued,addr,expected_addr);
            data_pipe[0]<={pattern(addr/WORDS,addr%WORDS),8'd0};
            issued=issued+1;
        end
        if(!rst && done) dones=dones+1;
    end
    task frame;
        input integer z, b, old, split;
        begin
            @(negedge pix_clk);
            frame_level=z; frame_bank=b; frame_old=old; frame_prefix=split;
            level=z; bank=b; old_bank=old; prefix=split; issued=0; start=1;
            @(negedge pix_clk); start=0;
            repeat(WIDTH+100) @(negedge pix_clk);
            if(actual_level!=z) $fatal(1,"Zoom descriptor not latched");
            // Changing the incoming controls during a frame must not tear it.
            level=(z+1)%4; bank=old; old_bank=b; prefix=0;
            for(y=0;y<HEIGHT;y=y+1) begin
                for(x=0;x<WIDTH;x=x+1) begin
                    case(z)
                        0:sx=2*(x-WIDTH/4);
                        2:sx=WIDTH/6+(x*2)/3;
                        3:sx=WIDTH/4+x/2;
                        default:sx=x;
                    endcase
                    sy=source_y(y,z);
                    selected=y*WIDTH+x<split ? b : old;
                    expected=pattern(selected,sy*WIDTH+sx);
                    if(z==0 && (x<WIDTH/4 || x>=WIDTH*3/4 || y<HEIGHT/4 || y>=HEIGHT*3/4)) expected=0;
                    if(!valid || rgb!==expected)
                        $fatal(1,"Zoom pixel level=%0d x=%0d y=%0d valid=%b got=%h expected=%h",z,x,y,valid,rgb,expected);
                    consume=1;
                    @(negedge pix_clk);
                end
                consume=0;
                repeat(WIDTH/4) @(negedge pix_clk);
            end
            repeat(30) @(negedge pix_clk);
            frames=frames+1;
            if(error || underflow || busy || issued!=WORDS || dones!=frames)
                $fatal(1,"Zoom reader completion error=%h underflow=%b issued=%0d done=%0d",error,underflow,issued,dones);
        end
    endtask
    initial begin
        repeat(12) @(negedge pix_clk); rst=0;
        frame(1,3,3,WORDS);
        frame(0,3,0,WORDS/2);
        frame(2,0,3,WORDS/2);
        frame(3,1,2,WORDS);
        frame(2,2,1,WORDS);
        frame(0,0,0,WORDS);
        frame(1,1,1,WORDS);
        // Reset while a zoom row is being fetched discards the old stream.
        check_reads=0; level=3; start=1;
        @(negedge pix_clk); start=0;
        repeat(10) @(negedge pix_clk); rst=1;
        repeat(12) @(negedge pix_clk);
        if(busy || valid || underflow) $fatal(1,"Zoom reset failed");
        rst=0; check_reads=1; frame(3,3,3,WORDS);
        $display("PASS: zoom %0dx%0d all four levels, every pixel/address, full-rate active pixels, wipe banks, frame latch, repeat changes and reset",WIDTH,HEIGHT);
        $finish;
    end
    initial begin #2000000000; $fatal(1,"Zoom timeout"); end
endmodule
