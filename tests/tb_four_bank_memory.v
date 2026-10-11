`timescale 1ns/1ps
// Real loader, command CDC, reader and vendor FIFO RTL; only SDRAM is modeled.
// Small frames make every address/data comparison practical. This is not a
// physical SDRAM timing model or a 640x480 video bandwidth qualification.
module tb_four_bank_memory;
    reg src_clk=0, mem_clk=0, pix_clk=0, rst_n=0;
    always #5 src_clk=~src_clk;
    always #4 mem_clk=~mem_clk;
    always #20 pix_clk=~pix_clk;
    reg load_start=0, pixel_valid=0, frame_commit=0;
    reg [1:0] image_index=0, read_index=0;
    reg [23:0] pixel_rgb=0;
    reg [7:0] decode_error=0;
    wire load_done, load_busy, full, overflow, dst_idle;
    wire [3:0] complete;
    wire [7:0] memory_error;
    wire src_we, dst_we;
    wire [9:0] src_addr, dst_addr;
    wire [31:0] src_data, dst_data;
    wire [3:0] src_mask, dst_mask;
    reg bus_hold=1;
    wire reader_bus_busy;
    media_frame_loader #(.ADDR_WIDTH(10), .IMAGE_WORDS(64),
        .FRAME_WIDTH(8), .FRAME_HEIGHT(8), .FIFO_DEPTH(128),
        .BURST_SIZE(4), .STALL_TIMEOUT(10000)) loader (
        .clk(src_clk), .rst_n(rst_n), .load_start(load_start),
        .image_index(image_index), .pixel_valid(pixel_valid), .pixel_rgb(pixel_rgb),
        .frame_commit(frame_commit), .decode_error(decode_error),
        .command_ready(!full), .load_busy(load_busy), .load_done(load_done),
        .image_complete(complete), .memory_error(memory_error),
        .controller_init_done(1'b1), .controller_busy(1'b0), .refresh_active(1'b0),
        .app_wr_en(src_we), .app_wr_addr(src_addr), .app_wr_mask(src_mask), .app_wr_data(src_data));
    media_sdram_write_cdc #(.ADDR_WIDTH(10), .FIFO_ADDR_WIDTH(4),
        .BURST_SIZE(4), .BURST_GAP_CYCLES(3)) cdc (
        .src_clk(src_clk), .src_rst_n(rst_n), .src_wr_en(src_we),
        .src_wr_addr(src_addr), .src_wr_mask(src_mask), .src_wr_data(src_data),
        .src_full(full), .src_overflow(overflow), .dst_clk(mem_clk), .dst_rst_n(rst_n),
        .dst_allow(!bus_hold && !reader_bus_busy), .dst_app_wr_en(dst_we),
        .dst_app_wr_addr(dst_addr), .dst_app_wr_mask(dst_mask),
        .dst_app_wr_data(dst_data), .dst_idle(dst_idle));
    reg frame_start=0, consume=0;
    wire rd_en, rgb_valid, underflow, reader_busy, read_done;
    wire [9:0] rd_addr;
    wire [23:0] rgb;
    wire [7:0] reader_error;
    reg rd_valid=0;
    reg [31:0] rd_data=0;
    media_frame_reader #(.ADDR_WIDTH(10), .IMAGE_WORDS(64), .FIFO_DEPTH(32),
        .FIFO_MARGIN(4), .BURST_SIZE(4), .BURST_GAP_CYCLES(3)) reader (
        .mem_clk(mem_clk), .mem_rst_n(rst_n), .controller_init_done(1'b1),
        .controller_busy(bus_hold), .refresh_active(1'b0), .external_write_busy(!dst_idle),
        .app_rd_en(rd_en), .app_rd_addr(rd_addr), .sdram_rd_valid(rd_valid), .sdram_rd_data(rd_data),
        .pix_clk(pix_clk), .pix_rst_n(rst_n), .frame_start(frame_start), .active_index(read_index),
        .pixel_consume(consume), .pixel_rgb(rgb), .pixel_valid(rgb_valid), .underflow(underflow),
        .reader_busy(reader_busy), .reader_bus_busy(reader_bus_busy),
        .frame_read_done(read_done), .reader_index(), .reader_error(reader_error));
    reg [31:0] ram[0:255];
    integer writes=0, reads=0, bank, i, expected_addr;
    reg saw_full=0;
    function [23:0] pattern;
        input integer b, p;
        begin pattern=24'h120000 + b*256 + p; end
    endfunction
    always @(posedge src_clk) if (rst_n && full) saw_full<=1;
    always @(posedge mem_clk) begin
        rd_valid <= rd_en;
        if (rd_en) begin
            if (dst_we) $fatal(1, "read/write overlap");
            rd_data <= ram[rd_addr];
            reads=reads+1;
        end
        if (dst_we) begin
            expected_addr=(writes/64)*64 + (7-(writes%64)/8)*8 + writes%8;
            if (dst_addr !== expected_addr || dst_mask !== 0 ||
                dst_data !== {pattern(writes/64,writes%64),8'h00})
                $fatal(1,"write mismatch word=%0d address=%0d expected=%0d data=%h",writes,dst_addr,expected_addr,dst_data);
            ram[dst_addr]=dst_data;
            writes=writes+1;
        end
    end
    task start_load;
        input [1:0] b;
        begin
            @(negedge src_clk); image_index=b; load_start=1;
            @(negedge src_clk); load_start=0;
        end
    endtask
    initial begin
        #100; rst_n=1;
        for (bank=0; bank<4; bank=bank+1) begin
            bus_hold=1;
            start_load(bank);
            for (i=0; i<64; i=i+1) begin
                @(negedge src_clk); pixel_valid=1; pixel_rgb=pattern(bank,i);
            end
            @(negedge src_clk); pixel_valid=0; frame_commit=1;
            @(negedge src_clk); frame_commit=0;
            repeat(30) @(negedge src_clk);
            if (!full) $fatal(1,"CDC backpressure was not exercised");
            bus_hold=0;
            wait(load_done); #1;
            if (memory_error || !complete[bank]) $fatal(1,"loader completion failure");
            wait(writes == (bank+1)*64);
            wait(dst_idle);
            if (overflow) $fatal(1,"CDC overflow under backpressure");
        end
        if (complete !== 4'b1111 || !saw_full) $fatal(1,"four banks not complete");
        for (bank=3; bank>=0; bank=bank-1) begin
            @(negedge pix_clk); frame_start=1; read_index=bank;
            @(negedge pix_clk); frame_start=0;
            // Allow the reader's cross-domain request and FIFO reset to settle.
            repeat(30) @(negedge pix_clk);
            for (i=0; i<64; i=i+1) begin
                while (!rgb_valid) @(negedge pix_clk);
                if (rgb !== pattern(bank,(7-i/8)*8+i%8))
                    $fatal(1,"read mismatch bank=%0d pixel=%0d rgb=%h",bank,i,rgb);
                consume=1;
                @(negedge pix_clk); consume=0;
                repeat(2) @(negedge pix_clk);
            end
            repeat(10) @(negedge pix_clk);
            if (reader_error || underflow) $fatal(1,"reader error=%h",reader_error);
        end
        if (reads != 256) $fatal(1,"read count=%0d",reads);
        // An error also raises load_done, but must clear the bank-complete bit.
        start_load(0);
        @(negedge src_clk); decode_error=8'h25;
        wait(load_done); #1;
        if (memory_error !== 8'h44 || complete[0]) $fatal(1,"decode failure marked complete");
        @(negedge src_clk); decode_error=0;
        start_load(1);
        @(negedge src_clk); frame_commit=1;
        @(negedge src_clk); frame_commit=0;
        wait(load_done); #1;
        if (memory_error !== 8'h45 || complete[1]) $fatal(1,"short frame marked complete");
        $display("PASS: four-bank write/read data, vertical flip, CDC backpressure, decode/short-frame failure");
        $finish;
    end
    initial begin #2000000; $fatal(1,"memory test timeout"); end
endmodule
