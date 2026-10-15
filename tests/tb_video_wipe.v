`timescale 1ns/1ps
// Real display commit, reader, async FIFO and prefetch. Only memory responses
// are modeled. Check every address and RGB, both bank directions and all steps.
module tb_video_wipe #(
    parameter integer WIDTH = 8,
    parameter integer HEIGHT = 8,
    parameter integer FRAMES = 4,
    parameter integer BURST = 4,
    parameter integer FIFO_DEPTH = 32
);
    localparam integer WORDS = WIDTH * HEIGHT;
    localparam integer AW = $clog2(WORDS * 4);
    reg src_clk=0, pix_clk=0, mem_clk=0, rst=1;
    always #5 src_clk=~src_clk;
    always #20 pix_clk=~pix_clk;
    always #4 mem_clk=~mem_clk;
    reg valid_src=0, frame_req=0, consume=0;
    reg [1:0] bank_src=0;
    reg [3:0] id_src=0;
    wire valid_pix, changed, audio_toggle, frame_start, wipe;
    wire [1:0] bank_pix, old_bank, ack_bank;
    wire [3:0] id_pix, ack_id;
    wire [AW:0] new_words;
    wire ack_valid;
    media_display_commit #(.ADDR_WIDTH(AW), .IMAGE_WORDS(WORDS),
        .FRAME_WIDTH(WIDTH), .WIPE_FRAMES(FRAMES)) commit (
        .src_clk(src_clk), .rst(rst), .display_valid_src(valid_src),
        .display_index_src(bank_src), .display_image_id_src(id_src),
        .pixel_clk(pix_clk), .frame_request(frame_req),
        .reader_frame_start(frame_start), .display_valid_pix(valid_pix),
        .display_index_pix(bank_pix), .display_image_id_pix(id_pix),
        .display_changed_pix(changed), .audio_commit_toggle(audio_toggle),
        .reader_old_index_pix(old_bank), .reader_new_words_pix(new_words),
        .wipe_active_pix(wipe), .display_ack_valid_src(ack_valid),
        .display_ack_bank_src(ack_bank), .display_ack_image_id_src(ack_id)
    );
    wire rd_en, rgb_valid, underflow, busy, read_done;
    wire [AW-1:0] rd_addr;
    wire [23:0] rgb;
    wire [7:0] error;
    reg rd_valid=0;
    reg [31:0] rd_data=0;
    reg hold_bus=0, refresh=0;
    media_frame_reader #(.ADDR_WIDTH(AW), .IMAGE_WORDS(WORDS),
        .FIFO_DEPTH(FIFO_DEPTH), .FIFO_MARGIN(4), .BURST_SIZE(BURST),
        .BURST_GAP_CYCLES(3), .REGION_READ_ENABLE(1)) reader (
        .mem_clk(mem_clk), .mem_rst_n(~rst), .controller_init_done(1'b1),
        .controller_busy(hold_bus), .refresh_active(refresh), .external_write_busy(1'b0),
        .app_rd_en(rd_en), .app_rd_addr(rd_addr),
        .sdram_rd_valid(rd_valid), .sdram_rd_data(rd_data),
        .pix_clk(pix_clk), .pix_rst_n(~rst), .frame_start(frame_start),
        .active_index(bank_pix), .old_index(old_bank), .new_words(new_words),
        .pixel_consume(consume), .pixel_rgb(rgb), .pixel_valid(rgb_valid),
        .underflow(underflow), .reader_busy(busy), .reader_bus_busy(),
        .frame_read_done(read_done), .reader_index(), .reader_error(error)
    );
    integer issued=0, total_reads=0, done_count=0, pulses=0, frame_count=0;
    integer expected_new=0, expected_old=0, expected_prefix=WORDS;
    integer expected_ack=0, expected_id=0, word_i, step;
    reg check_reads=0;
    reg saved_toggle;
    function [23:0] pattern;
        input integer bank, offset;
        begin pattern = 24'h200000 + bank * 24'h100000 + offset; end
    endfunction
    function integer selected_bank;
        input integer offset;
        begin selected_bank = offset < expected_prefix ? expected_new : expected_old; end
    endfunction
    always @(posedge mem_clk) begin
        rd_valid <= !rst && rd_en;
        if(!rst && rd_en) begin
            if(!check_reads || issued >= WORDS ||
               rd_addr !== selected_bank(issued)*WORDS + issued)
                $fatal(1,"Wrong bank/address: word=%0d addr=%0d prefix=%0d",issued,rd_addr,expected_prefix);
            rd_data <= {pattern(selected_bank(issued),issued),8'h00};
            issued=issued+1; total_reads=total_reads+1;
        end
        if(!rst && read_done) done_count=done_count+1;
    end
    always @(posedge pix_clk) if(!rst && changed) pulses=pulses+1;

    task source;
        input integer bank, image;
        begin
            @(negedge src_clk); bank_src=bank; id_src=image; valid_src=1;
            repeat(24) @(negedge pix_clk);
        end
    endtask
    task frame;
        input integer new_bank, previous_bank, prefix, ack, image;
        begin
            expected_new=new_bank; expected_old=previous_bank; expected_prefix=prefix;
            expected_ack=ack; expected_id=image; issued=0; check_reads=1;
            @(negedge pix_clk); frame_req=1;
            @(negedge pix_clk); frame_req=0;
            // Hold and refresh before the first burst. This is a functional
            // model, not a physical SDRAM bandwidth/timing qualification.
            hold_bus=1;
            repeat(20) @(negedge pix_clk);
            hold_bus=0; refresh=1;
            repeat(4) @(negedge pix_clk);
            refresh=0;
            repeat(20) @(negedge pix_clk);
            if(!valid_pix || new_words!==prefix || bank_pix!==new_bank[1:0])
                $fatal(1,"Wrong committed frame descriptor");
            if(!ack_valid || ack_bank!==ack[1:0] || ack_id!==image[3:0])
                $fatal(1,"Old-bank protection/ack failed: got %0d/%0d want %0d/%0d",ack_bank,ack_id,ack,image);
            for(word_i=0;word_i<WORDS;word_i=word_i+1) begin
                if(!rgb_valid || rgb!==pattern(selected_bank(word_i),word_i))
                    $fatal(1,"Wrong pixel %0d rgb=%h prefix=%0d",word_i,rgb,prefix);
                consume=1;
                @(negedge pix_clk); consume=0;
                repeat(2) @(negedge pix_clk);
            end
            repeat(16) @(negedge pix_clk);
            if(error || underflow || busy || issued!=WORDS)
                $fatal(1,"Reader failure error=%h underflow=%b issued=%0d",error,underflow,issued);
            frame_count=frame_count+1;
            if(done_count!=frame_count) $fatal(1,"Wrong frame-done count");
            check_reads=0;
        end
    endtask
    initial begin
        #100; rst=0;
        source(3,5); frame(3,3,WORDS,3,5);
        if(wipe || pulses!=0) $fatal(1,"First image triggered wipe");
        saved_toggle=audio_toggle;
        // Same logical image at another bank must not start a transition.
        source(2,5); frame(2,2,WORDS,2,5);
        if(wipe || pulses || audio_toggle!=saved_toggle) $fatal(1,"Same image triggered wipe/audio");
        source(0,6);
        for(step=1;step<=FRAMES;step=step+1) begin
            frame(0,2,(HEIGHT/FRAMES)*step*WIDTH,step<FRAMES ? 2:0,step<FRAMES ? 5:6);
            if(step<FRAMES && !wipe) $fatal(1,"Wipe stopped early");
            // An early further request must not replace either frame mid-wipe.
            if(step==1) source(1,7);
        end
        if(wipe || pulses!=1 || audio_toggle==saved_toggle) $fatal(1,"Wipe event/audio completion");
        for(step=1;step<=FRAMES;step=step+1)
            frame(1,0,(HEIGHT/FRAMES)*step*WIDTH,step<FRAMES ? 0:1,step<FRAMES ? 6:7);
        if(wipe || pulses!=2 || audio_toggle!=saved_toggle) $fatal(1,"Queued/reverse bank wipe failed");
        // Reset in a mixed frame must clear protection and discard the stream.
        source(3,8); frame(3,1,WORDS/FRAMES,1,7);
        @(negedge pix_clk); rst=1; valid_src=0;
        repeat(12) @(negedge pix_clk);
        if(wipe || ack_valid || valid_pix || busy) $fatal(1,"Reset did not clear wipe/reader");
        if(total_reads!=frame_count*WORDS) $fatal(1,"Wipe increased the pixel read volume");
        $display("PASS: true vertical wipe %0dx%0d, every address/RGB, one read/pixel, frame CDC, ACK protection, queued switch, reset",WIDTH,HEIGHT);
        $finish;
    end
    initial begin #2000000000; $fatal(1,"Wipe test timeout"); end
endmodule
