`timescale 1ns/1ps
// Real scheduler + coherent display commit + real audio bridge.
// Force only the image-source transaction boundary; this bench does not
// exercise physical SD commands or FAT sector parsing.
module tb_four_buffer_audio;
    reg sd_clk=0, pix_clk=0, rst=1;
    always #5 sd_clk=~sd_clk;
    always #20 pix_clk=~pix_clk;
    reg key_next=1, key_auto=1, key_audio_mode=1, frame_req=0, write_toggle=0;
    reg scan_done=0, source_done=0;
    reg [3:0] resources=4'b1111;
    reg [7:0] source_error=0;
    wire display_valid, pix_valid, read_start, audio_toggle;
    wire [1:0] write_bank, display_bank, pix_bank;
    wire [3:0] state_code;
    wire audio_valid;
    wire [23:0] audio_left;
    integer load_count=0;

    sd_card_bmp_fat32 #(.CLK_FREQ_HZ(1000), .LONG_PRESS_MS(200), .PCM_ENABLE(1)) control (
        .clk(sd_clk), .rst(rst), .key_next(key_next), .key_auto(key_auto),
        .key_audio_mode(key_audio_mode),
        .pixel_clk(pix_clk), .pcm_ready(1'b0),
        .bmp_width(16'd640), .bmp_height(16'd480),
        .display_valid(display_valid), .state_code(state_code),
        .write_finish_toggle(write_toggle), .write_buf_idx(write_bank),
        .disp_buf_idx(display_bank), .write_req(), .write_req_ack(1'b1),
        .write_en(), .write_data(), .write_frame_commit(),
        .SD_nCS(), .SD_DCLK(), .SD_MOSI(), .SD_MISO(1'b1)
    );
    media_display_commit commit (
        .src_clk(sd_clk), .rst(rst), .display_valid_src(display_valid),
        .display_index_src(display_bank), .pixel_clk(pix_clk),
        .frame_request(frame_req), .reader_frame_start(read_start),
        .display_valid_pix(pix_valid), .display_index_pix(pix_bank),
        .audio_commit_toggle(audio_toggle)
    );
    media_hdmi_audio_bridge #(.BLIP_SAMPLES(2), .NOTE_SAMPLES(4)) audio (
        .pixel_clk(pix_clk), .rst(rst), .display_valid_sd(pix_valid),
        .media_id_sd(pix_bank), .commit_toggle_sd(audio_toggle),
        .audio_valid(audio_valid), .audio_left_data(audio_left),
        .audio_right_data(), .acr_valid(), .acr_cts(), .acr_n()
    );
    always @(posedge sd_clk)
        if (!rst && control.load_start_pulse) load_count=load_count+1;

    task frame;
        input integer expected;
        begin
            // Allow the asynchronous snapshot to arrive; no visible change yet.
            repeat(20) @(negedge pix_clk);
            frame_req=1;
            #1;
            if (read_start !== (expected>=0)) $fatal(1,"Incorrect reader start");
            @(negedge pix_clk); frame_req=0;
            repeat(8) @(negedge pix_clk);
            if (pix_valid !== (expected>=0) || audio.media_active !== (expected>=0))
                $fatal(1,"Display/audio validity mismatch");
            if (expected>=0 && (pix_bank !== expected[1:0] || audio.media_id !== expected[1:0]))
                $fatal(1,"Video/audio bank mismatch: want %0d got %0d/%0d",expected,pix_bank,audio.media_id);
        end
    endtask
    task scan;
        begin
            wait(control.scan_start_pulse);
            repeat(3) @(negedge sd_clk);
            scan_done=1;
            @(negedge sd_clk); scan_done=0;
        end
    endtask
    task source_finish;
        begin
            @(negedge sd_clk); source_done=1;
            @(negedge sd_clk); source_done=0;
        end
    endtask
    task finish_bank;
        input integer bank;
        begin
            wait(control.load_busy && write_bank==bank);
            repeat(3) @(negedge sd_clk);
            if (bank[0]) begin
                write_toggle=~write_toggle;
                repeat(10) @(negedge sd_clk);
                if(control.image_complete[bank]) $fatal(1,"Committed before source done");
                source_finish;
            end else begin
                source_finish;
                repeat(10) @(negedge sd_clk);
                if(control.image_complete[bank]) $fatal(1,"Committed before SDRAM drain");
                write_toggle=~write_toggle;
            end
            wait(control.image_complete[bank]);
            repeat(20) @(negedge pix_clk);
        end
    endtask
    task next_key;
        begin
            @(negedge sd_clk); key_next=0;
            repeat(65) @(negedge sd_clk);
            key_next=1;
            repeat(65) @(negedge sd_clk);
        end
    endtask
    task auto_key;
        input integer cycles;
        begin
            @(negedge sd_clk); key_auto=0;
            repeat(cycles) @(negedge sd_clk);
            key_auto=1;
            repeat(65) @(negedge sd_clk);
        end
    endtask
    task reset_control;
        begin
            @(negedge sd_clk); rst=1; write_toggle=0; source_done=0; source_error=0;
            key_next=1; key_auto=1; key_audio_mode=1;
            repeat(10) @(negedge sd_clk);
            rst=0; load_count=0;
        end
    endtask
    reg saved_toggle;
    integer i;
    initial begin
        force control.sd_init_done=1'b1;
        force control.source_ready=1'b1;
        force control.source_load_busy=1'b0;
        force control.scan_busy=1'b0;
        force control.scan_done=scan_done;
        force control.scan_error=8'd0;
        force control.resource_valid=resources;
        force control.resource0_cluster=32'd10;
        force control.resource1_cluster=32'd20;
        force control.resource2_cluster=32'd30;
        force control.resource3_cluster=32'd40;
        force control.resource0_size=32'd921654;
        force control.resource1_size=32'd921654;
        force control.resource2_size=32'd921654;
        force control.resource3_size=32'd921654;
        force control.source_load_done=source_done;
        force control.source_load_error=source_error;
        reset_control;
        scan;
        frame(-1);
        finish_bank(0);
        if(pix_valid || audio.media_active) $fatal(1,"First picture audible before frame boundary");
        frame(0);
        saved_toggle=audio_toggle;
        for(i=1;i<4;i=i+1) begin
            finish_bank(i);
            frame(0);
            if(audio_toggle!==saved_toggle) $fatal(1,"Preloading changed the melody");
        end
        if(control.image_complete!==4'b1111 || load_count!=4) $fatal(1,"Four-bank preload failed");
        // Board KEY3: reject bounce, switch on a short press, never change images.
        repeat(4) begin
            @(negedge sd_clk); key_audio_mode=0;
            repeat(5) @(negedge sd_clk);
            key_audio_mode=1;
            repeat(5) @(negedge sd_clk);
        end
        repeat(65) @(negedge sd_clk);
        if(control.pcm_mode) $fatal(1,"KEY3 bounce switched audio mode");
        @(negedge sd_clk); key_audio_mode=0;
        repeat(65) @(negedge sd_clk);
        if(!control.pcm_mode || display_bank!=0 || control.auto_play_en)
            $fatal(1,"Short KEY3 did not switch only the audio mode");
        key_audio_mode=1;
        repeat(65) @(negedge sd_clk);
        if(!control.pcm_mode || display_bank!=0) $fatal(1,"KEY3 release changed mode/image");
        // A second press switches back; holding it must not repeat.
        @(negedge sd_clk); key_audio_mode=0;
        repeat(65) @(negedge sd_clk);
        if(control.pcm_mode) $fatal(1,"Second KEY3 press did not switch back");
        repeat(600) @(negedge sd_clk);
        if(control.pcm_mode || display_bank!=0) $fatal(1,"Held KEY3 repeated or changed image");
        key_audio_mode=1;
        repeat(65) @(negedge sd_clk);
        // Board KEY2 long hold no longer changes audio; release advances once.
        @(negedge sd_clk); key_next=0;
        repeat(600) @(negedge sd_clk);
        if(control.pcm_mode || display_bank!=0) $fatal(1,"Held KEY2 switched audio/image");
        key_next=1;
        repeat(65) @(negedge sd_clk);
        if(control.pcm_mode || display_bank!=1) $fatal(1,"KEY2 release did not advance once");
        for(i=2;i<=4;i=i+1) next_key;
        frame(0);
        for(i=1;i<=4;i=i+1) begin
            next_key;
            if(control.pcm_mode) $fatal(1,"Short KEY2 changed audio mode");
            if(audio.media_id!==pix_bank) $fatal(1,"Audio changed before video");
            frame(i%4);
        end
        // Board KEY4 long press: 0 -> 3. Short press toggles automatic playback.
        auto_key(280); frame(3);
        auto_key(65);
        wait(control.auto_play_en);
        wait(display_bank==0);
        frame(0);
        auto_key(65);
        if(control.auto_play_en) $fatal(1,"Short press did not stop automatic playback");
        if(load_count!=4) $fatal(1,"Switching cached banks reread the SD card");

        // Sparse directory: only PIC2 exists; compact image/bank 0 is used.
        resources=4'b0010;
        reset_control; scan;
        wait(control.load_busy);
        #1;
        if(control.load_cluster_lut!=20) $fatal(1,"Sparse FAT directory mapped wrong cluster: mask=%b cluster=%0d slot=%0d",control.image_resource_valid,control.load_cluster_lut,control.img_cluster1);
        finish_bank(0); frame(0);
        saved_toggle=audio_toggle;
        next_key; frame(0);
        if(load_count!=1 || audio_toggle!==saved_toggle) $fatal(1,"Single image restarted audio or reloaded");

        // A failed background decode must keep the first picture and melody.
        resources=4'b1111;
        reset_control; scan; finish_bank(0); frame(0);
        wait(control.load_busy && write_bank==1);
        @(negedge sd_clk); source_error=8'h25;
        source_finish;
        repeat(10) @(negedge sd_clk);
        if(!control.preload_halted || control.image_complete!==4'b0001)
            $fatal(1,"Failed preload was committed");
        saved_toggle=audio_toggle;
        next_key; frame(0);
        if(audio_toggle!==saved_toggle) $fatal(1,"Failed image changed audio");
        $display("PASS: KEY3 short/bounce/hold, KEY2 image-only, four-bank preload, frame/audio commit, next/previous/auto, sparse/single image and failed load");
        $finish;
    end
    initial begin #5000000; $fatal(1,"Four-buffer audio test timeout"); end
endmodule
