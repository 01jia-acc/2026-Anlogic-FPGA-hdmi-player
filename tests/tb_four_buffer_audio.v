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
    reg [15:0] resources=16'h000f;
    reg [7:0] source_error=0;
    wire display_valid, pix_valid, read_start, audio_toggle;
    wire [1:0] write_bank, display_bank, pix_bank;
    wire [3:0] image_id, pix_image_id, ack_image_id;
    wire [1:0] ack_bank;
    wire ack_valid;
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
        .display_image_id(image_id), .display_ack_valid(ack_valid),
        .display_ack_bank(ack_bank), .display_ack_image_id(ack_image_id),
        .write_finish_toggle(write_toggle), .write_buf_idx(write_bank),
        .disp_buf_idx(display_bank), .write_req(), .write_req_ack(1'b1),
        .write_en(), .write_data(), .write_frame_commit(),
        .SD_nCS(), .SD_DCLK(), .SD_MOSI(), .SD_MISO(1'b1)
    );
    media_display_commit commit (
        .src_clk(sd_clk), .rst(rst), .display_valid_src(display_valid),
        .display_index_src(display_bank), .pixel_clk(pix_clk),
        .display_image_id_src(image_id), .display_image_id_pix(pix_image_id),
        .display_ack_valid_src(ack_valid), .display_ack_bank_src(ack_bank),
        .display_ack_image_id_src(ack_image_id),
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
            wait(control.load_prepare == 0);
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
        // Scheduler now reads the scanner's single packed resource table.
        force control.resource_clusters={384'd0,32'd40,32'd30,32'd20,32'd10};
        force control.resource_sizes={384'd0,{4{32'd921654}}};
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
        if(control.pcm_mode) $fatal(1,"KEY3 switched before release");
        key_audio_mode=1;
        repeat(65) @(negedge sd_clk);
        if(!control.pcm_mode || display_bank!=0) $fatal(1,"KEY3 release changed mode/image");
        // A second short press switches back on release.
        @(negedge sd_clk); key_audio_mode=0;
        repeat(65) @(negedge sd_clk);
        key_audio_mode=1;
        repeat(65) @(negedge sd_clk);
        if(control.pcm_mode) $fatal(1,"Second KEY3 release did not switch back");
        // Long KEY3 enters zoom exactly once and does not change audio.
        key_audio_mode=0;
        repeat(650) @(negedge sd_clk);
        if(!control.zoom_mode || control.pcm_mode) $fatal(1,"Long KEY3 did not enter zoom once");
        key_audio_mode=1;
        repeat(65) @(negedge sd_clk);
        next_key;
        if(control.zoom_level!=2 || display_bank!=0) $fatal(1,"Zoom in 1.5x failed");
        next_key; next_key;
        if(control.zoom_level!=3 || display_bank!=0) $fatal(1,"Zoom maximum failed");
        auto_key(65); auto_key(280); auto_key(65); auto_key(65);
        if(control.zoom_level!=0 || display_bank!=0 || control.auto_play_en)
            $fatal(1,"Zoom shrink/minimum/navigation isolation failed");
        // Automatic playback retains its setting but its timer pauses.
        force control.auto_play_en=1'b1;
        repeat(5200) @(negedge sd_clk);
        if(display_bank!=0 || control.auto_cnt!=0) $fatal(1,"Zoom did not pause autoplay");
        release control.auto_play_en;
        key_audio_mode=0;
        repeat(280) @(negedge sd_clk);
        key_audio_mode=1;
        repeat(65) @(negedge sd_clk);
        if(control.zoom_mode || !control.auto_play_en || control.pcm_mode)
            $fatal(1,"Zoom exit/autoplay setting failed");
        auto_key(65);
        if(control.auto_play_en) $fatal(1,"Cannot stop resumed autoplay");
        // Board KEY2 long hold no longer changes audio; release advances once.
        @(negedge sd_clk); key_next=0;
        repeat(600) @(negedge sd_clk);
        if(control.pcm_mode || display_bank!=0) $fatal(1,"Held KEY2 switched audio/image");
        key_next=1;
        repeat(65) @(negedge sd_clk);
        if(control.pcm_mode || display_bank!=1) $fatal(1,"KEY2 release did not advance once");
        frame(1);
        if(control.zoom_level!=1) $fatal(1,"New image did not reset zoom");
        for(i=2;i<=4;i=i+1) begin next_key; frame(i%4); end
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
        wait(control.load_prepare == 0);
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
