`timescale 1ns/1ps
// Real keys/scheduler/display commit/mailbox. File completion and the reader's
// one-clock-delayed descriptor capture are modeled; reader RGB tested separately.
module tb_thumbnail_control #(parameter SWITCH_TEST=0, parameter VOLUME_TEST=0, parameter BRIGHTNESS_TEST=0);
    reg clk=0, pix_clk=0, rst=1;
    always #5 clk=~clk;
    always #20 pix_clk=~pix_clk;
    reg key_reset=1, key_next=1, key_auto=1, key_audio=1, frame_req=0, write_toggle=0;
    reg switch_pcm_n=1, switch_zoom_n=1;
    reg switch_volume_n=1;
    reg switch_brightness_n=1;
    wire brightness_mode;
    wire [4:0] brightness_level;
    wire [5:0] contrast_level;
    wire volume_mode;
    wire [4:0] volume_level;
    reg scan_done=0, source_done=0;
    reg [15:0] resources=16'hffff;
    reg [511:0] clusters=0, sizes=0;
    wire display_valid, ack_valid, audio_toggle, zoom_mode, pcm_mode;
    wire [1:0] display_bank, write_bank, ack_bank, zoom_level;
    wire [3:0] image_id, ack_id;
    wire [3:0] committed_id;
    wire [31:0] descriptor;
    wire [95:0] mailbox;
    wire mailbox_ready;
    wire [31:0] reader_descriptor;
    reg [31:0] displayed=0;
    reg capture_pending=0;
    integer loads=0, i, saved_loads, loading_bank, k;
    reg saved_audio;
    sd_card_bmp_fat32 #(.CLK_FREQ_HZ(1000), .LONG_PRESS_MS(200),
        .LOAD_TIMEOUT_MS(200000), .PCM_ENABLE(1), .THUMBNAIL_ENABLE(1),
        .SWITCH_MODE_ENABLE(SWITCH_TEST), .VOLUME_CONTROL_ENABLE(VOLUME_TEST || BRIGHTNESS_TEST),
        .BRIGHTNESS_CONTROL_ENABLE(BRIGHTNESS_TEST)) dut (
        .clk(clk), .rst(rst), .pixel_clk(pix_clk), .pcm_ready(1'b0),
        .key_next(key_next), .key_auto(key_auto), .key_audio_mode(key_audio),
        .switch_pcm_n(switch_pcm_n), .switch_zoom_n(switch_zoom_n),
        .switch_volume_n(switch_volume_n), .volume_mode(volume_mode), .volume_level(volume_level),
        .switch_brightness_n(switch_brightness_n), .brightness_mode(brightness_mode), .brightness_level(brightness_level),
        .key_contrast_up(key_reset), .contrast_level(contrast_level),
        .bmp_width(16'd640), .bmp_height(16'd480), .SD_MISO(1'b1),
        .display_valid(display_valid), .display_image_id(image_id),
        .display_ack_valid(ack_valid), .display_ack_bank(ack_bank), .display_ack_image_id(ack_id),
        .disp_buf_idx(display_bank), .write_buf_idx(write_bank), .write_finish_toggle(write_toggle),
        .write_req_ack(1'b1), .zoom_mode(zoom_mode), .zoom_level(zoom_level), .pcm_mode(pcm_mode),
        .thumbnail_descriptor(descriptor), .thumbnail_ack_epoch(displayed[22])
    );
    media_display_commit #(.WIPE_FRAMES(4)) commit (
        .src_clk(clk), .rst(rst), .display_valid_src(display_valid),
        .display_index_src(display_bank), .display_image_id_src(image_id),
        .pixel_clk(pix_clk), .frame_request(frame_req), .audio_commit_toggle(audio_toggle),
        .display_image_id_pix(committed_id),
        .display_ack_valid_src(ack_valid), .display_ack_bank_src(ack_bank), .display_ack_image_id_src(ack_id)
    );
    thumbnail_exit_gate exit_gate(.requested(mailbox[31:0]), .displayed(displayed),
        .committed_image(committed_id), .descriptor(reader_descriptor));
    subtitle_frame_commit packet (
        .src_clk(clk), .src_rst_n(~rst), .src_commit(mailbox_ready), .src_ready(mailbox_ready),
        .src_text({64'd0,descriptor}), .src_length(6'd0), .src_enable(1'b1),
        .dst_clk(pix_clk), .dst_rst_n(~rst), .frame_start(1'b1), .dst_text(mailbox)
    );
    always @(posedge pix_clk) begin
        if(rst) begin displayed<=0; capture_pending<=0; end
        else begin
            capture_pending<=frame_req;
            if(capture_pending) displayed<=reader_descriptor;
        end
    end
    always @(posedge clk) if(!rst && dut.load_start_pulse) loads=loads+1;
    always @(posedge clk) if(!rst && dut.load_start_pulse && display_valid &&
        (write_bank==display_bank || (ack_valid && write_bank==ack_bank)))
        $fatal(1,"Loading overwrites current or acknowledged bank");
    task frame;
        integer f;
        begin
            for(f=0;f<4;f=f+1) begin
                repeat(30) @(negedge pix_clk); frame_req=1;
                @(negedge pix_clk); frame_req=0;
                repeat(30) @(negedge pix_clk);
            end
        end
    endtask
    task brightness_scenarios;
        integer n;
        reg [31:0] saved_descriptor;
        begin
            fill_window;
            if(brightness_mode || brightness_level!=20 || contrast_level!=20 || volume_level!=15) $fatal(1,"Image controls startup");
            repeat(5) begin
                switch_brightness_n=0; repeat(10) @(negedge clk);
                switch_brightness_n=1; repeat(10) @(negedge clk);
                if(brightness_mode) $fatal(1,"SW1 bounce");
            end
            switch_brightness_n=0; repeat(65) @(negedge clk);
            press(4,65); if(brightness_level!=19) $fatal(1,"Brightness down");
            press(3,65); if(brightness_level!=20 || dut.thumbnail_pending) $fatal(1,"Brightness up / KEY3 isolation");
            press(1,65); if(contrast_level!=21) $fatal(1,"Contrast up");
            press(2,65); if(contrast_level!=20) $fatal(1,"Contrast down");
            press(4,280); press(3,280); press(2,280); press(1,280);
            if(brightness_level!=20 || contrast_level!=20 || image_id!=0 || dut.auto_play_en)
                $fatal(1,"Adjustment long press isolation");
            for(n=0;n<25;n=n+1) press(4,65);
            if(brightness_level!=0) $fatal(1,"Brightness lower clamp");
            for(n=0;n<25;n=n+1) press(3,65);
            if(brightness_level!=20) $fatal(1,"Brightness upper clamp");
            for(n=0;n<45;n=n+1) press(2,65);
            if(contrast_level!=0) $fatal(1,"Contrast lower clamp");
            for(n=0;n<45;n=n+1) press(1,65);
            if(contrast_level!=40 || volume_level!=15 || image_id!=0 || dut.auto_play_en)
                $fatal(1,"Contrast upper clamp / nav / volume isolation");
            switch_volume_n=0; repeat(65) @(negedge clk);
            press(4,65);
            if(volume_mode || !brightness_mode || brightness_level!=19 || volume_level!=15 || contrast_level!=40)
                $fatal(1,"SW1 must override SW2");
            // Hold through brightness -> volume: release must affect neither.
            key_next=0; key_auto=0; repeat(65) @(negedge clk);
            switch_brightness_n=1; repeat(65) @(negedge clk);
            key_next=1; key_auto=1; repeat(65) @(negedge clk);
            if(!volume_mode || brightness_level!=19 || contrast_level!=40 || volume_level!=15 || image_id!=0)
                $fatal(1,"Brightness to volume held cancellation");
            press(4,65); if(volume_level!=14 || brightness_level!=19) $fatal(1,"Restore volume keys");
            // Hold through volume -> brightness and confirm source independence.
            key_next=0; key_auto=0; repeat(65) @(negedge clk);
            switch_brightness_n=0; switch_pcm_n=0; repeat(65) @(negedge clk);
            key_next=1; key_auto=1; repeat(65) @(negedge clk);
            if(volume_level!=14 || brightness_level!=19 || contrast_level!=40 || !pcm_mode) $fatal(1,"Volume to brightness / SW3");
            switch_volume_n=1;
            // Hold through brightness -> navigation then navigation -> brightness.
            key_next=0; key_auto=0; repeat(65) @(negedge clk);
            switch_brightness_n=1; repeat(65) @(negedge clk);
            key_next=1; key_auto=1; repeat(65) @(negedge clk);
            if(image_id!=0 || dut.auto_play_en) $fatal(1,"Brightness exit held navigation");
            key_next=0; key_auto=0; repeat(65) @(negedge clk);
            switch_brightness_n=0; repeat(65) @(negedge clk);
            key_next=1; key_auto=1; repeat(65) @(negedge clk);
            if(brightness_level!=19 || contrast_level!=40 || image_id!=0 || dut.auto_play_en)
                $fatal(1,"Brightness entry held navigation");
            switch_zoom_n=0; repeat(65) @(negedge clk);
            press(4,65);
            if(brightness_level!=18 || zoom_level!=1) $fatal(1,"Brightness zoom priority");
            press(3,65); frame;
            if(descriptor[23] || dut.thumbnail_pending || brightness_level!=19)
                $fatal(1,"KEY3 must brighten without opening preview in SW1 mode");
            key_audio=0; repeat(65) @(negedge clk);
            switch_brightness_n=1; repeat(65) @(negedge clk);
            key_audio=1; repeat(65) @(negedge clk); frame;
            if(descriptor[23] || dut.thumbnail_pending || brightness_level!=19)
                $fatal(1,"Held KEY3 release must not open preview or adjust brightness");
            press(3,65); frame; saved_descriptor=descriptor;
            if(!descriptor[23]) $fatal(1,"Preview not restored when SW1 OFF");
            switch_brightness_n=0; repeat(65) @(negedge clk);
            press(2,65); press(4,65); press(3,65);
            if(brightness_level!=19 || contrast_level!=39 || descriptor!=saved_descriptor)
                $fatal(1,"Adjustment preview priority bri=%0d con=%0d descriptor=%h expected=%h",
                    brightness_level,contrast_level,descriptor,saved_descriptor);
            switch_brightness_n=1; repeat(65) @(negedge clk);press(2,65);
            if(descriptor[21:20]==saved_descriptor[21:20]) $fatal(1,"Preview navigation not restored");
            switch_zoom_n=1; repeat(65) @(negedge clk);frame;
            press(4,65); if(!dut.auto_play_en) $fatal(1,"Restore slideshow key");
            switch_brightness_n=0; repeat(4000) @(negedge clk);
            if(image_id!=0 || !dut.auto_play_en || dut.auto_cnt!=0) $fatal(1,"Brightness auto pause");
            switch_brightness_n=1; repeat(100) @(negedge clk);
            if(dut.auto_cnt==0 || brightness_level!=19 || contrast_level!=39) $fatal(1,"Adjustment retention/auto resume");
            press(4,65);press(2,65);frame;
            if(image_id!=1 || brightness_level!=19 || contrast_level!=39) $fatal(1,"Picture change adjustment retention");
            switch_brightness_n=0;switch_volume_n=0;boot;
            if(!brightness_mode || volume_mode || brightness_level!=20 || contrast_level!=20 || volume_level!=15)
                $fatal(1,"SW1 ON reset/defaults/priority");
            $display("PASS: SW1 brightness/contrast keys, 5%% steps/clamps/reset, priority, held gesture cancellation, KEY3 isolation, preview routing, slideshow pause/resume, picture retention");
        end
    endtask
    task volume_scenarios;
        integer n;
        reg [31:0] saved_descriptor;
        begin
            fill_window;
            if(volume_mode || volume_level!=15) $fatal(1,"Volume startup");
            repeat(5) begin
                switch_volume_n=0; repeat(10) @(negedge clk);
                switch_volume_n=1; repeat(10) @(negedge clk);
                if(volume_mode) $fatal(1,"SW2 bounce");
            end
            switch_volume_n=0; repeat(65) @(negedge clk);
            press(2,65); if(volume_level!=16) $fatal(1,"Volume up");
            press(4,65); if(volume_level!=15) $fatal(1,"Volume down");
            press(2,280); press(4,280);
            if(volume_level!=15 || image_id!=0 || dut.auto_play_en)
                $fatal(1,"Volume long hold leaked to navigation");
            for(n=0;n<25;n=n+1) press(2,65);
            if(volume_level!=20) $fatal(1,"Upper clamp");
            for(n=0;n<25;n=n+1) press(4,65);
            if(volume_level!=0 || image_id!=0 || dut.auto_play_en) $fatal(1,"Lower clamp/nav isolation");
            // A held gesture must not become a navigation event on SW2 OFF.
            key_next=0; key_auto=0; repeat(65) @(negedge clk);
            switch_volume_n=1; repeat(65) @(negedge clk);
            key_next=1; key_auto=1; repeat(65) @(negedge clk);
            if(image_id!=0 || dut.auto_play_en || volume_level!=0) $fatal(1,"OFF held gesture");
            // Nor should a normal-mode gesture become volume on SW2 ON.
            key_next=0; key_auto=0; repeat(65) @(negedge clk);
            switch_volume_n=0; repeat(65) @(negedge clk);
            key_next=1; key_auto=1; repeat(65) @(negedge clk);
            if(image_id!=0 || dut.auto_play_en || volume_level!=0) $fatal(1,"ON held gesture");
            switch_zoom_n=0; repeat(65) @(negedge clk);
            press(2,65); press(2,65);
            if(volume_level!=2 || zoom_level!=1) $fatal(1,"Zoom priority");
            press(3,65); frame; saved_descriptor=descriptor;
            press(2,65); press(4,65);
            if(volume_level!=2 || descriptor!=saved_descriptor) $fatal(1,"Preview priority");
            switch_volume_n=1; repeat(65) @(negedge clk);
            press(2,65);
            if(descriptor[21:20]==saved_descriptor[21:20]) $fatal(1,"Preview navigation restore");
            switch_zoom_n=1; repeat(65) @(negedge clk); frame;
            press(4,65); if(!dut.auto_play_en) $fatal(1,"Auto toggle restore");
            switch_volume_n=0; repeat(4000) @(negedge clk);
            if(image_id!=0 || !dut.auto_play_en || dut.auto_cnt!=0) $fatal(1,"Auto pause");
            switch_volume_n=1; repeat(100) @(negedge clk);
            if(dut.auto_cnt==0) $fatal(1,"Auto resume");
            switch_volume_n=0; boot;
            if(!volume_mode || volume_level!=15) $fatal(1,"SW2 reset ON");
            $display("PASS: SW2 debounce, 5%% steps/clamps/reset, short/long presses, held-mode cancellation, normal/zoom/preview ownership, auto pause/resume");
        end
    endtask
    task switch_scenarios;
        begin
            if(pcm_mode || zoom_mode) $fatal(1,"OFF switches startup");
            // No short contact bounce may change the selected mode.
            repeat(5) begin
                switch_pcm_n=0; switch_zoom_n=0; repeat(10) @(negedge clk);
                switch_pcm_n=1; switch_zoom_n=1; repeat(10) @(negedge clk);
                if(pcm_mode || zoom_mode) $fatal(1,"Switch bounce leaked");
            end
            press(3,65); press(3,280);
            if(pcm_mode || zoom_mode) $fatal(1,"KEY3 still changes modes");
            switch_pcm_n=0; switch_zoom_n=0; repeat(65) @(negedge clk);
            if(!pcm_mode || !zoom_mode) $fatal(1,"Independent ON modes");
            press(2,65);
            if(zoom_level!=2 || image_id!=0) $fatal(1,"Switch zoom key routing");
            press(3,65);
            if(!dut.thumbnail_pending) $fatal(1,"Queued menu entry");
            switch_zoom_n=1; repeat(65) @(negedge clk);
            if(zoom_mode || dut.thumbnail_pending || descriptor[23] || !pcm_mode)
                $fatal(1,"OFF must cancel queued menu without changing PCM");
            fill_window;
            switch_zoom_n=0; repeat(65) @(negedge clk);
            press(3,65);
            if(!descriptor[23]) $fatal(1,"Menu entry");
            // Switch OFF before the reader has acknowledged entry: keep bank
            // protection until entry ACK, then publish cancel, then await exit ACK.
            switch_zoom_n=1; repeat(65) @(negedge clk);
            if(!descriptor[23] || !zoom_mode) $fatal(1,"Unacknowledged entry was abandoned");
            frame;
            if(descriptor[23] || zoom_mode || dut.thumbnail_hold || image_id!=0)
                $fatal(1,"Safe switch exit failed");
            switch_zoom_n=0; repeat(65) @(negedge clk);
            press(3,65); frame; press(2,65); press(3,65); frame;
            if(image_id!=1 || !zoom_mode || descriptor[23] || !pcm_mode)
                $fatal(1,"Switch menu confirmation");
            fill_window;
            press(3,65); frame; press(3,280); frame;
            if(descriptor[23] || !zoom_mode) $fatal(1,"Long KEY3 cancel must stay in zoom");
            switch_pcm_n=1; repeat(65) @(negedge clk);
            if(pcm_mode || !zoom_mode) $fatal(1,"Audio OFF independence");
            // Levels survive reset: ON does not require a new toggle edge.
            switch_pcm_n=0; boot;
            if(!pcm_mode || !zoom_mode) $fatal(1,"ON switches after reset");
            switch_pcm_n=1; switch_zoom_n=1; repeat(65) @(negedge clk);
            if(pcm_mode || zoom_mode) $fatal(1,"Both switches OFF");
            $display("PASS: SW3/SW4 debounce, startup/reset ON/OFF, independent audio/zoom, KEY3 menu routing, queued entry cancellation, pre-ACK exit bank protection, confirm/cancel");
        end
    endtask
    task fill_window;
        integer attempt;
        begin
            for(attempt=0;attempt<6;attempt=attempt+1) begin
                repeat(80) @(negedge clk);
                if(dut.load_busy) finish_load(dut.load_idx);
                frame;
            end
            if(dut.window_missing || dut.load_busy) $fatal(1,"Window not filled");
        end
    endtask
    task check_window;
        input integer first, current;
        integer s, physical;
        begin
            if(image_id!=current || descriptor[19:16]!=15) $fatal(1,"Window current/valid mismatch");
            for(s=0;s<4;s=s+1) begin
                physical=descriptor[24+s*2 +:2];
                if(descriptor[s*4 +:4]!=first+s || !dut.image_complete[physical] ||
                   dut.bank_image_ids[physical*4 +:4]!=first+s)
                    $fatal(1,"Window current=%0d slot=%0d logical=%0d physical=%0d",current,s,descriptor[s*4 +:4],physical);
            end
        end
    endtask
    task press;
        input integer button, duration;
        begin
            @(negedge clk);
            case(button) 1:key_reset=0; 2:key_next=0; 3:key_audio=0; 4:key_auto=0; endcase
            repeat(duration) @(negedge clk);
            key_reset=1; key_next=1; key_audio=1; key_auto=1;
            repeat(65) @(negedge clk);
        end
    endtask
    task finish_load;
        input integer expected;
        begin
            wait(dut.load_busy && dut.load_prepare==0);
            repeat(8) @(negedge clk);
            if(dut.load_idx!=expected) $fatal(1,"Wrong prefetch image %0d",dut.load_idx);
            source_done=1;
            @(negedge clk); source_done=0;
            repeat(8) @(negedge clk);
            write_toggle=~write_toggle;
            wait(!dut.load_busy);
            repeat(8) @(negedge clk);
        end
    endtask
    task boot;
        begin
            rst=1; key_reset=1; key_next=1; key_auto=1; key_audio=1; write_toggle=0;
            repeat(12) @(negedge clk); rst=0; loads=0;
            wait(dut.scan_start_pulse); repeat(4) @(negedge clk);
            scan_done=1; @(negedge clk); scan_done=0;
            finish_load(0); frame;
        end
    endtask
    initial begin
        for(i=0;i<16;i=i+1) begin clusters[i*32 +:32]=(i+1)*10; sizes[i*32 +:32]=921654; end
        force dut.sd_init_done=1'b1; force dut.source_ready=1'b1;
        force dut.source_load_busy=1'b0; force dut.scan_busy=1'b0;
        force dut.scan_done=scan_done; force dut.scan_error=8'd0;
        force dut.resource_valid=resources; force dut.resource_clusters=clusters;
        force dut.resource_sizes=sizes;
        force dut.resource0_cluster=32'd10; force dut.resource1_cluster=32'd20;
        force dut.resource2_cluster=32'd30; force dut.resource3_cluster=32'd40;
        force dut.resource0_size=32'd921654; force dut.resource1_size=32'd921654;
        force dut.resource2_size=32'd921654; force dut.resource3_size=32'd921654;
        force dut.source_load_done=source_done; force dut.source_load_error=8'd0;
        boot;
        if(BRIGHTNESS_TEST) begin brightness_scenarios; $finish; end
        if(VOLUME_TEST) begin volume_scenarios; $finish; end
        if(SWITCH_TEST) begin switch_scenarios; $finish; end
        // Entry queues until the complete logical window is available.
        press(3,280); press(3,65);
        if(descriptor[23] || !dut.thumbnail_pending) $fatal(1,"Entry did not wait for loading");
        press(3,280);
        if(dut.thumbnail_pending || !zoom_mode) $fatal(1,"Pending entry cancel failed");
        press(3,65);
        finish_load(1); finish_load(2); finish_load(3);
        repeat(80) @(negedge clk);
        if(!descriptor[23]) $fatal(1,"Queued menu did not open");
        press(3,65); // entry not ACKed, ignore confirm
        if(!descriptor[23]) $fatal(1,"Premature confirm");
        frame; check_window(0,0);
        saved_loads=loads;
        press(3,280);
        if(!dut.thumbnail_hold || descriptor[23]) $fatal(1,"Cancel lock");
        frame; press(3,280);
        press(3,65); if(!pcm_mode) $fatal(1,"Normal audio toggle");
        press(3,65); if(pcm_mode) $fatal(1,"Normal audio toggle back");
        // Sweep every page forward, including start/end clamping.
        for(i=1;i<10;i=i+1) begin
            press(2,65); frame; fill_window;
            check_window(i<2 ? 0 : i>8 ? 6 : i-2,i);
        end
        // Playback still wraps; the preview window itself never wraps.
        if(dut.image_read_needed) $fatal(1,"Settled last window unnecessarily blocks PCM");
        press(2,65); finish_load(0); frame; fill_window; check_window(0,0);
        press(4,280); finish_load(9); frame; fill_window; check_window(6,9);
        // Backward navigation must refill the opposite side too.
        for(i=8;i>=2;i=i-1) begin
            press(4,280); frame; fill_window;
            check_window(i>8 ? 6 : i-2,i);
        end
        check_window(0,2); // user image3 -> 1,2,3,4
        press(3,280); press(2,65); press(3,65); frame;
        if(!descriptor[23] || descriptor[21:20]!=2 || zoom_level!=2) $fatal(1,"Current logical focus");
        saved_audio=audio_toggle;
        for(k=0;k<4;k=k+1) press(2,65);
        if(descriptor[21:20]!=2 || audio_toggle!=saved_audio) $fatal(1,"Focus wrap/audio");
        press(4,65); // choose previous logical image; physical bank can be anything
        loading_bank=descriptor[26 +:2]; saved_loads=loads;
        @(negedge clk); key_next=0; key_audio=0;
        repeat(65) @(negedge clk); key_next=1; key_audio=1;
        repeat(65) @(negedge clk);
        if(descriptor[23] || image_id!=1 || display_bank!=loading_bank || zoom_level!=1 || !zoom_mode)
            $fatal(1,"Logical selection/confirm priority");
        repeat(500) @(negedge clk);
        if(loads!=saved_loads || !dut.thumbnail_hold || descriptor[7:4]!=1)
            $fatal(1,"Exit descriptor/window changed before ACK");
        frame; fill_window; check_window(0,1);
        if(dut.thumbnail_hold || ack_id!=1) $fatal(1,"Exit ACK");
        press(2,65); press(3,65); frame;
        press(3,650);
        if(descriptor[23] || !zoom_mode || zoom_level!=2 || image_id!=1 || pcm_mode)
            $fatal(1,"Cancel must preserve zoom/image");
        frame; press(3,280);
        // Two and three resources must leave only truly absent slots empty.
        resources=16'h0201; boot; finish_load(1); frame;
        press(3,280); press(3,65); frame;
        if(!descriptor[23] || descriptor[19:16]!=3 || descriptor[7:0]!=8'h10)
            $fatal(1,"Two-image window");
        resources=16'h0203; boot; finish_load(1); finish_load(2); frame;
        press(3,280); press(3,65); frame;
        if(!descriptor[23] || descriptor[19:16]!=7 || descriptor[11:0]!=12'h210)
            $fatal(1,"Three-image window");
        // Sparse/single-image directory: absent slots remain unselectable.
        resources=16'h0200; boot;
        press(3,280); press(3,65); frame;
        press(4,65); press(3,65);
        if(!descriptor[23] || descriptor[19:16]!=1 || image_id!=0)
            $fatal(1,"Single-image empty slot confirmation");
        rst=1; repeat(12) @(negedge clk);
        if(dut.thumbnail_hold || descriptor[23] || zoom_mode) $fatal(1,"Menu reset failed");
        $display("PASS: ordered four-image window all 10 pages forward/back, edge clamping, queued entry/cancel, nonphysical slot selection, confirm priority, exit bank lock/wipe ACK, sparse/single/reset");
        $finish;
    end
    initial begin #20000000; $fatal(1,"Thumbnail controller timeout"); end
endmodule

module tb_mode_switch_control;
    tb_thumbnail_control #(.SWITCH_TEST(1)) scenario();
endmodule
