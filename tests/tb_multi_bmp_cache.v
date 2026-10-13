`timescale 1ns/1ps
// Real scheduler and bidirectional frame commit; mock only file/write completion.
module tb_multi_bmp_cache;
    reg clk=0, pixel_clk=0, rst=1;
    always #5 clk=~clk;
    always #17 pixel_clk=~pixel_clk;
    reg key_next=1, key_auto=1, frame_req=0, write_toggle=0;
    reg scan_done=0, source_done=0;
    reg [15:0] resources=16'hffff;
    reg [511:0] clusters=0, sizes=0;
    reg [7:0] source_error=0;
    wire display_valid, pix_valid, ack_valid, audio_toggle;
    wire [1:0] write_bank, display_bank, pix_bank, ack_bank;
    wire [3:0] image_id, pix_id, ack_id;
    integer loads=0, i, saved_loads, old_id, bank, boot_count;
    reg saved_toggle;
    sd_card_bmp_fat32 #(.CLK_FREQ_HZ(1000), .LONG_PRESS_MS(200),
        .LOAD_TIMEOUT_MS(2000), .PCM_ENABLE(1)) dut (
        .clk(clk), .rst(rst), .pixel_clk(pixel_clk), .pcm_ready(1'b0),
        .key_next(key_next), .key_auto(key_auto), .key_audio_mode(1'b1),
        .bmp_width(16'd640), .bmp_height(16'd480),
        .display_valid(display_valid), .display_image_id(image_id),
        .display_ack_valid(ack_valid), .display_ack_bank(ack_bank),
        .display_ack_image_id(ack_id), .write_finish_toggle(write_toggle),
        .write_buf_idx(write_bank), .disp_buf_idx(display_bank),
        .write_req_ack(1'b1), .SD_MISO(1'b1)
    );
    media_display_commit commit (
        .src_clk(clk), .rst(rst), .display_valid_src(display_valid),
        .display_index_src(display_bank), .display_image_id_src(image_id),
        .pixel_clk(pixel_clk), .frame_request(frame_req),
        .display_valid_pix(pix_valid), .display_index_pix(pix_bank),
        .display_image_id_pix(pix_id), .audio_commit_toggle(audio_toggle),
        .display_ack_valid_src(ack_valid), .display_ack_bank_src(ack_bank),
        .display_ack_image_id_src(ack_id)
    );
    always @(posedge clk) if(!rst && dut.load_start_pulse) begin
        loads=loads+1;
        if(pix_valid && write_bank==pix_bank)
            $fatal(1,"Writing currently displayed bank: image %0d",pix_id);
        if(display_valid && write_bank==display_bank)
            $fatal(1,"Writing requested display bank");
    end
    task frame;
        input integer expected;
        begin
            repeat(30) @(negedge pixel_clk);
            frame_req=1;
            @(negedge pixel_clk); frame_req=0;
            repeat(30) @(negedge pixel_clk);
            if(!pix_valid || pix_id!==expected[3:0] || !ack_valid ||
               ack_id!==expected[3:0] || ack_bank!==pix_bank)
                $fatal(1,"Frame/ack mismatch want=%0d got=%0d/%0d",expected,pix_id,ack_id);
        end
    endtask
    task finish_load;
        input integer expected;
        integer slot, logical;
        begin
            wait(dut.load_busy);
            repeat(4) @(negedge clk);
            bank=write_bank;
            if(dut.load_idx!==expected[3:0]) $fatal(1,"Wrong logical load id");
            // Check actual file selection, including sparse PIC10/PIC16 slots.
            logical=0;
            for(slot=0;slot<16;slot=slot+1) if(resources[slot]) begin
                if(logical==expected &&
                   (dut.load_cluster_lut!==clusters[slot*32 +:32] ||
                    dut.load_size_lut!==sizes[slot*32 +:32]))
                    $fatal(1,"Wrong file selected: logical=%0d PIC%0d",expected,slot+1);
                logical=logical+1;
            end
            old_id=image_id;
            if(expected[0]) begin
                write_toggle=~write_toggle;
                repeat(12) @(negedge clk);
                if(dut.image_complete[bank] || image_id!=old_id)
                    $fatal(1,"Published before file completion");
                source_done=1;
                @(negedge clk); source_done=0;
            end else begin
                source_done=1;
                @(negedge clk); source_done=0;
                repeat(12) @(negedge clk);
                if(dut.image_complete[bank] || image_id!=old_id)
                    $fatal(1,"Published before SDRAM completion");
                write_toggle=~write_toggle;
            end
            wait(!dut.load_busy);
            if(!dut.image_complete[bank] || dut.bank_image_ids[bank*4 +:4]!=expected)
                $fatal(1,"Cache entry not published");
        end
    endtask
    task next_key;
        begin
            @(negedge clk); key_next=0;
            repeat(65) @(negedge clk); key_next=1;
            repeat(65) @(negedge clk);
        end
    endtask
    task auto_key;
        input integer cycles;
        begin
            @(negedge clk); key_auto=0;
            repeat(cycles) @(negedge clk); key_auto=1;
            repeat(65) @(negedge clk);
        end
    endtask
    task navigate;
        input integer expected;
        begin
            if(dut.load_busy && dut.load_idx==expected) finish_load(expected);
            wait(image_id==expected);
            frame(expected);
        end
    endtask
    task boot;
        begin
            @(negedge clk); rst=1; write_toggle=0; source_done=0; source_error=0;
            key_next=1; key_auto=1;
            repeat(10) @(negedge clk); rst=0; loads=0;
            wait(dut.scan_start_pulse);
            repeat(3) @(negedge clk); scan_done=1;
            @(negedge clk); scan_done=0;
            finish_load(0); frame(0);
            boot_count=0;
            for(i=0;i<16;i=i+1) if(resources[i]) boot_count=boot_count+1;
            if(boot_count>4) boot_count=4;
            for(i=1;i<boot_count;i=i+1) begin finish_load(i); frame(0); end
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
        force dut.source_load_done=source_done; force dut.source_load_error=source_error;
        boot;
        if(dut.img_found_count!=16 || loads!=4) $fatal(1,"16-resource scan/preload failed");
        // The first three cache hits must not withdraw PCM read permission.
        for(i=1;i<=3;i=i+1) begin next_key; navigate(i); end
        wait(dut.load_busy);
        if(!dut.load_is_prefetch || image_id!=3 || pix_id!=3)
            $fatal(1,"Fifth image was not prefetched while displaying fourth");
        saved_toggle=audio_toggle;
        finish_load(4);
        frame(3);
        if(image_id!=3 || audio_toggle!=saved_toggle || loads!=5)
            $fatal(1,"Background completion changed display/audio or reread image");
        saved_loads=loads;
        next_key;
        if(image_id!=4 || loads!=saved_loads || !dut.u_image_source.pcm_play_enable)
            $fatal(1,"Prefetched cache hit reloaded or paused PCM");
        repeat(150) @(negedge clk);
        if(pix_id!=3 || dut.load_busy)
            $fatal(1,"Started prefetch before actual display acknowledgment");
        frame(4);
        wait(dut.load_busy);
        // Cached previous/next navigation stays responsive during background I/O.
        auto_key(280); navigate(3);
        if(dut.load_idx!=5 || !dut.load_busy) $fatal(1,"Cached previous interrupted prefetch");
        next_key; navigate(4);
        next_key; // Sixth image is still loading; keep the fifth visible.
        repeat(100) @(negedge clk);
        if(image_id!=4 || pix_id!=4 || !dut.next_req_pending)
            $fatal(1,"Displayed incomplete prefetch or lost pending key");
        finish_load(5);
        wait(image_id==5); frame(5);
        wait(dut.load_busy); finish_load(6);
        if(image_id!=5) $fatal(1,"Prefetch automatically advanced picture");

        // Read-ahead continues through all 16 images and the wrap to image zero.
        boot;
        for(i=1;i<16;i=i+1) begin
            next_key; navigate(i);
            if(dut.load_busy) finish_load((i+1)%16);
        end
        if(loads!=17) $fatal(1,"Full wrap did not prefetch image zero: %0d",loads);
        saved_loads=loads;
        next_key;
        if(image_id!=0 || loads!=saved_loads) $fatal(1,"Wrapped cache hit reread card");
        frame(0); wait(dut.load_busy); finish_load(1);
        auto_key(280); navigate(15);
        auto_key(280); navigate(14);
        auto_key(280); navigate(13); // A replaced previous image loads on demand.
        if(image_id!=13) $fatal(1,"Previous demand navigation failed");

        // Auto playback also crosses cache boundary and waits for each load.
        resources=16'h003f; boot;
        for(i=1;i<=3;i=i+1) begin next_key; navigate(i); end
        auto_key(65);
        wait(dut.load_busy); finish_load(4);
        wait(image_id==4); frame(4);
        wait(dut.load_busy); finish_load(5);
        wait(image_id==5); frame(5);
        wait(dut.load_busy); finish_load(0);
        wait(image_id==0); frame(0); auto_key(65);
        if(dut.auto_play_en) $fatal(1,"Could not stop auto playback");

        // Sparse high-number files compact to four logical images.
        resources=16'h8211; boot;
        if(dut.img_found_count!=4 || dut.bank_image_ids!=16'h3210 ||
           dut.image_clusters[9*32 +:32]!=100 || dut.image_clusters[15*32 +:32]!=160)
            $fatal(1,"Sparse upper slots missing");
        for(i=1;i<=4;i=i+1) begin next_key; navigate(i%4); end
        if(loads!=4) $fatal(1,"Sparse cached navigation reloaded");

        resources=16'h8000; boot;
        next_key; frame(0);
        if(loads!=1 || dut.load_busy) $fatal(1,"Single image unnecessarily prefetched");

        // A failed demand load preserves the previous display and cache safety.
        resources=16'hffff; boot;
        for(i=1;i<=3;i=i+1) begin next_key; navigate(i); end
        wait(dut.load_busy);
        @(negedge clk); source_error=8'h25; source_done=1;
        @(negedge clk); source_done=0;
        repeat(20) @(negedge clk);
        if(dut.load_busy || !dut.preload_halted || image_id!=3 || dut.last_error!=8'h25)
            $fatal(1,"Failed demand changed display or did not halt writer");
        frame(3);
        auto_key(280); navigate(2);
        next_key; navigate(3); next_key; frame(3);
        if(loads!=5) $fatal(1,"Failed writer reused");

        // Hung file/write completion must time out without publishing it.
        boot;
        for(i=1;i<=3;i=i+1) begin next_key; navigate(i); end
        wait(dut.load_busy);
        wait(dut.preload_halted);
        if(dut.last_error!=8'he0 || image_id!=3 || dut.image_complete[write_bank])
            $fatal(1,"Timed-out demand published an incomplete cache");
        frame(3);
        $display("PASS: next-image prefetch, stable background display/audio, PCM permission on hits, cached navigation during I/O, early keys, 16-image wrap, auto, sparse/single files, failure/timeout");
        $finish;
    end
    initial begin #10000000; $fatal(1,"Multi-BMP cache test timeout"); end
endmodule
