`timescale 1ns/1ps

module tb_hdmi_audio;
    reg clk = 0;
    always #20 clk = ~clk; // Actual 25 MHz pixel clock.
    reg rst = 1;
    reg display_valid = 0;
    reg [1:0] image_id = 0;
    reg commit_toggle = 0;
    wire valid, acr_valid;
    wire [23:0] left, right;
    wire [19:0] cts, n;
    wire test_valid;
    wire [23:0] test_left;
    wire [23:0] muted_left;

    // Shorten melody timing only; retain the real clock/sample rate and CDC.
    media_hdmi_audio_bridge #(.BLIP_SAMPLES(48), .NOTE_SAMPLES(96)) dut (
        .pixel_clk(clk), .rst(rst), .display_valid_sd(display_valid),
        .media_id_sd(image_id), .commit_toggle_sd(commit_toggle),
        .audio_valid(valid), .audio_left_data(left), .audio_right_data(right),
        .acr_valid(acr_valid), .acr_cts(cts), .acr_n(n)
    );
    media_hdmi_audio_bridge #(.TEST_ONLY(1)) test_only (
        .pixel_clk(clk), .rst(rst), .display_valid_sd(display_valid),
        .media_id_sd(image_id), .commit_toggle_sd(commit_toggle),
        .audio_valid(test_valid), .audio_left_data(test_left),
        .audio_right_data(), .acr_valid(), .acr_cts(), .acr_n()
    );
    media_hdmi_audio_bridge #(.VOLUME(0)) muted (
        .pixel_clk(clk), .rst(rst), .display_valid_sd(display_valid),
        .media_id_sd(image_id), .commit_toggle_sd(commit_toggle),
        .audio_valid(), .audio_left_data(muted_left),
        .audio_right_data(), .acr_valid(), .acr_cts(), .acr_n()
    );

    integer cycles = 0;
    integer samples = 0;
    integer acrs = 0;
    integer previous_sample_cycle = 0;
    integer previous_acr_cycle = 0;
    integer windows = 0;
    integer test_positive = 0;
    integer test_negative = 0;
    integer test_sum = 0;
    reg old_valid = 0;
    reg old_acr_valid = 0;
    reg [23:0] previous_left = 0;
    reg signed [15:0] pcm;
    reg signed [15:0] test_pcm;

    // Observe registered interface signals before the following clock edge.
    always @(negedge clk) begin
        if (rst) begin
            cycles = 0;
            samples = 0;
            acrs = 0;
            previous_sample_cycle = 0;
            previous_acr_cycle = 0;
            old_valid = 0;
            old_acr_valid = 0;
            previous_left = 0;
        end else begin
            cycles = cycles + 1;
            if (valid && old_valid) $fatal(1, "PCM valid is not a one-cycle strobe");
            if (acr_valid && old_acr_valid) $fatal(1, "ACR valid is not a one-cycle strobe");
            if (test_valid !== valid) $fatal(1, "Source selection changed sample cadence");
            if (muted_left !== 24'd0) $fatal(1, "Zero volume did not mute PCM");
            if (valid) begin
                samples = samples + 1;
                if (previous_sample_cycle != 0 &&
                    cycles - previous_sample_cycle != 520 &&
                    cycles - previous_sample_cycle != 521)
                    $fatal(1, "Sample interval must be 520 or 521 pixels");
                previous_sample_cycle = cycles;
                if (left !== right || left[7:0] !== 8'd0 || (^left === 1'bx))
                    $fatal(1, "Stereo PCM or 16-to-24-bit alignment is invalid");
                pcm = $signed(left[23:8]);
                if (pcm > 9000 || pcm < -9000) $fatal(1, "PCM scaling clipped or lost its sign");
                test_pcm = $signed(test_left[23:8]);
                if (test_pcm > 0) test_positive = test_positive + 1;
                if (test_pcm < 0) test_negative = test_negative + 1;
                test_sum = test_sum + test_pcm;
                // A 3 kHz test tone returns to zero every 16 samples.
                if ((samples % 16) == 1 && test_pcm != 0)
                    $fatal(1, "Forced test tone changed on an image commit");
                if ((samples % 16) == 0) begin
                    if (test_sum < -8 || test_sum > 0)
                        $fatal(1, "Test sine has an unexpected DC bias");
                    test_sum = 0;
                end
            end else if (left !== previous_left) begin
                $fatal(1, "PCM data changed outside its valid strobe");
            end
            if (acr_valid) begin
                acrs = acrs + 1;
                if (cts !== 20'd25000 || n !== 20'd6144)
                    $fatal(1, "ACR does not describe 25 MHz / 48 kHz");
                if ((samples % 48) != 0) $fatal(1, "ACR not aligned to 48 samples");
                if (previous_acr_cycle != 0 && cycles - previous_acr_cycle != 25000)
                    $fatal(1, "ACR interval is not 1 ms");
                previous_acr_cycle = cycles;
            end
            if ((cycles % 25000) == 0) begin
                windows = windows + 1;
                if (samples != windows * 48 || acrs != windows)
                    $fatal(1, "Sample/ACR counts drifted over a 1 ms window");
            end
            previous_left = left;
            old_valid = valid;
            old_acr_valid = acr_valid;
        end
    end

    task commit_image;
        input [1:0] next_id;
        reg [1:0] old_id;
        begin
            old_id = dut.media_id;
            @(negedge clk); #7;
            // Loading a different image alone must not change the audible ID.
            image_id = next_id;
            repeat (8) @(posedge clk);
            #1;
            if (dut.media_id !== old_id) $fatal(1, "Image changed before commit");
            @(negedge clk); #7;
            display_valid = 1;
            commit_toggle = ~commit_toggle;
            repeat (8) @(posedge clk);
            #1;
            if (dut.media_id !== next_id || !dut.media_active)
                $fatal(1, "CDC lost or corrupted an image commit");
            if (!dut.u_media_tone.blip_active || dut.u_media_tone.note_index != 0)
                $fatal(1, "Commit did not restart cue/melody");
        end
    endtask

    integer i;
    integer cue_count;
    integer note_count;
    integer note;
    initial begin
        repeat (4) @(posedge clk);
        @(negedge clk); #1; rst = 0;
        // Test sine before first committed image.
        repeat (25000) @(posedge clk);
        #1;
        if (dut.media_active) $fatal(1, "Media source enabled before first image");

        for (i = 0; i < 4; i = i + 1) begin
            commit_image(i[1:0]);
            cue_count = 0;
            while (dut.u_media_tone.blip_active) begin
                @(posedge clk);
                if (dut.sample_tick) cue_count = cue_count + 1;
                #1;
            end
            if (cue_count != 48) $fatal(1, "Image cue duration is incorrect");
            for (note = 0; note < 6; note = note + 1) begin
                note_count = 0;
                while (dut.u_media_tone.note_index == note) begin
                    @(posedge clk);
                    if (dut.sample_tick) note_count = note_count + 1;
                    #1;
                end
                if (note_count != 96) $fatal(1, "Melody note duration is incorrect");
            end
            if (dut.u_media_tone.note_index != 0)
                $fatal(1, "Melody did not wrap after six notes");
        end
        // Exercise the reverse multi-bit transition and wraparound as well.
        commit_image(2'd0);
        @(negedge clk); #7; display_valid = 0;
        repeat (8) @(posedge clk);
        #1;
        if (dut.media_active) $fatal(1, "Invalid display did not restore the test source");
        if (windows < 40 || test_positive == 0 || test_negative == 0)
            $fatal(1, "Insufficient sample-rate/signed-waveform coverage");
        $display("PASS: 25 MHz / 48 kHz PCM, ACR, signed stereo, volume, test mode, image CDC and six-note loops (%0d ms)", windows);
        rst = 1;
        #1;
        if (valid || acr_valid || left != 0 || right != 0)
            $fatal(1, "Reset did not clear registered audio outputs");
        $finish;
    end

    initial begin
        #100000000;
        $fatal(1, "Audio bridge simulation timed out");
    end
endmodule
