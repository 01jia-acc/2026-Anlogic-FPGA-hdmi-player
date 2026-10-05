`timescale 1ns/1ps

module tb_pcm_media_tone;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst = 1;
    reg enable = 0;
    reg tick = 0;
    reg [1:0] media_id = 0;
    wire signed [15:0] sample;
    wire cue;
    pcm_media_tone dut (
        .clk(clk), .rst(rst), .enable(enable), .sample_tick(tick),
        .media_id(media_id), .sample(sample), .blip_active(cue)
    );

    // Check tuning against musical frequencies, not a duplicate increment LUT.
    function real note_hz;
        input integer image;
        input integer note;
        begin
            case (image)
                0: case (note)
                    0: note_hz = 523.25;
                    1, 5: note_hz = 659.25;
                    2, 4: note_hz = 783.99;
                    default: note_hz = 1046.5;
                endcase
                1: case (note)
                    0: note_hz = 440.0;
                    1, 5: note_hz = 523.25;
                    2, 4: note_hz = 659.25;
                    default: note_hz = 880.0;
                endcase
                2: case (note)
                    0: note_hz = 523.25;
                    1: note_hz = 587.33;
                    2: note_hz = 659.25;
                    3: note_hz = 783.99;
                    4: note_hz = 880.0;
                    default: note_hz = 1046.5;
                endcase
                default: case (note)
                    0, 3: note_hz = 523.25;
                    1, 4: note_hz = 392.0;
                    default: note_hz = 659.25;
                endcase
            endcase
        end
    endfunction

    integer image, note, count, crossings;
    real frequency, difference;
    reg previous_nonpositive;
    reg [19:0] phase_before;
    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk); rst = 0;
        for (image = 0; image < 4; image = image + 1) begin
            tick = 0;
            enable = 1;
            media_id = image[1:0];
            @(posedge clk); #1;
            if (!cue) $fatal(1, "Missing image cue");
            // Pausing sample enables must pause musical time.
            phase_before = dut.phase;
            repeat (7) @(posedge clk);
            #1;
            if (dut.phase !== phase_before || dut.blip_countdown != 4800)
                $fatal(1, "DDS advanced without a sampling enable");
            @(negedge clk); tick = 1;
            count = 0;
            while (cue) begin
                @(posedge clk); #1; count = count + 1;
            end
            if (count != 4800) $fatal(1, "Cue must last exactly 100 ms at 48 kHz");
            for (note = 0; note < 6; note = note + 1) begin
                count = 0;
                crossings = 0;
                previous_nonpositive = sample <= 0;
                while (dut.note_index == note) begin
                    @(posedge clk); #1;
                    count = count + 1;
                    if (previous_nonpositive && sample > 0) crossings = crossings + 1;
                    previous_nonpositive = sample <= 0;
                    if (sample > 12000 || sample < -12000 || (^sample === 1'bx))
                        $fatal(1, "DDS waveform range/sign is invalid");
                end
                if (count != 12000) $fatal(1, "Each note must last exactly 250 ms");
                frequency = crossings * 48000.0 / count;
                difference = frequency - note_hz(image, note);
                if (difference < -4.1 || difference > 4.1)
                    $fatal(1, "Wrong image/note frequency: image=%0d note=%0d observed=%f", image, note, frequency);
            end
            if (dut.note_index != 0) $fatal(1, "Six-note loop did not restart");
            @(negedge clk);
        end
        enable = 0;
        @(posedge clk); #1;
        if (cue || sample != 0) $fatal(1, "Disabled source did not reset");
        $display("PASS: all 24 musical notes, 100 ms cue, 250 ms notes, six-note wrap and sampling-enable pause");
        $finish;
    end
endmodule
