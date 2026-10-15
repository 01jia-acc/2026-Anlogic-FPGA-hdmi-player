`timescale 1ns/1ps

module tb_video_fade_in;
    reg clk = 1'b0;
    reg rst = 1'b1;
    reg frame_start = 1'b0;
    reg transition_start = 1'b0;
    reg player_active = 1'b1;
    wire [7:0] alpha;
    wire active;
    integer i;

    always #5 clk = ~clk;

    video_fade_in #(.FADE_FRAMES(4)) dut (
        .clk(clk), .rst(rst), .frame_start(frame_start),
        .transition_start(transition_start), .player_active(player_active),
        .alpha(alpha), .active(active)
    );

    task pulse_frame;
        begin
            @(negedge clk); frame_start = 1'b1;
            @(negedge clk); frame_start = 1'b0;
        end
    endtask

    initial begin
        #20; rst = 1'b0;
        if (alpha !== 8'd255 || active !== 1'b0) $fatal(1, "initial state");

        @(negedge clk); transition_start = 1'b1;
        @(negedge clk); transition_start = 1'b0;
        if (alpha !== 8'd0 || active !== 1'b1) $fatal(1, "transition did not start at black");

        for (i = 1; i <= 4; i = i + 1) begin
            pulse_frame();
            if (i < 4 && alpha !== i * 64) $fatal(1, "wrong alpha at frame %0d: %0d", i, alpha);
        end
        if (alpha !== 8'd255 || active !== 1'b0) $fatal(1, "transition did not finish");

        player_active = 1'b0;
        #1;
        if (alpha !== 8'd0) $fatal(1, "inactive player was not black");
        $display("PASS: video_fade_in frame-boundary ramp");
        $finish;
    end
endmodule
