`timescale 1ns/1ps
module tb_volume_audio;
    reg clk=0,rst=1,pcm_mode=1,commit=0;
    always #5 clk=~clk;
    reg [4:0] level=15;
    reg signed [15:0] sample=32767;
    wire av,ready,acr;
    wire [23:0] left,right;
    wire [19:0] cts,n;
    media_hdmi_audio_bridge #(.CLOCK_HZ(480000), .PCM_ENABLE(1), .DYNAMIC_VOLUME(1)) dut (
        .pixel_clk(clk), .rst(rst), .volume_level(level),
        .display_valid_sd(1'b1), .media_id_sd(2'd1), .commit_toggle_sd(commit),
        .pcm_active(1'b1), .pcm_valid(1'b1), .pcm_mode_sd(pcm_mode),
        .pcm_commit_ack_sd(commit), .stream_sample(sample), .pcm_ready(ready),
        .audio_valid(av), .audio_left_data(left), .audio_right_data(right),
        .acr_valid(acr), .acr_cts(cts), .acr_n(n));
    integer gain=192,target,expected,cycle=0,count=0,ready_count=0;
    reg tick;
    always @(posedge clk) begin
        if(rst) begin gain=192; cycle=0; count=0; ready_count=0; end
        else begin
            cycle=cycle+1; tick=(cycle%10==0);
            expected=($signed(dut.selected_sample)*gain) >>> 8;
            if(ready) ready_count=ready_count+1;
            if(tick) begin
                count=count+1;
                target=(level*256+10)/20;
                if(gain<target) gain=gain+1;
                else if(gain>target) gain=gain-1;
            end
            #1;
            if(av!==tick) $fatal(1,"48kHz cadence changed");
            if(acr!==(tick && count%48==0) || cts!=480 || n!=6144) $fatal(1,"ACR changed");
            if(tick && ($signed(left[23:8])!==expected || left!==right || left[7:0]!=0))
                $fatal(1,"Gain arithmetic/slew got %0d expected %0d at level %0d",$signed(left[23:8]),expected,level);
        end
    end
    task settle;
        begin repeat(2700) @(negedge clk); end
    endtask
    integer k,saved_ready;
    initial begin
        repeat(5) @(negedge clk);rst=0;settle;
        commit=1;settle;
        for(k=0;k<=20;k=k+1) begin
            level=k;sample=32767;settle;
            sample=-32768;repeat(100) @(negedge clk);
        end
        if(left!==24'h800000) $fatal(1,"100 percent must preserve signed full scale");
        level=0;settle;saved_ready=ready_count;
        repeat(1000) @(negedge clk);
        if(left!==0 || ready_count-saved_ready!=100) $fatal(1,"Muted PCM must keep consuming");
        pcm_mode=0;level=10;settle;
        if(!dut.use_media || dut.gain!=128) $fatal(1,"Arpeggio gain/source");
        repeat(5000) @(negedge clk);
        pcm_mode=1;settle;
        if(dut.gain!=128) $fatal(1,"Source switch lost volume");
        $display("PASS: PCM and arpeggio shared gain, all 21 levels, signed full scale, stereo, mute consumption, smooth gain, source retention, 48kHz/ACR");
        $finish;
    end
    initial begin #5000000; $fatal(1,"volume audio timeout"); end
endmodule
