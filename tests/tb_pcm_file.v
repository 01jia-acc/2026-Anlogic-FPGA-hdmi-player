`timescale 1ns/1ps
// Real FAT scanner, fragmented file streamer, sector adapter, asynchronous
// buffer and HDMI bridge. Only the physical SPI card response is substituted.
module tb_pcm_file;
    reg clk=0, pixel_clk=0, rst=1;
    always #5 clk=~clk;
    always #7 pixel_clk=~pixel_clk;
    reg scan=0, allow_play=0, mode=0;
    wire scan_done, active, valid, ended, ready, underflow;
    wire signed [15:0] sample;
    wire [7:0] error;
    wire audio_valid;
    wire [23:0] audio_left, audio_right;
    reg [7:0] disk[0:16383];
    reg [7:0] card_byte=0;
    reg card_valid=0, card_end=0;
    integer card_index=-1, card_lba=0, received=0, expected=0, scenario=0;
    integer chain[0:12];
    integer i,j,n,blocked=0, consumed_before;
    integer loop_samples=0, run_kind=0, file_passes=0;
    reg stall=0, was_pcm=0, was_valid=0;
    reg [15:0] expected_sample;
    sd_fat32_image_source #(.PCM_ENABLE(1), .PCM_FIFO_ADDR_WIDTH(9),
        .PCM_SECTOR_TIMEOUT(2048)) dut (
        .clk(clk), .rst(rst), .pixel_clk(pixel_clk), .pcm_ready(ready),
        .pcm_play_enable(allow_play), .pcm_sample(sample), .pcm_valid(valid),
        .pcm_active(active), .pcm_end(ended), .pcm_underflow(underflow), .pcm_error(error),
        .scan_start(scan), .scan_done(scan_done), .load_start(1'b0), .load_abort(1'b0),
        .load_cluster(32'd0), .load_size(32'd0), .write_req_ack(1'b1), .SD_MISO(1'b1)
    );
    media_hdmi_audio_bridge #(.CLOCK_HZ(4_800_000), .PCM_ENABLE(1), .VOLUME(128)) bridge (
        .pixel_clk(pixel_clk), .rst(rst), .display_valid_sd(1'b1),
        .media_id_sd(2'd0), .commit_toggle_sd(1'b0), .pcm_mode_sd(mode),
        .pcm_active(active), .pcm_valid(valid), .stream_sample(sample), .pcm_ready(ready),
        .audio_valid(audio_valid), .audio_left_data(audio_left), .audio_right_data(audio_right)
    );
    function [15:0] pattern;
        input integer index;
        begin
            case (index % 4)
                0: pattern=16'h8000;
                1: pattern=16'h7fff;
                default: pattern=(index * 7919) & 65535;
            endcase
        end
    endfunction
    task put32;
        input integer offset,value;
        begin
            disk[offset]=value; disk[offset+1]=value>>8;
            disk[offset+2]=value>>16; disk[offset+3]=value>>24;
        end
    endtask
    task prepare;
        input integer samples, kind;
        begin
            for(i=0;i<16384;i=i+1) disk[i]=0;
            disk[0]=8'heb; disk[12]=2; disk[13]=1; disk[14]=1; disk[16]=1;
            put32(32,32); put32(36,1); put32(44,2);
            disk[510]=8'h55; disk[511]=8'haa;
            // Four images followed by AUDIO.PCM, all in the first root cluster.
            for(i=0;i<4;i=i+1) begin
                n=1024+i*32;
                disk[n]="P"; disk[n+1]="I"; disk[n+2]="C"; disk[n+3]="1"+i;
                for(j=4;j<8;j=j+1) disk[n+j]=" ";
                disk[n+8]="B"; disk[n+9]="M"; disk[n+10]="P";
                disk[n+11]=8'h20; disk[n+26]=20+i; put32(n+28,54);
            end
            n=1152;
            disk[n]="A"; disk[n+1]="U"; disk[n+2]="D"; disk[n+3]="I"; disk[n+4]="O";
            disk[n+5]=" "; disk[n+6]=" "; disk[n+7]=" ";
            disk[n+8]="P"; disk[n+9]="C"; disk[n+10]="M";
            disk[n+11]=8'h20; disk[n+26]=3; put32(n+28,samples*2);
            if(kind==1) disk[n]=8'he5; // deleted
            if(kind==2) put32(n+28,samples*2+1); // odd byte count
            if(kind==3) disk[n+11]=8'h08; // volume label
            if(kind==4) disk[1024]=0; // directory end before plausible entries
            for(i=0;i<13;i=i+1) begin
                put32(512+chain[i]*4, (i+1)*256>=samples ? 32'h0fffffff : chain[i+1]);
                for(j=0;j<256;j=j+1) begin
                    disk[chain[i]*512+j*2]=pattern(i*256+j);
                    disk[chain[i]*512+j*2+1]=pattern(i*256+j)>>8;
                end
            end
            if(kind==5) put32(512+3*4,32'h0fffffff); // early chain end
        end
    endtask
    // Response producer cannot bypass the real sector adapter.
    always @(negedge clk) begin
        card_valid=0; card_end=0;
        if(rst) card_index=-1;
        else if(dut.sd_sec_read) begin
            if(card_index!=-1) $fatal(1,"Overlapping SD requests");
            card_lba=dut.sd_sec_read_addr; card_index=0;
            if(card_lba>31) $fatal(1,"LBA out of disk");
        end else if(card_index>=0 && !(stall && card_lba==3)) begin
            if(card_index<512) begin
                card_byte=disk[card_lba*512+card_index]; card_valid=1;
                card_index=card_index+1;
            end else begin card_end=1; card_index=-1; end
        end
        if(!rst && dut.source_state==3 && !dut.pcm_sector_ready) blocked=blocked+1;
        if(!rst && dut.source_state==3 && dut.streamer_file_done) begin
            file_passes=file_passes+1;
            // The first pass succeeds; the next pass encounters a broken chain.
            if(run_kind==7 && file_passes==1) put32(512+3*4,32'h0fffffff);
        end
    end
    always @(posedge pixel_clk) begin
        was_pcm=bridge.use_pcm && bridge.sample_tick;
        was_valid=valid;
        if(!rst && ready && valid) begin
            if(received>=expected || sample!==pattern(received % loop_samples))
                $fatal(1,"PCM mismatch scenario=%0d index=%0d actual=%h",scenario,received,sample);
            expected_sample=$signed(sample) >>> 1;
            received=received+1;
        end
        #1;
        if(!rst && was_pcm) begin
            if(!audio_valid || audio_left!==audio_right) $fatal(1,"HDMI cadence/stereo failure");
            if(was_valid && audio_left!=={expected_sample,8'd0}) $fatal(1,"PCM gain/alignment");
            if(!was_valid && audio_left!==0) $fatal(1,"Underrun must emit silence");
        end
    end
    task pause_playback;
        begin
            @(negedge pixel_clk); mode=0;
            repeat(10) @(negedge pixel_clk);
            consumed_before=received;
            repeat(3000) @(negedge pixel_clk);
            if(received!=consumed_before) $fatal(1,"PCM advanced while arpeggio selected");
            if(!active || ended) $fatal(1,"Loop stopped while paused");
            mode=1;
        end
    endtask
    task run_case;
        input integer samples,kind,want_samples,want_error;
        begin
            @(negedge clk); rst=1; allow_play=0; scan=0; mode=0;
            received=0; expected=want_samples; blocked=0; stall=(kind==6);
            loop_samples=samples; run_kind=kind; file_passes=0;
            prepare(samples,kind);
            repeat(8) @(negedge clk);
            rst=0; scan=1;
            @(negedge clk); scan=0;
            wait(scan_done);
            if(kind!=4 && dut.resource_valid!==4'b1111) $fatal(1,"Picture lookup regression");
            repeat(40) @(negedge clk);
            if(active) $fatal(1,"PCM started before preload permission");
            allow_play=1;
            if(want_samples>0) begin
                wait(active); mode=1;
                if(samples>1000 && kind==0) begin
                    wait(received==300);
                    pause_playback;
                end
                if(kind==0) begin
                    wait(received==samples);
                    if(!active || ended || !bridge.use_pcm)
                        $fatal(1,"Normal EOF fell back to arpeggio");
                    // Switch away after one complete playback, then return and
                    // verify the next two passes from their very first samples.
                    pause_playback;
                    wait(received==want_samples);
                    @(negedge pixel_clk); mode=0;
                    repeat(300) @(negedge pixel_clk);
                    if(received!=want_samples || !active || ended || error!=0)
                        $fatal(1,"PCM loop did not remain available after three passes");
                    if(file_passes<3) $fatal(1,"File was not restarted for every pass");
                end else begin
                    wait(ended);
                    if(received!=want_samples) $fatal(1,"Premature EOF %0d/%0d",received,want_samples);
                end
                if(samples>1000 && blocked==0) $fatal(1,"Backpressure not exercised");
            end else repeat(4000) @(negedge clk);
            if(error!==want_error) $fatal(1,"Error mismatch scenario=%0d got=%h",scenario,error);
            repeat(300) @(negedge pixel_clk);
            if(kind!=0 || samples==0) begin
                if(active || bridge.use_pcm) $fatal(1,"No fallback after error/invalid file");
                if(kind==7 && file_passes!=2) $fatal(1,"Read error retried forever");
            end
            $display("PASS PCM case %0d samples=%0d error=%h",scenario,received,error);
            scenario=scenario+1;
        end
    endtask
    initial begin
        force dut.sd_init_done=1'b1;
        force dut.sd_sec_read_data=card_byte;
        force dut.sd_sec_read_data_valid=card_valid;
        force dut.sd_sec_read_end=card_end;
        chain[0]=3;chain[1]=7;chain[2]=4;chain[3]=9;chain[4]=5;chain[5]=11;
        chain[6]=6;chain[7]=13;chain[8]=8;chain[9]=15;chain[10]=10;chain[11]=17;chain[12]=12;
        run_case(3102,0,9306,0);
        run_case(7,0,21,0);
        run_case(1,0,3,0);
        run_case(256,0,768,0);
        run_case(257,0,771,0);
        run_case(7,1,0,0);
        run_case(7,2,0,8'h71);
        run_case(0,0,0,8'h71);
        run_case(7,3,0,0);
        run_case(7,4,0,0);
        run_case(300,5,256,8'h25);
        run_case(300,7,556,8'h25);
        run_case(7,6,0,8'h73);
        $display("PASS: FAT PCM loops, fragmented chain, FIFO wrap, pause/resume across EOF, later-loop errors, invalid entries, timeout and HDMI samples");
        $finish;
    end
    initial begin #30000000; $fatal(1,"PCM test timeout"); end
endmodule
