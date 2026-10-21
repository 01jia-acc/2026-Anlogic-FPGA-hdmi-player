`timescale 1ns/1ps
// Real FAT scanner, fragmented file streamer, sector adapter, asynchronous
// buffer and HDMI bridge. Only the physical SPI card response is substituted.
module tb_pcm_file #(
    parameter integer BMP_WIDTH=640, BMP_HEIGHT=480
);
    localparam integer BMP_BYTES=54+((BMP_WIDTH*3+3)/4)*4*BMP_HEIGHT;
    localparam integer BMP_LAST_CLUSTER=32+(BMP_BYTES-1)/512;
    reg clk=0, pixel_clk=0, rst=1;
    always #5 clk=~clk;
    always #7 pixel_clk=~pixel_clk;
    reg scan=0, allow_play=0, mode=0, image_start=0;
    reg audio_commit=0;
    wire audio_ack;
    wire image_done, image_write, image_commit;
    wire [31:0] image_data;
    wire [7:0] image_error;
    integer image_pixels=0, image_commits=0, interleaved_audio=0;
    wire scan_done, active, valid, ended, ready, underflow;
    wire signed [15:0] sample;
    wire [7:0] error;
    wire audio_valid;
    wire [23:0] audio_left, audio_right;
    reg [7:0] disk[0:1048575];
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
        .pcm_image_id_pix(4'd0), .pcm_commit_toggle_pix(audio_commit), .pcm_commit_ack(audio_ack),
        .pcm_play_enable(allow_play), .pcm_sample(sample), .pcm_valid(valid),
        .pcm_active(active), .pcm_end(ended), .pcm_underflow(underflow), .pcm_error(error),
        .scan_start(scan), .scan_done(scan_done), .load_start(image_start), .load_abort(1'b0),
        .load_cluster(32'd32), .load_size(BMP_BYTES), .write_req_ack(1'b1), .SD_MISO(1'b1),
        .load_done(image_done), .load_error(image_error),
        .write_en(image_write), .write_data(image_data), .frame_commit(image_commit)
    );
    media_hdmi_audio_bridge #(.CLOCK_HZ(4_800_000), .PCM_ENABLE(1), .VOLUME(128)) bridge (
        .pixel_clk(pixel_clk), .rst(rst), .display_valid_sd(1'b1),
        .media_id_sd(2'd0), .commit_toggle_sd(audio_commit), .pcm_mode_sd(mode),
        .pcm_commit_ack_sd(audio_ack),
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
            for(i=0;i<(kind==8 ? 1048576 : 16384);i=i+1) disk[i]=0;
            disk[0]=8'heb; disk[12]=2; disk[13]=1; disk[14]=1; disk[16]=1;
            put32(32,kind==8 ? 2048 : 32); put32(36,kind==8 ? 16 : 1); put32(44,2);
            disk[510]=8'h55; disk[511]=8'haa;
            // Four images followed by AUDIO1.PCM, all in the first root cluster.
            for(i=0;i<4;i=i+1) begin
                n=(kind==8 ? 8704 : 1024)+i*32;
                disk[n]="P"; disk[n+1]="I"; disk[n+2]="C"; disk[n+3]="1"+i;
                for(j=4;j<8;j=j+1) disk[n+j]=" ";
                disk[n+8]="B"; disk[n+9]="M"; disk[n+10]="P";
                disk[n+11]=8'h20; disk[n+26]=20+i; put32(n+28,54);
            end
            n=(kind==8 ? 8704 : 1024)+128;
            disk[n]="A"; disk[n+1]="U"; disk[n+2]="D"; disk[n+3]="I"; disk[n+4]="O";
            disk[n+5]="1"; disk[n+6]=" "; disk[n+7]=" ";
            disk[n+8]="P"; disk[n+9]="C"; disk[n+10]="M";
            disk[n+11]=8'h20; disk[n+26]=3; put32(n+28,samples*2);
            if(kind==1) disk[n]=8'he5; // deleted
            if(kind==2) put32(n+28,samples*2+1); // odd byte count
            if(kind==3) disk[n+11]=8'h08; // volume label
            if(kind==4) disk[1024]=0; // directory end before plausible entries
            for(i=0;i<13;i=i+1) begin
                put32(512+chain[i]*4, (i+1)*256>=samples ? 32'h0fffffff : chain[i+1]);
                for(j=0;j<256;j=j+1) begin
                    disk[(chain[i]+(kind==8 ? 15 : 0))*512+j*2]=pattern(i*256+j);
                    disk[(chain[i]+(kind==8 ? 15 : 0))*512+j*2+1]=pattern(i*256+j)>>8;
                end
            end
            if(kind==5) put32(512+3*4,32'h0fffffff); // early chain end
            if(kind==8) begin
                // A full 640x480 BMP, distinct from the fragmented PCM chain.
                put32(512+2*4,32'h0fffffff);
                n=8704; disk[n+26]=32; put32(n+28,BMP_BYTES);
                for(i=32;i<=BMP_LAST_CLUSTER;i=i+1)
                    put32(512+i*4,i==BMP_LAST_CLUSTER ? 32'h0fffffff : i+1);
                n=47*512; disk[n]="B"; disk[n+1]="M";
                put32(n+2,BMP_BYTES); put32(n+10,54); put32(n+14,40);
                put32(n+18,BMP_WIDTH); put32(n+22,BMP_HEIGHT);
                disk[n+26]=1; disk[n+28]=24;
            end
        end
    endtask
    // Response producer cannot bypass the real sector adapter.
    always @(negedge clk) begin
        card_valid=0; card_end=0;
        if(rst) card_index=-1;
        else if(dut.sd_sec_read) begin
            if(card_index!=-1) $fatal(1,"Overlapping SD requests");
            card_lba=dut.sd_sec_read_addr; card_index=0;
            if(dut.source_state==2 && dut.sector_owner==2)
                interleaved_audio=interleaved_audio+1;
            if(card_lba>(run_kind==8 ? 2047 : 31)) $fatal(1,"LBA out of disk");
        end else if(card_index>=0 && !(stall && card_lba==3)) begin
            if(card_index<512) begin
                card_byte=disk[card_lba*512+card_index]; card_valid=1;
                card_index=card_index+1;
            end else begin card_end=1; card_index=-1; end
        end
        if(!rst && dut.source_state==3 && !dut.pcm_sector_ready) blocked=blocked+1;
        if(!rst && image_write) begin
            image_pixels=image_pixels+1;
            if(image_data!==32'd0) $fatal(1,"PCM bytes corrupted BMP pixels");
        end
        if(!rst && image_commit) image_commits=image_commits+1;
        if(!rst && dut.audio_file_done) begin
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
    task withdraw_unissued_audio_grant;
        begin
            // Permission can be withdrawn after grant but before the registered
            // request. It must remain pending, never disappear into a WAIT state.
            wait(dut.sector_owner==2 && !dut.sector_busy && !dut.audio_sector_req);
            @(negedge clk); allow_play=0;
            wait(dut.source_ready);
            if(!dut.audio_sector_pending) $fatal(1,"Unissued PCM request was lost on preemption");
            repeat(30) @(negedge clk);
            allow_play=1;
        end
    endtask
    task load_image_during_pcm;
        begin
            // Withdraw permission in the middle of a PCM SD sector. The source
            // must finish that sector, then share BMP/PCM at sector boundaries.
            wait(dut.source_state==3 && dut.sector_busy && card_index>100);
            @(negedge clk); allow_play=0;
            wait(dut.source_ready);
            @(negedge clk); image_start=1;
            @(negedge clk); image_start=0;
            wait(image_done);
            if(image_error!=0 || image_pixels!=307200 || image_commits!=1)
                $fatal(1,"BMP preemption failed error=%h pixels=%0d commits=%0d",
                    image_error,image_pixels,image_commits);
            if(!active || ended || error!=0) $fatal(1,"PCM state lost while loading BMP");
            if(underflow || interleaved_audio<2)
                $fatal(1,"PCM was not continuously replenished during full BMP read");
            @(negedge clk); allow_play=1;
        end
    endtask
    task run_case;
        input integer samples,kind,want_samples,want_error;
        begin
            @(negedge clk); rst=1; allow_play=0; scan=0; mode=0;
            audio_commit=0;
            received=0; expected=want_samples; blocked=0; stall=(kind==6);
            image_start=0; image_pixels=0; image_commits=0; interleaved_audio=0;
            loop_samples=samples; run_kind=kind; file_passes=0;
            prepare(samples,kind);
            repeat(8) @(negedge clk);
            rst=0; scan=1;
            @(negedge clk); scan=0;
            wait(scan_done);
            if(kind!=4 && dut.resource_valid!==4'b1111) $fatal(1,"Picture lookup regression");
            repeat(40) @(negedge clk);
            if(active) $fatal(1,"PCM started before preload permission");
            audio_commit=1;
            allow_play=1;
            if(want_samples>0) begin
                wait(active); mode=1;
                if(kind==8) begin
                    wait(received==300);
                    load_image_during_pcm;
                end
                if(samples>1000 && kind==0) begin
                    wait(received==300);
                    withdraw_unissued_audio_grant;
                    pause_playback;
                end
                if(kind==0 || kind==8) begin
                    wait(received>=samples);
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
            if((kind!=0 && kind!=8) || samples==0) begin
                if(active || bridge.use_pcm) $fatal(1,"No fallback after error/invalid file");
                if(kind==7 && file_passes!=2) $fatal(1,"Read error retried forever");
            end
            $display("PASS PCM case %0d samples=%0d error=%h",scenario,received,error);
            scenario=scenario+1;
        end
    endtask
    task bad_bmp_chain;
        input integer kind;
        input [31:0] next_cluster;
        input [7:0] expected_error;
        begin
            @(negedge clk); rst=1; allow_play=0; scan=0; mode=0;
            audio_commit=0;
            received=0; image_start=0; image_pixels=0; image_commits=0;
            run_kind=8; stall=0; prepare(3102,8);
            put32(512+32*4,next_cluster);
            repeat(8) @(negedge clk); rst=0; scan=1;
            @(negedge clk); scan=0;
            wait(scan_done); wait(dut.source_ready);
            @(negedge clk); image_start=1;
            @(negedge clk); image_start=0;
            wait(image_done);
            if(image_error!==expected_error || image_commits!=0 || image_pixels>=307200)
                $fatal(1,"Original BMP FAT error lost: kind=%0d error=%h",kind,image_error);
            $display("PASS BMP raw FAT error kind=%0d code=%h",kind,image_error);
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
        run_case(3102,8,BMP_WIDTH==640 && BMP_HEIGHT==480 ? 27918 : 55836,0);
        bad_bmp_chain(0,32'h0fffffff,8'h25);
        bad_bmp_chain(1,32'd32,8'h23);
        bad_bmp_chain(2,32'd5000,8'h24);
        $display("PASS: FAT PCM loops, fragmented chain, FIFO wrap, pause/resume across EOF, later-loop errors, invalid entries, timeout, full BMP/PCM sector sharing without underflow and HDMI samples");
        $finish;
    end
    initial begin #200000000; $fatal(1,"PCM test timeout"); end
endmodule
