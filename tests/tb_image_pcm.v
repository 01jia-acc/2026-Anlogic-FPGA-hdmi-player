`timescale 1ns/1ps
// Real FAT scan/stream/FIFO, actual video commit including wipe, and HDMI samples.
// Only the physical SPI card and SDRAM frame storage are replaced.
module tb_image_pcm;
    reg clk=0, pixel_clk=0, rst=1;
    always #5 clk=~clk;
    always #13 pixel_clk=~pixel_clk;
    reg scan=0, allow_play=1, mode=1, load=0;
    reg request_valid=0, frame=0;
    reg [3:0] request_image=0;
    wire video_valid, commit_toggle, ack, active, valid, ready, scan_done, image_done;
    wire [3:0] video_image;
    wire [7:0] error, image_error;
    wire signed [15:0] sample;
    wire [159:0] pcm_diagnostic;
    wire av;
    wire [23:0] left, right;
    wire wipe;
    reg [7:0] disk[0:262143];
    reg [7:0] card_byte=0;
    reg card_valid=0, card_end=0, card_hold=0;
    integer card_index=-1, card_lba=0;
    integer i,j,n,c,sz,offset,received=0,total=0,epoch=0,bmp_pixels=0,stopped;
    reg last_commit=0;
    reg was_selected,was_pcm,was_valid;
    reg [15:0] expected_hdmi;
    media_display_commit #(.IMAGE_WORDS(16),.FRAME_WIDTH(4),.WIPE_FRAMES(4)) display (
        .src_clk(clk),.rst(rst),.display_valid_src(request_valid),
        .display_index_src(request_image[1:0]),.display_image_id_src(request_image),
        .pixel_clk(pixel_clk),.frame_request(frame),.display_valid_pix(video_valid),
        .display_image_id_pix(video_image),.audio_commit_toggle(commit_toggle),.wipe_active_pix(wipe)
    );
    sd_fat32_image_source #(.PCM_ENABLE(1),.PCM_FIFO_ADDR_WIDTH(9),
        .DISPLAY_WIDTH(4),.DISPLAY_HEIGHT(4)) dut (
        .clk(clk),.rst(rst),.pixel_clk(pixel_clk),.pcm_ready(ready),
        .pcm_image_id_pix(video_image),.pcm_commit_toggle_pix(commit_toggle),.pcm_commit_ack(ack),
        .pcm_play_enable(allow_play),.pcm_sample(sample),.pcm_valid(valid),.pcm_active(active),
        .pcm_error(error),.scan_start(scan),.scan_done(scan_done),
        .load_start(load),.load_abort(1'b0),.load_cluster(32'd205),.load_size(32'd102),
        .write_req_ack(1'b1),.SD_MISO(1'b1),.load_done(image_done),.load_error(image_error),
        .debug_pcm_status(pcm_diagnostic)
    );
    media_hdmi_audio_bridge #(.CLOCK_HZ(4_800_000),.PCM_ENABLE(1),.VOLUME(128)) bridge (
        .pixel_clk(pixel_clk),.rst(rst),.display_valid_sd(video_valid),
        .media_id_sd(video_image[1:0]),.commit_toggle_sd(commit_toggle),.pcm_mode_sd(mode),
        .pcm_commit_ack_sd(ack),.pcm_active(active),.pcm_valid(valid),.stream_sample(sample),
        .pcm_ready(ready),.audio_valid(av),.audio_left_data(left),.audio_right_data(right)
    );
    function integer file_number;
        input integer logical_id;
        begin
            case(logical_id)
                0:file_number=1; 1:file_number=5; 2:file_number=9;
                3:file_number=10; 4:file_number=12; default:file_number=16;
            endcase
        end
    endfunction
    function integer samples;
        input integer number;
        begin samples=number==1 ? 1300 : number==16 ? 257 : number==9 ? 7 : number+2; end
    endfunction
    function [15:0] pattern;
        input integer number,index;
        begin pattern=(number*1009+index*7919)&65535; end
    endfunction
    function [87:0] audio_name;
        input integer number;
        reg [7:0] a,b;
        begin
            a=number<10 ? "0"+number : "1";
            b=number<10 ? " " : "0"+number-10;
            audio_name={"AUDIO",a,b," PCM"};
        end
    endfunction
    function [87:0] pic_name;
        input integer number;
        reg [7:0] a,b;
        begin
            a=number<10 ? "0"+number : "1";
            b=number<10 ? " " : "0"+number-10;
            pic_name={"PIC",a,b,"   BMP"};
        end
    endfunction
    task put32;
        input integer addr,value;
        begin
            disk[addr]=value; disk[addr+1]=value>>8;
            disk[addr+2]=value>>16; disk[addr+3]=value>>24;
        end
    endtask
    task entry;
        input integer addr;
        input [87:0] name;
        input integer cluster,bytes;
        begin
            for(j=0;j<11;j=j+1) disk[addr+j]=name[87-j*8 -:8];
            disk[addr+11]=8'h20; disk[addr+26]=cluster;
            put32(addr+28,bytes);
        end
    endtask
    initial begin
        for(i=0;i<262144;i=i+1) disk[i]=0;
        disk[0]=8'heb; disk[12]=2; disk[13]=1; disk[14]=1; disk[16]=1;
        put32(32,512); put32(36,4); put32(44,2);
        disk[510]=8'h55; disk[511]=8'haa;
        put32(512+2*4,7); put32(512+7*4,9); put32(512+9*4,32'h0fffffff);
        for(i=0;i<16;i=i+1) begin disk[5*512+i*32]=8'he5; disk[10*512+i*32]=8'he5; end
        for(i=0;i<6;i=i+1) begin
            entry(5*512+i*32,pic_name(file_number(i)),200+i,102);
            put32(512+(200+i)*4,32'h0fffffff);
            n=(203+i)*512; disk[n]="B"; disk[n+1]="M";
            put32(n+2,102); put32(n+10,54); put32(n+14,40);
            put32(n+18,4); put32(n+22,4); disk[n+26]=1; disk[n+28]=24;
        end
        entry(5*512+6*32,"AUDIO   PCM",220,8); // Must never be a fallback.
        for(i=1;i<=16;i=i+1) begin
            c=32+(i-1)*8; sz=samples(i);
            entry(10*512+(i-1)*32,audio_name(i),c,sz*2);
            if(i==5) disk[10*512+(i-1)*32]=8'he5; // Missing match.
            if(i==10) put32(10*512+(i-1)*32+28,0); // Empty file.
            if(i==12) put32(10*512+(i-1)*32+28,3); // Odd length.
            for(n=0;n<(sz+255)/256;n=n+1) begin
                // Non-contiguous data clusters, including a FAT-sector boundary.
                offset=c+(n==0 ? 0 : n+1);
                put32(512+offset*4,n==(sz-1)/256 ? 32'h0fffffff : c+n+2);
                for(j=0;j<256;j=j+1) begin
                    disk[(offset+3)*512+j*2]=pattern(i,n*256+j);
                    disk[(offset+3)*512+j*2+1]=pattern(i,n*256+j)>>8;
                end
            end
        end
        entry(12*512,"AUDIO1  PCM",220,2); // Duplicate: first wins.
        entry(12*512+32,"AUDIO17 PCM",220,2);
        entry(12*512+64,"AUDIO01 PCM",220,2);
        entry(12*512+96,"AUDIO0  PCM",220,2);
        force dut.sd_init_done=1'b1;
        force dut.sd_sec_read_data=card_byte;
        force dut.sd_sec_read_data_valid=card_valid;
        force dut.sd_sec_read_end=card_end;
        repeat(8) @(negedge clk); rst=0; scan=1;
        @(negedge clk); scan=0; wait(scan_done);
        if(dut.resource_valid!==16'h8b11 || dut.audio_resource_valid!==16'hffef)
            $fatal(1,"Numbered/sparse metadata mask mismatch pic=%h audio=%h",dut.resource_valid,dut.audio_resource_valid);
        for(i=0;i<16;i=i+1) if(i!=4) begin
            if(dut.audio_clusters[i*32 +:32]!==32+i*8)
                $fatal(1,"Wrong numbered metadata slot %0d",i);
        end
        repeat(3000) @(negedge clk);
        if(active || dut.audio_running) $fatal(1,"Audio began before first actual frame");
        request_valid=1; present(0);
        wait(received>=1300*2+100);
        @(negedge pixel_clk); // Observe after the accepted sample's NBA updates.
        if(pcm_diagnostic[159:144]!==16'hffef ||
           pcm_diagnostic[111:80]!==32'd32 || pcm_diagnostic[79:48]!==32'd2600 ||
           pcm_diagnostic[47:16]<2700 || pcm_diagnostic[15:0]!==sample)
            $fatal(1,"PCM diagnostic lost scanned mask, selected file or consumed samples: %h",pcm_diagnostic);
        // A new requested image must not switch audio until a video frame commits.
        request_image=5; repeat(3000) @(negedge pixel_clk);
        if(video_image!=0 || dut.audio_size!=2600) $fatal(1,"Requested-only image changed audio");
        request_image=0;
        // Background load of PIC16: no audio restart or sample contamination.
        @(negedge clk); allow_play=0; wait(dut.source_ready);
        @(negedge clk); load=1;
        @(negedge clk); load=0; wait(image_done);
        if(image_error!=0 || bmp_pixels!=16 || video_image!=0)
            $fatal(1,"Background BMP read failed");
        allow_play=1; repeat(3000) @(negedge pixel_clk);
        // Commit while an old PCM sector is in flight; it must drain safely.
        wait(dut.sector_owner==2 && card_index>100 && card_index<400);
        present(1); // PIC5 has no AUDIO5: silence even during wipe.
        wait(dut.audio_switch_state==0 && dut.audio_commit_seen==commit_toggle);
        repeat(3000) @(negedge pixel_clk);
        if(active || error!=0 || received!=0) $fatal(1,"Missing file is not silent");
        end_wipe;
        present(2); wait(received>=30); end_wipe; // PIC9 -> AUDIO9, not AUDIO3.
        present(3); wait(dut.audio_switch_state==0 && error==8'h71);
        repeat(3000) @(negedge pixel_clk);
        if(active || received!=0) $fatal(1,"Empty file not silent");
        end_wipe;
        present(4); wait(dut.audio_switch_state==0 && error==8'h71);
        repeat(3000) @(negedge pixel_clk);
        if(active || received!=0) $fatal(1,"Odd length not silent");
        end_wipe;
        present(5); wait(received>=257*3); end_wipe;
        // A revisit restarts from sample zero, including while arpeggio is selected.
        @(negedge pixel_clk); mode=0;
        repeat(20) @(negedge pixel_clk);
        present(0); end_wipe;
        wait(active && dut.audio_switch_state==0);
        repeat(3000) @(negedge pixel_clk);
        if(received!=0) $fatal(1,"PCM consumed in arpeggio mode");
        mode=1; wait(received>=150);
        // Stall an old sector across two actual commits: last actual image wins.
        wait(dut.sector_owner==2 && card_index>100 && card_index<400);
        @(negedge clk); card_hold=1;
        end_wipe; present(1); end_wipe; present(5);
        if(dut.audio_switch_state!=1 || dut.audio_commit_seen!=commit_toggle || active)
            $fatal(1,"Rapid commits did not wait for the stalled old sector safely");
        @(negedge clk); card_hold=0;
        wait(received>=600);
        $display("PASS: per-image PCM, all 16 numbered slots, sparse PIC mapping, no legacy fallback, actual frame/wipe commit, prefetch isolation, fragmented loops, atomic clear, missing/invalid silence, revisit from zero, mode pause, rapid commits, HDMI samples (%0d total)",total);
        $finish;
    end
    task frame_pulse;
        begin
            @(negedge pixel_clk); frame=1;
            @(negedge pixel_clk); frame=0;
            repeat(6) @(negedge pixel_clk);
        end
    endtask
    task present;
        input [3:0] target;
        begin
            request_image=target;
            repeat(100) @(negedge pixel_clk); frame_pulse;
            if(video_image!=target) $fatal(1,"Actual commit not delivered");
        end
    endtask
    task end_wipe;
        begin while(wipe) frame_pulse; end
    endtask
    always @(negedge clk) begin
        card_valid=0; card_end=0;
        if(rst) card_index=-1;
        else if(dut.sd_sec_read) begin
            if(card_index!=-1) $fatal(1,"Overlapping sectors while switching");
            card_lba=dut.sd_sec_read_addr; card_index=0;
            if(card_lba>=512) $fatal(1,"Read outside disk");
        end else if(card_index>=0 && !card_hold) begin
            if(card_index<512) begin
                card_byte=disk[card_lba*512+card_index]; card_valid=1; card_index=card_index+1;
            end else begin card_end=1; card_index=-1; end
        end
        if(!rst && dut.write_en) bmp_pixels=bmp_pixels+1;
    end
    always @(posedge pixel_clk) begin
        if(!rst) begin
            if(commit_toggle!=last_commit) begin
                received=0; epoch=epoch+1; last_commit=commit_toggle;
            end
            was_selected=bridge.pcm_selected; was_pcm=bridge.use_pcm;
            was_valid=valid;
            if(ready && valid) begin
                if(video_image==1 || video_image==3 || video_image==4)
                    $fatal(1,"Audio emitted for missing/invalid track");
                if(sample!==pattern(file_number(video_image),received%samples(file_number(video_image))))
                    $fatal(1,"Old/wrong/reordered sample epoch=%0d image=%0d index=%0d got=%h",epoch,video_image,received,sample);
                expected_hdmi=$signed(sample)>>>1;
                received=received+1; total=total+1;
            end
            #1;
            if(av && was_selected) begin
                if(left!==right) $fatal(1,"Stereo mismatch");
                if(was_pcm && was_valid) begin
                    if(left!=={expected_hdmi,8'd0}) $fatal(1,"HDMI sample mismatch");
                end else if(left!==0) $fatal(1,"Missing/switching audio fell back to a tone");
            end
        end
    end
    initial begin #100000000; $fatal(1,"Per-image PCM timeout image=%0d switch=%0d samples=%0d",video_image,dut.audio_switch_state,received); end
endmodule
