`timescale 1ns/1ps
module tb_multi_bmp_scanner;
    reg clk=0, rst_n=0, start=0;
    always #5 clk=~clk;
    wire req, busy, done, audio_valid;
    wire [31:0] lba, audio_cluster, audio_size, partition;
    wire [7:0] error;
    wire [15:0] valid;
    wire [511:0] clusters, sizes;
    wire [31:0] legacy_cluster, legacy_size;
    reg sector_busy=0, sector_valid=0, sector_done=0;
    reg [7:0] sector_byte=0;
    reg [8:0] sector_index=0;
    reg [7:0] disk[0:133631];
    integer card_index=-1, card_lba=0, base=0, scenario=0, i, j, offset;
    integer root_requests=0, fat_requests=0;
    fat32_volume_scanner dut (
        .clk(clk), .rst_n(rst_n), .scan_start(start), .scan_done(done),
        .fs_error(error), .partition_lba(partition), .resource_valid(valid),
        .resource_clusters(clusters), .resource_sizes(sizes),
        .resource0_cluster(legacy_cluster), .resource0_size(legacy_size),
        .audio_resource_valid(audio_valid), .audio_cluster(audio_cluster), .audio_size(audio_size),
        .sector_req(req), .sector_lba(lba), .sector_busy(sector_busy),
        .sector_valid(sector_valid), .sector_byte(sector_byte),
        .sector_byte_index(sector_index), .sector_done(sector_done), .sector_error(8'd0)
    );
    task put32;
        input integer addr, value;
        begin
            disk[addr]=value; disk[addr+1]=value>>8;
            disk[addr+2]=value>>16; disk[addr+3]=value>>24;
        end
    endtask
    task entry;
        input integer addr;
        input [87:0] name;
        input integer cluster, bytes;
        begin
            for(j=0;j<11;j=j+1) disk[addr+j]=name[87-j*8 -:8];
            disk[addr+11]=8'h20;
            disk[addr+26]=cluster; disk[addr+27]=cluster>>8;
            disk[addr+20]=cluster>>16; disk[addr+21]=cluster>>24;
            put32(addr+28,bytes);
        end
    endtask
    function [87:0] pic_name;
        input integer number;
        reg [7:0] first, second;
        begin
            first=number<10 ? "0"+number : "1";
            second=number<10 ? " " : "0"+number-10;
            pic_name={"PIC",first,second,"   BMP"};
        end
    endfunction
    task prepare;
        input integer kind;
        begin
            base=kind==1 ? 5 : 0;
            for(i=0;i<133632;i=i+1) disk[i]=0;
            offset=base*512;
            disk[offset]=8'heb; disk[offset+12]=2; disk[offset+13]=1;
            disk[offset+14]=1; disk[offset+16]=1;
            put32(offset+32,256); put32(offset+36,1); put32(offset+44,127);
            disk[offset+510]=8'h55; disk[offset+511]=8'haa;
            if(base!=0) begin
                disk[450]=8'h0c; put32(454,base);
                disk[510]=8'h55; disk[511]=8'haa;
            end
            // Root cluster 127 -> 7; FAT[127] is at bytes 508..511.
            put32((base+1)*512+127*4,7);
            put32((base+1)*512+7*4,32'h0fffffff);
            for(i=0;i<16;i=i+1) begin
                disk[(base+127)*512+i*32]=8'he5;
                disk[(base+7)*512+i*32]=8'he5;
            end
            for(i=1;i<=8;i=i+1) entry((base+127)*512+(i-1)*32,pic_name(i),40+i,921654+i);
            for(i=9;i<=16;i=i+1) entry((base+7)*512+(i-9)*32,pic_name(i),40+i,921654+i);
            // Plausible entries which must be ignored in the first cluster.
            entry((base+127)*512+8*32,pic_name(9),200,100); disk[(base+127)*512+8*32]=8'he5;
            entry((base+127)*512+9*32,pic_name(10),200,100); disk[(base+127)*512+9*32+11]=8'h0f;
            entry((base+127)*512+10*32,pic_name(11),200,100); disk[(base+127)*512+10*32+11]=8'h10;
            entry((base+127)*512+11*32,pic_name(12),200,100); disk[(base+127)*512+11*32+11]=8'h08;
            entry((base+127)*512+12*32,pic_name(17),200,100);
            entry((base+7)*512+8*32,"AUDIO   PCM",123,96000);
            entry((base+7)*512+9*32,pic_name(1),200,100); // duplicate: keep first
            if(kind!=1 && kind!=5) begin
                disk[(base+7)*512+10*32]=0;
                entry((base+7)*512+11*32,"AUDIO   PCM",201,2);
            end
            if(kind==2) put32((base+1)*512+127*4,127);
            if(kind==3) put32((base+1)*512+127*4,300);
            if(kind==4) disk[(base+127)*512]=0;
            if(kind==5) put32((base+1)*512+7*4,127);
        end
    endtask
    always @(negedge clk) begin
        sector_valid=0; sector_done=0;
        if(!rst_n) begin card_index=-1; sector_busy=0; end
        else if(req) begin
            if(card_index!=-1) $fatal(1,"Scanner overlaps sector transactions");
            card_lba=lba; card_index=0; sector_busy=1;
            if(card_lba<0 || card_lba>=261) $fatal(1,"Scanner LBA out of range");
            if(card_lba==base+127 || card_lba==base+7) root_requests=root_requests+1;
            if(card_lba==base+1) fat_requests=fat_requests+1;
        end else if(card_index>=0) begin
            if(card_index<512) begin
                sector_index=card_index; sector_byte=disk[card_lba*512+card_index];
                sector_valid=1; card_index=card_index+1;
                // Exercise simultaneous last FAT byte/done, including high nibble.
                if(card_index==512 && card_lba==base+1) begin
                    sector_done=1; sector_busy=0; card_index=-1;
                end
            end else begin sector_done=1; sector_busy=0; card_index=-1; end
        end
    end
    task run_case;
        input integer kind, want_error;
        begin
            @(posedge clk); #1; rst_n=0; start=0; prepare(kind);
            root_requests=0; fat_requests=0;
            repeat(4) @(posedge clk);
            #1; rst_n=1; start=1;
            @(posedge clk); #1; start=0;
            wait(done); #1;
            if(error!=want_error) $fatal(1,"Root scan error case=%0d actual=%h",kind,error);
            if(kind<=1) begin
                if(valid!==16'hffff || !audio_valid || audio_cluster!=123 || audio_size!=96000 || partition!=base)
                    $fatal(1,"16-image/root-chain/audio scan failed");
                for(i=0;i<16;i=i+1)
                    if(clusters[i*32 +:32]!=41+i || sizes[i*32 +:32]!=921655+i)
                        $fatal(1,"Wrong metadata for PIC%0d",i+1);
                if(legacy_cluster!=41 || legacy_size!=921655) $fatal(1,"Duplicate changed legacy metadata");
                if(root_requests!=2 || fat_requests!=(kind==1 ? 2 : 1)) $fatal(1,"Incorrect root traversal");
            end
            if(kind==4 && (valid!=0 || fat_requests!=0)) $fatal(1,"Read beyond directory end");
            if(kind==5 && dut.root_clusters_visited!=1024) $fatal(1,"Cycle bound not exercised");
            $display("PASS root case %0d error=%h root sectors=%0d",kind,error,root_requests);
        end
    endtask
    initial begin
        run_case(0,0); run_case(1,0); run_case(2,8'h23);
        run_case(3,8'h23); run_case(4,8'h22); run_case(5,8'h23);
        $display("PASS: PIC1..16, fragmented root, MBR, EOC, ignored entries, first duplicate, simultaneous FAT done, invalid/cyclic root");
        $finish;
    end
    initial begin #100000000; $fatal(1,"Root scanner timeout"); end
endmodule
