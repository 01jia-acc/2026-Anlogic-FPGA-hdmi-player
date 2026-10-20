`timescale 1ns/1ps
// Replays only captured boot/root/FAT sectors, without accessing the card.
module tb_card_scan;
    reg clk=0,rst_n=0,start=0;
    always #5 clk=~clk;
    wire req,done;
    wire [31:0] lba;
    wire [7:0] error;
    wire [15:0] pictures,audios;
    wire [511:0] clusters,sizes;
    wire [511:0] audio_clusters,audio_sizes;
    wire [31:0] root_crc;
    wire [15:0] root_bytes;
    reg busy=0,valid=0,sector_done=0;
    reg [8:0] index=0;
    reg [7:0] data=0;
    reg [7:0] bytes[0:65535];
    reg [31:0] lbas[0:127];
    integer sector_count,entry_no=-1,byte_no=-1,i,slot;
    fat32_volume_scanner dut (
        .clk(clk),.rst_n(rst_n),.scan_start(start),.scan_done(done),.fs_error(error),
        .resource_valid(pictures),.audio_resource_valid(audios),
        .resource_clusters(clusters),.resource_sizes(sizes),
        .audio_cluster(audio_clusters),.audio_size(audio_sizes),
        .debug_root_crc(root_crc),.debug_root_bytes(root_bytes),
        .sector_req(req),.sector_lba(lba),.sector_busy(busy),.sector_valid(valid),
        .sector_byte(data),.sector_byte_index(index),.sector_done(sector_done),.sector_error(8'd0)
    );
    always @(negedge clk) begin
        valid=0; sector_done=0;
        if(req) begin
            if(busy) $fatal(1,"Overlapping request");
            entry_no=-1;
            for(slot=0;slot<sector_count;slot=slot+1)
                if(lbas[slot]==lba) entry_no=slot;
            if(entry_no<0) $fatal(1,"Requested uncaptured LBA %0d",lba);
            $display("Reading captured LBA %0d",lba);
            byte_no=0; busy=1;
        end else if(byte_no>=0) begin
            if(byte_no<512) begin
                valid=1; index=byte_no; data=bytes[entry_no*512+byte_no]; byte_no=byte_no+1;
            end else begin sector_done=1; busy=0; byte_no=-1; end
        end
    end
    initial begin
        if(!$value$plusargs("SECTORS=%d",sector_count)) $fatal(1,"Missing +SECTORS");
        $readmemh("tests/build/card_scan/lbas.hex",lbas,0,sector_count-1);
        $readmemh("tests/build/card_scan/bytes.hex",bytes,0,sector_count*512-1);
        repeat(4) @(negedge clk); rst_n=1; start=1;
        @(negedge clk); start=0;
        wait(done); #1;
        $display("Actual card directory: PIC mask=%h AUDIO mask=%h error=%h",pictures,audios,error);
        $display("Root received CRC32=%h bytes=%h",root_crc,root_bytes);
        // Independently calculated with Python zlib.crc32 over captured root sectors.
        if(root_crc!==32'hde89ae04 || root_bytes!==16'h0400)
            $fatal(1,"Root byte count/CRC differs from captured card data");
        for(i=0;i<16;i=i+1) if(pictures[i]) begin
            $display("PIC%0d.BMP cluster=%0d size=%0d",i+1,clusters[i*32 +:32],sizes[i*32 +:32]);
            if(clusters[i*32 +:32] !== (i<4 ? 3+i*113 : 810+(i-4)*113) ||
               sizes[i*32 +:32] !== 32'd921654)
                $fatal(1,"PIC%0d exact cluster/size mismatch",i+1);
        end
        if(audio_clusters[31:0]!==32'd458 || audio_clusters[63:32]!==32'd1488 ||
           audio_sizes[31:0]!==32'd2880000 || audio_sizes[63:32]!==32'd2880000)
            $fatal(1,"Actual card PCM metadata mismatch");
        if(error!=0 || pictures!==16'h03ff || audios!==16'h0003)
            $fatal(1,"Card scan did not recognize 10 pictures and 2 PCM files");
        $display("PASS: actual card directory recognizes PIC1..PIC10 and AUDIO1..AUDIO2");
        $finish;
    end
    initial begin #10000000; $fatal(1,"Card scan timeout"); end
endmodule
