`timescale 1ns/1ps
// Exercise real file addressing across a fragmented chain for every FAT32 SPC.
module tb_fat32_file_geometry;
    reg clk=0, rst_n=0, start=0;
    always #5 clk=~clk;
    reg [7:0] spc=1;
    reg [31:0] file_size=0;
    wire busy, done, valid, req;
    wire [7:0] data, error;
    wire [31:0] offset, lba;
    reg sector_busy=0, sector_valid=0, sector_done=0;
    reg [7:0] sector_byte=0;
    reg [8:0] sector_index=0;
    integer card_index=-1, requests=0, bytes_seen=0, test_kind=0, n;
    reg [31:0] card_lba;
    reg [27:0] fat_entry;
    integer expected_lba, expected_index;
    fat32_file_streamer dut (
        .clk(clk), .rst_n(rst_n), .file_start(start), .file_abort(1'b0),
        .first_cluster(32'd130), .file_size(file_size), .sectors_per_cluster(spc),
        .fat_start_lba(32'd10), .data_start_lba(32'd1000), .max_cluster(32'd1000),
        .file_busy(busy), .file_done(done), .file_valid(valid), .file_byte(data),
        .file_byte_offset(offset), .fs_error(error), .sector_req(req), .sector_lba(lba),
        .sector_busy(sector_busy), .sector_valid(sector_valid), .sector_byte(sector_byte),
        .sector_byte_index(sector_index), .sector_done(sector_done), .sector_error(8'd0)
    );
    function [7:0] pattern;
        input integer address, index;
        begin pattern=(address*13+index*7) & 255; end
    endfunction
    always @(negedge clk) begin
        sector_valid=0; sector_done=0;
        if(!rst_n) begin sector_busy=0; card_index=-1; end
        else if(req) begin
            if(sector_busy) $fatal(1,"Overlapping file requests");
            card_lba=lba; card_index=0; sector_busy=1; requests=requests+1;
            if(lba==11) begin
                case(test_kind)
                    1: fat_entry=28'hfffffff; // premature EOF
                    2: fat_entry=28'd130; // self loop
                    3: fat_entry=28'd1001; // out of range
                    default: fat_entry=28'd257;
                endcase
            end else fat_entry=28'hfffffff;
        end else if(card_index>=0) begin
            if(card_index<512) begin
                sector_valid=1; sector_index=card_index;
                sector_byte=pattern(card_lba,card_index);
                if(card_lba==11 && card_index>=8 && card_index<12)
                    sector_byte=fat_entry >> ((card_index-8)*8);
                if(card_lba==12 && card_index>=4 && card_index<8)
                    sector_byte=fat_entry >> ((card_index-4)*8);
                card_index=card_index+1;
            end else begin sector_busy=0; sector_done=1; card_index=-1; end
        end
    end
    always @(posedge clk) if(rst_n && valid) begin
        if(bytes_seen < spc*512)
            expected_lba=1000+128*spc+bytes_seen/512;
        else expected_lba=1000+255*spc+(bytes_seen-spc*512)/512;
        expected_index=bytes_seen%512;
        if(offset!==bytes_seen || data!==pattern(expected_lba,expected_index))
            $fatal(1,"File addressing mismatch SPC=%0d offset=%0d",spc,bytes_seen);
        bytes_seen=bytes_seen+1;
    end
    task run_case;
        input integer sectors, kind, want_error;
        begin
            @(negedge clk); rst_n=0; start=0;
            repeat(4) @(negedge clk);
            spc=sectors; test_kind=kind; file_size=sectors*512+17;
            bytes_seen=0; requests=0; rst_n=1;
            @(negedge clk); start=1;
            @(negedge clk); start=0;
            wait(done);
            @(negedge clk);
            if(error!==want_error || bytes_seen!=(kind==0 ? file_size : sectors*512))
                $fatal(1,"File geometry result SPC=%0d kind=%0d bytes=%0d error=%h",
                    sectors,kind,bytes_seen,error);
            if(requests!=(kind==0 ? sectors+3 : sectors+1))
                $fatal(1,"Unexpected data/FAT request count");
            $display("PASS file geometry SPC=%0d kind=%0d error=%h",sectors,kind,error);
        end
    endtask
    initial begin
        for(n=1;n<=128;n=n*2) run_case(n,0,0);
        run_case(8,1,8'h25); run_case(8,2,8'h23); run_case(8,3,8'h24);
        $display("PASS: pipelined LBA, all FAT32 cluster sizes, fragmented data, FAT offsets and errors");
        $finish;
    end
    initial begin #10000000; $fatal(1,"Geometry test timeout"); end
endmodule
