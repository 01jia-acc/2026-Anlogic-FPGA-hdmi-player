`timescale 1ns/1ps
// Real image source wrapper, decoder, byte FIFO, scaler and frame loader.
// Substitute only the FAT byte producer: dense unpausable 512-byte sectors.
module tb_bilinear_image #(
    parameter integer WIDTH=8, HEIGHT=6, FULL_SIZE=0
);
    reg clk=0, rst=1, load_start=0, load_abort=0;
    always #5 clk=~clk;
    reg file_valid=0, file_done=0;
    reg [7:0] file_byte=0, fs_error=0;
    reg [31:0] file_size=0;
    wire write_req, write_en, commit, source_done;
    wire [31:0] write_data;
    wire [7:0] source_error;
    wire loader_busy, loader_done, mem_en;
    wire [20:0] mem_addr;
    wire [31:0] mem_data;
    wire [7:0] mem_error;
    wire [3:0] complete;
    integer sw,sh,kind=0, stride, bytes, outputs=0, writes=0, commits=0, stalls=0;
    integer x,y,i,j,offset,sector_end;
    reg reuse_session=0;
    reg [7:0] header[0:53];
    reg [23:0] expected_rgb;
    sd_fat32_image_source #(.DISPLAY_WIDTH(WIDTH),.DISPLAY_HEIGHT(HEIGHT)) dut (
        .clk(clk),.rst(rst),.pixel_clk(clk),.pcm_ready(1'b0),.pcm_play_enable(1'b0),
        .scan_start(1'b0),.load_start(load_start),.load_abort(load_abort),
        .load_cluster(32'd2),.load_size(file_size),.write_req_ack(loader_busy),
        .write_req(write_req),.write_en(write_en),.write_data(write_data),
        .frame_commit(commit),.load_done(source_done),.load_error(source_error),.SD_MISO(1'b1)
    );
    media_frame_loader #(.IMAGE_WORDS(WIDTH*HEIGHT),.FRAME_WIDTH(WIDTH),
        .FRAME_HEIGHT(HEIGHT),.FIFO_DEPTH(32),.BURST_SIZE(8)) loader (
        .clk(clk),.rst_n(~rst),.load_start(write_req && !loader_busy),.image_index(2'd2),
        .pixel_valid(write_en),.pixel_rgb(write_data[31:8]),.frame_commit(commit),
        .decode_error(source_error),.command_ready(1'b1),.load_busy(loader_busy),
        .load_done(loader_done),.memory_error(mem_error),.image_complete(complete),
        .controller_init_done(1'b1),.controller_busy(1'b0),.refresh_active(1'b0),
        .app_wr_en(mem_en),.app_wr_addr(mem_addr),.app_wr_data(mem_data),.app_wr_mask()
    );
    function [23:0] pattern;
        input integer px,py;
        begin
            if(kind==1) pattern=24'hffffff;
            else if(kind==2) pattern=24'h000000;
            else begin
                pattern[23:16]=(px*73+py*19+11)&255;
                pattern[15:8]=(px*17+py*91+57)&255;
                pattern[7:0]=(px*103+py*7+233)&255;
            end
        end
    endfunction
    function [23:0] reference;
        input integer dx,dy;
        reg [63:0] qx,qy;
        integer ax,ay,bx,by,wx,wy,c,a,b,d,e,top,bottom,result;
        reg [23:0] p0,p1,p2,p3;
        begin
            qx=((sw-1)*64'd65536/(WIDTH-1))*dx;
            qy=((sh-1)*64'd65536/(HEIGHT-1))*dy;
            if(dx==WIDTH-1) qx=(sw-1)*64'd65536;
            if(dy==HEIGHT-1) qy=(sh-1)*64'd65536;
            ax=qx>>16; ay=qy>>16; bx=ax<sw-1 ? ax+1:ax; by=ay<sh-1 ? ay+1:ay;
            wx=(qx>>8)&255; wy=(qy>>8)&255;
            p0=pattern(ax,ay);p1=pattern(bx,ay);p2=pattern(ax,by);p3=pattern(bx,by);
            for(c=0;c<3;c=c+1) begin
                a=(p0>>(8*c))&255;b=(p1>>(8*c))&255;
                d=(p2>>(8*c))&255;e=(p3>>(8*c))&255;
                top=a*(256-wx)+b*wx; bottom=d*(256-wx)+e*wx;
                result=(top*(256-wy)+bottom*wy+32768)>>16;
                reference[c*8 +:8]=result;
            end
        end
    endfunction
    always @(negedge clk) if(!rst) begin
        if(dut.byte_count>1024) $fatal(1,"Sector FIFO overflow");
        if(write_en) begin
            expected_rgb=reference(outputs%WIDTH,outputs/WIDTH);
            if(outputs>=WIDTH*HEIGHT || write_data!=={expected_rgb,8'd0})
                $fatal(1,"RGB mismatch %0dx%0d out=%0d actual=%h expected=%h",sw,sh,outputs,write_data,expected_rgb);
            outputs=outputs+1;
        end
        if(mem_en) begin
            expected_rgb=reference(writes%WIDTH,writes/WIDTH);
            if(mem_addr!==2*WIDTH*HEIGHT+(HEIGHT-1-writes/WIDTH)*WIDTH+writes%WIDTH ||
               mem_data!=={expected_rgb,8'd0}) $fatal(1,"Scaled frame loader address/orientation/data");
            writes=writes+1;
        end
        if(commit) begin
            if(outputs!=WIDTH*HEIGHT || !dut.decoder_finished || !dut.scaler_finished)
                $fatal(1,"Premature scaled frame commit");
            commits=commits+1;
        end
    end
    task put32;
        input integer at,value;
        begin header[at]=value;header[at+1]=value>>8;header[at+2]=value>>16;header[at+3]=value>>24;end
    endtask
    task boot;
        begin
            @(posedge clk); #1; rst=1;load_start=0;load_abort=0;file_valid=0;file_done=0;fs_error=0;
            repeat(6) @(posedge clk);
            #1;rst=0;outputs=0;writes=0;commits=0;
        end
    endtask
    task make_header;
        input integer width,height,mode;
        begin
            sw=width;sh=height;kind=mode;
            stride=((sw*3+3)/4)*4;bytes=54+stride*sh;file_size=bytes;
            for(i=0;i<54;i=i+1) header[i]=0;
            header[0]="B";header[1]="M";put32(2,bytes);put32(10,54);put32(14,40);
            put32(18,sw);put32(22,sh);header[26]=1;header[28]=24;
        end
    endtask
    task begin_file;
        begin
            wait(dut.source_ready);
            @(posedge clk);#1;load_start=1;
            @(posedge clk);#1;load_start=0;
            wait(dut.decoder_start);
            repeat(2) @(posedge clk);
        end
    endtask
    task send_bytes;
        input integer count;
        reg [23:0] p;
        integer n,local_offset,channel;
        begin
            offset=0;
            while(offset<count) begin
                while(!dut.image_sector_space) begin
                    @(posedge clk);#1;file_valid=0;stalls=stalls+1;
                end
                sector_end=offset+512;if(sector_end>count) sector_end=count;
                while(offset<sector_end) begin
                    @(posedge clk);#1;
                    file_valid=1;
                    if(offset<54) file_byte=header[offset];
                    else begin
                        local_offset=(offset-54)%stride; y=(offset-54)/stride;
                        x=local_offset/3;channel=local_offset%3;
                        p=pattern(x,y);
                        file_byte=local_offset<sw*3 ? (p>>(channel*8)):8'ha5;
                    end
                    offset=offset+1;
                end
                @(posedge clk);#1;file_valid=0;
                repeat(5) @(posedge clk);
            end
            @(posedge clk);#1;file_done=1;
            @(posedge clk);#1;file_done=0;
        end
    endtask
    task good_image;
        input integer width,height,mode;
        begin
            if(!reuse_session) boot;
            else begin
                @(posedge clk); #1; outputs=0;writes=0;commits=0;
            end
            make_header(width,height,mode);begin_file;send_bytes(bytes);
            wait(source_done); #1;
            if(source_error) $fatal(1,"Unexpected source error %h dimensions=%0dx%0d pixels=%0d offset=%0d fifo=%0d row=%0d",source_error,sw,sh,dut.u_bmp24_decoder.pixels_received,dut.next_decode_offset,dut.byte_count,dut.u_bmp24_decoder.physical_row);
            wait(loader_done);repeat(4) @(posedge clk);
            if(outputs!=WIDTH*HEIGHT || writes!=WIDTH*HEIGHT || commits!=1 || mem_error || !complete[2])
                $fatal(1,"Wrong scaled count/commit/loader completion");
            $display("Checked bilinear %0dx%0d -> %0dx%0d mode=%0d",sw,sh,WIDTH,HEIGHT,kind);
        end
    endtask
    task bad_image;
        input integer width,height,cut,error_code;
        begin
            boot;make_header(width,height,0);begin_file;send_bytes(cut ? bytes-7:bytes);
            wait(source_done);repeat(3) @(posedge clk);
            if(source_error!=error_code || commits || complete[2])
                $fatal(1,"Invalid/truncated image published: error=%h want=%h",source_error,error_code);
        end
    endtask
    initial begin
        force dut.sd_init_done=1'b1;
        force dut.streamer_file_busy=1'b0;
        force dut.streamer_sector_pending=1'b0;
        force dut.streamer_file_valid=file_valid;
        force dut.streamer_file_byte=file_byte;
        force dut.streamer_file_done=file_done;
        force dut.streamer_fs_error=fs_error;
        good_image(1,1,1);good_image(1,5,0);good_image(5,1,0);
        good_image(2,2,0);good_image(3,3,0);good_image(7,5,0);
        good_image(WIDTH,HEIGHT,0);good_image(13,11,0);
        good_image(9,7,1);good_image(9,7,2);
        // Keep every RTL state alive across several file transactions.
        reuse_session=1;
        good_image(WIDTH,HEIGHT,0);good_image(WIDTH,HEIGHT,1);
        good_image(13,11,0);good_image(2,2,0);good_image(WIDTH,HEIGHT,2);
        reuse_session=0;
        if(FULL_SIZE) begin
            good_image(320,240,0);good_image(800,600,0);good_image(1920,1080,0);
        end
        bad_image(7,5,1,8'h37);bad_image(1921,1,0,8'h32);
        // Abort while interpolation is active; restart with a different image.
        boot;make_header(2,2,0);begin_file;send_bytes(bytes);
        wait(dut.u_scaler.state==9);
        @(posedge clk);#1;load_abort=1;
        @(posedge clk);#1;load_abort=0;
        repeat(10) @(posedge clk);
        if(write_en || commit || source_done) $fatal(1,"Output after abort");
        good_image(5,3,0);
        if(stalls==0 && WIDTH*HEIGHT>100) $fatal(1,"Did not exercise buffered sector stalls");
        $display("PASS: bilinear RGB, up/down/identity, edges, singleton axes, white/black, row padding, sector stalls, loader vertical flip, truncated/invalid files, abort/reset");
        $finish;
    end
    initial begin #2000000000;$fatal(1,"Bilinear image timeout");end
endmodule
