`timescale 1ns/1ps
module tb_audio_waveform;
    reg clk=0,rst_n=0,vs=1,de=0,enable=1,tap_valid=0;
    always #20 clk=~clk;
    reg [11:0] x=0,y=0;
    reg [23:0] rgb=24'h123456;
    reg signed [15:0] sample=0;
    wire [23:0] out_rgb;
    wire out_vs,out_de,hit,frame;
    wire [11:0] ox,oy;
    promax_audio_waveform #(.WAVE_Y(368)) dut(
        .clk(clk),.rst_n(rst_n),.vsync(vs),.de(de),.rgb_in(rgb),
        .pixel_x(x),.pixel_y(y),.enable(enable),.pcm_tap_valid(tap_valid),
        .pcm_tap_left(sample),.rgb_out(out_rgb),.out_vs(out_vs),.out_de(out_de),
        .out_x(ox),.out_y(oy),.waveform_active(hit),.frame_start_out(frame));
    task boundary;
        begin
            @(negedge clk);de=0;vs=0;tap_valid=0;
            repeat(4) @(negedge clk);vs=1;
            repeat(3) @(negedge clk);
        end
    endtask
    task feed;
        input integer level,n_samples;
        integer n;
        begin
            for(n=0;n<n_samples;n=n+1) begin
                @(negedge clk);tap_valid=1;
                sample=(n%2) ? -level : (level==32768 ? 32767 : level);
                @(negedge clk);tap_valid=0;
            end
        end
    endtask
    task pixel;
        input integer px,py;
        begin
            @(negedge clk);x=px;y=py;de=1;
            repeat(2) @(posedge clk);#1;
            if(ox!==px || oy!==py || !out_de || !out_vs)
                $fatal(1,"pixel pipeline misaligned");
        end
    endtask
    task level_at;
        input integer col;
        output integer blocks;
        integer b;
        begin
            blocks=0;
            for(b=0;b<5;b=b+1) begin
                pixel(68+col*16,430-b*8);
                if(hit) begin
                    if(out_rgb!==24'h20ee70 || blocks!=b) $fatal(1,"bar hole / wrong color");
                    blocks=blocks+1;
                end else if(out_rgb!==rgb) $fatal(1,"background changed");
            end
            if(blocks<1 || blocks>4) $fatal(1,"bar exceeds four blocks");
        end
    endtask
    integer xx,yy;
    reg [23:0] expected;
    task scan;
        input integer blocks;
        begin
            // Audio changes during scanning must not mutate visible history.
            tap_valid=1;sample=0;
            for(yy=366;yy<434;yy=yy+1)
                for(xx=62;xx<595;xx=xx+1) begin
                    expected=rgb;
                    if(xx>=64 && xx<576 && yy>=432-blocks*8 && yy<432 &&
                        (xx-64)%16>=2 && (xx-64)%16<14 && (431-yy)%8<4)
                        expected=(431-yy)%8==3 ? 24'hb8f438 : 24'h20ee70;
                    pixel(xx,yy);
                    if(out_rgb!==expected || hit!==(expected!=rgb))
                        $fatal(1,"segment geometry/color at %0d,%0d got %h expected %h",xx,yy,out_rgb,expected);
                end
            @(negedge clk);tap_valid=0;de=0;
        end
    endtask
    integer col,blocks,f,level,target;
    integer history[0:31];
    task check_history;
        begin
            for(col=0;col<32;col=col+1) begin
                level_at(col,blocks);
                if(blocks!=history[col])
                    $fatal(1,"time history column %0d got %0d expected %0d",col,blocks,history[col]);
            end
        end
    endtask
    task commit_history;
        input integer newest;
        integer j;
        begin
            for(j=0;j<31;j=j+1) history[j]=history[j+1];
            history[31]=newest;
            boundary;
            check_history;
        end
    endtask
    initial begin
        repeat(4) @(negedge clk);rst_n=1;
        scan(1);
        @(negedge clk);rst_n=0;repeat(3) @(negedge clk);rst_n=1;
        for(col=0;col<32;col=col+1) history[col]=1;
        // Distinct levels prove newest-right ordering, shift direction, partial
        // history masking and more than two complete circular-buffer wraps.
        for(f=0;f<70;f=f+1) begin
            case(f%5)
                0: begin level=128;target=1;end
                1: begin level=3072;target=3;end
                2: begin level=1024;target=2;end
                3: begin level=32768;target=4;end
                4: begin level=2048;target=2;end
            endcase
            feed(level,16000);
            check_history;
            commit_history(target);
        end
        feed(32768,8000);
        for(f=0;f<32;f=f+1) begin feed(32768,800);boundary;end
        scan(4);
        feed(32768,8000);boundary;
        for(f=0;f<6;f=f+1) begin feed(0,800);boundary;end
        level_at(31,blocks);
        if(blocks<2 || blocks>3) $fatal(1,"release too fast or stuck");
        // Silence shifts old activity off the left rather than clearing it.
        for(f=0;f<40;f=f+1) begin feed(0,800);boundary;end
        scan(1);
        boundary;feed(32768,2);feed(0,798);boundary;scan(1);
        feed(4096,48);boundary;level_at(31,blocks);
        if(blocks!=1) $fatal(1,"attack not softened");
        feed(4096,800);boundary;level_at(31,blocks);
        if(blocks!=3) $fatal(1,"attack failed to rise");
        // Hidden mode keeps collecting; old loud history survives stop-stream.
        enable=0;pixel(68,430);if(hit || out_rgb!==rgb) $fatal(1,"preview hiding failed");
        feed(32768,8000);boundary;
        enable=1;
        level_at(31,blocks);if(blocks!=4) $fatal(1,"hidden capture stopped");
        boundary;level_at(31,blocks);if(blocks!=1) $fatal(1,"stopped stream stale newest");
        level_at(30,blocks);if(blocks!=4) $fatal(1,"history erased on stop");
        for(f=0;f<32;f=f+1) boundary;
        scan(1);
        @(negedge clk);rst_n=0;repeat(3) @(negedge clk);rst_n=1;
        scan(1);
        // A sample coincident with VS must be appended, not lost.
        @(negedge clk);rst_n=0;repeat(3) @(negedge clk);rst_n=1;
        feed(767,8000);boundary;
        @(negedge clk);vs=0;tap_valid=1;sample=32767;
        @(negedge clk);tap_valid=0;vs=1;
        repeat(4) @(negedge clk);
        level_at(31,blocks);if(blocks!=2) $fatal(1,"boundary sample lost");
        $display("PASS: chronological left scroll, newest right, wrap/startup, frame stability, amplitude/smoothing, silence/stop/hide/reset, boundary sample, exact pixels");
        $finish;
    end
    initial begin #500000000; $fatal(1,"waveform timeout");end
endmodule
