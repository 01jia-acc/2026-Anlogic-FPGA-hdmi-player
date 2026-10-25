`timescale 1ns/1ps
module tb_video_brightness;
    reg clk=0,rst_n=0,frame=0;
    always #5 clk=~clk;
    reg [4:0] level=20;
    reg [7:0] alpha=255;
    reg [23:0] rgb=24'hffffff;
    wire [23:0] out_rgb;
    video_brightness dut(.clk(clk),.rst_n(rst_n),.frame_start(frame),.level(level),
        .fade_alpha(alpha),.rgb_in(rgb),.rgb_out(out_rgb));
    task commit_frame;
        begin @(negedge clk);frame=1;@(negedge clk);frame=0;end
    endtask
    integer k,a,c,gain,eff,r,g,b,old_result;
    reg [23:0] expected;
    initial begin
        repeat(3) @(negedge clk);rst_n=1;
        #1;if(out_rgb!==rgb) $fatal(1,"Reset must preserve original RGB");
        for(k=0;k<=20;k=k+1) begin
            level=k;
            // Parameter change alone cannot affect a visible frame.
            old_result=out_rgb;repeat(7) @(negedge clk);
            if(out_rgb!==old_result[23:0]) $fatal(1,"Midframe gain change");
            commit_frame;
            gain=(k*256+10)/20;
            for(a=0;a<256;a=a+1) begin
                alpha=a;
                eff=(a*gain+128)/256;
                for(c=0;c<256;c=c+1) begin
                    rgb={c[7:0],(8'd255-c[7:0]),(c[7:0]^8'h55)};
                    r=(c*eff+255)/256;g=((255-c)*eff+255)/256;b=((c^85)*eff+255)/256;
                    expected={r[7:0],g[7:0],b[7:0]};#1;
                    if(out_rgb!==expected) $fatal(1,"RGB level=%0d alpha=%0d channel=%0d got=%h expected=%h",k,a,c,out_rgb,expected);
                    if(k==20 && a==255 && out_rgb!==rgb) $fatal(1,"Full brightness bit-exact failure");
                    if((k==0 || a==0) && out_rgb!==0) $fatal(1,"Zero brightness/fade must be black");
                end
            end
        end
        level=0;commit_frame;rst_n=0;#1;
        if(out_rgb!==rgb) $fatal(1,"Reset restore from black");
        $display("PASS: all 21 brightness levels x 256 fade values x 256 RGB values, exact full/zero, frame stability, reset, original fade compatibility");
        $finish;
    end
endmodule
