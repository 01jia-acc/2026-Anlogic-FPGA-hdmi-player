`timescale 1ns/1ps
module tb_subtitle_axis;
    reg clk=0,rst=1;
    always #20 clk=~clk;
    reg vs=1,de=0,frame_start=0,hide=1;
    reg [11:0] x=0,y=0;
    reg [23:0] rgb=0;
    wire sub_vs,sub_de,axis_user,axis_valid,axis_last;
    wire [23:0] sub_rgb,axis_rgb;
    scrolling_subtitle subtitle(.control_clk(clk),.pixel_clk(clk),.rst(rst),
        .rx_data(8'd0),.rx_valid(1'b0),.frame_start(frame_start),.hide(hide),
        .in_vs(vs),.in_de(de),.pixel_x(x),.pixel_y(y),.rgb_in(rgb),
        .out_vs(sub_vs),.out_de(sub_de),.rgb_out(sub_rgb),.busy(),.rejected_lines());
    video_rgb_to_axis_640x480 axis(.I_clk(clk),.I_rst(rst),.I_vs(sub_vs),.I_de(sub_de),
        .I_rgb(sub_rgb),.O_video_user(axis_user),.O_video_valid(axis_valid),
        .O_video_last(axis_last),.O_video_data(axis_rgb));
    reg [23:0] colors [0:2];
    reg [2:0] valid_pipe=0,last_pipe=0,user_pipe=0;
    reg arm=1,prev_vs=1;
    integer i, pixels=0,lines=0,frames=0;
    always @(posedge clk) begin
        if(rst) begin valid_pipe=0;last_pipe=0;user_pipe=0;arm=1;prev_vs=vs; end
        else begin
            if(vs!=prev_vs) arm=1;
            prev_vs=vs;
            for(i=2;i>0;i=i-1) colors[i]=colors[i-1];
            colors[0]=rgb;
            valid_pipe={valid_pipe[1:0],de};
            last_pipe={last_pipe[1:0],de && x==639};
            user_pipe={user_pipe[1:0],de && arm};
            if(de) arm=0;
            #1;
            if(axis_valid!==valid_pipe[2] || axis_last!==last_pipe[2] || axis_user!==user_pipe[2])
                $fatal(1,"AXIS framing x=%0d y=%0d valid=%b/%b last=%b/%b SOF=%b/%b",
                    x,y,axis_valid,valid_pipe[2],axis_last,last_pipe[2],axis_user,user_pipe[2]);
            if(axis_valid) begin
                if(axis_rgb!==colors[2]) $fatal(1,"RGB changed outside subtitle / while hidden");
                pixels=pixels+1;
            end
            if(axis_last) lines=lines+1;
            if(axis_user) frames=frames+1;
        end
    end
    integer f,xx,yy;
    initial begin
        repeat(4) @(negedge clk); rst=0;
        for(f=0;f<2;f=f+1) begin
            @(negedge clk); vs=0; frame_start=1;
            repeat(5) @(negedge clk); vs=1;frame_start=0;
            for(yy=0;yy<480;yy=yy+1) begin
                for(xx=0;xx<640;xx=xx+1) begin
                    @(negedge clk);de=1;x=xx;y=yy;
                    // White, gray and independent RGB bits reveal cyan corruption.
                    case(xx%4)
                        0:rgb=24'hffffff;
                        1:rgb=24'he0e0e0;
                        2:rgb={xx[7:0],yy[7:0],8'hf5};
                        3:rgb=24'h010204;
                    endcase
                end
                @(negedge clk);de=0;
                repeat(160) @(negedge clk);
            end
            repeat(20) @(negedge clk);
        end
        if(pixels!=2*640*480 || lines!=960 || frames!=2)
            $fatal(1,"AXIS counts pixels=%0d lines=%0d frames=%0d",pixels,lines,frames);
        $display("PASS: 640x480 subtitle-to-AXIS RGB, 640 pixels/line, one TLAST/line, one SOF/frame");
        $finish;
    end
endmodule
