`timescale 1ns/1ps
// Real post-gain HDMI samples -> waveform -> subtitle -> AXIS at 640x480.
module tb_waveform_axis;
    reg clk=0,rst=1;
    always #20 clk=~clk;
    reg vs=1,de=0,enable=1;
    reg [11:0] x=0,y=0;
    reg [23:0] rgb=0;
    reg pcm_mode=1,pcm_valid=1;
    reg signed [15:0] pcm_sample=16384;
    wire av;
    wire [23:0] left;
    media_hdmi_audio_bridge #(.PCM_ENABLE(1),.VOLUME(192)) audio(
        .pixel_clk(clk),.rst(rst),.display_valid_sd(1'b1),.media_id_sd(2'd0),
        .commit_toggle_sd(1'b0),.pcm_active(1'b1),.pcm_valid(pcm_valid),
        .pcm_mode_sd(pcm_mode),.pcm_commit_ack_sd(1'b0),.stream_sample(pcm_sample),
        .pcm_ready(),.audio_valid(av),.audio_left_data(left),.audio_right_data(),
        .acr_valid(),.acr_cts(),.acr_n());
    wire wvs,wde,wframe,svs,sde,valid,last,user;
    wire [11:0] wx,wy;
    wire [23:0] wrgb,srgb,data;
    promax_audio_waveform #(.WAVE_Y(368)) wave(
        .clk(clk),.rst_n(!rst),.vsync(vs),.de(de),.rgb_in(rgb),
        .pixel_x(x),.pixel_y(y),.enable(enable),.pcm_tap_valid(av),.pcm_tap_left(left[23:8]),
        .rgb_out(wrgb),.out_vs(wvs),.out_de(wde),.out_x(wx),.out_y(wy),
        .frame_start_out(wframe),.waveform_active());
    scrolling_subtitle subtitle(.control_clk(clk),.pixel_clk(clk),.rst(rst),
        .rx_data(8'd0),.rx_valid(1'b0),.frame_start(wframe),.hide(1'b1),
        .in_vs(wvs),.in_de(wde),.pixel_x(wx),.pixel_y(wy),.rgb_in(wrgb),
        .out_vs(svs),.out_de(sde),.rgb_out(srgb),.busy(),.rejected_lines());
    video_rgb_to_axis_640x480 axis(.I_clk(clk),.I_rst(rst),.I_vs(svs),.I_de(sde),.I_rgb(srgb),
        .O_video_user(user),.O_video_valid(valid),.O_video_last(last),.O_video_data(data));

    reg [23:0] colors[0:4];
    reg [4:0] vp=0,lp=0,up=0;
    reg arm=1,prev_vs=1;
    integer i,pixels=0,lines=0,frames=0,amplitude=12,peak=12;
    reg [23:0] golden;
    always @(posedge clk) begin
        if(rst) begin vp=0;lp=0;up=0;arm=1;prev_vs=vs;end
        else begin
            if(vs!=prev_vs) arm=1;
            prev_vs=vs;
            golden=rgb;
            if(enable && de && x>=64 && x<576 && y>=368 && y<432) begin
                if(y==400) golden=24'h406060;
                if(y>=399-amplitude && y<=401-amplitude) golden=24'h20ff90;
            end
            if(enable && de && x>=584 && x<592 && y>=432-peak && y<432) golden=24'hffb020;
            for(i=4;i>0;i=i-1) colors[i]=colors[i-1];
            colors[0]=golden;
            vp={vp[3:0],de};lp={lp[3:0],de && x==639};up={up[3:0],de && arm};
            if(de) arm=0;
            #1;
            if(valid!==vp[4] || last!==lp[4] || user!==up[4]) $fatal(1,"waveform/subtitle AXIS framing mismatch");
            if(valid) begin
                if(data!==colors[4]) $fatal(1,"RGB alignment/amplitude mismatch at %0d,%0d: %h / %h",x,y,data,colors[4]);
                pixels=pixels+1;
            end
            if(last) lines=lines+1;
            if(user) frames=frames+1;
        end
    end
    integer f,xx,yy;
    initial begin
        repeat(4) @(negedge clk);rst=0;
        // At 48 kHz fill the 32x12 sample window with the actual 3/4-gain output.
        repeat(220000) @(negedge clk);
        for(f=0;f<3;f=f+1) begin
            @(negedge clk);vs=0;enable=(f!=1);
            if(f==2) begin amplitude=0;peak=0;end
            repeat(5) @(negedge clk);vs=1;
            for(yy=0;yy<480;yy=yy+1) begin
                for(xx=0;xx<640;xx=xx+1) begin
                    @(negedge clk);de=1;x=xx;y=yy;
                    rgb={xx[7:0],yy[7:0],8'hf5};
                end
                @(negedge clk);de=0;
                repeat(160) @(negedge clk);
            end
            // Fill with HDMI silence BEFORE the next frame: no valid PCM.
            if(f==0) pcm_valid=0;
            repeat(20) @(negedge clk);
        end
        if(pixels!=3*640*480 || lines!=1440 || frames!=3)
            $fatal(1,"frame counts pixels=%0d lines=%0d frames=%0d",pixels,lines,frames);
        $display("PASS: actual HDMI gain and missing-PCM silence, waveform/subtitle 640x480 RGB, preview hiding, SOF and TLAST");
        $finish;
    end
    initial begin #100000000; $fatal(1,"AXIS timeout");end
endmodule
