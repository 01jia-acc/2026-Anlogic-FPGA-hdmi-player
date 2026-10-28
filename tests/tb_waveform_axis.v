`timescale 1ns/1ps
// Real post-gain HDMI samples -> waveform -> subtitle -> AXIS at 640x480.
module tb_waveform_axis #(parameter BRIGHTNESS_TEST=0);
    reg clk=0,rst=1;
    always #20 clk=~clk;
    reg vs=1,de=0,enable=1;
    reg [11:0] x=0,y=0;
    reg [23:0] rgb=0;
    reg control_clk=0;
    always #7 control_clk=~control_clk;
    reg [4:0] requested_brightness=20;
    reg [5:0] requested_contrast=40;
    integer expected_brightness=20;
    integer expected_contrast=40;
    wire [23:0] picture_rgb;
    generate if(BRIGHTNESS_TEST) begin : g_brightness
        wire ready;
        wire [95:0] snapshot;
        wire [4:0] level=snapshot[50] ? snapshot[55:51] : 5'd20;
        wire [5:0] contrast_level=snapshot[50] ? snapshot[62:57] : 6'd20;
        wire [23:0] adjusted_rgb;
        wire [23:0] brightness_rgb;
        reg prev_vs=1;
        always @(posedge clk) if(rst) prev_vs<=1; else prev_vs<=vs;
        wire frame_start=prev_vs && !vs;
        subtitle_frame_commit mailbox(
            .src_clk(control_clk),.src_rst_n(!rst),.src_commit(ready),.src_ready(ready),
            .src_text({33'd0,requested_contrast,1'b1,requested_brightness,1'b1,50'd0}),.src_length(6'd0),.src_enable(1'b1),
            .dst_clk(clk),.dst_rst_n(!rst),.frame_start(!de),.dst_text(snapshot),.dst_length(),.dst_enable());
        video_brightness brightness(.clk(clk),.rst_n(!rst),.frame_start(frame_start),
            .level(level),.contrast_level(contrast_level),.fade_alpha(8'd255),.rgb_in(rgb),.rgb_out(adjusted_rgb));
        volume_status_osd #(.OSD_X(200),.LABEL("BRI"),.DEFAULT_LEVEL(20)) osd(
            .clk(clk),.rst_n(!rst),.frame_start(frame_start),.de(de),.pixel_x(x),.pixel_y(y),
            .level(level),.adjust_mode(snapshot[56]),.rgb_in(adjusted_rgb),.rgb_out(brightness_rgb));
        volume_status_osd #(.OSD_X(376),.LABEL("CON"),.LEVEL_BITS(6),.DEFAULT_LEVEL(20)) contrast_osd(
            .clk(clk),.rst_n(!rst),.frame_start(frame_start),.de(de),.pixel_x(x),.pixel_y(y),
            .level(contrast_level),.adjust_mode(snapshot[56]),.rgb_in(brightness_rgb),.rgb_out(picture_rgb));
    end else begin : g_original
        assign picture_rgb=rgb;
    end endgenerate
    reg pcm_mode=1,pcm_valid=1;
    // Quiet PCM: 2560 becomes 1920 after the real 192/256 HDMI gain.
    // Raised thresholds must show two blocks instead of the previous three.
    reg signed [15:0] pcm_sample=2560;
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
        .clk(clk),.rst_n(!rst),.vsync(vs),.de(de),.rgb_in(picture_rgb),
        .pixel_x(x),.pixel_y(y),.enable(enable),.arpeggio_mode(1'b0),.pcm_tap_valid(av),.pcm_tap_left(left[23:8]),
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
    integer i,pixels=0,lines=0,frames=0,blocks=2;
    integer history[0:31];
    integer h,column_blocks;
    reg [23:0] golden;
    integer brightness_gain,brightness_alpha,contrast_gain,rr,gg,bb,osd_col;
    reg [7:0] osd_char;
    reg [34:0] osd_glyph;
    reg [4:0] osd_row;
    function [34:0] reference_glyph;
        input [7:0] ch;
        begin
            case(ch)
                "B":reference_glyph=35'b11110_10001_10001_11110_10001_10001_11110;
                "R":reference_glyph=35'b11110_10001_10001_11110_10100_10010_10001;
                "I":reference_glyph=35'b11111_00100_00100_00100_00100_00100_11111;
                "C":reference_glyph=35'b01111_10000_10000_10000_10000_10000_01111;
                "O":reference_glyph=35'b01110_10001_10001_10001_10001_10001_01110;
                "N":reference_glyph=35'b10001_11001_10101_10011_10001_10001_10001;
                "2":reference_glyph=35'b01110_10001_00001_00010_00100_01000_11111;
                "0":reference_glyph=35'b01110_10001_10001_10001_10001_10001_01110;
                "1":reference_glyph=35'b00100_01100_00100_00100_00100_00100_01110;
                "5":reference_glyph=35'b11111_10000_10000_11110_00001_00001_11110;
                "%":reference_glyph=35'b11001_11010_00010_00100_01000_01011_10011;
                default:reference_glyph=0;
            endcase
        end
    endfunction
    always @(posedge clk) begin
        if(rst) begin
            vp=0;lp=0;up=0;arm=1;prev_vs=vs;
            for(h=0;h<32;h=h+1) history[h]=1;
        end
        else begin
            if(vs!=prev_vs) arm=1;
            if(prev_vs && !vs) begin
                for(h=0;h<31;h=h+1) history[h]=history[h+1];
                history[31]=blocks;
            end
            prev_vs=vs;
            golden=rgb;
            if(BRIGHTNESS_TEST) begin
                brightness_gain=(expected_brightness*256+10)/20;
                brightness_alpha=(255*brightness_gain+128)/256;
                contrast_gain=(expected_contrast*128+10)/20;
                rr=rgb[23:16];gg=rgb[15:8];bb=rgb[7:0];
                rr=128+(((rr-128)*contrast_gain) >>> 7);
                gg=128+(((gg-128)*contrast_gain) >>> 7);
                bb=128+(((bb-128)*contrast_gain) >>> 7);
                if(rr<0) rr=0; else if(rr>255) rr=255;
                if(gg<0) gg=0; else if(gg>255) gg=255;
                if(bb<0) bb=0; else if(bb>255) bb=255;
                rr=(rr*brightness_alpha+255)/256;
                gg=(gg*brightness_alpha+255)/256;
                bb=(bb*brightness_alpha+255)/256;
                golden={rr[7:0],gg[7:0],bb[7:0]};
                if(de && x>=200 && x<264 && y>=8 && y<15) begin
                    osd_col=(x-200)/8;
                    case(osd_col)
                        0:osd_char="B"; 1:osd_char="R"; 2:osd_char="I";
                        4:osd_char=expected_brightness==20 ? "1" : " ";
                        5:osd_char=expected_brightness==20 ? "0" : (expected_brightness==10 ? "5" : " ");
                        6:osd_char="0"; 7:osd_char="%"; default:osd_char=" ";
                    endcase
                    osd_glyph=reference_glyph(osd_char);
                    osd_row=osd_glyph >> ((14-y)*5);
                    if((x-200)%8<5 && osd_row[4-(x-200)%8]) golden=24'hffff00;
                end
                if(de && x>=376 && x<440 && y>=8 && y<15) begin
                    osd_col=(x-376)/8;
                    case(osd_col)
                        0:osd_char="C"; 1:osd_char="O"; 2:osd_char="N";
                        4:osd_char=expected_contrast>=40 ? "2" : expected_contrast>=20 ? "1" : " ";
                        5:osd_char=expected_contrast<2 ? " " : 8'd48+((expected_contrast/2)%10);
                        6:osd_char=expected_contrast%2 ? "5" : "0";
                        7:osd_char="%"; default:osd_char=" ";
                    endcase
                    osd_glyph=reference_glyph(osd_char);
                    osd_row=osd_glyph >> ((14-y)*5);
                    if((x-376)%8<5 && osd_row[4-(x-376)%8]) golden=24'hffff00;
                end
            end
            column_blocks=(x>=64 && x<576) ? history[(x-64)/16] : 1;
            if(enable && de && x>=64 && x<576 && y>=432-column_blocks*8 && y<432 &&
                (x-64)%16>=2 && (x-64)%16<14 && (431-y)%8<4)
                golden=(431-y)%8==3 ? 24'hb8f438 : 24'h20ee70;
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
        // Settle real 48kHz attack, then append snapshots at VS boundaries.
        repeat(1000000) @(negedge clk);
        vs=0;repeat(5) @(negedge clk);vs=1;
        repeat(420000) @(negedge clk);
        for(f=0;f<3;f=f+1) begin
            if(BRIGHTNESS_TEST) begin
                expected_brightness=f==0 ? 20 : (f==1 ? 0 : 10);
                expected_contrast=f==0 ? 40 : (f==1 ? 0 : 20);
            end
            @(negedge clk);vs=0;enable=(f!=1);
            if(f==2) blocks=1;
            repeat(5) @(negedge clk);vs=1;
            for(yy=0;yy<480;yy=yy+1) begin
                for(xx=0;xx<640;xx=xx+1) begin
                    @(negedge clk);de=1;x=xx;y=yy;
                    rgb={xx[7:0],yy[7:0],8'hf5};
                end
                @(negedge clk);de=0;
                // The mailbox may change at the next line blanking; brightness
                // AND its displayed value must still wait until the next frame.
                if(BRIGHTNESS_TEST && yy==100) begin
                    requested_brightness=f==0 ? 0 : (f==1 ? 10 : 20);
                    requested_contrast=f==0 ? 0 : (f==1 ? 20 : 40);
                end
                repeat(160) @(negedge clk);
            end
            // Fill with HDMI silence BEFORE the next frame: no valid PCM.
            if(f==0) pcm_valid=0;
            repeat(20) @(negedge clk);
            if(f==1) begin
                // Allow slow release; new silence must retain older columns.
                repeat(3000000) @(negedge clk);
                blocks=1;
                vs=0;repeat(5) @(negedge clk);vs=1;
                repeat(420000) @(negedge clk);
            end
        end
        if(pixels!=3*640*480 || lines!=1440 || frames!=3)
            $fatal(1,"frame counts pixels=%0d lines=%0d frames=%0d",pixels,lines,frames);
        $display("PASS: actual HDMI gain, scrolling history with silent tail, waveform/subtitle 640x480 RGB, preview hiding, SOF and TLAST");
        if(BRIGHTNESS_TEST) $display("PASS: asynchronous brightness/contrast mailbox, three full frames at 200/0/100 percent contrast, frame-stable RGB/BRI/CON, waveform and AXIS alignment");
        $finish;
    end
    initial begin #300000000; $fatal(1,"AXIS timeout");end
endmodule
