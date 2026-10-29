`timescale 1ns/1ps
module tb_image_likes_osd;
    reg clk=0, src_clk=0, rst_n=0, frame_start=0, de=1, enable=1;
    always #7 clk=~clk;
    always #5 src_clk=~src_clk;
    reg [11:0] x=0,y=0,count=0;
    reg [3:0] source_image=0,display_image=0;
    wire [95:0] packet;
    wire ready;
    wire [23:0] rgb;
    subtitle_frame_commit mailbox (
        .src_clk(src_clk),.src_rst_n(rst_n),.src_commit(ready),.src_ready(ready),
        .src_text({17'd0,source_image,count,63'd0}),.src_length(6'd0),.src_enable(1'b1),
        .dst_clk(clk),.dst_rst_n(rst_n),.frame_start(!de),.dst_text(packet),
        .dst_length(),.dst_enable());
    image_likes_osd dut (
        .clk(clk),.rst_n(rst_n),.frame_start(frame_start),.de(de),.enable(enable),
        .pixel_x(x),.pixel_y(y),.count_bcd(packet[74:63]),.count_image(packet[78:75]),
        .display_image(display_image),.rgb_in(24'h123456),.rgb_out(rgb));
    task snapshot;
        begin
            de=0;repeat(40) @(negedge clk);
            frame_start=1;@(negedge clk);frame_start=0;de=1;#1;
        end
    endtask
    task pixel;
        input integer px,py;
        input [23:0] expected;
        begin x=px;y=py;#1;if(rgb!==expected) $fatal(1,"Pixel (%0d,%0d) = %h expected %h",px,py,rgb,expected);end
    endtask
    integer px,py,hearts,letters,file_handle;
    initial begin
        repeat(4) @(negedge clk);rst_n=1;snapshot;
        pixel(596,202,24'hff3048); // Left lobe.
        pixel(592,202,24'h123456); // Heart opening is transparent.
        pixel(601,230,24'hffffff); // One-digit zero centered below heart.
        pixel(589,230,24'h123456); // Suppressed leading zero is transparent.
        pixel(551,210,24'h123456);
        pixel(632,210,24'h123456);
        pixel(604,225,24'h123456); // Gap between heart and count.
        pixel(604,210,24'hff3048);
        pixel(560,220,24'h123456); // Old black panel location.
        pixel(612,230,24'h123456); // No duplicate units after shifted digit.
        count=12'h123;
        de=0;repeat(40) @(negedge clk);de=1;
        if(dut.count_q!=0) $fatal(1,"Count changed within frame");
        snapshot;
        if(dut.count_q!=12'h123) $fatal(1,"Mailbox count not frame committed");
        // Generate a small inspectable badge preview as well as checking coverage.
        file_handle=$fopen("tests/build/likes_badge.ppm","w");
        $fdisplay(file_handle,"P3\n100 72\n255");
        hearts=0;letters=0;
        for(py=192;py<264;py=py+1) begin
            for(px=542;px<642;px=px+1) begin
                x=px;y=py;#1;
                if(rgb==24'hff3048) hearts=hearts+1;
                if(rgb==24'hffffff) letters=letters+1;
                $fdisplay(file_handle,"%0d %0d %0d",rgb[23:16],rgb[15:8],rgb[7:0]);
            end
        end
        $fclose(file_handle);
        if(hearts<200 || letters<100) $fatal(1,"Missing heart or digits");
        pixel(591,230,24'hffffff); // Hundreds '1', three-digit left seat.
        count=12'h010;snapshot;
        pixel(597,230,24'hffffff); // Two-digit '10' centered at x=604.
        pixel(607,230,24'hffffff);
        pixel(589,230,24'h123456);
        pixel(619,230,24'h123456);
        // A different displayed picture cannot reuse the old picture's count.
        display_image=1;pixel(596,202,24'h123456);
        snapshot;pixel(596,202,24'h123456);
        source_image=1;count=12'h999;snapshot;
        pixel(596,202,24'hff3048);
        if(dut.count_q!=12'h999) $fatal(1,"New picture count not received");
        enable=0;pixel(596,202,24'h123456);
        enable=1;de=0;pixel(596,202,24'h123456);
        de=1;rst_n=0;#1;pixel(596,202,24'h123456);
        $display("PASS: async like snapshot, frame stability, heart/decimal glyphs, leading zero suppression, stale-image/preview/blanking/reset hiding and RGB passthrough");
        $finish;
    end
    initial begin #1000000;$fatal(1,"Likes OSD timeout");end
endmodule
