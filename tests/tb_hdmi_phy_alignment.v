`timescale 1ns/1ps
// Independent lane propagation delays plus all eight serial-clock phases.
// A live-bus load near a rising pixel edge mixes adjacent TMDS symbols.
module tb_hdmi_phy_alignment;
    parameter integer SERIAL_PHASE=2;
    reg pixel_clk=0, serial_clk=0, rst=1;
    always #20 pixel_clk=~pixel_clk;
    initial begin
        #(SERIAL_PHASE);
        forever begin serial_clk=1; #4; serial_clk=0; #4; end
    end
    reg [9:0] ch0=0, ch1=0, ch2=0, chclk=10'b0000011111;
    wire [3:0] serial_out;
    hdmi_phy_wrapper dut(.I_pixel_clk(pixel_clk),.I_serial_clk(serial_clk),.I_rst(rst),
        .I_tmds_channel_0(ch0),.I_tmds_channel_1(ch1),.I_tmds_channel_2(ch2),
        .I_tmds_channel_clk(chclk),.O_tmds_ch0_p(serial_out[0]),
        .O_tmds_ch1_p(serial_out[1]),.O_tmds_ch2_p(serial_out[2]),
        .O_tmds_clk_p(serial_out[3]));
    integer symbol=0;
    always @(posedge pixel_clk) begin
        symbol=symbol+1;
        ch0 <= #1 symbol[9:0];
        ch1 <= #3 (symbol[9:0]^10'h2aa);
        ch2 <= #6 (~symbol[9:0]);
    end
    function [9:0] reverse_bits;
        input [9:0] value;
        integer j;
        begin for(j=0;j<10;j=j+1) reverse_bits[j]=value[9-j]; end
    endfunction
    reg [9:0] held [0:3];
    reg [9:0] shifted [0:3];
    reg [1:0] pair [0:3];
    reg [3:0] expected_rise, expected_fall;
    integer j, loads=0, since_load=0, epochs=0;
    time last_buffer_update;
    always @(negedge pixel_clk or posedge rst) begin
        last_buffer_update=$time;
        if(rst) begin for(j=0;j<4;j=j+1) held[j]=0; end
        else begin
            held[0]=reverse_bits(ch0); held[1]=reverse_bits(ch1);
            held[2]=reverse_bits(ch2); held[3]=reverse_bits(chclk);
        end
    end
    always @(posedge serial_clk or posedge rst) begin
        if(rst) begin
            for(j=0;j<4;j=j+1) begin shifted[j]=0; pair[j]=0; end
            since_load=0; epochs=0; expected_rise=0; expected_fall=0;
        end else begin
            since_load=since_load+1;
            for(j=0;j<4;j=j+1) begin
                expected_rise[j]=pair[j][1]; expected_fall[j]=pair[j][0];
                pair[j]=shifted[j][9:8];
            end
            if(dut.S_load_en) begin
                if(epochs!=0 && since_load!=5) $fatal(1,"Word boundary not five serial clocks");
                if($time-last_buffer_update<15 || $time-last_buffer_update>25)
                    $fatal(1,"Load is outside buffered symbol's stable window");
                epochs=epochs+1; loads=loads+1; since_load=0;
                for(j=0;j<4;j=j+1) shifted[j]=held[j];
            end else for(j=0;j<4;j=j+1) shifted[j]=shifted[j]<<2;
            #0.1;
            if(dut.u0_lane_lvds_8_1.S_shift_data!==shifted[0] ||
               dut.u1_lane_lvds_8_1.S_shift_data!==shifted[1] ||
               dut.u2_lane_lvds_8_1.S_shift_data!==shifted[2] ||
               dut.u3_lane_lvds_8_1.S_shift_data!==shifted[3])
                $fatal(1,"Mixed/lost TMDS symbols phase=%0d",SERIAL_PHASE);
            if(serial_out!==expected_rise) $fatal(1,"DDR rising bit order");
        end
    end
    always @(negedge serial_clk) if(!rst) begin
        #0.1;
        if(serial_out!==expected_fall) $fatal(1,"DDR falling bit order");
    end
    initial begin
        #73; rst=0;
        #10000; rst=1;
        #23; rst=0;
        #10000;
        if(loads<490) $fatal(1,"Too few transmitted symbols");
        $display("PASS: four HDMI lanes, phase %0d, %0d words, reset, DDR bit order",SERIAL_PHASE,loads);
        $finish;
    end
endmodule

// Functional DDR primitive model; physical setup/hold still requires TD STA.
module EG_LOGIC_ODDR(input wire clk,rst,d0,d1,output reg q);
    reg fall_bit;
    always @(posedge clk or posedge rst) begin
        if(rst) begin q<=0; fall_bit<=0; end
        else begin q<=d0; fall_bit<=d1; end
    end
    always @(negedge clk) if(!rst) q<=fall_bit;
endmodule
