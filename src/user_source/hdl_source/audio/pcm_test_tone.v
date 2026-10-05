`timescale 1ns/1ps

// Adapted from the supplied HDMI audio lesson: 16 samples per sine period.
// The bridge supplies an exact-average 48 kHz clock enable, giving 3 kHz.
module pcm_test_tone (
    input  wire              clk,
    input  wire              rst,
    input  wire              enable,
    input  wire              sample_tick,
    output wire signed [15:0] sample,
    output reg  [3:0]        rom_index
);
    function signed [15:0] sine_sample;
        input [3:0] index;
        begin
            case (index)
                4'd0:  sine_sample = 16'sd0;
                4'd1:  sine_sample = 16'sd4592;
                4'd2:  sine_sample = 16'sd8485;
                4'd3:  sine_sample = 16'sd11087;
                4'd4:  sine_sample = 16'sd12000;
                4'd5:  sine_sample = 16'sd11087;
                4'd6:  sine_sample = 16'sd8485;
                4'd7:  sine_sample = 16'sd4592;
                4'd8:  sine_sample = 16'sd0;
                4'd9:  sine_sample = -16'sd4592;
                4'd10: sine_sample = -16'sd8485;
                4'd11: sine_sample = -16'sd11087;
                4'd12: sine_sample = -16'sd12000;
                4'd13: sine_sample = -16'sd11087;
                4'd14: sine_sample = -16'sd8485;
                default: sine_sample = -16'sd4592;
            endcase
        end
    endfunction

    assign sample = sine_sample(rom_index);

    always @(posedge clk or posedge rst) begin
        if (rst)
            rom_index <= 4'd0;
        else if (!enable)
            rom_index <= 4'd0;
        else if (sample_tick)
            rom_index <= rom_index + 1'b1;
    end
endmodule
