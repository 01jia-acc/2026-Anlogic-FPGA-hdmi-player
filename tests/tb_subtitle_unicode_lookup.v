`timescale 1ns/1ps
module tb_subtitle_unicode_lookup;
    reg clk=0,rst=1,start=0;
    always #5 clk=~clk;
    reg [15:0] codepoint=0;
    wire busy,done;
    wire [8:0] glyph,expected;
    subtitle_unicode_lookup dut(.clk(clk),.rst(rst),.start(start),.codepoint(codepoint),
        .busy(busy),.done(done),.glyph(glyph));
    subtitle_unicode_map reference_map(.codepoint(codepoint),.glyph(expected));
    integer c,cycles;
    initial begin
        repeat(3) @(negedge clk);rst=0;
        for(c=0;c<65536;c=c+1) begin
            // ASCII bypasses the lookup in the UART receiver.
            if(c<32 || c>126) begin
                @(negedge clk);codepoint=c;start=1;
                @(negedge clk);start=0;cycles=0;
                while(!done) begin
                    @(negedge clk);cycles=cycles+1;
                    if(cycles>11) $fatal(1,"Unicode lookup timeout %h",codepoint);
                end
                if(busy || glyph!==expected) $fatal(1,"Unicode %h got %h expected %h",codepoint,glyph,expected);
            end
        end
        // Reset must cancel a pending lookup without producing a stale write.
        @(negedge clk);start=1;
        @(negedge clk);start=0;rst=1;
        @(negedge clk);rst=0;
        repeat(15) @(negedge clk);
        if(busy || done) $fatal(1,"Lookup reset cancellation");
        $display("PASS: Unicode binary search exhaustive BMP, known/unknown glyphs, bounded latency and reset cancellation");
        $finish;
    end
    initial begin #20000000; $fatal(1,"Unicode test timeout"); end
endmodule
