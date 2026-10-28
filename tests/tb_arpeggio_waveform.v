`timescale 1ns/1ps
module tb_arpeggio_waveform;
    reg clk=0, rst_n=0, vs=1, de=0, enable=1, mode=1, valid=0;
    always #5 clk=~clk;
    reg signed [15:0] sample=0;
    reg [11:0] x=0,y=0;
    wire [23:0] rgb;
    wire hit,ovs,ode,frame;
    wire [11:0] ox,oy;
    promax_audio_waveform #(.WAVE_Y(368)) dut (
        .clk(clk),.rst_n(rst_n),.vsync(vs),.de(de),.rgb_in(24'h123456),
        .pixel_x(x),.pixel_y(y),.enable(enable),.arpeggio_mode(mode),
        .pcm_tap_valid(valid),.pcm_tap_left(sample),.rgb_out(rgb),
        .out_vs(ovs),.out_de(ode),.out_x(ox),.out_y(oy),
        .frame_start_out(frame),.waveform_active(hit));
    integer expected[0:128];
    integer i,j,k,start_index,period,amplitude,n,bank,cycles,old_cycles;
    integer a,b,py,py_next,lo,hi,xx,yy;
    reg [23:0] want;
    function integer sine_value;
        input integer index,p,amp;
        begin sine_value=$rtoi(amp*$sin(6.283185307179586*index/p)); end
    endfunction
    function integer ordinate;
        input integer value;
        integer scaled;
        begin
            scaled=value >>> 9;
            if(scaled>28) scaled=28;
            if(scaled< -28) scaled=-28;
            ordinate=32-scaled;
        end
    endfunction
    task boundary;
        begin
            @(negedge clk);de=0;valid=0;vs=0;
            repeat(3) @(negedge clk);vs=1;
            repeat(3) @(negedge clk);
        end
    endtask
    task reset_scope;
        begin
            @(negedge clk);rst_n=0;valid=0;vs=1;de=0;mode=1;enable=1;
            repeat(3) @(negedge clk);rst_n=1;
            repeat(3) @(negedge clk);
            boundary;
        end
    endtask
    task feed_window;
        input integer p,amp;
        integer idx,value,previous;
        begin
            previous=0;start_index=-1;
            for(idx=0;idx<2*p+260;idx=idx+1) begin
                value=sine_value(idx,p,amp);
                if(start_index<0 && previous<0 && value>=0) start_index=idx;
                @(negedge clk);sample=value;valid=1;
                @(negedge clk);valid=0;
                previous=value;
            end
            if(start_index<0) $fatal(1,"No reference crossing");
            for(idx=0;idx<=128;idx=idx+1)
                expected[idx]=ordinate(sine_value(start_index+idx*2,p,amp));
            if(!dut.u_scope.pending) $fatal(1,"Capture must finish");
        end
    endtask
    task pixel;
        input integer px,py_in;
        begin
            @(negedge clk);x=px;y=py_in;de=1;
            repeat(2) @(posedge clk);#1;
            if(ox!==px || oy!==py_in || !ode || !ovs) $fatal(1,"scope RGB/timing alignment");
        end
    endtask
    task check_snapshot;
        begin
            if(!dut.u_scope.display_valid) $fatal(1,"No complete scope snapshot");
            bank=dut.u_scope.display_bank;
            cycles=0;
            for(j=0;j<128;j=j+1) begin
                a=dut.u_scope.points[bank*128+j] >> 8;
                b=dut.u_scope.points[bank*128+j] & 255;
                if(a!==expected[j] || b!==expected[j+1])
                    $fatal(1,"Actual sampled endpoints differ at %0d: %0d,%0d expected %0d,%0d",j,a,b,expected[j],expected[j+1]);
                if(a>32 && b<=32) cycles=cycles+1;
            end
        end
    endtask
    task scan_snapshot;
        begin
            for(yy=366;yy<434;yy=yy+1)
                for(xx=62;xx<578;xx=xx+1) begin
                    want=24'h123456;
                    if(xx>=64 && xx<576 && yy>=368 && yy<432) begin
                        j=(xx-64)/4;k=(xx-64)%4;
                        py=expected[j]+((expected[j+1]-expected[j])*k)/4;
                        py_next=expected[j]+((expected[j+1]-expected[j])*(k+1))/4;
                        lo=py<py_next ? py : py_next;
                        hi=py>py_next ? py : py_next;
                        want=(yy-368>=lo && yy-368<=hi+1) ? 24'h40efb0 :
                            yy==400 ? 24'h465864 : 24'h101c28;
                    end
                    pixel(xx,yy);
                    if(rgb!==want) $fatal(1,"Scope pixel %0d,%0d got %h expected %h",xx,yy,rgb,want);
                end
        end
    endtask
    initial begin
        // 400/800/1600 Hz: the same timebase must show more cycles at higher pitch.
        old_cycles=0;
        for(i=0;i<3;i=i+1) begin
            reset_scope;period=120 >> i;
            feed_window(period,9000);boundary;check_snapshot;
            if(cycles<=old_cycles) $fatal(1,"Higher frequency did not produce denser cycles");
            old_cycles=cycles;
            if(i==0) begin
                scan_snapshot;
                // A completed replacement may not modify the visible bank midframe.
                feed_window(period,4500);
                if(dut.u_scope.display_bank!=bank) $fatal(1,"Midframe bank swap");
                boundary;check_snapshot;scan_snapshot;
            end
        end
        // Saturation remains within the scope area; all pixels stay connected.
        reset_scope;feed_window(60,32767);boundary;check_snapshot;scan_snapshot;
        // Preview hides both trace and panel, while capture can still complete.
        enable=0;pixel(64,400);
        if(rgb!==24'h123456 || hit) $fatal(1,"Preview must hide scope panel");
        enable=1;
        // Missing samples clear stale oscillation at the next boundary.
        boundary;
        if(dut.u_scope.display_valid) $fatal(1,"Stalled stream retained old curve");
        pixel(100,400);if(rgb!==24'h40efb0) $fatal(1,"Silence baseline missing");
        pixel(100,380);if(rgb!==24'h101c28) $fatal(1,"Silence stale peak");
        // PCM switch is visually frame-held, and incomplete captures are discarded.
        @(negedge clk);mode=0;
        repeat(5) @(negedge clk);
        if(!dut.u_scope.mode_frame) $fatal(1,"Midframe display mode change");
        boundary;
        if(dut.u_scope.mode_frame || dut.u_scope.pending || dut.u_scope.capturing) $fatal(1,"PCM mode isolation");
        pixel(100,380);if(rgb!==24'h123456) $fatal(1,"PCM retained scope panel");
        @(negedge clk);mode=1;repeat(3) @(negedge clk);boundary;
        if(!dut.u_scope.mode_frame || dut.u_scope.display_valid) $fatal(1,"Source switch retained stale capture");
        feed_window(60,9000);boundary;check_snapshot;
        // Actual zero-valued samples, not just a missing strobe, flatten the trace.
        repeat(800) begin
            @(negedge clk);valid=1;sample=0;
            @(negedge clk);valid=0;
        end
        boundary;
        if(dut.u_scope.display_valid) $fatal(1,"Muted samples retained curve");
        // Switch sources during a partial acquisition, then acquire while hidden.
        reset_scope;
        for(n=0;n<12;n=n+1) begin
            @(negedge clk);valid=1;sample=(n%2) ? 1000 : -1000;
            @(negedge clk);valid=0;
        end
        if(!dut.u_scope.capturing) $fatal(1,"Expected unfinished capture");
        mode=0;repeat(4) @(negedge clk);boundary;
        if(dut.u_scope.capturing || dut.u_scope.pending) $fatal(1,"Source switch kept partial capture");
        mode=1;enable=0;repeat(4) @(negedge clk);boundary;
        feed_window(60,9000);boundary;check_snapshot;
        pixel(100,400);if(rgb!==24'h123456) $fatal(1,"Hidden capture changed background");
        enable=1;scan_snapshot;
        $display("PASS: arpeggio sampled waveform, 400/800/1600Hz density, trigger, gain/clipping, interpolation/pixels, frame-atomic buffers, silence, source switch and preview");
        $finish;
    end
    initial begin #20000000; $fatal(1,"scope timeout"); end
endmodule
