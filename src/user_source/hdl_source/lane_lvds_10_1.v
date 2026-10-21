


module lane_lvds_10_1 #(
    parameter DEVICE = "EG"
    )(
    input wire      I_pixel_clk,
    input wire      I_serial_clk,
    input wire      I_rst,
    input wire      I_load,

    input wire[9:0] I_data_in,

    output wire     O_serial_out
);

    reg[9:0] S_pixel_data;
    reg[9:0] S_shift_data;   
    reg[1:0] S_oddr_data;    


    // The TMDS encoder launches on the rising pixel edge. Capture halfway
    // through that pixel, then load all lanes together in the serial domain.
    // Never sample the live encoder bus at the next rising pixel edge.
    always @(negedge I_pixel_clk or posedge I_rst) begin
        if(I_rst)
            S_pixel_data <= 10'd0;
        else
            S_pixel_data <= I_data_in;
    end

    always @(posedge I_serial_clk or posedge I_rst) begin
        if(I_rst)
            S_shift_data <= 'd0;
        else
            if(I_load)
                S_shift_data <= S_pixel_data;
            else 
                S_shift_data <= S_shift_data << 2;
    end

    always @(posedge I_serial_clk or posedge I_rst) begin
        if(I_rst) S_oddr_data <= 2'd0;
        else S_oddr_data <= S_shift_data[9:8];
    end

    generate
        if(DEVICE == "EF2")
            begin
                EF2_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "EF3")
            begin
                EF3_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "EF4")
            begin
                EF4_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "SF1")
            begin
                SF1_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "EG")
            begin
                EG_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "PH1A")
            begin
                PH1_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "PH1P")
            begin
                PH1P_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "DR1")
            begin
                DR1_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
    endgenerate
    
endmodule
