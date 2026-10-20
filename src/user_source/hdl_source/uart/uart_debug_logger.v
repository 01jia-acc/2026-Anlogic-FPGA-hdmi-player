`timescale 1ns/1ps

// Periodic diagnostic logger for the board USB-UART interface.
// Output format (one line every second, plus one line after reset):
//   P=1 I=1 T=1 B=0 D=0 E=00 V=0 R=00\r\n
module uart_debug_logger #(
    parameter integer CLOCK_FREQ_HZ = 50_000_000,
    parameter integer BAUD_RATE     = 115_200,
    parameter integer LOG_INTERVAL_MS = 1000
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       pll_ready,
    input  wire       sd_init_done,
    input  wire [3:0] state_code,
    input  wire       scan_busy,
    input  wire       scan_done,
    input  wire [7:0] scan_error,
    input  wire       display_valid,
    input  wire [7:0] rx_data,
    // FAT/SD 诊断快照：这些信号来自 SD 时钟域，仅用于低频日志观察。
    input  wire [3:0] scanner_state,
    input  wire       sector_active,
    input  wire [31:0] sector_lba,
    input  wire [8:0] sector_byte_count,
    input  wire [4:0] sd_state,
    input  wire       sd_cmd_error_latched,
    input  wire [15:0] resource_valid,
    input  wire [4:0] image_count,
    input wire [31:0] root_crc,
    input wire [15:0] root_bytes,
    input  wire [15:0] sector_done_count,
    input wire [4:0] load_image_number,
    input wire [31:0] load_cluster, load_size,
    input wire [31:0] accepted_cluster,
    input wire [63:0] load_pipeline,
    input wire [159:0] pcm_status,
    output wire       tx
);
    localparam integer LOG_INTERVAL_CYCLES =
        (CLOCK_FREQ_HZ / 1000) * LOG_INTERVAL_MS;
    // 诊断行示例：
    // P=1 I=1 T=1 B=1 D=0 E=00 V=0 S=2 A=1 L=00000000 C=000 Q=06 X=0 F=0 N=0000 R=00\r\n
    // J为加载的播放序号（十六进制、从1开始），K为首簇，Z为文件字节数。
    // H is the live pipeline snapshot, frozen by the scheduler on load abort.
    // M: all 16 PIC slots; U: scheduler picture count, hexadecimal.
    // Keep legacy F (the low four slots) for existing log readers.
    // O: PCM mask/flags/image/slot/error/cluster/size/consumed samples/current sample.
    // G: first cluster actually accepted by the image file source (compare K).
    localparam integer MESSAGE_LENGTH = 210;

    reg [31:0] interval_count;
    reg        boot_pending;
    reg        message_active;
    reg [7:0]  message_index;
    reg        tx_start;

    reg        snap_pll_ready;
    reg        snap_sd_init_done;
    reg [3:0]  snap_state_code;
    reg        snap_scan_busy;
    reg        snap_scan_done;
    reg [7:0]  snap_scan_error;
    reg        snap_display_valid;
    reg [7:0]  snap_rx_data;
    reg [3:0]  snap_scanner_state;
    reg        snap_sector_active;
    reg [31:0] snap_sector_lba;
    reg [8:0]  snap_sector_byte_count;
    reg [4:0]  snap_sd_state;
    reg        snap_sd_cmd_error_latched;
    reg [15:0] snap_resource_valid;
    reg [4:0] snap_image_count;
    reg [31:0] snap_root_crc;
    reg [15:0] snap_root_bytes;
    reg [15:0] snap_sector_done_count;
    reg [4:0] snap_load_image_number;
    reg [31:0] snap_load_cluster, snap_load_size;
    reg [31:0] snap_accepted_cluster;
    reg [63:0] snap_load_pipeline;
    reg [159:0] snap_pcm_status;

    wire       tx_busy;
    reg [7:0] tx_byte;
    reg [8:0] tx_symbol;
    reg [2:0] byte_settle;

    function [7:0] hex_ascii;
        input [3:0] value;
        begin
            if (value < 10)
                hex_ascii = 8'h30 + value;
            else
                hex_ascii = 8'h41 + value - 10;
        end
    endfunction

    // Register symbol selection before hexadecimal ASCII conversion.
    function [8:0] hex_symbol;
        input [3:0] value;
        begin hex_symbol = {1'b1,4'd0,value}; end
    endfunction

    function [8:0] message_byte;
        input [7:0] index;
        begin
            case (index)
                7'd0:  message_byte = "P";
                7'd1:  message_byte = "=";
                7'd2:  message_byte = hex_symbol({3'd0, snap_pll_ready});
                7'd3:  message_byte = " ";
                7'd4:  message_byte = "I";
                7'd5:  message_byte = "=";
                7'd6:  message_byte = hex_symbol({3'd0, snap_sd_init_done});
                7'd7:  message_byte = " ";
                7'd8:  message_byte = "T";
                7'd9:  message_byte = "=";
                7'd10: message_byte = hex_symbol(snap_state_code);
                7'd11: message_byte = " ";
                7'd12: message_byte = "B";
                7'd13: message_byte = "=";
                7'd14: message_byte = hex_symbol({3'd0, snap_scan_busy});
                7'd15: message_byte = " ";
                7'd16: message_byte = "D";
                7'd17: message_byte = "=";
                7'd18: message_byte = hex_symbol({3'd0, snap_scan_done});
                7'd19: message_byte = " ";
                7'd20: message_byte = "E";
                7'd21: message_byte = "=";
                7'd22: message_byte = hex_symbol(snap_scan_error[7:4]);
                7'd23: message_byte = hex_symbol(snap_scan_error[3:0]);
                7'd24: message_byte = " ";
                7'd25: message_byte = "V";
                7'd26: message_byte = "=";
                7'd27: message_byte = hex_symbol({3'd0, snap_display_valid});
                7'd28: message_byte = " ";
                7'd29: message_byte = "S";
                7'd30: message_byte = "=";
                7'd31: message_byte = hex_symbol(snap_scanner_state);
                7'd32: message_byte = " ";
                7'd33: message_byte = "A";
                7'd34: message_byte = "=";
                7'd35: message_byte = hex_symbol({3'd0, snap_sector_active});
                7'd36: message_byte = " ";
                7'd37: message_byte = "L";
                7'd38: message_byte = "=";
                7'd39: message_byte = hex_symbol(snap_sector_lba[31:28]);
                7'd40: message_byte = hex_symbol(snap_sector_lba[27:24]);
                7'd41: message_byte = hex_symbol(snap_sector_lba[23:20]);
                7'd42: message_byte = hex_symbol(snap_sector_lba[19:16]);
                7'd43: message_byte = hex_symbol(snap_sector_lba[15:12]);
                7'd44: message_byte = hex_symbol(snap_sector_lba[11:8]);
                7'd45: message_byte = hex_symbol(snap_sector_lba[7:4]);
                7'd46: message_byte = hex_symbol(snap_sector_lba[3:0]);
                7'd47: message_byte = " ";
                7'd48: message_byte = "C";
                7'd49: message_byte = "=";
                7'd50: message_byte = hex_symbol({3'd0, snap_sector_byte_count[8]});
                7'd51: message_byte = hex_symbol(snap_sector_byte_count[7:4]);
                7'd52: message_byte = hex_symbol(snap_sector_byte_count[3:0]);
                7'd53: message_byte = " ";
                7'd54: message_byte = "Q";
                7'd55: message_byte = "=";
                7'd56: message_byte = hex_symbol({3'd0, snap_sd_state[4]});
                7'd57: message_byte = hex_symbol(snap_sd_state[3:0]);
                7'd58: message_byte = " ";
                7'd59: message_byte = "X";
                7'd60: message_byte = "=";
                7'd61: message_byte = hex_symbol({3'd0, snap_sd_cmd_error_latched});
                7'd62: message_byte = " ";
                7'd63: message_byte = "F";
                7'd64: message_byte = "=";
                7'd65: message_byte = hex_symbol(snap_resource_valid[3:0]);
                7'd66: message_byte = " ";
                7'd67: message_byte = "N";
                7'd68: message_byte = "=";
                7'd69: message_byte = hex_symbol(snap_sector_done_count[15:12]);
                7'd70: message_byte = hex_symbol(snap_sector_done_count[11:8]);
                7'd71: message_byte = hex_symbol(snap_sector_done_count[7:4]);
                7'd72: message_byte = hex_symbol(snap_sector_done_count[3:0]);
                7'd73: message_byte = " ";
                7'd74: message_byte = "R";
                7'd75: message_byte = "=";
                7'd76: message_byte = hex_symbol(snap_rx_data[7:4]);
                7'd77: message_byte = hex_symbol(snap_rx_data[3:0]);
                7'd78: message_byte = " ";
                7'd79: message_byte = "J";
                7'd80: message_byte = "=";
                7'd81: message_byte = hex_symbol({3'd0,snap_load_image_number[4]});
                7'd82: message_byte = hex_symbol(snap_load_image_number[3:0]);
                7'd83: message_byte = " ";
                7'd84: message_byte = "K";
                7'd85: message_byte = "=";
                7'd86: message_byte = hex_symbol(snap_load_cluster[31:28]);
                7'd87: message_byte = hex_symbol(snap_load_cluster[27:24]);
                7'd88: message_byte = hex_symbol(snap_load_cluster[23:20]);
                7'd89: message_byte = hex_symbol(snap_load_cluster[19:16]);
                7'd90: message_byte = hex_symbol(snap_load_cluster[15:12]);
                7'd91: message_byte = hex_symbol(snap_load_cluster[11:8]);
                7'd92: message_byte = hex_symbol(snap_load_cluster[7:4]);
                7'd93: message_byte = hex_symbol(snap_load_cluster[3:0]);
                7'd94: message_byte = " ";
                7'd95: message_byte = "Z";
                7'd96: message_byte = "=";
                7'd97: message_byte = hex_symbol(snap_load_size[31:28]);
                7'd98: message_byte = hex_symbol(snap_load_size[27:24]);
                7'd99: message_byte = hex_symbol(snap_load_size[23:20]);
                7'd100: message_byte = hex_symbol(snap_load_size[19:16]);
                7'd101: message_byte = hex_symbol(snap_load_size[15:12]);
                7'd102: message_byte = hex_symbol(snap_load_size[11:8]);
                7'd103: message_byte = hex_symbol(snap_load_size[7:4]);
                7'd104: message_byte = hex_symbol(snap_load_size[3:0]);
                7'd105: message_byte = " ";
                7'd106: message_byte = "H";
                7'd107: message_byte = "=";
                8'd124: message_byte = " ";
                8'd125: message_byte = "M";
                8'd126: message_byte = "=";
                8'd127: message_byte = hex_symbol(snap_resource_valid[15:12]);
                8'd128: message_byte = hex_symbol(snap_resource_valid[11:8]);
                8'd129: message_byte = hex_symbol(snap_resource_valid[7:4]);
                8'd130: message_byte = hex_symbol(snap_resource_valid[3:0]);
                8'd131: message_byte = " ";
                8'd132: message_byte = "U";
                8'd133: message_byte = "=";
                8'd134: message_byte = hex_symbol({3'd0,snap_image_count[4]});
                8'd135: message_byte = hex_symbol(snap_image_count[3:0]);
                8'd136: message_byte = " ";
                8'd137: message_byte = "W";
                8'd138: message_byte = "=";
                8'd147: message_byte = " ";
                8'd148: message_byte = "Y";
                8'd149: message_byte = "=";
                8'd154: message_byte = " ";
                8'd155: message_byte = "O";
                8'd156: message_byte = "=";
                8'd197: message_byte = " ";
                8'd198: message_byte = "G";
                8'd199: message_byte = "=";
                8'd208: message_byte = 8'h0d;
                8'd209: message_byte = 8'h0a;
                default: begin
                    if(index>=108 && index<=123)
                        message_byte = hex_symbol(snap_load_pipeline[(123-index)*4 +:4]);
                    else if(index>=139 && index<=146)
                        message_byte = hex_symbol(snap_root_crc[(146-index)*4 +:4]);
                    else if(index>=150 && index<=153)
                        message_byte = hex_symbol(snap_root_bytes[(153-index)*4 +:4]);
                    else if(index>=157 && index<=196)
                        message_byte = hex_symbol(snap_pcm_status[(196-index)*4 +:4]);
                    else if(index>=200 && index<=207)
                        message_byte = hex_symbol(snap_accepted_cluster[(207-index)*4 +:4]);
                    else message_byte = " ";
                end
            endcase
        end
    endfunction

    always @(posedge clk or posedge rst) begin
        if(rst) begin tx_symbol<=0; tx_byte<=0; end
        else begin
            tx_symbol <= message_byte(message_index);
            tx_byte <= tx_symbol[8] ? hex_ascii(tx_symbol[3:0]) : tx_symbol[7:0];
        end
    end

    uart_tx #(
        .CLOCK_FREQ_HZ(CLOCK_FREQ_HZ),
        .BAUD_RATE    (BAUD_RATE)
    ) u_uart_tx (
        .clk      (clk),
        .rst      (rst),
        .tx_start (tx_start),
        .tx_data  (tx_byte),
        .tx_busy  (tx_busy),
        .tx       (tx)
    );

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            interval_count    <= 32'd0;
            boot_pending      <= 1'b1;
            message_active    <= 1'b0;
            message_index     <= 8'd0;
            tx_start          <= 1'b0;
            snap_pll_ready    <= 1'b0;
            snap_sd_init_done <= 1'b0;
            snap_state_code   <= 4'd0;
            snap_scan_busy    <= 1'b0;
            snap_scan_done    <= 1'b0;
            snap_scan_error   <= 8'd0;
            snap_display_valid<= 1'b0;
            snap_rx_data      <= 8'd0;
            snap_scanner_state<= 4'd0;
            snap_sector_active<= 1'b0;
            snap_sector_lba   <= 32'd0;
            snap_sector_byte_count <= 9'd0;
            snap_sd_state     <= 5'd0;
            snap_sd_cmd_error_latched <= 1'b0;
            snap_resource_valid <= 16'd0;
            snap_image_count <= 5'd0;
            snap_root_crc <= 0;
            snap_root_bytes <= 0;
            snap_sector_done_count <= 16'd0;
            snap_load_image_number<=0; snap_load_cluster<=0; snap_load_size<=0;
            snap_load_pipeline<=0;
            snap_pcm_status<=0;
            snap_accepted_cluster<=0; byte_settle<=0;
        end else begin
            // tx_start 是送给 uart_tx 的寄存器脉冲。由于两个 always 块在同一
            // 个时钟沿使用非阻塞赋值，当前 always 置 tx_start=1 后，uart_tx
            // 要到“下一拍”才能看到它。因此 message_index 不能在“发起请求”
            // 的这一拍递增，否则 uart_tx 采样到的将是下一个字符，导致第 0
            // 个字符丢失、后续字符隔一个丢一个。
            tx_start <= 1'b0;
            if(byte_settle!=0) byte_settle<=byte_settle-1'b1;

            if (!message_active) begin
                if (interval_count < LOG_INTERVAL_CYCLES - 1)
                    interval_count <= interval_count + 1'b1;

                if (boot_pending || interval_count >= LOG_INTERVAL_CYCLES - 1) begin
                    snap_pll_ready     <= pll_ready;
                    snap_sd_init_done  <= sd_init_done;
                    snap_state_code    <= state_code;
                    snap_scan_busy     <= scan_busy;
                    snap_scan_done     <= scan_done;
                    snap_scan_error    <= scan_error;
                    snap_display_valid <= display_valid;
                    snap_rx_data       <= rx_data;
                    snap_scanner_state <= scanner_state;
                    snap_sector_active <= sector_active;
                    snap_sector_lba    <= sector_lba;
                    snap_sector_byte_count <= sector_byte_count;
                    snap_sd_state      <= sd_state;
                    snap_sd_cmd_error_latched <= sd_cmd_error_latched;
                    snap_resource_valid <= resource_valid;
                    snap_image_count <= image_count;
                    snap_root_crc <= root_crc;
                    snap_root_bytes <= root_bytes;
                    snap_sector_done_count <= sector_done_count;
                    snap_load_image_number<=load_image_number;
                    snap_load_cluster<=load_cluster; snap_load_size<=load_size;
                    snap_load_pipeline<=load_pipeline;
                    snap_pcm_status<=pcm_status;
                    snap_accepted_cluster<=accepted_cluster;
                    byte_settle<=4;
                    message_index      <= 8'd0;
                    message_active     <= 1'b1;
                    boot_pending       <= 1'b0;
                    interval_count     <= 32'd0;
                end
            end else if (tx_start) begin
                // 本周期 tx_start 的旧值为 1，表示 uart_tx 将在本时钟沿
                // 接受 tx_byte(message_index)。现在才推进索引，确保当前字符
                // 在 uart_tx 采样前保持稳定。
                if (message_index == MESSAGE_LENGTH - 1) begin
                    // 最后一个字节（LF）已经被接受；允许下一轮定时重新组帧。
                    message_active <= 1'b0;
                    message_index  <= 8'd0;
                end else begin
                    message_index <= message_index + 1'b1;
                    byte_settle<=4;
                end
            end else if (!tx_busy && byte_settle==0) begin
                // 只发起请求，暂不改变 message_index。下一拍进入上面的
                // tx_start 分支后，才确认这个字符确实交给了 uart_tx。
                tx_start <= 1'b1;
            end
        end
    end
endmodule
