`timescale 1ns/1ps

// Reuses the lesson's PCM/DDS approach with the existing HDMI 1.4b core.
// Its audio input has no ready output: one registered strobe per PCM frame.
module media_hdmi_audio_bridge #(
    parameter integer CLOCK_HZ = 25_000_000,
    parameter integer TEST_ONLY = 0,
    parameter integer PCM_ENABLE = 0,
    parameter [7:0] VOLUME = 8'd192,
    parameter integer BLIP_SAMPLES = 4800,
    parameter integer NOTE_SAMPLES = 12000
) (
    input  wire        pixel_clk,
    input  wire        rst,
    input  wire        display_valid_sd,
    input  wire [1:0]  media_id_sd,
    input  wire        commit_toggle_sd,
    input wire pcm_active, pcm_valid,
    input wire pcm_mode_sd,
    input wire pcm_commit_ack_sd,
    input wire signed [15:0] stream_sample,
    output wire pcm_ready,
    output reg         audio_valid,
    output reg  [23:0] audio_left_data,
    output reg  [23:0] audio_right_data,
    output reg         acr_valid,
    output wire [19:0] acr_cts,
    output wire [19:0] acr_n
);
    localparam integer SAMPLE_HZ = 48000;
    localparam integer RATE_WIDTH = $clog2(CLOCK_HZ);
    localparam [RATE_WIDTH-1:0] RATE_STEP = SAMPLE_HZ;
    localparam [RATE_WIDTH-1:0] RATE_LIMIT = CLOCK_HZ - SAMPLE_HZ;

    // Bundled-data CDC: the source holds img_idx between completed loads.
    // Data passes two registers; the commit toggle passes three plus edge
    // detection, so 1 -> 2 cannot transiently select image 0 or 3.
    (* ASYNC_REG = "TRUE" *) reg [1:0] media_id_meta;
    (* ASYNC_REG = "TRUE" *) reg [1:0] media_id_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] display_valid_sync;
    (* ASYNC_REG = "TRUE" *) reg [2:0] commit_sync;
    reg commit_seen;
    reg [1:0] media_id;
    reg media_active;

    always @(posedge pixel_clk or posedge rst) begin
        if (rst) begin
            media_id_meta <= 2'd0;
            media_id_sync <= 2'd0;
            display_valid_sync <= 2'b00;
            commit_sync <= 3'b000;
            commit_seen <= 1'b0;
            media_id <= 2'd0;
            media_active <= 1'b0;
        end else begin
            media_id_meta <= media_id_sd;
            media_id_sync <= media_id_meta;
            display_valid_sync <= {display_valid_sync[0], display_valid_sd};
            commit_sync <= {commit_sync[1:0], commit_toggle_sd};
            commit_seen <= commit_sync[2];

            if (!display_valid_sync[1])
                media_active <= 1'b0;
            else if (commit_sync[2] != commit_seen) begin
                media_id <= media_id_sync;
                media_active <= 1'b1;
            end
        end
    end

    reg [RATE_WIDTH-1:0] rate_accumulator;
    wire sample_tick = rate_accumulator >= RATE_LIMIT;
    wire use_media = media_active && (TEST_ONLY == 0);
    wire signed [15:0] test_sample;
    wire signed [15:0] media_sample;
    (* ASYNC_REG = "TRUE" *) reg [1:0] pcm_mode_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] pcm_ack_sync;
    always @(posedge pixel_clk or posedge rst)
        if (rst) begin pcm_mode_sync <= 0; pcm_ack_sync<=0; end
        else begin
            pcm_mode_sync <= {pcm_mode_sync[0], pcm_mode_sd};
            pcm_ack_sync <= {pcm_ack_sync[0],pcm_commit_ack_sd};
        end
    wire pcm_selected = PCM_ENABLE && !TEST_ONLY && pcm_mode_sync[1];
    wire use_pcm = pcm_selected && pcm_active && pcm_ack_sync[1]==commit_toggle_sd;
    assign pcm_ready = sample_tick && use_pcm;
    // PCM模式缺文件、换音或欠载时静音，不自动回退琶音或测试音。
    wire signed [15:0] selected_sample = pcm_selected ? (use_pcm && pcm_valid ? stream_sample : 16'sd0) :
                                       use_media ? media_sample : test_sample;
    wire signed [24:0] scaled_sample = selected_sample * $signed({1'b0, VOLUME});
    // Gain is 0..255 / 256, so even full-scale signed file samples cannot clip.
    wire signed [15:0] pcm_sample = scaled_sample >>> 8;

    pcm_test_tone u_test_tone (
        .clk(pixel_clk), .rst(rst), .enable(!use_media && !use_pcm),
        .sample_tick(sample_tick), .sample(test_sample), .rom_index()
    );

    pcm_media_tone #(
        .BLIP_SAMPLES(BLIP_SAMPLES), .NOTE_SAMPLES(NOTE_SAMPLES)
    ) u_media_tone (
        .clk(pixel_clk), .rst(rst), .enable(use_media && !use_pcm),
        .sample_tick(sample_tick), .media_id(media_id),
        .sample(media_sample), .blip_active()
    );

    // 48 samples per ACR period: fs = Fpixel * N / (128 * CTS).
    // With 25 MHz / 48 kHz: N=6144, CTS=25000, one ACR strobe per ms.
    // CLOCK_HZ must be a multiple of 1000 for this fixed integer CTS.
    assign acr_n = 20'd6144;
    assign acr_cts = CLOCK_HZ / 1000;
    reg [5:0] acr_sample_count;

    always @(posedge pixel_clk or posedge rst) begin
        if (rst) begin
            rate_accumulator <= 0;
            audio_valid <= 1'b0;
            audio_left_data <= 24'd0;
            audio_right_data <= 24'd0;
            acr_sample_count <= 6'd0;
            acr_valid <= 1'b0;
        end else begin
            audio_valid <= sample_tick;
            acr_valid <= 1'b0;
            if (sample_tick) begin
                rate_accumulator <= rate_accumulator - RATE_LIMIT;
                // Left-align signed 16-bit PCM into the core's 24-bit word.
                audio_left_data <= {pcm_sample, 8'd0};
                audio_right_data <= {pcm_sample, 8'd0};
                if (acr_sample_count == 6'd47) begin
                    acr_sample_count <= 6'd0;
                    acr_valid <= 1'b1;
                end else begin
                    acr_sample_count <= acr_sample_count + 1'b1;
                end
            end else begin
                rate_accumulator <= rate_accumulator + RATE_STEP;
            end
        end
    end
endmodule
