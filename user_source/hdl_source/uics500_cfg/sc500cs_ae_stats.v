`timescale 1ns / 1ps

// Passive RAW10 statistics. Four consecutive pixels, lowest slice first.
// BLACK_LEVEL=0 and SAT_THRESHOLD=1000 are engineering defaults, not sensor
// calibration values. No ready, control request, or I2C output exists here.
//
// Native frame markers are REQUIRED: uial2axis generates user/last by counting
// beats and wraps after a nominal frame, so these alone cannot detect a short
// or oversized physical frame. Publish only on the independent native EOF.
// A valid bubble is NOT an error or an end of line. Missing data is detected
// by position/size checks at native EOF or by an early native SOF.
module sc500cs_ae_stats #(
    parameter integer FRAME_WIDTH = 1280,
    parameter integer FRAME_HEIGHT = 720,
    parameter integer AE_X_START = FRAME_WIDTH / 4,
    parameter integer AE_X_END = FRAME_WIDTH * 3 / 4,
    parameter integer AE_Y_START = FRAME_HEIGHT / 4,
    parameter integer AE_Y_END = FRAME_HEIGHT * 3 / 4,
    parameter [9:0] BLACK_LEVEL = 10'd0,
    parameter [9:0] SAT_THRESHOLD = 10'd1000,
    parameter integer COUNT_WIDTH = $clog2(FRAME_WIDTH * FRAME_HEIGHT + 1),
    parameter integer SUM_WIDTH = COUNT_WIDTH + 10
)(
    input wire I_clk,
    input wire I_rst_n,
    input wire [39:0] I_tdata,
    input wire I_tvalid,
    input wire I_tuser,
    input wire I_tlast,
    input wire [15:0] I_pixel_x,
    input wire [13:0] I_pixel_y,
    input wire I_frame_start,
    input wire I_frame_end,
    output reg O_ae_result_valid,
    output reg [SUM_WIDTH-1:0] O_ae_raw_sum,
    output reg [COUNT_WIDTH-1:0] O_ae_pixel_count,
    output reg [COUNT_WIDTH-1:0] O_ae_saturated_count,
    // Pulses with result_valid for a good frame; zero for rejected frames.
    output reg O_ae_frame_good,
    output wire [15:0] O_dbg_x,
    output wire [13:0] O_dbg_y,
    output wire [COUNT_WIDTH-1:0] O_dbg_frame_pixel_count,
    output wire [14:0] O_dbg_frame_line_count,
    output wire O_dbg_frame_bad
);
    localparam [COUNT_WIDTH-1:0] FRAME_PIXELS = FRAME_WIDTH * FRAME_HEIGHT;
    localparam [COUNT_WIDTH-1:0] ROI_PIXELS =
        (AE_X_END - AE_X_START) * (AE_Y_END - AE_Y_START);
    localparam [15:0] LAST_X = FRAME_WIDTH - 4;
    localparam [13:0] LAST_Y = FRAME_HEIGHT - 1;
    localparam [14:0] FRAME_LINES = FRAME_HEIGHT;
    localparam [COUNT_WIDTH-1:0] FOUR_PIXELS = 4;

    reg [11:0] beat_sum; // 4 * 1023 = 4092: twelve bits
    reg [2:0] beat_count, beat_saturated_count; // zero through four
    reg [9:0] raw_pixel, corrected;
    reg [16:0] pixel_x;
    integer lane;
    always @(*) begin
        beat_sum = 12'd0;
        beat_count = 3'd0;
        beat_saturated_count = 3'd0;
        raw_pixel = 10'd0;
        corrected = 10'd0;
        pixel_x = 17'd0;
        for (lane = 0; lane < 4; lane = lane + 1) begin
            raw_pixel = I_tdata[lane*10 +: 10];
            corrected = (raw_pixel > BLACK_LEVEL) ?
                        raw_pixel - BLACK_LEVEL : 10'd0;
            pixel_x = {1'b0, I_pixel_x} + {15'd0, lane[1:0]};
            if (I_tvalid && pixel_x >= AE_X_START && pixel_x < AE_X_END &&
                I_pixel_y >= AE_Y_START && I_pixel_y < AE_Y_END) begin
                beat_sum = beat_sum + {2'b00, corrected};
                beat_count = beat_count + 3'd1;
                if (corrected >= SAT_THRESHOLD)
                    beat_saturated_count = beat_saturated_count + 3'd1;
            end
        end
    end

    reg active, started, complete, bad;
    reg active_next, started_next, complete_next, bad_next;
    reg [15:0] expected_x, expected_x_next;
    reg [13:0] expected_y, expected_y_next;
    reg [14:0] line_count, line_count_next;
    reg [COUNT_WIDTH-1:0] frame_pixels, frame_pixels_next;
    reg [SUM_WIDTH-1:0] frame_sum, frame_sum_next;
    reg [COUNT_WIDTH-1:0] roi_count, roi_count_next;
    reg [COUNT_WIDTH-1:0] sat_count, sat_count_next;
    reg publish;

    assign O_dbg_x = I_pixel_x;
    assign O_dbg_y = I_pixel_y;
    assign O_dbg_frame_pixel_count = frame_pixels;
    assign O_dbg_frame_line_count = line_count;
    assign O_dbg_frame_bad = bad;

    // Compute the post-beat state BEFORE deciding whether EOF can publish it.
    // There is exactly one assignment to each accumulator in the clocked block.
    always @(*) begin
        active_next = active;
        started_next = started;
        complete_next = complete;
        bad_next = bad;
        expected_x_next = expected_x;
        expected_y_next = expected_y;
        line_count_next = line_count;
        frame_pixels_next = frame_pixels;
        frame_sum_next = frame_sum;
        roi_count_next = roi_count;
        sat_count_next = sat_count;
        publish = 1'b0;

        if (I_frame_start) begin
            // Discard any previous uncommitted frame, even if nominally full.
            active_next = 1'b1;
            started_next = 1'b0;
            complete_next = 1'b0;
            // Existing converter resets counters on native SOF instead of
            // advancing them. A coincident payload is therefore not accepted.
            bad_next = I_tvalid || I_frame_end;
            expected_x_next = 16'd0;
            expected_y_next = 14'd0;
            line_count_next = 15'd0;
            frame_pixels_next = {COUNT_WIDTH{1'b0}};
            frame_sum_next = {SUM_WIDTH{1'b0}};
            roi_count_next = {COUNT_WIDTH{1'b0}};
            sat_count_next = {COUNT_WIDTH{1'b0}};
        end

        if (active_next && !I_frame_start) begin
            if (!I_tvalid && (I_tuser || I_tlast))
                bad_next = 1'b1;
            if (I_tvalid && !bad_next) begin
                if (complete_next ||
                    I_pixel_x != expected_x_next || I_pixel_y != expected_y_next ||
                    I_tuser != !started_next ||
                    I_tlast != (I_pixel_x == LAST_X)) begin
                    bad_next = 1'b1;
                end else begin
                    started_next = 1'b1;
                    frame_sum_next = frame_sum_next +
                                     {{(SUM_WIDTH-12){1'b0}}, beat_sum};
                    roi_count_next = roi_count_next +
                                     {{(COUNT_WIDTH-3){1'b0}}, beat_count};
                    sat_count_next = sat_count_next +
                                     {{(COUNT_WIDTH-3){1'b0}}, beat_saturated_count};
                    frame_pixels_next = frame_pixels_next + FOUR_PIXELS;
                    if (I_tlast) begin
                        line_count_next = line_count_next + 15'd1;
                        expected_x_next = 16'd0;
                        if (I_pixel_y == LAST_Y)
                            complete_next = 1'b1;
                        else
                            expected_y_next = expected_y_next + 14'd1;
                    end else
                        expected_x_next = expected_x_next + 16'd4;
                end
            end
        end

        if (I_frame_end && active_next) begin
            publish = !I_frame_start && !bad_next && started_next && complete_next &&
                      frame_pixels_next == FRAME_PIXELS &&
                      line_count_next == FRAME_LINES && roi_count_next == ROI_PIXELS;
            bad_next = !publish;
            active_next = 1'b0;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            active <= 1'b0;
            started <= 1'b0;
            complete <= 1'b0;
            bad <= 1'b0;
            expected_x <= 16'd0;
            expected_y <= 14'd0;
            line_count <= 15'd0;
            frame_pixels <= {COUNT_WIDTH{1'b0}};
            frame_sum <= {SUM_WIDTH{1'b0}};
            roi_count <= {COUNT_WIDTH{1'b0}};
            sat_count <= {COUNT_WIDTH{1'b0}};
            O_ae_result_valid <= 1'b0;
            O_ae_frame_good <= 1'b0;
            O_ae_raw_sum <= {SUM_WIDTH{1'b0}};
            O_ae_pixel_count <= {COUNT_WIDTH{1'b0}};
            O_ae_saturated_count <= {COUNT_WIDTH{1'b0}};
        end else begin
            active <= active_next;
            started <= started_next;
            complete <= complete_next;
            bad <= bad_next;
            expected_x <= expected_x_next;
            expected_y <= expected_y_next;
            line_count <= line_count_next;
            frame_pixels <= frame_pixels_next;
            frame_sum <= frame_sum_next;
            roi_count <= roi_count_next;
            sat_count <= sat_count_next;
            O_ae_result_valid <= publish;
            O_ae_frame_good <= publish;
            if (publish) begin
                O_ae_raw_sum <= frame_sum_next;
                O_ae_pixel_count <= roi_count_next;
                O_ae_saturated_count <= sat_count_next;
            end
        end
    end

    // synthesis translate_off
    initial begin
        if (FRAME_WIDTH < 4 || FRAME_WIDTH > 65536 || FRAME_WIDTH % 4 != 0 ||
            FRAME_HEIGHT < 1 || FRAME_HEIGHT > 16384 ||
            AE_X_START < 0 || AE_X_END > FRAME_WIDTH || AE_X_START >= AE_X_END ||
            AE_Y_START < 0 || AE_Y_END > FRAME_HEIGHT || AE_Y_START >= AE_Y_END ||
            COUNT_WIDTH < 3 || COUNT_WIDTH < $clog2(FRAME_WIDTH*FRAME_HEIGHT+1) ||
            SUM_WIDTH < COUNT_WIDTH+10) begin
            $error("Invalid sc500cs_ae_stats geometry or accumulator width");
            $finish;
        end
    end
    // synthesis translate_on
endmodule
