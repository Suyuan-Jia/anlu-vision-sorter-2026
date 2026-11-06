`timescale 1ns / 1ps

// STEP 3B: standalone decision logic, all ports synchronous to I_clk.
// CURRENT is exclusively owned by the future external state manager.
// REQUESTED is a proposal, NOT proof of a write or of sensor effectiveness.
// No I2C, register writer, CDC, frame-delay manager, or digital gain control.
// All brightness/step/ratio defaults below are engineering tuning values.
module sc500cs_ae_controller #(
    parameter integer COUNT_WIDTH = 20,
    parameter integer SUM_WIDTH = COUNT_WIDTH + 10,
    // RAW10 mean target for both exposure and analog gain control.
    // Brighter target: 640..768 (previously 512..640).
    parameter [9:0] AE_LOW_THRESHOLD = 10'd640,
    parameter [9:0] AE_HIGH_THRESHOLD = 10'd768,
    parameter [9:0] AE_DARK_SEVERE_THRESHOLD = 10'd320,
    parameter [9:0] AE_BRIGHT_SEVERE_THRESHOLD = 10'd896,
    parameter [23:0] EXPOSURE_STEP_SMALL = 24'd16,
    parameter [23:0] EXPOSURE_STEP_LARGE = 24'd128,
    parameter integer GAIN_STEP = 1,
    parameter [15:0] SAT_RATIO_NUM = 16'd1,
    parameter [15:0] SAT_RATIO_DEN = 16'd100,
    parameter integer AE_CONFIRM_FRAMES = 2,
    parameter integer CONFIRM_WIDTH = (AE_CONFIRM_FRAMES < 2) ? 1 :
                                     $clog2(AE_CONFIRM_FRAMES + 1)
)(
    input wire I_clk,
    input wire I_rst_n,
    input wire I_controller_enable,
    input wire I_update_busy,
    input wire I_ae_result_valid,
    input wire I_ae_frame_good,
    input wire [SUM_WIDTH-1:0] I_ae_raw_sum,
    input wire [COUNT_WIDTH-1:0] I_ae_pixel_count,
    input wire [COUNT_WIDTH-1:0] I_ae_saturated_count,
    input wire [23:0] I_current_exposure,
    input wire [23:0] I_exposure_min,
    input wire [23:0] I_exposure_max,
    input wire [6:0] I_current_gain_index,
    output reg O_update_request,
    output reg [23:0] O_requested_exposure,
    output reg [6:0] O_requested_gain_index,
    // Register BYTES for a future writer; these outputs never write hardware.
    output reg [7:0] O_requested_analog_gain,
    output wire [7:0] O_requested_analog_fine_gain,
    output reg O_ae_too_dark,
    output reg O_ae_good,
    output reg O_ae_too_bright,
    output reg O_highlight_saturated,
    output reg [CONFIRM_WIDTH-1:0] O_dark_confirm_count,
    output reg [CONFIRM_WIDTH-1:0] O_bright_confirm_count,
    output reg [SUM_WIDTH-1:0] O_last_raw_sum,
    output reg [COUNT_WIDTH-1:0] O_last_pixel_count
);
    // Products are explicitly widened BEFORE multiplication. For the defaults,
    // 20-bit count * 10-bit threshold fits 30 bits, and ratio products fit 36.
    // These are constant multiplications: synthesis is checked separately.
    localparam integer RATIO_WIDTH = COUNT_WIDTH + 16;
    localparam [CONFIRM_WIDTH-1:0] CONFIRM_LAST = AE_CONFIRM_FRAMES - 1;
    localparam [7:0] GAIN_STEP_EXT = GAIN_STEP;
    wire [SUM_WIDTH-1:0] count_wide =
        {{(SUM_WIDTH-COUNT_WIDTH){1'b0}}, I_ae_pixel_count};
    wire [SUM_WIDTH-1:0] low_product = count_wide *
        {{(SUM_WIDTH-10){1'b0}}, AE_LOW_THRESHOLD};
    wire [SUM_WIDTH-1:0] high_product = count_wide *
        {{(SUM_WIDTH-10){1'b0}}, AE_HIGH_THRESHOLD};
    wire [SUM_WIDTH-1:0] dark_severe_product = count_wide *
        {{(SUM_WIDTH-10){1'b0}}, AE_DARK_SEVERE_THRESHOLD};
    wire [SUM_WIDTH-1:0] bright_severe_product = count_wide *
        {{(SUM_WIDTH-10){1'b0}}, AE_BRIGHT_SEVERE_THRESHOLD};
    wire [SUM_WIDTH-1:0] max_sum_product = count_wide *
        {{(SUM_WIDTH-10){1'b0}}, 10'd1023};
    wire [RATIO_WIDTH-1:0] saturated_product =
        {{16{1'b0}}, I_ae_saturated_count} *
        {{COUNT_WIDTH{1'b0}}, SAT_RATIO_DEN};
    wire [RATIO_WIDTH-1:0] allowed_product =
        {{16{1'b0}}, I_ae_pixel_count} *
        {{COUNT_WIDTH{1'b0}}, SAT_RATIO_NUM};

    wire highlight = saturated_product > allowed_product;
    wire too_bright = highlight || (I_ae_raw_sum > high_product);
    wire too_dark = !highlight && (I_ae_raw_sum < low_product);
    wire severe = (too_dark && I_ae_raw_sum < dark_severe_product) ||
                  (too_bright && I_ae_raw_sum > bright_severe_product);
    // A highlight-only decision uses the small exposure step unless the mean
    // is also severely bright. It still passes the normal frame confirmation.
    wire [23:0] exposure_step = severe ? EXPOSURE_STEP_LARGE : EXPOSURE_STEP_SMALL;
    wire [24:0] exposure_up = {1'b0,I_current_exposure} + {1'b0,exposure_step};
    wire [24:0] exposure_room_down = {1'b0,I_current_exposure} - {1'b0,I_exposure_min};
    wire [7:0] gain_up = {1'b0,I_current_gain_index} + GAIN_STEP_EXT;

    wire input_ok = (I_ae_pixel_count != {COUNT_WIDTH{1'b0}}) &&
                    (I_ae_saturated_count <= I_ae_pixel_count) &&
                    (I_ae_raw_sum <= max_sum_product) &&
                    (I_exposure_min <= I_exposure_max) &&
                    (I_current_exposure >= I_exposure_min) &&
                    (I_current_exposure <= I_exposure_max);
    wire accept_result = I_controller_enable && !I_update_busy &&
                         I_ae_result_valid && I_ae_frame_good && input_ok;

    reg [23:0] next_exposure;
    reg [6:0] next_gain;
    always @(*) begin
        next_exposure = I_current_exposure;
        next_gain = I_current_gain_index;
        if (too_dark) begin
            if (I_current_exposure < I_exposure_max)
                next_exposure = (exposure_up > {1'b0,I_exposure_max}) ?
                                I_exposure_max : exposure_up[23:0];
            else if (I_current_gain_index < 7'd127)
                next_gain = (gain_up > 8'd127) ? 7'd127 : gain_up[6:0];
        end else if (too_bright) begin
            if (I_current_gain_index != 7'd0)
                next_gain = ({1'b0,I_current_gain_index} < GAIN_STEP_EXT) ?
                            7'd0 : I_current_gain_index - GAIN_STEP_EXT[6:0];
            else
                next_exposure = (exposure_room_down < {1'b0,exposure_step}) ?
                                I_exposure_min : I_current_exposure - exposure_step;
        end
    end

    // SC500CS Table 2-4: 128 entries represented by four exact segments.
    // This is an index-to-register mapping, not a linear gain-register model.
    always @(*) begin
        case (O_requested_gain_index[6:5])
            2'd0: O_requested_analog_gain = 8'h03;
            2'd1: O_requested_analog_gain = 8'h07;
            2'd2: O_requested_analog_gain = 8'h0f;
            2'd3: O_requested_analog_gain = 8'h1f;
            default: O_requested_analog_gain = 8'h03;
        endcase
    end
    assign O_requested_analog_fine_gain =
        8'h20 + {3'b000,O_requested_gain_index[4:0]};

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_update_request <= 1'b0;
            O_requested_exposure <= 24'd0; // no valid proposal until request
            O_requested_gain_index <= 7'd0;
            O_ae_too_dark <= 1'b0;
            O_ae_good <= 1'b0;
            O_ae_too_bright <= 1'b0;
            O_highlight_saturated <= 1'b0;
            O_dark_confirm_count <= {CONFIRM_WIDTH{1'b0}};
            O_bright_confirm_count <= {CONFIRM_WIDTH{1'b0}};
            O_last_raw_sum <= {SUM_WIDTH{1'b0}};
            O_last_pixel_count <= {COUNT_WIDTH{1'b0}};
        end else begin
            O_update_request <= 1'b0;
            // Invalid/disabled/busy results freeze all persistent AE state.
            // Debug classifications describe the LAST ACCEPTED result.
            // "Consecutive" means consecutive ACCEPTED good-frame results;
            // invalid frames cannot increment, reset, or reverse confirmation.
            if (accept_result) begin
                O_last_raw_sum <= I_ae_raw_sum;
                O_last_pixel_count <= I_ae_pixel_count;
                O_ae_too_dark <= too_dark;
                O_ae_too_bright <= too_bright;
                O_ae_good <= !too_dark && !too_bright;
                O_highlight_saturated <= highlight;
                if (too_dark) begin
                    O_bright_confirm_count <= {CONFIRM_WIDTH{1'b0}};
                    if (O_dark_confirm_count >= CONFIRM_LAST) begin
                        O_dark_confirm_count <= {CONFIRM_WIDTH{1'b0}};
                        if (next_exposure != I_current_exposure || next_gain != I_current_gain_index) begin
                            O_requested_exposure <= next_exposure;
                            O_requested_gain_index <= next_gain;
                            O_update_request <= 1'b1;
                        end
                    end else
                        O_dark_confirm_count <= O_dark_confirm_count + {{(CONFIRM_WIDTH-1){1'b0}},1'b1};
                end else if (too_bright) begin
                    O_dark_confirm_count <= {CONFIRM_WIDTH{1'b0}};
                    if (O_bright_confirm_count >= CONFIRM_LAST) begin
                        O_bright_confirm_count <= {CONFIRM_WIDTH{1'b0}};
                        if (next_exposure != I_current_exposure || next_gain != I_current_gain_index) begin
                            O_requested_exposure <= next_exposure;
                            O_requested_gain_index <= next_gain;
                            O_update_request <= 1'b1;
                        end
                    end else
                        O_bright_confirm_count <= O_bright_confirm_count + {{(CONFIRM_WIDTH-1){1'b0}},1'b1};
                end else begin
                    O_dark_confirm_count <= {CONFIRM_WIDTH{1'b0}};
                    O_bright_confirm_count <= {CONFIRM_WIDTH{1'b0}};
                end
            end
        end
    end

    // synthesis translate_off
    initial begin
        if (COUNT_WIDTH < 1 || SUM_WIDTH < COUNT_WIDTH+10 ||
            AE_LOW_THRESHOLD >= AE_HIGH_THRESHOLD ||
            AE_DARK_SEVERE_THRESHOLD >= AE_LOW_THRESHOLD ||
            AE_BRIGHT_SEVERE_THRESHOLD <= AE_HIGH_THRESHOLD ||
            EXPOSURE_STEP_SMALL == 0 || EXPOSURE_STEP_LARGE < EXPOSURE_STEP_SMALL ||
            GAIN_STEP < 1 || GAIN_STEP > 127 ||
            SAT_RATIO_DEN == 0 || SAT_RATIO_NUM > SAT_RATIO_DEN ||
            AE_CONFIRM_FRAMES < 1 || CONFIRM_WIDTH < 1 ||
            CONFIRM_WIDTH < $clog2(AE_CONFIRM_FRAMES+1)) begin
            $error("Invalid sc500cs_ae_controller parameters");
            $finish;
        end
    end
    // synthesis translate_on
endmodule
