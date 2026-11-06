`timescale 1ns/1ps
// RAW-domain only. Passive frame/result observer; no video ready or data ports.
// Stats buses remain unchanged: gate their valid with O_controller_result_valid.
// Physical SOF counts effect delay; valid tuser identifies the first RAW pixel.
module sc500cs_ae_effect_manager #(
    parameter integer STARTUP_GOOD_FRAMES=2
)(
    input wire I_clk, I_rst_n,
    input wire I_physical_frame_start, I_first_valid_pixel,
    input wire I_stats_result_valid, I_stats_frame_good,
    input wire I_update_request,
    input wire [23:0] I_requested_exposure,
    input wire [6:0] I_requested_gain,
    input wire I_cdc_ready,
    output wire O_cdc_request,
    output wire [23:0] O_cdc_exposure,
    output wire [6:0] O_cdc_gain,
    input wire I_response_valid,
    input wire [1:0] I_response_kind,
    input wire [23:0] I_response_exposure, I_response_exposure_max,
    input wire [6:0] I_response_gain,
    input wire [7:0] I_response_error,
    output wire O_request_ready,
    output reg O_request_accepted, O_request_rejected,
    output reg [23:0] O_requested_exposure, O_pending_exposure, O_written_exposure,
    output reg [6:0] O_requested_gain, O_pending_gain, O_written_gain,
    output reg O_written_valid,
    output reg [23:0] O_current_exposure, O_exposure_max,
    output reg [6:0] O_current_gain,
    output reg O_current_valid,
    output wire O_update_in_flight, O_controller_busy, O_controller_enable,
    output wire O_measurement_enable, O_controller_result_valid,
    output reg O_effective_commit, O_error,
    output reg [7:0] O_error_code,
    output wire [3:0] O_debug_state,
    output wire [1:0] O_wait_frame_count
);
    localparam [3:0] STARTUP=0, WRITE_WAIT=1, WAIT_EFFECT=2, ARM=3,
                     QUALIFY=4, START_ARM=5, ACTIVE=6, ERROR=7;
    localparam integer QW=(STARTUP_GOOD_FRAMES<2)?1:$clog2(STARTUP_GOOD_FRAMES+1);
    localparam [QW-1:0] QUALIFY_LAST=STARTUP_GOOD_FRAMES-1;
    reg [3:0] state;
    reg [1:0] wait_count;
    reg [QW-1:0] good_count;
    reg startup_wait, request_previous, update_pending;
    reg physical_seen, pixel_seen, frame_eligible, probe_open, probe_good, result_seen;
    wire request_edge=I_update_request && !request_previous;
    wire response_failure=I_response_valid && I_response_kind==2'd2;
    assign O_debug_state=state;
    assign O_wait_frame_count=wait_count;
    assign O_measurement_enable=(state==ACTIVE) && frame_eligible && O_current_valid && !O_error;
    assign O_controller_busy=(state!=ACTIVE) || !O_current_valid || !frame_eligible;
    // Keep the transaction busy through the gap between the commit SOF and
    // the first valid RAW pixel. ERROR terminates it; controller_busy stays high.
    assign O_update_in_flight=update_pending;
    assign O_controller_enable=!O_controller_busy && !O_error;
    assign O_request_ready=O_controller_enable && I_cdc_ready && !I_response_valid;
    wire take_request=request_edge && O_request_ready;
    // Local RAW-domain combinational handoff; CDC captures on the same edge.
    assign O_cdc_request=take_request;
    assign O_cdc_exposure=I_requested_exposure;
    assign O_cdc_gain=I_requested_gain;
    assign O_controller_result_valid=O_measurement_enable && I_stats_result_valid &&
        I_stats_frame_good && !result_seen && !I_physical_frame_start && !I_first_valid_pixel &&
        !take_request && !I_response_valid;
    task enter_error;
        input [7:0] code;
        begin
            state<=ERROR;O_error<=1;O_error_code<=code;O_current_valid<=0;
            O_written_valid<=0;frame_eligible<=0;probe_open<=0;update_pending<=0;
        end
    endtask
    task commit_current;
        begin
            O_current_exposure<=O_pending_exposure;O_current_gain<=O_pending_gain;
            O_current_valid<=1;O_effective_commit<=1;state<=ACTIVE;
        end
    endtask
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            state<=STARTUP;wait_count<=0;good_count<=0;startup_wait<=1;request_previous<=0;update_pending<=0;
            physical_seen<=0;pixel_seen<=0;frame_eligible<=0;probe_open<=0;probe_good<=0;result_seen<=0;
            O_request_accepted<=0;O_request_rejected<=0;
            O_requested_exposure<=0;O_pending_exposure<=0;O_written_exposure<=0;
            O_requested_gain<=0;O_pending_gain<=0;O_written_gain<=0;O_written_valid<=0;
            O_current_exposure<=0;O_current_gain<=0;O_current_valid<=0;O_exposure_max<=0;
            O_effective_commit<=0;O_error<=0;O_error_code<=0;
        end else begin
            request_previous<=I_update_request;
            O_request_accepted<=0;O_request_rejected<=0;O_effective_commit<=0;
            if(request_edge && !O_request_ready)O_request_rejected<=1;
            if(state!=ERROR)begin
                if(I_physical_frame_start)begin
                    physical_seen<=1;pixel_seen<=0;frame_eligible<=0;result_seen<=0;
                    probe_open<=0;probe_good<=0;
                    if(state==QUALIFY && !probe_good)good_count<=0;
                    if(state==WAIT_EFFECT)begin
                        if(wait_count==2)begin wait_count<=3;state<=ARM;end
                        else wait_count<=wait_count+1'b1;
                    end
                    if(state==ARM)begin
                        if(startup_wait)begin state<=QUALIFY;good_count<=0;end
                        else commit_current;
                    end
                    if(state==START_ARM)commit_current;
                end
                if(I_first_valid_pixel && (physical_seen || I_physical_frame_start))begin
                    if(!pixel_seen || I_physical_frame_start)begin
                        pixel_seen<=1;
                        // ARM/START_ARM still belong to the protected frame
                        // until a NEW physical SOF arrives. A late first pixel
                        // must not pre-arm eligibility or startup qualification.
                        if(state==ACTIVE || (I_physical_frame_start &&
                           (state==START_ARM || (state==ARM && !startup_wait))))
                            frame_eligible<=1;
                        if(state==ACTIVE || (state==ARM && !startup_wait && I_physical_frame_start))
                            update_pending<=0;
                        if(state==QUALIFY || (state==ARM && startup_wait && I_physical_frame_start))probe_open<=1;
                    end else begin
                        // Duplicate first-pixel marker invalidates this frame.
                        frame_eligible<=0;probe_open<=0;good_count<=0;probe_good<=0;
                    end
                end
                if(I_stats_result_valid && !I_physical_frame_start && !I_first_valid_pixel)begin
                    result_seen<=1;
                    if(state==QUALIFY && probe_open)begin
                        probe_open<=0;probe_good<=I_stats_frame_good;
                        if(I_stats_frame_good)begin
                            if(good_count==QUALIFY_LAST)state<=START_ARM;
                            else good_count<=good_count+1'b1;
                        end else good_count<=0;
                    end
                end
                if(take_request)begin
                    O_request_accepted<=1;O_requested_exposure<=I_requested_exposure;
                    O_requested_gain<=I_requested_gain;O_pending_exposure<=I_requested_exposure;
                    O_pending_gain<=I_requested_gain;O_written_valid<=0;
                    state<=WRITE_WAIT;frame_eligible<=0;physical_seen<=0;update_pending<=1;
                end
                // Response wins over coincident SOF: that SOF is NEVER count1.
                if(I_response_valid)begin
                    if(response_failure)enter_error(I_response_error);
                    else if((I_response_kind==0 && state==STARTUP) ||
                            (I_response_kind==1 && state==WRITE_WAIT))begin
                        if(I_response_exposure<24'd3 || I_response_exposure>I_response_exposure_max ||
                           (I_response_kind==1 && (I_response_exposure!=O_requested_exposure ||
                                                  I_response_gain!=O_requested_gain)))enter_error(8'hf1);
                        else begin
                            O_written_exposure<=I_response_exposure;O_written_gain<=I_response_gain;
                            O_written_valid<=1;O_pending_exposure<=I_response_exposure;
                            O_pending_gain<=I_response_gain;O_exposure_max<=I_response_exposure_max;
                            startup_wait<=(I_response_kind==0);state<=WAIT_EFFECT;wait_count<=0;
                            frame_eligible<=0;physical_seen<=0;pixel_seen<=0;probe_open<=0;probe_good<=0;good_count<=0;
                        end
                    end else enter_error(8'hf2);
                end
            end
        end
    end
    // synthesis translate_off
    initial if(STARTUP_GOOD_FRAMES<1)$fatal(1,"At least one startup good frame required");
    // synthesis translate_on
endmodule
