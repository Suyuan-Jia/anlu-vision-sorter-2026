`timescale 1ns/1ps
// Bundled-data mailboxes. Only the toggles enter synchronizer chains.
// Snapshot data is sampled AFTER two synchronizer stages plus a settle cycle,
// and held at its source until the corresponding return/ack is received.
// Either reset flushes BOTH mailboxes. O_control_rst_n MUST also reset the
// runtime manager/master; sensor initialization must then be performed again.
module sc500cs_ae_cdc #(
    parameter integer RESPONSE_TIMEOUT_CYCLES = 3000000
)(
    input wire I_raw_clk, I_raw_rst_n, I_control_clk, I_control_rst_n,
    output wire O_raw_reset_n, O_control_reset_n,
    input wire I_request,
    input wire [23:0] I_requested_exposure,
    input wire [6:0] I_requested_gain,
    output wire O_request_ready,
    output reg O_request_accepted, O_request_rejected,
    output reg O_response_valid,
    // 0=startup verified snapshot, 1=verified update, 2=fatal error.
    output reg [1:0] O_response_kind,
    output reg [23:0] O_response_exposure, O_response_exposure_max,
    output reg [6:0] O_response_gain,
    output reg [7:0] O_response_error,
    input wire I_runtime_ready, I_snapshot_valid, I_register_state_valid,
    input wire [23:0] I_snapshot_exposure, I_exposure_max,
    input wire [6:0] I_snapshot_gain,
    input wire I_runtime_accepted, I_runtime_rejected,
    input wire I_write_success, I_write_error, I_state_unknown,
    input wire [23:0] I_written_exposure,
    input wire [6:0] I_written_gain,
    input wire [7:0] I_error_code,
    output reg O_runtime_request,
    output reg [23:0] O_runtime_exposure,
    output reg [7:0] O_runtime_gain,
    output wire [3:0] O_debug_control_state
);
    wire combined_reset_n=I_raw_rst_n && I_control_rst_n;
    (* ASYNC_REG="TRUE" *) reg [1:0] raw_release, control_release;
    always @(posedge I_raw_clk or negedge combined_reset_n)
        if(!combined_reset_n)raw_release<=0;
        else raw_release<={raw_release[0],1'b1};
    always @(posedge I_control_clk or negedge combined_reset_n)
        if(!combined_reset_n)control_release<=0;
        else control_release<={control_release[0],1'b1};
    assign O_raw_reset_n=raw_release[1];
    assign O_control_reset_n=control_release[1];

    reg [30:0] request_snapshot;
    reg request_toggle, raw_busy, raw_started, raw_failed, request_previous;
    reg response_ack, response_seen, response_capture;
    // 2 kind + 24 exposure + 7 gain + 24 max + 8 error = 65 bits.
    reg [64:0] response_snapshot;
    reg response_toggle;
    (* ASYNC_REG="TRUE" *) reg [1:0] response_sync;
    (* ASYNC_REG="TRUE" *) reg [1:0] request_sync, ack_sync;
    always @(posedge I_raw_clk or negedge O_raw_reset_n)begin
        if(!O_raw_reset_n)response_sync<=0;
        else response_sync<={response_sync[0],response_toggle};
    end
    always @(posedge I_control_clk or negedge O_control_reset_n)begin
        if(!O_control_reset_n)begin request_sync<=0;ack_sync<=0;end
        else begin
            request_sync<={request_sync[0],request_toggle};
            ack_sync<={ack_sync[0],response_ack};
        end
    end
    assign O_request_ready=O_raw_reset_n && raw_started && !raw_busy && !raw_failed;
    always @(posedge I_raw_clk or negedge O_raw_reset_n)begin
        if(!O_raw_reset_n)begin
            request_snapshot<=0;request_toggle<=0;raw_busy<=0;raw_started<=0;raw_failed<=0;
            request_previous<=0;response_ack<=0;response_seen<=0;response_capture<=0;
            O_request_accepted<=0;O_request_rejected<=0;O_response_valid<=0;
            O_response_kind<=0;O_response_exposure<=0;O_response_gain<=0;
            O_response_exposure_max<=0;O_response_error<=0;
        end else begin
            request_previous<=I_request;
            O_request_accepted<=0;O_request_rejected<=0;O_response_valid<=0;
            if(I_request && !request_previous)begin
                if(O_request_ready)begin
                    request_snapshot<={I_requested_exposure,I_requested_gain};
                    request_toggle<=!request_toggle;raw_busy<=1;O_request_accepted<=1;
                end else O_request_rejected<=1;
            end
            if(!response_capture && response_sync[1]!=response_seen)response_capture<=1;
            if(response_capture)begin
                // Source holds all 65 bits until this ACK crosses back.
                {O_response_kind,O_response_exposure,O_response_gain,
                 O_response_exposure_max,O_response_error}<=response_snapshot;
                O_response_valid<=1;response_seen<=response_sync[1];
                response_ack<=response_sync[1];response_capture<=0;
                if(response_snapshot[64:63]==2'd0)raw_started<=1;
                if(response_snapshot[64:63]==2'd1)raw_busy<=0;
                if(response_snapshot[64:63]==2'd2)begin raw_failed<=1;raw_busy<=0;end
            end
        end
    end

    localparam [3:0] BOOT=0, IDLE=1, CAPTURE=2, ISSUE=3,
                     ACCEPT=4, RESULT=5, RETURN_ACK=6, HALT=7;
    localparam integer CW=(RESPONSE_TIMEOUT_CYCLES<2)?1:$clog2(RESPONSE_TIMEOUT_CYCLES);
    localparam [CW-1:0] TIMEOUT_LAST=RESPONSE_TIMEOUT_CYCLES-1;
    reg [3:0] state;
    reg [CW-1:0] timer;
    reg request_seen, startup_sent, error_pending, error_sent;
    reg [7:0] saved_error;
    wire mailbox_free=(response_toggle==ack_sync[1]);
    wire timed_state=(state==CAPTURE || state==ISSUE || state==ACCEPT || state==RESULT);
    assign O_debug_control_state=state;
    task publish;
        input [1:0] kind;
        input [23:0] exposure;
        input [6:0] gain;
        input [7:0] code;
        begin
            response_snapshot<={kind,exposure,gain,I_exposure_max,code};
            response_toggle<=!response_toggle;state<=RETURN_ACK;
        end
    endtask
    always @(posedge I_control_clk or negedge O_control_reset_n)begin
        if(!O_control_reset_n)begin
            state<=BOOT;timer<=0;request_seen<=0;startup_sent<=0;
            error_pending<=0;error_sent<=0;saved_error<=0;
            response_snapshot<=0;response_toggle<=0;
            O_runtime_request<=0;O_runtime_exposure<=0;O_runtime_gain<=0;
        end else begin
            O_runtime_request<=0;
            if(timed_state)timer<=timer+1'b1;else timer<=0;
            if((I_write_error || I_state_unknown || I_runtime_rejected) && !error_sent)begin
                error_pending<=1;saved_error<=(I_error_code==0)?8'he1:I_error_code;
            end
            if(timed_state && timer==TIMEOUT_LAST && !error_sent)begin
                error_pending<=1;saved_error<=8'he2;
            end
            // Fatal notifications also work before READY or between requests.
            if(error_pending && !error_sent && mailbox_free)begin
                publish(2'd2,24'd0,7'd0,saved_error);error_sent<=1;
            end else if(!error_pending && !(I_write_error || I_state_unknown || I_runtime_rejected) &&
                        !(timed_state && timer==TIMEOUT_LAST))begin
                case(state)
                    BOOT:if(I_runtime_ready && I_snapshot_valid && I_register_state_valid)begin
                        publish(2'd0,I_snapshot_exposure,I_snapshot_gain,8'd0);startup_sent<=1;
                    end
                    IDLE:if(request_sync[1]!=request_seen)state<=CAPTURE;
                    CAPTURE:begin
                        {O_runtime_exposure,O_runtime_gain}<={request_snapshot[30:7],1'b0,request_snapshot[6:0]};
                        request_seen<=request_sync[1];state<=ISSUE;
                    end
                    ISSUE:if(I_runtime_ready)begin O_runtime_request<=1;state<=ACCEPT;end
                    ACCEPT:if(I_runtime_accepted)state<=RESULT;
                    RESULT:if(I_write_success)begin
                        publish(2'd1,I_written_exposure,I_written_gain,8'd0);
                    end
                    RETURN_ACK:if(mailbox_free)state<=error_sent?HALT:(startup_sent?IDLE:BOOT);
                    HALT:state<=HALT;
                    default:begin error_pending<=1;saved_error<=8'he3;end
                endcase
            end else if(error_sent && state==RETURN_ACK && mailbox_free)state<=HALT;
        end
    end
    // synthesis translate_off
    initial if(RESPONSE_TIMEOUT_CYCLES<2)$fatal(1,"Invalid CDC timeout");
    // synthesis translate_on
endmodule
