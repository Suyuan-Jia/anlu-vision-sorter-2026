`timescale 1ns / 1ps
// STEP 4A: standalone register manager, NOT integrated into the board project.
// Connect this transaction interface to the EXISTING sole uii2c master after
// initialization has released ownership. No second master is instantiated here.
// All ports are in that master's I_clk domain. No RAW-domain CDC/effect timer.
// READY means register preparation verified, NOT optical AE/effect readiness.
module sc500cs_runtime_manager #(
    parameter integer TRANSACTION_TIMEOUT_CYCLES = 240000,
    parameter integer BATCH_TIMEOUT_CYCLES = 2400000
)(
    input wire I_clk, I_rst_n, I_init_done,
    input wire I_update_req,
    input wire [23:0] I_requested_exposure,
    // Intentionally eight bits: values 128..255 can be explicitly rejected.
    // A future seven-bit controller output connects as {1'b0,gain_index}.
    input wire [7:0] I_requested_gain_index,
    input wire I_iic_busy, I_iic_bus_error,
    input wire [7:0] I_iic_rd_data,
    output reg O_iic_req,
    output reg [31:0] O_iic_wr_data,
    output reg [7:0] O_iic_wr_cnt, O_iic_rd_cnt,
    output reg O_iic_mode,
    output wire O_runtime_ready, O_update_busy,
    output reg O_request_accepted, O_request_rejected,
    output reg [3:0] O_reject_reason,
    output reg O_snapshot_valid,
    output wire [95:0] O_snapshot_raw,
    output reg [22:0] O_vts,
    output wire [23:0] O_exposure_min,
    output reg [23:0] O_exposure_max, O_snapshot_exposure,
    output reg [6:0] O_snapshot_gain_index,
    output reg O_register_state_valid,
    output reg [23:0] O_written_exposure,
    output reg [6:0] O_written_gain_index,
    output reg O_written_valid, O_write_success, O_write_error,
    output reg O_error, O_state_unknown,
    output reg [7:0] O_error_code,
    output reg [15:0] O_error_register,
    output reg O_transaction_error_seen,
    output wire [3:0] O_debug_state,
    output wire [2:0] O_debug_phase,
    output wire [3:0] O_debug_index
);
    localparam [3:0] WAIT_INIT=0, ISSUE=1, WAIT_BUSY=2, WAIT_DONE=3,
                     CHECK=4, VALIDATE=5, READY=6, STATE_UNKNOWN=7;
    localparam [2:0] SNAPSHOT=0, PREP_WRITE=1, PREP_VERIFY=2,
                     UPDATE_WRITE=3, UPDATE_VERIFY=4;
    // Error codes: 1=NACK, 2=transaction timeout, 3=batch timeout,
    // 4=invalid VTS, 5=invalid snapshot exposure, 6=invalid snapshot gain,
    // 7=verify mismatch, 8=invalid requested exposure, 9=invalid requested gain,
    // 10=lost initialization/ownership, 11=illegal internal state.
    localparam integer TW=(TRANSACTION_TIMEOUT_CYCLES<2)?1:$clog2(TRANSACTION_TIMEOUT_CYCLES);
    localparam integer BW=(BATCH_TIMEOUT_CYCLES<2)?1:$clog2(BATCH_TIMEOUT_CYCLES);
    localparam [TW-1:0] TX_LAST=TRANSACTION_TIMEOUT_CYCLES-1;
    localparam [BW-1:0] BATCH_LAST=BATCH_TIMEOUT_CYCLES-1;
    reg [3:0] state, index;
    reg [2:0] phase;
    reg [TW-1:0] tx_timer;
    reg [BW-1:0] batch_timer;
    reg req_previous;
    reg [7:0] snapshot[0:11];
    reg [23:0] requested_exposure;
    reg [6:0] requested_gain;
    reg [7:0] gain_coarse;
    wire [7:0] gain_fine=8'h20+{3'b000,requested_gain[4:0]};
    reg [7:0] mode_value;
    reg [3:0] exposure_high_reserved, exposure_low_reserved;
    reg [15:0] transaction_address;
    reg [7:0] transaction_expected;
    wire update_edge=I_update_req && !req_previous;
    wire [22:0] decoded_vts={snapshot[0][6:0],snapshot[1],snapshot[2]};
    wire [24:0] decoded_max={1'b0,decoded_vts,1'b0}-25'd10;
    wire [23:0] decoded_exposure={snapshot[3][3:0],snapshot[4],snapshot[5],snapshot[6][7:4]};
    reg gain_legal;
    reg [6:0] decoded_gain;
    integer r;
    genvar g;
    generate for(g=0;g<12;g=g+1)begin: snapshot_outputs
        assign O_snapshot_raw[g*8+:8]=snapshot[g];
    end endgenerate
    assign O_runtime_ready=(state==READY);
    assign O_update_busy=(state!=WAIT_INIT && state!=READY && state!=STATE_UNKNOWN);
    assign O_exposure_min=24'd3;
    assign O_debug_state=state;
    assign O_debug_phase=phase;
    assign O_debug_index=index;

    always @(*) begin
        case(requested_gain[6:5])
            0:gain_coarse=8'h03;1:gain_coarse=8'h07;
            2:gain_coarse=8'h0f;3:gain_coarse=8'h1f;
            default:gain_coarse=8'h03;
        endcase
        decoded_gain=7'd0;
        gain_legal=(snapshot[8]>=8'h20 && snapshot[8]<=8'h3f);
        case(snapshot[7])
            8'h03:decoded_gain={2'd0,snapshot[8][4:0]};
            8'h07:decoded_gain={2'd1,snapshot[8][4:0]};
            8'h0f:decoded_gain={2'd2,snapshot[8][4:0]};
            8'h1f:decoded_gain={2'd3,snapshot[8][4:0]};
            default:gain_legal=1'b0;
        endcase
    end

    function [15:0] snapshot_address;
        input [3:0] n;
        begin case(n)
            0:snapshot_address=16'h326d;1:snapshot_address=16'h320e;
            2:snapshot_address=16'h320f;3:snapshot_address=16'h3e20;
            4:snapshot_address=16'h3e00;5:snapshot_address=16'h3e01;
            6:snapshot_address=16'h3e02;7:snapshot_address=16'h3e08;
            8:snapshot_address=16'h3e09;9:snapshot_address=16'h3e03;
            10:snapshot_address=16'h3e06;11:snapshot_address=16'h3e07;
            default:snapshot_address=16'h0000;
        endcase end
    endfunction

    reg [15:0] command_address;
    reg [7:0] command_value;
    reg command_read;
    always @(*) begin
        command_address=16'd0;command_value=8'd0;
        command_read=(phase==SNAPSHOT || phase==PREP_VERIFY || phase==UPDATE_VERIFY);
        if(phase==SNAPSHOT)command_address=snapshot_address(index);
        else if(phase==PREP_WRITE || phase==PREP_VERIFY)begin
            case(index)
                0:begin command_address=16'h3e03;command_value=mode_value;end
                1:begin command_address=16'h3e06;command_value=8'h00;end
                2:begin command_address=16'h3e07;command_value=8'h80;end
                default:begin command_address=0;command_value=0;end
            endcase
        end else begin
            case(index)
                0:begin command_address=16'h3e20;command_value={exposure_high_reserved,requested_exposure[23:20]};end
                1:begin command_address=16'h3e00;command_value=requested_exposure[19:12];end
                2:begin command_address=16'h3e01;command_value=requested_exposure[11:4];end
                3:begin command_address=16'h3e02;command_value={requested_exposure[3:0],exposure_low_reserved};end
                4:begin command_address=16'h3e08;command_value=gain_coarse;end
                5:begin command_address=16'h3e09;command_value=gain_fine;end
                6:begin command_address=16'h3e03;command_value=mode_value;end
                7:begin command_address=16'h3e06;command_value=8'h00;end
                8:begin command_address=16'h3e07;command_value=8'h80;end
                default:begin command_address=0;command_value=0;end
            endcase
        end
    end

    // Fatal errors require an external reset AND sensor reinitialization before
    // recovery. Do not rollback, retry, or enqueue a second transaction here.
    // Keep transaction data/mode stable so an in-flight master can finish STOP.
    task fail;
        input [7:0] code;
        begin
            state<=STATE_UNKNOWN;O_iic_req<=1'b0;
            O_error<=1'b1;O_error_code<=code;O_error_register<=transaction_address;
            O_state_unknown<=1'b1;O_register_state_valid<=1'b0;
            O_snapshot_valid<=1'b0;O_written_valid<=1'b0;O_write_error<=1'b1;
        end
    endtask

    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            state<=WAIT_INIT;phase<=SNAPSHOT;index<=0;tx_timer<=0;batch_timer<=0;
            req_previous<=0;requested_exposure<=0;requested_gain<=0;
            mode_value<=0;exposure_high_reserved<=0;exposure_low_reserved<=0;
            transaction_address<=0;transaction_expected<=0;
            O_iic_req<=0;O_iic_wr_data<=0;O_iic_wr_cnt<=0;O_iic_rd_cnt<=0;O_iic_mode<=0;
            O_request_accepted<=0;O_request_rejected<=0;O_reject_reason<=0;
            O_snapshot_valid<=0;O_vts<=0;O_exposure_max<=0;O_snapshot_exposure<=0;O_snapshot_gain_index<=0;
            O_register_state_valid<=0;O_written_exposure<=0;O_written_gain_index<=0;O_written_valid<=0;
            O_write_success<=0;O_write_error<=0;O_error<=0;O_state_unknown<=0;
            O_error_code<=0;O_error_register<=0;O_transaction_error_seen<=0;
            for(r=0;r<12;r=r+1)snapshot[r]<=0;
        end else begin
            req_previous<=I_update_req;
            O_request_accepted<=0;O_request_rejected<=0;O_write_success<=0;O_write_error<=0;
            if(update_edge && state!=READY)begin
                O_request_rejected<=1;O_reject_reason<=O_update_busy?4'd1:4'd4;
            end
            if(O_update_busy)batch_timer<=batch_timer+1'b1;
            if(state==ISSUE || state==WAIT_BUSY || state==WAIT_DONE)tx_timer<=tx_timer+1'b1;
            if((state==WAIT_BUSY || state==WAIT_DONE || state==CHECK) && I_iic_bus_error)
                O_transaction_error_seen<=1;

            if(state!=WAIT_INIT && state!=STATE_UNKNOWN && !I_init_done)fail(8'd10);
            else if(O_update_busy && batch_timer==BATCH_LAST)fail(8'd3);
            else if((state==ISSUE || state==WAIT_BUSY || state==WAIT_DONE) && tx_timer==TX_LAST)fail(8'd2);
            else case(state)
                WAIT_INIT:if(I_init_done)begin
                    phase<=SNAPSHOT;index<=0;batch_timer<=0;tx_timer<=0;state<=ISSUE;
                end
                ISSUE:if(!I_iic_busy)begin
                    // All bus fields are held through busy assertion/deassertion.
                    O_iic_wr_data<={command_value,command_address[7:0],command_address[15:8],8'h6c};
                    O_iic_wr_cnt<=command_read?8'd3:8'd4;
                    O_iic_rd_cnt<=command_read?8'd1:8'd0;
                    O_iic_mode<=command_read;O_iic_req<=1;
                    transaction_address<=command_address;transaction_expected<=command_value;
                    O_transaction_error_seen<=0;state<=WAIT_BUSY;
                end
                WAIT_BUSY:if(I_iic_busy)begin O_iic_req<=0;state<=WAIT_DONE;end
                WAIT_DONE:if(!I_iic_busy)state<=CHECK;
                CHECK:begin
                    tx_timer<=0;
                    if(O_transaction_error_seen || I_iic_bus_error)fail(8'd1);
                    else if((phase==PREP_VERIFY || phase==UPDATE_VERIFY) && I_iic_rd_data!=transaction_expected)
                        fail(8'd7);
                    else begin
                        if(phase==SNAPSHOT)begin
                            snapshot[index]<=I_iic_rd_data;
                            if(index==11)state<=VALIDATE;
                            else begin index<=index+1'b1;state<=ISSUE;end
                        end else if(phase==PREP_WRITE)begin
                            state<=ISSUE;
                            if(index==2)begin phase<=PREP_VERIFY;index<=0;end
                            else index<=index+1'b1;
                        end else if(phase==PREP_VERIFY)begin
                            if(index==2)begin O_register_state_valid<=1;state<=READY;end
                            else begin index<=index+1'b1;state<=ISSUE;end
                        end else if(phase==UPDATE_WRITE)begin
                            state<=ISSUE;
                            if(index==5)begin phase<=UPDATE_VERIFY;index<=0;end
                            else index<=index+1'b1;
                        end else if(phase==UPDATE_VERIFY)begin
                            if(index==8)begin
                                O_written_exposure<=requested_exposure;O_written_gain_index<=requested_gain;
                                O_written_valid<=1;O_write_success<=1;O_register_state_valid<=1;state<=READY;
                            end else begin index<=index+1'b1;state<=ISSUE;end
                        end else fail(8'd11);
                    end
                end
                VALIDATE:begin
                    // VTS<7 makes 2*VTS-10 smaller than the minimum exposure.
                    if(decoded_vts<23'd7)fail(8'd4);
                    else if(decoded_exposure<24'd3 || {1'b0,decoded_exposure}>decoded_max)fail(8'd5);
                    else if(!gain_legal)fail(8'd6);
                    else begin
                        O_vts<=decoded_vts;O_exposure_max<=decoded_max[23:0];
                        O_snapshot_exposure<=decoded_exposure;O_snapshot_gain_index<=decoded_gain;O_snapshot_valid<=1;
                        exposure_high_reserved<=snapshot[3][7:4];exposure_low_reserved<=snapshot[6][3:0];
                        mode_value<={snapshot[9][7:4],4'hb};phase<=PREP_WRITE;index<=0;state<=ISSUE;
                    end
                end
                READY:if(update_edge)begin
                    if(I_requested_exposure<24'd3 || I_requested_exposure>O_exposure_max)begin
                        O_request_rejected<=1;O_reject_reason<=2;O_error<=1;O_error_code<=8;
                    end else if(I_requested_gain_index[7])begin
                        O_request_rejected<=1;O_reject_reason<=3;O_error<=1;O_error_code<=9;
                    end else begin
                        requested_exposure<=I_requested_exposure;requested_gain<=I_requested_gain_index[6:0];
                        O_request_accepted<=1;O_error<=0;O_error_code<=0;O_reject_reason<=0;
                        O_register_state_valid<=0;O_written_valid<=0;
                        phase<=UPDATE_WRITE;index<=0;tx_timer<=0;batch_timer<=0;state<=ISSUE;
                    end
                end
                STATE_UNKNOWN:O_iic_req<=0;
                default:fail(8'd11);
            endcase
        end
    end
    // synthesis translate_off
    initial if(TRANSACTION_TIMEOUT_CYCLES<2 || BATCH_TIMEOUT_CYCLES<2)begin
        $error("Timeout parameters must be at least two I_clk cycles");$finish;
    end
    // synthesis translate_on
endmodule
