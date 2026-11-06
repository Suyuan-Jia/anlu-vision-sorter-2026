`timescale 1ns/1ps
// STEP 5 passive AE control plane. Stats are external and never stalled.
// One I2C master, with irreversible init -> runtime ownership until reset.
module sc500cs_ae_system #(
 parameter integer CLK_DIV=239,
 parameter integer POST_STREAM_DELAY=240000,
 parameter integer INIT_TIMEOUT=240000
)(
 input wire I_raw_clk,I_control_clk,I_rst_n,
 input wire I_frame_start,I_first_pixel,I_stats_valid,I_stats_good,
 input wire [29:0] I_raw_sum,
 input wire [19:0] I_pixel_count,I_saturated_count,
 output wire O_scl,inout wire IO_sda,
 output reg O_init_done,O_init_error,
 output wire O_raw_reset_n,
 output wire [3:0] ae_state,
 output wire ae_busy,ae_error,ae_measurement_enable,
 output wire ae_too_dark,ae_good,ae_too_bright,
 output wire [23:0] current_exposure,requested_exposure,written_exposure,
 output wire [6:0] current_gain_index,requested_gain_index,written_gain_index,
 output wire update_request,write_success,
 output wire [1:0] effect_wait_state,
 output reg [31:0] frame_counter
);
 wire rr,cr;
 assign O_raw_reset_n=rr;
 wire cdc_req,cdc_ready,resp,rt_req,rt_ready,rt_sv,rt_rv,rt_acc,rt_rej,rt_success,rt_error,rt_unknown;
 wire [23:0] cdc_exp,resp_exp,resp_max,rt_exp,rt_snapshot_exp,rt_written_exp,rt_max;
 wire [6:0] cdc_gain,resp_gain,rt_snapshot_gain,rt_written_gain;
 wire [7:0] rt_gain,rt_code,resp_err;
 wire [1:0] kind;
 wire enable,consumable,accepted,inflight,current_valid;
 wire [23:0] max_exp;
 wire [7:0] error_code;
 assign write_success=resp && kind==1; // RAW-domain debug event
 sc500cs_ae_controller controller(
  .I_clk(I_raw_clk),.I_rst_n(rr),.I_controller_enable(enable),.I_update_busy(ae_busy),
  .I_ae_result_valid(consumable),.I_ae_frame_good(I_stats_good),.I_ae_raw_sum(I_raw_sum),
  .I_ae_pixel_count(I_pixel_count),.I_ae_saturated_count(I_saturated_count),
  .I_current_exposure(current_exposure),.I_current_gain_index(current_gain_index),
  .I_exposure_min(24'd3),.I_exposure_max(max_exp),.O_update_request(update_request),
  .O_requested_exposure(requested_exposure),.O_requested_gain_index(requested_gain_index),
  .O_ae_too_dark(ae_too_dark),.O_ae_good(ae_good),.O_ae_too_bright(ae_too_bright),
  .O_requested_analog_gain(),.O_requested_analog_fine_gain(),.O_highlight_saturated(),
  .O_dark_confirm_count(),.O_bright_confirm_count(),.O_last_raw_sum(),.O_last_pixel_count());
 sc500cs_ae_effect_manager effect(
  .I_clk(I_raw_clk),.I_rst_n(rr),.I_physical_frame_start(I_frame_start),.I_first_valid_pixel(I_first_pixel),
  .I_stats_result_valid(I_stats_valid),.I_stats_frame_good(I_stats_good),
  .I_update_request(update_request),.I_requested_exposure(requested_exposure),.I_requested_gain(requested_gain_index),
  .I_cdc_ready(cdc_ready),.O_cdc_request(cdc_req),.O_cdc_exposure(cdc_exp),.O_cdc_gain(cdc_gain),
  .I_response_valid(resp),.I_response_kind(kind),.I_response_exposure(resp_exp),.I_response_exposure_max(resp_max),
  .I_response_gain(resp_gain),.I_response_error(resp_err),.O_request_accepted(accepted),
  .O_written_exposure(written_exposure),.O_written_gain(written_gain_index),
  .O_current_exposure(current_exposure),.O_current_gain(current_gain_index),.O_current_valid(current_valid),
  .O_exposure_max(max_exp),.O_controller_busy(ae_busy),.O_controller_enable(enable),
  .O_measurement_enable(ae_measurement_enable),.O_controller_result_valid(consumable),
  .O_update_in_flight(inflight),.O_error(ae_error),.O_error_code(error_code),
  .O_debug_state(ae_state),.O_wait_frame_count(effect_wait_state),.O_request_ready(),.O_request_rejected(),
  .O_requested_exposure(),.O_requested_gain(),.O_pending_exposure(),.O_pending_gain(),.O_written_valid(),.O_effective_commit());
 sc500cs_ae_cdc cdc(
  .I_raw_clk(I_raw_clk),.I_raw_rst_n(I_rst_n),.I_control_clk(I_control_clk),.I_control_rst_n(I_rst_n),
  .O_raw_reset_n(rr),.O_control_reset_n(cr),.I_request(cdc_req),.I_requested_exposure(cdc_exp),.I_requested_gain(cdc_gain),
  .O_request_ready(cdc_ready),.O_response_valid(resp),.O_response_kind(kind),.O_response_exposure(resp_exp),
  .O_response_gain(resp_gain),.O_response_exposure_max(resp_max),.O_response_error(resp_err),
  .I_runtime_ready(rt_ready),.I_snapshot_valid(rt_sv),.I_register_state_valid(rt_rv),
  .I_snapshot_exposure(rt_snapshot_exp),.I_snapshot_gain(rt_snapshot_gain),.I_exposure_max(rt_max),
  .I_runtime_accepted(rt_acc),.I_runtime_rejected(rt_rej),.I_write_success(rt_success),
  .I_write_error(rt_error || O_init_error),.I_state_unknown(rt_unknown),
  .I_written_exposure(rt_written_exp),.I_written_gain(rt_written_gain),.I_error_code(O_init_error?8'hd1:rt_code),
  .O_runtime_request(rt_req),.O_runtime_exposure(rt_exp),.O_runtime_gain(rt_gain),
  .O_request_accepted(),.O_request_rejected(),.O_debug_control_state());
 wire mreq,mmode,mbusy,merror;
 wire [31:0] mdata;
 wire [7:0] wc,rc,rd;
 sc500cs_runtime_manager runtime(
  .I_clk(I_control_clk),.I_rst_n(cr),.I_init_done(O_init_done),.I_update_req(rt_req),
  .I_requested_exposure(rt_exp),.I_requested_gain_index(rt_gain),
  .I_iic_busy(mbusy),.I_iic_bus_error(merror),.I_iic_rd_data(rd),
  .O_iic_req(mreq),.O_iic_wr_data(mdata),.O_iic_wr_cnt(wc),.O_iic_rd_cnt(rc),.O_iic_mode(mmode),
  .O_runtime_ready(rt_ready),.O_request_accepted(rt_acc),.O_request_rejected(rt_rej),
  .O_snapshot_valid(rt_sv),.O_register_state_valid(rt_rv),.O_snapshot_exposure(rt_snapshot_exp),
  .O_snapshot_gain_index(rt_snapshot_gain),.O_exposure_max(rt_max),
  .O_written_exposure(rt_written_exp),.O_written_gain_index(rt_written_gain),
  .O_write_success(rt_success),.O_write_error(rt_error),.O_state_unknown(rt_unknown),.O_error_code(rt_code),
  .O_update_busy(),.O_reject_reason(),.O_snapshot_raw(),.O_vts(),.O_exposure_min(),.O_written_valid(),
  .O_error(),.O_error_register(),.O_transaction_error_seen(),.O_debug_state(),.O_debug_phase(),.O_debug_index());
 reg init_req;
 reg [31:0] init_data;
 reg [8:0] init_index;
 wire [31:0] table_data;
 wire [8:0] table_size;
 reg [2:0] init_state;
 reg [31:0] init_timer,delay_timer;
 uics500reg_720p60 table_rom(.REG_INDEX(init_index),.REG_DATA(table_data),.REG_SIZE(table_size));
 // Same table, 128-clock reset settling, and post-stream delay as legacy init.
 // Added NACK/timeout fail-stop; ownership changes only after final STOP.
 always @(posedge I_control_clk or negedge cr)begin
  if(!cr)begin
   O_init_done<=0;O_init_error<=0;init_req<=0;init_data<=0;
   init_index<=0;init_state<=0;init_timer<=0;delay_timer<=0;
  end else if(!O_init_done && !O_init_error)begin
   case(init_state)
    0:if(init_timer==127)begin init_timer<=0;init_state<=1;end else init_timer<=init_timer+1'b1;
    1:begin
     init_timer<=0;
     if(init_index==table_size)O_init_done<=1;
     else if(init_index==115 && delay_timer<POST_STREAM_DELAY)delay_timer<=delay_timer+1'b1;
     else if(!mbusy)begin
      init_data<={table_data[7:0],table_data[15:8],table_data[23:16],8'h6c};
      init_req<=1;init_state<=2;
     end
    end
    2:begin
     init_timer<=init_timer+1'b1;
     if(mbusy)begin init_req<=0;init_state<=3;end
     else if(init_timer==INIT_TIMEOUT-1)begin O_init_error<=1;init_req<=0;end
    end
    3:begin
     init_timer<=init_timer+1'b1;
     if(merror)O_init_error<=1;
     else if(!mbusy)begin init_index<=init_index+1'b1;init_state<=1;end
     else if(init_timer==INIT_TIMEOUT-1)O_init_error<=1;
    end
    default:O_init_error<=1;
   endcase
  end
 end
 uii2c #(.WMEN_LEN(4),.RMEN_LEN(1),.CLK_DIV(CLK_DIV)) master(
  .I_clk(I_control_clk),.I_rstn(cr),.I_iic_req(!O_init_error && (O_init_done?mreq:init_req)),
  .I_wr_data(O_init_done?mdata:init_data),.I_wr_cnt(O_init_done?wc:8'd4),
  .I_rd_cnt(O_init_done?rc:8'd0),.I_iic_mode(O_init_done?mmode:1'b0),
  .O_iic_busy(mbusy),.O_iic_bus_error(merror),.O_rd_data(rd),.O_iic_scl(O_scl),.IO_iic_sda(IO_sda),.IO_iic_sda_dg());
 always @(posedge I_raw_clk or negedge rr)
  if(!rr)frame_counter<=0;else if(I_frame_start)frame_counter<=frame_counter+1'b1;
endmodule
