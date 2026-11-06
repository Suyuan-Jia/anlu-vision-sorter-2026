// Board A: unchanged camera + complete ISP; GPIO transport replaces DDR/HDMI.
module top_board_a_rgb_source #(
 parameter AE_ENABLE = 1
) (
 input wire I_sys_clk, I_rst_n,
 output wire O_cam_scl, inout wire IO_cam_sda,
 output wire O_cam_24m, O_cam_rst,
 inout wire [1:0] I_button,
 inout wire IO_rx_clk_pad_n, IO_rx_clk_pad_p,
 inout wire [3:0] IO_rx_data_pad_n, IO_rx_data_pad_p,
 input wire I_switch, I_style_key_n,
 output wire [23:0] O_link_data,
 output wire [3:0] O_led,
 output wire O_link_clk, O_link_valid, O_link_sop, O_link_frame, O_link_run
);
    wire        	S_100m_clk;
    wire        	S_24m_clk;
    wire        	S_aux_50m_clk;
	wire        	S_10m_clk;
		
	wire        	S_pll_lock;
    wire        	S_rst_n;
	
	wire [15:0] 	S_ae;
    wire [15:0] 	S_ag;
    wire 			S_cam_cfg_done;
    wire 			S_ae_cfg_done;
    wire 			S_ae_req; 
	
	wire        	S_csi_rx_clk;  
    wire        	S_hs_rx_valid;  
    wire[15:0]  	S_hs_rx_data;   
	wire[1:0]   	S_lane_error;
	
	wire        	S_csi_frame_start;       
	wire        	S_csi_frame_end;         
	wire        	S_csi_valid;             
	wire[31:0]  	S_csi_data;  
		
	wire        	S_raw10_frame_start; 
	wire        	S_raw10_frame_end;   
	wire        	S_raw10_valid;       
	wire [39:0] 	S_raw10_data;
		
	wire 			S_axis_tlast;  
	wire 			S_axis_tuser;  
	wire [39:0]		S_axis_tdata;  
	wire 			S_axis_tvalid; 
	
	wire [39:0]		S_raw_tdata ;
	wire			S_raw_tlast ;
	wire 			S_raw_tvalid;
	wire 			S_raw_tuser ;
	
	// Current camera-to-DDR path is a continuous-stream pipeline.
    // video_in does not provide per-beat backpressure to the ISP.
    wire            S_isp_downstream_ready;

    wire [127:0]    S_ISP_O_tdata;
    wire            S_ISP_O_tlast;
    wire            S_ISP_O_tuser;
    wire            S_ISP_O_tvalid;

    // Sticky indication that the video input FIFO overflowed.
    wire            S_video_in_fifo_overflow; // synthesis keep
	

    wire        S_ddr_clk;
    wire        S_hdmi_pixel_clk;
    wire        S_hdmi_serial_clk;
    wire        S_link_forward_clk;
    wire        S_hdmi_rst_n;
    wire        S_video_out_rst_n;


    wire        S_vi_128b_frame_start;
    wire        S_vi_128b_valid;      
    wire[127:0] S_vi_128b_data;    
    
    wire        S_video_out_rd_busy;
    wire        S_video_in_wr_busy; 
    wire[1:0]   S_video_out_rp;     

    wire        S_ddr_user_wr_en;       
    wire        S_ddr_user_rd_en;       
    wire[24:0]  S_ddr_user_addr;        
    wire[127:0] S_ddr_user_wr_data;     
    wire        S_ddr_user_ready;       
    wire        S_ddr_user_rd_valid;    
    wire[127:0] S_ddr_user_rd_data;     

    wire        S_vi_ddr_wr_en;   
    wire[24:0]  S_vi_ddr_wr_addr; 
    wire[127:0] S_vi_ddr_wr_data;
    wire        S_vi_ddr_wr_ready;

    wire        S_vo_ddr_rd_en;   
    wire[24:0]  S_vo_ddr_rd_addr; 
    wire        S_vo_ddr_rd_ready;
    wire        S_vo_dbg_fifo_rst;
    wire        S_vo_dbg_ddr_rd_valid;
    wire        S_vo_dbg_fifo_rd_en;
    wire        S_vo_dbg_fifo_empty;

    wire        S_init_calib_complete; 
    wire[24:0]  S_mc_app_addr;          
    wire[2:0]   S_mc_app_cmd;           
    wire        S_mc_app_en;            
    wire[127:0] S_mc_app_wdf_data;      
    wire        S_mc_app_wdf_end;       
    wire[15:0]  S_mc_app_wdf_mask;      
    wire        S_mc_app_wdf_wren;      
    wire[127:0] S_mc_app_rd_data;       
    wire        S_mc_app_rd_data_end;   
    wire        S_mc_app_rd_data_valid; 
    wire        S_mc_app_rdy;           
    wire        S_mc_app_wdf_rdy;       
    wire        S_mc_dbg_fifo_afull;
    wire        S_mc_dbg_app_rdy;
    wire        S_mc_dbg_wdf_rdy;
    wire        S_mc_dbg_cmd_pop;

    wire        S_hdmi_vsync;
    wire        S_hdmi_hsync;
    wire        S_hdmi_de;
    wire        S_hdmi_user;
    wire        S_hdmi_last;
    wire        S_video_out_vsync;
    wire        S_hdmi_window_rd_en;
    wire        S_hdmi_out_vsync;
    wire        S_hdmi_out_hsync;
    wire        S_hdmi_out_de;
    wire[23:0]  S_hdmi_out_data;

    wire[23:0]  S_video_rd_data;
    reg         S_dbg_raw_seen;
    reg         S_dbg_isp_seen;
    reg         S_dbg_ddr_wr_seen;
    reg         S_dbg_ddr_rd_seen;
    reg         S_dbg_cam_cfg_seen;
    reg         S_dbg_hs_seen;
    reg         S_dbg_csi_seen;
    reg         S_dbg_raw10_seen;
    reg         S_dbg_isp_fs_seen;
    reg         S_dbg_isp_valid_seen;
    reg         S_dbg_ddr_req_seen;
    reg         S_dbg_ddr_ready_seen;
    reg [1:0]   S_dbg_raw_seen_sync;
    reg [1:0]   S_dbg_isp_seen_sync;
    reg [1:0]   S_dbg_ddr_wr_seen_sync;
    reg [1:0]   S_dbg_ddr_rd_seen_sync;
    reg [1:0]   S_dbg_cam_cfg_seen_sync;
    reg [1:0]   S_dbg_hs_seen_sync;
    reg [1:0]   S_dbg_csi_seen_sync;
    reg [1:0]   S_dbg_raw10_seen_sync;
    reg [1:0]   S_dbg_isp_fs_seen_sync;
    reg [1:0]   S_dbg_isp_valid_seen_sync;
    reg [1:0]   S_dbg_ddr_req_seen_sync;
    reg [1:0]   S_dbg_ddr_ready_seen_sync;
    reg         S_dbg_video_nonzero_seen;
    reg         S_dbg_rd_req_seen;
    reg         S_dbg_rd_valid_seen;
    reg         S_dbg_rd_nonzero_seen;
    reg [1:0]   S_dbg_rd_req_seen_sync;
    reg [1:0]   S_dbg_rd_valid_seen_sync;
    reg [1:0]   S_dbg_rd_nonzero_seen_sync;
    reg         S_dbg_mc_init_seen;
    reg         S_dbg_mc_app_rdy_seen;
    reg         S_dbg_mc_wdf_rdy_seen;
    reg         S_dbg_mc_cmd_pop_seen;
    reg [1:0]   S_dbg_mc_init_seen_sync;
    reg [1:0]   S_dbg_mc_app_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_wdf_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_cmd_pop_seen_sync;
    wire[3:0]   S_hdmi_debug_status;
    wire        S_lane_error_any;



 assign S_rst_n = S_pll_lock;
 assign O_cam_rst = 1'b1;
 assign O_cam_24m = S_24m_clk;
    pll_link_phase #(.FWD_CPHASE(0)) u_PLL(
        .refclk      ( I_sys_clk         ),
        .reset    	 ( ~I_rst_n          ),
        
        .clk0_out    ( S_100m_clk        ),
        .clk1_out    ( S_24m_clk         ),

        .clk2_out    ( S_link_forward_clk ),
        .clk4_out    ( S_hdmi_pixel_clk  ),
        .clk5_out    ( S_hdmi_serial_clk ),

        .lock        ( S_pll_lock        )

    );


	

  
  ae_set u_ae_set (
      .I_clk(S_24m_clk),
      .I_rst(~S_rst_n),
      .I_btn({I_button,2'b11}),
      .I_cam_cfg_done(S_cam_cfg_done),
      .I_ae_cfg_done(S_ae_cfg_done),
      .O_ae_req(S_ae_req),
      .O_ae(S_ae),
      .O_ag(S_ag)
  );
  
 // Declare the statistics nets before generate: otherwise older elaborators
 // create shadow implicit scalar wires inside g_ae.
 wire S_ae_result_valid; // synthesis keep
 wire S_ae_frame_good; // synthesis keep
 wire [29:0] S_ae_raw_sum; // synthesis keep
 wire [19:0] S_ae_pixel_count; // synthesis keep
 wire [19:0] S_ae_saturated_count; // synthesis keep
 wire [3:0] ae_state; // synthesis keep
 wire ae_busy, ae_error, ae_measurement_enable; // synthesis keep
 wire ae_too_dark, ae_good, ae_too_bright; // synthesis keep
 wire [23:0] current_exposure, requested_exposure, written_exposure; // synthesis keep
 wire [6:0] current_gain_index, requested_gain_index, written_gain_index; // synthesis keep
 wire update_request, write_success; // synthesis keep
 wire [1:0] effect_wait_state; // synthesis keep
 wire [31:0] ae_frame_counter; // synthesis keep
 wire ae_raw_reset_n;
 generate if (AE_ENABLE) begin: g_ae
  sc500cs_ae_system u_ae_system (
   .I_raw_clk(S_csi_rx_clk), .I_control_clk(S_24m_clk), .I_rst_n(S_rst_n),
   .I_frame_start(S_raw10_frame_start), .I_first_pixel(S_raw_tuser && S_raw_tvalid),
   .I_stats_valid(S_ae_result_valid), .I_stats_good(S_ae_frame_good),
   .I_raw_sum(S_ae_raw_sum), .I_pixel_count(S_ae_pixel_count), .I_saturated_count(S_ae_saturated_count),
   .O_scl(O_cam_scl), .IO_sda(IO_cam_sda), .O_init_done(S_cam_cfg_done), .O_init_error(),
   .O_raw_reset_n(ae_raw_reset_n), .ae_state(ae_state), .ae_busy(ae_busy), .ae_error(ae_error),
   .ae_measurement_enable(ae_measurement_enable), .ae_too_dark(ae_too_dark), .ae_good(ae_good), .ae_too_bright(ae_too_bright),
   .current_exposure(current_exposure), .current_gain_index(current_gain_index),
   .requested_exposure(requested_exposure), .requested_gain_index(requested_gain_index),
   .written_exposure(written_exposure), .written_gain_index(written_gain_index),
   .update_request(update_request), .write_success(write_success),
   .effect_wait_state(effect_wait_state), .frame_counter(ae_frame_counter)
  );
  assign S_ae_cfg_done=S_cam_cfg_done;
 end else begin: g_legacy
  // Exactly the original init-only camera controller; no runtime preparation
  // writes and no AE requests exist in this elaborated branch.
  assign ae_raw_reset_n=S_rst_n;
  assign ae_state=0;assign ae_busy=0;assign ae_error=0;assign ae_measurement_enable=0;
  assign ae_too_dark=0;assign ae_good=0;assign ae_too_bright=0;
  assign current_exposure=0;assign requested_exposure=0;assign written_exposure=0;
  assign current_gain_index=0;assign requested_gain_index=0;assign written_gain_index=0;
  assign update_request=0;assign write_success=0;assign effect_wait_state=0;assign ae_frame_counter=0;
  uicfgcs500_720p #(
      .CLK_DIV(24000000 / 100000 - 1)
  ) u_uicfgcs500 (
      .I_clk(S_24m_clk),  //系统时钟输入
      .I_rst_n(S_rst_n),  //系统复位输入
      .I_ae_req(1'b0),
      .I_ae(S_ae),
      .I_ag(S_ag),
      .O_cam_scl(O_cam_scl),  //I2C总线，SCL时钟
      .IO_cam_sda(IO_cam_sda),  //I2C总线，SDA数据
      .O_cfg_done(S_cam_cfg_done),  //摄像头寄存器初始化完成
      .O_ae_cfg_done(S_ae_cfg_done)  //AE配置完成
  );
 end endgenerate
	
    mipi_dphy_rx_ph1p_mipiio_wrapper#(
        .DPHY_RX_LOCATION      ( "DPHY0" ),
        .HS_EQUALIZER          ( "0dB"   ),//"0dB","1.5dB","3dB","4.5dB","6dB","7.5dB","9dB","10.5dB"
        .HS_VGA_GAIN           ( "8dB" ),//"-3dB","-1.5dB","0dB","1.5dB","3dB","4.5dB","6dB","7.5dB"
        .LANE_NUM              ( 2       ),
        .BYTE_NUM              ( 1       )
    )u_mipi_dphy_rx_ph1p_mipiio_wrapper(
        .I_lp_clk              ( S_100m_clk       ),
        .I_rst                 ( ~S_rst_n         ),

        .I_clk_lane_in_delay   ( 6'd0             ),
        .I_data_lane0_in_delay ( 6'd0             ),
        .I_data_lane1_in_delay ( 6'd0             ),
        .I_data_lane2_in_delay ( 6'd0             ),
        .I_data_lane3_in_delay ( 6'd0             ),

        .I_lane_invert         ( 4'b0000          ),
     
        .O_hs_rx_clk           ( S_csi_rx_clk     ),
        .O_hs_rx_valid         ( S_hs_rx_valid    ),
        .O_hs_rx_data          ( S_hs_rx_data     ),
      
        .O_lp_rx_lane0_p       (  ),
        .O_lp_rx_lane0_n       (  ),
      
        .I_lp_tx_en            ( 1'b0             ),
        .I_lp_tx_lane0_p       ( 1'b1             ),
        .I_lp_tx_lane0_n       ( 1'b1             ),
      
        .O_lane_match_error    (                  ),
        .O_lane_error          ( S_lane_error     ),
      
        .IO_rx_clk_pad_n       ( IO_rx_clk_pad_n  ),
        .IO_rx_clk_pad_p       ( IO_rx_clk_pad_p  ),
        .IO_rx_data_pad_n      ( IO_rx_data_pad_n ),
        .IO_rx_data_pad_p      ( IO_rx_data_pad_p )
    );
	
	
 //  cwc cwc_inst
 //(
 //    .probe0(S_hs_rx_valid),
 //    .probe1(S_hs_rx_data),
 //    .probe2(S_lane_error),
 //    .clk(S_csi_rx_clk)
 //);

 //csi 解码为RAW数据
csi_unpacket_2lane u_csi_unpacket(
.I_clk                 ( S_csi_rx_clk       ),
.I_rst_n               ( S_rst_n        	),
.I_hs_valid            ( S_hs_rx_valid     ),
.I_hs_data             ( S_hs_rx_data      ),

.O_csi_frame_start     ( S_csi_frame_start ),
.O_csi_frame_end       ( S_csi_frame_end   ),
.O_csi_valid           ( S_csi_valid       ),
.O_csi_data            ( S_csi_data        )
);


//解码为RAW8
raw10_unpacket_2lane u_raw10_unpacket (
.I_clk  (S_csi_rx_clk),
.I_rst_n(S_rst_n),

.I_csi_frame_start(S_csi_frame_start),
.I_csi_frame_end  (S_csi_frame_end),
.I_csi_valid      (S_csi_valid),
.I_csi_data       (S_csi_data),

.O_raw10_frame_start(S_raw10_frame_start),
.O_raw10_frame_end  (S_raw10_frame_end),
.O_raw10_valid      (S_raw10_valid),
.O_raw10_data       (S_raw10_data)
  );


//将数据转为stream流
wire [15:0] S_raw_pixel_x;
wire [13:0] S_raw_pixel_y;
uial2axis #(
.IMG_WIDTH(1280),
.IMG_HEIGHT(720),
.INPUT_DATA_WIDTH(40)
) 
u_uial2axis (
.I_native_clk(S_csi_rx_clk),
.I_rst_n     (S_rst_n),
.I_data      (S_raw10_data       ),
.I_data_valid(S_raw10_valid      ),
.I_data_start(S_raw10_frame_start),
.I_data_end  (S_raw10_frame_end  ),
.axis_tvalid (S_axis_tvalid),
.axis_tdata  (S_axis_tdata ),
.axis_tuser  (S_axis_tuser ),
.axis_tlast  (S_axis_tlast ),
.O_pixel_x  (S_raw_pixel_x),
.O_pixel_y  (S_raw_pixel_y)
);
	

// 原厂HD模式直接输出1280x720 RAW10，不再进行FPGA固定窗口裁剪。
assign S_raw_tdata  = S_axis_tdata;
assign S_raw_tlast  = S_axis_tlast;
assign S_raw_tvalid = S_axis_tvalid;
assign S_raw_tuser  = S_axis_tuser;

// Passive RAW tap: STEP 5 gates results, never the RAW/ISP data path.
// Full-frame bounds: 921600 pixels, max sum 942796800 (30 bits).
wire [15:0] S_ae_dbg_x; // synthesis keep
wire [13:0] S_ae_dbg_y; // synthesis keep
wire [19:0] S_ae_dbg_frame_pixel_count; // synthesis keep
wire [14:0] S_ae_dbg_frame_line_count; // synthesis keep
wire S_ae_dbg_frame_bad; // synthesis keep
sc500cs_ae_stats #(
    .FRAME_WIDTH(1280), .FRAME_HEIGHT(720),
    .AE_X_START(320), .AE_X_END(960),
    .AE_Y_START(180), .AE_Y_END(540),
    .BLACK_LEVEL(10'd0), .SAT_THRESHOLD(10'd1000)
) u_ae_stats (
    .I_clk(S_csi_rx_clk), .I_rst_n(ae_raw_reset_n),
    .I_tdata(S_raw_tdata), .I_tvalid(S_raw_tvalid),
    .I_tuser(S_raw_tuser), .I_tlast(S_raw_tlast),
    .I_pixel_x(S_raw_pixel_x), .I_pixel_y(S_raw_pixel_y),
    .I_frame_start(S_raw10_frame_start), .I_frame_end(S_raw10_frame_end),
    .O_ae_result_valid(S_ae_result_valid), .O_ae_frame_good(S_ae_frame_good),
    .O_ae_raw_sum(S_ae_raw_sum), .O_ae_pixel_count(S_ae_pixel_count),
    .O_ae_saturated_count(S_ae_saturated_count),
    .O_dbg_x(S_ae_dbg_x), .O_dbg_y(S_ae_dbg_y),
    .O_dbg_frame_pixel_count(S_ae_dbg_frame_pixel_count),
    .O_dbg_frame_line_count(S_ae_dbg_frame_line_count),
    .O_dbg_frame_bad(S_ae_dbg_frame_bad)
);


//cwc1 cwc1_Inst
//  (
//      .probe0(S_axis_tuser),
//      .probe1(S_axis_tvalid),
//      .probe2(S_axis_tlast),
//      .probe3(S_axis_tdata),
//      .probe4(S_ISP_O_tuser),
//      .probe5(S_ISP_O_tvalid),
//      .probe6(S_ISP_O_tlast),
//      .probe7(S_ISP_O_tdata),
//      .probe8 (S_raw_tdata ),
//      .probe9 (S_raw_tlast ),
//      .probe10(S_raw_tvalid),
//      .probe11(S_raw_tuser ),
//      .clk(S_csi_rx_clk)
//  );



// -----------------------------------------------------------------------------
// SW1 algorithm-mode control
//
// I_switch is asynchronous to S_csi_rx_clk, therefore it first passes through
// a two-flop synchronizer.
//
// Debounce does not depend on a guessed clock frequency.  The synchronized
// level must be identical at three consecutive RAW frame starts before it is
// accepted as a new requested mode.
//
// Electrical convention:
//   0 = color AWB image
//   1 = verified Sobel edge image
// -----------------------------------------------------------------------------
reg        S_switch_meta;
reg        S_switch_sync;
reg        S_switch_candidate;
reg [1:0]  S_switch_stable_frames;
reg        S_alg_sel_req;

always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
    if (!S_rst_n) begin
        S_switch_meta          <= 1'b0;
        S_switch_sync          <= 1'b0;
        S_switch_candidate     <= 1'b0;
        S_switch_stable_frames <= 2'd0;
        S_alg_sel_req          <= 1'b0;
    end
    else begin
        S_switch_meta <= I_switch;
        S_switch_sync <= S_switch_meta;

        if (S_raw_tvalid && S_raw_tuser) begin
            if (S_switch_sync == S_switch_candidate) begin
                if (S_switch_stable_frames < 2'd3)
                    S_switch_stable_frames <= S_switch_stable_frames + 1'b1;

                if (S_switch_stable_frames == 2'd2)
                    S_alg_sel_req <= S_switch_candidate;
            end
            else begin
                S_switch_candidate     <= S_switch_sync;
                S_switch_stable_frames <= 2'd1;
            end
        end
    end
end

// The MIPI camera source cannot be paused by AXI ready.
// The verified baseline therefore runs the ISP continuously.
// Downstream FIFO overflow is monitored separately.
assign S_isp_downstream_ready = 1'b1;

// -----------------------------------------------------------------------------
//卡通化切换按键
// -----------------------------------------------------------------------------
// 按键先同步到五十兆赫时钟域，再进行连续稳定电平消抖。
reg        S_style_key_meta;
reg        S_style_key_sync;
reg        S_style_key_stable;
reg [19:0] S_style_key_count;
reg        S_cartoon_req_sys;

always @(posedge I_sys_clk or negedge S_rst_n) begin
    if (!S_rst_n) begin
        S_style_key_meta   <= 1'b1;
        S_style_key_sync   <= 1'b1;
        S_style_key_stable <= 1'b1;
        S_style_key_count  <= 20'd0;
        S_cartoon_req_sys  <= 1'b0;
    end
    else begin
        S_style_key_meta <= I_style_key_n;
        S_style_key_sync <= S_style_key_meta;

        if (S_style_key_sync == S_style_key_stable) begin
            S_style_key_count <= 20'd0;
        end
        else if (S_style_key_count == 20'd999999) begin
            S_style_key_count  <= 20'd0;
            S_style_key_stable <= S_style_key_sync;

            // 只在确认按下时切换，松开和持续长按不重复切换。
            if (!S_style_key_sync)
                S_cartoon_req_sys <= ~S_cartoon_req_sys;
        end
        else begin
            S_style_key_count <= S_style_key_count + 1'b1;
        end
    end
end

// 将保持型模式请求同步到摄像头处理时钟域。
// 真正的显示模式由叠加模块在帧首更新。
reg S_cartoon_req_meta;
reg S_cartoon_req;

always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
    if (!S_rst_n) begin
        S_cartoon_req_meta <= 1'b0;
        S_cartoon_req      <= 1'b0;
    end
    else begin
        S_cartoon_req_meta <= S_cartoon_req_sys;
        S_cartoon_req      <= S_cartoon_req_meta;
    end
end


// SW1 polarity: electrical 0 enables CNN; electrical 1 selects original.
//ISP算法顶层模块
wire probe_active,probe_aligned,probe_color_frame,probe_edge_frame;
 isp_color_source u_isp_top (
 .I_recognition_enable(~I_switch),
 .probe_active(probe_active),.probe_aligned(probe_aligned),
 .probe_color_frame(probe_color_frame),.probe_edge_frame(probe_edge_frame),
.axi4s_video_aclk(S_csi_rx_clk),
.I_rst_n         (S_rst_n),
.I_tlast         (S_raw_tlast	),
.I_tuser         (S_raw_tuser	),
.I_tdata         (S_raw_tdata	),
.I_tvalid        (S_raw_tvalid	),
.I_tdest         (),
.O_tready        (S_isp_downstream_ready),
.O_tdata         (S_ISP_O_tdata ),
.O_tlast         (S_ISP_O_tlast ),
.O_tuser         (S_ISP_O_tuser ),
.O_tvalid        (S_ISP_O_tvalid),
.I_tready        (),
.I_alg_sel_req   (~S_alg_sel_req),
.I_cartoon_req   (S_cartoon_req)
  );



 localparam [23:0] LINK_DATA_XOR=24'haaaaaa;
 wire [23:0] tx_data,probe_data;
 wire tx_valid,tx_sop,tx_frame;
 wire link_overflow; // synthesis keep
 gpio_video_tx_dense #(.FIFO_ADDR_WIDTH(11)) u_link_tx (
   .src_clk(S_csi_rx_clk), .link_clk(S_hdmi_pixel_clk), .rst_n(S_rst_n),
   .i_data(S_ISP_O_tdata), .i_valid(S_ISP_O_tvalid), .i_frame(S_ISP_O_tuser),
   .o_data(tx_data), .o_valid(tx_valid), .o_sop(tx_sop),
   .o_frame(tx_frame), .o_run(O_link_run), .overflow(link_overflow)
 );

 wire probe_request,probe_ack;
 wire [255:0] probe_held;
 reg aligned_seen,unaligned_seen;
 always @(posedge S_csi_rx_clk or negedge S_rst_n)
   if(!S_rst_n) begin aligned_seen<=0; unaligned_seen<=0; end
   else begin
     if(probe_aligned) aligned_seen<=1;
     if(probe_color_frame ^ probe_edge_frame) unaligned_seen<=1;
   end
 probe_source u_probe_source(
   .clk(S_csi_rx_clk),.rst_n(S_rst_n),.frame(S_ISP_O_tuser),.valid(S_ISP_O_tvalid),
   .raw_frame(S_raw_tuser),.raw_valid(S_raw_tvalid),
   .color_frame(probe_color_frame),.edge_frame(probe_edge_frame),
   .mode({link_overflow,7'h13}),
   .request(probe_request),.ack(probe_ack),.held(probe_held));
 probe_tx u_probe_tx(
   .clk(S_hdmi_pixel_clk),.rst_n(S_rst_n && O_link_run),
   .data(tx_data),.valid(tx_valid),.sop(tx_sop),.frame(tx_frame),
   .ack(probe_ack),.source_held(probe_held),.request(probe_request),.output_data(probe_data));
 reg [26:0] probe_output; // synthesis keep
 always @(posedge S_hdmi_pixel_clk or negedge O_link_run)
   if(!O_link_run) probe_output<={3'b000,LINK_DATA_XOR};
   else probe_output<={tx_frame,tx_sop,tx_valid,probe_data ^ LINK_DATA_XOR};
 assign {O_link_frame,O_link_sop,O_link_valid,O_link_data}=probe_output;
 // HRIO output on F14: d0 is rising-edge data, d1 is falling-edge data.
 // Dedicated inverted C2 forwards 78.947 MHz; vendor-model sampling phase
 // is about 7.167 ns after C4 data launch, matching the reference link PLL.
 PH1P_LOGIC_ODDR u_link_clock (
   .clk(S_link_forward_clk), .rst(!S_rst_n), .d0(1'b0), .d1(1'b1), .q(O_link_clk)
 );
// Independent visible evidence: cfg done, ISP frame seen, overflow, RUN.
 reg isp_frame_seen;
 always @(posedge S_csi_rx_clk or negedge S_rst_n)
   if (!S_rst_n) isp_frame_seen <= 0;
   else if (S_ISP_O_tuser) isp_frame_seen <= 1;
 // These sticky levels drive indicators only, not another clock domain.
 assign O_led = {O_link_run, link_overflow, isp_frame_seen, S_cam_cfg_done};
endmodule
