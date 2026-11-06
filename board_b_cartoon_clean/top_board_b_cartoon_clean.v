// 功能说明：B板已验证卡通工程的无叠加版本：协议、算法、时钟及帧保护保持。

// Board B: unchanged frame buffer and HDMI; GPIO receiver replaces camera/ISP.

// 0: 160x90 equivalent timing-fix comparison; 1: 320x180 DDR + boundary reconstruction.
module top_board_b_cartoon_clean #(parameter REGION_USE_DDR=1) ( input wire I_sys_clk,
    I_rst_n,
    I_switch,
    I_motion_switch,
    I_key1_n,
    input wire [23 : 0] I_link_data,
    input wire I_link_clk,
    I_link_valid,
    I_link_sop,
    I_link_frame,
    I_link_run,
    output wire O_screen_pwm,
    O_tmds_ch0_p,
    O_tmds_ch1_p,
    O_tmds_ch2_p,
    O_tmds_clk_p,
    output wire [12 : 0] ddr_addr,
    output wire [1 : 0] ddr_ba,
    output wire [0 : 0] ddr_cke,
    output wire [0 : 0] ddr_odt,
    output wire [0 : 0] ddr_cs_n,
    output wire ddr_ras_n,
    output wire ddr_cas_n,
    output wire ddr_we_n,
    output wire [0 : 0] ddr_ck_p,
    output wire [0 : 0] ddr_ck_n,
    inout wire [1 : 0] ddr_dm,
    inout wire [15 : 0] ddr_dq,
    inout wire [1 : 0] ddr_dqs_p,
    inout wire [1 : 0] ddr_dqs_n );

    localparam [23:0] LINK_DATA_XOR=24'haaaaaa;
    wire S_100m_clk;
    wire S_24m_clk;
    wire S_aux_50m_clk;
    wire S_10m_clk;
    wire S_pll_lock;
    wire S_rst_n;
    wire [15 : 0] S_ae;
    wire [15 : 0] S_ag;
    wire S_cam_cfg_done;
    wire S_ae_cfg_done;
    wire S_ae_req;
    wire S_csi_rx_clk;
    wire S_hs_rx_valid;
    wire [15 : 0] S_hs_rx_data;
    wire [1 : 0] S_lane_error;
    wire S_csi_frame_start;
    wire S_csi_frame_end;
    wire S_csi_valid;
    wire [31 : 0] S_csi_data;
    wire S_raw10_frame_start;
    wire S_raw10_frame_end;
    wire S_raw10_valid;
    wire [39 : 0] S_raw10_data;
    wire S_axis_tlast;
    wire S_axis_tuser;
    wire [39 : 0] S_axis_tdata;
    wire S_axis_tvalid;
    wire [39 : 0] S_raw_tdata;
    wire S_raw_tlast;
    wire S_raw_tvalid;
    wire S_raw_tuser;
    // Current camera-to-DDR path is a continuous-stream pipeline.
    // video_in does not provide per-beat backpressure to the ISP.
    wire S_isp_downstream_ready;
    wire [127 : 0] S_ISP_O_tdata;
    wire S_ISP_O_tlast;
    wire S_ISP_O_tuser;
    wire S_ISP_O_tvalid;
    // Sticky indication that the video input FIFO overflowed.
    wire S_video_in_fifo_overflow; // synthesis keep
    wire S_ddr_clk;
    wire S_hdmi_pixel_clk;
    wire S_hdmi_serial_clk;
    wire S_hdmi_rst_n;
    wire S_video_out_rst_n;
    wire S_vi_128b_frame_start;
    wire S_vi_128b_valid;
    wire [127 : 0] S_vi_128b_data;
    wire S_video_out_rd_busy;
    wire S_video_in_wr_busy;
    wire [1 : 0] S_video_out_rp;
    wire S_ddr_user_wr_en;
    wire S_ddr_user_rd_en;
    wire [24 : 0] S_ddr_user_addr;
    wire [127 : 0] S_ddr_user_wr_data;
    wire S_ddr_user_ready;
    wire S_ddr_user_rd_valid;
    wire [127 : 0] S_ddr_user_rd_data;
    wire S_vi_ddr_wr_en;
    wire [24 : 0] S_vi_ddr_wr_addr;
    wire [127 : 0] S_vi_ddr_wr_data;
    wire S_vi_ddr_wr_ready;
    wire S_vo_ddr_rd_en;
    wire [24 : 0] S_vo_ddr_rd_addr;
    wire S_vo_ddr_rd_ready;
    wire S_vo_dbg_fifo_rst;
    wire S_vo_dbg_ddr_rd_valid;
    wire S_vo_dbg_fifo_rd_en;
    wire S_vo_dbg_fifo_empty;
    wire S_init_calib_complete;
    wire [24 : 0] S_mc_app_addr;
    wire [2 : 0] S_mc_app_cmd;
    wire S_mc_app_en;
    wire [127 : 0] S_mc_app_wdf_data;
    wire S_mc_app_wdf_end;
    wire [15 : 0] S_mc_app_wdf_mask;
    wire S_mc_app_wdf_wren;
    wire [127 : 0] S_mc_app_rd_data;
    wire S_mc_app_rd_data_end;
    wire S_mc_app_rd_data_valid;
    wire S_mc_app_rdy;
    wire S_mc_app_wdf_rdy;
    wire S_mc_dbg_fifo_afull;
    wire S_mc_dbg_app_rdy;
    wire S_mc_dbg_wdf_rdy;
    wire S_mc_dbg_cmd_pop;
    wire S_hdmi_vsync;
    wire S_hdmi_hsync;
    wire S_hdmi_de;
    wire S_hdmi_user;
    wire S_hdmi_last;
    wire S_video_out_vsync;
    wire S_hdmi_window_rd_en;
    wire S_hdmi_out_vsync;
    wire S_hdmi_out_hsync;
    wire S_hdmi_out_de;
    wire [23 : 0] S_hdmi_out_data;
    wire [23 : 0] S_video_rd_data;
    reg S_dbg_raw_seen;
    reg S_dbg_isp_seen;
    reg S_dbg_ddr_wr_seen;
    reg S_dbg_ddr_rd_seen;
    reg S_dbg_cam_cfg_seen;
    reg S_dbg_hs_seen;
    reg S_dbg_csi_seen;
    reg S_dbg_raw10_seen;
    reg S_dbg_isp_fs_seen;
    reg S_dbg_isp_valid_seen;
    reg S_dbg_ddr_req_seen;
    reg S_dbg_ddr_ready_seen;
    reg [1 : 0] S_dbg_raw_seen_sync;
    reg [1 : 0] S_dbg_isp_seen_sync;
    reg [1 : 0] S_dbg_ddr_wr_seen_sync;
    reg [1 : 0] S_dbg_ddr_rd_seen_sync;
    reg [1 : 0] S_dbg_cam_cfg_seen_sync;
    reg [1 : 0] S_dbg_hs_seen_sync;
    reg [1 : 0] S_dbg_csi_seen_sync;
    reg [1 : 0] S_dbg_raw10_seen_sync;
    reg [1 : 0] S_dbg_isp_fs_seen_sync;
    reg [1 : 0] S_dbg_isp_valid_seen_sync;
    reg [1 : 0] S_dbg_ddr_req_seen_sync;
    reg [1 : 0] S_dbg_ddr_ready_seen_sync;
    reg S_dbg_video_nonzero_seen;
    reg S_dbg_rd_req_seen;
    reg S_dbg_rd_valid_seen;
    reg S_dbg_rd_nonzero_seen;
    reg [1 : 0] S_dbg_rd_req_seen_sync;
    reg [1 : 0] S_dbg_rd_valid_seen_sync;
    reg [1 : 0] S_dbg_rd_nonzero_seen_sync;
    reg S_dbg_mc_init_seen;
    reg S_dbg_mc_app_rdy_seen;
    reg S_dbg_mc_wdf_rdy_seen;
    reg S_dbg_mc_cmd_pop_seen;
    reg [1 : 0] S_dbg_mc_init_seen_sync;
    reg [1 : 0] S_dbg_mc_app_rdy_seen_sync;
    reg [1 : 0] S_dbg_mc_wdf_rdy_seen_sync;
    reg [1 : 0] S_dbg_mc_cmd_pop_seen_sync;
    wire [3 : 0] S_hdmi_debug_status;
    wire S_lane_error_any;
    assign S_rst_n = S_pll_lock;
    assign S_hdmi_rst_n = S_pll_lock;
    assign O_screen_pwm = 1'b1;
    assign S_video_out_vsync = ~ S_hdmi_vsync;
    assign S_hdmi_debug_status = { ( | S_ddr_user_rd_data ), S_ddr_user_rd_valid, S_vo_ddr_rd_en, S_vi_ddr_wr_en };
    // 系统PLL：保持原像素、串行发送时钟及锁定复位连接。
    PLL u_PLL ( .refclk ( I_sys_clk ),
        .reset ( ~ I_rst_n ),
        .clk0_out ( S_100m_clk ),
        .clk1_out ( S_24m_clk ),
        .clk4_out ( S_hdmi_pixel_clk ),
        .clk5_out ( S_hdmi_serial_clk ),
        .lock ( S_pll_lock ) );

    // 板间转发时钟接入及接收域复位同步。
    PH1P_LOGIC_BUFG u_link_bufg ( .i ( I_link_clk ),
        .o ( S_csi_rx_clk ) );

    reg [2 : 0] link_reset_pipe;
    wire link_reset_async_n = S_rst_n && I_link_run;

    // 时序/组合逻辑：按以下触发条件更新本功能状态。
    always @ ( posedge S_csi_rx_clk or negedge link_reset_async_n ) if ( ! link_reset_async_n ) link_reset_pipe <= 0;
    else link_reset_pipe <= { link_reset_pipe [1 : 0], 1'b1 };
    wire link_protocol_error; // synthesis keep
    wire previous_good, frame_available, display_valid;
    wire [1 : 0] read_bank;
    wire [31 : 0] good_count, bad_count, error_count, committed, rejected;
    wire [3 : 0] last_error;
    wire [17 : 0] last_words;
    wire [26 : 0] probe_bus;
    wire [255 : 0] first_record, continuous_record;
    wire [127 : 0] diagnostic_summary;
    wire [31 : 0] rx_frames;
    wire [127 : 0] rx_data;
    wire rx_valid, rx_frame, processed_good, edge_active, adapter_error;
    wire [2:0] processed_mode, published_mode, selected_mode;
    wire [31 : 0] telemetry_packets, telemetry_errors;
    // SW1选择：原始电平两级同步，连续三帧一致后更新彩色/算法请求。
    // SW1 level synchronizer and frame-based debounce (same scheme as unsplit top).
    // Keep the physical level unchanged here; apply polarity only at edge_request.
    reg switch_meta;
    reg switch_sync;
    reg switch_candidate;
    reg switch_debounced;
    reg [1 : 0] stable_frames;
    wire edge_request;
    reg motion_meta,motion_sync,motion_candidate,motion_request;
    reg [1:0] motion_stable;
    // Electrical 0 selects edge; electrical 1 selects color.
    // rgb_edge_bridge applies this request at an image frame start, never mid-frame.
    assign edge_request = ~switch_debounced && !motion_request;

    // 时序/组合逻辑：按以下触发条件更新本功能状态。
    always @ ( posedge S_csi_rx_clk or negedge link_reset_pipe [2] ) begin
        if ( ! link_reset_pipe [2] ) begin
            switch_meta <= 1'b1;
            switch_sync <= 1'b1;
            switch_candidate <= 1'b1;
            switch_debounced <= 1'b1;
            // Start in color until SW1 is confirmed.
            stable_frames <= 2'd0;
        end
        else begin
            // Two flip-flops synchronize the asynchronous physical switch.
            switch_meta <= I_switch;
            switch_sync <= switch_meta;
            // Three matching received frame markers confirm the new level.
            // A different sample restarts the count; a missing frame holds state.
            if ( rx_frame ) begin
                if ( switch_sync != switch_candidate ) begin
                    switch_candidate <= switch_sync;
                    stable_frames <= 2'd1;
                end
                else begin
                    if ( stable_frames < 2'd3 ) stable_frames <= stable_frames + 1'b1;
                    if ( stable_frames == 2'd2 ) switch_debounced <= switch_candidate;
                end
            end
        end
    end
    // B SW2/B4 inverted: 0 enables motion, 1 bypasses; three-frame debounce.
    always @(posedge S_csi_rx_clk or negedge link_reset_pipe[2])
      if(!link_reset_pipe[2])begin
        motion_meta<=0;motion_sync<=0;motion_candidate<=0;motion_request<=0;motion_stable<=0;
      end else begin
        motion_meta<=~I_motion_switch;motion_sync<=motion_meta;
        if(rx_frame)begin
          if(motion_sync!=motion_candidate)begin motion_candidate<=motion_sync;motion_stable<=1;end
          else begin
            if(motion_stable<3)motion_stable<=motion_stable+1'b1;
            if(motion_stable==2)motion_request<=motion_candidate;
          end
        end
      end
    wire [95:0] motion_pixels;
    wire motion_valid,motion_sof,motion_end,motion_good;
    wire motion_cmd,motion_wr,motion_accept,motion_response;
    wire [24:0] motion_address;
    wire [127:0] motion_wdata,motion_rdata;
    wire [44:0] motion_result;
    wire motion_result_toggle,motion_result_ack;
    wire [31:0] motion_completed,motion_skipped;
    motion_detector_ddr u_motion(
      .clk(S_csi_rx_clk),.rst_n(link_reset_pipe[2]),.enable(motion_request),
      .pixels(motion_pixels),.valid(motion_valid),.sof(motion_sof),
      .frame_end(motion_end),.frame_good(motion_good),
      .request(motion_cmd),.write(motion_wr),.address(motion_address),.wdata(motion_wdata),
      .accept(motion_accept),.response(motion_response),.rdata(motion_rdata),
      .result(motion_result),.result_toggle(motion_result_toggle),.result_ack(motion_result_ack),
      .completed(motion_completed),.skipped(motion_skipped));
    // One debounced press toggles an event bit in 50 MHz, then the event
    // crosses into the link clock. Keep the three-state mode entirely local
    // to avoid incoherent multi-bit CDC during GRAY -> CARTOON.
    wire key_press_toggle_sys;
    reg key_press_meta, key_press_sync, key_press_seen;
    reg [1:0] effect_mode; // 0: edge, 1: grayscale, 2: cartoon
    wire gray_request=(effect_mode==1);
    wire cartoon_request=(effect_mode==2);
    key1_style_control u_key1(.clk(I_sys_clk),.rst_n(S_rst_n),
      .key_n(I_key1_n),.press_toggle(key_press_toggle_sys));
    always @(posedge S_csi_rx_clk or negedge link_reset_pipe[2])
      if(!link_reset_pipe[2]) begin
        key_press_meta<=0;key_press_sync<=0;key_press_seen<=0;effect_mode<=0;
      end else begin
        key_press_meta<=key_press_toggle_sys;
        key_press_sync<=key_press_meta;
        key_press_seen<=key_press_sync;
        if(!edge_request) effect_mode<=0;
        else if(key_press_sync!=key_press_seen)
          effect_mode<=(effect_mode==2)?0:effect_mode+1'b1;
      end
    // Local image processing: color bypass and edge share the protected DDR path.
    // 图像处理：彩色旁路、纯边缘、灰度和卡通模式，保留原完整帧边界。
    wire replay_ready,replay_drained;
    wire capture_ready,capture_drained,capture_complete,capture_error,memory_fault;
    wire capture_begin,capture_bank,raw_valid,low_valid,mem_request,mem_wr,mem_accept,mem_response;
    wire [127:0] raw_data,low_data,mem_data,mem_rdata;
    wire [24:0] mem_address;
    wire legacy_ready,legacy_read_ready,legacy_rvalid;wire [127:0] legacy_rdata;
    rgb_edge_bridge #(.USE_DDR(REGION_USE_DDR),.ENABLE_GRAY(1)) u_local_edge ( .clk ( S_csi_rx_clk ),
        .rst_n ( link_reset_pipe [2] ),
        .i_data ( rx_data ),
        .i_valid ( rx_valid ),
        .i_frame ( rx_frame ),
        .i_previous_good ( previous_good ),
        .edge_request ( edge_request ),
        .gray_request(gray_request),
        .cartoon_request ( cartoon_request ),
        .replay_ready(replay_ready),.replay_drained(replay_drained),
        .capture_ready(capture_ready),.capture_drained(capture_drained),.capture_complete(capture_complete),.capture_error(capture_error),.memory_fault(memory_fault),
        .capture_begin(capture_begin),.capture_bank(capture_bank),.raw_valid(raw_valid),.raw_data(raw_data),.low_valid(low_valid),.low_data(low_data),
        .mem_request(mem_request),.mem_wr(mem_wr),.mem_address(mem_address),.mem_data(mem_data),
        .mem_accept(mem_accept),.mem_response(mem_response),.mem_rdata(mem_rdata),
        .o_data ( S_ISP_O_tdata ),
        .o_valid ( S_ISP_O_tvalid ),
        .o_frame ( S_ISP_O_tuser ),
        .o_mode(processed_mode),
        .o_previous_good ( processed_good ),
        .edge_active ( edge_active ),
        .adapter_error ( adapter_error ),.motion_request(motion_request),
        .analysis_pixels(motion_pixels),.analysis_valid(motion_valid),.analysis_sof(motion_sof),
        .analysis_end(motion_end),.analysis_good(motion_good) );

    // 板间接收：保持既有协议、长度检查、坏帧抑制及恢复。
    gpio_video_rx_fault_cont u_link_rx ( .diagnostic_mode ( edge_active ? 8'h1c : 8'h13 ),
        .diagnostic_packets ( telemetry_packets ),
        .diagnostic_summary ( diagnostic_summary ),
        .continuous_record ( continuous_record ),
        .probe_bus ( probe_bus ),
        .first_record ( first_record ),
        .frame_count ( rx_frames ),
        .clk ( S_csi_rx_clk ),
        .rst_n ( link_reset_pipe [2] ),
        .i_data ( I_link_data ^ LINK_DATA_XOR ),
        .i_valid ( I_link_valid ),
        .i_sop ( I_link_sop ),
        .i_frame ( I_link_frame ),
        .o_data ( rx_data ),
        .o_valid ( rx_valid ),
        .o_frame ( rx_frame ),
        .protocol_error ( link_protocol_error ),
        .previous_good ( previous_good ),
        .good_count ( good_count ),
        .bad_count ( bad_count ),
        .error_count ( error_count ),
        .last_error ( last_error ),
        .last_words ( last_words ) );

    wire [31 : 0] last_written, reject_flags, bank_debug, selected_seq, display_frames, read_commands, read_responses, underflows;
    // DDR写入：仅发布完整帧，防止坏帧覆盖当前显示。
    video_in_live u_video_in ( .O_last_written ( last_written ),
        .O_reject_flags ( reject_flags ),
        .O_bank_debug ( bank_debug ),
        .O_camera_ready(replay_ready),.O_camera_drained(replay_drained),
        .I_rst_n ( S_rst_n ),
        .I_camera_clk ( S_csi_rx_clk ),
        .I_camera_frame_start ( S_ISP_O_tuser ),
        .I_camera_valid ( S_ISP_O_tvalid ),
        .I_camera_data ( S_ISP_O_tdata ),
        .I_camera_mode(processed_mode),.O_published_mode(published_mode),
        .I_previous_good ( processed_good ),
        .I_read_bank ( read_bank ),
        .O_frame_available ( frame_available ),
        .O_committed ( committed ),
        .O_rejected ( rejected ),
        .I_ddr_clk ( S_ddr_clk ),
        .I_video_out_rd_busy ( S_video_out_rd_busy ),
        .O_video_in_wr_busy ( S_video_in_wr_busy ),
        .O_video_out_rp ( S_video_out_rp ),
        .O_ddr_user_wr_en ( S_vi_ddr_wr_en ),
        .O_ddr_user_addr ( S_vi_ddr_wr_addr ),
        .O_ddr_user_wr_data ( S_vi_ddr_wr_data ),
        .I_ddr_user_ready ( legacy_ready ),
        .O_fifo_overflow ( S_video_in_fifo_overflow ) );

    region_ddr_port u_region_memory(
      .link_clk(S_csi_rx_clk),.ddr_clk(S_ddr_clk),.rst_n(S_rst_n),
      .capture_begin(capture_begin),.capture_bank(capture_bank),.raw_valid(raw_valid),.raw_data(raw_data),.low_valid(low_valid),.low_data(low_data),
      .capture_ready(capture_ready),.capture_drained(capture_drained),.capture_complete(capture_complete),.capture_error(capture_error),
      .request(mem_request),.wr(mem_wr),.address(mem_address),.data(mem_data),.accept(mem_accept),.response(mem_response),.rdata(mem_rdata),
      .legacy_wr(S_vi_ddr_wr_en),.legacy_rd(S_vo_ddr_rd_en),.legacy_addr(S_vi_ddr_wr_en?S_vi_ddr_wr_addr:S_vo_ddr_rd_addr),.legacy_data(S_vi_ddr_wr_data),
      .legacy_ready(legacy_ready),.legacy_read_ready(legacy_read_ready),.legacy_rvalid(legacy_rvalid),.legacy_rdata(legacy_rdata),
      .user_wr(S_ddr_user_wr_en),.user_rd(S_ddr_user_rd_en),.user_addr(S_ddr_user_addr),.user_data(S_ddr_user_wr_data),
      .user_ready(S_ddr_user_ready),.user_rvalid(S_ddr_user_rd_valid),.user_rdata(S_ddr_user_rd_data),.fault(memory_fault));
    assign S_video_out_rst_n = S_hdmi_rst_n;
    video_out_live u_video_out ( .I_published_mode(published_mode),.O_selected_mode(selected_mode),
        .I_published_seq ( committed ),
        .O_selected_seq ( selected_seq ),
        .O_display_frames ( display_frames ),
        .O_read_commands ( read_commands ),
        .O_read_responses ( read_responses ),
        .O_underflows ( underflows ),
        .I_frame_available ( frame_available ),
        .O_read_bank ( read_bank ),
        .O_display_valid ( display_valid ),
        .I_rst_n ( S_video_out_rst_n ),
        .I_ddr_clk ( S_ddr_clk ),
        .O_video_out_rd_busy ( S_video_out_rd_busy ),
        .I_video_in_wr_busy ( S_video_in_wr_busy ),
        .I_video_out_rp ( S_video_out_rp ),
        .O_ddr_user_rd_en ( S_vo_ddr_rd_en ),
        .O_ddr_user_addr ( S_vo_ddr_rd_addr ),
        .I_ddr_user_ready ( legacy_read_ready ),
        .I_ddr_user_rd_valid ( legacy_rvalid ),
        .I_ddr_user_rd_data ( legacy_rdata ),
        .I_dsi_clk ( S_hdmi_pixel_clk ),
        .I_video_vsync ( S_video_out_vsync ),
        .I_video_rd_en ( S_hdmi_window_rd_en ),
        .O_vdieo_data ( S_video_rd_data ) );

    // New analysis traffic is below the unchanged legacy DDR interface.
    wire shared_wr,shared_rd,shared_ready,shared_rvalid;
    wire [24:0] shared_addr;
    wire [127:0] shared_wdata,shared_rdata;
    motion_ddr_port u_motion_memory(
      .link_clk(S_csi_rx_clk),.ddr_clk(S_ddr_clk),.rst_n(S_rst_n),
      .request(motion_cmd),.write(motion_wr),.address(motion_address),.wdata(motion_wdata),
      .accept(motion_accept),.response(motion_response),.rdata(motion_rdata),
      .legacy_wr(S_ddr_user_wr_en),.legacy_rd(S_ddr_user_rd_en),
      .legacy_address(S_ddr_user_addr),.legacy_wdata(S_ddr_user_wr_data),
      .legacy_ready(S_ddr_user_ready),.legacy_rvalid(S_ddr_user_rd_valid),.legacy_rdata(S_ddr_user_rd_data),
      .user_wr(shared_wr),.user_rd(shared_rd),.user_address(shared_addr),.user_wdata(shared_wdata),
      .user_ready(shared_ready),.user_rvalid(shared_rvalid),.user_rdata(shared_rdata));

    mc_to_user_interface u_mc_to_user_interface ( .I_clk ( S_ddr_clk ),
        .I_rst_n ( S_rst_n ),
        .I_ddr_user_wr_en ( shared_wr ),
        .I_ddr_user_rd_en ( shared_rd ),
        .I_ddr_user_addr ( shared_addr ),
        .I_ddr_user_wr_data ( shared_wdata ),
        .O_ddr_user_ready ( shared_ready ),
        .O_ddr_user_rd_valid ( shared_rvalid ),
        .O_ddr_user_rd_data ( shared_rdata ),
        .O_mc_app_en ( S_mc_app_en ),
        .O_mc_app_addr ( S_mc_app_addr ),
        .O_mc_app_cmd ( S_mc_app_cmd ),
        .I_mc_app_rdy ( S_mc_app_rdy ),
        .O_mc_app_wdf_wren ( S_mc_app_wdf_wren ),
        .O_mc_app_wdf_data ( S_mc_app_wdf_data ),
        .O_mc_app_wdf_end ( S_mc_app_wdf_end ),
        .O_mc_app_wdf_mask ( S_mc_app_wdf_mask ),
        .I_mc_app_wdf_rdy ( S_mc_app_wdf_rdy ),
        .I_mc_app_rd_data ( S_mc_app_rd_data ),
        .I_mc_app_rd_data_end ( S_mc_app_rd_data_end ),
        .I_mc_app_rd_data_valid ( S_mc_app_rd_data_valid ) );

    ph1p35_324_ddr_wrapper #(.SAFE_HANDSHAKE(1)) u_ph1p35_324_ddr_wrapper ( .I_sys_clk ( I_sys_clk ),
        .I_sys_rst_n ( S_rst_n ),
        .O_ddr_clk ( S_ddr_clk ),
        .O_init_calib_complete ( S_init_calib_complete ),
        .I_mc_app_addr ( S_mc_app_addr ),
        .I_mc_app_cmd ( S_mc_app_cmd ),
        .I_mc_app_en ( S_mc_app_en ),
        .I_mc_app_wdf_data ( S_mc_app_wdf_data ),
        .I_mc_app_wdf_end ( S_mc_app_wdf_end ),
        .I_mc_app_wdf_mask ( S_mc_app_wdf_mask ),
        .I_mc_app_wdf_wren ( S_mc_app_wdf_wren ),
        .O_mc_app_rd_data ( S_mc_app_rd_data ),
        .O_mc_app_rd_data_end ( S_mc_app_rd_data_end ),
        .O_mc_app_rd_data_valid ( S_mc_app_rd_data_valid ),
        .O_mc_app_rdy ( S_mc_app_rdy ),
        .O_mc_app_wdf_rdy ( S_mc_app_wdf_rdy ),
        .ddr_addr ( ddr_addr ),
        .ddr_ba ( ddr_ba ),
        .ddr_cke ( ddr_cke ),
        .ddr_odt ( ddr_odt ),
        .ddr_cs_n ( ddr_cs_n ),
        .ddr_ras_n ( ddr_ras_n ),
        .ddr_cas_n ( ddr_cas_n ),
        .ddr_we_n ( ddr_we_n ),
        .ddr_ck_p ( ddr_ck_p ),
        .ddr_ck_n ( ddr_ck_n ),
        .ddr_dm ( ddr_dm ),
        .ddr_dq ( ddr_dq ),
        .ddr_dqs_p ( ddr_dqs_p ),
        .ddr_dqs_n ( ddr_dqs_n ) );

    uivtc # ( .H_ActiveSize ( 1280 ),
        .H_FrameSize ( 1650 ),
        .H_SyncStart ( 1390 ),
        .H_SyncEnd ( 1430 ),
        .V_ActiveSize ( 720 ),
        .V_FrameSize ( 750 ),
        .V_SyncStart ( 725 ),
        .V_SyncEnd ( 730 ) ) u_hdmi_vtc ( .I_vtc_rstn ( S_hdmi_rst_n ),
        .I_vtc_clk ( S_hdmi_pixel_clk ),
        .O_vtc_vs ( S_hdmi_vsync ),
        .O_vtc_hs ( S_hdmi_hsync ),
        .O_vtc_de_valid ( S_hdmi_de ),
        .O_vtc_user ( S_hdmi_user ),
        .O_vtc_last ( S_hdmi_last ) );

    wire [23 : 0] diagnostic_rgb;
    wire [383 : 0] a_payload;
    probe_telemetry_rx u_telemetry ( .clk ( S_csi_rx_clk ),
        .rst_n ( link_reset_pipe [2] ),
        .captured ( probe_bus ),
        .payload ( a_payload ),
        .packets ( telemetry_packets ),
        .errors ( telemetry_errors ) );

    reg [4 : 0] probe_meta, probe_status;

    // 时序/组合逻辑：按以下触发条件更新本功能状态。
    always @ ( posedge S_csi_rx_clk or negedge S_rst_n ) if ( ! S_rst_n ) begin
        probe_meta <= 0;
        probe_status <= 0;
    end
    else begin
        probe_meta <= { S_init_calib_complete, S_video_in_fifo_overflow, frame_available, link_protocol_error, I_link_run };
        probe_status <= probe_meta;
    end
    wire [319 : 0] live_link_stats = { telemetry_errors, 10'd0, last_words, last_error, error_count, bad_count, good_count, rx_frames, a_payload [319 : 288], a_payload [63 : 32], 14'd0, adapter_error, edge_request, 3'd0, probe_status, ( edge_active ? 8'h1c : 8'h13 ), telemetry_packets };
    wire [287 : 0] live_ddr_stats = { bank_debug, read_responses, read_commands, display_frames, selected_seq, reject_flags, last_written, rejected, committed };
    wire [23 : 0] live_rgb;
    // KEY1 in B original mode cycles off / frame counters / link details.
    // This HUD is downstream of DDR, so it continues updating if input freezes.
    reg [1:0] diagnostic_page,diagnostic_page_meta,diagnostic_page_sync;
    always @(posedge S_csi_rx_clk or negedge S_rst_n)
      if(!S_rst_n) diagnostic_page<=0;
      else if(!edge_request && key_press_sync!=key_press_seen)
        diagnostic_page<=(diagnostic_page==2)?0:diagnostic_page+1'b1;
    always @(posedge S_hdmi_pixel_clk or negedge S_rst_n)
      if(!S_rst_n) begin diagnostic_page_meta<=0;diagnostic_page_sync<=0;end
      else begin diagnostic_page_meta<=diagnostic_page;diagnostic_page_sync<=diagnostic_page_meta;end
    // Production display: suppress the optional bottom numeric HUD.
    mini_status_hud #(.ENABLE(0)) u_live_hud(
      .pixel_clk(S_hdmi_pixel_clk),.link_clk(S_csi_rx_clk),.ddr_clk(S_ddr_clk),.rst_n(S_rst_n),
      .link_stats({telemetry_errors,a_payload[319:288],a_payload[351:320],
        a_payload[63:32],a_payload[95:64],{last_error,10'd0,last_words},
        error_count,good_count,bad_count}),
      .ddr_stats({reject_flags,selected_seq,rejected,committed}),
      .underflows(underflows),.video_rgb(S_hdmi_out_data),
      .detail_page(diagnostic_page_sync==2),
      .de(S_hdmi_out_de),.vsync(S_hdmi_out_vsync),.rgb(diagnostic_rgb));
    assign live_rgb = (diagnostic_page_sync==1 || diagnostic_page_sync==2) ? diagnostic_rgb : S_hdmi_out_data;
    reg [1 : 0] display_valid_sync;

    // 时序/组合逻辑：按以下触发条件更新本功能状态。
    always @ ( posedge S_hdmi_pixel_clk or negedge S_rst_n ) if ( ! S_rst_n ) display_valid_sync <= 0;
    else display_valid_sync <= { display_valid_sync [0], display_valid };
    wire [23 : 0] guarded_video_data = display_valid_sync [1] ? S_video_rd_data : 24'd0;
    wire [23:0] motion_display_rgb;
    motion_display u_motion_display(
      .clk(S_hdmi_pixel_clk),.rst_n(S_hdmi_rst_n),.mode(selected_mode),
      .video_user(S_hdmi_user),.video_last(S_hdmi_last),.video_de(S_hdmi_de),
      .rgb(guarded_video_data),.rgb_out(motion_display_rgb),
      .result(motion_result),.result_toggle(motion_result_toggle),.result_ack(motion_result_ack));
    // 显示像素与同步对齐：开启OSD，原读使能及延迟不变。
    hdmi_mixer # ( .H_OFFSET ( 0 ),
        .V_OFFSET ( 0 ),
        .IMG_WIDTH ( 1280 ),
        .IMG_HEIGHT ( 720 ),
        .DEBUG_MODE ( 0 ),
        .MODE_OSD_ENABLE(1),
        .OSD_ENABLE ( 1 ) ) u_hdmi_mixer ( .I_clk ( S_hdmi_pixel_clk ),
        .I_rst_n ( S_hdmi_rst_n ),
        .I_video_vsync ( S_hdmi_vsync ),
        .I_video_hsync ( S_hdmi_hsync ),
        .I_video_de ( S_hdmi_de ),
        .I_video_user ( S_hdmi_user ),
        .I_video_last ( S_hdmi_last ),
        .I_display_mode(selected_mode),
        .I_debug_status ( S_hdmi_debug_status ),
        .O_video_rd_en ( S_hdmi_window_rd_en ),
        .I_video_rd_data ( motion_display_rgb ),
        .O_hdmi_vsync ( S_hdmi_out_vsync ),
        .O_hdmi_hsync ( S_hdmi_out_hsync ),
        .O_hdmi_de ( S_hdmi_out_de ),
        .O_hdmi_data ( S_hdmi_out_data ) );

    reg [25 : 0] live_video;

    // 时序/组合逻辑：按以下触发条件更新本功能状态。
    always @ ( posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n ) if ( ! S_hdmi_rst_n ) live_video <= 0;
    else live_video <= { S_hdmi_out_vsync, S_hdmi_out_de, live_rgb };
    // HDMI发送：保持原TMDS编码器、复位及引脚连接。
    hdmi_tx u_hdmi_tx ( .I_pixel_clk ( S_hdmi_pixel_clk ),
        .I_serial_clk ( S_hdmi_serial_clk ),
        .I_rst ( ~ S_hdmi_rst_n ),
        .I_key_in ( 1'b0 ),
        .I_edid_read_trig ( 1'b0 ),
        .O_edid_read_valid ( ),
        .O_edid_read_data ( ),
        .I_video_rgb_enable ( 1'b1 ),
        .I_video_in_vs ( live_video [25] ),
        .I_video_in_de ( live_video [24] ),
        .I_video_in_user ( 1'b0 ),
        .I_video_in_valid ( 1'b0 ),
        .I_video_in_last ( 1'b0 ),
        .O_video_in_ready ( ),
        .I_video_in_data ( live_video [23 : 0] ),
        .I_audio_valid ( 1'b0 ),
        .I_audio_left_data ( 24'd0 ),
        .I_audio_right_data ( 24'd0 ),
        .I_i2s_BCLK ( 1'b0 ),
        .I_i2s_LRCK ( 1'b0 ),
        .I_i2s_DOUT ( 1'b0 ),
        .O_ddc_scl ( ),
        .O_hdmi_clk_p ( ),
        .O_hdmi_tx_p ( ),
        .O_tmds_ch0_p ( O_tmds_ch0_p ),
        .O_tmds_ch1_p ( O_tmds_ch1_p ),
        .O_tmds_ch2_p ( O_tmds_ch2_p ),
        .O_tmds_clk_p ( O_tmds_clk_p ) );

endmodule


// Compact optional display-end diagnostics: seven counters, 64 active rows.
// Default-off selection is in the top; no video handshake or memory ownership.
module mini_status_hud #(parameter ENABLE=1)(
 input wire pixel_clk,link_clk,ddr_clk,rst_n,
 input wire [287:0] link_stats,
 input wire [127:0] ddr_stats,
 input wire [31:0] underflows,
 input wire [23:0] video_rgb,
 input wire de,vsync,detail_page,
 output reg [23:0] rgb
);
 reg vs_last,de_last;
 reg [10:0] x,y;
 wire trigger=vsync && !vs_last;
 wire [287:0] links;
 wire [127:0] memory;
 reg [31:0] underflow_snapshot;
 fault_probe_snapshot #(.WIDTH(288)) ls(link_clk,pixel_clk,rst_n,trigger,link_stats,links);
 fault_probe_snapshot #(.WIDTH(128)) ds(ddr_clk,pixel_clk,rst_n,trigger,ddr_stats,memory);
 always @(posedge pixel_clk or negedge rst_n)
  if(!rst_n) begin vs_last<=0;de_last<=0;x<=0;y<=0;underflow_snapshot<=0;end
  else begin
   vs_last<=vsync;de_last<=de;
   if(trigger) begin y<=0;underflow_snapshot<=underflows;end
   else if(de_last && !de) y<=y+1'b1;
   if(!de) x<=0;else x<=x+1'b1;
  end
 function [6:0] seven(input [3:0] n);
   case(n)
     0:seven=7'b1111110; 1:seven=7'b0110000; 2:seven=7'b1101101; 3:seven=7'b1111001;
     4:seven=7'b0110011; 5:seven=7'b1011011; 6:seven=7'b1011111; 7:seven=7'b1110000;
     8:seven=7'b1111111; 9:seven=7'b1111011; 10:seven=7'b1110111; 11:seven=7'b0011111;
     12:seven=7'b1001110; 13:seven=7'b0111101; 14:seven=7'b1001111; 15:seven=7'b1000111;
   endcase
 endfunction

 integer j;
 reg [2:0] index;
 reg [10:0] xx,dy,dx;
 reg [3:0] digit,nibble;
 reg [31:0] number;
 reg [6:0] seg;
 reg lit,ink;
 always @* begin
  if(x>=960) begin xx=x-960;index=3;end
  else if(x>=640) begin xx=x-640;index=2;end
  else if(x>=320) begin xx=x-320;index=1;end
  else begin xx=x;index=0;end
  if(y>=672) begin index=index+4;dy=y-672;end else dy=y-640;
  if(detail_page) begin
   case(index)
    0:number=links[95:64];
    1:number=links[127:96];
    2:number=links[159:128];
    3:number=links[191:160];
    4:number=links[223:192];
    5:number=links[255:224];
    6:number=links[287:256];
    default:number=0;
   endcase
  end else case(index)
   0:number=links[63:32];
   1:number=links[31:0];
   2:number=memory[31:0];
   3:number=memory[63:32];
   4:number=memory[95:64];
   5:number=memory[127:96];
   6:number=underflow_snapshot;
   default:number=0;
  endcase
  digit=0;dx=xx-40;
  for(j=1;j<8;j=j+1) if(xx>=40+j*34) begin digit=j;dx=xx-(40+j*34);end
  nibble=number>>((7-digit)*4);
  ink=xx>=40 && xx<312;
  if(xx<34) begin nibble={1'b0,index};dx=xx;ink=1;end
  seg=seven(nibble);
  lit=(seg[6] && dy>=2 && dy<5 && dx>=6 && dx<24)||
      (seg[5] && dx>=24 && dx<27 && dy>=5 && dy<15)||
      (seg[4] && dx>=24 && dx<27 && dy>=18 && dy<28)||
      (seg[3] && dy>=28 && dy<31 && dx>=6 && dx<24)||
      (seg[2] && dx>=3 && dx<6 && dy>=18 && dy<28)||
      (seg[1] && dx>=3 && dx<6 && dy>=5 && dy<15)||
      (seg[0] && dy>=15 && dy<18 && dx>=6 && dx<24);
  rgb=video_rgb;
  if(ENABLE && y>=640 && y<704 && index<7) begin
   rgb=24'h101820;
   if(lit && ink) rgb=xx<34 ? (detail_page ? 24'hffbf00 : 24'h00ffff) : 24'hffffff;
  end
 end
endmodule

// Keep motion definitions available when an older TD session saves its source list.
`include "rtl/motion_detector_ddr.v"
`include "rtl/motion_ddr_port.v"
`include "rtl/motion_display.v"
