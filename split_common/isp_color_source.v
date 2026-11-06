// Board A camera ISP and RGB packer. Legacy target recognition is removed.
// Demosaic, white balance, stream control, and video probes retain their wiring.
module isp_color_source (
    input axi4s_video_aclk,
    input I_rst_n,
    input I_tlast,
    input I_tuser,
    input [39:0] I_tdata,
    input I_tvalid,
    input [9:0] I_tdest,
    input O_tready,
    output [127:0] O_tdata,
    output O_tlast,
    output O_tuser,
    output O_tvalid,
    output I_tready,
    input I_recognition_enable,
    input I_alg_sel_req,
    input I_cartoon_req,
    output wire probe_active, probe_aligned, probe_color_frame, probe_edge_frame
);

wire         m_aixs_tvalid;  //synthesis keep 
  wire [127:0] m_aixs_tdata;  //synthesis keep 
  wire         m_aixs_tuser;  //synthesis keep 
  wire         m_aixs_tlast;  //synthesis keep 
  wire         m_aixs_tready;  //synthesis keep 
  
  wire [ 95:0] m_aixs_tdata_96;  //synthesis keep 


  wire         awb_O_tlast;  //synthesis keep 
  wire         awb_O_tuser;  //synthesis keep 
  wire [ 95:0] awb_O_tdata;  //synthesis keep 
  wire         awb_O_tvalid;  //synthesis keep 
  wire         awb_O_tready;  //synthesis keep 

  wire         awb_O_tuser_128;  //synthesis keep 
  wire         awb_O_tvalid_128;  //synthesis keep 
  wire [127:0] awb_O_tdata_128;  //synthesis keep 

demosaic #(
    .IMG_HEIGHT          (720),  // 图像高度
    .IMG_WIDTH           (1280), // 图像宽度
    .data_complete_delay (50  ),
    .BAYER_MODE          ("BGGR")
)
u_demosaic
(
    .I_clk   			(axi4s_video_aclk)	,   // 时钟信号
    .I_rst_n 			(I_rst_n)	,   // 复位信号，低有效
    .axi4s_video_tdata 	(I_tdata)	,  // AXI4-Stream视频数据
    .axi4s_video_tdest 	(I_tdest)	,
    .axi4s_video_tlast 	(I_tlast)	,  // 行结束信号
    .axi4s_video_tvalid	(I_tvalid)	,  // 数据有效信号
    .axi4s_video_tuser 	(I_tuser)	,  // 帧开始信号
    .axi4s_video_tready	(I_tready)	,  // 从模块准备好接受数据
    .O_tlast  			(m_aixs_tlast)	,  // 输出行结束信号
    .O_tuser  			(m_aixs_tuser)	,  // 输出帧开始信号
    .O_tdata  			(m_aixs_tdata)	,  // 输出数据
    .O_tvalid 			(m_aixs_tvalid)	,  // 输出数据有效信号
    .O_tready   		(m_aixs_tready)	   // 输出数据准备好信号
);
 
data128_96 u_data128_96 (
    .I_tdata(m_aixs_tdata),
    .O_tdata(m_aixs_tdata_96)
);

awb #(
    .IMG_HEIGHT(720),
    .IMG_WIDTH (1280)
) u_awb (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tlast (m_aixs_tlast),
    .I_tuser (m_aixs_tuser),
    .I_tdata (m_aixs_tdata_96),
    .I_tvalid(m_aixs_tvalid),
    .I_tready(m_aixs_tready),
    .O_tlast (awb_O_tlast ),
    .O_tuser (awb_O_tuser ),
    .O_tdata (awb_O_tdata ),
    .O_tvalid(awb_O_tvalid),
    .O_tready(awb_O_tready)
);


 assign awb_O_tready=O_tready;
 assign probe_active=1'b0;
 assign probe_aligned=awb_O_tuser;
 assign probe_color_frame=O_tuser;
 assign probe_edge_frame=1'b0;
 assign O_tlast=1'b0; // GPIO framing uses SOF plus exact payload length.
 // Read-only CNN analysis tap. Overlay RGB, SOF and valid share two clocks
 // latency in BOTH modes. AWB ready and stream cadence never depend on CNN.
 wire [95:0] rgb_to_pack;
 wire frame_to_pack,valid_to_pack;
 wire recognition_mode,inference_busy;
 wire [31:0] recognition_skipped_frames;
 cnn_recognition_branch recognition(
  .clk(axi4s_video_aclk),.rst_n(I_rst_n),.enable_async(I_recognition_enable),
  .frame(awb_O_tuser),.valid(awb_O_tvalid),.rgb(awb_O_tdata),.rgb_out(rgb_to_pack),
  .recognition_mode(recognition_mode),.inference_busy(inference_busy),.skipped_frames(recognition_skipped_frames),
  .frame_out(frame_to_pack),.valid_out(valid_to_pack));
 rgb96_pack pack(axi4s_video_aclk,I_rst_n,frame_to_pack,valid_to_pack,rgb_to_pack,
                 O_tuser,O_tvalid,O_tdata);
endmodule
