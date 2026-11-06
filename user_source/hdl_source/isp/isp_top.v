module isp_top #(
    // NMS magnitude threshold for cartoon black lines only (compile-time).
    parameter [11:0] CARTOON_EDGE_TH = 12'd100
)(
    input 			axi4s_video_aclk,
    input 			I_rst_n			,
    input 			I_tlast			,
    input 			I_tuser			,
    input [39:0] 	I_tdata			,
    input 			I_tvalid		,
    input [9:0] 	I_tdest			,
    input 			O_tready		,
    output [127:0] 	O_tdata			,
    output 			O_tlast			,
    output 			O_tuser			,
    output 			O_tvalid		,
    output 			I_tready,
    input           I_alg_sel_req   ,
    input           I_cartoon_req
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

    wire         edge_O_tlast;  //synthesis keep
    wire         edge_O_tuser;  //synthesis keep
    wire [ 95:0] edge_O_tdata;  //synthesis keep
    wire         edge_O_tvalid;  //synthesis keep
    wire [3:0]   cartoon_edge;

    // Color AWB path is delayed by four clocks to match the registered
    // edge_detect_96 output. Both streams then use identical 96->128
    // converters, making their frame-start/data/valid timing equivalent.
    reg [3:0] color_tlast_pipe;
    reg [3:0] color_tuser_pipe;
    reg [3:0] color_tvalid_pipe;

    reg [95:0] color_tdata_d1;
    reg [95:0] color_tdata_d2;
    reg [95:0] color_tdata_d3;
    reg [95:0] color_tdata_d4;

    wire         color_O_tuser_128;
    wire         color_O_tvalid_128;
    wire [127:0] color_O_tdata_128;

    reg          color_O_tlast_128;
    reg          edge_O_tlast_128;

    reg          alg_sel_active;
    wire         paths_frame_start_aligned;
    wire         alg_sel_mux;

    // Keep both paths running continuously.
    // edge_detect_96 contributes four registered cycles; mirror that delay
    // on the color path.
    always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            color_tlast_pipe <= 4'b0000;
            color_tuser_pipe <= 4'b0000;
            color_tvalid_pipe <= 4'b0000;
    
            color_tdata_d1 <= 96'd0;
            color_tdata_d2 <= 96'd0;
            color_tdata_d3 <= 96'd0;
            color_tdata_d4 <= 96'd0;
    
            color_O_tlast_128 <= 1'b0;
            edge_O_tlast_128  <= 1'b0;
        end
        else begin
            color_tlast_pipe <= {
                color_tlast_pipe[2:0],
                awb_O_tlast
            };
    
            color_tuser_pipe <= {
                color_tuser_pipe[2:0],
                awb_O_tuser
            };
    
            color_tvalid_pipe <= {
                color_tvalid_pipe[2:0],
                awb_O_tvalid
            };
    
            if (awb_O_tvalid)
                color_tdata_d1 <= awb_O_tdata;
            else
                color_tdata_d1 <= 96'd0;
    
            color_tdata_d2 <= color_tdata_d1;
            color_tdata_d3 <= color_tdata_d2;
            color_tdata_d4 <= color_tdata_d3;
    
            // 与两个96→128转换器的输出寄存时钟对齐
            color_O_tlast_128 <= color_tlast_pipe[3];
            edge_O_tlast_128  <= edge_O_tlast;
        end
    end

    // After the four-cycle compensation above, these two frame-starts should
    // occur together. Requiring both provides a fail-safe boundary.
    // Both paths have already received the required latency compensation
    // and identical 96->128 conversion here.  Use their final frame-start
    // signals as the safe switching boundary.
    assign paths_frame_start_aligned =
        color_O_tuser_128 && awb_O_tuser_128;

    // On the frame-start cycle itself, use the new requested mode
    // immediately.  alg_sel_active then retains it for the rest of the frame.
    assign alg_sel_mux =
        paths_frame_start_aligned ? I_alg_sel_req : alg_sel_active;

    // Never drive the video MUX directly from the asynchronous switch.
    // I_alg_sel_req is already synchronized/debounced in the top level.
    // Commit it only at the aligned output frame boundary.
    always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
        if (!I_rst_n)
            alg_sel_active <= 1'b0;
        else if (paths_frame_start_aligned)
            alg_sel_active <= I_alg_sel_req;
    end

    // 0 = color AWB image
    // 1 = verified official Sobel image
    assign O_tdata  = alg_sel_mux ? awb_O_tdata_128  : color_O_tdata_128;
    assign O_tlast  = alg_sel_mux ? edge_O_tlast_128 : color_O_tlast_128;
    assign O_tuser  = alg_sel_mux ? awb_O_tuser_128  : color_O_tuser_128;
    assign O_tvalid = alg_sel_mux ? awb_O_tvalid_128 : color_O_tvalid_128;
  //assign m_aixs_tready = O_tready;
  //assign O_tdata  = m_aixs_tdata;
  //assign O_tlast  = m_aixs_tlast    ;
  //assign O_tuser  = m_aixs_tuser    ;
  //assign O_tvalid = m_aixs_tvalid   ;
 
 
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

edge_detect_96 #(
    .IMG_WIDTH (1280),
    .IMG_HEIGHT(720),
    .EDGE_TH   (11'd120),
    .CARTOON_EDGE_TH(CARTOON_EDGE_TH)
) u_edge_detect_96 (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tlast (awb_O_tlast),
    .I_tuser (awb_O_tuser),
    .I_tdata (awb_O_tdata),
    .I_tvalid(awb_O_tvalid),
    .I_tready(O_tready),
    .O_tlast (edge_O_tlast),
    .O_tuser (edge_O_tuser),
    .O_tdata (edge_O_tdata),
    .O_tvalid(edge_O_tvalid),
    .O_tready(awb_O_tready),
    .O_cartoon_edge(cartoon_edge)
);

wire [95:0] edge_style_data;

// 两条输入分别来自原始白平衡输出和现有边缘检测输出。
// 模块内部完成 RGB 3x3 均值滤波、三位颜色量化、空间对齐和帧级模式选择。
cartoon_overlay_96 #(
    .IMG_WIDTH  (1280),
    .IMG_HEIGHT (720)
) u_cartoon_overlay_96 (
    .I_clk          (axi4s_video_aclk),
    .I_rst_n        (I_rst_n),
    .I_ce           (O_tready),

    .I_color_data   (awb_O_tdata),
    .I_color_valid  (awb_O_tvalid),
    .I_color_user   (awb_O_tuser),
    .I_color_last   (awb_O_tlast),

    .I_edge_data    (edge_O_tdata),
    .I_cartoon_edge (cartoon_edge),
    .I_cartoon_req  (I_cartoon_req),

    .O_data         (edge_style_data)
);

    // Color path converter. Its input has already been delayed four clocks,
    // matching edge_detect_96 output timing.
    data_96bit_to_128bit u_color_data_96bit_to_128bit(
        .I_clk              ( axi4s_video_aclk   ),
        .I_rst_n            ( I_rst_n            ),

        .I_96b_frame_start  (color_tuser_pipe[3]),
        .I_96b_valid        (color_tvalid_pipe[3]),
        .I_96b_data         (color_tdata_d4),

        .O_128b_frame_start ( color_O_tuser_128  ),
        .O_128b_valid       ( color_O_tvalid_128 ),
        .O_128b_data        ( color_O_tdata_128  )
    );

    data_96bit_to_128bit u_data_96bit_to_128bit (
        .I_clk              (axi4s_video_aclk),
        .I_rst_n            (I_rst_n),
    
        // 同步和有效信号继续使用现有边缘模块的输出。
        .I_96b_frame_start  (edge_O_tuser),
        .I_96b_valid        (edge_O_tvalid),
    
        // 数据改为纯边缘或量化叠边的选择结果。
        .I_96b_data         (edge_style_data),
    
        .O_128b_frame_start (awb_O_tuser_128),
        .O_128b_valid       (awb_O_tvalid_128),
        .O_128b_data        (awb_O_tdata_128)
    );



//data96_128 u_data96_128 (
//    .I_tdata(awb_O_tdata),
//    .O_tdata(awb_O_tdata_128)
//);
	
endmodule
