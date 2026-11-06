// 功能说明：板间数据重组、彩色旁路、边缘及卡通处理、打包和完整帧状态对齐。

// Dense RX emits three consecutive 128-bit words per sixteen link cycles.
// Reassemble their original MSB-first stream into four 96-bit pixel groups.
// SOF waits for the previous group to drain; the link's 64-cycle SOF guard
// leaves ample room before the next frame. DDR boundary follows old pixels.

module rgb_edge_bridge # ( parameter WIDTH = 1280,
    HEIGHT = 720, REGION_GRID_X=16, REGION_GRID_Y=9, USE_DDR=1, ENABLE_GRAY=0 ) ( input wire clk,
    rst_n,
    input wire [127 : 0] i_data,
    input wire i_valid,
    i_frame,
    i_previous_good,
    edge_request,
    cartoon_request,
    gray_request,
    replay_ready, replay_drained,
    capture_ready,capture_drained,capture_complete,capture_error,memory_fault,
    mem_accept,mem_response,
    input wire [127:0] mem_rdata,
    output wire capture_begin,capture_bank,raw_valid,low_valid,mem_request,mem_wr,
    output wire [127:0] raw_data,low_data,mem_data,
    output wire [24:0] mem_address,
    output wire [127 : 0] o_data,
    output wire [2:0] o_mode, // mode of each valid output word
    output wire o_valid,
    output wire o_frame,
    output wire o_previous_good,
    output reg edge_active,
    output wire adapter_error,
    input wire motion_request,
    output wire [95:0] analysis_pixels,
    output wire analysis_valid,analysis_sof,analysis_end,analysis_good );

    reg adapter_error_base;
    reg normal_previous_good;
    localparam GROUPS = WIDTH * HEIGHT / 4;
    reg [255 : 0] build;
    reg [1 : 0] part;
    reg [383 : 0] emit;
    reg [2 : 0] left;
    reg pending, held_good, frame_bad;
    reg [31 : 0] groups;
    reg [10 : 0] x;
    reg [95 : 0] pixels;
    reg pv, pu, pl, marker, marker_good;
    wire accepting = i_valid && ! i_frame;

    // 时序/组合逻辑：按以下触发条件更新本功能状态。
    always @ ( posedge clk or negedge rst_n ) begin
        if ( ! rst_n ) begin
            build <= 0;
            part <= 0;
            emit <= 0;
            left <= 0;
            pending <= 0;
            held_good <= 0;
            frame_bad <= 0;
            groups <= 0;
            x <= 0;
            pixels <= 0;
            pv <= 0;
            pu <= 0;
            pl <= 0;
            marker <= 0;
            marker_good <= 0;
            adapter_error_base <= 0;
        end
        else begin
            pv <= 0;
            pu <= 0;
            pl <= 0;
            marker <= 0;
            if ( i_frame ) begin
                pending <= 1;
                held_good <= i_previous_good;
            end
            if ( accepting ) begin
                if ( part == 0 ) begin
                    build [255 : 128] <= i_data;
                    part <= 1;
                end
                else if ( part == 1 ) begin
                    build [127 : 0] <= i_data;
                    part <= 2;
                end
                else begin
                    part <= 0;
                    if ( left != 0 || pending ) begin
                        frame_bad <= 1;
                        adapter_error_base <= 1;
                    end
                    else begin
                        emit <= { build, i_data };
                        left <= 4;
                    end
                end
            end
            if ( left != 0 ) begin
                emit <= emit << 96;
                left <= left - 1'b1;
                if ( groups < GROUPS ) begin
                    pixels <= emit [383 : 288];
                    pv <= 1;
                    pu <= ( groups == 0 );

                    pl <= ( x == WIDTH / 4 - 1 );

                    groups <= groups + 1'b1;
                    if ( x == WIDTH / 4 - 1 ) x <= 0;
                    else x <= x + 1'b1;
                end
                else begin
                    frame_bad <= 1;
                    adapter_error_base <= 1;
                end
            end
            if ( pending && left == 0 ) begin
                pending <= 0;
                marker <= 1;
                marker_good <= held_good && ! frame_bad && groups == GROUPS && part == 0;
                part <= 0;
                groups <= 0;
                x <= 0;
                frame_bad <= 0;
            end
        end
    end
    wire [95 : 0] edge_pixels;
    wire [31:0] gray_values;
    wire [95:0] gray_pixels={{3{gray_values[31:24]}},{3{gray_values[23:16]}},
                            {3{gray_values[15:8]}},{3{gray_values[7:0]}}};
    reg gray_active;
    wire [3 : 0] cartoon_edge;
    wire [95 : 0] styled_pixels;
    wire ev, eu, el;
    // Video control comes only from the unpacker, never from an algorithm.
    // Both raw and processed data have six clocks of fixed latency.
    reg [5:0] video_valid_pipe, video_user_pipe;
    reg edge_input_active;
    wire edge_input_enable=(pv && pu) ? edge_request : edge_input_active;
    always @(posedge clk or negedge rst_n)
      if(!rst_n) begin video_valid_pipe<=0;video_user_pipe<=0;edge_input_active<=0;end
      else begin
        video_valid_pipe<={video_valid_pipe[4:0],pv};
        video_user_pipe<={video_user_pipe[4:0],pu};
        if(pv && pu) edge_input_active<=edge_request;
      end
    wire video_valid=video_valid_pipe[5], video_user=video_user_pipe[5];
    // Passive taps only: the receiver/unpacker and video timing are unchanged.
    assign analysis_pixels=pixels;
    assign analysis_valid=pv;
    assign analysis_sof=pu;
    assign analysis_end=marker;
    assign analysis_good=marker_good;
    reg motion_active;
    always @(posedge clk or negedge rst_n)
      if(!rst_n)motion_active<=0;
      else if(video_valid && video_user)motion_active<=(motion_request===1'b1);
    edge_detect_96 # ( .EXPORT_GRAY(ENABLE_GRAY), .PIPELINE_SOBEL(1), .IMG_WIDTH ( WIDTH ),
        .IMG_HEIGHT ( HEIGHT ) ) edge_pipe ( .I_clk ( clk ),
        .I_rst_n ( rst_n ),
        .I_tlast ( pl ),
        .I_tuser ( pu && edge_input_enable ),
        .I_tdata ( edge_input_enable ? pixels : 96'd0 ),
        .I_tvalid ( pv && edge_input_enable ),
        .I_tready ( 1'b1 ),
        .O_tlast ( el ),
        .O_tuser ( eu ),
        .O_gray_data(gray_values),
        .O_tdata ( edge_pixels ),
        .O_tvalid ( ev ),
        .O_tready ( ),
        .O_cartoon_edge ( cartoon_edge ) );

    reg [95 : 0] color [0 : 5];
    integer j;

    // 时序/组合逻辑：按以下触发条件更新本功能状态。
    always @ ( posedge clk or negedge rst_n ) if ( ! rst_n ) begin
        for ( j = 0; j < 6; j = j + 1 ) color [j] <= 0;
        edge_active <= 0;
        gray_active <= 0;
    end
    else begin
        color [0] <= pixels;
        for ( j = 1; j < 6; j = j + 1 ) color [j] <= color [j - 1];
        if ( video_valid && video_user ) begin
            edge_active <= edge_request;
            gray_active <= ENABLE_GRAY && gray_request;
        end
    end
    wire choose_edge = ( video_valid && video_user ) ? edge_request : edge_active;
    wire choose_gray = ( video_valid && video_user ) ? (ENABLE_GRAY && gray_request) : gray_active;
    wire unused_frame;
    wire normal_valid;
    wire [127:0] normal_data;
    rgb96_pack pack ( clk,
    rst_n,
    video_valid && video_user,
    video_valid,
    choose_edge ? (choose_gray ? gray_pixels : edge_pixels) : color [5],
    unused_frame,
    normal_valid,
    normal_data );

    // Separate DDR SOF from algorithm SOF: it must exist for rejected/empty
    // frames too, and arrive after old algorithm/packer output has drained.
    reg [9 : 0] frame_pipe, good_pipe;
    wire normal_frame = frame_pipe[9];

    // 时序/组合逻辑：按以下触发条件更新本功能状态。
    always @ ( posedge clk or negedge rst_n ) if ( ! rst_n ) begin
        frame_pipe <= 0;
        good_pipe <= 0;
        normal_previous_good <= 0;
    end
    else begin
        frame_pipe <= { frame_pipe [8 : 0], marker };
        good_pipe <= { good_pipe [8 : 0], marker_good };
        // Prepare good status a cycle before the consuming DDR input sees SOF.
        if ( frame_pipe [8] ) normal_previous_good <= good_pipe [8];
    end
    // Frame arbiter: finish a complete replay before enabling the next bypass frame.
    wire want_cartoon=edge_request && cartoon_request;
    wire region_ready,region_rendering,region_fault;
    wire [95:0] region_pixels;
    wire region_valid,region_frame,region_good;
    wire region_launch=normal_frame && want_cartoon && region_ready && !region_rendering;
    reg bypass_on;
    always @(posedge clk or negedge rst_n)
      if(!rst_n) bypass_on<=1;
      else if(normal_frame && !region_rendering) bypass_on<=!want_cartoon;
    wire old_ready,old_render,old_valid,old_frame,old_good,old_fault;
    wire [95:0] old_pixels;
    region_cartoon_160 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.GRID_X(REGION_GRID_X),.GRID_Y(REGION_GRID_Y)) u_region(
      .clk(clk),.rst_n(rst_n),.enable(want_cartoon && !USE_DDR),.cancel(normal_frame && !want_cartoon),
      .pixels(pixels),.valid(pv),.sof(pu),.frame_end(marker),.frame_good(marker_good),
      .launch(region_launch && !USE_DDR),.output_ready(replay_ready),.output_drained(replay_drained),
      .ready(old_ready),.rendering(old_render),.out_pixels(old_pixels),.out_valid(old_valid),
      .out_frame(old_frame),.out_good(old_good),.fault(old_fault),.completed(),.skipped(),.processing_cycles());
    wire new_ready,new_render,new_valid,new_frame,new_good,new_fault;
    wire [95:0] new_pixels;
    wire cb,cbank,rv,lv,mq,mw;wire [127:0] rdata,ldata,mdata;wire [24:0] maddr;
    region_cartoon_stream #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) u_region_stream(
      .clk(clk),.rst_n(rst_n),.enable(want_cartoon && USE_DDR),
      .pixels(pixels),.valid(pv),.sof(pu),.frame_end(marker),.frame_good(marker_good),
      .output_ready(replay_ready),.output_drained(replay_drained),
      .out_pixels(new_pixels),.out_valid(new_valid),.out_frame(new_frame),.out_good(new_good),
      .rendering(new_render),.fault(new_fault),.completed());
    assign new_ready=0;
    assign cb=0;assign cbank=0;assign rv=0;assign lv=0;assign mq=0;assign mw=0;
    assign rdata=0;assign ldata=0;assign mdata=0;assign maddr=0;
    assign capture_begin=USE_DDR && cb;assign capture_bank=cbank;
    assign raw_valid=USE_DDR && rv;assign low_valid=USE_DDR && lv;
    assign raw_data=rdata;assign low_data=ldata;
    assign mem_request=USE_DDR && mq;assign mem_wr=mw;assign mem_address=maddr;assign mem_data=mdata;
    assign region_ready=USE_DDR?new_ready:old_ready;
    assign region_rendering=USE_DDR?new_render:old_render;
    assign region_valid=USE_DDR?new_valid:old_valid;
    assign region_frame=USE_DDR?new_frame:old_frame;
    assign region_good=USE_DDR?new_good:old_good;
    assign region_pixels=USE_DDR?new_pixels:old_pixels;
    assign region_fault=USE_DDR?(new_fault || memory_fault):old_fault;
    wire [127:0] region_data;
    wire region_packed_valid,unused_region_frame;
    rgb96_pack region_pack(clk,rst_n,region_frame,region_valid,region_pixels,
      unused_region_frame,region_packed_valid,region_data);
    wire emit_normal_frame=normal_frame && !region_rendering && !want_cartoon;
    assign o_frame=region_frame || emit_normal_frame;
    assign o_previous_good=region_frame ? region_good : (normal_previous_good && bypass_on);
    assign o_valid=region_packed_valid || (normal_valid && bypass_on && !region_rendering && !want_cartoon);
    assign o_data=region_packed_valid ? region_data : normal_data;
    assign o_mode=region_packed_valid ? 3'd4 : (motion_active ? 3'd5 : (edge_active ? (gray_active ? 3'd3 : 3'd2) : 3'd1));
    assign adapter_error=adapter_error_base || region_fault;
endmodule

