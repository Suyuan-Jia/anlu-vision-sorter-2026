// Passive analysis with a fixed-latency display pipeline. Analysis busy/done
// never controls video valid or SOF. Both display modes have two clocks latency.
module cnn_recognition_branch(
 input wire clk,rst_n,enable_async,frame,valid,
 input wire [95:0] rgb,
 output wire [95:0] rgb_out,
 output wire recognition_mode,inference_busy,
 output wire [31:0] skipped_frames,
 output wire frame_out,valid_out
);
 reg [1:0] frame_pipe,valid_pipe;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin frame_pipe<=0;valid_pipe<=0;end
  else begin frame_pipe<={frame_pipe[0],frame};valid_pipe<={valid_pipe[0],valid};end
 end
 assign frame_out=frame_pipe[1];
 assign valid_out=valid_pipe[1];
 reg mode_meta,mode_sync,candidate,mode_latched,mode_frame_d;
 reg [1:0] stable_frames;
 wire sof=frame && !mode_frame_d;
 wire confirmed=mode_sync==candidate && stable_frames>=2;
 assign recognition_mode=rst_n && (sof && confirmed ? candidate : mode_latched);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin mode_meta<=0;mode_sync<=0;candidate<=0;stable_frames<=0;mode_latched<=0;mode_frame_d<=0;end
  else begin
   mode_meta<=enable_async;mode_sync<=mode_meta;mode_frame_d<=frame;
   if(sof)begin
    if(mode_sync!=candidate)begin candidate<=mode_sync;stable_frames<=1;end
    else if(stable_frames<3)stable_frames<=stable_frames+1'b1;
    if(confirmed)mode_latched<=candidate;
   end
  end
 end
 wire capture_wr,pending,post_busy,post_done,nn_done;
 reg mode_previous,pending_previous;
 reg [7:0] display_epoch,analysis_epoch;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin mode_previous<=0;pending_previous<=0;display_epoch<=0;analysis_epoch<=0;end
  else begin
   mode_previous<=recognition_mode;pending_previous<=pending;
   if(recognition_mode && !mode_previous)display_epoch<=display_epoch+1'b1;
   if(pending && !pending_previous)analysis_epoch<=display_epoch;
  end
 end
 // Ignore an in-flight result from an earlier recognition-mode session.
 // At three-frame debounce and MAX_AGE=180, 8 bits cannot wrap within a
 // still-fresh record's lifetime, even with repeated switch toggles.
 wire [7:0] active_epoch=recognition_mode && !mode_previous ? display_epoch+8'd1 : display_epoch;
 wire current_session=analysis_epoch==active_epoch;
 wire [13:0] capture_addr;wire [15:0] capture_rgb;
 wire [31:0] captured_frame_id,video_frame_id,malformed_frames,inference_cycles;
 cnn_capture_4px capture(clk,rst_n,recognition_mode,frame,valid,rgb,post_done,
  capture_wr,capture_addr,capture_rgb,pending,captured_frame_id,video_frame_id,skipped_frames,malformed_frames);
 wire [1:0] predicted_shape,identity_quality;
 wire [2:0] service_bank,service_write_bank;
 wire [14:0] service_addr,service_write_addr;
 wire [15:0] service_data,service_write_data;
 wire service_write;
 cnn_integer_core core(clk,rst_n,pending && !inference_busy && !post_busy,post_done,
  capture_wr,capture_addr,capture_rgb,inference_busy,nn_done,predicted_shape,inference_cycles,
  service_bank,service_addr,service_data,service_write,service_write_bank,service_write_addr,service_write_data,identity_quality);
 wire result_valid;wire [2:0] result_color;wire [7:0] seed_r,seed_g;wire [9:0] seed_sum;wire seed_valid;
 wire [10:0] x0,y0,x1,y1,xc,yc;wire [15:0] result_area;
 cnn_postprocess postprocess(clk,rst_n,nn_done,predicted_shape,post_busy,post_done,result_valid,result_color,
  x0,y0,x1,y1,xc,yc,result_area,service_bank,service_addr,service_data,service_write,service_write_bank,service_write_addr,service_write_data,seed_r,seed_g,seed_valid,seed_sum);
 wire visible,locked;wire [1:0] shown_shape;wire [2:0] shown_color;
 wire [10:0] sx0,sy0,sx1,sy1,scx,scy;wire [31:0] shown_frame_id;
 wire coarse_visible,coarse_locked;wire [1:0] coarse_shape;wire [2:0] coarse_color;
 wire [10:0] qx0,qy0,qx1,qy1,qcx,qcy;wire tracking;wire [31:0] coarse_frame_id;
 // Slow CNN certifies identity; the passive RGB tap refines current positions.
 // Analysis remains a passive tap and cannot stall the video stream.
 cnn_result_snapshot #(.MAX_AGE(24)) snapshot(clk,rst_n,recognition_mode,frame,video_frame_id,
  post_done && current_session,result_valid && (identity_quality!=0 || tracking),captured_frame_id,predicted_shape,result_color,x0,y0,x1,y1,xc,yc,
  coarse_visible,coarse_locked,coarse_shape,coarse_color,qx0,qy0,qx1,qy1,qcx,qcy,coarse_frame_id);
 cnn_live_tracker tracker(
  .clk(clk),.rst_n(rst_n),.enable(recognition_mode),.frame(frame),.valid(valid),.rgb(rgb),.video_id(video_frame_id),
  .result_update(post_done && current_session),.result_valid(result_valid && (identity_quality!=0 || tracking)),.seed_valid(seed_valid),
  .result_id(captured_frame_id),.result_shape(identity_quality==0 && tracking ? shown_shape : predicted_shape),.result_color(result_color),
  .result_x0(x0),.result_y0(y0),.result_x1(x1),.result_y1(y1),.result_area(result_area),
  .seed_r(seed_r),.seed_g(seed_g),.seed_sum(seed_sum),
  .coarse_visible(coarse_visible),.coarse_locked(coarse_locked),.coarse_shape(coarse_shape),.coarse_color(coarse_color),
  .coarse_x0(qx0),.coarse_y0(qy0),.coarse_x1(qx1),.coarse_y1(qy1),.coarse_cx(qcx),.coarse_cy(qcy),
  .coarse_frame_id(coarse_frame_id),.position_frame_id(shown_frame_id),.result_very_strong(identity_quality==2),
  .visible(visible),.locked(locked),.shape(shown_shape),.color_id(shown_color),
  .xmin(sx0),.ymin(sy0),.xmax(sx1),.ymax(sy1),.cx(scx),.cy(scy),.tracking(tracking));
 cnn_osd_4px #(.PIPELINE(1),.AIM_SIZE(320),.AIM_HEIGHT(240)) display(clk,rst_n,frame,valid,recognition_mode,rgb,visible,locked,
  shown_shape,shown_color,sx0,sy0,sx1,sy1,scx,scy,rgb_out);
endmodule
