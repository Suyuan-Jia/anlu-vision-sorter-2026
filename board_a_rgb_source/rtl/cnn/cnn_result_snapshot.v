// Atomic result publication at SOF. All coordinates are full-video pixels.
// There are no outputs to video flow control. The analysis can be much slower
// than video: hold a coherent record for MAX_AGE video frames, then expire it.
module cnn_result_snapshot #(
 parameter WIDTH=1280,HEIGHT=720,MAX_AGE=60,LOCK_DISTANCE=256
)(
 input wire clk,rst_n,enable_frame,frame,
 input wire [31:0] video_frame_id,
 input wire result_update,result_valid,
 input wire [31:0] result_frame_id,
 input wire [1:0] result_shape,
 input wire [2:0] result_color,
 input wire [10:0] result_xmin,result_ymin,result_xmax,result_ymax,result_cx,result_cy,
 output wire visible,locked,
 output wire [1:0] shape,
 output wire [2:0] color_id,
 output wire [10:0] xmin,ymin,xmax,ymax,cx,cy,
 output wire [31:0] shown_frame_id
);
 reg frame_d,pvalid,plocked,dvalid,dlocked,candidate_valid;
 reg [1:0] ps,ds;
 reg [2:0] pc,dc;
 reg [10:0] px0,py0,px1,py1,pcx,pcy,dx0,dy0,dx1,dy1,dcx,dcy;
 reg [31:0] pid,did;
 wire sof=frame && !frame_d;
 // video_frame_id is the capture observer's counter before this sampling edge.
 wire [31:0] current_id=video_frame_id+(sof ? 32'd1 : 32'd0);
 wire pending_fresh=(current_id-pid)<=MAX_AGE;
 wire incoming_fresh=(current_id-result_frame_id)<=MAX_AGE;
 wire geometry_ok=result_xmin<=result_cx && result_cx<=result_xmax &&
                  result_ymin<=result_cy && result_cy<=result_ymax &&
                  result_xmax<WIDTH && result_ymax<HEIGHT;
 wire [10:0] move_x=result_cx>pcx ? result_cx-pcx : pcx-result_cx;
 wire [10:0] move_y=result_cy>pcy ? result_cy-pcy : pcy-result_cy;
 wire incoming_ok=result_valid && result_shape!=0 && result_color<=4 && geometry_ok && incoming_fresh;
 wire consistent=candidate_valid && pending_fresh && result_frame_id!=pid && result_shape==ps && move_x<=LOCK_DISTANCE && move_y<=LOCK_DISTANCE;
 assign visible=rst_n && enable_frame && (sof ? pvalid && pending_fresh : dvalid);
 assign locked=visible && (sof ? plocked : dlocked);
 assign shape=sof ? ps : ds;
 assign color_id=sof ? pc : dc;
 assign xmin=sof ? px0 : dx0;assign ymin=sof ? py0 : dy0;
 assign xmax=sof ? px1 : dx1;assign ymax=sof ? py1 : dy1;
 assign cx=sof ? pcx : dcx;assign cy=sof ? pcy : dcy;
 assign shown_frame_id=sof ? pid : did;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   candidate_valid<=0;frame_d<=0;pvalid<=0;plocked<=0;dvalid<=0;dlocked<=0;
   ps<=0;ds<=0;pc<=0;dc<=0;pid<=0;did<=0;
   px0<=0;py0<=0;px1<=0;py1<=0;pcx<=0;pcy<=0;
   dx0<=0;dy0<=0;dx1<=0;dy1<=0;dcx<=0;dcy<=0;
  end else begin
   frame_d<=frame;
   if(!enable_frame)begin candidate_valid<=0;pvalid<=0;dvalid<=0;plocked<=0;dlocked<=0;end
   else begin
    if(sof)begin
     dvalid<=pvalid && pending_fresh;dlocked<=plocked;
     ds<=ps;dc<=pc;did<=pid;dx0<=px0;dy0<=py0;dx1<=px1;dy1<=py1;dcx<=pcx;dcy<=pcy;
     if(!pending_fresh)begin candidate_valid<=0;pvalid<=0;plocked<=0;end
    end
    // Coincident result/SOF is intentionally staged for the following frame.
    if(result_update)begin
     candidate_valid<=incoming_ok;
     pvalid<=incoming_ok && consistent;
     plocked<=incoming_ok && consistent;
     ps<=result_shape;pc<=result_color;pid<=result_frame_id;
     px0<=result_xmin;py0<=result_ymin;px1<=result_xmax;py1<=result_ymax;pcx<=result_cx;pcy<=result_cy;
    end
   end
  end
 end
endmodule
