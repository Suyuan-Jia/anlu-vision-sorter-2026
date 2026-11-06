// Read-only analysis tap. There is deliberately no video ready/stall output.
// 1280x720 RGB888, four MSB-first pixels/beat -> 160x90 area-average RGB565.
// A captured frame stays owned by inference until release. Busy frames are skipped.
module cnn_capture_4px #(
 parameter WIDTH=1280, HEIGHT=720
)(
 input wire clk,rst_n,enable_frame,frame,valid,
 input wire [95:0] rgb,
 input wire release_frame,
 output reg wr,
 output reg [13:0] wr_addr,
 output reg [15:0] wr_rgb565,
 output reg frame_pending,
 output reg [31:0] captured_frame_id,
 output reg [31:0] video_frame_id,
 output reg [31:0] skipped_frames,
 output reg [31:0] malformed_frames
);
 localparam SMALL_W=WIDTH/8, SMALL_H=HEIGHT/8;
 reg frame_d,active,capturing;
 reg [11:0] next_x,next_y;
 wire sof=frame && !frame_d;
 wire [11:0] x=sof ? 12'd0 : next_x;
 wire [11:0] y=sof ? 12'd0 : next_y;
 wire take=valid && (active || sof);
 wire start_capture=sof && enable_frame && !frame_pending;
 wire use_capture=sof ? start_capture : capturing;
 reg [10:0] half_r,half_g,half_b;
 reg [13:0] row_r[0:SMALL_W-1],row_g[0:SMALL_W-1],row_b[0:SMALL_W-1];
 wire [9:0] four_r={2'b0,rgb[95:88]}+{2'b0,rgb[71:64]}+{2'b0,rgb[47:40]}+{2'b0,rgb[23:16]};
 wire [9:0] four_g={2'b0,rgb[87:80]}+{2'b0,rgb[63:56]}+{2'b0,rgb[39:32]}+{2'b0,rgb[15:8]};
 wire [9:0] four_b={2'b0,rgb[79:72]}+{2'b0,rgb[55:48]}+{2'b0,rgb[31:24]}+{2'b0,rgb[7:0]};
 wire [13:0] sum_r={3'b0,half_r}+four_r+(y[2:0]==0 ? 14'd0 : row_r[x>>3]);
 wire [13:0] sum_g={3'b0,half_g}+four_g+(y[2:0]==0 ? 14'd0 : row_g[x>>3]);
 wire [13:0] sum_b={3'b0,half_b}+four_b+(y[2:0]==0 ? 14'd0 : row_b[x>>3]);
 wire [14:0] rounded_r={1'b0,sum_r}+15'd32,rounded_g={1'b0,sum_g}+15'd32,rounded_b={1'b0,sum_b}+15'd32;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin
   frame_d<=0;active<=0;capturing<=0;next_x<=0;next_y<=0;
   half_r<=0;half_g<=0;half_b<=0;wr<=0;wr_addr<=0;wr_rgb565<=0;
   frame_pending<=0;captured_frame_id<=0;video_frame_id<=0;skipped_frames<=0;malformed_frames<=0;
  end else begin
   frame_d<=frame;wr<=0;
   if(release_frame)frame_pending<=0;
   if(sof)begin
    video_frame_id<=video_frame_id+1'b1;
    active<=1;next_x<=0;next_y<=0;capturing<=start_capture;
    if(active)malformed_frames<=malformed_frames+1'b1;
    if(enable_frame && frame_pending)skipped_frames<=skipped_frames+1'b1;
   end
   if(take)begin
    if(x==WIDTH-4)begin
     next_x<=0;
     if(y==HEIGHT-1)begin next_y<=0;active<=0;capturing<=0;end
     else next_y<=y+1'b1;
    end else next_x<=x+4;
    if(use_capture)begin
     if(!x[2])begin half_r<=four_r;half_g<=four_g;half_b<=four_b;end
     else begin
      row_r[x>>3]<=sum_r;row_g[x>>3]<=sum_g;row_b[x>>3]<=sum_b;
      if(y[2:0]==7)begin
       wr<=1;wr_addr<=(y>>3)*SMALL_W+(x>>3);
       wr_rgb565<={rounded_r[13:9],rounded_g[13:8],rounded_b[13:9]};
       if(x==WIDTH-4 && y==HEIGHT-1)begin
        frame_pending<=1;captured_frame_id<=sof ? video_frame_id+1'b1 : video_frame_id;
       end
      end
     end
    end
   end
  end
 end
 // synthesis translate_off
 initial if(WIDTH%8!=0 || HEIGHT%8!=0 || WIDTH>4096 || HEIGHT>4096 || SMALL_W*SMALL_H>16384)
  $fatal(1,"unsupported CNN capture geometry");
 // synthesis translate_on
endmodule
