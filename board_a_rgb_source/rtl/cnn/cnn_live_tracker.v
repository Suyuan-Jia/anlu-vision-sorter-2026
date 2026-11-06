// CNN-authorized full-row RGB boundary refinement. This module is a passive
// tap: no ready signal, no frame RAM, no connection to video flow control.
// Chromaticity AND brightness come from the accepted same-frame CNN component.
(* dsp_style="no" *) module cnn_live_tracker #(
 parameter WIDTH=1280,HEIGHT=720,SEARCH_MARGIN=96,MAX_CERT_AGE=90,TOLERANCE=12,COAST_FRAMES=2,EDGE_PAD_X=4,EDGE_PAD_Y=2
)(
 input wire clk,rst_n,enable,frame,valid,
 input wire [95:0] rgb,
 input wire [31:0] video_id,
 input wire result_update,result_valid,seed_valid,
 input wire [31:0] result_id,
 input wire [1:0] result_shape,
 input wire [2:0] result_color,
 input wire [10:0] result_x0,result_y0,result_x1,result_y1,
 input wire [15:0] result_area,
 input wire [7:0] seed_r,seed_g,
 input wire [9:0] seed_sum,
 input wire coarse_visible,coarse_locked,
 input wire [1:0] coarse_shape,
 input wire [2:0] coarse_color,
 input wire [10:0] coarse_x0,coarse_y0,coarse_x1,coarse_y1,coarse_cx,coarse_cy,
 input wire [31:0] coarse_frame_id,
 output wire visible,locked,
 output wire [1:0] shape,
 output wire [2:0] color_id,
 output wire [10:0] xmin,ymin,xmax,ymax,cx,cy,
 output wire tracking,
 output wire [31:0] position_frame_id,
 input wire result_very_strong
);
 reg finalize;reg [10:0] fx0,fy0,fx1,fy1;reg displayed_fine,had_fine,ever_fine,coarse_allowed;
 reg frame_d,active;
 reg [10:0] nx,ny;
 wire sof=frame && !frame_d;
 wire [10:0] x=sof ? 11'd0 : nx,y=sof ? 11'd0 : ny;
 wire take=valid && (active || sof);
 wire [31:0] current_id=video_id+(sof ? 32'd1 : 32'd0);
 reg authorized,auth_chroma,identity_confirmed,previous_positive;
 reg [1:0] previous_shape;
 reg [10:0] previous_cx,previous_cy;
 wire [10:0] incoming_cx=({1'b0,result_x0}+result_x1)>>1;
 wire [10:0] incoming_cy=({1'b0,result_y0}+result_y1)>>1;
 wire [10:0] identity_dx=incoming_cx>previous_cx ? incoming_cx-previous_cx : previous_cx-incoming_cx;
 wire [10:0] identity_dy=incoming_cy>previous_cy ? incoming_cy-previous_cy : previous_cy-incoming_cy;
 wire identity_consistent=previous_positive && result_shape==previous_shape && identity_dx<=256 && identity_dy<=256;
 wire [10:0] track_cx=({1'b0,fx0}+fx1)>>1,track_cy=({1'b0,fy0}+fy1)>>1;
 wire [10:0] link_dx=incoming_cx>track_cx ? incoming_cx-track_cx : track_cx-incoming_cx;
 wire [10:0] link_dy=incoming_cy>track_cy ? incoming_cy-track_cy : track_cy-incoming_cy;
 wire [11:0] incoming_w={1'b0,result_x1}-{1'b0,result_x0}+1'b1,incoming_h={1'b0,result_y1}-{1'b0,result_y0}+1'b1;
 wire incoming_linked=!had_fine || (link_dx<=256 && link_dy<=256 &&
  incoming_w<=2*({1'b0,fx1}-fx0+1)+32 && incoming_h<=2*({1'b0,fy1}-fy0+1)+32 &&
  incoming_w+32>=(({1'b0,fx1}-fx0+1)>>1) && incoming_h+32>=(({1'b0,fy1}-fy0+1)>>1));
 wire incoming_geometry_ok=result_x0<=result_x1 && result_y0<=result_y1 && result_x1<WIDTH && result_y1<HEIGHT && result_area>=16 && result_color<=4;
 reg [1:0] negative_count,as;
 reg [1:0] class_candidate,class_count;
 reg [2:0] ac;
 reg [31:0] auth_id,last_result_id;
 reg [7:0] ar,ag;
 reg [9:0] ab;
 reg [15:0] aa;
 reg [10:0] ax0,ay0,ax1,ay1;
 wire certificate_fresh=authorized && current_id-auth_id<=MAX_CERT_AGE;
 reg m_enabled,m_acquiring;
 reg [7:0] mr,mg;
 reg [9:0] mb;
 reg [15:0] ma;
 reg [10:0] mx0,my0,mx1,my1;
 reg [10:0] reference_w,reference_h;
 wire [10:0] roi_x0=mx0>SEARCH_MARGIN ? mx0-SEARCH_MARGIN : 11'd0;
 wire [10:0] roi_y0=my0>SEARCH_MARGIN ? my0-SEARCH_MARGIN : 11'd0;
 wire [11:0] roi_x1={1'b0,mx1}+SEARCH_MARGIN;
 wire [11:0] roi_y1={1'b0,my1}+SEARCH_MARGIN;
 wire in_roi=x>=roi_x0 && x<=roi_x1 && y>=roi_y0 && y<=roi_y1;
 wire [9:0] sum_r={2'd0,rgb[95:88]}+rgb[71:64]+rgb[47:40]+rgb[23:16];
 wire [9:0] sum_g={2'd0,rgb[87:80]}+rgb[63:56]+rgb[39:32]+rgb[15:8];
 wire [9:0] sum_b={2'd0,rgb[79:72]}+rgb[55:48]+rgb[31:24]+rgb[7:0];
 reg v0,v1,e0,e1,roi0,roi1;
 reg [7:0] r0,g0,b0;
 reg [10:0] x0,y0,x1,y1;
 reg [9:0] total1;
 reg signed [18:0] dr1,dg1;
 // Constant multipliers are shifts; two small variable products use LUTs so
 // the CNN's DSP budget is preserved. Both products terminate in registers.
 wire [9:0] total0={2'd0,r0}+g0+b0;
 (* dsp_style="no" *) wire [17:0] ref_r=mr*total0;
 (* dsp_style="no" *) wire [17:0] ref_g=mg*total0;
 wire [18:0] abs_r=dr1<0 ? -dr1 : dr1,abs_g=dg1<0 ? -dg1 : dg1;
 wire hit=v1 && roi1 && total1>=48 && {total1,1'b0}>=mb && total1<=2*mb &&
            abs_r<=TOLERANCE*total1 && abs_g<=TOLERANCE*total1;
 wire component_done,component_found,component_overflow;
 wire [19:0] component_count;
 wire [10:0] bx0,by0,bx1,by1;
 cnn_component_select #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) components(
  .clk(clk),.rst_n(rst_n),.clear(sof || !enable),.frame_end(finalize),.take(v1),.hit(hit),.x(x1),.y(y1),
  .acquiring(m_acquiring),.ref_x0(mx0),.ref_y0(my0),.ref_x1(mx1),.ref_y1(my1),.seed_area(ma),
  .done(component_done),.found(component_found),.xmin(bx0),.ymin(by0),.xmax(bx1),.ymax(by1),.count(component_count),.overflow(component_overflow));
 reg [2:0] misses;
 reg fine_measured,displayed_measured;
 reg signed [7:0] vx,vy;
 wire signed [12:0] delta_x=$signed({2'd0,bx0})-$signed({2'd0,fx0});
 wire signed [12:0] delta_y=$signed({2'd0,by0})-$signed({2'd0,fy0});
 function [10:0] shift_position;
  input [10:0] low,high;input signed [7:0] speed;input [10:0] limit;input upper;
  reg signed [12:0] proposed;reg [10:0] width;begin
   width=high-low;proposed=$signed({2'd0,low})+speed;
   if(proposed<0)shift_position=upper ? width : 11'd0;
   else if(proposed+width>=limit)shift_position=upper ? limit-1'b1 : limit-1'b1-width;
   else shift_position=upper ? proposed+width : proposed;
  end
 endfunction
 wire [10:0] predicted_x0=shift_position(fx0,fx1,vx,WIDTH,0),predicted_x1=shift_position(fx0,fx1,vx,WIDTH,1);
 wire [10:0] predicted_y0=shift_position(fy0,fy1,vy,HEIGHT,0),predicted_y1=shift_position(fy0,fy1,vy,HEIGHT,1);
 reg fine_valid;
 reg [31:0] fine_id;
 reg [1:0] fine_streak;
 reg [10:0] dx0,dy0,dx1,dy1;
 reg [1:0] ds;
 reg [2:0] dc;
 reg [31:0] displayed_position_id;
 wire measured_ok=m_enabled && component_found;
 wire pending_fine=fine_valid && certificate_fresh && current_id-fine_id<=2;
 wire show_fine=sof ? pending_fine && fine_streak>=2 && identity_confirmed : displayed_fine;
 assign tracking=rst_n && enable && show_fine && (sof ? fine_measured : displayed_measured);
 assign position_frame_id=show_fine ? (sof ? fine_id : displayed_position_id) : coarse_frame_id;
 wire [11:0] coarse_w={1'b0,coarse_x1}-{1'b0,coarse_x0}+1'b1;
 wire [11:0] coarse_h={1'b0,coarse_y1}-{1'b0,coarse_y0}+1'b1;
 wire [11:0] fine_w={1'b0,fx1}-{1'b0,fx0}+1'b1,fine_h={1'b0,fy1}-{1'b0,fy0}+1'b1;
 wire [10:0] coarse_mid_x=({1'b0,coarse_x0}+coarse_x1)>>1,coarse_mid_y=({1'b0,coarse_y0}+coarse_y1)>>1;
 wire [10:0] coarse_dx=coarse_mid_x>track_cx ? coarse_mid_x-track_cx : track_cx-coarse_mid_x;
 wire [10:0] coarse_dy=coarse_mid_y>track_cy ? coarse_mid_y-track_cy : track_cy-coarse_mid_y;
 wire coarse_size_safe=!ever_fine || (coarse_w<=fine_w+(fine_w>>1)+16 && coarse_h<=fine_h+(fine_h>>1)+16 && coarse_dx<=192 && coarse_dy<=192);
 wire coarse_candidate=!auth_chroma && coarse_visible && coarse_size_safe && !show_fine;
 wire show_coarse=sof ? coarse_candidate : coarse_allowed;
 assign visible=rst_n && enable && (show_fine || show_coarse);
 assign locked=show_fine ? visible && (sof ? fine_measured : displayed_measured) : coarse_locked && visible;
 assign shape=show_fine ? (sof ? as : ds) : coarse_shape;
 assign color_id=show_fine ? (sof ? ac : dc) : coarse_color;
 wire [10:0] raw_xmin=show_fine ? (sof ? fx0 : dx0) : coarse_x0;
 wire [10:0] raw_ymin=show_fine ? (sof ? fy0 : dy0) : coarse_y0;
 wire [10:0] raw_xmax=show_fine ? (sof ? fx1 : dx1) : coarse_x1;
 wire [10:0] raw_ymax=show_fine ? (sof ? fy1 : dy1) : coarse_y1;
 assign xmin=raw_xmin>=EDGE_PAD_X ? raw_xmin-EDGE_PAD_X : 11'd0;
 assign ymin=raw_ymin>=EDGE_PAD_Y ? raw_ymin-EDGE_PAD_Y : 11'd0;
 assign xmax={1'b0,raw_xmax}+EDGE_PAD_X<WIDTH ? raw_xmax+EDGE_PAD_X : WIDTH-1;
 assign ymax={1'b0,raw_ymax}+EDGE_PAD_Y<HEIGHT ? raw_ymax+EDGE_PAD_Y : HEIGHT-1;
 assign cx=show_fine ? (({1'b0,raw_xmin}+raw_xmax)>>1) : coarse_cx;
 assign cy=show_fine ? (({1'b0,raw_ymin}+raw_ymax)>>1) : coarse_cy;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   frame_d<=0;active<=0;nx<=0;ny<=0;
   authorized<=0;auth_chroma<=0;identity_confirmed<=0;previous_positive<=0;previous_shape<=0;previous_cx<=0;previous_cy<=0;negative_count<=0;as<=0;ac<=0;auth_id<=0;last_result_id<=32'hffffffff;
   ar<=0;ag<=0;ab<=0;aa<=0;ax0<=0;ay0<=0;ax1<=0;ay1<=0;class_candidate<=0;class_count<=0;
   m_enabled<=0;m_acquiring<=1;mr<=0;mg<=0;mb<=0;ma<=0;mx0<=0;my0<=0;mx1<=0;my1<=0;reference_w<=0;reference_h<=0;
   roi0<=0;roi1<=0;v0<=0;v1<=0;e0<=0;e1<=0;r0<=0;g0<=0;b0<=0;x0<=0;y0<=0;x1<=0;y1<=0;total1<=0;dr1<=0;dg1<=0;
   misses<=0;fine_measured<=0;displayed_measured<=0;vx<=0;vy<=0;
   finalize<=0;fine_valid<=0;fine_id<=0;fine_streak<=0;fx0<=0;fy0<=0;fx1<=0;fy1<=0;
   ever_fine<=0;coarse_allowed<=0;displayed_fine<=0;had_fine<=0;dx0<=0;dy0<=0;dx1<=0;dy1<=0;ds<=0;dc<=0;displayed_position_id<=0;
  end else begin
   frame_d<=frame;
   if(sof)begin active<=1;nx<=0;ny<=0;end
   if(take)begin
    if(x==WIDTH-4)begin nx<=0;if(y==HEIGHT-1)begin active<=0;ny<=0;end else ny<=y+1'b1;end
    else nx<=x+4;
   end
   v0<=take;v1<=v0;roi0<=m_enabled && in_roi;roi1<=roi0;
   e0<=take && x==WIDTH-4 && y==HEIGHT-1;e1<=e0;
   r0<=({1'b0,sum_r}+2)>>2;g0<=({1'b0,sum_g}+2)>>2;b0<=({1'b0,sum_b}+2)>>2;
   x0<=x;y0<=y;x1<=x0;y1<=y0;total1<=total0;
   dr1<=$signed({3'd0,r0,8'd0})-$signed({1'b0,ref_r});
   dg1<=$signed({3'd0,g0,8'd0})-$signed({1'b0,ref_g});
   finalize<=e1;
   if(component_done)begin
    fine_id<=video_id;
    if(measured_ok)begin
     fine_valid<=1;fine_measured<=1;misses<=0;
     fx0<=bx0;fy0<=by0;fx1<=bx1;fy1<=by1;
     if(fine_streak<3)fine_streak<=fine_streak+1'b1;
     vx<=had_fine ? (delta_x>48 ? 8'sd48 : delta_x< -48 ? -8'sd48 : delta_x) : 8'sd0;
     vy<=had_fine ? (delta_y>48 ? 8'sd48 : delta_y< -48 ? -8'sd48 : delta_y) : 8'sd0;
    end else if(had_fine && misses<COAST_FRAMES && certificate_fresh)begin
     fine_valid<=1;fine_measured<=0;misses<=misses+1'b1;
     fx0<=mx0;fy0<=my0;fx1<=mx1;fy1<=my1;
    end else begin fine_valid<=0;fine_measured<=0;fine_streak<=0;had_fine<=0;misses<=3;vx<=0;vy<=0;end
   end
   if(sof)begin
    displayed_fine<=pending_fine && fine_streak>=2 && identity_confirmed;displayed_measured<=fine_measured;coarse_allowed<=coarse_candidate;
    if(pending_fine && fine_streak>=2 && identity_confirmed)begin had_fine<=1;ever_fine<=1;dx0<=fx0;dy0<=fy0;dx1<=fx1;dy1<=fy1;ds<=as;dc<=ac;displayed_position_id<=fine_id;end
    m_enabled<=certificate_fresh && auth_chroma;
    m_acquiring<=!pending_fine;
    mr<=ar;mg<=ag;mb<=ab;ma<=aa;
    mx0<=pending_fine ? predicted_x0 : ax0;my0<=pending_fine ? predicted_y0 : ay0;
    mx1<=pending_fine ? predicted_x1 : ax1;my1<=pending_fine ? predicted_y1 : ay1;
    reference_w<=pending_fine ? fx1-fx0+1'b1 : ax1-ax0+1'b1;
    reference_h<=pending_fine ? fy1-fy0+1'b1 : ay1-ay0+1'b1;
    v0<=0;v1<=0;roi0<=0;roi1<=0;e0<=0;e1<=0;finalize<=0;
    if(!certificate_fresh)begin if(!fine_valid)ever_fine<=0;authorized<=0;identity_confirmed<=0;previous_positive<=0;had_fine<=0;end
   end
   if(result_update && result_id!=last_result_id)begin
    last_result_id<=result_id;
    if(result_valid && result_shape!=0 && incoming_geometry_ok && current_id-result_id<=MAX_CERT_AGE && incoming_linked)begin
     previous_positive<=1;previous_shape<=result_shape;previous_cx<=incoming_cx;previous_cy<=incoming_cy;
     if(identity_consistent || result_very_strong)identity_confirmed<=1;
     if(authorized && (identity_dx>512 || identity_dy>512))begin identity_confirmed<=0;had_fine<=0;fine_valid<=0;fine_streak<=0;end
     authorized<=1;auth_chroma<=seed_valid;negative_count<=0;auth_id<=result_id;
     ar<=seed_r;ag<=seed_g;ab<=seed_sum;aa<=result_area;ac<=result_color;
     // A physical target keeps its class while moving. Require three
     // independent agreeing CNN results to change a continuously tracked class;
     // geometry updates remain at video cadence. Reacquisition can set it anew.
     if(!identity_confirmed || !had_fine)begin as<=result_shape;class_count<=0;end
     else if(result_shape==as)class_count<=0;
     else begin
      class_candidate<=result_shape;
      if(class_count!=0 && class_candidate==result_shape)begin
       if(class_count>=2)begin as<=result_shape;class_count<=0;end
       else class_count<=class_count+1'b1;
      end else class_count<=1;
     end
     ax0<=result_x0;ay0<=result_y0;ax1<=result_x1;ay1<=result_y1;
    end else begin
     previous_positive<=0;class_count<=0;
     if(negative_count<3)negative_count<=negative_count+1'b1;
     if(negative_count>=1 && !had_fine)begin authorized<=0;identity_confirmed<=0;auth_chroma<=0;had_fine<=0;end
    end
   end
   if(!enable)begin
    ever_fine<=0;coarse_allowed<=0;authorized<=0;auth_chroma<=0;identity_confirmed<=0;previous_positive<=0;negative_count<=0;class_count<=0;last_result_id<=32'hffffffff;
    m_enabled<=0;fine_valid<=0;fine_streak<=0;displayed_fine<=0;had_fine<=0;v0<=0;v1<=0;roi0<=0;roi1<=0;misses<=0;fine_measured<=0;displayed_measured<=0;vx<=0;vy<=0;
   end
  end
 end
endmodule
