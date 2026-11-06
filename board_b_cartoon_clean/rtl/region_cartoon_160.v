// Frame-based bounded integer region simplification. Default working image 160x90.
// Captures 8x8 box means, RGB565, 144 fixed seeds, three local assignment passes,
// two raster adjacency merge passes, then flat region-color replay at full size.
// Input is never backpressured: whole frames are skipped when both slots are owned.
module region_cartoon_160 #(
 parameter WIDTH=1280, HEIGHT=720, LOW_W=WIDTH/8, LOW_H=HEIGHT/8,
 parameter GRID_X=16, GRID_Y=9, ITERATIONS=3, MERGE_PASSES=2,
 parameter MERGE_THRESHOLD=24
)(input clk,rst_n,enable,cancel,
 input [95:0] pixels, input valid,sof,frame_end,frame_good,
 input launch,output_ready,output_drained,
 output ready,rendering,
 output reg [95:0] out_pixels, output reg out_valid,out_frame,out_good,
 output reg fault,
 output reg [31:0] completed,skipped,processing_cycles);
 localparam N=LOW_W*LOW_H, K=GRID_X*GRID_Y;
 localparam CW=LOW_W/GRID_X, CH=LOW_H/GRID_Y;
 localparam IDLE=0,SEED_ADDR=1,SEED_WAIT=2,SEED_LOAD=3,CLEAR=4,
 PIX_ADDR=5,PIX_WAIT=6,PIX_LOAD=7,CAND_ADDR=8,CAND_EVAL=9,
 ACC_ADDR=10,ACC_WAIT=11,ACC_WRITE=12,NEXT_PIX=13,
 MEAN_ADDR=14,MEAN_WAIT=15,MEAN_LOAD=16,DIV_START=17,DIV_WAIT=18,MEAN_STORE=19,
 PAIR_ADDR=20,PAIR_WAIT=21,PAIR_LOAD=22,ROOT_A=23,ROOT_B=24,
 ROOT_COMPARE=25,MERGE_A_ADDR=26,MERGE_A_WAIT=27,MERGE_A_LOAD=28,
 MERGE_B_ADDR=29,MERGE_B_WAIT=30,MERGE_B_LOAD=31,MERGE_WRITE=32,
 NEXT_PAIR=33,PAL_ROOT=34,PAL_WRITE=35,READY=36,
 OPEN_FRAME=37,OPEN_GAP=38,RENDER_LABEL=39,RENDER_LABEL_WAIT=40,
 RENDER_PAL=41,RENDER_EMIT=42,DRAIN=43,CLOSE_FRAME=44,CLOSE_GAP=45;
 reg [5:0] state;
 reg [1:0] cap_state;
 reg cap_bank,work_bank;
 reg [10:0] cap_x,cap_y;
 reg [9:0] horizontal_r,horizontal_g,horizontal_b;
 reg [13:0] columns_r[0:LOW_W-1],columns_g[0:LOW_W-1],columns_b[0:LOW_W-1];
 wire take_capture=(state==IDLE && cap_state==3 && !cancel);
 wire start_capture=sof && enable && cap_state==0 &&
                    (state==IDLE || cap_bank!=work_bank);
 wire capture_valid=valid && (cap_state==1 || start_capture) && !cancel;
 wire [10:0] use_x=sof?11'd0:cap_x;
 wire [10:0] use_y=sof?11'd0:cap_y;
 wire [9:0] lane_r={2'b0,pixels[95:88]}+{2'b0,pixels[71:64]}+{2'b0,pixels[47:40]}+{2'b0,pixels[23:16]};
 wire [9:0] lane_g={2'b0,pixels[87:80]}+{2'b0,pixels[63:56]}+{2'b0,pixels[39:32]}+{2'b0,pixels[15:8]};
 wire [9:0] lane_b={2'b0,pixels[79:72]}+{2'b0,pixels[55:48]}+{2'b0,pixels[31:24]}+{2'b0,pixels[7:0]};
 wire [7:0] col_index=use_x>>1;
 wire [13:0] total_r=(use_y[2:0]==0?14'd0:columns_r[col_index])+horizontal_r+lane_r;
 wire [13:0] total_g=(use_y[2:0]==0?14'd0:columns_g[col_index])+horizontal_g+lane_g;
 wire [13:0] total_b=(use_y[2:0]==0?14'd0:columns_b[col_index])+horizontal_b+lane_b;
 wire cap_we=capture_valid && use_x[0] && use_y[2:0]==7;
 wire [13:0] cap_addr=(use_y>>3)*LOW_W+(use_x>>1);
 wire [15:0] cap_rgb={total_r[13:9],total_g[13:8],total_b[13:9]};
 wire [15:0] input_rd0,input_rd1;
 reg [13:0] input_ra;
 wire [15:0] input_rd=work_bank?input_rd1:input_rd0;
 region_ram #(.WIDTH(16)) input0(clk,cap_we&&!cap_bank,cap_addr,input_ra,cap_rgb,input_rd0);
 region_ram #(.WIDTH(16)) input1(clk,cap_we&& cap_bank,cap_addr,input_ra,cap_rgb,input_rd1);

 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin cap_state<=0;cap_bank<=0;cap_x<=0;cap_y<=0;
   horizontal_r<=0;horizontal_g<=0;horizontal_b<=0;skipped<=0;end
  else begin
   if(take_capture) begin cap_state<=0;cap_bank<=~cap_bank;end
   if(frame_end) begin
    if(cap_state==2) cap_state<=frame_good?3:0;
    else if(cap_state==1) cap_state<=0;
   end
   if(sof) begin
    if(start_capture) begin cap_state<=1;cap_x<=0;cap_y<=0;end
    else if(enable) begin skipped<=skipped+1'b1;if(cap_state==1) cap_state<=0;end
   end
   if(capture_valid) begin
    if(!use_x[0]) begin horizontal_r<=lane_r;horizontal_g<=lane_g;horizontal_b<=lane_b;end
    if(use_x==WIDTH/4-1) begin cap_x<=0;cap_y<=use_y+1'b1;end
    else begin cap_x<=use_x+1'b1;cap_y<=use_y;end
    if(use_x==WIDTH/4-1 && use_y==HEIGHT-1) cap_state<=2;
   end
   if(cancel) cap_state<=0;
  end
 end
 // First row of each tile explicitly starts at zero, so RAM needs no reset.
 always @(posedge clk) if(capture_valid && use_x[0]) begin
  columns_r[col_index]<=total_r;columns_g[col_index]<=total_g;columns_b[col_index]<=total_b;
 end

 function [23:0] expand565(input [15:0] c);
  begin expand565={{c[15:11],c[15:13]},{c[10:5],c[10:9]},{c[4:0],c[4:2]}};end
 endfunction
 function integer absval(input integer a);begin absval=a<0?-a:a;end endfunction
 function integer color_distance(input [23:0] a,b);
  begin color_distance=absval({1'b0,a[23:16]}-{1'b0,b[23:16]})+
    absval({1'b0,a[15:8]}-{1'b0,b[15:8]})+absval({1'b0,a[7:0]}-{1'b0,b[7:0]});end
 endfunction
 function [7:0] clip(input integer a);begin clip=a<0?0:(a>255?255:a);end endfunction
 function [7:0] region_quant(input [7:0] y);
 begin case(y)
  8'd0:region_quant=8'd0;
  8'd1:region_quant=8'd0;
  8'd2:region_quant=8'd0;
  8'd3:region_quant=8'd0;
  8'd4:region_quant=8'd0;
  8'd5:region_quant=8'd0;
  8'd6:region_quant=8'd0;
  8'd7:region_quant=8'd0;
  8'd8:region_quant=8'd0;
  8'd9:region_quant=8'd0;
  8'd10:region_quant=8'd0;
  8'd11:region_quant=8'd0;
  8'd12:region_quant=8'd0;
  8'd13:region_quant=8'd0;
  8'd14:region_quant=8'd0;
  8'd15:region_quant=8'd0;
  8'd16:region_quant=8'd0;
  8'd17:region_quant=8'd0;
  8'd18:region_quant=8'd0;
  8'd19:region_quant=8'd0;
  8'd20:region_quant=8'd0;
  8'd21:region_quant=8'd23;
  8'd22:region_quant=8'd23;
  8'd23:region_quant=8'd23;
  8'd24:region_quant=8'd23;
  8'd25:region_quant=8'd23;
  8'd26:region_quant=8'd23;
  8'd27:region_quant=8'd23;
  8'd28:region_quant=8'd23;
  8'd29:region_quant=8'd23;
  8'd30:region_quant=8'd23;
  8'd31:region_quant=8'd23;
  8'd32:region_quant=8'd23;
  8'd33:region_quant=8'd23;
  8'd34:region_quant=8'd23;
  8'd35:region_quant=8'd23;
  8'd36:region_quant=8'd23;
  8'd37:region_quant=8'd23;
  8'd38:region_quant=8'd23;
  8'd39:region_quant=8'd23;
  8'd40:region_quant=8'd23;
  8'd41:region_quant=8'd23;
  8'd42:region_quant=8'd23;
  8'd43:region_quant=8'd23;
  8'd44:region_quant=8'd23;
  8'd45:region_quant=8'd23;
  8'd46:region_quant=8'd23;
  8'd47:region_quant=8'd23;
  8'd48:region_quant=8'd23;
  8'd49:region_quant=8'd23;
  8'd50:region_quant=8'd23;
  8'd51:region_quant=8'd23;
  8'd52:region_quant=8'd23;
  8'd53:region_quant=8'd46;
  8'd54:region_quant=8'd46;
  8'd55:region_quant=8'd46;
  8'd56:region_quant=8'd46;
  8'd57:region_quant=8'd46;
  8'd58:region_quant=8'd46;
  8'd59:region_quant=8'd46;
  8'd60:region_quant=8'd46;
  8'd61:region_quant=8'd46;
  8'd62:region_quant=8'd46;
  8'd63:region_quant=8'd46;
  8'd64:region_quant=8'd46;
  8'd65:region_quant=8'd46;
  8'd66:region_quant=8'd46;
  8'd67:region_quant=8'd46;
  8'd68:region_quant=8'd46;
  8'd69:region_quant=8'd46;
  8'd70:region_quant=8'd46;
  8'd71:region_quant=8'd46;
  8'd72:region_quant=8'd46;
  8'd73:region_quant=8'd46;
  8'd74:region_quant=8'd46;
  8'd75:region_quant=8'd46;
  8'd76:region_quant=8'd46;
  8'd77:region_quant=8'd69;
  8'd78:region_quant=8'd69;
  8'd79:region_quant=8'd69;
  8'd80:region_quant=8'd69;
  8'd81:region_quant=8'd69;
  8'd82:region_quant=8'd69;
  8'd83:region_quant=8'd69;
  8'd84:region_quant=8'd69;
  8'd85:region_quant=8'd69;
  8'd86:region_quant=8'd69;
  8'd87:region_quant=8'd69;
  8'd88:region_quant=8'd69;
  8'd89:region_quant=8'd69;
  8'd90:region_quant=8'd69;
  8'd91:region_quant=8'd69;
  8'd92:region_quant=8'd69;
  8'd93:region_quant=8'd69;
  8'd94:region_quant=8'd69;
  8'd95:region_quant=8'd69;
  8'd96:region_quant=8'd69;
  8'd97:region_quant=8'd69;
  8'd98:region_quant=8'd69;
  8'd99:region_quant=8'd92;
  8'd100:region_quant=8'd92;
  8'd101:region_quant=8'd92;
  8'd102:region_quant=8'd92;
  8'd103:region_quant=8'd92;
  8'd104:region_quant=8'd92;
  8'd105:region_quant=8'd92;
  8'd106:region_quant=8'd92;
  8'd107:region_quant=8'd92;
  8'd108:region_quant=8'd92;
  8'd109:region_quant=8'd92;
  8'd110:region_quant=8'd92;
  8'd111:region_quant=8'd92;
  8'd112:region_quant=8'd92;
  8'd113:region_quant=8'd92;
  8'd114:region_quant=8'd92;
  8'd115:region_quant=8'd92;
  8'd116:region_quant=8'd92;
  8'd117:region_quant=8'd115;
  8'd118:region_quant=8'd115;
  8'd119:region_quant=8'd115;
  8'd120:region_quant=8'd115;
  8'd121:region_quant=8'd115;
  8'd122:region_quant=8'd115;
  8'd123:region_quant=8'd115;
  8'd124:region_quant=8'd115;
  8'd125:region_quant=8'd115;
  8'd126:region_quant=8'd115;
  8'd127:region_quant=8'd115;
  8'd128:region_quant=8'd115;
  8'd129:region_quant=8'd115;
  8'd130:region_quant=8'd115;
  8'd131:region_quant=8'd115;
  8'd132:region_quant=8'd115;
  8'd133:region_quant=8'd115;
  8'd134:region_quant=8'd139;
  8'd135:region_quant=8'd139;
  8'd136:region_quant=8'd139;
  8'd137:region_quant=8'd139;
  8'd138:region_quant=8'd139;
  8'd139:region_quant=8'd139;
  8'd140:region_quant=8'd139;
  8'd141:region_quant=8'd139;
  8'd142:region_quant=8'd139;
  8'd143:region_quant=8'd139;
  8'd144:region_quant=8'd139;
  8'd145:region_quant=8'd139;
  8'd146:region_quant=8'd139;
  8'd147:region_quant=8'd139;
  8'd148:region_quant=8'd139;
  8'd149:region_quant=8'd139;
  8'd150:region_quant=8'd162;
  8'd151:region_quant=8'd162;
  8'd152:region_quant=8'd162;
  8'd153:region_quant=8'd162;
  8'd154:region_quant=8'd162;
  8'd155:region_quant=8'd162;
  8'd156:region_quant=8'd162;
  8'd157:region_quant=8'd162;
  8'd158:region_quant=8'd162;
  8'd159:region_quant=8'd162;
  8'd160:region_quant=8'd162;
  8'd161:region_quant=8'd162;
  8'd162:region_quant=8'd162;
  8'd163:region_quant=8'd162;
  8'd164:region_quant=8'd162;
  8'd165:region_quant=8'd162;
  8'd166:region_quant=8'd185;
  8'd167:region_quant=8'd185;
  8'd168:region_quant=8'd185;
  8'd169:region_quant=8'd185;
  8'd170:region_quant=8'd185;
  8'd171:region_quant=8'd185;
  8'd172:region_quant=8'd185;
  8'd173:region_quant=8'd185;
  8'd174:region_quant=8'd185;
  8'd175:region_quant=8'd185;
  8'd176:region_quant=8'd185;
  8'd177:region_quant=8'd185;
  8'd178:region_quant=8'd185;
  8'd179:region_quant=8'd185;
  8'd180:region_quant=8'd185;
  8'd181:region_quant=8'd185;
  8'd182:region_quant=8'd185;
  8'd183:region_quant=8'd185;
  8'd184:region_quant=8'd185;
  8'd185:region_quant=8'd208;
  8'd186:region_quant=8'd208;
  8'd187:region_quant=8'd208;
  8'd188:region_quant=8'd208;
  8'd189:region_quant=8'd208;
  8'd190:region_quant=8'd208;
  8'd191:region_quant=8'd208;
  8'd192:region_quant=8'd208;
  8'd193:region_quant=8'd208;
  8'd194:region_quant=8'd208;
  8'd195:region_quant=8'd208;
  8'd196:region_quant=8'd208;
  8'd197:region_quant=8'd208;
  8'd198:region_quant=8'd208;
  8'd199:region_quant=8'd208;
  8'd200:region_quant=8'd208;
  8'd201:region_quant=8'd208;
  8'd202:region_quant=8'd208;
  8'd203:region_quant=8'd208;
  8'd204:region_quant=8'd208;
  8'd205:region_quant=8'd208;
  8'd206:region_quant=8'd231;
  8'd207:region_quant=8'd231;
  8'd208:region_quant=8'd231;
  8'd209:region_quant=8'd231;
  8'd210:region_quant=8'd231;
  8'd211:region_quant=8'd231;
  8'd212:region_quant=8'd231;
  8'd213:region_quant=8'd231;
  8'd214:region_quant=8'd231;
  8'd215:region_quant=8'd231;
  8'd216:region_quant=8'd231;
  8'd217:region_quant=8'd231;
  8'd218:region_quant=8'd231;
  8'd219:region_quant=8'd231;
  8'd220:region_quant=8'd231;
  8'd221:region_quant=8'd231;
  8'd222:region_quant=8'd231;
  8'd223:region_quant=8'd231;
  8'd224:region_quant=8'd231;
  8'd225:region_quant=8'd231;
  8'd226:region_quant=8'd231;
  8'd227:region_quant=8'd231;
  8'd228:region_quant=8'd231;
  8'd229:region_quant=8'd231;
  8'd230:region_quant=8'd231;
  8'd231:region_quant=8'd231;
  8'd232:region_quant=8'd231;
  8'd233:region_quant=8'd231;
  8'd234:region_quant=8'd255;
  8'd235:region_quant=8'd255;
  8'd236:region_quant=8'd255;
  8'd237:region_quant=8'd255;
  8'd238:region_quant=8'd255;
  8'd239:region_quant=8'd255;
  8'd240:region_quant=8'd255;
  8'd241:region_quant=8'd255;
  8'd242:region_quant=8'd255;
  8'd243:region_quant=8'd255;
  8'd244:region_quant=8'd255;
  8'd245:region_quant=8'd255;
  8'd246:region_quant=8'd255;
  8'd247:region_quant=8'd255;
  8'd248:region_quant=8'd255;
  8'd249:region_quant=8'd255;
  8'd250:region_quant=8'd255;
  8'd251:region_quant=8'd255;
  8'd252:region_quant=8'd255;
  8'd253:region_quant=8'd255;
  8'd254:region_quant=8'd255;
  8'd255:region_quant=8'd255;
 endcase end endfunction
 function [7:0] tone(input [7:0] y);
  integer p,t,v0,v1;
  begin
   p=y>>5;t=y&31;
   case(p)
    0:begin v0=0;v1=18;end 1:begin v0=18;v1=44;end
    2:begin v0=44;v1=78;end 3:begin v0=78;v1=120;end
    4:begin v0=120;v1=166;end 5:begin v0=166;v1=207;end
    6:begin v0=207;v1=239;end default:begin v0=239;v1=255;end
   endcase
   tone=(p==7)?(v0+(v1-v0)*t/31):(v0+((v1-v0)*t+16)/32);
  end
 endfunction
 // Style is evaluated once per palette entry, never in the 75 MHz input path.
 function [23:0] style(input [23:0] c);
  integer y,r,g,b,t,q,d;
  begin
   y=(77*c[23:16]+150*c[15:8]+29*c[7:0])>>8;
   r=clip(y+((($signed({1'b0,c[23:16]})-y)*410)>>>8));
   g=clip(y+((($signed({1'b0,c[15:8]})-y)*410)>>>8));
   b=clip(y+((($signed({1'b0,c[7:0]})-y)*410)>>>8));
   y=(77*r+150*g+29*b)>>8;t=tone(y);q=((t*11+127)/255)*255/11;d=q-y;
   style={clip(r+d),clip(g+d),clip(b+d)};
  end
 endfunction

 reg [23:0] means[0:255];
 reg [7:0] parent[0:255];
 reg [7:0] idx,mean_idx,pal_idx;
 reg [1:0] iteration,merge_pass,channel;
 reg mean_after_merge;
 reg [13:0] pixel_index,pair_index;
 reg [7:0] px,py;
 reg [3:0] candidate;
 reg [7:0] candidate_id,best_id;
 reg candidate_ok;
 reg [12:0] best_cost;
 reg [23:0] pixel_rgb,mean_rgb;
 reg [95:0] stats,merge_a,merge_b;
 reg [7:0] root_a,root_b,winner,loser,walk_steps;
 reg pair_down;
 reg [13:0] label_ra;
 wire [7:0] label_rd;
 wire label_we=state==ACC_WRITE && iteration==ITERATIONS-1;
 region_ram #(.WIDTH(8)) labels(clk,label_we,pixel_index,label_ra,best_id,label_rd);
 reg [7:0] accum_ra;
 wire [95:0] accum_rd;
 wire accum_we=(state==CLEAR || state==ACC_WRITE || state==MERGE_WRITE);
 wire [7:0] accum_wa=state==CLEAR?idx:(state==MERGE_WRITE?winner:best_id);
 wire [95:0] accum_wd=state==CLEAR?96'd0:state==MERGE_WRITE?
  {8'd0,merge_a[87:64]+merge_b[87:64],merge_a[63:40]+merge_b[63:40],
         merge_a[39:16]+merge_b[39:16],merge_a[15:0]+merge_b[15:0]}:
  {8'd0,stats[87:64]+{16'd0,pixel_rgb[23:16]},stats[63:40]+{16'd0,pixel_rgb[15:8]},
         stats[39:16]+{16'd0,pixel_rgb[7:0]},stats[15:0]+16'd1};
 region_ram #(.WIDTH(96),.AW(9),.DEPTH(512)) sums(clk,accum_we,{1'b0,accum_wa},{1'b0,accum_ra},accum_wd,accum_rd);
 wire [23:0] palette_rd;
 reg [7:0] palette_ra;
 wire palette_we=state==PAL_WRITE;
 reg [23:0] style_src,style_sat,style_result;
 reg [7:0] style_y,style_y2,style_t;
 reg signed [10:0] style_delta;
 wire [23:0] palette_wd=style_result;
 region_ram #(.WIDTH(24),.AW(9),.DEPTH(512)) palette(clk,palette_we,{1'b0,pal_idx},{1'b0,palette_ra},palette_wd,palette_rd);
 wire div_done;
 wire [23:0] quotient;
 wire [23:0] numerator=channel==0?stats[87:64]:channel==1?stats[63:40]:stats[39:16];
 region_divider divider(clk,rst_n,state==DIV_START,numerator,stats[15:0],div_done,quotient);
 integer gx,gy,ci,cost;
 always @* begin
  gx=px/CW+(candidate%3)-1;gy=py/CH+(candidate/3)-1;
  ci=gy*GRID_X+gx;
  cost=color_distance(pixel_rgb,means[candidate_id])+2*(
    absval(px-((candidate_id%GRID_X)*CW+CW/2))+
    absval(py-((candidate_id/GRID_X)*CH+CH/2)));
 end
 reg [10:0] render_x,render_y;
 reg [12:0] gap;
 reg [23:0] replay_color;
 assign ready=state==READY;
 assign rendering=state>=OPEN_FRAME && state<=47;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   state<=IDLE;work_bank<=0;idx<=0;mean_idx<=0;pal_idx<=0;iteration<=0;merge_pass<=0;channel<=0;
   input_ra<=0;label_ra<=0;accum_ra<=0;palette_ra<=0;pixel_index<=0;pair_index<=0;
   px<=0;py<=0;candidate<=0;candidate_id<=0;candidate_ok<=0;best_id<=0;best_cost<=8191;
   pixel_rgb<=0;mean_rgb<=0;stats<=0;merge_a<=0;merge_b<=0;root_a<=0;root_b<=0;
   winner<=0;loser<=0;walk_steps<=0;pair_down<=0;mean_after_merge<=0;
   render_x<=0;render_y<=0;gap<=0;replay_color<=0;
   out_pixels<=0;out_valid<=0;out_frame<=0;out_good<=0;fault<=0;completed<=0;processing_cycles<=0;
   style_src<=0;style_sat<=0;style_result<=0;style_y<=0;style_y2<=0;style_t<=0;style_delta<=0;
  end else begin
   out_valid<=0;out_frame<=0;
   if(state!=IDLE && state!=READY && !rendering) processing_cycles<=processing_cycles+1'b1;
   case(state)
    IDLE:if(take_capture) begin
     work_bank<=cap_bank;idx<=0;iteration<=0;processing_cycles<=0;state<=SEED_ADDR;
    end
    SEED_ADDR:begin input_ra<=((idx/GRID_X)*CH+CH/2)*LOW_W+(idx%GRID_X)*CW+CW/2;state<=SEED_WAIT;end
    SEED_WAIT:state<=SEED_LOAD;
    SEED_LOAD:begin means[idx]<=expand565(input_rd);
     if(idx==K-1) begin idx<=0;state<=CLEAR;end else begin idx<=idx+1'b1;state<=SEED_ADDR;end
    end
    CLEAR:begin
     if(iteration==0) parent[idx]<=idx;
     if(idx==255) begin pixel_index<=0;px<=0;py<=0;state<=PIX_ADDR;end else idx<=idx+1'b1;
    end
    PIX_ADDR:begin input_ra<=pixel_index;state<=PIX_WAIT;end
    PIX_WAIT:state<=PIX_LOAD;
    PIX_LOAD:begin pixel_rgb<=expand565(input_rd);candidate<=0;best_cost<=8191;best_id<=0;state<=CAND_ADDR;end
    CAND_ADDR:begin candidate_id<=ci;candidate_ok<=gx>=0 && gx<GRID_X && gy>=0 && gy<GRID_Y;state<=CAND_EVAL;end
    CAND_EVAL:begin
     if(candidate_ok && cost<best_cost) begin best_cost<=cost;best_id<=candidate_id;end
     if(candidate==8) state<=ACC_ADDR;else begin candidate<=candidate+1'b1;state<=CAND_ADDR;end
    end
    ACC_ADDR:begin accum_ra<=best_id;state<=ACC_WAIT;end
    ACC_WAIT:state<=48;
    48:begin state<=ACC_WRITE;stats<=accum_rd;end
    ACC_WRITE:state<=NEXT_PIX;
    NEXT_PIX:begin
     if(pixel_index==N-1) begin mean_idx<=0;mean_after_merge<=0;state<=MEAN_ADDR;end
     else begin pixel_index<=pixel_index+1'b1;if(px==LOW_W-1)begin px<=0;py<=py+1'b1;end else px<=px+1'b1;state<=PIX_ADDR;end
    end
    MEAN_ADDR:begin accum_ra<=mean_idx;state<=MEAN_WAIT;end
    MEAN_WAIT:state<=MEAN_LOAD;
    MEAN_LOAD:begin stats<=accum_rd;mean_rgb<=means[mean_idx];channel<=0;
     if(accum_rd[15:0]==0) state<=MEAN_STORE;else state<=DIV_START;
    end
    DIV_START:state<=DIV_WAIT;
    DIV_WAIT:if(div_done) begin
     case(channel)0:mean_rgb[23:16]<=quotient[7:0];1:mean_rgb[15:8]<=quotient[7:0];2:mean_rgb[7:0]<=quotient[7:0];endcase
     if(channel==2) state<=MEAN_STORE;else begin channel<=channel+1'b1;state<=DIV_START;end
    end
    MEAN_STORE:begin
     // Match the RGB565 precision of the integer reference during clustering.
     means[mean_idx]<=(mean_after_merge || iteration==ITERATIONS-1)?mean_rgb:expand565({mean_rgb[23:19],mean_rgb[15:10],mean_rgb[7:3]});
     if(mean_after_merge) state<=NEXT_PAIR;
     else if(mean_idx==K-1) begin
      if(iteration==ITERATIONS-1) begin pair_index<=0;px<=0;py<=0;pair_down<=0;merge_pass<=0;state<=PAIR_ADDR;end
      else begin iteration<=iteration+1'b1;idx<=0;state<=CLEAR;end
     end else begin mean_idx<=mean_idx+1'b1;state<=MEAN_ADDR;end
    end
    PAIR_ADDR:begin
     if((!pair_down && px==LOW_W-1) || (pair_down && py==LOW_H-1)) state<=NEXT_PAIR;
     else begin label_ra<=pair_index;state<=PAIR_WAIT;end
    end
    PAIR_WAIT:state<=PAIR_LOAD;
    PAIR_LOAD:begin root_a<=label_rd;label_ra<=pair_index+(pair_down?LOW_W:1);walk_steps<=0;state<=ROOT_A;end
    ROOT_A:begin
     if(walk_steps==K) begin fault<=1;state<=IDLE;end
     else if(parent[root_a]!=root_a) begin root_a<=parent[root_a];walk_steps<=walk_steps+1'b1;end
     else state<=49;
    end
    49:begin root_b<=label_rd;walk_steps<=0;state<=ROOT_B;end
    ROOT_B:begin
     if(walk_steps==K) begin fault<=1;state<=IDLE;end
     else if(parent[root_b]!=root_b) begin root_b<=parent[root_b];walk_steps<=walk_steps+1'b1;end
     else state<=ROOT_COMPARE;
    end
    ROOT_COMPARE:if(root_a!=root_b && color_distance(means[root_a],means[root_b])<=MERGE_THRESHOLD) state<=MERGE_A_ADDR;else state<=NEXT_PAIR;
    MERGE_A_ADDR:begin accum_ra<=root_a;state<=MERGE_A_WAIT;end
    MERGE_A_WAIT:state<=MERGE_A_LOAD;
    MERGE_A_LOAD:begin merge_a<=accum_rd;accum_ra<=root_b;state<=MERGE_B_WAIT;end
    MERGE_B_WAIT:state<=MERGE_B_LOAD;
    MERGE_B_LOAD:begin merge_b<=accum_rd;
     winner<=merge_a[15:0]>=accum_rd[15:0]?root_a:root_b;
     loser<=merge_a[15:0]>=accum_rd[15:0]?root_b:root_a;
     state<=MERGE_WRITE;
    end
    MERGE_WRITE:begin parent[loser]<=winner;mean_idx<=winner;mean_after_merge<=1;state<=MEAN_ADDR;end
    NEXT_PAIR:begin
     if(!pair_down) begin pair_down<=1;state<=PAIR_ADDR;end
     else begin
      pair_down<=0;
      if(pair_index==N-1) begin
       if(merge_pass==MERGE_PASSES-1) begin pal_idx<=0;root_a<=0;walk_steps<=0;state<=PAL_ROOT;end
       else begin merge_pass<=merge_pass+1'b1;pair_index<=0;px<=0;py<=0;state<=PAIR_ADDR;end
      end else begin pair_index<=pair_index+1'b1;if(px==LOW_W-1)begin px<=0;py<=py+1'b1;end else px<=px+1'b1;state<=PAIR_ADDR;end
     end
    end
    PAL_ROOT:begin
     if(walk_steps==K) begin fault<=1;state<=IDLE;end
     else if(parent[root_a]!=root_a) begin root_a<=parent[root_a];walk_steps<=walk_steps+1'b1;end
     else state<=50;
    end
    50:begin style_src<=means[root_a];state<=51;end
    51:begin style_y<=(77*style_src[23:16]+150*style_src[15:8]+29*style_src[7:0])>>8;state<=52;end
    52:begin
     style_sat<={clip($signed({1'b0,style_y})+(($signed({1'b0,style_src[23:16]})-$signed({1'b0,style_y}))*410>>>8)),
                 clip($signed({1'b0,style_y})+(($signed({1'b0,style_src[15:8]})-$signed({1'b0,style_y}))*410>>>8)),
                 clip($signed({1'b0,style_y})+(($signed({1'b0,style_src[7:0]})-$signed({1'b0,style_y}))*410>>>8))};
     state<=53;
    end
    53:begin style_y2<=(77*style_sat[23:16]+150*style_sat[15:8]+29*style_sat[7:0])>>8;state<=54;end
    54:begin style_t<=region_quant(style_y2);state<=55;end
    55:begin style_delta<=$signed({1'b0,style_t})-$signed({1'b0,style_y2});state<=56;end
    56:begin style_result<={clip($signed({1'b0,style_sat[23:16]})+style_delta),
                           clip($signed({1'b0,style_sat[15:8]})+style_delta),
                           clip($signed({1'b0,style_sat[7:0]})+style_delta)};state<=PAL_WRITE;end
    PAL_WRITE:begin
     if(pal_idx==K-1) begin state<=READY;completed<=completed+1'b1;end
     else begin pal_idx<=pal_idx+1'b1;root_a<=pal_idx+1'b1;walk_steps<=0;state<=PAL_ROOT;end
    end
    READY:if(launch) begin render_x<=0;render_y<=0;state<=OPEN_FRAME;end
    OPEN_FRAME:begin out_frame<=1;out_good<=0;gap<=0;state<=OPEN_GAP;end
    OPEN_GAP:if(gap==63)state<=RENDER_LABEL;else gap<=gap+1'b1;
    RENDER_LABEL:begin label_ra<=(render_y>>3)*LOW_W+(render_x>>1);state<=RENDER_LABEL_WAIT;end
    RENDER_LABEL_WAIT:state<=RENDER_PAL;
    RENDER_PAL:begin palette_ra<=label_rd;state<=RENDER_EMIT;end
    RENDER_EMIT:begin replay_color<=palette_rd;gap<=0;state<=46;end
    46:begin replay_color<=palette_rd;state<=47;end
    47:if(output_ready) begin
     out_pixels<={4{replay_color}};out_valid<=1;
     if(render_x==WIDTH/4-1) begin render_x<=0;render_y<=render_y+1'b1;end else render_x<=render_x+1'b1;
     if(render_x==WIDTH/4-1 && render_y==HEIGHT-1) begin gap<=0;state<=DRAIN;end else state<=RENDER_LABEL;
    end
    DRAIN:begin
     if(gap<127) gap<=gap+1'b1;
     else if(output_drained) state<=CLOSE_FRAME;
    end
    CLOSE_FRAME:begin out_frame<=1;out_good<=1;gap<=0;state<=CLOSE_GAP;end
    CLOSE_GAP:if(gap==63)state<=IDLE;else gap<=gap+1'b1;
    default:begin fault<=1;state<=IDLE;end
   endcase
   // Complete an in-progress replay before returning to another video mode.
   if(cancel && !rendering) state<=IDLE;
  end
 end
 // synthesis translate_off
 initial begin
  if(WIDTH%8 || HEIGHT%8 || LOW_W!=WIDTH/8 || LOW_H!=HEIGHT/8 ||
     LOW_W%GRID_X || LOW_H%GRID_Y || K>255 || N>16384 || LOW_W>256 || LOW_H>256)
   $fatal(1,"unsupported region geometry");
 end
 // synthesis translate_on
endmodule
