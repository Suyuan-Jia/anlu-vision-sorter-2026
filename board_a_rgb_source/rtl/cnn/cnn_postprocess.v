// Integer postprocessing AFTER the joint CNN. 8-connected largest component,
// true area centroid, eroded-mask color voting from the SAME cached RGB frame.
// Probability RAM is reused as foreground/visited flags; arena as BFS queue.
(* dsp_style="no" *) module cnn_postprocess(
 input wire clk,rst_n,start,
 input wire [1:0] shape,
 output reg busy,done,result_valid,
 output reg [2:0] color_id,
 output reg [10:0] xmin,ymin,xmax,ymax,cx,cy,
 output reg [15:0] area,
 output reg [2:0] rd_bank,
 output reg [14:0] rd_addr,
 input wire [15:0] rd_data,
 output reg wr,
 output reg [2:0] wr_bank,
 output reg [14:0] wr_addr,
 output reg [15:0] wr_data,
 output reg [7:0] seed_r,seed_g,
 output reg seed_valid,
 output reg [9:0] seed_sum
);
 localparam IDLE=0,MEM_WAIT=1,MEM_USE=2,INIT_READ=3,INIT_WRITE=4,SCAN_READ=5,SCAN_CHECK=6,
  ROOT_PUSH=7,ROOT_MARK=8,POP=9,POP_GET=10,PIXEL_UPDATE=11,ERODE_READ=12,ERODE_CHECK=13,ERODE_STEP=14,
  RGB_READ=15,RGB_CHECK=16,NORM_START=17,NORM_WAIT=18,PROTO_PRODUCT=19,PROTO_ACCUM=20,RANK=21,
  SQRT1_WAIT=22,SQRT2_START=23,SQRT2_WAIT=24,VOTE=25,NEIGH_START=26,NEIGH_READ=27,NEIGH_CHECK=28,
  NEIGH_PUSH=29,NEIGH_MARK=30,NEIGH_STEP=31,COMPLETE_COMPONENT=32,ADVANCE_SCAN=33,FINISH_ALL=34,
  CENTER_X_START=35,CENTER_X_WAIT=36,CENTER_Y_START=37,CENTER_Y_WAIT=38,PUBLISH=39;

 localparam SEED_R_START=40,SEED_R_WAIT=41,SEED_G_START=42,SEED_G_WAIT=43,SEED_SUM_START=44,SEED_SUM_WAIT=45;
 reg [23:0] rgb_sum_r,rgb_sum_g,rgb_sum_b,best_rgb_r,best_rgb_g,best_rgb_b;
 wire [25:0] best_rgb_total={2'd0,best_rgb_r}+{2'd0,best_rgb_g}+{2'd0,best_rgb_b};
 wire [23:0] best_hi_rg=best_rgb_r>best_rgb_g ? best_rgb_r : best_rgb_g;
 wire [23:0] best_lo_rg=best_rgb_r<best_rgb_g ? best_rgb_r : best_rgb_g;
 wire [23:0] best_hi=best_hi_rg>best_rgb_b ? best_hi_rg : best_rgb_b;
 wire [23:0] best_lo=best_lo_rg<best_rgb_b ? best_lo_rg : best_rgb_b;
 reg [5:0] state,next_state;
 reg [15:0] memory_value;
 task read_memory;
  input [2:0] bank;
  input [14:0] address;
  input [5:0] after_read;
  begin rd_bank<=bank;rd_addr<=address;next_state<=after_read;state<=MEM_WAIT;end
 endtask
 reg [14:0] init_index;
 reg [7:0] scan_x,scan_y,px,py;
 reg [14:0] head,tail;
 reg [15:0] visited[0:959] /* synthesis ram_style=dram */;
 reg [3:0] neighbor;
 reg eroded,pixel_inner;
 wire signed [10:0] nx=$signed({3'd0,px})+$signed(neighbor%4'd3)-11'sd1;
 wire signed [10:0] ny=$signed({3'd0,py})+$signed(neighbor/4'd3)-11'sd1;
 wire neighbor_inside=nx>=0 && nx<160 && ny>=3 && ny<93;
 wire [13:0] scan_address=scan_y*160+scan_x;
 wire [13:0] neighbor_address=ny*160+nx;
 wire [13:0] visit_address=state==SCAN_CHECK || state==ROOT_MARK ? scan_address : neighbor_address;
 wire [15:0] visit_word=visited[visit_address[13:4]];
 wire visited_bit=visit_word[visit_address[3:0]];
 reg [15:0] component_area,inner_area,best_area,best_used_area;
 reg [31:0] sum_x,sum_y,best_sum_x,best_sum_y;
 reg [7:0] x0,y0,x1,y1,bx0,by0,bx1,by1;
 reg [15:0] votes_all[1:4],votes_inner[1:4],best_votes[1:4];
 reg [7:0] r,g,b;
 wire [7:0] r8=(memory_value[15:11]*255)/31;
 wire [7:0] g8=(memory_value[10:5]*255)/63;
 wire [7:0] b8=(memory_value[4:0]*255)/31;
 wire [7:0] hi_rg=r8>g8 ? r8 : g8,lo_rg=r8<g8 ? r8 : g8;
 wire [7:0] hi=hi_rg>b8 ? hi_rg : b8,lo=lo_rg<b8 ? lo_rg : b8;
 wire [1:0] clipped={1'b0,(r8>=248)}+{1'b0,(g8>=248)}+{1'b0,(b8>=248)};
 reg [9:0] rgb_sum;
 reg [1:0] norm_channel;
 reg [12:0] normalized[0:2];
 reg div_start;reg [63:0] div_n;reg [31:0] div_d;
 wire div_busy,div_done;wire [63:0] div_q;wire [31:0] div_rem;
 cnn_unsigned_divider divider(clk,rst_n,div_start,div_n,div_d,div_busy,div_done,div_q,div_rem);
 reg [5:0] prototype_index;
 reg [41:0] prototype;
 // GENERATED COLOR BEGIN
localparam PROTOTYPE_LAST=39;
always @* begin
case(prototype_index)
0:prototype=42'h0a1409ce2c8;
1:prototype=42'h0a3fc710379;
2:prototype=42'h0a26875e3b7;
3:prototype=42'h0a4f86fe343;
4:prototype=42'h0a100718434;
5:prototype=42'h0a074bac20d;
6:prototype=42'h0a5e870e2ff;
7:prototype=42'h0a38c7a8349;
8:prototype=42'h0a2f47043c1;
9:prototype=42'h0a1d47243fa;
10:prototype=42'h11018c7e5bb;
11:prototype=42'h11024bd460d;
12:prototype=42'h11180b9e5d1;
13:prototype=42'h10fdcc405e9;
14:prototype=42'h11030cba597;
15:prototype=42'h10f84cb05c7;
16:prototype=42'h110dcbd65de;
17:prototype=42'h10f34c5c606;
18:prototype=42'h1109cc1c5cc;
19:prototype=42'h1119cbe45a7;
20:prototype=42'h188e14e6355;
21:prototype=42'h18a99266426;
22:prototype=42'h188354463d0;
23:prototype=42'h18b1d39436f;
24:prototype=42'h188815ee2e9;
25:prototype=42'h187c5530377;
26:prototype=42'h1884136643d;
27:prototype=42'h189b153e2f5;
28:prototype=42'h189fd460351;
29:prototype=42'h189e53943bd;
30:prototype=42'h21d9ccbc23c;
31:prototype=42'h21b90dc8237;
32:prototype=42'h21d04c90277;
33:prototype=42'h21ad0d68298;
34:prototype=42'h21d00cda253;
35:prototype=42'h21bf4dfa206;
36:prototype=42'h21b34d9a266;
37:prototype=42'h21d6cc7e267;
38:prototype=42'h21bccca82b9;
39:prototype=42'h21cf8c4a29d;
default:prototype=0;
endcase
end
 // GENERATED COLOR END
 wire signed [13:0] delta_r=$signed({1'b0,normalized[0]})-$signed({1'b0,prototype[38:26]});
 wire signed [13:0] delta_g=$signed({1'b0,normalized[1]})-$signed({1'b0,prototype[25:13]});
 wire signed [13:0] delta_b=$signed({1'b0,normalized[2]})-$signed({1'b0,prototype[12:0]});
 (* dsp_style="yes" *) wire signed [27:0] product_r=delta_r*delta_r;
 (* dsp_style="yes" *) wire signed [27:0] product_g=delta_g*delta_g;
 (* dsp_style="yes" *) wire signed [27:0] product_b=delta_b*delta_b;
 reg [27:0] square_r,square_g,square_b;
 wire [27:0] distance_squared=square_r+square_g+square_b;
 reg [31:0] distances[1:4];
 reg [31:0] rank_best,rank_second;
 reg [2:0] rank_label;
 reg [2:0] pixel_label;
 reg [31:0] second_squared;
 reg sqrt_start;reg [31:0] sqrt_input;wire sqrt_done;wire [15:0] sqrt_result;
 reg [15:0] first_sqrt;
 cnn_integer_sqrt square_root(clk,rst_n,sqrt_start,sqrt_input,sqrt_done,sqrt_result);
 reg [17:0] accepted_votes;
 reg [15:0] winning_votes;
 reg [2:0] winning_label;
 reg [15:0] component_strong,best_strong;
 integer i;
 always @*begin
  rank_best=32'hffffffff;rank_second=32'hffffffff;rank_label=0;
  for(integer k=1;k<=4;k=k+1)begin
   if(distances[k]<rank_best)begin rank_second=rank_best;rank_best=distances[k];rank_label=k;end
   else if(distances[k]<rank_second)rank_second=distances[k];
  end
  accepted_votes=0;winning_votes=0;winning_label=0;
  for(integer k=1;k<=4;k=k+1)begin
   accepted_votes=accepted_votes+best_votes[k];
   if(best_votes[k]>winning_votes)begin winning_votes=best_votes[k];winning_label=k;end
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   seed_sum<=0;seed_r<=0;seed_g<=0;seed_valid<=0;rgb_sum_r<=0;rgb_sum_g<=0;rgb_sum_b<=0;best_rgb_r<=0;best_rgb_g<=0;best_rgb_b<=0;component_strong<=0;best_strong<=0;
   state<=IDLE;next_state<=IDLE;memory_value<=0;busy<=0;done<=0;result_valid<=0;color_id<=0;
   xmin<=0;ymin<=0;xmax<=0;ymax<=0;cx<=0;cy<=0;area<=0;
   rd_bank<=0;rd_addr<=0;wr<=0;wr_bank<=0;wr_addr<=0;wr_data<=0;
   init_index<=0;scan_x<=0;scan_y<=3;px<=0;py<=3;head<=0;tail<=0;neighbor<=0;eroded<=0;pixel_inner<=0;
   component_area<=0;inner_area<=0;best_area<=0;best_used_area<=0;sum_x<=0;sum_y<=0;best_sum_x<=0;best_sum_y<=0;
   x0<=160;y0<=96;x1<=0;y1<=0;bx0<=0;by0<=0;bx1<=0;by1<=0;
   r<=0;g<=0;b<=0;rgb_sum<=0;norm_channel<=0;div_start<=0;div_n<=0;div_d<=1;
   prototype_index<=0;square_r<=0;square_g<=0;square_b<=0;pixel_label<=0;second_squared<=0;
   sqrt_start<=0;sqrt_input<=0;first_sqrt<=0;
   for(i=0;i<3;i=i+1)normalized[i]<=0;
   for(i=1;i<=4;i=i+1)begin votes_all[i]<=0;votes_inner[i]<=0;best_votes[i]<=0;distances[i]<=32'hffffffff;end
  end else begin
   done<=0;wr<=0;div_start<=0;sqrt_start<=0;
   case(state)
    IDLE:if(start)begin seed_valid<=0;busy<=1;result_valid<=0;color_id<=0;area<=0;init_index<=0;best_area<=0;best_strong<=0;scan_x<=0;scan_y<=3;state<=INIT_READ;end
    MEM_WAIT:state<=MEM_USE;
    MEM_USE:begin memory_value<=rd_data;state<=next_state;end
    INIT_READ:state<=INIT_WRITE;
    INIT_WRITE:begin
     visited[init_index]<=0;
     if(init_index==959)state<=SCAN_READ;else begin init_index<=init_index+1'b1;state<=INIT_READ;end
    end
    SCAN_READ:read_memory(3,scan_y*160+scan_x,SCAN_CHECK);
    SCAN_CHECK:if(memory_value[7:0]>=102 && !visited_bit)begin
     rgb_sum_r<=0;rgb_sum_g<=0;rgb_sum_b<=0;head<=0;tail<=0;component_strong<=0;component_area<=0;inner_area<=0;sum_x<=0;sum_y<=0;x0<=160;y0<=96;x1<=0;y1<=0;
     for(i=1;i<=4;i=i+1)begin votes_all[i]<=0;votes_inner[i]<=0;end
     state<=ROOT_PUSH;
    end else state<=ADVANCE_SCAN;
    ROOT_PUSH:begin component_strong<=memory_value[7:0]>=204 ? 16'd1 : 16'd0;wr<=1;wr_bank<=0;wr_addr<=0;wr_data<={1'b0,scan_y[6:0],scan_x};tail<=1;state<=ROOT_MARK;end
    ROOT_MARK:begin visited[visit_address[13:4]]<=visit_word|(16'd1<<visit_address[3:0]);state<=POP;end
    POP:if(head==tail)state<=COMPLETE_COMPONENT;else read_memory(0,head,POP_GET);
    POP_GET:begin px<=memory_value[7:0];py<={1'b0,memory_value[14:8]};head<=head+1'b1;state<=PIXEL_UPDATE;end
    PIXEL_UPDATE:begin
     component_area<=component_area+1'b1;sum_x<=sum_x+px;sum_y<=sum_y+py;
     if(px<x0)x0<=px;if(py<y0)y0<=py;if(px>x1)x1<=px;if(py>y1)y1<=py;
     neighbor<=0;eroded<=1;state<=ERODE_READ;
    end
    ERODE_READ:if(nx<0 || nx>=160 || ny<0 || ny>=96)state<=ERODE_STEP;
     else if(ny<3 || ny>=93)begin eroded<=0;state<=ERODE_STEP;end else read_memory(3,ny*160+nx,ERODE_CHECK);
    ERODE_CHECK:begin if(memory_value[7:0]<102)eroded<=0;state<=ERODE_STEP;end
    ERODE_STEP:if(neighbor==8)begin pixel_inner<=eroded;if(eroded)inner_area<=inner_area+1'b1;state<=RGB_READ;end
               else begin neighbor<=neighbor+1'b1;state<=ERODE_READ;end
    RGB_READ:if(shape==0)state<=NEIGH_START;else read_memory(2,(py-3)*160+px,RGB_CHECK);
    RGB_CHECK:begin
     rgb_sum_r<=rgb_sum_r+r8;rgb_sum_g<=rgb_sum_g+g8;rgb_sum_b<=rgb_sum_b+b8;
     if(hi<32 || 510*(hi-lo)<59*hi || clipped>=2)state<=NEIGH_START;
     else begin r<=r8;g<=g8;b<=b8;rgb_sum<={2'b0,r8}+{2'b0,g8}+{2'b0,b8};norm_channel<=0;state<=NORM_START;end
    end
    NORM_START:begin div_start<=1;div_n<=((norm_channel==0 ? {56'd0,r} : norm_channel==1 ? {56'd0,g} : {56'd0,b})<<12)+(rgb_sum>>1);div_d<={22'd0,rgb_sum};state<=NORM_WAIT;end
    NORM_WAIT:if(div_done)begin
     normalized[norm_channel]<=div_q[12:0];
     if(norm_channel==2)begin prototype_index<=0;for(i=1;i<=4;i=i+1)distances[i]<=32'hffffffff;state<=PROTO_PRODUCT;end
     else begin norm_channel<=norm_channel+1'b1;state<=NORM_START;end
    end
    PROTO_PRODUCT:begin square_r<=product_r;square_g<=product_g;square_b<=product_b;state<=PROTO_ACCUM;end
    PROTO_ACCUM:begin
     if(distance_squared<distances[prototype[41:39]])distances[prototype[41:39]]<=distance_squared;
     if(prototype_index==PROTOTYPE_LAST)state<=RANK;else begin prototype_index<=prototype_index+1'b1;state<=PROTO_PRODUCT;end
    end
    RANK:begin
     pixel_label<=rank_label;second_squared<=rank_second;
     if(rank_best>167772)state<=NEIGH_START;
     else begin sqrt_start<=1;sqrt_input<=rank_best;state<=SQRT1_WAIT;end
    end
    SQRT1_WAIT:if(sqrt_done)begin first_sqrt<=sqrt_result;state<=SQRT2_START;end
    SQRT2_START:begin sqrt_start<=1;sqrt_input<=second_squared;state<=SQRT2_WAIT;end
    SQRT2_WAIT:if(sqrt_done)begin if(sqrt_result-first_sqrt>=21)state<=VOTE;else state<=NEIGH_START;end
    VOTE:begin votes_all[pixel_label]<=votes_all[pixel_label]+1'b1;if(pixel_inner)votes_inner[pixel_label]<=votes_inner[pixel_label]+1'b1;state<=NEIGH_START;end
    NEIGH_START:begin neighbor<=0;state<=NEIGH_READ;end
    NEIGH_READ:if(neighbor==4 || !neighbor_inside)state<=NEIGH_STEP;else read_memory(3,ny*160+nx,NEIGH_CHECK);
    NEIGH_CHECK:if(memory_value[7:0]>=102 && !visited_bit)state<=NEIGH_PUSH;else state<=NEIGH_STEP;
    NEIGH_PUSH:begin if(memory_value[7:0]>=204)component_strong<=component_strong+1'b1;wr<=1;wr_bank<=0;wr_addr<=tail;wr_data<={1'b0,ny[6:0],nx[7:0]};tail<=tail+1'b1;state<=NEIGH_MARK;end
    NEIGH_MARK:begin visited[visit_address[13:4]]<=visit_word|(16'd1<<visit_address[3:0]);state<=NEIGH_STEP;end
    NEIGH_STEP:if(neighbor==8)state<=POP;else begin neighbor<=neighbor+1'b1;state<=NEIGH_READ;end
    COMPLETE_COMPONENT:begin
     if(component_area>=16 && component_area>best_area)begin
      best_rgb_r<=rgb_sum_r;best_rgb_g<=rgb_sum_g;best_rgb_b<=rgb_sum_b;best_area<=component_area;best_strong<=component_strong;best_sum_x<=sum_x;best_sum_y<=sum_y;bx0<=x0;by0<=y0;bx1<=x1;by1<=y1;
      best_used_area<=inner_area>=12 ? inner_area : component_area;
      for(i=1;i<=4;i=i+1)best_votes[i]<=inner_area>=12 ? votes_inner[i] : votes_all[i];
     end
     state<=ADVANCE_SCAN;
    end
    ADVANCE_SCAN:if(scan_x==159)begin scan_x<=0;if(scan_y==92)state<=FINISH_ALL;else begin scan_y<=scan_y+1'b1;state<=SCAN_READ;end end
                 else begin scan_x<=scan_x+1'b1;state<=SCAN_READ;end
    FINISH_ALL:begin
     area<=best_area;
     if(best_area<16 || shape==0 || 5*best_strong<3*best_area)begin result_valid<=0;color_id<=0;state<=PUBLISH;end
     else begin
      result_valid<=1;
      color_id<=accepted_votes>=8 && 25*accepted_votes>=3*best_used_area && 20*winning_votes>=11*accepted_votes ? winning_label : 0;
      xmin<=bx0*8;xmax<=(bx1+1)*8-1;ymin<=(by0-3)*8;ymax<=(by1-2)*8-1;
      state<=CENTER_X_START;
     end
    end
    CENTER_X_START:begin div_start<=1;div_n<=best_sum_x*8+best_area*4+(best_area>>1);div_d<=best_area;state<=CENTER_X_WAIT;end
    CENTER_X_WAIT:if(div_done)begin cx<=div_q[10:0];state<=CENTER_Y_START;end
    CENTER_Y_START:begin div_start<=1;div_n<=(best_sum_y-best_area*3)*8+best_area*4+(best_area>>1);div_d<=best_area;state<=CENTER_Y_WAIT;end
    CENTER_Y_WAIT:if(div_done)begin cy<=div_q[10:0];state<=SEED_R_START;end
    SEED_R_START:begin
     seed_valid<=best_rgb_total!=0 && (best_hi-best_lo)>(best_rgb_total>>5);
     div_start<=1;div_n<=({40'd0,best_rgb_r}<<8)+(best_rgb_total>>1);div_d<=best_rgb_total;state<=SEED_R_WAIT;
    end
    SEED_R_WAIT:if(div_done)begin seed_r<=div_q>255 ? 8'd255 : div_q[7:0];state<=SEED_G_START;end
    SEED_G_START:begin div_start<=1;div_n<=({40'd0,best_rgb_g}<<8)+(best_rgb_total>>1);div_d<=best_rgb_total;state<=SEED_G_WAIT;end
    SEED_G_WAIT:if(div_done)begin seed_g<=div_q>255 ? 8'd255 : div_q[7:0];state<=SEED_SUM_START;end
    SEED_SUM_START:begin div_start<=1;div_n<=best_rgb_total+(best_area>>1);div_d<=best_area;state<=SEED_SUM_WAIT;end
    SEED_SUM_WAIT:if(div_done)begin seed_sum<=div_q[9:0];state<=PUBLISH;end
    PUBLISH:begin done<=1;busy<=0;state<=IDLE;end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
