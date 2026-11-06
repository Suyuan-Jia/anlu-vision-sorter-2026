// Shared-feature ROI pooling and soft_roi foreground attention. This consumes
// features of the SAME backbone; it is not a second mask-image recognizer.
// Memory read request is registered by the caller, data usable two edges later.
(* dsp_style="no" *) module cnn_roi_engine(
 input wire clk,rst_n,start,
 input wire [7:0] box_x0,box_x1,box_y0,box_y1,
 input wire [31:0] input_multiplier,
 output reg busy,done,
 output reg [2:0] rd_bank,
 output reg [14:0] rd_addr,
 input wire [15:0] rd_data,
 output reg wr,
 output reg [12:0] wr_addr,
 output reg [7:0] wr_data
);
 localparam IDLE=0,COORD_START=1,COORD_WAIT=2,CELL=3,READ_WAIT=4,READ_USE=5,
            MUL_Y=6,MUL_X=7,ACCUM=8,FINISH_CELL=9,ATTENTION=10,ATT_MUL=11,QUANT_MUL=12,WRITE=13,COORD_READ=14;
 reg [4:0] state;
 reg [10:0] lx,hx,ly,hy;
 reg [7:0] coordinate_index;
 reg signed [31:0] coordinates[0:191] /* synthesis ram_style=block */;
 reg [15:0] soft_roi[0:255] /* synthesis ram_style=block */;
 reg div_start;
 reg [63:0] div_n;
 reg [31:0] div_d;
 wire div_busy,div_done;wire [63:0] div_q;wire [31:0] div_r;
 cnn_unsigned_divider divider(clk,rst_n,div_start,div_n,div_d,div_busy,div_done,div_q,div_r);
 wire [1:0] coord_group=coordinate_index>>6;
 wire coord_y=coordinate_index[5];
 wire [6:0] tick={1'b0,coordinate_index[4:0],1'b1};
 wire [7:0] coord_size=coord_y ? (coord_group==0 ? 48 : coord_group==1 ? 24 : 96) :
                                             (coord_group==0 ? 80 : coord_group==1 ? 40 : 160);
 wire [10:0] coord_lo=coord_y ? ly : lx,coord_hi=coord_y ? hy : hx;
 (* dsp_style="yes" *) wire [64:0] shared_product;
 wire [31:0] coord_base=(coord_lo<<6)+shared_product[31:0];
 wire [31:0] coord_x_scaled=coord_group==0 ? (coord_base<<6)+(coord_base<<4) :
                           coord_group==1 ? (coord_base<<5)+(coord_base<<3) : (coord_base<<7)+(coord_base<<5);
 wire [31:0] coord_y_scaled=coord_group==0 ? (coord_base<<5)+(coord_base<<4) :
                           coord_group==1 ? (coord_base<<4)+(coord_base<<3) : (coord_base<<6)+(coord_base<<5);
 wire [31:0] coord_numerator=coord_y ? coord_y_scaled : coord_x_scaled;
 reg [4:0] ch;
 reg [7:0] roi_index;
 reg [1:0] sample_index;
 reg [1:0] neighbor;
 wire [1:0] feature_group=ch==16 ? 2 : ch>=8 ? 1 : 0;
 wire [7:0] feature_w=feature_group==0 ? 80 : feature_group==1 ? 40 : 160;
 wire [7:0] feature_h=feature_group==0 ? 48 : feature_group==1 ? 24 : 96;
 wire [7:0] coord_x_index=feature_group*64+{roi_index[3:0],sample_index[0]};
 wire [7:0] coord_y_index=feature_group*64+32+{roi_index[7:4],sample_index[1]};
 reg signed [31:0] sx,sy;
 reg [15:0] soft_value;
 always @(posedge clk)begin sx<=coordinates[coord_x_index];sy<=coordinates[coord_y_index];soft_value<=soft_roi[roi_index];end
 // Explicit signed operands: mixing an unsigned one-bit neighbor into >>>
 // changes a negative edge coordinate into an unsigned logical shift.
 wire signed [16:0] base_x=$signed(sx[31:16]),base_y=$signed(sy[31:16]);
 wire signed [16:0] ix=base_x+$signed({16'd0,neighbor[0]}),iy=base_y+$signed({16'd0,neighbor[1]});
 wire within_bounds=ix>=0 && iy>=0 && ix<feature_w && iy<feature_h;
 reg [16:0] wx,wy;
 reg [15:0] value,pooled,attended;
 reg [16:0] attention;
 reg [32:0] product_y,product_attention;
 reg [49:0] product_xy;
 reg [47:0] product_quant;
 reg [63:0] sum;
 wire [32:0] shared_a=state==COORD_START ? {22'd0,(coord_hi-coord_lo)} :
                         state==ATTENTION ? {17'd0,soft_value} :
                         state==MUL_Y ? {17'd0,value} : state==MUL_X ? product_y :
                         state==ATT_MUL ? {17'd0,pooled} :
                         ch==16 ? {17'd0,attended} : ((product_attention+33'd32768)>>16);
 wire [31:0] shared_b=state==COORD_START ? {25'd0,tick} : state==ATTENTION ? 32'd58982 :
                         state==MUL_Y ? {15'd0,wy} : state==MUL_X ? {15'd0,wx} :
                         state==ATT_MUL ? {15'd0,attention} : input_multiplier;
 assign shared_product=shared_a*shared_b;
 wire [14:0] ch_wide={10'd0,ch};
 wire [14:0] fine_ch={10'd0,ch-5'd4},mid_ch={10'd0,ch-5'd8};
 wire [14:0] row_wide={7'd0,iy[7:0]};
 wire [63:0] pooled_wide=(sum+64'd8589934592)>>34;
 wire [47:0] quant_rounded=(product_quant+48'd536870912)>>30;
 wire [10:0] input_dx=box_x1-box_x0<2 ? 2 : box_x1-box_x0;
 wire [10:0] input_dy=box_y1-box_y0<2 ? 2 : box_y1-box_y0;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;busy<=0;done<=0;wr<=0;wr_addr<=0;wr_data<=0;rd_bank<=0;rd_addr<=0;
   lx<=0;hx<=0;ly<=0;hy<=0;coordinate_index<=0;div_start<=0;div_n<=0;div_d<=1;
   ch<=16;roi_index<=0;sample_index<=0;neighbor<=0;sum<=0;wx<=0;wy<=0;value<=0;pooled<=0;attention<=0;
   product_y<=0;product_xy<=0;product_attention<=0;attended<=0;product_quant<=0;
  end else begin
   done<=0;wr<=0;div_start<=0;
   case(state)
    IDLE:if(start)begin
     busy<=1;coordinate_index<=0;
     lx<=box_x0*10<input_dx ? 0 : box_x0*10-input_dx;
     hx<=box_x1*10+input_dx>1600 ? 1600 : box_x1*10+input_dx;
     ly<=box_y0*10<input_dy ? 0 : box_y0*10-input_dy;
     hy<=box_y1*10+input_dy>960 ? 960 : box_y1*10+input_dy;
     state<=COORD_START;
    end
    COORD_START:begin div_n<={32'd0,coord_numerator}<<16;div_d<=coord_y ? 61440 : 102400;div_start<=1;state<=COORD_WAIT;end
    COORD_WAIT:if(div_done)begin
     coordinates[coordinate_index]<=$signed(div_q[31:0])-32'sd32768;
     if(coordinate_index==191)begin ch<=16;roi_index<=0;sample_index<=0;neighbor<=0;sum<=0;state<=COORD_READ;end
     else begin coordinate_index<=coordinate_index+1'b1;state<=COORD_START;end
    end
    COORD_READ:state<=CELL;
    CELL:begin
     wx<=neighbor[0] ? {1'b0,sx[15:0]} : 17'd65536-{1'b0,sx[15:0]};
     wy<=neighbor[1] ? {1'b0,sy[15:0]} : 17'd65536-{1'b0,sy[15:0]};
     if(!within_bounds)begin value<=0;state<=MUL_Y;end
     else begin
      if(ch==16)begin rd_bank<=3;rd_addr<=(row_wide<<7)+(row_wide<<5)+ix;end
      else if(ch<4)begin rd_bank<=1;rd_addr<=(ch_wide<<12)-(ch_wide<<8)+(row_wide<<6)+(row_wide<<4)+ix;end
      else if(ch<8)begin rd_bank<=0;rd_addr<=11520+(fine_ch<<12)-(fine_ch<<8)+(row_wide<<6)+(row_wide<<4)+ix;end
      else begin rd_bank<=0;rd_addr<=3840+(mid_ch<<10)-(mid_ch<<6)+(row_wide<<5)+(row_wide<<3)+ix;end
      state<=READ_WAIT;
     end
    end
    READ_WAIT:state<=READ_USE;
    READ_USE:begin value<=ch==16 ? (rd_data[7:0]*1024+127)/255 : rd_data;state<=MUL_Y;end
    MUL_Y:begin product_y<=shared_product[32:0];state<=MUL_X;end
    MUL_X:begin product_xy<=shared_product[49:0];state<=ACCUM;end
    ACCUM:begin
     sum<=sum+product_xy;
     if(neighbor==3)begin
      neighbor<=0;
      if(sample_index==3)begin sample_index<=0;state<=FINISH_CELL;end
      else begin sample_index<=sample_index+1'b1;state<=COORD_READ;end
     end else begin neighbor<=neighbor+1'b1;state<=COORD_READ;end
    end
    FINISH_CELL:begin
     pooled<=pooled_wide[15:0];sum<=0;
     if(ch==16)begin soft_roi[roi_index]<=pooled_wide[15:0];attended<=pooled_wide[15:0];state<=QUANT_MUL;end
     else state<=ATTENTION;
    end
    ATTENTION:begin attention<=17'd6554+((shared_product+65'd512)>>10);state<=ATT_MUL;end
    ATT_MUL:begin product_attention<=shared_product[32:0];state<=QUANT_MUL;end
    QUANT_MUL:begin
     product_quant<=shared_product[47:0];
     state<=WRITE;
    end
    WRITE:begin
     wr<=1;wr_addr<=ch*256+roi_index;wr_data<=quant_rounded>255 ? 8'd255 : quant_rounded[7:0];
     if(roi_index==255)begin
      roi_index<=0;
      if(ch==16)begin ch<=0;state<=COORD_READ;end
      else if(ch==15)begin busy<=0;done<=1;state<=IDLE;end
      else begin ch<=ch+1'b1;state<=COORD_READ;end
     end else begin roi_index<=roi_index+1'b1;state<=COORD_READ;end
    end
    default:state<=IDLE;
   endcase
  end
 end
endmodule

