// Largest 8-connected component of the classification mask. The bitmap has
// two bits (raw/clean) per pixel; probabilities and shared features stay intact.
// FIFO storage is independent of the live shared-feature arena.
// Each component has at most 14400 active pixels; no label-count truncation.
(* dsp_style="no" *) module cnn_roi_select(
 input wire clk,rst_n,start,use_clean,
 input wire mask_wr,
 input wire [13:0] mask_addr,
 input wire [1:0] mask_data,
 output reg busy,done,
 output reg [7:0] x0,y0,x1,y1,
 output reg [14:0] queue_rd_addr,queue_wr_addr,
 input wire [15:0] queue_rd_data,
 output reg queue_wr,
 output reg [15:0] queue_wr_data
);
 localparam IDLE=0,SCAN=1,WAIT_MASK=2,CHECK=3,POP=4,WAIT_QUEUE=5,
  USE_QUEUE=6,NEIGHBOR=7,WAIT_NEIGHBOR=8,CHECK_NEIGHBOR=9,STEP=10,
  COMPONENT=11,ADVANCE=12;
 reg [3:0] state;
 reg clean;
 reg [7:0] sx,sy,px,py,lx,ly,hx,hy;
 reg [3:0] neighbor;
 reg [14:0] head,tail;
 reg [14:0] area,best_area;
 reg bitmap_wr;
 reg [13:0] bitmap_wa,bitmap_ra;
 reg [1:0] bitmap_data;
 wire [1:0] bitmap_q;
 wire mask_present=clean ? bitmap_q[1] : bitmap_q[0];
 wire signed [9:0] nx=$signed({2'b0,px})+$signed({8'd0,neighbor%3})-10'sd1;
 wire signed [9:0] ny=$signed({2'b0,py})+$signed({8'd0,neighbor/3})-10'sd1;
 wire within_bounds=nx>=0 && nx<160 && ny>=3 && ny<93;
 wire [13:0] na=ny*160+nx;
 function [14:0] queue_address;
  input [14:0] index;
  begin queue_address=index;end
 endfunction
 cnn_block_ram #(.WIDTH(2),.AW(14),.DEPTH(15360)) bitmap(
  clk,mask_wr || bitmap_wr,mask_wr ? mask_addr : bitmap_wa,bitmap_ra,
  mask_wr ? mask_data : bitmap_data,bitmap_q);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;busy<=0;done<=0;clean<=0;sx<=0;sy<=3;px<=0;py<=3;
   lx<=0;ly<=0;hx<=0;hy<=0;neighbor<=0;head<=0;tail<=0;area<=0;best_area<=0;
   bitmap_wr<=0;bitmap_wa<=0;bitmap_ra<=0;bitmap_data<=0;
   queue_wr<=0;queue_rd_addr<=0;queue_wr_addr<=0;queue_wr_data<=0;
   x0<=0;y0<=3;x1<=160;y1<=93;
  end else begin
   done<=0;bitmap_wr<=0;queue_wr<=0;
   case(state)
    IDLE:if(start)begin
     clean<=use_clean;busy<=1;best_area<=0;sx<=0;sy<=3;
     x0<=0;y0<=3;x1<=160;y1<=93;state<=SCAN;
    end
    SCAN:begin bitmap_ra<=sy*160+sx;state<=WAIT_MASK;end
    WAIT_MASK:state<=CHECK;
    CHECK:if(mask_present)begin
     bitmap_wr<=1;bitmap_wa<=sy*160+sx;bitmap_data<=0;
     queue_wr<=1;queue_wr_addr<=queue_address(0);queue_wr_data<={sy,sx};
     head<=0;tail<=1;area<=0;lx<=sx;ly<=sy;hx<=sx+1'b1;hy<=sy+1'b1;state<=POP;
    end else state<=ADVANCE;
    POP:if(head==tail)state<=COMPONENT;
     else begin queue_rd_addr<=queue_address(head);state<=WAIT_QUEUE;end
    WAIT_QUEUE:state<=USE_QUEUE;
    USE_QUEUE:begin
     px<=queue_rd_data[7:0];py<=queue_rd_data[15:8];head<=head+1'b1;area<=area+1'b1;
     if(queue_rd_data[7:0]<lx)lx<=queue_rd_data[7:0];
     if(queue_rd_data[15:8]<ly)ly<=queue_rd_data[15:8];
     if(queue_rd_data[7:0]+1'b1>hx)hx<=queue_rd_data[7:0]+1'b1;
     if(queue_rd_data[15:8]+1'b1>hy)hy<=queue_rd_data[15:8]+1'b1;
     neighbor<=0;state<=NEIGHBOR;
    end
    NEIGHBOR:if(!within_bounds || neighbor==4)state<=STEP;
     else begin bitmap_ra<=na;state<=WAIT_NEIGHBOR;end
    WAIT_NEIGHBOR:state<=CHECK_NEIGHBOR;
    CHECK_NEIGHBOR:begin
     if(mask_present)begin
      bitmap_wr<=1;bitmap_wa<=na;bitmap_data<=0;
      queue_wr<=1;queue_wr_addr<=queue_address(tail);queue_wr_data<={ny[7:0],nx[7:0]};tail<=tail+1'b1;
     end
     state<=STEP;
    end
    STEP:if(neighbor==8)state<=POP;else begin neighbor<=neighbor+1'b1;state<=NEIGHBOR;end
    COMPONENT:begin
     if(area>best_area)begin best_area<=area;x0<=lx;y0<=ly;x1<=hx;y1<=hy;end
     state<=ADVANCE;
    end
    ADVANCE:if(sx==159)begin sx<=0;if(sy==92)begin busy<=0;done<=1;state<=IDLE;end
     else begin sy<=sy+1'b1;state<=SCAN;end end
     else begin sx<=sx+1'b1;state<=SCAN;end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
