// Exact rounded channel/sum normalization using a small serial divider.
// No reciprocal approximation, large lookup table, variable multiply or DSP.
(* dsp_style="no" *) module cnn_rgb_normalize(
 input wire clk,rst_n,start,input wire [15:0] rgb,
 output reg done,output reg [15:0] normalized
);
 localparam IDLE=0,PREPARE=1,DIVIDE=2,SAVE=3;
 reg [1:0] state,channel;
 reg [7:0] r,g,b,nr,ng;
 reg [9:0] total;
 reg [15:0] dividend;
 reg [9:0] remainder;
 reg [4:0] bit_count;
 wire [10:0] trial={remainder,dividend[15]};
 wire subtract=trial>={1'b0,total};
 wire [7:0] source=channel==0 ? r : channel==1 ? g : b;
 wire [7:0] red=rgb[15:11]*255/31,green=rgb[10:5]*255/63,blue=rgb[4:0]*255/31;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=IDLE;channel<=0;r<=0;g<=0;b<=0;nr<=0;ng<=0;
   total<=0;dividend<=0;remainder<=0;bit_count<=0;done<=0;normalized<=0;end
  else begin
   done<=0;
   case(state)
    IDLE:if(start)begin
     r<=red;g<=green;b<=blue;total<={2'b0,red}+green+blue;channel<=0;state<=PREPARE;
    end
    PREPARE:if(total==0)begin normalized<=0;done<=1;state<=IDLE;end
     else begin dividend<=({source,8'b0}-{8'b0,source})+(total>>1);remainder<=0;bit_count<=0;state<=DIVIDE;end
    DIVIDE:begin
     remainder<=subtract ? trial-total : trial;
     dividend<={dividend[14:0],subtract};
     if(bit_count==15)state<=SAVE;else bit_count<=bit_count+1'b1;
    end
    SAVE:begin
     if(channel==0)nr<=dividend[7:0];
     if(channel==1)ng<=dividend[7:0];
     if(channel==2)begin normalized<={nr[7:3],ng[7:2],dividend[7:3]};done<=1;state<=IDLE;end
     else begin channel<=channel+1'b1;state<=PREPARE;end
    end
   endcase
  end
 end
endmodule
