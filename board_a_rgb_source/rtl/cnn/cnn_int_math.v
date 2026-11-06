// Shared integer arithmetic. Wide products are explicitly registered so the
// multiply and rounding/saturation paths do not form one long pixel-clock path.
module cnn_requantize(
 input wire clk,rst_n,start,
 input wire signed [31:0] value,
 input wire [31:0] multiplier,
 input wire relu,
 output reg done,
 output reg signed [31:0] result
);

 reg [1:0] phase;
 reg busy,positive_only;
 reg [31:0] p00,p01;
 reg signed [32:0] p10,p11;
 reg signed [64:0] low_sum,high_sum,product;
 (* dsp_style="yes" *) wire [31:0] m00=value[15:0]*multiplier[15:0];
 (* dsp_style="yes" *) wire [31:0] m01=value[15:0]*multiplier[31:16];
 (* dsp_style="yes" *) wire signed [32:0] m10=$signed(value[31:16])*$signed({1'b0,multiplier[15:0]});
 (* dsp_style="yes" *) wire signed [32:0] m11=$signed(value[31:16])*$signed({1'b0,multiplier[31:16]});
 wire signed [64:0] extended00=$signed({33'd0,p00});
 wire signed [64:0] extended01=$signed({33'd0,p01});
 wire signed [64:0] extended10={{32{p10[32]}},p10};
 wire signed [64:0] extended11={{32{p11[32]}},p11};
 wire signed [64:0] rounded=(product+65'sd536870912) >>> 30;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   busy<=0;phase<=0;positive_only<=0;done<=0;result<=0;
   p00<=0;p01<=0;p10<=0;p11<=0;low_sum<=0;high_sum<=0;product<=0;
  end else begin
   done<=0;
   if(start && !busy)begin
    busy<=1;phase<=0;positive_only<=relu;
    p00<=m00;p01<=m01;p10<=m10;p11<=m11;
   end else if(busy)begin
    case(phase)
     0:begin low_sum<=extended00+(extended10<<16);high_sum<=(extended01<<16)+(extended11<<32);phase<=1;end
     1:begin product<=low_sum+high_sum;phase<=2;end
     2:begin
      done<=1;busy<=0;
      if(positive_only && rounded<0)result<=0;
      else if(positive_only && rounded>65535)result<=65535;
      else result<=rounded[31:0];
     end
    endcase
   end
  end
 end

endmodule

module cnn_integer_sqrt(
 input wire clk,rst_n,start,
 input wire [31:0] radicand,
 output reg done,
 output reg [15:0] result
);
 reg busy;
 reg [4:0] count;
 reg [31:0] bits;
 reg [33:0] remainder;
 reg [15:0] root;
 wire [33:0] trial=(remainder<<2)|bits[31:30];
 wire [33:0] subtract={16'd0,root,2'b01};
 wire yes=trial>=subtract;
 wire [15:0] next_root={root[14:0],yes};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin done<=0;result<=0;busy<=0;count<=0;bits<=0;remainder<=0;root<=0;end
  else begin
   done<=0;
   if(start && !busy)begin busy<=1;count<=0;bits<=radicand;remainder<=0;root<=0;end
   else if(busy)begin
    remainder<=yes ? trial-subtract : trial;root<=next_root;bits<=bits<<2;count<=count+1'b1;
    if(count==15)begin busy<=0;done<=1;result<=next_root;end
   end
  end
 end
endmodule

// Restoring unsigned 64/32 divider. Constant-time and shared by geometry/color;
// it cannot block video because it only operates on the captured analysis RAM.
module cnn_unsigned_divider(
 input wire clk,rst_n,start,
 input wire [63:0] numerator,
 input wire [31:0] denominator,
 output reg busy,done,
 output reg [63:0] quotient,
 output reg [31:0] remainder
);
 reg [63:0] bits,q;
 reg [32:0] r;
 reg [31:0] d;
 reg [6:0] count;
 wire [32:0] trial={r[31:0],bits[63]};
 wire subtract=trial>={1'b0,d};
 wire [32:0] next_r=subtract ? trial-{1'b0,d} : trial;
 wire [63:0] next_q={q[62:0],subtract};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin busy<=0;done<=0;quotient<=0;remainder<=0;bits<=0;q<=0;r<=0;d<=0;count<=0;end
  else begin
   done<=0;
   if(start && !busy)begin
    if(denominator==0)begin quotient<=64'hffffffffffffffff;remainder<=numerator[31:0];done<=1;end
    else begin bits<=numerator;d<=denominator;r<=0;q<=0;count<=0;busy<=1;end
   end else if(busy)begin
    bits<={bits[62:0],1'b0};q<=next_q;r<=next_r;count<=count+1'b1;
    if(count==63)begin busy<=0;done<=1;quotient<=next_q;remainder<=next_r[31:0];end
   end
  end
 end
endmodule
