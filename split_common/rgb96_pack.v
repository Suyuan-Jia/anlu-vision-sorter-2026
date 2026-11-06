// Four MSB-first RGB888 groups -> three 128-bit words. Valid bubbles do
// not reset phase. SOF may coincide with the first valid group.
module rgb96_pack(
 input wire clk,rst_n,frame,valid,
 input wire [95:0] data,
 output reg o_frame,o_valid,
 output reg [127:0] o_data
);
 reg [1:0] phase;
 reg [95:0] held;
 wire [1:0] use_phase=frame ? 2'd0 : phase;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin phase<=0;held<=0;o_frame<=0;o_valid<=0;o_data<=0;end
   else begin
     o_frame<=frame;o_valid<=0;
     if(frame) phase<=0;
     if(valid) begin
       held<=data;phase<=use_phase+1'b1;
       case(use_phase)
         1:begin o_valid<=1;o_data<={held,data[95:64]};end
         2:begin o_valid<=1;o_data<={held[63:0],data[95:32]};end
         3:begin o_valid<=1;o_data<={held[31:0],data};end
         default:;
       endcase
     end
   end
 end
endmodule
