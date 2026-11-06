`ifndef B_MOTION_DISPLAY_V
`define B_MOTION_DISPLAY_V
// Coherent result mailbox, latched only on a display frame boundary. It adds
// no pixel/sync registers; the accepted mixer retains its existing latency.
module motion_display(input clk,rst_n,input [2:0] mode,
 input video_user,video_last,video_de,input [23:0] rgb,output [23:0] rgb_out,
 input [44:0] result,input result_toggle,output reg result_ack);
 (* ASYNC_REG="TRUE" *) reg [2:0] result_sync;
 reg [2:0] mode_meta,mode_sync;
 reg [44:0] held;
 reg [2:0] age;
 reg [10:0] x,y;
 wire [10:0] x0=held[43:33],y0=held[32:22],x1=held[21:11],y1=held[10:0];
 wire inside_box=x>=x0 && x<=x1 && y>=y0 && y<=y1;
 wire border=inside_box && (x<x0+2 || x+2>x1 || y<y0+2 || y+2>y1);
 assign rgb_out=(video_de && mode_sync==5 && held[44] && age<6 && border)?24'h00ff40:rgb;
 always @(posedge clk or negedge rst_n)
   if(!rst_n)begin result_sync<=0;result_ack<=0;mode_meta<=1;mode_sync<=1;held<=0;age<=7;x<=0;y<=0;end
   else begin
     result_sync<={result_sync[1:0],result_toggle};mode_meta<=mode;mode_sync<=mode_meta;
     if(video_user)begin
       x<=0;y<=0;
       if(result_sync[2]!=result_ack)begin held<=result;result_ack<=result_sync[2];age<=0;end
       else if(age<7)age<=age+1'b1;
     end else if(video_de)begin if(video_last)begin x<=0;y<=y+1'b1;end else x<=x+1'b1;end
   end
endmodule

`endif
