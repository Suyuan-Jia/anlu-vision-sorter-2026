// Normal video plus numbered diagnostics. Each source clock domain has its own snapshot.
module camera_fault_live_hud(
 input wire pixel_clk,link_clk,ddr_clk,rst_n,
 input wire [319:0] link_stats,
 input wire [383:0] fault_stats,
 input wire [287:0] ddr_stats,
 input wire [31:0] underflows,
 input wire [23:0] video_rgb,
 input wire de,vsync,
 output reg [23:0] rgb
);
 reg vs_last,de_last;
 reg [10:0] x,y;
 wire [703:0] all_link_snapshot;
 wire [319:0] link_snapshot=all_link_snapshot[319:0];
 wire [287:0] ddr_snapshot;
 reg [31:0] underflow_snapshot;
 wire trigger=vsync && !vs_last;
 fault_probe_snapshot #(.WIDTH(704)) ls(link_clk,pixel_clk,rst_n,trigger,{fault_stats,link_stats},all_link_snapshot);
 fault_probe_snapshot #(.WIDTH(288)) ds(ddr_clk,pixel_clk,rst_n,trigger,ddr_stats,ddr_snapshot);
 wire [1023:0] snapshot={all_link_snapshot[703:320],ddr_snapshot[287:256],underflow_snapshot,ddr_snapshot[255:0],link_snapshot};
 always @(posedge pixel_clk or negedge rst_n)
   if(!rst_n) begin vs_last<=0;de_last<=0;x<=0;y<=0;underflow_snapshot<=0;end
   else begin
     vs_last<=vsync;de_last<=de;
     if(trigger) begin y<=0;underflow_snapshot<=underflows;end
     else if(de_last && !de) y<=y+1'b1;
     if(!de) x<=0;else x<=x+1'b1;
   end
 function [6:0] seven(input [3:0] n);
   case(n)
     0:seven=7'b1111110; 1:seven=7'b0110000; 2:seven=7'b1101101; 3:seven=7'b1111001;
     4:seven=7'b0110011; 5:seven=7'b1011011; 6:seven=7'b1011111; 7:seven=7'b1110000;
     8:seven=7'b1111111; 9:seven=7'b1111011; 10:seven=7'b1110111; 11:seven=7'b0011111;
     12:seven=7'b1001110; 13:seven=7'b0111101; 14:seven=7'b1001111; 15:seven=7'b1000111;
   endcase
 endfunction
 integer j;
 reg [4:0] row;
 reg [10:0] xx,dy,dx;
 reg [3:0] digit,nibble;
 reg [31:0] number;
 reg [6:0] seg;
 reg lit,ink;
 always @* begin
   xx=x>=640 ? x-640 : x;
   row=x>=640 ? 16 : 0;
   dy=y-200;
   for(j=1;j<16;j=j+1) if(y>=200+j*32) begin row=(x>=640 ? 16:0)+j; dy=y-(200+j*32); end
   number=snapshot>>(row*32);
   digit=0; dx=xx-150;
   for(j=1;j<8;j=j+1) if(xx>=150+j*34) begin digit=j; dx=xx-(150+j*34); end
   nibble=number>>((7-digit)*4);
   ink=xx>=150 && xx<422;
   if(xx>=40 && xx<74) begin nibble={3'd0,row[4]}; dx=xx-40; ink=1; end
   if(xx>=74 && xx<108) begin nibble=row[3:0]; dx=xx-74; ink=1; end
   seg=seven(nibble);
   lit=(seg[6] && dy>=2 && dy<5 && dx>=6 && dx<24)||
       (seg[5] && dx>=24 && dx<27 && dy>=5 && dy<15)||
       (seg[4] && dx>=24 && dx<27 && dy>=18 && dy<28)||
       (seg[3] && dy>=28 && dy<31 && dx>=6 && dx<24)||
       (seg[2] && dx>=3 && dx<6 && dy>=18 && dy<28)||
       (seg[1] && dx>=3 && dx<6 && dy>=5 && dy<15)||
       (seg[0] && dy>=15 && dy<18 && dx>=6 && dx<24);
   rgb=video_rgb;
   if(y>=200 && y<712) rgb=24'h101820;
   if(y>=200 && y<712 && lit && ink) rgb=xx<108 ? 24'h00ffff : 24'hffffff;
   if(x==639 && y>=200 && y<712) rgb=24'h405060;
 end
endmodule

module fault_probe_snapshot #(parameter WIDTH=32)(
 input wire source_clk,dest_clk,rst_n,trigger,
 input wire [WIDTH-1:0] source_data,
 output reg [WIDTH-1:0] snapshot
);
 reg request,ack,ack_last;
 reg [1:0] request_sync,ack_sync;
 reg [WIDTH-1:0] held;
 always @(posedge source_clk or negedge rst_n)
   if(!rst_n) begin request_sync<=0;ack<=0;held<=0;end
   else begin
     request_sync<={request_sync[0],request};
     if(request_sync[1]!=ack) begin held<=source_data;ack<=request_sync[1];end
   end
 always @(posedge dest_clk or negedge rst_n)
   if(!rst_n) begin request<=0;ack_sync<=0;ack_last<=0;snapshot<=0;end
   else begin
     ack_sync<={ack_sync[0],ack};
     if(trigger && ack_sync[1]==request) request<=~request;
     if(ack_sync[1]!=ack_last) begin snapshot<=held;ack_last<=ack_sync[1];end
   end
endmodule
