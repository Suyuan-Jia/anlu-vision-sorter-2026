`ifndef B_MOTION_DDR_PORT_V
`define B_MOTION_DDR_PORT_V
// One analysis transaction at a time; legacy commands are never gated by it.
// Bundled-data toggle CDC retains the complete command until acknowledgement.
// Read data is ordered by the existing DDR controller. A countdown identifies
// the sole analysis response without stopping subsequent legacy reads.
module motion_ddr_port(
 input link_clk,ddr_clk,rst_n,
 input request,write,input [24:0] address,input [127:0] wdata,
 output accept,output reg response,output reg [127:0] rdata,
 input legacy_wr,legacy_rd,input [24:0] legacy_address,input [127:0] legacy_wdata,
 output legacy_ready,legacy_rvalid,output [127:0] legacy_rdata,
 output user_wr,user_rd,output [24:0] user_address,output [127:0] user_wdata,
 input user_ready,user_rvalid,input [127:0] user_rdata);
 reg req_toggle,held_write;
 reg [24:0] held_address;
 reg [127:0] held_data;
 reg ack_toggle;
 reg [127:0] returned;
 (* ASYNC_REG="TRUE" *) reg [2:0] ack_sync,req_sync;
 reg ack_seen,req_seen,pending,pending_write,waiting;
 reg [24:0] pending_address;
 reg [127:0] pending_data;
 reg [10:0] legacy_outstanding,ahead;
 assign accept=(ack_sync[2]==req_toggle) && (ack_sync[2]==ack_seen);
 always @(posedge link_clk or negedge rst_n)
   if(!rst_n)begin
     req_toggle<=0;held_write<=0;held_address<=0;held_data<=0;
     ack_sync<=0;ack_seen<=0;response<=0;rdata<=0;
   end else begin
     ack_sync<={ack_sync[1:0],ack_toggle};response<=0;
     if(request && accept)begin
       held_write<=write;held_address<=address;held_data<=wdata;req_toggle<=~req_toggle;
     end
     if(ack_sync[2]!=ack_seen)begin ack_seen<=ack_sync[2];rdata<=returned;response<=1;end
   end
 wire legacy_any=legacy_wr || legacy_rd;
 wire grant=pending && !waiting && !legacy_any && user_ready;
 wire analysis_return=user_rvalid && waiting && ahead==0;
 wire legacy_return=user_rvalid && !analysis_return;
 wire legacy_read=user_ready && !legacy_wr && legacy_rd;
 assign legacy_ready=user_ready;
 assign legacy_rvalid=legacy_return;
 assign legacy_rdata=user_rdata;
 assign user_wr=legacy_wr || (grant && pending_write);
 assign user_rd=(!legacy_wr && legacy_rd) || (grant && !pending_write);
 assign user_address=legacy_any ? legacy_address : pending_address;
 assign user_wdata=legacy_any ? legacy_wdata : pending_data;
 always @(posedge ddr_clk or negedge rst_n)
   if(!rst_n)begin
     req_sync<=0;req_seen<=0;pending<=0;pending_write<=0;pending_address<=0;pending_data<=0;
     waiting<=0;ahead<=0;legacy_outstanding<=0;ack_toggle<=0;returned<=0;
   end else begin
     req_sync<={req_sync[1:0],req_toggle};
     if(req_sync[2]!=req_seen)begin
       req_seen<=req_sync[2];pending<=1;pending_write<=held_write;
       pending_address<=held_address;pending_data<=held_data;
     end
     case({legacy_read,legacy_return})
       2'b10:legacy_outstanding<=legacy_outstanding+1'b1;
       2'b01:legacy_outstanding<=legacy_outstanding-1'b1;
       default:;
     endcase
     if(grant)begin
       if(pending_write)begin pending<=0;returned<=0;ack_toggle<=~ack_toggle;end
       else begin waiting<=1;ahead<=legacy_outstanding-(legacy_return?1'b1:1'b0);end
     end
     if(waiting && user_rvalid)begin
       if(ahead!=0)ahead<=ahead-1'b1;
       else begin waiting<=0;pending<=0;returned<=user_rdata;ack_toggle<=~ack_toggle;end
     end
   end
endmodule

`endif
