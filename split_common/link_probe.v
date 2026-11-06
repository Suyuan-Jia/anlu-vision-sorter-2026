// Phase-1 observability only. Video/control beats are unchanged. Telemetry
// occupies 19 otherwise idle cycles inside the existing 64-cycle SOF guard.
module probe_source(
 input wire clk,rst_n,frame,valid,raw_frame,raw_valid,color_frame,edge_frame,
 input wire [7:0] mode,
 input wire request,
 output reg ack,
 output reg [255:0] held
);
 reg [1:0] sync;
 reg started;
 reg [31:0] seq,words,last_words,bad,raw_count,coincident,colors,edges;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     sync<=0; ack<=0; held<=0; started<=0; seq<=0; words<=0;
     last_words<=0; bad<=0; raw_count<=0; coincident<=0; colors<=0; edges<=0;
   end else begin
     sync<={sync[0],request};
     if(sync[1]!=ack) begin
       held<={edges,colors,24'd0,mode,coincident,raw_count,bad,last_words,seq};
       ack<=sync[1];
     end
     if(raw_frame) raw_count<=raw_count+1'b1;
     if(raw_frame && raw_valid) coincident<=coincident+1'b1;
     if(color_frame) colors<=colors+1'b1;
     if(edge_frame) edges<=edges+1'b1;
     if(frame) begin
       started<=1; words<=valid ? 1 : 0;
       if(started) begin
         seq<=seq+1'b1; last_words<=words;
         if(words!=172800) bad<=bad+1'b1;
       end
     end else if(valid && started) words<=words+1'b1;
   end
 end
endmodule

module probe_tx(
 input wire clk,rst_n,
 input wire [23:0] data,
 input wire valid,sop,frame,
 input wire ack,
 input wire [255:0] source_held,
 output reg request,
 output wire [23:0] output_data
);
 function [31:0] crc24(input [31:0] c,input [23:0] d);
   reg [31:0] t; integer k;
   begin t=c; for(k=0;k<24;k=k+1) t=(t[0]^d[k]) ? (t>>1)^32'hedb88320 : t>>1; crc24=t; end
 endfunction
 reg [1:0] ack_sync;
 reg [255:0] source_snapshot;
 reg started;
 reg [31:0] seq,beats,last_beats,errors;
 reg [3:0] part;
 reg [5:0] slot;
 reg [383:0] payload;
 reg [31:0] crc;
 wire marker=sop && frame && !valid;
 wire idle=!valid && !sop && !frame;
 wire [23:0] telemetry=(slot==2) ? 24'hd1a603 :
      (slot>=3 && slot<=18) ? payload[23:0] :
      slot==19 ? crc[23:0] : {16'h5ac3,crc[31:24]};
 assign output_data=(idle && slot>=2 && slot<=20) ? telemetry : data;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     ack_sync<=0; source_snapshot<=0; request<=0; started<=0;
     seq<=0; beats<=0; last_beats<=0; errors<=0; part<=0;
     slot<=0; payload<=0; crc<=32'hffffffff;
   end else begin
     ack_sync<={ack_sync[0],ack};
     if(ack_sync[1]==request) source_snapshot<=source_held;
     if(marker) begin
       started<=1; beats<=0; part<=0; slot<=1;
       if(started) begin seq<=seq+1'b1; last_beats<=beats; end
       if(part!=0) errors<=errors+1'b1;
       if(ack_sync[1]==request) request<=~request;
     end else begin
       if(valid && started) beats<=beats+1'b1;
       if(started) begin
         if((valid && sop!=(part==0)) || (!valid && part!=0) || frame)
           errors<=errors+1'b1;
         if(valid) part<=part+1'b1; else part<=0;
       end
       if(slot!=0) begin
         if(!idle || slot==20) slot<=0; else slot<=slot+1'b1;
         if(slot==1) begin
           payload<={32'h00010000,errors,last_beats,seq,source_snapshot};
           crc<=32'hffffffff;
         end else if(slot>=3 && slot<=18) begin
           crc<=crc24(crc,payload[23:0]); payload<=payload>>24;
         end
       end
     end
   end
 end
endmodule

module probe_telemetry_rx(
 input wire clk,rst_n,
 input wire [26:0] captured,
 output reg [383:0] payload,
 output reg [31:0] packets,errors
);
 function [31:0] crc24(input [31:0] c,input [23:0] d);
   reg [31:0] t; integer k;
   begin t=c; for(k=0;k<24;k=k+1) t=(t[0]^d[k]) ? (t>>1)^32'hedb88320 : t>>1; crc24=t; end
 endfunction
 reg [4:0] slot;
 reg [383:0] building;
 reg [31:0] crc;
 reg [23:0] crc_low;
 reg window;
 reg [6:0] age;
 wire [23:0] data=captured[23:0];
 wire marker=captured[26:24]==3'b110;
 wire idle=captured[26:24]==0;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin payload<=0; packets<=0; errors<=0; slot<=0; building<=0; crc<=0; crc_low<=0; window<=0; age<=0; end
   else begin
     if(marker) begin
       if(slot!=0) errors<=errors+1'b1;
       window<=1; age<=0; slot<=0;
     end else begin
       if(window) begin age<=age+1'b1; if(age==63 || !idle) window<=0; end
       if(slot==0) begin
         if(window && idle && data==24'hd1a603) begin slot<=1; crc<=32'hffffffff; end
       end else if(!idle) begin slot<=0; errors<=errors+1'b1; end
       else if(slot<=16) begin
         building<={data,building[383:24]}; crc<=crc24(crc,data); slot<=slot+1'b1;
       end else if(slot==17) begin crc_low<=data; slot<=18; end
       else begin
         slot<=0; window<=0;
         if(data[23:8]==16'h5ac3 && {data[7:0],crc_low}==crc) begin payload<=building; packets<=packets+1'b1; end
         else errors<=errors+1'b1;
       end
     end
   end
 end
endmodule
