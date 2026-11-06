// Region support plus DDR implementation modules. Keep all DDR definitions here
// so both existing TD GUI source lists and the updated .al resolve them once.
// The old separate DDR source paths are compatibility placeholders only.
// Fixed synchronous dual-port storage: retain NOREG/READBEFOREWRITE semantics.
module region_ram #(parameter WIDTH=16, AW=14, DEPTH=16384)(
 input clk, input we, input [AW-1:0] wa,ra,
 input [WIDTH-1:0] wd, output [WIDTH-1:0] rd);
 PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(WIDTH),.DATA_WIDTH_B(WIDTH),
 .ADDR_WIDTH_A(AW),.ADDR_WIDTH_B(AW),.DATA_DEPTH_A(DEPTH),.DATA_DEPTH_B(DEPTH),
 .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
 .WRITEMODE_A("NORMAL"),.WRITEMODE_B("READBEFOREWRITE"),
 .IMPLEMENT("20K(FAST)"),.ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
 .CLKMODE("SYNC"),.RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
 .INIT_FILE("NONE"),.FILL_ALL("NONE")) ram(
 .clka(clk),.clkb(clk),.cea(1'b1),.ceb(1'b1),.ocea(1'b0),.oceb(1'b0),
 .wea(we),.web(1'b0),.addra(wa),.addrb(ra),.dia(wd),.dib({WIDTH{1'b0}}),
 .bea(1'b0),.beb(1'b0),.rsta(1'b0),.rstb(1'b0),.doa(),.dob(rd),
 .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
endmodule

// One bounded restoring divider is shared by all region mean calculations.
module region_divider(input clk,rst_n,start,
 input [23:0] numerator, input [15:0] denominator,
 output reg done, output reg [23:0] quotient);
 reg [23:0] bits_left;
 reg [16:0] remainder;
 reg [15:0] divisor;
 reg [4:0] count;
 reg busy;
 wire [17:0] shifted={remainder,bits_left[23]};
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin busy<=0;done<=0;count<=0;quotient<=0;bits_left<=0;remainder<=0;divisor<=1;end
  else begin
   done<=0;
   if(start && !busy) begin
    bits_left<=numerator; divisor<=(denominator==0)?16'd1:denominator;
    remainder<=0;quotient<=0;count<=24;busy<=1;
   end else if(busy) begin
    bits_left<=bits_left<<1;
    if(shifted>={2'b0,divisor}) begin remainder<=shifted-divisor;quotient<={quotient[22:0],1'b1};end
    else begin remainder<=shifted;quotient<={quotient[22:0],1'b0};end
    count<=count-1'b1;
    if(count==1) begin busy<=0;done<=1;end
   end
  end
 end
endmodule

// ==== region_ddr_cache.v (bundled for existing TD source lists) ====
// Four fully associative 128-bit cache lines. One outstanding DDR transaction.
// Client address/data stay stable while hit=0. Dirty eviction precedes refill.
module region_ddr_cache #(parameter LINES=4)(
 input clk,rst_n,invalidate,read_en,write_en,
 input [24:0] address,input [127:0] write_data,input [15:0] byte_enable,
 output hit,output [127:0] read_data,
 output reg request,output reg wr,output reg [24:0] mem_addr,
 output reg [127:0] mem_wdata,input accept,response,input [127:0] mem_rdata);
 reg [127:0] data[0:LINES-1];reg [24:0] tags[0:LINES-1];
 reg [LINES-1:0] valid,dirty;
 reg [$clog2(LINES)-1:0] replace,index,victim;
 reg found;reg [2:0] state;reg [24:0] pending_address;
 integer i,b;
 always @* begin
  found=0;index=0;
  for(i=0;i<LINES;i=i+1)if(valid[i] && tags[i]==address)begin found=1;index=i;end
 end
 assign hit=found && state==0 && !invalidate;
 assign read_data=data[index];
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin valid<=0;dirty<=0;replace<=0;victim<=0;state<=0;request<=0;wr<=0;mem_addr<=0;mem_wdata<=0;pending_address<=0;end
  else begin
   case(state)
    0:begin
     if(invalidate)begin valid<=0;dirty<=0;end
     else if((read_en || write_en) && !found)begin
      victim<=replace;pending_address<=address;request<=1;
      if(valid[replace] && dirty[replace])begin wr<=1;mem_addr<=tags[replace];mem_wdata<=data[replace];state<=1;end
      else begin wr<=0;mem_addr<=address;state<=3;end
     end else if(write_en && found)begin
      for(b=0;b<16;b=b+1)if(byte_enable[b])data[index][b*8+:8]<=write_data[b*8+:8];
      dirty[index]<=1;
     end
    end
    1:if(accept)begin request<=0;state<=2;end
    2:if(response)begin dirty[victim]<=0;wr<=0;mem_addr<=pending_address;request<=1;state<=3;end
    3:if(accept)begin request<=0;state<=4;end
    4:if(response)begin data[victim]<=mem_rdata;tags[victim]<=pending_address;valid[victim]<=1;dirty[victim]<=0;replace<=replace+1'b1;state<=0;end
    default:state<=0;
   endcase
  end
 end
endmodule

// Join two cache masters without allowing responses to change owners.
module region_cache_join(input clk,rst_n,
 input a_req,a_wr,input [24:0] a_addr,input [127:0] a_data,output a_accept,a_response,
 input b_req,b_wr,input [24:0] b_addr,input [127:0] b_data,output b_accept,b_response,
 output request,wr,output [24:0] address,output [127:0] data,
 input accept,response,input [127:0] rdata,output [127:0] cache_rdata);
 reg busy,owner;
 assign request=!busy && (a_req || b_req);
 assign wr=a_req?a_wr:b_wr;assign address=a_req?a_addr:b_addr;assign data=a_req?a_data:b_data;
 assign a_accept=request && accept && a_req;
 assign b_accept=request && accept && !a_req;
 assign a_response=response && busy && !owner;
 assign b_response=response && busy && owner;
 assign cache_rdata=rdata;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin busy<=0;owner<=0;end
  else begin if(request && accept)begin busy<=1;owner<=!a_req;end if(response)busy<=0;end
endmodule

// ==== region_ddr_port.v (bundled for existing TD source lists) ====
// B DDR arbiter. Legacy video writes/reads have priority. Native addresses are
// 16-bit units: one 128-bit word advances by eight. Scratch starts above all
// existing four display banks. Native DDR read responses remain issue-ordered.
module region_ddr_port #(parameter WIDTH=1280,HEIGHT=720)(
 input link_clk,ddr_clk,rst_n,
 input capture_begin,capture_bank,
 input raw_valid,input [127:0] raw_data,
 input low_valid,input [127:0] low_data,
 output capture_ready,capture_drained,capture_complete,output reg capture_error,
 input request,wr,input [24:0] address,input [127:0] data,
 output accept,output reg response,output reg [127:0] rdata,
 input legacy_wr,legacy_rd,input [24:0] legacy_addr,input [127:0] legacy_data,
 output legacy_ready,legacy_read_ready,legacy_rvalid,output [127:0] legacy_rdata,
 output user_wr,user_rd,output [24:0] user_addr,output [127:0] user_data,
 input user_ready,user_rvalid,input [127:0] user_rdata,
 output reg fault);
 localparam RAW_WORDS=WIDTH*HEIGHT*3/16,LOW_WORDS=WIDTH*HEIGHT/16/8;
 localparam [24:0] RAW0=24000000,RAW1=26000000,LOW0=28000000,LOW1=28100000;
 wire [127:0] raw_q,low_q;wire raw_full,low_full;
 wire [8:0] raw_used,low_used,raw_rused,low_rused;
 reg raw_pop,low_pop;
 w128_d512_fifo raw_fifo(.rst(!rst_n),.clkw(link_clk),.we(raw_valid&&!raw_full),.di(raw_data),
  .wrusedw(raw_used),.afull(),.full_flag(raw_full),.clkr(ddr_clk),.re(raw_pop),.dout(raw_q),.rdusedw(raw_rused),.valid(),.empty_flag(),.aempty());
 w128_d512_fifo low_fifo(.rst(!rst_n),.clkw(link_clk),.we(low_valid&&!low_full),.di(low_data),
  .wrusedw(low_used),.afull(),.full_flag(low_full),.clkr(ddr_clk),.re(low_pop),.dout(low_q),.rdusedw(low_rused),.valid(),.empty_flag(),.aempty());
 reg cap_toggle,cap_bank_hold;
 reg req_toggle,link_busy;reg req_wr;reg [24:0] req_addr;reg [127:0] req_data;
 reg [2:0] ack_sync;reg ack_seen;
 reg [2:0] cap_sync,req_sync;
 reg cap_seen,req_seen,active_bank,cap_active;
 reg pending,pending_wr,waiting_read;
 reg [24:0] pending_addr;reg [127:0] pending_data;
 reg ack_toggle;reg [127:0] return_data;
 reg [17:0] raw_count;reg [13:0] low_count;
 reg raw_pipe,low_pipe;
 reg [24:0] bulk_addr;
 reg drain_ddr,complete_ddr;reg [2:0] drain_sync,complete_sync;
 wire queues_empty=raw_used==0 && low_used==0 && !raw_full && !low_full;
 assign capture_drained=drain_sync[2] && queues_empty;
 assign capture_complete=complete_sync[2] && capture_drained;
 assign capture_ready=capture_drained;
 assign accept=!link_busy;
 always @(posedge link_clk or negedge rst_n)begin
  if(!rst_n)begin cap_toggle<=0;cap_bank_hold<=0;capture_error<=0;req_toggle<=0;link_busy<=0;
   req_wr<=0;req_addr<=0;req_data<=0;ack_sync<=0;ack_seen<=0;response<=0;rdata<=0;drain_sync<=0;complete_sync<=0;end
  else begin
   response<=0;ack_sync<={ack_sync[1:0],ack_toggle};
   drain_sync<={drain_sync[1:0],drain_ddr};complete_sync<={complete_sync[1:0],complete_ddr};
   if(capture_begin)begin cap_toggle<=~cap_toggle;cap_bank_hold<=capture_bank;capture_error<=0;end
   if((raw_valid&&raw_full)||(low_valid&&low_full))capture_error<=1;
   if(request && accept)begin req_toggle<=~req_toggle;req_wr<=wr;req_addr<=address;req_data<=data;link_busy<=1;end
   if(ack_sync[2]!=ack_seen)begin ack_seen<=ack_sync[2];rdata<=return_data;response<=1;link_busy<=0;end
  end
 end
 // Do not mix display reads and an algorithm read in the controller.
 // Drain outstanding display reads first; the sole worker response then has
 // an unambiguous destination without a speculative ownership FIFO.
 reg [9:0] owner_count;
 wire owner_room=owner_count<512;
 assign legacy_ready=user_ready;
 assign legacy_read_ready=legacy_ready && !legacy_wr && !waiting_read && owner_room;
 assign legacy_rvalid=user_rvalid && owner_count!=0 && !waiting_read;
 assign legacy_rdata=user_rdata;
 wire legacy_any=legacy_wr || legacy_rd;
 wire bulk_pipe=raw_pipe || low_pipe;
 wire worker_grant=!legacy_any && !bulk_pipe && pending && !waiting_read && user_ready && (pending_wr || owner_count==0);
 wire worker_read=worker_grant&&!pending_wr;
 wire worker_write=worker_grant&&pending_wr;
 // A FIFO pop has one-cycle read latency. Latch its word and wait for a legacy
 // video command gap, including any already-launched delayed video write.
 reg bulk_wait;
 wire cap_metadata_stable=(cap_sync[2]==cap_seen && cap_sync[1]==cap_seen && cap_sync[0]==cap_seen);
 wire bulk_can_pop=!legacy_any && !pending && !bulk_pipe && !bulk_wait && user_ready && cap_active && cap_metadata_stable;
 always @*begin raw_pop=0;low_pop=0;
  if(bulk_can_pop)begin if(raw_rused!=0)raw_pop=1;else if(low_rused!=0)low_pop=1;end
 end
 // Buffered bulk word waits rather than colliding with a delayed video write.
 reg bulk_valid;reg [127:0] bulk_data_hold;reg [24:0] bulk_addr_hold;
 wire bulk_send=!legacy_any && bulk_valid && user_ready;
 assign user_wr=legacy_wr || (!legacy_any && (bulk_send || worker_write));
 assign user_rd=(!legacy_wr && legacy_rd) || (!legacy_any && !bulk_send && worker_read);
 assign user_addr=legacy_any?legacy_addr:bulk_send?bulk_addr_hold:pending_addr;
 assign user_data=legacy_any?legacy_data:bulk_send?bulk_data_hold:pending_data;
 wire push_owner=user_rd;wire pop_owner=user_rvalid && owner_count!=0;
 always @(posedge ddr_clk or negedge rst_n)begin
  if(!rst_n)begin cap_sync<=0;req_sync<=0;cap_seen<=0;req_seen<=0;active_bank<=0;cap_active<=0;
   pending<=0;pending_wr<=0;waiting_read<=0;pending_addr<=0;pending_data<=0;ack_toggle<=0;return_data<=0;
   raw_count<=0;low_count<=0;raw_pipe<=0;low_pipe<=0;bulk_addr<=0;bulk_valid<=0;bulk_wait<=0;
   bulk_data_hold<=0;bulk_addr_hold<=0;owner_count<=0;fault<=0;drain_ddr<=1;complete_ddr<=0;end
  else begin
   cap_sync<={cap_sync[1:0],cap_toggle};req_sync<={req_sync[1:0],req_toggle};
   if(cap_sync[2]!=cap_seen)begin cap_seen<=cap_sync[2];active_bank<=cap_bank_hold;cap_active<=1;raw_count<=0;low_count<=0;complete_ddr<=0;end
   if(req_sync[2]!=req_seen)begin req_seen<=req_sync[2];pending<=1;pending_wr<=req_wr;pending_addr<=req_addr;pending_data<=req_data;end
   raw_pipe<=raw_pop;low_pipe<=low_pop;
   if(raw_pop)begin bulk_addr<=(active_bank?RAW1:RAW0)+raw_count*8;raw_count<=raw_count+1'b1;bulk_wait<=1;end
   if(low_pop)begin bulk_addr<=(active_bank?LOW1:LOW0)+low_count*8;low_count<=low_count+1'b1;bulk_wait<=1;end
   if(bulk_pipe)begin bulk_valid<=1;bulk_data_hold<=raw_pipe?raw_q:low_q;bulk_addr_hold<=bulk_addr;end
   if(bulk_send)begin bulk_valid<=0;bulk_wait<=0;end
   if(worker_write && !bulk_send)begin pending<=0;return_data<=0;ack_toggle<=~ack_toggle;end
   if(worker_read && !bulk_send)waiting_read<=1;
   if(pop_owner)begin
    if(waiting_read)begin return_data<=user_rdata;ack_toggle<=~ack_toggle;pending<=0;waiting_read<=0;end
   end
   case({push_owner,pop_owner})2'b10:owner_count<=owner_count+1'b1;2'b01:owner_count<=owner_count-1'b1;default:;endcase
   if(user_rvalid && owner_count==0)fault<=1;
   if(owner_count>512)fault<=1;
   drain_ddr<=raw_rused==0 && low_rused==0 && !bulk_wait && !bulk_pipe && !bulk_valid;
   complete_ddr<=raw_count==RAW_WORDS && low_count==LOW_WORDS && raw_rused==0 && low_rused==0 && !bulk_wait && !bulk_valid;
  end
 end
endmodule

// ==== region_cartoon_ddr.v (bundled for existing TD source lists) ====
// DDR-backed integer regions, default 320x180 from exact 4x4 RGB565 box means.
// 144 fixed seeds, three assignment passes and two adjacency merge passes.
// Replay refines boundaries with the same captured full-resolution RGB frame.
// Input is never backpressured: whole frames are skipped when both slots are owned.
module region_cartoon_ddr #(
 parameter WIDTH=1280, HEIGHT=720, LOW_W=WIDTH/4, LOW_H=HEIGHT/4,
 parameter GRID_X=16, GRID_Y=9, ITERATIONS=3, MERGE_PASSES=2,
 parameter MERGE_THRESHOLD=24
)(input clk,rst_n,enable,cancel,
 input [95:0] pixels, input valid,sof,frame_end,frame_good,
 input launch,output_ready,output_drained,
 input capture_ready,capture_drained,capture_complete,capture_error,
 output capture_begin,capture_bank,
 output raw_valid,output [127:0] raw_data,
 output reg low_valid,output reg [127:0] low_data,
 output mem_request,mem_wr,output [24:0] mem_address,output [127:0] mem_data,
 input mem_accept,mem_response,input [127:0] mem_rdata,
 output ready,rendering,
 output reg [95:0] out_pixels, output reg out_valid,out_frame,out_good,
 output reg fault,
 output reg [31:0] completed,skipped,processing_cycles);
 localparam N=LOW_W*LOW_H, K=GRID_X*GRID_Y;
 localparam CW=LOW_W/GRID_X, CH=LOW_H/GRID_Y;
 localparam IDLE=0,SEED_ADDR=1,SEED_WAIT=2,SEED_LOAD=3,CLEAR=4,
 PIX_ADDR=5,PIX_WAIT=6,PIX_LOAD=7,CAND_ADDR=8,CAND_EVAL=9,
 ACC_ADDR=10,ACC_WAIT=11,ACC_WRITE=12,NEXT_PIX=13,
 MEAN_ADDR=14,MEAN_WAIT=15,MEAN_LOAD=16,DIV_START=17,DIV_WAIT=18,MEAN_STORE=19,
 PAIR_ADDR=20,PAIR_WAIT=21,PAIR_LOAD=22,ROOT_A=23,ROOT_B=24,
 ROOT_COMPARE=25,MERGE_A_ADDR=26,MERGE_A_WAIT=27,MERGE_A_LOAD=28,
 MERGE_B_ADDR=29,MERGE_B_WAIT=30,MERGE_B_LOAD=31,MERGE_WRITE=32,
 NEXT_PAIR=33,PAL_ROOT=34,PAL_WRITE=35,READY=36,
 OPEN_FRAME=37,OPEN_GAP=38,RENDER_LABEL=39,RENDER_LABEL_WAIT=40,
 RENDER_PAL=41,RENDER_EMIT=42,DRAIN=43,CLOSE_FRAME=44,CLOSE_GAP=45;
 reg [7:0] state;
 reg replaying;
 wire core_ce;
 reg [1:0] cap_state;
 reg cap_bank,work_bank;
 reg [10:0] cap_x,cap_y;
 reg [9:0] horizontal_r,horizontal_g,horizontal_b;
 reg [13:0] columns_r[0:LOW_W-1],columns_g[0:LOW_W-1],columns_b[0:LOW_W-1];
 wire take_capture=(state==IDLE && cap_state==3 && !cancel && capture_complete && !capture_error);
 wire start_capture=sof && enable && cap_state==0 &&
                    (state==IDLE || cap_bank!=work_bank) && capture_ready;
 wire capture_valid=valid && (cap_state==1 || start_capture) && !cancel;
 wire [10:0] use_x=sof?11'd0:cap_x;
 wire [10:0] use_y=sof?11'd0:cap_y;
 wire [9:0] lane_r={2'b0,pixels[95:88]}+{2'b0,pixels[71:64]}+{2'b0,pixels[47:40]}+{2'b0,pixels[23:16]};
 wire [9:0] lane_g={2'b0,pixels[87:80]}+{2'b0,pixels[63:56]}+{2'b0,pixels[39:32]}+{2'b0,pixels[15:8]};
 wire [9:0] lane_b={2'b0,pixels[79:72]}+{2'b0,pixels[55:48]}+{2'b0,pixels[31:24]}+{2'b0,pixels[7:0]};
 wire [8:0] col_index=use_x;
 wire [13:0] total_r=(use_y[1:0]==0?14'd0:columns_r[col_index])+lane_r;
 wire [13:0] total_g=(use_y[1:0]==0?14'd0:columns_g[col_index])+lane_g;
 wire [13:0] total_b=(use_y[1:0]==0?14'd0:columns_b[col_index])+lane_b;
 wire cap_we=capture_valid && use_y[1:0]==3;
 wire [15:0] cap_addr=(use_y>>2)*LOW_W+use_x;
 wire [15:0] cap_rgb={total_r[11:7],total_g[11:6],total_b[11:7]};
 reg [15:0] input_ra;
 wire [15:0] input_rd;
 assign capture_begin=start_capture && valid;
 assign capture_bank=cap_bank;
 wire unused_raw_frame;
 rgb96_pack snapshot_pack(clk,rst_n,capture_begin,capture_valid,pixels,unused_raw_frame,raw_valid,raw_data);
 reg [111:0] low_held;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin low_held<=0;low_valid<=0;low_data<=0;end
  else begin
   low_valid<=0;
   if(cap_we)begin low_held<={low_held[95:0],cap_rgb};
    if(cap_addr[2:0]==7)begin low_data<={low_held,cap_rgb};low_valid<=1;end
   end
  end

 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin cap_state<=0;cap_bank<=0;cap_x<=0;cap_y<=0;
   horizontal_r<=0;horizontal_g<=0;horizontal_b<=0;skipped<=0;end
  else begin
   if(take_capture) begin cap_state<=0;cap_bank<=~cap_bank;end
   if(frame_end) begin
    if(cap_state==2) cap_state<=frame_good?3:0;
    else if(cap_state==1) cap_state<=0;
   end
   if(sof) begin
    if(start_capture) begin cap_state<=1;cap_x<=0;cap_y<=0;end
    else if(enable) begin skipped<=skipped+1'b1;if(cap_state==1) cap_state<=0;end
   end
   if(capture_valid) begin
    if(!use_x[0]) begin horizontal_r<=lane_r;horizontal_g<=lane_g;horizontal_b<=lane_b;end
    if(use_x==WIDTH/4-1) begin cap_x<=0;cap_y<=use_y+1'b1;end
    else begin cap_x<=use_x+1'b1;cap_y<=use_y;end
    if(use_x==WIDTH/4-1 && use_y==HEIGHT-1) cap_state<=2;
   end
   if(cancel || (cap_state==3 && capture_drained && (!capture_complete || capture_error))) cap_state<=0;
  end
 end
 // First row of each 4x4 tile explicitly starts at zero, so RAM needs no reset.
 always @(posedge clk) if(capture_valid) begin
  columns_r[col_index]<=total_r;columns_g[col_index]<=total_g;columns_b[col_index]<=total_b;
 end

 function [23:0] expand565(input [15:0] c);
  begin expand565={{c[15:11],c[15:13]},{c[10:5],c[10:9]},{c[4:0],c[4:2]}};end
 endfunction
 function integer absval(input integer a);begin absval=a<0?-a:a;end endfunction
 function integer color_distance(input [23:0] a,b);
  begin color_distance=absval({1'b0,a[23:16]}-{1'b0,b[23:16]})+
    absval({1'b0,a[15:8]}-{1'b0,b[15:8]})+absval({1'b0,a[7:0]}-{1'b0,b[7:0]});end
 endfunction
 function [7:0] clip(input integer a);begin clip=a<0?0:(a>255?255:a);end endfunction
 function [7:0] region_quant(input [7:0] y);
 begin case(y)
  8'd0:region_quant=8'd0;
  8'd1:region_quant=8'd0;
  8'd2:region_quant=8'd0;
  8'd3:region_quant=8'd0;
  8'd4:region_quant=8'd0;
  8'd5:region_quant=8'd0;
  8'd6:region_quant=8'd0;
  8'd7:region_quant=8'd0;
  8'd8:region_quant=8'd0;
  8'd9:region_quant=8'd0;
  8'd10:region_quant=8'd0;
  8'd11:region_quant=8'd0;
  8'd12:region_quant=8'd0;
  8'd13:region_quant=8'd0;
  8'd14:region_quant=8'd0;
  8'd15:region_quant=8'd0;
  8'd16:region_quant=8'd0;
  8'd17:region_quant=8'd0;
  8'd18:region_quant=8'd0;
  8'd19:region_quant=8'd0;
  8'd20:region_quant=8'd0;
  8'd21:region_quant=8'd23;
  8'd22:region_quant=8'd23;
  8'd23:region_quant=8'd23;
  8'd24:region_quant=8'd23;
  8'd25:region_quant=8'd23;
  8'd26:region_quant=8'd23;
  8'd27:region_quant=8'd23;
  8'd28:region_quant=8'd23;
  8'd29:region_quant=8'd23;
  8'd30:region_quant=8'd23;
  8'd31:region_quant=8'd23;
  8'd32:region_quant=8'd23;
  8'd33:region_quant=8'd23;
  8'd34:region_quant=8'd23;
  8'd35:region_quant=8'd23;
  8'd36:region_quant=8'd23;
  8'd37:region_quant=8'd23;
  8'd38:region_quant=8'd23;
  8'd39:region_quant=8'd23;
  8'd40:region_quant=8'd23;
  8'd41:region_quant=8'd23;
  8'd42:region_quant=8'd23;
  8'd43:region_quant=8'd23;
  8'd44:region_quant=8'd23;
  8'd45:region_quant=8'd23;
  8'd46:region_quant=8'd23;
  8'd47:region_quant=8'd23;
  8'd48:region_quant=8'd23;
  8'd49:region_quant=8'd23;
  8'd50:region_quant=8'd23;
  8'd51:region_quant=8'd23;
  8'd52:region_quant=8'd23;
  8'd53:region_quant=8'd46;
  8'd54:region_quant=8'd46;
  8'd55:region_quant=8'd46;
  8'd56:region_quant=8'd46;
  8'd57:region_quant=8'd46;
  8'd58:region_quant=8'd46;
  8'd59:region_quant=8'd46;
  8'd60:region_quant=8'd46;
  8'd61:region_quant=8'd46;
  8'd62:region_quant=8'd46;
  8'd63:region_quant=8'd46;
  8'd64:region_quant=8'd46;
  8'd65:region_quant=8'd46;
  8'd66:region_quant=8'd46;
  8'd67:region_quant=8'd46;
  8'd68:region_quant=8'd46;
  8'd69:region_quant=8'd46;
  8'd70:region_quant=8'd46;
  8'd71:region_quant=8'd46;
  8'd72:region_quant=8'd46;
  8'd73:region_quant=8'd46;
  8'd74:region_quant=8'd46;
  8'd75:region_quant=8'd46;
  8'd76:region_quant=8'd46;
  8'd77:region_quant=8'd69;
  8'd78:region_quant=8'd69;
  8'd79:region_quant=8'd69;
  8'd80:region_quant=8'd69;
  8'd81:region_quant=8'd69;
  8'd82:region_quant=8'd69;
  8'd83:region_quant=8'd69;
  8'd84:region_quant=8'd69;
  8'd85:region_quant=8'd69;
  8'd86:region_quant=8'd69;
  8'd87:region_quant=8'd69;
  8'd88:region_quant=8'd69;
  8'd89:region_quant=8'd69;
  8'd90:region_quant=8'd69;
  8'd91:region_quant=8'd69;
  8'd92:region_quant=8'd69;
  8'd93:region_quant=8'd69;
  8'd94:region_quant=8'd69;
  8'd95:region_quant=8'd69;
  8'd96:region_quant=8'd69;
  8'd97:region_quant=8'd69;
  8'd98:region_quant=8'd69;
  8'd99:region_quant=8'd92;
  8'd100:region_quant=8'd92;
  8'd101:region_quant=8'd92;
  8'd102:region_quant=8'd92;
  8'd103:region_quant=8'd92;
  8'd104:region_quant=8'd92;
  8'd105:region_quant=8'd92;
  8'd106:region_quant=8'd92;
  8'd107:region_quant=8'd92;
  8'd108:region_quant=8'd92;
  8'd109:region_quant=8'd92;
  8'd110:region_quant=8'd92;
  8'd111:region_quant=8'd92;
  8'd112:region_quant=8'd92;
  8'd113:region_quant=8'd92;
  8'd114:region_quant=8'd92;
  8'd115:region_quant=8'd92;
  8'd116:region_quant=8'd92;
  8'd117:region_quant=8'd115;
  8'd118:region_quant=8'd115;
  8'd119:region_quant=8'd115;
  8'd120:region_quant=8'd115;
  8'd121:region_quant=8'd115;
  8'd122:region_quant=8'd115;
  8'd123:region_quant=8'd115;
  8'd124:region_quant=8'd115;
  8'd125:region_quant=8'd115;
  8'd126:region_quant=8'd115;
  8'd127:region_quant=8'd115;
  8'd128:region_quant=8'd115;
  8'd129:region_quant=8'd115;
  8'd130:region_quant=8'd115;
  8'd131:region_quant=8'd115;
  8'd132:region_quant=8'd115;
  8'd133:region_quant=8'd115;
  8'd134:region_quant=8'd139;
  8'd135:region_quant=8'd139;
  8'd136:region_quant=8'd139;
  8'd137:region_quant=8'd139;
  8'd138:region_quant=8'd139;
  8'd139:region_quant=8'd139;
  8'd140:region_quant=8'd139;
  8'd141:region_quant=8'd139;
  8'd142:region_quant=8'd139;
  8'd143:region_quant=8'd139;
  8'd144:region_quant=8'd139;
  8'd145:region_quant=8'd139;
  8'd146:region_quant=8'd139;
  8'd147:region_quant=8'd139;
  8'd148:region_quant=8'd139;
  8'd149:region_quant=8'd139;
  8'd150:region_quant=8'd162;
  8'd151:region_quant=8'd162;
  8'd152:region_quant=8'd162;
  8'd153:region_quant=8'd162;
  8'd154:region_quant=8'd162;
  8'd155:region_quant=8'd162;
  8'd156:region_quant=8'd162;
  8'd157:region_quant=8'd162;
  8'd158:region_quant=8'd162;
  8'd159:region_quant=8'd162;
  8'd160:region_quant=8'd162;
  8'd161:region_quant=8'd162;
  8'd162:region_quant=8'd162;
  8'd163:region_quant=8'd162;
  8'd164:region_quant=8'd162;
  8'd165:region_quant=8'd162;
  8'd166:region_quant=8'd185;
  8'd167:region_quant=8'd185;
  8'd168:region_quant=8'd185;
  8'd169:region_quant=8'd185;
  8'd170:region_quant=8'd185;
  8'd171:region_quant=8'd185;
  8'd172:region_quant=8'd185;
  8'd173:region_quant=8'd185;
  8'd174:region_quant=8'd185;
  8'd175:region_quant=8'd185;
  8'd176:region_quant=8'd185;
  8'd177:region_quant=8'd185;
  8'd178:region_quant=8'd185;
  8'd179:region_quant=8'd185;
  8'd180:region_quant=8'd185;
  8'd181:region_quant=8'd185;
  8'd182:region_quant=8'd185;
  8'd183:region_quant=8'd185;
  8'd184:region_quant=8'd185;
  8'd185:region_quant=8'd208;
  8'd186:region_quant=8'd208;
  8'd187:region_quant=8'd208;
  8'd188:region_quant=8'd208;
  8'd189:region_quant=8'd208;
  8'd190:region_quant=8'd208;
  8'd191:region_quant=8'd208;
  8'd192:region_quant=8'd208;
  8'd193:region_quant=8'd208;
  8'd194:region_quant=8'd208;
  8'd195:region_quant=8'd208;
  8'd196:region_quant=8'd208;
  8'd197:region_quant=8'd208;
  8'd198:region_quant=8'd208;
  8'd199:region_quant=8'd208;
  8'd200:region_quant=8'd208;
  8'd201:region_quant=8'd208;
  8'd202:region_quant=8'd208;
  8'd203:region_quant=8'd208;
  8'd204:region_quant=8'd208;
  8'd205:region_quant=8'd208;
  8'd206:region_quant=8'd231;
  8'd207:region_quant=8'd231;
  8'd208:region_quant=8'd231;
  8'd209:region_quant=8'd231;
  8'd210:region_quant=8'd231;
  8'd211:region_quant=8'd231;
  8'd212:region_quant=8'd231;
  8'd213:region_quant=8'd231;
  8'd214:region_quant=8'd231;
  8'd215:region_quant=8'd231;
  8'd216:region_quant=8'd231;
  8'd217:region_quant=8'd231;
  8'd218:region_quant=8'd231;
  8'd219:region_quant=8'd231;
  8'd220:region_quant=8'd231;
  8'd221:region_quant=8'd231;
  8'd222:region_quant=8'd231;
  8'd223:region_quant=8'd231;
  8'd224:region_quant=8'd231;
  8'd225:region_quant=8'd231;
  8'd226:region_quant=8'd231;
  8'd227:region_quant=8'd231;
  8'd228:region_quant=8'd231;
  8'd229:region_quant=8'd231;
  8'd230:region_quant=8'd231;
  8'd231:region_quant=8'd231;
  8'd232:region_quant=8'd231;
  8'd233:region_quant=8'd231;
  8'd234:region_quant=8'd255;
  8'd235:region_quant=8'd255;
  8'd236:region_quant=8'd255;
  8'd237:region_quant=8'd255;
  8'd238:region_quant=8'd255;
  8'd239:region_quant=8'd255;
  8'd240:region_quant=8'd255;
  8'd241:region_quant=8'd255;
  8'd242:region_quant=8'd255;
  8'd243:region_quant=8'd255;
  8'd244:region_quant=8'd255;
  8'd245:region_quant=8'd255;
  8'd246:region_quant=8'd255;
  8'd247:region_quant=8'd255;
  8'd248:region_quant=8'd255;
  8'd249:region_quant=8'd255;
  8'd250:region_quant=8'd255;
  8'd251:region_quant=8'd255;
  8'd252:region_quant=8'd255;
  8'd253:region_quant=8'd255;
  8'd254:region_quant=8'd255;
  8'd255:region_quant=8'd255;
 endcase end endfunction
 function [7:0] tone(input [7:0] y);
  integer p,t,v0,v1;
  begin
   p=y>>5;t=y&31;
   case(p)
    0:begin v0=0;v1=18;end 1:begin v0=18;v1=44;end
    2:begin v0=44;v1=78;end 3:begin v0=78;v1=120;end
    4:begin v0=120;v1=166;end 5:begin v0=166;v1=207;end
    6:begin v0=207;v1=239;end default:begin v0=239;v1=255;end
   endcase
   tone=(p==7)?(v0+(v1-v0)*t/31):(v0+((v1-v0)*t+16)/32);
  end
 endfunction
 // Style is evaluated once per palette entry, never in the 75 MHz input path.
 function [23:0] style(input [23:0] c);
  integer y,r,g,b,t,q,d;
  begin
   y=(77*c[23:16]+150*c[15:8]+29*c[7:0])>>8;
   r=clip(y+((($signed({1'b0,c[23:16]})-y)*410)>>>8));
   g=clip(y+((($signed({1'b0,c[15:8]})-y)*410)>>>8));
   b=clip(y+((($signed({1'b0,c[7:0]})-y)*410)>>>8));
   y=(77*r+150*g+29*b)>>8;t=tone(y);q=((t*11+127)/255)*255/11;d=q-y;
   style={clip(r+d),clip(g+d),clip(b+d)};
  end
 endfunction

 // Explicit synchronous ERAM ports prevent multi-read inferred tables from
 // becoming thousands of registers and large asynchronous selection trees.
 wire [23:0] mean_a,mean_b;
 wire [7:0] parent_a,parent_b;
 reg [7:0] idx,mean_idx,pal_idx;
 reg [1:0] iteration,merge_pass,channel;
 reg mean_after_merge;
 reg [15:0] pixel_index,pair_index;
 reg [8:0] px;reg [7:0] py;
 reg [3:0] candidate;
 reg [7:0] candidate_id,best_id;
 reg candidate_ok;
 reg [12:0] best_cost;
 reg [12:0] registered_cost;
 reg [23:0] pixel_rgb,mean_rgb;
 reg [95:0] stats,merge_a,merge_b;
 reg [7:0] root_a,root_b,winner,loser,walk_steps;
 reg pair_down;
 reg [15:0] label_ra;
 wire [7:0] label_rd;
 wire label_we=core_ce && state==ACC_WRITE && iteration==ITERATIONS-1;

 reg [7:0] accum_ra;
 wire [95:0] accum_rd;
 wire accum_we=core_ce && (state==CLEAR || state==ACC_WRITE || state==MERGE_WRITE);
 wire [7:0] accum_wa=state==CLEAR?idx:(state==MERGE_WRITE?winner:best_id);
 wire [95:0] accum_wd=state==CLEAR?96'd0:state==MERGE_WRITE?
  {8'd0,merge_a[87:64]+merge_b[87:64],merge_a[63:40]+merge_b[63:40],
         merge_a[39:16]+merge_b[39:16],merge_a[15:0]+merge_b[15:0]}:
  {8'd0,stats[87:64]+{16'd0,pixel_rgb[23:16]},stats[63:40]+{16'd0,pixel_rgb[15:8]},
         stats[39:16]+{16'd0,pixel_rgb[7:0]},stats[15:0]+16'd1};
 region_ram #(.WIDTH(96),.AW(9),.DEPTH(512)) sums(clk,accum_we,{1'b0,accum_wa},{1'b0,accum_ra},accum_wd,accum_rd);
 wire [23:0] palette_rd;
 reg [7:0] palette_ra;
 wire palette_we=core_ce && state==PAL_WRITE;
 reg [23:0] style_src,style_sat,style_result;
 reg [7:0] style_y,style_y2,style_t;
 reg signed [10:0] style_delta;
 wire [23:0] palette_wd=style_result;
 region_ram #(.WIDTH(24),.AW(9),.DEPTH(512)) palette(clk,palette_we,{1'b0,pal_idx},{1'b0,palette_ra},palette_wd,palette_rd);
 wire div_done;
 wire [23:0] quotient;
 wire [23:0] numerator=channel==0?stats[87:64]:channel==1?stats[63:40]:stats[39:16];
 region_divider divider(clk,rst_n,state==DIV_START && core_ce,numerator,stats[15:0],div_done,quotient);
 integer gx,gy,ci,cost;
 always @* begin
  gx=px/CW+(candidate%3)-1;gy=py/CH+(candidate/3)-1;
  ci=gy*GRID_X+gx;
  cost=color_distance(pixel_rgb,mean_a)+1*(
    absval(px-((candidate_id%GRID_X)*CW+CW/2))+
    absval(py-((candidate_id/GRID_X)*CH+CH/2)));
 end
 reg [10:0] render_x,render_y;
 reg neighbor_ok;
 reg abort_pending;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)abort_pending<=0;
  else if(state==IDLE)abort_pending<=0;
  else if(cancel)abort_pending<=1;
 wire input_need=state==SEED_WAIT || state==SEED_LOAD || state==PIX_WAIT || state==PIX_LOAD;
 wire label_need=state==PAIR_WAIT || state==PAIR_LOAD || state==ROOT_A || state==49 ||
                 (state==ACC_WRITE && iteration==ITERATIONS-1) || (state==70 && neighbor_ok);
 wire image_hit,label_hit;
 wire [127:0] image_line,label_line;
 reg [24:0] image_address;
 wire [24:0] image_native=input_need?(work_bank?25'd28100000:25'd28000000)+{input_ra[15:3],3'b0}:image_address;
 wire [24:0] label_native=25'd28200000+{(state==ACC_WRITE?pixel_index[15:4]:label_ra[15:4]),3'b0};
 wire [15:0] label_mask=16'h8000>>pixel_index[3:0];
 wire [127:0] label_write={16{best_id}};
 assign input_rd=image_line[127-input_ra[2:0]*16-:16];
 assign label_rd=label_line[127-label_ra[3:0]*8-:8];
 assign core_ce=(!input_need || image_hit) && (state!=66 || image_hit) && (!label_need || label_hit);
 wire ia,ir,la,lr,iq,lq,iw,lw;
 wire [24:0] iaddr,laddr;wire [127:0] idata,ldata,cache_rdata;
 region_ddr_cache image_cache(clk,rst_n,take_capture,input_need || state==66,1'b0,image_native,128'd0,16'd0,
   image_hit,image_line,iq,iw,iaddr,idata,ia,ir,cache_rdata);
 region_ddr_cache label_cache(clk,rst_n,take_capture,label_need,label_we && core_ce,label_native,label_write,label_mask,
   label_hit,label_line,lq,lw,laddr,ldata,la,lr,cache_rdata);
 region_cache_join join_caches(clk,rst_n,iq,iw,iaddr,idata,ia,ir,lq,lw,laddr,ldata,la,lr,
   mem_request,mem_wr,mem_address,mem_data,mem_accept,mem_response,mem_rdata,cache_rdata);
 wire mean_we=core_ce && (state==SEED_LOAD || state==MEAN_STORE);
 wire [7:0] mean_wa=state==SEED_LOAD?idx:mean_idx;
 wire [23:0] mean_wd=state==SEED_LOAD?expand565(input_rd):
  ((mean_after_merge || iteration==ITERATIONS-1)?mean_rgb:expand565({mean_rgb[23:19],mean_rgb[15:10],mean_rgb[7:3]}));
 wire [7:0] mean_addr_a=state==CAND_ADDR?ci[7:0]:state==57?candidate_id:
  (state==MEAN_ADDR || state==MEAN_WAIT || state==MEAN_LOAD)?mean_idx:root_a;
 region_ram #(.WIDTH(24),.AW(9),.DEPTH(512)) mean_port_a(clk,mean_we,{1'b0,mean_wa},{1'b0,mean_addr_a},mean_wd,mean_a);
 region_ram #(.WIDTH(24),.AW(9),.DEPTH(512)) mean_port_b(clk,mean_we,{1'b0,mean_wa},{1'b0,root_b},mean_wd,mean_b);
 wire parent_we=core_ce && ((state==CLEAR && iteration==0) || state==MERGE_WRITE);
 wire [7:0] parent_wa=state==CLEAR?idx:loser;
 wire [7:0] parent_wd=state==CLEAR?idx:winner;
 region_ram #(.WIDTH(8),.AW(9),.DEPTH(512)) parent_port_a(clk,parent_we,{1'b0,parent_wa},{1'b0,root_a},parent_wd,parent_a);
 region_ram #(.WIDTH(8),.AW(9),.DEPTH(512)) parent_port_b(clk,parent_we,{1'b0,parent_wa},{1'b0,root_b},parent_wd,parent_b);
 // Same-frame raw/styled colors have a registered read, consumed in state 73.
 wire [47:0] boundary_palette_rd;
 region_ram #(.WIDTH(48),.AW(9),.DEPTH(512)) boundary_colors(clk,palette_we,
  {1'b0,pal_idx},{1'b0,label_rd},{mean_a,style_result},boundary_palette_rd);
 reg [383:0] guide_words;reg [95:0] guide_pixels;
 reg [17:0] guide_word_index;reg [1:0] snapshot_part;
 reg [2:0] recon_candidate,recon_flush;
 integer neighbor_x,neighbor_y;
 always @*begin
  neighbor_x=render_x+(recon_candidate%3)-1;
  neighbor_y=(render_y>>2)+(render_y[1:0]<2?-1:0)+(recon_candidate/3);
  neighbor_ok=neighbor_x>=0 && neighbor_x<LOW_W && neighbor_y>=0 && neighbor_y<LOW_H;
 end
 reg [2:0] recon_valid;
 reg [23:0] recon_raw,recon_style0,recon_style1,recon_style2;
 reg signed [3:0] recon_dx,recon_dy;
 reg [8:0] diff_r[0:3],diff_g[0:3],diff_b[0:3];
 reg [7:0] spatial[0:3];reg [11:0] score[0:3];
 reg [11:0] boundary_best[0:3],boundary_second[0:3];
 reg [23:0] boundary_color[0:3],boundary_other[0:3];
 integer ri,rj;
 function [23:0] blend_boundary(input [23:0] a,b,input [11:0] best,second);
  integer c,weight;reg [23:0] v;
  begin
   weight=(second!=4095 && second-best<=3)?2:(second!=4095 && second-best<=8)?1:0;
   for(c=0;c<3;c=c+1)begin
    if(weight==2)v[c*8+:8]=({1'b0,a[c*8+:8]}+{1'b0,b[c*8+:8]}+1)>>1;
    else if(weight==1)v[c*8+:8]=(({2'b0,a[c*8+:8]}<<1)+{2'b0,a[c*8+:8]}+{2'b0,b[c*8+:8]}+2)>>2;
    else v[c*8+:8]=a[c*8+:8];
   end
   blend_boundary=v;
  end
 endfunction
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin recon_valid<=0;recon_raw<=0;recon_style0<=0;recon_style1<=0;recon_style2<=0;recon_dx<=0;recon_dy<=0;end
  else if(core_ce)begin
   recon_valid<={recon_valid[1:0],state==73 && neighbor_ok};
   if(state==73 && neighbor_ok)begin
    recon_raw<=boundary_palette_rd[47:24];recon_style0<=boundary_palette_rd[23:0];
    recon_dx<=neighbor_x-render_x;recon_dy<=neighbor_y-(render_y>>2);
   end
   recon_style1<=recon_style0;recon_style2<=recon_style1;
   for(rj=0;rj<4;rj=rj+1)begin
    diff_r[rj]<=absval($signed({1'b0,guide_pixels[95-rj*24-:8]})-$signed({1'b0,recon_raw[23:16]}));
    diff_g[rj]<=absval($signed({1'b0,guide_pixels[87-rj*24-:8]})-$signed({1'b0,recon_raw[15:8]}));
    diff_b[rj]<=absval($signed({1'b0,guide_pixels[79-rj*24-:8]})-$signed({1'b0,recon_raw[7:0]}));
    spatial[rj]<=4*(absval(rj-(4*recon_dx+2))+absval($signed({1'b0,render_y[1:0]})-(4*recon_dy+2)));
    score[rj]<=diff_r[rj]+diff_g[rj]+diff_b[rj]+spatial[rj];
   end
  end
 end

 reg [12:0] gap;
 reg [23:0] replay_color;
 assign ready=state==READY;
 assign rendering=replaying;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   state<=IDLE;replaying<=0;work_bank<=0;idx<=0;mean_idx<=0;pal_idx<=0;iteration<=0;merge_pass<=0;channel<=0;
   input_ra<=0;label_ra<=0;accum_ra<=0;palette_ra<=0;pixel_index<=0;pair_index<=0;
   px<=0;py<=0;candidate<=0;candidate_id<=0;candidate_ok<=0;best_id<=0;best_cost<=8191;
   pixel_rgb<=0;mean_rgb<=0;stats<=0;merge_a<=0;merge_b<=0;root_a<=0;root_b<=0;
   winner<=0;loser<=0;walk_steps<=0;pair_down<=0;mean_after_merge<=0;
   render_x<=0;render_y<=0;gap<=0;replay_color<=0;
   out_pixels<=0;out_valid<=0;out_frame<=0;out_good<=0;fault<=0;completed<=0;processing_cycles<=0;
   image_address<=0;guide_words<=0;guide_pixels<=0;guide_word_index<=0;snapshot_part<=0;recon_candidate<=0;recon_flush<=0;
   style_src<=0;style_sat<=0;style_result<=0;style_y<=0;style_y2<=0;style_t<=0;style_delta<=0;
  end else begin
   out_valid<=0;out_frame<=0;
   if(state!=IDLE && state!=READY && !rendering) processing_cycles<=processing_cycles+1'b1;
   if(core_ce)begin
   if(recon_valid[2])for(ri=0;ri<4;ri=ri+1)begin
    if(score[ri]<boundary_best[ri])begin
     if(recon_style2!=boundary_color[ri])begin boundary_second[ri]<=boundary_best[ri];boundary_other[ri]<=boundary_color[ri];end
     boundary_best[ri]<=score[ri];boundary_color[ri]<=recon_style2;
    end else if(score[ri]<boundary_second[ri] && recon_style2!=boundary_color[ri])begin
     boundary_second[ri]<=score[ri];boundary_other[ri]<=recon_style2;
    end
   end
   case(state)
    IDLE:if(take_capture) begin
     work_bank<=cap_bank;idx<=0;iteration<=0;processing_cycles<=0;state<=SEED_ADDR;
    end
    SEED_ADDR:begin input_ra<=((idx/GRID_X)*CH+CH/2)*LOW_W+(idx%GRID_X)*CW+CW/2;state<=SEED_WAIT;end
    SEED_WAIT:state<=SEED_LOAD;
    SEED_LOAD:begin
     if(idx==K-1) begin idx<=0;state<=CLEAR;end else begin idx<=idx+1'b1;state<=SEED_ADDR;end
    end
    CLEAR:begin
     if(idx==255) begin pixel_index<=0;px<=0;py<=0;state<=PIX_ADDR;end else idx<=idx+1'b1;
    end
    PIX_ADDR:begin input_ra<=pixel_index;state<=PIX_WAIT;end
    PIX_WAIT:state<=PIX_LOAD;
    PIX_LOAD:begin pixel_rgb<=expand565(input_rd);candidate<=0;best_cost<=8191;best_id<=0;state<=CAND_ADDR;end
    CAND_ADDR:begin candidate_id<=ci;candidate_ok<=gx>=0 && gx<GRID_X && gy>=0 && gy<GRID_Y;state<=57;end
    57:begin registered_cost<=cost;state<=CAND_EVAL;end
    CAND_EVAL:begin
     if(candidate_ok && registered_cost<best_cost) begin best_cost<=registered_cost;best_id<=candidate_id;end
     if(candidate==8) state<=ACC_ADDR;else begin candidate<=candidate+1'b1;state<=CAND_ADDR;end
    end
    ACC_ADDR:begin accum_ra<=best_id;state<=ACC_WAIT;end
    ACC_WAIT:state<=48;
    48:begin state<=ACC_WRITE;stats<=accum_rd;end
    ACC_WRITE:state<=NEXT_PIX;
    NEXT_PIX:begin
     if(pixel_index==N-1) begin mean_idx<=0;mean_after_merge<=0;state<=MEAN_ADDR;end
     else begin pixel_index<=pixel_index+1'b1;if(px==LOW_W-1)begin px<=0;py<=py+1'b1;end else px<=px+1'b1;state<=PIX_ADDR;end
    end
    MEAN_ADDR:begin accum_ra<=mean_idx;state<=MEAN_WAIT;end
    MEAN_WAIT:state<=MEAN_LOAD;
    MEAN_LOAD:begin stats<=accum_rd;mean_rgb<=mean_a;channel<=0;
     if(accum_rd[15:0]==0) state<=MEAN_STORE;else state<=DIV_START;
    end
    DIV_START:state<=DIV_WAIT;
    DIV_WAIT:if(div_done) begin
     case(channel)0:mean_rgb[23:16]<=quotient[7:0];1:mean_rgb[15:8]<=quotient[7:0];2:mean_rgb[7:0]<=quotient[7:0];endcase
     if(channel==2) state<=MEAN_STORE;else begin channel<=channel+1'b1;state<=DIV_START;end
    end
    MEAN_STORE:begin
     // Match the RGB565 precision of the integer reference during clustering.
     if(mean_after_merge) state<=NEXT_PAIR;
     else if(mean_idx==K-1) begin
      if(iteration==ITERATIONS-1) begin pair_index<=0;px<=0;py<=0;pair_down<=0;merge_pass<=0;state<=PAIR_ADDR;end
      else begin iteration<=iteration+1'b1;idx<=0;state<=CLEAR;end
     end else begin mean_idx<=mean_idx+1'b1;state<=MEAN_ADDR;end
    end
    PAIR_ADDR:begin
     if((!pair_down && px==LOW_W-1) || (pair_down && py==LOW_H-1)) state<=NEXT_PAIR;
     else begin label_ra<=pair_index;state<=PAIR_WAIT;end
    end
    PAIR_WAIT:state<=PAIR_LOAD;
    PAIR_LOAD:begin root_a<=label_rd;label_ra<=pair_index+(pair_down?LOW_W:1);walk_steps<=0;state<=58;end
    58:state<=ROOT_A;
    ROOT_A:begin
     if(walk_steps==K) begin fault<=1;state<=IDLE;end
     else if(parent_a!=root_a) begin root_a<=parent_a;walk_steps<=walk_steps+1'b1;state<=58;end
     else state<=49;
    end
    49:begin root_b<=label_rd;walk_steps<=0;state<=59;end
    59:state<=ROOT_B;
    ROOT_B:begin
     if(walk_steps==K) begin fault<=1;state<=IDLE;end
     else if(parent_b!=root_b) begin root_b<=parent_b;walk_steps<=walk_steps+1'b1;state<=59;end
     else state<=ROOT_COMPARE;
    end
    ROOT_COMPARE:if(root_a!=root_b && color_distance(mean_a,mean_b)<=MERGE_THRESHOLD) state<=MERGE_A_ADDR;else state<=NEXT_PAIR;
    MERGE_A_ADDR:begin accum_ra<=root_a;state<=MERGE_A_WAIT;end
    MERGE_A_WAIT:state<=MERGE_A_LOAD;
    MERGE_A_LOAD:begin merge_a<=accum_rd;accum_ra<=root_b;state<=MERGE_B_WAIT;end
    MERGE_B_WAIT:state<=MERGE_B_LOAD;
    MERGE_B_LOAD:begin merge_b<=accum_rd;
     winner<=merge_a[15:0]>=accum_rd[15:0]?root_a:root_b;
     loser<=merge_a[15:0]>=accum_rd[15:0]?root_b:root_a;
     state<=MERGE_WRITE;
    end
    MERGE_WRITE:begin mean_idx<=winner;mean_after_merge<=1;state<=MEAN_ADDR;end
    NEXT_PAIR:begin
     if(!pair_down) begin pair_down<=1;state<=PAIR_ADDR;end
     else begin
      pair_down<=0;
      if(pair_index==N-1) begin
       if(merge_pass==MERGE_PASSES-1) begin pal_idx<=0;root_a<=0;walk_steps<=0;state<=60;end
       else begin merge_pass<=merge_pass+1'b1;pair_index<=0;px<=0;py<=0;state<=PAIR_ADDR;end
      end else begin pair_index<=pair_index+1'b1;if(px==LOW_W-1)begin px<=0;py<=py+1'b1;end else px<=px+1'b1;state<=PAIR_ADDR;end
     end
    end
    60:state<=PAL_ROOT;
    PAL_ROOT:begin
     if(walk_steps==K) begin fault<=1;state<=IDLE;end
     else if(parent_a!=root_a) begin root_a<=parent_a;walk_steps<=walk_steps+1'b1;state<=60;end
     else state<=50;
    end
    50:begin style_src<=mean_a;state<=51;end
    51:begin style_y<=(77*style_src[23:16]+150*style_src[15:8]+29*style_src[7:0])>>8;state<=52;end
    52:begin
     style_sat<={clip($signed({1'b0,style_y})+(($signed({1'b0,style_src[23:16]})-$signed({1'b0,style_y}))*410>>>8)),
                 clip($signed({1'b0,style_y})+(($signed({1'b0,style_src[15:8]})-$signed({1'b0,style_y}))*410>>>8)),
                 clip($signed({1'b0,style_y})+(($signed({1'b0,style_src[7:0]})-$signed({1'b0,style_y}))*410>>>8))};
     state<=53;
    end
    53:begin style_y2<=(77*style_sat[23:16]+150*style_sat[15:8]+29*style_sat[7:0])>>8;state<=54;end
    54:begin style_t<=region_quant(style_y2);state<=55;end
    55:begin style_delta<=$signed({1'b0,style_t})-$signed({1'b0,style_y2});state<=56;end
    56:begin style_result<={clip($signed({1'b0,style_sat[23:16]})+style_delta),
                           clip($signed({1'b0,style_sat[15:8]})+style_delta),
                           clip($signed({1'b0,style_sat[7:0]})+style_delta)};state<=PAL_WRITE;end
    PAL_WRITE:begin
     if(pal_idx==K-1) begin state<=READY;completed<=completed+1'b1;end
     else begin pal_idx<=pal_idx+1'b1;root_a<=pal_idx+1'b1;walk_steps<=0;state<=60;end
    end
    READY:if(launch) begin render_x<=0;render_y<=0;replaying<=1;state<=OPEN_FRAME;end
    OPEN_FRAME:begin out_frame<=1;out_good<=0;gap<=0;state<=OPEN_GAP;end
    OPEN_GAP:if(gap==63)state<=64;else gap<=gap+1'b1;
    // Read the three RGB888 words containing sixteen original guide pixels.
    64:begin snapshot_part<=0;guide_word_index<=(render_y*(WIDTH/16)+(render_x>>2))*3;state<=65;end
    65:begin image_address<=(work_bank?25'd26000000:25'd24000000)+(guide_word_index+snapshot_part)*8;state<=66;end
    66:begin
     case(snapshot_part)0:guide_words[383:256]<=image_line;1:guide_words[255:128]<=image_line;2:guide_words[127:0]<=image_line;endcase
     if(snapshot_part==2)state<=68;else begin snapshot_part<=snapshot_part+1'b1;state<=65;end
    end
    68:begin
     case(render_x[1:0])0:guide_pixels<=guide_words[383:288];1:guide_pixels<=guide_words[287:192];2:guide_pixels<=guide_words[191:96];3:guide_pixels<=guide_words[95:0];endcase
     recon_candidate<=0;recon_flush<=0;
     for(ri=0;ri<4;ri=ri+1)begin boundary_best[ri]<=4095;boundary_second[ri]<=4095;boundary_color[ri]<=0;boundary_other[ri]<=0;end
     state<=69;
    end
    69:begin label_ra<=neighbor_y*LOW_W+neighbor_x;state<=70;end
    70:state<=73;
    73:begin
     if(recon_candidate==5)begin recon_flush<=0;state<=71;end
     else begin recon_candidate<=recon_candidate+1'b1;state<=69;end
    end
    71:if(recon_flush==4)state<=72;else recon_flush<=recon_flush+1'b1;
    72:if(output_ready)begin
     out_pixels<={blend_boundary(boundary_color[0],boundary_other[0],boundary_best[0],boundary_second[0]),
                  blend_boundary(boundary_color[1],boundary_other[1],boundary_best[1],boundary_second[1]),
                  blend_boundary(boundary_color[2],boundary_other[2],boundary_best[2],boundary_second[2]),
                  blend_boundary(boundary_color[3],boundary_other[3],boundary_best[3],boundary_second[3])};
     out_valid<=1;
     if(render_x==WIDTH/4-1)begin render_x<=0;render_y<=render_y+1'b1;end else render_x<=render_x+1'b1;
     if(render_x==WIDTH/4-1 && render_y==HEIGHT-1)begin gap<=0;state<=DRAIN;end
     else if(render_x==WIDTH/4-1 || render_x[1:0]==3)state<=64;
     else state<=68;
    end
    DRAIN:begin
     if(gap<127) gap<=gap+1'b1;
     else if(output_drained) state<=CLOSE_FRAME;
    end
    CLOSE_FRAME:begin out_frame<=1;out_good<=1;gap<=0;state<=CLOSE_GAP;end
    CLOSE_GAP:if(gap==63)begin replaying<=0;state<=IDLE;end else gap<=gap+1'b1;
    default:begin fault<=1;state<=IDLE;end
   endcase
   // Complete an in-progress replay before returning to another video mode.
   if(abort_pending && !rendering)state<=IDLE;
   end
  end
 end
 // synthesis translate_off
 initial begin
  if(WIDTH%16 || HEIGHT%4 || LOW_W!=WIDTH/4 || LOW_H!=HEIGHT/4 ||
     LOW_W%GRID_X || LOW_H%GRID_Y || K>255 || N>65536 || LOW_W>512 || LOW_H>256)
   $fatal(1,"unsupported region geometry");
 end
 // synthesis translate_on
endmodule
module cartoon_a_ram #(parameter WIDTH=16, AW=14, DEPTH=16384)(
 input clk, input we, input [AW-1:0] wa,ra,
 input [WIDTH-1:0] wd, output [WIDTH-1:0] rd);
 PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(WIDTH),.DATA_WIDTH_B(WIDTH),
 .ADDR_WIDTH_A(AW),.ADDR_WIDTH_B(AW),.DATA_DEPTH_A(DEPTH),.DATA_DEPTH_B(DEPTH),
 .MODE("PDPW"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
 .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
 .IMPLEMENT("20K(FAST)"),.ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
 .CLKMODE("SYNC"),.RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
 .INIT_FILE("NONE"),.FILL_ALL("NONE")) ram(
 .clka(clk),.clkb(clk),.cea(1'b1),.ceb(1'b1),.ocea(1'b0),.oceb(1'b0),
 .wea(we),.web(1'b0),.addra(wa),.addrb(ra),.dia(wd),.dib({WIDTH{1'b0}}),
 .bea(1'b0),.beb(1'b0),.rsta(1'b0),.rstb(1'b0),.doa(),.dob(rd),
 .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
endmodule

// Candidate A building blocks. Not registered in the production project.
// Each input is {original/filtered RGB565 auxiliary, working RGB565 value}.
module cartoon_a_window #(
 parameter W=320,H=180,TAPS=7,VERTICAL=0
)(input clk,rst_n,input [31:0] data,input valid,
 output reg [TAPS*32-1:0] window,output reg out_valid,output reg fault);
 localparam R=TAPS/2;
 generate if(!VERTICAL)begin: h
  reg [31:0] fifo[0:15];reg [3:0] wp,rp;reg [4:0] used;
  reg [31:0] shift[0:TAPS-2];reg [31:0] last;
  reg [9:0] x;reg [2:0] flush;
  wire pop=used!=0 && flush==0;
  wire step=pop || flush!=0;
  wire [31:0] value=flush!=0?last:fifo[rp];
  integer j;
  always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin wp<=0;rp<=0;used<=0;x<=0;flush<=0;last<=0;out_valid<=0;fault<=0;end
   else begin
    out_valid<=0;
    if(valid)begin
     if(used==16 && !pop)fault<=1;
     else begin fifo[wp]<=data;wp<=wp+1'b1;end
    end
    if(pop)rp<=rp+1'b1;
    case({valid,pop})2'b10:if(used!=16)used<=used+1'b1;2'b01:used<=used-1'b1;default:;endcase
    if(step)begin
     last<=value;
     for(j=0;j<TAPS-1;j=j+1)begin
      if(x==0)shift[j]<=value;else if(j==0)shift[j]<=value;else shift[j]<=shift[j-1];
     end
     window[31:0]<=value;
     for(j=1;j<TAPS;j=j+1)window[j*32+:32]<=x==0?value:shift[j-1];
     if(x>=R)out_valid<=1;
     if(flush!=0)begin
      flush<=flush-1'b1;
      if(flush==1)x<=0;else x<=x+1'b1;
     end else begin
      x<=x+1'b1;
      if(x==W-1)flush<=R;
     end
    end
   end
  end
 end else begin: v
  reg [9:0] x;reg [8:0] y;reg flushing;
  wire step=valid || flushing;
  reg qvalid,qflush;reg [31:0] qdata;reg [9:0] qx;reg [8:0] qy;
  wire [(TAPS-1)*32-1:0] history;
  wire [31:0] newest=qflush?history[31:0]:qdata;
  wire [(TAPS-1)*32-1:0] updated={history[(TAPS-2)*32-1:0],newest};
  cartoon_a_ram #(.WIDTH((TAPS-1)*32),.AW(9),.DEPTH(512)) rows(
   clk,qvalid,qx[8:0],x[8:0],updated,history);
  integer d;
  always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin x<=0;y<=0;flushing<=0;qvalid<=0;qflush<=0;qdata<=0;qx<=0;qy<=0;out_valid<=0;fault<=0;end
   else begin
    qvalid<=step;qflush<=flushing;qdata<=data;qx<=x;qy<=y;out_valid<=0;
    if(valid && flushing)fault<=1;
    if(step)begin
     if(x==W-1)begin
      x<=0;y<=y+1'b1;
      if(y==H-1)flushing<=1;
      if(y==H+R-1)begin y<=0;flushing<=0;end
     end else x<=x+1'b1;
    end
    if(qvalid)begin
     window[31:0]<=newest;
     for(d=1;d<TAPS;d=d+1)begin
      if(qy==0)window[d*32+:32]<=newest;
      else if(qy<d)window[d*32+:32]<=history[(qy-1)*32+:32];
      else window[d*32+:32]<=history[(d-1)*32+:32];
     end
     out_valid<=qy>=R;
    end
   end
  end
 end endgenerate
endmodule

module cartoon_a_average #(parameter THRESHOLD=64)(
 input clk,rst_n,input [223:0] window,input valid,
 output reg [31:0] data,output reg out_valid);
 function [23:0] rgb(input [15:0] c);
  begin rgb={{c[15:11],c[15:13]},{c[10:5],c[10:9]},{c[4:0],c[4:2]}};end
 endfunction
 function [8:0] distance(input [7:0] a,b);begin distance=a>b?a-b:b-a;end endfunction
 reg [6:0] keep;reg [23:0] colors[0:6];reg [15:0] aux0,aux1,aux2;
 reg v0,v1,v2;reg [10:0] sr,sg,sb;reg [2:0] count;
 reg [26:0] pr,pg,pb;
 wire [23:0] center=rgb(window[111:96]);
 reg [10:0] sumr,sumg,sumb;reg [2:0] count_sum;
 reg [15:0] reciprocal;
 integer j;
 always @*begin
  sumr=0;sumg=0;sumb=0;count_sum=0;
  for(integer k=0;k<7;k=k+1)if(keep[k])begin
   sumr=sumr+colors[k][23:16];sumg=sumg+colors[k][15:8];sumb=sumb+colors[k][7:0];count_sum=count_sum+1'b1;
  end
  case(count)3:reciprocal=21846;5:reciprocal=13108;6:reciprocal=10923;7:reciprocal=9363;default:reciprocal=0;endcase
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin v0<=0;v1<=0;v2<=0;out_valid<=0;data<=0;end
  else begin
   v0<=valid;v1<=v0;v2<=v1;out_valid<=v2;
   aux0<=window[127:112];aux1<=aux0;aux2<=aux1;
   for(j=0;j<7;j=j+1)begin
    colors[j]<=rgb(window[j*32+:16]);
    keep[j]<=distance(rgb(window[j*32+:16])>>16,center[23:16])+
      distance(rgb(window[j*32+:16])>>8,center[15:8])+
      distance(rgb(window[j*32+:16]),center[7:0])<=THRESHOLD;
   end
   sr<=sumr;sg<=sumg;sb<=sumb;count<=count_sum;
   case(count)
    1:begin pr<={sr,16'd0};pg<={sg,16'd0};pb<={sb,16'd0};end
    2:begin pr<={1'b0,sr,15'd0};pg<={1'b0,sg,15'd0};pb<={1'b0,sb,15'd0};end
    4:begin pr<={2'b0,sr,14'd0};pg<={2'b0,sg,14'd0};pb<={2'b0,sb,14'd0};end
    default:begin pr<=sr*reciprocal;pg<=sg*reciprocal;pb<=sb*reciprocal;end
   endcase
   data<={aux2,pr[23:19],pg[23:18],pb[23:19]};
  end
 end
endmodule

module cartoon_a_mode(input clk,rst_n,input [159:0] window,input valid,
 output reg [31:0] data,output reg out_valid);
 reg [2:0] scores[0:4];reg [15:0] codes[0:4];reg [15:0] aux;
 reg v0;reg [15:0] winner;reg [2:0] best;
 integer i,j;
 always @*begin
  best=scores[2];winner=codes[2];
  if(scores[3]>best)begin best=scores[3];winner=codes[3];end
  if(scores[1]>best)begin best=scores[1];winner=codes[1];end
  if(scores[4]>best)begin best=scores[4];winner=codes[4];end
  if(scores[0]>best)begin best=scores[0];winner=codes[0];end
 end
 reg [2:0] vote;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin out_valid<=0;v0<=0;data<=0;end
  else begin
   v0<=valid;out_valid<=v0;aux<=window[95:80];
   for(i=0;i<5;i=i+1)begin
    codes[i]<=window[i*32+:16];vote=0;
    for(j=0;j<5;j=j+1)if(window[i*32+:16]==window[j*32+:16])vote=vote+1'b1;
    scores[i]<=vote;
   end
   data<={aux,winner};
  end
 end
endmodule

module cartoon_a_style(input clk,rst_n,input [31:0] data,input valid,
 output reg [31:0] out_data,output out_valid);
 function [23:0] expand565(input [15:0] c);
  begin expand565={{c[15:11],c[15:13]},{c[10:5],c[10:9]},{c[4:0],c[4:2]}};end
 endfunction
 function integer absval(input integer a);begin absval=a<0?-a:a;end endfunction
 function integer color_distance(input [23:0] a,b);
  begin color_distance=absval({1'b0,a[23:16]}-{1'b0,b[23:16]})+
    absval({1'b0,a[15:8]}-{1'b0,b[15:8]})+absval({1'b0,a[7:0]}-{1'b0,b[7:0]});end
 endfunction
 function [7:0] clip(input integer a);begin clip=a<0?0:(a>255?255:a);end endfunction
 function [7:0] region_quant(input [7:0] y);
 begin case(y)
  8'd0:region_quant=8'd0;
  8'd1:region_quant=8'd0;
  8'd2:region_quant=8'd0;
  8'd3:region_quant=8'd0;
  8'd4:region_quant=8'd0;
  8'd5:region_quant=8'd0;
  8'd6:region_quant=8'd0;
  8'd7:region_quant=8'd0;
  8'd8:region_quant=8'd0;
  8'd9:region_quant=8'd0;
  8'd10:region_quant=8'd0;
  8'd11:region_quant=8'd0;
  8'd12:region_quant=8'd0;
  8'd13:region_quant=8'd0;
  8'd14:region_quant=8'd0;
  8'd15:region_quant=8'd0;
  8'd16:region_quant=8'd0;
  8'd17:region_quant=8'd0;
  8'd18:region_quant=8'd0;
  8'd19:region_quant=8'd0;
  8'd20:region_quant=8'd0;
  8'd21:region_quant=8'd23;
  8'd22:region_quant=8'd23;
  8'd23:region_quant=8'd23;
  8'd24:region_quant=8'd23;
  8'd25:region_quant=8'd23;
  8'd26:region_quant=8'd23;
  8'd27:region_quant=8'd23;
  8'd28:region_quant=8'd23;
  8'd29:region_quant=8'd23;
  8'd30:region_quant=8'd23;
  8'd31:region_quant=8'd23;
  8'd32:region_quant=8'd23;
  8'd33:region_quant=8'd23;
  8'd34:region_quant=8'd23;
  8'd35:region_quant=8'd23;
  8'd36:region_quant=8'd23;
  8'd37:region_quant=8'd23;
  8'd38:region_quant=8'd23;
  8'd39:region_quant=8'd23;
  8'd40:region_quant=8'd23;
  8'd41:region_quant=8'd23;
  8'd42:region_quant=8'd23;
  8'd43:region_quant=8'd23;
  8'd44:region_quant=8'd23;
  8'd45:region_quant=8'd23;
  8'd46:region_quant=8'd23;
  8'd47:region_quant=8'd23;
  8'd48:region_quant=8'd23;
  8'd49:region_quant=8'd23;
  8'd50:region_quant=8'd23;
  8'd51:region_quant=8'd23;
  8'd52:region_quant=8'd23;
  8'd53:region_quant=8'd46;
  8'd54:region_quant=8'd46;
  8'd55:region_quant=8'd46;
  8'd56:region_quant=8'd46;
  8'd57:region_quant=8'd46;
  8'd58:region_quant=8'd46;
  8'd59:region_quant=8'd46;
  8'd60:region_quant=8'd46;
  8'd61:region_quant=8'd46;
  8'd62:region_quant=8'd46;
  8'd63:region_quant=8'd46;
  8'd64:region_quant=8'd46;
  8'd65:region_quant=8'd46;
  8'd66:region_quant=8'd46;
  8'd67:region_quant=8'd46;
  8'd68:region_quant=8'd46;
  8'd69:region_quant=8'd46;
  8'd70:region_quant=8'd46;
  8'd71:region_quant=8'd46;
  8'd72:region_quant=8'd46;
  8'd73:region_quant=8'd46;
  8'd74:region_quant=8'd46;
  8'd75:region_quant=8'd46;
  8'd76:region_quant=8'd46;
  8'd77:region_quant=8'd69;
  8'd78:region_quant=8'd69;
  8'd79:region_quant=8'd69;
  8'd80:region_quant=8'd69;
  8'd81:region_quant=8'd69;
  8'd82:region_quant=8'd69;
  8'd83:region_quant=8'd69;
  8'd84:region_quant=8'd69;
  8'd85:region_quant=8'd69;
  8'd86:region_quant=8'd69;
  8'd87:region_quant=8'd69;
  8'd88:region_quant=8'd69;
  8'd89:region_quant=8'd69;
  8'd90:region_quant=8'd69;
  8'd91:region_quant=8'd69;
  8'd92:region_quant=8'd69;
  8'd93:region_quant=8'd69;
  8'd94:region_quant=8'd69;
  8'd95:region_quant=8'd69;
  8'd96:region_quant=8'd69;
  8'd97:region_quant=8'd69;
  8'd98:region_quant=8'd69;
  8'd99:region_quant=8'd92;
  8'd100:region_quant=8'd92;
  8'd101:region_quant=8'd92;
  8'd102:region_quant=8'd92;
  8'd103:region_quant=8'd92;
  8'd104:region_quant=8'd92;
  8'd105:region_quant=8'd92;
  8'd106:region_quant=8'd92;
  8'd107:region_quant=8'd92;
  8'd108:region_quant=8'd92;
  8'd109:region_quant=8'd92;
  8'd110:region_quant=8'd92;
  8'd111:region_quant=8'd92;
  8'd112:region_quant=8'd92;
  8'd113:region_quant=8'd92;
  8'd114:region_quant=8'd92;
  8'd115:region_quant=8'd92;
  8'd116:region_quant=8'd92;
  8'd117:region_quant=8'd115;
  8'd118:region_quant=8'd115;
  8'd119:region_quant=8'd115;
  8'd120:region_quant=8'd115;
  8'd121:region_quant=8'd115;
  8'd122:region_quant=8'd115;
  8'd123:region_quant=8'd115;
  8'd124:region_quant=8'd115;
  8'd125:region_quant=8'd115;
  8'd126:region_quant=8'd115;
  8'd127:region_quant=8'd115;
  8'd128:region_quant=8'd115;
  8'd129:region_quant=8'd115;
  8'd130:region_quant=8'd115;
  8'd131:region_quant=8'd115;
  8'd132:region_quant=8'd115;
  8'd133:region_quant=8'd115;
  8'd134:region_quant=8'd139;
  8'd135:region_quant=8'd139;
  8'd136:region_quant=8'd139;
  8'd137:region_quant=8'd139;
  8'd138:region_quant=8'd139;
  8'd139:region_quant=8'd139;
  8'd140:region_quant=8'd139;
  8'd141:region_quant=8'd139;
  8'd142:region_quant=8'd139;
  8'd143:region_quant=8'd139;
  8'd144:region_quant=8'd139;
  8'd145:region_quant=8'd139;
  8'd146:region_quant=8'd139;
  8'd147:region_quant=8'd139;
  8'd148:region_quant=8'd139;
  8'd149:region_quant=8'd139;
  8'd150:region_quant=8'd162;
  8'd151:region_quant=8'd162;
  8'd152:region_quant=8'd162;
  8'd153:region_quant=8'd162;
  8'd154:region_quant=8'd162;
  8'd155:region_quant=8'd162;
  8'd156:region_quant=8'd162;
  8'd157:region_quant=8'd162;
  8'd158:region_quant=8'd162;
  8'd159:region_quant=8'd162;
  8'd160:region_quant=8'd162;
  8'd161:region_quant=8'd162;
  8'd162:region_quant=8'd162;
  8'd163:region_quant=8'd162;
  8'd164:region_quant=8'd162;
  8'd165:region_quant=8'd162;
  8'd166:region_quant=8'd185;
  8'd167:region_quant=8'd185;
  8'd168:region_quant=8'd185;
  8'd169:region_quant=8'd185;
  8'd170:region_quant=8'd185;
  8'd171:region_quant=8'd185;
  8'd172:region_quant=8'd185;
  8'd173:region_quant=8'd185;
  8'd174:region_quant=8'd185;
  8'd175:region_quant=8'd185;
  8'd176:region_quant=8'd185;
  8'd177:region_quant=8'd185;
  8'd178:region_quant=8'd185;
  8'd179:region_quant=8'd185;
  8'd180:region_quant=8'd185;
  8'd181:region_quant=8'd185;
  8'd182:region_quant=8'd185;
  8'd183:region_quant=8'd185;
  8'd184:region_quant=8'd185;
  8'd185:region_quant=8'd208;
  8'd186:region_quant=8'd208;
  8'd187:region_quant=8'd208;
  8'd188:region_quant=8'd208;
  8'd189:region_quant=8'd208;
  8'd190:region_quant=8'd208;
  8'd191:region_quant=8'd208;
  8'd192:region_quant=8'd208;
  8'd193:region_quant=8'd208;
  8'd194:region_quant=8'd208;
  8'd195:region_quant=8'd208;
  8'd196:region_quant=8'd208;
  8'd197:region_quant=8'd208;
  8'd198:region_quant=8'd208;
  8'd199:region_quant=8'd208;
  8'd200:region_quant=8'd208;
  8'd201:region_quant=8'd208;
  8'd202:region_quant=8'd208;
  8'd203:region_quant=8'd208;
  8'd204:region_quant=8'd208;
  8'd205:region_quant=8'd208;
  8'd206:region_quant=8'd231;
  8'd207:region_quant=8'd231;
  8'd208:region_quant=8'd231;
  8'd209:region_quant=8'd231;
  8'd210:region_quant=8'd231;
  8'd211:region_quant=8'd231;
  8'd212:region_quant=8'd231;
  8'd213:region_quant=8'd231;
  8'd214:region_quant=8'd231;
  8'd215:region_quant=8'd231;
  8'd216:region_quant=8'd231;
  8'd217:region_quant=8'd231;
  8'd218:region_quant=8'd231;
  8'd219:region_quant=8'd231;
  8'd220:region_quant=8'd231;
  8'd221:region_quant=8'd231;
  8'd222:region_quant=8'd231;
  8'd223:region_quant=8'd231;
  8'd224:region_quant=8'd231;
  8'd225:region_quant=8'd231;
  8'd226:region_quant=8'd231;
  8'd227:region_quant=8'd231;
  8'd228:region_quant=8'd231;
  8'd229:region_quant=8'd231;
  8'd230:region_quant=8'd231;
  8'd231:region_quant=8'd231;
  8'd232:region_quant=8'd231;
  8'd233:region_quant=8'd231;
  8'd234:region_quant=8'd255;
  8'd235:region_quant=8'd255;
  8'd236:region_quant=8'd255;
  8'd237:region_quant=8'd255;
  8'd238:region_quant=8'd255;
  8'd239:region_quant=8'd255;
  8'd240:region_quant=8'd255;
  8'd241:region_quant=8'd255;
  8'd242:region_quant=8'd255;
  8'd243:region_quant=8'd255;
  8'd244:region_quant=8'd255;
  8'd245:region_quant=8'd255;
  8'd246:region_quant=8'd255;
  8'd247:region_quant=8'd255;
  8'd248:region_quant=8'd255;
  8'd249:region_quant=8'd255;
  8'd250:region_quant=8'd255;
  8'd251:region_quant=8'd255;
  8'd252:region_quant=8'd255;
  8'd253:region_quant=8'd255;
  8'd254:region_quant=8'd255;
  8'd255:region_quant=8'd255;
 endcase end endfunction
 function [7:0] tone(input [7:0] y);
  integer p,t,v0,v1;
  begin
   p=y>>5;t=y&31;
   case(p)
    0:begin v0=0;v1=18;end 1:begin v0=18;v1=44;end
    2:begin v0=44;v1=78;end 3:begin v0=78;v1=120;end
    4:begin v0=120;v1=166;end 5:begin v0=166;v1=207;end
    6:begin v0=207;v1=239;end default:begin v0=239;v1=255;end
   endcase
   tone=(p==7)?(v0+(v1-v0)*t/31):(v0+((v1-v0)*t+16)/32);
  end
 endfunction
 // Style is evaluated once per palette entry, never in the 75 MHz input path.
 function [23:0] style(input [23:0] c);
  integer y,r,g,b,t,q,d;
  begin
   y=(77*c[23:16]+150*c[15:8]+29*c[7:0])>>8;
   r=clip(y+((($signed({1'b0,c[23:16]})-y)*410)>>>8));
   g=clip(y+((($signed({1'b0,c[15:8]})-y)*410)>>>8));
   b=clip(y+((($signed({1'b0,c[7:0]})-y)*410)>>>8));
   y=(77*r+150*g+29*b)>>8;t=tone(y);q=((t*11+127)/255)*255/11;d=q-y;
   style={clip(r+d),clip(g+d),clip(b+d)};
  end
 endfunction


 reg [7:0] vp;
 reg [15:0] raw[0:7];
 reg [23:0] source,source1,sat0,sat1,sat2,sat3,result;
 reg [7:0] y0,y1,y2,q;reg signed [10:0] delta;
 wire [23:0] base=expand565(data[31:16]);
 wire [23:0] filtered=expand565(data[15:0]);
 wire [23:0] restored=color_distance(base,filtered)>32?base:filtered;
 assign out_valid=vp[7];
 integer i;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin vp<=0;out_data<=0;end
  else begin
   vp<={vp[6:0],valid};
   raw[0]<={restored[23:19],restored[15:10],restored[7:3]};
   for(i=1;i<8;i=i+1)raw[i]<=raw[i-1];
   source<=restored;
   y0<=(77*source[23:16]+150*source[15:8]+29*source[7:0])>>8;
   source1<=source;
   sat0<={clip($signed({1'b0,y0})+(($signed({1'b0,source1[23:16]})-$signed({1'b0,y0}))*410>>>8)),
          clip($signed({1'b0,y0})+(($signed({1'b0,source1[15:8]})-$signed({1'b0,y0}))*410>>>8)),
          clip($signed({1'b0,y0})+(($signed({1'b0,source1[7:0]})-$signed({1'b0,y0}))*410>>>8))};
   y1<=(77*sat0[23:16]+150*sat0[15:8]+29*sat0[7:0])>>8;sat1<=sat0;
   q<=region_quant(y1);y2<=y1;sat2<=sat1;
   delta<=$signed({1'b0,q})-$signed({1'b0,y2});sat3<=sat2;
   result<={clip($signed({1'b0,sat3[23:16]})+delta),
            clip($signed({1'b0,sat3[15:8]})+delta),
            clip($signed({1'b0,sat3[7:0]})+delta)};
   out_data<={raw[6],result[23:19],result[15:10],result[7:3]};
  end
 end
endmodule

module cartoon_a_low #(parameter W=320,H=180)(
 input clk,rst_n,input [15:0] pixel,input valid,
 output [31:0] data,output out_valid,output fault);
 wire [31:0] d[0:6];wire [6:0] v,f;
 assign d[0]={pixel,pixel};assign v[0]=valid;
 genvar k;
 generate for(k=0;k<4;k=k+1)begin: smoothing
  wire [223:0] win;wire wv;
  cartoon_a_window #(.W(W),.H(H),.TAPS(7),.VERTICAL(k%2)) window(
   clk,rst_n,d[k],v[k],win,wv,f[k]);
  cartoon_a_average average(clk,rst_n,win,wv,d[k+1],v[k+1]);
 end endgenerate
 cartoon_a_style style(clk,rst_n,d[4],v[4],d[5],v[5]);
 wire [159:0] hw,vw;wire hv,vv;
 cartoon_a_window #(.W(W),.H(H),.TAPS(5),.VERTICAL(0)) mode_h_window(
  clk,rst_n,d[5],v[5],hw,hv,f[4]);
 cartoon_a_mode mode_h(clk,rst_n,hw,hv,d[6],v[6]);
 cartoon_a_window #(.W(W),.H(H),.TAPS(5),.VERTICAL(1)) mode_v_window(
  clk,rst_n,d[6],v[6],vw,vv,f[5]);
 cartoon_a_mode mode_v(clk,rst_n,vw,vv,data,out_valid);
 assign fault=|f[5:0];
endmodule
// Candidate A reconstruction: exact six-candidate ordering, two candidates/cycle.
// Low rows use a bounded twelve-row ring; RGB565 guide uses a 48-row ring.
module cartoon_a_render #(parameter W=320,H=180)(
 input clk,rst_n,input enable,input [31:0] low,input low_valid,
 input [63:0] guide,input guide_valid,input [8:0] guide_x,input [9:0] guide_y,
 input output_ready,output_drained,input frame_good,quality_toggle,
 output reg [95:0] pixels,output reg valid,frame,good,output reg rendering,
 output reg fault,output reg [31:0] completed);
 localparam WAIT=0,OPEN=1,GAP=2,INIT0=3,INIT1=4,INIT2=5,
  SCORE0=6,SCORE1=7,SCORE2=8,LOAD=9,WAIT_ROW=10,DRAIN=11,CLOSE=12,WAIT_NEXT=13;
 reg [3:0] state;
 reg [8:0] lx,ly,rows;
 reg [3:0] lb;
 reg low_gen,render_gen,guide_gen;
 reg [8:0] x;reg [9:0] y;
 reg [3:0] row_bank;
 reg [7:0] gap;
 reg active;
 reg [31:0] cycles;
 wire [8:0] available_rows=low_gen!=render_gen?H:rows;
 wire [8:0] required_rows=(y>>2)+2>H?H:(y>>2)+2;
 wire can_row=available_rows>=required_rows;
 wire end_row=x==W-1;
 wire end_image=end_row && y==H*4-1;
 wire advance=state==SCORE2 && !end_image && !end_row && output_ready;
 wire [8:0] next_x=x+1'b1;
 wire [8:0] read_x=state==INIT1?9'd1:state==INIT0?9'd0:
  state==SCORE2?(end_row?9'd0:(next_x+1>=W?W-1:next_x+1)):
  x+1>=W?W-1:x+1;
 wire [31:0] row_data[0:11];
 genvar r;
 generate for(r=0;r<12;r=r+1)begin: storage
  cartoon_a_ram #(.WIDTH(32),.AW(9),.DEPTH(512)) row(
   clk,low_valid && lb==r,lx,read_x,low,row_data[r]);
 end endgenerate
 wire [8:0] cy=y>>2;
 wire [3:0] rt=cy==0?0:row_bank==0?11:row_bank-1'b1;
 wire [3:0] rm=row_bank;
 wire [3:0] rb=cy==H-1?row_bank:row_bank==11?0:row_bank+1'b1;
 wire [95:0] column={row_data[rt],row_data[rm],row_data[rb]};
 reg [95:0] left,center,right;
 reg [63:0] guide_hold;
 reg [5:0] guide_ring;
 reg [5:0] read_ring;
 wire [13:0] guide_wa=guide_ring*W+guide_x;
 wire [13:0] guide_ra=read_ring*W+(state==SCORE2 && advance?next_x:x);
 wire [63:0] guide_rd;
 cartoon_a_guide_ram #(.WIDTH(64),.AW(14),.DEPTH(48*W)) guides(
  clk,guide_valid,guide_wa,guide_ra,guide,guide_rd);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin lx<=0;ly<=0;lb<=0;rows<=0;low_gen<=0;guide_ring<=0;guide_gen<=0;end
  else begin
   if(guide_valid && guide_x==W-1)begin
    if(guide_y==H*4-1 || guide_ring==47)guide_ring<=0;else guide_ring<=guide_ring+1'b1;
    if(guide_y==H*4-1)guide_gen<=~guide_gen;
   end
   if(low_valid)begin
    if(lx==W-1)begin
     lx<=0;ly<=ly+1'b1;rows<=ly+1'b1;
     lb<=lb==11?0:lb+1'b1;
     if(ly==H-1)begin ly<=0;lb<=0;rows<=0;low_gen<=~low_gen;end
    end else lx<=lx+1'b1;
   end
  end
 end
 function [23:0] rgb(input [15:0] c);
  begin rgb={{c[15:11],c[15:13]},{c[10:5],c[10:9]},{c[4:0],c[4:2]}};end
 endfunction
 function integer av(input integer a);begin av=a<0?-a:a;end endfunction
 function [8:0] cd(input [7:0] a,b);begin cd=a>b?a-b:b-a;end endfunction
 function [71:0] insert_color(input [71:0] prev,input [11:0] score,input [23:0] color,input ok);
  reg [11:0] best,second;reg [23:0] first,other;
  begin
   {best,second,first,other}=prev;
   if(ok)begin
    if(score<best)begin
     if(color!=first)begin second=best;other=first;end
     best=score;first=color;
    end else if(score<second && color!=first)begin second=score;other=color;end
   end
   insert_color={best,second,first,other};
  end
 endfunction
 function [23:0] blend(input [71:0] result);
  reg [11:0] best,second;reg [23:0] a,b,c;integer k;
  begin
   {best,second,a,b}=result;c=a;
   for(k=0;k<3;k=k+1)begin
    if(second!=4095 && second-best<=3)c[k*8+:8]=({1'b0,a[k*8+:8]}+{1'b0,b[k*8+:8]}+1)>>1;
    else if(second!=4095 && second-best<=8)c[k*8+:8]=(({2'b0,a[k*8+:8]}<<1)+{2'b0,a[k*8+:8]}+{2'b0,b[k*8+:8]}+2)>>2;
   end
   blend=c;
  end
 endfunction
 reg sv;reg [1:0] tag;reg last_score;
 reg [11:0] scores[0:1][0:3];reg [23:0] styles[0:1];reg [1:0] oks;
 reg [71:0] results[0:3];reg [71:0] folded[0:3];
 reg emit,emit_last;
 integer k,c,idx,dx,dy,ny,nx;
 reg [95:0] selected_column;reg [31:0] selected;
 reg [23:0] raw_color,guide_color;
 reg [3:0] spatial_x,spatial_y;
 reg [71:0] temporary;
 always @*begin
  for(integer n=0;n<4;n=n+1)begin
   temporary=tag==0?{12'd4095,12'd4095,48'd0}:results[n];
   temporary=insert_color(temporary,scores[0][n],styles[0],oks[0]);
   folded[n]=insert_color(temporary,scores[1][n],styles[1],oks[1]);
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin sv<=0;tag<=0;emit<=0;emit_last<=0;last_score<=0;valid<=0;pixels<=0;end
  else begin
   sv<=state==SCORE0 || state==SCORE1 || state==SCORE2;
   if(state==SCORE0)tag<=0;else if(state==SCORE1)tag<=1;else if(state==SCORE2)tag<=2;
   last_score<=state==SCORE2 && end_image;
   if(state==SCORE0 || state==SCORE1 || state==SCORE2)begin
    for(c=0;c<2;c=c+1)begin
     idx=(state==SCORE0?0:state==SCORE1?2:4)+c;
     selected_column=idx==0 || idx==3?left:idx==1 || idx==4?center:right;
     if(y[1:0]<2)selected=idx<3?selected_column[95:64]:selected_column[63:32];
     else selected=idx<3?selected_column[63:32]:selected_column[31:0];
     raw_color=rgb(selected[31:16]);styles[c]<=rgb(selected[15:0]);
     oks[c]<=!(x==0 && (idx==0 || idx==3)) && !(x==W-1 && (idx==2 || idx==5)) &&
      !(cy==0 && y[1:0]<2 && idx<3) && !(cy==H-1 && y[1:0]>=2 && idx>=3);
     case(y[1:0])0:spatial_y=2;1:spatial_y=idx<3?3:1;2:spatial_y=idx<3?0:4;3:spatial_y=idx<3?1:3;endcase
     for(k=0;k<4;k=k+1)begin
      guide_color=rgb(guide_hold[63-k*16-:16]);
      if(idx==0 || idx==3)spatial_x=k+2;
      else if(idx==1 || idx==4)spatial_x=k<2?2-k:k-2;
      else spatial_x=6-k;
      scores[c][k]<=cd(guide_color[23:16],raw_color[23:16])+cd(guide_color[15:8],raw_color[15:8])+
       cd(guide_color[7:0],raw_color[7:0])+({1'b0,spatial_x}+{1'b0,spatial_y})*6'd4;
     end
    end
   end
   emit<=sv && tag==2;
   if(sv)begin for(k=0;k<4;k=k+1)results[k]<=folded[k];end
   if(sv && tag==2)emit_last<=last_score;
   valid<=emit && active;
   if(emit)pixels<={blend(results[0]),blend(results[1]),blend(results[2]),blend(results[3])};
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=WAIT;x<=0;y<=0;read_ring<=0;row_bank<=0;frame<=0;good<=0;rendering<=0;active<=0;fault<=0;completed<=0;render_gen<=0;gap<=0;cycles<=0;end
  else begin
   frame<=0;
   if(state!=WAIT)cycles<=cycles+1'b1;
   if(low_valid && state!=WAIT && (low_gen==render_gen?ly:H+ly)>=(cy==0?12:cy+11))fault<=1;
   if(guide_valid && state!=WAIT &&
      (guide_gen==render_gen?$signed({1'b0,guide_y})-$signed({1'b0,y}):H*4+$signed({1'b0,guide_y})-$signed({1'b0,y}))>47)fault<=1;
   case(state)
    WAIT:if(available_rows>=2)begin x<=0;y<=0;read_ring<=0;row_bank<=0;active<=enable;rendering<=enable;state<=OPEN;cycles<=0;end
    OPEN:begin if(active)begin frame<=1;good<=0;end gap<=0;state<=GAP;end
    GAP:if(gap==63)state<=INIT0;else gap<=gap+1'b1;
    INIT0:if(output_ready || !active)state<=INIT1;
    INIT1:begin left<=column;center<=column;guide_hold<=guide_rd;state<=INIT2;end
    INIT2:begin right<=column;state<=SCORE0;end
    SCORE0:state<=SCORE1;
    SCORE1:state<=SCORE2;
    SCORE2:begin
     if(end_image)begin gap<=0;state<=DRAIN;end
     else if(end_row)begin
      x<=0;y<=y+1'b1;read_ring<=read_ring==47?0:read_ring+1'b1;state<=WAIT_ROW;
      if(y[1:0]==3)row_bank<=row_bank==11?0:row_bank+1'b1;
     end else begin
      x<=x+1'b1;left<=center;center<=right;
      if(output_ready || !active)state<=LOAD;else state<=WAIT_NEXT;
     end
    end
    LOAD:begin right<=column;guide_hold<=guide_rd;state<=SCORE0;end
    WAIT_NEXT:if(output_ready || !active)state<=LOAD;
    WAIT_ROW:if(can_row)state<=INIT0;
    DRAIN:begin
     if(gap!=127)gap<=gap+1'b1;
     else if(output_drained || !active)state<=CLOSE;
    end
    CLOSE:if(quality_toggle!=render_gen)begin
     if(active)begin frame<=1;good<=frame_good;end
     completed<=completed+1'b1;render_gen<=~render_gen;rendering<=0;state<=WAIT;
    end
    default:begin state<=WAIT;fault<=1;end
   endcase
  end
 end
endmodule
module region_cartoon_stream #(parameter WIDTH=1280,HEIGHT=720)(
 input clk,rst_n,enable,input [95:0] pixels,input valid,sof,frame_end,frame_good,
 input output_ready,output_drained,
 output [95:0] out_pixels,output out_valid,out_frame,out_good,rendering,
 output fault,output [31:0] completed);
 localparam W=WIDTH/4,H=HEIGHT/4;
 reg [8:0] x;reg [9:0] y;
 reg restart,good_hold,armed,abort_active,quality_toggle;
 wire pipeline_reset=rst_n && !restart;
 wire take=valid && (armed || sof) && pipeline_reset;
 wire [8:0] use_x=sof?0:x;
 wire [9:0] use_y=sof?0:y;
 wire [9:0] lr={2'b0,pixels[95:88]}+{2'b0,pixels[71:64]}+{2'b0,pixels[47:40]}+{2'b0,pixels[23:16]};
 wire [9:0] lg={2'b0,pixels[87:80]}+{2'b0,pixels[63:56]}+{2'b0,pixels[39:32]}+{2'b0,pixels[15:8]};
 wire [9:0] lb={2'b0,pixels[79:72]}+{2'b0,pixels[55:48]}+{2'b0,pixels[31:24]}+{2'b0,pixels[7:0]};
 reg qvalid;reg [8:0] qx;reg [9:0] qy;reg [29:0] sums;
 wire [35:0] column;
 wire [11:0] sr=(qy[1:0]==0?12'd0:column[35:24])+sums[29:20];
 wire [11:0] sg=(qy[1:0]==0?12'd0:column[23:12])+sums[19:10];
 wire [11:0] sb=(qy[1:0]==0?12'd0:column[11:0])+sums[9:0];
 cartoon_a_ram #(.WIDTH(36),.AW(9),.DEPTH(512)) columns(
  clk,qvalid,qx,use_x,{sr,sg,sb},column);
 wire [15:0] low_pixel={sr[11:7],sg[11:6],sb[11:7]};
 wire low_valid=qvalid && qy[1:0]==3;
 wire [31:0] filtered;wire fv,filter_fault;
 cartoon_a_low #(.W(W),.H(H)) low(clk,pipeline_reset,low_pixel,low_valid,filtered,fv,filter_fault);
 wire [63:0] guide={pixels[95:91],pixels[87:82],pixels[79:75],
                       pixels[71:67],pixels[63:58],pixels[55:51],
                       pixels[47:43],pixels[39:34],pixels[31:27],
                       pixels[23:19],pixels[15:10],pixels[7:3]};
 wire core_frame,core_good,core_valid,core_render,core_fault;
 wire [95:0] core_pixels;
 cartoon_a_render #(.W(W),.H(H)) render(clk,pipeline_reset,enable,filtered,fv,
  guide,take,use_x,use_y,output_ready,output_drained,good_hold,quality_toggle,
  core_pixels,core_valid,core_frame,core_good,core_render,core_fault,completed);
 assign out_pixels=core_pixels;
 assign out_valid=core_valid && pipeline_reset;
 assign out_frame=core_frame || (restart && abort_active);
 assign out_good=!restart && core_good;
 assign rendering=core_render;
 assign fault=filter_fault || core_fault;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin x<=0;y<=0;qvalid<=0;qx<=0;qy<=0;sums<=0;restart<=0;good_hold<=0;armed<=0;abort_active<=0;quality_toggle<=0;end
  else begin
   restart<=(frame_end && !frame_good) || filter_fault || core_fault;
   if((frame_end && !frame_good) || filter_fault || core_fault)abort_active<=core_render;
   if(restart)begin armed<=0;quality_toggle<=0;end
   else begin
    if(sof && valid)armed<=1;
    if(frame_end && armed)begin good_hold<=frame_good;quality_toggle<=~quality_toggle;end
   end
   qvalid<=take;qx<=use_x;qy<=use_y;sums<={lr,lg,lb};
   if(take)begin
    if(use_x==W-1)begin x<=0;y<=use_y+1'b1;end else begin x<=use_x+1'b1;y<=use_y;end
   end
  end
 end
endmodule

module cartoon_a_guide_ram #(parameter WIDTH=16, AW=14, DEPTH=16384)(
 input clk, input we, input [AW-1:0] wa,ra,
 input [WIDTH-1:0] wd, output [WIDTH-1:0] rd);
 PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(WIDTH),.DATA_WIDTH_B(WIDTH),
 .ADDR_WIDTH_A(AW),.ADDR_WIDTH_B(AW),.DATA_DEPTH_A(DEPTH),.DATA_DEPTH_B(DEPTH),
 .MODE("PDPW"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
 .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
 .IMPLEMENT("20K"),.ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
 .CLKMODE("SYNC"),.RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
 .INIT_FILE("NONE"),.FILL_ALL("NONE")) ram(
 .clka(clk),.clkb(clk),.cea(1'b1),.ceb(1'b1),.ocea(1'b0),.oceb(1'b0),
 .wea(we),.web(1'b0),.addra(wa),.addrb(ra),.dia(wd),.dib({WIDTH{1'b0}}),
 .bea(1'b0),.beb(1'b0),.rsta(1'b0),.rstb(1'b0),.doa(),.dob(rd),
 .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
endmodule
