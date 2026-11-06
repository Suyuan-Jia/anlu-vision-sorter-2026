`ifndef B_MOTION_DETECTOR_DDR_V
`define B_MOTION_DETECTOR_DDR_V
// Passive 4x4 box-mean grayscale capture. Two DDR snapshots occupy independent
// scratch addresses; display pixels never wait for capture or analysis.
// Native addresses are 16-bit units, hence +8 per 128-bit transaction.
module motion_detector_ddr #(
 parameter WIDTH=1280,HEIGHT=720,THRESHOLD=24,MIN_HITS=12,
 parameter [24:0] BASE0=29000000,BASE1=29100000
)(input clk,rst_n,enable,input [95:0] pixels,input valid,sof,frame_end,frame_good,
 output request,write,output [24:0] address,output [127:0] wdata,
 input accept,response,input [127:0] rdata,
 output reg [44:0] result,output reg result_toggle,input result_ack,
 output reg [31:0] completed,skipped);
 localparam LOW_W=WIDTH/4,LOW_H=HEIGHT/4,N=LOW_W*LOW_H,WORDS=N/16;
 localparam IDLE=0,CAPTURE=1,DRAIN=2,READ_CUR=3,WAIT_CUR=4,
            READ_REF=5,WAIT_REF=6,SCAN=7,PUBLISH=8,ABORT=9;
 reg [3:0] state;
 reg cap_pending,inflight;
 reg reference_bank,capture_bank,reference_valid,capture_bad;
 reg [8:0] x;
 reg [10:0] y;
 wire start=enable && valid && sof && state==IDLE && !inflight && !cap_pending;
 wire take=valid && enable && (state==CAPTURE || start);
 wire [8:0] use_x=sof?9'd0:x;
 wire [10:0] use_y=sof?11'd0:y;
 function [7:0] gray(input [23:0] p);
   reg [9:0] t;begin t={2'b0,p[23:16]}+{1'b0,p[15:8],1'b0}+{2'b0,p[7:0]};gray=t[9:2];end
 endfunction
 wire [9:0] lane_sum={2'b0,gray(pixels[95:72])}+{2'b0,gray(pixels[71:48])}+
                    {2'b0,gray(pixels[47:24])}+{2'b0,gray(pixels[23:0])};
 reg qvalid;
 reg [8:0] qx;
 reg [10:0] qy;
 reg [9:0] qsum;
 wire [11:0] column;
 wire [11:0] total=(qy[1:0]==0?12'd0:column)+{2'b0,qsum};
 region_ram #(.WIDTH(12),.AW(9),.DEPTH(512)) columns(
   .clk(clk),.we(qvalid && state==CAPTURE),.wa(qx),.ra(use_x),.wd(total),.rd(column));
 reg [15:0] gray_count;
 reg [11:0] write_word,work_word;
 reg [127:0] gray_pack,cap_data,current_word,previous_word;
 reg [24:0] cap_address;
 reg [1:0] transaction_kind;
 // Display bursts can occupy hundreds of DDR clocks. Queue complete sampled
 // words locally; queue overflow rejects analysis only, never video pixels.
 wire fifo_full,fifo_empty,fifo_valid;
 wire [127:0] fifo_data;
 reg fifo_read_inflight;
 reg [11:0] drain_word;
 wire fifo_reset=!rst_n || !enable || state==ABORT || start;
 wire fifo_push=qvalid && state==CAPTURE && qy[1:0]==3 && gray_count[3:0]==15;
 wire fifo_pop=!fifo_empty && !cap_pending && !fifo_read_inflight && (state==CAPTURE || state==DRAIN);
 soft_fifo_al_4057d6b76aa6 #(
   .DATA_WIDTH_W(128),.DATA_WIDTH_R(128),.ADDR_WIDTH_W(6),.ADDR_WIDTH_R(6),
   .AL_FULL_NUM(60),.AL_EMPTY_NUM(2),.SHOW_AHEAD_EN(0),.OUTREG_EN("NOREG")
 ) sample_fifo(
   .rst(fifo_reset),.clkw(clk),.clkr(clk),.we(fifo_push && !fifo_full),
   .di({gray_pack[119:0],total[11:4]}),.re(fifo_pop),.dout(fifo_data),.valid(fifo_valid),
   .full_flag(fifo_full),.empty_flag(fifo_empty),.afull(),.aempty(),.wrusedw(),.rdusedw());
 wire cap_send=cap_pending;
 wire work_send=(state==READ_CUR || state==READ_REF);
 assign request=!inflight && (cap_send || work_send);
 assign write=cap_send;
 wire selected_bank=(state==READ_CUR)?capture_bank:reference_bank;
 assign address=cap_send?cap_address:(selected_bank?BASE1:BASE0)+{10'd0,work_word,3'b0};
 assign wdata=cap_data;
 wire sent=request && accept;
 reg [3:0] lane;
 reg [8:0] scan_x,scan_y,run_length,xmin,ymin,xmax,ymax;
 reg box_valid;
 reg [15:0] hits,filtered_hits;
 wire [7:0] a=current_word[127:120],b=previous_word[127:120];
 wire [8:0] delta=a>=b?{1'b0,a}-{1'b0,b}:{1'b0,b}-{1'b0,a};
 wire changed=delta>=THRESHOLD;
 reg scan_pending,scan_changed;
 reg [8:0] filtered_x,filtered_y,filtered_run;
 wire [1:0] previous_rows;
 wire [1:0] row_history={filtered_y==0?1'b0:previous_rows[0],scan_changed};
 region_ram #(.WIDTH(2),.AW(9),.DEPTH(512)) mask_columns(
   .clk(clk),.we(scan_pending),.wa(filtered_x),.ra(scan_x),.wd(row_history),.rd(previous_rows));
 wire horizontal=filtered_run>=2;
 wire vertical=filtered_y>=2 && (&previous_rows);
 wire [8:0] box_left=horizontal?filtered_x-2:filtered_x;
 wire [8:0] box_top=vertical?filtered_y-2:filtered_y;
 (* ASYNC_REG="TRUE" *) reg [2:0] result_ack_sync;
 reg clear_sent;
 wire result_free=result_ack_sync[2]==result_toggle;
 always @(posedge clk or negedge rst_n)
   if(!rst_n)begin
     state<=IDLE;reference_bank<=0;capture_bank<=1;reference_valid<=0;capture_bad<=0;
     x<=0;y<=0;qvalid<=0;qx<=0;qy<=0;qsum<=0;gray_count<=0;write_word<=0;work_word<=0;
     gray_pack<=0;cap_data<=0;cap_address<=0;cap_pending<=0;inflight<=0;transaction_kind<=0;
     fifo_read_inflight<=0;drain_word<=0;
     current_word<=0;previous_word<=0;lane<=0;scan_x<=0;scan_y<=0;run_length<=0;
     xmin<=0;ymin<=0;xmax<=0;ymax<=0;box_valid<=0;hits<=0;filtered_hits<=0;
     scan_pending<=0;scan_changed<=0;filtered_x<=0;filtered_y<=0;filtered_run<=0;
     result<=0;result_toggle<=0;result_ack_sync<=0;clear_sent<=0;completed<=0;skipped<=0;
   end else begin
     result_ack_sync<={result_ack_sync[1:0],result_ack};
     qvalid<=take;qx<=use_x;qy<=use_y;qsum<=lane_sum;
     scan_pending<=state==SCAN;
     scan_changed<=changed;filtered_x<=scan_x;filtered_y<=scan_y;filtered_run<=run_length;
     if(scan_pending)begin
       if(scan_changed)begin
         hits<=hits+1'b1;
         // Three adjacent samples in either direction reject isolated noise
         // while retaining thin vertical changes from slow horizontal motion.
         if(horizontal || vertical)begin
           filtered_hits<=filtered_hits+1'b1;
           if(!box_valid || box_left<xmin)xmin<=box_left;
           if(!box_valid || filtered_x>xmax)xmax<=filtered_x;
           if(!box_valid || box_top<ymin)ymin<=box_top;
           if(!box_valid || filtered_y>ymax)ymax<=filtered_y;
           box_valid<=1;
         end
       end
     end
     if(take)begin
       if(use_x==LOW_W-1)begin x<=0;y<=use_y+1'b1;end
       else begin x<=use_x+1'b1;y<=use_y;end
     end
     if(sent)begin
       inflight<=1;
       if(cap_send)begin cap_pending<=0;transaction_kind<=0;end
       else if(state==READ_CUR)begin transaction_kind<=1;state<=WAIT_CUR;end
       else begin transaction_kind<=2;state<=WAIT_REF;end
     end
     if(response && inflight)begin
       inflight<=0;
       if(state!=ABORT && enable)begin
         if(transaction_kind==1)begin current_word<=rdata;state<=READ_REF;end
         if(transaction_kind==2)begin previous_word<=rdata;lane<=0;state<=SCAN;end
       end
     end
     if(start)begin
       state<=CAPTURE;capture_bank<=~reference_bank;capture_bad<=0;gray_count<=0;write_word<=0;gray_pack<=0;
       drain_word<=0;fifo_read_inflight<=0;
     end
     if(fifo_pop)fifo_read_inflight<=1;
     if(fifo_valid && (state==CAPTURE || state==DRAIN))begin
       fifo_read_inflight<=0;cap_pending<=1;cap_data<=fifo_data;
       cap_address<=(capture_bank?BASE1:BASE0)+{10'd0,drain_word,3'b0};
       drain_word<=drain_word+1'b1;
     end
     if(qvalid && state==CAPTURE && qy[1:0]==3)begin
       gray_pack<={gray_pack[119:0],total[11:4]};gray_count<=gray_count+1'b1;
       if(gray_count[3:0]==15)begin
         write_word<=write_word+1'b1;
         if(fifo_full)capture_bad<=1;
       end
     end
     if(frame_end && state==CAPTURE)begin
       if(frame_good && !capture_bad && gray_count==N && write_word==WORDS)state<=DRAIN;
       else begin state<=ABORT;cap_pending<=0;skipped<=skipped+1'b1;end
     end
     case(state)
       DRAIN:if(fifo_empty && !fifo_read_inflight && !cap_pending && !inflight && drain_word==WORDS)begin
         if(!reference_valid)begin reference_bank<=capture_bank;reference_valid<=1;state<=IDLE;end
         else begin
           work_word<=0;scan_x<=0;scan_y<=0;run_length<=0;box_valid<=0;hits<=0;filtered_hits<=0;
           xmin<=LOW_W-1;ymin<=LOW_H-1;xmax<=0;ymax<=0;state<=READ_CUR;
         end
       end
       SCAN:begin
         current_word<=current_word<<8;previous_word<=previous_word<<8;
         if(changed)begin
           if(run_length<LOW_W)run_length<=run_length+1'b1;
         end else run_length<=0;
         if(scan_x==LOW_W-1)begin scan_x<=0;scan_y<=scan_y+1'b1;run_length<=0;end
         else scan_x<=scan_x+1'b1;
         if(lane==15)begin
           if(work_word==WORDS-1)state<=PUBLISH;
           else begin work_word<=work_word+1'b1;state<=READ_CUR;end
         end else lane<=lane+1'b1;
       end
       PUBLISH:if(result_free && !scan_pending)begin
         // Large global changes are treated as exposure/camera change, not a target.
         result<={box_valid && hits>=MIN_HITS && filtered_hits>=6 && hits<=N/3,
                   xmin,2'b00,ymin,2'b00,xmax,2'b11,ymax,2'b11};
         result_toggle<=~result_toggle;reference_bank<=capture_bank;reference_valid<=1;
         completed<=completed+1'b1;state<=IDLE;
       end
       ABORT:begin cap_pending<=0;fifo_read_inflight<=0;if(!inflight)state<=IDLE;end
       default:;
     endcase
     if(sof && valid && enable && !start)skipped<=skipped+1'b1;
     if(enable)clear_sent<=0;
     else begin
       reference_valid<=0;cap_pending<=0;fifo_read_inflight<=0;
       if(state!=IDLE && state!=ABORT)state<=ABORT;
       if(result_free && !clear_sent)begin result<=0;result_toggle<=~result_toggle;clear_sent<=1;end
     end
   end
endmodule

`endif
