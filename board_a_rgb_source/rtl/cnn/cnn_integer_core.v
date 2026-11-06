// Time-shared integer execution of the frozen, single shared grayscale CNN.
// This module owns only analysis RAM. It has no connection to video ready.
module cnn_integer_core #(
 parameter [32:0] MIN_SHAPE_MARGIN_Q10=33'd64,
 parameter [32:0] MIN_UNKNOWN_MARGIN_Q10=33'd512,
 parameter [32:0] ACQUIRE_SHAPE_MARGIN_Q10=33'd1024,ACQUIRE_UNKNOWN_MARGIN_Q10=33'd2048,FAST_MARGIN_Q10=33'd8192
)( 
 input wire clk,rst_n,start,release_frame,
 input wire capture_wr,
 input wire [13:0] capture_addr,
 input wire [15:0] capture_rgb565,
 output reg busy,nn_done,
 output reg [1:0] predicted_shape,
 output reg [31:0] inference_cycles,
 input wire [2:0] service_rd_bank,
 input wire [14:0] service_rd_addr,
 output wire [15:0] service_rd_data,
 input wire service_wr,
 input wire [2:0] service_wr_bank,
 input wire [14:0] service_wr_addr,
 input wire [15:0] service_wr_data,
 output reg [1:0] identity_quality
);
 // GENERATED CONSTANTS BEGIN
// Generated integer coefficients, matching the selected RGB graph.
localparam L_E1_0=0, W_E1_0=0, B_E1_0=0;
localparam L_E2_0=1, W_E2_0=216, B_E2_0=4;
localparam L_E3_0=2, W_E3_0=504, B_E3_0=12;
localparam L_E4_0=3, W_E4_0=1656, B_E4_0=28;
localparam L_CONTEXT1_0=4, W_CONTEXT1_0=5112, B_CONTEXT1_0=52;
localparam L_CONTEXT2_0=5, W_CONTEXT2_0=10296, B_CONTEXT2_0=76;
localparam L_P3_0=6, W_P3_0=15480, B_P3_0=100;
localparam L_D3_0=7, W_D3_0=15864, B_D3_0=116;
localparam L_P2_0=8, W_P2_0=18168, B_P2_0=132;
localparam L_D2_0=9, W_D2_0=18296, B_D2_0=140;
localparam L_P1_0=10, W_P1_0=18872, B_P1_0=148;
localparam L_D1_0=11, W_D1_0=18904, B_D1_0=152;
localparam L_DETAIL_0=12, W_DETAIL_0=19048, B_DETAIL_0=156;
localparam L_REFINE_0=13, W_REFINE_0=19052, B_REFINE_0=160;
localparam L_HEAD=14, W_HEAD=19196, B_HEAD=164;
localparam L_SHAPE_CONV_0_0=15, W_SHAPE_CONV_0_0=19200, B_SHAPE_CONV_0_0=165;
localparam L_SHAPE_CONV_2_0=16, W_SHAPE_CONV_2_0=22872, B_SHAPE_CONV_2_0=189;
localparam L_SHAPE_HEAD=17, W_SHAPE_HEAD=28056, B_SHAPE_HEAD=213;
localparam FILE_WEIGHTS="E:/Anlogic_FPGA/anlogic-vision-sorter-split-two-boards-final-copy-1/board_a_rgb_source/rtl/cnn/generated/weights.mem";
localparam FILE_BIAS="E:/Anlogic_FPGA/anlogic-vision-sorter-split-two-boards-final-copy-1/board_a_rgb_source/rtl/cnn/generated/bias.mem";
localparam FILE_OUTPUT_MULTIPLIER="E:/Anlogic_FPGA/anlogic-vision-sorter-split-two-boards-final-copy-1/board_a_rgb_source/rtl/cnn/generated/output_multiplier.mem";
localparam FILE_INPUT_MULTIPLIER="E:/Anlogic_FPGA/anlogic-vision-sorter-split-two-boards-final-copy-1/board_a_rgb_source/rtl/cnn/generated/input_multiplier.mem";
localparam FILE_SIGMOID="E:/Anlogic_FPGA/anlogic-vision-sorter-split-two-boards-final-copy-1/board_a_rgb_source/rtl/cnn/generated/sigmoid.mem";
 // GENERATED CONSTANTS END
 localparam IDLE=0,CONFIG=1,PIXEL=2,TAP=3,READ_WAIT=4,READ_USE=5,SECOND_WAIT=6,SECOND_USE=7,
  DETAIL_WAIT=8,INPUT_PRODUCT=9,INPUT_QUANT=10,MAC=11,ADVANCE=12,OUTPUT_START=13,OUTPUT_WAIT=14,
  HEAD_INPUT=15,HEAD_QUANT=16,HEAD_MAC=17,HEAD_NEXT=18,HEAD_OUT=19,HEAD_WAIT=20,
  COPY_READ=21,COPY_WAIT=22,COPY_WRITE=23,
  BOX_TAP=24,BOX_WAIT=25,BOX_USE=26,BOX_STEP=27,BOX_EVAL=28,BOX_FINISH=29,
  ROI_START=30,ROI_WAIT=31,POOL_READ=32,POOL_WAIT=33,POOL_USE=34,POOL_STEP=35,POOL_WRITE=36,FINISHED=37,
  GRAY_NORMALIZE=38,CACHE_READ=39,CACHE_WAIT=40,CACHE_GRAY=41,CACHE_WEIGHT=42,CACHE_START=43,CACHE_RQWAIT=44,DOT_DRAIN=45;
 localparam NORM_READ=46,NORM_WAIT=47,NORM_USE=48,NORM_START=49,NORM_DONE=50,NORM_WRITE=51;
 reg norm_start;reg [13:0] norm_index;reg [15:0] norm_rgb;
 wire norm_done;wire [15:0] norm_value;
 cnn_rgb_normalize color_normalizer(clk,rst_n,norm_start,norm_rgb,norm_done,norm_value);
 reg [5:0] state,stage;
 reg [7:0] iw,ih,ow,oh,ci_count,co_count,kernel,stride,dilation;
 reg [2:0] mode,src_bank,skip_bank,dst_bank;
 reg [14:0] src_base,skip_base,dst_base,weight_base;
 reg [7:0] bias_base,layer_id;
 reg [14:0] input_plane,skip_plane;
 reg [7:0] source_row_width,kernel_volume;
 // GENERATED PROGRAM BEGIN
always @* begin
iw=1;ih=1;ow=1;oh=1;ci_count=1;co_count=1;kernel=1;stride=1;dilation=1;
mode=0;src_bank=0;skip_bank=0;dst_bank=0;src_base=0;skip_base=0;dst_base=0;weight_base=0;bias_base=0;layer_id=0;
input_plane=0;skip_plane=0;source_row_width=0;kernel_volume=0;
case(stage)
0:begin iw=160;ih=96;ow=80;oh=48;ci_count=6;co_count=4;kernel=3;stride=2;dilation=1;mode=0;src_bank=2;src_base=0;skip_bank=0;skip_base=0;dst_bank=1;dst_base=0;weight_base=0;bias_base=0;layer_id=0;input_plane=15360;skip_plane=15360;source_row_width=160;kernel_volume=54;end
1:begin iw=80;ih=48;ow=40;oh=24;ci_count=4;co_count=8;kernel=3;stride=2;dilation=1;mode=0;src_bank=1;src_base=0;skip_bank=0;skip_base=0;dst_bank=0;dst_base=0;weight_base=216;bias_base=4;layer_id=1;input_plane=3840;skip_plane=3840;source_row_width=80;kernel_volume=36;end
2:begin iw=40;ih=24;ow=20;oh=12;ci_count=8;co_count=16;kernel=3;stride=2;dilation=1;mode=0;src_bank=0;src_base=0;skip_bank=0;skip_base=0;dst_bank=0;dst_base=7680;weight_base=504;bias_base=12;layer_id=2;input_plane=960;skip_plane=960;source_row_width=40;kernel_volume=72;end
3:begin iw=20;ih=12;ow=10;oh=6;ci_count=16;co_count=24;kernel=3;stride=2;dilation=1;mode=0;src_bank=0;src_base=7680;skip_bank=0;skip_base=0;dst_bank=0;dst_base=11520;weight_base=1656;bias_base=28;layer_id=3;input_plane=240;skip_plane=240;source_row_width=20;kernel_volume=144;end
4:begin iw=10;ih=6;ow=10;oh=6;ci_count=24;co_count=24;kernel=3;stride=1;dilation=2;mode=0;src_bank=0;src_base=11520;skip_bank=0;skip_base=0;dst_bank=0;dst_base=12960;weight_base=5112;bias_base=52;layer_id=4;input_plane=60;skip_plane=60;source_row_width=10;kernel_volume=216;end
5:begin iw=10;ih=6;ow=10;oh=6;ci_count=24;co_count=24;kernel=3;stride=1;dilation=1;mode=0;src_bank=0;src_base=12960;skip_bank=0;skip_base=0;dst_bank=0;dst_base=11520;weight_base=10296;bias_base=76;layer_id=5;input_plane=60;skip_plane=60;source_row_width=10;kernel_volume=216;end
6:begin iw=10;ih=6;ow=10;oh=6;ci_count=24;co_count=16;kernel=1;stride=1;dilation=1;mode=0;src_bank=0;src_base=11520;skip_bank=0;skip_base=0;dst_bank=0;dst_base=12960;weight_base=15480;bias_base=100;layer_id=6;input_plane=60;skip_plane=60;source_row_width=10;kernel_volume=24;end
7:begin iw=20;ih=12;ow=20;oh=12;ci_count=16;co_count=16;kernel=3;stride=1;dilation=1;mode=1;src_bank=0;src_base=12960;skip_bank=0;skip_base=7680;dst_bank=0;dst_base=13920;weight_base=15864;bias_base=116;layer_id=7;input_plane=60;skip_plane=240;source_row_width=10;kernel_volume=144;end
8:begin iw=20;ih=12;ow=20;oh=12;ci_count=16;co_count=8;kernel=1;stride=1;dilation=1;mode=0;src_bank=0;src_base=13920;skip_bank=0;skip_base=0;dst_bank=0;dst_base=7680;weight_base=18168;bias_base=132;layer_id=8;input_plane=240;skip_plane=240;source_row_width=20;kernel_volume=16;end
9:begin iw=40;ih=24;ow=40;oh=24;ci_count=8;co_count=8;kernel=3;stride=1;dilation=1;mode=1;src_bank=0;src_base=7680;skip_bank=0;skip_base=0;dst_bank=0;dst_base=9600;weight_base=18296;bias_base=140;layer_id=9;input_plane=240;skip_plane=960;source_row_width=20;kernel_volume=72;end
11:begin iw=40;ih=24;ow=40;oh=24;ci_count=8;co_count=4;kernel=1;stride=1;dilation=1;mode=0;src_bank=0;src_base=3840;skip_bank=0;skip_base=0;dst_bank=0;dst_base=0;weight_base=18872;bias_base=148;layer_id=10;input_plane=960;skip_plane=960;source_row_width=40;kernel_volume=8;end
12:begin iw=80;ih=48;ow=80;oh=48;ci_count=4;co_count=4;kernel=3;stride=1;dilation=1;mode=1;src_bank=0;src_base=0;skip_bank=1;skip_base=0;dst_bank=0;dst_base=11520;weight_base=18904;bias_base=152;layer_id=11;input_plane=960;skip_plane=3840;source_row_width=40;kernel_volume=36;end
13:begin iw=160;ih=96;ow=160;oh=96;ci_count=4;co_count=4;kernel=3;stride=1;dilation=1;mode=2;src_bank=0;src_base=11520;skip_bank=0;skip_base=0;dst_bank=3;dst_base=0;weight_base=19052;bias_base=160;layer_id=13;input_plane=3840;skip_plane=15360;source_row_width=80;kernel_volume=36;end
16:begin iw=16;ih=16;ow=16;oh=16;ci_count=17;co_count=24;kernel=3;stride=1;dilation=1;mode=0;src_bank=4;src_base=0;skip_bank=0;skip_base=0;dst_bank=0;dst_base=11520;weight_base=19200;bias_base=165;layer_id=15;input_plane=256;skip_plane=256;source_row_width=16;kernel_volume=153;end
18:begin iw=8;ih=8;ow=8;oh=8;ci_count=24;co_count=24;kernel=3;stride=1;dilation=1;mode=0;src_bank=0;src_base=4100;skip_bank=0;skip_base=0;dst_bank=0;dst_base=0;weight_base=22872;bias_base=189;layer_id=16;input_plane=64;skip_plane=64;source_row_width=8;kernel_volume=216;end
20:begin iw=1;ih=1;ow=1;oh=1;ci_count=96;co_count=4;kernel=1;stride=1;dilation=1;mode=0;src_bank=0;src_base=1700;skip_bank=0;skip_base=0;dst_bank=0;dst_base=0;weight_base=28056;bias_base=213;layer_id=17;input_plane=1;skip_plane=1;source_row_width=1;kernel_volume=96;end
default:begin end
endcase
end
 // GENERATED PROGRAM END
 reg signed [7:0] weights[0:28439] /* synthesis rom_style=dram */;
 reg signed [31:0] biases[0:216] /* synthesis rom_style=dram */;
 reg [31:0] multipliers[0:216] /* synthesis rom_style=dram */;
 reg [31:0] input_multipliers[0:17] /* synthesis rom_style=dram */;
 reg [7:0] sigmoid[0:4096] /* synthesis rom_style=dram */;
 initial begin
  $readmemh(FILE_WEIGHTS,weights);$readmemh(FILE_BIAS,biases);
  $readmemh(FILE_OUTPUT_MULTIPLIER,multipliers);$readmemh(FILE_INPUT_MULTIPLIER,input_multipliers);
  $readmemh(FILE_SIGMOID,sigmoid);
 end
 reg [2:0] core_rd_bank;
 reg [14:0] core_rd_addr;
 reg core_wr;reg [2:0] core_wr_bank;reg [14:0] core_wr_addr;reg [15:0] core_wr_data;
 wire roi_busy,roi_done,roi_wr;reg roi_start;
 wire [2:0] roi_rd_bank;wire [14:0] roi_rd_addr;wire [12:0] roi_wr_addr;wire [7:0] roi_wr_data;
 reg [7:0] box_x0,box_y0,box_x1,box_y1;
 wire select_busy,select_done,select_queue_wr;
 wire [7:0] select_x0,select_y0,select_x1,select_y1;
 wire [14:0] select_queue_ra,select_queue_wa;
 wire [15:0] select_queue_data,select_queue_q;
 cnn_packed_ram #(.WIDTH(16),.AW(14),.DEPTH(14400)) roi_queue(clk,select_queue_wr,
  select_queue_wa[13:0],select_queue_ra[13:0],select_queue_data,select_queue_q);
 wire [2:0] read_bank=roi_busy ? roi_rd_bank : state==FINISHED ? service_rd_bank : core_rd_bank;
 wire [14:0] read_addr=roi_busy ? roi_rd_addr : state==FINISHED ? service_rd_addr : core_rd_addr;
 reg [2:0] read_bank_d;
 always @(posedge clk)read_bank_d<=read_bank;
 wire [15:0] arena_q,shallow_q,rgb_q;wire [7:0] prob_q,roi_q;
 reg roi_byte_d;
 reg [7:0] roi_even;
 always @(posedge clk)begin
  roi_byte_d<=read_addr[0];
  if(roi_wr && !roi_wr_addr[0])roi_even<=roi_wr_data;
 end
 wire [14:0] arena_read_addr=read_bank==4 ? (read_addr[12] ? 15'd2050+{8'd0,read_addr[7:1]} : {1'b0,read_addr[14:1]}) : read_addr;
 wire roi_pair_write=roi_wr && roi_wr_addr[0];
 wire [14:0] roi_pair_addr=roi_wr_addr[12] ? 15'd2050+{8'd0,roi_wr_addr[7:1]} : {3'd0,roi_wr_addr[12:1]};
 wire [15:0] shallow_low_q,shallow_mid_q,shallow_high_q;
 wire [11:0] shallow_mid_read=read_addr-15'd7680,shallow_mid_write=core_wr_addr-15'd7680;
 wire [11:0] shallow_high_read=read_addr-15'd11776,shallow_high_write=core_wr_addr-15'd11776;
 reg [1:0] shallow_region_d;
 always @(posedge clk)shallow_region_d<=read_addr<7680 ? 0 : read_addr<11776 ? 1 : 2;
 assign shallow_q=shallow_region_d==0 ? shallow_low_q : shallow_region_d==1 ? shallow_mid_q : shallow_high_q;
 wire service_write=state==FINISHED && service_wr;
 cnn_packed_ram arena(clk,roi_pair_write || (service_write && service_wr_bank==0) || (core_wr && core_wr_bank==0),
  roi_pair_write ? roi_pair_addr : service_write ? service_wr_addr : core_wr_addr,arena_read_addr,
  roi_pair_write ? {roi_wr_data,roi_even} : service_write ? service_wr_data : core_wr_data,arena_q);
 cnn_packed_ram #(.WIDTH(16),.AW(13),.DEPTH(7680)) shallow_low(clk,core_wr && core_wr_bank==1 && core_wr_addr<7680,
  core_wr_addr[12:0],read_addr[12:0],core_wr_data,shallow_low_q);
 cnn_packed_ram #(.WIDTH(16),.AW(12),.DEPTH(4096)) shallow_mid(clk,core_wr && core_wr_bank==1 && core_wr_addr>=7680 && core_wr_addr<11776,
  shallow_mid_write,shallow_mid_read,core_wr_data,shallow_mid_q);
 cnn_packed_ram #(.WIDTH(16),.AW(12),.DEPTH(3584)) shallow_high(clk,core_wr && core_wr_bank==1 && core_wr_addr>=11776,
  shallow_high_write,shallow_high_read,core_wr_data,shallow_high_q);
 cnn_packed_ram #(.WIDTH(16),.AW(14),.DEPTH(14400)) rgb_cache(clk,capture_wr,capture_addr,read_addr[13:0],capture_rgb565,rgb_q);
 cnn_packed_ram #(.WIDTH(8),.AW(14),.DEPTH(15360)) probability(clk,(service_write && service_wr_bank==3) || (core_wr && core_wr_bank==3),
  service_write ? service_wr_addr[13:0] : core_wr_addr[13:0],read_addr[13:0],service_write ? service_wr_data[7:0] : core_wr_data[7:0],prob_q);
 assign roi_q=roi_byte_d ? arena_q[15:8] : arena_q[7:0];
 wire [15:0] read_data=read_bank_d==0 ? arena_q : read_bank_d==1 ? shallow_q : read_bank_d==2 ? rgb_q :
                        read_bank_d==3 ? {8'd0,prob_q} : {8'd0,roi_q};
 assign service_rd_data=read_data;
 cnn_roi_engine roi_engine(clk,rst_n,roi_start,box_x0,box_x1,box_y0,box_y1,
  input_multipliers[L_SHAPE_CONV_0_0],roi_busy,roi_done,roi_rd_bank,roi_rd_addr,read_data,roi_wr,roi_wr_addr,roi_wr_data);
 reg rq_start,rq_relu;reg signed [31:0] rq_value;reg [31:0] rq_multiplier;
 wire rq_done;wire signed [31:0] rq_result;
 cnn_requantize requant(clk,rst_n,rq_start,rq_value,rq_multiplier,rq_relu,rq_done,rq_result);
 reg [7:0] co,ci,ox,oy,kx,ky;
 reg signed [31:0] accumulator,best_logit,second_logit,unknown_logit;
 reg [1:0] best_shape,head_ci;
 // Include the final class output in ranking before applying rejection.
 wire final_is_best=rq_result>best_logit;
 wire signed [31:0] final_best=final_is_best ? rq_result : best_logit;
 wire signed [31:0] final_second=final_is_best ? best_logit : rq_result>second_logit ? rq_result : second_logit;
 wire [1:0] final_shape=final_is_best ? co[1:0] : best_shape;
 // Extend before subtracting so extreme signed logits cannot wrap.
 wire signed [32:0] shape_margin=$signed({final_best[31],final_best})-$signed({final_second[31],final_second});
 wire signed [32:0] unknown_margin=$signed({final_best[31],final_best})-$signed({unknown_logit[31],unknown_logit});
 wire shape_accepted=final_shape!=0 && shape_margin>=$signed(MIN_SHAPE_MARGIN_Q10) && unknown_margin>=$signed(MIN_UNKNOWN_MARGIN_Q10);

 reg [15:0] refine_pixel[0:3];
 reg [16:0] input_value;
 reg [15:0] first_value;
 reg [48:0] input_product;
 reg [7:0] input_q;
 reg gray_padding;
 wire [7:0] gray_pixel;
 wire signed [12:0] sx,sy;
 reg [7:0] gray_cached;
 reg [6:0] cache_row;
 reg [7:0] cache_x;
 reg [1:0] cache_ch,cache_slot;
 reg cache_prefill;
 reg [6:0] cache_tag0,cache_tag1,cache_tag2;
 wire [1:0] detail_slot=sy[6:0]==cache_tag0 ? 0 : sy[6:0]==cache_tag1 ? 1 : 2;
 wire [14:0] detail_address=(detail_slot<<9)+(detail_slot<<7)+(sx<<2)+ci;
 wire [14:0] detail_write_address=(cache_slot<<9)+(cache_slot<<7)+(cache_x<<2)+cache_ch;
 reg [14:0] weight_pointer,channel_weight_base,input_channel_base,skip_channel_base,output_address;
 assign sx=(stride==2 ? {4'd0,ox,1'b0} : {5'd0,ox})+
                     (dilation==2 ? {4'd0,kx,1'b0} : {5'd0,kx})-(kernel==3 ? (dilation==2 ? 13'd2 : 13'd1) : 13'd0);
 assign sy=(stride==2 ? {4'd0,oy,1'b0} : {5'd0,oy})+
                     (dilation==2 ? {4'd0,ky,1'b0} : {5'd0,ky})-(kernel==3 ? (dilation==2 ? 13'd2 : 13'd1) : 13'd0);
 wire within_bounds=sx>=0 && sy>=0 && sx<iw && sy<ih;
 wire [7:0] source_y=mode==1 || mode==2 ? sy[8:1] : sy[7:0];
 (* dsp_style="yes" *) wire [15:0] source_row_offset=source_y*source_row_width;
 (* dsp_style="yes" *) wire [15:0] skip_row_offset=sy[7:0]*iw;
 wire [14:0] shared_weight_addr=state==HEAD_INPUT ? W_HEAD+head_ci :
                            state==CACHE_WEIGHT ? W_DETAIL_0+cache_ch : weight_pointer;

 reg [12:0] packed_base;
 always @*begin case(layer_id)
0:packed_base=13'd0;
1:packed_base=13'd54;
2:packed_base=13'd126;
3:packed_base=13'd414;
4:packed_base=13'd1278;
5:packed_base=13'd2574;
6:packed_base=13'd3870;
7:packed_base=13'd3966;
8:packed_base=13'd4542;
9:packed_base=13'd4574;
10:packed_base=13'd4718;
11:packed_base=13'd4726;
12:packed_base=13'd4762;
13:packed_base=13'd4763;
14:packed_base=13'd4799;
15:packed_base=13'd4803;
16:packed_base=13'd5721;
17:packed_base=13'd7017;
default:packed_base=0;endcase end
 (* dsp_style="no" *) wire [12:0] packed_group=co[4:2]*kernel_volume;
 wire [12:0] packed_address=packed_base+packed_group+(weight_pointer-channel_weight_base);
 reg [31:0] packed_weights[0:7113-1] /* synthesis rom_style=block */;
 reg [31:0] packed_q;
 initial $readmemh("E:/Anlogic_FPGA/anlogic-vision-sorter-split-two-boards-final-copy-1/board_a_rgb_source/rtl/cnn/generated/weights_packed.mem",packed_weights);
 always @(posedge clk)packed_q<=packed_weights[packed_address];
 function [95:0] group_bias;
 input [7:0] code;begin case(code)
8'd0:group_bias=96'hffff59ccffffab86fffebd0f;
8'd8:group_bias=96'h000004ea00001ffdfffff21d;
8'd9:group_bias=96'hfffff489fffffdd800000319;
8'd16:group_bias=96'h00000c690000172ffffff933;
8'd17:group_bias=96'hfffffecb00000e48fffffa0c;
8'd18:group_bias=96'hfffff6e6fffffa08000001a1;
8'd19:group_bias=96'h00000b4200000ec60000039b;
8'd24:group_bias=96'h0000128dfffffc38ffffff42;
8'd25:group_bias=96'hffffe978000008c000000a34;
8'd26:group_bias=96'h000021daffffe89400001aa3;
8'd27:group_bias=96'hffffe11b0000048affffff0c;
8'd28:group_bias=96'h0000045d00000e94ffffdd49;
8'd29:group_bias=96'hfffff974fffffd2200001090;
8'd32:group_bias=96'h000000b1000005ab0000101d;
8'd33:group_bias=96'hfffff7400000035800000dc5;
8'd34:group_bias=96'h00000b760000100e00000807;
8'd35:group_bias=96'h000005e600000bc8fffff825;
8'd36:group_bias=96'h00000209000003d3000001f2;
8'd37:group_bias=96'h0000099b0000066f00000141;
8'd40:group_bias=96'hfffffd49ffffea7d00001c7b;
8'd41:group_bias=96'h0000244500001b130000139c;
8'd42:group_bias=96'h00001d4a00000fdd00001738;
8'd43:group_bias=96'hffffffd3000003b000001e98;
8'd44:group_bias=96'hfffffe76ffffdeac00000689;
8'd45:group_bias=96'hfffff96afffff2e700000f5c;
8'd48:group_bias=96'h00000267fffffae7fffff78b;
8'd49:group_bias=96'h000001320000004000000594;
8'd50:group_bias=96'h00000381000002a300000b53;
8'd51:group_bias=96'h000007da00000ff100000390;
8'd56:group_bias=96'hfffff7ea00001b830000156d;
8'd57:group_bias=96'h0000101c000014a400001602;
8'd58:group_bias=96'h000016570000279600002b1a;
8'd59:group_bias=96'h00003d28000001d5000006e4;
8'd64:group_bias=96'hffffff9a0000064afffff871;
8'd65:group_bias=96'hfffffac2000004ac0000075a;
8'd72:group_bias=96'hfffff97d0000120d00002a63;
8'd73:group_bias=96'h000010460000172a00000958;
8'd80:group_bias=96'h0000009ffffffa93fffffe32;
8'd88:group_bias=96'hfffffa09ffffe731ffffd353;
8'd96:group_bias=96'hffffd8c0ffffcf76fffef49d;
8'd104:group_bias=96'h00001e70000021d6fffffdcc;
8'd112:group_bias=96'h000000000000000000000000;
8'd120:group_bias=96'h000008df000015d3ffffedbf;
8'd121:group_bias=96'h00000a5affffff4effffd94e;
8'd122:group_bias=96'h00000fad0000146900001604;
8'd123:group_bias=96'h00000bd40000064fffffe6fa;
8'd124:group_bias=96'h00000489000005c6fffffd63;
8'd125:group_bias=96'h000009fa000012f5000012a1;
8'd128:group_bias=96'hffffd1daffffd67e00003114;
8'd129:group_bias=96'hffffeeb1000001ceffffe8e6;
8'd130:group_bias=96'h00000d980000190e000032aa;
8'd131:group_bias=96'hfffffb1900000735000014c2;
8'd132:group_bias=96'hfffff789fffffc22ffffd995;
8'd133:group_bias=96'h00000fc3fffff756ffffddfa;
8'd136:group_bias=96'hfffffe8e00000212000000be;default:group_bias=0;endcase end endfunction
 wire [95:0] parallel_bias=group_bias({layer_id[4:0],co[4:2]});
 function signed [7:0] small_weight;
 input is_head;input [1:0] channel;begin case({is_head,channel})
3'd0:small_weight=8'h81;
3'd1:small_weight=8'h7f;
3'd2:small_weight=8'h7f;
3'd3:small_weight=8'h7f;
3'd4:small_weight=8'h2d;
3'd5:small_weight=8'h2b;
3'd6:small_weight=8'h82;
3'd7:small_weight=8'h81;
endcase end endfunction
 reg signed [7:0] small_cached;
 wire signed [7:0] weight_cached=state==TAP || state==SECOND_WAIT || state==DOT_DRAIN ? $signed(packed_q[7:0]) : small_cached;
 always @(posedge clk)if(state==HEAD_INPUT || state==CACHE_WEIGHT)small_cached<=small_weight(state==HEAD_INPUT,state==HEAD_INPUT ? head_ci : cache_ch);
 reg [14:0] output_plane;
 always @*begin case(stage)
  0:output_plane=3840;1:output_plane=960;2:output_plane=240;
  3,4,5,6:output_plane=60;7,8:output_plane=240;9,11:output_plane=960;
  12:output_plane=3840;13:output_plane=15360;16:output_plane=260;18:output_plane=65;
  default:output_plane=1;endcase end
 wire [14:0] output_offset=co[1:0]==0 ? 15'd0 : co[1:0]==1 ? output_plane : co[1:0]==2 ? output_plane<<1 : (output_plane<<1)+output_plane;


 // Each issued read carries its boundary, residual and final-tap tags. RAM
 // addresses are registered here; the RAM/mux contract is one further clock.
 reg [2:0] rgb_ci0,rgb_ci1;reg rv0,rv1,second0,second1,pad0,pad1,last0,last1;
 reg signed [7:0] rw0,rw1,first_weight,value_weight,product_weight,pipe_mac_weight;
 reg [15:0] residual_first;
 reg value_valid,value_last,product_valid,product_last,mac_valid,mac_last;
 reg [7:0] bypass_value;
 wire issue=state==TAP || state==SECOND_WAIT;
 wire final_tap=kx+1>=kernel && ky+1>=kernel && ci+1>=ci_count;
 task advance_dot;
  begin
   weight_pointer<=weight_pointer+1'b1;
   if(final_tap)state<=DOT_DRAIN;
   else begin
    state<=TAP;
    if(kx+1<kernel)kx<=kx+1'b1;
    else begin kx<=0;
     if(ky+1<kernel)ky<=ky+1'b1;
     else begin ky<=0;ci<=ci+1'b1;
      input_channel_base<=input_channel_base+input_plane;
      skip_channel_base<=skip_channel_base+skip_plane;
     end
    end
   end
  end
 endtask

 wire dot_active=state==TAP || state==SECOND_WAIT || state==DOT_DRAIN;
 wire [7:0] shared_input=state==CACHE_START ? gray_cached : input_q;
 wire signed [7:0] mac_weight=dot_active ? pipe_mac_weight : weight_cached;
 (* dsp_style="yes" *) wire signed [16:0] shared_mac=$signed({1'b0,shared_input})*mac_weight;
 wire [16:0] shared_quant_input=state==HEAD_INPUT ? {1'b0,refine_pixel[head_ci]} : input_value;
 wire [7:0] input_multiplier_addr=state==HEAD_INPUT ? L_HEAD : layer_id;
 wire [31:0] shared_quant_multiplier=input_multipliers[input_multiplier_addr];
 wire [7:0] channel_addr=state==CACHE_START ? B_DETAIL_0+cache_ch :
       state==HEAD_OUT || (state==OUTPUT_WAIT && stage==13 && co==3) ? B_HEAD : bias_base+co;
 wire signed [31:0] bias_value=biases[channel_addr];
 wire [31:0] multiplier_value=multipliers[channel_addr];
 (* dsp_style="yes" *) wire [48:0] shared_quant_product=shared_quant_input*shared_quant_multiplier;
 wire [48:0] input_rounded=(input_product+49'd536870912)>>30;

 // Four output channels share each refine input. Only 144 extra ROM bytes;
 // coefficients, quantization, operation order and all outputs are unchanged.
 reg signed [31:0] parallel_acc1,parallel_acc2,parallel_acc3;
 wire signed [7:0] pw1=$signed(packed_q[8+:8]);reg signed [7:0] pr1,pf1,pv1,pp1,pm1;
 (* dsp_style="yes" *) wire signed [16:0] parallel_mac1=$signed({1'b0,shared_input})*pm1;
 wire signed [7:0] pw2=$signed(packed_q[16+:8]);reg signed [7:0] pr2,pf2,pv2,pp2,pm2;
 (* dsp_style="yes" *) wire signed [16:0] parallel_mac2=$signed({1'b0,shared_input})*pm2;
 wire signed [7:0] pw3=$signed(packed_q[24+:8]);reg signed [7:0] pr3,pf3,pv3,pp3,pm3;
 (* dsp_style="yes" *) wire signed [16:0] parallel_mac3=$signed({1'b0,shared_input})*pm3;

 // GENERATED PIXEL LUTS BEGIN
function [7:0] expand5; input [4:0] v; begin case(v)
0:expand5=8'd0;
1:expand5=8'd8;
2:expand5=8'd16;
3:expand5=8'd24;
4:expand5=8'd32;
5:expand5=8'd41;
6:expand5=8'd49;
7:expand5=8'd57;
8:expand5=8'd65;
9:expand5=8'd74;
10:expand5=8'd82;
11:expand5=8'd90;
12:expand5=8'd98;
13:expand5=8'd106;
14:expand5=8'd115;
15:expand5=8'd123;
16:expand5=8'd131;
17:expand5=8'd139;
18:expand5=8'd148;
19:expand5=8'd156;
20:expand5=8'd164;
21:expand5=8'd172;
22:expand5=8'd180;
23:expand5=8'd189;
24:expand5=8'd197;
25:expand5=8'd205;
26:expand5=8'd213;
27:expand5=8'd222;
28:expand5=8'd230;
29:expand5=8'd238;
30:expand5=8'd246;
31:expand5=8'd255;
default:expand5=0;endcase end endfunction
function [7:0] expand6; input [5:0] v; begin case(v)
0:expand6=8'd0;
1:expand6=8'd4;
2:expand6=8'd8;
3:expand6=8'd12;
4:expand6=8'd16;
5:expand6=8'd20;
6:expand6=8'd24;
7:expand6=8'd28;
8:expand6=8'd32;
9:expand6=8'd36;
10:expand6=8'd40;
11:expand6=8'd44;
12:expand6=8'd48;
13:expand6=8'd52;
14:expand6=8'd56;
15:expand6=8'd60;
16:expand6=8'd64;
17:expand6=8'd68;
18:expand6=8'd72;
19:expand6=8'd76;
20:expand6=8'd80;
21:expand6=8'd85;
22:expand6=8'd89;
23:expand6=8'd93;
24:expand6=8'd97;
25:expand6=8'd101;
26:expand6=8'd105;
27:expand6=8'd109;
28:expand6=8'd113;
29:expand6=8'd117;
30:expand6=8'd121;
31:expand6=8'd125;
32:expand6=8'd129;
33:expand6=8'd133;
34:expand6=8'd137;
35:expand6=8'd141;
36:expand6=8'd145;
37:expand6=8'd149;
38:expand6=8'd153;
39:expand6=8'd157;
40:expand6=8'd161;
41:expand6=8'd165;
42:expand6=8'd170;
43:expand6=8'd174;
44:expand6=8'd178;
45:expand6=8'd182;
46:expand6=8'd186;
47:expand6=8'd190;
48:expand6=8'd194;
49:expand6=8'd198;
50:expand6=8'd202;
51:expand6=8'd206;
52:expand6=8'd210;
53:expand6=8'd214;
54:expand6=8'd218;
55:expand6=8'd222;
56:expand6=8'd226;
57:expand6=8'd230;
58:expand6=8'd234;
59:expand6=8'd238;
60:expand6=8'd242;
61:expand6=8'd246;
62:expand6=8'd250;
63:expand6=8'd255;
default:expand6=0;endcase end endfunction
function [10:0] gray_q10; input [7:0] v; begin case(v)
0:gray_q10=11'd0;
1:gray_q10=11'd4;
2:gray_q10=11'd8;
3:gray_q10=11'd12;
4:gray_q10=11'd16;
5:gray_q10=11'd20;
6:gray_q10=11'd24;
7:gray_q10=11'd28;
8:gray_q10=11'd32;
9:gray_q10=11'd36;
10:gray_q10=11'd40;
11:gray_q10=11'd44;
12:gray_q10=11'd48;
13:gray_q10=11'd52;
14:gray_q10=11'd56;
15:gray_q10=11'd60;
16:gray_q10=11'd64;
17:gray_q10=11'd68;
18:gray_q10=11'd72;
19:gray_q10=11'd76;
20:gray_q10=11'd80;
21:gray_q10=11'd84;
22:gray_q10=11'd88;
23:gray_q10=11'd92;
24:gray_q10=11'd96;
25:gray_q10=11'd100;
26:gray_q10=11'd104;
27:gray_q10=11'd108;
28:gray_q10=11'd112;
29:gray_q10=11'd116;
30:gray_q10=11'd120;
31:gray_q10=11'd124;
32:gray_q10=11'd129;
33:gray_q10=11'd133;
34:gray_q10=11'd137;
35:gray_q10=11'd141;
36:gray_q10=11'd145;
37:gray_q10=11'd149;
38:gray_q10=11'd153;
39:gray_q10=11'd157;
40:gray_q10=11'd161;
41:gray_q10=11'd165;
42:gray_q10=11'd169;
43:gray_q10=11'd173;
44:gray_q10=11'd177;
45:gray_q10=11'd181;
46:gray_q10=11'd185;
47:gray_q10=11'd189;
48:gray_q10=11'd193;
49:gray_q10=11'd197;
50:gray_q10=11'd201;
51:gray_q10=11'd205;
52:gray_q10=11'd209;
53:gray_q10=11'd213;
54:gray_q10=11'd217;
55:gray_q10=11'd221;
56:gray_q10=11'd225;
57:gray_q10=11'd229;
58:gray_q10=11'd233;
59:gray_q10=11'd237;
60:gray_q10=11'd241;
61:gray_q10=11'd245;
62:gray_q10=11'd249;
63:gray_q10=11'd253;
64:gray_q10=11'd257;
65:gray_q10=11'd261;
66:gray_q10=11'd265;
67:gray_q10=11'd269;
68:gray_q10=11'd273;
69:gray_q10=11'd277;
70:gray_q10=11'd281;
71:gray_q10=11'd285;
72:gray_q10=11'd289;
73:gray_q10=11'd293;
74:gray_q10=11'd297;
75:gray_q10=11'd301;
76:gray_q10=11'd305;
77:gray_q10=11'd309;
78:gray_q10=11'd313;
79:gray_q10=11'd317;
80:gray_q10=11'd321;
81:gray_q10=11'd325;
82:gray_q10=11'd329;
83:gray_q10=11'd333;
84:gray_q10=11'd337;
85:gray_q10=11'd341;
86:gray_q10=11'd345;
87:gray_q10=11'd349;
88:gray_q10=11'd353;
89:gray_q10=11'd357;
90:gray_q10=11'd361;
91:gray_q10=11'd365;
92:gray_q10=11'd369;
93:gray_q10=11'd373;
94:gray_q10=11'd377;
95:gray_q10=11'd381;
96:gray_q10=11'd386;
97:gray_q10=11'd390;
98:gray_q10=11'd394;
99:gray_q10=11'd398;
100:gray_q10=11'd402;
101:gray_q10=11'd406;
102:gray_q10=11'd410;
103:gray_q10=11'd414;
104:gray_q10=11'd418;
105:gray_q10=11'd422;
106:gray_q10=11'd426;
107:gray_q10=11'd430;
108:gray_q10=11'd434;
109:gray_q10=11'd438;
110:gray_q10=11'd442;
111:gray_q10=11'd446;
112:gray_q10=11'd450;
113:gray_q10=11'd454;
114:gray_q10=11'd458;
115:gray_q10=11'd462;
116:gray_q10=11'd466;
117:gray_q10=11'd470;
118:gray_q10=11'd474;
119:gray_q10=11'd478;
120:gray_q10=11'd482;
121:gray_q10=11'd486;
122:gray_q10=11'd490;
123:gray_q10=11'd494;
124:gray_q10=11'd498;
125:gray_q10=11'd502;
126:gray_q10=11'd506;
127:gray_q10=11'd510;
128:gray_q10=11'd514;
129:gray_q10=11'd518;
130:gray_q10=11'd522;
131:gray_q10=11'd526;
132:gray_q10=11'd530;
133:gray_q10=11'd534;
134:gray_q10=11'd538;
135:gray_q10=11'd542;
136:gray_q10=11'd546;
137:gray_q10=11'd550;
138:gray_q10=11'd554;
139:gray_q10=11'd558;
140:gray_q10=11'd562;
141:gray_q10=11'd566;
142:gray_q10=11'd570;
143:gray_q10=11'd574;
144:gray_q10=11'd578;
145:gray_q10=11'd582;
146:gray_q10=11'd586;
147:gray_q10=11'd590;
148:gray_q10=11'd594;
149:gray_q10=11'd598;
150:gray_q10=11'd602;
151:gray_q10=11'd606;
152:gray_q10=11'd610;
153:gray_q10=11'd614;
154:gray_q10=11'd618;
155:gray_q10=11'd622;
156:gray_q10=11'd626;
157:gray_q10=11'd630;
158:gray_q10=11'd634;
159:gray_q10=11'd638;
160:gray_q10=11'd643;
161:gray_q10=11'd647;
162:gray_q10=11'd651;
163:gray_q10=11'd655;
164:gray_q10=11'd659;
165:gray_q10=11'd663;
166:gray_q10=11'd667;
167:gray_q10=11'd671;
168:gray_q10=11'd675;
169:gray_q10=11'd679;
170:gray_q10=11'd683;
171:gray_q10=11'd687;
172:gray_q10=11'd691;
173:gray_q10=11'd695;
174:gray_q10=11'd699;
175:gray_q10=11'd703;
176:gray_q10=11'd707;
177:gray_q10=11'd711;
178:gray_q10=11'd715;
179:gray_q10=11'd719;
180:gray_q10=11'd723;
181:gray_q10=11'd727;
182:gray_q10=11'd731;
183:gray_q10=11'd735;
184:gray_q10=11'd739;
185:gray_q10=11'd743;
186:gray_q10=11'd747;
187:gray_q10=11'd751;
188:gray_q10=11'd755;
189:gray_q10=11'd759;
190:gray_q10=11'd763;
191:gray_q10=11'd767;
192:gray_q10=11'd771;
193:gray_q10=11'd775;
194:gray_q10=11'd779;
195:gray_q10=11'd783;
196:gray_q10=11'd787;
197:gray_q10=11'd791;
198:gray_q10=11'd795;
199:gray_q10=11'd799;
200:gray_q10=11'd803;
201:gray_q10=11'd807;
202:gray_q10=11'd811;
203:gray_q10=11'd815;
204:gray_q10=11'd819;
205:gray_q10=11'd823;
206:gray_q10=11'd827;
207:gray_q10=11'd831;
208:gray_q10=11'd835;
209:gray_q10=11'd839;
210:gray_q10=11'd843;
211:gray_q10=11'd847;
212:gray_q10=11'd851;
213:gray_q10=11'd855;
214:gray_q10=11'd859;
215:gray_q10=11'd863;
216:gray_q10=11'd867;
217:gray_q10=11'd871;
218:gray_q10=11'd875;
219:gray_q10=11'd879;
220:gray_q10=11'd883;
221:gray_q10=11'd887;
222:gray_q10=11'd891;
223:gray_q10=11'd895;
224:gray_q10=11'd900;
225:gray_q10=11'd904;
226:gray_q10=11'd908;
227:gray_q10=11'd912;
228:gray_q10=11'd916;
229:gray_q10=11'd920;
230:gray_q10=11'd924;
231:gray_q10=11'd928;
232:gray_q10=11'd932;
233:gray_q10=11'd936;
234:gray_q10=11'd940;
235:gray_q10=11'd944;
236:gray_q10=11'd948;
237:gray_q10=11'd952;
238:gray_q10=11'd956;
239:gray_q10=11'd960;
240:gray_q10=11'd964;
241:gray_q10=11'd968;
242:gray_q10=11'd972;
243:gray_q10=11'd976;
244:gray_q10=11'd980;
245:gray_q10=11'd984;
246:gray_q10=11'd988;
247:gray_q10=11'd992;
248:gray_q10=11'd996;
249:gray_q10=11'd1000;
250:gray_q10=11'd1004;
251:gray_q10=11'd1008;
252:gray_q10=11'd1012;
253:gray_q10=11'd1016;
254:gray_q10=11'd1020;
255:gray_q10=11'd1024;
default:gray_q10=0;endcase end endfunction
 // GENERATED PIXEL LUTS END
 function [7:0] rgb_gray;
  input [15:0] rgb;
  reg [7:0] r,g,b;
  reg [17:0] sum;
  begin r=expand5(rgb[15:11]);g=expand6(rgb[10:5]);b=expand5(rgb[4:0]);
   sum=77*r+150*g+29*b+128;rgb_gray=sum>>8;end
 endfunction
 assign gray_pixel=gray_padding ? 8'd0 : rgb_gray(read_data);
 reg [12:0] copy_index;
 reg [7:0] bx,by,box_neighbor;
 reg [3:0] box_count;
 reg box_raw;
 reg [14:0] clean_area,raw_area;
 reg [7:0] rx0,ry0,rx1,ry1,cx0,cy0,cx1,cy1;
 wire signed [10:0] box_sx=$signed({3'd0,bx})+$signed({9'd0,box_neighbor%3})-1;
 wire signed [10:0] box_sy=$signed({3'd0,by})+$signed({9'd0,box_neighbor/3})-1;
 reg [31:0] pool_value;
 cnn_roi_select roi_selector(.clk(clk),.rst_n(rst_n),.start(state==BOX_FINISH && !select_busy && !select_done),
  .use_clean(clean_area>=8),.mask_wr(state==BOX_EVAL),.mask_addr(by*160+bx),
  .mask_data({(box_count>=4),box_raw}),.busy(select_busy),.done(select_done),
  .x0(select_x0),.y0(select_y0),.x1(select_x1),.y1(select_y1),
  .queue_rd_addr(select_queue_ra),.queue_wr_addr(select_queue_wa),.queue_rd_data(select_queue_q),
  .queue_wr(select_queue_wr),.queue_wr_data(select_queue_data));
 wire [7:0] pool_kernel=stage==17 ? 2 : 4;
 wire [7:0] pool_iw=stage==17 ? 16 : 8;
 wire [7:0] pool_ow=stage==17 ? 8 : 2;
 wire [7:0] pool_co=stage==17 ? 24 : 24;
 wire [14:0] pool_src=stage==17 ? 0 : 0;
 wire [14:0] pool_dst=stage==17 ? 4100 : 1700;
 wire signed [31:0] sigmoid_index=((rq_result+32'sd4) >>> 3)+32'sd2048;
 integer j;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   rgb_ci0<=0;rgb_ci1<=0;rv0<=0;rv1<=0;second0<=0;second1<=0;pad0<=0;pad1<=0;last0<=0;last1<=0;
   rw0<=0;rw1<=0;first_weight<=0;value_weight<=0;product_weight<=0;pipe_mac_weight<=0;
   residual_first<=0;value_valid<=0;value_last<=0;product_valid<=0;product_last<=0;mac_valid<=0;mac_last<=0;bypass_value<=0;
   parallel_acc1<=0;parallel_acc2<=0;parallel_acc3<=0;pr1<=0;pf1<=0;pv1<=0;pp1<=0;pm1<=0;pr2<=0;pf2<=0;pv2<=0;pp2<=0;pm2<=0;pr3<=0;pf3<=0;pv3<=0;pp3<=0;pm3<=0;
   norm_start<=0;norm_index<=0;norm_rgb<=0;state<=IDLE;stage<=0;busy<=0;nn_done<=0;predicted_shape<=0;identity_quality<=0;inference_cycles<=0;
   core_rd_bank<=0;core_rd_addr<=0;core_wr<=0;core_wr_bank<=0;core_wr_addr<=0;core_wr_data<=0;
   rq_start<=0;rq_value<=0;rq_multiplier<=0;rq_relu<=1;roi_start<=0;
   co<=0;ci<=0;ox<=0;oy<=0;kx<=0;ky<=0;accumulator<=0;best_logit<=32'sh80000000;second_logit<=32'sh80000000;unknown_logit<=0;best_shape<=0;
   input_value<=0;first_value<=0;input_product<=0;input_q<=0;gray_padding<=0;copy_index<=0;head_ci<=0;
   weight_pointer<=0;channel_weight_base<=0;input_channel_base<=0;skip_channel_base<=0;output_address<=0;
   gray_cached<=0;cache_row<=0;cache_x<=0;cache_ch<=0;cache_slot<=0;cache_prefill<=0;cache_tag0<=127;cache_tag1<=127;cache_tag2<=127;
   box_x0<=0;box_x1<=160;box_y0<=3;box_y1<=93;bx<=0;by<=3;box_neighbor<=0;box_count<=0;box_raw<=0;
   clean_area<=0;raw_area<=0;rx0<=160;rx1<=0;ry0<=96;ry1<=0;cx0<=160;cx1<=0;cy0<=96;cy1<=0;pool_value<=0;
   for(j=0;j<4;j=j+1)refine_pixel[j]<=0;
  end else begin
   norm_start<=0;core_wr<=0;rq_start<=0;roi_start<=0;nn_done<=0;
   if(busy && state!=FINISHED)inference_cycles<=inference_cycles+1'b1;
   if(dot_active)begin
    rv0<=issue;rv1<=rv0;rgb_ci0<=ci[2:0];rgb_ci1<=rgb_ci0;
    second0<=state==SECOND_WAIT;second1<=second0;
    pad0<=!within_bounds || (src_bank==2 && (sy<3 || sy>=93));pad1<=pad0;
    last0<=final_tap;last1<=last0;
    // Use the existing single registered ROM port; a second syntactic read
    // would replicate all 25,308 weight bytes as distributed ROM in TD.
    rw1<=weight_cached;pr1<=pw1;pr2<=pw2;pr3<=pw3;
    value_valid<=0;
    if(rv1)begin
     if(!pad1 && (mode==1 || mode==2) && !second1)begin
      residual_first<=read_data;first_weight<=rw1;pf1<=pr1;pf2<=pr2;pf3<=pr3;
     end else begin
      value_valid<=1;value_last<=last1;
      value_weight<=second1 ? first_weight : rw1;pv1<=second1 ? pf1 : pr1;pv2<=second1 ? pf2 : pr2;pv3<=second1 ? pf3 : pr3;
      if(pad1)input_value<=0;
      else if(second1)input_value<={1'b0,residual_first}+{1'b0,read_data};
      else if(src_bank==2)input_value<=gray_q10(stage==0 ? ((rgb_ci1==0 || rgb_ci1==3) ? expand5(read_data[15:11]) : (rgb_ci1==1 || rgb_ci1==4) ? expand6(read_data[10:5]) : expand5(read_data[4:0])) : rgb_gray(read_data));
      else input_value<={1'b0,read_data};
     end
    end
    product_valid<=value_valid;product_last<=value_last;product_weight<=value_weight;pp1<=pv1;pp2<=pv2;pp3<=pv3;
    if(value_valid)begin input_product<=shared_quant_product;bypass_value<=input_value[7:0];end
    mac_valid<=product_valid;mac_last<=product_last;pipe_mac_weight<=product_weight;pm1<=pp1;pm2<=pp2;pm3<=pp3;
    if(product_valid)input_q<=src_bank==4 ? bypass_value : input_rounded>255 ? 8'd255 : input_rounded[7:0];
    if(mac_valid)begin
     accumulator<=accumulator+shared_mac;
     begin parallel_acc1<=parallel_acc1+parallel_mac1;parallel_acc2<=parallel_acc2+parallel_mac2;parallel_acc3<=parallel_acc3+parallel_mac3;end
     if(mac_last)state<=OUTPUT_START;
    end
   end else begin rv0<=0;rv1<=0;value_valid<=0;product_valid<=0;mac_valid<=0;end
   case(state)
    IDLE:if(start)begin identity_quality<=0;busy<=1;inference_cycles<=0;stage<=0;norm_index<=0;state<=NORM_READ;end
    NORM_READ:begin core_rd_bank<=2;core_rd_addr<=norm_index;state<=NORM_WAIT;end
    NORM_WAIT:state<=NORM_USE;
    NORM_USE:begin norm_rgb<=read_data;state<=NORM_START;end
    NORM_START:begin norm_start<=1;state<=NORM_DONE;end
    NORM_DONE:if(norm_done)begin core_wr<=1;core_wr_bank<=0;core_wr_addr<=norm_index;core_wr_data<=norm_value;state<=NORM_WRITE;end
    NORM_WRITE:if(norm_index==14399)state<=CONFIG;else begin norm_index<=norm_index+1'b1;state<=NORM_READ;end
    CONFIG:begin
     co<=0;ci<=0;ox<=0;oy<=0;kx<=0;ky<=0;
     channel_weight_base<=weight_base;output_address<=stage==17 ? 4100 : stage==19 ? 1700 : dst_base;
     case(stage)
      10:begin copy_index<=0;state<=COPY_READ;end
      13:begin cache_row<=0;cache_x<=0;cache_ch<=0;cache_slot<=0;cache_prefill<=1;state<=CACHE_READ;end
      14:begin bx<=0;by<=3;box_neighbor<=0;box_count<=0;box_raw<=0;clean_area<=0;raw_area<=0;
       rx0<=160;rx1<=0;ry0<=96;ry1<=0;cx0<=160;cx1<=0;cy0<=96;cy1<=0;state<=BOX_TAP;end
      15:state<=ROI_START;
      17,19:begin pool_value<=0;state<=POOL_READ;end
      default:begin state<=PIXEL;if(stage==20)begin best_logit<=32'sh80000000;second_logit<=32'sh80000000;unknown_logit<=0;best_shape<=0;end end
     endcase
    end
    PIXEL:begin parallel_acc1<=parallel_bias[0+:32];parallel_acc2<=parallel_bias[32+:32];parallel_acc3<=parallel_bias[64+:32]; accumulator<=bias_value;ci<=0;kx<=0;ky<=0;weight_pointer<=channel_weight_base;
     input_channel_base<=src_base;skip_channel_base<=skip_base;state<=TAP;end
    TAP:begin
     if(src_bank==2)begin core_rd_bank<=stage==0 && ci>=3 ? 0 : 2;core_rd_addr<=sy<3 || sy>=93 ? 0 : (sy-3)*160+sx;end
     else if(mode==1 || mode==2)begin core_rd_bank<=src_bank;core_rd_addr<=input_channel_base+source_row_offset+sx[8:1];end
     else begin core_rd_bank<=src_bank;core_rd_addr<=input_channel_base+source_row_offset+sx;end
     if(within_bounds && (mode==1 || mode==2))state<=SECOND_WAIT;
     else advance_dot;
    end
    SECOND_WAIT:begin
     if(mode==1)begin core_rd_bank<=skip_bank;core_rd_addr<=skip_channel_base+skip_row_offset+sx;end
     else begin core_rd_bank<=0;core_rd_addr<=detail_address;end
     advance_dot;
    end
    DOT_DRAIN:begin end
    OUTPUT_START:begin rq_value<=co[1:0]==0 ? accumulator : co[1:0]==1 ? parallel_acc1 : co[1:0]==2 ? parallel_acc2 : parallel_acc3;rq_multiplier<=multiplier_value;rq_relu<=stage!=20;rq_start<=1;state<=OUTPUT_WAIT;end
    OUTPUT_WAIT:if(rq_done)begin
     if(stage==13)begin
      refine_pixel[co]<=rq_result[15:0];
      if(co==3)begin head_ci<=0;accumulator<=bias_value;state<=HEAD_INPUT;end
      else begin co<=co+1'b1;state<=OUTPUT_START;end
     end else begin
      if(stage==20)begin
       if(co==0)unknown_logit<=rq_result;
       if(rq_result>best_logit)begin second_logit<=best_logit;best_logit<=rq_result;best_shape<=co[1:0];end
       else if(rq_result>second_logit)second_logit<=rq_result;
      end else begin core_wr<=1;core_wr_bank<=dst_bank;core_wr_addr<=output_address+output_offset;if(co[1:0]==3)output_address<=output_address+1'b1;core_wr_data<=rq_result[15:0];end
      if(co[1:0]!=3 && co+1<co_count)begin co<=co+1'b1;state<=OUTPUT_START;end
      else if(ox+1<ow)begin ox<=ox+1'b1;co<={co[7:2],2'b0};state<=PIXEL;end
      else begin ox<=0;if(oy+1<oh)begin oy<=oy+1'b1;co<={co[7:2],2'b0};state<=PIXEL;end
       else begin oy<=0;if(co+1<co_count)begin
        co<=co+1'b1;channel_weight_base<=channel_weight_base+(kernel_volume<<2);
        output_address<=output_address+3*output_plane+(stage==16 ? 5 : stage==18 ? 2 : 1);state<=PIXEL;end
        else if(stage==20)begin predicted_shape<=shape_accepted ? final_shape : 2'd0;
         identity_quality<=!shape_accepted ? 2'd0 : shape_margin>=$signed(FAST_MARGIN_Q10) && unknown_margin>=$signed(FAST_MARGIN_Q10) ? 2'd2 :
          shape_margin>=$signed(ACQUIRE_SHAPE_MARGIN_Q10) && unknown_margin>=$signed(ACQUIRE_UNKNOWN_MARGIN_Q10) ? 2'd1 : 2'd0;
         nn_done<=1;state<=FINISHED;end
        else begin stage<=stage+1'b1;state<=CONFIG;end
       end
      end
     end
    end
    HEAD_INPUT:begin input_product<=shared_quant_product;state<=HEAD_QUANT;end
    HEAD_QUANT:begin input_q<=input_rounded>255 ? 8'd255 : input_rounded[7:0];state<=HEAD_MAC;end
    HEAD_MAC:begin
     accumulator<=accumulator+shared_mac;
     if(head_ci==3)state<=HEAD_OUT;else begin head_ci<=head_ci+1'b1;state<=HEAD_INPUT;end
    end
    HEAD_NEXT:if(head_ci==3)state<=HEAD_OUT;else begin head_ci<=head_ci+1'b1;state<=HEAD_INPUT;end
    HEAD_OUT:begin rq_start<=1;rq_value<=accumulator;rq_multiplier<=multiplier_value;rq_relu<=0;state<=HEAD_WAIT;end
    HEAD_WAIT:if(rq_done)begin
     core_wr<=1;core_wr_bank<=3;core_wr_addr<=oy*160+ox;
     core_wr_data<={8'd0,sigmoid[sigmoid_index<0 ? 0 : sigmoid_index>4096 ? 4096 : sigmoid_index]};
     co<=0;
     channel_weight_base<=weight_base;
     if(ox==159)begin ox<=0;if(oy==95)begin stage<=14;state<=CONFIG;end
      else begin
       oy<=oy+1'b1;
       if(oy<94)begin cache_row<=oy+2;cache_x<=0;cache_ch<=0;cache_slot<=cache_slot==2 ? 0 : cache_slot+1'b1;state<=CACHE_READ;end
       else state<=PIXEL;
      end
     end
     else begin ox<=ox+1'b1;state<=PIXEL;end
    end
    COPY_READ:begin core_rd_bank<=0;core_rd_addr<=9600+copy_index;state<=COPY_WAIT;end
    COPY_WAIT:state<=COPY_WRITE;
    COPY_WRITE:begin
     core_wr<=1;core_wr_bank<=0;core_wr_addr<=3840+copy_index;core_wr_data<=read_data;
     if(copy_index==7679)begin stage<=11;state<=CONFIG;end else begin copy_index<=copy_index+1'b1;state<=COPY_READ;end
    end
    BOX_TAP:begin
     if(box_sx<0 || box_sx>=160 || box_sy<3 || box_sy>=93)state<=BOX_STEP;
     else begin core_rd_bank<=3;core_rd_addr<=box_sy*160+box_sx;state<=BOX_WAIT;end
    end
    BOX_WAIT:state<=BOX_USE;
    BOX_USE:begin
     if(read_data[7:0]>=102)begin box_count<=box_count+1'b1;if(box_neighbor==4)box_raw<=1;end
     state<=BOX_STEP;
    end
    BOX_STEP:if(box_neighbor==8)state<=BOX_EVAL;else begin box_neighbor<=box_neighbor+1'b1;state<=BOX_TAP;end
    BOX_EVAL:begin
     if(box_raw)begin raw_area<=raw_area+1'b1;if(bx<rx0)rx0<=bx;if(by<ry0)ry0<=by;if(bx+1>rx1)rx1<=bx+1'b1;if(by+1>ry1)ry1<=by+1'b1;end
     if(box_count>=4)begin clean_area<=clean_area+1'b1;if(bx<cx0)cx0<=bx;if(by<cy0)cy0<=by;if(bx+1>cx1)cx1<=bx+1'b1;if(by+1>cy1)cy1<=by+1'b1;end
     box_count<=0;box_raw<=0;box_neighbor<=0;
     if(bx==159)begin bx<=0;if(by==92)state<=BOX_FINISH;else begin by<=by+1'b1;state<=BOX_TAP;end end
     else begin bx<=bx+1'b1;state<=BOX_TAP;end
    end
    BOX_FINISH:if(select_done)begin
     box_x0<=select_x0;box_y0<=select_y0;box_x1<=select_x1;box_y1<=select_y1;
     stage<=15;state<=CONFIG;
    end
    ROI_START:begin roi_start<=1;state<=ROI_WAIT;end
    ROI_WAIT:if(roi_done)begin stage<=16;state<=CONFIG;end
    POOL_READ:begin
     core_rd_bank<=0;
     core_rd_addr<=stage==17 ? 11520+(co<<8)+(co<<2)+((oy*2+ky)<<4)+ox*2+kx : (co<<6)+co+((oy*4+ky)<<3)+ox*4+kx;
     state<=POOL_WAIT;
    end
    POOL_WAIT:state<=POOL_USE;
    POOL_USE:begin
     if(stage==17)begin if(read_data>pool_value)pool_value<=read_data;end else pool_value<=pool_value+read_data;
     state<=POOL_STEP;
    end
    POOL_STEP:begin
     if(kx+1<pool_kernel)begin kx<=kx+1'b1;state<=POOL_READ;end
     else begin kx<=0;if(ky+1<pool_kernel)begin ky<=ky+1'b1;state<=POOL_READ;end else begin ky<=0;state<=POOL_WRITE;end end
    end
    POOL_WRITE:begin
     core_wr<=1;core_wr_bank<=0;core_wr_addr<=output_address;output_address<=output_address+1'b1;
     core_wr_data<=stage==17 ? pool_value[15:0] : (pool_value+8)>>4;pool_value<=0;
     if(ox+1<pool_ow)begin ox<=ox+1'b1;state<=POOL_READ;end
     else begin ox<=0;if(oy+1<pool_ow)begin oy<=oy+1'b1;state<=POOL_READ;end
      else begin oy<=0;if(co+1<pool_co)begin co<=co+1'b1;state<=POOL_READ;end else begin stage<=stage+1'b1;state<=CONFIG;end end
     end
    end
    FINISHED:if(release_frame)begin busy<=0;state<=IDLE;end
    GRAY_NORMALIZE:begin input_value<=gray_q10(gray_cached);state<=INPUT_PRODUCT;end
    CACHE_READ:begin core_rd_bank<=2;gray_padding<=cache_row<3 || cache_row>=93;
     core_rd_addr<=cache_row<3 || cache_row>=93 ? 0 : (cache_row-3)*160+cache_x;state<=CACHE_WAIT;end
    CACHE_WAIT:state<=CACHE_GRAY;
    CACHE_GRAY:begin gray_cached<=gray_pixel;state<=CACHE_WEIGHT;end
    CACHE_WEIGHT:state<=CACHE_START;
    CACHE_START:begin rq_value<=bias_value+shared_mac;rq_multiplier<=multiplier_value;rq_relu<=1;rq_start<=1;state<=CACHE_RQWAIT;end
    CACHE_RQWAIT:if(rq_done)begin
     core_wr<=1;core_wr_bank<=0;core_wr_addr<=detail_write_address;core_wr_data<=rq_result[15:0];
     if(cache_ch<3)begin cache_ch<=cache_ch+1'b1;state<=CACHE_WEIGHT;end
     else begin
      cache_ch<=0;
      if(cache_x<159)begin cache_x<=cache_x+1'b1;state<=CACHE_READ;end
      else begin
       if(cache_slot==0)cache_tag0<=cache_row;else if(cache_slot==1)cache_tag1<=cache_row;else cache_tag2<=cache_row;
       if(cache_prefill && cache_row==0)begin cache_row<=1;cache_slot<=1;cache_x<=0;state<=CACHE_READ;end
       else begin cache_prefill<=0;state<=PIXEL;end
      end
     end
    end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
