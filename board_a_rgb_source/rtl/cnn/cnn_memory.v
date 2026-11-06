// Explicit vendor RAM mapping; simulation uses the same one-cycle synchronous
// read contract. No memory array is reset, which would prevent ERAM inference.
module cnn_block_ram #(
 parameter WIDTH=16,AW=15,DEPTH=26880
)(
 input wire clk,we,
 input wire [AW-1:0] wa,ra,
 input wire [WIDTH-1:0] wd,
 output wire [WIDTH-1:0] rd
);
 // TD does not define SYNTHESIS automatically. Behavioral RAM is selected
 // only by the explicit simulator flag +define+CNN_SIM.
 `ifndef CNN_SIM
 PH1P_LOGIC_ERAM #(
  .DATA_WIDTH_A(WIDTH),.DATA_WIDTH_B(WIDTH),.ADDR_WIDTH_A(AW),.ADDR_WIDTH_B(AW),
  .DATA_DEPTH_A(DEPTH),.DATA_DEPTH_B(DEPTH),.MODE("DP"),
  .REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("READBEFOREWRITE"),
  .IMPLEMENT("20K"),.CLKMODE("SYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE")
 )mem(.clka(clk),.clkb(clk),.cea(1'b1),.ceb(1'b1),.ocea(1'b0),.oceb(1'b0),
  .wea(we),.web(1'b0),.addra(wa),.addrb(ra),.dia(wd),.dib({WIDTH{1'b0}}),
  .bea(1'b0),.beb(1'b0),.rsta(1'b0),.rstb(1'b0),.doa(),.dob(rd),
  .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
 `else
 reg [WIDTH-1:0] mem[0:DEPTH-1];
 reg [WIDTH-1:0] q;
 always @(posedge clk)begin
  if(we && wa<DEPTH)mem[wa]<=wd;
  q<=ra<DEPTH ? mem[ra] : {WIDTH{1'b0}};
 end
 assign rd=q;
 `endif
endmodule

// Dense 80-bit PDPW rows. Writers use aligned contiguous tensor/queue runs;
// a one-row register preserves earlier lanes while appending the next word.
// Random visited flags live in the postprocessor's separate LUT bitmap.
module cnn_packed_ram #(
 parameter WIDTH=16,AW=15,DEPTH=26880,
 parameter LANES=80/WIDTH,ROWS=(DEPTH+LANES-1)/LANES,ROW_AW=$clog2(ROWS)
)(
 input wire clk,we,
 input wire [AW-1:0] wa,ra,
 input wire [WIDTH-1:0] wd,
 output wire [WIDTH-1:0] rd
);
 reg [3:0] cache_valid=0;reg forwarding=0;
 reg [ROW_AW-1:0] cache_row0=0,cache_row1=0,cache_row2=0,cache_row3=0;
 reg [79:0] cache_word0=0,cache_word1=0,cache_word2=0,cache_word3=0,forward_word=0;
 reg [1:0] victim=0;
 reg [3:0] read_slot=0;
 wire [ROW_AW-1:0] write_row=wa/LANES,read_row=ra/LANES;
 wire [3:0] write_slot=wa%LANES;
 wire wh0=cache_valid[0] && cache_row0==write_row,wh1=cache_valid[1] && cache_row1==write_row;
 wire wh2=cache_valid[2] && cache_row2==write_row,wh3=cache_valid[3] && cache_row3==write_row;
 wire write_hit=wh0 || wh1 || wh2 || wh3;
 wire [1:0] write_way=wh0 ? 0 : wh1 ? 1 : wh2 ? 2 : wh3 ? 3 : victim;
 wire [79:0] base_word=wh0 ? cache_word0 : wh1 ? cache_word1 : wh2 ? cache_word2 : wh3 ? cache_word3 : 80'd0;
 wire [79:0] slot_mask={{(80-WIDTH){1'b0}},{WIDTH{1'b1}}} << (write_slot*WIDTH);
 wire [79:0] shifted_data={{(80-WIDTH){1'b0}},wd} << (write_slot*WIDTH);
 wire [79:0] write_word=(base_word & ~slot_mask)|shifted_data;
 wire rh0=cache_valid[0] && cache_row0==read_row,rh1=cache_valid[1] && cache_row1==read_row;
 wire rh2=cache_valid[2] && cache_row2==read_row,rh3=cache_valid[3] && cache_row3==read_row;
 wire [79:0] read_word;
 `ifndef CNN_SIM
 PH1P_LOGIC_ERAM #(
  .DATA_WIDTH_A(80),.DATA_WIDTH_B(80),.ADDR_WIDTH_A(ROW_AW),.ADDR_WIDTH_B(ROW_AW),
  .DATA_DEPTH_A(ROWS),.DATA_DEPTH_B(ROWS),.MODE("PDPW"),
  .REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
  .IMPLEMENT("20K"),.CLKMODE("SYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE")
 )storage(.clka(clk),.clkb(clk),.cea(1'b1),.ceb(1'b1),.ocea(1'b0),.oceb(1'b0),
  .wea(we),.web(1'b0),.addra(write_row),.addrb(read_row),.dia(write_word),.dib(80'd0),
  .bea(1'b0),.beb(1'b0),.rsta(1'b0),.rstb(1'b0),.doa(),.dob(read_word),
  .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
 `else
 reg [79:0] mem[0:ROWS-1];reg [79:0] qb;
 always @(posedge clk)begin
  if(we)mem[write_row]<=write_word;
  qb<=read_row<ROWS ? mem[read_row] : 80'd0;
 end
 assign read_word=qb;
 `endif
 wire [79:0] selected_word=forwarding ? forward_word : read_word;
 assign rd=selected_word[read_slot*WIDTH+:WIDTH];
 always @(posedge clk)begin
  read_slot<=ra%LANES;
  forwarding<=rh0 || rh1 || rh2 || rh3;
  forward_word<=rh0 ? cache_word0 : rh1 ? cache_word1 : rh2 ? cache_word2 : cache_word3;
  if(we)begin
   if(!write_hit)victim<=victim+1'b1;
   case(write_way)
    0:begin cache_valid[0]<=1;cache_row0<=write_row;cache_word0<=write_word;end
    1:begin cache_valid[1]<=1;cache_row1<=write_row;cache_word1<=write_word;end
    2:begin cache_valid[2]<=1;cache_row2<=write_row;cache_word2<=write_word;end
    3:begin cache_valid[3]<=1;cache_row3<=write_row;cache_word3<=write_word;end
   endcase
  end
 end
endmodule

module cnn_distributed_ram #(
 parameter WIDTH=16,AW=14,DEPTH=15360
)(
 input wire clk,we,
 input wire [AW-1:0] wa,ra,
 input wire [WIDTH-1:0] wd,
 output reg [WIDTH-1:0] rd
);
 reg [WIDTH-1:0] mem[0:DEPTH-1] /* synthesis ram_style=dram */;
 always @(posedge clk)begin
  if(we)mem[wa]<=wd;
  rd<=mem[ra];
 end
endmodule
