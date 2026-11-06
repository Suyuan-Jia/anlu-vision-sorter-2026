// Bounded streaming connected components over four-pixel horizontal beats.
// All raster beats visit the line RAM, including zeros outside the search ROI.
// Direct parent links are updated on union, so lookup is one table read. Overflow
// drops new components; it cannot merge an unrelated component into a target.
(* dsp_style="no" *) module cnn_component_select #(
 parameter WIDTH=1280,HEIGHT=720
)(
 input wire clk,rst_n,clear,frame_end,take,hit,
 input wire [10:0] x,y,
 input wire acquiring,
 input wire [10:0] ref_x0,ref_y0,ref_x1,ref_y1,
 input wire [15:0] seed_area,
 output reg done,found,
 output reg [10:0] xmin,ymin,xmax,ymax,
 output reg [19:0] count,
 output reg overflow
);
 localparam C=8;
 reg [3:0] line_label[0:WIDTH/4-1] /* synthesis ram_style=dram */;
 reg p_take,p_hit,end_d;
 reg [10:0] px,py;
 reg [3:0] above_q;
 // Register the previous-row label read before the parent/merge logic.
 // A full row separates read/write hazards; the left label remains live at
 // processing cadence, so adjacent input beats still merge without stalls.
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin p_take<=0;p_hit<=0;end_d<=0;px<=0;py<=0;above_q<=0;end
  else begin
   p_take<=take && !clear;end_d<=frame_end && !clear;
   if(take)begin p_hit<=hit;px<=x;py<=y;above_q<=y==0 ? 4'd0 : line_label[x>>2];end
  end
 end
 reg [3:0] parent[0:C];
 reg [1:0] rank[0:C];
 reg [19:0] area[0:C];
 reg [10:0] xlo[0:C],ylo[0:C],xhi[0:C],yhi[0:C];
 reg previous_hit,run_active;
 reg [10:0] previous_x,previous_y;
 reg [3:0] left_label,previous_above;
 // A merged alias can be reused once no tag in the completed line refers
 // to it. Tracking written tags (not just roots) protects the line RAM from
 // alias reuse while any old tag is still live. Roots retain their statistics
 // through frame end, including completed disconnected components.
 reg [C:1] available,row_tags;
 function [3:0] first_available;
  input [C:1] slots;integer k;begin
   first_available=C+1;
   for(k=C;k>=1;k=k-1)if(slots[k])first_available=k;
  end
 endfunction
 wire [3:0] next_label=first_available(available);
 function [3:0] root;
  input [3:0] a;begin root=parent[a];end
 endfunction
 wire adjacent=previous_hit && previous_y==py && previous_x+4==px;
 wire [3:0] above=above_q;
 wire [3:0] up=root(above);
 wire [3:0] left=adjacent ? root(left_label) : 4'd0;
 wire [3:0] first_up=adjacent && !run_active ? root(previous_above) : 4'd0;
 wire [3:0] a=up!=0 ? up : first_up;
 wire [3:0] b=left;
 wire merge=a!=0 && b!=0 && a!=b;
 wire choose_a=rank[a]>rank[b] || (rank[a]==rank[b] && a<b);
 wire [3:0] winner=merge ? (choose_a ? a : b) : a!=0 ? a : b!=0 ? b : next_label;
 wire [3:0] loser=choose_a ? b : a;
 wire allocate=a==0 && b==0;
 wire accepted=p_hit && adjacent && (!allocate || next_label<=C);
 wire [19:0] added=run_active ? 20'd1 : 20'd2;
 wire [10:0] start_x=run_active ? px : px-4;
 wire [10:0] old_xlo=allocate ? WIDTH-1 : xlo[winner];
 wire [10:0] old_ylo=allocate ? HEIGHT-1 : ylo[winner];
 wire [10:0] old_xhi=allocate ? 11'd0 : xhi[winner];
 wire [10:0] old_yhi=allocate ? 11'd0 : yhi[winner];
 wire [19:0] old_area=allocate ? 20'd0 : area[winner];
 wire [10:0] merged_xlo=merge && xlo[loser]<old_xlo ? xlo[loser] : old_xlo;
 wire [10:0] merged_ylo=merge && ylo[loser]<old_ylo ? ylo[loser] : old_ylo;
 wire [10:0] merged_xhi=merge && xhi[loser]>old_xhi ? xhi[loser] : old_xhi;
 wire [10:0] merged_yhi=merge && yhi[loser]>old_yhi ? yhi[loser] : old_yhi;
 reg [19:0] saved_area[1:C];
 reg [10:0] saved_x0[1:C],saved_y0[1:C],saved_x1[1:C],saved_y1[1:C];
 reg [10:0] sx0,sy0,sx1,sy1;
 reg [15:0] sa;
 reg acq,scanning;
 reg [3:0] scan;
 reg [12:0] best_score;
 wire [11:0] rw={1'b0,sx1}-{1'b0,sx0}+1'b1,rh={1'b0,sy1}-{1'b0,sy0}+1'b1;
 wire [11:0] cw={1'b0,saved_x1[scan]}-{1'b0,saved_x0[scan]}+1'b1;
 wire [11:0] ch={1'b0,saved_y1[scan]}-{1'b0,saved_y0[scan]}+1'b1;
 wire [11:0] ccx={1'b0,saved_x0[scan]}+saved_x1[scan],ccy={1'b0,saved_y0[scan]}+saved_y1[scan];
 wire [11:0] rcx={1'b0,sx0}+sx1,rcy={1'b0,sy0}+sy1;
 wire [11:0] dx=ccx>rcx ? ccx-rcx : rcx-ccx,dy=ccy>rcy ? ccy-rcy : rcy-ccy;
 wire [12:0] score={1'b0,dx}+dy;
 wire [23:0] box_area=cw*ch;
 wire density={saved_area[scan],4'd0}>=box_area;
 wire size_ok=acq ? cw<=2*rw+32 && ch<=2*rh+32 :
                  cw<=rw+(rw>>1)+16 && ch<=rh+(rh>>1)+16 && cw+16>=(rw>>1) && ch+16>=(rh>>1);
 wire seed_ok=!acq || ({saved_area[scan],3'd0}>=({8'd0,sa}<<6) && {saved_area[scan],2'd0}<=({8'd0,sa}<<7));
 wire good=saved_area[scan]>=16 && cw>=8 && ch>=8 && density && size_ok && seed_ok;
 integer i;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   previous_hit<=0;run_active<=0;left_label<=0;previous_above<=0;previous_x<=0;previous_y<=0;available<={C{1'b1}};row_tags<=0;
   done<=0;found<=0;count<=0;xmin<=0;ymin<=0;xmax<=0;ymax<=0;overflow<=0;
   scanning<=0;scan<=1;best_score<=8191;sx0<=0;sy0<=0;sx1<=0;sy1<=0;sa<=0;acq<=1;
   for(i=0;i<=C;i=i+1)begin parent[i]<=i;rank[i]<=0;area[i]<=0;xlo[i]<=WIDTH-1;ylo[i]<=HEIGHT-1;xhi[i]<=0;yhi[i]<=0;end
   for(i=1;i<=C;i=i+1)begin saved_area[i]<=0;saved_x0[i]<=0;saved_y0[i]<=0;saved_x1[i]<=0;saved_y1[i]<=0;end
  end else begin
   done<=0;
   if(p_take && !clear)begin
    previous_hit<=p_hit;previous_x<=px;previous_y<=py;previous_above<=above;
    line_label[px>>2]<=accepted ? winner : 4'd0;
    left_label<=accepted ? winner : 4'd0;
    run_active<=accepted;
    if(p_hit && adjacent && allocate && next_label>C)overflow<=1;
    if(accepted)begin
     row_tags[winner]<=1'b1;
     if(allocate)available[winner]<=1'b0;
     if(merge)begin for(i=1;i<=C;i=i+1)if(parent[i]==loser)parent[i]<=winner;parent[loser]<=winner;area[loser]<=0;if(rank[a]==rank[b])rank[winner]<=rank[winner]+1'b1;end
     area[winner]<=old_area+(merge ? area[loser] : 20'd0)+added;
     xlo[winner]<=start_x<merged_xlo ? start_x : merged_xlo;
     ylo[winner]<=py<merged_ylo ? py : merged_ylo;
     xhi[winner]<=px+3>merged_xhi ? px+3 : merged_xhi;
     yhi[winner]<=py>merged_yhi ? py : merged_yhi;
    end
   end
   if(p_take && !clear && px==WIDTH-4)begin
    row_tags<=0;
    for(i=1;i<=C;i=i+1)begin
     if(parent[i]!=i && !row_tags[i] && !(accepted && winner==i))begin
      available[i]<=1'b1;parent[i]<=i;rank[i]<=0;
     end
    end
   end
   if(end_d)begin
    for(i=1;i<=C;i=i+1)begin
     saved_area[i]<=parent[i]==i ? area[i] : 20'd0;
     saved_x0[i]<=xlo[i];saved_y0[i]<=ylo[i];saved_x1[i]<=xhi[i];saved_y1[i]<=yhi[i];
    end
    sx0<=ref_x0;sy0<=ref_y0;sx1<=ref_x1;sy1<=ref_y1;sa<=seed_area;acq<=acquiring;
    found<=0;scanning<=1;scan<=1;best_score<=8191;count<=0;
   end else if(scanning)begin
    if(good && (!found || score<best_score || (score==best_score && saved_area[scan]>count)))begin
     found<=1;best_score<=score;count<=saved_area[scan];
     xmin<=saved_x0[scan];ymin<=saved_y0[scan];xmax<=saved_x1[scan];ymax<=saved_y1[scan];
    end
    if(scan==C)begin scanning<=0;done<=1;end else scan<=scan+1'b1;
   end
   if(clear)begin
    previous_hit<=0;run_active<=0;left_label<=0;previous_above<=0;available<={C{1'b1}};row_tags<=0;overflow<=0;
    for(i=0;i<=C;i=i+1)begin parent[i]<=i;rank[i]<=0;area[i]<=0;xlo[i]<=WIDTH-1;ylo[i]<=HEIGHT-1;xhi[i]<=0;yhi[i]<=0;end
   end
  end
 end
endmodule
