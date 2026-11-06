// Passive RGB overlay, MSB pixel first. Raw mode is bit-exact.
// PIPELINE=1 splits geometry/character selection from font lookup/rendering.
// PIPELINE=0 is combinational; PIPELINE=1 keeps the original two registers.
// Geometry and text consume the SAME frame-boundary result snapshot.
module cnn_osd_4px #(
 parameter WIDTH=1280,HEIGHT=720,AIM_SIZE=320,AIM_HEIGHT=AIM_SIZE,PIPELINE=0
)(
 input wire clk,rst_n,frame,valid,enable_frame,
 input wire [95:0] rgb,
 input wire target_visible,target_locked,
 input wire [1:0] shape,
 input wire [2:0] color_id,
 input wire [10:0] xmin,ymin,xmax,ymax,cx,cy,
 output wire [95:0] rgb_out
);
 reg frame_d,active;
 reg [11:0] nx,ny;
 wire sof=frame && !frame_d;
 wire [11:0] x=sof ? 12'd0 : nx,y=sof ? 12'd0 : ny;
 wire draw=rst_n && valid && enable_frame && (sof || active);
 localparam [191:0] CNN_MODE_SIGNATURE=192'h4dccd3fe6d015c523e6cec45a247f75cb96286ef6d454dcb;
 function [23:0] tagged_rgb;
  input [23:0] value;input [2:0] tag;begin
   tagged_rgb={value[23:17],tag[2],value[15:9],tag[1],value[7:1],tag[0]};
  end
 endfunction
 localparam AX0=(WIDTH-AIM_SIZE)/2,AX1=AX0+AIM_SIZE-1;
 localparam AY0=(HEIGHT-AIM_HEIGHT)/2,AY1=AY0+AIM_HEIGHT-1;
 localparam LABEL_W=280,LABEL_H=60;
 wire [11:0] label_x=xmin>WIDTH-LABEL_W ? WIDTH-LABEL_W : xmin;
 wire [11:0] label_y=ymin>=LABEL_H+4 ? ymin-LABEL_H-4 : (ymax+LABEL_H+4<HEIGHT ? ymax+4 : 12'd0);
 function [7:0] label_character;
  input [4:0] position;
  input [1:0] s;
  input [2:0] c;
  reg [55:0] word_color;
  reg [71:0] word_shape;
  begin
   case(s)1:word_shape="TRIANGLE ";2:word_shape="RECTANGLE";3:word_shape="CIRCLE   ";default:word_shape="UNKNOWN  ";endcase
   case(c)1:word_color="RED    ";2:word_color="BLUE   ";3:word_color="GREEN  ";4:word_color="YELLOW ";default:word_color="UNKNOWN";endcase
   if(position<9)label_character=word_shape >> ((8-position)*8);
   else if(position==9)label_character=" ";
   else if(position<17)label_character=word_color >> ((16-position)*8);
   else label_character=" ";
  end
 endfunction
 // Shared combinational decimal conversion: no clock, handshake or video delay.
 function [15:0] decimal4;
  input [10:0] number;reg [15:0] bcd;integer bit_index,digit;begin
   bcd=0;
   for(bit_index=10;bit_index>=0;bit_index=bit_index-1)begin
    for(digit=0;digit<4;digit=digit+1)
     if(bcd[digit*4+:4]>=5)bcd[digit*4+:4]=bcd[digit*4+:4]+4'd3;
    bcd={bcd[14:0],number[bit_index]};
   end
   decimal4=bcd;
  end
 endfunction
 wire [15:0] decimal_x=decimal4(cx),decimal_y=decimal4(cy);
 function [7:0] coordinate_character;
  input [4:0] position;input [15:0] xx,yy;begin
   case(position)
    0:coordinate_character="X";1:coordinate_character=":";
    2:coordinate_character=8'h30+xx[15:12];3:coordinate_character=8'h30+xx[11:8];
    4:coordinate_character=8'h30+xx[7:4];5:coordinate_character=8'h30+xx[3:0];
    7:coordinate_character="Y";8:coordinate_character=":";
    9:coordinate_character=8'h30+yy[15:12];10:coordinate_character=8'h30+yy[11:8];
    11:coordinate_character=8'h30+yy[7:4];12:coordinate_character=8'h30+yy[3:0];
    default:coordinate_character=" ";
   endcase
  end
 endfunction
 // Private label font: same cap height and stroke width for every letter.
 // The shared board font and all recognition/tracker inputs stay unchanged.
 function [4:0] label_glyph;
  input [7:0] ch;input [2:0] row;reg [34:0] bits;begin
   case(ch)
    8'h41:bits=35'b01110100011000111111100011000110001;
    8'h42:bits=35'b11110100011000111110100011000111110;
    8'h43:bits=35'b01111100001000010000100001000001111;
    8'h44:bits=35'b11110100011000110001100011000111110;
    8'h45:bits=35'b11111100001000011110100001000011111;
    8'h47:bits=35'b01111100001000010111100011000101111;
    8'h49:bits=35'b11111001000010000100001000010011111;
    8'h4b:bits=35'b10001100101010011000101001001010001;
    8'h4c:bits=35'b10000100001000010000100001000011111;
    8'h4e:bits=35'b10001110011100110101100111001110001;
    8'h4f:bits=35'b01110100011000110001100011000101110;
    8'h52:bits=35'b11110100011000111110101001001010001;
    8'h54:bits=35'b11111001000010000100001000010000100;
    8'h55:bits=35'b10001100011000110001100011000101110;
    8'h57:bits=35'b10001100011000110101101011010101010;
    8'h59:bits=35'b10001100010101000100001000010000100;
    8'h30:bits=35'b01110100011001110101110011000101110;
    8'h31:bits=35'b00100011000010000100001000010001110;
    8'h32:bits=35'b01110100010000100010001000100011111;
    8'h33:bits=35'b11110000010000101110000010000111110;
    8'h34:bits=35'b00010001100101010010111110001000010;
    8'h35:bits=35'b11111100001000011110000010000111110;
    8'h36:bits=35'b01110100001000011110100011000101110;
    8'h37:bits=35'b11111000010001000100010000100001000;
    8'h38:bits=35'b01110100011000101110100011000101110;
    8'h39:bits=35'b01110100011000101111000010000101110;
    8'h58:bits=35'b10001100010101000100010101000110001;
    8'h3a:bits=35'b00000001000010000000001000010000000;
    default:bits=0;
   endcase
   label_glyph=row<7 ? bits>>((6-row)*5) : 5'd0;
  end
 endfunction
 function [2:0] glyph_row;
  input [11:0] yy;begin
   if(yy<3)glyph_row=0;else if(yy<6)glyph_row=1;
   else if(yy<9)glyph_row=2;else if(yy<12)glyph_row=3;
   else if(yy<15)glyph_row=4;else if(yy<18)glyph_row=5;
   else if(yy<21)glyph_row=6;else glyph_row=7;
  end
 endfunction
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin frame_d<=0;active<=0;nx<=0;ny<=0;end
  else begin
   frame_d<=frame;
   if(sof)begin nx<=0;ny<=0;active<=1;end
   if(valid && (sof || active))begin
    if(x==WIDTH-4)begin nx<=0;if(y==HEIGHT-1)begin ny<=0;active<=0;end else ny<=y+1'b1;end
    else nx<=x+4;
   end
  end
 end
 genvar lane;
 generate for(lane=0;lane<4;lane=lane+1)begin:g_lane
  wire [11:0] px=x+lane;
  // Only CNN frames carry state; each channel changes by at most one LSB.
  wire mode_tag=draw && y==0 && px<64;
  wire [2:0] tag_bits=CNN_MODE_SIGNATURE >> ((63-px[5:0])*3);
  wire aim_h=(y==AY0 || y==AY1) && px>=AX0 && px<=AX1 && !px[3];
  wire aim_v=(px==AX0 || px==AX1) && y>=AY0 && y<=AY1 && !y[3];
  wire center=((px==WIDTH/2 || px==WIDTH/2+1) && y>=HEIGHT/2-12 && y<=HEIGHT/2+12) ||
              ((y==HEIGHT/2 || y==HEIGHT/2+1) && px>=WIDTH/2-12 && px<=WIDTH/2+12);
  // Static 16-pixel dashes / 16-pixel gaps, anchored to the target box.
  // Black halo follows each dash; gaps pass the original video through.
  wire [11:0] dx=px-xmin,dy=y-ymin;
  wire [11:0] dx_before=dx-1'b1,dy_before=dy-1'b1;
  wire [11:0] dx_after=dx+1'b1,dy_after=dy+1'b1;
  wire horizontal_dash=!dx[4],vertical_dash=!dy[4];
  wire horizontal_halo=horizontal_dash || !dx_before[4] || !dx_after[4];
  wire vertical_halo=vertical_dash || !dy_before[4] || !dy_after[4];
  wire target_inside=px>=xmin && px<=xmax && y>=ymin && y<=ymax;
  wire target_stroke=target_inside &&
   (((y<ymin+3 || y+3>ymax) && horizontal_dash) ||
    ((px<xmin+3 || px+3>xmax) && vertical_dash));
  wire target_outline=px+1>=xmin && px<=xmax+1 && y+1>=ymin && y<=ymax+1 &&
   (((y<ymin+4 || y+4>ymax) && horizontal_halo) ||
    ((px<xmin+4 || px+4>xmax) && vertical_halo));
  wire centroid=((px==cx || px==cx+1) && y+8>=cy && y<=cy+8) ||
                ((y==cy || y==cy+1) && px+8>=cx && px<=cx+8);
  wire text_area=px>=label_x && px<label_x+LABEL_W && y>=label_y && y<label_y+LABEL_H;
  wire text_ink=px>=label_x+4 && px<label_x+276 && y>=label_y+4 && (y<label_y+28 || (y>=label_y+32 && y<label_y+56));
  wire coordinate_line=y>=label_y+32;
  wire [11:0] tx=px-label_x-4,ty=coordinate_line ? y-label_y-32 : y-label_y-4;
  wire [7:0] character=coordinate_line ? coordinate_character(tx[8:4],decimal_x,decimal_y) : label_character(tx[8:4],shape,color_id);
  wire [15:0] glyph;
  wire [7:0] font_character;
  wire [2:0] font_row;
  wire [4:0] small_glyph=label_glyph(font_character,font_row);
  assign glyph={{3{small_glyph[4]}},{3{small_glyph[3]}},{3{small_glyph[2]}},{3{small_glyph[1]}},{3{small_glyph[0]}},1'b0};
  wire text_pixel=glyph[15-tx[3:0]];
  wire hit=aim_h || aim_v || center || (target_visible && (target_outline || centroid || text_area));
  reg [23:0] paint;
  always @*begin
   paint=24'h4080ff;
   if(target_visible && target_outline)paint=24'h000000;
   if(target_visible && target_stroke)paint=target_locked ? 24'hff00ff : 24'hffaa00;
   if(center || (target_visible && centroid))paint=24'hff4040;
   if(target_visible && text_area)paint=text_ink && text_pixel ? 24'hffff00 : 24'h000000;
  end
  if(PIPELINE==0)begin:g_comb
   assign font_character=character;
   assign font_row=glyph_row(ty);
   wire [23:0] untagged=draw && hit ? paint : rgb[95-lane*24-:24];
   assign rgb_out[95-lane*24-:24]=mode_tag ? tagged_rgb(untagged,tag_bits) : untagged;
  end else begin:g_pipe
   reg [7:0] character_d;
   reg [2:0] row_d;reg [3:0] column_d;
   reg hit_d,text_d,ink_d,tag_d;reg [2:0] tag_bits_d;
   reg [23:0] background_d,paint_d,pixel_d;
   wire [23:0] base_paint=(center || (target_visible && centroid)) ? 24'hff4040 :
     target_visible && target_stroke ? (target_locked ? 24'hff00ff : 24'hffaa00) :
     target_visible && target_outline ? 24'h000000 : 24'h4080ff;
   wire [23:0] untagged=hit_d ? (text_d ? (ink_d && glyph[15-column_d] ? 24'hffff00 : 24'h000000) : paint_d) : background_d;
   assign font_character=character_d;
   assign font_row=row_d;
   always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
     character_d<=0;row_d<=0;column_d<=0;hit_d<=0;text_d<=0;ink_d<=0;
     background_d<=0;paint_d<=0;pixel_d<=0;tag_d<=0;tag_bits_d<=0;
    end else begin
     character_d<=character;row_d<=glyph_row(ty);column_d<=tx[3:0];
     tag_d<=mode_tag;tag_bits_d<=tag_bits;
     hit_d<=draw && hit;text_d<=target_visible && text_area;ink_d<=text_ink;
     background_d<=rgb[95-lane*24-:24];paint_d<=base_paint;
     pixel_d<=tag_d ? tagged_rgb(untagged,tag_bits_d) : untagged;
    end
   end
   assign rgb_out[95-lane*24-:24]=pixel_d;
  end
 end endgenerate
 // synthesis translate_off
 initial if(WIDTH%4!=0 || WIDTH<280 || HEIGHT<68 || AIM_HEIGHT>HEIGHT || AIM_SIZE>WIDTH)
  $fatal(1,"unsupported CNN OSD geometry");
 // synthesis translate_on
endmodule
