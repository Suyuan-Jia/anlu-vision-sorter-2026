// 功能说明：保留像素与同步延迟，通过独立参数关闭屏幕文字和Logo叠加。

module hdmi_mixer # ( parameter H_OFFSET = 128,
    parameter V_OFFSET = 60,
    parameter IMG_WIDTH = 1280,
    parameter IMG_HEIGHT = 720,
    parameter integer DEBUG_MODE = 0,
    parameter integer MODE_OSD_ENABLE = 0,
    parameter integer OSD_ENABLE = 1 ) ( input wire I_clk,
    input wire I_rst_n,
    input wire I_video_vsync,
    input wire I_video_hsync,
    input wire I_video_de,
    input wire I_video_user,
    input wire I_video_last,
    input wire [2:0] I_display_mode,
    input wire [3 : 0] I_debug_status,
    output wire O_video_rd_en,
    input wire [23 : 0] I_video_rd_data,
    output reg O_hdmi_vsync,
    output reg O_hdmi_hsync,
    output reg O_hdmi_de,
    output reg [23 : 0] O_hdmi_data );

    function [7:0] mode_char;
      input [2:0] mode;input [4:0] index;reg [207:0] label;
      begin
        case(mode)
          3'd1:label="MODE 1: ORIGINAL          ";
          3'd2:label="MODE 2: EDGE              ";
          3'd3:label="MODE 3: GRAYSCALE         ";
          3'd4:label="MODE 4: CARTOON           ";
          3'd5:label="MODE 1-2: MOTION DETECTION";
          3'd6:label="MODE 1-1: OBJECT DETECTION";
          default:label="                          ";
        endcase
        mode_char=index<26 ? label >> ((25-index)*8) : 8'h20;
      end
    endfunction
    function [15:0] mode_font_row;
      input [7:0] ch; input [3:0] row; input [15:0] fallback;
      begin
        mode_font_row=fallback;
        case(ch)
          8'h2d: mode_font_row=(row==7 || row==8)?16'h0ff0:16'h0000;
          8'h42: case(row)
            4'd2,4'd7,4'd12:mode_font_row=16'h3ff0;
            4'd3,4'd4,4'd5,4'd6,4'd8,4'd9,4'd10,4'd11:mode_font_row=16'h300c;
            default:mode_font_row=0;
          endcase
          8'h4a: case(row)
            4'd2:mode_font_row=16'h0ffc;
            4'd3,4'd4,4'd5,4'd6,4'd7,4'd8,4'd9:mode_font_row=16'h0030;
            4'd10:mode_font_row=16'h3030;
            4'd11:mode_font_row=16'h1860;
            4'd12:mode_font_row=16'h0fc0;
            default:mode_font_row=0;
          endcase
          8'h4d: case(row)
            4'd2,4'd3:mode_font_row=16'h300c;
            4'd4,4'd5:mode_font_row=16'h381c;
            4'd6,4'd7:mode_font_row=16'h342c;
            4'd8:mode_font_row=16'h324c;
            4'd9,4'd10,4'd11,4'd12:mode_font_row=16'h300c;
            default:mode_font_row=0;
          endcase
          8'h41: case(row)
            4'd2: mode_font_row=16'h03f0;
            4'd3: mode_font_row=16'h0c0c;
            4'd4: mode_font_row=16'h1806;
            4'd5: mode_font_row=16'h300c;
            4'd6: mode_font_row=16'h300c;
            4'd7: mode_font_row=16'h3ffc;
            4'd8: mode_font_row=16'h300c;
            4'd9: mode_font_row=16'h300c;
            4'd10: mode_font_row=16'h300c;
            4'd11: mode_font_row=16'h300c;
            4'd12: mode_font_row=16'h300c;
            default: mode_font_row=0;
          endcase
          8'h44: case(row)
            4'd2: mode_font_row=16'h3fe0;
            4'd3: mode_font_row=16'h3030;
            4'd4: mode_font_row=16'h3018;
            4'd5: mode_font_row=16'h300c;
            4'd6: mode_font_row=16'h300c;
            4'd7: mode_font_row=16'h300c;
            4'd8: mode_font_row=16'h300c;
            4'd9: mode_font_row=16'h300c;
            4'd10: mode_font_row=16'h3018;
            4'd11: mode_font_row=16'h3030;
            4'd12: mode_font_row=16'h3fe0;
            default: mode_font_row=0;
          endcase
          8'h45: case(row)
            4'd2: mode_font_row=16'h3ffc;
            4'd3: mode_font_row=16'h3000;
            4'd4: mode_font_row=16'h3000;
            4'd5: mode_font_row=16'h3000;
            4'd6: mode_font_row=16'h3000;
            4'd7: mode_font_row=16'h3ff0;
            4'd8: mode_font_row=16'h3000;
            4'd9: mode_font_row=16'h3000;
            4'd10: mode_font_row=16'h3000;
            4'd11: mode_font_row=16'h3000;
            4'd12: mode_font_row=16'h3ffc;
            default: mode_font_row=0;
          endcase
          8'h47: case(row)
            4'd2: mode_font_row=16'h03f0;
            4'd3: mode_font_row=16'h0c0c;
            4'd4: mode_font_row=16'h1800;
            4'd5: mode_font_row=16'h3000;
            4'd6: mode_font_row=16'h3000;
            4'd7: mode_font_row=16'h30fc;
            4'd8: mode_font_row=16'h300c;
            4'd9: mode_font_row=16'h300c;
            4'd10: mode_font_row=16'h180c;
            4'd11: mode_font_row=16'h0c0c;
            4'd12: mode_font_row=16'h03f0;
            default: mode_font_row=0;
          endcase
          8'h49: case(row)
            4'd2: mode_font_row=16'h0ff0;
            4'd3: mode_font_row=16'h0180;
            4'd4: mode_font_row=16'h0180;
            4'd5: mode_font_row=16'h0180;
            4'd6: mode_font_row=16'h0180;
            4'd7: mode_font_row=16'h0180;
            4'd8: mode_font_row=16'h0180;
            4'd9: mode_font_row=16'h0180;
            4'd10: mode_font_row=16'h0180;
            4'd11: mode_font_row=16'h0180;
            4'd12: mode_font_row=16'h0ff0;
            default: mode_font_row=0;
          endcase
          8'h4c: case(row)
            4'd2: mode_font_row=16'h3000;
            4'd3: mode_font_row=16'h3000;
            4'd4: mode_font_row=16'h3000;
            4'd5: mode_font_row=16'h3000;
            4'd6: mode_font_row=16'h3000;
            4'd7: mode_font_row=16'h3000;
            4'd8: mode_font_row=16'h3000;
            4'd9: mode_font_row=16'h3000;
            4'd10: mode_font_row=16'h3000;
            4'd11: mode_font_row=16'h3000;
            4'd12: mode_font_row=16'h3ffc;
            default: mode_font_row=0;
          endcase
          8'h4f: case(row)
            4'd2: mode_font_row=16'h03f0;
            4'd3: mode_font_row=16'h0c0c;
            4'd4: mode_font_row=16'h1806;
            4'd5: mode_font_row=16'h300c;
            4'd6: mode_font_row=16'h300c;
            4'd7: mode_font_row=16'h300c;
            4'd8: mode_font_row=16'h300c;
            4'd9: mode_font_row=16'h300c;
            4'd10: mode_font_row=16'h1806;
            4'd11: mode_font_row=16'h0c0c;
            4'd12: mode_font_row=16'h03f0;
            default: mode_font_row=0;
          endcase
          8'h52: case(row)
            4'd2: mode_font_row=16'h3ff0;
            4'd3: mode_font_row=16'h300c;
            4'd4: mode_font_row=16'h300c;
            4'd5: mode_font_row=16'h300c;
            4'd6: mode_font_row=16'h300c;
            4'd7: mode_font_row=16'h3ff0;
            4'd8: mode_font_row=16'h3300;
            4'd9: mode_font_row=16'h3180;
            4'd10: mode_font_row=16'h30c0;
            4'd11: mode_font_row=16'h3060;
            4'd12: mode_font_row=16'h3030;
            default: mode_font_row=0;
          endcase
          8'h53: case(row)
            4'd2: mode_font_row=16'h0ff0;
            4'd3: mode_font_row=16'h300c;
            4'd4: mode_font_row=16'h3000;
            4'd5: mode_font_row=16'h3000;
            4'd6: mode_font_row=16'h1c00;
            4'd7: mode_font_row=16'h03e0;
            4'd8: mode_font_row=16'h001c;
            4'd9: mode_font_row=16'h000c;
            4'd10: mode_font_row=16'h000c;
            4'd11: mode_font_row=16'h300c;
            4'd12: mode_font_row=16'h0ff0;
            default: mode_font_row=0;
          endcase
          8'h59: case(row)
            4'd2: mode_font_row=16'h300c;
            4'd3: mode_font_row=16'h300c;
            4'd4: mode_font_row=16'h1818;
            4'd5: mode_font_row=16'h0c30;
            4'd6: mode_font_row=16'h0660;
            4'd7: mode_font_row=16'h03c0;
            4'd8: mode_font_row=16'h0180;
            4'd9: mode_font_row=16'h0180;
            4'd10: mode_font_row=16'h0180;
            4'd11: mode_font_row=16'h0180;
            4'd12: mode_font_row=16'h0180;
            default: mode_font_row=0;
          endcase
          default: mode_font_row=fallback;
        endcase
      end
    endfunction

    // Three stages, identical to the accepted mixer latency:
    // coordinates/address -> glyph/logo pixels -> final RGB selection.
    // No combinational address multiplier + logo ROM + color mux in one cycle.
    reg [11:0] S_x,S_y;
    reg [1:0] de_pipe,hs_pipe,vs_pipe;
    reg [23:0] rgb1,rgb2;
    reg [2:0] display_mode_meta,display_mode_sync;
    // Read-only display metadata from the displayed frame, after DDR. Never
    // drives video read enables, timing, transport or frame-bank selection.
    localparam [191:0] CNN_MODE_SIGNATURE=192'h4dccd3fe6d015c523e6cec45a247f75cb96286ef6d454dcb;
    reg cnn_frame_mode,signature_collect;
    reg [6:0] signature_count;
    reg [191:0] signature_window;
    wire [2:0] received_signature={I_video_rd_data[16],I_video_rd_data[8],I_video_rd_data[0]};
    wire [191:0] signature_next={signature_window[188:0],received_signature};
    wire [2:0] label_mode=(display_mode_sync==1 && cnn_frame_mode)?3'd6:display_mode_sync;
    always @(posedge I_clk or negedge I_rst_n)
      if(!I_rst_n)begin cnn_frame_mode<=0;signature_collect<=0;signature_count<=0;signature_window<=0;end
      else if(I_video_user)begin
        cnn_frame_mode<=0;signature_collect<=1;signature_count<=I_video_de?1:0;
        signature_window<=I_video_de?{189'd0,received_signature}:192'd0;
      end else if(signature_collect && I_video_de)begin
        // A sliding comparison tolerates the unchanged DDR/FIFO output delay.
        // Detection finishes within the first 128 pixels, before the mode row.
        signature_window<=signature_next;
        if(signature_count>=63 && signature_next==CNN_MODE_SIGNATURE)cnn_frame_mode<=1;
        if(signature_count==127)signature_collect<=0;
        else signature_count<=signature_count+1'b1;
      end
    reg [3:0] digits[0:5];
    integer i;
    always @(posedge I_clk or negedge I_rst_n)
      if(!I_rst_n)begin S_x<=0;S_y<=0;end
      else if(I_video_user)begin S_x<=0;S_y<=0;end
      else if(I_video_de)begin
        if(I_video_last)begin S_x<=0;S_y<=S_y+1'b1;end
        else S_x<=S_x+1'b1;
      end
    always @(posedge I_clk or negedge I_rst_n)
      if(!I_rst_n)begin
        display_mode_meta<=1;display_mode_sync<=1;
        for(i=0;i<6;i=i+1)digits[i]<=0;
      end else begin
        display_mode_meta<=I_display_mode;display_mode_sync<=display_mode_meta;
        if(I_video_user)begin
          if(digits[0]!=9)digits[0]<=digits[0]+1'b1;
          else begin
            digits[0]<=0;
            if(digits[1]!=9)digits[1]<=digits[1]+1'b1;
            else begin
              digits[1]<=0;
              if(digits[2]!=9)digits[2]<=digits[2]+1'b1;
              else begin
                digits[2]<=0;
                if(digits[3]!=9)digits[3]<=digits[3]+1'b1;
                else begin
                  digits[3]<=0;
                  if(digits[4]!=9)digits[4]<=digits[4]+1'b1;
                  else begin digits[4]<=0;digits[5]<=digits[5]==9?0:digits[5]+1'b1;end
                end
              end
            end
          end
        end
      end
    // Exact floor(n*16/24) for n=0..23; small logic, no divider or DSP.
    function [3:0] scale24(input [4:0] n);
      begin case(n)
        5'd0:scale24=4'd0;
        5'd1:scale24=4'd0;
        5'd2:scale24=4'd1;
        5'd3:scale24=4'd2;
        5'd4:scale24=4'd2;
        5'd5:scale24=4'd3;
        5'd6:scale24=4'd4;
        5'd7:scale24=4'd4;
        5'd8:scale24=4'd5;
        5'd9:scale24=4'd6;
        5'd10:scale24=4'd6;
        5'd11:scale24=4'd7;
        5'd12:scale24=4'd8;
        5'd13:scale24=4'd8;
        5'd14:scale24=4'd9;
        5'd15:scale24=4'd10;
        5'd16:scale24=4'd10;
        5'd17:scale24=4'd11;
        5'd18:scale24=4'd12;
        5'd19:scale24=4'd12;
        5'd20:scale24=4'd13;
        5'd21:scale24=4'd14;
        5'd22:scale24=4'd14;
        5'd23:scale24=4'd15;
        default:scale24=0;
      endcase end
    endfunction
    reg [7:0] ch;
    reg [3:0] row,col;
    reg text_valid;
    reg [11:0] tx,ty;
    reg [4:0] local_y;
    always @*begin
      ch=0;row=0;col=0;text_valid=0;tx=0;ty=0;local_y=0;
      if(S_x>=375 && S_x<575 && S_y>=120 && S_y<144)begin
        tx=S_x-375;ty=S_y-120;
        if(tx<48 && ty>=2)begin
          ch=tx<24?8'h80:8'h81;
          col=scale24(tx<24?tx:tx-24);row=scale24(ty-2);text_valid=1;
        end else if(tx>=52 && tx<100 && ty>=4 && ty<20)begin
          ch=tx<68?8'h43:tx<84?8'h4e:8'h54;
          col=tx<68?tx-52:tx<84?tx-68:tx-84;row=ty-4;text_valid=1;
        end else if(tx>=100 && tx<108 && ty>=4 && ty<20)begin
          ch=8'h3a;col=(tx-100)<<1;row=ty-4;text_valid=1;
        end else if(tx>=110)begin
          if(tx<125)begin ch=8'h30+digits[5];col=tx-110;end
          else if(tx<140)begin ch=8'h30+digits[4];col=tx-125;end
          else if(tx<155)begin ch=8'h30+digits[3];col=tx-140;end
          else if(tx<170)begin ch=8'h30+digits[2];col=tx-155;end
          else if(tx<185)begin ch=8'h30+digits[1];col=tx-170;end
          else begin ch=8'h30+digits[0];col=tx-185;end
          row=scale24(ty);text_valid=1;
        end
      end else if(MODE_OSD_ENABLE && S_x>=375 && S_x<((label_mode==5 || label_mode==6)?791:647) && S_y>=152 && S_y<176)begin
        tx=S_x-375;local_y=S_y-152;
        ch=mode_char(label_mode,tx[8:4]);col=tx[3:0];row=scale24(local_y);text_valid=1;
      end
    end
    reg [14:0] logo_addr1;
    reg logo_inside1,logo_inside2;
    reg [24:0] logo_pixel2;
    wire [24:0] logo_pixel;
    reg [7:0] char1;
    reg [3:0] row1,col1;
    reg text_valid1,text_on2;
    wire [15:0] font_bits;
    wire [15:0] glyph_bits=MODE_OSD_ENABLE?mode_font_row(char1,row1,font_bits):font_bits;
    anlogic_logo_rom u_anlogic_logo_rom(.I_addr(logo_addr1),.O_pixel(logo_pixel));
    osd_char_lib u_osd_char_lib(.I_char(char1),.I_row(row1),.O_row_bits(font_bits));
    assign O_video_rd_en=I_video_de;
    reg [11:0] debug_x1,debug_x2;
    always @(posedge I_clk or negedge I_rst_n)
      if(!I_rst_n)begin
        de_pipe<=0;hs_pipe<=0;vs_pipe<=0;rgb1<=0;rgb2<=0;
        logo_addr1<=0;logo_inside1<=0;logo_inside2<=0;logo_pixel2<=0;
        char1<=0;row1<=0;col1<=0;text_valid1<=0;text_on2<=0;
        O_hdmi_de<=0;O_hdmi_hsync<=0;O_hdmi_vsync<=0;O_hdmi_data<=0;
        debug_x1<=0;debug_x2<=0;
      end else begin
        de_pipe<={de_pipe[0],I_video_de};hs_pipe<={hs_pipe[0],I_video_hsync};vs_pipe<={vs_pipe[0],I_video_vsync};
        rgb1<=I_video_rd_data;rgb2<=rgb1;
        debug_x1<=S_x;debug_x2<=debug_x1;
        // 160*y+x = (y<<7)+(y<<5)+x, deliberately before the ROM register.
        logo_addr1<=({3'd0,S_y}<<7)+({3'd0,S_y}<<5)+{3'd0,S_x};
        logo_inside1<=I_video_de && S_x<160 && S_y<160;
        logo_inside2<=logo_inside1;logo_pixel2<=logo_pixel;
        char1<=ch;row1<=row;col1<=col;text_valid1<=text_valid && I_video_de;
        text_on2<=text_valid1 && glyph_bits[15-col1];
        O_hdmi_de<=de_pipe[1];O_hdmi_hsync<=hs_pipe[1];O_hdmi_vsync<=vs_pipe[1];
        if(DEBUG_MODE==1)begin
          if(!de_pipe[1])O_hdmi_data<=0;
          else if(debug_x2<128)O_hdmi_data<=24'hff0000;
          else if(debug_x2<256)O_hdmi_data<=24'h00ff00;
          else if(debug_x2<384)O_hdmi_data<=24'h0000ff;
          else if(debug_x2<512)O_hdmi_data<=24'hffff00;
          else if(debug_x2<640)O_hdmi_data<=24'h00ffff;
          else if(debug_x2<768)O_hdmi_data<=24'hff00ff;
          else if(debug_x2<896)O_hdmi_data<=24'hffffff;
          else O_hdmi_data<=24'h202020;
        end else if(DEBUG_MODE==2)begin
          if(!de_pipe[1])O_hdmi_data<=0;
          else case(I_debug_status)
            0:O_hdmi_data<=0;1:O_hdmi_data<=24'hff0000;3:O_hdmi_data<=24'hffff00;
            7:O_hdmi_data<=24'h00ff00;15:O_hdmi_data<=rgb2;default:O_hdmi_data<=24'h0000ff;
          endcase
        end else if(DEBUG_MODE==3)begin
          if(!de_pipe[1])O_hdmi_data<=0;
          else if(debug_x2<256)O_hdmi_data<=I_debug_status[0]?24'hff8000:24'h201000;
          else if(debug_x2<512)O_hdmi_data<=I_debug_status[1]?24'hffff00:24'h202000;
          else if(debug_x2<768)O_hdmi_data<=I_debug_status[2]?24'hff0000:24'h200000;
          else O_hdmi_data<=I_debug_status[3]?24'h00ffff:24'h002020;
        end else if(OSD_ENABLE && logo_inside2 && logo_pixel2[24])O_hdmi_data<=logo_pixel2[23:0];
        else if(OSD_ENABLE && text_on2)O_hdmi_data<=24'hfff200;
        else O_hdmi_data<=rgb2;
      end
endmodule
