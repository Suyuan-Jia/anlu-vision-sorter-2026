// Four-pixel-per-clock streaming Canny edge detector.
//
// External interface and signal polarity are kept identical to the original
// edge_detect_96 module:
//   I_rst_n            active-low asynchronous reset
//   I_tuser            active-high start of frame
//   I_tlast            active-high end of line
//   I_tvalid/I_tready  active-high transfer handshake
//
// The implementation keeps all four input pixels independent.  Each 3x3
// stage operates on a full-resolution pixel window; there is no four-pixel
// averaging and no replication of one edge decision across four pixels.
module edge_detect_96 #(
    parameter IMG_WIDTH  = 1280,
    parameter IMG_HEIGHT = 720,
    parameter EDGE_TH    = 11'd120,
    parameter integer EXPORT_GRAY = 0,
    parameter integer PIPELINE_SOBEL = 0,
    parameter [11:0] CARTOON_EDGE_TH = 12'd100
)(
    input               I_clk,
    input               I_rst_n,
    input               I_tlast,
    input               I_tuser,
    input      [95:0]   I_tdata,
    input               I_tvalid,
    input               I_tready,
    output reg          O_tlast,
    output reg          O_tuser,
    output reg [95:0]   O_tdata,
    output reg          O_tvalid,
    output              O_tready,
    output wire [31:0] O_gray_data, // raw BT.601 luminance, aligned to O_tvalid
    output      [3:0]    O_cartoon_edge
);

localparam integer WORDS_PER_LINE = (IMG_WIDTH + 3) >> 2;
localparam integer EDGE_LATENCY = PIPELINE_SOBEL ? 6 : 4;

// The whole pipeline freezes when the downstream is not ready.
assign O_tready = I_tready;

wire [23:0] in_pix0 = I_tdata[95:72];
wire [23:0] in_pix1 = I_tdata[71:48];
wire [23:0] in_pix2 = I_tdata[47:24];
wire [23:0] in_pix3 = I_tdata[23:0];

// BT.601 fixed-point luminance.
// Constant coefficients are synthesised as constant multipliers/shift-adders.
wire [15:0] lum0_sum =
    in_pix0[23:16] * 8'd77 +
    in_pix0[15:8]  * 8'd150 +
    in_pix0[7:0]   * 8'd29;

wire [15:0] lum1_sum =
    in_pix1[23:16] * 8'd77 +
    in_pix1[15:8]  * 8'd150 +
    in_pix1[7:0]   * 8'd29;

wire [15:0] lum2_sum =
    in_pix2[23:16] * 8'd77 +
    in_pix2[15:8]  * 8'd150 +
    in_pix2[7:0]   * 8'd29;

wire [15:0] lum3_sum =
    in_pix3[23:16] * 8'd77 +
    in_pix3[15:8]  * 8'd150 +
    in_pix3[7:0]   * 8'd29;

wire [31:0] gray_data = {
    lum0_sum[15:8],
    lum1_sum[15:8],
    lum2_sum[15:8],
    lum3_sum[15:8]
};

// Reuse the luminance arithmetic above. Four CE-controlled registers mirror
// Gaussian/Sobel/NMS/hysteresis latency, including bubbles and stalls.
// Disabled in other users of this shared detector; no extra arithmetic/RAM.
generate if(EXPORT_GRAY) begin: g_gray_tap
    reg [31:0] delay_gray [0:EDGE_LATENCY-1];
    integer g;
    always @(posedge I_clk or negedge I_rst_n)
      if(!I_rst_n) begin
        for(g=0;g<EDGE_LATENCY;g=g+1) delay_gray[g]<=0;
      end else if(I_tready) begin
        delay_gray[0]<=gray_data;
        for(g=1;g<EDGE_LATENCY;g=g+1) delay_gray[g]<=delay_gray[g-1];
      end
    assign O_gray_data=delay_gray[EDGE_LATENCY-1];
end else begin: g_no_gray_tap
    assign O_gray_data=0;
end endgenerate

wire [31:0] gauss_data;
wire        gauss_valid;
wire        gauss_user;
wire        gauss_last;

wire [47:0] sobel_mag;
wire [7:0]  sobel_dir;
wire        sobel_valid;
wire        sobel_user;
wire        sobel_last;

wire [47:0] nms_data;
wire        nms_valid;
wire        nms_user;
wire        nms_last;

wire [3:0]  hyst_edge;
wire [3:0]  cartoon_edge;
wire        hyst_valid;
wire        hyst_user;
wire        hyst_last;

canny_gaussian_4px #(
    .WORDS_PER_LINE(WORDS_PER_LINE),
    .IMG_HEIGHT    (IMG_HEIGHT)
) u_canny_gaussian (
    .clk    (I_clk),
    .rst_n  (I_rst_n),
    .ce     (I_tready),
    .i_data (gray_data),
    .i_valid(I_tvalid),
    .i_user (I_tuser),
    .i_last (I_tlast),
    .o_data (gauss_data),
    .o_valid(gauss_valid),
    .o_user (gauss_user),
    .o_last (gauss_last)
);

generate if(PIPELINE_SOBEL)begin:g_sobel_pipeline
canny_sobel_pipelined_4px #(
    .WORDS_PER_LINE(WORDS_PER_LINE),
    .IMG_HEIGHT    (IMG_HEIGHT)
) u_canny_sobel (
    .clk    (I_clk),
    .rst_n  (I_rst_n),
    .ce     (I_tready),
    .i_data (gauss_data),
    .i_valid(gauss_valid),
    .i_user (gauss_user),
    .i_last (gauss_last),
    .o_mag  (sobel_mag),
    .o_dir  (sobel_dir),
    .o_valid(sobel_valid),
    .o_user (sobel_user),
    .o_last (sobel_last)
);
end else begin:g_sobel_legacy
canny_sobel_4px #(
    .WORDS_PER_LINE(WORDS_PER_LINE),
    .IMG_HEIGHT    (IMG_HEIGHT)
) u_canny_sobel (
    .clk    (I_clk),
    .rst_n  (I_rst_n),
    .ce     (I_tready),
    .i_data (gauss_data),
    .i_valid(gauss_valid),
    .i_user (gauss_user),
    .i_last (gauss_last),
    .o_mag  (sobel_mag),
    .o_dir  (sobel_dir),
    .o_valid(sobel_valid),
    .o_user (sobel_user),
    .o_last (sobel_last)
);
end endgenerate

canny_nms_4px #(
    .WORDS_PER_LINE(WORDS_PER_LINE),
    .IMG_HEIGHT    (IMG_HEIGHT)
) u_canny_nms (
    .clk    (I_clk),
    .rst_n  (I_rst_n),
    .ce     (I_tready),
    .i_mag  (sobel_mag),
    .i_dir  (sobel_dir),
    .i_valid(sobel_valid),
    .i_user (sobel_user),
    .i_last (sobel_last),
    .o_nms  (nms_data),
    .o_valid(nms_valid),
    .o_user (nms_user),
    .o_last (nms_last)
);

canny_hysteresis_4px #(
    .WORDS_PER_LINE(WORDS_PER_LINE),
    .IMG_HEIGHT    (IMG_HEIGHT),
    // Tune the two thresholds independently.
    .HIGH_TH       ({1'b0, EDGE_TH} >> 1),
    .LOW_TH        ({1'b0, EDGE_TH} >> 2),
    .CARTOON_EDGE_TH(CARTOON_EDGE_TH)
) u_canny_hysteresis (
    .clk    (I_clk),
    .rst_n  (I_rst_n),
    .ce     (I_tready),
    .i_nms  (nms_data),
    .i_valid(nms_valid),
    .i_user (nms_user),
    .i_last (nms_last),
    .o_edge (hyst_edge),
    .o_valid(hyst_valid),
    .o_user (hyst_user),
    .o_last (hyst_last),
    .o_cartoon_edge(cartoon_edge)
);

// Output hysteresis directly: do not expand isolated edge pixels.
// Four causal 3x3 stages require eight preceding rows/columns. Suppress
// padding-induced edges while retaining raster timing and spatial offset.
// Masks follow the same four register stages, CE stalls and valid bubbles.
reg [10:0] mask_x, mask_y;
wire [10:0] mask_use_x = I_tuser ? 11'd0 : mask_x;
wire [10:0] mask_use_y = I_tuser ? 11'd0 : mask_y;
reg [3:0] region_mask [0:EDGE_LATENCY-1];
integer mask_stage, mask_lane;
always @(posedge I_clk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        mask_x <= 11'd0;
        mask_y <= 11'd0;
        for (mask_stage = 0; mask_stage < EDGE_LATENCY; mask_stage = mask_stage + 1)
            region_mask[mask_stage] <= 4'd0;
    end else if (I_tready) begin
        for (mask_stage = 1; mask_stage < EDGE_LATENCY; mask_stage = mask_stage + 1)
            region_mask[mask_stage] <= region_mask[mask_stage-1];
        for (mask_lane = 0; mask_lane < 4; mask_lane = mask_lane + 1)
            region_mask[0][3-mask_lane] <= I_tvalid &&
                (mask_use_y >= 8) && (mask_use_x * 4 + mask_lane >= 8) &&
                (mask_use_x * 4 + mask_lane < IMG_WIDTH);
        if (I_tvalid) begin
            if (I_tlast || mask_use_x == WORDS_PER_LINE - 1) begin
                mask_x <= 11'd0;
                mask_y <= (mask_use_y == IMG_HEIGHT - 1) ? 11'd0 : mask_use_y + 1'b1;
            end else begin
                mask_x <= mask_use_x + 1'b1;
                mask_y <= mask_use_y;
            end
        end
    end
end
wire [3:0] visible_edge = hyst_edge & region_mask[EDGE_LATENCY-1];
assign O_cartoon_edge = cartoon_edge & region_mask[EDGE_LATENCY-1];

always @(*) begin
    O_tlast  = hyst_last;
    O_tuser  = hyst_user;
    O_tvalid = hyst_valid;
    O_tdata  = {
        visible_edge[3] ? 24'hFFFFFF : 24'h000000,
        visible_edge[2] ? 24'hFFFFFF : 24'h000000,
        visible_edge[1] ? 24'hFFFFFF : 24'h000000,
        visible_edge[0] ? 24'hFFFFFF : 24'h000000
    };
end

endmodule


// -----------------------------------------------------------------------------
// 3x3 Gaussian filter, four independent pixels per accepted clock.
//
// Output pixels are shifted by one pixel and one line.  This causal mapping
// permits one output word for every input word without requiring a fifth
// pixel from the next clock beat.
// -----------------------------------------------------------------------------
module canny_gaussian_4px #(
    parameter integer WORDS_PER_LINE = 320,
    parameter integer IMG_HEIGHT     = 720
)(
    input             clk,
    input             rst_n,
    input             ce,
    input      [31:0] i_data,
    input             i_valid,
    input             i_user,
    input             i_last,
    output reg [31:0] o_data,
    output reg        o_valid,
    output reg        o_user,
    output reg        o_last
);

(* ram_style = "block" *) reg [31:0] line1 [0:WORDS_PER_LINE-1];
(* ram_style = "block" *) reg [31:0] line2 [0:WORDS_PER_LINE-1];

reg [10:0] word_x;
reg [10:0] line_y;
reg [10:0] use_x;
reg [10:0] use_y;

reg [31:0] top_word;
reg [31:0] mid_word;

reg [15:0] top_prev;
reg [15:0] mid_prev;
reg [15:0] bot_prev;

reg [7:0] t0, t1, t2, t3;
reg [7:0] m0, m1, m2, m3;
reg [7:0] b0, b1, b2, b3;

reg [7:0] q0, q1, q2, q3;

function [7:0] gaussian3x3;
    input [7:0] a00, a01, a02;
    input [7:0] a10, a11, a12;
    input [7:0] a20, a21, a22;
    reg [11:0] sum;
begin
    sum =
        {4'd0, a00} +
        ({4'd0, a01} << 1) +
        {4'd0, a02} +
        ({4'd0, a10} << 1) +
        ({4'd0, a11} << 2) +
        ({4'd0, a12} << 1) +
        {4'd0, a20} +
        ({4'd0, a21} << 1) +
        {4'd0, a22};

    gaussian3x3 = sum[11:4];
end
endfunction

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        word_x   <= 11'd0;
        line_y   <= 11'd0;
        top_prev <= 16'd0;
        mid_prev <= 16'd0;
        bot_prev <= 16'd0;
        o_data   <= 32'd0;
        o_valid  <= 1'b0;
        o_user   <= 1'b0;
        o_last   <= 1'b0;
    end
    else if (ce) begin
        o_valid <= i_valid;
        o_user  <= i_user;
        o_last  <= i_last;

        if (i_valid) begin
            if (i_user) begin
                use_x    = 11'd0;
                use_y    = 11'd0;
                top_prev = 16'd0;
                mid_prev = 16'd0;
                bot_prev = 16'd0;
            end
            else begin
                use_x = word_x;
                use_y = line_y;
            end

            top_word = line2[use_x];
            mid_word = line1[use_x];

            t0 = top_word[31:24];
            t1 = top_word[23:16];
            t2 = top_word[15:8];
            t3 = top_word[7:0];

            m0 = mid_word[31:24];
            m1 = mid_word[23:16];
            m2 = mid_word[15:8];
            m3 = mid_word[7:0];

            b0 = i_data[31:24];
            b1 = i_data[23:16];
            b2 = i_data[15:8];
            b3 = i_data[7:0];

            q0 = gaussian3x3(
                top_prev[15:8], top_prev[7:0], t0,
                mid_prev [15:8], mid_prev [7:0], m0,
                bot_prev [15:8], bot_prev [7:0], b0
            );

            q1 = gaussian3x3(
                top_prev[7:0], t0, t1,
                mid_prev [7:0], m0, m1,
                bot_prev [7:0], b0, b1
            );

            q2 = gaussian3x3(
                t0, t1, t2,
                m0, m1, m2,
                b0, b1, b2
            );

            q3 = gaussian3x3(
                t1, t2, t3,
                m1, m2, m3,
                b1, b2, b3
            );

            if (use_y < 11'd2) begin
                o_data <= 32'd0;
            end
            else if (use_x == 11'd0) begin
                o_data <= {8'd0, q1, q2, q3};
            end
            else begin
                o_data <= {q0, q1, q2, q3};
            end

            line2[use_x] <= mid_word;
            line1[use_x] <= i_data;

            top_prev = top_word[15:0];
            mid_prev = mid_word[15:0];
            bot_prev = i_data[15:0];

            if (i_last || (use_x == WORDS_PER_LINE - 1)) begin
                word_x   <= 11'd0;
                top_prev = 16'd0;
                mid_prev = 16'd0;
                bot_prev = 16'd0;

                if (use_y == IMG_HEIGHT - 1)
                    line_y <= 11'd0;
                else
                    line_y <= use_y + 1'b1;
            end
            else begin
                word_x <= use_x + 1'b1;
                line_y <= use_y;
            end
        end
        else begin
            o_data <= 32'd0;
        end
    end
end

endmodule


// -----------------------------------------------------------------------------
// Four-pixel Sobel magnitude and four-direction quantisation.
// -----------------------------------------------------------------------------
module canny_sobel_4px #(
    parameter integer WORDS_PER_LINE = 320,
    parameter integer IMG_HEIGHT     = 720
)(
    input             clk,
    input             rst_n,
    input             ce,
    input      [31:0] i_data,
    input             i_valid,
    input             i_user,
    input             i_last,
    output reg [47:0] o_mag,
    output reg [7:0]  o_dir,
    output reg        o_valid,
    output reg        o_user,
    output reg        o_last
);

(* ram_style = "block" *) reg [31:0] line1 [0:WORDS_PER_LINE-1];
(* ram_style = "block" *) reg [31:0] line2 [0:WORDS_PER_LINE-1];

reg [10:0] word_x;
reg [10:0] line_y;
reg [10:0] use_x;
reg [10:0] use_y;

reg [31:0] top_word;
reg [31:0] mid_word;

reg [15:0] top_prev;
reg [15:0] mid_prev;
reg [15:0] bot_prev;

reg [7:0] t0, t1, t2, t3;
reg [7:0] m0, m1, m2, m3;
reg [7:0] b0, b1, b2, b3;

reg [13:0] s0, s1, s2, s3;

function [13:0] sobel3x3;
    input [7:0] a00, a01, a02;
    input [7:0] a10, a11, a12;
    input [7:0] a20, a21, a22;

    reg signed [12:0] gx;
    reg signed [12:0] gy;

    // Explicitly extend every Sobel operand to 13 bits before shifting.
    // The center pixel a11 is not used by the Sobel operator.
    reg signed [12:0] sx00, sx01, sx02;
    reg signed [12:0] sx10,       sx12;
    reg signed [12:0] sx20, sx21, sx22;

    reg [12:0] ax;
    reg [12:0] ay;
    reg [12:0] mag;
    reg [15:0] ax5;
    reg [15:0] ay5;
    reg [15:0] ax2;
    reg [15:0] ay2;
    reg [1:0] dir;
begin
    // 5 leading zero bits plus the 8-bit pixel form a 13-bit
    // positive signed operand.
    sx00 = $signed({5'b00000, a00});
    sx01 = $signed({5'b00000, a01});
    sx02 = $signed({5'b00000, a02});
    sx10 = $signed({5'b00000, a10});
    sx12 = $signed({5'b00000, a12});
    sx20 = $signed({5'b00000, a20});
    sx21 = $signed({5'b00000, a21});
    sx22 = $signed({5'b00000, a22});

    gx =
        -sx00 +
         sx02 -
        (sx10 <<< 1) +
        (sx12 <<< 1) -
         sx20 +
         sx22;

    gy =
        -sx00 -
        (sx01 <<< 1) -
         sx02 +
         sx20 +
        (sx21 <<< 1) +
         sx22;

    ax = gx[12] ? (~gx + 13'sd1) : gx;
    ay = gy[12] ? (~gy + 13'sd1) : gy;

    mag = ax + ay;

    ax5 = ({3'd0, ax} << 2) + {3'd0, ax};
    ay5 = ({3'd0, ay} << 2) + {3'd0, ay};
    ax2 = {3'd0, ax} << 1;
    ay2 = {3'd0, ay} << 1;

    // tan(22.5 degrees) is approximately 0.414.
    // 5*minor <= 2*major gives an inexpensive 0.4 boundary.
    if (ay5 <= ax2)
        dir = 2'd0;             // compare west/east
    else if (ax5 <= ay2)
        dir = 2'd2;             // compare north/south
    else if (gx[12] == gy[12])
        dir = 2'd3;             // compare north-west/south-east
    else
        dir = 2'd1;             // compare north-east/south-west

    sobel3x3 = {dir, mag[11:0]};
end
endfunction

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        word_x   <= 11'd0;
        line_y   <= 11'd0;
        top_prev <= 16'd0;
        mid_prev <= 16'd0;
        bot_prev <= 16'd0;
        o_mag    <= 48'd0;
        o_dir    <= 8'd0;
        o_valid  <= 1'b0;
        o_user   <= 1'b0;
        o_last   <= 1'b0;
    end
    else if (ce) begin
        o_valid <= i_valid;
        o_user  <= i_user;
        o_last  <= i_last;

        if (i_valid) begin
            if (i_user) begin
                use_x    = 11'd0;
                use_y    = 11'd0;
                top_prev = 16'd0;
                mid_prev = 16'd0;
                bot_prev = 16'd0;
            end
            else begin
                use_x = word_x;
                use_y = line_y;
            end

            top_word = line2[use_x];
            mid_word = line1[use_x];

            t0 = top_word[31:24];
            t1 = top_word[23:16];
            t2 = top_word[15:8];
            t3 = top_word[7:0];

            m0 = mid_word[31:24];
            m1 = mid_word[23:16];
            m2 = mid_word[15:8];
            m3 = mid_word[7:0];

            b0 = i_data[31:24];
            b1 = i_data[23:16];
            b2 = i_data[15:8];
            b3 = i_data[7:0];

            s0 = sobel3x3(
                top_prev[15:8], top_prev[7:0], t0,
                mid_prev [15:8], mid_prev [7:0], m0,
                bot_prev [15:8], bot_prev [7:0], b0
            );

            s1 = sobel3x3(
                top_prev[7:0], t0, t1,
                mid_prev [7:0], m0, m1,
                bot_prev [7:0], b0, b1
            );

            s2 = sobel3x3(
                t0, t1, t2,
                m0, m1, m2,
                b0, b1, b2
            );

            s3 = sobel3x3(
                t1, t2, t3,
                m1, m2, m3,
                b1, b2, b3
            );

            if (use_y < 11'd2) begin
                o_mag <= 48'd0;
                o_dir <= 8'd0;
            end
            else if (use_x == 11'd0) begin
                o_mag <= {12'd0, s1[11:0], s2[11:0], s3[11:0]};
                o_dir <= {2'd0,  s1[13:12], s2[13:12], s3[13:12]};
            end
            else begin
                o_mag <= {s0[11:0], s1[11:0], s2[11:0], s3[11:0]};
                o_dir <= {s0[13:12], s1[13:12], s2[13:12], s3[13:12]};
            end

            line2[use_x] <= mid_word;
            line1[use_x] <= i_data;

            top_prev = top_word[15:0];
            mid_prev = mid_word[15:0];
            bot_prev = i_data[15:0];

            if (i_last || (use_x == WORDS_PER_LINE - 1)) begin
                word_x   <= 11'd0;
                top_prev = 16'd0;
                mid_prev = 16'd0;
                bot_prev = 16'd0;

                if (use_y == IMG_HEIGHT - 1)
                    line_y <= 11'd0;
                else
                    line_y <= use_y + 1'b1;
            end
            else begin
                word_x <= use_x + 1'b1;
                line_y <= use_y;
            end
        end
        else begin
            o_mag <= 48'd0;
            o_dir <= 8'd0;
        end
    end
end

endmodule


// -----------------------------------------------------------------------------
// Directional non-maximum suppression, four pixels per accepted clock.
// -----------------------------------------------------------------------------
module canny_nms_4px #(
    parameter integer WORDS_PER_LINE = 320,
    parameter integer IMG_HEIGHT     = 720
)(
    input             clk,
    input             rst_n,
    input             ce,
    input      [47:0] i_mag,
    input      [7:0]  i_dir,
    input             i_valid,
    input             i_user,
    input             i_last,
    output reg [47:0] o_nms,
    output reg        o_valid,
    output reg        o_user,
    output reg        o_last
);

(* ram_style = "block" *) reg [47:0] mag_line1 [0:WORDS_PER_LINE-1];
(* ram_style = "block" *) reg [47:0] mag_line2 [0:WORDS_PER_LINE-1];
(* ram_style = "block" *) reg [7:0]  dir_line1 [0:WORDS_PER_LINE-1];
(* ram_style = "block" *) reg [7:0]  dir_line2 [0:WORDS_PER_LINE-1];

reg [10:0] word_x;
reg [10:0] line_y;
reg [10:0] use_x;
reg [10:0] use_y;

reg [47:0] top_mag_word;
reg [47:0] mid_mag_word;
reg [7:0]  top_dir_word;
reg [7:0]  mid_dir_word;

reg [23:0] top_mag_prev;
reg [23:0] mid_mag_prev;
reg [23:0] bot_mag_prev;

reg [3:0] top_dir_prev;
reg [3:0] mid_dir_prev;
reg [3:0] bot_dir_prev;

reg [11:0] tm0, tm1, tm2, tm3;
reg [11:0] mm0, mm1, mm2, mm3;
reg [11:0] bm0, bm1, bm2, bm3;

reg [1:0] md0, md1, md2, md3;
reg [11:0] n0, n1, n2, n3;

function [11:0] suppress;
    input [11:0] a00, a01, a02;
    input [11:0] a10, a11, a12;
    input [11:0] a20, a21, a22;
    input [1:0]  dir;
begin
    suppress = 12'd0;

    if (a11 != 12'd0) begin
        case (dir)
            2'd0:
                if ((a11 >= a10) && (a11 >= a12))
                    suppress = a11;

            2'd1:
                if ((a11 >= a02) && (a11 >= a20))
                    suppress = a11;

            2'd2:
                if ((a11 >= a01) && (a11 >= a21))
                    suppress = a11;

            2'd3:
                if ((a11 >= a00) && (a11 >= a22))
                    suppress = a11;

            default:
                suppress = 12'd0;
        endcase
    end
end
endfunction

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        word_x       <= 11'd0;
        line_y       <= 11'd0;
        top_mag_prev <= 24'd0;
        mid_mag_prev <= 24'd0;
        bot_mag_prev <= 24'd0;
        top_dir_prev <= 4'd0;
        mid_dir_prev <= 4'd0;
        bot_dir_prev <= 4'd0;
        o_nms        <= 48'd0;
        o_valid      <= 1'b0;
        o_user       <= 1'b0;
        o_last       <= 1'b0;
    end
    else if (ce) begin
        o_valid <= i_valid;
        o_user  <= i_user;
        o_last  <= i_last;

        if (i_valid) begin
            if (i_user) begin
                use_x        = 11'd0;
                use_y        = 11'd0;
                top_mag_prev = 24'd0;
                mid_mag_prev = 24'd0;
                bot_mag_prev = 24'd0;
                top_dir_prev = 4'd0;
                mid_dir_prev = 4'd0;
                bot_dir_prev = 4'd0;
            end
            else begin
                use_x = word_x;
                use_y = line_y;
            end

            top_mag_word = mag_line2[use_x];
            mid_mag_word = mag_line1[use_x];
            top_dir_word = dir_line2[use_x];
            mid_dir_word = dir_line1[use_x];

            tm0 = top_mag_word[47:36];
            tm1 = top_mag_word[35:24];
            tm2 = top_mag_word[23:12];
            tm3 = top_mag_word[11:0];

            mm0 = mid_mag_word[47:36];
            mm1 = mid_mag_word[35:24];
            mm2 = mid_mag_word[23:12];
            mm3 = mid_mag_word[11:0];

            bm0 = i_mag[47:36];
            bm1 = i_mag[35:24];
            bm2 = i_mag[23:12];
            bm3 = i_mag[11:0];

            md0 = mid_dir_word[7:6];
            md1 = mid_dir_word[5:4];
            md2 = mid_dir_word[3:2];
            md3 = mid_dir_word[1:0];

            n0 = suppress(
                top_mag_prev[23:12], top_mag_prev[11:0], tm0,
                mid_mag_prev[23:12], mid_mag_prev[11:0], mm0,
                bot_mag_prev[23:12], bot_mag_prev[11:0], bm0,
                mid_dir_prev[1:0]
            );

            n1 = suppress(
                top_mag_prev[11:0], tm0, tm1,
                mid_mag_prev[11:0], mm0, mm1,
                bot_mag_prev[11:0], bm0, bm1,
                md0
            );

            n2 = suppress(
                tm0, tm1, tm2,
                mm0, mm1, mm2,
                bm0, bm1, bm2,
                md1
            );

            n3 = suppress(
                tm1, tm2, tm3,
                mm1, mm2, mm3,
                bm1, bm2, bm3,
                md2
            );

            if (use_y < 11'd2)
                o_nms <= 48'd0;
            else if (use_x == 11'd0)
                o_nms <= {12'd0, n1, n2, n3};
            else
                o_nms <= {n0, n1, n2, n3};

            mag_line2[use_x] <= mid_mag_word;
            mag_line1[use_x] <= i_mag;
            dir_line2[use_x] <= mid_dir_word;
            dir_line1[use_x] <= i_dir;

            top_mag_prev = top_mag_word[23:0];
            mid_mag_prev = mid_mag_word[23:0];
            bot_mag_prev = i_mag[23:0];

            top_dir_prev = top_dir_word[3:0];
            mid_dir_prev = mid_dir_word[3:0];
            bot_dir_prev = i_dir[3:0];

            if (i_last || (use_x == WORDS_PER_LINE - 1)) begin
                word_x       <= 11'd0;
                top_mag_prev = 24'd0;
                mid_mag_prev = 24'd0;
                bot_mag_prev = 24'd0;
                top_dir_prev = 4'd0;
                mid_dir_prev = 4'd0;
                bot_dir_prev = 4'd0;

                if (use_y == IMG_HEIGHT - 1)
                    line_y <= 11'd0;
                else
                    line_y <= use_y + 1'b1;
            end
            else begin
                word_x <= use_x + 1'b1;
                line_y <= use_y;
            end
        end
        else begin
            o_nms <= 48'd0;
        end
    end
end

endmodule


// -----------------------------------------------------------------------------
// Streaming bounded hysteresis.
//
// Full recursive edge tracing requires a frame store and multiple passes.
// This hardware-friendly version retains a weak centre pixel when any pixel
// in its 3x3 neighbourhood is strong.
// -----------------------------------------------------------------------------
module canny_hysteresis_4px #(
    parameter integer WORDS_PER_LINE = 320,
    parameter integer IMG_HEIGHT     = 720,
    parameter [11:0]  HIGH_TH        = 12'd40,
    parameter [11:0]  LOW_TH         = 12'd20,
    parameter [11:0]  CARTOON_EDGE_TH = 12'd100
)(
    input             clk,
    input             rst_n,
    input             ce,
    input      [47:0] i_nms,
    input             i_valid,
    input             i_user,
    input             i_last,
    output reg [3:0]  o_edge,
    output reg        o_valid,
    output reg        o_user,
    output reg        o_last,
    output reg [3:0]  o_cartoon_edge
);

// Cartoon uses only strong NMS centres. Reuse this stage's existing window
// to retain exactly the same four-row/four-column offset as normal Canny.
// Clamp to HIGH_TH so cartoon edges are always a subset of normal edges.
localparam [11:0] CARTOON_TH = (CARTOON_EDGE_TH < HIGH_TH)
    ? HIGH_TH : CARTOON_EDGE_TH;

(* ram_style = "block" *) reg [47:0] line1 [0:WORDS_PER_LINE-1];
(* ram_style = "block" *) reg [47:0] line2 [0:WORDS_PER_LINE-1];

reg [10:0] word_x;
reg [10:0] line_y;
reg [10:0] use_x;
reg [10:0] use_y;

reg [47:0] top_word;
reg [47:0] mid_word;

reg [23:0] top_prev;
reg [23:0] mid_prev;
reg [23:0] bot_prev;

reg [11:0] t0, t1, t2, t3;
reg [11:0] m0, m1, m2, m3;
reg [11:0] b0, b1, b2, b3;

reg e0, e1, e2, e3;

function hysteresis3x3;
    input [11:0] a00, a01, a02;
    input [11:0] a10, a11, a12;
    input [11:0] a20, a21, a22;
    reg neighbour_strong;
begin
    neighbour_strong =
        (a00 >= HIGH_TH) ||
        (a01 >= HIGH_TH) ||
        (a02 >= HIGH_TH) ||
        (a10 >= HIGH_TH) ||
        (a12 >= HIGH_TH) ||
        (a20 >= HIGH_TH) ||
        (a21 >= HIGH_TH) ||
        (a22 >= HIGH_TH);

    hysteresis3x3 =
        (a11 >= HIGH_TH) ||
        ((a11 >= LOW_TH) && neighbour_strong);
end
endfunction

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        word_x   <= 11'd0;
        line_y   <= 11'd0;
        top_prev <= 24'd0;
        mid_prev <= 24'd0;
        bot_prev <= 24'd0;
        o_edge   <= 4'd0;
        o_cartoon_edge <= 4'd0;
        o_valid  <= 1'b0;
        o_user   <= 1'b0;
        o_last   <= 1'b0;
    end
    else if (ce) begin
        o_valid <= i_valid;
        o_user  <= i_user;
        o_last  <= i_last;

        if (i_valid) begin
            if (i_user) begin
                use_x    = 11'd0;
                use_y    = 11'd0;
                top_prev = 24'd0;
                mid_prev = 24'd0;
                bot_prev = 24'd0;
            end
            else begin
                use_x = word_x;
                use_y = line_y;
            end

            top_word = line2[use_x];
            mid_word = line1[use_x];

            t0 = top_word[47:36];
            t1 = top_word[35:24];
            t2 = top_word[23:12];
            t3 = top_word[11:0];

            m0 = mid_word[47:36];
            m1 = mid_word[35:24];
            m2 = mid_word[23:12];
            m3 = mid_word[11:0];

            b0 = i_nms[47:36];
            b1 = i_nms[35:24];
            b2 = i_nms[23:12];
            b3 = i_nms[11:0];

            e0 = hysteresis3x3(
                top_prev[23:12], top_prev[11:0], t0,
                mid_prev [23:12], mid_prev [11:0], m0,
                bot_prev [23:12], bot_prev [11:0], b0
            );

            e1 = hysteresis3x3(
                top_prev[11:0], t0, t1,
                mid_prev [11:0], m0, m1,
                bot_prev [11:0], b0, b1
            );

            e2 = hysteresis3x3(
                t0, t1, t2,
                m0, m1, m2,
                b0, b1, b2
            );

            e3 = hysteresis3x3(
                t1, t2, t3,
                m1, m2, m3,
                b1, b2, b3
            );

            if (use_y < 11'd2)
                o_edge <= 4'd0;
            else if (use_x == 11'd0)
                o_edge <= {1'b0, e1, e2, e3};
            else
                o_edge <= {e0, e1, e2, e3};

            // The four window centres are previous-word tail, m0, m1, m2.
            // Thresholding i_nms directly here would shift the black lines.
            if (use_y < 11'd2)
                o_cartoon_edge <= 4'd0;
            else
                o_cartoon_edge <= {
                    (use_x != 0) && (mid_prev[11:0] >= CARTOON_TH),
                    (m0 >= CARTOON_TH), (m1 >= CARTOON_TH), (m2 >= CARTOON_TH)
                };

            line2[use_x] <= mid_word;
            line1[use_x] <= i_nms;

            top_prev = top_word[23:0];
            mid_prev = mid_word[23:0];
            bot_prev = i_nms[23:0];

            if (i_last || (use_x == WORDS_PER_LINE - 1)) begin
                word_x   <= 11'd0;
                top_prev = 24'd0;
                mid_prev = 24'd0;
                bot_prev = 24'd0;

                if (use_y == IMG_HEIGHT - 1)
                    line_y <= 11'd0;
                else
                    line_y <= use_y + 1'b1;
            end
            else begin
                word_x <= use_x + 1'b1;
                line_y <= use_y;
            end
        end
        else begin
            o_edge <= 4'd0;
            o_cartoon_edge <= 4'd0;
        end
    end
end

endmodule


// Additional timing-safe implementation; legacy mode remains four cycles.
module canny_sobel_pipelined_4px #(
    parameter integer WORDS_PER_LINE = 320,
    parameter integer IMG_HEIGHT     = 720
)(
    input             clk,
    input             rst_n,
    input             ce,
    input      [31:0] i_data,
    input             i_valid,
    input             i_user,
    input             i_last,
    output reg [47:0] o_mag,
    output reg [7:0]  o_dir,
    output reg        o_valid,
    output reg        o_user,
    output reg        o_last
);

(* ram_style = "block" *) reg [31:0] line1 [0:WORDS_PER_LINE-1];
(* ram_style = "block" *) reg [31:0] line2 [0:WORDS_PER_LINE-1];

reg [10:0] word_x;
reg [10:0] line_y;
reg [10:0] use_x;
reg [10:0] use_y;

reg [31:0] top_word;
reg [31:0] mid_word;

reg [15:0] top_prev;
reg [15:0] mid_prev;
reg [15:0] bot_prev;

reg [7:0] t0, t1, t2, t3;
reg [7:0] m0, m1, m2, m3;
reg [7:0] b0, b1, b2, b3;

reg [25:0] s0, s1, s2, s3;
reg [25:0] gradients[0:3];
reg [12:0] abs_x[0:3],abs_y[0:3];
reg [3:0] same_sign;
reg grad_valid,grad_user,grad_last,abs_valid,abs_user,abs_last;
integer lane;

function [25:0] sobel_grad;
 input [7:0] a00,a01,a02,a10,a11,a12,a20,a21,a22;
 reg signed [12:0] x0,x1,x2,x3,x4,x5,y0,y1,gx,gy;
 begin
  x0=$signed({5'd0,a02})+$signed({5'd0,a22});
  x1=$signed({5'd0,a00})+$signed({5'd0,a20});
  x2=$signed({5'd0,a12})-$signed({5'd0,a10});
  y0=$signed({5'd0,a20})+$signed({5'd0,a22});
  y1=$signed({5'd0,a00})+$signed({5'd0,a02});
  x3=$signed({5'd0,a21})-$signed({5'd0,a01});
  gx=(x0-x1)+(x2<<<1);gy=(y0-y1)+(x3<<<1);
  sobel_grad={gx,gy};
 end
endfunction
function [1:0] quant_dir;
 input [12:0] ax,ay;input same;
 reg [15:0] ax5,ay5,ax2,ay2;
 begin
  ax5=({3'd0,ax}<<2)+{3'd0,ax};ay5=({3'd0,ay}<<2)+{3'd0,ay};
  ax2={3'd0,ax}<<1;ay2={3'd0,ay}<<1;
  if(ay5<=ax2)quant_dir=0;else if(ax5<=ay2)quant_dir=2;
  else quant_dir=same ? 3 : 1;
 end
endfunction
// Three CE-controlled stages: line-buffer/gradient, absolute value, direction.
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
  abs_valid<=0;abs_user<=0;abs_last<=0;same_sign<=0;
  o_mag<=0;o_dir<=0;o_valid<=0;o_user<=0;o_last<=0;
  for(lane=0;lane<4;lane=lane+1)begin abs_x[lane]<=0;abs_y[lane]<=0;end
 end else if(ce)begin
  abs_valid<=grad_valid;abs_user<=grad_user;abs_last<=grad_last;
  o_valid<=abs_valid;o_user<=abs_user;o_last<=abs_last;
  for(lane=0;lane<4;lane=lane+1)begin
   abs_x[lane]<=gradients[lane][25] ? (~gradients[lane][25:13]+13'd1) : gradients[lane][25:13];
   abs_y[lane]<=gradients[lane][12] ? (~gradients[lane][12:0]+13'd1) : gradients[lane][12:0];
   same_sign[lane]<=gradients[lane][25]==gradients[lane][12];
   o_mag[47-lane*12-:12]<=abs_valid ? abs_x[lane]+abs_y[lane] : 12'd0;
   o_dir[7-lane*2-:2]<=abs_valid ? quant_dir(abs_x[lane],abs_y[lane],same_sign[lane]) : 2'd0;
  end
 end
end


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        word_x   <= 11'd0;
        line_y   <= 11'd0;
        top_prev <= 16'd0;
        mid_prev <= 16'd0;
        bot_prev <= 16'd0;
        gradients[0]<=0;gradients[1]<=0;gradients[2]<=0;gradients[3]<=0;
        grad_valid  <= 1'b0;
        grad_user   <= 1'b0;
        grad_last   <= 1'b0;
    end
    else if (ce) begin
        grad_valid <= i_valid;
        grad_user  <= i_user;
        grad_last  <= i_last;

        if (i_valid) begin
            if (i_user) begin
                use_x    = 11'd0;
                use_y    = 11'd0;
                top_prev = 16'd0;
                mid_prev = 16'd0;
                bot_prev = 16'd0;
            end
            else begin
                use_x = word_x;
                use_y = line_y;
            end

            top_word = line2[use_x];
            mid_word = line1[use_x];

            t0 = top_word[31:24];
            t1 = top_word[23:16];
            t2 = top_word[15:8];
            t3 = top_word[7:0];

            m0 = mid_word[31:24];
            m1 = mid_word[23:16];
            m2 = mid_word[15:8];
            m3 = mid_word[7:0];

            b0 = i_data[31:24];
            b1 = i_data[23:16];
            b2 = i_data[15:8];
            b3 = i_data[7:0];

            s0 = sobel_grad(
                top_prev[15:8], top_prev[7:0], t0,
                mid_prev [15:8], mid_prev [7:0], m0,
                bot_prev [15:8], bot_prev [7:0], b0
            );

            s1 = sobel_grad(
                top_prev[7:0], t0, t1,
                mid_prev [7:0], m0, m1,
                bot_prev [7:0], b0, b1
            );

            s2 = sobel_grad(
                t0, t1, t2,
                m0, m1, m2,
                b0, b1, b2
            );

            s3 = sobel_grad(
                t1, t2, t3,
                m1, m2, m3,
                b1, b2, b3
            );

            gradients[0]<=use_y<2 || use_x==0 ? 26'd0 : s0;
            gradients[1]<=use_y<2 ? 26'd0 : s1;
            gradients[2]<=use_y<2 ? 26'd0 : s2;
            gradients[3]<=use_y<2 ? 26'd0 : s3;

            line2[use_x] <= mid_word;
            line1[use_x] <= i_data;

            top_prev = top_word[15:0];
            mid_prev = mid_word[15:0];
            bot_prev = i_data[15:0];

            if (i_last || (use_x == WORDS_PER_LINE - 1)) begin
                word_x   <= 11'd0;
                top_prev = 16'd0;
                mid_prev = 16'd0;
                bot_prev = 16'd0;

                if (use_y == IMG_HEIGHT - 1)
                    line_y <= 11'd0;
                else
                    line_y <= use_y + 1'b1;
            end
            else begin
                word_x <= use_x + 1'b1;
                line_y <= use_y;
            end
        end
        else begin
            gradients[0]<=0;gradients[1]<=0;gradients[2]<=0;gradients[3]<=0;
        end
    end
end

endmodule
