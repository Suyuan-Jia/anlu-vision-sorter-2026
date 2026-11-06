// RGB888 3x3 均值滤波、RGB333 量化与边缘叠加。
// 匹配当前边缘模块：四级寄存延迟，空间偏移四行、四列。
// 图像宽度必须是四的整数倍。
module cartoon_overlay_96 #(
    parameter integer IMG_WIDTH  = 1280,
    parameter integer IMG_HEIGHT = 720
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_ce,

    // 来自白平衡模块的原始彩色数据。
    input  wire [95:0] I_color_data,
    input  wire        I_color_valid,
    input  wire        I_color_user,
    input  wire        I_color_last,

    // 来自现有边缘模块的黑底白边数据。
    input  wire [95:0] I_edge_data,

    // 已同步到当前时钟域，零为纯边缘，一为量化叠边。
    input  wire        I_cartoon_req,

    // 输出与现有边缘模块的有效、帧首、行尾信号对齐。
    output reg  [95:0] O_data,
    // Independent strong-edge mask, aligned to I_edge_data; lane 0 is bit 3.
    input  wire [3:0]  I_cartoon_edge
);

localparam integer WORDS_PER_LINE = IMG_WIDTH / 4;

localparam integer ROW_ADDR_W = ($clog2(WORDS_PER_LINE) < 2) ? 2 : $clog2(WORDS_PER_LINE);

reg [10:0] word_x;
reg [10:0] line_y;
reg [2:0] write_bank;

wire [10:0] use_x = I_color_user ? 11'd0 : word_x;
wire [10:0] use_y = I_color_user ? 11'd0 : line_y;

wire [2:0] use_bank = I_color_user ? 3'd0 : write_bank;

function [2:0] bank_offset;
    input [2:0] bank, offset;
    reg [3:0] sum;
begin
    sum = {1'b0, bank} + {1'b0, offset};
    bank_offset = (sum >= 6) ? sum - 4'd6 : sum;
end
endfunction

// 六行 RGB888 环形缓存：当前写 y，读取 y-5/y-4/y-3，中心为 y-4。
// 保留已有空间偏移，无需增加边缘分支的延迟。显式 ERAM 避免
// TD 将带读使能的推断 RAM 回退成占用大量 slice 的分布式 RAM。
wire [95:0] row_read [0:5];
reg [95:0] previous_word [0:5];
reg [23:0] previous_tail [0:5];
wire [143:0] row_window [0:5];
reg [10:0] x_d1;
reg [2:0] top_bank_d1, center_bank_d1, bottom_bank_d1;
genvar bank;
generate
    for (bank = 0; bank < 6; bank = bank + 1) begin : color_rows
        // NOREG 仍有一拍同步读；NORMAL 在写入时保持读输出。
        // 当前写 bank 不参与窗口读取，不依赖同址读写结果。
        PH1P_LOGIC_ERAM #(
            .DATA_WIDTH_A(96), .DATA_WIDTH_B(96),
            .ADDR_WIDTH_A(ROW_ADDR_W), .ADDR_WIDTH_B(ROW_ADDR_W),
            .DATA_DEPTH_A(2**ROW_ADDR_W), .DATA_DEPTH_B(2**ROW_ADDR_W),
            .MODE("SP"), .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
            .WRITEMODE_A("NORMAL"), .WRITEMODE_B("NORMAL"),
            .CLKMODE("ASYNC"), .IMPLEMENT("20K(FAST)"),
            .ASYNC_RESET_RELEASE_A("ASYNC"), .ASYNC_RESET_RELEASE_B("ASYNC")
        ) row_ram (
            .dia(I_color_data), .addra(use_x[ROW_ADDR_W-1:0]),
            .clka(I_clk), .cea(I_rst_n && I_ce && I_color_valid),
            .wea(use_bank == bank), .ocea(1'b0), .rsta(1'b0), .bea(1'b0),
            .doa(row_read[bank]),
            .dib(96'd0), .addrb({ROW_ADDR_W{1'b0}}),
            .clkb(I_clk), .ceb(1'b0), .web(1'b0), .oceb(1'b0),
            .rstb(1'b0), .beb(1'b0), .dob(),
            .ecc_sbiterrinj(1'b0), .ecc_dbiterrinj(1'b0),
            .ecc_sbiterr(), .ecc_dbiterr()
        );
        // 六个横向邻居：前两字尾、前一完整字、本字首。
        // 中心字位于图像左侧时，复制首像素作为左邻居。
        assign row_window[bank] = {
            (x_d1 == 1) ? previous_word[bank][95:72] : previous_tail[bank],
            previous_word[bank], row_read[bank][95:72]
        };
    end
endgenerate

wire [143:0] top_window = row_window[top_bank_d1];
wire [143:0] center_window = row_window[center_bank_d1];
wire [143:0] bottom_window = row_window[bottom_bank_d1];

// 先共享六列的三像素和，再生成四个窗口和。每通道只需
// 6*2+4*2=20 次加法，而四个窗口分别求九像素和需要 4*8=32 次。
wire [9:0] column_red [0:5];
wire [9:0] column_green [0:5];
wire [9:0] column_blue [0:5];
wire [143:0] filtered_sum;
genvar column, lane;
generate
    for (column = 0; column < 6; column = column + 1) begin : mean_columns
        assign column_red[column] =
            {2'd0, top_window[143-24*column -: 8]} +
            {2'd0, center_window[143-24*column -: 8]} +
            {2'd0, bottom_window[143-24*column -: 8]};
        assign column_green[column] =
            {2'd0, top_window[135-24*column -: 8]} +
            {2'd0, center_window[135-24*column -: 8]} +
            {2'd0, bottom_window[135-24*column -: 8]};
        assign column_blue[column] =
            {2'd0, top_window[127-24*column -: 8]} +
            {2'd0, center_window[127-24*column -: 8]} +
            {2'd0, bottom_window[127-24*column -: 8]};
    end
    for (lane = 0; lane < 4; lane = lane + 1) begin : mean_lanes
        assign filtered_sum[143-36*lane -: 12] =
            {2'd0, column_red[lane]} + {2'd0, column_red[lane+1]} + {2'd0, column_red[lane+2]};
        assign filtered_sum[131-36*lane -: 12] =
            {2'd0, column_green[lane]} + {2'd0, column_green[lane+1]} + {2'd0, column_green[lane+2]};
        assign filtered_sum[119-36*lane -: 12] =
            {2'd0, column_blue[lane]} + {2'd0, column_blue[lane+1]} + {2'd0, column_blue[lane+2]};
    end
endgenerate

// 精确实现 floor(floor(sum/9)/32) = floor((sum>>5)/9)。
// sum 范围 0..2295，因此 sum>>5 范围 0..71；仅需三个常量
// 比较和两个减法，无通用除法器、乘法器或近似倒数。
/*function [2:0] quantize_mean;
    input [11:0] sum;
    reg [6:0] remainder;
begin
    remainder = sum[11:5];
    quantize_mean = 3'd0;
    if (remainder >= 7'd36) begin
        quantize_mean[2] = 1'b1;
        remainder = remainder - 7'd36;
    end
    if (remainder >= 7'd18) begin
        quantize_mean[1] = 1'b1;
        remainder = remainder - 7'd18;
    end
    if (remainder >= 7'd9) quantize_mean[0] = 1'b1;
end
endfunction*/

// 将三位颜色展开为八位。
// 对应级别为零、三十六、七十三、一百零九等。
/*function [7:0] expand_channel;
    input [2:0] value;
begin
    expand_channel = {value, value, value[2:1]};
end
endfunction

function [23:0] expand_rgb;
    input [8:0] rgb;
begin
    expand_rgb = {
        expand_channel(rgb[8:6]),
        expand_channel(rgb[5:3]),
        expand_channel(rgb[2:0])
    };
end
endfunction*/

// 九像素和恢复为 8 位均值。
// 对 sum=0..2295，精确等于 floor(sum / 9)。
// 1821 = 2048 - 256 + 32 - 4 + 1。
function [7:0] mean_channel;
    input [11:0] sum;
    reg [22:0] x;
    reg [22:0] scaled;
begin
    x = {11'd0, sum};
    scaled = (x << 11) - (x << 8)
           + (x << 5)  - (x << 2) + x;

    mean_channel = scaled[21:14];
end
endfunction

// 带符号结果裁剪到 0..255。
function [7:0] clip_u8;
    input signed [9:0] value;
begin
    if (value < 10'sd0)
        clip_u8 = 8'd0;
    else if (value > 10'sd255)
        clip_u8 = 8'd255;
    else
        clip_u8 = value[7:0];
end
endfunction

// 只量化亮度：RGB 同时加上 Y_quant - Y。
// 无需显式计算 UV；未发生裁剪时保持色度。
function [23:0] quantize_luma;
    input [23:0] rgb;

    reg [15:0] r_ext, g_ext, b_ext;
    reg [15:0] y_sum;
    reg [7:0] y, y_quant;
    reg signed [9:0] delta_y;
    reg signed [9:0] r_out, g_out, b_out;
begin
    r_ext = {8'd0, rgb[23:16]};
    g_ext = {8'd0, rgb[15:8]};
    b_ext = {8'd0, rgb[7:0]};

    // Y = (77R + 150G + 29B) >> 8。
    // 先扩位，再进行移位加法。
    y_sum =
        ((r_ext << 6) + (r_ext << 3)
                     + (r_ext << 2) + r_ext)
      + ((g_ext << 7) + (g_ext << 4)
                     + (g_ext << 2) + (g_ext << 1))
      + ((b_ext << 4) + (b_ext << 3)
                     + (b_ext << 2) + b_ext);

    y = y_sum[15:8];

    // 16 级亮度：0、17、34、...、238、255。
    y_quant = {y[7:4], y[7:4]};

    delta_y = $signed({2'b00, y_quant})
            - $signed({2'b00, y});

    r_out = $signed({2'b00, rgb[23:16]}) + delta_y;
    g_out = $signed({2'b00, rgb[15:8]})  + delta_y;
    b_out = $signed({2'b00, rgb[7:0]})   + delta_y;

    quantize_luma = {
        clip_u8(r_out),
        clip_u8(g_out),
        clip_u8(b_out)
    };
end
endfunction

// 四级：同步读 -> 九像素和 -> RGB888 均值 -> 亮度分层。
reg [143:0] sum_d2;
reg [95:0] color_d3;
reg [95:0] color_d4;

reg [3:0] valid_pipe;
reg [3:0] mode_pipe;
reg       mode_active;

// 第一阶段对应的颜色有效区域。
reg [3:0] color_mask_d1;

// 四个三乘三窗口累计需要八行、八列的历史覆盖。
// 在覆盖不完整的边界区域不叠加边缘，减少填零引起的假轮廓。
reg [3:0] edge_mask_d1;
reg [3:0] edge_mask_d2;
reg [3:0] edge_mask_d3;
reg [3:0] edge_mask_d4;

integer k;

always @(posedge I_clk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        write_bank   <= 3'd0;
        word_x       <= 11'd0;
        line_y       <= 11'd0;

        x_d1          <= 11'd0;
        top_bank_d1    <= 3'd0;
        center_bank_d1 <= 3'd0;
        bottom_bank_d1 <= 3'd0;
        for (k = 0; k < 6; k = k + 1) begin
            previous_word[k] <= 96'd0;
            previous_tail[k] <= 24'd0;
        end
        sum_d2        <= 144'd0;
        color_d3      <= 96'd0;
        color_d4      <= 96'd0;

        valid_pipe <= 4'd0;
        mode_pipe  <= 4'd0;
        mode_active <= 1'b0;

        color_mask_d1 <= 4'd0;
        edge_mask_d1  <= 4'd0;
        edge_mask_d2  <= 4'd0;
        edge_mask_d3  <= 4'd0;
        edge_mask_d4  <= 4'd0;
    end
    else if (I_ce) begin
        valid_pipe <= {valid_pipe[2:0], I_color_valid};
        x_d1 <= use_x;
        // 顶部中心位于第零行时，用中心行复制上邻居。
        top_bank_d1 <= bank_offset(use_bank, (use_y == 4) ? 3'd2 : 3'd1);
        center_bank_d1 <= bank_offset(use_bank, 3'd2);
        bottom_bank_d1 <= bank_offset(use_bank, 3'd3);

        // 只在彩色输入帧首接纳模式请求。
        // 模式与像素一起延迟，避免一帧内出现两种效果。
        if (I_color_user)
            mode_active <= I_cartoon_req;

        mode_pipe <= {
            mode_pipe[2:0],
            I_color_user ? I_cartoon_req : mode_active
        };

        for (k = 0; k < 4; k = k + 1) begin
            color_mask_d1[3-k] <=
                I_color_valid &&
                (use_y >= 4) &&
                ((use_x * 4 + k) >= 4);

            edge_mask_d1[3-k] <=
                I_color_valid &&
                (use_y >= 8) &&
                ((use_x * 4 + k) >= 8);
        end

        edge_mask_d2 <= edge_mask_d1;
        edge_mask_d3 <= edge_mask_d2;
        edge_mask_d4 <= edge_mask_d3;

        // 只在有效组推进横向历史；空拍不丢邻居，CE=0 全部冻结。
        if (valid_pipe[0]) begin
            for (k = 0; k < 6; k = k + 1) begin
                previous_word[k] <= row_read[k];
                previous_tail[k] <= previous_word[k][23:0];
            end
            for (k = 0; k < 4; k = k + 1) begin
                sum_d2[143-36*k -: 36] <=
                    color_mask_d1[3-k]
                        ? filtered_sum[143-36*k -: 36]
                        : 36'd0;
            end
        end
        else begin
            sum_d2 <= 144'd0;
        end

        // 第三级计算 RGB888 均值，第四级进行亮度分层。
        // 保持原有四级延迟，与边缘输出同拍。
        for (k = 0; k < 4; k = k + 1) begin
            color_d3[95-24*k -: 24] <= {
                mean_channel(sum_d2[143-36*k -: 12]),
                mean_channel(sum_d2[131-36*k -: 12]),
                mean_channel(sum_d2[119-36*k -: 12])
            };

            color_d4[95-24*k -: 24] <=
                quantize_luma(color_d3[95-24*k -: 24]);
        end

        if (I_color_valid) begin
            if (I_color_last || (use_x == WORDS_PER_LINE - 1)) begin
                word_x <= 11'd0;
                write_bank <= (use_y == IMG_HEIGHT - 1)
                    ? 3'd0 : bank_offset(use_bank, 3'd1);

                if (use_y == IMG_HEIGHT - 1)
                    line_y <= 11'd0;
                else
                    line_y <= use_y + 1'b1;
            end
            else begin
                word_x <= use_x + 1'b1;
                line_y <= use_y;
                write_bank <= use_bank;
            end
        end
        else if (I_color_user) begin
            // 同时兼容帧首标记先于第一个有效像素到达。
            write_bank <= 3'd0;
            word_x    <= 11'd0;
            line_y    <= 11'd0;
        end
    end
end

integer p;

always @(*) begin
    O_data = 96'd0;

    if (valid_pipe[3]) begin
        if (!mode_pipe[3]) begin
            // 纯边缘模式保持原来的黑底白边效果。
            O_data = I_edge_data;
        end
        else begin
            for (p = 0; p < 4; p = p + 1) begin
                // 卡通仅使用独立强边缘；纯边缘模式继续原样输出 I_edge_data。
                if (edge_mask_d4[3-p] &&
                    I_cartoon_edge[3-p])
                    O_data[95-24*p -: 24] = 24'h000000;
                else
                    O_data[95-24*p -: 24] =
                        color_d4[95-24*p -: 24];
            end
        end
    end
end

endmodule
