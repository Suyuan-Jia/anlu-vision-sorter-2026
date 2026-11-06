// 3 x 128-bit ISP words -> 16 x 24-bit beats, LSB first, with no padding.
// Each complete frame must contain a multiple of three words (720p: 172800).
// Uses the 78.947 MHz link, frame token, 64-cycle guard and fail-stop RUN.
module gpio_video_tx_dense #(
    parameter FIFO_ADDR_WIDTH = 11
)(
    input wire src_clk, link_clk, rst_n,
    input wire [127:0] i_data,
    input wire i_valid, i_frame,
    output reg [23:0] o_data,
    output reg o_valid, o_sop, o_frame,
    output wire o_run,
    output reg overflow
);
    reg started;
    wire full, empty, fifo_valid;
    wire [129:0] fifo_data;
    wire push = rst_n && !overflow && (started || i_frame) && (i_valid || i_frame);
    always @(posedge src_clk or negedge rst_n) begin
        if (!rst_n) begin started <= 0; overflow <= 0; end
        else begin
            if (i_frame) started <= 1;
            if (push && full) overflow <= 1;
        end
    end
    assign o_run = rst_n && !overflow;
    wire read_request;
    soft_fifo_al_4057d6b76aa6 #(
        .DATA_WIDTH_W(130), .DATA_WIDTH_R(130),
        .ADDR_WIDTH_W(FIFO_ADDR_WIDTH), .ADDR_WIDTH_R(FIFO_ADDR_WIDTH),
        .AL_FULL_NUM((1 << FIFO_ADDR_WIDTH)-12), .AL_EMPTY_NUM(2),
        .SHOW_AHEAD_EN(0), .OUTREG_EN("NOREG")
    ) u_link_fifo (
        .rst(!rst_n), .clkw(src_clk), .clkr(link_clk),
        .we(push && !full), .di({i_frame, i_valid, i_data}),
        .re(read_request), .dout(fifo_data), .valid(fifo_valid),
        .full_flag(full), .empty_flag(empty), .afull(), .aempty(),
        .wrusedw(), .rdusedw()
    );
    reg [129:0] queue [0:3];
    reg [1:0] wr_ptr, rd_ptr;
    reg [2:0] count, outstanding;
    reg [383:0] build_group;
    reg [1:0] build_count;
    reg [359:0] shift;
    reg [3:0] remaining;
    reg [6:0] frame_gap;
    reg [1:0] emit_guard;
    wire take_frame = count != 0 && queue[rd_ptr][129] &&
                      build_count != 3 && remaining == 0 && frame_gap == 0 && emit_guard == 0;
    wire take_word = count != 0 && !queue[rd_ptr][129] && build_count != 3;
    wire take = take_frame || take_word;
    assign read_request = o_run && !empty && (count + outstanding < 4);
    always @(posedge link_clk or negedge o_run) begin
        if (!o_run) begin
            wr_ptr <= 0; rd_ptr <= 0; count <= 0; outstanding <= 0;
            build_group <= 0; build_count <= 0; shift <= 0;
            remaining <= 0; frame_gap <= 0; emit_guard <= 0;
            o_data <= 0; o_valid <= 0; o_sop <= 0; o_frame <= 0;
        end else begin
            o_data <= 0; o_valid <= 0; o_sop <= 0; o_frame <= 0;
            // RX emits three words after a group. Finish those before a SOF
            // can reset its output buffer, even for queued back-to-back frames.
            if (emit_guard != 0) emit_guard <= emit_guard - 1'b1;
            if (remaining == 1) emit_guard <= 3;
            case ({read_request, fifo_valid})
                2'b10: outstanding <= outstanding + 1'b1;
                2'b01: outstanding <= outstanding - 1'b1;
                default: ;
            endcase
            if (fifo_valid) begin queue[wr_ptr] <= fifo_data; wr_ptr <= wr_ptr + 1'b1; end
            if (take) rd_ptr <= rd_ptr + 1'b1;
            case ({fifo_valid, take})
                2'b10: count <= count + 1'b1;
                2'b01: count <= count - 1'b1;
                default: ;
            endcase
            if (take_word && queue[rd_ptr][128]) begin
                case (build_count)
                    0: build_group[127:0] <= queue[rd_ptr][127:0];
                    1: build_group[255:128] <= queue[rd_ptr][127:0];
                    2: build_group[383:256] <= queue[rd_ptr][127:0];
                    default: ;
                endcase
                build_count <= build_count + 1'b1;
            end
            if (frame_gap != 0) frame_gap <= frame_gap - 1'b1;
            else if (remaining != 0) begin
                o_data <= shift[23:0]; o_valid <= 1;
                shift <= shift >> 24; remaining <= remaining - 1'b1;
            end else if (build_count == 3) begin
                o_data <= build_group[23:0]; o_valid <= 1; o_sop <= 1;
                shift <= build_group[383:24]; remaining <= 15; build_count <= 0;
            end else if (take_frame) begin
                o_sop <= 1; o_frame <= 1; frame_gap <= 64;
                // A short source frame can leave one or two words buffered.
                // They cannot form a group: discard only that incomplete tail
                // and forward SOF so RX rejects the short frame and recovers.
                // Waiting for build_count==0 here deadlocks behind this SOF.
                build_count <= 0;
                if (queue[rd_ptr][128]) begin
                    build_group[127:0] <= queue[rd_ptr][127:0]; build_count <= 1;
                end
            end
        end
    end
endmodule
