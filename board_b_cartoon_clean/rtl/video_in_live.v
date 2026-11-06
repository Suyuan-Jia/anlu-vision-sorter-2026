// 功能说明：DDR写入及四帧缓存发布；完整帧才能发布，错误帧拒绝并保留上幅画面。

// Publish only a complete, protocol-clean frame whose writes have drained.
// Four-bank ownership prevents rewriting the bank currently being displayed.

module video_in_live # ( parameter FRAME_WORDS = 172800, BURST_WORDS=240 ) ( input wire I_rst_n,
    I_camera_clk,
    I_camera_frame_start,
    I_camera_valid,
    input wire [127 : 0] I_camera_data,
    input wire I_previous_good,
    input wire [2:0] I_camera_mode,
    output reg [2:0] O_published_mode,
    input wire I_ddr_clk,
    I_video_out_rd_busy,
    input wire [1 : 0] I_read_bank,
    output wire O_video_in_wr_busy,
    output reg [1 : 0] O_video_out_rp,
    output reg O_frame_available,
    output reg O_ddr_user_wr_en,
    output reg [24 : 0] O_ddr_user_addr,
    output wire [127 : 0] O_ddr_user_wr_data,
    input wire I_ddr_user_ready,
    output reg O_fifo_overflow,
    output reg [31 : 0] O_committed,
    O_rejected,
    output reg [31 : 0] O_last_written,
    O_reject_flags,
    output wire O_camera_ready,
    output wire O_camera_drained,
    output wire [31 : 0] O_bank_debug );

    // Bundled metadata follows the same closing marker and commit as pixels.
    // Hold the completed frame tag through the existing frame-event CDC.
    wire commit;
    reg [2:0] frame_mode, previous_mode_hold, mode_meta, mode_sync;
    always @(posedge I_camera_clk or negedge I_rst_n)
      if(!I_rst_n) begin frame_mode<=1; previous_mode_hold<=1; end
      else begin
        if(I_camera_valid) frame_mode<=I_camera_mode;
        if(I_camera_frame_start) previous_mode_hold<=frame_mode;
      end
    always @(posedge I_ddr_clk or negedge I_rst_n)
      if(!I_rst_n) begin mode_meta<=1; mode_sync<=1; end
      else begin mode_meta<=previous_mode_hold; mode_sync<=mode_meta; end
    always @(posedge I_ddr_clk or negedge I_rst_n)
      if(!I_rst_n) O_published_mode<=1;
      else if(commit) O_published_mode<=mode_sync;

    reg frame_extend;
    reg [3 : 0] extend_count;
    reg previous_good_hold, frame_overflow;
    wire fifo_full;
    wire [8:0] camera_fifo_words;
    reg [1:0] drained_sync;
    // Replay can pause. Original camera/bypass stream retains its existing behavior.
    assign O_camera_ready=!fifo_full && camera_fifo_words<400;
    assign O_camera_drained=drained_sync[1] && camera_fifo_words==0 && !fifo_full;

    // 时序/组合逻辑：按以下触发条件更新本功能状态。
    always @ ( posedge I_camera_clk or negedge I_rst_n ) begin
        if ( ! I_rst_n ) begin
            frame_extend <= 0;
            extend_count <= 0;
            previous_good_hold <= 0;
            frame_overflow <= 0;
            O_fifo_overflow <= 0;
        end
        else begin
            if ( I_camera_frame_start ) begin
                frame_extend <= 1;
                extend_count <= 0;
                previous_good_hold <= I_previous_good && ! frame_overflow;
                frame_overflow <= 0;
            end
            else if ( frame_extend ) begin
                if ( extend_count == 7 ) frame_extend <= 0;
                else extend_count <= extend_count + 1'b1;
            end
            if ( I_camera_valid && fifo_full ) begin
                frame_overflow <= 1;
                O_fifo_overflow <= 1;
            end
        end
    end
    reg [2 : 0] frame_sync;
    reg [1 : 0] good_sync;

    // 时序/组合逻辑：按以下触发条件更新本功能状态。
    always @ ( posedge I_ddr_clk or negedge I_rst_n ) if ( ! I_rst_n ) begin
        frame_sync <= 0;
        good_sync <= 0;
    end
    else begin
        frame_sync <= { frame_sync [1 : 0], frame_extend };
        good_sync <= { good_sync [0], previous_good_hold };
    end
    wire start_frame = frame_sync [1] && ! frame_sync [2];
    wire fifo_reset = ! I_rst_n || frame_sync [2];
    wire [8 : 0] fifo_words;
    reg burst;
    reg drained_ddr;
    always @(posedge I_ddr_clk or negedge I_rst_n)
      if(!I_rst_n) drained_ddr<=0;
      else drained_ddr<=(fifo_words==0 && !burst && !O_ddr_user_wr_en && !fifo_reset);
    always @(posedge I_camera_clk or negedge I_rst_n)
      if(!I_rst_n) drained_sync<=0;
      else drained_sync<={drained_sync[0],drained_ddr};
    reg [7 : 0] burst_count;
    reg [17 : 0] written;
    reg have_input;
    reg [1 : 0] write_bank;
    wire pop = burst && I_ddr_user_ready && ! fifo_reset && ! start_frame;
    assign commit = start_frame && have_input && good_sync [1] && written == FRAME_WORDS && ! burst && ! O_ddr_user_wr_en && fifo_words == 0;
    assign O_bank_debug = { 15'd0, fifo_words, 2'd0, write_bank, O_video_out_rp, I_read_bank };

    // 时序/组合逻辑：按以下触发条件更新本功能状态。
    always @ ( posedge I_ddr_clk or negedge I_rst_n ) if ( ! I_rst_n ) begin
        O_last_written <= 0;
        O_reject_flags <= 0;
    end
    else if ( start_frame && have_input ) begin
        O_last_written <= { 14'd0, written };
        O_reject_flags <= { 27'd0, ( fifo_words != 0 ), O_ddr_user_wr_en, burst, ( written != FRAME_WORDS ), ! good_sync [1] };
    end
    reg [1 : 0] free_bank;
    integer b;

    // 时序/组合逻辑：按以下触发条件更新本功能状态。
    always @ * begin
        free_bank = write_bank;
        // Reserve both old publication and current read bank: a display VSYNC
        // on this same edge can latch the old publication before it is replaced.
        for ( b = 3; b >= 0; b = b - 1 ) if ( b != I_read_bank && b != O_video_out_rp && ( ! commit || b != write_bank ) ) free_bank = b;
    end

    function [24 : 0] base ( input [1 : 0] bank );

        case ( bank )
            0 : base = 0;
            1 : base = 7000000;
            2 : base = 14000000;
            3 : base = 21000000;
        endcase

    endfunction

    assign O_video_in_wr_busy = burst;
    w128_d512_fifo fifo ( .rst ( fifo_reset ),
        .clkw ( I_camera_clk ),
        .we ( I_camera_valid && ! fifo_full ),
        .di ( I_camera_data ),
        .afull ( ),
        .full_flag ( fifo_full ),
        .wrusedw ( camera_fifo_words ),
        .clkr ( I_ddr_clk ),
        .re ( pop ),
        .dout ( O_ddr_user_wr_data ),
        .rdusedw ( fifo_words ),
        .valid ( ),
        .empty_flag ( ),
        .aempty ( ) );

    // 时序/组合逻辑：按以下触发条件更新本功能状态。
    always @ ( posedge I_ddr_clk or negedge I_rst_n ) begin
        if ( ! I_rst_n ) begin
            burst <= 0;
            burst_count <= 0;
            written <= 0;
            have_input <= 0;
            write_bank <= 1;
            O_video_out_rp <= 0;
            O_frame_available <= 0;
            O_ddr_user_wr_en <= 0;
            O_ddr_user_addr <= 7000000;
            O_committed <= 0;
            O_rejected <= 0;
        end
        else begin
            O_ddr_user_wr_en <= pop;
            if ( start_frame ) begin
                if ( commit ) begin
                    O_video_out_rp <= write_bank;
                    O_frame_available <= 1;
                    O_committed <= O_committed + 1'b1;
                end
                else if ( have_input ) O_rejected <= O_rejected + 1'b1;
                have_input <= 1;
                write_bank <= free_bank;
                O_ddr_user_addr <= base ( free_bank );

                burst <= 0;
                burst_count <= 0;
                written <= 0;
            end
            else begin
                if ( O_ddr_user_wr_en ) begin
                    O_ddr_user_addr <= O_ddr_user_addr + 8;
                    if ( written != 18'h3ffff ) written <= written + 1'b1;
                end
                if ( fifo_reset ) begin
                    burst <= 0;
                    burst_count <= 0;
                end
                else if ( burst ) begin
                    if ( pop ) begin
                        if ( burst_count == BURST_WORDS-1 ) begin
                            burst <= 0;
                            burst_count <= 0;
                        end
                        else burst_count <= burst_count + 1'b1;
                    end
                end
                else if ( have_input && fifo_words >= BURST_WORDS && ! I_video_out_rd_busy && written < FRAME_WORDS ) burst <= 1;
            end
        end
    end
endmodule

