// 功能说明：板间协议解包、长度检查及错误抑制；下个帧标记重新同步。

// Compatible with the dense (16 beats / 3 words) sender. No wiring change.
// Counts complete frames and exposes per-frame validity separately from sticky errors.

module gpio_video_rx_fault_cont # ( parameter FRAME_WORDS = 172800,
    WINDOW_FRAMES = 120,
    SETTLE_FRAMES = 4 ) ( input wire clk,
    rst_n,
    input wire [7 : 0] diagnostic_mode,
    input wire [31 : 0] diagnostic_packets,
    output wire [127 : 0] diagnostic_summary,
    output reg [255 : 0] continuous_record,
    input wire [23 : 0] i_data,
    input wire i_valid,
    i_sop,
    i_frame,
    output reg [127 : 0] o_data,
    output reg o_valid,
    o_frame,
    protocol_error,
    output reg previous_good,
    output reg [31 : 0] good_count,
    bad_count,
    error_count,
    output reg [3 : 0] last_error,
    output reg [17 : 0] last_words,
    output wire [26 : 0] probe_bus,
    output wire [255 : 0] first_record,
    output reg [31 : 0] frame_count );

    reg [26 : 0] captured; // synthesis keep

    // 时序/组合逻辑：按以下触发条件更新本功能状态。
    always @ ( posedge clk or negedge rst_n ) if ( ! rst_n ) captured <= 0;
    else captured <= { i_frame, i_sop, i_valid, i_data };
    wire [23 : 0] data = captured [23 : 0];
    wire valid = captured [24], sop = captured [25], frame = captured [26];
    wire marker = sop && frame && ! valid;
    reg synced, seen_frame, frame_bad;
    reg [17 : 0] words;
    reg [3 : 0] part;
    reg [359 : 0] collecting;
    reg [383 : 0] emit_group;
    reg [1 : 0] emit_count;
    reg [3 : 0] error_now;

    // 时序/组合逻辑：按以下触发条件更新本功能状态。
    always @ * begin
        error_now = 0;
        if ( marker ) begin
            if ( part != 0 || emit_count != 0 ) error_now = 4;
        end
        else if ( synced ) begin
            if ( frame ) error_now = 5;
            else if ( sop && ( ! valid || part != 0 ) ) error_now = 2;
            else if ( valid && part == 0 && ! sop ) error_now = 3;
            else if ( ! valid && part != 0 ) error_now = 1;
        end
    end
    assign probe_bus = captured;
    reg first_seen;
    reg mode_known, active, done;
    reg [7 : 0] mode, settle, window_frames;
    reg [15 : 0] error_types [1 : 6];
    integer ei;
    wire mode_change = diagnostic_packets != 0 && ( ! mode_known || diagnostic_mode [3] != mode [3] || diagnostic_mode [6] != mode [6] );

    wire [3 : 0] event_code = error_now != 0 ? error_now : ( marker && seen_frame && words != FRAME_WORDS ) ? 4'd6 : 4'd0;
    assign diagnostic_summary = { error_types [6], error_types [5], error_types [4], error_types [3], error_types [2], error_types [1], 3'd0, mode_known, first_seen, done, active, 1'b0, settle, mode, window_frames };
    reg [31 : 0] cycles, valid_beats;
    reg [8 : 0] history;
    reg [255 : 0] record;
    assign first_record = record;

    // 时序/组合逻辑：按以下触发条件更新本功能状态。
    always @ ( posedge clk or negedge rst_n ) begin
        if ( ! rst_n ) begin
            mode_known <= 0;
            active <= 0;
            done <= 0;
            mode <= 0;
            settle <= 0;
            window_frames <= 0;
            for ( ei = 1; ei <= 6; ei = ei + 1 ) error_types [ei] <= 0;
            first_seen <= 0;
            cycles <= 0;
            valid_beats <= 0;
            history <= 0;
            record <= 0;
            frame_count <= 0;
        end
        else begin
            history <= { history [5 : 0], captured [26 : 24] };
            if ( marker ) begin
                frame_count <= frame_count + 1'b1;
                cycles <= 0;
                valid_beats <= 0;
            end
            else if ( seen_frame ) begin
                cycles <= cycles + 1'b1;
                if ( valid ) valid_beats <= valid_beats + 1'b1;
            end
            // Rearm only on the CRC-accepted actual algorithm/style mode. Do not
            // clear the decoder, counters or DDR. Skip complete frame intervals
            // so delayed source telemetry and an in-flight switch are not attributed.
            if ( mode_change ) begin
                mode_known <= 1;
                mode <= diagnostic_mode;
                settle <= 0;
                window_frames <= 0;
                active <= 0;
                done <= 0;
                first_seen <= 0;
                record <= 0;
                for ( ei = 1; ei <= 6; ei = ei + 1 ) error_types [ei] <= 0;
            end
            else begin
                if ( mode_known && ! active && ! done && marker ) begin
                    if ( settle == SETTLE_FRAMES - 1 ) active <= 1;
                    else settle <= settle + 1'b1;
                end
                if ( active ) begin
                    if ( marker ) begin
                        window_frames <= window_frames + 1'b1;
                        if ( window_frames == WINDOW_FRAMES - 1 ) begin
                            active <= 0;
                            done <= 1;
                        end
                    end
                    for ( ei = 1; ei <= 6; ei = ei + 1 ) if ( event_code == ei && error_types [ei] != 16'hffff ) error_types [ei] <= error_types [ei] + 1'b1;
                end
                if ( active && ! first_seen && event_code != 0 ) begin
                    first_seen <= 1;
                    record <= { 32'h46525354, 8'd0, data, 24'd0, 1'b1, synced, frame_bad, seen_frame, ( error_now != 0 ? error_now : 4'd6 ), 16'd0, history, captured [26 : 24], part, 14'd0, words, valid_beats, cycles, frame_count };
                end
            end
        end
    end
    // Independent of telemetry availability, mode changes, settling and window
    // completion. Cleared only by receiver reset (including loss of A RUN).
    // Context is the latest CRC-accepted telemetry, NOT an atomic source-frame tag.
    reg continuous_seen;
    wire [7 : 0] fault_context = { diagnostic_packets != 0, mode_change, active, done, mode_known, seen_frame, frame_bad, 1'b0 };

    // 时序/组合逻辑：按以下触发条件更新本功能状态。
    always @ ( posedge clk or negedge rst_n ) begin
        if ( ! rst_n ) begin
            continuous_seen <= 0;
            continuous_record <= 0;
        end
        else if ( ! continuous_seen && event_code != 0 ) begin
            continuous_seen <= 1;
            continuous_record <= { 32'h46525354, 8'd0, data, diagnostic_mode, mode, fault_context, 1'b1, synced, frame_bad, seen_frame, event_code, 16'd0, history, captured [26 : 24], part, 14'd0, words, valid_beats, cycles, frame_count };
        end
    end
    wire complete_frame = seen_frame && ! frame_bad && error_now == 0 && words == FRAME_WORDS;

    // 时序/组合逻辑：按以下触发条件更新本功能状态。
    always @ ( posedge clk or negedge rst_n ) begin
        if ( ! rst_n ) begin
            synced <= 0;
            seen_frame <= 0;
            frame_bad <= 0;
            words <= 0;
            part <= 0;
            collecting <= 0;
            emit_group <= 0;
            emit_count <= 0;
            o_data <= 0;
            o_valid <= 0;
            o_frame <= 0;
            protocol_error <= 0;
            previous_good <= 0;
            good_count <= 0;
            bad_count <= 0;
            error_count <= 0;
            last_error <= 0;
            last_words <= 0;
        end
        else begin
            o_valid <= 0;
            o_frame <= 0;
            // Count the same registered word pulses consumed by video_in.
            if ( o_valid && words != 18'h3ffff ) words <= words + 1'b1;
            if ( emit_count != 0 ) begin
                o_data <= emit_group [127 : 0];
                o_valid <= 1;
                emit_group <= emit_group >> 128;
                emit_count <= emit_count - 1'b1;
            end
            if ( error_now != 0 ) begin
                protocol_error <= 1;
                frame_bad <= 1;
                error_count <= error_count + 1'b1;
                last_error <= error_now;
            end
            if ( marker ) begin
                previous_good <= complete_frame;
                last_words <= words;
                if ( seen_frame ) begin
                    if ( complete_frame ) good_count <= good_count + 1'b1;
                    else bad_count <= bad_count + 1'b1;
                end
                synced <= 1;
                seen_frame <= 1;
                frame_bad <= 0;
                words <= 0;
                part <= 0;
                o_frame <= 1;
                emit_count <= 0;
                o_valid <= 0;
            end
            else if ( synced ) begin
                if ( error_now != 0 ) begin
                    synced <= 0;
                    part <= 0;
                end
                else if ( valid ) begin
                    collecting <= { data, collecting [359 : 24] };
                    if ( part == 15 ) begin
                        emit_group <= { data, collecting };
                        emit_count <= 3;
                        part <= 0;
                    end
                    else part <= part + 1'b1;
                end
            end
        end
    end
endmodule

