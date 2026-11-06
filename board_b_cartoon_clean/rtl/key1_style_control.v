// 功能说明：KEY1两级同步和20ms消抖，确认按下才翻转按键事件标志，长按不重复。

// KEY1 产生跨时钟域的保持型按键事件：先同步异步按键，再进行 20ms 连续稳定电平消抖。
// 按下有效；长按只切换一次，稳定松开后才允许下一次切换。

module key1_style_control # ( parameter integer DEBOUNCE_CYCLES = 1000000 ) ( input wire clk,
    input wire rst_n,
    input wire key_n,
    output reg press_toggle );

    localparam integer COUNT_WIDTH = ( DEBOUNCE_CYCLES < 2 ) ? 1 : $clog2 ( DEBOUNCE_CYCLES );

    reg key_meta;
    reg key_sync;
    reg key_stable;
    reg [COUNT_WIDTH - 1 : 0] stable_count;
    // 所有按键状态复位为松开；上电没有待处理按键事件。

    // 时序/组合逻辑：按以下触发条件更新本功能状态。
    always @ ( posedge clk or negedge rst_n ) begin
        if ( ! rst_n ) begin
            key_meta <= 1'b1;
            key_sync <= 1'b1;
            key_stable <= 1'b1;
            stable_count <= 0;
            press_toggle <= 1'b0;
        end
        else begin
            key_meta <= key_n;
            key_sync <= key_meta;
            if ( key_sync == key_stable ) begin
                stable_count <= 0;
            end
            else if ( stable_count == DEBOUNCE_CYCLES - 1 ) begin
                stable_count <= 0;
                key_stable <= key_sync;
                if ( ! key_sync ) press_toggle <= ~ press_toggle;
            end
            else begin
                stable_count <= stable_count + 1'b1;
            end
        end
    end
endmodule

