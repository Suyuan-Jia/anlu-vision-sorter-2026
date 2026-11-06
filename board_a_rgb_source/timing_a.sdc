create_clock -name sys_clk_50m -period 20.000 [get_ports {I_sys_clk}]
create_clock -name mipi_rx_ck_pad -period 4.167 [get_nets {IO_rx_clk_pad_p}]
derive_clocks
rename_clock -name pll_clk_100m [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[0]}]
rename_clock -name pll_clk_24m [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[1]}]
rename_clock -name LINK_TX_CLK [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[4]}]
rename_clock -name LINK_FWD_PLL_CLK [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[2]}]
rename_clock -name MIPI_RX_BYTE_CLK [get_clocks {u_mipi_dphy_rx_ph1p_mipiio_wrapper/u_ph1p_mipiio_rx_wrapper/u_PH1P_LOGIC_DPHY_MIPI_RX.o_fabric_div4_8_clk}]
set_clock_groups -asynchronous -group [get_clocks {mipi_rx_ck_pad MIPI_RX_BYTE_CLK}] -group [get_clocks {sys_clk_50m pll_clk_100m pll_clk_24m}]
set_clock_groups -asynchronous -group [get_clocks {mipi_rx_ck_pad}] -group [get_clocks {LINK_TX_CLK}]
# Keep byte/link FIFO Gray paths bounded below; broad clock groups would
# override their max-delay checks. Cut only fail-stop reset and telemetry.
set_false_path -from [get_clocks {MIPI_RX_BYTE_CLK}] -through [get_nets {link_overflow probe_ack probe_held[*]}] -to [get_clocks {LINK_TX_CLK}]
set_false_path -from [get_clocks {LINK_TX_CLK}] -through [get_nets {probe_request}] -to [get_clocks {MIPI_RX_BYTE_CLK}]
# 78.947 MHz dedicated inverted forwarded clock; unchanged I/O budgets.
# Board A bounds data transitions to +/-3 ns about the falling connector edge.
# Refer to the PLL pin: the ODDR instance can be absorbed into the pad during packing.
create_generated_clock -name LINK_FWD -source [get_pins {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[2]}] -divide_by 1 -invert [get_ports {O_link_clk}]
set_output_delay -clock [get_clocks LINK_FWD] -max 3.667 [get_ports {O_link_data[*] O_link_valid O_link_sop O_link_frame}]
set_output_delay -clock [get_clocks LINK_FWD] -min -3.667 [get_ports {O_link_data[*] O_link_valid O_link_sop O_link_frame}]
set_clock_uncertainty -setup 0.300 [get_clocks LINK_FWD]
# RUN is an asynchronous reset/status level, never a data strobe.
set_false_path -to [get_ports O_link_run]
# 12.3 ns is below the 12.666667 ns link period. Only the Gray source to
# first synchronization stage is asynchronous; second stages remain timed.
set_max_delay -datapath_only -from [get_clocks {LINK_TX_CLK}] -through [get_nets {u_link_tx/u_link_fifo/rd_to_wr_cross_inst/primary_addr_gray_reg[*]}] -to [get_clocks {MIPI_RX_BYTE_CLK}] 12.300
set_max_delay -datapath_only -from [get_clocks {MIPI_RX_BYTE_CLK}] -through [get_nets {u_link_tx/u_link_fifo/wr_to_rd_cross_inst/primary_addr_gray_reg[*]}] -to [get_clocks {LINK_TX_CLK}] 12.300

set_false_path -to [get_ports {O_led[*]}]

# The original uii2c also clocks shift/state registers from its internal divider.
# CLK_DIV=239 -> SCL_DIV=119 -> one toggle per 120 control clocks.
# Resolve the driver from the stable net name; packing can merge/rename the FF.
# Cover AE-on and legacy branches. Keep the expression literal: TD stores
# SDC commands for reevaluation and does not preserve temporary Tcl variables.
# Divider updates on source falling edges. Edge 2 is T/2; 242 and 482
# preserve 120T half-periods, hence /240, with a T/2 steady-state phase.
create_generated_clock -name SENSOR_I2C_SCL -source [get_pins {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[1]}] -edges {2 242 482} [get_pins -of [get_nets -nowarn {*master/scl_clk *u_i2c/scl_clk}] -filter {DIRECTION == output}]
set_clock_groups -asynchronous -group [get_clocks {MIPI_RX_BYTE_CLK mipi_rx_ck_pad}] -group [get_clocks {SENSOR_I2C_SCL}]

# STEP 5 bundled-data mailboxes: bound flight time below two destination
# periods, while the handshake waits two synchronizer stages + capture cycle.
# Empty collections are permitted for AE_ENABLE=0 only; the AE-on audit checks
# non-empty endpoints and reports both directions after routing.
set_max_delay -datapath_only -from [get_regs -nowarn {*cdc/request_snapshot*}] -to [get_regs -nowarn {*cdc/O_runtime_exposure* *cdc/O_runtime_gain*}] 80.000 -nowarn
set_max_delay -datapath_only -from [get_regs -nowarn {*cdc/response_snapshot*}] -to [get_regs -nowarn {*cdc/O_response_exposure* *cdc/O_response_gain* *cdc/O_response_kind* *cdc/O_response_error*}] 30.000 -nowarn



