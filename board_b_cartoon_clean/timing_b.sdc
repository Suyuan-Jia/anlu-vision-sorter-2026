create_clock -name sys_clk_50m -period 20.000 [get_ports {I_sys_clk}]
create_clock -name LINK_RX_CLK -period 12.667 [get_ports {I_link_clk}]
derive_clocks
rename_clock -name pll_clk_100m [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[0]}]
rename_clock -name HDMI_PIXEL_CLK [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[4]}]
rename_clock -name HDMI_SERIAL_CLK [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[5]}]
set_clock_groups -asynchronous -group [get_clocks LINK_RX_CLK] -group [get_clocks {sys_clk_50m pll_clk_100m HDMI_PIXEL_CLK HDMI_SERIAL_CLK}] -group [get_clocks {u_ph1p35_324_ddr_wrapper/u_ddr2/ddr_clk u_ph1p35_324_ddr_wrapper/u_ddr2/usr_clk}]
# Source launches on the falling edge of the forwarded clock. Budget includes
# board-A clock/data skew (+/-3 ns) plus +/-0.5 ns cable skew.
set_input_delay -clock [get_clocks LINK_RX_CLK] -clock_fall -max 3.500 [get_ports {I_link_data[*] I_link_valid I_link_sop I_link_frame}]
set_input_delay -clock [get_clocks LINK_RX_CLK] -clock_fall -min -3.500 [get_ports {I_link_data[*] I_link_valid I_link_sop I_link_frame}]
# Additional implementation guard band; clocks and I/O delays remain unchanged.
set_clock_uncertainty -setup 0.800 [get_clocks LINK_RX_CLK]
set_clock_uncertainty -setup 0.500 [get_clocks HDMI_PIXEL_CLK]
# Reserve extra hold margin on the active processing/display/DDR user clocks.
set_clock_uncertainty -hold 0.050 [get_clocks {LINK_RX_CLK HDMI_PIXEL_CLK u_ph1p35_324_ddr_wrapper/u_ddr2/usr_clk}]
set_false_path -from [get_ports I_link_run]

# Asynchronous physical switch; synchronized before frame debounce.
set_false_path -from [get_ports I_switch]

# KEY1异步输入先两级同步，再做按下消抖。
set_false_path -from [get_ports I_key1_n]

# Physical SW2 is asynchronous; first stage is motion_meta.
set_false_path -from [get_ports I_motion_switch]
