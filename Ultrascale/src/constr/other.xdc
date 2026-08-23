############## NET - IOSTANDARD ##################
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.PERSIST NO [current_design]
#############SPI Configurate Setting##################
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 51.0 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.UNUSEDPIN Pullup [current_design]


set_property IOSTANDARD LVCMOS33 [get_ports clk_in_50M]
set_property PACKAGE_PIN AC13 [get_ports clk_in_50M]


set_property IOSTANDARD LVCMOS33 [get_ports {key_tri_i[*]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led_tri_o[*]}]

set_property PACKAGE_PIN K9 [get_ports {key_tri_i[0]}]
set_property PACKAGE_PIN K10 [get_ports {key_tri_i[1]}]
set_property PACKAGE_PIN J10 [get_ports {key_tri_i[2]}]
set_property PACKAGE_PIN J11 [get_ports {key_tri_i[3]}]

set_property PACKAGE_PIN H9 [get_ports {led_tri_o[0]}]
set_property PACKAGE_PIN J9 [get_ports {led_tri_o[1]}]
set_property PACKAGE_PIN G11 [get_ports {led_tri_o[2]}]
set_property PACKAGE_PIN H11 [get_ports {led_tri_o[3]}]

set_property IOSTANDARD LVCMOS33 [get_ports uart_rtl_0_rxd]
set_property IOSTANDARD LVCMOS33 [get_ports uart_rtl_0_txd]
set_property PACKAGE_PIN AD13 [get_ports uart_rtl_0_rxd]
set_property PACKAGE_PIN AC14 [get_ports uart_rtl_0_txd]

set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets clk_in_50M_IBUF_inst/O]

