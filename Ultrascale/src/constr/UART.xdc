############## NET - IOSTANDARD ##################
set_property CFGBVS GND [current_design];
set_property CONFIG_VOLTAGE 1.8 [current_design];
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design];
set_property BITSTREAM.CONFIG.PERSIST NO [current_design];
############# SPI Configurate Setting ##################
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design];
set_property CONFIG_MODE SPIx4 [current_design];
set_property BITSTREAM.CONFIG.CONFIGRATE 51.0 [current_design];
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design];
set_property BITSTREAM.CONFIG.UNUSEDPIN Pullup [current_design];
############# Clocking Ports ##################
#set_property IOSTANDARD LVCMOS33 [get_ports CLK_IN_50M];
#set_property PACKAGE_PIN AC13 [get_ports CLK_IN_50M];
set_property IOSTANDARD LVDS [get_ports SYS_CLK_P];
set_property IOSTANDARD LVDS [get_ports SYS_CLK_N];
set_property PACKAGE_PIN T24 [get_ports SYS_CLK_P];
set_property PACKAGE_PIN U24 [get_ports SYS_CLK_N];
############# UART Ports ##################
set_property IOSTANDARD LVCMOS33 [get_ports UART_RX_i];
set_property IOSTANDARD LVCMOS33 [get_ports UART_TX_o];
set_property PACKAGE_PIN AD13 [get_ports UART_RX_i];
set_property PACKAGE_PIN AC14 [get_ports UART_TX_o];
############# Buttons and LEDs ##################
set_property IOSTANDARD LVCMOS33 [get_ports {KEY_i[*]}];
set_property IOSTANDARD LVCMOS33 [get_ports {LED_o[*]}];
set_property PACKAGE_PIN K9 [get_ports {KEY_i[0]}];
set_property PACKAGE_PIN K10 [get_ports {KEY_i[1]}];
set_property PACKAGE_PIN J10 [get_ports {KEY_i[2]}];
set_property PACKAGE_PIN J11 [get_ports {KEY_i[3]}];
set_property PACKAGE_PIN H9 [get_ports {LED_o[0]}];
set_property PACKAGE_PIN J9 [get_ports {LED_o[1]}];
set_property PACKAGE_PIN G11 [get_ports {LED_o[2]}];
set_property PACKAGE_PIN H11 [get_ports {LED_o[3]}];
