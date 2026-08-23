#PWDN
set_property IOSTANDARD LVCMOS18 [get_ports {MIPI_GPIO_tri_o[1]}]
#RST
set_property IOSTANDARD LVCMOS18 [get_ports {MIPI_GPIO_tri_o[0]}]
set_property PACKAGE_PIN M21 [get_ports {MIPI_GPIO_tri_o[0]}]
set_property PACKAGE_PIN J21 [get_ports {MIPI_GPIO_tri_o[1]}]
##MIPI_CLK
set_property IOSTANDARD LVCMOS18 [get_ports mipi_clk]
set_property PACKAGE_PIN K21 [get_ports mipi_clk]
#create_clock -period 41.667 -name mipi_clk -waveform {0.000 5.000} [get_ports mipi_clk]


set_property IOSTANDARD LVCMOS18 [get_ports MIPI_IIC_scl_io]
set_property IOSTANDARD LVCMOS18 [get_ports MIPI_IIC_sda_io]
set_property PACKAGE_PIN J26 [get_ports MIPI_IIC_sda_io]
set_property PACKAGE_PIN J25 [get_ports MIPI_IIC_scl_io]

set_property INTERNAL_VREF 0.6 [get_iobanks 65]

set_property CLOCK_DEDICATED_ROUTE BACKBONE [get_nets design_1_i/eth_udp_loop_1/inst/u_gmii_to_rgmii/u_rgmii_rx/BUFG_inst_0]
set_property CLOCK_DEDICATED_ROUTE BACKBONE [get_nets design_1_i/ddr4_0/inst/u_ddr4_infrastructure/addn_ui_clkout3]