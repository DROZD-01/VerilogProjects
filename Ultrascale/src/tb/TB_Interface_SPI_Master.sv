`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/23/2026 02:33:24 PM
// Design Name: 
// Module Name: TB_Interface_SPI_Master
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module TB_Interface_SPI_Master;
  
  localparam real TB_SPI_BAUDRATE =   5e+6 ; // SPI frequency in Hz
  localparam real TB_SYSTEM_FREQ  = 100e+6 ; // System clock frequency in Hz
  localparam int  TB_MAX_DATAW    =     16 ; // Maximum SPI packet size
  localparam bit  TB_DEFAULT_CS   =   1'b1 ; // Default CS   value on reset
  localparam bit  TB_DEFAULT_SCLK =   1'b0 ; // Default SCLK value on reset
  localparam int  TB_MAX_DATABW   = $clog2(TB_MAX_DATAW + 1);
  
  logic                     TB_RST_i             ;
  logic                     TB_CLK_i             ;
  logic                     TB_CS_o              ;
  logic                     TB_SCLK_o            ;
  logic                     TB_MOSI_o            ;
  logic                     TB_MISO_i            ;
  logic                     TB_SET_Valid         ;
  logic [              2:0] TB_SET_SPI_Mode      ;
  logic                     TB_SET_MSB           ;
  logic                     TB_SET_CS_Keep       ;
  logic                     TB_SET_CS_Pol        ;
  logic [              5:0] TB_SET_CS_WaitLength ;
  logic [              5:0] TB_SET_CS_WaitStart  ;
  logic [              5:0] TB_SET_CS_WaitEnd    ;
  logic                     TB_TRC_o             ;
  logic                     TB_DRE_o             ;
  logic                     TB_Valid_i           ;
  logic [TB_MAX_DATABW-1:0] TB_Data_DW           ;
  logic [ TB_MAX_DATAW-1:0] TB_Data_i            ;
  logic                     TB_Valid_o           ;
  logic [ TB_MAX_DATAW-1:0] TB_Data_o            ;
  
  initial 
  begin
    TB_CLK_i = 1'b0;
    forever 
    begin
      #((1e+9/TB_SYSTEM_FREQ)/2);
      TB_CLK_i = ~TB_CLK_i;
    end
  end
  
  initial 
  begin
    TB_RST_i = 1'b1;
    #(1000ns);
    TB_RST_i = 1'b0;
  end
  
  initial 
  begin
    TB_SET_Valid         = 1'b0;
    TB_SET_SPI_Mode      = 'd1;
    TB_SET_MSB           = 1'b1;
    TB_SET_CS_Keep       = 1'b0;
    TB_SET_CS_Pol        = 1'b0;
    TB_SET_CS_WaitLength = 'd0;
    TB_SET_CS_WaitStart  = 'd0;
    TB_SET_CS_WaitEnd    = 'd0;
    #(1500ns);
    @(posedge TB_CLK_i) TB_SET_Valid = 1'b1;
    @(posedge TB_CLK_i) TB_SET_Valid = 1'b0;
  end
  
  initial 
  begin
    TB_Valid_i = 1'b0;
    TB_Data_DW = 'd8;
    TB_Data_i  = 16'h0055;
    #(2000ns);
    @(posedge TB_CLK_i) TB_Valid_i = 1'b1;
    @(posedge TB_CLK_i) TB_Valid_i = 1'b0;
    wait(TB_DRE_o);
    TB_Data_DW = 'd12;
    TB_Data_i  = 16'h0AAA;
    @(posedge TB_CLK_i) TB_Valid_i = 1'b1;
    @(posedge TB_CLK_i) TB_Valid_i = 1'b0;
  end
  
  initial 
  begin
    TB_MISO_i = 1'b1;
  end
  
  Interface_SPI_Master
  #(
    .SPI_BAUDRATE (TB_SPI_BAUDRATE ), // SPI frequency in Hz
    .SYSTEM_FREQ  (TB_SYSTEM_FREQ  ), // System clock frequency in Hz
    .MAX_DATAW    (TB_MAX_DATAW    ), // Maximum SPI packet size
    .DEFAULT_CS   (TB_DEFAULT_CS   ), // Default CS   value on reset
    .DEFAULT_SCLK (TB_DEFAULT_SCLK )  // Default SCLK value on reset
  )
  UUT
  (
    .RST_i             (TB_RST_i             ), // System reset
    .CLK_i             (TB_CLK_i             ), // System clock
    .CS_o              (TB_CS_o              ), // 
    .SCLK_o            (TB_SCLK_o            ), // 
    .MOSI_o            (TB_MOSI_o            ), // 
    .MISO_i            (TB_MISO_i            ), // 
    .SET_Valid         (TB_SET_Valid         ), // Settings Valid (Can be set to '1', will just autoupdate between transfers)
    .SET_SPI_Mode      (TB_SET_SPI_Mode      ), // SPI mode (see table)
    .SET_MSB           (TB_SET_MSB           ), // '0' - LSB first, '1' - MSB first
    .SET_CS_Keep       (TB_SET_CS_Keep       ), // Keep CS Active after packet was sent
    .SET_CS_Pol        (TB_SET_CS_Pol        ), // CS polarity: '0' - Active LOW, '1' - Active HIGH
    .SET_CS_WaitLength (TB_SET_CS_WaitLength ), // >=0, Minimum CS length between deactivation and activation (in SCLK half-periods)
    .SET_CS_WaitStart  (TB_SET_CS_WaitStart  ), // >=0, Delay between CS activation and first SCLK edge       (in SCLK half-periods)
    .SET_CS_WaitEnd    (TB_SET_CS_WaitEnd    ), // >=1, Delay between last SCLK edge and CS deactivation      (in SCLK half-periods)
    .TRC_o             (TB_TRC_o             ), // '0' - Transmission in progress
    .DRE_o             (TB_DRE_o             ), // '0' - Buffer full
    .Valid_i           (TB_Valid_i           ), // Outgoing (MOSI) packet data valid
    .Data_DW           (TB_Data_DW           ), // Outgoing (MOSI) packet data width
    .Data_i            (TB_Data_i            ), // Outgoing (MOSI) packet data
    .Valid_o           (TB_Valid_o           ), // Incoming (MISO) packed data valid
    .Data_o            (TB_Data_o            )  // Incoming (MISO) packed data
  );
  
  
endmodule
