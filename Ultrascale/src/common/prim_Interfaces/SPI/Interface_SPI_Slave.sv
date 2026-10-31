`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/28/2026 07:01:03 PM
// Design Name: 
// Module Name: Interface_SPI_Slave
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
  
/*********************************************************************************
* 
* SPI Slave Modes:
* 
* |   |             |    SLAVE    |    MASTER   |
* | № | CPOL | PRSH | TXSE | RXSE | TXSE | RXSE | 
* | 0 |  LO  |  ++  | '\_' | '_/' | '\_' | '_/' | - Standard SPI mode 0
* | 1 |  LO  |  --  | '_/' | '\_' | '_/' | '\_' | - Standard SPI mode 1
* | 2 |  HI  |  ++  | '_/' | '\_' | '_/' | '\_' | - Standard SPI mode 2
* | 3 |  HI  |  --  | '\_' | '_/' | '\_' | '_/' | - Standard SPI mode 3
* | 4 |  LO  |  --  | '\_' | '\_' | '_/' | '_/' | - Custom   SPI mode 0 (sampling and shifting are both on falling edge)
* | 5 |  LO  |  ++  | '_/' | '_/' | '\_' | '\_' | - Custom   SPI mode 1 (sampling and shifting are both on rising  edge)
* | 6 |  HI  |  --  | '_/' | '_/' | '\_' | '\_' | - Custom   SPI mode 2 (sampling and shifting are both on rising  edge)
* | 7 |  HI  |  ++  | '\_' | '\_' | '_/' | '_/' | - Custom   SPI mode 3 (sampling and shifting are both on falling edge)
*
* CPOL - Clock Polarity (IDLE state value)
* PRSH - Data needs to be pre-latched before first SCLK edge
* RXSE - On which SCLK edge MISO data needs to be sampled
* TXSE - On which SCLK edge MOSI data needs to be shifted out
* 
* '_/' - Rising  edge 
* '\_' - Falling edge 
*
* It is assumed that SPI slave uses the same mode for the 0-3, 
*   and a different one from Master in pairs 4-5 and 6-7
*********************************************************************************/
  
module Interface_SPI_Slave
  #(
    parameter  int  MAX_DATAW    =     32 , // Maximum SPI packet size
    parameter  bit  DEFAULT_CS   =   1'b1 , // Default CS   value on reset
    parameter  bit  DEFAULT_SCLK =   1'b0 , // Default SCLK value on reset
    localparam int  MAX_DATABW   = $clog2(MAX_DATAW + 1)
  )
  (
    // Clocking
    input  logic                  RST_i             , // System reset
    input  logic                  CLK_i             , // System clock
    // SPI I/O
    input  logic                  CS_i              , // 
    input  logic                  SCLK_i            , // 
    input  logic                  MOSI_i            , // 
    output logic                  MISO_o            , // 
    // Settings
    input  logic                  SET_Valid         , // Settings Valid (Can be set to '1', will just autoupdate between transfers)
    input  logic [           2:0] SET_SPI_Mode      , // SPI mode (see table)
    input  logic                  SET_MSB           , // '0' - LSB first, '1' - MSB first
    input  logic                  SET_CS_Pol        , // CS polarity: '0' - Active LOW, '1' - Active HIGH
    // Parallel I/O
    input  logic                  Valid_i           , // Outgoing (MISO) packet data valid
    input  logic [MAX_DATABW-1:0] Data_DW           , // Outgoing (MISO) packet data width
    input  logic [ MAX_DATAW-1:0] Data_i            , // Outgoing (MISO) packet data
    output logic                  Valid_o           , // Incoming (MOSI) packed data valid
    output logic [ MAX_DATAW-1:0] Data_o              // Incoming (MOSI) packed data
  );
  
  
  typedef struct
  {
    logic                 CE    ;
    logic                 L     ;
    logic                 SLI   ;
    logic                 SRI   ;
    logic                 LEFT  ;
    logic [MAX_DATAW-1:0] D     ;
    logic [MAX_DATAW-1:0] Q     ;
  } sregi_t;
  sregi_t IData;
  
  typedef struct
  {
    logic                 CE    ;
    logic                 L     ;
    logic                 SLI   ;
    logic                 SRI   ;
    logic                 LEFT  ;
    logic [MAX_DATAW-1:0] D     ;
    logic                 SLO   ;
    logic                 SRO   ;
  } srego_t;
  srego_t OData;
  
  logic SET_CPOL    ; // '0' - Active LOW, '1' - Active HIGH
  logic SET_Preload ; // '1' - preload needed
  logic SET_RX_Edge ; // '0' - posedge, '1' - negedge
  logic SET_TX_Edge ; // '0' - posedge, '1' - negedge
  
  always_comb
  begin:spimode_comb
    case(SET_SPI_Mode)
      (3'd0)  : {SET_CPOL, SET_Preload, SET_RX_Edge, SET_TX_Edge} = 4'b0_1_01; // Standard SPI mode 0
      (3'd1)  : {SET_CPOL, SET_Preload, SET_RX_Edge, SET_TX_Edge} = 4'b0_0_10; // Standard SPI mode 1
      (3'd2)  : {SET_CPOL, SET_Preload, SET_RX_Edge, SET_TX_Edge} = 4'b1_1_10; // Standard SPI mode 2
      (3'd3)  : {SET_CPOL, SET_Preload, SET_RX_Edge, SET_TX_Edge} = 4'b1_0_01; // Standard SPI mode 3
      (3'd4)  : {SET_CPOL, SET_Preload, SET_RX_Edge, SET_TX_Edge} = 4'b0_0_11; // Custom   SPI mode 0 
      (3'd5)  : {SET_CPOL, SET_Preload, SET_RX_Edge, SET_TX_Edge} = 4'b0_1_00; // Custom   SPI mode 1 
      (3'd6)  : {SET_CPOL, SET_Preload, SET_RX_Edge, SET_TX_Edge} = 4'b1_0_00; // Custom   SPI mode 2 
      (3'd7)  : {SET_CPOL, SET_Preload, SET_RX_Edge, SET_TX_Edge} = 4'b1_1_11; // Custom   SPI mode 3 
      default : {SET_CPOL, SET_Preload, SET_RX_Edge, SET_TX_Edge} = 4'b0_0_00; // ERR
    endcase
  end
  
  typedef struct
  {
    logic MSB     ;
    logic CPOL    ;
    logic Preload ;
    logic RX_Edge ;
    logic TX_Edge ;
    logic CS_POL  ;
  } settings_t;
  settings_t SB; // Buffered settings
  
  always_ff @(posedge CLK_i)
  begin
    if(RST_i)
      SB <= '{default:0};
    else if(SET_Valid & ~CS_Active)
      SB <= '{
              MSB: SET_MSB, CPOL: SET_CPOL, Preload: SET_Preload, 
              RX_Edge: SET_RX_Edge, TX_Edge: SET_TX_Edge, CS_POL: SET_CS_Pol
            };
  end
  
  //------------------------------------------------------------------------------------------
  // CS detection
  //------------------------------------------------------------------------------------------
  
  logic CS_Buf     ; // 
  logic CS_EdgeP   ; // 
  logic CS_EdgeN   ; // 
  logic CS_Start   ; // 
  logic CS_End     ; // 
  logic CS_Active  ; // 
  
  always_comb
  begin
    CS_Active  = (CS_i == SB.CS_POL);
    CS_EdgeP   = ~CS_Buf &  CS_i;
    CS_EdgeN   =  CS_Buf & ~CS_i;
    CS_Start   = (SB.CS_POL)? CS_EdgeP : CS_EdgeN;
    CS_End     = (SB.CS_POL)? CS_EdgeN : CS_EdgeP;
  end
  
  always_ff @(posedge CLK_i)
  begin:cs_detect
    if(RST_i)
      CS_Buf <= DEFAULT_CS;
    else
      CS_Buf <= CS_i;
  end
  
  
  //------------------------------------------------------------------------------------------
  // SCLK Detection
  //------------------------------------------------------------------------------------------
  
  logic SCLK_Buf   ; // 
  logic SCLK_EdgeP ; // 
  logic SCLK_EdgeN ; // 
  logic TX_Latch   ; // 
  logic RX_Latch   ; // 
  
  always_comb
  begin
    SCLK_EdgeP = ~SCLK_Buf &  SCLK_i & CS_Active;
    SCLK_EdgeN =  SCLK_Buf & ~SCLK_i & CS_Active;
    TX_Latch   = (SB.TX_Edge)? SCLK_EdgeN : SCLK_EdgeP;
    RX_Latch   = (SB.RX_Edge)? SCLK_EdgeN : SCLK_EdgeP;
  end
  
  always_ff @(posedge CLK_i)
  begin:sclk_detect
    if(RST_i)
      SCLK_Buf <= DEFAULT_SCLK;
    else
      SCLK_Buf <= SCLK_i;
  end
  
  //------------------------------------------------------------------------------------------
  // MOSI Preload and output
  //------------------------------------------------------------------------------------------
  
  logic MISO_Sel;
  
  always_ff @(posedge CLK_i)
  begin:miso_out
    if(RST_i)
      MISO_o <= 1'b0;
    else if(TX_Latch | (CS_Start & SB.Preload))
      MISO_o <= MISO_Sel;
  end
  
  always_comb
  begin
    OData.CE    = TX_Latch | (CS_Start & SB.Preload);
    OData.L     = Valid_i & ~CS_Active  ;
    OData.SLI   = 1'b0                  ;
    OData.SRI   = 1'b0                  ;
    OData.LEFT  = SB.MSB                ;
    OData.D     = Data_i                ;
    MISO_Sel    = (SB.MSB)? OData.SLO : OData.SRO;
  end
  
  prim_SRxWRLED
  #(
    .DATA_W  (MAX_DATAW), // Shift register length
    .RST_POL (1'b0     ), // Default value of shift register
    .CLK_POL (1'b0     )  // '0' - posedge, '1' - negedge
  )
  MISO_Shift
  (
    .C    (CLK_i          ), // Clock
    .R    (RST_i || CS_End), // Reset
    .CE   (OData.CE       ), // Clock Enable
    .L    (OData.L        ), // Load Enable
    .SLI  (OData.SLI      ), // Shift-left  IN
    .SRI  (OData.SRI      ), // Shift-right IN
    .LEFT (OData.LEFT     ), // Left('1', Qx <- Q0) / Right('0', Qx -> Q0)
    .D    (OData.D        ), // Parallel Input
    .Q    (               ), // Parallel Output
    .SLO  (OData.SLO      ), // Shift-left  OUT
    .SRO  (OData.SRO      )  // Shift-right OUT
  );
  
  always_comb
  begin
    IData.CE    = RX_Latch    ;
    IData.L     = 1'b0        ;
    IData.SLI   = MOSI_i      ;
    IData.SRI   = MOSI_i      ;
    IData.LEFT  = SB.MSB      ;
    IData.D     = '{default:0};
  end
  
  prim_SRxWRLED
  #(
    .DATA_W  (MAX_DATAW), // Shift register length
    .RST_POL (1'b0     ), // Default value of shift register
    .CLK_POL (1'b0     )  // '0' - posedge, '1' - negedge
  )
  MOSI_Shift
  (
    .C    (CLK_i            ), // Clock
    .R    (RST_i || CS_Start), // Reset
    .CE   (IData.CE         ), // Clock Enable
    .L    (IData.L          ), // Load Enable
    .SLI  (IData.SLI        ), // Shift-left  IN
    .SRI  (IData.SRI        ), // Shift-right IN
    .LEFT (IData.LEFT       ), // Left('1', Qx <- Q0) / Right('0', Qx -> Q0)
    .D    (IData.D          ), // Parallel Input
    .Q    (IData.Q          ), // Parallel Output
    .SLO  (                 ), // Shift-left  OUT
    .SRO  (                 )  // Shift-right OUT
  );
  
endmodule
