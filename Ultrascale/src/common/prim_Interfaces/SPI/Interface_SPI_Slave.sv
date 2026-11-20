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
    parameter  int         MAX_DW        =    8 , // Maximum SPI packet size
    parameter  bit         DEFAULT_SCLK  = 1'b0 , // Default SCLK value on reset
    parameter  bit         MODE_VAR_WORD = 1'b0 , // '0' - Word size == MAX_DW, '1' - Word size <= MAX_DW, controlled by SET_Data_DW_i
    parameter  bit         MODE_VAR_SPIM = 1'b0 , // '0' - Fixed SPI mode (MODE_TYP_SPIM), '1' - controlled by SET_SPI_Mode_i
    parameter  logic [2:0] MODE_TYP_SPIM = 3'd0 , // Fixed mode SPI mode value
    parameter  bit         MODE_VAR_CS   = 1'b0 , // '0' - Fixed CS  mode (MODE_TYP_CS), '1' - controlled by SET_CS_Pol_i
    parameter  bit         MODE_TYP_CS   = 1'b1 , // Fixed CS polarity: '0' - Active LOW, '1' - Active HIGH
    parameter  bit         MODE_VAR_MSB  = 1'b0 , // '0' - Fixed MSB mode (MODE_TYP_MSB), '1' - controlled by SET_MSB_i
    parameter  bit         MODE_TYP_MSB  = 1'b1 , // Fixed bit order mode: '0' - LSB (default), '1' - MSB
    localparam int         MAX_DBW       = $clog2(MAX_DW + 1)
  )
  (
    // Clocking
    input  logic               RST_i             , // System reset
    input  logic               CLK_i             , // System clock
    // SPI I/O
    input  logic               CS_i              , // 
    input  logic               SCLK_i            , // 
    input  logic               MOSI_i            , // 
    output logic               MISO_o            , // 
    // Settings
    input  logic               SET_Valid_i       , // Settings Valid (Can be set to '1', will just autoupdate between transfers)
    input  logic [        2:0] SET_SPI_Mode_i    , // SPI mode (see table)
    input  logic               SET_MSB_i         , // '0' - LSB first, '1' - MSB first
    input  logic               SET_CS_Pol_i      , // CS polarity: '0' - Active LOW, '1' - Active HIGH
    input  logic [MAX_DBW-1:0] SET_Data_DW_i     , // Outgoing (MISO) packet data width
    // Parallel I/O
    input  logic               Valid_i           , // Outgoing (MISO) packet data valid
    input  logic [ MAX_DW-1:0] Data_i            , // Outgoing (MISO) packet data
    output logic               Valid_o           , // Incoming (MOSI) packed data valid
    output logic [ MAX_DW-1:0] Data_o              // Incoming (MOSI) packed data
  );
  
  
  typedef struct
  {
    logic              CE    ;
    logic              L     ;
    logic              SLI   ;
    logic              SRI   ;
    logic              LEFT  ;
    logic [MAX_DW-1:0] D     ;
    logic [MAX_DW-1:0] Q     ;
  } sregi_t;
  sregi_t IData;
  
  typedef struct
  {
    logic              CE    ;
    logic              L     ;
    logic              SLI   ;
    logic              SRI   ;
    logic              LEFT  ;
    logic [MAX_DW-1:0] D     ;
    logic              SLO   ;
    logic              SRO   ;
  } srego_t;
  srego_t OData;
  
  logic [2:0] SET_SPI_Mode  ; // Routing
  logic       SET_Preload   ; // '1' - preload needed
  logic       SET_RX_Edge   ; // '0' - posedge, '1' - negedge
  logic       SET_TX_Edge   ; // '0' - posedge, '1' - negedge
  
  always_comb
  begin:spimode_comb
    case(SET_SPI_Mode)
      (3'd0)  : {SET_Preload, SET_RX_Edge, SET_TX_Edge} = 3'b1_01; // Standard SPI mode 0
      (3'd1)  : {SET_Preload, SET_RX_Edge, SET_TX_Edge} = 3'b0_10; // Standard SPI mode 1
      (3'd2)  : {SET_Preload, SET_RX_Edge, SET_TX_Edge} = 3'b1_10; // Standard SPI mode 2
      (3'd3)  : {SET_Preload, SET_RX_Edge, SET_TX_Edge} = 3'b0_01; // Standard SPI mode 3
      (3'd4)  : {SET_Preload, SET_RX_Edge, SET_TX_Edge} = 3'b0_11; // Custom   SPI mode 0 
      (3'd5)  : {SET_Preload, SET_RX_Edge, SET_TX_Edge} = 3'b1_00; // Custom   SPI mode 1 
      (3'd6)  : {SET_Preload, SET_RX_Edge, SET_TX_Edge} = 3'b0_00; // Custom   SPI mode 2 
      (3'd7)  : {SET_Preload, SET_RX_Edge, SET_TX_Edge} = 3'b1_11; // Custom   SPI mode 3 
      default : {SET_Preload, SET_RX_Edge, SET_TX_Edge} = 3'b0_00; // ERR
    endcase
  end
  
  // Buffered settings
  logic               SB_MSB     ;
  logic               SB_Preload ;
  logic               SB_RX_Edge ;
  logic               SB_TX_Edge ;
  logic               SB_CS_POL  ;
  logic [MAX_DBW-1:0] SB_DATAW   ;
  
  
  generate
    if(MODE_VAR_WORD)
    begin
      always_ff @(posedge CLK_i)
      begin
        if(RST_i)
          SB_DATAW <= '{default:0};
        else if(SET_Valid_i & ~CS_Active)
          SB_DATAW <= SET_Data_DW_i;
      end
    end
    else
    begin
      assign SB_DATAW = MAX_DBW'(MAX_DW);
    end
  endgenerate
  
  generate
    if(MODE_VAR_SPIM)
    begin
      assign SET_SPI_Mode = SET_SPI_Mode_i;
      always_ff @(posedge CLK_i)
      begin
        if(RST_i)
          {SB_Preload, SB_RX_Edge, SB_TX_Edge} <= '{default:0};
        else if(SET_Valid_i & ~CS_Active)
          {SB_Preload, SB_RX_Edge, SB_TX_Edge} <= {SET_Preload, SET_RX_Edge, SET_TX_Edge};
      end
    end
    else
    begin
      assign SET_SPI_Mode = MODE_TYP_SPIM; // 
      assign {SB_Preload, SB_RX_Edge, SB_TX_Edge} = {SET_Preload, SET_RX_Edge, SET_TX_Edge};
    end
  endgenerate
  
  generate
    if(MODE_VAR_CS)
    begin
      always_ff @(posedge CLK_i)
      begin
        if(RST_i)
          SB_CS_POL <= 1'b0;
        else if(SET_Valid_i & ~CS_Active)
          SB_CS_POL <= SET_CS_Pol_i;
      end
    end
    else
    begin
      assign SB_CS_POL = MODE_TYP_CS;
    end
  endgenerate
  
  generate
    if(MODE_VAR_MSB)
    begin
      always_ff @(posedge CLK_i)
      begin
        if(RST_i)
          SB_MSB <= 1'b0;
        else if(SET_Valid_i & ~CS_Active)
          SB_MSB <= SET_MSB_i;
      end
    end
    else
    begin
      assign SB_MSB = MODE_TYP_MSB;
    end
  endgenerate
  
  logic [MAX_DW-1:0] MSB_Flag;
  
  prim_OneHot_Conv 
  #(
    .BIN_W (MAX_DBW), // binary input width (derived, normally leave default)
    .OUT_W (MAX_DW )  // one-hot output width
  )
  DataW_OneHot
  (
    .Binary_i (SB_DATAW - 1'd1), // normal (binary-encoded) input, valid range 0 .. OUT_W-1
    .OneHot_o (MSB_Flag       )  // one-hot output: onehot_out[bin_in] = 1'b1
  );
  
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
    CS_Active  = (CS_i == SB_CS_POL);
    CS_EdgeP   = ~CS_Buf &  CS_i;
    CS_EdgeN   =  CS_Buf & ~CS_i;
    CS_Start   = (SB_CS_POL)? CS_EdgeP : CS_EdgeN;
    CS_End     = (SB_CS_POL)? CS_EdgeN : CS_EdgeP;
  end
  
  always_ff @(posedge CLK_i)
  begin:cs_detect
    if(RST_i)
      CS_Buf <= ~MODE_TYP_CS;
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
    TX_Latch   = (SB_TX_Edge)? SCLK_EdgeN : SCLK_EdgeP;
    RX_Latch   = (SB_RX_Edge)? SCLK_EdgeN : SCLK_EdgeP;
  end
  
  always_ff @(posedge CLK_i)
  begin:sclk_detect
    if(RST_i)
      SCLK_Buf <= DEFAULT_SCLK;
    else
      SCLK_Buf <= SCLK_i;
  end
  
  //------------------------------------------------------------------------------------------
  // MISO Preload and output
  //------------------------------------------------------------------------------------------
  
  logic MISO_Sel;
  
  always_ff @(posedge CLK_i)
  begin:miso_out
    if(RST_i)
      MISO_o <= 1'b0;
    else if(TX_Latch | (CS_Start & SB_Preload))
      MISO_o <= MISO_Sel;
  end
  
  always_comb
  begin
    OData.CE    = TX_Latch | (CS_Start & SB_Preload);
    OData.L     = Valid_i & ~CS_Active  ;
    OData.SLI   = 1'b0                  ;
    OData.SRI   = 1'b0                  ;
    OData.LEFT  = SB_MSB                ;
    OData.D     = Data_i                ;
    MISO_Sel    = (SB_MSB)? OData.SLO : OData.SRO;
  end
  
  prim_SRxWRLED
  #(
    .DATA_W  (MAX_DW), // Shift register length
    .RST_POL (1'b0  ), // Default value of shift register
    .CLK_POL (1'b0  )  // '0' - posedge, '1' - negedge
  )
  MISO_Shift
  (
    .C    (CLK_i            ), // Clock
    .R    (RST_i || CS_End  ), // Reset
    .CE   (OData.CE         ), // Clock Enable
    .L    (OData.L          ), // Load Enable
    .SLI  (OData.SLI        ), // Shift-left  IN
    .SRI  (OData.SRI        ), // Shift-right IN
    .LEFT (OData.LEFT       ), // Left('1', Qx <- Q0) / Right('0', Qx -> Q0)
    .M    (MSB_Flag         ), // Parallel Mask Input (one-hot MSB-end cell of window [top:0])
    .D    (OData.D          ), // Parallel Input
    .Q    (                 ), // Parallel Output
    .SLO  (OData.SLO        ), // Shift-left  OUT
    .SRO  (OData.SRO        )  // Shift-right OUT
  );
  
  
  //------------------------------------------------------------------------------------------
  // MOSI Load and output
  //------------------------------------------------------------------------------------------
  
  always_ff @(posedge CLK_i)
  begin:mosi_in
    if(RST_i)
    begin
      Valid_o <= 1'b0;
      Data_o  <= '{default:0};
    end
    else 
    begin
      Valid_o <= 1'b0;
      
      if(CS_End)
      begin
        Valid_o <= 1'b1;
        Data_o  <= IData.Q;
      end
    end
  end
  
  always_comb
  begin
    IData.CE    = RX_Latch    ;
    IData.L     = 1'b0        ;
    IData.SLI   = MOSI_i      ;
    IData.SRI   = MOSI_i      ;
    IData.LEFT  = SB_MSB      ;
    IData.D     = '{default:0};
  end
  
  prim_SRxWRLED
  #(
    .DATA_W  (MAX_DW), // Shift register length
    .RST_POL (1'b0  ), // Default value of shift register
    .CLK_POL (1'b0  )  // '0' - posedge, '1' - negedge
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
    .M    (MSB_Flag         ), // Parallel Mask Input (one-hot MSB-end cell of window [top:0])
    .D    (IData.D          ), // Parallel Input
    .Q    (IData.Q          ), // Parallel Output
    .SLO  (                 ), // Shift-left  OUT
    .SRO  (                 )  // Shift-right OUT
  );
  
endmodule
