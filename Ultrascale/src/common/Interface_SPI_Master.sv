`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/21/2026 07:01:03 PM
// Design Name: 
// Module Name: Interface_SPI_Master
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
* SPI Master Modes:
* 
* |   |             |    MASTER   |    SLAVE    |
* | № | CPOL | PRSH | TXSE | RXSE | TXSE | RXSE | 
* | 0 |  LO  |  ++  | '\_' | '_/' | '\_' | '_/' | - Standard SPI mode 0
* | 1 |  LO  |  --  | '_/' | '\_' | '_/' | '\_' | - Standard SPI mode 1
* | 2 |  HI  |  ++  | '_/' | '\_' | '_/' | '\_' | - Standard SPI mode 2
* | 3 |  HI  |  --  | '\_' | '_/' | '\_' | '_/' | - Standard SPI mode 3
* | 4 |  LO  |  --  | '_/' | '_/' | '\_' | '\_' | - Custom   SPI mode 0 (sampling and shifting are both on rising  edge)
* | 5 |  LO  |  ++  | '\_' | '\_' | '_/' | '_/' | - Custom   SPI mode 1 (sampling and shifting are both on falling edge)
* | 6 |  HI  |  --  | '\_' | '\_' | '_/' | '_/' | - Custom   SPI mode 2 (sampling and shifting are both on falling edge)
* | 7 |  HI  |  ++  | '_/' | '_/' | '\_' | '\_' | - Custom   SPI mode 3 (sampling and shifting are both on rising  edge)
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
  
module Interface_SPI_Master
  #(
    parameter  real SPI_BAUDRATE =   5e+6 , // SPI frequency in Hz
    parameter  real SYSTEM_FREQ  = 100e+6 , // System clock frequency in Hz
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
    output logic                  CS_o              , // 
    output logic                  SCLK_o            , // 
    output logic                  MOSI_o            , // 
    input  logic                  MISO_i            , // 
    // Settings
    input  logic                  SET_Valid         , // Settings Valid (Can be set to '1', will just autoupdate between transfers)
    input  logic [           2:0] SET_SPI_Mode      , // SPI mode (see table)
    input  logic                  SET_MSB           , // '0' - LSB first, '1' - MSB first
    input  logic                  SET_CS_Keep       , // Keep CS Active after packet was sent
    input  logic                  SET_CS_Pol        , // CS polarity: '0' - Active LOW, '1' - Active HIGH
    input  logic [           5:0] SET_CS_WaitLength , // >=1, Minimum CS length between deactivation and activation (in SCLK half-periods)
    input  logic [           5:0] SET_CS_WaitStart  , // >=0, Delay between CS activation and first SCLK edge       (in SCLK half-periods)
    input  logic [           5:0] SET_CS_WaitEnd    , // >=0, Delay between last SCLK edge and CS deactivation      (in SCLK half-periods)
    // Parallel I/O
    output logic                  TRC_o             , // '0' - Transmission in progress
    output logic                  DRE_o             , // '0' - Buffer full
    input  logic                  Valid_i           , // Outgoing (MOSI) packet data valid
    input  logic [MAX_DATABW-1:0] Data_DW           , // Outgoing (MOSI) packet data width
    input  logic [ MAX_DATAW-1:0] Data_i            , // Outgoing (MOSI) packet data
    output logic                  Valid_o           , // Incoming (MISO) packed data valid
    output logic [ MAX_DATAW-1:0] Data_o              // Incoming (MISO) packed data
  );
  
  localparam int SCLK_DIV_W  = (SYSTEM_FREQ/SPI_BAUDRATE)/2;
  localparam int SCLK_DIV_BW = $clog2(SCLK_DIV_W);
  
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
      (3'd4)  : {SET_CPOL, SET_Preload, SET_RX_Edge, SET_TX_Edge} = 4'b0_0_00; // Custom   SPI mode 0 
      (3'd5)  : {SET_CPOL, SET_Preload, SET_RX_Edge, SET_TX_Edge} = 4'b0_1_11; // Custom   SPI mode 1 
      (3'd6)  : {SET_CPOL, SET_Preload, SET_RX_Edge, SET_TX_Edge} = 4'b1_0_11; // Custom   SPI mode 2 
      (3'd7)  : {SET_CPOL, SET_Preload, SET_RX_Edge, SET_TX_Edge} = 4'b1_1_00; // Custom   SPI mode 3 
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
    
    struct
    {
      logic                  Keep       ;
      logic                  Pol        ;
      logic [           5:0] WaitLength ;
      logic [           5:0] WaitStart  ;
      logic [           5:0] WaitEnd    ;
    } CS;
  } settings_t;
  settings_t SB; // Buffered settings
  
  typedef struct
  {
    logic                 CE    ;
    logic                 L     ;
    logic                 SLI   ;
    logic                 SRI   ;
    logic                 LEFT  ;
    logic [MAX_DATAW-1:0] D     ;
    logic [MAX_DATAW-1:0] Q     ;
  } sreg_t;
  sreg_t IData, OData;
  
  
  typedef struct
  {
    logic [MAX_DATABW-1:0] DW   ;
    logic [ MAX_DATAW-1:0] Data ;
  } dbuf_t;
  dbuf_t Buffer, Trans;
  
  typedef enum bit [2:0]
  {
    IDLE      = 3'd0,
    SETTINGS  = 3'd1,
    PRELOAD   = 3'd2,
    TRANSMIT  = 3'd3,
    CS_OFF    = 3'd4,
    COOLDOWN  = 3'd5
  } state_e;
  state_e State;
  
  logic                   CS_Buf      ;
  logic                   SCLK_Buf    ;
  logic                   MOSI_Buf    ;
  logic                   MOSI_Sel    ;
  logic                   TRC_Buf     ;
  logic                   DRE_Buf     ;
  
  logic [SCLK_DIV_BW-1:0] SCLK_Div    ;
  logic                   SCLK_HBit   ;
  logic                   SCLK_EdgeP  ;
  logic                   SCLK_EdgeN  ;
  
  logic                   Edge_TX          ; // Shift out MOSI
  logic                   Edge_RX          ; // Shift in  MISO
  logic [MAX_DATABW-1:0]  BitCount_TX      ; // 
  logic [MAX_DATABW-1:0]  BitCount_RX      ; // 
  logic                   BitCount_TX_Done ; // 
  logic                   BitCount_RX_Done ; // 
  logic                   FastPreload      ; // Skips waiting for Half-Bit if it's second+ packet
  logic [MAX_DATABW-1:0]  MISO_BitShift    ; // 
  
  logic [           5:0] Count_WaitLength  ; // 
  logic [           5:0] Count_WaitStart   ; // 
  logic [           5:0] Count_WaitEnd     ; // 
  logic                  CDone_WaitLength  ; // 
  logic                  CDone_WaitStart   ; // 
  logic                  CDone_WaitEnd     ; // 
  logic                  DW_IsEmpty        ; // 
  
  always_comb
  begin
    CS_o   = CS_Buf   ;
    SCLK_o = SCLK_Buf ;
    MOSI_o = MOSI_Buf ;
    TRC_o  = TRC_Buf  ;
    DRE_o  = DRE_Buf  ;
  end
  
  always_comb
  begin
    SCLK_HBit        = (SCLK_Div >= (SCLK_DIV_W - 1));
    SCLK_EdgeP       = SCLK_HBit & ~SCLK_Buf;
    SCLK_EdgeN       = SCLK_HBit &  SCLK_Buf;
    Edge_TX          = (SB.TX_Edge)? SCLK_EdgeN : SCLK_EdgeP;
    Edge_RX          = (SB.RX_Edge)? SCLK_EdgeN : SCLK_EdgeP;
    BitCount_TX_Done = BitCount_TX >= Trans.DW;
    BitCount_RX_Done = BitCount_RX >= Trans.DW;
    MISO_BitShift    = MAX_DATAW - Trans.DW;
    CDone_WaitLength = Count_WaitLength >= SB.CS.WaitLength ;
    CDone_WaitStart  = Count_WaitStart  >= SB.CS.WaitStart  ;
    CDone_WaitEnd    = Count_WaitEnd    >= SB.CS.WaitEnd    ;
    DW_IsEmpty       = (Trans.DW == 'd0);
    MOSI_Sel         = (SB.MSB & ~DW_IsEmpty)? Trans.Data[Trans.DW-1] : Trans.Data[0];
  end
  
  always_ff @(posedge CLK_i)
  begin:clkdiv_proc
    if(RST_i || SCLK_HBit)
      SCLK_Div <= 'd0;
    else
      SCLK_Div <= SCLK_Div + 'd1;
  end
  
  always_ff @(posedge CLK_i)
  begin
    if(RST_i)
      SB <= '{default:0};
    else if(SET_Valid && State == IDLE)
      SB <= '{
              MSB: SET_MSB, CPOL: SET_CPOL, Preload: SET_Preload, 
              RX_Edge: SET_RX_Edge, TX_Edge: SET_TX_Edge,
              CS:'{
                Keep: SET_CS_Keep, Pol: SET_CS_Pol, 
                WaitLength: SET_CS_WaitLength     , 
                WaitStart:  SET_CS_WaitStart      , 
                WaitEnd:    SET_CS_WaitEnd        } 
            };
  end
  
  always_ff @(posedge CLK_i)
  begin:wait_proc
    if(RST_i || State == IDLE)
    begin
      Count_WaitStart  <= 'd0;
      Count_WaitEnd    <= 'd0;
      Count_WaitLength <= 'd0;
    end
    else
    begin
      if(SCLK_HBit)
      begin
        case(State)
          TRANSMIT : if(~CDone_WaitStart ) Count_WaitStart  <= Count_WaitStart  + 'd1;
          CS_OFF   : if(~CDone_WaitEnd   ) Count_WaitEnd    <= Count_WaitEnd    + 'd1;
          COOLDOWN : if(~CDone_WaitLength) Count_WaitLength <= Count_WaitLength + 'd1;
        endcase
      end
    end
  end
  
  always_ff @(posedge CLK_i)
  begin:main_proc
    if(RST_i)
    begin
      CS_Buf           <= DEFAULT_CS;
      SCLK_Buf         <= DEFAULT_SCLK;
      MOSI_Buf         <= 1'b0;
      TRC_Buf          <= 1'b1;
      DRE_Buf          <= 1'b1;
      Buffer           <= '{default:0}; 
      Trans.DW         <= 'd0;
      BitCount_TX      <= 'd0;
      BitCount_RX      <= 'd0;
      FastPreload      <= 1'b0;
      Valid_o          <= 1'b0;
      Data_o           <= '{default:0}; 
      State            <= IDLE;
    end
    else
    begin
      Valid_o <= 1'b0;
      
      if(DRE_Buf & Valid_i)
      begin
        Buffer  <= '{DW: Data_DW, Data: Data_i};
        DRE_Buf <= 1'b0;
        
      end
      
      case(State)
        IDLE     :
        begin
          TRC_Buf <= 1'b1;
          if(~DRE_Buf)
          begin
            State    <= SETTINGS ;
            Trans.DW <= Buffer.DW;
            TRC_Buf  <= 1'b0     ;
          end
        end
        
        SETTINGS :
        begin
          if(DW_IsEmpty)
          begin
            DRE_Buf <= 1'b1;
            TRC_Buf <= 1'b1;
            State   <= IDLE;
          end
          else
          begin
            SCLK_Buf    <= SB.CPOL    ;
            State       <= PRELOAD    ;
            if(~SB.CS.Keep) CS_Buf <= ~SB.CS.Pol ;
          end
        end
        
        PRELOAD  :
        begin
          BitCount_TX <= 'd0;
          BitCount_RX <= 'd0;
          
          if(SCLK_HBit | FastPreload)
          begin
            CS_Buf      <= SB.CS.Pol ;
            State       <= TRANSMIT  ;
            DRE_Buf     <= 1'b1      ;
            FastPreload <= 1'b0      ;
            if(SB.Preload)
            begin
              MOSI_Buf <= MOSI_Sel;
              BitCount_TX <= 'd1;
            end
          end
        end
        
        TRANSMIT :
        begin
          // Clock data I/O by half-periods of SCLK
          if(SCLK_HBit & CDone_WaitStart)
          begin
            SCLK_Buf <= ~SCLK_Buf;
            if(Edge_TX) MOSI_Buf    <= MOSI_Sel;
            if(Edge_TX) BitCount_TX <= BitCount_TX + 1'd1;
            if(Edge_RX) BitCount_RX <= BitCount_RX + 1'd1;
          end
          
          if(BitCount_TX_Done & BitCount_RX_Done)
          begin
            Valid_o <= 1'b1;
            Data_o  <= IData.Q >> ((SB.MSB)? 1'd0 : MISO_BitShift);
            
            if(CDone_WaitLength && ~DRE_Buf)
            begin
              Trans.DW    <= Buffer.DW;
              FastPreload <= 1'b1;
              State       <= PRELOAD;
            end
            else
              State   <= CS_OFF;
          end
          
        end
        
        CS_OFF   :
        begin
          if(SCLK_HBit & CDone_WaitEnd)
          begin
            if(~SB.CS.Keep) CS_Buf <= ~SB.CS.Pol;
            State  <= (CDone_WaitLength)? IDLE : COOLDOWN;
          end
        end
        
        COOLDOWN :
        begin
          if(CDone_WaitLength) State <= IDLE;
        end
        
        default:;
      endcase
    end
  end
  
  always_comb
  begin
    case(State)
      PRELOAD : OData.CE = ((SCLK_HBit | FastPreload) & SB.Preload)? 1'b1 : 1'b0;
      TRANSMIT: OData.CE = (Edge_TX & ~BitCount_TX_Done & CDone_WaitStart);
      default : OData.CE = 1'b0;
    endcase
    
    case(State)
      IDLE     : OData.L = (~DRE_Buf);
      TRANSMIT : OData.L = (BitCount_TX_Done & BitCount_RX_Done & CDone_WaitLength && ~DRE_Buf);
      default  : OData.L = 1'b0;
    endcase
    
    OData.SLI   = 1'b0        ;
    OData.SRI   = 1'b0        ;
    OData.LEFT  = SB.MSB      ;
    OData.D     = Buffer.Data ;
    Trans.Data  = OData.Q     ;
  end
  
  prim_SRxRLED
  #(
    .DATA_W  (MAX_DATAW), // Shift register length
    .RST_POL (1'b0     ), // Default value of shift register
    .CLK_POL (1'b0     )  // '0' - posedge, '1' - negedge
  )
  MOSI_Shift
  (
    .C    (CLK_i      ), // Clock
    .R    (RST_i      ), // Reset
    .CE   (OData.CE   ), // Clock Enable
    .L    (OData.L    ), // Load Enable
    .SLI  (OData.SLI  ), // Shift-left  IN
    .SRI  (OData.SRI  ), // Shift-right IN
    .LEFT (OData.LEFT ), // Left('1', Qx <- Q0) / Right('0', Qx -> Q0)
    .D    (OData.D    ), // Parallel Input
    .Q    (OData.Q    ), // Parallel Output
    .SLO  (           ), // Shift-left  OUT
    .SRO  (           )  // Shift-right OUT
  );
  
  always_comb
  begin
    case(State)
      TRANSMIT: IData.CE = (Edge_RX & ~BitCount_RX_Done & CDone_WaitStart);
      default : IData.CE = 1'b0;
    endcase
    
    case(State)
      IDLE     : IData.L = (~DRE_Buf);
      TRANSMIT : IData.L = (BitCount_TX_Done & BitCount_RX_Done & CDone_WaitLength && ~DRE_Buf);
      default  : IData.L = 1'b0;
    endcase
    
    IData.SLI   = MISO_i      ;
    IData.SRI   = MISO_i      ;
    IData.LEFT  = SB.MSB      ;
    IData.D     = {default:0} ;
  end
  
  prim_SRxRLED
  #(
    .DATA_W  (MAX_DATAW), // Shift register length
    .RST_POL (1'b0     ), // Default value of shift register
    .CLK_POL (1'b0     )  // '0' - posedge, '1' - negedge
  )
  MISO_Shift
  (
    .C    (CLK_i      ), // Clock
    .R    (RST_i      ), // Reset
    .CE   (IData.CE   ), // Clock Enable
    .L    (IData.L    ), // Load Enable
    .SLI  (IData.SLI  ), // Shift-left  IN
    .SRI  (IData.SRI  ), // Shift-right IN
    .LEFT (IData.LEFT ), // Left('1', Qx <- Q0) / Right('0', Qx -> Q0)
    .D    (IData.D    ), // Parallel Input
    .Q    (IData.Q    ), // Parallel Output
    .SLO  (           ), // Shift-left  OUT
    .SRO  (           )  // Shift-right OUT
  );
  
  
  
endmodule
