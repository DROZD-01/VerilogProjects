`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/10/2026 04:04:03 PM
// Design Name: 
// Module Name: Interface_UART
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


module Interface_UART
  #(
    parameter real FREQ_SYSCLK     = 100e6    , // System clock frequency (in Hz)
    parameter bit  MODE_BAUDCTRL   = 1'b0     , // Baudrate control mode: '1' - fixed, '0' - variable
    parameter int  BAUD_UART_DEF   = 115200   , // UART baudrate (for fixed mode)
    parameter int  MAX_DW          = 8        , // Maximum data bits in a word
    parameter bit  MODE_FULLDUPLEX = 1'b0     , // Enable full-duplex mode
    parameter bit  EN_RECEIVER     = 1'b1     , // Enable receiver circuitry generation
    parameter bit  EN_TRANSMITTER  = 1'b1     , // Enable transmitter circuitry generation
    parameter bit  MODE_RE_POL     = 1'b0     , // RE port polarity: '1' - active high / '0' - active low
    parameter bit  MODE_DE_POL     = 1'b1     , // DE port polarity: '1' - active high / '0' - active low
    parameter bit  MODE_VAR_WORD   = 1'b0     , // '0' - Word size == MAX_DW, '1' - Word size <= MAX_DW, controlled by DataBits_i
    parameter bit  MODE_MSB        = 1'b0     , // '0' - LSB (default), '1' - MSB
    parameter bit  MODE_VAR_PAR    = 1'b0     , // '0' - Parity type == MODE_TYP_PAR, '1' - Parity type is controlled by ParityType_i
    parameter int  MODE_TYP_PAR    = 1        , // Parity type (in fixed mode): '0' - N, '1' - E, '2' - O, '3' - M, '4' - S.
    parameter bit  MODE_VAR_STP    = 1'b0     , // '0' - Stop bits == MODE_TYP_STP, '1' - Stop bits <= 3, is controlled by StopBits_i
    parameter int  MODE_TYP_STP    = 1          // Maximum stop bits in a word
  )
  (
    input  logic                        RST_i         , // System reset
    input  logic                        CLK_i         , // System clock
    // Data I/O
    input  logic                        DataValid_i   , // 
    input  logic [          MAX_DW-1:0] Data_i        , // 
    output logic                        DataValid_o   , // 
    output logic [          MAX_DW-1:0] Data_o        , // 
    // Settings
    input  logic [                 4:0] BaudSel_i     , // 
    input  logic [$clog2(MAX_DW+1)-1:0] DataBits_i    , // 
    input  logic [                 2:0] ParityType_i  , // 
    input  logic [                 1:0] StopBits_i    , // 
    // Indicators
    output logic                        RX_Active_o   , // 
    output logic                        ParityError_o , // 
    output logic                        FrameError_o  , // 
    output logic                        TX_Active_o   , // 
    output logic                        BuffFull_o    , // 
    // External I/O
    output logic                        RE_o          , // Receiver    enable (for external ICs)
    output logic                        DE_o          , // Transmitter enable (for external ICs)
    input  logic                        RX_i          , // 
    output logic                        TX_o            // 
  );
  
  localparam OVERSAMPLE_N = 16; // Number of samples in one bit
  
  logic TickPulse_RX  ; // 
  logic TickResync_RX ; // 
  logic Inhibit_RX    ; // 
  
  logic TickPulse_TX  ; // 
  logic TickResync_TX ; // 
  logic Inhibit_TX    ; // 
  
  //------------------------------------------------------------------------------------------
  //  Baudrate clock gen
  //------------------------------------------------------------------------------------------
  generate
    if(MODE_FULLDUPLEX == 1'b1) // Full duplex mode
    begin:fulldupl_gen
      
      always_comb
      begin
        Inhibit_RX = 1'b0;
        Inhibit_TX = 1'b0;
        RE_o = (MODE_RE_POL)? 1'b1 : 1'b0;
        DE_o = (MODE_DE_POL)? 1'b1 : 1'b0;
      end
      
      prim_FractionalTickGen
      #(
        .FREQ_SYSCLK (FREQ_SYSCLK    ), // System clock frequency (in Hz)
        .MODE_FIXED  (MODE_BAUDCTRL  ), // Generation mode:'1' - fixed tick rate, '0' - variable tick rate
        .TICKR_FIXED (BAUD_UART_DEF  ), // Target tickrate (used in fixed mode)
        .OVERSAMPLE  (OVERSAMPLE_N   ), // Oversampling mode (ticks per bit)
        .LOGIC_DELAY (2              )  // 
      )
      TickGen_RX
      (
        .RST_i       (RST_i         ), // System reset
        .CLK_i       (CLK_i         ), // System clock
        .BaudSel_i   (BaudSel_i     ), // 
        .Resync_i    (TickResync_RX ), // Clear and start counting
        .TickPulse_o (TickPulse_RX  )  // Tick pulse output
      );
      
      prim_FractionalTickGen
      #(
        .FREQ_SYSCLK (FREQ_SYSCLK    ), // System clock frequency (in Hz)
        .MODE_FIXED  (MODE_BAUDCTRL  ), // Generation mode:'1' - fixed tick rate, '0' - variable tick rate
        .TICKR_FIXED (BAUD_UART_DEF  ), // Target tickrate (used in fixed mode)
        .OVERSAMPLE  (OVERSAMPLE_N   ), // Oversampling mode (ticks per bit)
        .LOGIC_DELAY (2              )  // 
      )
      TickGen_TX
      (
        .RST_i       (RST_i         ), // System reset
        .CLK_i       (CLK_i         ), // System clock
        .BaudSel_i   (BaudSel_i     ), // 
        .Resync_i    (TickResync_TX ), // Clear and start counting
        .TickPulse_o (TickPulse_TX  )  // Tick pulse output
      );
    end
    else // Half duplex mode
    begin:halfdupl_gen 
      
      logic TickResync;
      logic TickPulse ;
      
      always_comb
      begin
        TickResync = TickResync_RX | TickResync_TX;
        TickPulse_RX = TickPulse;
        TickPulse_TX = TickPulse;
        Inhibit_RX = TX_Active_o;
        Inhibit_TX = RX_Active_o;
        RE_o = (MODE_RE_POL)? ~TX_Active_o :  TX_Active_o;
        DE_o = (MODE_DE_POL)?  TX_Active_o : ~TX_Active_o;
      end
      
      prim_FractionalTickGen
      #(
        .FREQ_SYSCLK (FREQ_SYSCLK    ), // System clock frequency (in Hz)
        .MODE_FIXED  (MODE_BAUDCTRL  ), // Generation mode:'1' - fixed tick rate, '0' - variable tick rate
        .TICKR_FIXED (BAUD_UART_DEF  ), // Target tickrate (used in fixed mode)
        .OVERSAMPLE  (OVERSAMPLE_N   ), // Oversampling mode (ticks per bit)
        .LOGIC_DELAY (2              )  // 
      )
      TickGen_Comb
      (
        .RST_i       (RST_i         ), // System reset
        .CLK_i       (CLK_i         ), // System clock
        .BaudSel_i   (BaudSel_i     ), // 
        .Resync_i    (TickResync    ), // Clear and start counting
        .TickPulse_o (TickPulse     )  // Tick pulse output
      );
    end
  endgenerate
  
  
  
  //------------------------------------------------------------------------------------------
  //  RX side gen
  //------------------------------------------------------------------------------------------
  
  prim_UART_RX
  #(
    .OVERSAMPL     (OVERSAMPLE_N  ), // Oversampling mode (ticks per bit)
    .MODE_VAR_WORD (MODE_VAR_WORD ), // '0' - Word size == MAX_DW, '1' - Word size <= MAX_DW, controlled by DataBits_i
    .MAX_DW        (MAX_DW        ), // Maximum data bits in a word
    .MODE_MSB      (MODE_MSB      ), // '0' - LSB (default), '1' - MSB
    .MODE_VAR_PAR  (MODE_VAR_PAR  ), // '0' - Parity type == MODE_TYP_PAR, '1' - Parity type is controlled by ParityType_i
    .MODE_TYP_PAR  (MODE_TYP_PAR  ), // Parity type (in fixed mode): '0' - N, '1' - E, '2' - O, '3' - M, '4' - S.
    .MODE_VAR_STP  (MODE_VAR_STP  ), // '0' - Stop bits == MODE_TYP_STP, '1' - Stop bits <= 3, is controlled by StopBits_i
    .MODE_TYP_STP  (MODE_TYP_STP  )  // Maximum stop bits in a word
  )
  RX
  (
    .RST_i         (RST_i         ), // System reset
    .CLK_i         (CLK_i         ), // System clock
    .RX_i          (RX_i          ), // Serial data input
    .TickPulse_i   (TickPulse_RX  ), // Tick pulse input
    .Resync_o      (TickResync_RX ), // 
    .DataBits_i    (DataBits_i    ), // 
    .ParityType_i  (ParityType_i  ), // 
    .StopBits_i    (StopBits_i    ), // 
    .Inhibit_i     (Inhibit_RX    ), // Inhibits work of the main SM
    .Active_o      (RX_Active_o   ), // Main SM is NOT IDLE
    .ParityError_o (ParityError_o ), // 
    .FrameError_o  (FrameError_o  ), // 
    .DataValid_o   (DataValid_o   ), // 
    .Data_o        (Data_o        )  // 
  );
  
  //------------------------------------------------------------------------------------------
  //  TX side gen
  //------------------------------------------------------------------------------------------
  
  prim_UART_TX
  #(
    .OVERSAMPL     (OVERSAMPLE_N  ), // Oversampling mode (ticks per bit)
    .MODE_VAR_WORD (MODE_VAR_WORD ), // '0' - Word size == MAX_DW, '1' - Word size <= MAX_DW, controlled by DataBits_i
    .MAX_DW        (MAX_DW        ), // Maximum data bits in a word
    .MODE_MSB      (MODE_MSB      ), // '0' - LSB (default), '1' - MSB
    .MODE_VAR_PAR  (MODE_VAR_PAR  ), // '0' - Parity type == MODE_TYP_PAR, '1' - Parity type is controlled by ParityType_i
    .MODE_TYP_PAR  (MODE_TYP_PAR  ), // Parity type (in fixed mode): '0' - N, '1' - E, '2' - O, '3' - M, '4' - S.
    .MODE_VAR_STP  (MODE_VAR_STP  ), // '0' - Stop bits == MODE_TYP_STP, '1' - Stop bits <= 3, is controlled by StopBits_i
    .MODE_TYP_STP  (MODE_TYP_STP  )  // Maximum stop bits in a word
  )
  TX
  (
    .RST_i        (RST_i          ), // System reset
    .CLK_i        (CLK_i          ), // System clock
    .TX_o         (TX_o           ), // Serial data output
    .TickPulse_i  (TickPulse_TX   ), // Tick pulse input
    .Resync_o     (TickResync_TX  ), // 
    .DataBits_i   (DataBits_i     ), // 
    .ParityType_i (ParityType_i   ), // 
    .StopBits_i   (StopBits_i     ), // 
    .Inhibit_i    (Inhibit_TX     ), // Inhibits work of the main SM
    .Active_o     (TX_Active_o    ), // Main SM is NOT IDLE
    .BuffFull_o   (BuffFull_o     ), // Message pre-buffer is full
    .DataValid_i  (DataValid_i    ), // 
    .Data_i       (Data_i         )  // 
  );
  
  
endmodule
