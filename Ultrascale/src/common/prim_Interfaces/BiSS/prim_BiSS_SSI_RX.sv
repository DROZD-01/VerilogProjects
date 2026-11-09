`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/04/2026 12:21:31 AM
// Design Name: 
// Module Name: prim_BiSS_SSI_RX
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

module prim_BISS_SSI_RX
  #(
    parameter real FREQ_SYSCLK   = 100e6  , // Main system clock frequency, Hz
    parameter bit  MODE_BAUDCTRL = 1'b0   , // Baudrate control mode: '1' - fixed, '0' - variable
    parameter int  BAUD_UART_DEF = 115200 , // UART baudrate (for fixed mode)
    parameter int  MAX_DW        = 27     , // Maximum data payload 
    parameter int  MAX_CW        = 6      , // Maximum crc payload 
    parameter int  MAX_MA_DELAY  = 4        // Timeout value (full MA cycle periods)
  )
  (
    input  logic                        RST_i           , // 
    input  logic                        CLK_i           , // 
    input  logic                        Start_i         , // Initiate exchange
    // Parameters
    input  logic                        InterfaceMode_i , // '1' - SSI, '0' - BiSS
    input  logic [$clog2(MAX_DW+1)-1:0] DataW_i         , // Data payload width
    // Output
    output logic                        DataValid_o     , // 
    output logic [          MAX_DW-1:0] Data_o          , // 
    output logic                        BiSS_ErrCRC_o   , // (Only in BiSS mode) 
    output logic                        BiSS_LineTO_o   , // (Only in BiSS mode) SL Line delay timeout
    output logic                        BiSS_StopTO_o   , // (Only in BiSS mode) SL stop timeout
    output logic                        BiSS_FrameErr_o , // (Only in BiSS mode) Unexpected value in a fixed field
    // Input
    output logic                        SS_MA           , // Master -> Slave clock line
    input  logic                        SS_SL             // Master <- Slave data line
  );
  
  localparam int OVERSAMPLE  = 32                                ; // Ticks per baud
  localparam int MA_DELAY_HP = $clog2(MAX_MA_DELAY*2 + 1)        ; // History depth = 2^RAM_LOG2 half-period strobes.
  localparam int TAP_W       = MA_DELAY_HP + $clog2(OVERSAMPLE/2);
  
  //--------------------------------------------------------------------
  // SL re-synchronizer (2 FF)
  //--------------------------------------------------------------------
  (* ASYNC_REG = "TRUE" *) logic [1:0] SL_Resync;
  logic SL_Sync;
  
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i) 
      SL_Resync <= 2'b00;
    else        
      SL_Resync <= {SL_Resync[0], SS_SL};
  end
  assign SL_Sync = SL_Resync[1];
  
  
  //--------------------------------------------------------------------
  // Tick-generator selection (external vs. reference)
  //--------------------------------------------------------------------
  
  logic TickResync ; // 
  logic TickPulse  ; // 
  
  prim_FractionalTickGen
  #(
    .FREQ_SYSCLK (FREQ_SYSCLK    ), // System clock frequency (in Hz)
    .MODE_FIXED  (MODE_BAUDCTRL  ), // Generation mode:'1' - fixed tick rate, '0' - variable tick rate
    .TICKR_FIXED (BAUD_UART_DEF  ), // Target tickrate (used in fixed mode)
    .OVERSAMPLE  (OVERSAMPLE     )  // Oversampling mode (ticks per bit)
  )
  TickGen
  (
    .RST_i       (RST_i         ), // System reset
    .CLK_i       (CLK_i         ), // System clock
    .BaudSel_i   (              ), // 
    .Resync_i    (TickResync    ), // Clear and start counting
    .TickPulse_o (TickPulse     )  // Tick pulse output
  );
  
  
  
  //--------------------------------------------------------------------
  // Per-frame round-trip line-delay measurement
  //--------------------------------------------------------------------
  
  logic [TAP_W-1:0] TickDelay ; // Line delay in tick pulses
  
  prim_BiSS_SSI_LineDelay
  #(
    .TIMEOUT_MA (MAX_MA_DELAY ), // Timeout value (full MA cycle periods)
    .RAM_LOG2   (MA_DELAY_HP  ), // History depth = 2^RAM_LOG2 half-period strobes.
    .OVERSAMPLE (OVERSAMPLE   )  // Oversampling mode (ticks per bit)
  )
  SL_Delay_Measure
  (
    .RST_i           (RST_i     ), // System reset
    .CLK_i           (CLK_i     ), // System clock
    .TickPulse_i     (TickPulse ), // Tick pulse input
    .InterfaceMode_i (          ), // 1'b1 - SSI, 1'b0 - BiSS
    .FrameStart_i    (          ), // frame launched (auto mode)
    .FrameActive_i   (          ), // MA clocking in progress
    .MA_RefEdge_i    (          ), // Reference edge pulse (BiSS - 2nd rising MA edge; SSI - 1st MA edge)
    .SL_Synched_i    (SL_Sync   ), // 
    .Meas_Error_o    (          ), // 
    .Meas_Done_o     (          ), // 
    .TickDelay_o     (TickDelay )  // Line delay in tick pulses
  );
  
  
  
  //--------------------------------------------------------------------
  // Delay-compensated sampling strobes
  //--------------------------------------------------------------------
  
  prim_BiSS_SSI_Pipeline
  #(
    .RAM_LOG2   (MA_DELAY_HP), // History depth = 2^RAM_LOG2 half-period strobes.
    .OVERSAMPLE (OVERSAMPLE )  // Oversampling mode (ticks per bit)
  )
  SL_Delay_Pipeline
  (
    .RST_i        (RST_i      ), // System reset
    .CLK_i        (CLK_i      ), // System clock
    .TickPulse_i  (TickPulse  ), // Tick pulse input
    .TickDelay_i  (TickDelay  ), // Line delay in tick pulses
    .MA_Edge_i    (           ), // Strobe at every MA edge
    .MA_Level_i   (           ), // MA level after that edge
    .MA_Edge_o    (           ), // Strobe, delayed by 'TickDelay_i'
    .MA_Level_o   (           )  // MA level, same delay
  );
  
  
  
  
endmodule