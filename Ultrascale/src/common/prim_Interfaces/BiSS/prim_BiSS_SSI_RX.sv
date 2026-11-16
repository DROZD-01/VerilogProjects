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
    parameter  real FREQ_SYSCLK   = 100e6  , // Main system clock frequency, Hz
    parameter  bit  MODE_BAUDCTRL = 1'b0   , // Baudrate control mode: '1' - fixed, '0' - variable
    parameter  int  BAUD_UART_DEF = 115200 , // UART baudrate (for fixed mode)
    parameter  int  MAX_DW        = 27     , // Maximum data payload 
    parameter  int  MAX_CW        = 6      , // Maximum crc payload 
    parameter  int  MAX_MA_DELAY  = 4      , // Timeout value (full MA cycle periods)
    parameter  int  OVERSAMPLE    = 32     , // Ticks per baud
    parameter  int  WIDTH_TICKDEL = 16       // SET_TickDelay_i bit width
  )
  (
    input  logic                        RST_i               , // 
    input  logic                        CLK_i               , // 
    input  logic                        Start_i             , // Initiate exchange
    // Parameters
    input  logic                        SET_InterfaceMode_i , // '1' - SSI, '0' - BiSS
    input  logic [$clog2(MAX_DW+1)-1:0] DataW_i             , // Data payload width
    input  logic                        SET_Lvl_LaunchSL_i  , // MA level AFTER the edge the slave launches on
    input  logic                        SET_EN_AutoLD_i     , // per-frame automatic line-delay tracking
    input  logic [   WIDTH_TICKDEL-1:0] SET_TickDelay_i     , // Manual line delay
    
    // Output
    output logic                        DataValid_o         , // 
    output logic [          MAX_DW-1:0] Data_o              , // 
    output logic                        BiSS_ErrCRC_o       , // (Only in BiSS mode) 
    output logic                        BiSS_LineTO_o       , // (Only in BiSS mode) SL Line delay timeout
    output logic                        BiSS_StopTO_o       , // (Only in BiSS mode) SL stop timeout
    output logic                        BiSS_FrameErr_o     , // (Only in BiSS mode) Unexpected value in a fixed field
    // Input
    output logic                        SS_MA               , // Master -> Slave clock line
    input  logic                        SS_SL                 // Master <- Slave data line
  );
  
  localparam int MA_DELAY_HP = $clog2(MAX_MA_DELAY*2 + 1)        ; // History depth = 2^RAM_LOG2 half-period strobes.
  localparam int TAP_W       = MA_DELAY_HP + $clog2(OVERSAMPLE/2);
  localparam int HP_W        = $clog2(OVERSAMPLE/2);
  localparam int IMM_TICKS   = 3 * (OVERSAMPLE/2);
  
  //--------------------------------------------------------------------
  // Interconnect declarations
  //--------------------------------------------------------------------
  
  
  // SL synchronizer
  (* ASYNC_REG = "TRUE" *) logic [1:0] SL_Resync;
  logic             SL_Sync           ; // 
  
  // Delay pipeline / sampling
  logic             MA_Edge_Del       ; // 
  logic             MA_Level_Del      ; // 
  logic             ImmAppl_Delay     ; // Immediate delay aplication is allowed
  logic [TAP_W-1:0] TickDelay_Active  ; // 
  logic [TAP_W-1:0] TickDelay_Shadow  ; // 
  logic             SL_Sample_Level   ; // 
  logic             SL_Sample         ; // 
  logic             TickHalfP_Empty   ; // 
  
  // internal tick generator + half-period divider
  logic             TickResync        ; // 
  logic             TickPulse         ; // 
  logic [ HP_W-1:0] Tick_Phase        ; // free-running mod-(OVERSAMPLE/2) tick phase
  logic             HalfPulse         ; // 
  
  //--------------------------------------------------------------------
  // Mod-(OVERSAMPLE/2) tick divider: hp_ce = every OVERSAMPLE/2-th tick,
  // the MA half-period boundary (free-running: boundaries while MA is
  // idle carry the idle level and are rejected downstream by polarity).
  //--------------------------------------------------------------------
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i) 
      Tick_Phase <= 'd0;
    else if(TickPulse)
      Tick_Phase <= Tick_Phase + 1'd1;
  end
  assign HalfPulse = TickPulse && (Tick_Phase == {HP_W{1'b1}});
  
  
  //--------------------------------------------------------------------
  // SL re-synchronizer (2 FF)
  //--------------------------------------------------------------------
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i) 
      SL_Resync <= 2'b00;
    else
      SL_Resync <= {SL_Resync[0], SS_SL};
  end
  assign SL_Sync = SL_Resync[1];
  
  
  //--------------------------------------------------------------------
  // ---- tick manager ----
  // Immediate application is safe while D <= 1.5 MA periods = 3*HP ticks
  // (the start-bit strobe is emitted at R2 + 3T/2, still behind the tap
  // update point R2 + D). Larger delays are applied at the next frame
  // start (tracking). Meas_Done_o implies the value is inside the pipe's
  // representable range - no extra range check here.
  //--------------------------------------------------------------------
  assign ImmAppl_Delay = (TickDelay <= TAP_W'(IMM_TICKS));
  
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i) 
    begin
      TickDelay_Active <= 'd0;
      TickDelay_Shadow <= 'd0;
    end
    else
    begin
      if(FR_Load)
        TickDelay_Active <= (SET_EN_AutoLD_i)? TickDelay_Shadow : TAP_W'(SET_TickDelay_i);
      
      if(Meas_Done)
      begin
        TickDelay_Shadow <= TickDelay;
        if(SET_EN_AutoLD_i && ImmAppl_Delay)
          TickDelay_Active <= TickDelay;
      end
    end
  end
  
  
  //--------------------------------------------------------------------
  // Tick-generator selection (external vs. reference)
  //--------------------------------------------------------------------
  assign TickResync = 1'b0;
  
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
  // Delay-compensated sampling strobes
  //--------------------------------------------------------------------
  
  always_comb
  begin
    TickHalfP_Empty = (TickDelay_Active[TAP_W-1:HP_W] == 'd0);
    SL_Sample_Level = (TickHalfP_Empty) ? SET_Lvl_LaunchSL_i : ~SET_Lvl_LaunchSL_i;
    SL_Sample       = MA_Edge_Del && (MA_Level_Del == SL_Sample_Level);
  end
  
  prim_BiSS_SSI_Pipeline
  #(
    .RAM_LOG2   (MA_DELAY_HP), // History depth = 2^RAM_LOG2 half-period strobes.
    .OVERSAMPLE (OVERSAMPLE )  // Oversampling mode (ticks per bit)
  )
  SL_Delay_Pipeline
  (
    .RST_i        (RST_i            ), // System reset
    .CLK_i        (CLK_i            ), // System clock
    .TickPulse_i  (TickPulse        ), // Tick pulse input
    .TickDelay_i  (TickDelay_Active ), // Line delay in tick pulses
    .MA_Edge_i    (                 ), // Strobe at every MA edge
    .MA_Level_i   (~SS_MA           ), // MA level after that edge
    .MA_Edge_o    (MA_Edge_Del      ), // Strobe, delayed by 'TickDelay_i'
    .MA_Level_o   (MA_Level_Del     )  // MA level, same delay
  );
  
  
  //--------------------------------------------------------------------
  // Per-frame round-trip line-delay measurement
  //--------------------------------------------------------------------
  
  logic             Meas_Error  ; // 
  logic             Meas_Done   ; // 
  logic [TAP_W-1:0] TickDelay   ; // Line delay in tick pulses
  
  prim_BiSS_SSI_LineDelay
  #(
    .TIMEOUT_MA (MAX_MA_DELAY ), // Timeout value (full MA cycle periods)
    .RAM_LOG2   (MA_DELAY_HP  ), // History depth = 2^RAM_LOG2 half-period strobes.
    .OVERSAMPLE (OVERSAMPLE   )  // Oversampling mode (ticks per bit)
  )
  SL_Delay_Measure
  (
    .RST_i           (RST_i         ), // System reset
    .CLK_i           (CLK_i         ), // System clock
    .TickPulse_i     (TickPulse     ), // Tick pulse input
    .InterfaceMode_i (              ), // 1'b1 - SSI, 1'b0 - BiSS
    .FrameStart_i    (              ), // frame launched (auto mode)
    .FrameActive_i   (FR_Active     ), // MA clocking in progress
    .MA_RefEdge_i    (MA_RefEdge    ), // Reference edge pulse (BiSS - 2nd rising MA edge; SSI - 1st MA edge)
    .SL_Synched_i    (SL_Sync       ), // 
    .Meas_Error_o    (Meas_Error    ), // 
    .Meas_Done_o     (Meas_Done     ), // 
    .TickDelay_o     (TickDelay     )  // Line delay in tick pulses
  );
  
  
  //--------------------------------------------------------------------
  // TX frame controller / MA generator
  //--------------------------------------------------------------------
  
  logic FR_Load    ; // 
  logic FR_Launch  ; // 
  logic FR_Abort   ; // 
  logic FR_End     ; // 
  logic FR_Active  ; // 
  logic MA_RefEdge ; // 
  
  prim_BiSS_SSI_MAGen
  #(
    //parameter  int MAX_FRAME_BITS = 75 , // 
    .WIDTH_FRAMEBIT (  8 ), // Frame length [bits]
    .WIDTH_INTFRGAP ( 16 ), // inter-frame gap [ovs ticks]
    .WIDTH_LIMHALFM ( 16 ), // extra MA cycles before TX abort
    .WIDTH_LIMTTIMO ( 24 ), // frame watchdog [ovs ticks], 0 = infinite
    .WIDTH_EXTRTCDM ( 12 )  // CDM '1' start extension [ovs ticks]
  )
  MA_Generator
  (
    .RST_i               (RST_i                ), // 
    .CLK_i               (CLK_i                ), // 
    // Controls
    .SET_InterfaceMode_i (                     ), // '1' - SSI, '0' - BiSS
    .SET_FrameBits_i     (                     ), // Frame length [bits]
    .SET_InterFrameGap_i (                     ), // inter-frame gap [ovs ticks]
    .SET_Limit_HalfMA_i  (                     ), // extra MA cycles before TX abort
    .SET_Limit_TickTO_i  (                     ), // frame watchdog [ovs ticks], 0 = infinite
    .SET_Extra_TickCDM_i (                     ), // CDM '1' start extension [ovs ticks]
    .SET_Lvl_MA_Idle     (                     ), // MA level while idle
    .SET_FSM_Enable      (                     ), // global enable
    // Clocking
    .TickPulse_i         (TickPulse            ), // oversampling tick strobe
    .HalfPulse_i         (HalfPulse            ), // half-period boundary strobe
    // Frame Requests
    .REQ_Data            (                     ), // process-data frame request
    .REQ_CDM             (                     ), // CDM frame request (has priority)
    .CDM_Bit             (                     ), // CDM bit to encode into this frame
    // Feedback
    .RX_FrameDone_i      (                     ), // RX completed the frame (delayed!)
    // Events
    .FR_Load_o           (FR_Load              ), // 
    .FR_Launch_o         (FR_Launch            ), // 
    .FR_Abort_o          (FR_Abort             ), // 
    .FR_End_o            (FR_End               ), // 
    .MA_RefEdge_o        (MA_RefEdge           ), // Reference edge pulse (BiSS - 2nd rising MA edge; SSI - 1st MA edge)
    .WatchdogExpired_o   (                     ), // 
    // Status & I/O
    .FR_Active_o         (FR_Active            ), // 
    .FR_IsCDM_o          (                     ), // 
    .MA_o                (SS_MA                )  // 
  );
  
  
  //--------------------------------------------------------------------
  // RX deserializer: start-bit search, bit counting, SSI monoflop marker
  // handling, BiSS-C CRC check (prim_CRC_Calculator inside) and result
  // extraction. 
  //--------------------------------------------------------------------
  
  prim_BiSS_SSI_DeserRX
  #(
    .MAX_PDUBIT (32  ), // Widest process-data payload [bits]
    .MAX_CDSBIT (4   ), // Widest CDS field [bits]
    .VAL_CRCBIT (6   )  // BiSS-C CRC width [bits]
  )
  RX_Deserializer
  (
    .RST_i               (RST_i             ), // 
    .CLK_i               (CLK_i             ), // 
    // Controls
    .SET_InterfaceMode_i (                  ), // '1' - SSI, '0' - BiSS
    .SET_EN_AutoLD_i     (                  ), // per-frame automatic line-delay tracking
    .SET_EN_BiSS_CRC     (                  ), // 
    .SET_CRC_InclCDS     (                  ), // 
    .SET_SSI_LenMode     (                  ), // 
    .SET_CDSBits_i       (                  ), // 
    .SET_PDUBits_i       (                  ), // 
    .SET_FrameBits_i     (                  ), // 
    // Events
    .FR_Launch_i         (FR_Launch         ), // frame start pulse (tick-aligned)
    .FR_Active_i         (FR_Active         ), // TX-side frame in progress
    .Meas_Error_i        (                  ), // line-delay measurement finished
    .Meas_Done_i         (                  ), // line-delay measurement failed
    // Input
    .SL_Sample_i         (SL_Sample         ), // one strobe per SL bit, at bit center
    .SL_Synched_i        (SL_Sync           ), // SL line value at the strobe
    // Output
    .RX_FrameDone_o      (                  ), // RX completed the frame
    .Recv_Valid_o        (                  ), // 
    .Recv_PDULength_o    (                  ), // 
    .Recv_PDUBits_o      (                  ), // 
    .Recv_CDSBits_o      (                  ), // 
    .Recv_SSIPadding_o   (                  ), // 
    // Status
    .Err_NoStart_o       (                  ), // 
    .Err_NoMarker_o      (                  ), // 
    .Err_BadCRC_o        (                  )  // 
  );
  
  
  
  
  
endmodule