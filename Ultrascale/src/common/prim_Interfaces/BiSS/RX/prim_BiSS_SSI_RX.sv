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
    parameter  real FREQ_SYSCLK     = 100e6  , // Main system clock frequency, Hz
    parameter  bit  MODE_BAUDCTRL   = 1'b0   , // Baudrate control mode: '1' - fixed baudrate, '0' - variable baudrate
    parameter  int  BAUD_FIXED_DEF  = 500000 , // UART baudrate (for fixed mode)
    parameter  int  MAX_PDUBIT      = 32     , // Widest process-data payload [bits]
    parameter  int  VAL_CRCBIT      =  6     , // BiSS-C CRC width [bits]
    parameter  int  WIDTH_CDADDR    =  7     , // Control Communication address payload bit width [bits]
    parameter  int  WIDTH_CDDATA    =  8     , // Control Communication data    payload bit width [bits]
    parameter  int  WIDTH_CDSLID    =  8     , // Control Communication ID      payload bit width [bits]
    parameter  int  MAX_MA_DELAY    =  4     , // Timeout value (full MA cycle periods)
    parameter  bit  EN_ERR_LAT      = 1'b1   , // Enable error latching (reset with "Clear_Status_i")
    parameter  bit  EN_STS_CNT      = 1'b1   , // Enable frame and error counters
    parameter  int  WIDTH_CNT_FR    =  4     , // Error event counter width [bits]
    parameter  int  WIDTH_CNT_ER    =  4     , // Frame counter width [bits]
    parameter  int  OVERSAMPLE      = 32     , // Oversampling ticks per baud
    parameter  int  WIDTH_TICKDEL   =  8     , // Manual SL line delay bit width [bits]
    parameter  int  WIDTH_INTFRGAP  = 16     , // inter-frame gap [ovs ticks]
    parameter  int  WIDTH_LIMHALFM  = 16     , // extra half-MA cycles before TX abort
    parameter  int  WIDTH_LIMTTIMO  = 16     , // frame watchdog [ovs ticks], 0 = infinite
    parameter  int  WIDTH_EXTRTCDM  = 12     , // CDM '1' start extension [ovs ticks]
    parameter  int  BV_NUM          = 4      , // Number of entries in BAUD_VAL
    parameter  int  BAUD_VAL [0:BV_NUM-1] = '{500000, 1000000, 1500000, 2000000}, // Baudrates, selectable by "SET_Baudrate_i"
    
    localparam int  MAX_FRAMEBIT    = 1 + 1 + MAX_PDUBIT + VAL_CRCBIT, // 
    localparam int  WIDTH_PDUBIT    = $clog2(MAX_PDUBIT   + 1) , // 
    localparam int  WIDTH_FRAMEBIT  = $clog2(MAX_FRAMEBIT + 1) , // 
    localparam int  BV_WIDTH        = $clog2(BV_NUM)
  )
  (
    input  logic                      RST_i                , // 
    input  logic                      CLK_i                , // 
    input  logic                      Start_i              , // Initiate a single exchange
    // Parameters
    input  logic                      SET_FSM_Enable_i     , // 0 -> FSMs idle, MA pinned to idle level
    input  logic                      SET_InterfaceMode_i  , // '1' - SSI, '0' - BiSS
    input  logic                      SET_FSM_Continuous_i , // repeat process frames forever
    input  logic                      SET_EN_BiSS_CRC_i    , // BiSS: append/check CRC-6
    input  logic                      SET_CRC_InclCDS_i    , // 1: CRC covers CDS bits as well as PDU
    input  logic                      SET_EN_AutoLD_i      , // per-frame automatic line-delay tracking
    input  logic                      SET_SSI_LenMode_i    , // 0 = length from pdu_bits, 1 = discover from the monoflop marker (zero-padded MSBs!)
    input  logic                      SET_Lvl_LaunchSL_i   , // MA level AFTER the edge the slave launches on
    input  logic                      SET_Lvl_MA_Idle_i    , // MA level while idle
    input  logic [  WIDTH_PDUBIT-1:0] SET_PDUBits_i        , // PDU width (clamped to MAX_PDU_BITS)
    input  logic [      BV_WIDTH-1:0] SET_Baudrate_i       , // MA speed gear -> MA_BAUD_VAL table below; snapshotted, so speed changes land at frame boundaries
    input  logic [WIDTH_INTFRGAP-1:0] SET_InterFrameGap_i  , // inter-frame gap [ovs ticks]
    input  logic [WIDTH_LIMHALFM-1:0] SET_Limit_HalfMA_i   , // extra MA cycles before TX abort (0 = infinite)
    input  logic [WIDTH_LIMTTIMO-1:0] SET_Limit_TickTO_i   , // frame watchdog [ovs ticks], 0 = infinite
    input  logic [ WIDTH_TICKDEL-1:0] SET_TickDelay_i      , // Manual SL line delay
    input  logic [WIDTH_EXTRTCDM-1:0] SET_Extra_TickCDM_i  , // CDM '1' start extension [ovs ticks]
    // Status
    input  logic                      Clear_Status_i       , // Clears frame counter and error flags/counter
    output logic [  WIDTH_CNT_FR-1:0] Count_Frame_o        , // Frame received counter
    output logic [  WIDTH_CNT_ER-1:0] Count_Error_o        , // Error event counter
    output logic                      ERR_NoStart_o        , // Error: Frame start not detected
    output logic                      ERR_NoMarker_o       , // Error: SSI monoflop marker not detected
    output logic                      ERR_BadCRC_o         , // Error: BiSS-C CRC mismatch
    output logic                      ERR_WdExp_o          , // Error: Watchdog timeout
    output logic                      ERR_Measure_o        , // Error: Line delay measurement failed
    output logic                      ERR_CDM_o            , // Error: CDM access rejected, timed out or aborted
    // Output, Data channel
    output logic                      Recv_Valid_o         , // Process Data: data valid
    output logic [    MAX_PDUBIT-1:0] Recv_PDUBits_o       , // Process Data: data
    output logic [  WIDTH_PDUBIT-1:0] Recv_PDULength_o     , // Process Data: data bits received
    output logic [  WIDTH_PDUBIT-1:0] Recv_SSIPadding_o    , // Process Data: SSI-AUTO leading pad zeros
    // Input/Output, CD channel
    input  logic                      COMM_Request_i       , // Control Communication: request
    input  logic [               1:0] COMM_Type_i          , // Control Communication: command type, '00' - single-byte read, '01' - single-byte write, '10' - command, '11' - broadcast command
    input  logic [               1:0] COMM_CMD_i           , // Control Communication: command code (bit 1 = CMD1, first on the wire)
    input  logic [  WIDTH_CDSLID-1:0] COMM_IDSel_i         , // Control Communication: linear-select slave-ID mask (bit 7 = IDS0, first on the wire)
    input  logic [  WIDTH_CDADDR-1:0] COMM_Addr_i          , // Control Communication: address in
    input  logic [  WIDTH_CDDATA-1:0] COMM_Data_i          , // Control Communication: data    in
    output logic                      COMM_Reply_o         , // Control Communication: reply
    output logic [  WIDTH_CDDATA-1:0] COMM_Data_o          , // Control Communication: data out
    // Input
    output logic                      SS_MA                , // Master -> Slave clock line
    input  logic                      SS_SL                  // Master <- Slave data line
  );
  
  localparam int MA_DELAY_HP = $clog2(MAX_MA_DELAY*2 + 1)        ; // History depth = 2^RAM_LOG2 half-period strobes.
  localparam int TAP_W       = MA_DELAY_HP + $clog2(OVERSAMPLE/2);
  localparam int HP_W        = $clog2(OVERSAMPLE/2);
  localparam int IMM_TICKS   = 3 * (OVERSAMPLE/2);
  
  //--------------------------------------------------------------------
  // Interconnect declarations
  //--------------------------------------------------------------------
  
  // Settings buffer
  logic                      SetR_FSM_Enable     ; // 
  logic                      SetR_InterfaceMode  ; // 
  logic                      SetR_FSM_Continuous ; // 
  logic                      SetR_EN_BiSS_CRC    ; // 
  logic                      SetR_CRC_InclCDS    ; // 
  logic                      SetR_EN_AutoLD      ; // 
  logic                      SetR_SSI_LenMode    ; // 
  logic                      SetR_Lvl_LaunchSL   ; // 
  logic                      SetR_Lvl_MA_Idle    ; // 
  logic [  WIDTH_PDUBIT-1:0] SetR_PDUBits        ; // 
  logic [      BV_WIDTH-1:0] SetR_Baudrate       ; // 
  logic [WIDTH_INTFRGAP-1:0] SetR_InterFrameGap  ; // 
  logic [WIDTH_LIMHALFM-1:0] SetR_Limit_HalfMA   ; // 
  logic [WIDTH_LIMTTIMO-1:0] SetR_Limit_TickTO   ; // 
  logic [WIDTH_EXTRTCDM-1:0] SetR_Extra_TickCDM  ; // 
  
  
  // SL synchronizer
  (* ASYNC_REG = "TRUE" *) logic [1:0] SL_Resync;
  logic             SL_Sync           ; // 
  
  // Line-delay measurement
  logic                      Meas_Error       ; // 
  logic                      Meas_Done        ; // 
  logic                      Meas_Valid       ; // 
  logic [         TAP_W-1:0] TickDelay        ; // Line delay in tick pulses
  
  // Delay pipeline / sampling
  logic                      MA_Edge_Del      ; // 
  logic                      MA_Level_Del     ; // 
  logic [         TAP_W-1:0] TickDelay_Active ; // 
  logic [         TAP_W-1:0] TickDelay_Shadow ; // 
  logic                      SL_Sample_Level  ; // 
  (* syn_maxfan = 16 *) logic                      SL_Sample        ; // 
  logic                      TickHalfP_Empty  ; // 
  
  // internal tick generator + half-period divider
  logic                      TickResync       ; // 
  logic                      TickPulse        ; // 
  logic [          HP_W-1:0] Tick_Phase       ; // free-running mod-(OVERSAMPLE/2) tick phase
  logic                      HalfPulse        ; // 
  
  // RX deserializer / CRC
  logic                      RX_FrameDone     ; // 
  logic                      Recv_CDSBits     ; // 
  logic                      Err_NoStart      ; // 
  logic                      Err_NoMarker     ; // 
  logic                      Err_BadCRC       ; // 
  
  // MA_Generator interconnect
  logic                      REQ_Data         ; // 
  logic                      FR_Load          ; // 
  logic                      FR_Launch        ; // 
  logic                      FR_Abort         ; // 
  logic                      FR_End           ; // 
  logic                      FR_Active        ; // 
  logic                      FR_Start         ; // 
  logic                      FR_IsCDM         ; // 
  logic                      MA_RefEdge       ; // 
  logic                      Err_WdExp        ; // Frame watchdog expired
  logic                      StartPending     ; // 
  logic                      REQ_CDM          ; // 
  logic                      CDM_Bit          ; // 
  
  
  // CDM sequencer interconnect
  logic                      CDM_Busy         ; // 
  logic                      Err_CDM          ; // 
  
  // Frame geometry
  logic [WIDTH_FRAMEBIT-1:0] Set_FrameBits    ; // 
  logic [WIDTH_FRAMEBIT-1:0] SetR_FrameBits   ; // 
  
  
  //--------------------------------------------------------------------
  // Configuration: clamp (live) + snapshot at frame acceptance.
  //--------------------------------------------------------------------
  
  always_comb
  begin
    case({SET_InterfaceMode_i, SET_EN_BiSS_CRC_i})
      2'b00: Set_FrameBits = WIDTH_FRAMEBIT'(SET_PDUBits_i              + 1'd1 + 1'd1); // BiSS, CRC disabled
      2'b01: Set_FrameBits = WIDTH_FRAMEBIT'(SET_PDUBits_i + VAL_CRCBIT + 1'd1 + 1'd1); // BiSS, CRC enabled
      2'b10: Set_FrameBits = WIDTH_FRAMEBIT'(SetR_PDUBits                            ); // SSI
      2'b11: Set_FrameBits = WIDTH_FRAMEBIT'(SetR_PDUBits                            ); // SSI
    endcase
  end
  
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i) 
    begin
      SetR_FSM_Enable     <= 1'd0;
      SetR_InterfaceMode  <= 1'd0;
      SetR_FSM_Continuous <= 1'd0;
      SetR_EN_BiSS_CRC    <= 1'd0;
      SetR_CRC_InclCDS    <= 1'd0;
      SetR_EN_AutoLD      <= 1'd0;
      SetR_SSI_LenMode    <= 1'd0;
      SetR_Lvl_LaunchSL   <= 1'd0;
      SetR_Lvl_MA_Idle    <= 1'd0;
      SetR_PDUBits        <=  'd0;
      SetR_FrameBits      <=  'd0;
      SetR_Baudrate       <=  'd0;
      SetR_InterFrameGap  <=  'd0;
      SetR_Limit_HalfMA   <=  'd0;
      SetR_Limit_TickTO   <=  'd0;
      SetR_Extra_TickCDM  <=  'd0;
    end
    else if(FR_Load || Start_i)
    begin
      SetR_FSM_Enable     <= SET_FSM_Enable_i     ; // 
      SetR_InterfaceMode  <= SET_InterfaceMode_i  ; // 
      SetR_FSM_Continuous <= SET_FSM_Continuous_i ; // 
      SetR_EN_BiSS_CRC    <= SET_EN_BiSS_CRC_i    ; // 
      SetR_CRC_InclCDS    <= SET_CRC_InclCDS_i    ; // 
      SetR_EN_AutoLD      <= SET_EN_AutoLD_i      ; // 
      SetR_SSI_LenMode    <= SET_SSI_LenMode_i    ; // 
      SetR_Lvl_LaunchSL   <= SET_Lvl_LaunchSL_i   ; // 
      SetR_Lvl_MA_Idle    <= SET_Lvl_MA_Idle_i    ; // 
      SetR_PDUBits        <= SET_PDUBits_i        ; // 
      SetR_FrameBits      <= Set_FrameBits        ; // 
      SetR_Baudrate       <= SET_Baudrate_i       ; // 
      SetR_InterFrameGap  <= SET_InterFrameGap_i  ; // 
      SetR_Limit_HalfMA   <= SET_Limit_HalfMA_i   ; // 
      SetR_Limit_TickTO   <= SET_Limit_TickTO_i   ; // 
      SetR_Extra_TickCDM  <= SET_Extra_TickCDM_i  ; // 
    end
  end
  
  
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
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i) 
    begin
      TickDelay_Active <=  'd0;
      TickDelay_Shadow <=  'd0;
      Meas_Valid       <= 1'b0;
    end
    else
    begin
      if(FR_Load)
      begin
        // FIX R1: a frame with AutoLD disabled must invalidate the tracked
        // shadow.  No measurement runs while AutoLD is off (LineDelay arms
        // only on FR_Launch && SET_EN_AutoLD), so a shadow that survives a
        // manual-LD period is arbitrarily stale - it was measured under a
        // different delay regime (e.g. the end of a tracking sweep) and can
        // sit whole bits away from the current line delay.  Applying it to
        // the first AutoLD frame after re-enabling corrupts that frame and
        // raises spurious error flags; the manual SET_TickDelay_i seed is
        // the documented fallback "until the first AutoLD measurement
        // completes".
        case({SET_EN_AutoLD_i, Meas_Valid})
          2'b11   : 
          begin
            Meas_Valid       <= 1'b0;
            TickDelay_Active <= TickDelay_Shadow;
          end
          default : 
          begin
            TickDelay_Active <= TAP_W'(SET_TickDelay_i);
            if(!SET_EN_AutoLD_i)
              Meas_Valid     <= 1'b0;
          end
        endcase
      end
      
      if(Meas_Done)
      begin
        Meas_Valid       <= 1'b1;
        TickDelay_Shadow <= TickDelay;
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
    .TICKR_FIXED (BAUD_FIXED_DEF ), // Target tickrate (used in fixed mode)
    .BV_NUM      (BV_NUM         ), // Number of entries in BAUD_VALUES
    .BAUD_VAL    (BAUD_VAL       ), // 
    .OVERSAMPLE  (OVERSAMPLE     )  // Oversampling mode (ticks per bit)
  )
  TickGen
  (
    .RST_i       (RST_i         ), // System reset
    .CLK_i       (CLK_i         ), // System clock
    .BaudSel_i   (SetR_Baudrate ), // 
    .Resync_i    (TickResync    ), // Clear and start counting
    .TickPulse_o (TickPulse     )  // Tick pulse output
  );
  
  
  //--------------------------------------------------------------------
  // Delay-compensated sampling strobes
  //--------------------------------------------------------------------
  
  always_comb
  begin
    TickHalfP_Empty = (TickDelay_Active[TAP_W-1:HP_W] == 'd0);
    SL_Sample_Level = (TickHalfP_Empty) ? SetR_Lvl_LaunchSL : ~SetR_Lvl_LaunchSL;
    SL_Sample       = MA_Edge_Del && FR_Active && (MA_Level_Del == SL_Sample_Level);
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
    .MA_Edge_i    (HalfPulse        ), // Strobe at every MA edge
    .MA_Level_i   (~SS_MA           ), // MA level after that edge
    .MA_Edge_o    (MA_Edge_Del      ), // Strobe, delayed by 'TickDelay_i'
    .MA_Level_o   (MA_Level_Del     )  // MA level, same delay
  );
  
  
  //--------------------------------------------------------------------
  // Per-frame round-trip line-delay measurement
  //--------------------------------------------------------------------
  
  prim_BiSS_SSI_LineDelay
  #(
    .TIMEOUT_MA (MAX_MA_DELAY ), // Timeout value (full MA cycle periods)
    .RAM_LOG2   (MA_DELAY_HP  ), // History depth = 2^RAM_LOG2 half-period strobes.
    .OVERSAMPLE (OVERSAMPLE   )  // Oversampling mode (ticks per bit)
  )
  SL_Delay_Measure
  (
    .RST_i               (RST_i             ), // System reset
    .CLK_i               (CLK_i             ), // System clock
    .TickPulse_i         (TickPulse         ), // Tick pulse input
    .SET_InterfaceMode_i (SetR_InterfaceMode), // 1'b1 - SSI, 1'b0 - BiSS
    .SET_EN_AutoLD_i     (SetR_EN_AutoLD    ), // per-frame automatic line-delay tracking
    .FR_Launch_i         (FR_Launch         ), // frame launched (auto mode)
    .FR_Active_i         (FR_Active         ), // MA clocking in progress
    .MA_RefEdge_i        (MA_RefEdge        ), // Reference edge pulse (BiSS - 2nd rising MA edge; SSI - 1st MA edge)
    .SL_Synched_i        (SL_Sync           ), // 
    .Meas_Error_o        (Meas_Error        ), // 
    .Meas_Done_o         (Meas_Done         ), // 
    .TickDelay_o         (TickDelay         )  // Line delay in tick pulses
  );
  
  
  //--------------------------------------------------------------------
  // TX frame controller / MA generator
  //--------------------------------------------------------------------
  
  prim_BiSS_SSI_MAGen
  #(
    .WIDTH_FRAMEBIT (WIDTH_FRAMEBIT ), // Frame length [bits]
    .WIDTH_INTFRGAP (WIDTH_INTFRGAP ), // inter-frame gap [ovs ticks]
    .WIDTH_LIMHALFM (WIDTH_LIMHALFM ), // extra MA cycles before TX abort
    .WIDTH_LIMTTIMO (WIDTH_LIMTTIMO ), // frame watchdog [ovs ticks], 0 = infinite
    .WIDTH_EXTRTCDM (WIDTH_EXTRTCDM )  // CDM '1' start extension [ovs ticks]
  )
  MA_Generator
  (
    .RST_i               (RST_i                ), // 
    .CLK_i               (CLK_i                ), // 
    // Controls
    .SET_InterfaceMode_i (SetR_InterfaceMode   ), // '1' - SSI, '0' - BiSS
    .SET_FrameBits_i     (SetR_FrameBits       ), // Frame length [bits]
    .SET_InterFrameGap_i (SetR_InterFrameGap   ), // inter-frame gap [ovs ticks]
    .SET_Limit_HalfMA_i  (SetR_Limit_HalfMA    ), // extra MA cycles before TX abort
    .SET_Limit_TickTO_i  (SetR_Limit_TickTO    ), // frame watchdog [ovs ticks], 0 = infinite
    .SET_Extra_TickCDM_i (SetR_Extra_TickCDM   ), // CDM '1' start extension [ovs ticks]
    .SET_Lvl_MA_Idle_i   (SetR_Lvl_MA_Idle     ), // MA level while idle
    .SET_FSM_Enable_i    (SetR_FSM_Enable      ), // global enable
    // Clocking
    .TickPulse_i         (TickPulse            ), // oversampling tick strobe
    .HalfPulse_i         (HalfPulse            ), // half-period boundary strobe
    // Frame Requests
    .REQ_Data_i          (REQ_Data             ), // process-data frame request
    .REQ_CDM_i           (REQ_CDM              ), // CDM frame request (has priority)
    .CDM_Bit_i           (CDM_Bit              ), // CDM bit to encode into this frame
    // Feedback
    .RX_FrameDone_i      (RX_FrameDone         ), // RX completed the frame (delayed!)
    // Events
    .FR_Load_o           (FR_Load              ), // 
    .FR_Launch_o         (FR_Launch            ), // 
    .FR_Abort_o          (FR_Abort             ), // 
    .FR_End_o            (FR_End               ), // 
    .MA_RefEdge_o        (MA_RefEdge           ), // Reference edge pulse (BiSS - 2nd rising MA edge; SSI - 1st MA edge)
    .WatchdogExpired_o   (Err_WdExp            ), // 
    // Status & I/O
    .FR_Active_o         (FR_Active            ), // 
    .FR_IsCDM_o          (FR_IsCDM             ), // 
    .MA_o                (SS_MA                )  // 
  );
  
  // one-shot bookkeeping
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i || (FR_Load & !FR_IsCDM)) 
      StartPending <= 1'b0;
    else if(Start_i)
      StartPending <= 1'b1;
  end
  
  assign REQ_Data = SetR_FSM_Enable && (SetR_FSM_Continuous || StartPending) && !CDM_Busy;
  
  //--------------------------------------------------------------------
  // RX deserializer: start-bit search, bit counting, SSI monoflop marker
  // handling, BiSS-C CRC check (prim_CRC_Calculator inside) and result
  // extraction. 
  //--------------------------------------------------------------------
  
  prim_BiSS_SSI_DeserRX
  #(
    .MAX_PDUBIT (MAX_PDUBIT), // Widest process-data payload [bits]
    .VAL_CRCBIT (VAL_CRCBIT)  // BiSS-C CRC width [bits]
  )
  RX_Deserializer
  (
    .RST_i               (RST_i             ), // 
    .CLK_i               (CLK_i             ), // 
    // Controls
    .SET_InterfaceMode_i (SetR_InterfaceMode), // '1' - SSI, '0' - BiSS
    .SET_EN_AutoLD_i     (SetR_EN_AutoLD    ), // per-frame automatic line-delay tracking
    .SET_EN_BiSS_CRC_i   (SetR_EN_BiSS_CRC  ), // 
    .SET_CRC_InclCDS_i   (SetR_CRC_InclCDS  ), // 
    .SET_SSI_LenMode_i   (SetR_SSI_LenMode  ), // 
    .SET_PDUBits_i       (SetR_PDUBits      ), // 
    .SET_FrameBits_i     (SetR_FrameBits    ), // 
    // Events
    .FR_Launch_i         (FR_Launch         ), // frame start pulse (tick-aligned)
    .FR_Active_i         (FR_Active         ), // TX-side frame in progress
    .Meas_Error_i        (Meas_Error        ), // line-delay measurement finished
    .Meas_Done_i         (Meas_Done         ), // line-delay measurement failed
    // Input
    .SL_Sample_i         (SL_Sample         ), // one strobe per SL bit, at bit center
    .SL_Synched_i        (SL_Sync           ), // SL line value at the strobe
    // Output
    .RX_FrameDone_o      (RX_FrameDone      ), // RX completed the frame
    .Recv_Valid_o        (Recv_Valid_o      ), // 
    .Recv_PDUBits_o      (Recv_PDUBits_o    ), // 
    .Recv_PDULength_o    (Recv_PDULength_o  ), // 
    .Recv_SSIPadding_o   (Recv_SSIPadding_o ), // 
    .Recv_CDSBits_o      (Recv_CDSBits      ), // 
    // Status
    .Err_NoStart_o       (Err_NoStart       ), // 
    .Err_NoMarker_o      (Err_NoMarker      ), // 
    .Err_BadCRC_o        (Err_BadCRC        )  // 
  );
  
  
  //--------------------------------------------------------------------
  // CDM register-access sequencer
  //--------------------------------------------------------------------
  
  prim_BiSS_SSI_CDMCtrl_GEN
  #(
    .WIDTH_CDADDR   (WIDTH_CDADDR ), // 
    .WIDTH_CDDATA   (WIDTH_CDDATA )  // register data payload bits
  )
  CDM_Sequencer
  (
    .RST_i               (RST_i             ), // 
    .CLK_i               (CLK_i             ), // 
    .SET_FSM_Enable_i    (SetR_FSM_Enable   ), // global enable
    // Register access request (pulse; sampled when idle)
    .COMM_Request_i      (COMM_Request_i    ), // 
    .COMM_Type_i         (COMM_Type_i       ), // 
    .COMM_Cmd_i          (COMM_CMD_i        ), // COMM_CMD_i
    .COMM_IDSel_i        (COMM_IDSel_i      ), // 
    .COMM_Addr_i         (COMM_Addr_i       ), // 
    .COMM_Data_i         (COMM_Data_i       ), // 
    // Frame events from the frame controller 
    .FR_Load_i           (FR_Load           ), // pulse: a frame was accepted
    .FR_IsCDM_i          (FR_IsCDM          ), // level: the accepted frame is ours
    .FR_Abort_i          (FR_Abort          ), // pulse: frame aborted
    .FR_End_i            (FR_End            ), // pulse: frame finished (or aborted)
    // RX result of the finished frame (latched by the RX before frame_end)
    .Recv_CDSBits_i      (Recv_CDSBits      ), // received CDS bits
    // Towards the frame controller
    .REQ_CDM_o           (REQ_CDM           ), // CDM frame request (has priority)
    .CDM_Bit_o           (CDM_Bit           ), // CDM bit to encode into this frame
    // Status
    .CDM_Busy_o          (CDM_Busy          ), // CDM access is in progress
    .COMM_Reply_o        (COMM_Reply_o      ), // pulse: COMM_Data_o valid (read)
    .COMM_Data_o         (COMM_Data_o       ), // 
    .Err_CDM_o           (Err_CDM           )  // pulse: access aborted
  );
  
  //--------------------------------------------------------------------
  // Sticky status / counters / data outputs
  //--------------------------------------------------------------------
  
  generate
    if(EN_ERR_LAT)
    begin // Sticky errors
      always_ff @(posedge CLK_i) 
      begin
        if(RST_i || Clear_Status_i) 
        begin
          ERR_NoStart_o  <= 1'b0;
          ERR_NoMarker_o <= 1'b0;
          ERR_BadCRC_o   <= 1'b0;
          ERR_WdExp_o    <= 1'b0;
          ERR_Measure_o  <= 1'b0;
          ERR_CDM_o      <= 1'b0;
        end
        else
        begin
          if(Err_NoStart  ) ERR_NoStart_o  <= 1'b1;
          if(Err_NoMarker ) ERR_NoMarker_o <= 1'b1;
          if(Err_BadCRC   ) ERR_BadCRC_o   <= 1'b1;
          if(Err_WdExp    ) ERR_WdExp_o    <= 1'b1;
          if(Meas_Error   ) ERR_Measure_o  <= 1'b1;
          if(Err_CDM      ) ERR_CDM_o      <= 1'b1;
        end
      end
    end // Pulse errors
    else
    begin
      always_comb
      begin
        ERR_NoStart_o  = Err_NoStart  ;
        ERR_NoMarker_o = Err_NoMarker ;
        ERR_BadCRC_o   = Err_BadCRC   ;
        ERR_WdExp_o    = Err_WdExp    ;
        ERR_Measure_o  = Meas_Error   ;
        ERR_CDM_o      = Err_CDM      ;
      end
    end
  endgenerate
  
  generate
    if(EN_STS_CNT)
    begin // Counters enabled
      logic Err_AnyErr;
      assign Err_AnyErr = |{Err_NoStart, Err_NoMarker, Err_BadCRC, Err_WdExp, Meas_Error};
      
      always_ff @(posedge CLK_i) 
      begin
        if(RST_i || Clear_Status_i) 
        begin
          Count_Frame_o    <=  'd0;
          Count_Error_o    <=  'd0;
        end
        else
        begin
          if(FR_End    ) Count_Frame_o <= Count_Frame_o + 1'd1;
          if(Err_AnyErr) Count_Error_o <= Count_Error_o + 1'd1;
        end
      end
    end // Counters disabled
    else
    begin
      always_comb
      begin
        Count_Frame_o =  'd0;
        Count_Error_o =  'd0;
      end
    end
  endgenerate
  
  
endmodule