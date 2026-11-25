`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/26/2026 
// Design Name: 
// Module Name: prim_BiSS_SSI_TX
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Combo BiSS-C / SSI transmitter (BiSS slave, sensor side).
//              Companion of prim_BiSS_SSI_RX (master); implements the sensor
//              side of the official "BiSS C Protocol Description" (BiSS
//              Association e.V., Rev D2) and a classic SSI slave:
//              
//              - SCD frame on SL, launched at every MA edge of the configured
//                launch polarity (official: rising edges):
//                  LAT (latch point) -> ACK='0' -> STR='1' -> CDS -> PDU
//                  -> CRC (inverted) -> STOP='0' at STP -> timeout -> SL='1'
//                SSI mode sends the raw PDU (MSB first, launched on falling
//                edges) and ends with the monoflop timeout.
//              - Full register communication (control communication): CDM bits
//                are captured at every slave timeout expiry (CDM = inverted MA
//                level) and answered with one CDS bit per frame, one frame
//                later. Register read/write (incl. sequential access, R/W
//                acknowledge with W inversion, busy-able start bit echo),
//                BiSS commands with ID assignment (IDL), ID acknowledge (IDA),
//                broadcast handling and the CMD='00'/'01' protocol effects
//                (short frame / control-communication off) are supported. The
//              register content itself is served by an external backend.
//              - Adaptive timeout (1.5 x TMA, measured during the header) or
//                static timeout; the timeout also forms the frame watchdog.
//              - The top module contains wiring only; all logic lives in:
//                  prim_BiSS_SSI_TX_MAGMon  MA physical layer
//                  prim_BiSS_SSI_TX_Ser     SL frame engine + CRC
//                  prim_BiSS_SSI_TX_CDPSeq  control communication sequencer
//                  prim_BiSS_SSI_TX_Ctrl    settings snapshot + status
//                  prim_SyncEdge            generic 2-FF synchronizer
//              Timing: the SL launch path is a 3-CLK pipeline (2-FF sync ->
//              bit-index mux -> SL register); CRC and frame assembly run one
//              bit-time ahead of the shift-out, off the launch path.
// 
// Dependencies: prim_FractionalTickGen, prim_CRC_Calculator
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments: Settings are snapshotted at the slave timeout expiry
//                      (the BiSS frame boundary) or on Start_i, so changes
//                      land at frame boundaries, mirroring prim_BiSS_SSI_RX.
// 
//////////////////////////////////////////////////////////////////////////////////

module prim_BiSS_SSI_TX
  #(
    parameter  real FREQ_SYSCLK     = 100e6  , // Main system clock frequency, Hz
    parameter  bit  MODE_BAUDCTRL   = 1'b0   , // Baudrate control mode: '1' - fixed baudrate, '0' - variable baudrate
    parameter  int  BAUD_FIXED_DEF  = 500000 , // UART baudrate (for fixed mode)
    parameter  int  MAX_PDUBIT      = 32     , // Widest process-data payload [bits]
    parameter  int  VAL_CRCBIT      =  6     , // BiSS-C CRC width [bits]
    parameter  logic [VAL_CRCBIT-1:0] CRC_POLY     = 'h2F, // Polynome, excluding the x^CRC_WIDTH term
    parameter  logic [VAL_CRCBIT-1:0] CRC_INIT     = 'h2A, // Register seed
    parameter  logic [VAL_CRCBIT-1:0] CRC_XOR_OUT  = 'h15, // Final XOR
    parameter  bit  CRC_TX_INVERT   = 1'b1   , // Official protocol: CRC is inverted before transmission
    parameter  int  WIDTH_CDADDR    =  7     , // Control Communication address payload bit width [bits]
    parameter  int  WIDTH_CDDATA    =  8     , // Control Communication data    payload bit width [bits]
    parameter  int  WIDTH_CDSLID    =  8     , // Control Communication linear-select ID bit width [bits]
    parameter  bit  EN_ERR_LAT      = 1'b1   , // Enable error latching (reset with "Clear_Status_i")
    parameter  bit  EN_STS_CNT      = 1'b1   , // Enable frame and error counters
    parameter  int  WIDTH_CNT_FR    =  4     , // Frame counter width [bits]
    parameter  int  WIDTH_CNT_ER    =  4     , // Error event counter width [bits]
    parameter  int  OVERSAMPLE      = 32     , // Oversampling ticks per MA period
    parameter  int  WIDTH_LIMTTIMO  = 16     , // Static timeout width [ovs ticks]
    parameter  int  WIDTH_LIMHALFM  = 16     , // Over-clock tolerance width [h[launch edges]
    parameter  int  BV_NUM          = 4      ,
    parameter  int  BAUD_VAL [0:BV_NUM-1] = '{500000, 1000000, 1500000, 2000000}, // Baudrates, selectable by "SET_Baudrate_i"
    
    localparam int  WIDTH_PDUBIT    = $clog2(MAX_PDUBIT + 1) , // 
    localparam int  BV_WIDTH        = $clog2(BV_NUM)
  )
  (
    input  logic                      RST_i                , // 
    input  logic                      CLK_i                , // 
    input  logic                      Start_i              , // Initiate a single exchange
    // Parameters
    input  logic                      SET_FSM_Enable_i     , // 0 -> TX idle, SL pinned to idle level
    input  logic                      SET_InterfaceMode_i  , // '1' - SSI, '0' - BiSS
    input  logic                      SET_FSM_Continuous_i , // answer every MA burst forever
    input  logic                      SET_EN_BiSS_CRC_i    , // BiSS: append CRC
    input  logic                      SET_CRC_InclCDS_i    , // 1: CRC covers CDS bit as well as PDU
    input  logic                      SET_Lvl_LaunchSL_i   , // MA level AFTER the edge the TX launches on ('1': rising/BiSS, '0': falling/SSI)
    input  logic                      SET_Lvl_SL_Idle_i    , // SL level while idle (BiSS-C: '1')
    input  logic [  WIDTH_PDUBIT-1:0] SET_PDUBits_i        , // PDU width (clamped to MAX_PDUBIT)
    input  logic [      BV_WIDTH-1:0] SET_Baudrate_i       , // MA speed gear -> BAUD_VAL table above; snapshotted, so speed changes land at frame boundaries
    input  logic                      SET_EN_AdaptTO_i     , // '1' - adaptive timeout (1.5 x TMA), '0' - static
    input  logic                      SET_EN_HoldCDM_i     , // '1' - master holds MA low through the CDM timeout until the next frame
    input  logic [WIDTH_LIMTTIMO-1:0] SET_Limit_TickTO_i   , // Static timeout [ovs ticks], 0 = infinite (debug only)
    input  logic [WIDTH_LIMHALFM-1:0] SET_Limit_HalfMA_i   , // Over-clock tolerance [h[launch edges], 0 = infinite
    input  logic [               3:0] SET_SlaveID_i        , // Own BiSS Slave ID for control communication (point-to-point: 0)
    // Status
    input  logic                      Clear_Status_i       , // Clears frame counter and error flags/counter
    output logic [  WIDTH_CNT_FR-1:0] Count_Frame_o        , // Frame transmitted counter
    output logic [  WIDTH_CNT_ER-1:0] Count_Error_o        , // Error event counter
    output logic                      ERR_WdExp_o          , // Error: Frame watchdog (MA stalled mid-frame)
    output logic                      ERR_Overrun_o        , // Error: MA clocked beyond the frame limit
    output logic                      ERR_CDM_o            , // Error: Control communication failed
    // Input, Data channel
    input  logic [    MAX_PDUBIT-1:0] Send_PDUBits_i       , // Process Data: data in (MSB-justified: bit MAX_PDUBIT-1 goes first)
    output logic                      Send_Latched_o       , // Process Data: pulse, data sampled for the running frame
    // Input/Output, CD channel (register backend)
    output logic                      COMM_Req_o           , // Control Communication: request (access or command started)
    output logic [               1:0] COMM_Type_o          , // Control Communication: '00' single read, '01' single write, '10' command, '11' broadcast command
    output logic [               1:0] COMM_CMD_o           , // Control Communication: command code (bit 1 = CMD1, first on the wire)
    output logic [ WIDTH_CDADDR-1:0]  COMM_Addr_o          , // Control Communication: address out
    output logic                      COMM_Write_o         , // Control Communication: '1' = write access
    output logic [ WIDTH_CDDATA-1:0]  COMM_Data_o          , // Control Communication: write data out
    input  logic                      COMM_Accept_i        , // Control Communication: access accepted (sampled at the W echo)
    input  logic [ WIDTH_CDDATA-1:0]  COMM_RData_i         , // Control Communication: read data in
    input  logic                      COMM_Rdy_i           , // Control Communication: backend ready ('0' delays the S echo = busy)
    input  logic                      COMM_Cont_i          , // Control Communication: allow sequential continuation (P bit)
    output logic                      COMM_WStrb_o         , // Control Communication: verified write strobe
    output logic                      COMM_Reply_o         , // Control Communication: reply (byte completed)
    // Pins
    input  logic                      SS_MA                , // Master -> Slave clock line
    output logic                      SS_SL                  // Master <- Slave data line
  );
  
  //--------------------------------------------------------------------
  // Interconnect declarations
  //--------------------------------------------------------------------
  
  // Tick generator
  logic TickPulse; // 
  
  // Settings buffer
  logic                      SetR_FSM_Enable     ; // 
  logic                      SetR_InterfaceMode  ; // 
  logic                      SetR_FSM_Continuous ; // 
  logic                      SetR_EN_BiSS_CRC    ; // 
  logic                      SetR_CRC_InclCDS    ; // 
  logic                      SetR_Lvl_LaunchSL   ; // 
  logic                      SetR_Lvl_SL_Idle    ; // 
  logic [  WIDTH_PDUBIT-1:0] SetR_PDUBits        ; // 
  logic [      BV_WIDTH-1:0] SetR_Baudrate       ; // 
  logic                      SetR_EN_AdaptTO     ; // 
  logic                      SetR_EN_HoldCDM     ; // 
  logic [WIDTH_LIMTTIMO-1:0] SetR_Limit_TickTO   ; // 
  logic [WIDTH_LIMHALFM-1:0] SetR_Limit_HalfMA   ; // 
  
  // One-shot
  logic StartPending; // 
  
  // MA monitor
  logic TO_Expire    ; // 
  logic CDM_Bit      ; // 
  logic MA_LaunchEdge; // 
  logic FR_Start     ; // 
  
  // Frame engine
  logic FR_Rsp       ; // 
  logic FR_End       ; // 
  logic Err_WdExp    ; // 
  logic Err_Overrun  ; // 
  
  // Control communication
  logic CDS_Bit      ; // 
  logic CDM_Busy     ; // 
  logic EN_PDUCH     ; // 
  logic Err_CDM      ; // 
  
  //--------------------------------------------------------------------
  // Oversampling tick generator (nominal MA rate x OVERSAMPLE)
  //--------------------------------------------------------------------
  
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
    .Resync_i    (1'b0          ), // Free-running, like prim_BiSS_SSI_RX
    .TickPulse_o (TickPulse     )  // Tick pulse output
  );
  
  //--------------------------------------------------------------------
  // Settings snapshot / status
  //--------------------------------------------------------------------
  
  prim_BiSS_SSI_TX_Ctrl
  #(
    .WIDTH_PDUBIT   (WIDTH_PDUBIT   ), // 
    .WIDTH_BAUDSEL  (BV_WIDTH       ), // 
    .WIDTH_LIMTTIMO (WIDTH_LIMTTIMO ), // 
    .WIDTH_LIMHALFM (WIDTH_LIMHALFM ), // 
    .MAX_PDUBIT     (MAX_PDUBIT     ), // 
    .EN_ERR_LAT     (EN_ERR_LAT     ), // 
    .EN_STS_CNT     (EN_STS_CNT     ), // 
    .WIDTH_CNT_FR   (WIDTH_CNT_FR   ), // 
    .WIDTH_CNT_ER   (WIDTH_CNT_ER   )  // 
  )
  Control
  (
    .RST_i                 (RST_i               ), // 
    .CLK_i                 (CLK_i               ), // 
    .SET_FSM_Enable_i      (SET_FSM_Enable_i    ), // 
    .SET_InterfaceMode_i   (SET_InterfaceMode_i ), // 
    .SET_FSM_Continuous_i  (SET_FSM_Continuous_i), // 
    .SET_EN_BiSS_CRC_i     (SET_EN_BiSS_CRC_i   ), // 
    .SET_CRC_InclCDS_i     (SET_CRC_InclCDS_i   ), // 
    .SET_Lvl_LaunchSL_i    (SET_Lvl_LaunchSL_i  ), // 
    .SET_Lvl_SL_Idle_i     (SET_Lvl_SL_Idle_i   ), // 
    .SET_PDUBits_i         (SET_PDUBits_i       ), // 
    .SET_Baudrate_i        (SET_Baudrate_i      ), // 
    .SET_EN_AdaptTO_i      (SET_EN_AdaptTO_i    ), // 
    .SET_EN_HoldCDM_i      (SET_EN_HoldCDM_i    ), // 
    .SET_Limit_TickTO_i    (SET_Limit_TickTO_i  ), // 
    .SET_Limit_HalfMA_i    (SET_Limit_HalfMA_i  ), // 
    .Start_i               (Start_i             ), // 
    .TO_Expire_i           (TO_Expire           ), // 
    .FR_Rsp_i              (FR_Rsp              ), // 
    .CDM_Busy_i            (CDM_Busy            ), // 
    .FR_End_i              (FR_End              ), // 
    .Err_WdExp_i           (Err_WdExp           ), // 
    .Err_Overrun_i         (Err_Overrun         ), // 
    .Err_CDM_i             (Err_CDM             ), // 
    .Clear_Status_i        (Clear_Status_i      ), // 
    .SetR_FSM_Enable_o     (SetR_FSM_Enable     ), // 
    .SetR_InterfaceMode_o  (SetR_InterfaceMode  ), // 
    .SetR_FSM_Continuous_o (SetR_FSM_Continuous ), // 
    .SetR_EN_BiSS_CRC_o    (SetR_EN_BiSS_CRC    ), // 
    .SetR_CRC_InclCDS_o    (SetR_CRC_InclCDS    ), // 
    .SetR_Lvl_LaunchSL_o   (SetR_Lvl_LaunchSL   ), // 
    .SetR_Lvl_SL_Idle_o    (SetR_Lvl_SL_Idle    ), // 
    .SetR_PDUBits_o        (SetR_PDUBits        ), // 
    .SetR_Baudrate_o       (SetR_Baudrate       ), // 
    .SetR_EN_AdaptTO_o     (SetR_EN_AdaptTO     ), // 
    .SetR_EN_HoldCDM_o     (SetR_EN_HoldCDM     ), // 
    .SetR_Limit_TickTO_o   (SetR_Limit_TickTO   ), // 
    .SetR_Limit_HalfMA_o   (SetR_Limit_HalfMA   ), // 
    .StartPending_o        (StartPending        ), // 
    .Count_Frame_o         (Count_Frame_o       ), // 
    .Count_Error_o         (Count_Error_o       ), // 
    .ERR_WdExp_o           (ERR_WdExp_o         ), // 
    .ERR_Overrun_o         (ERR_Overrun_o       ), // 
    .ERR_CDM_o             (ERR_CDM_o           )  // 
  );
  
  //--------------------------------------------------------------------
  // MA physical layer: edge strobes, frame start, timeout, CDM sampling
  //--------------------------------------------------------------------
  
  prim_BiSS_SSI_TX_MAGMon
  #(
    .OVERSAMPLE     (OVERSAMPLE    ), // Oversampling ticks per MA period
    .WIDTH_LIMTTIMO (WIDTH_LIMTTIMO)  // Static timeout width [ovs ticks]
  )
  MA_Monitor
  (
    .RST_i               (RST_i             ), // 
    .CLK_i               (CLK_i             ), // 
    .SS_MA               (SS_MA             ), // 
    .TickPulse_i         (TickPulse         ), // 
    .SET_Lvl_LaunchSL_i  (SetR_Lvl_LaunchSL ), // 
    .SET_EN_AdaptTO_i    (SetR_EN_AdaptTO   ), // 
    .SET_EN_HoldCDM_i    (SetR_EN_HoldCDM   ), // 
    .SET_Limit_TickTO_i  (SetR_Limit_TickTO ), // 
    .TO_Expire_o         (TO_Expire         ), // 
    .CDM_Bit_o           (CDM_Bit           ), // 
    .MA_LaunchEdge_o     (MA_LaunchEdge     ), // 
    .FR_Start_o          (FR_Start          )  // 
  );
  
  //--------------------------------------------------------------------
  // Control communication sequencer (CDM ingest / CDS response)
  //--------------------------------------------------------------------
  
  prim_BiSS_SSI_TX_CDPSeq
  #(
    .WIDTH_CDADDR   (WIDTH_CDADDR ), // 
    .WIDTH_CDDATA   (WIDTH_CDDATA ), // 
    .WIDTH_CDSLID   (WIDTH_CDSLID )  // 
  )
  CDP_Sequencer
  (
    .RST_i               (RST_i             ), // 
    .CLK_i               (CLK_i             ), // 
    .SET_InterfaceMode_i (SetR_InterfaceMode), // 
    .SET_SlaveID_i       (SET_SlaveID_i     ), // 
    .TO_Expire_i         (TO_Expire         ), // 
    .CDM_Bit_i           (CDM_Bit           ), // 
    .COMM_Req_o          (COMM_Req_o        ), // 
    .COMM_Type_o         (COMM_Type_o       ), // 
    .COMM_CMD_o          (COMM_CMD_o        ), // 
    .COMM_Addr_o         (COMM_Addr_o       ), // 
    .COMM_Write_o        (COMM_Write_o      ), // 
    .COMM_Data_o         (COMM_Data_o       ), // 
    .COMM_Accept_i       (COMM_Accept_i     ), // 
    .COMM_RData_i        (COMM_RData_i      ), // 
    .COMM_Rdy_i          (COMM_Rdy_i        ), // 
    .COMM_Cont_i         (COMM_Cont_i       ), // 
    .COMM_WStrb_o        (COMM_WStrb_o      ), // 
    .COMM_Reply_o        (COMM_Reply_o      ), // 
    .CDS_Bit_o           (CDS_Bit           ), // 
    .CDM_Busy_o          (CDM_Busy          ), // 
    .EN_PDUCH_o          (EN_PDUCH         ), // 
    .Err_CDM_o           (Err_CDM           )  // 
  );
  
  //--------------------------------------------------------------------
  // SL frame engine
  //--------------------------------------------------------------------
  
  prim_BiSS_SSI_TX_Ser
  #(
    .MAX_PDUBIT     (MAX_PDUBIT    ), // Widest process-data payload [bits]
    .VAL_CRCBIT     (VAL_CRCBIT    ), // BiSS-C CRC width [bits]
    .CRC_POLY       (CRC_POLY      ), // Polynome, excluding the x^CRC_WIDTH term
    .CRC_INIT       (CRC_INIT      ), // Register seed
    .CRC_XOR_OUT    (CRC_XOR_OUT   ), // Final XOR
    .CRC_TX_INVERT  (CRC_TX_INVERT ), // Official protocol: CRC inverted before transmission
    .WIDTH_PDUBIT   (WIDTH_PDUBIT  ), // 
    .WIDTH_LIMHALFM (WIDTH_LIMHALFM)  // 
  )
  SL_Serializer
  (
    .RST_i                (RST_i              ), // 
    .CLK_i                (CLK_i              ), // 
    .SET_InterfaceMode_i  (SetR_InterfaceMode ), // 
    .SET_EN_BiSS_CRC_i    (SetR_EN_BiSS_CRC   ), // 
    .SET_CRC_InclCDS_i    (SetR_CRC_InclCDS   ), // 
    .SET_Lvl_SL_Idle_i    (SetR_Lvl_SL_Idle   ), // 
    .SET_PDUBits_i        (SetR_PDUBits       ), // 
    .SET_Limit_HalfMA_i   (SetR_Limit_HalfMA  ), // 
    .SET_FSM_Enable_i     (SetR_FSM_Enable    ), // 
    .SET_FSM_Continuous_i (SetR_FSM_Continuous), // 
    .StartPending_i       (StartPending       ), // 
    .CDM_Busy_i           (CDM_Busy           ), // 
    .EN_PDUCH_i           (EN_PDUCH           ), // 
    .Send_PDUBits_i       (Send_PDUBits_i     ), // 
    .Send_Latched_o       (Send_Latched_o     ), // 
    .CDS_Bit_i            (CDS_Bit            ), // 
    .MA_LaunchEdge_i      (MA_LaunchEdge      ), // 
    .FR_Start_i           (FR_Start           ), // 
    .TO_Expire_i          (TO_Expire          ), // 
    .FR_Rsp_o             (FR_Rsp             ), // 
    .FR_End_o             (FR_End             ), // 
    .Err_WdExp_o          (Err_WdExp          ), // 
    .Err_Overrun_o        (Err_Overrun        ), // 
    .SS_SL                (SS_SL              )  // 
  );
  
endmodule
