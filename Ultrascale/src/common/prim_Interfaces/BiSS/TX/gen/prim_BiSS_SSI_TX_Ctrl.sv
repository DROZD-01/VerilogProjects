`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/26/2026 
// Design Name: 
// Module Name: prim_BiSS_SSI_TX_Ctrl
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Control & status block of the BiSS-C/SSI transmitter (slave).
//              Contains all housekeeping that is not frame-format related:
//              - Settings snapshot registers (SetR_*). Live settings are captured
//                at the slave timeout expiry (the natural frame boundary of a
//                BiSS slave, equivalent to the FR_Load snapshot of the companion
//                master prim_BiSS_SSI_RX) or on Start_i, so that configuration
//                changes always land at frame boundaries.
//              - One-shot bookkeeping (Start_i arms a single exchange, consumed
//                by a responded data frame - frames that belong to a running
//                control-communication transaction do not consume it).
//              - Sticky / passthrough error flags and frame / error counters
//                (generate structure mirrors prim_BiSS_SSI_RX).
//              The module intentionally holds no protocol knowledge beyond this.
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module prim_BiSS_SSI_TX_Ctrl
  #(
    parameter int  WIDTH_PDUBIT    = $clog2(32+1)  , // PDU width field width [bits]
    parameter int  WIDTH_BAUDSEL   = 2             , // Baudrate selector width [bits]
    parameter int  WIDTH_LIMTTIMO  = 16            , // Static timeout width [ovs ticks]
    parameter int  WIDTH_LIMHALFM  = 16            , // Over-clock tolerance width [h[launch edges]
    parameter int  MAX_PDUBIT      = 32            , // Widest process-data payload [bits]
    parameter bit  EN_ERR_LAT      = 1'b1          , // Enable error latching (reset with "Clear_Status_i")
    parameter bit  EN_STS_CNT      = 1'b1          , // Enable frame and error counters
    parameter int  WIDTH_CNT_FR    = 4             , // Frame counter width [bits]
    parameter int  WIDTH_CNT_ER    = 4               // Error event counter width [bits]
  )
  (
    input  logic                      RST_i                , // 
    input  logic                      CLK_i                , // 
    // Live settings (mirror of the prim_BiSS_SSI_TX port list)
    input  logic                      SET_FSM_Enable_i     , // 0 -> TX idle, SL pinned to idle level
    input  logic                      SET_InterfaceMode_i  , // '1' - SSI, '0' - BiSS
    input  logic                      SET_FSM_Continuous_i , // answer every MA burst forever
    input  logic                      SET_EN_BiSS_CRC_i    , // BiSS: append CRC
    input  logic                      SET_CRC_InclCDS_i    , // 1: CRC covers CDS bit as well as PDU
    input  logic                      SET_Lvl_LaunchSL_i   , // MA level AFTER the edge the TX launches on
    input  logic                      SET_Lvl_SL_Idle_i    , // SL level while idle (BiSS-C: '1')
    input  logic [  WIDTH_PDUBIT-1:0] SET_PDUBits_i        , // PDU width (clamped to MAX_PDUBIT)
    input  logic [ WIDTH_BAUDSEL-1:0] SET_Baudrate_i       , // MA speed gear -> BAUD_VAL table
    input  logic                      SET_EN_AdaptTO_i     , // '1' - adaptive timeout (1.5 x TMA), '0' - static
    input  logic                      SET_EN_HoldCDM_i     , // '1' - master holds MA low through the timeout until the next frame (Hold-CDM)
    input  logic [WIDTH_LIMTTIMO-1:0] SET_Limit_TickTO_i   , // Static timeout [ovs ticks], 0 = infinite (debug!)
    input  logic [WIDTH_LIMHALFM-1:0] SET_Limit_HalfMA_i   , // Over-clock tolerance [h[launch edges], 0 = infinite
    input  logic                      Start_i              , // Initiate a single exchange
    // Events
    input  logic                      TO_Expire_i          , // Slave timeout expired (snapshot point)
    input  logic                      FR_Rsp_i             , // Pulse: a frame is being responded to
    input  logic                      CDM_Busy_i           , // Level: control-communication transaction active
    input  logic                      FR_End_i             , // Pulse: frame ended (completed or aborted)
    input  logic                      Err_WdExp_i          , // Pulse: MA stalled / frame incomplete at timeout
    input  logic                      Err_Overrun_i        , // Pulse: MA clocked beyond the frame limit
    input  logic                      Err_CDM_i            , // Pulse: control-communication error
    input  logic                      Clear_Status_i       , // Clears frame counter and error flags/counter
    // Snapshot-domain settings
    output logic                      SetR_FSM_Enable_o    , // 
    output logic                      SetR_InterfaceMode_o , // 
    output logic                      SetR_FSM_Continuous_o, // 
    output logic                      SetR_EN_BiSS_CRC_o   , // 
    output logic                      SetR_CRC_InclCDS_o   , // 
    output logic                      SetR_Lvl_LaunchSL_o  , // 
    output logic                      SetR_Lvl_SL_Idle_o   , // 
    output logic [  WIDTH_PDUBIT-1:0] SetR_PDUBits_o       , // 
    output logic [ WIDTH_BAUDSEL-1:0] SetR_Baudrate_o      , // 
    output logic                      SetR_EN_AdaptTO_o    , // 
    output logic                      SetR_EN_HoldCDM_o    , // 
    output logic [WIDTH_LIMTTIMO-1:0] SetR_Limit_TickTO_o  , // 
    output logic [WIDTH_LIMHALFM-1:0] SetR_Limit_HalfMA_o  , // 
    // One-shot status
    output logic                      StartPending_o       , // 
    // Status
    output logic [  WIDTH_CNT_FR-1:0] Count_Frame_o        , // Frame transmitted counter
    output logic [  WIDTH_CNT_ER-1:0] Count_Error_o        , // Error event counter
    output logic                      ERR_WdExp_o          , // Error: Frame watchdog (MA stalled mid-frame)
    output logic                      ERR_Overrun_o        , // Error: MA clocked beyond the frame limit
    output logic                      ERR_CDM_o              // Error: Control communication failed
  );
  
  //--------------------------------------------------------------------
  // Settings snapshot: captured at timeout expiry (frame boundary of a
  // BiSS slave) or on Start_i. Reset defaults follow the official
  // "BiSS C Protocol Description": SL idles at '1' and every slave must
  // support the adaptive timeout, so both reset to the protocol default.
  //--------------------------------------------------------------------
  
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i) 
    begin
      SetR_FSM_Enable_o     <= 1'b0;
      SetR_InterfaceMode_o  <= 1'b0;
      SetR_FSM_Continuous_o <= 1'b0;
      SetR_EN_BiSS_CRC_o    <= 1'b0;
      SetR_CRC_InclCDS_o    <= 1'b0;
      SetR_Lvl_LaunchSL_o   <= 1'b1;
      SetR_Lvl_SL_Idle_o    <= 1'b1;
      SetR_PDUBits_o        <=  'd0;
      SetR_Baudrate_o       <=  'd0;
      SetR_EN_AdaptTO_o     <= 1'b1;
      SetR_EN_HoldCDM_o     <= 1'b0;
      SetR_Limit_TickTO_o   <=  'd0;
      SetR_Limit_HalfMA_o   <=  'd0;
    end
    else if(TO_Expire_i || Start_i)
    begin
      SetR_FSM_Enable_o     <= SET_FSM_Enable_i     ; // 
      SetR_InterfaceMode_o  <= SET_InterfaceMode_i  ; // 
      SetR_FSM_Continuous_o <= SET_FSM_Continuous_i ; // 
      SetR_EN_BiSS_CRC_o    <= SET_EN_BiSS_CRC_i    ; // 
      SetR_CRC_InclCDS_o    <= SET_CRC_InclCDS_i    ; // 
      SetR_Lvl_LaunchSL_o   <= SET_Lvl_LaunchSL_i   ; // 
      SetR_Lvl_SL_Idle_o    <= SET_Lvl_SL_Idle_i    ; // 
      SetR_PDUBits_o        <= (SET_PDUBits_i > WIDTH_PDUBIT'(MAX_PDUBIT))? 
                               WIDTH_PDUBIT'(MAX_PDUBIT) : SET_PDUBits_i; // clamped
      SetR_Baudrate_o       <= SET_Baudrate_i       ; // 
      SetR_EN_AdaptTO_o     <= SET_EN_AdaptTO_i     ; // 
      SetR_EN_HoldCDM_o     <= SET_EN_HoldCDM_i     ; // 
      SetR_Limit_TickTO_o   <= SET_Limit_TickTO_i   ; // 
      SetR_Limit_HalfMA_o   <= SET_Limit_HalfMA_i   ; // 
    end
  end
  
  //--------------------------------------------------------------------
  // One-shot bookkeeping (mirrors prim_BiSS_SSI_RX: a pending exchange is
  // consumed by a responded data frame; frames that carry control-
  // communication bits while a transaction is running do not consume it)
  //--------------------------------------------------------------------
  
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i || (FR_Rsp_i && !CDM_Busy_i)) 
      StartPending_o <= 1'b0;
    else if(Start_i)
      StartPending_o <= 1'b1;
  end
  
  //--------------------------------------------------------------------
  // Sticky status / counters
  //--------------------------------------------------------------------
  
  generate
    if(EN_ERR_LAT)
    begin // Sticky errors
      always_ff @(posedge CLK_i) 
      begin
        if(RST_i || Clear_Status_i) 
        begin
          ERR_WdExp_o   <= 1'b0;
          ERR_Overrun_o <= 1'b0;
          ERR_CDM_o     <= 1'b0;
        end
        else
        begin
          if(Err_WdExp_i  ) ERR_WdExp_o   <= 1'b1;
          if(Err_Overrun_i) ERR_Overrun_o <= 1'b1;
          if(Err_CDM_i    ) ERR_CDM_o     <= 1'b1;
        end
      end
    end // Pulse errors
    else
    begin
      always_comb
      begin
        ERR_WdExp_o   = Err_WdExp_i  ;
        ERR_Overrun_o = Err_Overrun_i;
        ERR_CDM_o     = Err_CDM_i    ;
      end
    end
  endgenerate
  
  generate
    if(EN_STS_CNT)
    begin // Counters enabled
      logic Err_AnyErr;
      assign Err_AnyErr = |{Err_WdExp_i, Err_Overrun_i, Err_CDM_i};
      
      always_ff @(posedge CLK_i) 
      begin
        if(RST_i || Clear_Status_i) 
        begin
          Count_Frame_o <=  'd0;
          Count_Error_o <=  'd0;
        end
        else
        begin
          if(FR_End_i  ) Count_Frame_o <= Count_Frame_o + 1'd1;
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
