`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/26/2026 
// Design Name: 
// Module Name: prim_BiSS_SSI_TX_MAGMon
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: MA physical-layer monitor of the BiSS-C/SSI transmitter (slave).
//              Implements everything the "BiSS C Protocol Description" demands
//              from a slave with respect to the clock input MAI:
//              - Synchronization and edge strobes; the SL launch edge is the MA
//                edge after which MA assumes the "launch" level ('1' = rising
//                edge per protocol, configurable for SSI where data is launched
//                on falling edges).
//              - Frame start recognition: the first launch edge after the slave
//                has re-armed (timeout expiry or reset) is LAT. This covers both
//                the normal "falling edge followed by rising edge" start and the
//                Hold-CDM case where MA is still low when the slave re-arms.
//              - Adaptive timeout measurement: the MA period is measured in
//                oversampling ticks between the first two launch edges of every
//                frame ("measured during the Header phase"); the adaptive
//                timeout is 1.5 x TMA plus a 3-tick sampling margin.
//              - Slave timeout: a free-running timer that is cleared by every MA
//                edge and fires once, TO ticks after the last edge. At expiry
//                the CDM bit is sampled (CDM = inverted MA level, per protocol
//                the master holds MA low through the timeout to send CDM='1').
//                The timeout doubles as the frame watchdog: firing while the
//                serializer is still mid-frame aborts that frame.
// 
// Dependencies: prim_SyncEdge, prim_FractionalTickGen (tick source, in top)
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module prim_BiSS_SSI_TX_MAGMon
  #(
    parameter int  OVERSAMPLE     = 32    , // Oversampling ticks per MA period
    parameter int  WIDTH_LIMTTIMO = 16      // Static timeout width [ovs ticks]
  )
  (
    input  logic                  RST_i              , // 
    input  logic                  CLK_i              , // 
    input  logic                  SS_MA              , // Master -> Slave clock line
    // Tick source
    input  logic                  TickPulse_i        , // Oversampling tick strobe
    // Settings (snapshot domain)
    input  logic                  SET_Lvl_LaunchSL_i , // MA level AFTER the edge the TX launches on
    input  logic                  SET_EN_AdaptTO_i   , // '1' - adaptive timeout (1.5 x TMA), '0' - static
    input  logic                  SET_EN_HoldCDM_i   , // '1' - master holds MA low through the timeout until the next frame (Hold-CDM)
    input  logic [WIDTH_LIMTTIMO-1:0] SET_Limit_TickTO_i, // Static timeout [ovs ticks], 0 = infinite (debug)
    // Events
    output logic                  TO_Expire_o        , // Pulse: slave timeout expired
    output logic                  CDM_Bit_o          , // CDM bit sampled at timeout expiry
    output logic                  MA_LaunchEdge_o    , // Comb strobe: MA edge the TX launches on
    output logic                  FR_Start_o           // Comb strobe: first launch edge after re-arm (LAT)
  );
  
  // MA period counter: nominal one period = OVERSAMPLE ticks, give room for
  // master/tick-rate mismatch; the adaptive timeout adds 50% plus margin.
  localparam int TMAW   = $clog2(OVERSAMPLE) + 3;
  localparam int TOCNTW = (WIDTH_LIMTTIMO > TMAW+1)? WIDTH_LIMTTIMO : TMAW+1;
  
  //--------------------------------------------------------------------
  // MA synchronization and edge strobes
  //--------------------------------------------------------------------
  
  logic MA_Sync     ; // 
  logic MA_Rise     ; // 
  logic MA_Fall     ; // 
  logic MA_AnyEdge  ; // 
  
  prim_SyncEdge
  #(
    .RESET_VAL (1'b1)   // MA idles high in BiSS-C and classic SSI
  )
  MA_Synchronizer
  (
    .RST_i  (RST_i    ), // 
    .CLK_i  (CLK_i    ), // 
    .SIG_i  (SS_MA    ), // 
    .SIG_o  (MA_Sync  ), // 
    .Rise_o (MA_Rise  ), // 
    .Fall_o (MA_Fall  ), // 
    .Any_o  (MA_AnyEdge)  // 
  );
  
  // Launch edge: the MA edge after which MA assumes the launch level
  // ('1' = rising edge, the BiSS-C protocol default; '0' = falling edge,
  // classic SSI). Selected directly from the edge strobes so the decision
  // uses the post-edge level, not the delayed synchronized level.
  assign MA_LaunchEdge_o = SET_Lvl_LaunchSL_i ? MA_Rise : MA_Fall;
  
  //--------------------------------------------------------------------
  // CDM-release swallowing: when a master sends CDM='1' it holds MA low
  // through the slave timeout. A Hold-CDM master then keeps MA low until
  // the next frame, so the next rising edge IS that frame's LAT. A master
  // that releases MA to idle instead produces one rising edge (the idle
  // entry) that must not start a frame and must not restart the timeout
  // (its CDM bit was already sampled at the expiry). With
  // SET_EN_HoldCDM_i='0' that edge is swallowed; with '1' it is treated as
  // the next frame's LAT.
  //--------------------------------------------------------------------
  
  logic SwallowRise ; // Swallow the next rising edge (CDM release to idle)
  logic SwallowedRise; // This cycle's rising edge is being swallowed
  
  assign SwallowedRise = MA_Rise && SwallowRise;
  
  //--------------------------------------------------------------------
  // Re-arm / frame start: the first launch edge seen after the timeout
  // expired (or after reset) starts a frame. Non-launch edges (e.g. the
  // leading falling edge of a normal frame) keep the slave armed.
  //--------------------------------------------------------------------
  
  logic Armed;
  
  assign FR_Start_o = MA_LaunchEdge_o && Armed && !SwallowedRise;
  
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i) 
    begin
      Armed      <= 1'b1;
      SwallowRise <= 1'b0;
    end
    else
    begin
      if(TO_Expire_o)
      begin
        Armed      <= 1'b1;
        SwallowRise <= (!SET_EN_HoldCDM_i) && !MA_Sync; // MA still low: expect the release
      end
      else if(FR_Start_o)
        Armed <= 1'b0;
      
      if(SwallowedRise)
        SwallowRise <= 1'b0;
    end
  end
  
  //--------------------------------------------------------------------
  // Adaptive timeout: measure one MA period in oversampling ticks between
  // the first two launch edges of every frame ("during the Header phase").
  //--------------------------------------------------------------------
  
  logic [TMAW-1:0] TMA_Cnt  ; // Tick counter since the frame start edge
  logic [TMAW-1:0] TMA_Ticks; // Measured MA period [ovs ticks]
  logic            MeasBusy ; // Measurement running
  
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i) 
    begin
      TMA_Cnt   <= TMAW'(OVERSAMPLE); // Nominal period as power-on default
      TMA_Ticks <= TMAW'(OVERSAMPLE);
      MeasBusy  <= 1'b0;
    end
    else
    begin
      if(FR_Start_o)
      begin
        TMA_Cnt  <= 'd0;
        MeasBusy <= 1'b1;
      end
      else if(MA_LaunchEdge_o && MeasBusy)
      begin
        TMA_Ticks <= TMA_Cnt; // Distance between two launch edges = 1 period
        MeasBusy  <= 1'b0;
      end
      else if(TickPulse_i && MeasBusy && !(&TMA_Cnt))
        TMA_Cnt <= TMA_Cnt + 1'd1;
    end
  end
  
  //--------------------------------------------------------------------
  // Slave timeout: cleared by every MA edge, fires once TO ticks after
  // the last edge, then waits for the next edge. At expiry the CDM bit
  // is sampled (CDM = inverted MA level on the wire).
  //--------------------------------------------------------------------
  
  logic [TOCNTW-1:0] TO_Cnt    ; // Ticks since the last MA edge
  logic [TOCNTW-1:0] TO_Limit  ; // Effective timeout [ovs ticks]
  logic              TO_Fired  ; // One-shot latch, cleared by the next MA edge
  logic              TO_Enable ; // Timeout armed (static '0' means infinite)
  
  function automatic logic [TOCNTW-1:0] SetAdaptTO_f ();
    if(SET_EN_AdaptTO_i)
      return TOCNTW'(TMA_Ticks) + TOCNTW'(TMA_Ticks >> 1) + TOCNTW'(3); // 1.5 x TMA + margin
    else
      return TOCNTW'(SET_Limit_TickTO_i);
  endfunction
  
  always_comb
  begin
    TO_Limit  = SetAdaptTO_f();
    TO_Enable = SET_EN_AdaptTO_i || (SET_Limit_TickTO_i != 'd0);
  end
  
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i) 
    begin
      TO_Cnt     <= 'd0;
      TO_Fired   <= 1'b0;
      TO_Expire_o <= 1'b0;
      CDM_Bit_o  <= 1'b0;
    end
    else
    begin
      TO_Expire_o <= 1'b0;
      
      if(MA_AnyEdge && !SwallowedRise)
      begin
        TO_Cnt   <= 'd0;
        TO_Fired <= 1'b0;
      end
      else if(TickPulse_i && TO_Enable && !TO_Fired)
      begin
        if(TO_Cnt >= (TO_Limit - 1'd1))
        begin
          TO_Expire_o <= 1'b1;
          TO_Fired    <= 1'b1;
          CDM_Bit_o   <= ~MA_Sync; // CDM is the inverted MA level
        end
        else
          TO_Cnt <= TO_Cnt + 1'd1;
      end
    end
  end
  
endmodule
