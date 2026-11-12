`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/09/2026 08:28:26 PM
// Design Name: 
// Module Name: prim_BiSS_SSI_MAGen
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


module prim_BiSS_SSI_MAGen
  #(
    //parameter  int MAX_FRAME_BITS = 75 , // 
    parameter int WIDTH_FRAMEBIT =   8 , // Frame length [bits]
    parameter int WIDTH_INTFRGAP =  16 , // inter-frame gap [ovs ticks]
    parameter int WIDTH_LIMHALFM =  16 , // extra MA cycles before TX abort
    parameter int WIDTH_LIMTTIMO =  24 , // frame watchdog [ovs ticks], 0 = infinite
    parameter int WIDTH_EXTRTCDM =  12   // CDM '1' start extension [ovs ticks]
  )
  (
    input  logic                      RST_i               , // 
    input  logic                      CLK_i               , // 
    // Controls
    input  logic                      SET_InterfaceMode_i , // '1' - SSI, '0' - BiSS
    input  logic [WIDTH_FRAMEBIT-1:0] SET_FrameBits_i     , // Frame length [bits]
    input  logic [WIDTH_INTFRGAP-1:0] SET_InterFrameGap_i , // inter-frame gap [ovs ticks]
    input  logic [WIDTH_LIMHALFM-1:0] SET_Limit_HalfMA_i  , // extra MA cycles before TX abort
    input  logic [WIDTH_LIMTTIMO-1:0] SET_Limit_TickTO_i  , // frame watchdog [ovs ticks], 0 = infinite
    input  logic [WIDTH_EXTRTCDM-1:0] SET_Extra_TickCDM_i , // CDM '1' start extension [ovs ticks]
    input  logic                      SET_Lvl_MA_Idle     , // MA level while idle
    input  logic                      SET_FSM_Enable      , // global enable
    // Clocking
    input  logic                      TickPulse_i         , // oversampling tick strobe
    input  logic                      HalfPulse_i         , // half-period boundary strobe
    // Frame Requests
    input  logic                      REQ_Data            , // process-data frame request
    input  logic                      REQ_CDM             , // CDM frame request (has priority)
    input  logic                      CDM_Bit             , // CDM bit to encode into this frame
    // Feedback
    input  logic                      RX_FrameDone_i      , // RX completed the frame (delayed!)
    // Events
    output logic                      FR_Load_o           , // 
    output logic                      FR_Launch_o         , // 
    output logic                      FR_Abort_o          , // 
    output logic                      FR_End_o            , // 
    output logic                      MA_RefEdge_o        , // Reference edge pulse (BiSS - 2nd rising MA edge; SSI - 1st MA edge)
    output logic                      WatchdogExpired_o   , // 
    // Status & I/O
    output logic                      FR_Active_o         , // 
    output logic                      FR_IsCDM_o          , // 
    output logic                      MA_o                  // 
  );
  
  typedef enum logic [2:0] 
  {
    MF_IDLE    = 3'd0,
    MF_CDM_DLY = 3'd1,
    MF_RUN     = 3'd2,
    MF_GAP     = 3'd3
  } mf_state_e;
  
  localparam int MAX_HALF_W = WIDTH_LIMHALFM + 1;
  
  
  mf_state_e                 Main_SM             ; // 
  logic [    MAX_HALF_W-1:0] HalfMA_Count        ; // MA half-cycles clocked this frame
  logic [    MAX_HALF_W-1:0] HalfMA_Max          ; // margin abort bound
  logic [               1:0] PosEdge_Count       ; // rising MA edges seen this frame
  logic                      MA_Launched         ; // first MA edge emitted
  logic                      RX_Done_Latch       ; // sticky of RX_FrameDone_i
  logic [WIDTH_INTFRGAP-1:0] InterFrameGap_Count ; // 
  logic [WIDTH_EXTRTCDM-1:0] Extra_TickCDM_Count ; // 
  logic [WIDTH_LIMTTIMO-1:0] Limit_TickTO_Count  ; // 
  
  logic MA_NextLevel  ; // MA level AFTER the edge that an HalfPulse_i strobe produces
  logic FR_Accept     ; // frame acceptance (combinational)
  
  always_comb
  begin
    MA_NextLevel = ~MA_o;
    FR_Accept    = (Main_SM == MF_IDLE) && SET_FSM_Enable && (REQ_Data || REQ_CDM);
    FR_Load_o    = FR_Accept;
    FR_IsCDM_o   = FR_Accept && REQ_CDM;
  end
  
  //--------------------------------------------------------------------
  // Combinational, tick-aligned edge events. Every input is a registered
  // signal or a registered strobe, so the pulses are glitch-safe.
  //   Edge_Now      : this boundary actually toggles MA (it is not the
  //                   idle-aligned clean-stop strobe at frame end);
  //   Edge_Launch   : the frame's first MA edge;
  //   Edge_Positive : a rising MA edge of the frame (the launch edge included,
  //                   since PosEdge_Count is 0 during its HalfPulse_i cycle).
  //--------------------------------------------------------------------
  logic Edge_Now      ; 
  logic Edge_Launch   ; 
  logic Edge_Positive ; 
  
  always_comb
  begin
    Edge_Now      = (Main_SM == MF_RUN) && HalfPulse_i && ~(RX_Done_Latch && (MA_o == SET_Lvl_MA_Idle));
    Edge_Launch   = Edge_Now && MA_Launched ;
    Edge_Positive = Edge_Now && MA_NextLevel;
    FR_Launch_o   = Edge_Launch;
  end
  
  //--------------------------------------------------------------------
  // Delay-measurement reference edge: the burst-start edge for SSI, the 2nd
  // rising MA edge (ACK edge) for BiSS - PosEdge_Count reads 1 while that edge's
  // HalfPulse_i cycle is in progress.
  //--------------------------------------------------------------------
  assign MA_RefEdge_o = (SET_InterfaceMode_i)? Edge_Launch : (Edge_Positive && (PosEdge_Count == 2'd1));
  
  //--------------------------------------------------------------------
  // Abort bound: 2*(frame_bits + margin_cyc) half-cycles. margin_cyc == 0
  // saturates the bound (practically infinite); the watchdog is then the
  // only frame limit.
  //--------------------------------------------------------------------
  assign HalfMA_Max = (SET_Limit_HalfMA_i == 'd0)? '{default:1'b1} : ((SET_FrameBits_i + SET_Limit_HalfMA_i) << 1);
  
  
  always_ff @(posedge CLK_i)
  begin
    if(RST_i) 
    begin
      Main_SM             <= MF_IDLE;
      MA_o                <= 1'b0;
      HalfMA_Count        <=  'd0;
      PosEdge_Count       <=  'd0;
      InterFrameGap_Count <=  'd0;
      Extra_TickCDM_Count <=  'd0;
      Limit_TickTO_Count  <=  'd0;
      MA_Launched         <= 1'b0;
      RX_Done_Latch       <= 1'b0;
      FR_Abort_o          <= 1'b0;
      FR_End_o            <= 1'b0;
      WatchdogExpired_o   <= 1'b0;
      FR_Active_o         <= 1'b0;
    end 
    else 
    begin
      FR_Abort_o          <= 1'b0;
      FR_End_o            <= 1'b0;
      WatchdogExpired_o   <= 1'b0;
      
      case(Main_SM)
        MF_IDLE    :
        begin
          MA_o <= SET_Lvl_MA_Idle;
          
          if(FR_Accept)
          begin
            MA_Launched         <= 1'b0;
            PosEdge_Count       <=  'd0;
            HalfMA_Count        <=  'd0;
            InterFrameGap_Count <=  'd0;
            Limit_TickTO_Count  <=  'd0;
            RX_Done_Latch       <= 1'b0;
            
            if(REQ_CDM && CDM_Bit && (SET_Extra_TickCDM_i != 'd0))
            begin
              Extra_TickCDM_Count <= SET_Extra_TickCDM_i;
              Main_SM             <= MF_CDM_DLY;
            end
            else
              Main_SM             <= MF_RUN;
          end
        end
        
        // -------------------------------------------------------------
        // CDM '1' encoding: prolonged MA idle before the frame start,
        // counted on the oversampling tick grid.
        MF_CDM_DLY :
        begin
          MA_o <= SET_Lvl_MA_Idle;
          
          if(Extra_TickCDM_Count != 'd0)
            if(TickPulse_i)
              Extra_TickCDM_Count <= Extra_TickCDM_Count - 1'd1;
          else
            Main_SM             <= MF_RUN;
        end
        
        MF_RUN     :
        begin
          RX_Done_Latch <= RX_Done_Latch | RX_FrameDone_i;
          
          // ---- frame watchdog (oversampling ticks; 0 = infinite) ----
          if((SET_Limit_TickTO_i != 'd0) && (Limit_TickTO_Count >= SET_Limit_TickTO_i))
          begin
            WatchdogExpired_o   <= 1'b1;
            FR_Abort_o          <= 1'b1;
            FR_End_o            <= 1'b1;
            FR_Active_o         <= 1'b0;
            Main_SM             <= MF_GAP;
          end
          else
          begin
            if(TickPulse_i && (SET_Limit_TickTO_i != 'd0)) Limit_TickTO_Count <= Limit_TickTO_Count + 1'd1;
            
            if(HalfPulse_i)
            begin
              if(RX_Done_Latch && (MA_o == SET_Lvl_MA_Idle))
              begin
                // ---- clean stop: MA already at idle level ----
                // (no MA toggle at this boundary; the boundary strobe
                // carries the idle level and is rejected downstream by
                // the sampling-polarity filter)
                FR_End_o    <= 1'b1;
                FR_Active_o <= 1'b0;
                Main_SM     <= MF_GAP;
              end
              else
              begin
                // toggle one MA half-cycle
                MA_o         <= MA_NextLevel;
                HalfMA_Count <= HalfMA_Count + 1'd1;
                
                // BiSS ACK reference bookkeeping: count every rising edge
                // of the frame, the launch edge included (correct for
                // either idle level - see header).
                if(MA_NextLevel && ~SET_InterfaceMode_i)
                  PosEdge_Count <= PosEdge_Count + 1'b1;
                
                if(~MA_Launched)
                begin
                  // ---- frame launch: the very first MA edge ----
                  // frame_launch (combinational) is already asserted this
                  // cycle; frame_active rises with this edge.
                  MA_Launched <= 1'b1;
                  FR_Active_o <= 1'b1;
                end
                else if(HalfMA_Count >= HalfMA_Max)
                begin
                  // ---- margin abort (dead / stuck line) ----
                  FR_Abort_o  <= 1'b1;
                  FR_End_o    <= 1'b1;
                  FR_Active_o <= 1'b0;
                  Main_SM     <= MF_GAP;
                end
              end
            end
          end
        end
        
        // -------------------------------------------------------------
        // Inter-frame gap = BiSS-C "timeout state", counted on the
        // oversampling tick grid (load value converted from system-clock
        // ticks by the top level, preserving absolute-time semantics).
        MF_GAP     :
        begin
          MA_o <= SET_Lvl_MA_Idle;
          
          if(InterFrameGap_Count < SET_InterFrameGap_i)
            if(TickPulse_i)
              InterFrameGap_Count = InterFrameGap_Count + 1'd1;
          else
            Main_SM <= MF_IDLE;
        end
        
        default    : Main_SM <= MF_IDLE;
      endcase
    end
  end
  
endmodule
