`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/04/2026 02:52:52 PM
// Design Name: 
// Module Name: prim_BiSS_SSI_LineDelay
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


module prim_BiSS_SSI_LineDelay
  #(
    parameter  int TIMEOUT_MA = 4  , // Timeout value (full MA cycle periods)
    parameter  int RAM_LOG2   = 4  , // History depth = 2^RAM_LOG2 half-period strobes.
    parameter  int OVERSAMPLE = 16 , // Oversampling mode (ticks per bit)
    localparam int TAP_W      = RAM_LOG2 + $clog2(OVERSAMPLE/2)
  )
  (
    input  logic             RST_i             , // System reset
    input  logic             CLK_i             , // System clock
    input  logic             TickPulse_i       , // Tick pulse input
    input  logic             InterfaceMode_i   , // 1'b1 - SSI, 1'b0 - BiSS
    input  logic             FrameStart_i      , // frame launched (auto mode)
    input  logic             FrameActive_i     , // MA clocking in progress
    input  logic             MA_RefEdge_i      , // Reference edge pulse (BiSS - 2nd rising MA edge; SSI - 1st MA edge)
    input  logic             SL_Synched_i      , // 
    output logic             Meas_Error_o      , // 
    output logic             Meas_Done_o       , // 
    output logic [TAP_W-1:0] TickDelay_o         // Line delay in tick pulses
  );
  
  // +16 ticks of slack: covers the ~SYNC_FF-clock synchronizer latency
  // (< 4 ticks at any feasible sys/tick ratio) and the 1-tick
  // detection quantization, rounded up to half a MA period.
  localparam int LM_TO_TICKS = (TIMEOUT_MA * OVERSAMPLE) + OVERSAMPLE/2;
  localparam int CNT_W       = $clog2(LM_TO_TICKS + 1);
  // First tick count the delay pipe cannot represent (16 * 2^RAM_LOG2).
  // For RAM_LOG2 >= 4 this is above the 8-bit counter range, so the
  // compare folds to constant-false: zero logic in normal builds.
  localparam int LM_MAX_TAP  = 2**TAP_W;
  
  initial begin
    if(RAM_LOG2 <= 1 || RAM_LOG2 >= 16) $error("[prim_BiSS_SSI_LineDelay]: RAM_LOG2 out of range");
  end
  
  // Line-delay measurement
  typedef enum logic [1:0] {
    LM_IDLE = 2'd0,
    LM_ARM  = 2'd1, // armed: wait for the reference MA edge
    LM_WAIT = 2'd2  // counting sys ticks: wait for the SL edge
  } lm_state_e;
  lm_state_e Main_SM;
  
  logic [CNT_W-1:0] TickCount   ; // 
  logic             SL_Buff     ; // 
  logic             SL_Negedge  ; // 
  logic             SL_Edge     ; // 
  logic             Edge_Sel    ; // 
  logic             Tap_Timeout ; // 
  logic             Tap_Overrun ; // 
  
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i)
      SL_Buff <= 'd0;
    else
      SL_Buff <= SL_Synched_i;
  end
  
  always_comb
  begin
    SL_Negedge  = (SL_Buff & ~SL_Synched_i);
    SL_Edge     = (SL_Buff != SL_Synched_i);
    Edge_Sel    = (InterfaceMode_i) ? SL_Edge : SL_Negedge;
    Tap_Timeout = (TickCount >= CNT_W'(LM_TO_TICKS));
    Tap_Overrun = (TickCount >=       (LM_MAX_TAP ));
  end
  
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i)
    begin
      Main_SM      <= LM_IDLE;
      TickCount     <=  'd0;
      Meas_Error_o <= 1'b0;
      Meas_Done_o  <= 1'b0;
      TickDelay_o   <=  'd0;
    end
    else
    begin
      Meas_Error_o <= 1'b0;
      Meas_Done_o  <= 1'b0;
      
      case(Main_SM)
        LM_IDLE :
        begin
          if(FrameStart_i && MA_RefEdge_i)
          begin
            TickCount <= 1'd1;
            Main_SM  <= LM_WAIT;
          end
          else if(FrameStart_i)
          begin
            TickCount <= 1'd0;
            Main_SM  <= LM_ARM;
          end
        end
        
        LM_ARM  :
        begin
          if(MA_RefEdge_i)
          begin
            TickCount <= 1'd1;
            Main_SM  <= LM_WAIT;
          end
          else if(!FrameActive_i)
          begin
            Main_SM  <= LM_IDLE;
          end
        end
        
        LM_WAIT :
        begin
          if(TickPulse_i)
            TickCount <= TickCount + 1'd1;
          
          if(Edge_Sel) 
          begin
            if(Tap_Overrun) 
              Meas_Error_o <= 1'b1;         // beyond pipe range: keep old tap
            else 
            begin
              TickDelay_o  <= TAP_W'(TickCount);
              Meas_Done_o <= 1'b1;
            end
            
            Main_SM <= LM_IDLE;
          end 
          else if (Tap_Timeout || !FrameActive_i) 
          begin
            Meas_Error_o <= 1'b1;           // keep previous tap (top decides)
            Main_SM      <= LM_IDLE;
          end
        end
        
        default: Main_SM <= LM_IDLE;
      endcase
    end
  end
  
endmodule
