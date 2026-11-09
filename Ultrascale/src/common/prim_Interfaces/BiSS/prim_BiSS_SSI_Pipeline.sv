`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/05/2026 10:47:32 PM
// Design Name: 
// Module Name: prim_BiSS_SSI_Pipeline
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


module prim_BiSS_SSI_Pipeline
  #(
    parameter  int RAM_LOG2   = 4  , // History depth = 2^RAM_LOG2 half-period strobes.
    parameter  int OVERSAMPLE = 16 , // Oversampling mode (ticks per bit)
    localparam int TAP_W      = RAM_LOG2 + $clog2(OVERSAMPLE/2)
  )
  (
    input  logic             RST_i        , // System reset
    input  logic             CLK_i        , // System clock
    input  logic             TickPulse_i  , // Tick pulse input
    input  logic [TAP_W-1:0] TickDelay_i  , // Line delay in tick pulses
    input  logic             MA_Edge_i    , // Strobe at every MA edge
    input  logic             MA_Level_i   , // MA level after that edge
    output logic             MA_Edge_o    , // Strobe, delayed by 'TickDelay_i'
    output logic             MA_Level_o     // MA level, same delay
  );
  
  localparam int OVSMP_HW = $clog2(OVERSAMPLE/2);
  localparam int DEPTH    = 2**RAM_LOG2;
  
  initial begin
    if(RAM_LOG2 <= 1 || RAM_LOG2 >= 16) $error("[prim_BiSS_SSI_Pipeline]: RAM_LOG2 out of range");
  end
  
  
  // -------------------------------------------------------------------
  // Tap split: tap = (OVERSAMPLE/2)*Tick_HalfP + Tick_Fract
  // -------------------------------------------------------------------
  logic [RAM_LOG2-1:0] Tick_HalfP; // half-period (strobe) part of the delay
  logic [OVSMP_HW-1:0] Tick_Fract; // fractional tick part (0..15)
  
  assign Tick_HalfP = TickDelay_i[   TAP_W-1:OVSMP_HW];
  assign Tick_Fract = TickDelay_i[OVSMP_HW-1:0       ];
  
  
  // -------------------------------------------------------------------
  // Mod-16 tick Phase counter, re-aligned by every MA_Edge_i.
  // During tick k of an edge group (k=0 is the MA_Edge_i tick itself),
  // Phase reads k-1.
  // -------------------------------------------------------------------
  logic [OVSMP_HW-1:0] Phase;
  
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i)
      Phase <= 'd0;
    else
    begin
      if(MA_Edge_i)
        Phase <= 'd0;
      else if(TickPulse_i) 
        Phase <= Phase + 1'd1;
    end
  end
  
  
  // -------------------------------------------------------------------
  // MA_Edge_o: the input edge strobe delayed by Tick_Fract ticks.
  //  Tick_Fract == 0 : fires on the edge tick itself 
  //  Tick_Fract >= 1 : fires on the Tick_Fract-th tick after the edge.
  // -------------------------------------------------------------------
  logic TickPulse;
  
  assign TickPulse = TickPulse_i && !MA_Edge_i && ((Phase + 1'd1) == Tick_Fract);
  
  always_ff @(posedge CLK_i) 
  begin
    MA_Edge_o <= (Tick_Fract == 4'd0) ? MA_Edge_i : TickPulse;
  end
  
  // -------------------------------------------------------------------
  // Level history ring: 1 bit x DEPTH, written with the level of every
  // MA_Edge_i strobe. 
  // -------------------------------------------------------------------
  (* ram_style = "distributed" *) logic HMem [DEPTH];
  logic [RAM_LOG2-1:0] WRAddr; // next write address (increments per strobe)
  
  always_ff @(posedge CLK_i) 
  begin
    if(MA_Edge_i) HMem[WRAddr] <= MA_Level_i; // (warm up after DEPTH strobes)
  end
  
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i)
      WRAddr <= 'd0;
    else if(MA_Edge_i)
      WRAddr <= WRAddr + 1'd1;
  end
  
  
  // -------------------------------------------------------------------
  // Read address: slot of the strobe Tick_HalfP half-periods back. The -(!MA_Edge_i)
  // term compensates WRAddr's increment at the edge cycle itself
  // -------------------------------------------------------------------
  logic [RAM_LOG2-1:0] RDAddr   ;
  logic [RAM_LOG2-1:0] Decrement;
  
  assign Decrement = MA_Edge_i ? 'd0 : 'd1;
  assign RDAddr    = WRAddr - Tick_HalfP - Decrement;
  
  
  // -------------------------------------------------------------------
  // Most recent strobe level (for the q==0, r>=1 bypass path).
  // -------------------------------------------------------------------
  logic Level_Hold;
  
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i)
      Level_Hold <= 1'b0;
    else if(MA_Edge_i)
      Level_Hold <= MA_Level_i;
  end
  
  // -------------------------------------------------------------------
  // MA_Level_o: the level of the strobe Tick_HalfP half-periods back.
  //  Tick_HalfP == 0, Tick_Fract == 0 : full combinational bypass 
  //  Tick_HalfP == 0, Tick_Fract >= 1 : level latched at the last MA_Edge_i
  //  Tick_HalfP >= 1                  : ring read 
  // -------------------------------------------------------------------
  always_ff @(posedge CLK_i) 
  begin
    case({(Tick_HalfP == 'd0), (Tick_Fract == 'd0)})
      2'b10  : MA_Level_o <= MA_Level_i  ;
      2'b11  : MA_Level_o <= Level_Hold  ;
      default: MA_Level_o <= HMem[RDAddr];
    endcase
  end
  
endmodule
