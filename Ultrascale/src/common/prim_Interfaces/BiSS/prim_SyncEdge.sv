`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/26/2026 
// Design Name: 
// Module Name: prim_SyncEdge
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Generic 2-FF signal synchronizer with combinational edge strobes.
//              Frequently used construct, kept as a separate primitive so that it
//              can be reused wherever an asynchronous single-bit input (BiSS MA,
//              SSI CLK, push button, ...) enters a clock domain.
//              SIG_o settles 2 CLK after the asynchronous edge; the edge strobes
//              (Rise_o/Fall_o/Any_o) are combinational functions of the two sync
//              registers only, so they are glitch-free and valid for exactly one
//              CLK period. Registers clocked with an edge strobe as clock-enable
//              capture the event 3 CLK after the physical edge.
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module prim_SyncEdge
  #(
    parameter bit  RESET_VAL = 1'b0     // Synchronized level after reset
  )
  (
    input  logic RST_i     , // Synchronous active-high reset
    input  logic CLK_i     , // System clock
    input  logic SIG_i     , // Asynchronous (or foreign-clock) input signal
    output logic SIG_o     , // 2-FF-synchronized level
    output logic Rise_o    , // Combinational rising-edge strobe
    output logic Fall_o    , // Combinational falling-edge strobe
    output logic Any_o       // Combinational any-edge strobe
  );
  
  //-----------------------------------------------------------------------------
  // 2-FF synchronizer
  //-----------------------------------------------------------------------------
  
  (* ASYNC_REG = "TRUE" *) logic [1:0] Sync;
  
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i) 
      Sync <= {2{RESET_VAL}};
    else
      Sync <= {Sync[0], SIG_i};
  end
  
  //-----------------------------------------------------------------------------
  // Edge strobes (glitch-free: both operands are registers). Sync[0] holds
  // the newest sample and Sync[1] the previous one, so a strobe fires in
  // the cycle right after the first synchronizer stage captured the
  // transition (one CLK after the physical edge, valid for one CLK).
  //-----------------------------------------------------------------------------
  
  assign SIG_o  = Sync[1];
  assign Rise_o =  Sync[0] & ~Sync[1];
  assign Fall_o = ~Sync[0] &  Sync[1];
  assign Any_o  =  Sync[0] ^  Sync[1];
  
endmodule
