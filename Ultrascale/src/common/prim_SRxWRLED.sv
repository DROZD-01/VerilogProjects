`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/29/2026 11:43:40 PM
// Design Name: 
// Module Name: prim_SRxWRLED
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


module prim_SRxWRLED
  #(
    parameter int DATA_W  = 8    , // Shift register length
    parameter bit RST_POL = 1'b0 , // Default value of shift register
    parameter bit CLK_POL = 1'b0   // '0' - posedge, '1' - negedge
  )
  (
    input  logic              C     , // Clock
    input  logic              R     , // Reset
    input  logic              CE    , // Clock Enable
    input  logic              L     , // Load Enable
    input  logic              SLI   , // Shift-left  IN
    input  logic              SRI   , // Shift-right IN
    input  logic              LEFT  , // Left('1', Qx <- Q0) / Right('0', Qx -> Q0)
    input  logic [DATA_W-1:0] M     , // Parallel Mask Input (one-hot MSB-end cell of window [top:0])
    input  logic [DATA_W-1:0] D     , // Parallel Input
    output logic [DATA_W-1:0] Q     , // Parallel Output
    output logic              SLO   , // Shift-left  OUT
    output logic              SRO     // Shift-right OUT
  );
  
  logic L_OR_CE ;
  
  logic [DATA_W-1:0] MDLD; // Load / Shift
  logic [DATA_W-1:0] MDL ; // Left shift
  logic [DATA_W-1:0] MDR ; // Right shift
  logic [DATA_W-1:0] MDLR; // Left shift / Right shift
  logic [DATA_W-1:0] Q_b;
  
  always_comb
  begin
    L_OR_CE = CE || L;
    
    MDL  = {Q_b[DATA_W-2:0], SLI};
    MDR  = ({Q_b[DATA_W-1], Q_b[DATA_W-1:1]} & ~M) | ({DATA_W{SRI}} & M);
    
    MDLR = (LEFT)? MDL : MDR ;
    MDLD = (L   )? D   : MDLR;
    
    Q    = Q_b;
    SLO  = |(Q_b & M);
    SRO  = Q_b[0];
  end
  
  generate
  begin
    if(CLK_POL == 1'b1)
    begin
      always_ff @(negedge C)
      begin
        if(R)
          Q_b <= '{default:RST_POL};
        else if(L_OR_CE)
          Q_b <= MDLD;
      end
    end
    else if(CLK_POL == 1'b0)
    begin
      always_ff @(posedge C)
      begin
        if(R)
          Q_b <= '{default:RST_POL};
        else if(L_OR_CE)
          Q_b <= MDLD;
      end
    end
  end
  endgenerate
  
endmodule
