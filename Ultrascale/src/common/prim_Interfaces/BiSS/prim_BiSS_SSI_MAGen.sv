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
    parameter int unsigned MAX_FRAME_BITS = 75
  )
  (
    input  logic                        RST_i           , // 
    input  logic                        CLK_i           , // 
    
    input  logic                        InterfaceMode_i , // '1' - SSI, '0' - BiSS
    // Loaded controls
    input  logic                        
    input  logic                        
    input  logic                        
    input  logic                        
    
  );
endmodule
