`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/01/2026 10:52:56 AM
// Design Name: 
// Module Name: prim_OneHot_Conv
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

module prim_OneHot_Conv 
  #(
    parameter int unsigned BIN_W = 5        , // binary input width (derived, normally leave default)
    parameter int unsigned OUT_W = 2**BIN_W   // one-hot output width
    
  )
  (
    input  logic [BIN_W-1:0] Binary_i, // normal (binary-encoded) input, valid range 0 .. OUT_W-1
    output logic [OUT_W-1:0] OneHot_o  // one-hot output: onehot_out[bin_in] = 1'b1
  );
  
    always_comb 
    begin
      OneHot_o = '0;                       // default: no bit hot -> covers out-of-range bin_in
      for(int i = 0; i < OUT_W; i++) 
      begin  // synthesizes to a plain AND-based decoder
        OneHot_o[i] = (Binary_i == i);
      end
    end
  
endmodule