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
    parameter  int MAX_FRAME_BITS = 75  , // 
    
    parameter  bit MODE_FRAMEBIT  = 1'b1, // '0' - Fixed, ; '1' - Variable, 
    parameter  bit MODE_INTFRGAP  = 1'b1, // '0' - Fixed, ; '1' - Variable, 
    parameter  bit MODE_LIMHALFM  = 1'b1, // '0' - Fixed, ; '1' - Variable, 
    parameter  bit MODE_LIMTTIMO  = 1'b1, // '0' - Fixed, ; '1' - Variable, 
    parameter  bit MODE_EXTRTCDM  = 1'b1, // '0' - Fixed, ; '1' - Variable, 
    
    parameter  int FVAL_FRAMEBIT  = 26  , // 
    parameter  int FVAL_INTFRGAP  =  8  , // 
    parameter  int FVAL_LIMHALFM  =  8  , // 
    parameter  int FVAL_LIMTTIMO  =  8  , // 
    parameter  int FVAL_EXTRTCDM  =  8    // 
    
    localparam int WIDTH_FRAMEBIT = $clog2(MAX_FRAME_BITS+1), // 
    parameter  int WIDTH_INTFRGAP =  8  , // 
    parameter  int WIDTH_LIMHALFM =  8  , // 
    parameter  int WIDTH_LIMTTIMO =  8  , // 
    parameter  int WIDTH_EXTRTCDM =  8    // 
  )
  (
    input  logic                        RST_i               , // 
    input  logic                        CLK_i               , // 
    // Controls
    input  logic                        SET_InterfaceMode_i , // '1' - SSI, '0' - BiSS
    input  logic [                -1:0] SET_FrameBits_i     , // Frame length [bits]
    input  logic [                -1:0] SET_InterFrameGap_i , // inter-frame gap [ovs ticks]
    input  logic [                -1:0] SET_Limit_HalfMA_i  , // extra MA cycles before TX abort
    input  logic [                -1:0] SET_Limit_TickTO_i  , // frame watchdog [ovs ticks], 0 = infinite
    input  logic [                -1:0] SET_Extra_TickCDM_i , // CDM '1' start extension [ovs ticks]
    
    
  );
endmodule
