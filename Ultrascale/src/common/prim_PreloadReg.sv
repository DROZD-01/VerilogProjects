`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/25/2026 08:28:26 PM
// Design Name: 
// Module Name: prim_PreloadReg
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

module prim_PreloadReg
  #(
    parameter  int                ADDR_W    = 5             , // Address port width
    parameter  int                DATA_W    = 32            , // Data I/O port width
    parameter  int                SLICE_W   = 8             , // Slice width
    parameter  logic [ADDR_W-1:0] ADDR_VAL  = 5'h00         , // Register address in the SPI map
    parameter  logic [DATA_W-1:0] DEF_VAL   = 32'h0F0F_ABAB , // Power-on default
    parameter  bit                SLICE_EN  = 1'b1          , // Enable byte-wide write access
    parameter  bit                LOAD_EN   = 1'b1          , // Enable register preloading
    
    localparam int                DATA_SW   = DATA_W/SLICE_W
  )
  (
    input  logic               CLK_i    , // 
    input  logic               RST_i    , // 
    input  logic               LD_i     , // Load Enable
    input  logic               WE_i     , // Write Enable
    input  logic [ ADDR_W-1:0] AD_i     , // Address
    input  logic [DATA_SW-1:0] SE_i     , // Slice Enable
    input  logic [ DATA_W-1:0] D_i      , // Data input
    output logic [ DATA_W-1:0] Q_o        // Data output
  );
  
  initial begin
    if(SLICE_EN && (DATA_W % DATA_SW != 0)) $error("[prim_PreloadReg]: Slice write access enabled, but DATA_W is not a multiple of DATA_SW");
  end
  
  always_ff @(posedge CLK_i)
  begin
    if(RST_i)
      Q_o <= '{default:0}; 
    else if(LD_i && LOAD_EN)
      Q_o <= DATA_W'(DEF_VAL);
    else if(WE_i && (AD_i == ADDR_VAL))    // normal SPI write
    begin
      if(SLICE_EN)
      begin
        for(int i = 0; i < DATA_SW; i = i + 1)
        begin
          if(SE_i[i]) Q_o[i*8+:8] <= D_i[i*8+:8];
        end
      end
      else
        Q_o <= D_i;
    end
  end

endmodule