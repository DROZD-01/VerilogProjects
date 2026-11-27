`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/26/2026 09:25:17 PM
// Design Name: 
// Module Name: prim_SafeReg
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

module prim_SafeReg
  #(
    parameter  int                ADDR_W   = 5             , // Address port width
    parameter  int                DATA_W   = 32            , // Data I/O port width
    parameter  int                SLICE_W  = 8             , // Slice width
    parameter  logic [ADDR_W-1:0] ADDR_VAL = 5'h00         , // Register address in the SPI map
    localparam int                DATA_SW  = DATA_W/SLICE_W
  )
  (
    input  logic               CLK_i  , // 
    input  logic               RST_i  , // 
    input  logic               WE_i   , // Write Enable
    input  logic               RE_i   , // Read Enable
    input  logic [ ADDR_W-1:0] AD_i   , // Address
    input  logic [DATA_SW-1:0] SE_i   , // Slice Enable
    input  logic [ DATA_W-1:0] D_i    , // Data input
    output logic [ DATA_W-1:0] Q_o      // Data output
  );
  
  initial begin
    if(DATA_W % DATA_SW != 0) $error("[prim_SafeReg]: Slice write access enabled, but DATA_W is not a multiple of DATA_SW");
    if(DATA_SW <= 1         ) $error("[prim_SafeReg]: SLICE_W cannot be equal to DATA_W");
  end
  
  //--------------------------------------------------------------------------
  // PORT_B capture with pair-read protection (B_DATA / PREBUF)
  //--------------------------------------------------------------------------
  logic               Q_Locked    ; // Q_o pair read in flight
  logic               Q_Unlock    ; // 
  logic               Q_Protected ; // Q_o is protected from being overwritten while reading is in progress
  logic               D_Buff_Full ; // D_Buff holds a word to promote
  logic [DATA_SW-1:0] SE_RE       ; // 
  logic [DATA_SW-1:0] SE_RE_Latch ; // 
  logic [ DATA_W-1:0] D_Buff      ; // holds B words arriving during a pair read
  
  always_comb
  begin
    for(int i = 0; i < DATA_SW; i = i + 1)
    begin
      SE_RE[i] = SE_i[i] && RE_i && (AD_i == ADDR_VAL);
    end
    Q_Locked    = |{SE_RE_Latch};
    Q_Unlock    = &{SE_RE_Latch};
    Q_Protected = (Q_Locked && !Q_Unlock) || RE_i;
  end
  
  always_ff @(posedge CLK_i)
  begin
    if(RST_i || Q_Unlock) 
    begin
      SE_RE_Latch <=   'h0;
    end
    else
    begin
      for(int i = 0; i < DATA_SW; i = i + 1)
      begin
        if(SE_RE[i]) SE_RE_Latch[i] <= 1'b1;
      end
    end
  end
  
  always_ff @(posedge CLK_i)
  begin
    if(RST_i) 
    begin
        Q_o         <=   'd0;
        D_Buff      <=   'd0;
        D_Buff_Full <=  1'b0;
    end 
    else
    begin
      if(D_Buff_Full && ~Q_Protected)
      begin
        Q_o         <= D_Buff;
        D_Buff_Full <= 1'b0;
      end
      
      if(WE_i) 
      begin
        D_Buff      <= D_i;
        D_Buff_Full <= 1'b1;
      end
    end
  end

endmodule