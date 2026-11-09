`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/03/2026 10:52:13 PM
// Design Name: 
// Module Name: prim_RST_Sync
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

module prim_RST_Sync
  #(
    parameter int SYNC_STAGES   = 4     , // 
    parameter bit EN_EXTERNAL   = 1'b0  , // Enable external asynchronous reset signal input
    parameter bit POL_EXTERNAL  = 1'b1  , // 1'b1 - Active High, 1'b0 - Active Low
    parameter bit POL_LOCKED    = 1'b0  , // 1'b1 - Ready High,  1'b0  - Ready Low
    parameter bit POL_SYNCHED   = 1'b1    // 1'b1 - Active High, 1'b0 - Active Low
  )
  (
    input  wire  Gen_CLK_i    , // Clock signal  (from a PLL/MMCM)
    input  wire  Gen_Locked_i , // LOCKED signal (from a PLL/MMCM)
    input  wire  Ext_RST_i    , // Asynchronous reset signal (External)
    output logic Sync_RST_o     // Synchronous reset signal
  );
  
  initial begin
    if(SYNC_STAGES < 2) $error("[prim_RST_Sync] Sync stages number must be >= 2");
  end
  
  reg [SYNC_STAGES-1:0] ResetSync;
  
  wire LockedAdj;
  assign LockedAdj = (POL_LOCKED)? Gen_Locked_i : ~Gen_Locked_i;
  
  generate
    if(EN_EXTERNAL == 1'b1)
    begin
      if(POL_EXTERNAL == 1'b1)
      begin
        
        always_ff @(posedge Gen_CLK_i or posedge Ext_RST_i) 
        begin
          if(Ext_RST_i)
            ResetSync <= '{default:0};
          else
            ResetSync <= {ResetSync[SYNC_STAGES-2:0], LockedAdj};
        end
        
        always_ff @(posedge Gen_CLK_i) 
        begin
          Sync_RST_o <= (POL_SYNCHED)? ~&ResetSync : &ResetSync;
        end
        
      end
      else
      begin
        
        always_ff @(posedge Gen_CLK_i or negedge Ext_RST_i) 
        begin
          if(~Ext_RST_i)
            ResetSync <= '{default:0};
          else
            ResetSync <= {ResetSync[SYNC_STAGES-2:0], LockedAdj};
        end
        
        always_ff @(posedge Gen_CLK_i) 
        begin
          Sync_RST_o <= (POL_SYNCHED)? ~&ResetSync : &ResetSync;
        end
        
      end
    end
    else
    begin
      
      always_ff @(posedge Gen_CLK_i) 
      begin
        ResetSync  <= {ResetSync[SYNC_STAGES-2:0], LockedAdj};
        Sync_RST_o <= (POL_SYNCHED)? ~&ResetSync : &ResetSync;
      end
      
    end
  endgenerate
  
  
endmodule