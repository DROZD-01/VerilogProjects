`timescale 1ns / 1ps
/***************************************************************************************************
* Company: 
* Engineer: 
* 
* Create Date: 10/14/2026 06:21:14 PM
* Design Name: 
* Module Name: prim_FractionalTickGen
* Project Name: 
* Target Devices: 
* Tool Versions: 
* Description: 
* 
* Dependencies: 
* 
* Revision:
* Revision 0.01 - File Created
* Additional Comments:
* 
* Parameter definition:
*   
*   
*   
* 
* 
***************************************************************************************************/

import prim_FractionalTickGen_pkg::*;

module prim_FractionalTickGen
  #(
    parameter  real FREQ_SYSCLK  = 100.0e6  , // System clock frequency (in Hz)
    parameter  bit  MODE_FIXED   = 1'b0     , // Generation mode:'1' - fixed tick rate, '0' - variable tick rate
    parameter  int  TICKR_FIXED  = 115200   , // Target tickrate (used in fixed mode)
    parameter  int  OVERSAMPLE   = 16       , // Oversampling mode (ticks per bit)
    parameter  int  LOGIC_DELAY  = 0        , // Preload value multiplier
    parameter  int  BV_NUM       = BV_DEF   , // Number of entries in BAUD_VALUES
    parameter  int  BAUD_VAL [0:BV_NUM-1] = BAUD_DEF,
    localparam int  BV_WIDTH     = $clog2(BV_NUM)
  )
  (
    input  logic                RST_i       , // System reset
    input  logic                CLK_i       , // System clock
    input  logic [BV_WIDTH-1:0] BaudSel_i   , // Baudrate selector in variable mode
    input  logic                Resync_i    , // Clear and start counting
    output logic                TickPulse_o   // Tick pulse output
  );
  
  localparam int TICKR_MIN = BAUD_VAL[0]; // Minimum allowed tickrate (used in variable mode)
  
  //------------------------------------------------------------------------------------------
  // Pre-synthesis verification
  //------------------------------------------------------------------------------------------
  initial begin
    if(OVERSAMPLE  == 0) $error("[FractionalTickGen]: Unsupported Oversample value");
    if(TICKR_FIXED == 0) $error("[FractionalTickGen]: Unsupported Fixed-mode Baudrate value");
  end
  
  //------------------------------------------------------------------------------------------
  // Calculation functions
  //------------------------------------------------------------------------------------------
  
  //-----------------------------------------------------------------------------
  // Selector -> Value mapper
  function automatic int BR_ToVal_f (input logic [BV_WIDTH-1:0] BaudSelect_C);
    return (BaudSelect_C < BV_NUM)? BAUD_VAL[BaudSelect_C] : 0; // 0 = unsupported
  endfunction
  
  
  //-----------------------------------------------------------------------------
  // Selects an appropriate counter width for the Fractional Tick Generator
  function int FTGenWidthVar_f (input int  BaudMin_C    ,  // Minimum Baudrate Value
                                input int  BaudFixed_C  ,  // Fixed Baudrate Value
                                input bit  MODE_C       ,  // Fixed / Variable
                                input int  OverSample_C ,  // Oversampling
                                input real SysCLK_C     ); // 
    if(MODE_C)
      return $clog2(int'(((10.5*SysCLK_C)/(real'(BaudFixed_C) * real'(OverSample_C))) / 0.01)); // 
    else
      return $clog2(int'(((10.5*SysCLK_C)/(real'(BaudMin_C  ) * real'(OverSample_C))) / 0.01)); // 
  endfunction
  localparam int FRAC_W = FTGenWidthVar_f(TICKR_MIN, TICKR_FIXED, MODE_FIXED, OVERSAMPLE, FREQ_SYSCLK);
  
  
  //-----------------------------------------------------------------------------
  // Increment ROM generation, step 1
  function automatic int IncrementVal_f (input int  Baudrate_C    ,  // 
                                         input int  OverSample_C  ,  // 
                                         input int  FractWidth_C  ,  // 
                                         input real SysCLK_C      ); // 
    return int'((real'(Baudrate_C) * real'(OverSample_C) * real'(2**FractWidth_C)) / SysCLK_C);
  endfunction
  
  typedef int br_incr_t [0:BV_NUM-1];
  function automatic br_incr_t BR_IncrTable_f (input int  OverSample_C  , 
                                               input int  FractWidth_C  , 
                                               input real SysCLK_C      );
    br_incr_t t;
    for(int i = 0; i < BV_NUM; i++)
      t[i] = IncrementVal_f(BAUD_VAL[i], OverSample_C, FractWidth_C, SysCLK_C);
    return t;
  endfunction
  localparam br_incr_t BR_INCR = BR_IncrTable_f(OVERSAMPLE, FRAC_W, FREQ_SYSCLK);
  
  
  //------------------------------------------------------------------------------------------
  // Increment ROM generation, step 2
  function int FTGenIncrVar_f (input logic [BV_WIDTH-1:0] BaudSelect_C ,  // Current selected Baudrate Code
                               input int                  BaudFixed_C  ,  // Fixed mode Baudrate Code
                               input int                  BaudBackup_C ,  // Backup for out-of-range Baudrate code
                               input br_incr_t            BR_INCR_C    ,  // 
                               input bit                  MODE_C       ,  // Fixed / Variable
                               input int                  OverSample_C ,  // Oversampling
                               input int                  FractWidth_C ,  // 
                               input real                 SysCLK_C     ); // 
      if(MODE_C)
        return IncrementVal_f(BaudFixed_C , OverSample_C, FractWidth_C, SysCLK_C);
      if(BaudSelect_C < BV_NUM)
        return BR_INCR_C[BaudSelect_C];
      return IncrementVal_f(BaudBackup_C, OverSample_C, FractWidth_C, SysCLK_C);
  endfunction
  
  
  //------------------------------------------------------------------------------------------
  // Main Logic
  //------------------------------------------------------------------------------------------
  
  logic [FRAC_W-1:0] Increment  ; // round(baud*OS*2^FRAC_W / f_clk), >= 1
  logic [FRAC_W-1:0] Accumulate ; // 
  logic [FRAC_W  :0] Sum        ; // 
  
  always_comb
  begin
    Increment   = FTGenIncrVar_f(BaudSel_i, TICKR_FIXED, TICKR_MIN, BR_INCR, MODE_FIXED, OVERSAMPLE, FRAC_W, FREQ_SYSCLK);
    Sum         = ({1'b0, Accumulate} + {1'b0, Increment});
  end
  
  always_ff @(posedge CLK_i)
  begin
    if(RST_i)
    begin
      Accumulate  <=  'd0; 
      TickPulse_o <= 1'b0; 
    end
    else
    begin
      Accumulate  <= (Resync_i)? (Increment*(LOGIC_DELAY+1)) : Sum[FRAC_W-1:0];
      TickPulse_o <= Sum[FRAC_W];
    end
  end
  
  
endmodule
