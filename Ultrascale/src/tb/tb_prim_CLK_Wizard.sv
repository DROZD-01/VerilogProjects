`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/19/2026 03:20:41 PM
// Design Name: 
// Module Name: tb_prim_CLK_Wizard
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

module tb_prim_CLK_Wizard;
  
  import prim_CLK_Wizard_pkg::*;
  
  logic        TB_CLKIN_SEL     ; // 
  logic        TB_CLKIN_1       ; // 
  logic        TB_CLKIN_1_P     ; // 
  logic        TB_CLKIN_1_N     ; // 
  logic        TB_CLKIN_2       ; // 
  logic        TB_CLKIN_2_P     ; // 
  logic        TB_CLKIN_2_N     ; // 
  logic        TB_CLKOUT_1      ; // 
  logic        TB_CLKOUT_2      ; // 
  logic        TB_CLKOUT_3      ; // 
  logic        TB_CLKOUT_4      ; // 
  logic        TB_CLKOUT_5      ; // 
  logic        TB_CLKOUT_6      ; // 
  logic        TB_CLKOUT_7      ; // 
  logic        TB_CLKOUT_1_CE   ; // 
  logic        TB_CLKOUT_2_CE   ; // 
  logic        TB_CLKOUT_3_CE   ; // 
  logic        TB_CLKOUT_4_CE   ; // 
  logic        TB_CLKOUT_5_CE   ; // 
  logic        TB_CLKOUT_6_CE   ; // 
  logic        TB_CLKOUT_7_CE   ; // 
  logic        TB_CLKOUTB_1     ; // 
  logic        TB_CLKOUTB_2     ; // 
  logic        TB_CLKOUTB_3     ; // 
  logic        TB_CLKOUTB_4     ; // 
  logic        TB_CLKOUTB_1_CE  ; // 
  logic        TB_CLKOUTB_2_CE  ; // 
  logic        TB_CLKOUTB_3_CE  ; // 
  logic        TB_CLKOUTB_4_CE  ; // 
  logic        TB_CLKFB_IN      ; // 
  logic        TB_CLKFB_IN_P    ; // 
  logic        TB_CLKFB_IN_N    ; // 
  logic        TB_CLKFB_OUT     ; // 
  logic        TB_CLKFB_OUT_P   ; // 
  logic        TB_CLKFB_OUT_N   ; // 
  logic        TB_RESET         ; // 
  logic        TB_RESETn        ; // 
  logic        TB_POWER_DOWN    ; // 
  logic        TB_LOCKED        ; // 
  logic        TB_REF_CLK       ; // 
  logic        TB_USER_CLK_0    ; // 
  logic        TB_USER_CLK_1    ; // 
  logic        TB_USER_CLK_2    ; // 
  logic        TB_USER_CLK_3    ; // 
  logic [ 3:0] TB_CLK_STOP      ; // 
  logic [ 3:0] TB_CLK_GLITCH    ; // 
  logic [ 3:0] TB_CLK_OOR       ; // 
  logic        TB_PSCLK         ; // 
  logic        TB_PSEN          ; // 
  logic        TB_PSINCDEC      ; // 
  logic        TB_PSDONE        ; // 
  logic        TB_CDDC_REQ      ; // 
  logic        TB_CDDC_DONE     ; // 
  logic        TB_STOPPED_CLKIN ; // 
  logic        TB_STOPPED_CLKFB ; // 
  
  
  prim_CLK_Wizard
  #(
    .FPGA_FAMILY     ("SERIES7"                                   ),
    .FPGA_SERIES     ("ARTIX"                                     ),
    .MODE_CLKIN      (ICB0                                        ),
    .EN_CLKIN_X      (2'b01                                       ),
    .EN_CLKOUT_X     (7'b0000111                                  ),
    .EN_CLKOUTB_X    (4'b0000                                     ),
    .MODE_CLKOUT_X   ('{OCB0, OCB0, OCB0, OCB0, OCB0, OCB0, OCB0} ),
    .MODE_CLKFB      (FCB0                                        ),
    .EN_CLKFB_DIFF   (1'b0                                        ),
    .EN_RESET        (1'b1                                        ),
    .POL_RESET       (1'b0                                        ),
    .EN_SAFECLKSTART (1'b1                                        ),
    .EN_SEQUENCER    (1'b1                                        ),
    .MODE_SEQUENCER  ('{3'd1, 3'd1, 3'd1, 3'd1, 3'd3, 3'd2, 3'd1} ),
    .EN_CLKMONITOR   (1'b0                                        ),
    .EN_DYNRECONF    (1'b0                                        ),
    .MODE_DYNRECONF  (1'b0                                        ) 
  )
  UUT
  (
    .CLKIN_SEL     (TB_CLKIN_SEL     ), // 
    .CLKIN_1       (TB_CLKIN_1       ), // 
    .CLKIN_1_P     (TB_CLKIN_1_P     ), // 
    .CLKIN_1_N     (TB_CLKIN_1_N     ), // 
    .CLKIN_2       (TB_CLKIN_2       ), // 
    .CLKIN_2_P     (TB_CLKIN_2_P     ), // 
    .CLKIN_2_N     (TB_CLKIN_2_N     ), // 
    .CLKOUT_1      (TB_CLKOUT_1      ), // 
    .CLKOUT_2      (TB_CLKOUT_2      ), // 
    .CLKOUT_3      (TB_CLKOUT_3      ), // 
    .CLKOUT_4      (TB_CLKOUT_4      ), // 
    .CLKOUT_5      (TB_CLKOUT_5      ), // 
    .CLKOUT_6      (TB_CLKOUT_6      ), // 
    .CLKOUT_7      (TB_CLKOUT_7      ), // 
    .CLKOUT_1_CE   (TB_CLKOUT_1_CE   ), // 
    .CLKOUT_2_CE   (TB_CLKOUT_2_CE   ), // 
    .CLKOUT_3_CE   (TB_CLKOUT_3_CE   ), // 
    .CLKOUT_4_CE   (TB_CLKOUT_4_CE   ), // 
    .CLKOUT_5_CE   (TB_CLKOUT_5_CE   ), // 
    .CLKOUT_6_CE   (TB_CLKOUT_6_CE   ), // 
    .CLKOUT_7_CE   (TB_CLKOUT_7_CE   ), // 
    .CLKOUTB_1     (TB_CLKOUTB_1     ), // 
    .CLKOUTB_2     (TB_CLKOUTB_2     ), // 
    .CLKOUTB_3     (TB_CLKOUTB_3     ), // 
    .CLKOUTB_4     (TB_CLKOUTB_4     ), // 
    .CLKOUTB_1_CE  (TB_CLKOUTB_1_CE  ), // 
    .CLKOUTB_2_CE  (TB_CLKOUTB_2_CE  ), // 
    .CLKOUTB_3_CE  (TB_CLKOUTB_3_CE  ), // 
    .CLKOUTB_4_CE  (TB_CLKOUTB_4_CE  ), // 
    .CLKFB_IN      (TB_CLKFB_IN      ), // 
    .CLKFB_IN_P    (TB_CLKFB_IN_P    ), // 
    .CLKFB_IN_N    (TB_CLKFB_IN_N    ), // 
    .CLKFB_OUT     (TB_CLKFB_OUT     ), // 
    .CLKFB_OUT_P   (TB_CLKFB_OUT_P   ), // 
    .CLKFB_OUT_N   (TB_CLKFB_OUT_N   ), // 
    .RESET         (TB_RESET         ), // 
    .RESETn        (TB_RESETn        ), // 
    .POWER_DOWN    (TB_POWER_DOWN    ), // 
    .LOCKED        (TB_LOCKED        ), // 
    .REF_CLK       (TB_REF_CLK       ), // 
    .USER_CLK_0    (TB_USER_CLK_0    ), // 
    .USER_CLK_1    (TB_USER_CLK_1    ), // 
    .USER_CLK_2    (TB_USER_CLK_2    ), // 
    .USER_CLK_3    (TB_USER_CLK_3    ), // 
    .CLK_STOP      (TB_CLK_STOP      ), // 
    .CLK_GLITCH    (TB_CLK_GLITCH    ), // 
    .CLK_OOR       (TB_CLK_OOR       ), // 
    .PSCLK         (TB_PSCLK         ), // 
    .PSEN          (TB_PSEN          ), // 
    .PSINCDEC      (TB_PSINCDEC      ), // 
    .PSDONE        (TB_PSDONE        ), // 
    .CDDC_REQ      (TB_CDDC_REQ      ), // 
    .CDDC_DONE     (TB_CDDC_DONE     ), // 
    .STOPPED_CLKIN (TB_STOPPED_CLKIN ), // 
    .STOPPED_CLKFB (TB_STOPPED_CLKFB )  // 
  );
  
  
  
  
  
  
  
  
  
  
  
  
endmodule
