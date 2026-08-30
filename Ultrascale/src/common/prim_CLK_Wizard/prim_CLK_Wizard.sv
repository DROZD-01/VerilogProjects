`timescale 1ns / 1ps
/***************************************************************************************************
* Company: 
* Engineer: 
* 
* Create Date: 08/25/2026 06:21:14 PM
* Design Name: 
* Module Name: prim_CLK_Wizard
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
*   MODE_CLKIN      :: Input clock buffer types
*     '0' - Single ended clock capable pin
*     '1' - Differential clock capable pin
*     '2' - Global buffer
*     '3' - No buffer
*   EN_CLKIN_X      :: Enable use of CLKIN 2-1 clock input
*   EN_CLKOUT_X     :: Enable clock outputs 7-1
*   EN_CLKOUTB_X    :: Enable inverted clock outputs 4-1
*   MODE_CLKOUT_X   :: Output clock buffer types
*     '0' - Buffer
*     '1' - Buffer with CE
*     '2' - BUFG
*     '3' - BUFGCE
*     '4' - BUFGCE DIV
*     '5' - No buffer
*   MODE_CLKFB      :: Feedback clock connection types
*     '0' - Automatic control On-Chip
*     '1' - Automatic control Off-Chip
*     '2' - User-controlled On-Chip
*     '3' - User-controlled Off-Chip
*   EN_CLKFB_DIFF   :: Enable CLKFB differential signaling
*   EN_RESET        :: Enable RESET port
*   POL_RESET       :: Select RESET port polarity
*     '0' - RESET
*     '1' - RESETn
*   EN_SAFECLKSTART :: Enable Safe clock startup
*   EN_CLKMONITOR   :: Enable clock monitoring
*   EN_DYNRECONF    :: Enable dynamic reconfig 
*   MODE_DYNRECONF  :: Dynamic reconfig interface
*     '0' - AXI4Lite 
*     '1' - DRP
*   
*   
*   
*   
* 
* 
***************************************************************************************************/

import prim_CLK_Wizard_pkg::*;

module prim_CLK_Wizard
  #(
    parameter             FPGA_FAMILY     = "SERIES7"                                   ,
    parameter             FPGA_SERIES     = "ARTIX"                                     ,
    parameter icb_e       MODE_CLKIN      = ICB0                                        ,
    parameter bit   [2:1] EN_CLKIN_X      = 2'b11                                       ,
    parameter bit   [7:1] EN_CLKOUT_X     = 7'b0001111                                  ,
    parameter bit   [4:1] EN_CLKOUTB_X    = 4'b1111                                     ,
    parameter ocb_e [7:1] MODE_CLKOUT_X   = '{OCB0, OCB0, OCB0, OCB0, OCB0, OCB0, OCB0} ,
    parameter fcb_e       MODE_CLKFB      = FCB0                                        ,
    parameter bit         EN_CLKFB_DIFF   = 1'b0                                        ,
    parameter bit         EN_RESET        = 1'b1                                        ,
    parameter bit         POL_RESET       = 1'b0                                        ,
    parameter bit         EN_SAFECLKSTART = 1'b1                                        ,
    parameter bit         EN_SEQUENCER    = 1'b0                                        ,
    parameter seq_e [7:1] MODE_SEQUENCER  = '{SEQ1, SEQ1, SEQ1, SEQ1, SEQ1, SEQ1, SEQ1} ,
    parameter bit         EN_CLKMONITOR   = 1'b0                                        ,
    parameter bit         EN_DYNRECONF    = 1'b0                                        ,
    parameter bit         MODE_DYNRECONF  = 1'b0                                         
    
  )
  (
    input  logic        CLKIN_SEL     , // 
    
    input  logic        CLKIN_1       , // 
    input  logic        CLKIN_1_P     , // 
    input  logic        CLKIN_1_N     , // 
    
    input  logic        CLKIN_2       , // 
    input  logic        CLKIN_2_P     , // 
    input  logic        CLKIN_2_N     , // 
    
    output logic        CLKOUT_1      , // 
    output logic        CLKOUT_2      , // 
    output logic        CLKOUT_3      , // 
    output logic        CLKOUT_4      , // 
    output logic        CLKOUT_5      , // 
    output logic        CLKOUT_6      , // 
    output logic        CLKOUT_7      , // 
    
    input  logic        CLKOUT_1_CE   , // 
    input  logic        CLKOUT_2_CE   , // 
    input  logic        CLKOUT_3_CE   , // 
    input  logic        CLKOUT_4_CE   , // 
    input  logic        CLKOUT_5_CE   , // 
    input  logic        CLKOUT_6_CE   , // 
    input  logic        CLKOUT_7_CE   , // 
    
    output logic        CLKOUTB_1     , // 
    output logic        CLKOUTB_2     , // 
    output logic        CLKOUTB_3     , // 
    output logic        CLKOUTB_4     , // 
    
    output logic        CLKOUTB_1_CE  , // 
    output logic        CLKOUTB_2_CE  , // 
    output logic        CLKOUTB_3_CE  , // 
    output logic        CLKOUTB_4_CE  , // 
    
    input  logic        CLKFB_IN      , // 
    input  logic        CLKFB_IN_P    , // 
    input  logic        CLKFB_IN_N    , // 
    
    output logic        CLKFB_OUT     , // 
    output logic        CLKFB_OUT_P   , // 
    output logic        CLKFB_OUT_N   , // 
    
    input  logic        RESET         , // 
    input  logic        RESETn        , // 
    input  logic        POWER_DOWN    , // 
    output logic        LOCKED        , // 
    // Clock Monitor I/O
    input  logic        REF_CLK       , // 
    input  logic        USER_CLK_0    , // 
    input  logic        USER_CLK_1    , // 
    input  logic        USER_CLK_2    , // 
    input  logic        USER_CLK_3    , // 
    output logic [ 3:0] CLK_STOP      , // 
    output logic [ 3:0] CLK_GLITCH    , // 
    output logic [ 3:0] CLK_OOR       , // 
    // Phase shift I/O
    input  logic        PSCLK         , // 
    input  logic        PSEN          , // 
    input  logic        PSINCDEC      , // 
    output logic        PSDONE        , // 
    // Dynamic Divide I/O
    input  logic        CDDC_REQ      , // 
    output logic        CDDC_DONE     , // 
    // Indicators
    output logic        STOPPED_CLKIN , // 
    output logic        STOPPED_CLKFB   // 
  );
  
  logic MMCM_LOCKED;
  
  //------------------------------------------------------------------------------------------
  //  Clock input gen
  //------------------------------------------------------------------------------------------
  logic MMCM_CLKIN_1   ;
  logic MMCM_CLKIN_2   ;
  logic MMCM_CLKIN_SEL ;
  
  generate
    case(MODE_CLKIN)
      ICB0: // Single ended clock capable pin
      begin:clkin_0
        case(EN_CLKIN_X)
          2'b01:
          begin:en_01
            assign MMCM_CLKIN_SEL = 1'b1;
            assign MMCM_CLKIN_2   = 1'b0;
            IBUF clkin1_ibuf (.I (CLKIN_1), .O (MMCM_CLKIN_1));
          end
          2'b10:
          begin:en_10
            assign MMCM_CLKIN_SEL = 1'b0;
            assign MMCM_CLKIN_1   = 1'b0;
            IBUF clkin2_ibuf (.I (CLKIN_2), .O (MMCM_CLKIN_2));
            $warning("[CLKWIZ] ClockIN - EN_CLKIN_X: Only CLKIN_2 is selected, switching to CLKIN_1 is advised");
          end
          2'b11:
          begin:en_11
            assign MMCM_CLKIN_SEL = CLKIN_SEL;
            IBUF clkin1_ibuf (.I (CLKIN_1), .O (MMCM_CLKIN_1));
            IBUF clkin2_ibuf (.I (CLKIN_2), .O (MMCM_CLKIN_2));
          end
          2'b00:
          begin:en_00
            assign MMCM_CLKIN_1   = 1'b0;
            assign MMCM_CLKIN_2   = 1'b0;
            $warning("[CLKWIZ] ClockIN - EN_CLKIN_X: No clock inputs were enabled");
          end
          default: $error("[CLKWIZ] ClockIN - EN_CLKIN_X has incorrect value");
        endcase
      end
      
      ICB1: // Differential clock capable pin
      begin:clkin_1
        case(EN_CLKIN_X)
          2'b01:
          begin:en_01
            assign MMCM_CLKIN_SEL = 1'b1;
            assign MMCM_CLKIN_2   = 1'b0;
            IBUFDS clkin1_ibufds (.I (CLKIN_1_P), .IB (CLKIN_1_N), .O (MMCM_CLKIN_1));
          end
          2'b10:
          begin:en_10
            assign MMCM_CLKIN_SEL = 1'b0;
            assign MMCM_CLKIN_1   = 1'b0;
            IBUFDS clkin2_ibufds (.I (CLKIN_2_P), .IB (CLKIN_2_N), .O (MMCM_CLKIN_2));
            $warning("[CLKWIZ] ClockIN - EN_CLKIN_X: Only CLKIN_2 is selected, switching to CLKIN_1 is advised");
          end
          2'b11:
          begin:en_11
            assign MMCM_CLKIN_SEL = CLKIN_SEL;
            IBUFDS clkin1_ibufds (.I (CLKIN_1_P), .IB (CLKIN_1_N), .O (MMCM_CLKIN_1));
            IBUFDS clkin2_ibufds (.I (CLKIN_2_P), .IB (CLKIN_2_N), .O (MMCM_CLKIN_2));
          end
          2'b00:
          begin:en_00
            assign MMCM_CLKIN_1   = 1'b0;
            assign MMCM_CLKIN_2   = 1'b0;
            $warning("[CLKWIZ] ClockIN - EN_CLKIN_X: No clock inputs were enabled");
          end
          default: $error("[CLKWIZ] ClockIN - EN_CLKIN_X has incorrect value");
        endcase
      end
      
      ICB2: // Global buffer
      begin:clkin_2
        case(EN_CLKIN_X)
          2'b01:
          begin:en_01
            assign MMCM_CLKIN_SEL = 1'b1;
            assign MMCM_CLKIN_2   = 1'b0;
            BUFG clkin1_bufgce (.I (CLKIN_1), .O (MMCM_CLKIN_1));
          end
          2'b10:
          begin:en_10
            assign MMCM_CLKIN_SEL = 1'b0;
            assign MMCM_CLKIN_1   = 1'b0;
            BUFG clkin2_bufgce (.I (CLKIN_2), .O (MMCM_CLKIN_2));
            $warning("[CLKWIZ] ClockIN - EN_CLKIN_X: Only CLKIN_2 is selected, switching to CLKIN_1 is advised");
          end
          2'b11:
          begin:en_11
            assign MMCM_CLKIN_SEL = CLKIN_SEL;
            BUFG clkin1_bufgce (.I (CLKIN_1), .O (MMCM_CLKIN_1));
            BUFG clkin2_bufgce (.I (CLKIN_2), .O (MMCM_CLKIN_2));
          end
          2'b00:
          begin:en_00
            assign MMCM_CLKIN_1   = 1'b0;
            assign MMCM_CLKIN_2   = 1'b0;
            $warning("[CLKWIZ] ClockIN - EN_CLKIN_X: No clock inputs were enabled");
          end
          default: $error("[CLKWIZ] ClockIN - EN_CLKIN_X has incorrect value");
        endcase
      end
      
      ICB3: // No buffer
      begin:clkin_3
        case(EN_CLKIN_X)
          2'b01:
          begin:en_01
            assign MMCM_CLKIN_SEL = 1'b1;
            assign MMCM_CLKIN_1   = CLKIN_1;
            assign MMCM_CLKIN_2   = 1'b0;
          end
          2'b10:
          begin:en_10
            assign MMCM_CLKIN_SEL = 1'b0;
            assign MMCM_CLKIN_1   = 1'b0;
            assign MMCM_CLKIN_2   = CLKIN_2;
            $warning("[CLKWIZ] ClockIN - EN_CLKIN_X: Only CLKIN_2 is selected, switching to CLKIN_1 is advised");
          end
          2'b11:
          begin:en_11
            assign MMCM_CLKIN_SEL = CLKIN_SEL;
            assign MMCM_CLKIN_1   = CLKIN_1;
            assign MMCM_CLKIN_2   = CLKIN_2;
          end
          2'b00:
          begin:en_00
            assign MMCM_CLKIN_1   = 1'b0;
            assign MMCM_CLKIN_2   = 1'b0;
            $warning("[CLKWIZ] ClockIN - EN_CLKIN_X: No clock inputs were enabled");
          end
          default: $error("[CLKWIZ] ClockIN - EN_CLKIN_X has incorrect value");
        endcase
      end
      
      default: $error("[CLKWIZ] ClockIN - MODE_CLKIN has incorrect value");
    endcase
  endgenerate
  
  //------------------------------------------------------------------------------------------
  //  Clock output gen
  //------------------------------------------------------------------------------------------
  
  logic [7:1] MMCM_CLKOUT   ;
  logic [4:1] MMCM_CLKOUTB  ;
  
  logic [7:1] CLKOUT_X;
  logic [7:1] CLKOUT_X_CE;
  logic [7:1] CLKOUT_X_EN;
  logic [4:1] CLKOUTB_X;
  logic [4:1] CLKOUTB_X_CE;
  logic [4:1] CLKOUTB_X_EN;
  
  assign {CLKOUT_7, CLKOUT_6, CLKOUT_5, CLKOUT_4, CLKOUT_3, CLKOUT_2, CLKOUT_1} = CLKOUT_X;
  assign CLKOUT_X_CE = {CLKOUT_7_CE, CLKOUT_6_CE, CLKOUT_5_CE, CLKOUT_4_CE, CLKOUT_3_CE, CLKOUT_2_CE, CLKOUT_1_CE};
  
  assign {CLKOUTB_4, CLKOUTB_3, CLKOUTB_2, CLKOUTB_1} = CLKOUTB_X;
  assign CLKOUTB_X_CE = {CLKOUTB_4_CE, CLKOUTB_3_CE, CLKOUTB_2_CE, CLKOUTB_1_CE};
  
  genvar clko;
  genvar clkob;
  generate
    case(EN_SAFECLKSTART)
      1'b0: // Safe clock startup disabled
      begin:safest_off
        for(clko = 1; clko < 7; clko = clko + 'd1)
        begin
          case(EN_CLKOUT_X[clko])
            1'b1: 
            begin:o_on
              case(MODE_CLKOUT_X[clko])
                OCB0: BUFG   clkout_bufg   (.I (MMCM_CLKOUT[clko]), .O (CLKOUT_X[clko]));
                OCB1: BUFGCE clkout_bufgce (.I (MMCM_CLKOUT[clko]), .CE (CLKOUT_X_CE[clko]), .O (CLKOUT_X[clko]));
                OCB2: BUFG   clkout_bufg   (.I (MMCM_CLKOUT[clko]), .O (CLKOUT_X[clko]));
                OCB3: BUFGCE clkout_bufgce (.I (MMCM_CLKOUT[clko]), .CE (CLKOUT_X_CE[clko]), .O (CLKOUT_X[clko]));
                OCB4: ;
                OCB5: assign CLKOUT_X[clko] = MMCM_CLKOUT[clko];
                default: $error("[CLKWIZ] ClockOUT - MODE_CLKOUT_X[%0d] has incorrect value", clko);
              endcase
            end
            1'b0: 
            begin:o_off
              assign CLKOUT_X[clko] = 1'b0;
            end
            default: $error("[CLKWIZ] ClockOUT - EN_CLKOUT_X[%0d] has incorrect value", clko);
          endcase
        end
        
        for(clkob = 1; clkob < 4; clkob = clkob + 'd1)
        begin
          case(EN_CLKOUTB_X[clkob])
            1'b1: 
            begin:ob_on
              case(MODE_CLKOUT_X[clkob])
                OCB0: BUFG   clkoutb_bufg   (.I (MMCM_CLKOUTB[clkob]), .O (CLKOUTB_X[clkob]));
                OCB1: BUFGCE clkoutb_bufgce (.I (MMCM_CLKOUTB[clkob]), .CE (CLKOUTB_X_CE[clkob]), .O (CLKOUTB_X[clkob]));
                OCB2: BUFG   clkoutb_bufg   (.I (MMCM_CLKOUTB[clkob]), .O (CLKOUTB_X[clkob]));
                OCB3: BUFGCE clkoutb_bufgce (.I (MMCM_CLKOUTB[clkob]), .CE (CLKOUTB_X_CE[clkob]), .O (CLKOUTB_X[clkob]));
                OCB4: ;
                OCB5: assign CLKOUTB_X[clkob] = MMCM_CLKOUTB[clkob];
                default: $error("[CLKWIZ] ClockOUT - MODE_CLKOUT_X[%0d] has incorrect value", clkob);
              endcase
            end
            1'b0: 
            begin:ob_off
              assign CLKOUTB_X[clko] = 1'b0;
            end
            default: $error("[CLKWIZ] ClockOUT - EN_CLKOUTB_X[%0d] has incorrect value", clkob);
          endcase
        end
      end
      
      1'b1: // Safe clock startup enabled
      begin:safest_on
        for(clko = 1; clko < 7; clko = clko + 'd1)
        begin
          case(EN_CLKOUT_X[clko])
            1'b1: 
            begin:o_on
              
              logic [7:0] seq_reg_x;
              
              BUFGCE clkout_bufgce    (.I (MMCM_CLKOUT[clko]), .CE (seq_reg_x[7]), .O (CLKOUT_X[clko]   ));
              BUFGCE clkout_bufgce_en (.I (MMCM_CLKOUT[clko]), .CE (1'b1        ), .O (CLKOUT_X_EN[clko]));
              
              case(EN_RESET)
                1'b1:
                begin:rst_on
                  case(POL_RESET)
                    1'b0:
                    begin:rpos
                      always_ff @(posedge CLKOUT_X_EN[clko] or posedge RESET) 
                      begin
                        if(RESET == 1'b1) seq_reg_x <= 8'h00;
                        else seq_reg_x <= {seq_reg_x[6:0], MMCM_LOCKED};
                      end
                    end
                    1'b1:
                    begin:rneg
                      always_ff @(posedge CLKOUT_X_EN[clko] or negedge RESETn) 
                      begin
                        if(RESETn == 1'b0) seq_reg_x <= 8'h00;
                        else seq_reg_x <= {seq_reg_x[6:0], MMCM_LOCKED};
                      end
                    end
                    
                    default: $error("[CLKWIZ] ClockOUT - POL_RESET has incorrect value");
                  endcase
                end
                
                1'b0:
                begin:rst_off
                  always_ff @(posedge CLKOUT_X_EN[clko]) 
                  begin
                    seq_reg_x <= {seq_reg_x[6:0], MMCM_LOCKED};
                  end
                end
                
                default: $error("[CLKWIZ] ClockOUT - EN_RESET has incorrect value");
              endcase
              
              if(MODE_CLKOUT_X[clko] != OCB3) $warning("[CLKWIZ] ClockOUT - 'Safe Clock Startup' only supports MODE_CLKOUT_X[%0d] = OCB3(BUFGCE)", clko);
            end
            1'b0: 
            begin:o_off
              assign CLKOUT_X[clko] = 1'b0;
            end
            default: $error("[CLKWIZ] ClockOUT - EN_CLKOUT_X[%0d] has incorrect value", clko);
          endcase
        end
        
        for(clkob = 1; clkob < 4; clkob = clkob + 'd1)
        begin
          case(EN_CLKOUTB_X[clkob])
            1'b1: 
            begin:ob_on
              logic [7:0] seqb_reg_x;
              
              BUFGCE clkout_bufgce    (.I (MMCM_CLKOUTB[clkob]), .CE (seqb_reg_x[7]), .O (CLKOUTB_X[clkob]   ));
              BUFGCE clkout_bufgce_en (.I (MMCM_CLKOUTB[clkob]), .CE (1'b1         ), .O (CLKOUTB_X_EN[clkob]));
              
              case(EN_RESET)
                1'b1:
                begin:rst_on
                  case(POL_RESET)
                    1'b0:
                    begin:rpos
                      always_ff @(posedge CLKOUTB_X_EN[clkob] or posedge RESET) 
                      begin
                        if(RESET == 1'b1) seqb_reg_x <= 8'h00;
                        else seqb_reg_x <= {seqb_reg_x[6:0], MMCM_LOCKED};
                      end
                    end
                    1'b1:
                    begin:rneg
                      always_ff @(posedge CLKOUTB_X_EN[clkob] or negedge RESETn) 
                      begin
                        if(RESETn == 1'b0) seqb_reg_x <= 8'h00;
                        else seqb_reg_x <= {seqb_reg_x[6:0], MMCM_LOCKED};
                      end
                    end
                    
                    default: $error("[CLKWIZ] ClockOUT - POL_RESET has incorrect value");
                  endcase
                end
                
                1'b0:
                begin:rst_off
                  always_ff @(posedge CLKOUTB_X_EN[clkob]) 
                  begin
                    seqb_reg_x <= {seqb_reg_x[6:0], MMCM_LOCKED};
                  end
                end
                
                default: $error("[CLKWIZ] ClockOUT - EN_RESET has incorrect value");
              endcase
            end
            
            1'b0: 
            begin:ob_off
              assign CLKOUTB_X[clkob] = 1'b0;
            end
            default: $error("[CLKWIZ] ClockOUT - EN_CLKOUTB_X[%0d] has incorrect value", clkob);
          endcase
        end
      end
      
      default: $error("[CLKWIZ] ClockOUT - EN_SAFECLKSTART has incorrect value");
    endcase
  endgenerate
  
  //------------------------------------------------------------------------------------------
  //  Clock feedback gen
  //------------------------------------------------------------------------------------------
  logic MMCM_CLKFB_IN  ;
  logic MMCM_CLKFB_OUT ;
  logic MMCM_CLKFB_OUTB;
  
  generate
    case(MODE_CLKFB)
      FCB0: // Automatic control On-Chip
      begin:clkfb_0
        assign CLKFB_OUT   = 1'b0;
        assign CLKFB_OUT_P = 1'b0;
        assign CLKFB_OUT_N = 1'b0;
        
        BUFG clkf_buf (.I (MMCM_CLKFB_OUT), .O (MMCM_CLKFB_IN));
        
        if(EN_CLKFB_DIFF == 1'b1) 
          $warning("[CLKWIZ] ClockFeedback - EN_CLKFB_DIFF is not available in 'Automatic control On-Chip' mode");
      end
      
      FCB1: // Automatic control Off-Chip
      begin:clkfb_1
        case(EN_CLKFB_DIFF)
          1'b0:
          begin:nodiff
            assign CLKFB_OUT_P = 1'b0;
            assign CLKFB_OUT_N = 1'b0;
            
            IBUF clkfb_ibuf (.I (CLKFB_IN), .O (MMCM_CLKFB_IN));
            ODDRE1 clkfbout_oddr (.Q (CLKFB_OUT), .C (MMCM_CLKFB_OUT), .D1 (1'b1), .D2 (1'b0), .SR (1'b0));
          end
          
          1'b1:
          begin:diff
            assign CLKFB_OUT   = 1'b0;
            logic ODDR_CLKFB_OUT;
            
            IBUFDS clkfb_ibufds (.I (CLKFB_IN_P), .IB (CLKFB_IN_N), .O (MMCM_CLKFB_IN));
            ODDRE1 clkfb_oddr   (.Q (ODDR_CLKFB_OUT), .C (MMCM_CLKFB_OUT), .D1 (1'b1), .D2 (1'b0), .SR (1'b0));
            OBUFDS clkfb_obufds (.I (ODDR_CLKFB_OUT), .O (CLKFB_OUT_P), .OB (CLKFB_OUT_N));
          end
          
          default: $error("[CLKWIZ] ClockFeedback - EN_CLKFB_DIFF has incorrect value");
        endcase
      end
      
      FCB2: // User-controlled On-Chip
      begin:clkfb_2
        assign CLKFB_OUT   = MMCM_CLKFB_OUT;
        assign CLKFB_OUT_P = 1'b0;
        assign CLKFB_OUT_N = 1'b0;
        assign MMCM_CLKFB_IN = CLKFB_IN;
        
        if(EN_CLKFB_DIFF == 1'b1) 
          $warning("[CLKWIZ] ClockFeedback - EN_CLKFB_DIFF is not available in 'User-controlled On-Chip' mode");
      end
      
      FCB3: // User-controlled Off-Chip
      begin:clkfb_3
        assign CLKFB_OUT   = MMCM_CLKFB_OUT;
        assign CLKFB_OUT_P = 1'b0;
        assign CLKFB_OUT_N = 1'b0;
        assign MMCM_CLKFB_IN = CLKFB_IN;
        
        if(EN_CLKFB_DIFF == 1'b1) 
          $warning("[CLKWIZ] ClockFeedback - EN_CLKFB_DIFF is not available in 'User-controlled Off-Chip' mode");
      end
      
      default: $error("[CLKWIZ] ClockFeedback - MODE_CLKFB has incorrect value");
    endcase
  endgenerate
  
  //------------------------------------------------------------------------------------------
  //  Reset generation
  //------------------------------------------------------------------------------------------
  
  
  //------------------------------------------------------------------------------------------
  //  MMCM generation
  //------------------------------------------------------------------------------------------
  generate
    if(FPGA_FAMILY == "SERIES7")
    begin
      MMCME2_ADV
      #(
        .BANDWIDTH            ("OPTIMIZED"    ), // Jitter programming (OPTIMIZED, HIGH, LOW)
        .COMPENSATION         ("ZHOLD"        ), // ZHOLD, BUF_IN, EXTERNAL, INTERNAL
        .STARTUP_WAIT         ("FALSE"        ), // Delays DONE until MMCM is locked (FALSE, TRUE)
        .CLKOUT4_CASCADE      ("FALSE"        ), // Cascade CLKOUT4 counter with CLKOUT6 (FALSE, TRUE)
        
        .CLKIN1_PERIOD        (0.0            ), // CLKIN1_PERIOD: Input clock period in ns to ps resolution (i.e. 33.333 is 30MHz).
        .CLKIN2_PERIOD        (0.0            ), // CLKIN2_PERIOD: Input clock period in ns to ps resolution (i.e. 33.333 is 30MHz).
        .REF_JITTER1          (0.0            ), // REF_JITTER1: Reference input jitter in UI (0.000 - 0.999).
        .REF_JITTER2          (0.0            ), // REF_JITTER2: Reference input jitter in UI (0.000 - 0.999).
        
        .CLKFBOUT_MULT_F      (5.0            ), // Multiply value for all CLKOUT (2.000 - 64.000).
        .DIVCLK_DIVIDE        (1              ), // Master division value (1 - 106)
        .CLKFBOUT_PHASE       (0.0            ), // Phase offset in degrees of CLKFB (-360.000 - 360.000).
        
        .CLKOUT0_DIVIDE_F     (1.0            ), // CLKOUT0_DIVIDE: Divide amount for CLKOUT0 (1.000 - 128.000).
        .CLKOUT1_DIVIDE       (1              ), // CLKOUT1_DIVIDE: Divide amount for CLKOUT1 (1 - 128)
        .CLKOUT2_DIVIDE       (1              ), // CLKOUT2_DIVIDE: Divide amount for CLKOUT2 (1 - 128)
        .CLKOUT3_DIVIDE       (1              ), // CLKOUT3_DIVIDE: Divide amount for CLKOUT3 (1 - 128)
        .CLKOUT4_DIVIDE       (1              ), // CLKOUT4_DIVIDE: Divide amount for CLKOUT4 (1 - 128)
        .CLKOUT5_DIVIDE       (1              ), // CLKOUT5_DIVIDE: Divide amount for CLKOUT5 (1 - 128)
        .CLKOUT6_DIVIDE       (1              ), // CLKOUT6_DIVIDE: Divide amount for CLKOUT6 (1 - 128)
        
        .CLKOUT0_DUTY_CYCLE   (0.5            ), // CLKOUT0_DUTY_CYCLE: Duty cycle for CLKOUT outputs (0.01 - 0.99).
        .CLKOUT1_DUTY_CYCLE   (0.5            ), // CLKOUT1_DUTY_CYCLE: Duty cycle for CLKOUT outputs (0.01 - 0.99).
        .CLKOUT2_DUTY_CYCLE   (0.5            ), // CLKOUT2_DUTY_CYCLE: Duty cycle for CLKOUT outputs (0.01 - 0.99).
        .CLKOUT3_DUTY_CYCLE   (0.5            ), // CLKOUT3_DUTY_CYCLE: Duty cycle for CLKOUT outputs (0.01 - 0.99).
        .CLKOUT4_DUTY_CYCLE   (0.5            ), // CLKOUT4_DUTY_CYCLE: Duty cycle for CLKOUT outputs (0.01 - 0.99).
        .CLKOUT5_DUTY_CYCLE   (0.5            ), // CLKOUT5_DUTY_CYCLE: Duty cycle for CLKOUT outputs (0.01 - 0.99).
        .CLKOUT6_DUTY_CYCLE   (0.5            ), // CLKOUT6_DUTY_CYCLE: Duty cycle for CLKOUT outputs (0.01 - 0.99).
        
        .CLKOUT0_PHASE        (0.0            ), // CLKOUT0_PHASE: Phase offset for CLKOUT outputs (-360.000 - 360.000).
        .CLKOUT1_PHASE        (0.0            ), // CLKOUT1_PHASE: Phase offset for CLKOUT outputs (-360.000 - 360.000).
        .CLKOUT2_PHASE        (0.0            ), // CLKOUT2_PHASE: Phase offset for CLKOUT outputs (-360.000 - 360.000).
        .CLKOUT3_PHASE        (0.0            ), // CLKOUT3_PHASE: Phase offset for CLKOUT outputs (-360.000 - 360.000).
        .CLKOUT4_PHASE        (0.0            ), // CLKOUT4_PHASE: Phase offset for CLKOUT outputs (-360.000 - 360.000).
        .CLKOUT5_PHASE        (0.0            ), // CLKOUT5_PHASE: Phase offset for CLKOUT outputs (-360.000 - 360.000).
        .CLKOUT6_PHASE        (0.0            ), // CLKOUT6_PHASE: Phase offset for CLKOUT outputs (-360.000 - 360.000).
        
        .SS_EN                ("FALSE"        ), // Enables spread spectrum (FALSE, TRUE)
        .SS_MODE              ("CENTER_HIGH"  ), // CENTER_HIGH, CENTER_LOW, DOWN_HIGH, DOWN_LOW
        .SS_MOD_PERIOD        (10000          ), // Spread spectrum modulation period (ns) (VALUES)
        
        .CLKFBOUT_USE_FINE_PS ("FALSE"        ), // CLKFBOUT_USE_FINE_PS: Fine phase shift enable (TRUE/FALSE)
        .CLKOUT0_USE_FINE_PS  ("FALSE"        ), // CLKOUT0_USE_FINE_PS: Fine phase shift enable (TRUE/FALSE)
        .CLKOUT1_USE_FINE_PS  ("FALSE"        ), // CLKOUT1_USE_FINE_PS: Fine phase shift enable (TRUE/FALSE)
        .CLKOUT2_USE_FINE_PS  ("FALSE"        ), // CLKOUT2_USE_FINE_PS: Fine phase shift enable (TRUE/FALSE)
        .CLKOUT3_USE_FINE_PS  ("FALSE"        ), // CLKOUT3_USE_FINE_PS: Fine phase shift enable (TRUE/FALSE)
        .CLKOUT4_USE_FINE_PS  ("FALSE"        ), // CLKOUT4_USE_FINE_PS: Fine phase shift enable (TRUE/FALSE)
        .CLKOUT5_USE_FINE_PS  ("FALSE"        ), // CLKOUT5_USE_FINE_PS: Fine phase shift enable (TRUE/FALSE)
        .CLKOUT6_USE_FINE_PS  ("FALSE"        )  // CLKOUT6_USE_FINE_PS: Fine phase shift enable (TRUE/FALSE)
      )
      SERIES7_MMCME2_ADV
      (
        .CLKIN1       (MMCM_CLKIN_1         ), //  1-bit  input: Primary clock
        .CLKIN2       (MMCM_CLKIN_2         ), //  1-bit  input: Secondary clock
        .CLKINSEL     (MMCM_CLKIN_SEL       ), //  1-bit  input: Clock select, High = CLKIN1 Low = CLKIN2
        
        .PWRDWN       (PWRDWN               ), //  1-bit  input: Power-down
        .RST          (RST                  ), //  1-bit  input: Reset
        .LOCKED       (MMCM_LOCKED          ), //  1-bit output: LOCK
        
        .CLKOUT0      (MMCM_CLKOUT[1]       ), //  1-bit output: CLKOUT0
        .CLKOUT0B     (MMCM_CLKOUTB[1]      ), //  1-bit output: Inverted CLKOUT0
        .CLKOUT1      (MMCM_CLKOUT[2]       ), //  1-bit output: CLKOUT1
        .CLKOUT1B     (MMCM_CLKOUTB[2]      ), //  1-bit output: Inverted CLKOUT1
        .CLKOUT2      (MMCM_CLKOUT[3]       ), //  1-bit output: CLKOUT2
        .CLKOUT2B     (MMCM_CLKOUTB[3]      ), //  1-bit output: Inverted CLKOUT2
        .CLKOUT3      (MMCM_CLKOUT[4]       ), //  1-bit output: CLKOUT3
        .CLKOUT3B     (MMCM_CLKOUTB[4]      ), //  1-bit output: Inverted CLKOUT3
        .CLKOUT4      (MMCM_CLKOUT[5]       ), //  1-bit output: CLKOUT4
        .CLKOUT5      (MMCM_CLKOUT[6]       ), //  1-bit output: CLKOUT5
        .CLKOUT6      (MMCM_CLKOUT[7]       ), //  1-bit output: CLKOUT6
        
        .CLKFBOUT     (MMCM_CLKFB_OUT       ), //  1-bit output: Feedback clock
        .CLKFBOUTB    (MMCM_CLKFB_OUTB      ), //  1-bit output: Inverted CLKFBOUT
        .CLKFBIN      (MMCM_CLKFB_IN        ), //  1-bit  input: Feedback clock
        
        .DCLK         (DCLK                 ), //  1-bit  input: DRP clock
        .DEN          (DEN                  ), //  1-bit  input: DRP enable
        .DWE          (DWE                  ), //  1-bit  input: DRP write enable
        .DADDR        (DADDR                ), //  7-bit  input: DRP address
        .DI           (DI                   ), // 16-bit  input: DRP data
        .DRDY         (DRDY                 ), //  1-bit output: DRP ready
        .DO           (DO                   ), // 16-bit output: DRP data
        
        .PSCLK        (PSCLK                ), //  1-bit  input: Phase shift clock
        .PSEN         (PSEN                 ), //  1-bit  input: Phase shift enable
        .PSINCDEC     (PSINCDEC             ), //  1-bit  input: Phase shift increment/decrement
        .PSDONE       (PSDONE               ), //  1-bit output: Phase shift done
        
        .CLKFBSTOPPED (CLKFBSTOPPED         ), //  1-bit output: Feedback clock stopped
        .CLKINSTOPPED (CLKINSTOPPED         )  //  1-bit output: Input clock stopped
      );
    end
    else if(FPGA_FAMILY == "ULTRASCALE")
    begin
      MMCME3_ADV 
      #(
        .BANDWIDTH              ("OPTIMIZED"    ), // Jitter programming (HIGH, LOW, OPTIMIZED) 
        .COMPENSATION           ("AUTO"         ), // AUTO, BUF_IN, EXTERNAL, INTERNAL, ZHOLD 
        .STARTUP_WAIT           ("FALSE"        ), // Delays DONE until MMCM is locked (FALSE, TRUE)
        .CLKOUT4_CASCADE        ("FALSE"        ), // Cascade CLKOUT4 counter with CLKOUT6 (FALSE, TRUE)
        
        .CLKIN1_PERIOD          (0.0            ), // CLKIN1_PERIOD: Input clock period in ns units, ps resolution (i.e., 33.333 is 30 MHz). 
        .CLKIN2_PERIOD          (0.0            ), // CLKIN2_PERIOD: Input clock period in ns units, ps resolution (i.e., 33.333 is 30 MHz). 
        .REF_JITTER1            (0.0            ), // REF_JITTER1: Reference input jitter in UI (0.000-0.999). 
        .REF_JITTER2            (0.0            ), // REF_JITTER2: Reference input jitter in UI (0.000-0.999). 
        
        .CLKFBOUT_MULT_F        (5.0            ), // Multiply value for all CLKOUT (2.000-64.000) 
        .DIVCLK_DIVIDE          (1              ), // Master division value (1-106)
        .CLKFBOUT_PHASE         (0.0            ), // Phase offset in degrees of CLKFB (-360.000-360.000) 
        
        .CLKOUT0_DIVIDE_F       (1.0            ), // Divide amount for CLKOUT0 (1.000-128.000) 
        .CLKOUT1_DIVIDE         (1              ), // CLKOUT1_DIVIDE: Divide amount for CLKOUT (1-128) 
        .CLKOUT2_DIVIDE         (1              ), // CLKOUT2_DIVIDE: Divide amount for CLKOUT (1-128) 
        .CLKOUT3_DIVIDE         (1              ), // CLKOUT3_DIVIDE: Divide amount for CLKOUT (1-128) 
        .CLKOUT4_DIVIDE         (1              ), // CLKOUT4_DIVIDE: Divide amount for CLKOUT (1-128) 
        .CLKOUT5_DIVIDE         (1              ), // CLKOUT5_DIVIDE: Divide amount for CLKOUT (1-128) 
        .CLKOUT6_DIVIDE         (1              ), // CLKOUT6_DIVIDE: Divide amount for CLKOUT (1-128) 
        
        .CLKOUT0_DUTY_CYCLE     (0.5            ), // CLKOUT0_DUTY_CYCLE: Duty cycle for CLKOUT outputs (0.001-0.999). 
        .CLKOUT1_DUTY_CYCLE     (0.5            ), // CLKOUT1_DUTY_CYCLE: Duty cycle for CLKOUT outputs (0.001-0.999). 
        .CLKOUT2_DUTY_CYCLE     (0.5            ), // CLKOUT2_DUTY_CYCLE: Duty cycle for CLKOUT outputs (0.001-0.999). 
        .CLKOUT3_DUTY_CYCLE     (0.5            ), // CLKOUT3_DUTY_CYCLE: Duty cycle for CLKOUT outputs (0.001-0.999). 
        .CLKOUT4_DUTY_CYCLE     (0.5            ), // CLKOUT4_DUTY_CYCLE: Duty cycle for CLKOUT outputs (0.001-0.999). 
        .CLKOUT5_DUTY_CYCLE     (0.5            ), // CLKOUT5_DUTY_CYCLE: Duty cycle for CLKOUT outputs (0.001-0.999). 
        .CLKOUT6_DUTY_CYCLE     (0.5            ), // CLKOUT6_DUTY_CYCLE: Duty cycle for CLKOUT outputs (0.001-0.999). 
        
        .CLKOUT0_PHASE          (0.0            ), // CLKOUT0_PHASE: Phase offset for CLKOUT outputs (-360.000-360.000). 
        .CLKOUT1_PHASE          (0.0            ), // CLKOUT1_PHASE: Phase offset for CLKOUT outputs (-360.000-360.000). 
        .CLKOUT2_PHASE          (0.0            ), // CLKOUT2_PHASE: Phase offset for CLKOUT outputs (-360.000-360.000). 
        .CLKOUT3_PHASE          (0.0            ), // CLKOUT3_PHASE: Phase offset for CLKOUT outputs (-360.000-360.000). 
        .CLKOUT4_PHASE          (0.0            ), // CLKOUT4_PHASE: Phase offset for CLKOUT outputs (-360.000-360.000). 
        .CLKOUT5_PHASE          (0.0            ), // CLKOUT5_PHASE: Phase offset for CLKOUT outputs (-360.000-360.000). 
        .CLKOUT6_PHASE          (0.0            ), // CLKOUT6_PHASE: Phase offset for CLKOUT outputs (-360.000-360.000). 
        
        .SS_EN                  ("FALSE"        ), // Enables spread spectrum (FALSE, TRUE)
        .SS_MODE                ("CENTER_HIGH"  ), // CENTER_HIGH, CENTER_LOW, DOWN_HIGH, DOWN_LOW 
        .SS_MOD_PERIOD          (10000          ), // Spread spectrum modulation period (ns) (4000-40000)
        
        .CLKFBOUT_USE_FINE_PS   ("FALSE"        ), // CLKFBOUT_USE_FINE_PS: Fine phase shift enable (TRUE/FALSE) 
        .CLKOUT0_USE_FINE_PS    ("FALSE"        ), // CLKOUT0_USE_FINE_PS : Fine phase shift enable (TRUE/FALSE) 
        .CLKOUT1_USE_FINE_PS    ("FALSE"        ), // CLKOUT1_USE_FINE_PS : Fine phase shift enable (TRUE/FALSE) 
        .CLKOUT2_USE_FINE_PS    ("FALSE"        ), // CLKOUT2_USE_FINE_PS : Fine phase shift enable (TRUE/FALSE) 
        .CLKOUT3_USE_FINE_PS    ("FALSE"        ), // CLKOUT3_USE_FINE_PS : Fine phase shift enable (TRUE/FALSE) 
        .CLKOUT4_USE_FINE_PS    ("FALSE"        ), // CLKOUT4_USE_FINE_PS : Fine phase shift enable (TRUE/FALSE) 
        .CLKOUT5_USE_FINE_PS    ("FALSE"        ), // CLKOUT5_USE_FINE_PS : Fine phase shift enable (TRUE/FALSE) 
        .CLKOUT6_USE_FINE_PS    ("FALSE"        ), // CLKOUT6_USE_FINE_PS : Fine phase shift enable (TRUE/FALSE) 
        
        .IS_CLKFBIN_INVERTED    (1'b0           ), // Optional inversion for CLKFBIN 
        .IS_CLKIN1_INVERTED     (1'b0           ), // Optional inversion for CLKIN1 
        .IS_CLKIN2_INVERTED     (1'b0           ), // Optional inversion for CLKIN2 
        .IS_CLKINSEL_INVERTED   (1'b0           ), // Optional inversion for CLKINSEL 
        .IS_PSEN_INVERTED       (1'b0           ), // Optional inversion for PSEN 
        .IS_PSINCDEC_INVERTED   (1'b0           ), // Optional inversion for PSINCDEC 
        .IS_PWRDWN_INVERTED     (1'b0           ), // Optional inversion for PWRDWN 
        .IS_RST_INVERTED        (1'b0           )  // Optional inversion for RST  
      )
      ULTRASCALE_MMCME3_ADV 
      (
        .CLKIN1       (MMCM_CLKIN_1     ), //  1-bit  input: Primary clock 
        .CLKIN2       (MMCM_CLKIN_2     ), //  1-bit  input: Secondary clock 
        .CLKINSEL     (MMCM_CLKIN_SEL   ), //  1-bit  input: Clock select, High=CLKIN1 Low=CLKIN2 
        
        .PWRDWN       (PWRDWN           ), //  1-bit  input: Power-down 
        .RST          (RST              ), //  1-bit  input: Reset
        .LOCKED       (MMCM_LOCKED      ), //  1-bit output: LOCK 
        
        .CLKOUT0      (MMCM_CLKOUT[1]   ), //  1-bit output: CLKOUT0 
        .CLKOUT0B     (MMCM_CLKOUTB[1]  ), //  1-bit output: Inverted CLKOUT0. 
        .CLKOUT1      (MMCM_CLKOUT[2]   ), //  1-bit output: Primary clock 
        .CLKOUT1B     (MMCM_CLKOUTB[2]  ), //  1-bit output: Inverted CLKOUT1 
        .CLKOUT2      (MMCM_CLKOUT[3]   ), //  1-bit output: CLKOUT2 
        .CLKOUT2B     (MMCM_CLKOUTB[3]  ), //  1-bit output: Inverted CLKOUT2 
        .CLKOUT3      (MMCM_CLKOUT[4]   ), //  1-bit output: CLKOUT3 
        .CLKOUT3B     (MMCM_CLKOUTB[4]  ), //  1-bit output: Inverted CLKOUT3 
        .CLKOUT4      (MMCM_CLKOUT[5]   ), //  1-bit output: CLKOUT4 
        .CLKOUT5      (MMCM_CLKOUT[6]   ), //  1-bit output: CLKOUT5 
        .CLKOUT6      (MMCM_CLKOUT[7]   ), //  1-bit output: CLKOUT6
        
        .CLKFBOUT     (MMCM_CLKFB_OUT   ), //  1-bit output: Feedback clock 
        .CLKFBOUTB    (MMCM_CLKFB_OUTB  ), //  1-bit output: Inverted CLKFBOUT
        .CLKFBIN      (MMCM_CLKFB_IN    ), //  1-bit  input: Feedback clock
        
        .DCLK         (DCLK             ), //  1-bit  input: DRP clock 
        .DEN          (DEN              ), //  1-bit  input: DRP enable 
        .DWE          (DWE              ), //  1-bit  input: DRP write enable
        .DADDR        (DADDR            ), //  7-bit  input: DRP address 
        .DI           (DI               ), // 16-bit  input: DRP data 
        .DRDY         (DRDY             ), //  1-bit output: DRP ready
        .DO           (DO               ), // 16-bit output: DRP data 
        
        .PSCLK        (PSCLK            ), //  1-bit  input: Phase shift clock 
        .PSEN         (PSEN             ), //  1-bit  input: Phase shift enable 
        .PSINCDEC     (PSINCDEC         ), //  1-bit  input: Phase shift increment/decrement
        .PSDONE       (PSDONE           ), //  1-bit output: Phase shift done
        
        .CLKFBSTOPPED (CLKFBSTOPPED     ), //  1-bit output: Feedback clock stopped 
        .CLKINSTOPPED (CLKINSTOPPED     ), //  1-bit output: Input clock stopped 
        
        .CDDCREQ      (CDDCREQ          ), //  1-bit  input: Request to dynamic divide clock
        .CDDCDONE     (CDDCDONE         )  //  1-bit output: Clock dynamic divide done 
      );
    end
    else if(FPGA_FAMILY == "ULTRASCALE_PLUS")
    begin
      MMCME4_ADV 
      #(   
        .BANDWIDTH            ("OPTIMIZED"    ), // Jitter programming   
        .COMPENSATION         ("AUTO"         ), // Clock input compensation 
        .STARTUP_WAIT         ("FALSE"        ), // Delays DONE until MMCM is locked
        .CLKOUT4_CASCADE      ("FALSE"        ), // Cascade CLKOUT4 counter with CLKOUT6 (FALSE, TRUE)
        
        .CLKIN1_PERIOD        (0.0            ), // Input clock period in ns to ps resolution (i.e., 33.333 is 30 MHz).   
        .CLKIN2_PERIOD        (0.0            ), // Input clock period in ns to ps resolution (i.e., 33.333 is 30 MHz).   
        .REF_JITTER1          (0.0            ), // Reference input jitter in UI (0.000-0.999). 
        .REF_JITTER2          (0.0            ), // Reference input jitter in UI (0.000-0.999). 
        
        .CLKFBOUT_MULT_F      (5.0            ), // Multiply value for all CLKOUT   
        .DIVCLK_DIVIDE        (1              ), // Master division value 
        .CLKFBOUT_PHASE       (0.0            ), // Phase offset in degrees of CLKFB   
        
        .CLKOUT0_DIVIDE_F     (1.0            ), // Divide amount for CLKOUT0   
        .CLKOUT1_DIVIDE       (1              ), // Divide amount for CLKOUT (1-128) 
        .CLKOUT2_DIVIDE       (1              ), // Divide amount for CLKOUT (1-128) 
        .CLKOUT3_DIVIDE       (1              ), // Divide amount for CLKOUT (1-128) 
        .CLKOUT4_DIVIDE       (1              ), // Divide amount for CLKOUT (1-128) 
        .CLKOUT5_DIVIDE       (1              ), // Divide amount for CLKOUT (1-128) 
        .CLKOUT6_DIVIDE       (1              ), // Divide amount for CLKOUT (1-128) 
        
        .CLKOUT0_DUTY_CYCLE   (0.5            ), // Duty cycle for CLKOUT0   
        .CLKOUT1_DUTY_CYCLE   (0.5            ), // Duty cycle for CLKOUT outputs (0.001-0.999). 
        .CLKOUT2_DUTY_CYCLE   (0.5            ), // Duty cycle for CLKOUT outputs (0.001-0.999). 
        .CLKOUT3_DUTY_CYCLE   (0.5            ), // Duty cycle for CLKOUT outputs (0.001-0.999). 
        .CLKOUT4_DUTY_CYCLE   (0.5            ), // Duty cycle for CLKOUT outputs (0.001-0.999). 
        .CLKOUT5_DUTY_CYCLE   (0.5            ), // Duty cycle for CLKOUT outputs (0.001-0.999). 
        .CLKOUT6_DUTY_CYCLE   (0.5            ), // Duty cycle for CLKOUT outputs (0.001-0.999). 
        
        .CLKOUT0_PHASE        (0.0            ), // Phase offset for CLKOUT0 
        .CLKOUT1_PHASE        (0.0            ), // Phase offset for CLKOUT outputs (-360.000-360.000). 
        .CLKOUT2_PHASE        (0.0            ), // Phase offset for CLKOUT outputs (-360.000-360.000). 
        .CLKOUT3_PHASE        (0.0            ), // Phase offset for CLKOUT outputs (-360.000-360.000). 
        .CLKOUT4_PHASE        (0.0            ), // Phase offset for CLKOUT outputs (-360.000-360.000). 
        .CLKOUT5_PHASE        (0.0            ), // Phase offset for CLKOUT outputs (-360.000-360.000).
        .CLKOUT6_PHASE        (0.0            ), // Phase offset for CLKOUT outputs (-360.000-360.000). 
        
        .SS_EN                ("FALSE"        ), // Enables spread spectrum 
        .SS_MODE              ("CENTER_HIGH"  ), // Spread spectrum frequency deviation and the spread type 
        .SS_MOD_PERIOD        (10000          ), // Spread spectrum modulation period (ns) 
        
        .CLKFBOUT_USE_FINE_PS ("FALSE"        ), // Fine phase shift enable (TRUE/FALSE)   
        .CLKOUT0_USE_FINE_PS  ("FALSE"        ), // Fine phase shift enable (TRUE/FALSE) 
        .CLKOUT1_USE_FINE_PS  ("FALSE"        ), // Fine phase shift enable (TRUE/FALSE) 
        .CLKOUT2_USE_FINE_PS  ("FALSE"        ), // Fine phase shift enable (TRUE/FALSE) 
        .CLKOUT3_USE_FINE_PS  ("FALSE"        ), // Fine phase shift enable (TRUE/FALSE) 
        .CLKOUT4_USE_FINE_PS  ("FALSE"        ), // Fine phase shift enable (TRUE/FALSE) 
        .CLKOUT5_USE_FINE_PS  ("FALSE"        ), // Fine phase shift enable (TRUE/FALSE) 
        .CLKOUT6_USE_FINE_PS  ("FALSE"        ), // Fine phase shift enable (TRUE/FALSE) 
        
        .IS_CLKFBIN_INVERTED  (1'b0           ), // Optional inversion for CLKFBIN 
        .IS_CLKIN1_INVERTED   (1'b0           ), // Optional inversion for CLKIN1 
        .IS_CLKIN2_INVERTED   (1'b0           ), // Optional inversion for CLKIN2 
        .IS_CLKINSEL_INVERTED (1'b0           ), // Optional inversion for CLKINSEL 
        .IS_PSEN_INVERTED     (1'b0           ), // Optional inversion for PSEN 
        .IS_PSINCDEC_INVERTED (1'b0           ), // Optional inversion for PSINCDEC 
        .IS_PWRDWN_INVERTED   (1'b0           ), // Optional inversion for PWRDWN 
        .IS_RST_INVERTED      (1'b0           )  // Optional inversion for RST 
      )
      ULTRASCALE_P_MMCME4_ADV
      ( 
        .CLKIN1       (MMCM_CLKIN_1     ), //  1-bit  input: Primary clock 
        .CLKIN2       (MMCM_CLKIN_2     ), //  1-bit  input: Secondary clock 
        .CLKINSEL     (MMCM_CLKIN_SEL   ), //  1-bit  input: Clock select, High=CLKIN1 Low=CLKIN2 
        
        .PWRDWN       (PWRDWN           ), //  1-bit  input: Power-down   
        .RST          (RST              ), //  1-bit  input: Reset
        .LOCKED       (MMCM_LOCKED      ), //  1-bit output: LOCK 
        
        .CLKOUT0      (MMCM_CLKOUT[1]   ), //  1-bit output: CLKOUT0 
        .CLKOUT0B     (MMCM_CLKOUTB[1]  ), //  1-bit output: Inverted CLKOUT0 
        .CLKOUT1      (MMCM_CLKOUT[2]   ), //  1-bit output: CLKOUT1 
        .CLKOUT1B     (MMCM_CLKOUTB[2]  ), //  1-bit output: Inverted CLKOUT1 
        .CLKOUT2      (MMCM_CLKOUT[3]   ), //  1-bit output: CLKOUT2 
        .CLKOUT2B     (MMCM_CLKOUTB[3]  ), //  1-bit output: Inverted CLKOUT2 
        .CLKOUT3      (MMCM_CLKOUT[4]   ), //  1-bit output: CLKOUT3 
        .CLKOUT3B     (MMCM_CLKOUTB[4]  ), //  1-bit output: Inverted CLKOUT3 
        .CLKOUT4      (MMCM_CLKOUT[5]   ), //  1-bit output: CLKOUT4 
        .CLKOUT5      (MMCM_CLKOUT[6]   ), //  1-bit output: CLKOUT5 
        .CLKOUT6      (MMCM_CLKOUT[7]   ), //  1-bit output: CLKOUT6 
        
        .CLKFBOUT     (MMCM_CLKFB_OUT   ), //  1-bit output: Feedback clock 
        .CLKFBOUTB    (MMCM_CLKFB_OUTB  ), //  1-bit output: Inverted CLKFBOUT 
        .CLKFBIN      (MMCM_CLKFB_IN    ), //  1-bit  input: Feedback clock 
        
        .DCLK         (DCLK             ), //  1-bit  input: DRP clock 
        .DEN          (DEN              ), //  1-bit  input: DRP enable 
        .DWE          (DWE              ), //  1-bit  input: DRP write enable 
        .DADDR        (DADDR            ), //  7-bit  input: DRP address 
        .DI           (DI               ), // 16-bit  input: DRP data input 
        .DRDY         (DRDY             ), //  1-bit output: DRP ready 
        .DO           (DO               ), // 16-bit output: DRP data output 
        
        .PSCLK        (PSCLK            ), //  1-bit  input: Phase shift clock 
        .PSEN         (PSEN             ), //  1-bit  input: Phase shift enable 
        .PSINCDEC     (PSINCDEC         ), //  1-bit  input: Phase shift increment/decrement 
        .PSDONE       (PSDONE           ), //  1-bit output: Phase shift done 
        
        .CLKFBSTOPPED (CLKFBSTOPPED     ), //  1-bit output: Feedback clock stopped 
        .CLKINSTOPPED (CLKINSTOPPED     ), //  1-bit output: Input clock stopped 
        
        .CDDCREQ      (CDDCREQ          ), //  1-bit  input: Request to dynamic divide clock 
        .CDDCDONE     (CDDCDONE         )  //  1-bit output: Clock dynamic divide done 
      );
    end
  endgenerate
  
  
  
  
endmodule
