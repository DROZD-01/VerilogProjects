`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/25/2026 06:21:14 PM
// Design Name: 
// Module Name: prim_CLK_Wizard_pkg
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


package prim_CLK_Wizard_pkg;
  
  typedef struct
  {
    real MAX;
    real MIN;
  } minmax_real_t;
  
  typedef struct
  {
    int MAX;
    int MIN;
  } minmax_int_t;
  
  // Input clock buffer types
  typedef enum bit [1:0]
  {
    ICB0 = 2'd0, // Single ended clock capable pin
    ICB1 = 2'd1, // Differential clock capable pin
    ICB2 = 2'd2, // Global buffer
    ICB3 = 2'd3  // No buffer
  } icb_e;
  
  // Output clock buffer types
  typedef enum bit [2:0]
  {
    OCB0 = 3'd0, // Buffer
    OCB1 = 3'd1, // Buffer with CE
    OCB2 = 3'd2, // BUFG
    OCB3 = 3'd3, // BUFGCE
    OCB4 = 3'd4, // BUFGCE DIV
    OCB5 = 3'd5  // No buffer
  } ocb_e;
  
  // Feedback clock connection types
  typedef enum bit [1:0]
  {
    FCB0 = 2'd0, // Automatic control On-Chip
    FCB1 = 2'd1, // Automatic control Off-Chip
    FCB2 = 2'd2, // User-controlled On-Chip
    FCB3 = 2'd3  // User-controlled Off-Chip
  } fcb_e;
  
  typedef bit [2:0] seq_e;
  
  /*
    '0' - connect directly to MMCM_LOCKED
    '1' - connect to output to clk_seq_x[1][7]
    '2' - connect to output to clk_seq_x[2][7]
    '3' - connect to output to clk_seq_x[3][7]
    '4' - connect to output to clk_seq_x[4][7]
    '5' - connect to output to clk_seq_x[5][7]
    '6' - connect to output to clk_seq_x[6][7]
    '7' - connect to output to clk_seq_x[7][7]
  */
  function seq_e [7:1] GenerateSequencer_f (input seq_e [7:1] MODE_SEQUENCER_C ,
                                            input bit   [7:1] EN_CLKOUT_X_C    );
    begin
      seq_e CurrSearchVal;
      bit   SolutionFound;
      
      for(int i = 1; i < 8; i = i + 'd1)
      begin // For every available output...
        if((EN_CLKOUT_X_C[i] == 1'b1) && MODE_SEQUENCER_C[i] > 3'd1)
        begin
          SolutionFound = 1'b0;
          for(int j = 1; j <8; j = j + 'd1)
          begin // ...We look for the first enabled output with a higher sequence priority
            if((MODE_SEQUENCER_C[j] == MODE_SEQUENCER_C[i] - 3'd1) && (EN_CLKOUT_X_C[j] == 1'b1) && !SolutionFound)
            begin
              SolutionFound        = 1'b1;
              GenerateSequencer_f[i] = j;
            end
          end
        end
        else // If the output is disabled or  - default to connecting it to MMCM_LOCKED
        begin
          GenerateSequencer_f[i] = 3'd0;
        end
      end
    end
  endfunction
  
  typedef struct
  {
    minmax_real_t FIN  ; // Maximum/Minimum input clock frequency
    minmax_real_t FVCO ; // Maximum/Minimum MMCM VCO frequency
    minmax_real_t FOUT ; // Maximum/Minimum output clock frequency
  } mmcm_sw_char_t;
  
  typedef struct
  {
    minmax_real_t FBO_MULT ; // CLKFBOUT_MULT_F  Max/Min
    minmax_int_t  FBO_DIV  ; // DIVCLK_DIVIDE    Max/Min
    minmax_real_t CLK0_DIV ; // CLKOUT0_DIVIDE_F Max/Min
    minmax_int_t  CLK1_DIV ; // CLKOUT1_DIVIDE   Max/Min
    minmax_int_t  CLK2_DIV ; // CLKOUT2_DIVIDE   Max/Min
    minmax_int_t  CLK3_DIV ; // CLKOUT3_DIVIDE   Max/Min
    minmax_int_t  CLK4_DIV ; // CLKOUT4_DIVIDE   Max/Min
    minmax_int_t  CLK5_DIV ; // CLKOUT5_DIVIDE   Max/Min
    minmax_int_t  CLK6_DIV ; // CLKOUT6_DIVIDE   Max/Min
  } mmcm_paramlim_t;
  
  typedef struct
  {
    real FBO_MULT ; // CLKFBOUT_MULT_F  
    int  FBO_DIV  ; // DIVCLK_DIVIDE    
    real CLK0_DIV ; // CLKOUT0_DIVIDE_F 
    int  CLK1_DIV ; // CLKOUT1_DIVIDE   
    int  CLK2_DIV ; // CLKOUT2_DIVIDE   
    int  CLK3_DIV ; // CLKOUT3_DIVIDE   
    int  CLK4_DIV ; // CLKOUT4_DIVIDE   
    int  CLK5_DIV ; // CLKOUT5_DIVIDE   
    int  CLK6_DIV ; // CLKOUT6_DIVIDE   
  } mmcm_param_t;
  
  typedef real clko_tfrq_t [7:1];
  typedef real clki_tfrq_t [2:1];
  
  function mmcm_sw_char_t MMCM_SwChar_f (input bit [1:15] [7:0] FPGA_FAMILY_C     ,
                                         input bit [1:7 ] [7:0] FPGA_SERIES_C     ,
                                         input bit [1:8 ] [7:0] FPGA_SPEEDGRADE_C );
    begin
      automatic mmcm_sw_char_t LIM = '{default:0};
      
      case(FPGA_FAMILY_C)
        ("ULTRASCALE_PLUS"):
        begin
          case(FPGA_SERIES_C)
            ("SPARTAN"):
            begin
              case(FPGA_SPEEDGRADE_C)
                ("085V-2"  ):  LIM = '{FIN:'{ 933.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 775.00e6, 6.25e6}};
                ("085V-1"  ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 667.00e6, 6.25e6}};
                ("072V-1"  ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 667.00e6, 6.25e6}};
                default: $error("[CLKWIZ] MMCM_SwChar_f: Incompatible FPGA_SPEEDGRADE in SPARTAN ULTRASCALE+");
              endcase
            end
            
            ("ARTIX"  ):
            begin
              case(FPGA_SPEEDGRADE_C)
                ("085V-2"  ):  LIM = '{FIN:'{ 933.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 775.00e6, 6.25e6}};
                ("085V-1"  ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 667.00e6, 6.25e6}};
                ("072V-1"  ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 667.00e6, 6.25e6}};
                default: $error("[CLKWIZ] MMCM_SwChar_f: Incompatible FPGA_SPEEDGRADE in ARTIX ULTRASCALE+");
              endcase
            end
            
            ("KINTEX" ):
            begin
              case(FPGA_SPEEDGRADE_C)
                ("090V-3"  ):  LIM = '{FIN:'{1066.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 891.00e6, 6.25e6}};
                ("085V-2"  ):  LIM = '{FIN:'{ 933.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 775.00e6, 6.25e6}};
                ("085V-1"  ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 667.00e6, 6.25e6}};
                ("072V-2"  ):  LIM = '{FIN:'{ 933.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 725.00e6, 6.25e6}};
                ("072V-1"  ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 667.00e6, 6.25e6}};
                default: $error("[CLKWIZ] MMCM_SwChar_f: Incompatible FPGA_SPEEDGRADE in KINTEX ULTRASCALE+");
              endcase
            end
            
            ("VIRTEX" ):
            begin
              case(FPGA_SPEEDGRADE_C)
                ("090V-3"  ):  LIM = '{FIN:'{1066.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 891.00e6, 6.25e6}};
                ("085V-2"  ):  LIM = '{FIN:'{ 933.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 775.00e6, 6.25e6}};
                ("085V-1"  ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 667.00e6, 6.25e6}};
                ("072V-2"  ):  LIM = '{FIN:'{ 933.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 725.00e6, 6.25e6}};
                default: $error("[CLKWIZ] MMCM_SwChar_f: Incompatible FPGA_SPEEDGRADE in VIRTEX ULTRASCALE+");
              endcase
            end
            
            ("ZYNQMP" ):
            begin
              case(FPGA_SPEEDGRADE_C)
                ("090V-3"  ):  LIM = '{FIN:'{1066.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 891.00e6, 6.25e6}};
                ("085V-2"  ):  LIM = '{FIN:'{ 933.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 775.00e6, 6.25e6}};
                ("085V-1"  ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 667.00e6, 6.25e6}};
                ("072V-2"  ):  LIM = '{FIN:'{ 933.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 725.00e6, 6.25e6}};
                ("072V-1"  ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 667.00e6, 6.25e6}};
                default: $error("[CLKWIZ] MMCM_SwChar_f: Incompatible FPGA_SPEEDGRADE in ZYNQMP ULTRASCALE+");
              endcase
            end

            ("ZYNQRF" ):
            begin
              case(FPGA_SPEEDGRADE_C)
                ("085V-2"  ):  LIM = '{FIN:'{ 933.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 775.00e6, 6.25e6}};
                ("085V-1"  ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 667.00e6, 6.25e6}};
                ("072V-2"  ):  LIM = '{FIN:'{ 933.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 725.00e6, 6.25e6}};
                ("072V-1"  ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1600.00e6, 800.00e6}, FOUT:'{ 667.00e6, 6.25e6}};
                default: $error("[CLKWIZ] MMCM_SwChar_f: Incompatible FPGA_SPEEDGRADE in ZYNQRF ULTRASCALE+");
              endcase
            end
            
            default: $error("[CLKWIZ] MMCM_SwChar_f: Incompatible FPGA_SERIES in ULTRASCALE+ family");
          endcase
        end
        
        ("ULTRASCALE"     ):
        begin
          case(FPGA_SERIES_C)
            ("KINTEX" ):
            begin
              case(FPGA_SPEEDGRADE_C)
                ("100V-3"  ):  LIM = '{FIN:'{1066.00e6, 10.00e6}, FVCO:'{1600.00e6, 600.00e6}, FOUT:'{ 850.00e6, 4.69e6}};
                ("095V-2"  ):  LIM = '{FIN:'{ 933.00e6, 10.00e6}, FVCO:'{1440.00e6, 600.00e6}, FOUT:'{ 725.00e6, 4.69e6}};
                ("095V-1L" ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1200.00e6, 600.00e6}, FOUT:'{ 630.00e6, 4.69e6}};
                ("095V-1"  ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1200.00e6, 600.00e6}, FOUT:'{ 630.00e6, 4.69e6}};
                ("090V-1L" ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1200.00e6, 600.00e6}, FOUT:'{ 630.00e6, 4.69e6}};
                default: $error("[CLKWIZ] MMCM_SwChar_f: Incompatible FPGA_SPEEDGRADE in KINTEX ULTRASCALE");
              endcase
            end
            
            ("VIRTEX" ):
            begin
              case(FPGA_SPEEDGRADE_C)
                ("100V-3"  ):  LIM = '{FIN:'{1066.00e6, 10.00e6}, FVCO:'{1600.00e6, 600.00e6}, FOUT:'{ 850.00e6, 4.69e6}};
                ("100V-1H" ):  LIM = '{FIN:'{ 933.00e6, 10.00e6}, FVCO:'{1440.00e6, 600.00e6}, FOUT:'{ 725.00e6, 4.69e6}};
                ("095V-2"  ):  LIM = '{FIN:'{ 933.00e6, 10.00e6}, FVCO:'{1440.00e6, 600.00e6}, FOUT:'{ 725.00e6, 4.69e6}};
                ("095V-1"  ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1200.00e6, 600.00e6}, FOUT:'{ 630.00e6, 4.69e6}};
                default: $error("[CLKWIZ] MMCM_SwChar_f: Incompatible FPGA_SPEEDGRADE in VIRTEX ULTRASCALE");
              endcase
            end
            
            default: $error("[CLKWIZ] MMCM_SwChar_f: Incompatible FPGA_SERIES in ULTRASCALE family");
          endcase
        end
        
        ("SERIES7"        ):
        begin
          case(FPGA_SERIES_C)
            ("SPARTAN"):
            begin
              case(FPGA_SPEEDGRADE_C)
                ("100V-2"  ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1440.00e6, 600.00e6}, FOUT:'{ 800.00e6, 4.69e6}};
                ("100V-1"  ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1200.00e6, 600.00e6}, FOUT:'{ 800.00e6, 4.69e6}};
                ("095V-1L" ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1200.00e6, 600.00e6}, FOUT:'{ 800.00e6, 4.69e6}};
                default: $error("[CLKWIZ] MMCM_SwChar_f: Incompatible FPGA_SPEEDGRADE in SPARTAN 7SERIES");
              endcase
            end
            
            ("ARTIX"  ):
            begin
              case(FPGA_SPEEDGRADE_C)
                ("100V-3"  ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1600.00e6, 600.00e6}, FOUT:'{ 800.00e6, 4.69e6}};
                ("100V-2LE"):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1440.00e6, 600.00e6}, FOUT:'{ 800.00e6, 4.69e6}};
                ("100V-2"  ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1440.00e6, 600.00e6}, FOUT:'{ 800.00e6, 4.69e6}};
                ("100V-1"  ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1200.00e6, 600.00e6}, FOUT:'{ 800.00e6, 4.69e6}};
                ("095V-1LI"):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1200.00e6, 600.00e6}, FOUT:'{ 800.00e6, 4.69e6}};
                ("090V-2LE"):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1200.00e6, 600.00e6}, FOUT:'{ 630.00e6, 4.69e6}};
                default: $error("[CLKWIZ] MMCM_SwChar_f: Incompatible FPGA_SPEEDGRADE in ARTIX 7SERIES");
              endcase
            end
            
            ("KINTEX" ):
            begin
              case(FPGA_SPEEDGRADE_C)
                ("100V-3"  ):  LIM = '{FIN:'{1066.00e6, 10.00e6}, FVCO:'{1600.00e6, 600.00e6}, FOUT:'{1066.00e6, 4.69e6}};
                ("100V-2LE"):  LIM = '{FIN:'{ 933.00e6, 10.00e6}, FVCO:'{1440.00e6, 600.00e6}, FOUT:'{ 933.00e6, 4.69e6}};
                ("100V-2"  ):  LIM = '{FIN:'{ 933.00e6, 10.00e6}, FVCO:'{1440.00e6, 600.00e6}, FOUT:'{ 933.00e6, 4.69e6}};
                ("100V-1LM"):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1200.00e6, 600.00e6}, FOUT:'{ 800.00e6, 4.69e6}};
                ("100V-1M" ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1200.00e6, 600.00e6}, FOUT:'{ 800.00e6, 4.69e6}};
                ("100V-1Q" ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1200.00e6, 600.00e6}, FOUT:'{ 800.00e6, 4.69e6}};
                ("100V-1"  ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1200.00e6, 600.00e6}, FOUT:'{ 800.00e6, 4.69e6}};
                ("095V-2LI"):  LIM = '{FIN:'{ 933.00e6, 10.00e6}, FVCO:'{1440.00e6, 600.00e6}, FOUT:'{ 933.00e6, 4.69e6}};
                ("090V-2LE"):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1200.00e6, 600.00e6}, FOUT:'{ 800.00e6, 4.69e6}};
                default: $error("[CLKWIZ] MMCM_SwChar_f: Incompatible FPGA_SPEEDGRADE in KINTEX 7SERIES");
              endcase
            end
            
            ("VIRTEX" ):
            begin
              case(FPGA_SPEEDGRADE_C)
                ("100V-3"  ):  LIM = '{FIN:'{1066.00e6, 10.00e6}, FVCO:'{1600.00e6, 600.00e6}, FOUT:'{1066.00e6, 4.69e6}};
                ("100V-2L" ):  LIM = '{FIN:'{ 933.00e6, 10.00e6}, FVCO:'{1440.00e6, 600.00e6}, FOUT:'{ 933.00e6, 4.69e6}};
                ("100V-2G" ):  LIM = '{FIN:'{ 933.00e6, 10.00e6}, FVCO:'{1440.00e6, 600.00e6}, FOUT:'{ 933.00e6, 4.69e6}};
                ("100V-2"  ):  LIM = '{FIN:'{ 933.00e6, 10.00e6}, FVCO:'{1440.00e6, 600.00e6}, FOUT:'{ 933.00e6, 4.69e6}};
                ("100V-1M" ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1200.00e6, 600.00e6}, FOUT:'{ 800.00e6, 4.69e6}};
                ("100V-1"  ):  LIM = '{FIN:'{ 800.00e6, 10.00e6}, FVCO:'{1200.00e6, 600.00e6}, FOUT:'{ 800.00e6, 4.69e6}};
                default: $error("[CLKWIZ] MMCM_SwChar_f: Incompatible FPGA_SPEEDGRADE in VIRTEX 7SERIES");
              endcase
            end
            
            default: $error("[CLKWIZ] MMCM_SwChar_f: Incompatible FPGA_SERIES in 7Series family");
          endcase
        end
        default: $error("[CLKWIZ] MMCM_SwChar_f: Incompatible FPGA_FAMILY");
      endcase
      
      MMCM_SwChar_f = LIM;
    end
  endfunction
  
  //------------------------------------------------------------------------------------------
  // Multiplier/Divider limits for MMCME2/3/4
  //------------------------------------------------------------------------------------------
  function mmcm_paramlim_t MMCM_ParamLim_f (input bit [1:15] [7:0] FPGA_FAMILY_C);
    begin
      automatic mmcm_paramlim_t LIM = '{default:0};
      
      case(FPGA_FAMILY_C)
        ("ULTRASCALE_PLUS"): // 
        LIM = '{FBO_MULT:'{128.000, 2.000},  FBO_DIV:'{106    , 1    }, CLK0_DIV:'{128.000, 1.000}, 
                CLK1_DIV:'{128    , 1    }, CLK2_DIV:'{128    , 1    }, CLK3_DIV:'{128    , 1    }, 
                CLK4_DIV:'{128    , 1    }, CLK5_DIV:'{128    , 1    }, CLK6_DIV:'{128    , 1    }};
        ("ULTRASCALE"     ): //
        LIM = '{FBO_MULT:'{ 64.000, 2.000},  FBO_DIV:'{106    , 1    }, CLK0_DIV:'{128.000, 1.000}, 
                CLK1_DIV:'{128    , 1    }, CLK2_DIV:'{128    , 1    }, CLK3_DIV:'{128    , 1    }, 
                CLK4_DIV:'{128    , 1    }, CLK5_DIV:'{128    , 1    }, CLK6_DIV:'{128    , 1    }};
        ("SERIES7"        ): //
        LIM = '{FBO_MULT:'{ 64.000, 2.000},  FBO_DIV:'{106    , 1    }, CLK0_DIV:'{128.000, 1.000}, 
                CLK1_DIV:'{128    , 1    }, CLK2_DIV:'{128    , 1    }, CLK3_DIV:'{128    , 1    }, 
                CLK4_DIV:'{128    , 1    }, CLK5_DIV:'{128    , 1    }, CLK6_DIV:'{128    , 1    }};
        default: $error("[CLKWIZ] MMCM_ParamLim_f: Incompatible FPGA_FAMILY");
      endcase
      
      MMCM_ParamLim_f = LIM;
    end
  endfunction
  
  //------------------------------------------------------------------------------------------
  // Calculates multiplier/divider values to achieve target CLKOUT frequencies
  //------------------------------------------------------------------------------------------
  function mmcm_param_t MMCM_ParamCalc_f (input bit [1:0]       WIZ_DEBUG_C   ,
                                          input mmcm_sw_char_t  MMCM_SWCHAR_C ,
                                          input mmcm_paramlim_t MMCM_PARLIM_C ,
                                          input bit [2:1]       EN_CLKIN_C    ,
                                          input clki_tfrq_t     FREQ_CLKIN_C  ,
                                          input bit [7:1]       EN_CLKOUT_C   ,
                                          input clko_tfrq_t     FREQ_CLKOUT_C );
    begin
      
      // 1) Check the input parameters for out-of-bounds parameters: 
      VerifyInputClocks_f(WIZ_DEBUG_C, MMCM_SWCHAR_C, MMCM_PARLIM_C, EN_CLKIN_C, FREQ_CLKIN_C);
      VerifyOutputClocks_f(WIZ_DEBUG_C, MMCM_SWCHAR_C, MMCM_PARLIM_C, EN_CLKOUT_C, FREQ_CLKOUT_C);
      
      // 2) Calculate lower and upper limits for FBO_DIV 
      //automatic minmax_int_t [2:1] LIM_FBO_DIV;
      for(int i = 1; i < 3; i = i + 'd1)
      begin // For every existing input...
        LIM_FBO_DIV[i].MIN = (EN_CLKIN_C == 1'b1)? $ceil((FREQ_CLKIN_C[i] * MMCM_PARLIM_C.FBO_MULT.MIN) / MMCM_SWCHAR_C.FVCO.MAX) : 0;
        LIM_FBO_DIV[i].MAX = (EN_CLKIN_C == 1'b1)? $ceil((FREQ_CLKIN_C[i] * MMCM_PARLIM_C.FBO_MULT.MAX) / MMCM_SWCHAR_C.FVCO.MAX) : 0;
      end
      
      // 2.1) If both inputs are enabled - use the tightest limit
      //automatic minmax_int_t ACT_FBO_DIV;
      case(EN_CLKIN_C)
        2'b11   : 
        begin
          ACT_FBO_DIV.MIN = (LIM_FBO_DIV[1].MIN >= LIM_FBO_DIV[2].MIN)? LIM_FBO_DIV[1].MIN : LIM_FBO_DIV[2].MIN; // 
          ACT_FBO_DIV.MAX = (LIM_FBO_DIV[1].MAX >= LIM_FBO_DIV[2].MAX)? LIM_FBO_DIV[2].MAX : LIM_FBO_DIV[1].MAX; // 
        end
        2'b01   : ACT_FBO_DIV = LIM_FBO_DIV[1]; // 
        2'b10   : ACT_FBO_DIV = LIM_FBO_DIV[2]; // 
        2'b00   : $warning("[CLKWIZ] MMCM_ParamCalc - EN_CLKIN_X: No clock inputs were enabled");
        default : $error  ("[CLKWIZ] MMCM_ParamCalc - EN_CLKIN_X has incorrect value");
      endcase
      
      //minmax_int_t  LIM_FBO_MULT;
      
      //LIM_FBO_DIV  = '{0.0, 0.0};
      //LIM_FBO_MULT = '{0  , 0  };
      
      
    end
  endfunction
  
  //------------------------------------------------------------------------------------------
  // Verifies that input parameters are within limits
  //------------------------------------------------------------------------------------------
  function void VerifyInputClocks_f(input bit [1:0]       WIZ_DEBUG_C   ,
                                    input mmcm_sw_char_t  MMCM_SWCHAR_C ,
                                    input mmcm_paramlim_t MMCM_PARLIM_C ,
                                    input bit [2:1]       EN_CLKIN_C    ,
                                    input clki_tfrq_t     FREQ_CLKIN_C  );
    begin
      bit ValueInRange;
      bit OutputIsEn  ;
      
      for(int i = 1; i < 3; i = i + 'd1)
      begin // For every existing input...
        ValueInRange = ValInRange_f(MMCM_SWCHAR_C.FIN.MAX, FREQ_CLKIN_C[i], MMCM_SWCHAR_C.FIN.MIN);
        OutputIsEn   = (EN_CLKIN_C[i] == 1'b1)? 1'b1 : 1'b0;
        
        case({ValueInRange, OutputIsEn})
          // Frequency is in range, input is enabled
          2'b11  : if(WIZ_DEBUG_C >= 3) $info   ("[CLKWIZ] VerInClocks: CLKIN%0d is in range and enabled", i); 
          // Frequency is not in range, input is enabled
          2'b01  : if(WIZ_DEBUG_C >= 0) $error  ("[CLKWIZ] VerInClocks: CLKIN%0d is out of range and enabled", i); 
          // Frequency is in range, input is not enabled
          2'b10  : if(WIZ_DEBUG_C >= 3) $info   ("[CLKWIZ] VerInClocks: CLKIN%0d is in range and disabled", i); 
          // Frequency is not in range, input is not enabled
          2'b00  : if(WIZ_DEBUG_C >= 2) $warning("[CLKWIZ] VerInClocks: CLKIN%0d is out of range and disabled", i); 
          default:                      $error  ("[CLKWIZ] VerInClocks: Input values are corrupted");
        endcase
      end
    end
  endfunction
  
  function void VerifyOutputClocks_f(input bit [1:0]       WIZ_DEBUG_C   ,
                                     input mmcm_sw_char_t  MMCM_SWCHAR_C ,
                                     input mmcm_paramlim_t MMCM_PARLIM_C ,
                                     input bit [7:1]       EN_CLKOUT_C   ,
                                     input clko_tfrq_t     FREQ_CLKOUT_C );
    begin
      bit ValueInRange;
      bit OutputIsEn  ;
      
      for(int i = 1; i < 8; i = i + 'd1)
      begin // For every existing output...
        ValueInRange = ValInRange_f(MMCM_SWCHAR_C.FOUT.MAX, FREQ_CLKOUT_C[i], MMCM_SWCHAR_C.FOUT.MIN);
        OutputIsEn   = (EN_CLKOUT_C[i] == 1'b1)? 1'b1 : 1'b0;
        
        case({ValueInRange, OutputIsEn})
          // Frequency is in range, output is enabled
          2'b11  : if(WIZ_DEBUG_C >= 3) $info   ("[CLKWIZ] VerOutClocks: CLKOUT%0d is in range and enabled", i); 
          // Frequency is not in range, output is enabled
          2'b01  : if(WIZ_DEBUG_C >= 0) $error  ("[CLKWIZ] VerOutClocks: CLKOUT%0d is out of range and enabled", i); 
          // Frequency is in range, output is not enabled
          2'b10  : if(WIZ_DEBUG_C >= 3) $info   ("[CLKWIZ] VerOutClocks: CLKOUT%0d is in range and disabled", i); 
          // Frequency is not in range, output is not enabled
          2'b00  : if(WIZ_DEBUG_C >= 2) $warning("[CLKWIZ] VerOutClocks: CLKOUT%0d is out of range and disabled", i); 
          default:                      $error  ("[CLKWIZ] VerOutClocks: Input values are corrupted");
        endcase
      end
    end
  endfunction
  
  function minmax_int_t GlobDivLimCalc_f();
    begin
      automatic minmax_int_t [2:1] LIM_FBO_DIV;
      for(int i = 1; i < 3; i = i + 'd1)
      begin // For every existing input...
        LIM_FBO_DIV[i].MIN = (EN_CLKIN_C == 1'b1)? $ceil((FREQ_CLKIN_C[i] * MMCM_PARLIM_C.FBO_MULT.MIN) / MMCM_SWCHAR_C.FVCO.MAX) : 0;
        LIM_FBO_DIV[i].MAX = (EN_CLKIN_C == 1'b1)? $ceil((FREQ_CLKIN_C[i] * MMCM_PARLIM_C.FBO_MULT.MAX) / MMCM_SWCHAR_C.FVCO.MAX) : 0;
      end
      
      // 2.1) If both inputs are enabled - use the tightest limit
      case(EN_CLKIN_C)
        2'b11   : 
        begin
          GlobDivLimCalc_f.MIN = (LIM_FBO_DIV[1].MIN >= LIM_FBO_DIV[2].MIN)? LIM_FBO_DIV[1].MIN : LIM_FBO_DIV[2].MIN; // 
          GlobDivLimCalc_f.MAX = (LIM_FBO_DIV[1].MAX >= LIM_FBO_DIV[2].MAX)? LIM_FBO_DIV[2].MAX : LIM_FBO_DIV[1].MAX; // 
        end
        2'b01   : GlobDivLimCalc_f = LIM_FBO_DIV[1]; // 
        2'b10   : GlobDivLimCalc_f = LIM_FBO_DIV[2]; // 
        2'b00   : $warning("[CLKWIZ] MMCM_ParamCalc - EN_CLKIN_X: No clock inputs were enabled");
        default : $error  ("[CLKWIZ] MMCM_ParamCalc - EN_CLKIN_X has incorrect value");
      endcase
    end
  endfunction
  
  function bit ValInRange_f (input HIGH_C, input VAL_C, input LOW_C);
    if((HIGH_C >= VAL_C) && (VAL_C >= LOW_C))
      ValInRange_f = 1'b1;
    else
      ValInRange_f = 1'b0;
  endfunction
  
  
  
endpackage
