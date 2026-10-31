`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/30/2026 07:19:53 PM
// Design Name: 
// Module Name: top_UART_Test
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


module top_UART_Test
  (
    input  logic       SYS_CLK_P  , // 
    input  logic       SYS_CLK_N  , // 
    input  logic       UART_RX_i  , // 
    output logic       UART_TX_o  , // 
    input  logic [3:0] KEY_i      , // 
    output logic [3:0] LED_o        // 
  );
  
  parameter real FREQ_SYSCLK     =  50e6    ; // System clock frequency (in Hz)
  parameter bit  MODE_BAUDCTRL   = 1'b1     ; // Baudrate control mode: '1' - fixed, '0' - variable
  parameter int  BAUD_UART_DEF   = 256000   ; // UART baudrate (for fixed mode)
  parameter int  MAX_DW          = 8        ; // Maximum data bits in a word
  parameter bit  MODE_FULLDUPLEX = 1'b1     ; // Enable full-duplex mode
  parameter bit  EN_RECEIVER     = 1'b1     ; // Enable receiver circuitry generation
  parameter bit  EN_TRANSMITTER  = 1'b1     ; // Enable transmitter circuitry generation
  parameter bit  MODE_RE_POL     = 1'b1     ; // RE port polarity: '1' - active high / '0' - active low
  parameter bit  MODE_DE_POL     = 1'b1     ; // DE port polarity: '1' - active high / '0' - active low
  parameter bit  MODE_VAR_WORD   = 1'b0     ; // '0' - Word size == MAX_DW, '1' - Word size <= MAX_DW, controlled by DataBits_i
  parameter bit  MODE_MSB        = 1'b0     ; // '0' - LSB (default), '1' - MSB
  parameter bit  MODE_VAR_PAR    = 1'b0     ; // '0' - Parity type == MODE_TYP_PAR, '1' - Parity type is controlled by ParityType_i
  parameter int  MODE_TYP_PAR    = 1        ; // Parity type (in fixed mode): '0' - N, '1' - E, '2' - O, '3' - M, '4' - S.
  parameter bit  MODE_VAR_STP    = 1'b0     ; // '0' - Stop bits == MODE_TYP_STP, '1' - Stop bits <= 3, is controlled by StopBits_i
  parameter int  MODE_TYP_STP    = 1        ; // Maximum stop bits in a word
  
  logic                        SYS_CLK      ; // 
  logic                        SYS_Locked   ; // 
  logic                        SYS_RST      ; // 
  logic                        DataValid_i  ; // 
  logic [          MAX_DW-1:0] Data_i       ; // 
  logic                        DataValid_o  ; // 
  logic [          MAX_DW-1:0] Data_o       ; // 
  logic                        RX_Active_o  ; // 
  logic                        ParityError_o; // 
  logic                        FrameError_o ; // 
  logic                        TX_Active_o  ; // 
  logic                        BuffFull_o   ; // 
  
  assign SYS_RST = ~SYS_Locked;
  /*
  always_comb
  begin
    //LED_o[1:0] =  KEY_i[1:0];
    //LED_o[3:2] = ~KEY_i[3:2];
    //LED_o[0] = 1'b0;
    //LED_o[1] = RX_Active_o;
    //LED_o[2] = TX_Active_o;
    //LED_o[3] = FrameError_o;
  end
  */
  always_ff @(posedge SYS_CLK)
  begin
    if(SYS_RST)
    begin
      DataValid_i <= 1'b0        ; // 
      Data_i      <= '{default:0}; // 
      LED_o       <= '{default:0}; // 
    end
    else
    begin
      DataValid_i <= DataValid_o ; // 
      Data_i      <= Data_o      ; // 
      LED_o[0]    <= RX_Active_o ;
      LED_o[1]    <= TX_Active_o ;
      if(ParityError_o) LED_o[2] <= ~LED_o[2];
      if(FrameError_o ) LED_o[3] <= ~LED_o[3];
    end
  end
  
   SYSCLK_Wiz MainClock
   (
    .clk_out1  (SYS_CLK        ), // 
    .locked    (SYS_Locked     ), // 
    .clk_in1_p (SYS_CLK_P      ), // 
    .clk_in1_n (SYS_CLK_N      )  // 
   );
  
  Interface_UART
  #(
    .FREQ_SYSCLK     (FREQ_SYSCLK     ), // System clock frequency (in Hz)
    .MODE_BAUDCTRL   (MODE_BAUDCTRL   ), // Baudrate control mode: '1' - fixed, '0' - variable
    .BAUD_UART_DEF   (BAUD_UART_DEF   ), // UART baudrate (for fixed mode)
    .MAX_DW          (MAX_DW          ), // Maximum data bits in a word
    .MODE_FULLDUPLEX (MODE_FULLDUPLEX ), // Enable full-duplex mode
    .EN_RECEIVER     (EN_RECEIVER     ), // Enable receiver circuitry generation
    .EN_TRANSMITTER  (EN_TRANSMITTER  ), // Enable transmitter circuitry generation
    .MODE_RE_POL     (MODE_RE_POL     ), // RE port polarity: '1' - active high / '0' - active low
    .MODE_DE_POL     (MODE_DE_POL     ), // DE port polarity: '1' - active high / '0' - active low
    .MODE_VAR_WORD   (MODE_VAR_WORD   ), // '0' - Word size == MAX_DW, '1' - Word size <= MAX_DW, controlled by DataBits_i
    .MODE_MSB        (MODE_MSB        ), // '0' - LSB (default), '1' - MSB
    .MODE_VAR_PAR    (MODE_VAR_PAR    ), // '0' - Parity type == MODE_TYP_PAR, '1' - Parity type is controlled by ParityType_i
    .MODE_TYP_PAR    (MODE_TYP_PAR    ), // Parity type (in fixed mode): '0' - N, '1' - E, '2' - O, '3' - M, '4' - S.
    .MODE_VAR_STP    (MODE_VAR_STP    ), // '0' - Stop bits == MODE_TYP_STP, '1' - Stop bits <= 3, is controlled by StopBits_i
    .MODE_TYP_STP    (MODE_TYP_STP    )  // Maximum stop bits in a word
  )
  UART_RXTX
  (
    .RST_i         (SYS_RST          ), // System reset
    .CLK_i         (SYS_CLK          ), // System clock
    .DataValid_i   (DataValid_i      ), // Data Valid to TX
    .Data_i        (Data_i           ), // Data to TX
    .DataValid_o   (DataValid_o      ), // Data Valid from RX
    .Data_o        (Data_o           ), // Data from RX
    .BaudSel_i     (                 ), // Baudrate selector
    .DataBits_i    (                 ), // Number of data bits to transmit/receive
    .ParityType_i  (                 ), // Parity type selector
    .StopBits_i    (                 ), // Number of stop bits to transmit/receive
    .RX_Active_o   (RX_Active_o      ), // RX module is not IDLE
    .ParityError_o (ParityError_o    ), // Parity mismatch encountered during reception
    .FrameError_o  (FrameError_o     ), // Frame mismatch encountered during reception
    .TX_Active_o   (TX_Active_o      ), // TX module is not IDLE
    .BuffFull_o    (BuffFull_o       ), // TX message pre-buffer is full
    .RE_o          (                 ), // Receiver    enable (for external ICs)
    .DE_o          (                 ), // Transmitter enable (for external ICs)
    .RX_i          (UART_RX_i        ), // Receiver    input
    .TX_o          (UART_TX_o        )  // Transmitter output
  );
  
  
  
  
  
  
  
  
endmodule
