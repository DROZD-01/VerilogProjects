`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/24/2026 12:22:18 PM
// Design Name: 
// Module Name: tb_Interface_UART
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


module tb_Interface_UART;
  
  parameter real TB_FREQ_SYSCLK     = 100e6    ; // 
  parameter bit  TB_MODE_BAUDCTRL   = 1'b0     ; // 
  parameter int  TB_BAUD_UART_DEF   = 115200   ; // 
  parameter int  TB_MAX_DW          = 8        ; // 
  parameter bit  TB_MODE_FULLDUPLEX = 1'b0     ; // 
  parameter bit  TB_EN_RECEIVER     = 1'b1     ; // 
  parameter bit  TB_EN_TRANSMITTER  = 1'b1     ; // 
  parameter bit  TB_MODE_RE_POL     = 1'b0     ; // 
  parameter bit  TB_MODE_DE_POL     = 1'b1     ; // 
  parameter bit  TB_MODE_VAR_WORD   = 1'b0     ; // 
  parameter bit  TB_MODE_MSB        = 1'b0     ; // 
  parameter bit  TB_MODE_VAR_PAR    = 1'b0     ; // 
  parameter int  TB_MODE_TYP_PAR    = 1        ; // 
  parameter bit  TB_MODE_VAR_STP    = 1'b0     ; // 
  parameter int  TB_MODE_TYP_STP    = 1        ; // 
  
  logic                           TB_RST_i         ; // System reset
  logic                           TB_CLK_i         ; // System clock
  // Data I/O
  logic                           TB_DataValid_i   ; // 
  logic [          TB_MAX_DW-1:0] TB_Data_i        ; // 
  logic                           TB_DataValid_o   ; // 
  logic [          TB_MAX_DW-1:0] TB_Data_o        ; // 
  // Settings
  logic [                    4:0] TB_BaudSel_i     ; // 
  logic [$clog2(TB_MAX_DW+1)-1:0] TB_DataBits_i    ; // 
  logic [                    2:0] TB_ParityType_i  ; // 
  logic [                    1:0] TB_StopBits_i    ; // 
  // Indicators
  logic                           TB_RX_Active_o   ; // 
  logic                           TB_ParityError_o ; // 
  logic                           TB_FrameError_o  ; // 
  logic                           TB_TX_Active_o   ; // 
  logic                           TB_BuffFull_o    ; // 
  // External I/O
  logic                           TB_RE_o          ; // Receiver    enable (for external ICs)
  logic                           TB_DE_o          ; // Transmitter enable (for external ICs)
  logic                           TB_RX_i          ; // 
  logic                           TB_TX_o          ; // 
  
  
  Interface_UART
  #(
    .FREQ_SYSCLK     (TB_FREQ_SYSCLK     ), // System clock frequency (in Hz)
    .MODE_BAUDCTRL   (TB_MODE_BAUDCTRL   ), // Baudrate control mode: '1' - fixed, '0' - variable
    .BAUD_UART_DEF   (TB_BAUD_UART_DEF   ), // UART baudrate (for fixed mode)
    .MAX_DW          (TB_MAX_DW          ), // Maximum data bits in a word
    .MODE_FULLDUPLEX (TB_MODE_FULLDUPLEX ), // Enable full-duplex mode
    .EN_RECEIVER     (TB_EN_RECEIVER     ), // Enable receiver circuitry generation
    .EN_TRANSMITTER  (TB_EN_TRANSMITTER  ), // Enable transmitter circuitry generation
    .MODE_RE_POL     (TB_MODE_RE_POL     ), // RE port polarity: '1' - active high / '0' - active low
    .MODE_DE_POL     (TB_MODE_DE_POL     ), // DE port polarity: '1' - active high / '0' - active low
    .MODE_VAR_WORD   (TB_MODE_VAR_WORD   ), // '0' - Word size == MAX_DW, '1' - Word size <= MAX_DW, controlled by DataBits_i
    .MODE_MSB        (TB_MODE_MSB        ), // '0' - LSB (default), '1' - MSB
    .MODE_VAR_PAR    (TB_MODE_VAR_PAR    ), // '0' - Parity type == MODE_TYP_PAR, '1' - Parity type is controlled by ParityType_i
    .MODE_TYP_PAR    (TB_MODE_TYP_PAR    ), // Parity type (in fixed mode): '0' - N, '1' - E, '2' - O, '3' - M, '4' - S.
    .MODE_VAR_STP    (TB_MODE_VAR_STP    ), // '0' - Stop bits == MODE_TYP_STP, '1' - Stop bits <= 3, is controlled by StopBits_i
    .MODE_TYP_STP    (TB_MODE_TYP_STP    )  // Maximum stop bits in a word
  )
  UUT
  (
    .RST_i         (TB_RST_i         ), // System reset
    .CLK_i         (TB_CLK_i         ), // System clock
    .DataValid_i   (TB_DataValid_i   ), // Data Valid to TX
    .Data_i        (TB_Data_i        ), // Data to TX
    .DataValid_o   (TB_DataValid_o   ), // Data Valid from RX
    .Data_o        (TB_Data_o        ), // Data from RX
    .BaudSel_i     (TB_BaudSel_i     ), // Baudrate selector
    .DataBits_i    (TB_DataBits_i    ), // Number of data bits to transmit/receive
    .ParityType_i  (TB_ParityType_i  ), // Parity type selector
    .StopBits_i    (TB_StopBits_i    ), // Number of stop bits to transmit/receive
    .RX_Active_o   (TB_RX_Active_o   ), // RX module is not IDLE
    .ParityError_o (TB_ParityError_o ), // Parity mismatch encountered during reception
    .FrameError_o  (TB_FrameError_o  ), // Frame mismatch encountered during reception
    .TX_Active_o   (TB_TX_Active_o   ), // TX module is not IDLE
    .BuffFull_o    (TB_BuffFull_o    ), // TX message pre-buffer is full
    .RE_o          (TB_RE_o          ), // Receiver    enable (for external ICs)
    .DE_o          (TB_DE_o          ), // Transmitter enable (for external ICs)
    .RX_i          (TB_RX_i          ), // Receiver    input
    .TX_o          (TB_TX_o          )  // Transmitter output
  );
  
  
  
endmodule
