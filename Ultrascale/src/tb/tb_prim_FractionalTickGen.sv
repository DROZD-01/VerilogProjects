`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/14/2026 12:16:32 AM
// Design Name: 
// Module Name: tb_prim_FractionalTickGen
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


module tb_prim_FractionalTickGen;
  
  parameter real TB_FREQ_SYSCLK = 100.0e6 ; // System clock frequency (in Hz)
  parameter bit  TB_MODE_FIXED  = 1'b1    ; // Generation mode:'1' - fixed tick rate, '0' - variable tick rate
  parameter int  TB_TICKR_FIXED = 921600  ; // Target tickrate (used in fixed mode)
  parameter int  TB_OVERSAMPL   = 16      ; // Oversampling mode (ticks per bit)
  parameter int  TB_LOGIC_DEL   = 2       ; // 
  
  logic        TB_RST_i       ; // System reset
  logic        TB_CLK_i       ; // System clock
  logic [ 4:0] TB_BaudSel_i   ; // 
  logic        TB_Resync_i    ; // Clear and start counting
  logic        TB_TickPulse_o ; // Tick pulse output
  
  parameter real TB_PERIOD_SYSCLK  = (1e+9/TB_FREQ_SYSCLK);
  parameter real TB_PERIOD_BITRATE = (1e+9/TB_TICKR_FIXED);
  
  initial 
  begin
    TB_CLK_i = 1'b0;
    forever 
    begin
      #(TB_PERIOD_SYSCLK/2);
      TB_CLK_i = ~TB_CLK_i;
    end
  end
  
  logic TB_RX_Gen;
  
  initial 
  begin
    TB_RX_Gen = 1'b1;
    @(negedge TB_RST_i);
    TB_RX_Gen = ~TB_RX_Gen;
    forever 
    begin
      #(TB_PERIOD_BITRATE);
      TB_RX_Gen = ~TB_RX_Gen;
    end
  end
  
  initial 
  begin
    TB_RST_i = 1'b1;
    #(100ns);
    @(negedge TB_CLK_i);
    TB_RST_i = 1'b0;
  end
  
  initial 
  begin
    TB_BaudSel_i = 5'd22;
    TB_Resync_i  = 1'b0 ;
    TB_Stop_i    = 1'b0 ;
    
    @(negedge TB_RST_i);
    @(posedge TB_CLK_i);
    TB_Resync_i = 1'b1 ;
    @(posedge TB_CLK_i);
    TB_Resync_i = 1'b0;
    
  end
  
  prim_FractionalTickGen
  #(
    .FREQ_SYSCLK (TB_FREQ_SYSCLK ), // System clock frequency (in Hz)
    .MODE_FIXED  (TB_MODE_FIXED  ), // Generation mode:'1' - fixed tick rate, '0' - variable tick rate
    .TICKR_FIXED (TB_TICKR_FIXED ), // Target tickrate (used in fixed mode)
    .OVERSAMPL   (TB_OVERSAMPL   ), // Oversampling mode (ticks per bit)
    .LOGIC_DEL   (TB_LOGIC_DEL   )  // 
  )
  UUT
  (
    .RST_i       (TB_RST_i      ), // System reset
    .CLK_i       (TB_CLK_i      ), // System clock
    .BaudSel_i   (TB_BaudSel_i  ), // 
    .Resync_i    (TB_Resync_i   ), // Clear and start counting
    .TickPulse_o (TB_TickPulse_o)  // Tick pulse output
  );
  
  
  
  
  
endmodule
