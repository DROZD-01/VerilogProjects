`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/14/2026 04:52:13 PM
// Design Name: 
// Module Name: prim_UART_TX
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

module prim_UART_TX
  #(
    parameter  int OVERSAMPL     = 16   , // Oversampling mode (ticks per bit)
    parameter  bit MODE_VAR_WORD = 1'b0 , // '0' - Word size == MAX_DW, '1' - Word size <= MAX_DW, controlled by DataBits_i
    parameter  int MAX_DW        = 8    , // Maximum data bits in a word
    parameter  bit MODE_MSB      = 1'b0 , // '0' - LSB (default), '1' - MSB
    parameter  bit MODE_VAR_PAR  = 1'b0 , // '0' - Parity type == MODE_TYP_PAR, '1' - Parity type is controlled by ParityType_i
    parameter  int MODE_TYP_PAR  = 1    , // Parity type (in fixed mode): '0' - N, '1' - E, '2' - O, '3' - M, '4' - S.
    parameter  bit MODE_VAR_STP  = 1'b0 , // '0' - Stop bits == MODE_TYP_STP, '1' - Stop bits <= 3, is controlled by StopBits_i
    parameter  int MODE_TYP_STP  = 1      // Maximum stop bits in a word
  )
  (
    input  logic                        RST_i         , // System reset
    input  logic                        CLK_i         , // System clock
    output logic                        TX_o          , // Serial data output
    // Baudrate timing
    input  logic                        TickPulse_i   , // Tick pulse input
    output logic                        Resync_o      , // 
    // Settings
    input  logic [$clog2(MAX_DW+1)-1:0] DataBits_i    , // 
    input  logic [                 2:0] ParityType_i  , // 
    input  logic [                 1:0] StopBits_i    , // 
    // Indicators
    input  logic                        Inhibit_i     , // Inhibits work of the main SM
    output logic                        Active_o      , // Main SM is NOT IDLE
    output logic                        BuffFull_o    , // Message pre-buffer is full
    // Input
    input  logic                        DataValid_i   , // 
    input  logic [          MAX_DW-1:0] Data_i          // 
  );
  
  localparam int SAMP_W    = $clog2(OVERSAMPL); // Sample counter width
  localparam int FILT_NULL = 0                ; //
  
  typedef enum logic [2:0]
  {
    TX_IDLE   = 3'd0, // 
    TX_START  = 3'd1, // 
    TX_DATA   = 3'd2, // 
    TX_PARITY = 3'd3, // 
    TX_STOP   = 3'd4  // 
  } StateTX_t;
  
  typedef struct
  {
    logic              CE    ;
    logic              L     ;
    logic              SLI   ;
    logic              SRI   ;
    logic              LEFT  ;
    logic [MAX_DW-1:0] D     ;
    logic [MAX_DW-1:0] Q     ;
  } sregi_t;
  
  typedef struct
  {
    logic              CE    ;
    logic              L     ;
    logic              SLI   ;
    logic              SRI   ;
    logic              LEFT  ;
    logic [MAX_DW-1:0] D     ;
    logic              SLO   ;
    logic              SRO   ;
  } srego_t;
  
  logic [SAMP_W-1:0] TickCounter    ; // Tick-per-bit counter
  logic              SampFirst      ; // Last sampling period
  logic              BitValueValid  ; // Sampled bit valid
  
  always_comb
  begin
    SampFirst = TickPulse_i & (TickCounter == FILT_NULL);
  end
  
  always_ff @(posedge CLK_i)
  begin:bitsamp_proc
    if(RST_i)
    begin
      TickCounter   <= '{default:0};
      BitValueValid <= 1'b0;
    end
    else
    begin
      if(TickPulse_i) TickCounter <= TickCounter + 1'd1;
      BitValueValid <= SampFirst;
    end
  end
  
  StateTX_t                    SM         ; // Main state machine
  logic                        Proceed_EN ; // Enable transmission continuation
  // Data reception
  logic [$clog2(MAX_DW+1)-1:0] BitCount   ; // Data bit Count
  logic                        BitValue   ; // Output bit value
  logic                        BitDone    ; // All data bits transmitted
  // Parity verification
  logic                        ParityAcc  ; // Odd/Even parity accumulator
  logic [                 2:0] ParityType ; // Parity type selector
  // Stop bit verification
  logic                        StopDone   ; // 
  logic [                 1:0] StopCount  ; // 
  // Pre-transmission buffer
  logic                        BuffDone  ; // Shifting is ready for data grab
  logic                        BuffReady ; // All necessary shifts are complete
  logic                        BuffReset ; // Make Shifting ready for new input data
  logic [          MAX_DW-1:0] BuffData  ; // 
  sregi_t                      Buff      ; // 
  // Transmit data buffer
  srego_t                      Trsm       ; // 
  
  always_comb
  begin
    BitDone    = (MODE_VAR_WORD)? (BitCount == DataBits_i) : (BitCount == MAX_DW);
    ParityType = (MODE_VAR_PAR )? (ParityType_i) : (MODE_TYP_PAR);
    StopDone   = (MODE_VAR_STP )? (StopCount == StopBits_i) : (StopCount == MODE_TYP_STP);
    Active_o   = ~(SM == TX_IDLE);
  end
  
  generate
    if(MODE_VAR_WORD == 1'b1)
    begin
      // Pre-transmission data buffer + shift
      logic                        BuffEnable; // Shifting is active 
      logic [$clog2(MAX_DW+1)-1:0] BuffCount ; // 
      
      always_comb
      begin
        BuffReady  = (MODE_MSB)? (BuffCount == (MAX_DW - BitCount)) : (1'b1);
        Buff.CE    = BuffEnable & ~BuffReady;
        Buff.L     = DataValid_i & ~(BuffEnable | BuffDone);
        Buff.SLI   = 1'b0   ;
        Buff.SRI   = 1'b0   ;
        Buff.LEFT  = 1'b1   ;
        Buff.D     = Data_i ;
        BuffData   = Buff.Q;
        BuffFull_o = BuffDone | BuffEnable;
      end
      
      always_ff @(posedge CLK_i)
      begin:datashift_proc
        if(RST_i || BuffReset)
        begin
          BuffDone   <= 1'b0;
          BuffEnable <= 1'b0;
          BuffCount  <= '{default:0};
        end
        else
        begin
          casex({BuffReady, BuffEnable, DataValid_i})
            3'b11x: {BuffDone, BuffEnable} <= 2'b10;
            3'b101: {BuffDone, BuffEnable} <= 2'b10;
            3'b01x: BuffCount  <= BuffCount + 1'd1;
            3'b001: BuffEnable <= 1'b1;
            default:;
          endcase
        end
      end
      
    end
    else
    begin
      // Pre-transmission data buffer
      always_comb
      begin
        Buff.CE    = 1'b0   ;
        Buff.L     = DataValid_i & ~BuffDone;
        Buff.SLI   = 1'b0   ;
        Buff.SRI   = 1'b0   ;
        Buff.LEFT  = 1'b0   ;
        Buff.D     = Data_i ;
        BuffData   = Buff.Q ;
        BuffFull_o = BuffDone;
      end
      
      always_ff @(posedge CLK_i)
      begin:datashift_proc
        if(RST_i || BuffReset)
          BuffDone <= 1'b0;
        else if(~BuffDone & DataValid_i)
          BuffDone <= 1'b1;
      end
      
    end
  endgenerate
  
  prim_SRxRLED
  #(
    .DATA_W  (MAX_DW   ), // Shift register length
    .RST_POL (1'b0     ), // Default value of shift register
    .CLK_POL (1'b0     )  // '0' - posedge, '1' - negedge
  )
  LSB_Shift
  (
    .C    (CLK_i     ), // Clock
    .R    (RST_i     ), // Reset
    .CE   (Buff.CE   ), // Clock Enable
    .L    (Buff.L    ), // Load Enable
    .SLI  (Buff.SLI  ), // Shift-left  IN
    .SRI  (Buff.SRI  ), // Shift-right IN
    .LEFT (Buff.LEFT ), // Left('1', Qx <- Q0) / Right('0', Qx -> Q0)
    .D    (Buff.D    ), // Parallel Input
    .Q    (Buff.Q    ), // Parallel Output
    .SLO  (          ), // Shift-left  OUT
    .SRO  (          )  // Shift-right OUT
  );
  
  
  always_comb
  begin:trsm_proc
    Trsm.CE   = ((SM == TX_DATA) & BitValueValid         ); // 
    Trsm.L    = ((SM == TX_IDLE) & BuffDone & ~Inhibit_i ); // 
    Trsm.SLI  = 1'b0                                      ; // 
    Trsm.SRI  = 1'b0                                      ; // 
    Trsm.LEFT = (MODE_MSB)? 1'b1 : 1'b0                   ; // 
    Trsm.D    = BuffData                                  ; // 
    BitValue  = (MODE_MSB)? Trsm.SLO : Trsm.SRO           ; // 
  end
  
  prim_SRxRLED
  #(
    .DATA_W  (MAX_DW), // Shift register length
    .RST_POL (1'b0  ), // Default value of shift register
    .CLK_POL (1'b0  )  // '0' - posedge, '1' - negedge
  )
  Trsm_Shift
  (
    .C    (CLK_i     ), // Clock
    .R    (RST_i     ), // Reset
    .CE   (Trsm.CE   ), // Clock Enable
    .L    (Trsm.L    ), // Load Enable
    .SLI  (Trsm.SLI  ), // Shift-left  IN
    .SRI  (Trsm.SRI  ), // Shift-right IN
    .LEFT (Trsm.LEFT ), // Left('1', Qx <- Q0) / Right('0', Qx -> Q0)
    .D    (Trsm.D    ), // Parallel Input
    .Q    (          ), // Parallel Output
    .SLO  (Trsm.SLO  ), // Shift-left  OUT
    .SRO  (Trsm.SRO  )  // Shift-right OUT
  );
  
  always_ff @(posedge CLK_i)
  begin:main_proc
    if(RST_i)
    begin
      Resync_o      <= 1'b0;
      Proceed_EN    <= 1'b0;
      
      BuffReset     <= 1'b0;
      BitCount      <= '{default:0};
      ParityAcc     <= 1'b0;
      StopCount     <= '{default:0};
      
      TX_o          <= 1'b1;
      
      SM <= TX_IDLE;
    end
    else
    begin
      BuffReset     <= 1'b0;
      Resync_o      <= 1'b0;
      
      case(SM)
        (TX_IDLE   ): 
        begin
          if(BitValueValid)
            Proceed_EN <= 1'b0;
          
          if(BuffDone & ~Inhibit_i)
          begin
            Resync_o  <= ~Proceed_EN;
            BuffReset <= 1'b1;
            ParityAcc <= 1'b0;
            BitCount  <= '{default:0};
            StopCount <= '{default:0};
            SM <= TX_START;
          end
        end
        
        (TX_START  ): 
        begin
          if(BitValueValid)
          begin
            TX_o <= 1'b0;
            SM   <= TX_DATA;
          end
        end
        
        (TX_DATA   ): 
        begin
          if(BitDone)
            SM <= TX_PARITY;
          else if(BitValueValid)
          begin
            TX_o      <= BitValue;
            ParityAcc <= ParityAcc ^ BitValue;
            BitCount  <= BitCount + 1'd1;
          end
        end
        
        (TX_PARITY ): 
        begin
          case(ParityType)
            (3'd1 ): 
            if(BitValueValid)
            begin
              TX_o <= ParityAcc; // Even Parity
              SM <= TX_STOP;
            end
            (3'd2 ): 
            if(BitValueValid)
            begin
              TX_o <= ~ParityAcc; // Odd Parity
              SM <= TX_STOP;
            end
            (3'd3 ): 
            if(BitValueValid)
            begin
              TX_o <= 1'b1; // Mark Parity
              SM <= TX_STOP;
            end
            (3'd4 ): 
            if(BitValueValid)
            begin
              TX_o <= 1'b0; // Space Parity
              SM <= TX_STOP;
            end
            default: 
            begin
              // No Parity
              SM <= TX_STOP;
            end
          endcase
        end
        
        (TX_STOP   ): 
        begin
          if(StopDone)
          begin
            Proceed_EN <= 1'b1;
            SM <= TX_IDLE;
          end
          else if(BitValueValid)
          begin
            TX_o      <= 1'b1;
            StopCount <= StopCount + 1'd1;
          end
        end
        
        default:;
      endcase
    end
  end
  
endmodule
