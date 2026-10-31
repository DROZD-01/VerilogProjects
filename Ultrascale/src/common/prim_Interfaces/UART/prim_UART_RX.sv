`timescale 1ns / 1ps
/***************************************************************************************************
* Company: 
* Engineer: 
* 
* Create Date: 10/14/2026 06:21:14 PM
* Design Name: 
* Module Name: prim_UART_RX
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

module prim_UART_RX
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
    input  logic                        RX_i          , // Serial data input
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
    output logic                        ParityError_o , // 
    output logic                        FrameError_o  , // 
    // Output
    output logic                        DataValid_o   , // 
    output logic [          MAX_DW-1:0] Data_o          // 
  );
  
  localparam int SAMP_W  = $clog2(OVERSAMPL)              ; // Sample counter width
  localparam int FILT_W  = 1 + OVERSAMPL/4                ; // Sample filter width (16 -> 5, 8 -> 3, 4 -> 2)
  localparam int FILT_LL = (OVERSAMPL/2 - 1) - OVERSAMPL/8; // Sample filter lower limit (middle - shift)
  localparam int FILT_UL = (OVERSAMPL/2 - 1) + OVERSAMPL/8; // Sample filter upper limit (middle + shift)
  
  function automatic logic MajorityVote_f (logic [FILT_W-1:0] v);
    int unsigned ones = 0;
    for (int k = 0; k < FILT_W - 1; k++) ones += v[k];
    return (2*ones > FILT_W); // ties (even N) resolve to 0; use >= to bias to 1
  endfunction
  
  typedef enum logic [2:0]
  {
    RX_IDLE   = 3'd0, // 
    RX_START  = 3'd1, // 
    RX_DATA   = 3'd2, // 
    RX_PARITY = 3'd3, // 
    RX_STOP   = 3'd4  // 
  } StateRX_t;
  
  typedef struct
  {
    logic              CE    ;
    logic              L     ;
    logic              SLI   ;
    logic              SRI   ;
    logic              LEFT  ;
    logic [MAX_DW-1:0] D     ;
    logic [MAX_DW-1:0] Q     ;
  } sreg_t;
  
  logic [SAMP_W-1:0] TickCounter    ; // Tick-per-bit counter
  logic [FILT_W-1:0] SampFilt       ; // Multiple samples for filtration
  logic              SampActive     ; // Active sampling period
  logic              SampLast       ; // Last sampling period
  logic              BitValueValid  ; // Sampled bit valid
  logic              BitValue       ; // Sampled bit value
  
  logic [       2:0] Pipeline_RX    ; // 
  logic              PacketStart    ; // 
  
  always_comb
  begin
    SampActive  = TickPulse_i & ((TickCounter >= FILT_LL) & (TickCounter <= FILT_UL));
    SampLast    = TickPulse_i & (TickCounter == FILT_UL);
    PacketStart = Pipeline_RX[1] ^ Pipeline_RX[0];
  end
  
  always_ff @(posedge CLK_i)
  begin:rxpipe_proc
    if(RST_i)
      Pipeline_RX   <= '{default:1};
    else
      Pipeline_RX <= {Pipeline_RX[1:0], RX_i};
  end
  
  always_ff @(posedge CLK_i)
  begin:bitsamp_proc
    if(RST_i || ~Active_o)
    begin
      TickCounter   <= '{default:0};
      SampFilt      <= '{default:0};
      BitValueValid <= 1'b0;
      BitValue      <= 1'b0;
    end
    else
    begin
      if(TickPulse_i) TickCounter <= TickCounter + 1'd1;
      if(SampActive ) SampFilt    <= {SampFilt[FILT_W-2:0], Pipeline_RX[2]};
      
      BitValueValid <= SampLast;
      if(SampLast   ) BitValue    <= MajorityVote_f(SampFilt);
    end
  end
  
  
  StateRX_t                    SM         ; // Main state machine
  // Data reception
  logic [$clog2(MAX_DW+1)-1:0] BitCount   ; // Data bit Count
  logic                        BitDone    ; // All data bits received
  // Parity verification
  logic                        ParityAcc  ; // Odd/Even parity accumulator
  logic [                 2:0] ParityType ; // Parity type selector
  // Stop bit verification
  logic                        StopDone   ; // 
  logic [                 1:0] StopCount  ; // 
  // Received data buffer
  logic [          MAX_DW-1:0] RecvData   ; // 
  sreg_t                       Recv       ; // 
  
  always_comb
  begin
    BitDone    = (MODE_VAR_WORD)? (BitCount == DataBits_i) : (BitCount == MAX_DW);
    ParityType = (MODE_VAR_PAR )? (ParityType_i) : (MODE_TYP_PAR);
    StopDone   = (MODE_VAR_STP )? (StopCount == StopBits_i) : (StopCount == MODE_TYP_STP);
    Active_o   = ~(SM == RX_IDLE);
  end
  
  always_comb
  begin:recv_proc
    Recv.CE   = ((SM == RX_DATA) & BitValueValid); // 
    Recv.L    = ((SM == RX_IDLE) & PacketStart  ); // 
    Recv.SLI  = (MODE_MSB)? BitValue : 1'b0      ; // 
    Recv.SRI  = (MODE_MSB)? 1'b0     : BitValue  ; // 
    Recv.LEFT = (MODE_MSB)? 1'b1     : 1'b0      ; // 
    Recv.D    = '{default:0}                     ; // 
    RecvData  = Recv.Q                           ; // 
  end
  
  prim_SRxRLED
  #(
    .DATA_W  (MAX_DW), // Shift register length
    .RST_POL (1'b0  ), // Default value of shift register
    .CLK_POL (1'b0  )  // '0' - posedge, '1' - negedge
  )
  Recv_Shift
  (
    .C    (CLK_i     ), // Clock
    .R    (RST_i     ), // Reset
    .CE   (Recv.CE   ), // Clock Enable
    .L    (Recv.L    ), // Load Enable
    .SLI  (Recv.SLI  ), // Shift-left  IN
    .SRI  (Recv.SRI  ), // Shift-right IN
    .LEFT (Recv.LEFT ), // Left('1', Qx <- Q0) / Right('0', Qx -> Q0)
    .D    (Recv.D    ), // Parallel Input
    .Q    (Recv.Q    ), // Parallel Output
    .SLO  (          ), // Shift-left  OUT
    .SRO  (          )  // Shift-right OUT
  );
  
  always_ff @(posedge CLK_i)
  begin:main_proc
    if(RST_i)
    begin
      Resync_o      <= 1'b0;
      
      BitCount      <= '{default:0};
      ParityAcc     <= 1'b0;
      StopCount     <= '{default:0};
      
      ParityError_o <= 1'b0;
      FrameError_o  <= 1'b0;
      
      SM <= RX_IDLE;
    end
    else
    begin
      Resync_o      <= 1'b0;
      ParityError_o <= 1'b0;
      FrameError_o  <= 1'b0;
      
      case(SM)
        (RX_IDLE   ):
        begin
          if(PacketStart && ~Inhibit_i)
          begin
            Resync_o    <= 1'b1;
            ParityAcc   <= 1'b0;
            BitCount    <= '{default:0};
            StopCount   <= '{default:0};
            SM <= RX_START;
          end
        end
        
        (RX_START  ):
        begin
          if(BitValueValid)
          begin
            if(BitValue != 1'b0) FrameError_o  <= 1'b1;
            SM <= RX_DATA;
          end
        end
        
        (RX_DATA   ):
        begin
          if(BitDone)
            SM <= RX_PARITY;
          else if(BitValueValid)
          begin
            ParityAcc <= ParityAcc ^ BitValue;
            BitCount  <= BitCount + 1'd1;
          end
        end
        
        (RX_PARITY ):
        begin
          case(ParityType)
            (3'd1 ): 
            if(BitValueValid)
            begin
              ParityError_o <= (BitValue ==  ParityAcc)? 1'b0 : 1'b1; // Even Parity
              SM <= RX_STOP;
            end
            (3'd2 ): 
            if(BitValueValid)
            begin
              ParityError_o <= (BitValue == ~ParityAcc)? 1'b0 : 1'b1; // Odd Parity
              SM <= RX_STOP;
            end
            (3'd3 ): 
            if(BitValueValid)
            begin
              ParityError_o <= (BitValue ==  1'b1     )? 1'b0 : 1'b1; // Mark Parity
              SM <= RX_STOP;
            end
            (3'd4 ): 
            if(BitValueValid)
            begin
              ParityError_o <= (BitValue ==  1'b0     )? 1'b0 : 1'b1; // Space Parity
              SM <= RX_STOP;
            end
            default: 
            begin
              ParityError_o <= 1'b0; // No Parity
              SM <= RX_STOP;
            end
          endcase
        end
        
        (RX_STOP   ):
        begin
          if(StopDone)
          begin
            SM <= RX_IDLE;
          end
          else if(BitValueValid)
          begin
            if(BitValue != 1'b1) FrameError_o  <= 1'b1;
            StopCount <= StopCount + 1'd1;
          end
        end
        
        default:;
      endcase
    end
  end
  
  generate
    if(MODE_VAR_WORD == 1'b1)
    begin
      // Post-reception data shift
      logic                        ShiftEnable; // 
      logic                        ShiftDone  ; // LSB post-reception shift value
      logic [$clog2(MAX_DW+1)-1:0] ShiftCount ; // LSB post-reception shift counter
      logic [          MAX_DW-1:0] ShiftData  ; // 
      sreg_t                       Shift      ; // 
      
      always_comb
      begin
        ShiftDone = (MODE_MSB)? (1'b1) : (ShiftCount == (MAX_DW - BitCount));
        Shift.CE   = (ShiftEnable & ~ShiftDone);
        Shift.L    = (SM == RX_STOP) & StopDone;
        Shift.SLI  = 1'b0      ;
        Shift.SRI  = 1'b0      ;
        Shift.LEFT = 1'b0      ;
        Shift.D    = RecvData  ;
        ShiftData  = Shift.Q   ;
      end
      
      prim_SRxRLED
      #(
        .DATA_W  (MAX_DW   ), // Shift register length
        .RST_POL (1'b0     ), // Default value of shift register
        .CLK_POL (1'b0     )  // '0' - posedge, '1' - negedge
      )
      LSB_Shift
      (
        .C    (CLK_i      ), // Clock
        .R    (RST_i      ), // Reset
        .CE   (Shift.CE   ), // Clock Enable
        .L    (Shift.L    ), // Load Enable
        .SLI  (Shift.SLI  ), // Shift-left  IN
        .SRI  (Shift.SRI  ), // Shift-right IN
        .LEFT (Shift.LEFT ), // Left('1', Qx <- Q0) / Right('0', Qx -> Q0)
        .D    (Shift.D    ), // Parallel Input
        .Q    (Shift.Q    ), // Parallel Output
        .SLO  (           ), // Shift-left  OUT
        .SRO  (           )  // Shift-right OUT
      );
      
      always_ff @(posedge CLK_i)
      begin:datashift_proc
        if(RST_i)
        begin
          ShiftEnable <= 1'b0;
          ShiftCount  <= '{default:0};
          DataValid_o <= 1'b0;
          Data_o      <= '{default:0};
        end
        else
        begin
          DataValid_o   <= 1'b0;
          
          casex({Shift.L, ShiftEnable, ShiftDone})
            (3'b1xx): // Latch Recv, enable shifting
            begin
              ShiftCount  <= '{default:0};
              ShiftEnable <= 1'b1;
            end
            (3'b010):
            begin
              ShiftCount <= ShiftCount + 1'd1;
            end
            (3'b011):
            begin
              ShiftEnable <= 1'b0;
              DataValid_o <= 1'b1;
              Data_o      <= ShiftData;
            end
            default:;
          endcase
        end
      end
      
    end
    else
    begin
      always_ff @(posedge CLK_i)
      begin:dataout_proc
        if(RST_i)
        begin
          DataValid_o <= 1'b0;
          Data_o      <= '{default:0};
        end
        else
        begin
          DataValid_o   <= 1'b0;
          
          // Latch Recv, enable shifting
          if((SM == RX_STOP) & StopDone)
          begin
            DataValid_o <= 1'b1;
            Data_o      <= RecvData;
          end
        end
      end
    end
  endgenerate
  
  ILA_UART_RX ILA_RX
  (
    .clk     (CLK_i         ),
    .probe0  (RX_i          ),
    .probe1  (TickPulse_i   ),
    .probe2  (PacketStart   ),
    .probe3  (SampActive    ),
    .probe4  (SampLast      ),
    .probe5  (BitValueValid ),
    .probe6  (BitValue      ),
    .probe7  (SampFilt      ),
    .probe8  (SM            ),
    .probe9  (BitDone       ),
    .probe10 (ParityAcc     ),
    .probe11 (StopDone      ),
    .probe12 (StopCount     ),
    .probe13 (RecvData      ),
    .probe14 (Inhibit_i     ),
    .probe15 (ParityError_o ),
    .probe16 (FrameError_o  ),
    .probe17 (BitCount      ),
    .probe18 (TickCounter   )
  );
  
endmodule