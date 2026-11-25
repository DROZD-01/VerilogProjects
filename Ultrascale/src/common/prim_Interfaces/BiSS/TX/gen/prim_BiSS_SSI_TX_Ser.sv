`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/26/2026 
// Design Name: 
// Module Name: prim_BiSS_SSI_TX_Ser
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: SL frame engine of the BiSS-C/SSI transmitter (slave).
//              Generates the sensor data frame (SCD frame) of the "BiSS C
//              Protocol Description" on SLO, launched at every MA edge of the
//              configured launch polarity:
//                BiSS: LAT -> ACK='0' -> STR='1' -> CDS -> PDU[MSB..LSB] ->
//                      CRC[MSB..LSB] (inverted) -> STOP='0' at STP -> fill '0'
//                SSI : PDU[MSB..LSB] -> fill '0'   (no header, no CRC)
//              Timing-critical paths are kept shallow for high FMAX:
//              - The SL launch path is only  edge-sync -> bit-index mux -> SL
//                register (slave delay D = 3 CLK).
//              - The frame word is pre-assembled in a wide shift register: the
//                PDU is captured at the CDS launch edge (= LAT + 2T, matching
//                the protocol minimum processing time tbusy of two clocks), the
//                CRC engines run one beat in the background and patch the CRC
//                field into the tail of the shift register two CLK later, so
//                the CRC math never sits in the launch path.
//              - The CDS bit is a registered value that was already decided at
//                the previous frame's timeout (control communication responds
//                at the beginning of the next BiSS cycle), so it is stable for
//                the whole frame.
//              Over-clocking beyond STP is answered with '0' (timeout bits) and
//              optionally flagged after "SET_Limit_HalfMA_i" extra clocked bit slots.
//              A timeout while the frame is still incomplete aborts it and
//              raises the frame watchdog error.
// 
// Dependencies: prim_CRC_Calculator
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module prim_BiSS_SSI_TX_Ser
  #(
    parameter int                   MAX_PDUBIT     = 32           , // Widest process-data payload [bits]
    parameter int                   VAL_CRCBIT     = 6            , // BiSS-C CRC width [bits]
    parameter logic [VAL_CRCBIT-1:0] CRC_POLY      = 'h2F         , // Polynome, excluding the x^CRC_WIDTH term
    parameter logic [VAL_CRCBIT-1:0] CRC_INIT      = 'h2A         , // Register seed
    parameter logic [VAL_CRCBIT-1:0] CRC_XOR_OUT   = 'h15         , // Final XOR
    parameter bit                   CRC_TX_INVERT  = 1'b1         , // Official protocol: CRC is inverted before transmission
    parameter int                   WIDTH_PDUBIT   = $clog2(32+1) , // PDU width field width [bits]
    parameter int                   WIDTH_LIMHALFM = 16             // Over-clock tolerance width [h[launch edges]
  )
  (
    input  logic                  RST_i                , // 
    input  logic                  CLK_i                , // 
    // Settings (snapshot domain)
    input  logic                  SET_InterfaceMode_i  , // '1' - SSI, '0' - BiSS
    input  logic                  SET_EN_BiSS_CRC_i    , // BiSS: append CRC
    input  logic                  SET_CRC_InclCDS_i    , // 1: CRC covers CDS bit as well as PDU
    input  logic                  SET_Lvl_SL_Idle_i    , // SL level while idle
    input  logic [WIDTH_PDUBIT-1:0] SET_PDUBits_i      , // PDU width [bits]
    input  logic [WIDTH_LIMHALFM-1:0] SET_Limit_HalfMA_i, // Over-clock tolerance [h[launch edges], 0 = infinite
    input  logic                  SET_FSM_Enable_i     , // Global enable (snapshot domain)
    input  logic                  SET_FSM_Continuous_i , // Answer every MA burst forever
    input  logic                  StartPending_i       , // One-shot exchange armed
    input  logic                  CDM_Busy_i           , // Control-communication transaction active
    input  logic                  EN_PDUCH_i           , // Process-data channels active (short frame if '0')
    // Process data in
    input  logic [MAX_PDUBIT-1:0] Send_PDUBits_i       , // MSB-justified PDU (first bit on the wire is bit MAX_PDUBIT-1)
    output logic                  Send_Latched_o       , // Pulse: PDU sampled for the running frame
    // Control communication
    input  logic                  CDS_Bit_i            , // CDS bit of this frame (stable from frame start)
    // MA monitor events
    input  logic                  MA_LaunchEdge_i      , // Comb strobe: MA edge the TX launches on
    input  logic                  FR_Start_i           , // Comb strobe: first launch edge after re-arm (LAT)
    input  logic                  TO_Expire_i          , // Pulse: slave timeout expired
    // Outputs
    output logic                  FR_Rsp_o             , // Pulse: this frame is being responded to
    output logic                  FR_End_o             , // Pulse: frame ended (completed or aborted)
    output logic                  Err_WdExp_o          , // Pulse: frame incomplete at timeout expiry
    output logic                  Err_Overrun_o        , // Pulse: MA clocked beyond the frame limit
    output logic                  SS_SL                  // Master <- Slave data line
  );
  
  localparam int SR_W          = MAX_PDUBIT + VAL_CRCBIT;          // Frame shift register width
  localparam int MAX_FRAMEBIT  = 3 + MAX_PDUBIT + VAL_CRCBIT;      // ACK + STR + CDS + PDU + CRC
  localparam int WIDTH_FRAMEBIT = $clog2(MAX_FRAMEBIT + 1);
  
  //--------------------------------------------------------------------
  // Frame geometry
  //--------------------------------------------------------------------
  
  logic [WIDTH_FRAMEBIT-1:0] FrameLen; // Total launched bits incl. header, excl. STOP
  
  always_comb
  begin
    if(SET_InterfaceMode_i)
      FrameLen = WIDTH_FRAMEBIT'(SET_PDUBits_i); // SSI: raw data bits
    else
      FrameLen = WIDTH_FRAMEBIT'(3 + (EN_PDUCH_i? SET_PDUBits_i : 'd0) 
                                   + ((EN_PDUCH_i && (SET_PDUBits_i != 'd0) 
                                       && SET_EN_BiSS_CRC_i)? VAL_CRCBIT : 'd0));
  end
  
  // PDU sample edge index (BiSS: CDS launch edge = LAT + 2T = protocol
  // minimum processing time; SSI: frame start edge)
  logic [WIDTH_FRAMEBIT-1:0] LatchIdx;
  always_comb
    LatchIdx = (SET_InterfaceMode_i)? WIDTH_FRAMEBIT'(0) : WIDTH_FRAMEBIT'(2);
  
  // Frame completion index: BiSS ends with the STOP bit launched at STP
  // (index FrameLen); SSI has no stop bit, the frame ends with the last
  // data bit (index FrameLen-1), the monoflop timeout closes the frame.
  logic [WIDTH_FRAMEBIT-1:0] DoneIdx;
  always_comb
    DoneIdx = (SET_InterfaceMode_i && (FrameLen != 'd0))? (FrameLen - 1'd1) : FrameLen;
  
  //--------------------------------------------------------------------
  // Frame sequencing
  //--------------------------------------------------------------------
  
  typedef enum logic[1:0] {ST_IDLE, ST_ACTIVE, ST_FILL} State_t;
  
  State_t              State     ; // 
  logic [SR_W-1:0]     FrameSR   ; // Remaining PDU+CRC bits, MSB = next bit on the wire
  logic [WIDTH_FRAMEBIT-1:0] BitIdx    ; // Index of the bit launched at the NEXT launch edge
  logic [WIDTH_FRAMEBIT-1:0] EffIdx    ; // Effective bit index (0 while armed in ST_IDLE)
  logic [WIDTH_LIMHALFM-1:0] OverCnt  ; // Extra clocked bit slots after STP
  logic                OvrErrDone; // 
  logic                Rsp_Go    ; // This frame gets an answer
  logic                NextBit   ; // Bit to launch at the current launch edge
  logic                LatchStrobe; // Launch edge that captures the PDU
  logic                PatchPend  ; // CRC ready, patch the frame tail on the next CLK
  logic [VAL_CRCBIT-1:0] CRC_PDU    ; // CRC over PDU only
  logic [VAL_CRCBIT-1:0] CRC_CDS    ; // CRC over CDS + PDU
  logic [VAL_CRCBIT-1:0] CRC_Sel    ; // Selected raw CRC
  logic [VAL_CRCBIT-1:0] CRC_Wire   ; // CRC as transmitted on the wire
  logic                CRC_Clear  ; // Per-frame reseed of the engines
  
  assign Rsp_Go = SET_FSM_Enable_i && (SET_FSM_Continuous_i || StartPending_i || CDM_Busy_i);
  
  always_comb
    EffIdx = (State == ST_ACTIVE)? BitIdx : 'd0;
  
  // Next bit to launch
  always_comb
  begin
    if(EffIdx >= FrameLen)
      NextBit = 1'b0;                                   // STOP / timeout fill
    else if(SET_InterfaceMode_i)
      NextBit = (EffIdx == 'd0)? Send_PDUBits_i[MAX_PDUBIT-1] // SSI: live MSB at the frame start edge
                                : FrameSR[SR_W-1];     // SSI: shifted PDU
    else
      case(EffIdx)
        'd0    : NextBit = 1'b0;                       // ACK
        'd1    : NextBit = 1'b1;                       // STR
        'd2    : NextBit = CDS_Bit_i;                  // CDS
        default: NextBit = FrameSR[SR_W-1];            // PDU + CRC
      endcase
  end
  
  //--------------------------------------------------------------------
  // Frame shift register control: loaded at the PDU sample edge (fixed
  // shift, PDU MSB lands on the register MSB). The CRC engines are fed
  // serially: one 1-bit beat per launch edge while the PDU bits shift
  // out, so the covered bit count always equals the configured PDU width
  // (the CRC never sees the zero padding below a narrow PDU). The result
  // patches the CRC tail of FrameSR one CLK after the last PDU bit,
  // still a full MA period before those bits reach the launch edge.
  //--------------------------------------------------------------------
  
  logic PduRegion ; // Current launch edge shifts out a PDU bit
  logic CrcActive ; // CRC field transmitted in this frame
  logic CrcPduBeat; // Serial beat with the PDU bit being launched
  logic CrcCdsBeat; // Serial beat with the CDS bit (CDS+PDU engine only)
  logic LastPduBit; // Launch edge of the last PDU bit
  
  assign LatchStrobe = MA_LaunchEdge_i && ((State == ST_ACTIVE) || (FR_Start_i && Rsp_Go)) 
                       && (EffIdx == LatchIdx);
  assign CrcActive   = !SET_InterfaceMode_i && EN_PDUCH_i && SET_EN_BiSS_CRC_i 
                       && (SET_PDUBits_i != 'd0);
  assign PduRegion   = (EffIdx >= (LatchIdx + 1'd1)) 
                       && (EffIdx < (LatchIdx + 1'd1 + SET_PDUBits_i));
  assign CrcPduBeat  = MA_LaunchEdge_i && ((State == ST_ACTIVE) || (FR_Start_i && Rsp_Go)) 
                       && PduRegion && CrcActive;
  assign CrcCdsBeat  = LatchStrobe && CrcActive; // CDS bit, one beat ahead of the PDU
  assign LastPduBit  = CrcPduBeat && (EffIdx == (LatchIdx + SET_PDUBits_i));
  
  //--------------------------------------------------------------------
  // CRC engines (serial 1-bit beats, MSB first)
  //--------------------------------------------------------------------
  
  assign CRC_Sel   = SET_CRC_InclCDS_i? CRC_CDS : CRC_PDU;
  assign CRC_Wire  = CRC_TX_INVERT? ~CRC_Sel : CRC_Sel;
  // Reseed at every frame start so that an aborted frame (timeout mid-
  // frame) can never leave a partial fold in the engines
  assign CRC_Clear = RST_i || PatchPend || FR_Rsp_o;
  
  prim_CRC_Calculator
  #(
    .CRC_WIDTH   (VAL_CRCBIT       ), // CRC Width
    .DATA_WIDTH  (1                ), // One bit per beat (follows the shift register)
    .CRC_POLY    (CRC_POLY         ), // Polynome, excluding the x^CRC_WIDTH term
    .CRC_INIT    (CRC_INIT         ), // Register seed
    .CRC_XOR_OUT (CRC_XOR_OUT      )  // Final XOR
  )
  CRC_Engine_PDU
  (
    .CLK_i      (CLK_i            ), // System clock
    .RST_i      (CRC_Clear        ), // Synchronous active-high reset
    .DataValid_i(CrcPduBeat       ), // Data beat valid
    .Data_i     (FrameSR[SR_W-1]  ), // Data beat: the PDU bit being launched
    .CRC_o      (CRC_PDU          )  // Running/Final CRC
  );
  
  prim_CRC_Calculator
  #(
    .CRC_WIDTH   (VAL_CRCBIT       ), // CRC Width
    .DATA_WIDTH  (1                ), // One bit per beat
    .CRC_POLY    (CRC_POLY         ), // Polynome, excluding the x^CRC_WIDTH term
    .CRC_INIT    (CRC_INIT         ), // Register seed
    .CRC_XOR_OUT (CRC_XOR_OUT      )  // Final XOR
  )
  CRC_Engine_CDS
  (
    .CLK_i      (CLK_i                     ), // System clock
    .RST_i      (CRC_Clear                 ), // Synchronous active-high reset
    .DataValid_i(CrcPduBeat || CrcCdsBeat  ), // Data beat valid
    .Data_i     (CrcCdsBeat? CDS_Bit_i : FrameSR[SR_W-1]), // CDS bit, then the PDU bits
    .CRC_o      (CRC_CDS                   )  // Running/Final CRC
  );
  
  //--------------------------------------------------------------------
  // SL output register and frame FSM
  //--------------------------------------------------------------------
  
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i) 
    begin
      State        <= ST_IDLE;
      BitIdx       <= 'd0;
      OverCnt      <= 'd0;
      OvrErrDone   <= 1'b0;
      PatchPend    <= 1'b0;
      FrameSR      <= 'd0;
      SS_SL        <= 1'b1; // Protocol idle level
      FR_Rsp_o     <= 1'b0;
      FR_End_o     <= 1'b0;
      Send_Latched_o <= 1'b0;
      Err_WdExp_o  <= 1'b0;
      Err_Overrun_o <= 1'b0;
    end
    else
    begin
      FR_Rsp_o      <= 1'b0;
      FR_End_o      <= 1'b0;
      Send_Latched_o <= 1'b0;
      Err_WdExp_o   <= 1'b0;
      Err_Overrun_o <= 1'b0;
      
      //--------------------------------------------------------------
      // Frame shift register: load / CRC patch / shift
      //--------------------------------------------------------------
      if(LatchStrobe)
      begin
        // PDU MSB -> register MSB. In SSI mode the sample edge IS the first
        // data edge (its bit leaves through the live-MSB mux), so the
        // register is pre-shifted by one: its MSB must be the second bit.
        FrameSR      <= {Send_PDUBits_i, {VAL_CRCBIT{1'b0}}} 
                        << ((SET_InterfaceMode_i)? 1'd1 : 1'd0);
        Send_Latched_o <= 1'b1;
      end
      else if(PatchPend)
      begin
        FrameSR[SR_W-1 -: VAL_CRCBIT] <= CRC_Wire;
        PatchPend   <= 1'b0;
      end
      else if(MA_LaunchEdge_i && (State == ST_ACTIVE) && (EffIdx >= (LatchIdx + 1'd1)) 
              && (EffIdx < FrameLen))
        FrameSR <= FrameSR << 1;
      
      if(LastPduBit)
        PatchPend <= 1'b1;
      
      //--------------------------------------------------------------
      // FSM
      //--------------------------------------------------------------
      if(TO_Expire_i)
      begin
        // Timeout: back to Idle; SL returns to the idle level (rising edge
        // indicates the expired timeout to the master). A frame that is
        // still incomplete means MA stalled -> frame watchdog error.
        // Exception: a "frame" that never saw a second launch edge is the
        // phantom start a master creates when it releases MA to idle right
        // after a CDM='1' timeout instead of holding it until the next
        // frame (see "BiSS C Protocol Description", Hold-CDM); it is
        // discarded silently.
        SS_SL  <= SET_Lvl_SL_Idle_i;
        State  <= ST_IDLE;
        if((State == ST_ACTIVE) && (BitIdx > 'd1))
        begin
          FR_End_o   <= 1'b1;
          Err_WdExp_o <= 1'b1;
        end
      end
      else
      begin
        case(State)
          //-----------------------------------------------
          ST_IDLE:
          begin
            if(FR_Start_i && Rsp_Go)
            begin
              SS_SL    <= NextBit;  // ACK (BiSS) / PDU MSB (SSI)
              State    <= ST_ACTIVE;
              BitIdx   <= 'd1;
              OverCnt  <= 'd0;
              OvrErrDone <= 1'b0;
              FR_Rsp_o <= 1'b1;
            end
            else
              SS_SL  <= SET_Lvl_SL_Idle_i;
          end
          //-----------------------------------------------
          ST_ACTIVE:
          begin
            if(MA_LaunchEdge_i)
            begin
              SS_SL  <= NextBit;
              BitIdx <= BitIdx + 1'd1;
              if(EffIdx == DoneIdx)
              begin
                // Last bit launched (STOP at STP for BiSS, last data bit
                // for SSI); further clocks are answered '0'
                State    <= ST_FILL;
                FR_End_o <= 1'b1;
              end
            end
          end
          //-----------------------------------------------
          ST_FILL:
          begin
            if(MA_LaunchEdge_i)
            begin
              SS_SL <= 1'b0; // Timeout bits
              
              // Over-clock watchdog: each further launch edge is one extra
              // clocked bit slot beyond the frame
              if(!OvrErrDone && (SET_Limit_HalfMA_i != 'd0) 
                 && (OverCnt >= SET_Limit_HalfMA_i))
              begin
                Err_Overrun_o <= 1'b1;
                OvrErrDone    <= 1'b1;
              end
              OverCnt <= OverCnt + 1'd1;
            end
          end
          //-----------------------------------------------
          default:
          begin
            State <= ST_IDLE;
            SS_SL <= SET_Lvl_SL_Idle_i;
          end
        endcase
      end
    end
  end
  
endmodule
