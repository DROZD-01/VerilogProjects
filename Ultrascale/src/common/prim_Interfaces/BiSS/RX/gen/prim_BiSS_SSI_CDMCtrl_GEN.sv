`timescale 1ns / 1ps
/***************************************************************************************************
* Company:
* Engineer:
*
* Create Date: 11/18/2026 08:28:26 PM
* Design Name:
* Module Name: prim_BiSS_SSI_CDMCtrl
* Project Name:
* Target Devices:
* Tool Versions:
* Description: BiSS-C control-data channel sequencer (CDM/CDS frame composer).
*
* The control communication is embedded into the regular BiSS frames: every frame
* transports one CDM bit in its timeout slot (encoded on MA by the frame
* controller) and receives one CDS bit in the CDS field of the slave response.
* The CDM/CDS bits of consecutive frames compose one control data frame as
* specified by the BiSS Interface C-mode protocol description:
*
*   frame header : S, CTS, {ID2..0 + ADR6..0}  or  {IDS0..7 + CMD1..0},
*                  CRC3..0 (CRC-4, polynomial x^4+x+1 = 0x13, inverted on the
*                  wire, covers CTS and the addressing bits, S excluded)
*   byte read    : R=1 W=0, S, 13x don't-care          (S repeated while busy)
*                  slave returns: R W echo, S, DATA7..0, CRC3..0, P
*   byte write   : R=0 W=1, S, DATA7..0, CRC3..0, P     (S repeated while busy)
*                  slave echoes:   R W S DATA CRC P
*   command      : CTS=0, S, 7 idle cycles, EX
*                  slave returns:  IDA0..7 acknowledge mask
*   broadcast    : CTS=0, IDS=00000000, S, EX           (no IDA bits)
*
* The addressed slave mirrors the R and W bits on CDS (an inverted W bit marks a
* rejected access), delays its S echo while it needs processing time, and closes
* every data transfer with the stop bit P ('0' = accepted, '1' = terminate). A
* control data frame is started only after RUN_REQ consecutive CDM='0' frames -
* the BiSS-C slave reset condition - and any access can be cancelled the same
* way ("sending CDM='0' for 14 BiSS cycles").
*
* Dependencies: prim_CRC_Calculator (serial CRC-4, polynomial 0x13)
*
* Revision:
* Revision 0.01 - File Created
* Additional Comments:
*
* Parameter definition:
*   MAX_CDSBIT   - width of the received CDS field; the BiSS-C control channel
*                  uses exactly one CDS bit per frame (1)
*   SLAVE_ID     - BiSS Slave ID addressed by register frames (0..7); in a
*                  point-to-point connection the single slave occupies ID 0
*   MAX_BUSY_CYC - number of unconfirmed S repetitions before the access is
*                  aborted with Err_CDM_o
*
***************************************************************************************************/

module prim_BiSS_SSI_CDMCtrl_GEN
  #(
    parameter int MAX_CDSBIT   = 1   , // CDS field width [bits]; BiSS-C control channel: 1
    parameter int WIDTH_CDADDR = 7   , // register address width [bits]
    parameter int WIDTH_CDDATA = 8   , // register data payload bits
    parameter int SLAVE_ID     = 0   , // BiSS Slave ID addressed by register frames
    parameter int MAX_BUSY_CYC = 64    // start-bit repetition budget [frames]
  )
  (
    input  logic                      RST_i               , // synchronous active-high reset
    input  logic                      CLK_i               , // system clock
    input  logic                      SET_FSM_Enable_i    , // global enable
    // Register access / command request (pulse; sampled when idle)
    input  logic                      COMM_Request_i      , // pulse: start a control data frame
    input  logic [1:0]                COMM_Type_i         , // 00 single-byte read, 01 single-byte write, 10 command, 11 broadcast command
    input  logic [WIDTH_CDADDR-1:0]   COMM_Addr_i         , // register address (read/write frames)
    input  logic [WIDTH_CDDATA-1:0]   COMM_Data_i        , // register write data (write frames)
    input  logic [7:0]                COMM_IDSel_i        , // command frame: linear-select slave-ID mask (bit 7 = IDS0, first on the wire)
    input  logic [1:0]                COMM_Cmd_i         , // command frame: command code (bit 1 = CMD1, first on the wire)
    // Frame events from the frame controller
    input  logic                      FR_Load_i           , // pulse: a frame was accepted
    input  logic                      FR_IsCDM_i          , // level: the accepted frame carries our CDM bit
    input  logic                      FR_Abort_i          , // pulse: frame aborted by the frame controller
    input  logic                      FR_End_i            , // pulse: frame finished (or aborted); the stream advances at FR_Load
    // RX result of the finished frame (latched by the RX before frame_end)
    input  logic [MAX_CDSBIT-1:0]     Recv_CDSBits_i      , // received CDS field of the previous frame
    // Towards the frame controller
    output logic                      REQ_CDM_o           , // level: request frames while an access is running
    output logic                      CDM_Bit_o           , // CDM bit encoded into the timeout slot of the next frame
    // Status
    output logic                      CDM_Busy_o          , // control access in progress
    output logic                      COMM_Reply_o        , // pulse: COMM_Data_o valid (read data / IDA mask)
    output logic [WIDTH_CDDATA-1:0]   COMM_Data_o         , // read data byte or command IDA acknowledge mask
    output logic                      Err_CDM_o           // pulse: access rejected, timed out or aborted
  );

  initial begin
    if(MAX_CDSBIT != 1)
      $error("[prim_BiSS_SSI_CDMCtrl]: the BiSS-C control channel uses a 1-bit CDS field (MAX_CDSBIT must be 1)");
    if(WIDTH_CDADDR != 7)
      $error("[prim_BiSS_SSI_CDMCtrl]: the BiSS-C register frame addresses 7 register bits (WIDTH_CDADDR must be 7)");
    if(WIDTH_CDDATA != 8)
      $error("[prim_BiSS_SSI_CDMCtrl]: the BiSS-C control frame carries one data byte (WIDTH_CDDATA must be 8)");
    if(SLAVE_ID < 0 || SLAVE_ID > 7)
      $error("[prim_BiSS_SSI_CDMCtrl]: SLAVE_ID must address one of the eight BiSS Slave IDs");
    if(MAX_BUSY_CYC < 1)
      $error("[prim_BiSS_SSI_CDMCtrl]: MAX_BUSY_CYC must be >= 1");
  end

  //------------------------------------------------------------------------------------------
  // Types and locals
  //------------------------------------------------------------------------------------------

  // request types
  typedef enum logic [1:0]
  {
    CT_RD    = 2'd0,  // single-byte register read frame
    CT_WR    = 2'd1,  // single-byte register write frame
    CT_CMD   = 2'd2,  // command frame addressed through the IDS mask
    CT_BCAST = 2'd3   // broadcast command frame (IDS forced to zero)
  } cc_type_e;

  // sequencer states
  typedef enum logic [3:0]
  {
    CC_IDLE  = 4'd0,  // wait for a request; track the CDM='0' idle run
    CC_PRE   = 4'd1,  // transmit CDM='0' frames until the slave-reset run is complete
    CC_HEAD  = 4'd2,  // frame header: S, CTS, addressing, inverted CRC-4 (16 bits)
    CC_RW    = 4'd3,  // register frames: R and W bits
    CC_START = 4'd4,  // start bit S, repeated while the slave reports busy
    CC_WDATA = 4'd5,  // write: DATA7..0, inverted CRC-4, stop bit P
    CC_RDATA = 4'd6,  // read: don't-care frames while DATA, CRC and P return
    CC_CWAIT = 4'd7,  // command: S followed by 7 idle cycles (IDA0..7 return)
    CC_EXEC  = 4'd8,  // command: execute bit EX
    CC_TAIL  = 4'd9,  // write: one drain frame until the P echo is readable
    CC_ABRT  = 4'd10  // post-error CDM='0' run (>= RUN_REQ) before returning to idle
  } cc_state_e;

  localparam logic [3:0] RUN_REQ   = 4'd14;         // consecutive CDM='0' frames required by the slave reset
  localparam logic [3:0] RUN_REQ_M1 = 4'd13;         // RUN_REQ - 1: the run value that completes the reset run
  localparam int BUS_W  = (MAX_BUSY_CYC > 1) ? $clog2(MAX_BUSY_CYC) : 1;
  localparam logic [BUS_W-1:0] BUSY_LIM = BUS_W'(MAX_BUSY_CYC-1);   // last unconfirmed-S count before the abort

  cc_type_e            Type_q    ;                 // latched request type
  cc_state_e           MainSM    ;                 // sequencer state
  logic [3:0]          Idx_q     ;                 // stream position of the bit encoded into the next frame
  cc_state_e           ChkPhase_q;                 // phase of the bit whose CDS response is readable now
  logic [3:0]          ChkIdx_q  ;                 // stream position of that bit

  logic [WIDTH_CDDATA-1:0] Data_q;                 // latched write data byte
  logic [11:0]         Hdr_q     ;                 // {S, CTS, addressing[9:0]} of the current frame header
  logic [11:0]         Echo_q    ;                 // write: echoes of DATA + CRC received so far
  logic [WIDTH_CDDATA-1:0] DataRX_q;               // read: received data byte
  logic [3:0]          CRCSr_q   ;                 // read: received (inverted) CRC bits
  logic [7:0]          IDA_q     ;                 // command: received IDA acknowledge bits
  logic [BUS_W-1:0]    Busy_q    ;                 // unconfirmed start-bit responses
  logic [3:0]          ZeroRun_q ;                 // consecutive CDM='0' frames (saturating)
  logic                CDM_Bit_q ;                 // CDM bit for the next frame
  logic                REQD_q    ;                 // frame request, delayed until CDM_Bit_q is settled
  logic [3:0]          CRCTX_o   ;                 // transmit CRC-4 check value (header / write data)
  logic [3:0]          CRCRX_o   ;                 // receive CRC-4 check value (read data)

  //------------------------------------------------------------------------------------------
  // Combinational: request decoding and stream events
  //------------------------------------------------------------------------------------------

  logic FrmCmt_c;                                   // this cycle commits one CDM bit into a frame
  logic Rbit_c, Wbit_c;                             // R/W bits of the current register frame
  logic S_Conf_c, S_Busy_c, BusyTO_c;               // start-bit echo tracking
  logic RW_Err_c;                                   // R/W echo mismatch (access rejected)
  logic W_Vfy_c, W_Err_c, W_Done_c;                 // write echo verification
  logic R_Vfy_c, R_Err_c, R_Done_c;                 // read DATA/CRC/P verification
  logic X_Done_c;                                   // command frame completed

  assign FrmCmt_c = FR_Load_i && FR_IsCDM_i;
  assign Rbit_c   = (Type_q == CT_RD);
  assign Wbit_c   = (Type_q == CT_WR);

  // Start-bit handshake: the response to an S bit transmitted one frame earlier is
  // readable now; '1' confirms the slave, '0' asks for processing time.
  assign S_Conf_c = FrmCmt_c && (MainSM == CC_START) && (ChkPhase_q == CC_START) && Recv_CDSBits_i[0];
  assign S_Busy_c = FrmCmt_c && (MainSM == CC_START) && (ChkPhase_q == CC_START) && ~Recv_CDSBits_i[0];
  assign BusyTO_c = S_Busy_c && (Busy_q >= BUSY_LIM);

  // Register frame supervision: the slave must mirror R and W identically; an
  // inverted W bit signals an unavailable address or a denied operation.
  assign RW_Err_c = FrmCmt_c && (ChkPhase_q == CC_RW) &&
                   ( (ChkIdx_q == 4'd0) ? (Recv_CDSBits_i[0] != Rbit_c) :
                                           (Recv_CDSBits_i[0] != Wbit_c) );

  // Write verification: the P echo becomes readable one frame after the stop bit.
  assign W_Vfy_c  = FrmCmt_c && (MainSM == CC_TAIL) && (ChkPhase_q == CC_WDATA) && (ChkIdx_q == 4'd12);
  assign W_Err_c  = W_Vfy_c && ( (Echo_q != {Data_q, ~CRCTX_o}) ||
                                 (Recv_CDSBits_i[0] != 1'b0) );
  assign W_Done_c = W_Vfy_c && ~W_Err_c;

  // Read verification: the response readable together with the last don't-care
  // bit is the slave's stop bit P; the CRC check value has been collected before.
  assign R_Vfy_c  = FrmCmt_c && (MainSM == CC_RDATA) && (Idx_q == 4'd12);
  assign R_Err_c  = R_Vfy_c && ( (CRCSr_q != ~CRCRX_o) ||
                                 (Recv_CDSBits_i[0] != 1'b0) );
  assign R_Done_c = R_Vfy_c && ~R_Err_c;

  // Command frame completed: the execute bit was committed (broadcast replies 0).
  assign X_Done_c = FrmCmt_c && (MainSM == CC_EXEC) && (Idx_q == 4'd0);

  //------------------------------------------------------------------------------------------
  // Combinational: next sequencer state / stream position
  //------------------------------------------------------------------------------------------

  cc_state_e MainSmNext_c;
  logic [3:0] IdxNext_c;

  always_comb
  begin
    MainSmNext_c = MainSM;
    IdxNext_c    = Idx_q;

    // request accept while idle: a control frame may start immediately if the
    // CDM='0' idle run is already long enough
    if ((MainSM == CC_IDLE) && SET_FSM_Enable_i && COMM_Request_i)
      MainSmNext_c = (ZeroRun_q >= RUN_REQ) ? CC_HEAD : CC_PRE;

    if (FrmCmt_c)
    begin
      if (RW_Err_c || W_Err_c || R_Err_c || BusyTO_c)
      begin
        // rejected / failed access: cancel through the CDM='0' reset run
        MainSmNext_c = CC_ABRT;
        IdxNext_c    = 4'd0;
      end
      else
      begin
        case (MainSM)
          CC_PRE, CC_ABRT:
          begin
            // the '0' run is complete when this commit reaches RUN_REQ frames.
            // CC_ABRT needs one extra frame: its first '0' cancels a slave
            // that is still inside a transaction (start-bit wait), and only
            // the following frames rebuild the slave's timeout run.
            if (MainSM == CC_PRE) begin
              if (ZeroRun_q >= RUN_REQ_M1)
              begin
                MainSmNext_c = CC_HEAD;
                IdxNext_c    = 4'd0;
              end
            end
            else begin
              if (ZeroRun_q >= RUN_REQ + 4'd1)
              begin
                MainSmNext_c = CC_IDLE;
                IdxNext_c    = 4'd0;
              end
            end
          end

          CC_HEAD:
          begin
            if (Idx_q == 4'd15)
            begin
              MainSmNext_c = Type_q[1] ? CC_CWAIT : CC_RW;
              IdxNext_c    = 4'd0;
            end
            else
              IdxNext_c    = Idx_q + 4'd1;
          end

          CC_RW:
          begin
            if (Idx_q == 4'd1)
            begin
              MainSmNext_c = CC_START;
              IdxNext_c    = 4'd0;
            end
            else
              IdxNext_c    = Idx_q + 4'd1;
          end

          CC_START:
          begin
            // S is repeated (Idx saturates) until the slave confirms on CDS
            if (S_Conf_c)
            begin
              MainSmNext_c = (Type_q == CT_RD) ? CC_RDATA : CC_WDATA;
              IdxNext_c    = 4'd0;
            end
            else if (Idx_q < 4'd15)
              IdxNext_c    = Idx_q + 4'd1;
          end

          CC_WDATA:
          begin
            if (Idx_q == 4'd12)
            begin
              MainSmNext_c = CC_TAIL;
              IdxNext_c    = 4'd0;
            end
            else
              IdxNext_c    = Idx_q + 4'd1;
          end

          CC_RDATA:
          begin
            if (Idx_q == 4'd12)
            begin
              MainSmNext_c = CC_IDLE;
              IdxNext_c    = 4'd0;
            end
            else
              IdxNext_c    = Idx_q + 4'd1;
          end

          CC_CWAIT:
          begin
            // broadcast: S is immediately followed by EX; addressed: S + 7 cycles
            if (Idx_q == ((Type_q == CT_BCAST) ? 4'd0 : 4'd7))
            begin
              MainSmNext_c = CC_EXEC;
              IdxNext_c    = 4'd0;
            end
            else
              IdxNext_c    = Idx_q + 4'd1;
          end

          CC_EXEC:
          begin
            if (Idx_q == 4'd0)
            begin
              MainSmNext_c = CC_IDLE;
              IdxNext_c    = 4'd0;
            end
          end

          CC_TAIL:
          begin
            if (Idx_q == 4'd0)
            begin
              MainSmNext_c = CC_IDLE;
              IdxNext_c    = 4'd0;
            end
          end

          default: ;
        endcase
      end
    end
  end

  //------------------------------------------------------------------------------------------
  // Combinational: CDM bit encoded into the timeout slot of the next frame
  //------------------------------------------------------------------------------------------

  logic TXBit_c;

  always_comb
  begin
    case (MainSM)
      CC_HEAD:  TXBit_c = (Idx_q < 4'd12) ? Hdr_q[4'd11 - Idx_q] : ~CRCTX_o[4'd15 - Idx_q];
      CC_RW:    TXBit_c = (Idx_q == 4'd0)  ? Rbit_c : Wbit_c;
      CC_START: TXBit_c = 1'b1;
      CC_WDATA: TXBit_c = (Idx_q < 4'd8)   ? Data_q[4'd7 - Idx_q] :
                          (Idx_q < 4'd12)  ? ~CRCTX_o[4'd11 - Idx_q] : 1'b0;
      CC_CWAIT: TXBit_c = (Idx_q == 4'd0)  ? 1'b1 : 1'b0;
      CC_EXEC:  TXBit_c = 1'b1;
      default:  TXBit_c = 1'b0;                       // CC_IDLE, CC_PRE, CC_RDATA, CC_TAIL, CC_ABRT
    endcase
  end

  //------------------------------------------------------------------------------------------
  // CRC-4 (polynomial 0x13 = x^4+x+1) for the transmitted header / write data and
  // for the received read data. Both instances run serially: one bit is folded per
  // committed frame, CRC_o holds the running check value and the wire image of a
  // protected field is the inverted MSB-first CRC_o.
  //------------------------------------------------------------------------------------------

  logic       TXC_Feed_c, TXC_Rst_c;

  assign TXC_Feed_c = FrmCmt_c && ( ((MainSM == CC_HEAD)  && (Idx_q >= 4'd1) && (Idx_q <= 4'd11)) ||
                                    ((MainSM == CC_WDATA) && (Idx_q <= 4'd7)) );

  // the transmit CRC is re-seeded when a header or the write payload starts
  assign TXC_Rst_c = ( (MainSM == CC_IDLE) && SET_FSM_Enable_i && COMM_Request_i && (ZeroRun_q >= RUN_REQ) ) ||
                     ( (MainSM == CC_PRE)  && FrmCmt_c && (ZeroRun_q >= RUN_REQ_M1) ) ||
                     ( S_Conf_c && (Type_q == CT_WR) );

  prim_CRC_Calculator
  #(
    .CRC_WIDTH   (4      ),
    .DATA_WIDTH  (1      ),
    .CRC_POLY    (4'h03  ),                          // x^4 + x + 1 (0x13)
    .CRC_INIT    ('0     ),
    .CRC_XOR_OUT ('0     ),
    .REFIN       (1'b0   ),
    .REFOUT      (1'b0   ),
    .LSB_FIRST   (1'b0   )
  )
  TX_CRC_Calc
  (
    .CLK_i       (CLK_i              ),
    .RST_i       (RST_i | TXC_Rst_c  ),
    .DataValid_i (TXC_Feed_c         ),
    .Data_i      (CDM_Bit_q          ),
    .CRC_o       (CRCTX_o            )
  );

  logic       RXC_Feed_c, RXC_Rst_c;

  assign RXC_Feed_c = FrmCmt_c && (MainSM == CC_RDATA) && (Idx_q <= 4'd7);
  assign RXC_Rst_c  = S_Conf_c && (Type_q == CT_RD);

  prim_CRC_Calculator
  #(
    .CRC_WIDTH   (4      ),
    .DATA_WIDTH  (1      ),
    .CRC_POLY    (4'h03  ),                          // x^4 + x + 1 (0x13)
    .CRC_INIT    ('0     ),
    .CRC_XOR_OUT ('0     ),
    .REFIN       (1'b0   ),
    .REFOUT      (1'b0   ),
    .LSB_FIRST   (1'b0   )
  )
  RX_CRC_Calc
  (
    .CLK_i       (CLK_i              ),
    .RST_i       (RST_i | RXC_Rst_c  ),
    .DataValid_i (RXC_Feed_c         ),
    .Data_i      (Recv_CDSBits_i[0]  ),
    .CRC_o       (CRCRX_o            )
  );

  //------------------------------------------------------------------------------------------
  // Sequencer
  //------------------------------------------------------------------------------------------

  always_ff @(posedge CLK_i)
  begin
    if (RST_i)
    begin
      MainSM       <= CC_IDLE;
      Type_q       <= CT_RD;
      Idx_q        <= 4'd0;
      ChkPhase_q   <= CC_IDLE;
      ChkIdx_q     <= 4'd0;
      Hdr_q        <= 12'h000;
      Data_q       <= 'd0;
      Echo_q       <= 12'h000;
      DataRX_q     <= 'd0;
      CRCSr_q      <= 4'h0;
      IDA_q        <= 8'h00;
      Busy_q       <= 'd0;
      ZeroRun_q    <= 4'd0;
      CDM_Bit_q    <= 1'b0;
      COMM_Reply_o <= 1'b0;
      COMM_Data_o  <= 'd0;
      Err_CDM_o    <= 1'b0;
    end
    else
    begin
      // one-cycle pulses
      COMM_Reply_o <= 1'b0;
      Err_CDM_o   <= 1'b0;

      if (FR_Abort_i && (MainSM != CC_IDLE) && (MainSM != CC_ABRT))
      begin
        // the frame controller gave up: cancel the control data frame
        Err_CDM_o  <= 1'b1;
        MainSM     <= CC_ABRT;
        Idx_q      <= 4'd0;
        Busy_q     <= 'd0;
        ZeroRun_q  <= 4'd0;   // the slave restarts its timeout run after the cancel
      end
      else
      begin
        case (MainSM)
          CC_IDLE:
          begin
            if (SET_FSM_Enable_i && COMM_Request_i)
            begin
              // sample the request and build the frame header:
              //   {S, CTS, addressing[9:0]}
              //   register frames : CTS=1, addressing = {slave ID, register address}
              //   command frames  : CTS=0, addressing = {IDS mask (broadcast: 0), command}
              Type_q <= cc_type_e'(COMM_Type_i);
              Data_q <= COMM_Data_i;
              Busy_q <= 'd0;
              Hdr_q <= (COMM_Type_i[1])
                       ? {1'b1, 1'b0, ((COMM_Type_i == 2'd3) ? 8'h00 : COMM_IDSel_i), COMM_Cmd_i}
                       : {1'b1, 1'b1, 3'(SLAVE_ID), COMM_Addr_i};
            end
          end

          CC_START:
          begin
            if (S_Busy_c)
              Busy_q <= Busy_q + 1'b1;                 // one more unconfirmed S echo
            if (S_Conf_c)
              Busy_q <= 'd0;                           // slave ready: restart for the next access
          end

          default: ;
        endcase

        if (FrmCmt_c)
        begin
          // -------- response-side collection, keyed on the phase of the bit
          //          whose CDS response is readable in this cycle --------
          // write payload echo (DATA + CRC; the P echo is checked directly)
          if (ChkPhase_q == CC_WDATA)
            Echo_q <= {Echo_q[10:0], Recv_CDSBits_i[0]};

          // read data byte, inverted CRC bits
          if (MainSM == CC_RDATA)
          begin
            if (Idx_q <= 4'd7)
              DataRX_q <= {DataRX_q[WIDTH_CDDATA-2:0], Recv_CDSBits_i[0]};
            else if (Idx_q <= 4'd11)
              CRCSr_q  <= {CRCSr_q[2:0], Recv_CDSBits_i[0]};
          end

          // command IDA acknowledge bits (addressed command frames only)
          if ((Type_q == CT_CMD) &&
              ( ((MainSM == CC_CWAIT) && (Idx_q >= 4'd1)) || (MainSM == CC_EXEC) ))
            IDA_q <= {IDA_q[6:0], Recv_CDSBits_i[0]};

          // -------- completions --------
          if (R_Done_c)
          begin
            COMM_Reply_o <= 1'b1;
            COMM_Data_o  <= DataRX_q;
          end
          if (W_Done_c)
          begin
            COMM_Reply_o <= 1'b1;
            COMM_Data_o  <= 'd0;
          end
          if (X_Done_c)
          begin
            COMM_Reply_o <= 1'b1;
            COMM_Data_o  <= (Type_q == CT_CMD) ? {IDA_q[6:0], Recv_CDSBits_i[0]} : 'd0;
          end
          if (RW_Err_c || W_Err_c || R_Err_c || BusyTO_c)
            Err_CDM_o  <= 1'b1;

          // -------- check-phase tracking: the bit committed in this cycle --------
          ChkPhase_q <= MainSM;
          ChkIdx_q   <= Idx_q;
        end

        // -------- transmit-stream advance (frame commits and request accept) --------
        MainSM <= MainSmNext_c;
        Idx_q  <= IdxNext_c;

        // CDM='0' idle-run bookkeeping.  The slave restarts its own timeout
        // counter when a transaction completes or is cancelled, so the master
        // must mirror both restarts: after a completion/abort the next access
        // always needs a full RUN_REQ '0' run (CC_PRE) before its header.
        if (R_Done_c || W_Done_c || X_Done_c || RW_Err_c || W_Err_c || R_Err_c || BusyTO_c)
          ZeroRun_q <= 4'd0;
        else if (FR_Load_i)
        begin
          if (REQ_CDM_o && CDM_Bit_q)
            ZeroRun_q <= 4'd0;
          else if (ZeroRun_q != 4'd15)
            ZeroRun_q <= ZeroRun_q + 4'd1;
        end
      end

      // CDM bit for the timeout slot of the next accepted frame (registered
      // image of the current stream position; settles one clock after a commit)
      CDM_Bit_q <= TXBit_c;
    end
  end

  //------------------------------------------------------------------------------------------
  // Outputs
  //------------------------------------------------------------------------------------------

  always_ff @(posedge CLK_i)
  begin
    if (RST_i)
      REQD_q <= 1'b0;
    else
      REQD_q <= (MainSM != CC_IDLE);
  end

  assign CDM_Busy_o = (MainSM != CC_IDLE);
  assign REQ_CDM_o   = REQD_q;                 // rises one clock after the accept, when CDM_Bit_q is valid
  assign CDM_Bit_o   = CDM_Bit_q;

endmodule
