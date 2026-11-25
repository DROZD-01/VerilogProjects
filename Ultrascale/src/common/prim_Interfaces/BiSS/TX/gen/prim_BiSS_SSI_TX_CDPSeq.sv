`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/26/2026 
// Design Name: 
// Module Name: prim_BiSS_SSI_TX_CDPSeq
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Control-communication sequencer of the BiSS-C/SSI transmitter
//              (slave). Implements the register communication and BiSS command
//              mechanism of the "BiSS C Protocol Description" (chapter CONTROL
//              COMMUNICATION) that is embedded into the process data frames:
//              - One CDM bit is captured per BiSS frame (sampled by the MA
//                monitor at the slave timeout expiry) and one CDS bit is
//                returned in the header of the NEXT frame, so the complete
//                subordinate control data frame spans many BiSS frames.
//              - Frame formats (CDM bit order, response offset of one frame):
//                Register:  S CTS=1 ID2..0 ADR6..0 CRC3..0 | R W S [DATA CRC P]
//                Command  :  S CTS=0 IDS0..7 CMD1..0 CRC3..0 | S [IDA0..7] EX
//                The 4-bit control CRC uses polynomial 0x13 and is inverted on
//                the wire; it covers CTS plus the addressing bits (S excluded).
//              - Every control data frame starts with the 9-bit ID assignment
//                (IDL0..IDL8) on CDS; a point-to-point slave (SLI grounded)
//                always locks the first free ID, SET_SlaveID_i defaults to 0.
//              - R/W acknowledge: R and W are echoed; W is inverted if the
//                access is rejected or malformed. Read data may be delayed by
//                holding the S echo low (busy) while COMM_Rdy_i is low.
//              - Sequential accesses (master S after the slave's stop bit P)
//                auto-increment the address; P='1' terminates unless the
//                backend allows continuation via COMM_Cont_i.
//              - CDM='0' for 14 consecutive frames aborts/resets the control
//                communication to idle (also the normal end-of-transaction
//                cleanup recommended by the protocol).
//              - BiSS commands: addressed commands are acknowledged with the
//                IDA bit pattern and executed at EX; broadcast commands
//                (IDS=0) execute directly at EX. The protocol-defined effects
//                of CMD='00' (process data channels on/off) and CMD='01'
//                (control communication off/on) are applied internally; all
//              commands are additionally passed to the register backend.
// 
// Dependencies: prim_CRC_Calculator
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module prim_BiSS_SSI_TX_CDPSeq
  #(
    parameter int WIDTH_CDADDR = 7 , // Control Communication address payload bit width [bits]
    parameter int WIDTH_CDDATA = 8 , // Control Communication data    payload bit width [bits]
    parameter int WIDTH_CDSLID = 8   // Control Communication linear-select ID bit width [bits]
  )
  (
    input  logic                  RST_i              , // 
    input  logic                  CLK_i              , // 
    // Settings
    input  logic                  SET_InterfaceMode_i, // '1' - SSI, '0' - BiSS (SSI has no control communication)
    input  logic [           3:0] SET_SlaveID_i      , // Own BiSS Slave ID (point-to-point: 0)
    // CDM bit ingest (one bit per BiSS frame)
    input  logic                  TO_Expire_i        , // Pulse: slave timeout expired (control cycle tick)
    input  logic                  CDM_Bit_i          , // CDM bit sampled at timeout expiry
    // Register backend
    output logic                  COMM_Req_o         , // Pulse: register access / command started
    output logic [           1:0] COMM_Type_o        , // '00' read, '01' write, '10' command, '11' broadcast command
    output logic [           1:0] COMM_CMD_o         , // Command code (bit 1 = CMD1, first on the wire)
    output logic [WIDTH_CDADDR-1:0] COMM_Addr_o      , // Register address
    output logic                  COMM_Write_o       , // '1' = write access
    output logic [WIDTH_CDDATA-1:0] COMM_Data_o      , // Write data out (reads use COMM_RData_i)
    input  logic                  COMM_Accept_i      , // Level: current access accepted (sampled at the W echo)
    input  logic [WIDTH_CDDATA-1:0] COMM_RData_i     , // Read data in
    input  logic                  COMM_Rdy_i         , // Level: backend ready (delays the S echo = busy while low)
    input  logic                  COMM_Cont_i        , // Level: allow sequential continuation (P bit)
    output logic                  COMM_WStrb_o       , // Pulse: verified write, COMM_Addr_o/COMM_Data_o valid
    output logic                  COMM_Reply_o       , // Pulse: byte completed (stop bit P sent)
    // Towards the frame engine
    output logic                  CDS_Bit_o          , // CDS bit of the upcoming frame
    output logic                  CDM_Busy_o         , // Level: control-communication transaction active
    output logic                  EN_PDUCH_o         , // Level: process data channels active (short frame if '0')
    output logic                  Err_CDM_o            // Pulse: control-communication error
  );
  
  localparam int WID = WIDTH_CDDATA; // Shorthand
  
  //--------------------------------------------------------------------
  // Sequencer states
  //--------------------------------------------------------------------
  
  typedef enum logic[3:0] {
    ST_IDLE   , // No transaction
    ST_HDR    , // Receiving CTS / addressing / header CRC
    ST_RW     , // Receiving R and W bits
    ST_SWAIT  , // Waiting for (and echoing) the master start bit S
    ST_RDATA  , // Sending read data / CRC / stop bit
    ST_WDATA  , // Receiving and echoing write data / CRC / stop bit
    ST_SEQ    , // Byte done, waiting for sequential S
    ST_CMD_IDA, // Command: sending IDA bits, waiting for EX
    ST_CMD_BC , // Broadcast command: waiting for EX
    ST_IGN      // Not addressed / CRC failed / aborted: silent
  } State_t;
  
  State_t                State     ; // 
  logic [           4:0] Cnt       ; // Index of the CDM bit ingested at the current timeout
  logic [           3:0] RspIdx    ; // Read response bit index (DATA/CRC/P)
  logic [           3:0] AbortCnt  ; // Consecutive CDM='0' counter (14 -> reset to idle)
  logic                  CTS_q     ; // Control select ('1' - register, '0' - command)
  logic [           2:0] ID_q      ; // Register recipient ID (ID2 first on the wire)
  logic [WIDTH_CDSLID-1:0] IDS_q  ; // Linear-select ID (bit 7 = IDS0 = ID 0, first on the wire)
  logic [           1:0] CMD_q    ; // Command code (bit 1 = CMD1, first on the wire)
  logic [WIDTH_CDADDR-1:0] ADR_q  ; // Register address (ADR6 first on the wire)
  logic [           3:0] HCRC_q   ; // Received header CRC (un-inverted)
  logic [WIDTH_CDDATA-1:0] WDATA_q; // Received write data (DATA7 first on the wire)
  logic [WIDTH_CDDATA-1:0] RDATA_q; // Latched read data for the response stream
  logic [           3:0] WCRC_q   ; // Received data CRC (wire polarity)
  logic                  R_q       ; // Read flag
  logic                  FirstByte ; // First byte of the access (write S echo must not be delayed)
  logic [           3:0] SlaveID_q; // Latched own slave ID
  logic                  CtrlCommEn; // Control communication active (CMD='01' effect)
  logic                  WCRC_OK   ; // Verified write data CRC
  logic                  HdrCRC_OK ; // Verified header CRC
  logic                  SIngest   ; // This timeout ingests a transaction start bit S
  logic                  Advance   ; // This timeout confirms the master start bit S
  logic                  NextCDS   ; // CDS bit for the next frame
  
  // Broadcast = all IDS bits low; register match = binary ID equal
  logic IsBroadcast;
  logic RegSel;
  
  assign IsBroadcast = (IDS_q == 'd0);
  assign RegSel      = !SlaveID_q[3] && (ID_q == SlaveID_q[2:0]);
  
  //--------------------------------------------------------------------
  // CRC engines
  //  - Header check: 1-bit beats (CTS + 10 addressing bits), plain CRC.
  //  - Data: one beat per byte, final XOR 0xF bakes the protocol's
  //    "CRC is inverted" rule for the read response; for write
  //    verification the engine output is compared against the received
  //    wire CRC bits (which are inverted as well).
  //--------------------------------------------------------------------
  
  logic [3:0] CRC_Hdr ; // CRC over CTS + addressing bits
  logic [3:0] CRC_Data; // CRC over the data byte (wire polarity)
  logic       HdrBeat ; // Header engine beat
  logic       DataBeat; // Data engine beat
  logic       HdrClr  ; // Header engine reseed
  logic       DataClr ; // Data engine reseed
  
  assign HdrBeat  = TO_Expire_i && (State == ST_HDR) && (Cnt >= 5'd1) && (Cnt <= 5'd11);
  assign DataBeat = TO_Expire_i && ((State == ST_SWAIT) && Advance && R_q) 
                    || (TO_Expire_i && (State == ST_WDATA) && (Cnt == 5'd26));
  assign HdrClr   = RST_i || (TO_Expire_i && SIngest);
  assign DataClr  = RST_i || (TO_Expire_i && SIngest) 
                    || (TO_Expire_i && (State == ST_WDATA) && (Cnt == 5'd31))
                    || (TO_Expire_i && (State == ST_RDATA) && (RspIdx == 4'd12));
  
  prim_CRC_Calculator
  #(
    .CRC_WIDTH   (4          ), // CRC Width
    .DATA_WIDTH  (1          ), // Data bits per beat
    .CRC_POLY    (4'h3       ), // Control communication polynomial 0x13 = x^4+x+1, x^4 term excluded
    .CRC_INIT    (4'h0       ), // Register seed
    .CRC_XOR_OUT (4'h0       )  // Final XOR
  )
  CRC_Engine_Hdr
  (
    .CLK_i      (CLK_i       ), // System clock
    .RST_i      (HdrClr      ), // Synchronous active-high reset
    .DataValid_i(HdrBeat     ), // Data beat valid
    .Data_i     (CDM_Bit_i   ), // Data beat
    .CRC_o      (CRC_Hdr     )  // Running/Final CRC
  );
  
  prim_CRC_Calculator
  #(
    .CRC_WIDTH   (4          ), // CRC Width
    .DATA_WIDTH  (WIDTH_CDDATA), // Data bits per beat
    .CRC_POLY    (4'h3       ), // Control communication polynomial 0x13 = x^4+x+1, x^4 term excluded
    .CRC_INIT    (4'h0       ), // Register seed
    .CRC_XOR_OUT (4'hF       )  // Final XOR (protocol: CRC inverted on the wire)
  )
  CRC_Engine_Data
  (
    .CLK_i      (CLK_i                     ), // System clock
    .RST_i      (DataClr                   ), // Synchronous active-high reset
    .DataValid_i(DataBeat                  ), // Data beat valid
    .Data_i     ((State == ST_SWAIT)? COMM_RData_i : {WDATA_q[WID-2:0], CDM_Bit_i}), // Data beat
    .CRC_o      (CRC_Data                  )  // Running/Final CRC
  );
  
  //--------------------------------------------------------------------
  // CRC verdicts (checked when the last CRC bit arrives)
  //--------------------------------------------------------------------
  
  always_comb
  begin
    HdrCRC_OK = (CRC_Hdr == {HCRC_q[2:0], ~CDM_Bit_i});
    WCRC_OK   = (CRC_Data == WCRC_q); // All 4 wire CRC bits latched by Cnt==31
  end
  
  //--------------------------------------------------------------------
  // Start bit detection and busy-aware advance
  //--------------------------------------------------------------------
  
  assign SIngest = CDM_Bit_i && (State == ST_IDLE || State == ST_IGN);
  
  always_comb
  begin
    // Read accesses and sequential write bytes may delay the S echo while
    // the backend is busy; the first write byte must answer immediately.
    if(R_q)
      Advance = CDM_Bit_i && COMM_Rdy_i;
    else
      Advance = CDM_Bit_i && (FirstByte || COMM_Rdy_i);
  end
  
  //--------------------------------------------------------------------
  // CDS response bit (transmitted in the header of the next frame):
  // response(n) is generated at the timeout that ingests CDM bit n.
  //--------------------------------------------------------------------
  
  always_comb
  begin
    NextCDS = 1'b0;
    
    if(CtrlCommEn && !SET_InterfaceMode_i)
    begin
      case(State)
        //-----------------------------------------------
        ST_IDLE, ST_IGN:
        begin
          // Transaction start: IDL0 (lock bit for Slave ID 0)
          if(SIngest)
            NextCDS = (SET_SlaveID_i == 4'd0);
        end
        //-----------------------------------------------
        ST_HDR:
        begin
          if(Cnt <= 5'd8)
            NextCDS = (Cnt[3:0] == SlaveID_q); // IDL1..IDL8
        end
        //-----------------------------------------------
        ST_RW:
        begin
          if(Cnt == 5'd16)
            NextCDS = CDM_Bit_i; // R echo
          else if(Cnt == 5'd17)
            NextCDS = ((R_q == CDM_Bit_i) || !COMM_Accept_i)? ~CDM_Bit_i : CDM_Bit_i; // W echo
        end
        //-----------------------------------------------
        ST_SWAIT:
        begin
          if(Advance)
            NextCDS = 1'b1; // S echo (delayed while busy)
        end
        //-----------------------------------------------
        ST_RDATA:
        begin
          if(RspIdx <= 4'd7)
            NextCDS = RDATA_q[WID-1 - RspIdx];             // DATA7..0 (latched at the S echo)
          else if(RspIdx <= 4'd11)
            NextCDS = CRC_Data[3 - (RspIdx - 4'd8)];        // CRC3..0 (inverted)
          else if(RspIdx == 4'd12)
            NextCDS = ~COMM_Cont_i;                          // P ('1' terminates)
        end
        //-----------------------------------------------
        ST_WDATA:
        begin
          if(Cnt == 5'd31)
            NextCDS = WCRC_OK? ~COMM_Cont_i : 1'b0; // P echo (suppressed on CRC error)
          else
            NextCDS = CDM_Bit_i; // DATA/CRC echo
        end
        //-----------------------------------------------
        ST_CMD_IDA:
        begin
          if((Cnt >= 5'd17) && (Cnt <= 5'd24))
            NextCDS = ((Cnt - 5'd17) == SlaveID_q[3:0]); // IDA0..7
        end
        //-----------------------------------------------
        default:
          NextCDS = 1'b0;
      endcase
    end
  end
  
  //--------------------------------------------------------------------
  // Sequencer: one control cycle per BiSS frame timeout
  //--------------------------------------------------------------------
  
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i) 
    begin
      State      <= ST_IDLE;
      Cnt        <= 5'd0;
      RspIdx     <= 4'd0;
      AbortCnt   <= 4'd0;
      CTS_q      <= 1'b0;
      ID_q       <= 'd0;
      IDS_q      <= 'd0;
      CMD_q      <= 2'd0;
      ADR_q      <= 'd0;
      HCRC_q     <= 'd0;
      WDATA_q    <= 'd0;
      RDATA_q    <= 'd0;
      WCRC_q     <= 'd0;
      R_q        <= 1'b0;
      FirstByte  <= 1'b1;
      SlaveID_q  <= 4'd0;
      CtrlCommEn <= 1'b1;
      CDS_Bit_o  <= 1'b0;
      COMM_Req_o <= 1'b0;
      COMM_Type_o <= 2'd0;
      COMM_CMD_o <= 2'd0;
      COMM_Addr_o <= 'd0;
      COMM_Write_o <= 1'b0;
      COMM_Data_o <= 'd0;
      COMM_WStrb_o <= 1'b0;
      COMM_Reply_o <= 1'b0;
      Err_CDM_o  <= 1'b0;
      EN_PDUCH_o <= 1'b1;
    end
    else
    begin
      COMM_Req_o  <= 1'b0;
      COMM_WStrb_o <= 1'b0;
      COMM_Reply_o <= 1'b0;
      Err_CDM_o   <= 1'b0;
      
      if(TO_Expire_i && !SET_InterfaceMode_i)
      begin
        // CDS response for the next frame
        CDS_Bit_o <= NextCDS;
        
        //--------------------------------------------------------------
        // Abort: 14 consecutive CDM='0' reset the control communication
        //--------------------------------------------------------------
        if(CDM_Bit_i)
          AbortCnt <= 4'd0;
        else if(AbortCnt < 4'd14)
          AbortCnt <= AbortCnt + 1'd1;
        
        if(AbortCnt == 4'd13 && !CDM_Bit_i)
        begin
          State <= ST_IDLE; // 14th '0': back to idle, no error
        end
        else
        begin
          case(State)
            //-----------------------------------------------
            ST_IDLE, ST_IGN:
            begin
              if(SIngest)
              begin
                State      <= ST_HDR;
                Cnt        <= 5'd1;
                SlaveID_q  <= SET_SlaveID_i;
                FirstByte  <= 1'b1;
              end
            end
            //-----------------------------------------------
            ST_HDR:
            begin
              // Field deserialization (all fields MSB-first)
              if(Cnt == 5'd1)
                CTS_q <= CDM_Bit_i;
              else if((Cnt >= 5'd2) && (Cnt <= 5'd4))
              begin
                if(CTS_q)
                  ID_q <= {ID_q[1:0], CDM_Bit_i};
                else
                  IDS_q <= {IDS_q[WIDTH_CDSLID-2:0], CDM_Bit_i};
              end
              else if((Cnt >= 5'd5) && (Cnt <= 5'd9))
              begin
                if(CTS_q)
                  ADR_q <= {ADR_q[WIDTH_CDADDR-2:0], CDM_Bit_i};
                else
                  IDS_q <= {IDS_q[WIDTH_CDSLID-2:0], CDM_Bit_i};
              end
              else if(Cnt == 5'd10 || Cnt == 5'd11)
              begin
                if(CTS_q)
                  ADR_q <= {ADR_q[WIDTH_CDADDR-2:0], CDM_Bit_i};
                else
                  CMD_q <= {CMD_q[0], CDM_Bit_i};
              end
              else if(Cnt >= 5'd12)
                HCRC_q <= {HCRC_q[2:0], ~CDM_Bit_i}; // Un-invert on the fly
              
              if(Cnt == 5'd15)
              begin
                // Header complete: CRC verdict and addressing
                if(!HdrCRC_OK)
                begin
                  Err_CDM_o <= 1'b1; // Behave as not addressed
                  State     <= ST_IGN;
                end
                else if(CTS_q)
                begin
                  if(RegSel)
                  begin
                    State <= ST_RW;
                    Cnt   <= 5'd16;
                  end
                  else
                    State <= ST_IGN; // Other slave's register access
                end
                else
                begin
                  // BiSS command frame
                  if(IsBroadcast)
                  begin
                    State <= ST_CMD_BC; // Broadcast: always processed
                    Cnt   <= 5'd16;
                  end
                  else if(!SlaveID_q[3] && IDS_q[3'd7 - SlaveID_q[2:0]])
                  begin
                    State <= ST_CMD_IDA; // Addressed to us
                    Cnt   <= 5'd16;
                  end
                  else
                    State <= ST_IGN;
                end
              end
              else
                Cnt <= Cnt + 5'd1;
            end
            //-----------------------------------------------
            ST_RW:
            begin
              if(Cnt == 5'd16)
              begin
                R_q <= CDM_Bit_i;
                Cnt <= 5'd17;
                // Ask the backend about this access (direction known from R)
                COMM_Req_o   <= 1'b1;
                COMM_Type_o  <= CDM_Bit_i? 2'd0 : 2'd1; // read / write
                COMM_CMD_o   <= 2'd0;
                COMM_Addr_o  <= ADR_q;
                COMM_Write_o <= ~CDM_Bit_i;
              end
              else
              begin
                // Cnt == 17: W bit received, echo generated above
                if((R_q == CDM_Bit_i) || !COMM_Accept_i)
                begin
                  if(R_q == CDM_Bit_i)
                    Err_CDM_o <= 1'b1; // Malformed R/W
                  State <= ST_IGN;     // Rejected / malformed: inverted W echo, abort
                end
                else
                begin
                  State <= ST_SWAIT;
                  Cnt   <= 5'd18;
                end
              end
            end
            //-----------------------------------------------
            ST_SWAIT:
            begin
              if(Advance)
              begin
                FirstByte <= 1'b0;
                if(R_q)
                begin
                  State    <= ST_RDATA; // Read data phase (CRC beat runs on COMM_RData_i)
                  RspIdx   <= 4'd0;
                  RDATA_q  <= COMM_RData_i;
                end
                else
                begin
                  State <= ST_WDATA; // Write data phase
                  Cnt   <= 5'd19;
                end
              end
            end
            //-----------------------------------------------
            ST_RDATA:
            begin
              if(RspIdx == 4'd12)
              begin
                COMM_Reply_o <= 1'b1; // Stop bit P sent
                State        <= ST_SEQ;
              end
              else
                RspIdx <= RspIdx + 4'd1;
            end
            //-----------------------------------------------
            ST_WDATA:
            begin
              if((Cnt >= 5'd19) && (Cnt <= 5'd26))
                WDATA_q <= {WDATA_q[WID-2:0], CDM_Bit_i}; // DATA7..0
              else if((Cnt >= 5'd27) && (Cnt <= 5'd30))
                WCRC_q  <= {WCRC_q[2:0], CDM_Bit_i};      // CRC3..0 (wire polarity)
              
              if(Cnt == 5'd31)
              begin
                if(WCRC_OK)
                begin
                  COMM_WStrb_o <= 1'b1; // Apply the verified write
                  COMM_Data_o <= WDATA_q; // Write data for the backend
                  COMM_Reply_o <= 1'b1;
                  State        <= ST_SEQ;
                end
                else
                begin
                  Err_CDM_o <= 1'b1; // Data CRC mismatch: no write, no P echo
                  State     <= ST_IGN;
                end
              end
              else
                Cnt <= Cnt + 5'd1;
            end
            //-----------------------------------------------
            ST_SEQ:
            begin
              if(CDM_Bit_i)
              begin
                // Sequential access: auto-increment, re-arm the data phase
                ADR_q       <= ADR_q + 1'd1;
                COMM_Req_o  <= 1'b1;
                COMM_Type_o <= R_q? 2'd0 : 2'd1;
                COMM_CMD_o  <= 2'd0;
                COMM_Addr_o <= ADR_q + 1'd1;
                COMM_Write_o <= ~R_q;
                State       <= ST_SWAIT;
                Cnt         <= 5'd18;
              end
            end
            //-----------------------------------------------
            ST_CMD_IDA:
            begin
              // EX accepted in a two-cycle window around the IDA response
              if(CDM_Bit_i && (Cnt >= 5'd24) && (Cnt <= 5'd25))
              begin
                COMM_Req_o  <= 1'b1;
                COMM_Type_o <= 2'd2; // Command
                COMM_CMD_o  <= CMD_q;
                COMM_Addr_o <= 'd0;
                COMM_Write_o <= 1'b0;
                // Protocol-defined command effects
                if(CMD_q == 2'd0)
                  EN_PDUCH_o <= 1'b1; // CMD='00' addressed: activate process data
                if(CMD_q == 2'd1)
                  CtrlCommEn <= 1'b0; // CMD='01' addressed: deactivate control communication
                State <= ST_IGN;
              end
              else if(Cnt >= 5'd25)
                State <= ST_IGN; // EX never came
              else
                Cnt <= Cnt + 5'd1;
            end
            //-----------------------------------------------
            ST_CMD_BC:
            begin
              // Broadcast: EX directly follows the verification start bit
              if(CDM_Bit_i && (Cnt >= 5'd17) && (Cnt <= 5'd18))
              begin
                COMM_Req_o  <= 1'b1;
                COMM_Type_o <= 2'd3; // Broadcast command
                COMM_CMD_o  <= CMD_q;
                COMM_Addr_o <= 'd0;
                COMM_Write_o <= 1'b0;
                // Protocol-defined command effects
                if(CMD_q == 2'd0)
                  EN_PDUCH_o <= 1'b0; // CMD='00' broadcast: deactivate process data (short frame)
                if(CMD_q == 2'd1)
                  CtrlCommEn <= 1'b1; // CMD='01' broadcast: reactivate control communication
                State <= ST_IGN;
              end
              else if(Cnt >= 5'd18)
                State <= ST_IGN;
              else
                Cnt <= Cnt + 5'd1;
            end
            //-----------------------------------------------
            default:
              State <= ST_IDLE;
          endcase
        end
      end
    end
  end
  
  //--------------------------------------------------------------------
  // Status outputs
  //--------------------------------------------------------------------
  
  assign CDM_Busy_o = (State != ST_IDLE);
  
endmodule
