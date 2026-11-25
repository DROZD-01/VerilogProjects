`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// Testbench for prim_BiSS_SSI_TX (BiSS-C / SSI transmitter, slave side).
// Models a BiSS-C master per the "BiSS C Protocol Description" (Rev D2):
//  - MA bursts with the frame  LAT -> ACK -> STR -> CDS -> PDU -> CRC -> STP
//    (SL is sampled shortly before each rising MA edge, the way a real
//    master covers the slave delay D).
//  - CDM transmission during the slave timeout (MA held low = CDM '1').
//  - Register read / write transactions incl. ID assignment (IDL), R/W
//    echo, data + 4-bit control CRC (poly 0x13, inverted) and stop bit.
//  - Broadcast command frames (CMD='00' short frame effect).
//  - SSI mode (falling-edge launch, monoflop timeout).
//  - Watchdog (mid-frame MA stall) and over-clock error checks.
// Runs against the two Icarus shims; the DUT RTL itself is the real code.
//-----------------------------------------------------------------------------

module tb_prim_BiSS_SSI_TX;
  
  localparam real SYSCLK_HZ  = 100.0e6 ;
  localparam real T_CLK      = 10.0    ;
  localparam real T_MA       = 1000.0  ; // 1 MHz MA
  localparam real T_HALF     = 500.0   ;
  localparam real T_SETUP    = 100.0   ; // SL sample point before the rising MA edge
  localparam int  OVERSAMPLE = 32      ;
  localparam int  MAX_PDUBIT = 32      ;
  localparam int  PDU_BITS   = 24      ;
  localparam int  VAL_CRCBIT = 6       ;
  localparam int  FRAME_LEN  = 3 + PDU_BITS + VAL_CRCBIT; // ACK STR CDS PDU CRC
  localparam int  N_EDGES    = FRAME_LEN + 1;             // + STP edge
  
  // DUT signals
  logic RST, CLK = 1'b0;
  logic Start;
  logic SET_FSM_Enable, SET_InterfaceMode, SET_FSM_Continuous;
  logic SET_EN_BiSS_CRC, SET_CRC_InclCDS;
  logic SET_Lvl_LaunchSL, SET_Lvl_SL_Idle;
  logic SET_EN_AdaptTO, SET_EN_HoldCDM;
  logic [$clog2(MAX_PDUBIT+1)-1:0] SET_PDUBits;
  logic [1:0] SET_Baudrate;
  logic [15:0] SET_Limit_TickTO, SET_Limit_HalfMA;
  logic [3:0] SET_SlaveID;
  logic Clear_Status;
  logic [MAX_PDUBIT-1:0] Send_PDUBits;
  logic Send_Latched;
  logic SS_MA = 1'b1, SS_SL;
  logic COMM_Req, COMM_Write, COMM_Accept, COMM_Rdy, COMM_Cont, COMM_WStrb, COMM_Reply;
  logic [1:0] COMM_Type, COMM_CMD;
  logic [6:0] COMM_Addr;
  logic [7:0] COMM_Data, COMM_RData;
  logic [3:0] Count_Frame, Count_Error;
  logic ERR_WdExp, ERR_Overrun, ERR_CDM;
  
  prim_BiSS_SSI_TX
  #(
    .FREQ_SYSCLK    (SYSCLK_HZ  ),
    .MAX_PDUBIT     (MAX_PDUBIT ),
    .VAL_CRCBIT     (VAL_CRCBIT ),
    .OVERSAMPLE     (OVERSAMPLE )
  )
  DUT
  (
    .RST_i               (RST                ),
    .CLK_i               (CLK                ),
    .Start_i             (Start              ),
    .SET_FSM_Enable_i    (SET_FSM_Enable     ),
    .SET_InterfaceMode_i (SET_InterfaceMode  ),
    .SET_FSM_Continuous_i(SET_FSM_Continuous ),
    .SET_EN_BiSS_CRC_i   (SET_EN_BiSS_CRC    ),
    .SET_CRC_InclCDS_i   (SET_CRC_InclCDS    ),
    .SET_Lvl_LaunchSL_i  (SET_Lvl_LaunchSL   ),
    .SET_Lvl_SL_Idle_i   (SET_Lvl_SL_Idle    ),
    .SET_PDUBits_i       (SET_PDUBits        ),
    .SET_Baudrate_i      (SET_Baudrate       ),
    .SET_EN_AdaptTO_i    (SET_EN_AdaptTO     ),
    .SET_EN_HoldCDM_i    (SET_EN_HoldCDM     ),
    .SET_Limit_TickTO_i  (SET_Limit_TickTO   ),
    .SET_Limit_HalfMA_i  (SET_Limit_HalfMA   ),
    .SET_SlaveID_i       (SET_SlaveID        ),
    .Clear_Status_i      (Clear_Status       ),
    .Count_Frame_o       (Count_Frame        ),
    .Count_Error_o       (Count_Error        ),
    .ERR_WdExp_o         (ERR_WdExp          ),
    .ERR_Overrun_o       (ERR_Overrun        ),
    .ERR_CDM_o           (ERR_CDM            ),
    .Send_PDUBits_i      (Send_PDUBits       ),
    .Send_Latched_o      (Send_Latched       ),
    .COMM_Req_o          (COMM_Req           ),
    .COMM_Type_o         (COMM_Type          ),
    .COMM_CMD_o          (COMM_CMD           ),
    .COMM_Addr_o         (COMM_Addr          ),
    .COMM_Write_o        (COMM_Write         ),
    .COMM_Data_o         (COMM_Data          ),
    .COMM_Accept_i       (COMM_Accept        ),
    .COMM_RData_i        (COMM_RData         ),
    .COMM_Rdy_i          (COMM_Rdy           ),
    .COMM_Cont_i         (COMM_Cont          ),
    .COMM_WStrb_o        (COMM_WStrb         ),
    .COMM_Reply_o        (COMM_Reply         ),
    .SS_MA               (SS_MA              ),
    .SS_SL               (SS_SL              )
  );
  
  always #(T_CLK/2.0) CLK = ~CLK;
  
  //---------------------------------------------------------------------------
  // Register-file backend
  //---------------------------------------------------------------------------
  
  logic [7:0] RegFile [0:127];
  logic [7:0] RData_q;
  
  initial
  begin
    RegFile[0] = 8'hA5;
    RegFile[3] = 8'h5A;
    RegFile[7] = 8'h00;
    RData_q = 8'h00;
  end
  
  always_ff @(posedge CLK)
  begin
    if(RST)
      RData_q <= 8'h00;
    else if(COMM_Req && (COMM_Type == 2'd0))
      RData_q <= RegFile[COMM_Addr];
    if(COMM_WStrb)
      RegFile[COMM_Addr] <= COMM_Data;
  end
  
  assign COMM_RData  = RData_q;
  assign COMM_Accept = 1'b1;
  assign COMM_Rdy    = 1'b1;
  assign COMM_Cont   = 1'b0;
  
  //---------------------------------------------------------------------------
  // Reference CRCs
  //---------------------------------------------------------------------------
  
  // Data-channel CRC (matches the DUT parameters): poly 0x2F, init 0x2A,
  // final XOR 0x15; the DUT additionally inverts the result on the wire.
  function automatic logic [VAL_CRCBIT-1:0] crc6_f(input logic [MAX_PDUBIT-1:0] data, 
                                                    input int nbits);
    logic [VAL_CRCBIT-1:0] c;
    int i;
    c = 6'h2A;
    for(i = 0; i < nbits; i = i + 1)
      c = {c[VAL_CRCBIT-2:0], 1'b0} ^ (6'h2F & {VAL_CRCBIT{c[VAL_CRCBIT-1] ^ data[MAX_PDUBIT-1-i]}});
    return c;
  endfunction
  
  // Control-communication CRC (poly 0x13, init 0), MSB-first over 11 bits
  function automatic logic [3:0] crc4_f(input logic [10:0] bits11);
    logic [3:0] c;
    int i;
    c = 4'h0;
    for(i = 10; i >= 0; i = i - 1)
      c = {c[2:0], 1'b0} ^ (4'h3 & {4{c[3] ^ bits11[i]}});
    return c;
  endfunction
  
  function automatic logic [3:0] crc4_byte_f(input logic [7:0] b);
    logic [3:0] c;
    int i;
    c = 4'h0;
    for(i = 7; i >= 0; i = i - 1)
      c = {c[2:0], 1'b0} ^ (4'h3 & {4{c[3] ^ b[i]}});
    return c;
  endfunction
  
  //---------------------------------------------------------------------------
  // Master BFM
  //---------------------------------------------------------------------------
  
  int ErrCnt = 0;
  int FrameCnt = 0;
  int CurEdges = N_EDGES;
  
  logic [63:0] Samples; // Samples[k] = SL at rising edge k+2 (bit launched at edge k+1)
  
  task automatic WaitSLIdle;
    int g;
    begin
      g = 0;
      while(SS_SL !== 1'b1 && g < 20000)
      begin
        #10;
        g = g + 10;
      end
      if(g >= 20000)
      begin
        ErrCnt = ErrCnt + 1;
        $display("[%0t] FAIL: SL did not return to idle level (timeout indication missing)", $time);
      end
    end
  endtask
  
  // One BiSS frame: CurEdges rising edges, SL sampled before every edge from
  // edge 2 on, CDM bit encoded after the final falling edge, then wait for
  // the slave timeout indication.
  task automatic BissFrame(input logic CDM);
    int k;
    begin
      FrameCnt = FrameCnt + 1;
      Samples = '0;
      
      SS_MA = 1'b0;              // Frame start: falling edge
      #(T_HALF);
      SS_MA = 1'b1;              // LAT (edge 1): ACK is launched here
      #(T_HALF);
      
      for(k = 2; k <= CurEdges; k = k + 1)
      begin
        SS_MA = 1'b0;            // Falling edge
        #(T_HALF - T_SETUP);
        Samples[k-2] = SS_SL;    // Sample the bit launched at edge k-1
        #(T_SETUP);
        SS_MA = 1'b1;            // Rising edge k: launches the next bit
        #(T_HALF);
      end
      
      SS_MA = 1'b0;              // Final falling edge (after STP)
      #(T_HALF);
      
      if(CDM)
      begin
        // CDM = '1': hold MA low through the slave timeout, then release
        // to idle. The release edge hits the re-armed slave as a phantom
        // frame start; the slave discards it after one (adaptive) timeout,
        // so keep MA idle for longer than that before the next frame.
        #(T_HALF);
        WaitSLIdle;
        #(T_HALF);
        SS_MA = 1'b1;
        #5000;
      end
      else
      begin
        SS_MA = 1'b1;            // CDM = '0': back to idle level
        WaitSLIdle;
        #(T_HALF);
      end
    end
  endtask
  
  // Check one full SCD frame against the expected pattern
  task automatic CheckFrame(input logic [MAX_PDUBIT-1:0] pdu, input logic cds);
    logic [VAL_CRCBIT-1:0] exp_crc;
    int i;
    logic ok;
    begin
      ok = 1'b1;
      if(Samples[0] !== 1'b0) begin ok = 0; $display("[%0t] FAIL: ACK bit != 0", $time); end
      if(Samples[1] !== 1'b1) begin ok = 0; $display("[%0t] FAIL: STR bit != 1", $time); end
      if(Samples[2] !== cds  ) begin ok = 0; $display("[%0t] FAIL: CDS bit %b != %b", $time, Samples[2], cds); end
      for(i = 0; i < PDU_BITS; i = i + 1)
        if(Samples[3+i] !== pdu[MAX_PDUBIT-1-i])
        begin
          ok = 0;
          $display("[%0t] FAIL: PDU bit %0d: %b != %b", $time, i, Samples[3+i], pdu[MAX_PDUBIT-1-i]);
        end
      exp_crc = crc6_f(pdu, PDU_BITS) ^ 6'h15;
      for(i = 0; i < VAL_CRCBIT; i = i + 1)
        if(Samples[3+PDU_BITS+i] !== ~exp_crc[VAL_CRCBIT-1-i])
        begin
          ok = 0;
          $display("[%0t] FAIL: CRC bit %0d: %b != %b", $time, i, 
                   Samples[3+PDU_BITS+i], ~exp_crc[VAL_CRCBIT-1-i]);
        end
      if(!ok)
        ErrCnt = ErrCnt + 1;
    end
  endtask
  
  // Send one CDM bit, return the CDS bit sampled in that frame
  task automatic CDMBit(input logic b, output logic cds);
    begin
      BissFrame(b);
      cds = Samples[2];
    end
  endtask
  
  // Header common to all control frames (16 CDM bits: S, CTS, 10-bit
  // addressing, inverted CRC4), with checks of the ID assignment response
  task automatic SendHeader(input logic cts, input logic [9:0] addr10);
    logic [3:0] hcrc;
    logic cds;
    int i;
    begin
      hcrc = crc4_f({cts, addr10});
      CDMBit(1'b1, cds);                      //  0: S
      if(cds !== 1'b0) begin ErrCnt++; $display("[%0t] FAIL: pre-start CDS != 0", $time); end
      CDMBit(cts, cds);                       //  1: CTS
      if(cds !== 1'b1) begin ErrCnt++; $display("[%0t] FAIL: IDL0 != 1", $time); end
      for(i = 9; i >= 0; i = i - 1)
        CDMBit(addr10[i], cds);               //  2..11: addressing (MSB = first on wire)
      for(i = 3; i >= 0; i = i - 1)
        CDMBit(~hcrc[i], cds);                // 12..15: CRC (inverted)
    end
  endtask
  
  // Single-byte register read with full response checking
  task automatic RegRead(input logic [6:0] addr, input logic [7:0] expdata);
    logic cds, pbit;
    logic [63:0] resp;
    logic [7:0] rdata;
    logic [3:0] rcrc;
    int k, i;
    begin
      SendHeader(1'b1, {3'b000, addr});       // CTS=1, ID=0, ADR
      
      CDMBit(1'b1, cds);                      // 17: R=1
      if(cds !== 1'b0) begin ErrCnt++; $display("[%0t] FAIL: IDL tail CDS != 0", $time); end
      CDMBit(1'b0, cds);                      // 18: W=0
      if(cds !== 1'b1) begin ErrCnt++; $display("[%0t] FAIL: R echo != 1", $time); end
      CDMBit(1'b1, cds);                      // 19: S (data phase)
      if(cds !== 1'b0) begin ErrCnt++; $display("[%0t] FAIL: W echo != 0", $time); end
      
      // Frames 20..32: don't-care CDM; CDS = S echo, DATA7..0, CRC3..0, P
      for(k = 0; k <= 12; k = k + 1)
      begin
        BissFrame(1'b0);
        resp[k] = Samples[2];
      end
      BissFrame(1'b0);                        // P frame
      pbit = Samples[2];
      
      // resp[0] = S echo, resp[1..8] = DATA7..0, resp[9..12] = CRC3..0
      if(resp[0] !== 1'b1)
      begin
        ErrCnt++;
        $display("[%0t] FAIL: read S echo %b != 1", $time, resp[0]);
      end
      rdata = '0;
      for(i = 0; i < 8; i = i + 1)
        rdata[i] = resp[8-i];
      if(rdata !== expdata)
      begin
        ErrCnt++;
        $display("[%0t] FAIL: read data %h != %h", $time, rdata, expdata);
      end
      rcrc = ~crc4_byte_f(expdata);
      for(i = 0; i < 4; i = i + 1)
        if(resp[9+i] !== rcrc[3-i])
        begin
          ErrCnt++;
          $display("[%0t] FAIL: read data CRC bit %0d: %b != %b", $time, i, 
                   resp[9+i], rcrc[3-i]);
        end
      if(pbit !== 1'b1)
      begin
        ErrCnt++;
        $display("[%0t] FAIL: read stop bit P %b != 1 (no continuation)", $time, pbit);
      end
      
      // Cleanup: 14 CDM='0' frames reset the control communication
      for(k = 0; k < 14; k = k + 1)
        BissFrame(1'b0);
    end
  endtask
  
  // Single-byte register write with full response checking. The slave
  // echoes R, W, S, then repeats the received DATA and CRC bits; each echo
  // bit is the CDS bit of the frame AFTER the corresponding CDM bit.
  task automatic RegWrite(input logic [6:0] addr, input logic [7:0] data);
    logic cds, pbit;
    logic [63:0] resp;
    logic [7:0] wdata;
    logic [3:0] wcrc;
    int i;
    begin
      SendHeader(1'b1, {3'b000, addr});       // CTS=1, ID=0, ADR
      wcrc = ~crc4_byte_f(data);              // Inverted data CRC on the wire
      
      CDMBit(1'b0, cds);                      // 16: R=0 (CDS = header tail '0')
      if(cds !== 1'b0) begin ErrCnt++; $display("[%0t] FAIL: IDL tail CDS != 0", $time); end
      CDMBit(1'b1, cds);                      // 17: W=1 (CDS = R echo)
      if(cds !== 1'b0) begin ErrCnt++; $display("[%0t] FAIL: R echo != 0", $time); end
      CDMBit(1'b1, cds);                      // 18: S (CDS = W echo, must be '1')
      if(cds !== 1'b1) begin ErrCnt++; $display("[%0t] FAIL: W echo != 1", $time); end
      
      // The slave echo trails each CDM bit by one frame: the CDS sampled
      // while sending bit n carries the echo of bit n-1.
      //   frame 19 (DATA7) -> S echo, frames 20..27 -> DATA7..0 echoes,
      //   frames 28..31 -> CRC3..0 echoes, frame 32 -> stop bit P
      for(i = 7; i >= 1; i = i - 1)
      begin
        CDMBit(data[i], cds);                 // frames 19..25
        resp[7-i] = cds;
      end
      CDMBit(data[0], cds); resp[7]  = cds;   // frame 26
      CDMBit(wcrc[3], cds); resp[8] = cds;    // frame 27
      CDMBit(wcrc[2], cds); resp[9] = cds;    // frame 28
      CDMBit(wcrc[1], cds); resp[10] = cds;   // frame 29
      CDMBit(wcrc[0], cds); resp[11] = cds;   // frame 30
      CDMBit(1'b0, cds); resp[12] = cds;      // frame 31: master stop bit
      BissFrame(1'b0);                        // frame 32: stop bit P
      resp[13] = Samples[2];
      pbit = resp[13];
      
      // resp[0] = S echo, resp[1..8] = DATA7..0, resp[9..12] = CRC3..0, resp[13] = P
      if(resp[0] !== 1'b1)
      begin
        ErrCnt++;
        $display("[%0t] FAIL: write S echo %b != 1", $time, resp[0]);
      end
      wdata = '0;
      for(i = 0; i < 8; i = i + 1)
        wdata[i] = resp[8-i];                 // resp[1..8] = DATA7..0
      if(wdata !== data)
      begin
        ErrCnt++;
        $display("[%0t] FAIL: write data echo %h != %h", $time, wdata, data);
      end
      for(i = 0; i < 4; i = i + 1)
        if(resp[9+i] !== wcrc[3-i])
        begin
          ErrCnt++;
          $display("[%0t] FAIL: write CRC echo bit %0d", $time, i);
        end
      if(pbit !== 1'b1)
      begin
        ErrCnt++;
        $display("[%0t] FAIL: write stop bit P %b != 1", $time, pbit);
      end
      if(RegFile[addr] !== data)
      begin
        ErrCnt++;
        $display("[%0t] FAIL: register %02h not written (%h != %h)", 
                 $time, addr, RegFile[addr], data);
      end
      
      for(i = 0; i < 14; i = i + 1)
        BissFrame(1'b0);
    end
  endtask
  
  // Broadcast BiSS command frame
  task automatic BcastCmd(input logic [1:0] cmd);
    logic cds;
    int k;
    begin
      SendHeader(1'b0, {8'h00, cmd});         // CTS=0, IDS=0 (broadcast), CMD
      CDMBit(1'b1, cds);                      // Verification S
      CDMBit(1'b1, cds);                      // EX
      for(k = 0; k < 14; k = k + 1)
        BissFrame(1'b0);
    end
  endtask
  
  // SSI frame master: launch on falling edges, sample before rising edges
  task automatic SsiFrame(input logic [23:0] data);
    int k;
    logic [23:0] got;
    begin
      got = '0;
      for(k = 0; k < PDU_BITS; k = k + 1)
      begin
        SS_MA = 1'b0;              // Falling edge: slave launches bit k
        #(T_HALF - T_SETUP);
        got[PDU_BITS-1-k] = SS_SL; // Sample just before the rising edge
        #(T_SETUP);
        SS_MA = 1'b1;
        #(T_HALF);
      end
      SS_MA = 1'b1;                // Clock stops: monoflop timeout
      #5000;                       // Wait out the slave timeout (monoflop)
      if(got !== data)
      begin
        ErrCnt++;
        $display("[%0t] FAIL: SSI data %h != %h", $time, got, data);
      end
    end
  endtask
  
  // Hold-CDM master style: MA is held low through every timeout (CDM='1')
  // and released exactly at the next frame's LAT (no idle phase).
  task automatic HoldCDMBurst(input logic CDM, input int nframes);
    int f, k;
    begin
      for(f = 0; f < nframes; f++)
      begin
        FrameCnt = FrameCnt + 1;
        Samples = '0;
        
        if(SS_MA == 1'b1)
        begin
          SS_MA = 1'b0;            // Normal start from idle high
          #(T_HALF);
        end
        // else: MA is still low from the previous frame's CDM hold
        
        SS_MA = 1'b1;              // LAT (from a hold this is also the CDM release)
        #(T_HALF);
        
        for(k = 2; k <= CurEdges; k = k + 1)
        begin
          SS_MA = 1'b0;
          #(T_HALF - T_SETUP);
          Samples[k-2] = SS_SL;
          #(T_SETUP);
          SS_MA = 1'b1;
          #(T_HALF);
        end
        
        SS_MA = 1'b0;              // Final falling edge
        #(T_HALF);
        
        if(CDM)
        begin
          WaitSLIdle;              // Hold low through the slave timeout
          #(T_HALF);               // Next rising edge = next LAT
        end
        else
        begin
          SS_MA = 1'b1;
          WaitSLIdle;
          #(T_HALF);
        end
        // CDM='1' bits legitimately start a control data frame: the frame
        // after the first '1' carries IDL0='1' on CDS
        CheckFrame(Send_PDUBits, (CDM && (f == 1))? 1'b1 : 1'b0);
      end
    end
  endtask
  
  //---------------------------------------------------------------------------
  // Test sequence
  //---------------------------------------------------------------------------
  
  initial
  begin
    RST = 1'b1;
    Start = 1'b0;
    Clear_Status = 1'b0;
    SET_FSM_Enable = 1'b1;
    SET_InterfaceMode = 1'b0;
    SET_FSM_Continuous = 1'b1;
    SET_EN_BiSS_CRC = 1'b1;
    SET_CRC_InclCDS = 1'b0;
    SET_Lvl_LaunchSL = 1'b1;
    SET_Lvl_SL_Idle = 1'b1;
    SET_PDUBits = PDU_BITS;
    SET_Baudrate = 2'd1;      // 1 MHz
    SET_EN_AdaptTO = 1'b1;
    SET_EN_HoldCDM = 1'b0;
    SET_Limit_TickTO = 'd0;
    SET_Limit_HalfMA = 'd0;
    SET_SlaveID = 4'd0;
    Send_PDUBits = '0;
    
    #200;
    RST = 1'b0;
    #100;
    
    // Prime the settings snapshot (same usage pattern as prim_BiSS_SSI_RX)
    Start = 1'b1; #20; Start = 1'b0;
    #3000; // Let the post-reset timeout expire and re-arm
    
    //---------------------------------------------------------------
    $display("=== T1: three BiSS-C data frames ===");
    Send_PDUBits = 24'hABCDEF << (MAX_PDUBIT - PDU_BITS);
    BissFrame(1'b0); CheckFrame(Send_PDUBits, 1'b0);
    Send_PDUBits = 24'h05A5A5 << (MAX_PDUBIT - PDU_BITS);
    BissFrame(1'b0); CheckFrame(Send_PDUBits, 1'b0);
    Send_PDUBits = 24'hFFFFFF << (MAX_PDUBIT - PDU_BITS);
    BissFrame(1'b0); CheckFrame(Send_PDUBits, 1'b0);
    $display("T1 done: frames=%0d errors=%0d", Count_Frame, Count_Error);
    
    //---------------------------------------------------------------
    $display("=== T2: register read (addr 0 = A5, addr 3 = 5A) ===");
    RegRead(7'd0, 8'hA5);
    RegRead(7'd3, 8'h5A);
    $display("T2 done: frames=%0d errors=%0d", Count_Frame, Count_Error);
    
    //---------------------------------------------------------------
    $display("=== T3: register write (addr 7 = 96) + readback ===");
    RegWrite(7'd7, 8'h96);
    RegRead(7'd7, 8'h96);
    $display("T3 done: frames=%0d errors=%0d", Count_Frame, Count_Error);
    
    //---------------------------------------------------------------
    $display("=== T4: broadcast CMD='00' -> short frame, then re-enable ===");
    BcastCmd(2'b00);
    // Short frame: ACK STR CDS STOP
    CurEdges = 4 + 1;
    Send_PDUBits = 24'h0BEEF0 << (MAX_PDUBIT - PDU_BITS);
    BissFrame(1'b0);
    begin : SHORTCHK
      if(Samples[0] !== 1'b0) begin ErrCnt++; $display("[%0t] FAIL: short ACK", $time); end
      if(Samples[1] !== 1'b1) begin ErrCnt++; $display("[%0t] FAIL: short STR", $time); end
      if(Samples[2] !== 1'b0) begin ErrCnt++; $display("[%0t] FAIL: short CDS", $time); end
      if(Samples[3] !== 1'b0) begin ErrCnt++; $display("[%0t] FAIL: short STOP fill", $time); end
    end
    // Register access must still work in short-frame mode (control comm active)
    RegRead(7'd0, 8'hA5);
    // Addressed CMD='00' (IDS0=1) re-activates the process data channels.
    // Addressed commands: after the verification S the master waits out the
    // 8-cycle IDA window before sending EX.
    begin : CMD00A
      logic cds;
      int k;
      SendHeader(1'b0, {8'b1000_0000, 2'b00});  // IDS0 set (first bit on wire)
      CDMBit(1'b1, cds);                      // Verification S
      for(k = 0; k < 7; k = k + 1)            // IDA window (frames 17..23)
        CDMBit(1'b0, cds);
      CDMBit(1'b1, cds);                      // EX: process data re-enabled
      CurEdges = N_EDGES;                     // Full frames from here on
      for(k = 0; k < 14; k = k + 1)
        BissFrame(1'b0);
    end
    Send_PDUBits = 24'h77EE55 << (MAX_PDUBIT - PDU_BITS);
    BissFrame(1'b0); CheckFrame(Send_PDUBits, 1'b0);
    $display("T4 done: frames=%0d errors=%0d", Count_Frame, Count_Error);
    
    //---------------------------------------------------------------
    $display("=== T5: SSI mode ===");
    SET_InterfaceMode = 1'b1;
    SET_Lvl_LaunchSL = 1'b0;   // SSI: launch on falling edges
    SET_EN_BiSS_CRC = 1'b0;
    Start = 1'b1; #20; Start = 1'b0; // snapshot
    #3000;
    Send_PDUBits = 24'h1234FF << (MAX_PDUBIT - PDU_BITS);
    SsiFrame(24'h1234FF);
    Send_PDUBits = 24'h000001 << (MAX_PDUBIT - PDU_BITS);
    SsiFrame(24'h000001);
    SET_InterfaceMode = 1'b0;
    SET_Lvl_LaunchSL = 1'b1;
    SET_EN_BiSS_CRC = 1'b1;
    Start = 1'b1; #20; Start = 1'b0;
    #3000;
    $display("T5 done: frames=%0d errors=%0d", Count_Frame, Count_Error);
    
    //---------------------------------------------------------------
    $display("=== T6: frame watchdog (MA stalls mid-frame) ===");
    begin
      int k;
      SS_MA = 1'b0; #(T_HALF);
      SS_MA = 1'b1; #(T_HALF);
      for(k = 2; k <= 10; k = k + 1)
      begin
        SS_MA = 1'b0; #(T_HALF);
        SS_MA = 1'b1; #(T_HALF);
      end
      SS_MA = 1'b1;             // Stall mid-frame
      #10000;                   // > adaptive timeout
      if(ERR_WdExp !== 1'b1)
      begin
        ErrCnt++;
        $display("[%0t] FAIL: ERR_WdExp not set after mid-frame stall", $time);
      end
      WaitSLIdle;
      Clear_Status = 1'b1; #20; Clear_Status = 1'b0;
      #500;
    end
    
    //---------------------------------------------------------------
    $display("=== T7: over-clock beyond STP ===");
    SET_Limit_HalfMA = 16'd10;
    Start = 1'b1; #20; Start = 1'b0;
    #3000;
    begin
      Send_PDUBits = 24'h0F0F0F << (MAX_PDUBIT - PDU_BITS);
      CurEdges = N_EDGES + 15;  // 15 extra clocked bit slots
      BissFrame(1'b0);
      CurEdges = N_EDGES;
      if(ERR_Overrun !== 1'b1)
      begin
        ErrCnt++;
        $display("[%0t] FAIL: ERR_Overrun not set after over-clocking", $time);
      end
      Clear_Status = 1'b1; #20; Clear_Status = 1'b0;
      SET_Limit_HalfMA = 16'd0;
      Start = 1'b1; #20; Start = 1'b0;
      #3000;
    end
    
    //---------------------------------------------------------------
    $display("=== T8: Hold-CDM master style ===");
    SET_EN_HoldCDM = 1'b1;
    Start = 1'b1; #20; Start = 1'b0;
    #3000;
    Send_PDUBits = 24'h13579B << (MAX_PDUBIT - PDU_BITS);
    HoldCDMBurst(1'b1, 4);          // Back-to-back frames, MA held low
    Send_PDUBits = 24'h2468AC << (MAX_PDUBIT - PDU_BITS);
    HoldCDMBurst(1'b0, 2);          // CDM='0' frames in between
    SET_EN_HoldCDM = 1'b0;
    Start = 1'b1; #20; Start = 1'b0;
    #3000;
    $display("T8 done: errors=%0d", Count_Error);
    
    //---------------------------------------------------------------
    $display("=== SUMMARY ===");
    if(ErrCnt == 0 && Count_Error == 0)
      $display("ALL TESTS PASSED (frames transmitted: %0d)", Count_Frame);
    else
      $display("TESTS FAILED: TB errors=%0d, DUT errors=%0d", ErrCnt, Count_Error);
    $finish;
  end
  
endmodule
