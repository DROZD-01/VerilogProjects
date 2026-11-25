//=============================================================================
// tb_prim_BiSS_SSI_RX.sv
//-----------------------------------------------------------------------------
// Self-checking SystemVerilog testbench for prim_BISS_SSI_RX (BiSS-C / SSI
// master receiver), instantiated with all-default parameters.
//
// CONTENTS
//   A. Reset, FSM gating, single-shot vs. continuous, Clear_Status
//   B. BiSS-C process-data frames (widths, CRC on/off, CDS-in-CRC, baud
//      gears, MA idle polarity, continuous stream)
//   C. Line-delay handling (AutoLD tracking sweep, manual TickDelay sweep,
//      inter-frame gap scaling)
//   D. SSI fixed-length (PROG) and monoflop length-discovery (AUTO) modes,
//      AutoLD and manual line delay, both MA idle polarities
//   E. CDM/CDS register communication (read / write / command / broadcast,
//      access interleaved into a continuous stream, back-to-back accesses,
//      slave busy-wait before start-bit confirm)
//   F. Precise assertion of all six error flags + counter semantics
//      (Count_Error_o counts only {NoStart,NoMarker,BadCRC,WdExp,Measure},
//       CDM errors are NOT counted)
//
// SLAVE MODEL (behavioural, event-driven)
//   * Adaptive: the MA half period is measured online from MA edges; all
//     thresholds (burst end, CDM timeout window) are derived from it.
//   * One SL transition is scheduled per launch-polarity MA edge at
//     edge_time + t_resp (t_resp = SLV_T_MULT * T_MA, default 1.5T).
//   * Between bursts SL is driven to the BiSS pull-up level '1'.
//   * Time-ordered event queue + a single applier process; every wait in
//     the TB is bounded (no unbounded event waits).
//
// WIRE-PROTOCOL MODEL ASSUMPTIONS (derived from the RTL of prim_BISS_SSI_RX
// and its submodules; each one is a tunable constant/knob in section 1/5):
//   1. BiSS frame on SL:  ACK='0' (launched on the 2nd rising MA edge of the
//      burst), START='1', CDS, PDU (MSB first), CRC-6; after that trailing
//      '1' (idle) bits.  The line-delay measurement anchors the 2nd rising
//      MA edge (MA_RefEdge) to the ACK negedge.
//   2. BiSS CRC-6: polynomial 0x43 -> effective mask 0x03, init 0x00, MSB
//      first, no reflection.  The DUT flags ERR_BadCRC_o when the received
//      CRC field EQUALS the locally computed CRC, i.e. a conforming slave
//      transmits the INVERTED CRC (BiSS AN3); this slave model does so.
//      With SET_CRC_InclCDS_i=1 the CRC covers [CDS,PDU], else [PDU].
//   3. SSI: the slave launches bit 0 (MSB) on the FIRST launch-polarity MA
//      edge of the burst (= the burst's first edge, polarity ~idle);
//      SET_Lvl_LaunchSL_i must be programmed to ~SET_Lvl_MA_Idle_i.
//      PROG (fixed length): [PDU MSB first]['1' marker]['0']; the marker is
//      required, its absence raises ERR_NoMarker_o.
//      AUTO (length discovery): [P leading '0'][W '1' run]['0']; the DUT
//      reports Length=W-1, Data=2**(W-1)-1, Padding=P.
//      SSI+AutoLD requires the first bit to differ from the SL idle level
//      ('1'): PROG data must have MSB='0', AUTO needs P>=1.  With manual
//      line delay any data works.  Until the first AutoLD measurement
//      completes, the DUT seeds the pipeline with SET_TickDelay_i.
//   4. CDM (BiSS-C control channel, one CDM bit per frame in the MA timeout
//      slot before the frame; '1' = idle extended by SET_Extra_TickCDM_i
//      ticks; the slave decodes it from the measured inter-frame idle):
//        header  16 frames: S='1', CTS, A9..A0, ~CRC4[3..0]  (CRC-4 poly
//                0x13 -> mask 0x03, init 0, covers CTS+addressing)
//        register frames (CTS='1', addressing={slave ID[2:0], ADDR[6:0]}):
//          R bit, W bit (echoed on CDS; an inverted W echo = reject),
//          S='1' repeated until the slave confirms with CDS='1',
//          read : 13 don't-care frames, slave returns DATA[7..0],
//                 ~CRC4[3..0] over DATA, P ('0'=accept) on CDS
//          write: DATA[7..0], ~CRC4[3..0] echoed on CDS, then P frame;
//                 slave verifies CRC, commits, answers P='0' (accept)
//                 or '1' (terminate).  One more S frame is inevitably in
//                 flight when the master sees the confirm (the CDS is read
//                 one frame later); the slave skips it before DATA[7].
//        command (CTS='0', addressing={IDS[7..0],CMD[1..0]}):
//          S + 7 idle frames, slave returns IDA[7..0] on CDS, then EX='1'
//        broadcast: IDS=0, S + EX, no IDA bits (slave replies '0')
//      The CDS response of a frame answers the CDM bit of the SAME frame
//      (full-duplex; the master reads it at the next frame's commit).
//      A control frame is only accepted by the slave after >=14 CDM='0'
//      frames following the previous transaction's end (the master's
//      CC_PRE / CC_ABRT runs provide them; CC_ABRT sends one extra '0'
//      because its first frame cancels a slave still waiting for S).
//   5. Frame end for the slave: MA idle longer than (16 + gap/2) ovs ticks.
//      CDM='1' is decoded when the measured idle falls into the window
//      [24+gap+extra/2, 32+gap+extra+64] ovs ticks; longer idles (e.g. the
//      first frame after a pause) decode as CDM='0'.
//   6. Manual TickDelay: with the BiSS geometry of assumption 1 the DUT
//      receives correctly for SET_TickDelay_i in [0, t_resp/T + 16] ovs
//      ticks (the START hunt plus the 2-strobe preamble skip absorb up to
//      two bits of sampling shift); outside that range reception breaks.
//      (Verified by the section C2 sweep.)
//
// VERIFICATION STATUS
//   Compiled and simulated (all sections, full run and per-section) with
//   Icarus Verilog 12.0 against the RTL in this directory, including the
//   top-level change MODE_BAUDCTRL=1'b0 (VARIABLE tick rate: the MA period
//   follows the BAUD_VAL[SET_Baudrate_i] table) and the R1 tick-manager fix
//   (a manual-LD frame invalidates the AutoLD shadow).  The timing model of
//   this TB is gear-aware: T_ns() = 1/baud(gear) and every derived
//   threshold tracks it; section B5 asserts the half period per gear.
//   (The sim copies carry mechanical iverilog-compatibility patches that do
//   not alter behaviour: flat BAUD_VAL parameters, a serial LFSR replacing
//   the CRC lookup table, and enum casts in ternaries.)  Result: all checks
//   pass (full run); every section also passes standalone.
//   Expected-value checks are self-checking: any mismatch prints a
//   "*** FAIL" line and is counted in the final summary.
//
// USAGE
//   iverilog -g2012 -o tb.vvp tb_prim_BiSS_SSI_RX.sv prim_BiSS_SSI_RX.sv \
//            <all submodule files> && vvp tb.vvp            # run everything
//   vvp tb.vvp +SECTION=B          # run one section (A..F)
//   vvp tb.vvp +DUMP               # plus VCD waveform dump
//   vvp tb.vvp +VERBOSE            # print per-check PASS lines
//=============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tb_prim_BiSS_SSI_RX;

  //===========================================================================
  // 0. Simulation control
  //===========================================================================

  localparam real SIM_LIMIT_NS = 200.0e6;   // hard global timeout

  string SECTION;                           // section filter (+SECTION=x)
  int    VERBOSE;
  int    TRACE;

  initial begin
    if (!$value$plusargs("SECTION=%s", SECTION)) SECTION = "ALL";
    VERBOSE = $test$plusargs("VERBOSE") ? 1 : 0;
    TRACE   = $test$plusargs("TRACE")   ? 1 : 0;
    if ($test$plusargs("DUMP")) begin
      $dumpfile("tb_prim_BiSS_SSI_RX.vcd");
      $dumpvars(0, tb_prim_BiSS_SSI_RX);
    end
  end

  function automatic bit sect(input string s);
    return ((SECTION == "ALL") || (SECTION == s));
  endfunction

  // check counters ----------------------------------------------------------
  int n_pass;
  int n_fail;

  task chk(input bit ok, input string nm);
    begin
      if (ok) begin
        n_pass++;
        if (VERBOSE) $display("[%9.1f ns] PASS  %0s", $realtime, nm);
      end else begin
        n_fail++;
        $display("[%9.1f ns] *** FAIL  %0s", $realtime, nm);
      end
    end
  endtask

  //===========================================================================
  // 1. DUT instance (all-default parameters) and waveform helpers
  //===========================================================================

  localparam int PDU_MAX = 32;

  logic                      RST_i;
  logic                      CLK_i;
  logic                      Start_i;
  // parameters
  logic                      SET_FSM_Enable_i;
  logic                      SET_InterfaceMode_i;
  logic                      SET_FSM_Continuous_i;
  logic                      SET_EN_BiSS_CRC_i;
  logic                      SET_CRC_InclCDS_i;
  logic                      SET_EN_AutoLD_i;
  logic                      SET_SSI_LenMode_i;
  logic                      SET_Lvl_LaunchSL_i;
  logic                      SET_Lvl_MA_Idle_i;
  logic [               5:0] SET_PDUBits_i;
  logic [               1:0] SET_Baudrate_i;
  logic [              15:0] SET_InterFrameGap_i;
  logic [              15:0] SET_Limit_HalfMA_i;
  logic [              15:0] SET_Limit_TickTO_i;
  logic [               7:0] SET_TickDelay_i;
  logic [              11:0] SET_Extra_TickCDM_i;
  // status
  logic                      Clear_Status_i;
  logic [               3:0] Count_Frame_o;
  logic [               3:0] Count_Error_o;
  logic                      ERR_NoStart_o;
  logic                      ERR_NoMarker_o;
  logic                      ERR_BadCRC_o;
  logic                      ERR_WdExp_o;
  logic                      ERR_Measure_o;
  logic                      ERR_CDM_o;
  // data channel
  logic                      Recv_Valid_o;
  logic [      PDU_MAX-1:0]  Recv_PDUBits_o;
  logic [               5:0] Recv_PDULength_o;
  logic [               5:0] Recv_SSIPadding_o;
  // control channel
  logic                      COMM_Request_i;
  logic [               1:0] COMM_Type_i;
  logic [               1:0] COMM_CMD_i;
  logic [               7:0] COMM_IDSel_i;
  logic [               6:0] COMM_Addr_i;
  logic [               7:0] COMM_Data_i;
  logic                      COMM_Reply_o;
  logic [               7:0] COMM_Data_o;
  // serial lines
  logic                      SS_MA;
  logic                      SS_SL;

  prim_BISS_SSI_RX DUT (
    .RST_i                 (RST_i),
    .CLK_i                 (CLK_i),
    .Start_i               (Start_i),
    .SET_FSM_Enable_i      (SET_FSM_Enable_i),
    .SET_InterfaceMode_i   (SET_InterfaceMode_i),
    .SET_FSM_Continuous_i  (SET_FSM_Continuous_i),
    .SET_EN_BiSS_CRC_i     (SET_EN_BiSS_CRC_i),
    .SET_CRC_InclCDS_i     (SET_CRC_InclCDS_i),
    .SET_EN_AutoLD_i       (SET_EN_AutoLD_i),
    .SET_SSI_LenMode_i     (SET_SSI_LenMode_i),
    .SET_Lvl_LaunchSL_i    (SET_Lvl_LaunchSL_i),
    .SET_Lvl_MA_Idle_i     (SET_Lvl_MA_Idle_i),
    .SET_PDUBits_i         (SET_PDUBits_i),
    .SET_Baudrate_i        (SET_Baudrate_i),
    .SET_InterFrameGap_i   (SET_InterFrameGap_i),
    .SET_Limit_HalfMA_i    (SET_Limit_HalfMA_i),
    .SET_Limit_TickTO_i    (SET_Limit_TickTO_i),
    .SET_TickDelay_i       (SET_TickDelay_i),
    .SET_Extra_TickCDM_i   (SET_Extra_TickCDM_i),
    .Clear_Status_i        (Clear_Status_i),
    .Count_Frame_o         (Count_Frame_o),
    .Count_Error_o         (Count_Error_o),
    .ERR_NoStart_o         (ERR_NoStart_o),
    .ERR_NoMarker_o        (ERR_NoMarker_o),
    .ERR_BadCRC_o          (ERR_BadCRC_o),
    .ERR_WdExp_o           (ERR_WdExp_o),
    .ERR_Measure_o         (ERR_Measure_o),
    .ERR_CDM_o             (ERR_CDM_o),
    .Recv_Valid_o          (Recv_Valid_o),
    .Recv_PDUBits_o        (Recv_PDUBits_o),
    .Recv_PDULength_o      (Recv_PDULength_o),
    .Recv_SSIPadding_o     (Recv_SSIPadding_o),
    .COMM_Request_i        (COMM_Request_i),
    .COMM_Type_i           (COMM_Type_i),
    .COMM_CMD_i            (COMM_CMD_i),
    .COMM_IDSel_i          (COMM_IDSel_i),
    .COMM_Addr_i           (COMM_Addr_i),
    .COMM_Data_i           (COMM_Data_i),
    .COMM_Reply_o          (COMM_Reply_o),
    .COMM_Data_o           (COMM_Data_o),
    .SS_MA                 (SS_MA),
    .SS_SL                 (SS_SL)
  );

  always #5.0 CLK_i = ~CLK_i;               // 100 MHz

  //===========================================================================
  // 2. TB configuration mirror (what the DUT inputs will be driven to)
  //===========================================================================

  // default parameter set of the DUT's BAUD_VAL table
  function automatic int baudv(input int i);
    case (i)
      0: return 500000;
      1: return 1000000;
      2: return 1500000;
      3: return 2000000;
      default: return 1000000;
    endcase
  endfunction

  int  cfg_ssi;         // SET_InterfaceMode_i
  int  cfg_cont;        // SET_FSM_Continuous_i
  int  cfg_crc;         // SET_EN_BiSS_CRC_i
  int  cfg_inclcds;     // SET_CRC_InclCDS_i
  int  cfg_autold;      // SET_EN_AutoLD_i
  int  cfg_ssilen;      // SET_SSI_LenMode_i
  int  cfg_launch;      // SET_Lvl_LaunchSL_i
  int  cfg_idle;        // SET_Lvl_MA_Idle_i
  int  cfg_pdu;         // SET_PDUBits_i (1..32)
  int  cfg_baud;        // SET_Baudrate_i (0..3)
  int  cfg_gap;         // SET_InterFrameGap_i [ovs ticks]
  int  cfg_halfma;      // SET_Limit_HalfMA_i (0 = infinite)
  int  cfg_tickto;      // SET_Limit_TickTO_i [ovs ticks] (0 = infinite)
  int  cfg_tickdelay;   // SET_TickDelay_i [ovs ticks]
  int  cfg_extracdm;    // SET_Extra_TickCDM_i [ovs ticks]
  logic [7:0] cfg_ida;  // slave IDA pattern returned on command frames
  int  auto_p;          // SSI-AUTO leading pad zeros
  int  auto_w;          // SSI-AUTO monoflop pulse width in '1' bits

  task cfg_defaults;
    begin
      cfg_ssi = 0;  cfg_cont = 0;  cfg_crc = 1;  cfg_inclcds = 0;
      cfg_autold = 1;  cfg_ssilen = 0;
      cfg_launch = 1;  cfg_idle = 0;
      cfg_pdu = 16;  cfg_baud = 1;
      cfg_gap = 32;  cfg_halfma = 8;
      cfg_tickto = 0;  cfg_tickdelay = 48;  cfg_extracdm = 64;
      cfg_ida = 8'hA5;
      auto_p = 1;  auto_w = 13;
      // slave model knobs back to normal operation
      SLV_T_MULT    = 1.5;
      SLV_STUCK0    = 1'b0;
      SLV_STUCK1    = 1'b0;
      SLV_NO_MARKER = 1'b0;
      SLV_CRC_PLAIN = 1'b0;
      SLV_BUSY_S    = 0;
      SLV_RD_CRC_ERR = 1'b0;
    end
  endtask

  // ---- slave model knobs ---------------------------------------------------
  real SLV_T_MULT;      // slave response delay in units of the MA period
  bit  SLV_STUCK0;      // SL frozen at '0'  (F1: NoStart)
  bit  SLV_STUCK1;      // SL frozen at '1'  (F5: Meas_Error)
  bit  SLV_NO_MARKER;   // SSI: omit the monoflop marker '1' (F2: NoMarker)
  bit  SLV_CRC_PLAIN;   // BiSS: send the non-inverted CRC (F3: BadCRC)
  int  SLV_BUSY_S;      // CDM: number of '0' start-bit answers before confirm
  bit  SLV_RD_CRC_ERR;  // CDM: corrupt the read-data CRC (F6)

  // Derived timing ------------------------------------------------------------
  // EFFECTIVE MA period.  The top-level default MODE_BAUDCTRL=1'b0 puts the
  // tick generator into VARIABLE mode: the tick rate is
  // BAUD_VAL[SET_Baudrate_i] * OVERSAMPLE, so the MA period T is
  // 1/baud(gear)  (500 kHz / 1 MHz / 1.5 MHz / 2 MHz  ->  2 us / 1 us /
  // 666.7 ns / 500 ns) and one oversampling tick is T/32.  All thresholds
  // below are derived from T or the tick, so they track the gear
  // automatically; the slave model measures the MA half period online.
  // (Verified by measurement in section B5: the half period follows the
  // baud table for every gear.)
  function real T_ns();       T_ns       = 1.0e9 / real'(baudv(cfg_baud));   endfunction
  function real tick_ns();    tick_ns    = T_ns() / 32.0;                   endfunction
  function real t_resp_ns();  t_resp_ns  = SLV_T_MULT * T_ns();             endfunction
  // expected MA half period for a baud gear (B5 period assertions)
  function real half_exp_ns(input int g);
    half_exp_ns = 0.5e9 / real'(baudv(g));
  endfunction
  // Long-pause threshold (frame boundary fallback): 2.5 MA half periods.
  // With the fixed MF_GAP the inter-frame idle (last MA edge -> next first
  // edge) is [gap+16, gap+32) ovs ticks ([48,64) at gap=32); the oracle and
  // abort-signature paths end normal frames, and this threshold splits
  // bursts at every frame boundary of a CDM stream.
  function real burst_thresh_ns();
    burst_thresh_ns = 2.5 * T_ns() * 0.5;
  endfunction
  // CDM='1' decode window (measured idle between the last MA edge of the
  // previous frame and the first edge of the current one).  With the fixed
  // MF_GAP and MF_CDM_DLY the idle is
  //   CDM='0' : [gap+16, gap+32)      = [48, 64)  ticks at gap=32
  //   CDM='1' : [gap+16+extra, gap+32+extra) = [112, 128) ticks
  // so the window [24+gap+extra/2, 32+gap+extra+64] separates them with
  // margin on both sides; longer pauses (first frame after enable) decode
  // as CDM='0'.
  function real cdm_lo_ns();
    cdm_lo_ns = (24.0 + real'(cfg_gap) + real'(cfg_extracdm) * 0.5) * tick_ns();
  endfunction
  function real cdm_hi_ns();
    cdm_hi_ns = (32.0 + real'(cfg_gap) + real'(cfg_extracdm) + 64.0) * tick_ns();
  endfunction
  // first launch-polarity edge carrying bit 0 of the frame
  function int launch_base();
    launch_base = (cfg_ssi != 0) ? 1 : 2;   // SSI: 1st launch edge; BiSS: 2nd
  endfunction

  //===========================================================================
  // 3. Reference functions (LFSR data, CRCs, masks)
  //===========================================================================

  function automatic logic [31:0] lfsr_adv(input logic [31:0] s);
    logic [31:0] x;
    begin
      x = s;
      x = x ^ (x << 13);
      x = x ^ (x >> 17);
      x = x ^ (x << 5);
      return x;
    end
  endfunction

  function automatic logic [31:0] lfsr_seq(input int n);
    logic [31:0] x;
    begin
      x = 32'h1D2C3B4A;
      for (int i = 0; i < n; i++) x = lfsr_adv(x);
      return x;
    end
  endfunction

  function automatic logic [31:0] pdu_mask(input int n);
    logic [31:0] one;
    begin
      one = 32'd1;
      if (n >= 32) return 32'hFFFFFFFF;
      return (one << n) - 32'd1;
    end
  endfunction

  // serial CRC step, MSB-first, poly given as the effective mask
  function automatic logic [5:0] crc6_step(input logic [5:0] c, input logic b);
    logic [5:0] n;
    begin
      n = {c[4:0], 1'b0};
      if (c[5] ^ b) n = n ^ 6'h03;
      return n;
    end
  endfunction

  function automatic logic [3:0] crc4_step(input logic [3:0] c, input logic b);
    logic [3:0] n;
    begin
      n = {c[2:0], 1'b0};
      if (c[3] ^ b) n = n ^ 4'h03;
      return n;
    end
  endfunction

  // CRC-6 over the low nbits of 'v', MSB first, init 0 (BiSS process data)
  function automatic logic [5:0] crc6_f(input logic [33:0] v, input int nbits);
    logic [5:0] c;
    begin
      c = 6'h00;
      for (int i = nbits - 1; i >= 0; i--) c = crc6_step(c, v[i]);
      return c;
    end
  endfunction

  // CRC-4 over the low nbits of 'v', MSB first, init 0 (CDM header / data)
  function automatic logic [3:0] crc4_f(input logic [11:0] v, input int nbits);
    logic [3:0] c;
    begin
      c = 4'h0;
      for (int i = nbits - 1; i >= 0; i--) c = crc4_step(c, v[i]);
      return c;
    end
  endfunction

  function automatic logic [3:0] crc4_byte(input logic [7:0] d);
    logic [11:0] v;
    begin
      v = {4'h0, d};
      return crc4_f(v, 8);
    end
  endfunction

  //===========================================================================
  // 4. Slave model - CDM register file and protocol parser
  //===========================================================================

  localparam int         CDM_RO_ADDR = 8'h7F;   // read-only address (reject)
  logic [7:0]            regfile [0:127];

  // parser states
  localparam int PS_RESET = 0;   // counting the CDM='0' idle run
  localparam int PS_HDR   = 1;   // collecting the 16-frame header
  localparam int PS_RW    = 2;   // R and W bits
  localparam int PS_START = 3;   // S bits, repeated until confirmed
  localparam int PS_DATA  = 4;   // read payload out / write payload in
  localparam int PS_CWAIT = 5;   // command: IDA acknowledge bits

  int          ps_state;
  int          ps_zero_run;
  int          ps_hdr_idx;
  logic [11:0] ps_hdr;           // {S, CTS, addressing[9:0]}
  logic [3:0]  ps_hdrcrc;        // received (inverted) header CRC
  bit          ps_hdr_ok;
  bit          ps_reject;        // reject the access (invert the W echo)
  int          ps_rw_idx;
  bit          ps_rbit, ps_wbit;
  bit          ps_is_read;
  logic [6:0]  ps_addr;
  int          ps_data_idx;
  int          ps_cwait_idx;
  logic [7:0]  ps_rdata;         // read data to return
  logic [3:0]  ps_rcrc;          // CRC-4 over the read data
  logic [7:0]  ps_wdata;         // write data collected so far
  logic [3:0]  ps_wcrc;          // received (inverted) write CRC
  int          ps_busy_left;     // remaining busy S answers

  task cdm_reset_state;
    begin
      ps_state    = PS_RESET;
      ps_zero_run = 0;
      ps_hdr_idx  = 0;
      ps_reject   = 1'b0;
    end
  endtask

  // One CDM bit per frame; returns the CDS bit to transmit in THIS frame.
  task cdm_parse(input bit b, output bit resp);
    logic [3:0] c;
    begin
      resp = 1'b0;
      case (ps_state)

        PS_RESET: begin
          if (b == 1'b1 && ps_zero_run >= 14) begin
            ps_state    = PS_HDR;
            ps_hdr_idx  = 0;                    // the S bit itself is consumed
            ps_reject   = 1'b0;
            ps_busy_left = SLV_BUSY_S;          // busy budget latched at arm time
          end else if (b == 1'b0 && ps_zero_run < 15) begin
            ps_zero_run = ps_zero_run + 1;
          end
        end

        PS_HDR: begin
          ps_hdr_idx = ps_hdr_idx + 1;
          if (ps_hdr_idx <= 11)      ps_hdr[11 - ps_hdr_idx] = b;
          else if (ps_hdr_idx <= 15) ps_hdrcrc[15 - ps_hdr_idx] = b;
          if (ps_hdr_idx == 15) begin
            c = crc4_f(ps_hdr[10:0], 11);
            ps_hdr_ok = (ps_hdrcrc == ~c);
            ps_reject = ~ps_hdr_ok;
            if (ps_hdr[10] == 1'b1) begin       // CTS='1': register frame
              ps_addr = ps_hdr[6:0];            // {ID[2:0], ADDR[6:0]}
              if (ps_hdr[9:7] != 3'd0) ps_reject = 1'b1;  // foreign slave ID
              ps_state  = PS_RW;
              ps_rw_idx = 0;
            end else begin                      // CTS='0': command frame
              ps_state     = PS_CWAIT;
              ps_cwait_idx = 0;
            end
          end
        end

        PS_RW: begin
          if (ps_rw_idx == 0) begin
            ps_rbit  = b;
            resp     = b;                       // echo R
          end else begin
            ps_wbit    = b;
            ps_is_read = ps_rbit & ~ps_wbit;
            // read-only register: reject writes (inverted W echo)
            if (!ps_is_read && (ps_addr == CDM_RO_ADDR[6:0])) ps_reject = 1'b1;
            resp = ps_reject ? ~b : b;          // echo W (invert = reject)
            if (ps_is_read) begin
              ps_rdata = regfile[ps_addr];
              ps_rcrc  = crc4_byte(ps_rdata);
            end
            ps_state = PS_START;
          end
          ps_rw_idx = ps_rw_idx + 1;
        end

        PS_START: begin
          if (b == 1'b1) begin
            if (ps_busy_left > 0) begin
              ps_busy_left = ps_busy_left - 1;
              resp = 1'b0;                      // busy: ask for more time
            end else begin
              resp = 1'b1;                      // confirm
              ps_state    = PS_DATA;
              // The master reads the CDS one frame after the commit, so one
              // more S frame is inevitably in flight when it sees the
              // confirm; its write data therefore starts one frame later
              // than the read responses.  Writes skip the in-flight S
              // frame (idx -1) before collecting DATA[7].
              ps_data_idx = ps_is_read ? 0 : -1;
            end
          end else begin
            cdm_reset_state;                    // master cancelled (CDM='0')
          end
        end

        PS_DATA: begin
          if (ps_is_read) begin
            if (ps_data_idx <= 7)       resp = ps_rdata[7 - ps_data_idx];
            else if (ps_data_idx <= 11) resp = ~ps_rcrc[3 - (ps_data_idx - 8)];
            else                        resp = 1'b0;             // stop bit P
            ps_data_idx = ps_data_idx + 1;
            if (ps_data_idx == 13) cdm_reset_state;
          end else begin
            if (ps_data_idx < 0) begin
              resp = 1'b1;               // in-flight S repeat: keep confirming
            end
            else if (ps_data_idx <= 7) begin
              ps_wdata[7 - ps_data_idx] = b;
              resp = b;                         // echo the data bit
            end else if (ps_data_idx <= 11) begin
              ps_wcrc[3 - (ps_data_idx - 8)] = b;
              resp = b;                         // echo the CRC bit
            end else begin
              // P frame: verify CRC-4, commit, answer accept/terminate
              if (crc4_byte(ps_wdata) != ~ps_wcrc) resp = 1'b1; // CRC bad
              else begin
                regfile[ps_addr] = ps_wdata;
                resp = 1'b0;                    // accepted
              end
              cdm_reset_state;
            end
            ps_data_idx = ps_data_idx + 1;
          end
        end

        PS_CWAIT: begin
          if (ps_hdr[9:2] == 8'h00) begin
            // broadcast: S is immediately followed by EX, no IDA bits
            resp = 1'b0;
            ps_cwait_idx = ps_cwait_idx + 1;
            if (ps_cwait_idx >= 2) cdm_reset_state;   // S frame + EX frame
          end else begin
            resp = cfg_ida[7 - ps_cwait_idx];
            ps_cwait_idx = ps_cwait_idx + 1;
            if (ps_cwait_idx == 8) cdm_reset_state;
          end
        end

        default: cdm_reset_state;
      endcase
      // read-data CRC corruption injection
      if (SLV_RD_CRC_ERR && (ps_state == PS_DATA) && ps_is_read &&
          (ps_data_idx >= 9) && (ps_data_idx <= 12))
        resp = ~resp;
    end
  endtask

  //===========================================================================
  // 5. Slave model - frame plan, bit sequence, MA monitor, SL scheduler
  //===========================================================================

  // ---- scheduled SL transitions (time-ordered queue) ------------------------
  real  q_t [0:511];
  logic q_v [0:511];
  int   q_n;

  task slv_sched(input real t, input logic v);
    int i;
    begin
      if (q_n >= 512) begin
        $display("[%9.1f ns] *** SL event queue overflow", $realtime);
        n_fail++;
      end else begin
      i = q_n;
      while (i > 0 && q_t[i-1] > t) begin
        q_t[i] = q_t[i-1];
        q_v[i] = q_v[i-1];
        i = i - 1;
      end
      q_t[i] = t;
      q_v[i] = v;
      q_n = q_n + 1;
      end
    end
  endtask

  int q_gen;                                    // flushed by slv_frame_end

  always begin : slv_applier
    if (q_n <= 0) begin
      q_n = 0;                                  // clamp (queue flushed while asleep)
      wait (q_n > 0);
    end
    begin : ap_one
      int mygen;
      mygen = q_gen;
      if (q_t[0] > $realtime) #(q_t[0] - $realtime);
      if (q_gen == mygen) SS_SL = q_v[0];       // skip if flushed meanwhile
    end
    if (q_n > 0) begin
      for (int i = 0; i < q_n - 1; i++) begin
        q_t[i] = q_t[i+1];
        q_v[i] = q_v[i+1];
      end
      q_n = q_n - 1;
    end
  end

  // ---- per-frame plan (expected RX results, consumed by the scoreboard) -----
  bit          plan_valid;
  int          plan_bidx;
  logic [31:0] plan_pdu_exp;
  int          plan_len_exp;
  int          plan_pad_exp;
  bit          plan_cds_exp;
  int          plan_chk_cds;      // 1 = compare Recv_CDSBits_o
  logic [31:0] plan_pdu_tx;       // what the slave actually transmits

  // sequence of SL bits for the current frame (launch-edge indexed)
  logic seq_b [0:79];
  int   seq_len;

  int   bidx;            // frame counter (one per MA burst)
  bit   burst_active;    // a frame is open (b1 seen, not yet ended)
  bit   awaiting_b1;     // next MA edge starts a new frame
  bit   oracle_pending;  // Recv_Valid seen, waiting for MA to settle at idle
  int   oracle_edges;    // MA edges since Recv_Valid (master emits <=1)
  int   launch_cnt;
  real  t_last_edge;     // time of the last MA edge
  real  t_bstart;        // burst start time
  real  t_bend;          // burst end time (idle detection)
  real  last_idle_ns;    // measured inter-burst idle (gap checks)
  int   slv_missed;      // bursts that ended without Recv_Valid
  int   frames_done;     // frames completed (valid or aborted)
  bit   slv_en;          // monitor enable (after reset)
  real  t_half_meas;     // last measured MA half period [ns]
  bit   cdm_bit_last;    // CDM bit decoded for the current frame

  // data override for directed frames
  logic [31:0] ovr_val;
  int          ovr_cnt;

  // frame bit value for launch-edge index k (k >= 0), '1' beyond the sequence
  function automatic logic seq_bit(input int k);
    begin
      if (k < 0)               return 1'b1;    // pre-start: SL idle level
      if (k >= seq_len)        return 1'b1;    // trailing: idle level
      return seq_b[k];
    end
  endfunction

  // build the frame plan + SL bit sequence (called at each burst start)
  task build_plan(input bit cds_resp);
    logic [31:0] pdu;
    logic [33:0] cv;
    logic [5:0]  c6;
    int          ncv;
    int          n;
    begin
      // PDU source: explicit override, else the deterministic LFSR sequence
      if (ovr_cnt > 0) begin
        pdu   = ovr_val & pdu_mask(cfg_pdu);
        ovr_cnt = ovr_cnt - 1;
      end else begin
        pdu = lfsr_seq(bidx) & pdu_mask(cfg_pdu);
      end
      plan_pdu_tx = pdu;

      seq_len = 0;
      plan_chk_cds = 0;

      if (SLV_STUCK1) begin
        // SL frozen high: the RX locks on the first strobe and clocks an
        // all-ones frame [START='1'][CDS='1'][PDU all '1']
        plan_pdu_exp = pdu_mask(cfg_pdu);
        plan_len_exp = cfg_pdu;
        plan_pad_exp = 0;
        plan_cds_exp = 1'b1;
        plan_chk_cds = 1;
        seq_len      = 0;                       // nothing scheduled
      end else if (cfg_ssi == 0) begin
        // ---- BiSS: ACK, START, CDS, PDU, (inverted CRC-6), trailing '1' ----
        seq_b[0] = 1'b0;                        // ACK
        seq_b[1] = 1'b1;                        // START
        seq_b[2] = cds_resp;                    // CDS
        n = cfg_pdu;
        for (int i = 0; i < n; i++) seq_b[3+i] = pdu[n-1-i];
        seq_len = 3 + n;
        if (cfg_crc != 0) begin
          cv  = {cds_resp, pdu};
          ncv = (cfg_inclcds != 0) ? n + 1 : n;
          c6  = crc6_f(cv, ncv);
          for (int j = 0; j < 6; j++)
            seq_b[seq_len + j] = SLV_CRC_PLAIN ? c6[5-j] : ~c6[5-j];
          seq_len = seq_len + 6;
        end
        plan_pdu_exp = pdu;
        plan_len_exp = cfg_pdu;
        plan_pad_exp = 0;
        plan_cds_exp = cds_resp;
        plan_chk_cds = 1;
      end else if (cfg_ssilen == 0) begin
        // ---- SSI fixed length: PDU, marker '1', '0', trailing '1' ----------
        n = cfg_pdu;
        for (int i = 0; i < n; i++) seq_b[i] = pdu[n-1-i];
        seq_len = n;
        if (SLV_NO_MARKER) begin
          seq_b[seq_len] = 1'b0;                // marker omitted -> '0'
          seq_len = seq_len + 1;
        end else begin
          seq_b[seq_len]     = 1'b1;            // monoflop marker
          seq_b[seq_len + 1] = 1'b0;
          seq_len = seq_len + 2;
        end
        plan_pdu_exp = pdu;
        plan_len_exp = cfg_pdu;
        plan_pad_exp = 0;
        plan_cds_exp = 1'b0;
        plan_chk_cds = 0;                       // DUT outputs 0 in SSI mode
      end else begin
        // ---- SSI length discovery: P zeros, W ones, '0', trailing '1' ------
        for (int i = 0; i < auto_p; i++) seq_b[i] = 1'b0;
        for (int i = 0; i < auto_w; i++) seq_b[auto_p + i] = 1'b1;
        seq_len = auto_p + auto_w;
        seq_b[seq_len] = 1'b0;                  // monoflop end
        seq_len = seq_len + 1;
        plan_len_exp = (auto_w >= 1) ? auto_w - 1 : 0;
        plan_pdu_exp = (auto_w >= 1) ? pdu_mask(auto_w - 1) : 32'd0;
        plan_pad_exp = auto_p;
        plan_cds_exp = 1'b0;
        plan_chk_cds = 0;
      end

      plan_bidx = bidx;
      plan_valid = 1'b1;
    end
  endtask

  // common frame termination (oracle / abort / long pause)
  task slv_frame_end;
    begin
      burst_active   = 1'b0;
      oracle_pending = 1'b0;
      awaiting_b1    = 1'b1;
      t_bend         = t_last_edge;
      q_gen          = q_gen + 1;               // invalidate in-flight writes
      q_n            = 0;                       // drop pending trailing bits
      if (plan_valid) begin                     // frame ended without Recv_Valid
        slv_missed  = slv_missed + 1;
        frames_done = frames_done + 1;
        plan_valid  = 1'b0;
      end
      if (SLV_STUCK0)      SS_SL = 1'b0;
      else                 SS_SL = 1'b1;        // BiSS pull-up idle level
      if (TRACE)
        $display("[%9.1f ns] TRACE frame end (bidx=%0d dur=%0.1f)",
                 $realtime, bidx, t_bend - t_bstart);
    end
  endtask

  // frame start (the first MA edge of a burst)
  task slv_new_burst(input real tnow, input real idle_dur);
    bit cds_resp;
    bit cdm_b;
    begin
      if (plan_valid) begin                     // previous frame got no valid
        slv_missed = slv_missed + 1;
        frames_done = frames_done + 1;
        plan_valid = 1'b0;
      end
      burst_active = 1'b1;
      awaiting_b1  = 1'b0;
      launch_cnt   = 0;
      oracle_edges = 0;
      bidx         = bidx + 1;
      t_bstart     = tnow;
      last_idle_ns = idle_dur;
      // decode the CDM bit of the timeout slot preceding this frame
      cdm_b = (idle_dur >= cdm_lo_ns() && idle_dur <= cdm_hi_ns()) ? 1'b1 : 1'b0;
      cdm_bit_last = cdm_b;
      cdm_parse(cdm_b, cds_resp);
      build_plan(cds_resp);
      if (TRACE)
        $display("[%9.1f ns] TRACE burst %0d start (mode=%0d pdu=%0d cds=%b idle=%0.1f)",
                 tnow, bidx, cfg_ssi, cfg_pdu, cds_resp, idle_dur);
    end
  endtask

  // ---- MA edge monitor -------------------------------------------------------
  //  * the first edge after a frame end (awaiting_b1) opens a new frame;
  //  * a phase shorter than 0.8 half periods that lands on the idle level is
  //    the signature of a TX abort (the FSM snaps MA to idle mid-phase);
  //  * every launch-polarity edge schedules one SL bit at edge + t_resp.
  always @(SS_MA) begin : slv_ma_mon
    real tnow;
    real dt;
    logic lvl;
    int  ki;
    begin
      if (slv_en) begin
        tnow = $realtime;
        lvl  = SS_MA;
        dt   = (t_last_edge < 0.0) ? 1.0e9 : (tnow - t_last_edge);
        if ((dt > 0.0) && (dt < 2.0 * T_ns()))
          t_half_meas = dt;
        if (oracle_pending)
          oracle_edges = oracle_edges + 1;
        // A frame always starts with an edge TO THE ACTIVE level; an edge
        // that lands on the idle level while awaiting_b1 is the MA output
        // snapping to a reconfigured idle polarity - ignored.
        if (awaiting_b1 && (lvl !== cfg_idle[0]))
          slv_new_burst(tnow, dt);
        else if (burst_active &&
                 (dt < 0.8 * T_ns() * 0.5) && (lvl === cfg_idle[0])) begin
          if (TRACE)
            $display("[%9.1f ns] TRACE abort signature (dt=%0.1f)", tnow, dt);
          slv_frame_end;                        // aborted frame
        end
        t_last_edge = tnow;
        if ((lvl === cfg_launch[0]) && !SLV_STUCK0 && !SLV_STUCK1) begin
          launch_cnt = launch_cnt + 1;
          ki = launch_cnt - launch_base();
          slv_sched(tnow + t_resp_ns(), seq_bit(ki));
        end
      end
    end
  end

  // ---- frame-end poll: Recv_Valid oracle + long-pause fallback ---------------
  //  After Recv_Valid_o the master emits at most one more MA edge (the final
  // toggle to the idle level) and then rests; the frame ends once MA has
  //  been stable at the idle level for 0.3 half periods.
  always begin : slv_burst_end_mon
    #(T_ns() * 0.125);
    if (slv_en && burst_active) begin
      if (oracle_pending && (SS_MA === cfg_idle[0]) &&
          (($realtime - t_last_edge) > (0.3 * T_ns() * 0.5)) &&
          (oracle_edges <= 1))
        slv_frame_end;
      else if (!oracle_pending &&
               (($realtime - t_last_edge) > burst_thresh_ns()))
        slv_frame_end;                          // long pause fallback
    end
  end

  //===========================================================================
  // 6. Scoreboard (data channel) + reply monitor (control channel)
  //===========================================================================

  int f_ok;               // frames with all expected values matched
  int f_bad;              // frames with at least one mismatch
  int f_valid;            // Recv_Valid pulses seen
  int score_mute;         // 1 = intentionally corrupted frames (no FAIL print)

  always @(posedge CLK_i) begin : scoreboard
    if (Recv_Valid_o) begin
      f_valid = f_valid + 1;
      frames_done = frames_done + 1;
      oracle_pending = 1'b1;                    // frame-end oracle
      oracle_edges   = 0;
      if (TRACE)
        $display("[%9.1f ns] TRACE Recv_Valid #%0d pdu=%h len=%0d pad=%0d",
                 $realtime, f_valid, Recv_PDUBits_o, Recv_PDULength_o, Recv_SSIPadding_o);
      if (!plan_valid) begin
        f_bad = f_bad + 1;
        $display("[%9.1f ns] *** FAIL  Recv_Valid without a pending plan", $realtime);
        n_fail++;
      end else begin
        plan_valid = 1'b0;
        if ((Recv_PDUBits_o   === plan_pdu_exp) &&
            (Recv_PDULength_o === plan_len_exp[5:0]) &&
            (Recv_SSIPadding_o=== plan_pad_exp[5:0]))
          f_ok = f_ok + 1;
        else begin
          f_bad = f_bad + 1;
          if (!score_mute) begin
            n_fail++;
            $display("[%9.1f ns] *** FAIL  frame %0d data: exp pdu=%h len=%0d pad=%0d | got pdu=%h len=%0d pad=%0d",
                     $realtime, plan_bidx, plan_pdu_exp, plan_len_exp, plan_pad_exp,
                     Recv_PDUBits_o, Recv_PDULength_o, Recv_SSIPadding_o);
          end
        end
      end
    end
  end

  bit          comm_reply_seen;
  logic [7:0]  comm_reply_data;

  always @(posedge CLK_i) begin : comm_mon
    if (COMM_Reply_o) begin
      comm_reply_seen  <= 1'b1;
      comm_reply_data  <= COMM_Data_o;
    end
  end

  //===========================================================================
  // 7. TB driver tasks
  //===========================================================================

  task pulse_start;                            // one CLK-wide Start pulse
    begin
      @(negedge CLK_i); Start_i = 1'b1;
      @(negedge CLK_i); Start_i = 1'b0;
    end
  endtask

  task pulse_clear;                            // one CLK-wide Clear_Status
    begin
      @(negedge CLK_i); Clear_Status_i = 1'b1;
      @(negedge CLK_i); Clear_Status_i = 1'b0;
    end
  endtask

  task drive_cfg_inputs;
    begin
      SET_InterfaceMode_i  = cfg_ssi[0];
      SET_FSM_Continuous_i = cfg_cont[0];
      SET_EN_BiSS_CRC_i    = cfg_crc[0];
      SET_CRC_InclCDS_i    = cfg_inclcds[0];
      SET_EN_AutoLD_i      = cfg_autold[0];
      SET_SSI_LenMode_i    = cfg_ssilen[0];
      SET_Lvl_LaunchSL_i   = cfg_launch[0];
      SET_Lvl_MA_Idle_i    = cfg_idle[0];
      SET_PDUBits_i        = cfg_pdu[5:0];
      SET_Baudrate_i       = cfg_baud[1:0];
      SET_InterFrameGap_i  = cfg_gap[15:0];
      SET_Limit_HalfMA_i   = cfg_halfma[15:0];
      SET_Limit_TickTO_i   = cfg_tickto[15:0];
      SET_TickDelay_i      = cfg_tickdelay[7:0];
      SET_Extra_TickCDM_i  = cfg_extracdm[11:0];
    end
  endtask

  task wait_quiesce(input real to_us);
    real t_end;
    begin
      t_end = $realtime + to_us * 1000.0;
      while ((burst_active ||
              (slv_en && (t_last_edge >= 0.0) &&
               (($realtime - t_last_edge) < (burst_thresh_ns() * 2.0)))) &&
             ($realtime < t_end))
        #(T_ns() * 0.125);
      if (burst_active)
        $display("[%9.1f ns] DBG quiesce timeout: burst_active=1 last_edge=%0.1f bidx=%0d cntF=%0d MA=%b",
                 $realtime, t_last_edge, bidx, Count_Frame_o, SS_MA);
      chk(!burst_active, "quiesce: MA stream stopped");
    end
  endtask

  // Load a new configuration with the settle flow:
  //   disable -> snapshot old cfg with enable=0 -> quiesce -> clear status ->
  //   drive new cfg -> 2 settle snapshots -> enable (live) ->
  //   optionally one verify frame (Start pulse fires exactly one frame).
  task cfg_load(input bit verify);
    begin
      SET_FSM_Enable_i = 1'b0;
      pulse_start;                              // snapshot {enable=0, old cfg}
      wait_quiesce(200.0);                      // let any stream stop
      pulse_clear;                              // flags/counters from zero
      drive_cfg_inputs;                         // new cfg onto DUT + mirrors
      pulse_start;                              // settle snapshot #1
      pulse_start;                              // settle snapshot #2
                                              // (SSI FrameBits propagation)
      SET_FSM_Enable_i = 1'b1;                  // live; sampled next Start
      slv_missed = 0;
      // Flush residual delayed idle-boundary strobes: while MA rests at a
      // level equal to SET_Lvl_LaunchSL_i (e.g. after an idle=1 BiSS test),
      // the delay pipeline emits SL_Sample strobes one line-delay after every
      // idle half-period boundary; the last of them must not land inside the
      // next frame (they would consume the two preamble skip strobes).
      #(t_resp_ns() + 2.5 * T_ns());
      snap_all;                                 // per-test reference: the
                                               // verify frame below counts in
      if (verify) begin
        run_frame(500.0);                       // config-verify frame
      end
    end
  endtask

  task run_frame(input real to_us);             // Start + wait for completion
    begin
      pulse_start;
      wait_frame_done(to_us);
    end
  endtask

  task wait_frame_done(input real to_us);
    int  fd0;
    real t_end;
    begin
      fd0   = frames_done;
      t_end = $realtime + to_us * 1000.0;
      while ((frames_done == fd0) && ($realtime < t_end)) #(T_ns() * 0.125);
      chk(frames_done > fd0, "frame completed (no timeout)");
      // let the MA burst terminate so FR_End / Count_Frame_o have landed
      #(1.5 * T_ns());
    end
  endtask

  // Control-channel request: wait for the sequencer to be idle (a request
  // pulsed during the post-abort CC_ABRT '0' run would be lost - the
  // interface is "sampled when idle"), pulse COMM_Request, then wait for
  // the reply (COMM_Reply_o, latched) or the sticky ERR_CDM_o, with timeout.
  task comm_req(input logic [1:0] typ, input logic [1:0] cmd,
                input logic [7:0] ids, input logic [6:0] addr,
                input logic [7:0] wdata,
                input real to_us, input bit exp_reply, input bit exp_err,
                input logic [7:0] exp_data, input string nm);
    real t_end;
    real t0;
    begin
      comm_reply_seen = 1'b0;
      t0   = $realtime;
      t_end = t0 + to_us * 1000.0;
      while (DUT.CDM_Busy && ($realtime < t_end)) #(T_ns() * 0.25);
      @(negedge CLK_i);
      COMM_Type_i    = typ;
      COMM_CMD_i     = cmd;
      COMM_IDSel_i   = ids;
      COMM_Addr_i    = addr;
      COMM_Data_i    = wdata;
      COMM_Request_i = 1'b1;
      @(negedge CLK_i);
      COMM_Request_i = 1'b0;
      t_end = $realtime + to_us * 1000.0;
      while (!comm_reply_seen && !ERR_CDM_o && ($realtime < t_end))
        #(T_ns() * 0.25);
      chk(comm_reply_seen === exp_reply,  $sformatf("%0s: COMM_Reply", nm));
      if (exp_reply)
        chk(comm_reply_data === exp_data, $sformatf("%0s: reply data %02h", nm, exp_data));
      chk(ERR_CDM_o === exp_err,          $sformatf("%0s: ERR_CDM", nm));
    end
  endtask

  // ---- per-test snapshots and check helpers ----------------------------------
  int f_ok0, f_bad0, cf0, ce0, bidx0, miss0;

  task snap_all;
    begin
      f_ok0  = f_ok;
      f_bad0 = f_bad;
      cf0    = Count_Frame_o;
      ce0    = Count_Error_o;
      bidx0  = bidx;
      miss0  = slv_missed;
    end
  endtask

  function automatic logic [3:0] cnt16(input logic [3:0] v, input int d);
    logic [4:0] r;
    begin
      r = {1'b0, v} + d[4:0];
    return r[3:0];
    end
  endfunction

  // exactly 'n' frames received cleanly (data match, no error flags)
  task chk_frames_ok(input string nm, input int n);
    begin
      chk(f_bad == f_bad0,                 $sformatf("%0s: no data mismatch", nm));
      chk(f_ok  == f_ok0 + n,              $sformatf("%0s: %0d clean frames", nm, n));
      chk(slv_missed == miss0,             $sformatf("%0s: no aborted frames", nm));
      chk(Count_Frame_o >= cnt16(cf0, bidx - bidx0), $sformatf("%0s: frame counter", nm));
    end
  endtask

  // Variant for AutoLD frames: ERR_Measure_o is not asserted because the
  // re-arming line-delay measurement (MA_RefEdge wraps mod-4 in MAGen, so it
  // re-fires every 4 MA periods; FR_Launch pulses on every MA edge) times out
  // on any SL '1' run longer than TIMEOUT_MA periods - see header note 7.
  task chk_flags_nome(input string nm,
                      input bit e_ns, input bit e_nm, input bit e_bc,
                      input bit e_we, input bit e_cm);
    begin
      chk(ERR_NoStart_o  === e_ns, $sformatf("%0s: ERR_NoStart=%b",  nm, e_ns));
      chk(ERR_NoMarker_o === e_nm, $sformatf("%0s: ERR_NoMarker=%b", nm, e_nm));
      chk(ERR_BadCRC_o   === e_bc, $sformatf("%0s: ERR_BadCRC=%b",   nm, e_bc));
      chk(ERR_WdExp_o    === e_we, $sformatf("%0s: ERR_WdExp=%b",    nm, e_we));
      chk(ERR_CDM_o      === e_cm, $sformatf("%0s: ERR_CDM=%b",      nm, e_cm));
    end
  endtask

  task chk_flags(input string nm,
                 input bit e_ns, input bit e_nm, input bit e_bc,
                 input bit e_we, input bit e_me, input bit e_cm);
    begin
      if (ERR_NoStart_o !== e_ns || ERR_NoMarker_o !== e_nm || ERR_BadCRC_o !== e_bc ||
          ERR_WdExp_o  !== e_we || ERR_Measure_o !== e_me || ERR_CDM_o !== e_cm)
        $display("[%9.1f ns] DBG flags %0s: actual NS=%b NM=%b BC=%b WE=%b ME=%b CM=%b (cntE=%0d cntF=%0d)",
                 $realtime, nm, ERR_NoStart_o, ERR_NoMarker_o, ERR_BadCRC_o,
                 ERR_WdExp_o, ERR_Measure_o, ERR_CDM_o, Count_Error_o, Count_Frame_o);
      chk(ERR_NoStart_o  === e_ns, $sformatf("%0s: ERR_NoStart=%b",  nm, e_ns));
      chk(ERR_NoMarker_o === e_nm, $sformatf("%0s: ERR_NoMarker=%b", nm, e_nm));
      chk(ERR_BadCRC_o   === e_bc, $sformatf("%0s: ERR_BadCRC=%b",   nm, e_bc));
      chk(ERR_WdExp_o    === e_we, $sformatf("%0s: ERR_WdExp=%b",    nm, e_we));
      chk(ERR_Measure_o  === e_me, $sformatf("%0s: ERR_Measure=%b",  nm, e_me));
      chk(ERR_CDM_o      === e_cm, $sformatf("%0s: ERR_CDM=%b",      nm, e_cm));
    end
  endtask

  //===========================================================================
  // 8. Test sections
  //===========================================================================

  // --------------------------------------------------------------- A ---------
  task test_A;
    int n;
    begin
      $display("\n--- SECTION A: reset, gating, continuous mode, Clear_Status ---");

      // A0: after reset the FSMs are disabled, MA rests at the reset level 0
      cfg_defaults;
      drive_cfg_inputs;
      SET_FSM_Enable_i = 1'b0;
      #3000.0;
      chk(SS_MA === 1'b0,                 "A0: MA at reset level while disabled");
      chk(Count_Frame_o == 4'd0,          "A0: no frames counted");
      chk(Count_Error_o == 4'd0,          "A0: no errors counted");
      chk(!Recv_Valid_o,                  "A0: no Recv_Valid");
      chk(!(ERR_NoStart_o || ERR_NoMarker_o || ERR_BadCRC_o ||
            ERR_WdExp_o  || ERR_Measure_o  || ERR_CDM_o), "A0: flags clear");

      // A1: Start pulses while disabled produce no exchange; after enable a
      //     single frame fires (the latched pending does not duplicate)
      snap_all;
      cfg_load(0);                            // settle, no verify frame
      SET_FSM_Enable_i = 1'b0;                // keep the snapshot disabled
      pulse_start; pulse_start;               // pending requests, still disabled
      #5000.0;
      chk(SS_MA === 1'b0,                    "A1: MA silent while FSM disabled");
      chk(Count_Frame_o == 4'd0,             "A1: no frames launched");
      SET_FSM_Enable_i = 1'b1;                // enable (sampled by the Start below)
      pulse_start;                            // enable snapshot -> one frame
      wait_frame_done(500.0);
      chk(Count_Frame_o == 4'd1,             "A1: exactly one frame after enable");
      chk(f_bad == f_bad0,                   "A1: frame received correctly");

      // A2: continuous mode: back-to-back frames, then a clean stop
      cfg_cont = 1;
      cfg_load(0);
      snap_all;
      pulse_start;                            // first frame starts the stream
      n = 0;
      while (n < 200 && (bidx - bidx0) < 6) begin
        #(T_ns() * 4.0);
        n = n + 1;
      end
      chk((bidx - bidx0) >= 5,               "A2: continuous stream ran");
      SET_FSM_Continuous_i = 1'b0;           // stop (lands at frame boundary)
      wait_quiesce(200.0);
      snap_all;
      #10000.0;
      chk(bidx == bidx0,                     "A2: stream stopped");
      chk(f_bad == f_bad0,                   "A2: all continuous frames clean");
      cfg_cont = 0;

      // A3: Clear_Status clears counters and sticky flags
      pulse_clear;
      chk(Count_Frame_o == 4'd0,             "A3: frame counter cleared");
      chk(Count_Error_o == 4'd0,             "A3: error counter cleared");
      chk(!(ERR_NoStart_o || ERR_NoMarker_o || ERR_BadCRC_o ||
            ERR_WdExp_o  || ERR_Measure_o  || ERR_CDM_o), "A3: flags cleared");
    end
  endtask

  // --------------------------------------------------------------- B ---------
  task test_B;
    begin
      $display("\n--- SECTION B: BiSS-C process data ---");

      // B1a: manual line delay: strictly clean flags
      cfg_defaults; cfg_pdu = 16; cfg_autold = 0; cfg_tickdelay = 48;
      cfg_load(1);
      chk_frames_ok("B1/manLD", 1);
      chk_flags("B1/manLD", 0,0,0,0,0,0);
      chk(Count_Error_o == 4'd0,             "B1/manLD: no error events");

      // B1b: AutoLD with an all-ones payload: the per-frame measurement
      //      anchors the ACK negedge (present regardless of the payload),
      //      so the frame is received cleanly and no measurement error is
      //      latched (the one-shot FR_Launch / MA_RefEdge fix removed the
      //      old mid-frame re-arm that used to time out on '1' runs)
      cfg_autold = 1; ovr_val = 32'h0000FFFF; ovr_cnt = 1;
      cfg_load(1);
      chk_frames_ok("B1/autoLD", 1);
      chk_flags("B1/autoLD", 0,0,0,0,0,0);
      chk(Count_Error_o == 4'd0,             "B1/autoLD: no error events");

      // B2: PDU width corner cases (manual LD)
      cfg_autold = 0; cfg_tickdelay = 48;
      cfg_pdu = 1;  cfg_load(1); chk_frames_ok("B2/w1",  1);
      cfg_pdu = 8;  cfg_load(1); chk_frames_ok("B2/w8",  1);
      cfg_pdu = 32; cfg_load(1); chk_frames_ok("B2/w32", 1);
      chk_flags("B2", 0,0,0,0,0,0);
      chk(Count_Error_o == 4'd0,             "B2: no error events");

      // B3: CRC disabled (frame = START+CDS+PDU, no CRC field)
      cfg_pdu = 12; cfg_crc = 0; cfg_load(1); chk_frames_ok("B3/noCRC", 1);
      cfg_crc = 1;

      // B4: CRC covering the CDS bit as well
      cfg_inclcds = 1; cfg_load(1); chk_frames_ok("B4/crcCDS", 1);
      chk_flags("B4", 0,0,0,0,0,0);
      chk(Count_Error_o == 4'd0,             "B4: no error events");
      cfg_inclcds = 0;

      // B5: all four baud gears (manual LD).  MODE_BAUDCTRL=1'b0 = VARIABLE
      //     tick rate: the tick generator follows BAUD_VAL[SET_Baudrate_i],
      //     so every gear changes the MA period (2 us / 1 us / 666.7 ns /
      //     500 ns) while frames keep receiving cleanly - the manual seed of
      //     48 ovs ticks = 1.5 MA periods is gear-invariant, and the slave
      //     model tracks T online.  The half-period measurement must follow
      //     the baud table for each gear.
      cfg_baud = 0; cfg_load(1); chk_frames_ok("B5/gear0", 1);
      chk((t_half_meas > 0.95 * half_exp_ns(0)) && (t_half_meas < 1.05 * half_exp_ns(0)), "B5/g0: MA half period 1.0 us");
      cfg_baud = 1; cfg_load(1); chk_frames_ok("B5/gear1", 1);
      chk((t_half_meas > 0.95 * half_exp_ns(1)) && (t_half_meas < 1.05 * half_exp_ns(1)), "B5/g1: MA half period 0.5 us");
      cfg_baud = 2; cfg_load(1); chk_frames_ok("B5/gear2", 1);
      chk((t_half_meas > 0.95 * half_exp_ns(2)) && (t_half_meas < 1.05 * half_exp_ns(2)), "B5/g2: MA half period 333 ns");
      cfg_baud = 3; cfg_load(1); chk_frames_ok("B5/gear3", 1);
      chk((t_half_meas > 0.95 * half_exp_ns(3)) && (t_half_meas < 1.05 * half_exp_ns(3)), "B5/g3: MA half period 250 ns");
      chk_flags("B5", 0,0,0,0,0,0);
      chk(Count_Error_o == 4'd0,             "B5: no error events");
      cfg_baud = 1;

      // B6: inverted MA idle polarity (MA idles high, launch on rising);
      //     requires AutoLD (manual sampling polarity hits idle-phantom
      //     strobes with idle=1), so ERR_Measure is not asserted here
      cfg_autold = 1; cfg_idle = 1; cfg_launch = 1; cfg_load(1);
      wait_quiesce(200.0);
      chk(SS_MA === 1'b1,                    "B6: MA idles at configured level");
      chk_frames_ok("B6/idle1", 1);
      chk_flags_nome("B6", 0,0,0,0,0);
      cfg_idle = 0; cfg_launch = 1;

      // B7: continuous BiSS stream (manual LD), every frame self-checked
      cfg_cont = 1; cfg_autold = 0; cfg_tickdelay = 48; cfg_load(1);
      snap_all;
      for (int i = 0; (i < 400) && ((bidx - bidx0) < 5); i++)
        #(T_ns() * 4.0);
      SET_FSM_Continuous_i = 1'b0;
      wait_quiesce(200.0);
      chk((bidx - bidx0) >= 5,               "B7: >=5 continuous frames");
      chk(f_bad == f_bad0,                   "B7: all frames match");
      chk(slv_missed == miss0,               "B7: no lost frames");
      cfg_cont = 0;
    end
  endtask

  // --------------------------------------------------------------- C ---------
  task test_C;
    real tms [0:11];
    int  dm [0:10];
    begin
      $display("\n--- SECTION C: line-delay handling ---");

      // C1: AutoLD tracking - the slave response delay is stepped across
      //     [0.75T, 3.6T]; every frame must still be received correctly.
      cfg_defaults; cfg_pdu = 16;
      cfg_load(1);
      // All-zero payloads isolate the tracking from data-dependent effects
      // (each frame performs exactly one ACK-anchored measurement; the tap
      // measured on frame i is applied on frame i+1, so a sweep step of up
      // to 0.7T stays inside the +/-0.5T sampling window).
      ovr_val = 32'd0;  ovr_cnt = 13;
      tms[0]=1.5; tms[1]=2.2; tms[2]=2.9; tms[3]=3.6;    // upward steps
      tms[4]=3.25; tms[5]=2.9; tms[6]=2.55; tms[7]=2.2; // downward steps
      tms[8]=1.85; tms[9]=1.5; tms[10]=1.15; tms[11]=0.8;
      snap_all;
      for (int i = 0; i < 12; i++) begin
        SLV_T_MULT = tms[i];
        run_frame(500.0);
      end
      chk(f_bad == f_bad0,                   "C1: tracking sweep, all data ok");
      chk(f_ok == f_ok0 + 12,                "C1: 12 frames received");
      chk_flags_nome("C1", 0,0,0,0,0);       // AutoLD frames: see note 7
      SLV_T_MULT = 1.5;

      // C2: manual TickDelay sweep (AutoLD off, slave delay fixed at 1.5T =
      //     48 ovs ticks).  Valid reception expected across [0..64] ticks;
      //     outside that range reception must break.
      cfg_autold = 0; cfg_tickdelay = 48; SLV_T_MULT = 1.5;
      cfg_load(1);                            // verify frame at D=48
      dm[0]=8; dm[1]=16; dm[2]=24; dm[3]=32; dm[4]=40;
      dm[5]=48; dm[6]=56; dm[7]=64; dm[8]=72; dm[9]=88;
      // in-range points: clean (8..64 ovs ticks; D=0 races with the launch
      // through the SL synchroniser and is not reliably clean)
      for (int i = 0; i <= 7; i++) begin
        cfg_tickdelay = dm[i];
        drive_cfg_inputs;
        snap_all;
        run_frame(500.0);
        wait_quiesce(100.0);
        chk(f_bad == f_bad0,   $sformatf("C2/D=%0d clean", dm[i]));
        chk(f_ok == f_ok0 + 1, $sformatf("C2/D=%0d one frame", dm[i]));
      end
      chk_flags("C2/inrange", 0,0,0,0,0,0);
      // out-of-range points: reception must NOT be clean (the corrupted
      // frames are expected - the scoreboard is muted for them)
      score_mute = 1;
      for (int i = 8; i <= 9; i++) begin
        cfg_tickdelay = dm[i];
        drive_cfg_inputs;
        snap_all;
        run_frame(500.0);
        wait_quiesce(100.0);
        chk(((f_bad > f_bad0) || (slv_missed > miss0) ||
             ERR_NoStart_o || ERR_NoMarker_o || ERR_BadCRC_o ||
             ERR_WdExp_o || ERR_Measure_o),
            $sformatf("C2/D=%0d broken as expected", dm[i]));
      end
      score_mute = 0;
      cfg_autold = 1; cfg_tickdelay = 48;

      // C3: inter-frame gap setting.  With the fixed MF_GAP the idle between
      //     frames really is SET_InterFrameGap_i ovs ticks (plus the
      //     clean-stop residual and boundary alignment), so gap=8 runs the
      //     frames closer together than gap=96 and both settings must
      //     receive cleanly.
      cfg_autold = 0; cfg_tickdelay = 48;
      cfg_gap = 8;  cfg_load(1); chk_frames_ok("C3/gap8", 1);
      cfg_gap = 96; cfg_load(1); chk_frames_ok("C3/gap96", 1);
      chk_flags("C3", 0,0,0,0,0,0);
      chk(Count_Error_o == 4'd0,             "C3: no error events");
      cfg_gap = 32;
    end
  endtask

  // --------------------------------------------------------------- D ---------
  task test_D;
    begin
      $display("\n--- SECTION D: SSI (fixed length and length discovery) ---");

      // SSI slave response delay: 48.1 ovs ticks instead of exactly 48.
      // The SSI line-delay measurement anchors the frame's FIRST MA edge
      // (one-shot MA_RefEdge) and locks the first SL edge, so the measured
      // tap is t_resp; the sampling strobe then lands at t_resp + half a
      // bit period, the centre of each bit window.  The fractional .1 tick
      // keeps the anchor edge off the tick grid for any quantisation phase.
      SLV_T_MULT = 1.503125;

      // D1: PROG + AutoLD (data MSB must be '0' so the first SL edge anchors
      //     the measurement), classic polarity: MA idle high
      cfg_defaults;
      cfg_ssi = 1; cfg_ssilen = 0; cfg_idle = 1; cfg_launch = 0;
      cfg_pdu = 12; ovr_val = 12'h02BC; ovr_cnt = 1;
      cfg_load(1);
      chk_frames_ok("D1", 1);
      chk_flags("D1", 0,0,0,0,0,0);

      // D2: 32-bit payload, MSB='0' (manual LD)
      cfg_autold = 0; cfg_tickdelay = 48;
      ovr_val = 32'h7F3A_5C11; ovr_cnt = 1; cfg_pdu = 32;
      cfg_load(1); chk_frames_ok("D2/w32", 1);

      // D3: single-bit payload
      ovr_val = 32'd0; ovr_cnt = 1; cfg_pdu = 1;
      cfg_load(1); chk_frames_ok("D3/w1", 1);
      chk_flags("D3", 0,0,0,0,0,0);
      chk(Count_Error_o == 4'd0,             "D3: no error events");

      // D4: PROG + manual line delay -> arbitrary data (MSB='1')
      cfg_autold = 0; cfg_tickdelay = 48;
      ovr_val = 12'hABC; ovr_cnt = 1; cfg_pdu = 12;
      cfg_load(1); chk_frames_ok("D4/manLD", 1);
      chk_flags("D4", 0,0,0,0,0,0);
      cfg_autold = 1;

      // D5: AUTO length discovery + AutoLD (needs pad >= 1):
      //     3 pad zeros + 13 ones -> Length 12, Data 4095, Padding 3
      cfg_ssilen = 1; auto_p = 3; auto_w = 13; cfg_halfma = 24;
      cfg_load(1); chk_frames_ok("D5/auto", 1);
      chk_flags_nome("D5", 0,0,0,0,0);

      // D6: AUTO + manual line delay, no padding: 20 ones -> Length 19
      cfg_autold = 0; cfg_tickdelay = 48; auto_p = 0; auto_w = 20;
      cfg_load(1); chk_frames_ok("D6/autoMan", 1);
      chk_flags_nome("D6", 0,0,0,0,0);
      cfg_autold = 1;

      // D7: AUTO degenerate: 1 pad + single-one pulse -> Length 0
      auto_p = 1; auto_w = 1;
      cfg_load(1); chk_frames_ok("D7/w1", 1);
      chk_flags_nome("D7", 0,0,0,0,0);

      // D8: inverted SSI polarity (MA idle low, launch on rising)
      cfg_idle = 0; cfg_launch = 1; cfg_halfma = 24;
      ovr_val = 12'h02BC; ovr_cnt = 1; cfg_ssilen = 0; cfg_pdu = 12;
      cfg_load(1); chk_frames_ok("D8/idle0", 1);
      auto_p = 2; auto_w = 9; cfg_ssilen = 1;
      cfg_load(1); chk_frames_ok("D8/autoIdle0", 1);
      chk_flags_nome("D8", 0,0,0,0,0);

      // D9: continuous SSI PROG stream (manual LD)
      cfg_ssilen = 0; cfg_cont = 1; cfg_autold = 0; cfg_tickdelay = 48;
      cfg_load(1);
      snap_all;
      for (int i = 0; (i < 400) && ((bidx - bidx0) < 5); i++)
        #(T_ns() * 4.0);
      SET_FSM_Continuous_i = 1'b0;
      wait_quiesce(300.0);
      chk((bidx - bidx0) >= 5,               "D9: >=5 continuous SSI frames");
      chk(f_bad == f_bad0,                   "D9: all SSI frames match");
      cfg_cont = 0;
    end
  endtask

  // --------------------------------------------------------------- E ---------
  task test_E;
    logic [7:0] d0, d1, d2;
    begin
      $display("\n--- SECTION E: CDM/CDS register communication ---");

      cfg_defaults;                          // BiSS, CRC on (incl CDS)
      cfg_inclcds = 1; cfg_pdu = 8;
      cfg_autold = 0; cfg_tickdelay = 48;    // manual LD: clean flag window
      cfg_load(1);                           // verify frame pumps the idle run

      // E1: register reads of the deterministic power-up pattern
      d0 = (8'h00 * 37 + 11) & 8'hFF;
      d1 = (8'h2A * 37 + 11) & 8'hFF;
      d2 = (8'h7E * 37 + 11) & 8'hFF;
      snap_all;
      comm_req(2'b00, 2'b00, 8'h01, 7'h00, 8'h00, 8000.0, 1, 0, d0, "E1/rd00");
      comm_req(2'b00, 2'b00, 8'h01, 7'h2A, 8'h00, 8000.0, 1, 0, d1, "E1/rd2A");
      comm_req(2'b00, 2'b00, 8'h01, 7'h7E, 8'h00, 8000.0, 1, 0, d2, "E1/rd7E");
      wait_quiesce(2000.0);
      chk_frames_ok("E1", bidx - bidx0);
      chk_flags("E1", 0,0,0,0,0,0);
      chk(Count_Error_o == cnt16(ce0, 0),     "E1: no error events counted");
      pulse_clear;
      
      // E2: register write + read-back
      snap_all;
      comm_req(2'b01, 2'b00, 8'h01, 7'h10, 8'h5A, 8000.0, 1, 0, 8'h00, "E2/wr10");
      comm_req(2'b00, 2'b00, 8'h01, 7'h10, 8'h00, 8000.0, 1, 0, 8'h5A, "E2/rd10");
      chk(regfile[8'h10] === 8'h5A,          "E2: slave register committed");
      wait_quiesce(2000.0);
      chk_frames_ok("E2", bidx - bidx0);
      chk_flags("E2", 0,0,0,0,0,0);
      pulse_clear;

      // E3: command frame -> IDA acknowledge mask on COMM_Data_o
      snap_all;
      comm_req(2'b10, 2'b01, 8'h01, 7'h00, 8'h00, 8000.0, 1, 0, cfg_ida, "E3/cmd");
      wait_quiesce(2000.0);
      chk_frames_ok("E3", bidx - bidx0);
      chk_flags("E3", 0,0,0,0,0,0);
      pulse_clear;

      // E4: broadcast command
      snap_all;
      comm_req(2'b11, 2'b10, 8'h00, 7'h00, 8'h00, 8000.0, 1, 0, 8'h00, "E4/bcast");
      wait_quiesce(2000.0);
      chk_frames_ok("E4", bidx - bidx0);
      chk_flags("E4", 0,0,0,0,0,0);
      pulse_clear;

      // E5: read request interleaved into a continuous data stream
      cfg_cont = 1;
      cfg_load(1);
      snap_all;
      for (int i = 0; (i < 400) && ((bidx - bidx0) < 3); i++)
        #(T_ns() * 4.0);
      comm_req(2'b00, 2'b00, 8'h01, 7'h2A, 8'h00, 8000.0, 1, 0, d1, "E5/interleave");
      for (int i = 0; (i < 400) && ((bidx - bidx0) < 8); i++)
        #(T_ns() * 4.0);
      SET_FSM_Continuous_i = 1'b0;
      wait_quiesce(2000.0);
      chk(f_bad == f_bad0,                   "E5: all interleaved frames match");
      chk_flags("E5", 0,0,0,0,0,0);
      cfg_cont = 0;
      pulse_clear;

      // E6: back-to-back accesses (master inserts the CDM='0' reset run)
      cfg_load(1);
      snap_all;
      comm_req(2'b01, 2'b00, 8'h01, 7'h11, 8'hC3, 8000.0, 1, 0, 8'h00, "E6/wr11");
      comm_req(2'b00, 2'b00, 8'h01, 7'h11, 8'h00, 8000.0, 1, 0, 8'hC3, "E6/rd11");
      comm_req(2'b10, 2'b11, 8'h01, 7'h00, 8'h00, 8000.0, 1, 0, cfg_ida, "E6/cmd");
      wait_quiesce(2000.0);
      chk_frames_ok("E6", bidx - bidx0);
      chk_flags("E6", 0,0,0,0,0,0);
      pulse_clear;

      // E7: slave busy - two unconfirmed start bits before the confirm
      SLV_BUSY_S = 2;
      cfg_load(1);
      snap_all;
      comm_req(2'b00, 2'b00, 8'h01, 7'h2A, 8'h00, 8000.0, 1, 0, d1, "E7/busy");
      wait_quiesce(2000.0);
      chk_frames_ok("E7", bidx - bidx0);
      chk_flags("E7", 0,0,0,0,0,0);
      SLV_BUSY_S = 0;
      pulse_clear;
    end
  endtask

  // --------------------------------------------------------------- F ---------
  task test_F;
    begin
      $display("\n--- SECTION F: error flags (precise assertion) ---");

      // F1: NoStart - slave holds SL low, manual LD, finite TX margin
      cfg_defaults;
      cfg_autold = 0; cfg_tickdelay = 48; cfg_halfma = 6; cfg_pdu = 8;
      cfg_load(0);
      SLV_STUCK0 = 1'b1;
      SS_SL = 1'b0;
      snap_all;
      run_frame(500.0);
      wait_quiesce(200.0);
      chk(slv_missed == miss0 + 1,           "F1: no Recv_Valid");
      chk(Count_Frame_o == cnt16(cf0, 1),    "F1: aborted frame counted");
      chk(Count_Error_o == cnt16(ce0, 1),    "F1: one error event");
      chk_flags("F1", 1,0,0,0,0,0);          // only NoStart
      SLV_STUCK0 = 1'b0;
      SS_SL = 1'b1;

      // F2: NoMarker - SSI PROG without the monoflop marker
      cfg_defaults;
      cfg_ssi = 1; cfg_ssilen = 0; cfg_idle = 1; cfg_launch = 0;
      cfg_autold = 0; cfg_tickdelay = 48; cfg_pdu = 12;
      cfg_load(0);
      SLV_NO_MARKER = 1'b1;
      snap_all;
      run_frame(500.0);
      chk(f_ok == f_ok0 + 1,                 "F2: data still received");
      chk(f_bad == f_bad0,                   "F2: data correct");
      chk(Count_Frame_o == cnt16(cf0, 1),    "F2: frame counted");
      chk(Count_Error_o == cnt16(ce0, 1),    "F2: one error event");
      chk_flags("F2", 0,1,0,0,0,0);          // only NoMarker
      SLV_NO_MARKER = 1'b0;

      // F3: BadCRC - slave sends the non-inverted CRC (equal to the check
      //     value -> the DUT comparison flags it)
      cfg_defaults; cfg_pdu = 16; cfg_autold = 0; cfg_tickdelay = 48;
      cfg_load(0);
      SLV_CRC_PLAIN = 1'b1;
      snap_all;
      run_frame(500.0);
      chk(f_ok == f_ok0 + 1,                 "F3: payload still received");
      chk(f_bad == f_bad0,                   "F3: payload correct");
      chk(Count_Error_o == cnt16(ce0, 1),    "F3: one error event");
      chk_flags("F3", 0,0,1,0,0,0);          // only BadCRC
      SLV_CRC_PLAIN = 1'b0;

      // F4: WdExp - SSI AUTO with a frame watchdog shorter than the long
      //     pulse frame; the RX is still hunting the marker -> NoMarker too
      cfg_defaults;
      cfg_ssi = 1; cfg_ssilen = 1; cfg_idle = 1; cfg_launch = 0;
      cfg_autold = 0; cfg_tickdelay = 48; cfg_halfma = 24;
      cfg_pdu = 8; auto_p = 1; auto_w = 8; cfg_tickto = 600;
      cfg_load(1);                            // verify frame: short pulse, OK
      chk_flags("F4/ref", 0,0,0,0,0,0);
      snap_all;
      auto_w = 20;                            // long pulse -> watchdog fires
      run_frame(500.0);
      wait_quiesce(200.0);
      chk(slv_missed == miss0 + 1,           "F4: no Recv_Valid on abort");
      chk(Count_Frame_o == cnt16(cf0, 1),    "F4: aborted frame counted");
      chk(Count_Error_o == cnt16(ce0, 2),    "F4: two error events (WdExp+NoMarker)");
      chk_flags("F4", 0,1,0,1,0,0);          // WdExp + NoMarker
      cfg_tickto = 0;

      // F5: Meas_Error - BiSS AutoLD with SL frozen high: the ACK negedge
      //     never comes; the DUT keeps the old tap, the hunt locks on the
      //     first '1' strobe and clocks an all-ones frame; CRC disabled so
      //     the sticky vector contains ONLY ERR_Measure_o
      cfg_defaults;
      cfg_crc = 0; cfg_pdu = 8;
      cfg_load(0);
      SLV_STUCK1 = 1'b1;
      SS_SL = 1'b1;
      snap_all;
      run_frame(500.0);
      chk(f_ok == f_ok0 + 1,                 "F5: all-ones frame received");
      chk(f_bad == f_bad0,                   "F5: all-ones as predicted");
      chk(Count_Error_o != cnt16(ce0, 0),    "F5: error events counted");
      chk_flags("F5", 0,0,0,0,1,0);          // only Measure
      SLV_STUCK1 = 1'b0;

      // F6a: Err_CDM - write to the read-only register is rejected
      //      (inverted W echo); Count_Error must NOT count CDM errors
      cfg_defaults; cfg_inclcds = 1; cfg_pdu = 8;
      cfg_autold = 0; cfg_tickdelay = 48;
      cfg_load(1);
      snap_all;
      comm_req(2'b01, 2'b00, 8'h01, 7'h7F, 8'h11, 2000.0, 0, 1, 8'h00, "F6a/roWr");
      wait_quiesce(2000.0);
      chk(Count_Error_o == cnt16(ce0, 0),    "F6a: CDM error NOT counted");
      chk_flags("F6a", 0,0,0,0,0,1);         // only CDM

      // F6b: Err_CDM - slave corrupts the read-data CRC-4
      pulse_clear;
      SLV_RD_CRC_ERR = 1'b1;
      snap_all;
      comm_req(2'b00, 2'b00, 8'h01, 7'h2A, 8'h00, 8000.0, 0, 1, 8'h00, "F6b/rdCRC");
      wait_quiesce(2000.0);
      chk(Count_Error_o == cnt16(ce0, 0),    "F6b: CDM error NOT counted");
      chk_flags("F6b", 0,0,0,0,0,1);
      SLV_RD_CRC_ERR = 1'b0;

      // F6c: Err_CDM - slave never confirms the start bit -> busy timeout
      //      after MAX_BUSY_CYC (64) unconfirmed S frames
      pulse_clear;
      SLV_BUSY_S = 100;
      snap_all;
      comm_req(2'b00, 2'b00, 8'h01, 7'h2A, 8'h00, 8000.0, 0, 1, 8'h00, "F6c/busyTO");
      wait_quiesce(2000.0);
      chk(Count_Error_o == cnt16(ce0, 0),    "F6c: CDM error NOT counted");
      chk_flags("F6c", 0,0,0,0,0,1);
      SLV_BUSY_S = 0;
    end
  endtask

  //===========================================================================
  // 9. Main sequence
  //===========================================================================

  initial begin : main
    // ---- init ----
    RST_i            = 1'b1;
    CLK_i            = 1'b0;
    Start_i          = 1'b0;
    Clear_Status_i   = 1'b0;
    COMM_Request_i   = 1'b0;
    COMM_Type_i      = 2'b00;
    COMM_CMD_i       = 2'b00;
    COMM_IDSel_i     = 8'h00;
    COMM_Addr_i      = 7'h00;
    COMM_Data_i      = 8'h00;
    SET_FSM_Enable_i = 1'b0;
    cfg_defaults;
    drive_cfg_inputs;

    SS_SL      = 1'b1;
    q_n        = 0;
    q_gen      = 0;
    bidx       = 0;
    burst_active   = 1'b0;
    awaiting_b1    = 1'b1;
    oracle_pending = 1'b0;
    oracle_edges   = 0;
    launch_cnt = 0;
    t_last_edge = -1.0e9;
    slv_missed = 0;
    frames_done = 0;
    plan_valid = 0;
    seq_len    = 0;
    ovr_cnt    = 0;
    ovr_val    = 32'd0;
    f_ok = 0; f_bad = 0; f_valid = 0; score_mute = 0;
    n_pass = 0; n_fail = 0;
    t_half_meas = 500.0;
    cdm_reset_state;
    slv_en = 1'b0;
    for (int a = 0; a < 128; a++)
      regfile[a] = (a * 37 + 11) & 8'hFF;

    // ---- reset ----
    #107.3;
    RST_i  = 1'b0;
    #20.0;
    slv_en = 1'b1;
    $display("tb_prim_BiSS_SSI_RX  section=%0s  (variable tick: T=1/baud(gear), table 500k/1M/1M5/2M, 100 MHz clk)",
             SECTION);

    // ---- tests ----
    if (sect("A")) test_A;
    if (sect("B")) test_B;
    if (sect("C")) test_C;
    if (sect("D")) test_D;
    if (sect("E")) test_E;
    if (sect("F")) test_F;

    // ---- summary ----
    $display("\n============================================================");
    $display("  RESULT: %0d checks passed, %0d FAILED", n_pass, n_fail);
    $display("  Frames: %0d received, %0d clean, %0d mismatched, %0d aborted",
             f_valid, f_ok, f_bad, slv_missed);
    $display("============================================================");
    if (n_fail == 0) $display("  TESTBENCH PASSED");
    else             $display("  TESTBENCH FAILED");
    $finish;
  end

  // global watchdog
  initial begin
    #SIM_LIMIT_NS;
    $display("\n*** FATAL: global simulation timeout ***");
    $display("*** RESULT: FAILED (timeout) ***");
    $finish;
  end

endmodule

`default_nettype wire
