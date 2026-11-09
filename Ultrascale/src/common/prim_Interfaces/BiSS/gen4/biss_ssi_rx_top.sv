`timescale 1ns/1ps
// =============================================================================
// biss_ssi_rx_top.sv -- combined BiSS-C / SSI master-side receiver
// -----------------------------------------------------------------------------
// Single-channel master receiver for BiSS-C (sensor/SCD mode) and classic SSI
// encoders. The receiver generates the MA (SSI: CL) clock from the system
// clock, receives the SL answer line, compensates the round-trip line delay,
// deserializes the process data, checks the BiSS-C CRC and reports parallel
// data plus status. Instances share no state, so several channels can live in
// one clock domain.
//
// =========================== 1. ARCHITECTURE ========================
//
//                  +----------------------------------------------------+
//                  |                  biss_ssi_rx_top                   |
//                  |                                                    |
//   cfg ---------->| [clamp + snapshot @ frame_load] --> cfg_s          |
//                  |                                                    |
//                  |  prim_FractionalTickGen  --->  tick_ce             |
//                  |    baud = MA frequency, OVERSAMPLE ticks per       |
//                  |    MA period (4/8/16/32, top parameter)            |
//                  |        |                                           |
//                  |  [mod-(OVERSAMPLE/2) divider] --> hp_ce            |
//                  |        |                                           |
//   MA pin <-------|  ma_frame_fsm          prim_BiSS_SSI_Pipeline      |
//                  |   (MA owner, frame     (delays the MA edge/level   |
//                  |    events)             stream by the line-delay    |
//                  |        |                tap 'rx_dly_active')       |
//                  |        |                       |                   |
//                  |  prim_BiSS_SSI_LineDelay   sample strobes          |
//                  |   (per-frame round-trip      |                     |
//                  |    delay measurement)        v                     |
//                  |        |                 [rx_deser_fsm  +         |
//                  |        v                  prim_CRC_Calculator]     |
//                  |     tap manager                 |                  |
//                  |                                 v                  |
//                  |  cdm_ctrl_fsm ------------- data / status outputs  |
//                  +----------------------------------------------------+
//
//   * ma_frame_fsm            : MA generator, frame accept/abort/gap, the
//                               tick-aligned frame_launch / ld_ref events.
//   * prim_BiSS_SSI_Pipeline  : variable-tap delay line for the MA edge
//                               strobes; its delayed outputs form, after the
//                               polarity filter, the SL sampling strobes.
//   * prim_BiSS_SSI_LineDelay : measures the round-trip line delay in
//                               oversampling ticks, once per frame.
//   * rx_deser_fsm            : start-bit search, bit counting, SSI
//                               monoflop marker handling, CRC comparison
//                               and result extraction; instantiates
//                               prim_CRC_Calculator (BiSS-C CRC-6, fed
//                               bit-serially).
//   * cdm_ctrl_fsm            : BiSS-C register-access sequencing.
//
// ========================= 2. TIME BASE =============================
// Everything runs in one system-clock domain. The time base is the internal
// prim_FractionalTickGen:
//   * cfg.ma_speed (5 bit, snapshotted at frame start) selects the MA
//     frequency ("baud") from MA_BAUD_VAL (93.75 kHz .. 3.0 MHz in uniform
//     steps); the fractional Q16 division keeps the actual rate within
//     ~0.03 % of nominal, and the tick spacing jitter that this introduces
//     is invisible because every consumer counts strobes, not clock cycles;
//   * the OVERSAMPLE parameter (4/8/16/32) sets the number of oversampling
//     ticks per MA period; the tick rate is therefore OVERSAMPLE * f_MA and
//     one MA half-period is exactly OVERSAMPLE/2 ticks at every speed - the
//     mod-(OVERSAMPLE/2) divider below turns tick_ce into hp_ce, the MA
//     half-period boundary on which ma_frame_fsm toggles MA;
//   * ALL delay magnitudes (delay measurement, delay tap, manual
//     rx_dly_ticks) are in OVERSAMPLING TICKS, a speed-relative unit; no
//     sys<->tick conversion exists anywhere. A measurement taken at speed A
//     and applied at speed B therefore denotes a different physical delay -
//     auto mode re-measures every frame and converges after one frame;
//   * absolute-time magnitudes (gap_ticks, margin_cyc, wdt_ticks,
//     cdm_ext_ticks) stay in system-clock ticks; they bound real,
//     speed-independent quantities such as the sensor's ~20 us timeout.
// Feasibility: the fastest gear requires OVERSAMPLE * f_MA < FREQ_SYSCLK
// (at least one system clock per tick); checked at elaboration below.
//
// ================= 3. LINE DELAY COMPENSATION (BiSS) ================
// Protocol facts (BiSS Association / BiSS-C literature):
//   * the slave launches SL bits on one MA edge polarity (rising for
//     BiSS-C, falling for SSI); one bit per MA cycle;
//   * the slave drops SL (the ACK marker) at the 2nd rising MA edge of a
//     frame, delayed by the ROUND-TRIP line delay D;
//   * the master measures D in every frame, from the reference edge to the
//     falling edge of SL (prim_BiSS_SSI_LineDelay, counting tick strobes).
// Bit b_k, launched at edge E_k, is valid on [E_k + D, E_k + T + D) with T
// the MA period; its center lies at E_k + T/2 + D. The opposite-polarity
// strobe of E_k occurs exactly T/2 after E_k, so delaying ALL edge strobes
// by D ticks places a sampling strobe at the bit center - for any D within
// the tap range, with sub-cycle resolution.
//
// Tap update policy (tap manager below):
//   * frame start: auto mode loads the previous frame's measurement (the
//     first frame after reset may CRC-error and converges from frame 2, or
//     starts from the preset cfg.rx_dly_ticks); manual mode loads
//     cfg.rx_dly_ticks;
//   * during a frame, a fresh measurement is applied immediately while it
//     is at most 1.5 MA periods (= 3*OVERSAMPLE/2 ticks: the start-bit
//     strobe is still behind the tap update point), otherwise deferred to
//     the next frame;
//   * a failed measurement (timeout / out of range) keeps the old tap;
//     Meas_Done_o implies the value is inside the representable range.
//
// ========= 4. SAMPLING STROBES AND THE PIPELINE LEVEL CONVENTION =====
// prim_BiSS_SSI_Pipeline outputs, with every delayed edge strobe, a level
// that identifies the edge polarity. Its two implementation paths report
// that level with a different phase convention:
//   * tap q >= 1 (history ring): the level AFTER the edge that lies q
//     half-periods back - sampling-polarity edges satisfy
//     MA_Level_o == ~sl_launch_lvl;
//   * tap q == 0 (bypass paths): the level BEFORE the delayed edge -
//     sampling-polarity edges satisfy MA_Level_o == sl_launch_lvl.
// The sampling multiplexer below selects the compare term from the current
// tap, so exactly one strobe per SL bit is selected at its center for every
// tap value. While MA idles, the pipeline keeps emitting strobes at the
// idle level; whether they pass the filter is harmless - the deserializer
// only looks at sample strobes between frame_launch and frame_active.
//
// ============ 5. LONG PROCESSING DELAY INDIFFERENCE ==================
// The sensor may delay its START bit arbitrarily after the ACK. The
// receiver has no fixed-latency assumption:
//   * the RX search state re-synchronizes on the START bit every frame;
//   * frame length is bounded only by cfg.margin_cyc (0 = practically
//     infinite) and cfg.wdt_ticks (0 = infinite);
//   * cfg.gap_ticks (inter-frame "timeout state") is programmable up to
//     2^16 sys ticks - arbitrarily slow sensors are simply polled slower;
//   * corrupted frames are flagged (CRC / no-start / no-marker) and never
//     deadlock the FSMs; the next frame re-synchronizes.
//
// ================= 6. FRAME FORMATS (field layout) ==================
// BiSS (point-to-point, sensor mode), received MSB first:
//   START('1') | CDS(cds_bits) | PDU(pdu_bits) | CRC-6 (x^6+x+1, 0x03)
//   - PDU = sensor process data (GP / alarm / error + position), hence the
//     runtime-variable length;
//   - CDS width and CRC coverage (cfg.crc_inc_cds) per sensor profile;
//   - preamble: MA falls at frame start, the 1st rising MA edge activates
//     the slave, the 2nd rising edge is the ACK (SL low). The ACK-anchored
//     START search skips the preamble implicitly; manual delay mode skips
//     the fixed preamble count instead.
// SSI:
//   PROG : PDU(pdu_bits) | monoflop marker ('1' for one CL period);
//   AUTO : pad zeros | PDU | monoflop marker; length discovered from the
//          marker (valid only with zero-padded MSBs - heuristic; prefer
//          PROG when the length is known).
//
// =================== 7. INTEGRATION NOTES ===========================
//  * cfg.gap_ticks [sys ticks] must exceed BOTH the slave timeout (BiSS-C
//    typ. >= 20 us or adaptive) AND the round-trip line delay in absolute
//    time - mind this when running slow speed gears.
//  * cfg fields are snapshotted at frame acceptance (frame_load), so
//    mid-frame changes apply at the next frame; cfg.ma_speed included.
//  * The CRC parameters below implement the BiSS-C process-data CRC
//    (x^6 + x + 1, preset 0, MSB first, no final XOR). Confirm the preset
//    and bit order against your sensor's BiSS profile / BiSS AN3.
//  * The delay-line level history has no reset and warms up over
//    2^RAM_LOG2 half-period strobes; auto mode starts at tap 0 and only
//    applies a non-zero tap after a successful measurement, so the warm-up
//    is invisible except for manual mode's very first frame with
//    rx_dly_ticks > 0.
//  * dbg_state = {mf[2:0], rx[2:0], cc[2:0]} FSM state snapshots.
// =============================================================================
// -----------------------------------------------------------------------------
// Shared geometry limits and the configuration / status words. These
// declarations live at compilation-unit scope (this project uses no shared
// package file) and are consumed only by this module.
// -----------------------------------------------------------------------------
localparam int unsigned MAX_PDU_BITS   = 64; // widest process-data payload
localparam int unsigned MAX_CDS_BITS   = 4;  // widest CDS field
localparam int unsigned CRC_BITS       = 6;  // BiSS-C CRC width
localparam int unsigned MAX_FRAME_BITS = 1 + MAX_CDS_BITS + MAX_PDU_BITS + CRC_BITS;

typedef struct packed {
  // --- control ---
  logic        enable;         // 0 -> FSMs idle, MA pinned to idle level
  logic        proto;          // 0 = SSI, 1 = BiSS-C
  logic        continuous;     // repeat process frames forever
  logic        crc_en;         // BiSS: append/check CRC-6
  logic        crc_inc_cds;    // 1: CRC covers CDS bits as well as PDU
  logic        auto_ld_meas;   // per-frame automatic line-delay tracking
  logic        ssi_len_mode;   // 0 = length from pdu_bits, 1 = discover from
                               //    the monoflop marker (zero-padded MSBs!)
  // --- electrical / timing polarity ---
  logic        sl_launch_lvl;  // MA level AFTER the edge the slave launches on
  logic        ma_idle_lvl;    // MA level while idle
  // --- geometry (runtime-variable packet length) ---
  logic [2:0]  cds_bits;       // CDS field width (clamped to MAX_CDS_BITS)
  logic [6:0]  pdu_bits;       // PDU width (clamped to MAX_PDU_BITS)
  // --- timing ---
  logic [4:0]  ma_speed;       // MA speed gear -> MA_BAUD_VAL table below;
                               // snapshotted, so speed changes land at frame
                               // boundaries
  logic [15:0] gap_ticks;      // inter-frame idle [sys ticks]; >= sensor
                               // timeout AND >= round-trip delay in absolute
                               // time
  logic [15:0] margin_cyc;     // extra MA cycles before TX abort (0 = infinite)
  logic [23:0] wdt_ticks;      // frame watchdog [sys ticks] (0 = infinite)
  logic [15:0] rx_dly_ticks;   // manual sampling delay [oversampling ticks];
                               // clamped to the pipe range at the snapshot
  logic [11:0] cdm_ext_ticks;  // CDM '1' frame-start extension [sys ticks]
} cfg_t;

typedef struct packed {
  // sticky error flags (cleared with err_clr)
  logic        e_crc;        // CRC mismatch
  logic        e_no_start;   // start bit / data not found within TX margin
  logic        e_no_marker;  // SSI monoflop end marker missing/invalid
  logic        e_wdt;        // frame watchdog expired
  logic        e_meas;       // line-delay measurement failed
  logic        e_cdm;        // CDM register access aborted
  // measurements
  logic [23:0] ld_ticks;     // last measured round-trip line delay [ovs ticks]
  logic [15:0] ld_ma_cyc;    // same delay in MA half-cycles (informational)
  logic [15:0] tap_active;   // currently applied RX sampling delay [ovs ticks]
  // live state
  logic        busy;
  logic [15:0] frame_cnt;    // frames completed
  logic [15:0] err_cnt;      // frames with any error event
} status_t;

module biss_ssi_rx_top #(
  // System clock frequency [Hz]. Consumed at elaboration by the internal
  // prim_FractionalTickGen (Q16 increment table); keep it in sync with the
  // real clk period. The fastest speed gear requires
  // OVERSAMPLE * MA_BAUD_VAL[MA_SPEED_NUM-1] < FREQ_SYSCLK.
  parameter real FREQ_SYSCLK = 100.0e6,
  // Oversampling ratio: oversampling ticks per MA period. Supported
  // values: 4, 8, 16, 32 (power of two, at least 4).
  parameter int  OVERSAMPLE  = 16,
  // Delay history depth: 2^RAM_LOG2 MA half-period strobes. The maximum
  // representable delay is (2^RAM_LOG2 - 1) half-periods + OVERSAMPLE/2 - 1
  // ticks; every TAP_W-bit value is representable by construction.
  parameter int  RAM_LOG2    = 4
)(
  input  logic clk,     // system clock (single clock domain)
  input  logic rst_n,   // async, active low

  // ---- line pins ----
  output logic ma,      // master clock out (registered, glitch-free)
  input  logic sl,      // slave data in (async; 2FF-synced internally)

  // ---- runtime configuration (snapshot-latched at frame start) ----
  input  cfg_t cfg,

  // ---- control ----
  input  logic single_start,  // pulse: request one process-data frame
  input  logic reg_req,       // pulse: request a CDM register access
  input  logic reg_rw,        // 1 = write
  input  logic [7:0] reg_addr,
  input  logic [7:0] reg_wdata,
  input  logic err_clr,       // clears sticky error flags / counters

  // ---- data / events ----
  output logic data_valid,                        // pulse: new frame result
  output logic [MAX_PDU_BITS-1:0]           pos_out,
  output logic [$clog2(MAX_PDU_BITS+1)-1:0] pos_len_out, // effective PDU length
  output logic [$clog2(MAX_PDU_BITS+1)-1:0] pad_out,     // SSI-AUTO leading zeros
  output logic [MAX_CDS_BITS-1:0]           cds_out,   // received CDS bits
  output logic crc_ok,                            // frame CRC check (with data_valid)
  output logic reg_valid,                         // pulse: reg_rdata valid (read)
  output logic [7:0] reg_rdata

  // ---- status / debug ----
  //output status_t status,
  //output logic [31:0] dbg_state   // {mf[2:0], rx[2:0], cc[2:0]}
);

  // ===================================================================
  // Derived geometry
  // ===================================================================
  localparam int unsigned HP_TICKS = OVERSAMPLE/2;    // ticks per MA half-period
  localparam int unsigned HP_W     = $clog2(HP_TICKS);
  // Delay tap width [oversampling ticks]; identical to the TAP_W that
  // prim_BiSS_SSI_LineDelay and prim_BiSS_SSI_Pipeline derive from
  // RAM_LOG2 + $clog2(OVERSAMPLE/2).
  localparam int unsigned TAP_W    = RAM_LOG2 + HP_W;
  // Immediate tap application bound: D <= 1.5 MA periods = 3*OVERSAMPLE/2
  // ticks (the start-bit strobe is still behind the tap update point).
  localparam int unsigned IMM_TAPS = 3 * HP_TICKS;

  localparam int unsigned BITW = $clog2(MAX_FRAME_BITS+1);
  localparam int unsigned PADW = $clog2(MAX_PDU_BITS+1);
  localparam logic [2:0]      CDS_MAX = 3'(MAX_CDS_BITS);
  localparam logic [PADW-1:0] PDU_MAX = PADW'(MAX_PDU_BITS);
  localparam logic [15:0]     TAP_MAX = 16'((1 << TAP_W) - 1);
  localparam logic [BITW-1:0] ONE     = {{(BITW-1){1'b0}}, 1'b1};
  localparam logic [BITW-1:0] CRC_WV  = BITW'(CRC_BITS);

  localparam logic        PROTO_SSI  = 1'b0;
  localparam logic        PROTO_BISS = 1'b1;

  // MA speed gears ("baud" = MA frequency [Hz]). cfg.ma_speed selects one
  // entry; the tick rate is OVERSAMPLE * f_MA. The fastest gear must keep
  // that product below FREQ_SYSCLK (checked below).
  localparam int unsigned MA_SPEED_NUM = 32;   // 2^5 -> 5-bit selector
  localparam int MA_BAUD_VAL [0:MA_SPEED_NUM-1] = '{
     93_750,  187_500,  281_250,  375_000,  468_750,  562_500,  656_250,  750_000,
    843_750,  937_500, 1_031_250, 1_125_000, 1_218_750, 1_312_500, 1_406_250, 1_500_000,
  1_593_750, 1_687_500, 1_781_250, 1_875_000, 1_968_750, 2_062_500, 2_156_250, 2_250_000,
  2_343_750, 2_437_500, 2_531_250, 2_625_000, 2_718_750, 2_812_500, 2_906_250, 3_000_000
  };

  // -------------------------------------------------------------------
  // Elaboration-time checks
  // -------------------------------------------------------------------
  initial begin
    if ((OVERSAMPLE != 4) && (OVERSAMPLE != 8) && (OVERSAMPLE != 16) && (OVERSAMPLE != 32))
      $fatal(1, "biss_ssi_rx_top: OVERSAMPLE must be 4, 8, 16 or 32 ticks per MA period");
    if ((RAM_LOG2 <= 1) || (RAM_LOG2 >= 16))
      $fatal(1, "biss_ssi_rx_top: RAM_LOG2 out of range (2..15)");
    if ((real'(MA_BAUD_VAL[MA_SPEED_NUM-1]) * real'(OVERSAMPLE)) >= FREQ_SYSCLK)
      $fatal(1, "biss_ssi_rx_top: fastest MA speed gear exceeds FREQ_SYSCLK (tick < 1 sys clock)");
  end

  // ===================================================================
  // Interconnect declarations
  // ===================================================================

  // internal tick generator + half-period divider
  logic tick_ce;         // strobe per oversampling tick
  logic [HP_W-1:0] tick_ph; // free-running mod-(OVERSAMPLE/2) tick phase
  logic hp_ce;           // every (OVERSAMPLE/2)-th tick: MA half-period boundary

  // SL synchronizer
  logic sl_sync;

  // configuration
  cfg_t cfg_c;   // clamped, live
  cfg_t cfg_s;   // per-frame snapshot

  // frame geometry
  logic [BITW-1:0] frame_bits;

  // ma_frame_fsm interconnect
  logic frame_load, frame_launch, ld_ref, frame_abort, frame_end, ev_wdt;
  logic frame_active, frame_is_cdm;
  logic op_req, single_pending;
  logic cdm_req, cdm_bit, cdm_busy;
  logic [2:0] mf_dbg;

  // delay pipeline / sampling
  logic ma_edge_dly, ma_lvl_dly;
  logic [TAP_W-1:0] rx_dly_active, rx_dly_shadow;
  logic sample_ce;

  // line-delay measurement
  logic        meas_done, meas_err;
  logic [TAP_W-1:0] tick_delay;

  // RX deserializer / CRC
  logic rx_frame_done, res_valid, ev_crc_err, ev_no_start, ev_no_marker;
  logic [MAX_PDU_BITS-1:0] res_pos;
  logic [PADW-1:0]         res_len, res_pad;
  logic [MAX_CDS_BITS-1:0] res_cds;
  logic                    res_crc_ok;
  logic [2:0]              rx_dbg;

  // cdm_ctrl_fsm
  logic       reg_valid_i;
  logic       ev_cdm_err;
  logic [2:0] cc_dbg;

  // status
  logic e_crc, e_no_start, e_no_marker, e_wdt, e_meas, e_cdm;
  logic [15:0] frame_cnt, err_cnt;
  logic err_evt;

  // ===================================================================
  // Oversampling tick generator - the system's only time base.
  //   * MODE_FIXED = 1'b1 selects VARIABLE mode: BaudSel_i indexes the
  //     BAUD_VAL table (the parameter's polarity follows the module's own
  //     convention: '1' = variable tick rate);
  //   * "baud" = MA frequency; the tick rate is OVERSAMPLE * f_MA, i.e.
  //     exactly OVERSAMPLE ticks per MA period;
  //   * RST_i is synchronous ACTIVE-HIGH inside the prim module;
  //   * Resync_i tied low: the tick grid free-runs and is the system time
  //     base - there is no external phase to align it to, and all
  //     consumers count strobes, so the fractional-division jitter is
  //     invisible;
  //   * BaudSel comes from the per-frame SNAPSHOT cfg_s.ma_speed: speed
  //     changes land at frame boundaries; the accumulator phase is
  //     preserved across a change, only the spacing retimes.
  // ===================================================================
  prim_FractionalTickGen #(
    .FREQ_SYSCLK (FREQ_SYSCLK),
    .MODE_FIXED  (1'b1),
    .OVERSAMPLE  (OVERSAMPLE),
    .BV_NUM      (int'(MA_SPEED_NUM)),
    .BAUD_VAL    (MA_BAUD_VAL)
  ) u_tick_gen (
    .RST_i      (~rst_n),
    .CLK_i      (clk),
    .BaudSel_i  (cfg_s.ma_speed),
    .Resync_i   (1'b0),
    .TickPulse_o(tick_ce)
  );

  // ===================================================================
  // Mod-(OVERSAMPLE/2) tick divider: hp_ce = every OVERSAMPLE/2-th tick,
  // the MA half-period boundary (free-running: boundaries while MA is
  // idle carry the idle level and are rejected downstream by polarity).
  // ===================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)       tick_ph <= '0;
    else if (tick_ce) tick_ph <= tick_ph + 1'b1;
  end
  assign hp_ce = tick_ce && (tick_ph == {HP_W{1'b1}});

  // ===================================================================
  // SL re-synchronizer (2 FF). The latency is part of the measured
  // round-trip delay and therefore self-compensating: the measurement and
  // the data sampling both run on this re-synchronized signal.
  // ===================================================================
  (* ASYNC_REG = "TRUE" *) logic [1:0] sl_ff;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) sl_ff <= 2'b00;
    else        sl_ff <= {sl_ff[0], sl};
  end
  assign sl_sync = sl_ff[1];

  // ===================================================================
  // Configuration: clamp (live) + snapshot at frame acceptance.
  // frame_load is a combinational acceptance strobe -> the snapshot is
  // valid one cycle later, before the first MA edge can occur.
  // ===================================================================
  always_comb begin
    cfg_c = cfg;
    if (cfg.cds_bits > CDS_MAX)   cfg_c.cds_bits   = CDS_MAX;
    if (cfg.pdu_bits > PDU_MAX)   cfg_c.pdu_bits   = PDU_MAX;
    if (cfg.rx_dly_ticks > TAP_MAX) cfg_c.rx_dly_ticks = TAP_MAX;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)          cfg_s <= '0;
    else if (frame_load) cfg_s <= cfg_c;
  end

  // frame geometry from the snapshot:
  //   BiSS: START + CDS + PDU + CRC   SSI: PDU (+ marker, RX-side only)
  assign frame_bits = (cfg_s.proto == PROTO_BISS)
      ? (ONE + {{(BITW-3){1'b0}}, cfg_s.cds_bits} + cfg_s.pdu_bits
         + (cfg_s.crc_en ? CRC_WV : '0))
      : cfg_s.pdu_bits;

  // ===================================================================
  // TX frame controller / MA generator
  // ===================================================================
  ma_frame_fsm #(
    .MAX_FRAME_BITS (MAX_FRAME_BITS)
  ) u_ma_frame (
    .clk          (clk),
    .rst_n        (rst_n),
    .proto_biss   (cfg_s.proto == PROTO_BISS),
    .frame_bits   (frame_bits),
    .gap_ticks    (cfg_s.gap_ticks),
    .margin_cyc   (cfg_s.margin_cyc),
    .wdt_ticks    (cfg_s.wdt_ticks),
    .en           (cfg.enable),
    .ma_idle_lvl  (cfg.ma_idle_lvl),
    .cdm_ext_ticks(cfg.cdm_ext_ticks),
    .op_req       (op_req),
    .cdm_req      (cdm_req),
    .cdm_bit      (cdm_bit),
    .hp_ce        (hp_ce),
    .rx_frame_done(rx_frame_done),
    .frame_load   (frame_load),
    .frame_launch (frame_launch),
    .ld_ref       (ld_ref),
    .frame_abort  (frame_abort),
    .frame_end    (frame_end),
    .ev_wdt       (ev_wdt),
    .frame_active (frame_active),
    .frame_is_cdm (frame_is_cdm),
    .ma           (ma),
    .dbg_state    (mf_dbg)
  );

  // one-shot bookkeeping
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)                           single_pending <= 1'b0;
    else if (single_start)                single_pending <= 1'b1;
    else if (frame_load && !frame_is_cdm) single_pending <= 1'b0;
  end

  // process-frame request (CDM access has priority while it runs)
  assign op_req = cfg.enable && (cfg.continuous || single_pending)
                  && !cdm_busy;

  // ===================================================================
  // Delay-compensated edge stream. The pipeline delays every MA edge
  // strobe (hp_ce, level after the edge = ~ma) by the tap 'rx_dly_active'
  // in oversampling ticks; the delayed strobes are the SL sampling grid.
  // ===================================================================
  prim_BiSS_SSI_Pipeline #(
    .RAM_LOG2   (RAM_LOG2),
    .OVERSAMPLE (OVERSAMPLE)
  ) u_pipe (
    .RST_i      (~rst_n),
    .CLK_i      (clk),
    .TickPulse_i(tick_ce),
    .TickDelay_i(rx_dly_active),
    .MA_Edge_i  (hp_ce),
    .MA_Level_i (~ma),
    .MA_Edge_o  (ma_edge_dly),
    .MA_Level_o (ma_lvl_dly)
  );

  // Polarity filter: keep the strobes of the edge OPPOSITE to the SL launch
  // edge -> exactly one strobe per SL bit, at its center. The compare term
  // depends on the tap's half-period part q (header section 4): the
  // pipeline reports the post-edge level for q >= 1 and the pre-edge level
  // for q == 0.
  logic tap_q0, sample_lvl;
  assign tap_q0     = (rx_dly_active[TAP_W-1:HP_W] == '0);
  assign sample_lvl = tap_q0 ? cfg_s.sl_launch_lvl : ~cfg_s.sl_launch_lvl;
  assign sample_ce  = ma_edge_dly && (ma_lvl_dly == sample_lvl);

  // ===================================================================
  // Per-frame round-trip line-delay measurement. The reference edge is
  // tick-aligned (MA edges are emitted on half-period boundaries), the
  // module counts tick strobes and reports the delay in the tap's unit.
  // ===================================================================
  prim_BiSS_SSI_LineDelay #(
    .TIMEOUT_MA (4),
    .RAM_LOG2   (RAM_LOG2),
    .OVERSAMPLE (OVERSAMPLE)
  ) u_line_delay (
    .RST_i          (~rst_n),
    .CLK_i          (clk),
    .TickPulse_i    (tick_ce),
    .InterfaceMode_i(cfg_s.proto == PROTO_SSI), // 1 = SSI, 0 = BiSS
    .FrameStart_i   (frame_launch && cfg_s.auto_ld_meas),
    .FrameActive_i  (frame_active),
    .MA_RefEdge_i   (ld_ref),
    .SL_Synched_i   (sl_sync),
    .Meas_Error_o   (meas_err),
    .Meas_Done_o    (meas_done),
    .TickDelay_o    (tick_delay)
  );

  // ---- tap manager ----
  // Immediate application is safe while D <= 1.5 MA periods = 3*HP ticks
  // (the start-bit strobe is emitted at R2 + 3T/2, still behind the tap
  // update point R2 + D). Larger delays are applied at the next frame
  // start (tracking). Meas_Done_o implies the value is inside the pipe's
  // representable range - no extra range check here.
  logic imm_ok, ev_meas;
  assign imm_ok  = (tick_delay <= TAP_W'(IMM_TAPS));
  assign ev_meas = meas_err;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_dly_active <= '0;
      rx_dly_shadow <= '0;
    end else begin
      if (frame_load) begin
        rx_dly_active <= cfg_c.auto_ld_meas ? rx_dly_shadow
                                            : TAP_W'(cfg_c.rx_dly_ticks);
      end
      if (meas_done) begin
        rx_dly_shadow <= tick_delay;
        if (cfg_s.auto_ld_meas && imm_ok) begin
          rx_dly_active <= tick_delay;
        end
      end
    end
  end

  // ===================================================================
  // RX deserializer: start-bit search, bit counting, SSI monoflop marker
  // handling, BiSS-C CRC check (prim_CRC_Calculator inside) and result
  // extraction. Runs on the DELAY-COMPENSATED sampling strobes
  // ('sample_ce': delayed MA-edge strobes filtered to the sampling
  // polarity, i.e. one strobe per SL bit, at bit center). Long processing
  // delay indifference: the START search re-synchronizes on the START bit
  // in every frame, the sensor may take arbitrarily long after the ACK.
  // ===================================================================
  rx_deser_fsm #(
    .MAX_PDU_BITS (MAX_PDU_BITS),
    .MAX_CDS_BITS (MAX_CDS_BITS),
    .CRC_BITS     (CRC_BITS)
  ) u_rx_deser (
    .clk          (clk),
    .rst_n        (rst_n),
    .proto_biss   (cfg_s.proto == PROTO_BISS),
    .auto_ld_meas (cfg_s.auto_ld_meas),
    .crc_en       (cfg_s.crc_en),
    .crc_inc_cds  (cfg_s.crc_inc_cds),
    .ssi_len_mode (cfg_s.ssi_len_mode),
    .cds_bits     (cfg_s.cds_bits),
    .pdu_bits     (cfg_s.pdu_bits),
    .frame_bits   (frame_bits),
    .frame_launch (frame_launch),
    .frame_active (frame_active),
    .meas_done    (meas_done),
    .meas_err     (meas_err),
    .sample_ce    (sample_ce),
    .sl_bit       (sl_sync),
    .frame_done   (rx_frame_done),
    .res_valid    (res_valid),
    .ev_crc_err   (ev_crc_err),
    .ev_no_start  (ev_no_start),
    .ev_no_marker (ev_no_marker),
    .res_pos      (res_pos),
    .res_len      (res_len),
    .res_pad      (res_pad),
    .res_cds      (res_cds),
    .res_crc_ok   (res_crc_ok),
    .dbg_state    (rx_dbg)
  );

  // ===================================================================
  // CDM register-access sequencer
  // ===================================================================
  cdm_ctrl_fsm #(
    .CDS_W (MAX_CDS_BITS)
  ) u_cdm (
    .clk         (clk),
    .rst_n       (rst_n),
    .en          (cfg.enable),
    .reg_req     (reg_req),
    .reg_rw      (reg_rw),
    .reg_addr    (reg_addr),
    .reg_wdata   (reg_wdata),
    .frame_load  (frame_load),
    .frame_is_cdm(frame_is_cdm),
    .frame_end   (frame_end),
    .frame_abort (frame_abort),
    .cds_rx      (res_cds),
    .cdm_req     (cdm_req),
    .cdm_bit     (cdm_bit),
    .cdm_busy    (cdm_busy),
    .reg_valid   (reg_valid_i),
    .reg_rdata   (reg_rdata),
    .ev_cdm_err  (ev_cdm_err),
    .dbg_state   (cc_dbg)
  );
  assign reg_valid = reg_valid_i;

  // ===================================================================
  // Sticky status / counters / data outputs
  // ===================================================================
  assign err_evt = ev_crc_err | ev_no_start | ev_no_marker | ev_wdt
                   | ev_meas | ev_cdm_err;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      e_crc       <= 1'b0;
      e_no_start  <= 1'b0;
      e_no_marker <= 1'b0;
      e_wdt       <= 1'b0;
      e_meas      <= 1'b0;
      e_cdm       <= 1'b0;
      frame_cnt   <= '0;
      err_cnt     <= '0;
    end else if (err_clr) begin
      e_crc       <= 1'b0;
      e_no_start  <= 1'b0;
      e_no_marker <= 1'b0;
      e_wdt       <= 1'b0;
      e_meas      <= 1'b0;
      e_cdm       <= 1'b0;
      frame_cnt   <= '0;
      err_cnt     <= '0;
    end else begin
      if (ev_crc_err)   e_crc       <= 1'b1;
      if (ev_no_start)  e_no_start  <= 1'b1;
      if (ev_no_marker) e_no_marker <= 1'b1;
      if (ev_wdt)       e_wdt       <= 1'b1;
      if (ev_meas)      e_meas      <= 1'b1;
      if (ev_cdm_err)   e_cdm       <= 1'b1;
      if (frame_end)    frame_cnt   <= frame_cnt + 16'd1;
      if (err_evt)      err_cnt     <= err_cnt + 16'd1;
    end
  end

  // measurements: tick-domain values, zero-extended into the status fields
  //assign status.e_crc       = e_crc;
  //assign status.e_no_start  = e_no_start;
  //assign status.e_no_marker = e_no_marker;
  //assign status.e_wdt       = e_wdt;
  //assign status.e_meas      = e_meas;
  //assign status.e_cdm       = e_cdm;
  //assign status.ld_ticks    = 24'(tick_delay);            // [ovs ticks]
  //assign status.ld_ma_cyc   = 16'(tick_delay >> HP_W);    // [MA half-cycles]
  //assign status.tap_active  = 16'(rx_dly_active);         // [ovs ticks]
  //assign status.busy        = frame_active || cdm_busy;
  //assign status.frame_cnt   = frame_cnt;
  //assign status.err_cnt     = err_cnt;

  assign data_valid  = res_valid;
  assign pos_out     = res_pos;
  assign pos_len_out = res_len;
  assign pad_out     = res_pad;
  assign cds_out     = res_cds;
  assign crc_ok      = res_crc_ok;

  // FSM state debug: {mf[2:0], rx[2:0], cc[2:0]}
  //assign dbg_state = {23'd0, mf_dbg, rx_dbg, cc_dbg};

endmodule
