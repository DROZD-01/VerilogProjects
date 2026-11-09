`timescale 1ns/1ps
// =============================================================================
// ma_frame_fsm.sv -- MA clock generator and transmit-side frame controller
// -----------------------------------------------------------------------------
// The module owns the MA pin. One MA cycle carries one data bit; MA toggles on
// every 'hp_ce' strobe, the MA half-period boundary pulse that the top level
// derives from the shared tick generator (one boundary every OVERSAMPLE/2
// oversampling ticks). The boundary strobes free-run, so boundaries while no
// frame is running carry the idle level; downstream logic rejects them by
// sampling polarity. Because one MA half-period is exactly OVERSAMPLE/2 ticks
// at every configured MA speed, MA keeps a 50 % duty cycle and its edges stay
// locked to the tick grid.
//
// State diagram:
//
//   MF_IDLE --accept(op_req | cdm_req)--> [MF_CDM_DLY] --> MF_RUN
//      ^                                                      | toggles MA
//      |                                                      | on hp_ce
//      +---------------- MF_GAP <----(done / abort)-----------+
//                inter-frame gap ("timeout state")
//
//   MF_IDLE     : MA pinned to the idle level, waiting for a frame request.
//   MF_CDM_DLY  : optional extension of the pre-frame idle time that encodes
//                 a CDM bit of '1' (see "CDM encoding" below).
//   MF_RUN      : clocks MA. The frame ends only after the receiver confirms
//                 completion ('rx_frame_done' arrives one round-trip delay
//                 after the last bit was sampled), and then on a boundary that
//                 leaves MA at the idle level. The master therefore clocks
//                 exactly the number of extra cycles the delayed return path
//                 needs, for any line delay. Two safety nets bound a dead
//                 line: the bit margin (2*(frame_bits + margin_cyc) MA
//                 half-cycles) and the watchdog ('wdt_ticks' system-clock
//                 ticks); either aborts the frame with 'frame_abort'. Extra
//                 cycles after the slave's last bit are benign: the slave sits
//                 in its post-frame timeout region and ignores clocks until MA
//                 stays at the idle level long enough.
//   MF_GAP      : MA idle for 'gap_ticks' system-clock ticks - the BiSS-C
//                 "timeout state". The gap must exceed both the slave timeout
//                 and the round-trip line delay in absolute time, so that
//                 stale strobes drain before the next frame starts.
//
// Frame events (all one system clock in width):
//   frame_load   : combinational acceptance strobe. The top level latches its
//                  configuration snapshot in this cycle, so the snapshot is
//                  valid before the first MA edge can occur.
//   frame_launch : combinational pulse asserted IN the hp_ce cycle of the
//                  frame's first MA edge (tick-aligned by construction).
//   ld_ref       : combinational pulse asserted IN the hp_ce cycle of the
//                  delay-measurement reference edge - the burst-start edge for
//                  SSI, the 2nd rising MA edge for BiSS (the ACK edge).
//   frame_end    : registered pulse, frame finished or aborted.
//   frame_abort  : registered pulse, margin or watchdog abort.
//   ev_wdt       : registered pulse, watchdog expired.
//
// rise_cnt counts every rising MA edge of the frame, the launch edge included
// when it rises. While the 2nd rising edge's hp_ce cycle is in progress the
// register still reads 1, which is exactly how 'ld_ref' picks the ACK edge.
// The scheme is independent of the configured idle level.
//
// CDM encoding (BiSS-C register communication):
//   One control-data-master (CDM) bit travels in-band per frame. This module
//   implements "CDM = 1" as an extension of the pre-frame MA idle time by
//   'cdm_ext_ticks' system-clock ticks (state MF_CDM_DLY). The exact physical
//   encoding must be aligned with the licensed "BiSS C Protocol Description"
//   and AN11 "BiSS C Register Access Details" (biss-interface.com); the frame
//   sequencing (one CDM bit per frame) is unaffected by that choice.
// =============================================================================
module ma_frame_fsm #(
  // Widest frame the receiver can process; sets the 'frame_bits' port width.
  parameter int unsigned MAX_FRAME_BITS = 75
)(
  input  logic        clk,
  input  logic        rst_n,

  // per-frame snapshot (latched by the top level at frame_load)
  input  logic        proto_biss,    // 1 = BiSS-C, 0 = SSI
  input  logic [$clog2(MAX_FRAME_BITS+1)-1:0] frame_bits, // frame length [bits]
  input  logic [15:0] gap_ticks,     // inter-frame gap [sys ticks]
  input  logic [15:0] margin_cyc,    // extra MA cycles before TX abort
  input  logic [23:0] wdt_ticks,     // frame watchdog [sys ticks], 0 = infinite

  // live controls (act immediately)
  input  logic        en,            // global enable
  input  logic        ma_idle_lvl,   // MA level while idle
  input  logic [11:0] cdm_ext_ticks, // CDM '1' start extension [sys ticks]

  // frame requests
  input  logic        op_req,        // process-data frame request
  input  logic        cdm_req,       // CDM frame request (has priority)
  input  logic        cdm_bit,       // CDM bit to encode into this frame

  // MA half-period boundary strobe: one system-clock pulse every
  // OVERSAMPLE/2 oversampling ticks, free-running
  input  logic        hp_ce,

  // RX feedback
  input  logic        rx_frame_done, // RX completed the frame (delayed!)

  // events (see header)
  output logic        frame_load,
  output logic        frame_launch,
  output logic        ld_ref,
  output logic        frame_abort,
  output logic        frame_end,
  output logic        ev_wdt,

  // status
  output logic        frame_active,
  output logic        frame_is_cdm,
  output logic        ma,
  output logic [2:0]  dbg_state
);

  typedef enum logic [2:0] {
    MF_IDLE    = 3'd0,
    MF_CDM_DLY = 3'd1,
    MF_RUN     = 3'd2,
    MF_GAP     = 3'd3
  } mf_state_e;

  localparam int unsigned BITW       = $clog2(MAX_FRAME_BITS+1);
  localparam int unsigned MAX_HALF_W = 18;
  localparam logic [MAX_HALF_W-1:0] ONE18 = {{(MAX_HALF_W-1){1'b0}}, 1'b1};

  mf_state_e st;

  logic [MAX_HALF_W-1:0] half_cnt;     // MA half-cycles clocked this frame
  logic [MAX_HALF_W-1:0] max_half;     // margin abort bound
  logic [1:0]            rise_cnt;     // rising MA edges seen this frame
  logic                  launched;     // first MA edge emitted
  logic                  rx_done_seen; // sticky of rx_frame_done
  logic [15:0]           gap_cnt;
  logic [11:0]           cdm_dly_cnt;
  logic [23:0]           wdt_cnt;

  // MA level AFTER the edge that an hp_ce strobe produces
  wire new_lvl = ~ma;

  // frame acceptance (combinational)
  wire acc = (st == MF_IDLE) && en && (cdm_req || op_req);
  assign frame_load   = acc;
  assign frame_is_cdm = acc && cdm_req;

  // -------------------------------------------------------------------
  // Combinational, tick-aligned edge events. Every input is a registered
  // signal or a registered strobe, so the pulses are glitch-safe.
  //   edge_now    : this boundary actually toggles MA (it is not the
  //                 idle-aligned clean-stop strobe at frame end);
  //   launch_edge : the frame's first MA edge;
  //   rise_edge   : a rising MA edge of the frame (the launch edge included,
  //                 since rise_cnt is 0 during its hp_ce cycle).
  // -------------------------------------------------------------------
  wire edge_now    = (st == MF_RUN) && hp_ce
                     && !(rx_done_seen && (ma == ma_idle_lvl));
  wire launch_edge = edge_now && !launched;
  wire rise_edge   = edge_now &&  new_lvl;

  assign frame_launch = launch_edge;

  // Delay-measurement reference edge: the burst-start edge for SSI, the 2nd
  // rising MA edge (ACK edge) for BiSS - rise_cnt reads 1 while that edge's
  // hp_ce cycle is in progress.
  assign ld_ref = (!proto_biss) ? launch_edge
                                : (rise_edge && (rise_cnt == 2'd1));

  // Abort bound: 2*(frame_bits + margin_cyc) half-cycles. margin_cyc == 0
  // saturates the bound (practically infinite); the watchdog is then the
  // only frame limit.
  always_comb begin
    if (margin_cyc == '0) begin
      max_half = {MAX_HALF_W{1'b1}};
    end else begin
      max_half = ({{(MAX_HALF_W-BITW){1'b0}}, frame_bits}
                  + {{(MAX_HALF_W-16){1'b0}}, margin_cyc}) << 1;
    end
  end

  assign dbg_state = 3'(st);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st            <= MF_IDLE;
      ma            <= 1'b0;
      half_cnt      <= '0;
      rise_cnt      <= '0;
      launched      <= 1'b0;
      rx_done_seen  <= 1'b0;
      gap_cnt       <= '0;
      cdm_dly_cnt   <= '0;
      wdt_cnt       <= '0;
      frame_abort   <= 1'b0;
      frame_end     <= 1'b0;
      ev_wdt        <= 1'b0;
      frame_active  <= 1'b0;
    end else begin
      // registered event pulses de-asserted
      // (frame_launch / ld_ref are combinational - nothing to reset)
      frame_abort   <= 1'b0;
      frame_end     <= 1'b0;
      ev_wdt        <= 1'b0;

      case (st)

        // -------------------------------------------------------------
        MF_IDLE: begin
          ma <= ma_idle_lvl;
          if (acc) begin
            launched     <= 1'b0;
            rise_cnt     <= '0;
            half_cnt     <= '0;
            gap_cnt      <= '0;
            wdt_cnt      <= '0;
            rx_done_seen <= 1'b0;
            if (cdm_req && cdm_bit && (cdm_ext_ticks != '0)) begin
              cdm_dly_cnt <= cdm_ext_ticks;
              st          <= MF_CDM_DLY;
            end else begin
              st <= MF_RUN;
            end
          end
        end

        // -------------------------------------------------------------
        // CDM '1' encoding: prolonged MA idle before the frame start.
        MF_CDM_DLY: begin
          ma <= ma_idle_lvl;
          if (cdm_dly_cnt != '0) cdm_dly_cnt <= cdm_dly_cnt - 12'd1;
          else                   st          <= MF_RUN;
        end

        // -------------------------------------------------------------
        MF_RUN: begin
          rx_done_seen <= rx_done_seen | rx_frame_done;

          // ---- frame watchdog (sys ticks, absolute time; 0 = infinite) ----
          if ((wdt_ticks != '0) && (wdt_cnt >= wdt_ticks)) begin
            ev_wdt       <= 1'b1;
            frame_abort  <= 1'b1;
            frame_end    <= 1'b1;
            frame_active <= 1'b0;
            st           <= MF_GAP;
          end else begin
            if (wdt_ticks != '0) wdt_cnt <= wdt_cnt + 24'd1;

            if (hp_ce) begin
              if (rx_done_seen && (ma == ma_idle_lvl)) begin
                // ---- clean stop: MA already at idle level ----
                // (no MA toggle at this boundary; the boundary strobe
                // carries the idle level and is rejected downstream by
                // the sampling-polarity filter)
                frame_end    <= 1'b1;
                frame_active <= 1'b0;
                st           <= MF_GAP;
              end else begin
                // toggle one MA half-cycle
                ma       <= new_lvl;
                half_cnt <= half_cnt + ONE18;

                // BiSS ACK reference bookkeeping: count every rising edge
                // of the frame, the launch edge included (correct for
                // either idle level - see header).
                if (new_lvl && proto_biss) begin
                  rise_cnt <= rise_cnt + 2'd1;
                end

                if (!launched) begin
                  // ---- frame launch: the very first MA edge ----
                  // frame_launch (combinational) is already asserted this
                  // cycle; frame_active rises with this edge.
                  launched     <= 1'b1;
                  frame_active <= 1'b1;
                end else if (half_cnt >= max_half) begin
                  // ---- margin abort (dead / stuck line) ----
                  frame_abort  <= 1'b1;
                  frame_end    <= 1'b1;
                  frame_active <= 1'b0;
                  st           <= MF_GAP;
                end
              end
            end
          end
        end

        // -------------------------------------------------------------
        // Inter-frame gap = BiSS-C "timeout state", counted in system-clock
        // ticks (speed-independent absolute time).
        MF_GAP: begin
          ma <= ma_idle_lvl;
          if (gap_cnt >= gap_ticks) begin
            st <= MF_IDLE;
          end else begin
            gap_cnt <= gap_cnt + 16'd1;
          end
        end

        default: st <= MF_IDLE;
      endcase
    end
  end

endmodule
