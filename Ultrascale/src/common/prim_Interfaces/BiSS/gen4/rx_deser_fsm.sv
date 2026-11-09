`timescale 1ns/1ps
// =============================================================================
// rx_deser_fsm.sv -- RX deserializer for BiSS-C / SSI process data
// -----------------------------------------------------------------------------
// Rebuilds the parallel frame result from the slave answer stream. The input
// pair ('sample_ce', 'sl_bit') carries exactly one strobe per SL bit, at the
// bit center: the top level produces it by delaying the MA edge stream with
// prim_BiSS_SSI_Pipeline (line-delay tap) and filtering the delayed strobes
// to the sampling polarity. 'sl_bit' is the re-synchronized SL line value at
// the strobe.
//
// The sensor may delay its START bit arbitrarily after the ACK, so the search
// state re-synchronizes on it in every frame and never assumes a fixed
// response latency. Frames end by bit count (BiSS, SSI-PROG) or by the SSI
// monoflop marker (SSI-AUTO); truncated or corrupt frames are flagged and
// never deadlock the FSM - the next frame re-synchronizes.
//
// State diagram:
//
//   RX_RESET --frame_launch--> [BiSS + auto] RX_WAIT_MEAS --meas_done/err--> RX_SEARCH
//   RX_RESET --frame_launch--> [otherwise] ---------------------------------> RX_SEARCH
//   RX_SEARCH --START '1' (BiSS) / first strobe (SSI)--> RX_RUN
//   RX_RUN --last bit (BiSS)---------------------------> RX_DONE
//   RX_RUN --last bit (SSI-PROG)--> RX_M1 --marker '1'--> RX_M2 --'0'--> RX_DONE
//   RX_RUN (SSI-AUTO) --'1' followed by '0' (marker)---> RX_DONE
//   any active state --!frame_active (TX gave up)------> RX_RESET (+ error)
//
//   RX_RESET     : idle, waiting for the frame launch pulse.
//   RX_WAIT_MEAS : BiSS with automatic delay tracking: hold off the START
//                  search until the ACK-referenced line-delay measurement
//                  finishes, because the sampling grid only becomes valid
//                  once the measured tap is applied. On 'meas_err' the old
//                  tap stays valid and the search proceeds.
//   RX_SEARCH    : BiSS: hunt the START bit ('1'). Manual delay mode skips
//                  the fixed frame-sync preamble (activation bit + ACK) by
//                  count, so it cannot mis-lock on it; auto mode skips the
//                  preamble implicitly because RX_WAIT_MEAS ends at the ACK.
//                  SSI: no START bit exists, the first strobe is data bit 0.
//   RX_RUN       : shift / count / CRC feed. SSI-AUTO additionally watches
//                  for the monoflop marker ('1' followed by '0') and counts
//                  the leading pad zeros.
//   RX_M1/RX_M2  : SSI-PROG end-of-data marker check - expect one '1' CL
//                  period (monoflop), then the line back at '0'.
//   RX_DONE      : results latched one cycle after the last bit settled in
//                  the barrel; wait for 'frame_active' to drop, then reset.
//
// Field layout (received MSB first, left-shifting barrel):
//   BiSS : START(1) | CDS(cds_bits) | PDU(pdu_bits) | CRC-6 (if crc_en)
//          The CRC covers CDS+PDU (crc_inc_cds = 1) or PDU only; START and
//          the CRC field itself are never fed. CDS width and CRC coverage
//          follow the sensor profile.
//   SSI  : PDU(pdu_bits)                      [PROG: length from pdu_bits]
//          pad zeros | PDU | monoflop marker  [AUTO: length discovered from
//          the marker; valid only with zero-padded MSBs - heuristic, prefer
//          PROG when the length is known]
//
// BiSS-C process-data CRC: x^6 + x + 1, preset 0, MSB first, no final XOR,
// no reflection, one line bit per beat (prim_CRC_Calculator instance below;
// 'frame_launch' re-seeds it through the synchronous reset). Confirm the
// preset and bit order against your sensor's BiSS profile / BiSS AN3
// ("CRC generator").
// =============================================================================

module rx_deser_fsm #(
  // widest process-data payload [bits]
  parameter int unsigned MAX_PDU_BITS = 64,
  // widest CDS field [bits]
  parameter int unsigned MAX_CDS_BITS = 4,
  // BiSS-C CRC width [bits]
  parameter int unsigned CRC_BITS     = 6,
  // ---- derived geometry ----
  localparam int unsigned MAX_FRAME_BITS = 1 + MAX_CDS_BITS + MAX_PDU_BITS + CRC_BITS,
  localparam int unsigned BITW           = $clog2(MAX_FRAME_BITS + 1),
  localparam int unsigned PDW            = $clog2(MAX_PDU_BITS + 1),
  localparam int unsigned CDW            = $clog2(MAX_CDS_BITS + 1)
)(
  input  logic clk,      // system clock (single clock domain)
  input  logic rst_n,    // async, active low

  // ---- frame geometry (per-frame configuration snapshot) ----
  input  logic            proto_biss,   // 1 = BiSS-C process frame, 0 = SSI
  input  logic            auto_ld_meas, // automatic per-frame delay tracking
  input  logic            crc_en,       // BiSS: append/check CRC-6
  input  logic            crc_inc_cds,  // 1: CRC covers CDS as well as PDU
  input  logic            ssi_len_mode, // SSI: 0 = length from pdu_bits,
                                        //     1 = discover from the marker
  input  logic [CDW-1:0]  cds_bits,     // CDS field width (clamped upstream)
  input  logic [PDW-1:0]  pdu_bits,     // PDU width (clamped upstream)
  input  logic [BITW-1:0] frame_bits,   // total expected frame length [bits]

  // ---- frame timing / control ----
  input  logic            frame_launch, // frame start pulse (tick-aligned)
  input  logic            frame_active, // TX-side frame in progress
  input  logic            meas_done,    // line-delay measurement finished
  input  logic            meas_err,     // line-delay measurement failed

  // ---- sampled SL stream ----
  input  logic            sample_ce,    // one strobe per SL bit, at bit center
  input  logic            sl_bit,       // SL line value at the strobe

  // ---- results (valid with res_valid) ----
  output logic            frame_done,   // pulse: last bit of the frame handled
  output logic            res_valid,    // pulse: result fields valid
  output logic            ev_crc_err,   // pulse: CRC mismatch
  output logic            ev_no_start,  // pulse: START bit / data not found
  output logic            ev_no_marker, // pulse: SSI marker missing/invalid
  output logic [MAX_PDU_BITS-1:0] res_pos, // process data (right-aligned)
  output logic [PDW-1:0]          res_len, // effective PDU length
  output logic [PDW-1:0]          res_pad, // SSI-AUTO leading pad zeros
  output logic [MAX_CDS_BITS-1:0] res_cds, // received CDS bits
  output logic            res_crc_ok,      // frame CRC check result

  // ---- debug ----
  output logic [2:0]      dbg_state        // FSM state snapshot
);

  // -------------------------------------------------------------------
  // Derived constants
  // -------------------------------------------------------------------
  localparam logic [BITW-1:0]     ONE     = {{(BITW-1){1'b0}}, 1'b1};
  localparam logic [PDW-1:0]      PONE    = {{(PDW-1){1'b0}}, 1'b1};
  localparam logic [PDW-1:0]      PDU_MAX = PDW'(MAX_PDU_BITS);
  localparam logic [BITW-1:0]     CRC_WV  = BITW'(CRC_BITS);
  // BiSS-C frame-sync preamble before the START bit: the slave activates its
  // output at the 1st rising MA edge and drops SL (ACK) at the 2nd rising
  // edge. Manual delay mode skips these two bits by count (see RX_RESET).
  localparam int unsigned         BISS_PRE_BITS = 2;
  // BiSS-C process-data CRC: x^6 + x + 1 ("0x03" with the implicit x^6 term
  // dropped).
  localparam logic [CRC_BITS-1:0] CRC6_POLY     = 6'h03;

  // -------------------------------------------------------------------
  // FSM state
  // -------------------------------------------------------------------
  typedef enum logic [2:0] {
    RX_RESET      = 3'd0,
    RX_WAIT_MEAS  = 3'd1,  // BiSS+auto: wait for the ACK-referenced measurement
    RX_SEARCH     = 3'd2,  // BiSS: hunt the START bit; SSI: first strobe = bit 0
    RX_RUN        = 3'd3,  // shift / count / CRC feed (+ SSI-AUTO marker logic)
    RX_M1         = 3'd4,  // SSI-PROG: expect the monoflop marker '1'
    RX_M2         = 3'd5,  // SSI-PROG: expect the line low again after the marker
    RX_DONE       = 3'd6   // results latched; wait for frame_active to drop
  } rx_state_e;

  rx_state_e rx_st;

  logic [MAX_FRAME_BITS-1:0] rx_sr;      // left-shift receive barrel, MSB first
  logic [BITW-1:0]           rx_bit_cnt; // bits shifted so far this frame
  logic [PDW-1:0]            rx_pad_cnt; // SSI-AUTO leading zeros
  logic                      rx_seen_one;// SSI-AUTO: data MSB seen
  logic                      rx_prev_bit;// SSI-AUTO: previous sampled bit
  logic                      rx_done_q;  // results latched for this frame
  logic [1:0]                rx_skip_cnt;// preamble bits to skip (BiSS manual)

  // -------------------------------------------------------------------
  // Frame geometry from the per-frame configuration inputs:
  //   BiSS: START + CDS + PDU + CRC   SSI: PDU (+ marker, RX-side only)
  // -------------------------------------------------------------------
  logic [CDW-1:0]      cds_len;     // CDS width (local copy)
  logic [BITW-1:0]     idx_pdu_lo;  // first PDU bit index (after START+CDS)
  logic [BITW-1:0]     idx_pdu_hi;  // last PDU bit index
  logic [BITW-1:0]     tail_bits;   // trailing CRC bits (BiSS only)

  assign cds_len    = cds_bits;
  assign idx_pdu_lo = BITW'(cds_len) + ONE;
  assign idx_pdu_hi = idx_pdu_lo + pdu_bits - ONE;
  assign tail_bits  = (proto_biss && crc_en) ? CRC_WV : '0;

  // -------------------------------------------------------------------
  // CRC-6, fed with CDS?+PDU bits (START and the CRC field never fed).
  // One line bit per beat (DATA_WIDTH = 1). With CRC_INIT = 0, XOR_OUT = 0
  // and no reflection, CRC_o is the plain running register value.
  // -------------------------------------------------------------------
  logic                crc_bit_stb;
  logic [CRC_BITS-1:0] crc_run;

  assign crc_bit_stb = (rx_st == RX_RUN) && sample_ce &&
                       proto_biss && crc_en &&
                       (rx_bit_cnt >= (crc_inc_cds ? ONE : idx_pdu_lo)) &&
                       (rx_bit_cnt <= idx_pdu_hi);

  prim_CRC_Calculator #(
    .CRC_WIDTH  (CRC_BITS),
    .DATA_WIDTH (1),
    .CRC_POLY   (CRC6_POLY),
    .CRC_INIT   ('0),
    .CRC_XOR_OUT('0),
    .REFIN      (1'b0),
    .REFOUT     (1'b0),
    .LSB_FIRST  (1'b0)
  ) u_crc (
    .CLK_i      (clk),
    .RST_i      (~rst_n | frame_launch),
    .DataValid_i(crc_bit_stb),
    .Data_i     (sl_bit),
    .CRC_o      (crc_run)
  );

  // n-bit all-ones masks, n <= field width (n == width saturates)
  function automatic logic [MAX_PDU_BITS-1:0] pdu_mask_f(logic [PDW-1:0] n);
    logic [MAX_PDU_BITS-1:0] one;
    one = {{(MAX_PDU_BITS-1){1'b0}}, 1'b1};
    return (one << n) - 1'b1;
  endfunction

  function automatic logic [MAX_CDS_BITS-1:0] cds_mask_f(logic [CDW-1:0] n);
    logic [MAX_CDS_BITS-1:0] one;
    one = {{(MAX_CDS_BITS-1){1'b0}}, 1'b1};
    return (one << n) - 1'b1;
  endfunction

  // -------------------------------------------------------------------
  // FSM + sampling bookkeeping
  // -------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_st         <= RX_RESET;
      rx_sr         <= '0;
      rx_bit_cnt    <= '0;
      rx_pad_cnt    <= '0;
      rx_seen_one   <= 1'b0;
      rx_prev_bit   <= 1'b0;
      rx_done_q     <= 1'b0;
      rx_skip_cnt   <= 2'd0;
      frame_done    <= 1'b0;
      res_valid     <= 1'b0;
      ev_crc_err    <= 1'b0;
      ev_no_start   <= 1'b0;
      ev_no_marker  <= 1'b0;
      res_pos       <= '0;
      res_len       <= '0;
      res_pad       <= '0;
      res_cds       <= '0;
      res_crc_ok    <= 1'b0;
    end else begin
      // default: event pulses de-asserted
      frame_done    <= 1'b0;
      res_valid     <= 1'b0;
      ev_crc_err    <= 1'b0;
      ev_no_start   <= 1'b0;
      ev_no_marker  <= 1'b0;

      case (rx_st)

        // -------------------------------------------------------------
        RX_RESET: begin
          if (frame_launch) begin
            rx_bit_cnt  <= '0;
            rx_pad_cnt  <= '0;
            rx_seen_one <= 1'b0;
            rx_prev_bit <= 1'b0;
            rx_done_q   <= 1'b0;
            // Manual BiSS mode has no ACK anchoring: skip the fixed
            // frame-sync preamble (activation bit + ACK) before the START
            // search, so the deserializer cannot mis-lock on it. Auto mode
            // skips it implicitly: RX_WAIT_MEAS ends at the ACK, after the
            // preamble bit has passed.
            rx_skip_cnt <= (proto_biss && !auto_ld_meas)
                           ? 2'(BISS_PRE_BITS) : 2'd0;
            rx_st       <= (proto_biss && auto_ld_meas)
                           ? RX_WAIT_MEAS : RX_SEARCH;
          end
        end

        // -------------------------------------------------------------
        RX_WAIT_MEAS: begin
          if (!frame_active) begin
            ev_no_start <= 1'b1;          // TX gave up before the ACK
            rx_st       <= RX_RESET;
          end else if (meas_done || meas_err) begin
            rx_st <= RX_SEARCH;           // old tap kept on meas_err
          end
        end

        // -------------------------------------------------------------
        RX_SEARCH: begin
          if (!frame_active) begin
            ev_no_start <= 1'b1;          // no START bit within TX margin
            rx_st       <= RX_RESET;
          end else if (sample_ce) begin
            if (rx_skip_cnt != 2'd0) begin
              rx_skip_cnt <= rx_skip_cnt - 2'd1; // BiSS manual: skip preamble+ACK
            end else if (proto_biss) begin
              if (sl_bit) begin             // START bit ('1') locked
                rx_sr      <= {rx_sr[MAX_FRAME_BITS-2:0], sl_bit};
                rx_bit_cnt <= ONE;
                rx_st      <= RX_RUN;
              end
              // '0': keep hunting (ACK low phase / sensor processing delay)
            end else begin
              // SSI: no START bit, the first sampled bit is bit 0
              rx_sr      <= {rx_sr[MAX_FRAME_BITS-2:0], sl_bit};
              rx_bit_cnt <= ONE;
              rx_prev_bit<= sl_bit;
              if (ssi_len_mode) begin
                if (sl_bit) rx_seen_one <= 1'b1;
                else        rx_pad_cnt  <= PONE;
              end
              // single-bit frames jump directly to the marker check
              rx_st <= (!ssi_len_mode && (frame_bits == ONE))
                       ? RX_M1 : RX_RUN;
            end
          end
        end

        // -------------------------------------------------------------
        RX_RUN: begin
          if (!frame_active) begin
            ev_no_marker <= 1'b1;         // frame truncated after lock
            rx_st        <= RX_RESET;
          end else if (sample_ce) begin
            if (proto_biss) begin
              rx_sr      <= {rx_sr[MAX_FRAME_BITS-2:0], sl_bit};
              rx_bit_cnt <= rx_bit_cnt + ONE;
              if (rx_bit_cnt == frame_bits - ONE) rx_st <= RX_DONE;
            end else if (!ssi_len_mode) begin
              rx_sr      <= {rx_sr[MAX_FRAME_BITS-2:0], sl_bit};
              rx_bit_cnt <= rx_bit_cnt + ONE;
              rx_prev_bit<= sl_bit;
              if (rx_bit_cnt == frame_bits - ONE) rx_st <= RX_M1;
            end else begin
              // ---- SSI AUTO: monoflop marker = '1' followed by '0' ----
              if ((rx_bit_cnt >= ONE) && rx_prev_bit && !sl_bit) begin
                rx_st <= RX_DONE;         // previous bit was the marker;
                                          // the current '0' is not shifted
              end else if (rx_bit_cnt >= BITW'(MAX_FRAME_BITS)) begin
                ev_no_marker <= 1'b1;     // barrel full, no marker found
                rx_st        <= RX_DONE;
              end else begin
                rx_sr      <= {rx_sr[MAX_FRAME_BITS-2:0], sl_bit};
                rx_bit_cnt <= rx_bit_cnt + ONE;
                rx_prev_bit<= sl_bit;
                if (!rx_seen_one) begin
                  if (sl_bit) rx_seen_one <= 1'b1;
                  else        rx_pad_cnt  <= rx_pad_cnt + PONE;
                end
              end
            end
          end
        end

        // -------------------------------------------------------------
        RX_M1: begin // SSI-PROG: expect the monoflop end-of-data marker '1'
          if (!frame_active) begin
            ev_no_marker <= 1'b1;
            rx_st        <= RX_RESET;
          end else if (sample_ce) begin
            if (sl_bit) rx_st <= RX_M2;
            else begin
              ev_no_marker <= 1'b1;       // marker missing
              rx_st        <= RX_DONE;    // still complete: TX may stop
            end
          end
        end

        // -------------------------------------------------------------
        RX_M2: begin // SSI-PROG: expect the line back at '0' after the marker
          if (!frame_active) begin
            ev_no_marker <= 1'b1;
            rx_st        <= RX_RESET;
          end else if (sample_ce) begin
            if (sl_bit) ev_no_marker <= 1'b1; // stuck high: not a monoflop
            rx_st <= RX_DONE;
          end
        end

        // -------------------------------------------------------------
        RX_DONE: begin
          if (!rx_done_q) begin
            // 'rx_sr' settled in the previous cycle -> latch results now
            rx_done_q  <= 1'b1;
            frame_done <= 1'b1;
            res_valid  <= 1'b1;

            if (proto_biss) begin
              res_pos <= MAX_PDU_BITS'((rx_sr >> tail_bits)
                           & {{(MAX_FRAME_BITS-MAX_PDU_BITS){1'b0}},
                              pdu_mask_f(pdu_bits)});
              res_len <= pdu_bits;
              res_pad <= '0;
              res_cds <= MAX_CDS_BITS'((rx_sr >> (tail_bits + pdu_bits))
                           & {{(MAX_FRAME_BITS-MAX_CDS_BITS){1'b0}},
                              cds_mask_f(cds_len)});
              if (crc_en) begin
                res_crc_ok <= (rx_sr[CRC_BITS-1:0] == crc_run);
                if (rx_sr[CRC_BITS-1:0] != crc_run) ev_crc_err <= 1'b1;
              end else begin
                res_crc_ok <= 1'b1;
              end
            end else if (!ssi_len_mode) begin
              res_pos    <= rx_sr[MAX_PDU_BITS-1:0] & pdu_mask_f(pdu_bits);
              res_len    <= pdu_bits;
              res_pad    <= '0;
              res_cds    <= '0;
              res_crc_ok <= 1'b1;
            end else begin
              // SSI AUTO: drop the marker bit (last shifted '1'), strip the
              // leading pad zeros; everything in between is the payload.
              if ((rx_bit_cnt - ONE - rx_pad_cnt) > PDU_MAX) begin
                ev_no_marker <= 1'b1;     // discovered length too large
                res_pos <= '0; res_len <= '0; res_crc_ok <= 1'b0;
              end else begin
                res_pos <= MAX_PDU_BITS'((rx_sr >> 1)
                                & {{(MAX_FRAME_BITS-MAX_PDU_BITS){1'b0}},
                                   pdu_mask_f(rx_bit_cnt - ONE - rx_pad_cnt)});
                res_len    <= rx_bit_cnt - ONE - rx_pad_cnt;
                res_crc_ok <= 1'b1;
              end
              res_pad <= rx_pad_cnt;
              res_cds <= '0;
            end
          end else if (!frame_active) begin
            rx_st <= RX_RESET;
          end
        end

        default: rx_st <= RX_RESET;
      endcase
    end
  end

  assign dbg_state = 3'(rx_st);

endmodule
