`timescale 1ns/1ps
// =============================================================================
// cdm_ctrl_fsm.sv -- BiSS-C register-access (CDM/SCP) sequencer
// -----------------------------------------------------------------------------
// BiSS-C register communication is an in-band background protocol: ONE
// control-data-master (CDM) bit is transferred per frame (master -> slave),
// and the slave answers with its control-data-slave (CDS) bit in the same
// frame (slave -> master). A register access therefore spans a sequence of
// ordinary frames, each of which additionally carries normal process data:
//
//   CC_IDLE --reg_req--> CC_CMD --CDM_CMD_W frames (command bits,
//     |                     |        MSB first: start, M/S, R/W, addr)
//     |                     +--write--> CC_WDATA --CDM_DATA_W frames--> CC_DONE
//     |                     +--read ---> CC_RDATA --CDM_DATA_W frames
//     |                                   (collect CDS bit per frame)--> CC_DONE
//     +<------------------------ CC_DONE (reg_valid pulse) -------------+
//     any state --frame_abort--> CC_IDLE (+ ev_cdm_err)
//
// While an access is in progress ('cdm_busy') the sequencer requests one CDM
// frame after another; the top level suspends ordinary process-frame
// requests for that time.
//
// Command word composition ({start, M/S, R/W, addr[7:0]}), the CDM physical
// encoding (implemented in ma_frame_fsm) and the point where read data
// appears (here: one CDS bit per frame, collected MSB first) must be aligned
// with the licensed "BiSS C Protocol Description" and AN11 "BiSS C Register
// Access Details" (biss-interface.com). The frame sequencing structure
// itself is independent of those details.
// =============================================================================
module cdm_ctrl_fsm #(
  // Width of the received CDS field handed over by the top level.
  parameter int unsigned CDS_W = 4
)(
  input  logic clk,
  input  logic rst_n,

  input  logic en,                    // live enable

  // register access request (pulse; sampled when idle)
  input  logic        reg_req,
  input  logic        reg_rw,         // 1 = write
  input  logic [7:0]  reg_addr,
  input  logic [7:0]  reg_wdata,

  // frame events from the frame controller
  input  logic        frame_load,     // pulse: a frame was accepted
  input  logic        frame_is_cdm,   // level: the accepted frame is ours
  input  logic        frame_end,      // pulse: frame finished (or aborted)
  input  logic        frame_abort,    // pulse: frame aborted

  // RX result of the finished frame (latched by the RX before frame_end)
  input  logic [CDS_W-1:0] cds_rx,

  // towards the frame controller
  output logic        cdm_req,        // request the next CDM frame (held)
  output logic        cdm_bit,        // CDM bit to encode

  // status
  output logic        cdm_busy,       // an access is in progress
  output logic        reg_valid,      // pulse: reg_rdata valid (read)
  output logic [7:0]  reg_rdata,
  output logic        ev_cdm_err,     // pulse: access aborted
  output logic [2:0]  dbg_state
);

  typedef enum logic [2:0] {
    CC_IDLE  = 3'd0,
    CC_CMD   = 3'd1,  // shift command bits out, one CDM bit per frame
    CC_WDATA = 3'd2,  // write: shift data bits out, one CDM bit per frame
    CC_RDATA = 3'd3,  // read: collect one CDS bit per frame
    CC_DONE  = 3'd4   // completion event
  } cc_state_e;

  localparam int unsigned CDM_CMD_W  = 11; // {start, M/S, R/W, addr[7:0]}
  localparam int unsigned CDM_DATA_W = 8;  // register data payload bits

  cc_state_e st;

  logic [CDM_CMD_W-1:0]  cmd_sr;
  logic [CDM_DATA_W-1:0] w_sr, r_sr;
  logic [$clog2(CDM_CMD_W+1)-1:0]  cmd_idx;
  logic [$clog2(CDM_DATA_W+1)-1:0] data_idx;
  logic wait_end;   // 1: frame in flight, 0: drive cdm_req for the next bit
  logic rw_q;

  assign cdm_busy = (st != CC_IDLE);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st        <= CC_IDLE;
      cmd_sr    <= '0;
      w_sr      <= '0;
      r_sr      <= '0;
      cmd_idx   <= '0;
      data_idx  <= '0;
      wait_end  <= 1'b0;
      rw_q      <= 1'b0;
      cdm_req   <= 1'b0;
      cdm_bit   <= 1'b0;
      reg_valid <= 1'b0;
      reg_rdata <= '0;
      ev_cdm_err<= 1'b0;
    end else begin
      // defaults
      reg_valid <= 1'b0;
      ev_cdm_err<= 1'b0;
      cdm_req   <= 1'b0;
      cdm_bit   <= 1'b0;

      case (st)

        // -------------------------------------------------------------
        CC_IDLE: begin
          if (en && reg_req) begin
            // Command word, MSB transmitted first:
            // {start, M/S, R/W, addr[7:0]}
            cmd_sr   <= {1'b1, 1'b0, reg_rw, reg_addr};
            w_sr     <= reg_wdata;
            r_sr     <= '0;
            cmd_idx  <= '0;
            data_idx <= '0;
            wait_end <= 1'b0;
            rw_q     <= reg_rw;
            st       <= CC_CMD;
          end
        end

        // -------------------------------------------------------------
        // Shift the command bits out, one CDM bit per frame.
        CC_CMD: begin
          if (frame_abort) begin
            ev_cdm_err <= 1'b1;
            st         <= CC_IDLE;
          end else if (!wait_end) begin
            cdm_req <= 1'b1;
            cdm_bit <= cmd_sr[CDM_CMD_W-1];
            if (frame_load && frame_is_cdm) begin
              cmd_sr   <= {cmd_sr[CDM_CMD_W-2:0], 1'b0};
              cmd_idx  <= cmd_idx + 1'b1;
              wait_end <= 1'b1;
            end
          end else begin // frame in flight
            if (frame_end) begin
              wait_end <= 1'b0;
              if (cmd_idx == CDM_CMD_W) begin
                st <= rw_q ? CC_WDATA : CC_RDATA;
              end
            end
          end
        end

        // -------------------------------------------------------------
        // Write: shift the data bits out, one CDM bit per frame.
        CC_WDATA: begin
          if (frame_abort) begin
            ev_cdm_err <= 1'b1;
            st         <= CC_IDLE;
          end else if (!wait_end) begin
            cdm_req <= 1'b1;
            cdm_bit <= w_sr[CDM_DATA_W-1];
            if (frame_load && frame_is_cdm) begin
              w_sr     <= {w_sr[CDM_DATA_W-2:0], 1'b0};
              data_idx <= data_idx + 1'b1;
              wait_end <= 1'b1;
            end
          end else begin
            if (frame_end) begin
              wait_end <= 1'b0;
              if (data_idx == CDM_DATA_W) st <= CC_DONE;
            end
          end
        end

        // -------------------------------------------------------------
        // Read: collect one CDS bit per frame, MSB first.
        CC_RDATA: begin
          if (frame_abort) begin
            ev_cdm_err <= 1'b1;
            st         <= CC_IDLE;
          end else if (!wait_end) begin
            cdm_req <= 1'b1;          // CDM bit = 0 during read data frames
            cdm_bit <= 1'b0;
            if (frame_load && frame_is_cdm) begin
              wait_end <= 1'b1;
            end
          end else begin
            if (frame_end) begin
              r_sr     <= {r_sr[CDM_DATA_W-2:0], cds_rx[0]};
              data_idx <= data_idx + 1'b1;
              wait_end <= 1'b0;
              if (data_idx == CDM_DATA_W) st <= CC_DONE;
            end
          end
        end

        // -------------------------------------------------------------
        CC_DONE: begin
          reg_valid <= 1'b1;
          reg_rdata <= r_sr;
          st        <= CC_IDLE;
        end

        default: st <= CC_IDLE;
      endcase
    end
  end

  assign dbg_state = 3'(st);

endmodule
