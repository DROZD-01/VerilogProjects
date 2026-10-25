//=============================================================================
// tb_Interface_UART.sv
//-----------------------------------------------------------------------------
// Self-checking testbench for the "Interface_UART" transmitter/receiver.
//
// WHAT IS TESTED
//   * TX : every frame written via DataValid_i/Data_i is decoded from TX_o
//          (start bit, data-bit order LSB/MSB, parity bit N/E/O/M/S, stop
//          bits) and compared against the driven word.
//   * RX : frames generated on RX_i are checked at Data_o/DataValid_o,
//          ParityError_o and FrameError_o must stay quiet on clean frames.
//   * Error injection : corrupted parity bit  -> ParityError_o expected,
//                       corrupted stop  bit  -> FrameError_o  expected.
//   * Configuration sweep : data width 5..MAX_DW, all parity types,
//                           1..MODE_TYP_STP stop bits, several baud rates.
//   * Extras : back-to-back TX writes (pre-buffer / BuffFull_o),
//              concurrent full-duplex TX+RX, TX->RX loopback echo,
//              DE_o / RE_o polarity probes (non-fatal warnings),
//              TX idle level and post-reset sanity checks.
//
// CONTROL-WORD ENCODINGS (per RTL spec)
//   * BaudSel_i   [4:0]                   : index into baud_of_sel() below
//   * DataBits_i  [$clog2(MAX_DW+1)-1:0]  : exact number of data bits (0..MAX_DW)
//   * ParityType_i[2:0]                   : 0=N  1=E  2=O  3=M  4=S
//   * StopBits_i  [1:0]                   : exact number of stop bits (0..3)
//
// HOW TO RUN  (Xilinx Vivado xsim)
//   Tcl console:
//     xvlog -sv tb_Interface_UART.sv <rtl_path>/Interface_UART.sv
//     xelab -debug typical tb_Interface_UART -s tb_uart
//     xsim tb_uart -runall
//   Optional VCD dump:  xsim tb_uart -runall -testplusarg VCD
//   GUI: add both files to Simulation Sources, set tb_Interface_UART as the
//        simulation top, Run Simulation (waveforms are visible with -debug).
//
// HOW TO RUN  (Icarus Verilog, quick check)
//   iverilog -g2012 -o tb.vvp tb_Interface_UART.sv <rtl_path>/Interface_UART.sv
//   vvp tb.vvp
//
// IF YOUR BAUD TABLE DIFFERS: edit baud_of_sel() and the SWEEP_/MID_/SLOW_
// BaudSel localparams below. Everything else adapts automatically.
//=============================================================================
`timescale 1ns / 1ps

module tb_Interface_UART_Zai;

//-----------------------------------------------------------------------------
// 1. DUT parameter mirror (flexible-control configuration for runtime sweeps)
//-----------------------------------------------------------------------------
localparam integer TB_FREQ_SYSCLK     = 100_000_000; // system clock, Hz
localparam        TB_MODE_BAUDCTRL    = 1'b0;  // '0': variable baud via BaudSel_i
localparam integer TB_BAUD_UART_DEF   = 115200; // used only in fixed-baud mode
localparam integer TB_MAX_DW          = 8;      // max data bits in a word
localparam        TB_MODE_FULLDUPLEX  = 1'b1;
localparam        TB_EN_RECEIVER      = 1'b1;
localparam        TB_EN_TRANSMITTER   = 1'b1;
localparam        TB_MODE_RE_POL      = 1'b0;  // RE active low
localparam        TB_MODE_DE_POL      = 1'b0;  // DE active low
localparam        TB_MODE_VAR_WORD    = 1'b1;  // word size runtime-controlled
localparam        TB_MODE_MSB         = 1'b0;  // '0': LSB first
localparam        TB_MODE_VAR_PAR     = 1'b1;  // parity type runtime-controlled
localparam integer TB_MODE_TYP_PAR    = 0;     // fixed-mode parity: 0 = N
localparam        TB_MODE_VAR_STP     = 1'b1;  // stop bits runtime-controlled
localparam integer TB_MODE_TYP_STP    = 3;     // max stop bits

//-----------------------------------------------------------------------------
// 2. Sweep configuration
//-----------------------------------------------------------------------------
localparam integer SWEEP_BAUD_SEL = 15; // baud used for the big config sweeps
localparam integer MID_BAUD_SEL   = 12; // medium baud for misc tests
localparam integer SLOW_BAUD_SEL  = 5;  // slow baud for timing-realistic tests

// Loop bounds derived from the DUT parameterisation.
localparam integer W_LO = (TB_MODE_VAR_WORD == 1'b1)
                          ? ((TB_MAX_DW >= 5) ? 5 : TB_MAX_DW) // standard UART minimum
                          : TB_MAX_DW;
localparam integer W_HI = TB_MAX_DW;
localparam integer P_LO = (TB_MODE_VAR_PAR == 1'b1) ? 0 : TB_MODE_TYP_PAR;
localparam integer P_HI = (TB_MODE_VAR_PAR == 1'b1) ? 4 : TB_MODE_TYP_PAR;
localparam integer S_LO = (TB_MODE_VAR_STP == 1'b1) ? 1 : TB_MODE_TYP_STP;
localparam integer S_HI = TB_MODE_TYP_STP;

//-----------------------------------------------------------------------------
// 3. Baud selection table -- EDIT TO MATCH THE RTL BAUD TABLE
//-----------------------------------------------------------------------------
function automatic integer baud_of_sel(input integer sel);
  begin
    case (sel)
      00:      baud_of_sel =       50;
      01:      baud_of_sel =       75;
      02:      baud_of_sel =      110;
      03:      baud_of_sel =      134;
      04:      baud_of_sel =      150;
      05:      baud_of_sel =      200;
      06:      baud_of_sel =      300;
      07:      baud_of_sel =      600;
      08:      baud_of_sel =     1200;
      09:      baud_of_sel =     1800;
      10:      baud_of_sel =     2400;
      11:      baud_of_sel =     4800;
      12:      baud_of_sel =     9600;
      13:      baud_of_sel =    19200;
      14:      baud_of_sel =    28800;
      15:      baud_of_sel =    38400;
      16:      baud_of_sel =    57600;
      17:      baud_of_sel =    76800;
      18:      baud_of_sel =   115200;
      19:      baud_of_sel =   230400;
      20:      baud_of_sel =   460800;
      21:      baud_of_sel =   576000;
      22:      baud_of_sel =   921600;
      23:      baud_of_sel =  1000000;
      24:      baud_of_sel =  1500000;
      25:      baud_of_sel =  2000000;
      26:      baud_of_sel =  3000000;
      27:      baud_of_sel =  4000000;
      28:      baud_of_sel =  4500000;
      29:      baud_of_sel =  5000000;
      30:      baud_of_sel =  6000000;
      31:      baud_of_sel = 12000000;
      default: baud_of_sel =   115200;
    endcase
  end
endfunction

//-----------------------------------------------------------------------------
// 4. Clock, reset and DUT I/O
//-----------------------------------------------------------------------------
localparam real CLK_HALF_NS = 0.5e9 / TB_FREQ_SYSCLK;

reg TB_CLK_i = 1'b0;
always #(CLK_HALF_NS) TB_CLK_i = ~TB_CLK_i;

localparam integer CW_DB = $clog2(TB_MAX_DW + 1); // DataBits_i port width

reg                      TB_RST_i       = 1'b1;
reg  [TB_MAX_DW-1:0]     TB_Data_i      = {TB_MAX_DW{1'b0}};
wire [TB_MAX_DW-1:0]     TB_Data_o;
reg                      TB_DataValid_i = 1'b0;
wire                     TB_DataValid_o;
reg  [4:0]               TB_BaudSel_i   = 5'd15;
reg  [CW_DB-1:0]         TB_DataBits_i  = {CW_DB{1'b1}};
reg  [2:0]               TB_ParityType_i= 3'd0;
reg  [1:0]               TB_StopBits_i  = 2'd1;
wire                     TB_RX_Active_o;
wire                     TB_ParityError_o;
wire                     TB_FrameError_o;
wire                     TB_TX_Active_o;
wire                     TB_BuffFull_o;
wire                     TB_RE_o;
wire                     TB_DE_o;
wire                     TB_TX_o;

reg rx_gen = 1'b1;  // serial stimulus driven procedurally
reg lb_en  = 1'b0;  // '1': loopback (TX_o routed back into RX_i)
wire TB_RX_i = lb_en ? TB_TX_o : rx_gen;

//-----------------------------------------------------------------------------
// 5. DUT instantiation
//-----------------------------------------------------------------------------
  Interface_UART
  #(
    .FREQ_SYSCLK     (TB_FREQ_SYSCLK     ), // System clock frequency (in Hz)
    .MODE_BAUDCTRL   (TB_MODE_BAUDCTRL   ), // Baudrate control mode: '1' - fixed, '0' - variable
    .BAUD_UART_DEF   (TB_BAUD_UART_DEF   ), // UART baudrate (for fixed mode)
    .MAX_DW          (TB_MAX_DW          ), // Maximum data bits in a word
    .MODE_FULLDUPLEX (TB_MODE_FULLDUPLEX ), // Enable full-duplex mode
    .EN_RECEIVER     (TB_EN_RECEIVER     ), // Enable receiver circuitry generation
    .EN_TRANSMITTER  (TB_EN_TRANSMITTER  ), // Enable transmitter circuitry generation
    .MODE_RE_POL     (TB_MODE_RE_POL     ), // RE port polarity: '1' - active high / '0' - active low
    .MODE_DE_POL     (TB_MODE_DE_POL     ), // DE port polarity: '1' - active high / '0' - active low
    .MODE_VAR_WORD   (TB_MODE_VAR_WORD   ), // '0' - Word size == MAX_DW, '1' - Word size <= MAX_DW, controlled by DataBits_i
    .MODE_MSB        (TB_MODE_MSB        ), // '0' - LSB (default), '1' - MSB
    .MODE_VAR_PAR    (TB_MODE_VAR_PAR    ), // '0' - Parity type == MODE_TYP_PAR, '1' - Parity type is controlled by ParityType_i
    .MODE_TYP_PAR    (TB_MODE_TYP_PAR    ), // Parity type (in fixed mode): '0' - N, '1' - E, '2' - O, '3' - M, '4' - S.
    .MODE_VAR_STP    (TB_MODE_VAR_STP    ), // '0' - Stop bits == MODE_TYP_STP, '1' - Stop bits <= 3, is controlled by StopBits_i
    .MODE_TYP_STP    (TB_MODE_TYP_STP    )  // Maximum stop bits in a word
  )
  UUT
  (
    .RST_i         (TB_RST_i         ), // System reset
    .CLK_i         (TB_CLK_i         ), // System clock
    .DataValid_i   (TB_DataValid_i   ), // Data Valid to TX
    .Data_i        (TB_Data_i        ), // Data to TX
    .DataValid_o   (TB_DataValid_o   ), // Data Valid from RX
    .Data_o        (TB_Data_o        ), // Data from RX
    .BaudSel_i     (TB_BaudSel_i     ), // Baudrate selector
    .DataBits_i    (TB_DataBits_i    ), // Number of data bits to transmit/receive
    .ParityType_i  (TB_ParityType_i  ), // Parity type selector
    .StopBits_i    (TB_StopBits_i    ), // Number of stop bits to transmit/receive
    .RX_Active_o   (TB_RX_Active_o   ), // RX module is not IDLE
    .ParityError_o (TB_ParityError_o ), // Parity mismatch encountered during reception
    .FrameError_o  (TB_FrameError_o  ), // Frame mismatch encountered during reception
    .TX_Active_o   (TB_TX_Active_o   ), // TX module is not IDLE
    .BuffFull_o    (TB_BuffFull_o    ), // TX message pre-buffer is full
    .RE_o          (TB_RE_o          ), // Receiver    enable (for external ICs)
    .DE_o          (TB_DE_o          ), // Transmitter enable (for external ICs)
    .RX_i          (TB_RX_i          ), // Receiver    input
    .TX_o          (TB_TX_o          )  // Transmitter output
  );

//-----------------------------------------------------------------------------
// 6. Scoreboard
//-----------------------------------------------------------------------------
integer n_pass = 0;
integer n_fail = 0;
integer n_warn = 0;

task automatic check(input ok, input [8*64-1:0] tag);
  begin
    if (ok === 1'b1) begin
      n_pass = n_pass + 1;
    end else begin
      n_fail = n_fail + 1;
      $display("[TB][FAIL] %0s  (t=%0t)", tag, $time);
    end
  end
endtask

// Non-fatal check: design-dependent behaviour (DE/RE, loopback echo, ...)
task automatic check_soft(input ok, input [8*64-1:0] tag);
  begin
    if (ok !== 1'b1) begin
      n_warn = n_warn + 1;
      $display("[TB][WARN] %0s  (t=%0t)", tag, $time);
    end
  end
endtask

//-----------------------------------------------------------------------------
// 7. Deterministic PRNG (xorshift32) -- repeatable runs, no $urandom needed
//-----------------------------------------------------------------------------
reg [31:0] prng = 32'hDEADBEEF;

function automatic [31:0] xorshift32(input [31:0] x);
  reg [31:0] v;
  begin
    v = x;
    v = v ^ (v << 13);
    v = v ^ (v >> 17);
    v = v ^ (v << 5);
    xorshift32 = v;
  end
endfunction

task automatic prng_next();
  begin
    prng = xorshift32(prng);
  end
endtask

//-----------------------------------------------------------------------------
// 8. Helper functions
//-----------------------------------------------------------------------------
function automatic [TB_MAX_DW-1:0] ones_mask(input integer w);
  integer i;
  begin
    ones_mask = {TB_MAX_DW{1'b0}};
    for (i = 0; i < w; i = i + 1) ones_mask[i] = 1'b1;
  end
endfunction

// Maps a data word onto wire-bit order (index = order of appearance on wire).
function automatic [TB_MAX_DW-1:0] order_bits(input [TB_MAX_DW-1:0] d, input integer w);
  integer i;
  begin
    order_bits = {TB_MAX_DW{1'b0}};
    for (i = 0; i < w; i = i + 1)
      order_bits[i] = (TB_MODE_MSB == 1'b1) ? d[w-1-i] : d[i];
  end
endfunction

// Parity bit value for type: 1=Even 2=Odd 3=Mark 4=Space (0=None -> unused).
function automatic parity_bit(input integer ptype, input [TB_MAX_DW-1:0] data, input integer w);
  begin
    case (ptype)
      1:       parity_bit = ^(data & ones_mask(w));    // even
      2:       parity_bit = ~(^(data & ones_mask(w))); // odd
      3:       parity_bit = 1'b1;                      // mark
      4:       parity_bit = 1'b0;                      // space
      default: parity_bit = 1'b1;
    endcase
  end
endfunction

function automatic integer clks_per_bit(input integer baud);
  begin
    clks_per_bit = TB_FREQ_SYSCLK / baud;
  end
endfunction

// Baud the DUT is currently configured for.
function automatic integer cur_baud();
  begin
    if (TB_MODE_BAUDCTRL == 1'b1) cur_baud = TB_BAUD_UART_DEF;
    else                          cur_baud = baud_of_sel(TB_BaudSel_i);
  end
endfunction

//-----------------------------------------------------------------------------
// 9. Basic driving / waiting tasks
//-----------------------------------------------------------------------------
// Apply runtime configuration and let it settle.
task automatic cfg_apply(input integer bsel, input integer dbits,
                         input integer ptype, input integer stp);
  begin
    TB_BaudSel_i    = bsel[4:0];
    TB_DataBits_i   = dbits[CW_DB-1:0];
    TB_ParityType_i = ptype[2:0];
    TB_StopBits_i   = stp[1:0];
    repeat (8) @(posedge TB_CLK_i);
  end
endtask

// One-clock DataValid_i pulse; stable across exactly one sampling edge of
// either clock polarity thanks to NBA-at-posedge driving.
task automatic tx_send(input [TB_MAX_DW-1:0] data);
  begin
    @(posedge TB_CLK_i);
    TB_Data_i      <= data;
    TB_DataValid_i <= 1'b1;
    @(posedge TB_CLK_i);
    TB_DataValid_i <= 1'b0;
  end
endtask

// Wait for the receiver to report a result. Returns which of DataValid_o /
// ParityError_o / FrameError_o was seen (flags are OR-collected over the
// whole window, so 1-clock pulses are never missed).
task automatic rx_wait_result(
  input  integer timeout_clks,
  input  integer extra_clks,
  output reg     saw_valid,
  output reg     saw_perr,
  output reg     saw_ferr
);
  integer k;
  begin
    saw_valid = 1'b0; saw_perr = 1'b0; saw_ferr = 1'b0;
    begin : main_wait
      for (k = 0; k < timeout_clks; k = k + 1) begin
        @(posedge TB_CLK_i);
        if (TB_DataValid_o   === 1'b1) saw_valid = 1'b1;
        if (TB_ParityError_o === 1'b1) saw_perr  = 1'b1;
        if (TB_FrameError_o  === 1'b1) saw_ferr  = 1'b1;
        if ((saw_valid === 1'b1) || (saw_perr === 1'b1) || (saw_ferr === 1'b1))
          disable main_wait;
      end
    end
    begin : extra_wait
      for (k = 0; k < extra_clks; k = k + 1) begin
        @(posedge TB_CLK_i);
        if (TB_DataValid_o   === 1'b1) saw_valid = 1'b1;
        if (TB_ParityError_o === 1'b1) saw_perr  = 1'b1;
        if (TB_FrameError_o  === 1'b1) saw_ferr  = 1'b1;
      end
    end
  end
endtask

//-----------------------------------------------------------------------------
// 10. UART serial frame generator (drives RX_i) and monitor (samples TX_o)
//-----------------------------------------------------------------------------
// Generate one complete frame on RX_i. bad_par inverts the parity bit,
// bad_stop drives the first stop bit low (frame error).
task automatic gen_uart_frame(
  input integer         baud,
  input integer         width,
  input integer         ptype,
  input integer         nstop,
  input [TB_MAX_DW-1:0] data,
  input                 bad_par,
  input                 bad_stop
);
  real bt;
  reg [TB_MAX_DW-1:0] ordered;
  reg pb;
  integer i;
  begin
    bt      = 1.0e9 / baud;
    ordered = order_bits(data & ones_mask(width), width);
    rx_gen  = 1'b1;                              // idle
    #(2.0 * bt);
    rx_gen  = 1'b0;                              // START bit
    #(bt);
    for (i = 0; i < width; i = i + 1) begin      // data bits, wire order
      rx_gen = ordered[i];
      #(bt);
    end
    if (ptype != 0) begin                        // parity bit (if enabled)
      pb = parity_bit(ptype, data, width);
      if (bad_par == 1'b1) pb = ~pb;
      rx_gen = pb;
      #(bt);
    end
    for (i = 0; i < nstop; i = i + 1) begin      // stop bits
      rx_gen = ((i == 0) && (bad_stop == 1'b1)) ? 1'b0 : 1'b1;
      #(bt);
    end
    rx_gen = 1'b1;                               // back to idle
    #(bt);
  end
endtask

// Decode one frame from TX_o. Waits up to start_timeout_clks for the START
// edge, then samples every bit mid-bit. On timeout it reports the failure
// itself and returns with got == 0.
task automatic mon_uart_frame(
  input  integer               baud,
  input  integer               width,
  input  integer               ptype,
  input  integer               nstop,
  input  integer               start_timeout_clks,
  output reg [TB_MAX_DW-1:0]   rdata,
  output reg                   rpar,
  output reg                   par_ok,
  output reg                   stop_ok,
  output reg                   got
);
  real bt;
  reg bitv, exp_par;
  integer i, k;
  begin : body
    bt      = 1.0e9 / baud;
    got     = 1'b0;
    par_ok  = 1'b0;
    stop_ok = 1'b0;
    rdata   = {TB_MAX_DW{1'b0}};
    rpar    = 1'b0;

    // -- wait for START edge (idle 1 -> start 0)
    begin : find_start
      for (k = 0; k < start_timeout_clks; k = k + 1) begin
        @(posedge TB_CLK_i);
        if (TB_TX_o === 1'b0) disable find_start;
      end
    end
    if (TB_TX_o !== 1'b0) begin
      $display("[TB][FAIL] TX start-edge timeout (t=%0t)", $time);
      n_fail = n_fail + 1;
      disable body;
    end

    // -- centre of the START bit (<=1 clk detection skew, negligible vs bt/2)
    #(0.5 * bt);
    if (TB_TX_o !== 1'b0) begin
      $display("[TB][FAIL] TX glitch inside START bit (t=%0t)", $time);
      n_fail = n_fail + 1;
      disable body;
    end

    // -- data bits (rdata[i] = i-th bit on the wire)
    for (i = 0; i < width; i = i + 1) begin
      #(bt);
      bitv     = TB_TX_o;
      rdata[i] = bitv;
    end

    // -- parity bit
    if (ptype != 0) begin
      #(bt);
      rpar    = TB_TX_o;
      exp_par = parity_bit(ptype, rdata, width); // XOR is order-independent
      par_ok  = (rpar === exp_par);
    end else begin
      par_ok = 1'b1;
    end

    // -- stop bits must be high
    stop_ok = 1'b1;
    for (i = 0; i < nstop; i = i + 1) begin
      #(bt);
      if (TB_TX_o !== 1'b1) stop_ok = 1'b0;
    end
    got = 1'b1;
  end
endtask

//-----------------------------------------------------------------------------
// 11. Non-intrusive RS-485 style probes (warnings only)
//-----------------------------------------------------------------------------
task automatic probe_de_during_tx(input integer baud);
  integer k, tmo;
  begin
    tmo = 4 * clks_per_bit(baud) + 1000;
    begin : w
      for (k = 0; k < tmo; k = k + 1) begin
        @(posedge TB_CLK_i);
        if (TB_TX_Active_o === 1'b1) begin
          #(1.5 * (1.0e9 / baud));   // middle of the frame
          check_soft(TB_DE_o === TB_MODE_DE_POL, "DE_o not at active level during TX");
          disable w;
        end
      end
    end
    if (TB_TX_Active_o !== 1'b1)
      check_soft(1'b0, "TX_Active_o never asserted during TX test");
  end
endtask

task automatic probe_re_during_rx(input integer baud);
  integer k, tmo;
  begin
    tmo = 4 * clks_per_bit(baud) + 1000;
    begin : w
      for (k = 0; k < tmo; k = k + 1) begin
        @(posedge TB_CLK_i);
        if (TB_RX_Active_o === 1'b1) begin
          #(1.5 * (1.0e9 / baud));
          check_soft(TB_RE_o === TB_MODE_RE_POL, "RE_o not at active level during RX");
          disable w;
        end
      end
    end
    if (TB_RX_Active_o !== 1'b1)
      check_soft(1'b0, "RX_Active_o never asserted during RX test");
  end
endtask

//-----------------------------------------------------------------------------
// 12. Test-case wrappers
//-----------------------------------------------------------------------------
// Write one word to TX and verify the serialised frame on TX_o.
task automatic test_tx_word(input integer bsel, input integer w,
                            input integer p,  input integer s,
                            input [TB_MAX_DW-1:0] data);
  reg [TB_MAX_DW-1:0] rd;
  reg rpar, pok, sok, got;
  integer baud, tmo;
  begin
    cfg_apply(bsel, w, p, s);
    baud = cur_baud();
    tmo  = (w + 6) * clks_per_bit(baud) + 2000;
    fork
      mon_uart_frame(baud, w, p, s, tmo, rd, rpar, pok, sok, got);
      probe_de_during_tx(baud);
      tx_send(data);
    join
    if (got === 1'b1) begin
      check(rd === order_bits(data & ones_mask(w), w),
            "TX frame data mismatch (check baud table / bit order)");
      if (p != 0) check(pok === 1'b1, "TX parity bit mismatch");
      check(sok === 1'b1, "TX stop bit(s) not high");
    end
    $display("[TB][INFO] TX sel=%0d w=%0d par=%0d stp=%0d sent=%02h recv=%02h par_ok=%b stp_ok=%b",
             bsel, w, p, s, data, rd, pok, sok);
  end
endtask

// Generate one clean frame on RX_i and verify the received word.
task automatic test_rx_word(input integer bsel, input integer w,
                            input integer p,  input integer s,
                            input [TB_MAX_DW-1:0] data);
  reg sv, sp, sf;
  integer baud, tmo, xmo;
  begin
    cfg_apply(bsel, w, p, s);
    baud = cur_baud();
    tmo  = (w + 8) * clks_per_bit(baud) + 2000;
    xmo  = 4 * clks_per_bit(baud) + 500;
    fork
      rx_wait_result(tmo, xmo, sv, sp, sf);
      probe_re_during_rx(baud);
      gen_uart_frame(baud, w, p, s, data, 1'b0, 1'b0);
    join
    check(sv === 1'b1, "RX DataValid_o timeout (frame not accepted)");
    if (sv === 1'b1) begin
      check((TB_Data_o & ones_mask(w)) === (data & ones_mask(w)), "RX data mismatch");
      check(sp === 1'b0, "RX unexpected ParityError_o on clean frame");
      check(sf === 1'b0, "RX unexpected FrameError_o on clean frame");
    end
    $display("[TB][INFO] RX sel=%0d w=%0d par=%0d stp=%0d sent=%02h got=%02h v=%b pe=%b fe=%b",
             bsel, w, p, s, data, TB_Data_o, sv, sp, sf);
  end
endtask

// Frame with deliberately corrupted parity bit.
task automatic test_rx_parity_error(input integer bsel, input integer w,
                                    input integer p,  input integer s,
                                    input [TB_MAX_DW-1:0] data);
  reg sv, sp, sf;
  integer baud, tmo, xmo;
  begin
    cfg_apply(bsel, w, p, s);
    baud = cur_baud();
    tmo  = (w + 8) * clks_per_bit(baud) + 2000;
    xmo  = 2 * (w + 8) * clks_per_bit(baud) + 2000;
    fork
      rx_wait_result(tmo, xmo, sv, sp, sf);
      gen_uart_frame(baud, w, p, s, data, 1'b1, 1'b0);
    join
    check(sp === 1'b1, "RX ParityError_o not asserted on corrupt parity");
    $display("[TB][INFO] RX parity-error inj: valid=%b perr=%b ferr=%b", sv, sp, sf);
  end
endtask

// Frame with deliberately corrupted stop bit.
task automatic test_rx_frame_error(input integer bsel, input integer w,
                                   input integer p,  input integer s,
                                   input [TB_MAX_DW-1:0] data);
  reg sv, sp, sf;
  integer baud, tmo, xmo;
  begin
    cfg_apply(bsel, w, p, s);
    baud = cur_baud();
    tmo  = (w + 8) * clks_per_bit(baud) + 2000;
    xmo  = 2 * (w + 8) * clks_per_bit(baud) + 2000;
    fork
      rx_wait_result(tmo, xmo, sv, sp, sf);
      gen_uart_frame(baud, w, p, s, data, 1'b0, 1'b1);
    join
    check(sf === 1'b1, "RX FrameError_o not asserted on corrupt stop bit");
    $display("[TB][INFO] RX frame-error inj: valid=%b perr=%b ferr=%b", sv, sp, sf);
  end
endtask

// Two writes back-to-back -> exercises the TX pre-buffer (BuffFull_o).
task automatic test_tx_backtoback;
  reg [TB_MAX_DW-1:0] d0, d1, rd0, rd1;
  reg rp, pk, sk, g0, g1;
  integer baud, tmo;
  begin
    cfg_apply(MID_BAUD_SEL, TB_MAX_DW, 0, 1);
    baud = cur_baud();
    tmo  = (TB_MAX_DW + 6) * clks_per_bit(baud) + 2000;
    prng_next; d0 = prng[TB_MAX_DW-1:0];
    prng_next; d1 = prng[TB_MAX_DW-1:0];
    fork
      begin
        mon_uart_frame(baud, TB_MAX_DW, 0, 1, 2 * tmo, rd0, rp, pk, sk, g0);
        mon_uart_frame(baud, TB_MAX_DW, 0, 1, 2 * tmo, rd1, rp, pk, sk, g1);
      end
      begin
        tx_send(d0);
        @(negedge TB_BuffFull_o);
        tx_send(d1);
        $display("[TB][INFO] back-to-back: BuffFull_o=%b right after 2nd write", TB_BuffFull_o);
      end
    join
    if (g0 === 1'b1) check(rd0 === order_bits(d0, TB_MAX_DW), "B2B frame #1 data mismatch");
    if (g1 === 1'b1) check(rd1 === order_bits(d1, TB_MAX_DW), "B2B frame #2 data mismatch");
    $display("[TB][INFO] back-to-back done: #1=%02h #2=%02h", rd0, rd1);
  end
endtask

// Concurrent TX and RX traffic on independent serial lines.
task automatic test_full_duplex(input integer bsel, input integer w,
                                input integer p,  input integer s);
  reg [TB_MAX_DW-1:0] dtx, drx, rdt;
  reg rp, pk, sk, got;
  reg sv, sp, sf;
  integer baud, tmo, xmo;
  begin
    cfg_apply(bsel, w, p, s);
    baud = cur_baud();
    tmo  = (w + 8) * clks_per_bit(baud) + 2000;
    xmo  = 2 * (w + 8) * clks_per_bit(baud) + 2000;
    prng_next; dtx = prng[TB_MAX_DW-1:0];
    prng_next; drx = prng[TB_MAX_DW-1:0];
    fork
      mon_uart_frame(baud, w, p, s, tmo, rdt, rp, pk, sk, got);
      tx_send(dtx);
      rx_wait_result(tmo, xmo, sv, sp, sf);
      gen_uart_frame(baud, w, p, s, drx, 1'b0, 1'b0);
    join
    if (got === 1'b1)
      check(rdt === order_bits(dtx & ones_mask(w), w), "Full-duplex TX-side data mismatch");
    check(sv === 1'b1, "Full-duplex RX-side DataValid_o timeout");
    if (sv === 1'b1)
      check((TB_Data_o & ones_mask(w)) === (drx & ones_mask(w)), "Full-duplex RX-side data mismatch");
    check(sp === 1'b0, "Full-duplex unexpected ParityError_o");
    check(sf === 1'b0, "Full-duplex unexpected FrameError_o");
    $display("[TB][INFO] full-duplex sel=%0d w=%0d par=%0d stp=%0d done", bsel, w, p, s);
  end
endtask

// Wire TX_o straight back into RX_i (models an RS-485 bus echo).
task automatic test_loopback;
  reg [TB_MAX_DW-1:0] d, rdt;
  reg rp, pk, sk, got;
  reg sv, sp, sf;
  integer baud, tmo, xmo;
  begin
    cfg_apply(MID_BAUD_SEL, TB_MAX_DW, 0, 1);
    baud = cur_baud();
    tmo  = (TB_MAX_DW + 6) * clks_per_bit(baud) + 2000;
    xmo  = 2 * (TB_MAX_DW + 8) * clks_per_bit(baud) + 2000;
    prng_next; d = prng[TB_MAX_DW-1:0];
    lb_en = 1'b1;
    fork
      mon_uart_frame(baud, TB_MAX_DW, 0, 1, tmo, rdt, rp, pk, sk, got);
      rx_wait_result(tmo, xmo, sv, sp, sf);
      tx_send(d);
    join
    lb_en = 1'b0;
    if (got === 1'b1)
      check(rdt === order_bits(d, TB_MAX_DW), "Loopback TX frame corrupted");
    check_soft(sv === 1'b1, "Loopback echo not received (RX gated during TX?)");
    if (sv === 1'b1)
      check_soft((TB_Data_o & ones_mask(TB_MAX_DW)) === d, "Loopback echo data mismatch");
    $display("[TB][INFO] loopback: tx_mon=%02h rx_valid=%b rx_data=%02h", rdt, sv, TB_Data_o);
  end
endtask

//-----------------------------------------------------------------------------
// 13. Main test sequence
//-----------------------------------------------------------------------------
integer w_i, p_i, s_i;

initial begin : main
  $timeformat(-6, 2, " us", 12);
  $display("==================================================================");
  $display(" Interface_UART self-checking testbench");
  $display("   FREQ_SYSCLK   = %0d Hz", TB_FREQ_SYSCLK);
  $display("   MAX_DW        = %0d, MSB-first = %0d, full-duplex = %0d",
           TB_MAX_DW, TB_MODE_MSB, TB_MODE_FULLDUPLEX);
  $display("   Sweep baud    = %0d (BaudSel_i = %0d)",
           baud_of_sel(SWEEP_BAUD_SEL), SWEEP_BAUD_SEL);
  $display("   Misc bauds    = %0d (sel %0d), %0d (sel %0d)",
           baud_of_sel(MID_BAUD_SEL), MID_BAUD_SEL,
           baud_of_sel(SLOW_BAUD_SEL), SLOW_BAUD_SEL);
  $display("   If these do not match your RTL baud table, edit baud_of_sel()");
  $display("==================================================================");

  TB_RST_i = 1'b1;
  rx_gen   = 1'b1;
  lb_en    = 1'b0;
  cfg_apply(SWEEP_BAUD_SEL, TB_MAX_DW, 0, 1);
  repeat (16) @(posedge TB_CLK_i);
  TB_RST_i = 1'b0;
  repeat (16) @(posedge TB_CLK_i);

  // -- post-reset sanity
  check(TB_TX_o === 1'b1,        "TX_o not idle (high) after reset");
  check(TB_DataValid_o !== 1'b1, "DataValid_o asserted after reset");
  check(TB_TX_Active_o !== 1'b1, "TX_Active_o asserted after reset");
  check(TB_RX_Active_o !== 1'b1, "RX_Active_o asserted after reset");

  // ---------------- TX: full configuration sweep ----------------
  if (TB_EN_TRANSMITTER == 1'b1) begin
    $display("[TB] ---- TX configuration sweep (w x parity x stop) ----");
    for (w_i = W_LO; w_i <= W_HI; w_i = w_i + 1) begin
      for (p_i = P_LO; p_i <= P_HI; p_i = p_i + 1) begin
        for (s_i = S_LO; s_i <= S_HI; s_i = s_i + 1) begin
          prng_next;
          test_tx_word(SWEEP_BAUD_SEL, w_i, p_i, s_i, prng[TB_MAX_DW-1:0]);
        end
      end
    end
    $display("[TB] ---- TX baud sweep ----");
    test_tx_word(SLOW_BAUD_SEL, TB_MAX_DW, 0, 1, 8'hA5);
    test_tx_word(MID_BAUD_SEL,  TB_MAX_DW, 1, 2, 8'h5A);
    $display("[TB] ---- TX back-to-back writes (pre-buffer) ----");
    test_tx_backtoback;
  end

  // ---------------- RX: full configuration sweep ----------------
  if (TB_EN_RECEIVER == 1'b1) begin
    $display("[TB] ---- RX configuration sweep (w x parity x stop) ----");
    for (w_i = W_LO; w_i <= W_HI; w_i = w_i + 1) begin
      for (p_i = P_LO; p_i <= P_HI; p_i = p_i + 1) begin
        for (s_i = S_LO; s_i <= S_HI; s_i = s_i + 1) begin
          prng_next;
          test_rx_word(SWEEP_BAUD_SEL, w_i, p_i, s_i, prng[TB_MAX_DW-1:0]);
        end
      end
    end
    $display("[TB] ---- RX baud sweep ----");
    test_rx_word(SLOW_BAUD_SEL, TB_MAX_DW, 0, 1, 8'h3C);
    test_rx_word(MID_BAUD_SEL,  TB_MAX_DW, 2, 2, 8'hC3);
    $display("[TB] ---- RX error injection ----");
    prng_next; test_rx_parity_error(SWEEP_BAUD_SEL, TB_MAX_DW, 1, 1, prng[TB_MAX_DW-1:0]);
    prng_next; test_rx_parity_error(SWEEP_BAUD_SEL, TB_MAX_DW, 2, 1, prng[TB_MAX_DW-1:0]);
    prng_next; test_rx_parity_error(SWEEP_BAUD_SEL, TB_MAX_DW, 3, 1, prng[TB_MAX_DW-1:0]);
    prng_next; test_rx_parity_error(SWEEP_BAUD_SEL, TB_MAX_DW, 4, 1, prng[TB_MAX_DW-1:0]);
    prng_next; test_rx_frame_error (SWEEP_BAUD_SEL, TB_MAX_DW, 0, 1, prng[TB_MAX_DW-1:0]);
    prng_next; test_rx_frame_error (SWEEP_BAUD_SEL, TB_MAX_DW, 1, 2, prng[TB_MAX_DW-1:0]);
  end

  // ---------------- Full duplex and loopback ----------------
  if ((TB_MODE_FULLDUPLEX == 1'b1) && (TB_EN_RECEIVER == 1'b1) && (TB_EN_TRANSMITTER == 1'b1)) begin
    $display("[TB] ---- Full-duplex concurrent TX + RX ----");
    test_full_duplex(SWEEP_BAUD_SEL, TB_MAX_DW, 0, 1);
    test_full_duplex(MID_BAUD_SEL,   TB_MAX_DW, 1, 2);
  end

  if ((TB_EN_RECEIVER == 1'b1) && (TB_EN_TRANSMITTER == 1'b1)) begin
    $display("[TB] ---- Loopback (TX_o -> RX_i echo) ----");
    test_loopback;
  end

  repeat (100) @(posedge TB_CLK_i);

  $display("==================================================================");
  $display(" Scoreboard: PASS=%0d  FAIL=%0d  WARN=%0d", n_pass, n_fail, n_warn);
  if (n_fail == 0) $display(" *** TEST PASSED ***");
  else             $display(" *** TEST FAILED ***");
  $display("==================================================================");
  $finish;
end

//-----------------------------------------------------------------------------
// 14. Watchdog and optional waveform dump
//-----------------------------------------------------------------------------
// 100 ms of simulated time is far more than the whole suite needs.
initial begin
  #100_000_000_000;
  $display("[TB][FATAL] Global watchdog timeout - sequence hung");
  $display(" *** TEST FAILED ***");
  $finish;
end
endmodule