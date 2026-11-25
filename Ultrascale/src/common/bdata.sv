//------------------------------------------------------------------------------
// DataMover_BDataReg : B_DATA / PREBUF capture buffer with pair-read protection.
//
// Sensor-side words are captured into PREBUF and promoted into B_DATA (Q_o)
// when B_DATA is not "protected":
//  * SPI reads B_DATA as two 16-bit halfwords; RD_Hi_i selects the half
//    ('0' -> [15:00], '1' -> [31:16]). Buff_RD_Latch remembers which halves
//    have been fetched so far ("pair read in flight").
//  * Buff_Protect is active while a pair read is in flight (one half fetched,
//    the other still pending) and during ANY SPI read cycle (RD_Req_i), so a
//    word cannot be replaced while it is (possibly partially) being read.
//  * Once both halves are latched (Buff_Unlock), the latch clears, protection
//    is released, and a pending PREBUF word is promoted.
//
// The raw 32-bit word is exported as Q_o; the parent's SPI read mux slices it
// into halfwords at ADDR, exactly like it does for the CfgReg registers.
//------------------------------------------------------------------------------
module DataMover_BDataReg
  #(
    parameter logic [4:0] ADDR  = 5'h04, // Register address in the SPI map (A_BDATA)
    parameter int         IN_DW = 32     // Width of the captured input word [bits] (BISS_DW)
  )
  (
    input  logic             CLK_i    ,
    input  logic             RST_i    ,
    // Raw SPI read bus (fields decoded once by the parent)
    input  logic             RD_Req_i , // SPI read request strobe (any address)
    input  logic [ 4:0]      Addr_i   , // register address field
    input  logic             RD_Hi_i  , // halfword select: '0' - [15:00], '1' - [31:16]
    // PORT_B input (Sensor -> FPGA)
    input  logic             Valid_i  , // data valid strobe
    input  logic [IN_DW-1:0] Data_i   , // data word (zero-extended into the 32-bit buffer)
    // Buffer word (for SPI read-back at ADDR)
    output logic [31:0]      Q_o
  );

  //--------------------------------------------------------------------------
  // PORT_B capture with pair-read protection (B_DATA / PREBUF)
  //--------------------------------------------------------------------------
  logic        Buff_Locked  ; // B_DATA pair read in flight
  logic [31:0] Buff_Data    ; // word presented to PORT_A
  logic        PreBuff_Full ; // PREBUF holds a word to promote
  logic [31:0] PreBuff      ; // holds B words arriving during a pair read
  logic        Buff_RD_Lo   ; // Request to read low  part of B_DATA ([15:00])
  logic        Buff_RD_Hi   ; // Request to read high part of B_DATA ([31:16])
  logic [1:0]  Buff_RD_Latch; // Saves the read commands from SPI to B_DATA performed so far
  logic        Buff_Unlock  ; // 
  logic        Buff_Protect ; // B_DATA is protected from being overwritten while reading is in progress

  always_comb
  begin
    Buff_RD_Lo   = RD_Req_i && (Addr_i == ADDR) && ~RD_Hi_i;
    Buff_RD_Hi   = RD_Req_i && (Addr_i == ADDR) &&  RD_Hi_i;
    Buff_Locked  = |{Buff_RD_Latch};
    Buff_Unlock  = &{Buff_RD_Latch};
    Buff_Protect = (Buff_Locked && !Buff_Unlock) || RD_Req_i;
  end

  always_ff @(posedge CLK_i)
  begin
    if(RST_i) 
    begin
        Buff_Data     <= 32'b0;
        PreBuff       <= 32'b0;
        PreBuff_Full  <= 1'b0 ;
        Buff_RD_Latch <= 2'b00;
    end 
    else
    begin
      if(Buff_Unlock)
        Buff_RD_Latch <= 2'b00;
      else
      begin
        if(Buff_RD_Lo) Buff_RD_Latch[0] <= 1'b1;
        if(Buff_RD_Hi) Buff_RD_Latch[1] <= 1'b1;
      end
      
      if(PreBuff_Full && ~Buff_Protect)
      begin
        Buff_Data    <= PreBuff;
        PreBuff_Full <= 1'b0;
      end
      
      if(Valid_i) 
      begin
        PreBuff      <= Data_i;
        PreBuff_Full <= 1'b1;
      end
    end
  end

  assign Q_o = Buff_Data;

endmodule