`timescale 1ns / 1ps
/***************************************************************************************************
* Company: 
* Engineer: 
* 
* Create Date: 10/26/2026 10:21:14 PM
* Design Name: 
* Module Name: prim_UART_RX
* Project Name: 
* Target Devices: 
* Tool Versions: 
* Description: 
* 
* Dependencies: 
* 
* Revision:
* Revision 0.01 - File Created
* Additional Comments:
* 
* Parameter definition:
*   
*   
*   
* 
* 
***************************************************************************************************/

module prim_CRC_Calculator
  #(
    parameter int                   CRC_WIDTH   = 6       , // CRC Width
    parameter int                   DATA_WIDTH  = 24      , // Data bits per beat
    parameter logic [CRC_WIDTH-1:0] CRC_POLY    = 6'h2F   , // Polynome, excluding the x^CRC_WIDTH term
    parameter logic [CRC_WIDTH-1:0] CRC_INIT    = 6'h2A   , // Register seed
    parameter logic [CRC_WIDTH-1:0] CRC_XOR_OUT = 6'h15   , // Final XOR
    parameter bit                   REFIN       = 1'b0    , // Input reflection
    parameter bit                   REFOUT      = 1'b0    , // Output reflection
    parameter bit                   LSB_FIRST   = 1'b0      // Byte-lane order
  ) 
  (
    input  logic                   CLK_i        , // System clock
    input  logic                   RST_i        , // Synchronous active-high reset
    input  logic                   DataValid_i  , // Data beat valid
    input  logic [DATA_WIDTH-1:0]  Data_i       , // Data beat
    output logic [ CRC_WIDTH-1:0]  CRC_o          // Running/Final CRC
  );
  
  //-----------------------------------------------------------------------------
  // Parameter check
  initial begin:gen_param_check
    if(CRC_WIDTH < 1 || DATA_WIDTH < 1) 
      $error("[prim_CRC_Calculator]: CRC_WIDTH and DATA_WIDTH must be >= 1");
  end
  
  //------------------------------------------------------------------------------------------
  // Local params
  //------------------------------------------------------------------------------------------
  
  // Bits per table lookup
  localparam int unsigned CHUNK_W = (DATA_WIDTH < CRC_WIDTH)? 
                                    ((DATA_WIDTH < 8) ? DATA_WIDTH : 8): 
                                    ((CRC_WIDTH  < 8) ? CRC_WIDTH  : 8);
  
  // Full table lookups per data input 
  localparam int unsigned FULL_CHUNKS = DATA_WIDTH / CHUNK_W;
  
  // Bits remainder, table lookup (DATA_WIDTH - FULL_CHUNKS*CHUNK_W)
  localparam int unsigned TAIL_W      = DATA_WIDTH % CHUNK_W;
  // Bits remainder forced to be >= 1 (not actually used if < 1)
  localparam int unsigned TAIL_W_EFF  = (TAIL_W > 0) ? TAIL_W : 1;
  
  // Bytes per data input
  localparam int unsigned NUM_BYTES   = DATA_WIDTH / 8;
  // Bits remainder, REFIN / LSB_FIRST (DATA_WIDTH - NUM_BYTES*8)
  localparam int unsigned PART_W      = DATA_WIDTH % 8;
  
  
  //------------------------------------------------------------------------------------------
  // Table generation
  //------------------------------------------------------------------------------------------
  
  typedef logic [CRC_WIDTH-1:0] crc_table_chunk_t [2**CHUNK_W   ]; // Main lookup table
  typedef logic [CRC_WIDTH-1:0] crc_table_tail_t  [2**TAIL_W_EFF]; // Remainder lookup table
  
  //-----------------------------------------------------------------------------
  // Main lookup table generation function
  function automatic crc_table_chunk_t CRC_CalcChunkTable_f ();
    crc_table_chunk_t     tbl ;
    logic [CRC_WIDTH-1:0] lfsr;
    
    for(int unsigned idx = 0; idx < 2**CHUNK_W; idx++) 
    begin
      lfsr = '0;
      lfsr[CRC_WIDTH-1 -: CHUNK_W] = CHUNK_W'(idx);
      repeat(CHUNK_W) 
      begin
        lfsr = lfsr[CRC_WIDTH-1] ? ((lfsr << 1) ^ CRC_POLY) : (lfsr << 1);
      end
      tbl[idx] = lfsr;
    end
    
    return tbl;
  endfunction
  
  //-----------------------------------------------------------------------------
  // Remainder lookup table generation function
  function automatic crc_table_tail_t CRC_CalcTailTable_f ();
    crc_table_tail_t      tbl;
    logic [CRC_WIDTH-1:0] lfsr;
    
    for(int unsigned idx = 0; idx < 2**TAIL_W_EFF; idx++)
    begin
      lfsr = '0;
      lfsr[CRC_WIDTH-1 -: TAIL_W_EFF] = TAIL_W_EFF'(idx);
      repeat(TAIL_W_EFF) 
      begin
        lfsr = lfsr[CRC_WIDTH-1] ? ((lfsr << 1) ^ CRC_POLY) : (lfsr << 1);
      end
      tbl[idx] = lfsr;
    end
    
    return tbl;
  endfunction
  
  //-----------------------------------------------------------------------------
  // Lookup table generation into a parameter
  
  localparam crc_table_chunk_t CRC_TABLE      = CRC_CalcChunkTable_f();
  localparam crc_table_tail_t  CRC_TAIL_TABLE = CRC_CalcTailTable_f();
  
  
  //------------------------------------------------------------------------------------------
  // Input conversion
  //------------------------------------------------------------------------------------------
  
  // Reflect the bits inside every 8-bit chunk (remainder, if any, is reflected within itself).
  function automatic logic [DATA_WIDTH-1:0] CRC_ReflectByteLanes_f (input logic [DATA_WIDTH-1:0] data);
    logic [DATA_WIDTH-1:0] refl;
    refl = '0;
    
    for(int lane = 0; lane < int'(NUM_BYTES); lane++) 
    begin
      for(int unsigned i = 0; i < 8; i++) 
      begin
        refl[8*lane + i] = data[8*lane + 7 - i];
      end
    end
    
    if(PART_W > 0) 
    begin
      for(int i = 0; i < int'(PART_W); i++) 
      begin
        refl[NUM_BYTES*8 + i] = data[NUM_BYTES*8 + PART_W - 1 - i];
      end
    end
    
    return refl;
  endfunction

  // Reverse the order of the 8-bit chunks (little-endian beat. Remainder, if any, keeps its place)
  function automatic logic [DATA_WIDTH-1:0] CRC_ReverseByteLanes_f (input logic [DATA_WIDTH-1:0] data);
    logic [DATA_WIDTH-1:0] rev;
    rev = data;
    
    for(int lane = 0; lane < int'(NUM_BYTES); lane++) 
    begin
      for(int unsigned i = 0; i < 8; i++) 
      begin
        rev[8*lane + i] = data[8*(NUM_BYTES-1-lane) + i];
      end
    end
    
    return rev;
  endfunction
  
  
  //------------------------------------------------------------------------------------------
  // Sarwate table-folder
  //------------------------------------------------------------------------------------------
  
  function automatic logic [CRC_WIDTH-1:0] CRC_Fold_f (input logic [CRC_WIDTH-1:0] crc, input logic [DATA_WIDTH-1:0] data);
    logic [DATA_WIDTH-1:0] word     ;
    logic [ CRC_WIDTH-1:0] crc_next ;
    logic [   CHUNK_W-1:0] chunk    ;
    logic [TAIL_W_EFF-1:0] tail     ;
    
    word = data;
    crc_next = crc;
    
    if(REFIN    ) word = CRC_ReflectByteLanes_f(word);
    if(LSB_FIRST) word = CRC_ReverseByteLanes_f(word);
    
    // Fold the full CHUNK_W-bit chunks, MSB-most chunk first.
    for(int unsigned k = 0; k < FULL_CHUNKS; k++) 
    begin
      chunk    = word[(DATA_WIDTH-1) - k*CHUNK_W -: CHUNK_W];
      crc_next = (crc_next << CHUNK_W) ^ CRC_TABLE[crc_next[CRC_WIDTH-1 -: CHUNK_W] ^ chunk];
    end
    
    // Fold the remaining TAIL_W bits, if any.
    if(TAIL_W > 0) 
    begin
      tail     = word[TAIL_W_EFF-1:0];
      crc_next = (crc_next << TAIL_W) ^ CRC_TAIL_TABLE[crc_next[CRC_WIDTH-1 -: TAIL_W_EFF] ^ tail];
    end
    
    return crc_next;
  endfunction
  
  
  //------------------------------------------------------------------------------------------
  // Output conversion
  //------------------------------------------------------------------------------------------
  
  function automatic logic [CRC_WIDTH-1:0] CRC_ReflectReg_f (input logic [CRC_WIDTH-1:0] crc);
    logic [CRC_WIDTH-1:0] refl;
    
    for(int unsigned i = 0; i < CRC_WIDTH; i++) 
    begin
      refl[i] = crc[CRC_WIDTH-1-i];
    end
    
    return refl;
  endfunction

  function automatic logic [CRC_WIDTH-1:0] CRC_Finalize_f (input logic [CRC_WIDTH-1:0] crc);
    logic [CRC_WIDTH-1:0] fin;
    
    fin = REFOUT ? CRC_ReflectReg_f(crc) : crc;
    
    return fin ^ CRC_XOR_OUT;
  endfunction

  localparam logic [CRC_WIDTH-1:0] CRC_INIT_FINAL = CRC_Finalize_f(CRC_INIT);
  
  //------------------------------------------------------------------------------------------
  // State & Output
  //------------------------------------------------------------------------------------------
  
  logic [CRC_WIDTH-1:0] crc_q; 
  logic [CRC_WIDTH-1:0] crc_d; 
  
  always_comb 
  begin
    crc_d = (DataValid_i)? CRC_Fold_f(crc_q, Data_i) : crc_q;
  end

  always_ff @(posedge CLK_i) 
  begin
    if (RST_i) 
    begin
      crc_q <= CRC_INIT;
      CRC_o <= CRC_INIT_FINAL;
    end 
    else if (DataValid_i) 
    begin
      crc_q <= crc_d;
      CRC_o <= CRC_Finalize_f(crc_d);
    end
  end
  
  
  
  
  
endmodule