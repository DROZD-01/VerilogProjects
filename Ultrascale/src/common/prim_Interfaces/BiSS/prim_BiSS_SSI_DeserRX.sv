`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/13/2026 08:28:26 PM
// Design Name: 
// Module Name: prim_BiSS_SSI_DeserRX
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module prim_BiSS_SSI_DeserRX
  #(
    parameter  int MAX_PDUBIT     = 32  , // Widest process-data payload [bits]
    parameter  int MAX_CDSBIT     = 4   , // Widest CDS field [bits]
    parameter  int VAL_CRCBIT     = 6   , // BiSS-C CRC width [bits]
    
    localparam int MAX_FRAMEBIT   = 1 + MAX_CDSBIT + MAX_PDUBIT + VAL_CRCBIT, // 
    localparam int WIDTH_CDSBIT   = $clog2(MAX_CDSBIT   + 1) , // 
    localparam int WIDTH_PDUBIT   = $clog2(MAX_PDUBIT   + 1) , // 
    localparam int WIDTH_FRAMEBIT = $clog2(MAX_FRAMEBIT + 1)   // 
  )
  (
    input  logic                      RST_i               , // 
    input  logic                      CLK_i               , // 
    // Controls
    input  logic                      SET_InterfaceMode_i , // '1' - SSI, '0' - BiSS
    input  logic                      SET_EN_AutoLD_i     , // per-frame automatic line-delay tracking
    input  logic                      SET_EN_BiSS_CRC_i   , // 
    input  logic                      SET_CRC_InclCDS_i   , // 
    input  logic                      SET_SSI_LenMode_i   , // 
    input  logic [  WIDTH_CDSBIT-1:0] SET_CDSBits_i       , // 
    input  logic [  WIDTH_PDUBIT-1:0] SET_PDUBits_i       , // 
    input  logic [WIDTH_FRAMEBIT-1:0] SET_FrameBits_i     , // 
    // Events
    input  logic                      FR_Launch_i         , // frame start pulse (tick-aligned)
    input  logic                      FR_Active_i         , // TX-side frame in progress
    input  logic                      Meas_Error_i        , // line-delay measurement finished
    input  logic                      Meas_Done_i         , // line-delay measurement failed
    // Input
    input  logic                      SL_Sample_i         , // one strobe per SL bit, at bit center
    input  logic                      SL_Synched_i        , // SL line value at the strobe
    // Output
    output logic                      RX_FrameDone_o      , // RX completed the frame
    output logic                      Recv_Valid_o        , // 
    output logic [    MAX_PDUBIT-1:0] Recv_PDUBits_o      , // process data (right-aligned)
    output logic [  WIDTH_PDUBIT-1:0] Recv_PDULength_o    , // effective PDU length
    output logic [  WIDTH_PDUBIT-1:0] Recv_SSIPadding_o   , // SSI-AUTO leading pad zeros
    output logic [    MAX_CDSBIT-1:0] Recv_CDSBits_o      , // received CDS bits
    // Status
    output logic                      Err_NoStart_o       , // 
    output logic                      Err_NoMarker_o      , // 
    output logic                      Err_BadCRC_o          // 
  );
  
  
  typedef enum logic [2:0] 
  {
    RX_IDLE       = 3'd0,
    RX_WAIT_MEAS  = 3'd1,  // BiSS+auto: wait for the ACK-referenced measurement
    RX_SEARCH     = 3'd2,  // BiSS: hunt the START bit; SSI: first strobe = bit 0
    RX_RUN        = 3'd3,  // shift / count / CRC feed (+ SSI-AUTO marker logic)
    RX_M1         = 3'd4,  // SSI-PROG: expect the monoflop marker '1'
    RX_M2         = 3'd5,  // SSI-PROG: expect the line low again after the marker
    RX_DONE       = 3'd6   // results latched; wait for frame_active to drop
  } rx_state_e;
  
  
  rx_state_e                 Main_SM          ; // 
  logic [  MAX_FRAMEBIT-1:0] RX_ShiftReg      ; // left-shift receive barrel, MSB first
  logic [WIDTH_FRAMEBIT-1:0] RX_Bit_Count     ; // bits shifted so far this frame
  logic [  WIDTH_PDUBIT-1:0] RX_Padding_Count ; // SSI-AUTO leading zeros
  logic                      RX_MSB_Detected  ; // SSI-AUTO: data MSB seen
  logic                      RX_PrevBit       ; // SSI-AUTO: previous sampled bit
  logic                      RX_Done          ; // results latched for this frame
  logic [               1:0] RX_Skip_Count    ; // preamble bits to skip (BiSS manual)
  
  
  logic [WIDTH_FRAMEBIT-1:0] Index_PDU_Hi     ; // first PDU bit index (after START+CDS)
  logic [WIDTH_FRAMEBIT-1:0] Index_PDU_Lo     ; // last PDU bit index
  logic [WIDTH_FRAMEBIT-1:0] TrailingBits     ; // trailing CRC bits (BiSS only)
  
  always_comb
  begin
    Index_PDU_Lo = SET_CDSBits_i + 1'd1;
    Index_PDU_Hi = Index_PDU_Lo + SET_PDUBits_i - 1'd1;
    TrailingBits = (~SET_InterfaceMode_i && SET_EN_BiSS_CRC_i)? VAL_CRCBIT : 'd0;
  end
  
  
  //--------------------------------------------------------------------
  // CRC-6, fed with CDS?+PDU bits (START and the CRC field never fed).
  // One line bit per beat (DATA_WIDTH = 1). With CRC_INIT = 0, XOR_OUT = 0
  // and no reflection, CRC_o is the plain running register value.
  //--------------------------------------------------------------------
  logic                  CRC_Valid;
  logic [VAL_CRCBIT-1:0] CRC_Out  ;

  assign CRC_Valid  = (Main_SM == RX_RUN) && SL_Sample_i && ~SET_InterfaceMode_i && SET_EN_BiSS_CRC_i &&
                      (RX_Bit_Count >= (SET_CRC_InclCDS_i ? 'd1 : Index_PDU_Lo)) && (RX_Bit_Count <= Index_PDU_Hi);

  prim_CRC_Calculator 
  #(
    .CRC_WIDTH  (VAL_CRCBIT ), // 
    .DATA_WIDTH (1          ), // 
    .CRC_POLY   (6'h03      ), // 
    .CRC_INIT   ('0         ), // 
    .CRC_XOR_OUT('0         ), // 
    .REFIN      (1'b0       ), // 
    .REFOUT     (1'b0       ), // 
    .LSB_FIRST  (1'b0       )  // 
  ) 
  BiSS_CRC_Calc
  (
    .CLK_i       (CLK_i                 ),
    .RST_i       (RST_i | FR_Launch_i   ),
    .DataValid_i (CRC_Valid             ),
    .Data_i      (SL_Synched_i          ),
    .CRC_o       (CRC_Out               )
  );
  
  // n-bit all-ones masks, n <= field width (n == width saturates)
  function automatic logic [MAX_PDUBIT-1:0] pdu_mask_f(logic [WIDTH_PDUBIT-1:0] n);
    logic [MAX_PDUBIT-1:0] one;
    one = {{(MAX_PDUBIT-1){1'b0}}, 1'b1};
    return (one << n) - 1'b1;
  endfunction

  function automatic logic [MAX_PDUBIT-1:0] cds_mask_f(logic [WIDTH_CDSBIT-1:0] n);
    logic [MAX_PDUBIT-1:0] one;
    one = {{(MAX_PDUBIT-1){1'b0}}, 1'b1};
    return (one << n) - 1'b1;
  endfunction
  
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i) 
    begin
      Main_SM           <= RX_IDLE;
      RX_ShiftReg       <=  'd0;
      RX_Bit_Count      <=  'd0;
      RX_Padding_Count  <=  'd0;
      RX_MSB_Detected   <= 1'b0;
      RX_PrevBit        <= 1'b0;
      RX_Done           <= 1'b0;
      RX_Skip_Count     <=  'd0;
      RX_FrameDone_o    <= 1'b0;
      Recv_Valid_o      <= 1'b0;
      Recv_PDUBits_o    <=  'd0;
      Recv_PDULength_o  <=  'd0;
      Recv_SSIPadding_o <=  'd0;
      Recv_CDSBits_o    <=  'd0;
      Err_NoStart_o     <= 1'b0;
      Err_NoMarker_o    <= 1'b0;
      Err_BadCRC_o      <= 1'b0;
    end 
    else
    begin
      RX_FrameDone_o    <= 1'b0;
      Recv_Valid_o      <= 1'b0;
      Err_NoStart_o     <= 1'b0;
      Err_NoMarker_o    <= 1'b0;
      Err_BadCRC_o      <= 1'b0;
      
      case(Main_SM)
        RX_IDLE       :
        begin
          if(FR_Launch_i)
          begin
            RX_Bit_Count      <=  'd0;
            RX_Padding_Count  <=  'd0;
            RX_MSB_Detected   <= 1'b0;
            RX_PrevBit        <= 1'b0;
            RX_Done           <= 1'b0;
            // Manual BiSS mode has no ACK anchoring: skip the fixed
            // frame-sync preamble (activation bit + ACK) before the START
            // search, so the deserializer cannot mis-lock on it. Auto mode
            // skips it implicitly: RX_WAIT_MEAS ends at the ACK, after the
            // preamble bit has passed.
            RX_Skip_Count <= (~SET_InterfaceMode_i && !SET_EN_AutoLD_i)? 2'd2 : 2'd0;
            Main_SM       <= (~SET_InterfaceMode_i &&  SET_EN_AutoLD_i)? RX_WAIT_MEAS : RX_SEARCH;
          end
        end
        
        RX_WAIT_MEAS  :
        begin
          if(~FR_Active_i)
          begin
            Err_NoStart_o     <= 1'b1; // TX gave up before the ACK
            Main_SM           <= RX_IDLE;
          end
          else if(Meas_Done_i || Meas_Error_i)
            Main_SM           <= RX_SEARCH; // old tap kept on meas_err
        end
        
        RX_SEARCH     :
        begin
          if(~FR_Active_i)
          begin
            // no START bit within TX margin
            Err_NoStart_o     <= 1'b1;
            Main_SM           <= RX_IDLE;
          end
          else if(SL_Sample_i)
          begin
            if(RX_Skip_Count != 'd0)
              RX_Skip_Count <= RX_Skip_Count - 1'd1; // BiSS manual: skip preamble+ACK
            else if(~SET_InterfaceMode_i)
            begin
              if(SL_Synched_i) // START bit ('1') locked
              begin
                Main_SM           <= RX_RUN;
                RX_ShiftReg       <= {RX_ShiftReg[MAX_FRAMEBIT-2:0], SL_Synched_i};
                RX_Bit_Count      <=  'd1;
              end
              // '0': keep hunting (ACK low phase / sensor processing delay)
            end
            else
            begin
              // SSI: no START bit, the first sampled bit is bit 0
              RX_ShiftReg       <= {RX_ShiftReg[MAX_FRAMEBIT-2:0], SL_Synched_i};
              RX_Bit_Count      <=  'd1;
              RX_PrevBit        <= SL_Synched_i;
              if(SET_SSI_LenMode_i)
              begin
                if(SL_Synched_i)
                  RX_MSB_Detected   <= 1'b0;
                else
                  RX_Padding_Count  <=  'd1;
              end
              
              // single-bit frames jump directly to the marker check
              Main_SM <= (~SET_SSI_LenMode_i && (SET_FrameBits_i == 'd1))? RX_M1 : RX_RUN;
            end
          end
        end
        
        RX_RUN        :
        begin
          if(~FR_Active_i)
          begin
            // frame truncated after lock
            Err_NoMarker_o    <= 1'b1;
            Main_SM           <= RX_IDLE;
          end
          else if(SL_Sample_i)
          begin
            if(~SET_InterfaceMode_i)
            begin
              RX_ShiftReg       <= {RX_ShiftReg[MAX_FRAMEBIT-2:0], SL_Synched_i};
              RX_Bit_Count      <= RX_Bit_Count + 1'd1;
              if(RX_Bit_Count == (SET_FrameBits_i - 'd1)) Main_SM <= RX_DONE;
            end
            else if(~SET_SSI_LenMode_i)
            begin
              RX_ShiftReg       <= {RX_ShiftReg[MAX_FRAMEBIT-2:0], SL_Synched_i};
              RX_Bit_Count      <= RX_Bit_Count + 1'd1;
              RX_PrevBit        <= SL_Synched_i;
              if(RX_Bit_Count == (SET_FrameBits_i - 'd1)) Main_SM <= RX_M1;
            end
            else
            begin
              // ---- SSI AUTO: monoflop marker = '1' followed by '0' ----
              if((RX_Bit_Count >= 'd1) && RX_PrevBit && !SL_Synched_i)
                Main_SM           <= RX_DONE; // previous bit was the marker, the current '0' is not shifted
              else if(RX_Bit_Count >= WIDTH_FRAMEBIT'(MAX_FRAMEBIT))
              begin
                Err_NoMarker_o    <= 1'b1; // barrel full, no marker found
                Main_SM           <= RX_DONE;
              end
              else
              begin
                RX_ShiftReg       <= {RX_ShiftReg[MAX_FRAMEBIT-2:0], SL_Synched_i};
                RX_Bit_Count      <= RX_Bit_Count + 1'd1;
                RX_PrevBit        <= SL_Synched_i;
                
                if(~RX_MSB_Detected)
                begin
                  if(SL_Synched_i)
                    RX_MSB_Detected   <= 1'b1;
                  else
                    RX_Padding_Count  <= RX_Padding_Count + 1'd1;
                end
              end
            end
          end
        end
        
        RX_M1         : // SSI-PROG: expect the monoflop end-of-data marker '1'
        begin
          if(~FR_Active_i)
          begin
            Err_NoMarker_o    <= 1'b1;
            Main_SM           <= RX_IDLE;
          end
          else if(SL_Sample_i)
          begin
            if(SL_Synched_i)
              Main_SM         <= RX_M2;
            else
            begin
              Err_NoMarker_o  <= 1'b1; // marker missing
              Main_SM         <= RX_DONE; // still complete: TX may stop
            end
          end
        end
        
        RX_M2         : // SSI-PROG: expect the line back at '0' after the marker
        begin
          if(~FR_Active_i)
          begin
            Err_NoMarker_o    <= 1'b1;
            Main_SM           <= RX_IDLE;
          end
          else if(SL_Sample_i)
          begin
            if(SL_Synched_i)
              Err_NoMarker_o  <= 1'b1; // stuck high: not a monoflop
            
            Main_SM           <= RX_DONE;
          end
        end
        
        RX_DONE       :
        begin
          if(!RX_Done)
          begin
            // 'rx_sr' settled in the previous cycle -> latch results now
            RX_Done           <= 1'b1;
            RX_FrameDone_o    <= 1'b1;
            Recv_Valid_o      <= 1'b1;
            
            if(~SET_InterfaceMode_i)
            begin
              Recv_PDUBits_o    <= MAX_PDUBIT'((RX_ShiftReg >> TrailingBits) & {{(MAX_FRAMEBIT-MAX_PDUBIT){1'b0}}, pdu_mask_f(SET_PDUBits_i)});
              Recv_PDULength_o  <= SET_PDUBits_i;
              Recv_SSIPadding_o <= 'd0;
              Recv_CDSBits_o    <= MAX_CDSBIT'((RX_ShiftReg >> (TrailingBits + SET_PDUBits_i)) & {{(MAX_FRAMEBIT-MAX_CDSBIT){1'b0}}, cds_mask_f(SET_CDSBits_i)});
              
              if(SET_EN_BiSS_CRC_i && (RX_ShiftReg[VAL_CRCBIT-1:0] == CRC_Out))
                Err_BadCRC_o <= 1'b1;
            end
            else if(~SET_SSI_LenMode_i)
            begin
              Recv_PDUBits_o    <= RX_ShiftReg[MAX_PDUBIT-1:0] & pdu_mask_f(SET_PDUBits_i);
              Recv_PDULength_o  <= SET_PDUBits_i;
              Recv_SSIPadding_o <= 'd0;
              Recv_CDSBits_o    <= 'd0;
            end
            else
            begin
              // SSI AUTO: drop the marker bit (last shifted '1'), strip the
              // leading pad zeros; everything in between is the payload.
              if((RX_Bit_Count - RX_Padding_Count - 1'd1) > MAX_PDUBIT)
              begin
                Err_NoMarker_o    <= 1'b1;
                Recv_PDUBits_o    <=  'd0;
                Recv_PDULength_o  <=  'd0;
              end
              else
              begin
                Recv_PDUBits_o    <=  MAX_PDUBIT'((RX_ShiftReg >> 1) & {{(MAX_FRAMEBIT-MAX_PDUBIT){1'b0}}, pdu_mask_f(RX_Bit_Count - RX_Padding_Count - 1'd1)});
                Recv_PDULength_o  <= (RX_Bit_Count - RX_Padding_Count - 1'd1);
              end
              Recv_SSIPadding_o <= RX_Padding_Count;
              Recv_CDSBits_o    <= 'd0;
              
            end
          end
          else if (~FR_Active_i)
            Main_SM <= RX_IDLE;
        end
        
        default       : Main_SM <= RX_IDLE;
      endcase
    end
  end
  
  
  
  
  
  
  
  
endmodule
