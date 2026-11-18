`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/18/2026 08:28:26 PM
// Design Name: 
// Module Name: prim_BiSS_SSI_CDMCtrl
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


module prim_BiSS_SSI_CDMCtrl
  #(
    parameter  int MAX_CDSBIT     = 4   , // Widest CDS field [bits]
    parameter  int WIDTH_CDADDR   = 7   , // 
    parameter  int WIDTH_CDDATA   = 8     // register data payload bits
  )
  (
    input  logic                      RST_i               , // 
    input  logic                      CLK_i               , // 
    input  logic                      SET_FSM_Enable_i    , // global enable
    // Register access request (pulse; sampled when idle)
    input  logic                      COMM_Request_i      , //
    input  logic                      COMM_RdWr_i         , // 1 = write
    input  logic [  WIDTH_CDADDR-1:0] COMM_Addr_i         , //
    input  logic [  WIDTH_CDDATA-1:0] COMM_Data_i         , //
    // Frame events from the frame controller
    input  logic                      FR_Load_i           , // pulse: a frame was accepted
    input  logic                      FR_IsCDM_i          , // level: the accepted frame is ours
    input  logic                      FR_Abort_i          , // pulse: frame aborted
    input  logic                      FR_End_i            , // pulse: frame finished (or aborted)
    // RX result of the finished frame (latched by the RX before frame_end)
    input  logic [    MAX_CDSBIT-1:0] Recv_CDSBits_i      , // received CDS bits
    // Towards the frame controller
    output logic                      REQ_CDM_o           , // CDM frame request (has priority)
    output logic                      CDM_Bit_o           , // CDM bit to encode into this frame
    // Status
    output logic                      CDM_Busy_o          , // CDM access is in progress
    output logic                      COMM_Reply_o        , // pulse: COMM_Data_o valid (read)
    output logic [  WIDTH_CDDATA-1:0] COMM_Data_o         , // 
    output logic                      Err_CDM_o             // pulse: access aborted
  );
  
  initial begin
    if(MAX_CDSBIT > WIDTH_CDDATA) $error("[prim_BiSS_SSI_CDMCtrl]: Single frame CDS packet is larger than the whole exchange buffer");
  end
  
  typedef enum logic [2:0] 
  {
    CC_IDLE  = 3'd0,
    CC_CMD   = 3'd1,  // shift command bits out, one CDM bit per frame
    CC_WDATA = 3'd2,  // write: shift data bits out, one CDM bit per frame
    CC_RDATA = 3'd3,  // read: collect one CDS bit per frame
    CC_DONE  = 3'd4   // completion event
  } cc_state_e;
  
  localparam int unsigned WIDTH_CD_CMD = 1 + 1 + 1 + WIDTH_CDADDR; // {start, M/S, R/W, addr[WIDTH_CDADDR-1:0]}
  
  cc_state_e                         MainSM       ; //
  logic [          WIDTH_CD_CMD-1:0] ShiftReg_CMD ; //
  logic [          WIDTH_CDDATA-1:0] ShiftReg_WR  ; //
  logic [          WIDTH_CDDATA-1:0] ShiftReg_RD  ; //
  logic [$clog2(WIDTH_CD_CMD+1)-1:0] Index_CMD    ; //
  logic [$clog2(WIDTH_CDDATA+1)-1:0] Index_Data   ; //
  logic                              Wait_FR_End  ; // 1: frame in flight, 0: drive REQ_CDM_o for the next bit
  logic                              RdWr_Buff    ; //

  assign CDM_Busy_o = (MainSM != CC_IDLE);
  
  
  always_ff @(posedge CLK_i) 
  begin
    if(RST_i) 
    begin
      MainSM       <= CC_IDLE;
      ShiftReg_CMD <=  'd0; 
      ShiftReg_WR  <=  'd0; 
      ShiftReg_RD  <=  'd0; 
      Index_CMD    <=  'd0; 
      Index_Data   <=  'd0; 
      Wait_FR_End  <= 1'b0; 
      RdWr_Buff    <= 1'b0; 
      REQ_CDM_o    <= 1'b0; 
      CDM_Bit_o    <= 1'b0; 
      COMM_Reply_o <= 1'b0; 
      COMM_Data_o  <=  'd0; 
      Err_CDM_o    <= 1'b0; 
    end
    else
    begin
      REQ_CDM_o    <= 1'b0; 
      CDM_Bit_o    <= 1'b0; 
      COMM_Reply_o <= 1'b0; 
      Err_CDM_o    <= 1'b0; 
      
      case(MainSM)
        CC_IDLE  :
        begin
          if(SET_FSM_Enable_i && COMM_Request_i)
          begin
            ShiftReg_CMD <= {1'b1, 1'b0, COMM_RdWr_i, COMM_Addr_i};
            ShiftReg_WR  <= COMM_Data_i;
            ShiftReg_RD  <=  'd0;
            Index_CMD    <=  'd0; 
            Index_Data   <=  'd0; 
            Wait_FR_End  <= 1'b0; 
            RdWr_Buff    <= COMM_RdWr_i; 
            MainSM       <= CC_CMD;
          end
        end
        
        // -------------------------------------------------------------
        // Shift the command bits out, one CDM bit per frame.
        CC_CMD   :
        begin
          if(FR_Abort_i)
          begin
            Err_CDM_o    <= 1'b1; 
            MainSM       <= CC_IDLE;
          end
          else if(~Wait_FR_End)
          begin
            REQ_CDM_o    <= 1'b1; 
            CDM_Bit_o    <= ShiftReg_CMD[WIDTH_CD_CMD-1];
            
            if(FR_Load_i && FR_IsCDM_i)
            begin
              ShiftReg_CMD <= {ShiftReg_CMD[WIDTH_CD_CMD-2:0], 1'b0};
              Index_CMD    <= Index_CMD + 1'd1;
              Wait_FR_End  <= 1'b1;
            end
          end
          else if(FR_End_i) // frame in flight
          begin
            Wait_FR_End  <= 1'b0; 
            if(Index_CMD == WIDTH_CD_CMD)
              MainSM <= (RdWr_Buff)? CC_WDATA : CC_RDATA;
          end
        end
        
        // -------------------------------------------------------------
        // Write: shift the data bits out, one CDM bit per frame.
        CC_WDATA :
        begin
          if(FR_Abort_i)
          begin
            Err_CDM_o    <= 1'b1; 
            MainSM       <= CC_IDLE;
          end
          else if(~Wait_FR_End)
          begin
            REQ_CDM_o    <= 1'b1; 
            CDM_Bit_o    <= ShiftReg_WR[WIDTH_CDDATA-1];
            
            if(FR_Load_i && FR_IsCDM_i)
            begin
              ShiftReg_WR  <= {ShiftReg_WR[WIDTH_CDDATA-2:0], 1'b0};
              Index_Data   <= Index_Data + 1'd1;
              Wait_FR_End  <= 1'b1;
            end
          end
          else if(FR_End_i) // frame in flight
          begin
            Wait_FR_End  <= 1'b0; 
            if(Index_Data >= WIDTH_CDDATA)
              MainSM <= CC_DONE;
          end
        end
        
        CC_RDATA :
        begin
          if(FR_Abort_i)
          begin
            Err_CDM_o    <= 1'b1; 
            MainSM       <= CC_IDLE;
          end
          else if(~Wait_FR_End)
          begin
            REQ_CDM_o    <= 1'b1; 
            CDM_Bit_o    <= ShiftReg_RD[WIDTH_CDDATA-1];
            
            if(FR_Load_i && FR_IsCDM_i)
            begin
              ShiftReg_RD  <= {ShiftReg_RD[WIDTH_CDDATA-MAX_CDSBIT-1:0], Recv_CDSBits_i};
              Index_Data   <= Index_Data + MAX_CDSBIT;
              Wait_FR_End  <= 1'b1;
            end
          end
          else if(FR_End_i) // frame in flight
          begin
            Wait_FR_End  <= 1'b0; 
            if(Index_Data >= WIDTH_CDDATA)
              MainSM <= CC_DONE;
          end
        end
        
        CC_DONE  :
        begin
          COMM_Reply_o <= 1'b1;
          COMM_Data_o  <= ShiftReg_RD;
          MainSM       <= CC_IDLE;
        end
        
        default  : MainSM <= CC_IDLE;
      endcase
    end
  end
  
  
  
  
  
  
  
  
  
endmodule