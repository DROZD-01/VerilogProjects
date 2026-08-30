// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2026 Advanced Micro Devices, Inc. All Rights Reserved.
// -------------------------------------------------------------------------------
// This file contains confidential and proprietary information
// of AMD and is protected under U.S. and international copyright
// and other intellectual property laws.
//
// DISCLAIMER
// This disclaimer is not a license and does not grant any
// rights to the materials distributed herewith. Except as
// otherwise provided in a valid license issued to you by
// AMD, and to the maximum extent permitted by applicable
// law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
// WITH ALL FAULTS, AND AMD HEREBY DISCLAIMS ALL WARRANTIES
// AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
// BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
// INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
// (2) AMD shall not be liable (whether in contract or tort,
// including negligence, or under any other theory of
// liability) for any loss or damage of any kind or nature
// related to, arising under or in connection with these
// materials, including for any direct, or any indirect,
// special, incidental, or consequential loss or damage
// (including loss of data, profits, goodwill, or any type of
// loss or damage suffered as a result of any action brought
// by a third party) even if such damage or loss was
// reasonably foreseeable or AMD had been advised of the
// possibility of the same.
//
// CRITICAL APPLICATIONS
// AMD products are not designed or intended to be fail-
// safe, or for use in any application requiring fail-safe
// performance, such as life-support or safety devices or
// systems, Class III medical devices, nuclear facilities,
// applications related to the deployment of airbags, or any
// other applications that could lead to death, personal
// injury, or severe property or environmental damage
// (individually and collectively, "Critical
// Applications"). Customer assumes the sole risk and
// liability of any use of AMD products in Critical
// Applications, subject only to applicable laws and
// regulations governing limitations on product liability.
//
// THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
// PART OF THIS FILE AT ALL TIMES.
//
// DO NOT MODIFY THIS FILE.

// MODULE VLNV: xilinx.com:ip:microblaze_riscv:1.0

`timescale 1ps / 1ps

`include "vivado_interfaces.svh"

module CPU_Microblaze_RISCV_sv (
  (* X_INTERFACE_IGNORE = "true" *)
  input wire Clk,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire Reset,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire Interrupt,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire [0:31] Interrupt_Address,
  (* X_INTERFACE_IGNORE = "true" *)
  output wire [0:1] Interrupt_Ack,
  (* X_INTERFACE_IGNORE = "true" *)
  output wire [0:31] Instr_Addr,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire [0:63] Instr,
  (* X_INTERFACE_IGNORE = "true" *)
  output wire IFetch,
  (* X_INTERFACE_IGNORE = "true" *)
  output wire I_AS,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire IReady,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire IWAIT,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire ICE,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire IUE,
  (* X_INTERFACE_IGNORE = "true" *)
  output wire [0:31] Data_Addr,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire [0:63] Data_Read,
  (* X_INTERFACE_IGNORE = "true" *)
  output wire [0:63] Data_Write,
  (* X_INTERFACE_IGNORE = "true" *)
  output wire D_AS,
  (* X_INTERFACE_IGNORE = "true" *)
  output wire Read_Strobe,
  (* X_INTERFACE_IGNORE = "true" *)
  output wire Write_Strobe,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire DReady,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire DWait,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire DCE,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire DUE,
  (* X_INTERFACE_IGNORE = "true" *)
  output wire [0:7] Byte_Enable,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire Dbg_Clk,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire Dbg_TDI,
  (* X_INTERFACE_IGNORE = "true" *)
  output wire Dbg_TDO,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire [0:7] Dbg_Reg_En,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire Dbg_Shift,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire Dbg_Capture,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire Dbg_Update,
  (* X_INTERFACE_IGNORE = "true" *)
  output wire [0:7] Dbg_Trig_In,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire [0:7] Dbg_Trig_Ack_In,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire [0:7] Dbg_Trig_Out,
  (* X_INTERFACE_IGNORE = "true" *)
  output wire [0:7] Dbg_Trig_Ack_Out,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire Debug_Rst,
  (* X_INTERFACE_IGNORE = "true" *)
  input wire Dbg_Disable
);

  CPU_Microblaze_RISCV inst (
    .Clk(Clk),
    .Reset(Reset),
    .Interrupt(Interrupt),
    .Interrupt_Address(Interrupt_Address),
    .Interrupt_Ack(Interrupt_Ack),
    .Instr_Addr(Instr_Addr),
    .Instr(Instr),
    .IFetch(IFetch),
    .I_AS(I_AS),
    .IReady(IReady),
    .IWAIT(IWAIT),
    .ICE(ICE),
    .IUE(IUE),
    .Data_Addr(Data_Addr),
    .Data_Read(Data_Read),
    .Data_Write(Data_Write),
    .D_AS(D_AS),
    .Read_Strobe(Read_Strobe),
    .Write_Strobe(Write_Strobe),
    .DReady(DReady),
    .DWait(DWait),
    .DCE(DCE),
    .DUE(DUE),
    .Byte_Enable(Byte_Enable),
    .Dbg_Clk(Dbg_Clk),
    .Dbg_TDI(Dbg_TDI),
    .Dbg_TDO(Dbg_TDO),
    .Dbg_Reg_En(Dbg_Reg_En),
    .Dbg_Shift(Dbg_Shift),
    .Dbg_Capture(Dbg_Capture),
    .Dbg_Update(Dbg_Update),
    .Dbg_Trig_In(Dbg_Trig_In),
    .Dbg_Trig_Ack_In(Dbg_Trig_Ack_In),
    .Dbg_Trig_Out(Dbg_Trig_Out),
    .Dbg_Trig_Ack_Out(Dbg_Trig_Ack_Out),
    .Debug_Rst(Debug_Rst),
    .Dbg_Disable(Dbg_Disable)
  );

endmodule
