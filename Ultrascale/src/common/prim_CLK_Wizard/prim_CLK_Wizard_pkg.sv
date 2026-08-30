`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/25/2026 06:21:14 PM
// Design Name: 
// Module Name: prim_CLK_Wizard_pkg
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


package prim_CLK_Wizard_pkg;
  
  // Input clock buffer types
  typedef enum bit [1:0]
  {
    ICB0 = 2'd0, // Single ended clock capable pin
    ICB1 = 2'd1, // Differential clock capable pin
    ICB2 = 2'd2, // Global buffer
    ICB3 = 2'd3  // No buffer
  } icb_e;
  
  // Output clock buffer types
  typedef enum bit [2:0]
  {
    OCB0 = 3'd0, // Buffer
    OCB1 = 3'd1, // Buffer with CE
    OCB2 = 3'd2, // BUFG
    OCB3 = 3'd3, // BUFGCE
    OCB4 = 3'd4, // BUFGCE DIV
    OCB5 = 3'd5  // No buffer
  } ocb_e;
  
  // Feedback clock connection types
  typedef enum bit [1:0]
  {
    FCB0 = 2'd0, // Automatic control On-Chip
    FCB1 = 2'd1, // Automatic control Off-Chip
    FCB2 = 2'd2, // User-controlled On-Chip
    FCB3 = 2'd3  // User-controlled Off-Chip
  } fcb_e;
  
  typedef enum bit [2:0]
  {
    SEQ1 = 3'd0, // Sequencing stage 1
    SEQ2 = 3'd1, // Sequencing stage 2
    SEQ3 = 3'd2, // Sequencing stage 3
    SEQ4 = 3'd3, // Sequencing stage 4
    SEQ5 = 3'd4, // Sequencing stage 5
    SEQ6 = 3'd5, // Sequencing stage 6
    SEQ7 = 3'd6  // Sequencing stage 7
  } seq_e;
  
  
endpackage
