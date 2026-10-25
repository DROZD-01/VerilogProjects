`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/11/2026 10:52:40 AM
// Design Name: 
// Module Name: prim_FractionalTickGen_pkg
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

package prim_FractionalTickGen_pkg;
  
  // Default allowed baudrate values
  localparam int BV_DEF                = 32;
  localparam int BAUD_DEF [0:BV_DEF-1] = 
  '{
         50,       75,      110,      134, // 0-3
        150,      200,      300,      600, // 4-7
       1200,     1800,     2400,     4800, // 8-11
       9600,    19200,    28800,    38400, // 12-15
      57600,    76800,   115200,   230400, // 16-19
     460800,   576000,   921600,  1000000, // 20-23
    1500000,  2000000,  3000000,  4000000, // 24-27
    4500000,  5000000,  6000000, 12000000  // 28-31
  };
  
endpackage
