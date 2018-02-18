`default_nettype none

/*
 * verilog model of HuC6270 VDC VRAM
 *
 * (C) 2018 Ford Seidel and Amolak Nagi
 */

module VRAM (input clock,
             input logic [15:0] MA,     // Address signals to VRAM
             input logic we,          // write enable
             input logic [15:0] MD_in,  // data in
             output logic [15:0] MD_out // data out
            );



endmodule: VRAM

