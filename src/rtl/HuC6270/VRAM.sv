`default_nettype none

/*
 * verilog model of HuC6270 VDC VRAM
 *
 * (C) 2018 Ford Seidel and Amolak Nagi
 */

module VRAM (input clock, reset_n,
             input logic [15:0] MA,     // Address signals to VRAM
             input logic we,          // write enable
             input logic [15:0] MD_in,  // data in
             output logic [15:0] MD_out // data out
            );

  logic [15:0] data_in, data_out;
  logic [14:0] address;
  logic wren;



  RAMTest bram(.address(address),
               .clock(clock),
               .data(data_in),
               .wren(wren),
               .q(data_out)
              );

endmodule: VRAM

