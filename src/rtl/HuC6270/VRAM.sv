`default_nettype none

/*
 * verilog model of HuC6270 VDC VRAM
 *
 * (C) 2018 Ford Seidel and Amolak Nagi
 */
`define DUMMY_VRAM

module VRAM (input clock, reset_N,
             input logic [15:0] MA,     // Address signals to VRAM
             input logic re, we,        // write enable
             input logic [15:0] MD_in,  // data in
             output logic [15:0] MD_out // data out
            );

  logic [15:0] data_in, data_out;
  logic [14:0] address;
  logic wren;

`ifdef DUMMY_VRAM
  logic [15:0] VRAM_ARR[2**15-1:0]; //32K 16-bit words
  
  initial begin
    $readmemh("GFX.hex", VRAM_ARR);
  end
  
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) MD_out <= 0;
    else if(re) MD_out <= VRAM_ARR[MA];
  end
  
`else
  RAMTest bram(.address(address),
               .clock(clock),
               .data(data_in),
               .wren(wren),
               .q(data_out)
              );
`endif

endmodule: VRAM

