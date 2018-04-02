`default_nettype none

/*
 * verilog model of HuC6270 VDC VRAM
 *
 * (C) 2018 Ford Seidel and Amolak Nagi
 */

//`define VRAM_INIT
//`define FAKE_VRAM

module VRAM (input clock, reset_N,
             input logic [15:0] MA,     // Address signals to VRAM
             input logic re, we,        // write enable
             input logic [15:0] MD_in,  // data in
             output logic [15:0] MD_out // data out
            );

`ifdef FAKE_VRAM
  logic [15:0] VRAM_ARR[2**15-1:0]; //32K 16-bit words
  
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      MD_out <= 0;
      for(int i = 0; i < 2<<15; i++) begin
        VRAM_ARR[i] = 0; //some games think VRAM is 0 at the start
      end
 `ifdef VRAM_INIT
      $readmemh("gunhed_GFX.hex", VRAM_ARR);
 `endif
    end
    else if(re)
      MD_out <= VRAM_ARR[MA];
    else if(we) begin
      VRAM_ARR[MA] <= MD_in;
      $display("vram write %x to %x", MD_in, MA);
    end
  end
  
`else
  VRAM_synth vram_real(.address(MA[14:0]),
                       .clock(clock),
                       .data(MD_in),
                       .wren(we && MA < 16'h8000),
                       .q(MD_out));
`endif

endmodule: VRAM

