`default_nettype none

/*
 * verilog model of HuC6270 VDC VRAM
 *
 * (C) 2018 Ford Seidel and Amolak Nagi
 */

//`define VRAM_INIT
//`define FAKE_VRAM

module VRAM (input clock, reset_N,
             input logic [15:0] MA, MB,          // Address signals to VRAM
             input logic re, we_a, we_b,         // write enable
             input logic [15:0] MD_in, CPU_in,   // data in
             output logic [15:0] MD_out, CPU_out // data out
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
  VRAM_2port_synth vram_real(.address_a(MA[14:0]),
			     .address_b(MB[14:0]),
			     .clock(clock),
			     .data_a(MD_in),
			     .data_b(CPU_in),
			     .wren_a(we_a && MA < 16'h8000),
			     .wren_b(we_b && MB < 16'h8000),
			     .q_a(MD_out),
			     .q_b(CPU_out));
`endif

endmodule: VRAM

