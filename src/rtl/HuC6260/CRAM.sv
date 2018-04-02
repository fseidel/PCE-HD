`default_nettype none

/*
 * verilog model of HuC6260 VDC CRAM
 * 
 * Dual ported for simplicity, so sue me (fseidel).
 * (C) 2018 Ford Seidel and Amolak Nagi
 */

//`define CRAM_INIT
//`define FAKE_CRAM

module CRAM (input              clock, reset_N,
             input logic  [8:0] MA0, MA1, // Address signals to CRAM
             input logic        WE1,      // write enable
             input logic  [8:0] MD1_in,   // data in
             output logic [8:0] MD0_out, MD1_out); // data out
            
  
`ifdef FAKE_CRAM
  localparam CRAM_SIZE = 2**9;
  logic [15:0] CRAM_ARR[CRAM_SIZE-1:0]; //512 16-bit words (we only use 9 bits)
  
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      MD0_out <= 0;
      MD1_out <= 0;
      for(int i = 0; i < CRAM_SIZE; i++)
        CRAM_ARR[i] = 0;
 `ifdef CRAM_INIT
      $readmemh("gunhed_PAL.hex", CRAM_ARR);
 `endif
    end
    else begin
      MD0_out <= CRAM_ARR[MA0][8:0];
      if(~WE1)
        MD1_out <= CRAM_ARR[MA1][8:0];
      else begin
        CRAM_ARR[MA1] <= {7'h00, MD1_in};
        //$display("cram write %x to %x", MD1_in, MA1);
      end
    end
  end
  
`else
  CRAM_synth cram_real(.clock,
                       .address_a(MA0), .address_b(MA1),
                       .data_a(), .data_b(MD1_in), //no write data for port A
                       .wren_a(1'b0), .wren_b(WE1),
                       .q_a(MD0_out), .q_b(MD1_out));
`endif

endmodule: CRAM

