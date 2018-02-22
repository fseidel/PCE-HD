`default_nettype none

/*
 * Verilog Model of HuC6270 VDC Data Bus Buffer
 *
 * (C) 2018 Ford Seidel and Amolak Nagi
 */

module DataBusBuffer(input logic clock, reset_n,
                     input logic [15:0] D,
                     input logic ld,
                     output logic [7:0] buf_data);

/*
  // Only store the lower 8 bits of the incoming data bus
  Register #(8) databuf(.D(D[7:0]),
                        .Q(buf_data),
                        .load(ld),
                        .clear(1'b0),
                        .clock(clock),
                        .reset_n(reset_n)
                       );
*/

endmodule: DataBusBuffer
