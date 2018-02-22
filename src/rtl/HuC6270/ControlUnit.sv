`default_nettype none

/*
typedef enum logic [1:0] {
  A_STATUS_REG,
  A_ADDRESS_REG,
  A_DATA_LSB,
  A_DATA_MSB } 
*/

/*
 * HuC6270 Verilog Module for Control Unit
 *
 * (C) 2018 Ford Seidel and Amolak Nagi
 */

module ControlUnit( input logic   clock,
                                  reset_n,
                                  RD_n,
                                  WR_n,
                                  CS_n,
                    input a_sel_t A,
                   output logic   BUSY_n,
                                  IRQ_n 
                  );




  always_comb begin

    unique case (A)

      A_STATUS_REG: begin

      end



      A_ADDR_REG: begin

      end



      A_DATA_LSB: begin

      end



      A_DATA_MSB: begin

      end
      

    endcase

  end

endmodule: ControlUnit
