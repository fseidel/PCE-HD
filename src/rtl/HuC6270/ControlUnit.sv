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
                  input logic [7:0] DI,
                  output logic [7:0] DO,
                   output logic   BUSY_n,
                                  IRQ_n 
                  );

  logic [7:0] status_in, status;
  logic ld_status_reg;
  Register #($bits(status)) status_reg(
                                        .clk(clock),
                                        .en(),
                                        .rst_l(),
                                        .clear(),
                                        .D(status_in),
                                        .Q(status)
                                      );


  addr_reg_t addr_in, addr;
  logic ld_addr_reg;
  Register #($bits(addr), 5'd0) addr_reg(
                                         .clk(clock),
                                         .en(),
                                         .rst_l(),
                                         .clear(),
                                         .D(addr_in),
                                         .Q(addr)
                                       );


  always_comb begin

    status_in = 8'd0;
    addr_in = 5'd0;

    ld_status_reg = 1'd0;
    ld_addr_reg = 1'd0;

    if (~CS_n) begin

      unique case (A)


        A_STATUS_ADDR_REG: begin

          // If read, fetch data from our Status register
          if (~RD_n) begin
            
            
            

          end

          // If write, write data to 
          else if (~WR_n) begin

          end

        end


        A_DATA_LSB: begin

        end



        A_DATA_MSB: begin

        end
      

      endcase

    end /* if (!CS_n) */

  end

endmodule: ControlUnit
