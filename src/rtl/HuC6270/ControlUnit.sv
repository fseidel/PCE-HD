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


  logic WR_n_prev_in, WR_n_prev;
  assign WR_n_prev_in = WR_n;
  Register #($bits(WR_n_prev)) WR_reg(
                                        .clk(clock),
                                        .en(1'b1),
                                        .rst_l(reset_n),
                                        .clear(),
                                        .D(WR_n_prev_in),
                                        .Q(WR_n_prev)
                                     );


  logic RD_n_prev_in, RD_n_prev;
  assign RD_n_prev_in = RD_n;
  Register #($bits(RD_n_prev)) RD_reg(
                                        .clk(clock),
                                        .en(1'b1),
                                        .rst_l(reset_n),
                                        .clear(),
                                        .D(RD_n_prev_in),
                                        .Q(RD_n_prev)
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
  assign addr_in = DI;
  Register #($bits(addr), 5'd0) addr_reg(
                                         .clk(clock),
                                         .en(ld_addr_reg),
                                         .rst_l(),
                                         .clear(),
                                         .D(addr_in),
                                         .Q(addr)
                                       );

  reg_sel_t reg_sel;
  assign reg_sel = addr_in[4:0];


  logic read, write;

  always_comb begin

    // TODO: Wire this up to internals of VDC when needed
    status_in = 8'd0;

    ld_status_reg = 1'd0;
    ld_addr_reg = 1'd0;

    read = 1'd0;
    write = 1'd0;

    DO = 8'd0;


    read  = (~RD_n) && (RD_n != RD_n_prev);
    write = (~WR_n) && (WR_n != WR_n_prev);

    if (~CS_n) begin

      unique case (A)


        A_STATUS_ADDR_REG: begin

          // If read, fetch data from our Status register
          if (read) begin
            DO = status;
          end

          // If write, write data to the address register
          // (specifying which register we want to write to)
          else if (write) begin
            ld_addr_reg = 1'b1;
          end

        end


        A_DATA_LSB: begin

        end



        A_DATA_MSB: begin

        end
     
        default:
          // RETURN $00 IN UNUSED 2'b01 case!
        end 

      endcase

    end /* if (!CS_n) */

  end

endmodule: ControlUnit
