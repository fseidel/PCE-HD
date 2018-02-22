`default_nettype none

/*
 * verilog model of HuC6270 VDC
 *
 * (C) 2018 Ford Seidel and Amolak Nagi
 */

/*
module top();

  logic clock, CS_n, WR_n, reset_n, dummy;
  logic [15:0] D;

  vdc_HuC6270 vdc(.*);

  // This entire TB is just to make sure the BRAM is working
  // in simulation. Disregard all of this

  initial begin
    $monitor("state: %s data_out: %x", vdc.state, vdc.data_out);
    clock = 1'b0;
    reset_n = 1'b0;
    reset_n <= 1'b1;
    forever #5 clock = ~clock;
  end

  initial begin

    @(posedge clock);
    @(posedge clock);
    @(posedge clock);
    @(posedge clock);

    $finish;
  end

endmodule: top
*/


module vdc_HuC6270(input logic clock,
                               reset_n,         
                   input logic [15:0] D,// Data in, only lower 8 bits used
                   input logic MRD_n,   // "Memory Read Data" from CPU from VRAM 
                               MWR_n,   // Memory write from CPU to VRAM
                               HSYNC_n,
                               VSYNC_n,
                               CS_n,    // "when low, the CPU is able to read data from registers therein and sprite data thereinto"
                               WR_n,
                               RD_n,
                               EX_8_16, // 1 if 8-bit, 0 if 16. WILL ALWAYS BE SET TO 1
                   input logic [1:0] A, // Address bus indicator
                   output logic [8:0] VD,
                               BUSY_n,
                               IRQ_n
//                                CS_n,
//                                WR_n,
                  );	



  ControlUnit cu(.clock(clock),
                 .reset_n(reset_n),
                 .RD_n(RD_n),
                 .WR_n(WR_n),
                 .CS_n(CS_n),
                 .A(A),
                 .BUSY_n(BUSY_n),
                 .IRQ_n(IRQ_n)
                );



  // VRAM address and data bus wires
  logic [15:0] MA;
  logic [15:0] MD_in, MD_out;
  logic vram_we;

  VRAM vram(.clock(clock),
            .MA(MA),
            .we(vram_we),
            .MD_in(MD_in),
            .MD_out(MD_out)
            );


  AddressUnit au(.clock(clock),
                 .MA(MA)
                );


  logic [15:0] data_in, data_out;
  logic [14:0] address;
  logic wren;

  RAMTest bram(.address(address),
               .clock(clock),
               .data(data_in),
               .wren(wren),
               .q(data_out));


  logic [7:0] D_reg;
  DataBusBuffer(.clock(clock),
                .reset_n(reset_n),
                .D(D),
                .ld(),
                .buf_data(D_reg)
               );








  // This entire FSM is just to test the BRAM. DISREGARD
  enum logic [3:0] {WAIT, READ, WRITE, END} state, next_state;

  always_comb begin

    wren = 0;
    data_in = 16'd0;

    unique case (state)

      WAIT: begin

        next_state = WRITE;
        wren = 1'b1;
        data_in = 16'hF0F0;

      end

      WRITE: begin

        next_state = READ;

      end

      READ, END: begin

        next_state = END;

      end

    endcase

  end

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n)
      state <= WAIT;
    else
      state <= next_state;

  end


endmodule : vdc_HuC6270;







// Old VDC inputs, not needed but going to keep here in case it's ever useful
/*
                    inout logic [7:0]  D_low,
                    inout logic [8:15] D_hi,
                    input logic EW_8_16,     // Data bus width select (8 en, 16 dis), UNUSED
                    inout logic VSYNC_n,
                    inout logic HSYNC_n,
                    output logic DISP, // screen blanking status (blanked dis, displayed en)
                    inout logic SPBG,  // pixel bus (sprite en, bkgrnd dis) (VD8)
                    inout logic [7:0] VD,
                    output logic MWR_n, // VRAM write strobe
                    output logic MRD_n, // VRAM read strobe
                    inout logic [15:0] MD, // VRAM Data bus
                    output logic [15:0] MA, // VRAM Address bus
                    output logic IRQ_n, // IRQ output to HuC6280
                    output logic BUSY_n, // BUSY status output
                    input logic [1:0] A);
*/
