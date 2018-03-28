`default_nettype none

/*
 * verilog model of HuC6260 VCE
 *
 * (C) 2018 Ford Seidel and Amolak Nagi
 */


`define SIMULATION
//`define FAKE_CRAM

module vce_HuC6260( input logic clock, reset_N,

                    // From VDC
                    input logic [8:0]  VD,
                    input logic        HSYN,
                    input logic        VSYN,



                    // To/From CPU
                    input logic [2:0]  A,
                    inout wire [7:0]   D, // Data bus, there's an 9th bit but 
                                         // supposedly it's unused
                    // Output to VGA subsystem


                    // output logic RGB_l
                    output logic [2:0] VIDEO_G,
                    output logic [2:0] VIDEO_R,
                    output logic [2:0] VIDEO_B,
                    // output logic RGB_h,



                    output logic       clock_en, // pixel clock enable

                    input logic        RD_n,
                    input logic        WR_n,
                    input logic        CS_n,

                    input              address_mode);    // 8 if hi, 16 if lo, forced to 8 bits

  logic [7:0]   CR;  //control register
  logic [8:0]   CTA; //color table address register ($402)
  logic [7:0]   CTW; //color table write, INTENTIONALLY 8-bits wide
  logic [8:0]   CTR; //color table read

`ifdef SIMULATION
  logic [8:0]   CRAM[511:0];
`else
  logic [511:0][8:0] CRAM;
`endif
  
  // ALL of our color data
`ifdef FAKE_CRAM
  logic [15:0]  FAKE_CRAM[511:0];
`endif

  logic clk5_en, clk7_en, clk10_en;
  
  clock_divider #(4) clk5(.clk(clock), .reset(~reset_N), .clk_en(clk5_en));
  clock_divider #(3) clk7(.clk(clock), .reset(~reset_N), .clk_en(clk7_en));
  clock_divider #(2) clk10(.clk(clock), .reset(~reset_N), .clk_en(clk10_en));
  
  logic [1:0] mode;
  
  assign clock_en = (mode[1]) ? clk10_en :
                    (mode[0]) ? clk7_en  : clk5_en;
  
  assign mode  = CR[1:0];
  
  logic [8:0] CDATA, CBUF, addr, next_addr;

  logic [7:0] data_rd;

  assign D  = (~RD_n & ~CS_n) ? data_rd : 8'bz;
  assign data_rd  = (A == 4) ? CBUF[7:0] :
                    (A == 5) ? CBUF[8]   : 8'hFF;

  assign CDATA  = CRAM[VD];

  logic read, write, prev_RD_n, prev_WR_n;

  //color read buffer handling, SORT OF A HACK
  //ASSUMES WE HOLD FOR AT LEAST 1 PHYSICAL CYCLE
  //This is always true for us, but makes me (fseidel) sad
  //Easy fix is to mux between CRAM and CBUF after first cycle
  
  always_ff @(posedge clock, negedge reset_N) begin
    if(read)
      CBUF <= CRAM[CTA];
  end

  //read/write edge detection
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      prev_RD_n <= 1;
      prev_WR_n <= 1;
    end
    else if(clk7_en) begin //runs at MMIO clock
      prev_RD_n <= RD_n | CS_n;
      prev_WR_n <= WR_n | CS_n;
    end
  end

  assign read  = (~RD_n & prev_RD_n);
  assign write = (~WR_n & prev_WR_n);
  
  //MMIO
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      CR  <= 2'b00;
      CTA <= 9'h000;
      CTW <= 16'h0000;
`ifdef FAKE_CRAM   //load CRAM image
      $readmemh("PAL.hex", FAKE_CRAM);
      for(int i = 0; i < 512; i++) begin
        CRAM[i] <= FAKE_CRAM[i][8:0];
      end
`endif
    end
    else if(clk7_en & ~CS_n) begin //MMIO is always at CPU speed
      if(read) begin
        if(A == 5)
          CTA <= CTA + 1;
      end
      else if(write) begin
        case(A)
          0:
            CR <= D;
          2:
            CTA[7:0] <= D;
          3:
            CTA[8] <= D[0];
          4:
            CTW <= D;
          5: begin
            CRAM[CTA] <= {D[0], CTW};
            CTA <= CTA + 1;
          end
        endcase
      end
    end
  end

  //pixel fetch
  //TODO: emulate access artifact (makes this logic smaller/faster!)
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      VIDEO_B <= 0;
      VIDEO_R <= 0;
      VIDEO_G <= 0;
    end
    else if(clock_en)
      if(^VD === 1'bx) begin //make TVEmu happy during reset
        VIDEO_B <= 0;
        VIDEO_R <= 0;
        VIDEO_G <= 0;
      end
      else begin
        VIDEO_B <= CDATA[2:0];
        VIDEO_R <= CDATA[5:3];
        VIDEO_G <= CDATA[8:6];
      end
  end


endmodule: vce_HuC6260;


