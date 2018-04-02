`default_nettype none

/*
 * verilog model of HuC6260 VCE
 *
 * (C) 2018 Ford Seidel and Amolak Nagi
 */

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
  
  logic clk5_en, clk7_en, clk10_en;
  
  clock_divider #(4) clk5(.clk(clock), .reset(~reset_N), .clk_en(clk5_en));
  clock_divider #(3) clk7(.clk(clock), .reset(~reset_N), .clk_en(clk7_en));
  clock_divider #(2) clk10(.clk(clock), .reset(~reset_N), .clk_en(clk10_en));
  
  logic [1:0] mode;
  
  assign clock_en = (mode[1]) ? clk10_en :
                    (mode[0]) ? clk7_en  : clk5_en;
  
  assign mode  = CR[1:0];
  
  logic [7:0] data_rd;

  logic       CPU_we;
  logic [8:0] MA1, MD1_in, VCE_out, CPU_out;

  logic       read, write, prev_RD_n, prev_WR_n;

  //read/write edge detection
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      prev_RD_n <= 1;
      prev_WR_n <= 1;
    end
    else begin
      prev_RD_n <= RD_n | CS_n;
      prev_WR_n <= WR_n | CS_n;
    end
  end

  assign read  = (~RD_n & ~CS_n & prev_RD_n); 
  assign write = (~WR_n & ~CS_n & prev_WR_n);

  logic [8:0] CTA_latched;
  assign MA1 = (read | write) ? CTA : CTA_latched;
  
  assign CPU_we = (~WR_n & ~CS_n & (A == 5));
  CRAM cram(.clock, .reset_N,
            .MA0(VD), .MD0_out(VCE_out),
            .MA1,.MD1_out(CPU_out), .MD1_in({D[0], CTW}), 
            .WE1(CPU_we));

  
  assign D  = (~RD_n & ~CS_n) ? data_rd : 8'bz;
  assign data_rd  = (A == 4) ? CPU_out[7:0] :
                    (A == 5) ? CPU_out[8]   : 8'hFF;

  
  //MMIO
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      CR  <= 2'b00;
      CTA <= 9'h000;
      CTA_latched <= 9'h000;
      CTW <= 8'h00;
    end
    else if(read) begin
      if(A == 4) begin
        CTA_latched <= CTA;
      end
      else if(A == 5) begin
        CTA         <= CTA + 1;
        CTA_latched <= CTA;
      end
    end
    else if(write) begin
      case(A)
        0:
          CR <= D;
        2:
          CTA[7:0] <= D;
        3:
          CTA[8] <= D[0];
        4: begin
          CTW         <= D;
          CTA_latched <= CTA;
        end
        5: begin
          CTA         <= CTA + 1;
          CTA_latched <= CTA;
        end
      endcase
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
    else if(clock_en) begin
      /*
      if(^VD === 1'bx) begin //make TVEmu happy before VRAM init
        VIDEO_B <= 0;
        VIDEO_R <= 0;
        VIDEO_G <= 0;
      end*/
      //else begin
        VIDEO_B <= VCE_out[2:0];
        VIDEO_R <= VCE_out[5:3];
        VIDEO_G <= VCE_out[8:6];
      //end
    end
  end


endmodule: vce_HuC6260


