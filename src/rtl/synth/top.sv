`default_nettype none

`define SIMULATE

/*
 * Top module for synthesis
 */

module ChipInterface(input  logic        CLOCK_50,
                     output logic [7:0]  VGA_R, VGA_G, VGA_B,
                     output logic        VGA_BLANK_N, VGA_CLK, VGA_SYNC_N,
                     output logic        VGA_VS, VGA_HS,
                     output wire [19:0]  SRAM_ADDR,
                     output wire         SRAM_CE_N, SRAM_OE_N, SRAM_WE_N,
                     output wire         SRAM_LB_N, SRAM_UB_N, 
                     inout wire [15:0]   SRAM_DQ,
                     output logic [17:0] LEDR,
                     output logic [8:0]  LEDG,
                     input logic [17:0]  SW,
                     input logic [3:0]   KEY);


  assign SRAM_CE_N  = 1'b0;
  assign SRAM_OE_N  = 1'b0;
  assign SRAM_WE_N  = 1'b1;
  assign SRAM_LB_N  = 1'b0;
  assign SRAM_UB_N  = 1'b0;
  
  logic areset;
  assign areset = ~KEY[0];
  logic reset_N;
  assign reset_N = ~areset;

  wire  CLOCK_21;
  logic pll_locked;

  
  //generate a 21.47727MHz clock from the 50MHz input clock
  clockgen cg(.areset(1'b0), .inclk0(CLOCK_50), .c0(CLOCK_21), 
              .locked(pll_locked));
  
  //assign CLOCK_21 = CLOCK_50;
  //assign pll_locked  = 1'b1;
  
  logic [8:0] VD;
  wire  [7:0] IO_data;

  logic       HSYNC_n, VSYNC_n;
  assign VGA_HS  = HSYNC_n;
  assign VGA_VS  = VSYNC_n;

  logic [7:0] R, G, B;
  assign VGA_R  = R;
  assign VGA_G  = G;
  assign VGA_B  = B;
  assign VGA_SYNC_N = 1'b1;
  assign VGA_BLANK_N = 1'b1;
  assign VGA_CLK = CLOCK_21;
  
  logic [2:0] VIDEO_R, VIDEO_G, VIDEO_B;

  int         cycle;

  assign R  = VIDEO_R<<5;
  assign G  = VIDEO_G<<5;
  assign B  = VIDEO_B<<5;
  
  //reset synchronization for CPU
  reg sreset, sreset0;
  always_ff @(posedge CLOCK_21) begin
    {sreset, sreset0} <= {sreset0, areset};
  end
  
  assign LEDG[0]  = areset;
  assign LEDR[0]  = sreset;
  
  reg clk, reset;
  assign clk  = CLOCK_21;
  assign reset  = sreset;
  
  reg clock_en;
  
  wire [20:0] AB_21;
  wire [7:0]  DO, EXT_out;
  wire        RE, WE, IRQ1_n, IRQ2_n, NMI, HSM, RDY_n,
              CE_n, CER_n, CE7_n, CEK_n;


  wire        RD_n, WR_n;
  wire        BUSY_n;
  assign RD_n  = ~RE;
  assign WR_n  = ~WE;

  //assign BUSY_n = 1;
  assign RDY_n = ~(BUSY_n & pll_locked);
  //assign RDY_n = 1'b1;
  

  assign IRQ2_n = 1'b1;
  assign NMI  = 1'b0;

  assign IO_data = (RE) ? 8'hz : DO; //IO data bus

  assign EXT_out = IO_data;

  
  cpu_HuC6280 CPU(.*);

  ROM rom(.clk(clk), .reset(reset),
      .addr(AB_21[19:0]), .D(IO_data), .RD_n(RD_n), .CE_n(CE_n),
      .SW(SW[2:0]), .SRAM_ADDR(SRAM_ADDR), .SRAM_DQ(SRAM_DQ));  
  
  vdc_HuC6270 vdc(.clock(CLOCK_21), .reset_N(reset_N), .clock_en(clock_en),
                  .D(IO_data), .MRD_n(), .MWR_n(),
                  .HSYNC_n(HSYNC_n), .VSYNC_n(VSYNC_n),
                  .CS_n(CE7_n),
                  .WR_n(WR_n), .RD_n(RD_n), .EX_8_16(), .A(AB_21[1:0]), 
                  .VD(VD),
                  .BUSY_n(BUSY_n), .IRQ_n(IRQ1_n));
  
  vce_HuC6260 vce(.clock(CLOCK_21), .reset_N(reset_N),
                  .VD(VD), .HSYN(HSYNC_n), .VSYN(VSYNC_n),
                  .A(AB_21[2:0]), .D(IO_data),
                  .VIDEO_G(VIDEO_G), .VIDEO_R(VIDEO_R), .VIDEO_B(VIDEO_B),
                  .clock_en(clock_en), .RD_n(RD_n), .WR_n(WR_n), .CS_n(CEK_n),
                  .address_mode(1'b0));
  
  
endmodule: ChipInterface

`ifdef SIMULATE
module simTB();
  logic CLOCK_50, VGA_BLANK_N, VGA_CLK, VGA_SYNC_N, VGA_VS, VGA_HS;
  logic [7:0]  VGA_R, VGA_B, VGA_G;
  logic [19:0] SRAM_ADDR;
  logic        SRAM_CE_N, SRAM_OE_N, SRAM_WE_N, SRAM_LB_N, SRAM_UB_N;
  wire  [15:0] SRAM_DQ;
  logic [17:0] LEDR;
  logic [8:0]  LEDG;
  logic [17:0] SW;
  logic [3:0] KEY;
  
  ChipInterface CI(.CLOCK_50,
                   .VGA_R, .VGA_G, .VGA_B,
                   .VGA_BLANK_N, .VGA_CLK, .VGA_SYNC_N,
                   .VGA_VS, .VGA_HS,
                   .SRAM_ADDR,
                   .SRAM_CE_N, .SRAM_OE_N, .SRAM_WE_N,
                   .SRAM_LB_N, .SRAM_UB_N, 
                   .SRAM_DQ,
                   .LEDR,
                   .LEDG,
                   .SW,
                   .KEY);

  int framecnt;
  initial begin
    framecnt     = 0;
    CLOCK_50     = 0;
    KEY[0]       = 0;
    #100 KEY[0] <= 1;
  end
  
  initial begin
    forever #5 CLOCK_50 = ~CLOCK_50;
  end

  always @(negedge VGA_VS) begin
    framecnt++;
    $display("frame %d", framecnt);
    if(framecnt == 2) $finish;
  end
  
endmodule: simTB
`endif
