`default_nettype none

module gfx_tb;
  logic clock, reset_N;
  logic [8:0] VD;

  logic       HSYNC_n, VSYNC_n;

  logic [7:0] VIDEO_R, VIDEO_G, VIDEO_B;
  
  vdc_HuC6270 vdc(.clock(clock), .reset_N(reset_N),
                  .DI(), .MRD_n(), .MWR_n(), .HSYNC_n(HSYNC_n), .VSYNC_n(VSYNC_n),
                  .CS_n(), .WR_n(), .RD_n(), .EX_8_16(), .A(), .VD(VD), .DO(),
                  .BUSY_n(), .IRQ_n());
  
  vce_HuC6260 vce(.clock(clock), .reset_N(reset_N),
                  .VD(VD), .HSYN(HSYNC_n), .VSYN(VSYNC_n),
                  .A(), .D(),
                  .VIDEO_G(VIDEO_G), .VIDEO_R(VIDEO_R), .VIDEO_B(VIDEO_B),
                  .CK(), .RD_n(), .WR_n(), .CS_n(),
                  .address_mode());


  initial begin
    $monitor("MA: %x, VD: %x", vdc.MA, VD);
    clock        = 0;
    reset_N      = 1'b0;
    #10 reset_N <= 1'b1;
    #50000 $finish;
  end

  initial begin
    forever #10 clock = ~clock;
  end
  
endmodule: gfx_tb
