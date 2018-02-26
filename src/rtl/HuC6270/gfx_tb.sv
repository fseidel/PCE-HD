`default_nettype none

module gfx_tb;
  logic clock, reset_N;
  logic [8:0] VD;
  wire  [8:0] D;

  logic       HSYNC_n, VSYNC_n;

  logic [2:0] VIDEO_R, VIDEO_G, VIDEO_B;
  logic [7:0] R, G, B;

  int         cycle;

  assign R  = VIDEO_R<<5;
  assign G  = VIDEO_G<<5;
  assign B  = VIDEO_B<<5;
  
    
  vdc_HuC6270 vdc(.clock(clock), .reset_N(reset_N),
                  .DI(), .MRD_n(), .MWR_n(),
                  .HSYNC_n(HSYNC_n), .VSYNC_n(VSYNC_n),
                  .CS_n(), .WR_n(), .RD_n(), .EX_8_16(), .A(), .VD(VD), .DO(),
                  .BUSY_n(), .IRQ_n());
  
  vce_HuC6260 vce(.clock(clock), .reset_N(reset_N),
                  .VD(VD), .HSYN(HSYNC_n), .VSYN(VSYNC_n),
                  .A(), .D(D),
                  .VIDEO_G(VIDEO_G), .VIDEO_R(VIDEO_R), .VIDEO_B(VIDEO_B),
                  .CK(), .RD_n(), .WR_n(), .CS_n(),
                  .address_mode());

  int f, do_write;
  initial begin
    cycle = 0;
    f                = $fopen("log.txt", "w");
    $monitor("cycle: %d, MA: %x, VD: %x, RGB: (%x %x %x)",
             cycle, vdc.MA, VD, R, G, B);
    do_write         = 0;
    clock            = 0;
    reset_N          = 1'b0;
    #10 reset_N     <= 1'b1;
    do_write        <= 1'b1;
    #50000 do_write <= 1'b0;
    $fclose(f);
    $finish;
  end

  initial begin
    forever begin
      #10 clock  = ~clock;
      cycle++;
      if(do_write)
        $fwrite(f, "%d %d %d 0 0\n", R, G, B);
    end
  end

  
endmodule: gfx_tb
