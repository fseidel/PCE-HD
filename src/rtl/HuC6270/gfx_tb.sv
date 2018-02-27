`default_nettype none
`include "VDCDefines.vh"

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

  string filename = "log.txt";
  int f, do_write, frame_count, log_enabled, silent;
  
  assign log_enabled  = 1;
  assign silent = 1;

  initial begin
    cycle              = 0;
    frame_count        = 0;
    if(log_enabled) f  = $fopen(filename, "w");
    if(!silent) begin
      $monitor("cycle: %d, MA: %x, VD: %x, RGB: (%x %x %x), V_state: %s",
               cycle, vdc.MA, VD, R, G, B, vdc.V_state);
    end
    do_write     = 0;
    clock        = 0;
    reset_N      = 1'b0;
    #10 reset_N <= 1'b1;
    do_write    <= 1'b1;
    while(frame_count < 3) begin
      while(!(vdc.V_state == V_END && vdc.V_cnt == 0 && vdc.EOL)) #10 continue;
      frame_count++;
      while(vdc.V_state == V_END && vdc.V_cnt == 0 && vdc.EOL) #10 continue;
    end
    if(log_enabled) begin
      do_write <= 1'b0;
      $fclose(f);
      $system({"gzip ", filename});
    end
    $finish;
  end

  initial begin
    forever begin
      #10 clock  = ~clock;
      cycle++;
      if(log_enabled && do_write)
        $fwrite(f, "%3d %3d %3d %b %b\n", R, G, B, HSYNC_n, VSYNC_n);
    end
  end

  
endmodule: gfx_tb
