`default_nettype none
`include "VDCDefines.vh"

module gfx_tb;
  logic clock, reset_N, clock_en;
  logic [8:0] VD;
  wire  [8:0] D;

  logic       HSYNC_n, VSYNC_n;

  logic [2:0] VIDEO_R, VIDEO_G, VIDEO_B;
  logic [7:0] R, G, B;

  int         cycle;

  assign R  = VIDEO_R<<5;
  assign G  = VIDEO_G<<5;
  assign B  = VIDEO_B<<5;
  
    
  vdc_HuC6270 vdc(.clock(clock), .reset_N(reset_N), .clock_en(clock_en),
                  .DI(), .MRD_n(), .MWR_n(),
                  .HSYNC_n(HSYNC_n), .VSYNC_n(VSYNC_n),
                  .CS_n(), .WR_n(), .RD_n(), .EX_8_16(), .A(), .VD(VD), .DO(),
                  .BUSY_n(), .IRQ_n());
  
  vce_HuC6260 vce(.clock(clock), .reset_N(reset_N),
                  .VD(VD), .HSYN(HSYNC_n), .VSYN(VSYNC_n),
                  .A(), .D(D),
                  .VIDEO_G(VIDEO_G), .VIDEO_R(VIDEO_R), .VIDEO_B(VIDEO_B),
                  .clock_en(clock_en), .RD_n(), .WR_n(), .CS_n(),
                  .address_mode());

  string filename = "log.txt";
  int f, do_write, frame_count, log_enabled, silent;
  
  assign log_enabled  = 1;
  assign silent = 1;

  initial begin
    cycle              = 0;
    frame_count        = 0;
    if(log_enabled) f  = $fopen(filename, "w");
    //$monitor("x: %d y: %d y:%b", vdc.block_x_idx, vdc.block_y_idx, vdc.first_sprite.CGY);
    if(!silent) begin
      $monitor("cycle: %d, MA: %x, VD: %x, RGB: (%x %x %x), V_state: %s",
               cycle, vdc.MA, VD, R, G, B, vdc.V_state);
    end
    do_write     = 0;
    clock        = 0;
    reset_N      = 1'b0;
    #30 reset_N <= 1'b1;
    do_write    <= 1'b1;
    while(frame_count < 3) begin
      while(!(vdc.V_state == V_END && vdc.V_cnt == 0 && vdc.EOL)) begin
        #20;
//        $display("%d, H_state: %s, H_cnt: %d, V_state: %s, V_cnt: %d", frame_count, vdc.H_state, vdc.H_cnt, vdc.V_state, vdc.V_cnt);
        if (vdc.do_Spritecrawl) begin
//          $display("satb_idx: %d, line_sprite_idx: %d", vdc.satb_idx, vdc.line_sprite_idx);
        end	
      end
      frame_count++;
      $display("finish frame");
//      force vdc.BXR  = 4*frame_count;
//      force vdc.BYR  = 4*frame_count;
      while(vdc.V_state == V_END && vdc.V_cnt == 0 && vdc.EOL) #10 continue;
    end
    if(log_enabled) begin
      do_write <= 1'b0;
      $fclose(f);
      $system({"gzip ", filename});
    end

    $display("y_pos: %x", vdc.satb_entries[0].y_pos);
    $display("x_pos: %x", vdc.satb_entries[0].x_pos);
    $display("addr:  %x", vdc.satb_entries[0].addr);
    $display("CGY:   %b", vdc.satb_entries[0].CGY);
    $display("CGX:   %b", vdc.satb_entries[0].CGX);
    $display("y_pos: %x", vdc.satb_entries[1].y_pos);
    $display("x_pos: %x", vdc.satb_entries[1].x_pos);
    $display("addr:  %x", vdc.satb_entries[1].addr);
    $display("CGY:   %b", vdc.satb_entries[1].CGY);
    $display("CGX:   %b", vdc.satb_entries[1].CGX);
/*    $display("word0: %x", vdc.sprite_data[0]);
    $display("word1: %x", vdc.sprite_data[1]);
    $display("word0: %x", vdc.sprite_data[2]);
    $display("word1: %x", vdc.sprite_data[3]);
    $display("word0: %x", vdc.sprite_data[4]);
    $display("word1: %x", vdc.sprite_data[5]);
    $display("word0: %x", vdc.sprite_data[6]);
    $display("word1: %x", vdc.sprite_data[7]);
    $display("word0: %x", vdc.sprite_data[8]);
    $display("word1: %x", vdc.sprite_data[9]);
    $display("word0: %x", vdc.sprite_data[10]);
    $display("word1: %x", vdc.sprite_data[11]); */
    $finish;
  end

  initial begin
    forever begin
      #10 clock  = ~clock;
      cycle++;
      if(log_enabled && do_write && clock_en)
        $fwrite(f, "%3d %3d %3d %b %b\n", R, G, B, HSYNC_n, VSYNC_n);
    end
  end

  
endmodule: gfx_tb
