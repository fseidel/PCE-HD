`default_nettype none
`include "VDCDefines.vh"

module fullsys_tb;
  logic clock, reset_N, clock_en;
  logic [8:0] VD;
  wire  [7:0] IO_data;

  logic       HSYNC_n, VSYNC_n;

  logic [2:0] VIDEO_R, VIDEO_G, VIDEO_B;
  logic [7:0] R, G, B;

  int         cycle;

  assign R  = VIDEO_R<<5;
  assign G  = VIDEO_G<<5;
  assign B  = VIDEO_B<<5;



  reg clk, reset;
  assign clk  = clock;
  assign reset  = ~reset_N;
  
  wire [20:0] AB_21;
  wire [7:0]  DO, EXT_out;
  wire        RE, WE, IRQ1_n, IRQ2_n, NMI, HSM, RDY_n,
              CE_n, CER_n, CE7_n, CEK_n;


  wire        RD_n, WR_n;
  wire        BUSY_n;
  assign RD_n  = ~RE;
  assign WR_n  = ~WE;
 
  assign RDY_n = ~BUSY_n;
  //assign RDY_n = 1'b1;
  

  assign IRQ2_n = 1'b1;
  assign NMI  = 1'b0;

  assign IO_data = (RE) ? 8'hz : DO; //IO data bus

  assign EXT_out = IO_data;

  
  cpu_HuC6280 CPU(.*);
  /*
  memory mem(.clk, .re(RE), .we(WE), .addr(AB_21), .dIn(DO), .dOut(IO_data),
             .CE_n, .CER_n);
   */
  logic [19:0] SRAM_ADDR;
  logic [15:0] SRAM_DQ;
  ROM rom(.addr(AB_21[19:0]), .D(IO_data), .RD_n(RD_n), .CE_n(CE_n),
	  .SRAM_ADDR(SRAM_ADDR), .SRAM_DQ(SRAM_DQ));
  
    
  vdc_HuC6270 vdc(.clock(clock), .reset_N(reset_N), .clock_en(clock_en),
                  .D(IO_data), .MRD_n(), .MWR_n(),
                  .HSYNC_n(HSYNC_n), .VSYNC_n(VSYNC_n),
                  .CS_n(CE7_n),
                  .WR_n(WR_n), .RD_n(RD_n), .EX_8_16(), .A(AB_21[1:0]), 
                  .VD(VD),
                  .BUSY_n(BUSY_n), .IRQ_n(IRQ1_n));
  
  vce_HuC6260 vce(.clock(clock), .reset_N(reset_N),
                  .VD(VD), .HSYN(HSYNC_n), .VSYN(VSYNC_n),
                  .A(AB_21[2:0]), .D(IO_data),
                  .VIDEO_G(VIDEO_G), .VIDEO_R(VIDEO_R), .VIDEO_B(VIDEO_B),
                  .clock_en(clock_en), .RD_n(RD_n), .WR_n(WR_n), .CS_n(CEK_n),
                  .address_mode());

  string filename = "log.txt";
  int f, do_write, frame_count, log_enabled, silent, pclog, pclog_enabled;
  
  assign log_enabled  = 1;
  assign silent  = 1;
  assign pclog_enabled = 0;

  initial begin
    cycle              = 0;
    frame_count        = 0;
    //force IO_data      = DO;
    if(log_enabled) f  = $fopen(filename, "w");
    if(pclog_enabled) pclog  = $fopen("pclog.txt", "w");
    if(!silent) begin
      $monitor("cycle: %d, MA: %x, VD: %x, RGB: (%x %x %x), V_state: %s",
               cycle, vdc.MA, VD, R, G, B, vdc.V_state);
    end
    do_write     = 0;
    clock        = 0;
    reset_N      = 1'b0;
    #100 reset_N <= 1'b1;
    do_write    <= 1'b1;
    while(frame_count < 600) #10 continue; //352
    //while(VIDEO_R == 0 && VIDEO_G == 0 && VIDEO_B == 0) #10 continue;
      //force vdc.BXR  = 4*frame_count;
      //force vdc.BYR  = 4*frame_count;
      //while(vdc.V_state == V_END && vdc.V_cnt == 0 && vdc.EOL) #10 continue;
    //end
    if(log_enabled) begin
      do_write <= 1'b0;
      $fclose(f);
      $system({"gzip -f ", filename});
    end
    if(pclog_enabled) begin
      $fclose(pclog);
    end
    $strobe("A: %x, X: %x, Y: %x", CPU.A, CPU.X, CPU.Y);
    #1 $finish;
  end
  
  initial begin
    forever begin
      #10 clock  = ~clock;
      cycle++;
      if(CPU.state == 12 && CPU.RDY && clock) //DECODE
        $fwrite(pclog, "%04X\n", CPU.PC-1);
      if(frame_count >= 250)
	    force CPU.PAD_out = 8'b1111_0111; // Region bit == Japan*/
      if(vdc.V_state == V_END && vdc.V_cnt == 0 &&
         vdc.EOL && clock_en && clock) begin
        frame_count++;
        $display("frame %d", frame_count);
      end
      if(log_enabled && do_write && clock_en && (frame_count >= 500)) //500
        $fwrite(f, "%3d %3d %3d %b %b\n", R, G, B, HSYNC_n, VSYNC_n);
    end
  end

  
endmodule: fullsys_tb
