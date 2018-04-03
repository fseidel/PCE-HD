`default_nettype none
`include "./VDCDefines.vh"

/*
 * verilog model of HuC6270 VDC
 *
 * (C) 2018 Ford Seidel and Amolak Nagi
 */


module vdc_HuC6270(input logic clock, reset_N, clock_en,     
                   inout wire [7:0]   D, // Data in, only lower 8 bits used
                   input logic        MRD_n, // "Memory Read Data" from CPU 
                                      // from VRAM
                                      MWR_n, // Memory write from CPU to VRAM 
                                      CS_n, //chip select
                                      WR_n, RD_n, //CPU RW enables
                                      EX_8_16, // 1 if 8-bit, 0 if 16. WILL 
                                      // ALWAYS BE SET TO 1
                   output logic       HSYNC_n, VSYNC_n, //only support output
                   input logic [1:0]  A, // Address bus indicator
                   output logic [8:0] VD,
                   output logic [7:0] DO,
                   output logic       BUSY_n, IRQ_n
                  );	

/*
  ControlUnit cu(.clock(clock),
                 .reset_N(reset_N),
                 .RD_n(RD_n),
                 .WR_n(WR_n),
                 .CS_n(CS_n),
                 .DI(DI),
                 .DO(DO),
                 .A(A),
                 .BUSY_n(BUSY_n),
                 .IRQ_n(IRQ_n)
                );

*/

  // VRAM Initialization

  // VRAM address and data bus wires
  logic [15:0] MA, MB;
  logic [15:0] MD_in, MD_out, MD_in_buf, latched_read;
  logic [15:0] CPU_in, CPU_out;
  logic vram_re, vram_we, CPU_we;

  assign vram_re  = ~vram_we;


  VRAM vram(.clock(clock),  //fseidel: This interface needs some work
            .reset_N(reset_N),
            .MA(MA), .MB(MB),
            .re(vram_re),
            .we_a(vram_we), .we_b(CPU_we),
            //.MD_out(MD_in_buf),
            .MD_out(MD_in),
	    .CPU_out(CPU_in),
            .MD_in(MD_out),
	    .CPU_in(CPU_out));
 

  // VDC Registers

  //latch H*R in horizontal blanking, V*R in vertical blanking
  BXR_t BXR; // $07
  BYR_t BYR; // $08
  MWR_t MWR; // $09 
  HSR_t HSR; // $0A
  HDR_t HDR; // $0B
  VSR_t VSR; // $0C
  VDR_t VDR; // $0D
  VCR_t VCR; // $0E
  SATB_t SATB; // $13
  //assign SATB.data = 16'h0800; // TODO: SET THIS

  //test values from parasol stars
  assign HSR = 16'h0202; // $0A
  assign HDR = 16'h031F; // $0B
  assign VSR = 16'h0F02; // $0C
  assign VDR = 16'h00EF; // $0D
  assign VCR = 16'h0003; // $0E

  //despite what archaic pixels says, the patent (and my math) indicate that
  //HSW needs a "- 1"
  logic [4:0] HSW; // Horizontal synchronous pulse width - 1
  logic [6:0] HDS; // Horizontal display start position - 1
  logic [6:0] HDW; // Horizontal display width in tiles - 1
  logic [3:0] HDE; // Horizontal display ending period - 1

  
  assign HSW  = HSR.HSW;
  assign HDS  = HSR.HDS;
  assign HDW  = HDR.HDW;
  assign HDE  = HDR.HDE;

  //ditto for VSW
  logic [4:0] VSW; // Vertical synchronous pulse width - 1
  logic [7:0] VDS; // Vertical display start position - 2
  logic [8:0] VDW; // Vertical display width in pixels - 1
  logic [7:0] VDE; // Vertical display end position
  //VCR is passed through

  assign VSW  = VSR.VSW;
  assign VDS  = VSR.VDS;
  assign VDW  = VDR.VDW;
  assign VDE  = VCR.VDE;
  //VCR is passed through



  logic [2:0]  char_cycle; //current position in char cycle


  logic [9:0] H_cnt; //horizontal position counter, reused for each phase
  logic [9:0] V_cnt; //vertical position counter, reused for each phase

  h_state_t H_state; //state in horizontal line
  v_state_t V_state; //state in vertical screen

  logic [9:0]  x_start, y_start, cur_row; //BG start values per line
  logic [7:0]  x_mask, y_mask, y_shift;
  
  logic [2:0]  x_px_offset, y_px_offset;
  logic [6:0]  x_tl_offset, y_tl_offset;

  logic [1:0]  x_shift;
  assign y_shift = (MWR[5]) ? 7 :
                   (MWR[4]) ? 6 : 5;

  
  //assign y_shift = (MWR[6]) ? 6 : 5; //yet another hack
  assign x_mask = (1 << y_shift) - 1;
  assign y_mask = (1 << (MWR[6] + 8)) - 1; //ditto
  assign x_px_offset  = x_start[2:0];
  assign x_tl_offset  = x_start[9:3];
  assign y_px_offset  = y_start[2:0];
  assign y_tl_offset  = y_start[9:3];
  

  logic [15:0] MAWR, MARR;
  logic [9:0]  RCR;
  CR_t  CR;

  logic [7:0]  addr_incr;
  assign addr_incr  = (CR.addr_incr == ADDR_INCR_1)   ? 8'd01 :
                      (CR.addr_incr == ADDR_INCR_32)  ? 8'd32 :
                      (CR.addr_incr == ADDR_INCR_64)  ? 8'd64 :
                                                        8'd128;
  
  //high when CPU is issuing a read or write request
  logic VRAM_read_req;
  logic VRAM_write_req;
  
  /*
   * MMIO
   * There are a LOT of registers
   * TODO: no DMA controls yet (including SATB!)
   */
  logic [7:0]  D_out;
  assign D = (~RD_n & ~CS_n) ? D_out : 8'bz;

  logic [7:0]  status;
  
  reg_sel_t VDC_regnum;
  
  logic        do_BGfetch;   

  //make I/O signals active high for convention
  logic        RD, WR, CS;
  assign {RD, WR} = {(~RD_n & ~CS_n), (~WR_n & ~CS_n)};
  //read/write edge detection
  logic prev_RD, prev_WR;
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      prev_RD <= 0;
      prev_WR <= 0;
    end
    else if(clock_en) begin
      prev_RD <= RD;
      prev_WR <= WR;
    end
  end
  


  reg_sel_t D_regsel;
  always_comb begin
    D_regsel  = REG_MAWR;
    case(D[4:0])
      5'd0:  D_regsel = REG_MAWR;
      5'd1:  D_regsel = REG_MARR;
      5'd2:  D_regsel = REG_VRR_VWR;
      5'd5:  D_regsel = REG_CR;
      5'd6:  D_regsel = REG_RCR;
      5'd7:  D_regsel = REG_BXR;
      5'd8:  D_regsel = REG_BYR;
      5'd9:  D_regsel = REG_MWR;
      5'd10:  D_regsel = REG_HSR;
      5'd11:  D_regsel = REG_HDR;
      5'd12:  D_regsel = REG_VSR;
      5'd13:  D_regsel = REG_VDR;
      5'd14:  D_regsel = REG_VCR;
      5'd15:  D_regsel = REG_DCR;
      5'd16:  D_regsel = REG_SOUR;
      5'd17:  D_regsel = REG_DESR;
      5'd18:  D_regsel = REG_LENR;
      5'd19:  D_regsel = REG_SATB;
    endcase
  end
  
  logic vram_write_pending;
  logic MARR_increment;
  logic CPU_read_issue, CPU_write_issue;
  logic [7:0] CPU_write_buf;
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      VDC_regnum         <= REG_MAWR; //makes BUSY_n behave on reset
      vram_write_pending <= 0;
      RCR                <= 0;
      CR                 <= 0;
      MAWR               <= 0;
      MARR               <= 0;
      BXR                <= 0;
      BYR                <= 0;
      MWR                <= 16'h10; //Gunhed HACK
      SATB.data          <= 16'h800;//value for Gunhed and The Kung Fu
    end
    else begin
      if(clock_en) begin
        if(CPU_read_issue) begin
          MARR <= MARR + addr_incr; //256-byte boundary may be odd, check this!
        end
        if(CPU_write_issue) begin
          MAWR <= MAWR + addr_incr;
        end
        if(WR & ~prev_WR)
          case(A)
            A_STATUS_ADDR_REG:
              VDC_regnum <= D_regsel;
            A_DATA_LSB:
              case(VDC_regnum)
                REG_MAWR:
                  MAWR[7:0] <= D;
                REG_MARR:
                  MARR[7:0] <= D;
                REG_VRR_VWR:
                  CPU_write_buf <= D;
                REG_CR:
                  CR[7:0] <= D;
                REG_RCR:
                  RCR[7:0] <= D;
                REG_BXR:
                  BXR[7:0] <= D;
                REG_BYR:
                  BYR[7:0] <= D;
                REG_MWR:
                  MWR[7:0] <= D;
                REG_SATB:
                  SATB.data[7:0] <= D;
                /*
                 REG_HSR:
                 HSR[7:0] <= D;
                 REG_HDR:
                 HDR[7:0] <= D;
                 REG_VPR:
                 VPR[7:0] <= D;
                 REG_VDW:
                 VDW[7:0] <= D;
                 REG_VCR:
                 VCR[7:0] <= D;
                 REG_DCR:
                 DCR[7:0] <= D;
                 */
              endcase
            A_DATA_MSB:
              case(VDC_regnum)
                REG_MAWR:
                  MAWR[15:8] <= D;
                REG_MARR:
                  MARR[15:8] <= D;
                REG_VRR_VWR:;
                REG_CR:
                  CR[12:8] <= D[4:0];
                REG_RCR:
                  RCR[9:8] <= D[1:0];
                REG_BXR:
                  BXR[9:8] <= D[1:0];
                REG_BYR:
                  BYR[8]   <= D[0];
                REG_MWR:; //only 8 bits wide
                REG_SATB:
                  SATB.data[15:8] <= D;
              endcase
          endcase
      end
    end
  end

  //read logic


  //IRQ acknowledge + read invalidation
  //TODO:possibly lose an IRQ if it's delivered while ACKing another. Fix this.
  //Maybe not???
  logic ACK;
  always_comb begin
    ACK = 0;
    if(RD & ~prev_RD & (A == 0))
      ACK  = 1;
  end


  localparam num_sprites = 64;
  logic first_frame_done; 

  enum logic [1:0] {IDLE, READ, WRITE, WAIT}
       mem_state, mem_nextstate;
  
  assign VRAM_read_req = (A == 3) &
                         ((RD & (VDC_regnum == REG_VRR_VWR)) | 
                          (WR & (VDC_regnum == REG_MARR)));
  
  
  assign VRAM_write_req = WR & (A == 3) & (VDC_regnum == REG_VRR_VWR); 

  logic do_SATfetch;
  assign do_SATfetch = (V_state == V_SYNC && V_cnt < 4);
  
  //TODO: MARR update can affect BUSY_n
  //TODO: VBLANK access/BURST mode
  logic CPU_access_ok;
  logic do_Spritefetch;
  assign CPU_access_ok = ~do_Spritefetch;


  logic mem_wait;
  assign mem_wait = 1'b0; //hehehehe
  
  assign CPU_we  = (WR & ~prev_WR) && VDC_regnum == REG_VRR_VWR && A == 3;
  
  assign CPU_write_issue  = CPU_we;

  assign MB  = (CPU_write_issue) ? MAWR : MARR;

  assign CPU_out  = {D, CPU_write_buf[7:0]};

  always_comb begin
    CPU_read_issue = 1'b0;
    if((WR & ~prev_WR) && VDC_regnum  == REG_MARR && A == 3) //MARR write
      CPU_read_issue  = 1'b1;
    else if((RD & ~prev_RD) && VDC_regnum == REG_VRR_VWR && A == 3)
      CPU_read_issue  = 1'b1;
  end

  
  /*
  logic new_read_req, new_write_req, prev_read_req, prev_write_req,
        cur_read_req, cur_write_req;
  
  assign new_read_req  = (VRAM_read_req & ~prev_read_req) | cur_read_req;
  assign new_write_req  = (VRAM_write_req & ~prev_write_req) | cur_write_req;
  
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      prev_read_req  <= 0;
      prev_write_req <= 0;
      cur_read_req   <= 0;
      cur_write_req  <= 0;
    end
    else begin
      prev_read_req  <= VRAM_read_req;
      prev_write_req <= VRAM_write_req;
      if(VRAM_read_req & ~prev_read_req)
        cur_read_req <= 1;
      if(VRAM_write_req & ~prev_write_req)
        cur_write_req <= 1;
      if(CPU_read_issue)
        cur_read_req <= 0;
      if(CPU_write_issue)
        cur_write_req <= 0;
    end
  end
  
  //state, outputs
  //assumes a read won't start during an update. This shouldn't happen as long 
  //as long as busy works correctly.

  
/* 
   * READ occurs when the CPU reads the upper byte of the VRR
   * UPDATE occurs when the CPU modifies the upper byte of the MARR
   * An interesting quirk of the VDC is that modifying the lower byte of the
   * MARR does not trigger a memory fetch. The upper byte, however, will trigger
   * a memory fetch and MARR increment.
   */
  /*
  always_comb begin
    mem_nextstate    = IDLE;
    CPU_read_issue   = 0;
    CPU_write_issue  = 0;
    mem_wait         = 0;
    case(mem_state)
      IDLE: begin
	if(new_read_req) begin
          mem_nextstate  = READ;
	  mem_wait 	 = 1;
	end
    else if(new_write_req) begin
	  mem_nextstate  = WRITE;
	  mem_wait 	 = 1;
	end
      end
      READ: begin
	    mem_nextstate   = (CPU_access_ok) ? IDLE : READ;
        mem_wait        = ~CPU_access_ok;
        CPU_read_issue  = CPU_access_ok;
      end
      WRITE: begin
        mem_nextstate    = (CPU_access_ok) ? IDLE : WRITE;
        mem_wait         = ~CPU_access_ok;
        CPU_write_issue  = CPU_access_ok;
      end
      WAIT:
         if(VRAM_read_req | VRAM_write_req)
           mem_nextstate  = WAIT;
    endcase
  end
   
*/
  logic [15:0] CPU_maddr; //goes to address generator
  //assign CPU_maddr = (vram_we) ? MAWR : MARR;



  assign MD_out = CPU_write_buf; //only matters 1 cycle after clock_en

  assign CPU_maddr = (CPU_write_issue) ? MAWR : MARR;
  
  logic [15:0] VRAM_readbuf;
  //next state logic. Turns out that doing this 240-style is really clean
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      //mem_state <= IDLE;
      VRAM_readbuf <= 16'h0000;
    end
    else if(clock_en) begin
      //mem_state <= mem_nextstate;
      if(CPU_read_issue)
        VRAM_readbuf <= CPU_in;
    end
  end
  
  logic [15:0] VRAM_output; //TODO: is this okay?
  //assign VRAM_output = (char_cycle[0]) ? MD_in : VRAM_readbuf;
  assign VRAM_output = VRAM_readbuf;

  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      D_out <= 0;
    end
    else if(RD & ~prev_RD & clock_en) begin
      if(A == 0)
        D_out <= status;
      else if(A == 1)
        D_out <= 0; //reads back 0, writes ignored
      else if(VDC_regnum == REG_VRR_VWR) begin
        if(~A[0]) //A == 2
          D_out <= VRAM_readbuf[7:0];
        else //A == 3
          D_out <= VRAM_readbuf[15:8];
      end
    end
  end
  
  //hang CPU if it tries to touch VRAM outside of when it's allowed to
  //TODO: burst mode
  always_comb begin
    BUSY_n  = ~mem_wait;
    /*
    if(~CS_n & (A == 3) & (((~RD_n | ~WR_n) & (VDC_regnum == REG_VRR_VWR)) | 
                           (~WR_n & (VDC_regnum == REG_MARR)))) begin
      if(V_state != V_DISP || H_state != H_DISP)
        BUSY_n = 0;
     end
     */
  end


  //assign vram_we = CPU_write_issue;
  assign vram_we = 1'b0;
  
  /*
   * Horizontal Syncronization
   */         
  assign do_BGfetch = ((H_state == H_DISP) || 
                      (H_state == H_WAIT && H_cnt < 2)) && 
                      (V_state == V_DISP);

  // SAT Fetch
  logic [63:0]  satb_entries[num_sprites];
  logic [$clog2(num_sprites) - 1 : 0] SATfetch_cur_entry;


  // Sprite Crawl
  logic do_Spritecrawl;
  assign do_Spritecrawl = (V_state == V_WAIT && V_cnt == 0 &&
                           H_state == H_DISP) ||
                          (V_state == V_DISP && V_cnt != 0 
                           && H_state == H_DISP);

  //logic do_Spritefetch; //fseidel:had to move this up
  assign do_Spritefetch = (V_state == V_WAIT && V_cnt == 0 && H_state == H_END) 
                          || (V_state == V_DISP && V_cnt != 0 &&
                              (H_state == H_SYNC ||
                               (H_state == H_WAIT && H_cnt  >= 2) ||
                               H_state == H_END)) ||
                          (V_state == V_DISP && V_cnt == 0 &&
                           (H_state == H_SYNC ||
                            (H_state == H_WAIT && H_cnt >= 2)));



  logic EOL; //signal for end of line
  assign EOL = (char_cycle == 7) && (H_state == H_END) && (H_cnt == 0);
  
  assign HSYNC_n = ~(H_state == H_SYNC);


  // Next state logic for horizontal state (h_state)
  always_ff @(posedge clock, negedge reset_N) begin

    if(~reset_N) begin
      H_cnt   <= HSW;    // Reset to width of HSYNC (will count down)
      H_state <= H_SYNC; // Start off in HSYNC 
      x_start <= 0;      
      y_start <= 0;
    end
    else if(clock_en) begin
      
      if(char_cycle == 6 && H_state == H_SYNC) begin
        x_start <= BXR; //TODO: HACK! latch at end of DISP???
        y_start <= BYR;
      end
      if(char_cycle == 7) begin
        // Always decrement our H_cnt at the end of a char_cycle
        H_cnt <= H_cnt - 1;
        case(H_state)
          // If we're at the end of HSYNC, start the WAIT state
          H_SYNC:
            if(H_cnt == 0) begin
              H_cnt   <= HDS;   // Last for the start position width
              H_state <= H_WAIT;
            end

          // If we're at the end of the WAIT state, start displaying
          H_WAIT:
            if(H_cnt == 0) begin
              H_cnt   <= HDW;   // Last for the length of the horiz disp
              H_state <= H_DISP;
            end

          H_DISP:
            if(H_cnt == 0) begin
              H_cnt   <= HDE;  // Last for the length of the end period
              H_state <= H_END;
            end

          H_END:
            if(H_cnt == 0) begin
              H_cnt   <= HSW; // Last for the width of HSYNC
              H_state <= H_SYNC;
            end
        endcase
      end
    end
  end



  // Vertical state control

  logic frame_reset; //set below

  always_ff @(posedge clock, negedge reset_N) begin

    // Reset to WAIT state, last for length of state.
    if(~reset_N) begin
      V_cnt   <= VDS + 1;
      V_state <= V_WAIT;
      first_frame_done <= 0;
    end
    else if(clock_en) begin
      if(EOL) begin
        // Reset for a frame
        if(frame_reset) begin
          V_cnt <= VDS + 1;
          V_state <= V_WAIT;
          first_frame_done <= 1;
        end
        // Decrement V_cnt at the end of a line
        else begin
          V_cnt <= V_cnt - 1;

          case(V_state)

            // When we're finishing VSYNC (i.e. finising a frame),
            // restart the frame in the WAIT state again
            V_SYNC:
              if(V_cnt == 0) begin
                V_cnt   <= VDS + 1;
                V_state <= V_WAIT;
              end

            // When we finish up the wait state, start on the display state
            V_WAIT: 
              if(V_cnt == 0) begin
                V_cnt   <= VDW;
                V_state <= V_DISP;
              end

            // When we're done displaying, start the overscan at the end state
            V_DISP:
              if(V_cnt == 0) begin
                V_cnt   <= VDE - 1;
                V_state <= V_END;
              end

            // After the end of the END state, start on VSYNC
            V_END:
              if(V_cnt == 0) begin
                V_cnt   <= VSW;
                V_state <= V_SYNC;
              end

          endcase
        end
      end
    end
  end


  // Separate logic to determine VSYNC


   //the 4 vertical display states
  logic        V_top_blank, V_display, V_bot_blank, V_sync;
  logic [8:0]  frame_cnt; //notation from cmacdonald

  localparam NUM_TOTAL_LINES = 263;
  localparam NUM_TOP_BLANK_LINES  = 14;
  localparam NUM_DISPLAY_LINES  = 242;
  localparam NUM_BOT_BLANK_LINES  = 4;
  localparam NUM_SYNC_LINES  = 3;
  
  assign frame_reset  = (frame_cnt == NUM_TOTAL_LINES);
  //Vertical counters
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      frame_cnt <= 0;
    end
    else if(EOL & clock_en) begin
      frame_cnt <= (frame_reset) ? 0 : frame_cnt + 1;
    end
  end

  assign VSYNC_n  = ~(V_sync);
  
  //Vertical state
  always_comb begin
    {V_top_blank, V_display, V_bot_blank, V_sync}  = 0;
    if(frame_cnt < NUM_TOP_BLANK_LINES) V_top_blank = 1; 
    else if(frame_cnt < NUM_TOP_BLANK_LINES + NUM_DISPLAY_LINES) V_display = 1;
    else if(frame_cnt < NUM_TOP_BLANK_LINES + NUM_DISPLAY_LINES + 
            NUM_BOT_BLANK_LINES) V_bot_blank  = 1;
    else V_sync = 1;
  end
 
  // Keep the (x,y) indices in the display mode 
  logic [15:0] x_idx, y_idx;
  assign x_idx = ((HDW - H_cnt) << 3) + char_cycle;
  assign y_idx = VDW - V_cnt;

  ///////////////////////////////////////////////////////////////////////////
  // SPRITE CRAWL FSM
  ///////////////////////////////////////////////////////////////////////////

  // Counter of what sprite we're looking at
  logic [6:0] satb_idx;       

  // How many sprites we've stored so far
  logic [4:0] line_sprite_idx; 

  // Our current SATB entry
  satb_entry_t cur_entry;
  assign cur_entry = satb_entries[satb_idx];

  logic [9:0] next_line, cur_top, cur_bot;

  // Logic to VCE telling it if we're currently in active display 
  logic        in_vdw; 
  assign in_vdw = (H_state == H_DISP && V_state == H_DISP);

  // Look at the next line (plus a 64 line offset as per spec)
  //assign next_line = (in_vdw) ? y_idx + 10'd64 + 10'd1 : 10'd64;
  assign next_line = y_idx + 10'd64 + 10'd1;
  assign cur_top = cur_entry.y_pos[9:0];
  assign cur_bot = (cur_entry.CGY == HEIGHT_16) ? cur_top + 10'd16 :
                   (cur_entry.CGY == HEIGHT_32) ? cur_top + 10'd32 :
                                                  cur_top + 10'd64;
  
  // Check if the current sprite intersects with our next line
  logic sprite_vertical_intersect;
  assign sprite_vertical_intersect = (cur_top <= next_line) &&
                                     (next_line < cur_bot);

  line_sprite_info_t line_sprite_info[16];
  logic second_half;

  logic [15:0] sprite_base_addr;
  logic [9:0] temp_y;
  logic [6:0] sprite_y_mask;
  logic [15:0] corrected_sprite_addr;
  assign sprite_y_mask  = (cur_entry.CGY == HEIGHT_16) ? 6'h0F :
                          (cur_entry.CGY == HEIGHT_32) ? 6'h1F :
                                                         6'h3F;
  
  //assign temp_y = next_line - cur_top;
  assign temp_y = ((next_line - cur_top) ^ {10{cur_entry.y_invert}}) 
                  & sprite_y_mask;

  // Logic to determine sprite_base_addr
  always_comb begin
    corrected_sprite_addr  = cur_entry.addr;
    if(cur_entry.CGX == WIDTH_32)
      corrected_sprite_addr[0]  = 0;
    if(cur_entry.CGY != HEIGHT_16) begin
      corrected_sprite_addr[1]  = 0;
      if(cur_entry.CGY != HEIGHT_32)
        corrected_sprite_addr[2] = 0;
    end
    
    if (~(second_half ^ cur_entry.x_invert && cur_entry.CGX == WIDTH_32)) begin
      sprite_base_addr = (corrected_sprite_addr << 5) +
                         (temp_y & 4'hF) +
                         ((temp_y >> 4) << 7);
    end

    else begin
      sprite_base_addr = (corrected_sprite_addr << 5) +
                         (temp_y & 4'hF) +
                         ((temp_y >> 4) << 7) +
                         (64); // Add extra 64 to get to second half
    end

  end

  always_ff @(posedge clock, negedge reset_N) begin
 
    // When to reset OR clear to prep for next cycle,
    // which is when we're about to start H_DISP
    if (~reset_N) begin
      satb_idx        <= 7'd0;
      line_sprite_idx <= 5'd0;
      second_half     <= 1'd0;
    end

    else if ((H_state == H_WAIT && H_cnt == 0 && char_cycle == 7) 
             && clock_en) begin
      satb_idx        <= 7'd0;
      line_sprite_idx <= 5'd0;
      second_half     <= 1'd0;
    end

    else if (clock_en) begin

      if (do_Spritecrawl) begin
        // Check if there is space for more sprites or if 
        // we've already checked all of the sprites
        if (line_sprite_idx < 5'd16 && satb_idx < 7'd64) begin
          line_sprite_info[line_sprite_idx].x_pos <= cur_entry.x_pos[9:0];
          line_sprite_info[line_sprite_idx].x_invert <= cur_entry.x_invert;
          line_sprite_info[line_sprite_idx].y_invert <= cur_entry.y_invert;
          line_sprite_info[line_sprite_idx].SPBG <= cur_entry.SPBG;
          line_sprite_info[line_sprite_idx].base_addr <= sprite_base_addr;
          line_sprite_info[line_sprite_idx].color <= cur_entry.color;
          
          if (second_half) begin

            // If we're on the second_half of a sprite, 
            // assume double-wide and copy the data over.
            line_sprite_info[line_sprite_idx].x_pos <= cur_entry.x_pos[9:0] + 
                                                       10'd16;
            // Disable second_half
            second_half <= 1'd0;

            // Move onto next entry 
            line_sprite_idx <= line_sprite_idx + 5'd1;

            // Move onto next sprite
            satb_idx <= satb_idx + 7'd1;

          end // if (second_half) 

          // Check if the next line happens to intersect with this sprite
          else if (sprite_vertical_intersect) begin
            if (cur_entry.CGX == WIDTH_16) begin
              // Move onto the next entry
              line_sprite_idx <= line_sprite_idx + 5'd1;
              // Move onto the next sprite
              satb_idx <= satb_idx + 7'd1;
            end

            else begin
              // Do NOT move onto the next sprite, but enable second_half
              second_half <= 1'd1;
              // Move onto the next entry
              line_sprite_idx <= line_sprite_idx + 1'd1;
            end // else

          end // if (sprite_vertical_intersect) 

          // If there's still sprites left to go but there was no 
          // intersect (or second half), move onto the next sprite
          else begin
            satb_idx <= satb_idx + 7'd1;
          end

        end // if (line_sprite < 5'd16 && satb_idx < 7'd64) 

      end // if (do_Spritecrawl) 
      

    end // if (clock_en)
   
  end // always_ff @ (posedge clock, negedge reset_N)


  /////////////////////////////////////////////////////
  // LINE ARRANGE FSM

  // Index for which pixel we're looking at
  logic [7:0] pix_idx;
  logic [4:0] num_valid_sprites;

  logic [3:0] color_temp[16];
  logic [3:0] px_temp[16];
  logic       SPBG_temp[16];
  logic sprite_present[16];

  logic start_Pixelselect;
  assign start_Pixelselect = (H_state == H_WAIT && H_cnt == 1 &&
                              char_cycle > 0) ||
                             (H_state == H_WAIT && H_cnt == 0) ||
                             (H_state == H_DISP && H_cnt > 1) ||
                             (H_state == H_DISP && H_cnt == 1 &&
                              char_cycle == 0);

  logic clear_Pixelselect;
  assign clear_Pixelselect = (H_state == H_SYNC && H_cnt == 0 &&
                              char_cycle == 7);

  logic do_Pixelselect[15];
  logic [3:0] result_color[15];
  logic [3:0] result_px[15];
  logic result_SPBG[15];
  logic result_present[15];

  assign do_Pixelselect[0] = start_Pixelselect;

  always_ff @(posedge clock, negedge reset_N) begin
    if (~reset_N) begin
      pix_idx <= 8'd0;
      result_present[0] <= 1'b0;
      num_valid_sprites <= line_sprite_idx;
    end

    else if (clear_Pixelselect && clock_en) begin
      pix_idx <= 8'd0;
      result_present[0] <= 1'b0;
      num_valid_sprites <= line_sprite_idx;
    end

    else if (clock_en) begin

      // If it's time to start doing pixelselect, compare
      // sprites 0 and 1;
      if (do_Pixelselect[0]) begin

        pix_idx <= pix_idx + 8'd1;

        result_present[0] <= (sprite_present[0] || sprite_present[1]);

        // Give priority to sprite 0 if it intersects
        result_color[0]   <= (sprite_present[0]) ? color_temp[0] :
                                                 color_temp[1];
        result_px[0]      <= (sprite_present[0]) ? px_temp[0] :
                                                   px_temp[1];
        result_SPBG[0]    <= (sprite_present[0]) ? SPBG_temp[0] :
                                                   SPBG_temp[1];

      end

    end
  end

  genvar j;
  generate
    for (j = 1; j < 15; j++) begin : gen_pixsel_reg

      always_ff @(posedge clock, negedge reset_N) begin

        if (~reset_N) begin
          do_Pixelselect[j] <= 1'b0;
          result_present[j] <= 1'b0;
        end

        else if (clear_Pixelselect && clock_en) begin
          do_Pixelselect[j] <= 1'b0;
          result_present[j] <= 1'b0;
        end

        else if (clock_en) begin

          // Always continue the pipeline enables
          do_Pixelselect[j] <= do_Pixelselect[j - 1];

          // Do the comparison if it's our pipeline step's turn
          if (do_Pixelselect[j]) begin
            
            result_present[j] <= (result_present[j-1] || sprite_present[j+1]);

            result_color[j]   <= (result_present[j-1]) ? result_color[j-1] :
                                                         color_temp[j+1];
            result_px[j]      <= (result_present[j-1]) ? result_px[j-1] :
                                                         px_temp[j+1];

            result_SPBG[j]    <= (result_present[j-1]) ? result_SPBG[j-1] :
                                                         SPBG_temp[j+1];
          end

        end

      end

    end : gen_pixsel_reg

  endgenerate



  logic [7:0] local_pix_idx[16];
  assign local_pix_idx[0] = pix_idx;
  genvar k;
  generate
    for (k = 1; k < 16; k++) begin : gen_local_pix_idx
      always_comb begin
        local_pix_idx[k] = pix_idx - k + 1;
      end
    end : gen_local_pix_idx
  endgenerate

  line_sprite_data_t line_sprite_data[16];
  
  genvar i;
  generate
    for (i = 0; i < 16; i++) begin : gen_horiz_intersect
      line_sprite_data_t cur_line_sprite_data;
      always_comb begin
	    cur_line_sprite_data = line_sprite_data[i];
        // Separate, fetching the correct temp value for each sprite.
        color_temp[i] = cur_line_sprite_data.info.color;
        px_temp[i] = {
          cur_line_sprite_data.data[3][15 - (local_pix_idx[i] - 
          (cur_line_sprite_data.info.x_pos - 32))],
                      
          cur_line_sprite_data.data[2][15 - (local_pix_idx[i] - 
          (cur_line_sprite_data.info.x_pos - 32))],
                      
          cur_line_sprite_data.data[1][15 - (local_pix_idx[i] - 
          (cur_line_sprite_data.info.x_pos - 32))],
                      
          cur_line_sprite_data.data[0][15 - (local_pix_idx[i] - 
          (cur_line_sprite_data.info.x_pos - 32))]
        };

        SPBG_temp[i] = cur_line_sprite_data.info.SPBG;

        sprite_present[i] = 1'b0;

        // Only bother checking if this entry even is a valid sprite
        if (i < num_valid_sprites) begin

          if (px_temp[i] != 0) begin

            // Check if this sprite collides with the current pix
            if (cur_line_sprite_data.info.x_pos - 32 <= local_pix_idx[i] && 
                local_pix_idx[i] < cur_line_sprite_data.info.x_pos - 32 + 16) 
              begin

              sprite_present[i] = 1'b1;

            end

          end
        end


      end
    end : gen_horiz_intersect
  endgenerate



  localparam BG_PIPE_LEN = 3; //we always write 2 ahead of our read

  tile_line_t [BG_PIPE_LEN-1:0] tile_pipe; //need extra slot for current fetch
  tile_line_t output_tile;
  logic [$clog2(BG_PIPE_LEN)-1:0] bg_wr_ptr, bg_rd_ptr;
  
  assign output_tile = tile_pipe[bg_rd_ptr];
 

  /*
   * Interrupts
   */
  v_state_t prev_V_state;
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      IRQ_n        <= 1;
      prev_V_state <= V_SYNC;
      status       <= 0; 
    end
    else begin
      if(clock_en) begin
        prev_V_state <= V_state;
        //clear pending
        if(CR[0]); //TODO: implement sprite 0 interrupts
        if(CR[1]); //TODO: implement sprite overflow interrupts
        //Vblank interrupt
        if(V_state != V_DISP && prev_V_state == V_DISP)
          if(CR[3]) begin
            IRQ_n     <= 0;
            status[5] <= 1;
            end
        if(EOL) begin //end of line means it's time to process raster interrupts
          //RCR interrupts
          if((V_state == V_WAIT && V_cnt == 0) || (V_state == V_DISP)) begin
            if((cur_row+1 == RCR-64) & (RCR >= 64) & CR[2]) begin
              IRQ_n     <= 0; //TODO: investigate clock_en interrupt timing
              status[2] <= 1;
            end
          end
        end
      end
      if(clock_en & ACK) begin //interrupt acknowledge overrides all
          IRQ_n  <= 1;
          status <= 0;
      end
    end
  end


  logic [2:0]  cycle_adjusted;
  assign cycle_adjusted = char_cycle + x_px_offset;


  ///////////////////////////////////////////////////////////////////////////
  // SPRITE FETCH FSM
  ///////////////////////////////////////////////////////////////////////////

  // Which sprite we're fetching
  logic [3:0] cur_sprite_fetch, prev_sprite_fetch;

  // Which word we're fetching for the current sprite
  logic [1:0] sprite_word_idx, prev_sprite_word_idx;

  logic first_Spritefetch_cycle;
  
  always_ff @(posedge clock, negedge reset_N) begin

    if (~reset_N) begin
      cur_sprite_fetch <= 4'd0;
      sprite_word_idx <= 2'd0;
      prev_sprite_fetch <= 4'd0;
      prev_sprite_word_idx <= 2'd0;
      first_Spritefetch_cycle <= 1'd1;
    end

    else if (H_state == H_DISP && H_cnt == 0 && clock_en) begin
      cur_sprite_fetch <= 4'd0;
      sprite_word_idx <= 2'd0;
      prev_sprite_fetch <= 4'd0;
      prev_sprite_word_idx <= 2'd0;
      first_Spritefetch_cycle <= 1'd1;
    end

    else if (clock_en) begin
      first_Spritefetch_cycle <= 1'd0;

      prev_sprite_fetch <= cur_sprite_fetch;
      prev_sprite_word_idx <= sprite_word_idx;

      if (do_Spritefetch) begin
        // Copy over the info
        if (sprite_word_idx == 2'd0) begin
          line_sprite_data[cur_sprite_fetch].info <= 
                                           line_sprite_info[cur_sprite_fetch];
        end // if (sprite_word_idx == 2'd0)
        
        if (sprite_word_idx == 2'd3) begin
          cur_sprite_fetch <= cur_sprite_fetch + 4'd1;
          sprite_word_idx <= 2'd0;
        end // if (sprite_word_idx == 2'd3)

        else begin
          sprite_word_idx <= sprite_word_idx + 2'd1;
        end // else
        
        
      end // if (do_Spritefetch)

    end // else

  end // always_ff


/*
  always @(posedge (H_state == H_DISP)) begin
    $display("%x, %x", next_line, num_valid_sprites);
  end
 */

  logic [3:0] tile_pix;
  assign tile_pix = {output_tile.CG1[15 - cycle_adjusted],
                     output_tile.CG1[7 - cycle_adjusted], 
                     output_tile.CG0[15 - cycle_adjusted],
                     output_tile.CG0[7 - cycle_adjusted]};

  logic sprite_pixel_present;
  logic [3:0] sprite_pixel_color;
  logic [3:0] sprite_pixel_px;
  logic sprite_pixel_SPBG;

  assign sprite_pixel_present = result_present[14];
  assign sprite_pixel_color   = result_color[14];
  assign sprite_pixel_px      = result_px[14];
  assign sprite_pixel_SPBG    = result_SPBG[14]; 
  
  always_comb begin
    VD = 0;
    if(in_vdw) begin //TODO: make this work correctly with new VSYNC

      if (sprite_pixel_present) begin

        if (~sprite_pixel_SPBG) begin
          // If it's a background pixel and the BG pixel is the 0
          // pixel, then draw the sprite
          if (tile_pix == 0) begin 
            VD[8] = 1;
            VD[7:4] = sprite_pixel_color;
            VD[3:0] = sprite_pixel_px;;
          end

          // If the BG pixel is NOT 0 (i.e. in front of the sprite)
          // then draw the BG
          else begin
            VD[8] = 0;
            VD[7:4] = output_tile.palette_num;
            VD[3:0] = tile_pix;
          end

        end

        else begin

          VD[8] = 1;
          VD[7:4] = sprite_pixel_color;
          VD[3:0] = sprite_pixel_px;

        end

      end

      else begin

      	VD[8]    = 0; //BG selected
      	VD[7:4]  = output_tile.palette_num;
      	VD[3:0]  = tile_pix;
        if (tile_pix == 0)
          VD[7:4] = 0;

      end
    end
    else
      VD[8]  = 1; //sprite palette 0, color 0 in overscan
  end


 
  logic [15:0] tile_ptr;
  logic [2:0]  fetch_row;
  bat_entry_t curbat;
  assign curbat  = MD_in;
  assign fetch_row = y_px_offset + cur_row;
  logic [15:0] bat_ptr;
  logic [15:0] MD_invert; //MD_in with bits swapped (0->15, 1->14, etc.)
  always_comb begin
    for(int i = 0; i < 16; i++)
      MD_invert[i] = MD_in[15-i];
  end

  
  //background pipeline
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      for(int i = 0; i < BG_PIPE_LEN; i++) begin
        tile_pipe[i].palette_num <= 0;
        tile_pipe[i].CG0         <= 0;
        tile_pipe[i].CG1         <= 0;
      end
      SATfetch_cur_entry <= 0;
    end
    else if(clock_en) begin
      if(do_BGfetch) begin
        case(char_cycle)
          1: begin
            tile_pipe[bg_wr_ptr].palette_num <= curbat.palette_num;
            tile_ptr <= (curbat.tile_index << 4) + fetch_row;
            //$strobe("tile_ptr: %x", tile_ptr);
          end
          5: tile_pipe[bg_wr_ptr].CG0 <= MD_in;
          7: tile_pipe[bg_wr_ptr].CG1 <= MD_in;
        endcase
      end

      else if (do_SATfetch) begin
        case(char_cycle)
          1: satb_entries[SATfetch_cur_entry][63:48] <= MD_in;
          3: satb_entries[SATfetch_cur_entry][47:32] <= MD_in;
          5: satb_entries[SATfetch_cur_entry][31:16] <= MD_in;
          7: begin
            satb_entries[SATfetch_cur_entry][15:0]  <= MD_in;
            SATfetch_cur_entry <= SATfetch_cur_entry + 1;
          end
        endcase
      end


      else if (do_Spritefetch) begin
//        if (~first_Spritefetch_cycle)
        if(~line_sprite_data[cur_sprite_fetch].info.x_invert)
          line_sprite_data[cur_sprite_fetch].data[sprite_word_idx] <= MD_in;
        else
          line_sprite_data[cur_sprite_fetch].data[sprite_word_idx] <= MD_invert;
      end

    end
  end  


  //VRAM address control
  always_comb begin
    MA = 0;
    //assume we have dot width 00
    if(do_BGfetch) begin
      case(char_cycle)
        0: MA        = CPU_maddr;
        1: MA        = bat_ptr;         //fetch BAT
        2: MA        = CPU_maddr;
        4: MA        = CPU_maddr;
        5: MA        = tile_ptr;        //fetch CG0
        6: MA        = CPU_maddr;
        7: MA        = tile_ptr + 12'h8;//fetch CG1 (check the offset here)
      endcase
    end

    else if (do_SATfetch) begin
      case(char_cycle)
        1: MA = SATB.data + 0 + (SATfetch_cur_entry << 2);
        3: MA = SATB.data + 1 + (SATfetch_cur_entry << 2);
        5: MA = SATB.data + 2 + (SATfetch_cur_entry << 2);
        7: MA = SATB.data + 3 + (SATfetch_cur_entry << 2);
        default: MA = 0;
      endcase
    end

    else if (do_Spritefetch) begin
      MA = line_sprite_info[cur_sprite_fetch].base_addr + (sprite_word_idx<<4);
    end

    else MA = CPU_maddr;
  end


  logic  [10:0] cur_col; //COL IN TILES, ROW IN PIXELS
  assign bat_ptr = cur_col + ((((cur_row + y_start) & y_mask) >> 3) << y_shift);
  //BAT pointer management
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      cur_row <= 0; //TODO: actually handle this correctly
      cur_col <= 0;
    end
    else if(clock_en) begin

      if(V_state == V_WAIT) cur_row <= 0; //latch to 0 right before drawing

      // If we're just about to finish H_DISP mode, we should increment
      // our cur_row value
      else if(H_state == H_DISP && H_cnt == 0 && char_cycle == 7) begin
        cur_row <= cur_row + 1; //TODO: need to initialize cur_row
      end

      // If we're just about to finish HSYNC, set our bat_ptr
      else if(H_state == H_SYNC && H_cnt == 0 && char_cycle == 7) begin
        cur_col <= x_tl_offset;
      end

      // If we're doing a BG fetch, increment the BAT for the next fetch
      else if(do_BGfetch) begin
        if(char_cycle == 7)
          cur_col <= (cur_col + 1) & x_mask;
      end

    end
  end

  
  //char_cycle + rd/wr pointer adjustment
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      char_cycle <= 0;
      bg_wr_ptr  <= 0;
      bg_rd_ptr  <= 0;
    end
    else if(clock_en) begin

      // Increment our char_cycle on every clock
      char_cycle <= char_cycle + 1;


      if(in_vdw) begin

        // If we are indeed on last char cycle and we're in a DISPLAY mode,
        // we should increment bg_rd_ptr. Keep looping through the BG pipeline
        if(cycle_adjusted == 7) begin
          if(bg_rd_ptr == (BG_PIPE_LEN-1)) bg_rd_ptr <= 0;
          else bg_rd_ptr <= bg_rd_ptr + 1;
        end

      end

      else bg_rd_ptr <= 0; //TODO: jump to real start of line


      // Loop through the pipeline for writes, increment at the end of
      // every char cycle
      if(do_BGfetch) begin
        if(char_cycle == 7) begin
          if(bg_wr_ptr == (BG_PIPE_LEN-1)) bg_wr_ptr <= 0;
          else bg_wr_ptr <= bg_wr_ptr + 1;
        end
      end
      else bg_wr_ptr <= 0; //first write is garbage
    end
  end
endmodule : vdc_HuC6270
