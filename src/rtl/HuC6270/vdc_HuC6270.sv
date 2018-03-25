`default_nettype none
`include "VDCDefines.vh"

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
  logic [15:0] MA;
  logic [15:0] MD_in, MD_out, MD_in_buf, latched_read;
  logic vram_re, vram_we;

  assign vram_re  = ~vram_we;


  /*
  logic   read_delay; //selects whether or not we go for a real read on next 
                     //clock

  always_ff @(posedge clock) begin
    if(clock_en)
      latched_read <= MD_in_buf;
  end

  assign MD_in  = latched_read;
  */
  
  VRAM vram(.clock(clock),  //fseidel: This interface needs some work
            .reset_N(reset_N),
            .MA(MA),
            .re(vram_re),
            .we(vram_we),
            //.MD_out(MD_in_buf),
            .MD_out(MD_in),
            .MD_in(MD_out)
            );
 

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
  assign x_mask = (1 << 9) - 1;
  assign y_mask = (1 << 9) - 1; //ditto
  assign x_px_offset  = x_start[2:0];
  assign x_tl_offset  = x_start[9:3];
  assign y_px_offset  = y_start[2:0];
  assign y_tl_offset  = y_start[9:3];
  

  logic [15:0] MAWR, MARR;
  logic [9:0]  RCR;
  logic [12:0]  CR;

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
  
/*
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      prev_RD_n <= 1;
    end
    else if(clock_en) begin //TODO: wut
      prev_RD_n <= RD_n | CS_n | ~BUSY_n;
    end
  end
  */

  
  logic vram_write_pending;
  logic MARR_increment;
  logic CPU_read_issue, CPU_write_issue;
  logic [15:0] CPU_write_buf;
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      VDC_regnum         <= 0; //makes BUSY_n behave on reset
      vram_write_pending <= 0;
      RCR                <= 0;
      CR                 <= 0;
      MAWR               <= 0;
      MARR               <= 0;
      BXR                <= 0;
      BYR                <= 0;
      MWR                <= 0;
    end
    else begin
      if(clock_en) begin
        if(CPU_read_issue) begin
          MARR <= MARR + 1;
        end
        if(CPU_write_issue) begin
          MAWR <= MAWR + 1;
        end
        if(WR & ~prev_WR)
          case(A)
            A_STATUS_ADDR_REG:
              VDC_regnum <= D[4:0];
            A_DATA_LSB:
              case(VDC_regnum)
                REG_MAWR:
                  MAWR[7:0] <= D;
                REG_MARR:
                  MARR[7:0] <= D;
                REG_VRR_VWR:
                  CPU_write_buf[7:0] <= D;
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
                REG_VRR_VWR:
		  CPU_write_buf[15:8] <= D;
                REG_CR:
                  CR[12:8] <= D[4:0];
                REG_RCR:
                  RCR[9:8] <= D[1:0];
                REG_BXR:
                  BXR[9:8] <= D[1:0];
                REG_BYR:
                  BYR[8]   <= D[0];
                REG_MWR:; //only 8 bits wide
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


  enum logic [1:0] {IDLE, READ, WRITE, WAIT}
       mem_state, mem_nextstate;
  
  assign VRAM_read_req = (A == 3) &
                         ((RD & (VDC_regnum == REG_VRR_VWR)) | 
                          (WR & (VDC_regnum == REG_MARR)));
  
  
  assign VRAM_write_req = WR & (A == 3) & (VDC_regnum == REG_VRR_VWR); 
  
  //TODO: MARR update can affect BUSY_n
  //TODO: VBLANK access/BURST mode
  logic CPU_access_ok;
  assign CPU_access_ok = (V_state != V_DISP || H_state == H_DISP)
                         & ~char_cycle[0];  //CPU gets even cycles


  logic mem_wait;


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

  logic [15:0] CPU_maddr; //goes to address generator
  //assign CPU_maddr = (vram_we) ? MAWR : MARR;



  assign MD_out = CPU_write_buf; //only matters 1 cycle after clock_en

  assign CPU_maddr = (CPU_write_issue) ? MAWR : MARR;
  
  logic [15:0] VRAM_readbuf;
  //next state logic. Turns out that doing this 240-style is really clean
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      mem_state <= IDLE;
    end
    else if(clock_en) begin
      mem_state <= mem_nextstate;
      if(CPU_read_issue)
        VRAM_readbuf <= MD_in;
      
    end
  end
  
  logic [15:0] VRAM_output; //TODO: is this okay?
  //assign VRAM_output = (char_cycle[0]) ? MD_in : VRAM_readbuf;
  assign VRAM_output = VRAM_readbuf;

  //TODO: is this fast enough?
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      D_out <= 0;
    end
    else if(clock_en & RD & ~prev_RD) begin
      if(A == 0)
        D_out <= status;
      else if(A == 1)
        D_out <= 0; //reads back 0, writes ignored
      else if(VDC_regnum == REG_VRR_VWR) begin
        if(mem_state == IDLE) begin
          if(~A[0]) //A == 2
            D_out <= VRAM_readbuf[7:0];
          else //A == 3
            D_out <= VRAM_readbuf[15:8];
        end
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


  assign vram_we = CPU_write_issue;
  
  /*
   * Horizontal Syncronization
   */         
  assign do_BGfetch = ((H_state == H_DISP) || 
                      (H_state == H_WAIT && H_cnt < 2)) && 
                      (V_state == V_DISP);


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
    end
    else if(clock_en) begin
      if(EOL) begin
        // Reset for a frame
        if(frame_reset) begin
          V_cnt <= VDS + 1;
          V_state <= V_WAIT;
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

  localparam BG_PIPE_LEN = 3; //we always write 2 ahead of our read

  tile_line_t [BG_PIPE_LEN-1:0] tile_pipe; //need extra slot for current fetch
  tile_line_t output_tile;
  logic [$clog2(BG_PIPE_LEN)-1:0] bg_wr_ptr, bg_rd_ptr;
  
  assign output_tile = tile_pipe[bg_rd_ptr];
 


  // Logic to VCE telling it if we're currently in active display 
  logic        in_vdw; 
  assign in_vdw = (H_state == H_DISP && V_state == H_DISP);

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
  
  always_comb begin
    VD = 0;
    if(in_vdw) begin //TODO: make this work correctly with new VSYNC
      VD[8]    = 0; //BG selected
      VD[7:4]  = output_tile.palette_num;
      VD[3:0]  = {output_tile.CG1[15 - cycle_adjusted],
                  output_tile.CG1[7 - cycle_adjusted], 
                  output_tile.CG0[15 - cycle_adjusted],
                  output_tile.CG0[7 - cycle_adjusted]};
    end
  end
  
  logic [15:0] tile_ptr;
  logic [2:0]  fetch_row;
  bat_entry_t curbat;
  assign curbat  = MD_in;
  assign fetch_row = y_px_offset + cur_row;
  
  //background pipeline
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      for(int i = 0; i < BG_PIPE_LEN; i++) begin
        tile_pipe[i].palette_num <= 0;
        tile_pipe[i].CG0         <= 0;
        tile_pipe[i].CG1         <= 0;
      end
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
    end
  end  
  logic [15:0] bat_ptr;
  //VRAM address control
  always_comb begin
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
    else MA = CPU_maddr;
  end

  //BAT pointer management
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      bat_ptr <= 0;
      cur_row <= 0; //TODO: actually handle this correctly
    end
    else if(clock_en) begin
      if(V_state == V_WAIT) cur_row <= 0; //latch to 0 right before drawing
      else if(H_state == H_DISP && H_cnt == 0 && char_cycle == 7) begin
        cur_row <= cur_row + 1; //TODO: need to initialize cur_row
      end
      else if(H_state == H_SYNC && H_cnt == 0 && char_cycle == 7) begin
        bat_ptr <= (x_tl_offset & x_mask) + 
                   ((((cur_row + y_start) >> 3) & y_mask) << y_shift);
      end
      else if(do_BGfetch) begin
        if(char_cycle == 7)
          bat_ptr <= bat_ptr + 1; //TODO: scrolling still broken
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
      char_cycle <= char_cycle + 1;
      if(in_vdw) begin
        if(cycle_adjusted == 7) begin
          if(bg_rd_ptr == (BG_PIPE_LEN-1)) bg_rd_ptr <= 0;
          else bg_rd_ptr <= bg_rd_ptr + 1;
        end
      end
      else bg_rd_ptr <= 0; //TODO: jump to real start of line
      if(do_BGfetch) begin
        if(char_cycle == 7) begin
          if(bg_wr_ptr == (BG_PIPE_LEN-1)) bg_wr_ptr <= 0;
          else bg_wr_ptr <= bg_wr_ptr + 1;
        end
      end
      else bg_wr_ptr <= 0; //first write is garbage
    end
  end
endmodule : vdc_HuC6270;
