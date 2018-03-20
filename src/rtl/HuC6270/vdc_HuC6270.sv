`default_nettype none
`include "VDCDefines.vh"

/*
 * verilog model of HuC6270 VDC
 *
 * (C) 2018 Ford Seidel and Amolak Nagi
 */


module vdc_HuC6270(input logic clock, reset_N, clock_en, //MMIO_clock_en,     
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

  logic MMIO_clock_en;
  clock_divider MMIO_clock(.clk(clock), .reset(~reset_N), 
                          .clk_en(MMIO_clock_en));
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


  // VRAM address and data bus wires
  logic [15:0] MA;
  logic [15:0] MD_in, MD_out, MD_in_buf, latched_read;
  logic vram_re, vram_we;

  assign vram_re  = ~vram_we;


  logic   read_delay; //selects whether or not we go for a real read on next 
                     //clock

  always_ff @(posedge clock) begin
    if(~clock_en)
      latched_read <= MD_in;
  end

  
  //TODO: standardize resets
  always_ff @(posedge clock) begin
    if(~reset_N) begin
      read_delay   <= 0;
    end
    else if(clock_en) begin
      read_delay   <= 0;
    end
    else
      read_delay   <= 1;
  end

  assign MD_in = (read_delay) ? latched_read : MD_in_buf;
  
  VRAM vram(.clock(clock),  //fseidel: This interface needs some work
            .reset_N(reset_N),
            .MA(MA),
            .re(vram_re),
            .we(vram_we),
            .MD_out(MD_in_buf),
            .MD_in(MD_out)
            );
  
  logic [15:0] data_in, data_out;
  logic [14:0] address;
  logic wren;

  logic [2:0]  char_cycle; //current position in char cycle

  //latch H*R in horizontal blanking, V*R in vertical blanking
  logic [15:0] HSR, HDR, VSR; 
  logic [8:0]  VDR;
  logic [7:0]  VCR;
  
  logic [9:0] H_cnt; //horizontal position counter, reused for each phase
  logic [9:0] V_cnt; //vertical position counter, reused for each phase

  h_state_t H_state; //state in horizontal line
  v_state_t V_state; //state in vertical screen

  logic [15:0] BXR, BYR; //x and y scroll registers
  //assign BXR = 0; //These can be harcoded to force scroll
  //assign BYR = 0;

  logic [7:0]  MWR;

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
  
  //Hardcoded values for Parasol Stars title screen
  assign HSR  = 16'h0202;
  assign HDR  = 16'h031F;
  
  assign VSR  = 16'h0F02;
  assign VDR  = 16'h00EF;
  assign VCR  = 16'h0003;
    
  logic [4:0] HSW;
  logic [6:0] HDS;
  logic [6:0] HDW;
  logic [3:0] HDE;

  
  assign HSW  = HSR[4:0];
  assign HDS  = HSR[14:8];
  assign HDW  = HDR[6:0];
  assign HDE  = HDR[11:8];

  logic [4:0] VSW;
  logic [7:0] VDS;
  logic [8:0] VDW;
  //VCR is passed through

  assign VSW  = VSR[4:0];
  assign VDS  = VSR[15:8];
  assign VDW  = VDR[8:0];
  //VCR is passed through



  logic [15:0] MAWR, MARR;
  logic [9:0]  RCR;
  logic [12:0]  CR;
  
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

  //read/write edge detection
  logic read, write, prev_RD_n, prev_WR_n;
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      //prev_RD_n <= 1;
      prev_WR_n <= 1;
    end
    else if(MMIO_clock_en) begin //runs at MMIO clock
      //prev_RD_n <= RD_n | CS_n | ~BUSY_n;
      prev_WR_n <= WR_n | CS_n | ~BUSY_n;
    end
  end


  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      prev_RD_n <= 1;
    end
    else if(clock_en) begin //TODO: wut
      prev_RD_n <= RD_n | CS_n | ~BUSY_n;
    end
  end
  

  assign read   = (~RD_n & prev_RD_n & ~CS_n & BUSY_n);
  assign write  = (~WR_n & prev_WR_n & ~CS_n & BUSY_n);


  logic [15:0] CPU_maddr; //goes to address generator
  assign CPU_maddr = (vram_we) ? MAWR : MARR;

  
  logic vram_write_pending;
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
      //This next part must finish before another write is issued or bad
      //things will happen! Should be okay with our timings.
      //This goes first so that a new write will set vram_write_pending
      if(vram_we & vram_write_pending) begin //ignore DMA
        vram_write_pending <= 0; //clear the write pending bit ASAP
        MAWR <= MAWR + 1; //and increment the write address
      end
      if(MMIO_clock_en) begin
        if(write)
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
                  MD_out[7:0] <= D;
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
                REG_VRR_VWR: begin
                  MD_out[15:8] <= D;
                  vram_write_pending <= 1;
                end
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
  //TODO: basically everything
  logic ACK;
  always_comb begin
    ACK = 0;
    if(~RD_n & ~CS_n & (A == 0)) begin
      ACK = 1;
    end
  end

  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N)
      D_out <= 0;
    else if(MMIO_clock_en) begin //TODO: figure out how clock_en works here
      if(read & (A == 0))
        D_out <= status;
      else if(read & (A == 1))
        D_out <= 0; //reads back 0, writes ignored
      else if(read & (A == 2))
        case(VDC_regnum)
          default:
            D_out <= 0; //broken, but let's see what happens
        endcase
    end
  end



  
  //hang CPU if it tries to touch VRAM outside of when it's allowed to
  assign BUSY_n = ~((~WR_n | ~RD_n) & ~CS_n & (VDC_regnum == REG_VRR_VWR) 
                    & A[1] & ~do_BGfetch);

  /*
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N)
      BUSY_n <= 1;
    else if(write & )
  end
   */
  
  //CPU VRAM access
  //CPU gets to touch VRAM on even cycles during BGfetch
  always_comb begin
    vram_we = 0;
    if(do_BGfetch & ~char_cycle[0]) begin
      if(vram_write_pending)
        vram_we = 1;
    end
  end

  
  /*
   * Horizontal Syncronization
   */         
  assign do_BGfetch = ((H_state == H_DISP) || 
                      (H_state == H_WAIT && H_cnt < 2)) && 
                      (V_state == V_DISP);


  logic EOL; //signal for end of line
  assign EOL = (char_cycle == 7) && (H_state == H_END) && (H_cnt == 0);
  
  assign HSYNC_n = ~(H_state == H_SYNC);
  //H_state control
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      H_cnt   <= HSW;
      H_state <= H_SYNC;
      x_start <= 0;
      y_start <= 0;
    end
    else if(clock_en) begin
      if(char_cycle == 6 && H_state == H_SYNC) begin
        x_start <= BXR; //TODO: HACK! latch at end of DISP???
        y_start <= BYR;
      end
      if(char_cycle == 7) begin
        H_cnt <= H_cnt - 1; //default: decrement H_cnt
        case(H_state)
          H_SYNC:
            if(H_cnt == 0) begin
              H_cnt   <= HDS;
              H_state <= H_WAIT;
            end
          H_WAIT:
            if(H_cnt == 0) begin
              H_cnt   <= HDW;
              H_state <= H_DISP;
            end
          H_DISP:
            if(H_cnt == 0) begin
              H_cnt   <= HDE;
              H_state <= H_END;
            end
          H_END:
            if(H_cnt == 0) begin
              H_cnt   <= HSW;
              H_state <= H_SYNC;
            end
        endcase
      end
    end
  end

  //V_state control
  logic frame_reset; //set below
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      V_cnt   <= VDS + 1;
      V_state <= V_WAIT;
    end
    else if(clock_en) begin
      if(EOL) begin
        if(frame_reset) begin
          V_cnt <= VDS + 1;
          V_state <= V_WAIT;
        end
        else begin
          V_cnt <= V_cnt - 1;
          case(V_state)
            V_SYNC:
              if(V_cnt == 0) begin
                V_cnt   <= VDS + 1;
                V_state <= V_WAIT;
              end
            V_WAIT: 
              if(V_cnt == 0) begin
                V_cnt   <= VDW;
                V_state <= V_DISP;
              end
            V_DISP:
              if(V_cnt == 0) begin
                V_cnt   <= VCR - 1;
                V_state <= V_END;
              end
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
  
  //VDC -> VCE communications
  logic        in_vdw; //are we currently in active display?
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
      if(MMIO_clock_en & ACK) begin //interrupt acknowledge overrides all
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
          2: begin
            tile_pipe[bg_wr_ptr].palette_num <= curbat.palette_num;
            tile_ptr <= (curbat.tile_index << 4) + fetch_row;
            //$strobe("tile_ptr: %x", tile_ptr);
          end
          6: tile_pipe[bg_wr_ptr].CG0 <= MD_in;
          0: tile_pipe[bg_wr_ptr].CG1 <= MD_in;
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
    else MA = 0;
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
      bg_wr_ptr  <= BG_PIPE_LEN-1; //first write of a line is garbage
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
        if(char_cycle == 0) begin
          if(bg_wr_ptr == (BG_PIPE_LEN-1)) bg_wr_ptr <= 0;
          else bg_wr_ptr <= bg_wr_ptr + 1;
        end
      end
      else bg_wr_ptr <= BG_PIPE_LEN-1; //first write is garbage
    end
  end

  


/*
  // This entire FSM is just to test the BRAM. DISREGARD
  enum logic [3:0] {WAIT, READ, WRITE, END} state, next_state;

  always_comb begin

    wren = 0;
    data_in = 16'd0;

    unique case (state)

      WAIT: begin

        next_state = WRITE;
        wren = 1'b1;
        data_in = 16'hF0F0;

      end

      WRITE: begin

        next_state = READ;

      end

      READ, END: begin

        next_state = END;

      end

    endcase

  end

  always_ff @(posedge clock, negedge reset_N) begin
    if (~reset_N)
      state <= WAIT;
    else
      state <= next_state;

  end
*/

endmodule : vdc_HuC6270;



/*
module top();

  logic clock, CS_n, WR_n, reset_N, dummy;
  logic [15:0] D;

  vdc_HuC6270 vdc(.*);

  // This entire TB is just to make sure the BRAM is working
  // in simulation. Disregard all of this

  initial begin
    $monitor("state: %s data_out: %x", vdc.state, vdc.data_out);
    clock = 1'b0;
    reset_N = 1'b0;
    reset_N <= 1'b1;
    forever #5 clock = ~clock;
  end

  initial begin

    @(posedge clock);
    @(posedge clock);
    @(posedge clock);
    @(posedge clock);

    $finish;
  end

endmodule: top
*/





// Old VDC inputs, not needed but going to keep here in case it's ever useful
/*
                    inout logic [7:0]  D_low,
                    inout logic [8:15] D_hi,
                    input logic EW_8_16,     // Data bus width select (8 en, 16 dis), UNUSED
                    inout logic VSYNC_n,
                    inout logic HSYNC_n,
                    output logic DISP, // screen blanking status (blanked dis, displayed en)
                    inout logic SPBG,  // pixel bus (sprite en, bkgrnd dis) (VD8)
                    inout logic [7:0] VD,
                    output logic MWR_n, // VRAM write strobe
                    output logic MRD_n, // VRAM read strobe
                    inout logic [15:0] MD, // VRAM Data bus
                    output logic [15:0] MA, // VRAM Address bus
                    output logic IRQ_n, // IRQ output to HuC6280
                    output logic BUSY_n, // BUSY status output
                    input logic [1:0] A);
*/
