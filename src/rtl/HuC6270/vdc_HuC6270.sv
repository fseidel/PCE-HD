`default_nettype none
`include "VDCDefines.vh"

/*
 * verilog model of HuC6270 VDC
 *
 * (C) 2018 Ford Seidel and Amolak Nagi
 */


module vdc_HuC6270(input logic clock, reset_N,         
                   input logic [7:0]  DI,// Data in, only lower 8 bits used
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
  logic [15:0] MD_in, MD_out;
  logic vram_re, vram_we;

  //hack to force signals for testing
  assign vram_re  = 1;
  assign vram_we  = 0;

  VRAM vram(.clock(clock),  //fseidel: This interface needs some work
            .reset_N(reset_N),
            .MA(MA),
            .re(vram_re),
            .we(vram_we),
            .MD_out(MD_in),
            .MD_in(MD_out)
            );
 

  // VDC Registers

  //latch H*R in horizontal blanking, V*R in vertical blanking
  BXR_t BXR; // $07
  BYR_t BYR; // $08
  // MWR_t MWR; // $09 
  HSR_t HSR; // $0A
  HDR_t HDR; // $0B
  VSR_t VSR; // $0C
  VDR_t VDR; // $0D
  VCR_t VCR; // $0E
  SATB_t SATB; // $13
  assign SATB.data = 16'h7F00; // TODO: SET THIS

  Register #($bits(BXR), 16'd0) BXR_reg(
                                        .clk(clock),
                                        .en(1'b1),
                                        .rst_l(reset_N),
                                        .clear(1'b0),
                                        .D(16'd0),
                                        .Q(BXR)
                                       ); 

  Register #($bits(BYR), 16'd0) BYR_reg(
                                        .clk(clock),
                                        .en(1'b1),
                                        .rst_l(reset_N),
                                        .clear(1'b0),
                                        .D(16'd0),
                                        .Q(BYR)
                                       ); 

  Register #($bits(HSR), 16'h0202) HSR_reg(
                                        .clk(clock),
                                        .en(1'b1),
                                        .rst_l(reset_N),
                                        .clear(1'b0),
                                        .D(16'h0202),
                                        .Q(HSR)
                                       ); 

  Register #($bits(HDR), 16'h031F) HDR_reg(
                                        .clk(clock),
                                        .en(1'b1),
                                        .rst_l(reset_N),
                                        .clear(1'b0),
                                        .D(16'h031F),
                                        .Q(HDR)
                                       ); 

  Register #($bits(VSR), 16'h0F02) VSR_reg(
                                        .clk(clock),
                                        .en(1'b1),
                                        .rst_l(reset_N),
                                        .clear(1'b0),
                                        .D(16'h0F02),
                                        .Q(VSR)
                                      );

  Register #($bits(VDR), 16'h00EF) VDR_reg(
                                        .clk(clock),
                                        .en(1'b1),
                                        .rst_l(reset_N),
                                        .clear(1'b0),
                                        .D(16'h00EF),
                                        .Q(VDR)
                                      );

  Register #($bits(VCR), 16'h0003) VCR_reg(
                                        .clk(clock),
                                        .en(1'b1),
                                        .rst_l(reset_N),
                                        .clear(1'b0),
                                        .D(16'h0003),
                                        .Q(VCR)
                                      );
    
  logic [4:0] HSW; // Horizontal synchronous pulse width
  logic [6:0] HDS; // Horizontal display start position - 1
  logic [6:0] HDW; // Horizontal display width in tiles - 1
  logic [3:0] HDE; // Horizontal display ending period - 1

  
  assign HSW  = HSR.HSW;
  assign HDS  = HSR.HDS;
  assign HDW  = HDR.HDW;
  assign HDE  = HDR.HDE;

  logic [4:0] VSW; // Vertical synchronous pulse width
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
  
  assign y_shift = 5; //yet another hack
  assign x_mask = (1 << 9) - 1;
  assign y_mask = (1 << 9) - 1; //ditto
  assign x_px_offset  = x_start[2:0];
  assign x_tl_offset  = x_start[9:3];
  assign y_px_offset  = y_start[2:0];
  assign y_tl_offset  = y_start[9:3];





  logic first_frame_done; 


  logic       do_BGfetch;            
  assign do_BGfetch = ((H_state == H_DISP) || 
                      (H_state == H_WAIT && H_cnt < 2)) && 
                      (V_state == V_DISP);

  // SAT Fetch
  logic do_SATfetch;
  assign do_SATfetch = ((V_state == V_SYNC));
  satb_entry_t first_sprite;


  // Sprite Data Fetch
  logic do_Spritefetch;
  assign do_Spritefetch = ((H_state == H_SYNC) || (H_state == H_WAIT) || (H_state == H_END)) &&
                          (V_state == V_DISP) && 
                          ~do_BGfetch &&
                          first_frame_done;
  logic [15:0] sprite_data[256];

  // Sprite fetching FSM
  logic [5:0] block_y_idx;
  logic [2:0]  block_x_idx;

  logic [10:0] sprite_num_words;
  assign sprite_num_words = 11'd256;
  logic [7:0] sprite_word_count;

/*
  logic [15:0] addr_offset;
  always_comb begin
    if (first_sprite.CGX == WIDTH_16) begin
      addr_offset = (block_x_idx << 4) + block_y_idx;
    end

    else begin
      addr_offset = (block_x_idx << 4) + ((block_y_idx >> 4) << 7) + block_y_idx;
    end
  end

  
  always_ff @(posedge clock, negedge reset_N) begin
    if (~reset_N) begin
      block_x_idx <= 3'd0;
      block_y_idx <= 6'd0;
    end

    else begin

      // Only increment anything if we're even spriteFetching
      if (do_Spritefetch && (char_cycle & 1)) begin


        // If we're 16 wide and we're at the end of a line, loop around
        if ((block_x_idx == 3'd3) && (first_sprite.CGX == WIDTH_16)) begin
          block_x_idx <= 3'd0;

          if ((block_y_idx == 6'd15 && first_sprite.CGY == HEIGHT_16) ||
              (block_y_idx == 6'd31 && first_sprite.CGY == HEIGHT_32) ||
              (block_y_idx == 6'd63 && first_sprite.CGY == HEIGHT_64)) begin
            block_y_idx <= 6'd0;
          end

          else begin
            block_y_idx <= block_y_idx + 6'd1;
          end

        end

        // If we're 32 wide and we're at the end of a line, loop around
        else if (block_x_idx == 3'd7 && first_sprite.CGX == WIDTH_32) begin
          block_x_idx <= 3'd0;
          if ((block_y_idx == 6'd15 && first_sprite.CGY == HEIGHT_16) ||
              (block_y_idx == 6'd31 && first_sprite.CGY == HEIGHT_32) ||
              (block_y_idx == 6'd63 && first_sprite.CGY == HEIGHT_64)) begin
            block_y_idx <= 6'd0;
          end
          else begin
            block_y_idx <= block_y_idx + 6'd1;
          end
        end

        // If we're not at the end of a line, only increase the fetch_count
        else begin
          block_x_idx <= block_x_idx + 3'd1;
        end


      end
    end
  end
  */









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


    else begin


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

    else begin

      // If we're at the end of the line:
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
    else if(EOL) begin
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
 


  logic [15:0] x_idx, y_idx;
  assign x_idx = ((HDW - H_cnt) << 3) + char_cycle;
  assign y_idx = VDW - V_cnt;

  // Logic to VCE telling it if we're currently in active display 
  logic        in_vdw; 
  assign in_vdw = (H_state == H_DISP && V_state == H_DISP);


  logic [2:0]  cycle_adjusted;
  assign cycle_adjusted = char_cycle + x_px_offset;
  
  logic [15:0] top, bot, left, right;
  assign top = first_sprite.y_pos - 64;
  assign bot = first_sprite.y_pos - 64 + 15'd32;
  assign left = first_sprite.x_pos - 8*(HSW + HDS);
  assign right = first_sprite.x_pos - 8*(HSW +HDS) + 15'd32;

  logic [4:0] sprite_idx_x;
  logic [5:0] sprite_idx_y;
  assign sprite_idx_x = x_idx - left;
  assign sprite_idx_y = y_idx - top;

  logic [15:0] cur_word_idx;
  logic [3:0] cur_pix;

  always_comb begin
    if (first_sprite.CGX == WIDTH_32) begin
      cur_word_idx = ((sprite_idx_y >> 4) << 7) + (sprite_idx_y % 16) + ((sprite_idx_x / 16) << 6);
    end else begin
      cur_word_idx = 0;
    end
  end

  assign cur_pix = {sprite_data[cur_word_idx     ][15 - (sprite_idx_x % 16)],
                    sprite_data[cur_word_idx + 16][15 - (sprite_idx_x % 16)],
                    sprite_data[cur_word_idx + 32][15 - (sprite_idx_x % 16)],
                    sprite_data[cur_word_idx + 48][15 - (sprite_idx_x % 16)]};

  always_comb begin
    VD = 0;
    if(in_vdw) begin //TODO: make this work correctly with new VSYNC
//      if ((left <= x_idx && x_idx < right && top <= y_idx && y_idx < bot) &&
      if ((left <= x_idx && x_idx < right && top <= y_idx && y_idx < bot) &&
          first_frame_done) begin
        VD[8] = 1;
        VD[7:4] = first_sprite.color;
        VD[3:0] = cur_pix;
      end else begin
      	VD[8]    = 0; //BG selected
      	VD[7:4]  = output_tile.palette_num;
      	VD[3:0]  = {output_tile.CG1[15 - cycle_adjusted],
                  output_tile.CG1[7 - cycle_adjusted], 
                  output_tile.CG0[15 - cycle_adjusted],
                  output_tile.CG0[7 - cycle_adjusted]};
      end
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
      sprite_word_count <= 0;
    end
    else begin
      if(do_BGfetch) begin
        case(char_cycle)
          2: begin
            tile_pipe[bg_wr_ptr].palette_num <= curbat.palette_num;
            tile_ptr <= (curbat.tile_index << 4) + fetch_row;
          end
          6: tile_pipe[bg_wr_ptr].CG0 <= MD_in;
          0: tile_pipe[bg_wr_ptr].CG1 <= MD_in;
        endcase
      end

      else if (do_SATfetch) begin
        case(char_cycle)
          2: first_sprite[63:48] <= MD_in;
          4: first_sprite[47:32] <= MD_in;
          6: first_sprite[31:16] <= MD_in;
          0: first_sprite[15:0]  <= MD_in;
        endcase
      end


      else if (do_Spritefetch) begin
        if (!(char_cycle & 3'd1)) begin
          sprite_data[sprite_word_count] <= MD_in; 
          sprite_word_count <= sprite_word_count + 1;
        end
      end

    end
  end
  
  logic [15:0] bat_ptr;
  //VRAM address control
  always_comb begin
    //assume we have dot width 00
    if(do_BGfetch) begin
      case(char_cycle)
        1: MA = bat_ptr;         //fetch BAT
        5: MA = tile_ptr;        //fetch CG0
        7: MA = tile_ptr + 12'h8;//fetch CG1 (check the offset here)
        default: MA = 0; //TODO: this should be specified by CPU
      endcase
    end

    else if (do_SATfetch) begin
      case(char_cycle)
        1: MA = SATB.data;
        3: MA = SATB.data + 1;
        5: MA = SATB.data + 2;
        7: MA = SATB.data + 3;
        default: MA = 0;
      endcase
    end

    else if (do_Spritefetch) begin
      if (char_cycle & 1) MA = (first_sprite.addr << 5) + sprite_word_count;
      else MA = 0;
    end

    else MA = 0;
  end

  //BAT pointer management
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      bat_ptr <= 0;
      cur_row <= 0; //TODO: actually handle this correctly
    end
    else begin

      if(V_state == V_WAIT) cur_row <= 0; //latch to 0 right before drawing

      // If we're just about to finish H_DISP mode, we should increment
      // our cur_row value
      else if(H_state == H_DISP && H_cnt == 0 && char_cycle == 7) begin
        cur_row <= cur_row + 1; //TODO: need to initialize cur_row
      end

      // If we're just about to finish HSYNC, set our bat_ptr
      else if(H_state == H_SYNC && H_cnt == 0 && char_cycle == 7) begin
        bat_ptr <= (x_tl_offset & x_mask) + 
                   ((((cur_row + y_start) >> 3) & y_mask) << y_shift);
      end

      // If we're doing a BG fetch, increment the BAT for the next fetch
      else if(do_BGfetch) begin
        if(char_cycle == 7) bat_ptr <= bat_ptr + 1;
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
    else begin

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
