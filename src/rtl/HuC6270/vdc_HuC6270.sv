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




  localparam num_sprites = 64;
  logic first_frame_done; 


  logic       do_BGfetch;            
  assign do_BGfetch = ((H_state == H_DISP) || 
                      (H_state == H_WAIT && H_cnt < 2)) && 
                      (V_state == V_DISP);

  // SAT Fetch
  logic do_SATfetch;
  assign do_SATfetch = ((V_state == V_SYNC));
  satb_entry_t satb_entries[num_sprites];
  logic [$clog2(num_sprites) - 1 : 0] SATfetch_cur_entry;


  // Sprite Crawl
  logic do_Spritecrawl;
  assign do_Spritecrawl = (V_state == V_WAIT && V_cnt == 0 && H_state == H_DISP) ||
                          (V_state == V_DISP && V_cnt != 0 && H_state == H_DISP);

  // Sprite Data Fetch
  logic do_Spritefetch;
  assign do_Spritefetch = (V_state == V_WAIT && V_cnt == 0 && H_state == H_END) ||
                          (V_state == V_DISP && V_cnt != 0 && 
                            (H_state == H_SYNC || (H_state == H_WAIT && H_cnt >= 2) || H_state == H_END)) ||
                          (V_state == V_DISP && V_cnt == 0 &&
                            (H_state == H_SYNC || (H_state == H_WAIT && H_cnt >= 2)));


  logic do_Linearrange;
  assign do_Linearrange = (V_state == V_DISP && H_state == H_WAIT && H_cnt < 2);
/*
  assign do_Spritefetch = ((H_state == H_SYNC) || (H_state == H_WAIT) || (H_state == H_END)) &&
                          (V_state == V_DISP) && 
                          ~do_BGfetch &&
                          first_frame_done;

  logic [15:0] sprite_data[num_sprites][512];
  logic [$clog2(num_sprites) - 1 : 0] cur_sprite_fetch;
*/

//  logic [9:0] sprite_word_count;

//  logic [9:0] sprite_num_words;
/*
  always_comb begin

    sprite_num_words = 10'd0;

    case (satb_entries[cur_sprite_fetch].CGY) 

      HEIGHT_16: begin

        case (satb_entries[cur_sprite_fetch].CGX)

          // 16 x 16
          WIDTH_16: sprite_num_words = 10'd64;

          // 32 x 16
          WIDTH_32: sprite_num_words = 10'd128;

        endcase

      end

      HEIGHT_32: begin

        case (satb_entries[cur_sprite_fetch].CGX)

          // 16 x 32
          WIDTH_16: sprite_num_words = 10'd128;

          // 32 x 32
          WIDTH_32: sprite_num_words = 10'd256;

        endcase

      end

      HEIGHT_64: begin

        case (satb_entries[cur_sprite_fetch].CGX)

          // 16 x 64
          WIDTH_16: sprite_num_words = 10'd256;

          // 32 x 64
          WIDTH_32: sprite_num_words = 10'd512;

        endcase

      end


    endcase

  end
*/

/*
  // Sprite fetching FSM
  logic [5:0] block_y_idx;
  logic [2:0]  block_x_idx;

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

                    SATfetch_cur_entry <= SATfetch_cur_entry + 1'd1;// If we're 32 wide and we're at the end of a line, loop around
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

  // Look at the next line (plus a 64 line offset as per spec)
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
  assign temp_y = next_line - cur_top;

  // Logic to determine sprite_base_addr
  always_comb begin

    if (~second_half) begin
      sprite_base_addr = (cur_entry.addr << 5) +
                         (temp_y % 16) +
                         ((temp_y / 16) * 128);
    end

    else begin
      sprite_base_addr = (cur_entry.addr << 5) +
                         (temp_y % 16) +
                         ((temp_y / 16) * 128) +
                         (64); // Add extra 64 to get to second half
    end

  end

  always_ff @(posedge clock, negedge reset_N) begin
 
    // When to reset OR clear to prep for next cycle,
    // which is when we're about to start H_DISP
    if ((~reset_N) ||
        (H_state == H_WAIT && H_cnt == 0)) begin
      satb_idx <= 7'd0;
      line_sprite_idx <= 5'd0;
      second_half <= 1'd0;
    end

    else begin

      if (do_Spritecrawl) begin

        // Check if there is space for more sprites or if 
        // we've already checked all of the sprites
        if (line_sprite_idx < 5'd16 && satb_idx < 7'd64) begin


          if (second_half) begin

//            if (satb_idx < 2)
//              $display("second half! %x", sprite_base_addr);

            // If we're on the second_half of a sprite, 
            // assume double-wide and copy the data over.
            line_sprite_info[line_sprite_idx].x_pos <= cur_entry.x_pos[9:0] + 10'd16;
            line_sprite_info[line_sprite_idx].x_invert <= cur_entry.x_invert;
            line_sprite_info[line_sprite_idx].y_invert <= cur_entry.y_invert;
            line_sprite_info[line_sprite_idx].SPBG <= cur_entry.SPBG;
            line_sprite_info[line_sprite_idx].base_addr <= sprite_base_addr;
            line_sprite_info[line_sprite_idx].color <= cur_entry.color;

            // Disable second_half
            second_half <= 1'd0;

            // Move onto next entry 
            line_sprite_idx <= line_sprite_idx + 5'd1;

            // Move onto next sprite
            satb_idx <= satb_idx + 7'd1;

          end // if (second_half) 

          // Check if the next line happens to intersect with this sprite
          else if (sprite_vertical_intersect) begin

//            if (satb_idx < 2)
//              $display("intersect! line_sprite_idx: %d, satb_idx: %d, addr: %x, temp_y: %d", line_sprite_idx, satb_idx, sprite_base_addr, temp_y);


            // Save the Sprite information
            if (cur_entry.CGX == WIDTH_16) begin

              line_sprite_info[line_sprite_idx].x_pos <= cur_entry.x_pos[9:0];
              line_sprite_info[line_sprite_idx].x_invert <= cur_entry.x_invert;
              line_sprite_info[line_sprite_idx].y_invert <= cur_entry.y_invert;
              line_sprite_info[line_sprite_idx].SPBG <= cur_entry.SPBG;
              line_sprite_info[line_sprite_idx].base_addr <= sprite_base_addr;          
              line_sprite_info[line_sprite_idx].color <= cur_entry.color;

              // Move onto the next entry
              line_sprite_idx <= line_sprite_idx + 5'd1;

              // Move onto the next sprite
              satb_idx <= satb_idx + 7'd1;

            end // if (cur_entry.CGX == WIDTH_16)

            else begin

              line_sprite_info[line_sprite_idx].x_pos <= cur_entry.x_pos[9:0];
              line_sprite_info[line_sprite_idx].x_invert <= cur_entry.x_invert;
              line_sprite_info[line_sprite_idx].y_invert <= cur_entry.y_invert;
              line_sprite_info[line_sprite_idx].SPBG <= cur_entry.SPBG;
              line_sprite_info[line_sprite_idx].base_addr <= sprite_base_addr;          
              line_sprite_info[line_sprite_idx].color <= cur_entry.color;

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


    end // else 

  end


  /////////////////////////////////////////////////////
  // LINE ARRANGE FSM

  logic [3:0] sprite_arrange_idx;
  logic [4:0] disp_cycle_sprite_idx[16];

  always_ff @(posedge clock, negedge reset_N) begin

    if (~reset_N || 
        (H_state == H_WAIT && H_cnt == 2)) begin
      sprite_arrange_idx <= 0;
      disp_cycle_sprite_idx[0] <= 5'd0;
      disp_cycle_sprite_idx[1] <= 5'd0;
      disp_cycle_sprite_idx[2] <= 5'd0;
      disp_cycle_sprite_idx[3] <= 5'd0;
      disp_cycle_sprite_idx[4] <= 5'd0;
      disp_cycle_sprite_idx[5] <= 5'd0;
      disp_cycle_sprite_idx[6] <= 5'd0;
      disp_cycle_sprite_idx[7] <= 5'd0;
      disp_cycle_sprite_idx[8] <= 5'd0;
      disp_cycle_sprite_idx[9] <= 5'd0;
      disp_cycle_sprite_idx[10] <= 5'd0;
      disp_cycle_sprite_idx[11] <= 5'd0;
      disp_cycle_sprite_idx[12] <= 5'd0;
      disp_cycle_sprite_idx[13] <= 5'd0;
      disp_cycle_sprite_idx[14] <= 5'd0;
      disp_cycle_sprite_idx[15] <= 5'd0;
    end

    else begin

      if (do_Linearrange) begin

        // If the sprite idx we're on is within the bounds of the
        // number of sprites on the lines
        if (sprite_arrange_idx < line_sprite_idx) begin

          disp_cycle_sprite_idx[(line_sprite_data[sprite_arrange_idx].info.x_pos-32)/16][4] <= 1'b1;
          disp_cycle_sprite_idx[(line_sprite_data[sprite_arrange_idx].info.x_pos-32)/16][3:0] <= sprite_arrange_idx;

        end

        sprite_arrange_idx <= sprite_arrange_idx + 4'd1;

      end

    end

  end

  // Stuff from FORD
  /* 

  logic [7:0] sat_count, total_count;

  satb_entry_t cur_entry;
  sat_entry_t  line_sat_entries[16];
  logic second_half;
  assign cur_entry = satb_entries[sat_count];
  logic [5:0] cur_height, cur_width;
  assign cur_height = (cur_entry.CGY == HEIGHT_16) ?  6'd16 :
                      (cur_entry.CGY == HEIGHT_32) ?  6'd32 : 
                                                      6'd64;

  assign cur_width = (cur_entry.CGX == WIDTH_16) ? 6'd16 :
                                                   6'd32;

  logic [9:0] cur_x_pos, cur_y_pos;
  assign cur_x_pos = cur_entry.x_pos[9:0];
  assign cur_y_pos = cur_entry.y_pos[9:0];

  logic [9:0] next_line;
  assign next_line = y_idx + 10'd64 + 10'd1;

  logic [15:0] temp_y;
  logic [15:0] corrected_ptr;
  always_comb begin

    corrected_ptr = cur_entry.addr;

    if (cur_entry.CGX != WIDTH_16) begin
      corrected_ptr[0] = second_half ^ cur_entry.x_invert;
    end

    temp_y = next_line - cur_y_pos;
    
    unique case (cur_entry.CGY)

      HEIGHT_16: begin
        
      end

      HEIGHT_32: begin
        corrected_ptr[1] = (temp_y[4] ^ cur_entry.y_invert);
      end

      HEIGHT_64: begin

        corrected_ptr[2:1] = (temp_y[5:4] ^ {cur_entry.y_invert, cur_entry.y_invert});

      end

    endcase

  end

  always_ff @(posedge clock, negedge reset_n) begin

    if (~reset_N) begin
      sat_count <= 8'd0;
      total_count <= 8'd0;
      second_half <= 1'b0;
    end

    else begin

      if (V_state == VDW) begin
        if (total_count < 8'd16 && sat_count < 8'd64) begin

          // For non y-invert
          if (cur_y_pos <= next_line && 
              cur_y_pos + cur_height > next_line) begin

            // First copy the whole entry
            line_sat_entries[sat_count] <= cur_entry;
            line_sat_entries[sat_count].addr <= corrected_ptr;
            total_count <= total_count + 8'd1;

            // If normal width sprite:
            if (cur_entry.CGX == WIDTH_16) begin
              sat_count <= sat_count + 8'd1;
              
            end

            // If double wide:
            else begin

              if (~second_half) begin
                line_sat_entries[sat_count].CGX 
                second_half <= 1'b1;
              end 

              else begin
                second_half <= 1'b0;
                sat_count <= sat_count + 8'd1;
              end
            end

          end 


        end
      end

    end

  end

*/


  localparam BG_PIPE_LEN = 3; //we always write 2 ahead of our read

  tile_line_t [BG_PIPE_LEN-1:0] tile_pipe; //need extra slot for current fetch
  tile_line_t output_tile;
  logic [$clog2(BG_PIPE_LEN)-1:0] bg_wr_ptr, bg_rd_ptr;
  
  assign output_tile = tile_pipe[bg_rd_ptr];
 



  // Logic to VCE telling it if we're currently in active display 
  logic        in_vdw; 
  assign in_vdw = (H_state == H_DISP && V_state == H_DISP);


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

  line_sprite_data_t line_sprite_data[16];

  always_ff @(posedge clock, negedge reset_N) begin

    if (~reset_N ||
        (H_state == H_DISP && H_cnt == 0)) begin
      cur_sprite_fetch <= 4'd0;
      sprite_word_idx <= 2'd0;
      prev_sprite_fetch <= 4'd0;
      prev_sprite_word_idx <= 2'd0;
      first_Spritefetch_cycle <= 1'd1;
    end

    else begin
      first_Spritefetch_cycle <= 1'd0;

      prev_sprite_fetch <= cur_sprite_fetch;
      prev_sprite_word_idx <= sprite_word_idx;

      if (do_Spritefetch) begin
//        $display("%d %d, base_addr:%x, MA:%x y_idx:%d", cur_sprite_fetch, sprite_word_idx, line_sprite_info[cur_sprite_fetch].base_addr, MA, y_idx);
        // Copy over the info
        if (sprite_word_idx == 2'd0) begin
          line_sprite_data[cur_sprite_fetch].info <= line_sprite_info[cur_sprite_fetch];
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


  logic [3:0] tile_pix;
  assign tile_pix = {output_tile.CG1[15 - cycle_adjusted],
                  output_tile.CG1[7 - cycle_adjusted], 
                  output_tile.CG0[15 - cycle_adjusted],
                  output_tile.CG0[7 - cycle_adjusted]};


  always_comb begin
    VD = 0;
    if(in_vdw) begin //TODO: make this work correctly with new VSYNC

/*
//      if ((left <= x_idx && x_idx < right && top <= y_idx && y_idx < bot) &&
      if ((left[1] <= x_idx && x_idx < right[1] && top[1] <= y_idx && y_idx < bot[1]) &&
          first_frame_done) begin

        if (tile_pix != 0) begin
          VD[8] = 0;
          VD[7:4] = output_tile.palette_num;
          VD[3:0] = tile_pix;
        end else begin

        VD[8] = 1;
        VD[7:4] = satb_entries[1].color;
        VD[3:0] = cur_pix[1];
        end
      end else if ((left[0] <= x_idx && x_idx < right[0] && top[0] <= y_idx && y_idx < bot[0]) &&
          first_frame_done) begin

        if (tile_pix != 0) begin
          VD[8] = 0;
          VD[7:4] = output_tile.palette_num;
          VD[3:0] = tile_pix;
        end else begin
        VD[8] = 1;
        VD[7:4] = satb_entries[0].color;
        VD[3:0] = cur_pix[0]; 
        end
      end else begin
*/
      	VD[8]    = 0; //BG selected
      	VD[7:4]  = output_tile.palette_num;
      	VD[3:0]  = tile_pix;
        if (tile_pix == 0)
          VD[7:4] = 0;

        if (disp_cycle_sprite_idx[x_idx / 16][4]) begin

          VD[8] = 1;
          VD[7:4] = line_sprite_data[disp_cycle_sprite_idx[x_idx/16][3:0]].info.color;
          VD[3:0] = {
            line_sprite_data[disp_cycle_sprite_idx[x_idx/16][3:0]].data[3][15-(x_idx%16)],
            line_sprite_data[disp_cycle_sprite_idx[x_idx/16][3:0]].data[2][15-(x_idx%16)],
            line_sprite_data[disp_cycle_sprite_idx[x_idx/16][3:0]].data[1][15-(x_idx%16)],
            line_sprite_data[disp_cycle_sprite_idx[x_idx/16][3:0]].data[0][15-(x_idx%16)]
          };


        end    
/*
        if (line_sprite_data[0].info.x_pos - 32 <= x_idx && x_idx < line_sprite_data[0].info.x_pos - 32 + 16 && line_sprite_idx != 0) begin
          VD[8] = 1;
          VD[7:4] = line_sprite_data[0].info.color;
          VD[3:0] = {
            line_sprite_data[0].data[0][15-(x_idx%16)],
            line_sprite_data[0].data[1][15-(x_idx%16)],
            line_sprite_data[0].data[2][15-(x_idx%16)],
            line_sprite_data[0].data[3][15-(x_idx%16)]
          };
        end
        if (line_sprite_data[1].info.x_pos - 32 <= x_idx && x_idx < line_sprite_data[1].info.x_pos - 32 + 16 && line_sprite_idx > 0) begin
          VD[8] = 1;
          VD[7:4] = line_sprite_data[1].info.color;
          VD[3:0] = {
            line_sprite_data[1].data[0][15-(x_idx%16)],
            line_sprite_data[1].data[1][15-(x_idx%16)],
            line_sprite_data[1].data[2][15-(x_idx%16)],
            line_sprite_data[1].data[3][15-(x_idx%16)]
          };
        end
        if (line_sprite_data[2].info.x_pos - 32 <= x_idx && x_idx < line_sprite_data[2].info.x_pos - 32 + 16 && line_sprite_idx > 1) begin
          VD[8] = 1;
          VD[7:4] = line_sprite_data[2].info.color;
          VD[3:0] = {
            line_sprite_data[2].data[0][15-(x_idx%16)],
            line_sprite_data[2].data[1][15-(x_idx%16)],
            line_sprite_data[2].data[2][15-(x_idx%16)],
            line_sprite_data[2].data[3][15-(x_idx%16)]
          };
        end
*/
    end
  end


 
  logic [15:0] tile_ptr;
  logic [2:0]  fetch_row;
  bat_entry_t curbat;
  assign curbat  = MD_in;
  assign fetch_row = y_px_offset + cur_row;
  logic [15:0] bat_ptr;
  
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
          2: satb_entries[SATfetch_cur_entry][63:48] <= MD_in;
          4: satb_entries[SATfetch_cur_entry][47:32] <= MD_in;
          6: satb_entries[SATfetch_cur_entry][31:16] <= MD_in;
          0: begin
            satb_entries[SATfetch_cur_entry][15:0]  <= MD_in;
            SATfetch_cur_entry <= SATfetch_cur_entry + 1;
          end
        endcase
      end


      else if (do_Spritefetch) begin
//        if (~first_Spritefetch_cycle)
          line_sprite_data[prev_sprite_fetch].data[prev_sprite_word_idx] <= MD_in; 
      end

    end
  end
  
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
        1: MA = SATB.data     + (4 * SATfetch_cur_entry);
        3: MA = SATB.data + 1+ (4 * SATfetch_cur_entry);
        5: MA = SATB.data + 2+ (4 * SATfetch_cur_entry);
        7: MA = SATB.data + 3+ (4 * SATfetch_cur_entry);
        default: MA = 0;
      endcase
    end

    else if (do_Spritefetch) begin
      MA = line_sprite_info[cur_sprite_fetch].base_addr + (16'd16 * sprite_word_idx);
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

  


endmodule : vdc_HuC6270;


