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

/*
  AddressUnit au(.clock(clock),
                 .MA(MA)
                );
*/
  
  logic [15:0] data_in, data_out;
  logic [14:0] address;
  logic wren;

  /*
  RAMTest bram(.address(address),
               .clock(clock),
               .data(data_in),
               .wren(wren),
               .q(data_out));
   */


  /*
  logic [7:0] D_reg;
  DataBusBuffer(.clock(clock),
                .reset_N(reset_N),
                .D(DI),
                .ld(),
                .buf_data(D_reg)
               );
  */

  logic [2:0]  char_cycle; //current position in char cycle

  
  logic [15:0] HSR, HDR, VSR, VDR, VCR; //maybe these should be structs?
  logic [9:0] H_cnt; //horizontal position counter, reused for each phase

  h_state_t H_state; //state in horizontal line


  logic [15:0] BXR, BYR; //x and y positions to start reading from
  assign BXR = 0; //todo: scrolling
  assign BYR = 0;

                
  
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

  logic       do_BGfetch;
  assign HSW  = HSR[4:0]; //may not be minus 1, investigate
  assign HDS  = HSR[14:8];
  assign HDW  = HDR[6:0];
  assign HDE  = HDR[11:8];
                
  assign do_BGfetch = (H_state == H_DISP) || 
                      (H_state == H_WAIT && H_cnt == HDS - 2);

  
  assign HSYNC_n = (H_state == H_SYNC);
  //H_state control
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      H_cnt   <= HSW;
      H_state <= H_SYNC;
    end
    else begin
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

  localparam BG_PIPE_LEN = 3; //we always write 2 ahead of our read

  tile_line_t [BG_PIPE_LEN-1:0] tile_pipe; //need extra slot for current fetch
  tile_line_t output_tile;
  logic [$clog2(BG_PIPE_LEN)-1:0] bg_wr_ptr, bg_rd_ptr;
  
  assign output_tile = tile_pipe[bg_rd_ptr];
  
  //VDC -> VCE communications
  logic        in_vdw; //are we currently in active display?
  assign in_vdw = (H_state == H_DISP);
  
  always_comb begin
    VD = 0;
    if(in_vdw) begin
      VD[8]    = 0; //BG selected
      VD[7:4]  = output_tile.palette_num;
      VD[3:0] = {output_tile.CG1[char_cycle + 8],
                 output_tile.CG1[char_cycle], 
                 output_tile.CG0[char_cycle + 8],
                 output_tile.CG0[char_cycle]};
    end
  end
  
  logic [15:0] tile_ptr;
  bat_entry_t curbat;
  assign curbat  = MD_in;
  
  //background pipeline
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      for(int i = 0; i < BG_PIPE_LEN; i++) begin
        tile_pipe[i].palette_num <= 0;
        tile_pipe[i].CG0         <= 0;
        tile_pipe[i].CG1         <= 0;
      end
    end
    else begin
      if(do_BGfetch) begin
        case(char_cycle)
          2: begin
            tile_pipe[bg_wr_ptr].palette_num <= curbat.palette_num;
            tile_ptr                         <= curbat.tile_index << 4;
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
        1: MA = bat_ptr;         //fetch BAT
        5: MA = tile_ptr;        //fetch CG0
        7: MA = tile_ptr + 12'h8;//fetch CG1 (check the offset here)
        default: MA = 0; //TODO: this should be specified by CPU
      endcase
    end
    else MA = 0;
  end

  logic firstchar; //first character of a line
  //Counter management. Put counters here.
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      char_cycle <= 0;
      bat_ptr    <= 0;
      bg_wr_ptr  <= BG_PIPE_LEN-1; //first write of a line is garbage
      bg_rd_ptr  <= 0;
    end
    else begin
      char_cycle <= char_cycle + 1;
      if(in_vdw) begin
        if(char_cycle == 7) begin//TODO: handle scrolling correctly
          if(bg_wr_ptr == (BG_PIPE_LEN-1)) bg_wr_ptr <= 0;
          else bg_wr_ptr <= bg_wr_ptr + 1;
        end
      end
      else bg_rd_ptr <= 0; //TODO: jump to real start of line
      if(do_BGfetch) begin
        if(char_cycle == 7) bat_ptr <= bat_ptr + 1;
        if(char_cycle == 0) begin
          if(bg_wr_ptr == (BG_PIPE_LEN-1)) bg_wr_ptr <= 0;
          else bg_wr_ptr <= bg_wr_ptr + 1;
        end
      end
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
