`default_nettype none

/*
 * verilog model of HuC6260 VCE
 *
 * (C) 2018 Ford Seidel and Amolak Nagi
 */


`define SIMULATION
`define FAKE_CRAM

`ifdef SIMULATION


// Master clock here should be DOUBLE
// the PC engine's master clock.
module clk_divide(input logic clk, reset_N,
                  input logic [1:0] mode,
                  output logic clk_div);

  // Counter
  logic [1:0] counter;

  // All logic on clock because we're cool
  always_ff @(posedge clk, negedge reset_N) begin

    // Reset case
    if(~reset_N) begin
      clk_div <= 0;
      counter <= 0;
    end 

    else begin
      
      // Prepare to restart counter
      if (counter == 0) begin

        // Now that we've reached 0, flip our clock output
        clk_div <= ~clk_div;

        // Determine based on the mode input what we want to 
        // start our counter at. Always consider that there's
        // an inferred *2.
        unique case (mode)

          // 10.738635 MHz, divide by 4
          2'h2, 2'h3: begin
            counter <= 1;
          end

          // divide by 6
          2'h1: begin
            counter <= 2;
          end

          // divide by 8
          2'h0: begin
            counter <= 3;
          end

          default: begin
            counter <= 3;
          end

        endcase
      end

      else begin
        // Decrement our count, simple as that
        counter <= counter - 1;
      end

    end

  end
  
endmodule: clk_divide

`endif



module vce_HuC6260( input logic clock, reset_N,

                    // From VDC
                    input logic [8:0]  VD,
                    input logic        HSYN,
                    input logic        VSYN,



                    // To/From CPU
                    input logic [2:0]  A,
                    inout wire [8:0]   D, // Data bus, there's an 9th bit but 
                                         // supposedly it's unused
                    // Output to VGA


                    // output logic RGB_l,               // We won't be needing RGB_l/h
                    output logic [2:0] VIDEO_G,
                    output logic [2:0] VIDEO_R,
                    output logic [2:0] VIDEO_B,
                    // output logic RGB_h,



                    output logic       CK, // Likely the pixel clock

                    input logic        RD_n,
                    input logic        WR_n,
                    input logic        CS_n,

                    input              address_mode);    // 8 if hi, 16 if lo, forced to 8 bits


  // ALL of our color data
`ifdef FAKE_CRAM
  logic [15:0]     FAKE_CRAM[2**9-1:0]; //fseidel: make CRAM wide enough to support MEDNAFEN 
                                        //dumps natively
  logic [511:0][9:0]      CRAM;
  generate
    genvar                i;
    for(i = 0; i < 2**9; i++) begin
      assign CRAM[i] = FAKE_CRAM[i][8:0]; //fseidel: is there a better way than a generate block?
    end
  endgenerate
  
  //load CRAM image
  initial begin
    $readmemh("PAL.hex", FAKE_CRAM);
  end
  
`else
  logic [511:0][8:0]      CRAM;
`endif


  // CRAM FSM
  //enum logic [2:0] {IDLE, READ, WRITE, WAIT} state, next_state;


  
  logic clock_div;
  logic [1:0] mode;
  //clk_divide divide(.*);
  assign clock_div = clock;
  
  logic [7:0] CR;
  assign mode  = CR[1:0];

  
  logic [8:0] CDATA, addr, next_addr;

  logic [7:0] data_rd;

  assign D  = (~RD_n) ? data_rd : 8'bz;
  assign data_rd  = (A[0]) ? {7'h0, CRAM[8]} : CRAM[7:0];

  assign CDATA  = CRAM[VD];

  
  
  always_ff @(posedge clock, negedge reset_N) begin
    if(~reset_N) begin
      VIDEO_B <= 0;
      VIDEO_R <= 0;
      VIDEO_G <= 0;
    end
    else if(VD) begin
      VIDEO_B <= CDATA[2:0];
      VIDEO_R <= CDATA[5:3];
      VIDEO_G <= CDATA[8:6];
    end
    else begin //TODO: handle background color + blanking
      VIDEO_B <= 0;
      VIDEO_R <= 0;
      VIDEO_G <= 0;
    end
  end


  
  /*
  // Output and next-state generator
  always_comb begin
    next_addr         = addr;
    unique case (state)
      IDLE: begin
        if(~WR_n) next_state = WRITE;
        else if(~RD_n) next_state = READ;
        else next_state = IDLE;
      end

      READ: begin
        if(A[0]) next_addr = addr + 1'b1;
        next_state  = WAIT;
      end

      WRITE: begin
        if(A[0]) next_addr = addr + 1'b1;
        CRAM[addr] <= D;
        next_state  = WAIT;
      end

      WAIT: begin
        if(WR_n & RD_n) next_state = IDLE;
        else next_state = WAIT;
      end
      
    endcase
  end

  //should be on master clock
  always_ff @(posedge clk, negedge reset_N) begin
    if(~reset_N) begin
      state <= IDLE;
      addr <= 0;
    end
    else begin
      state <= next_state;
      addr <= next_addr;
    end
      
  end
   */
  
endmodule: vce_HuC6260;


