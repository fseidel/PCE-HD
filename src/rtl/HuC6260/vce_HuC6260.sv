`default_nettype none

/*
 * verilog model of HuC6260 VCE
 *
 * (C) 2018 Ford Seidel and Amolak Nagi
 */


`define SIMULATION


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
          2'b2, 2'b3: begin
            counter <= 1;
          end

          // divide by 6
          2'b1: begin
            counter <= 2;
          end

          // divide by 8
          2'b0: begin
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
    // Defaults
    counter_next = count;
    mode_next = mode;
    clk_next = 0;
    
    if (counter == 0) begin

      unique case (mode) 

        2'b0: begin
          

        end

        2'b1: begin

        end

        2'b2, 2'b3: begin

          counter_next = 0;
          

        end

        default: begin

        end

      endcase
    end

    always_ff 

  end

endmodule: clk_divide

`endif



module vce_HuC6260( input logic clk, reset_N,

                    // To/From VDC
                    inout logic [8:0] D, // Data bus, there's an 8th bit but supposedly it's unused
                    inout logic [8:0] VD,
                    inout logic HSYN,
                    inout logic VSYN,



                    // To/From CPU
                    input logic [2:0] A,

                    // Output to VGA


                    // output logic RGB_l,               // We won't be needing RGB_l/h
                    output logic [7:0] VIDEO_G,
                    output logic [7:0] VIDEO_R,
                    output logic [7:0] VIDEO_B,
                    // output logic RGB_h,



                    output logic CK, // Likely the pixel clock


                    input logic WR_n,
                    input logic CS_n,

                    input address_mode);    // 8 if hi, 16 if lo, forced to 8 bits


  // ALL of our color data
  logic [511:0][8:0] CRAM;

  // CRAM FSM
  enum logic [2:0] {WAIT,} state, next_state;

  
  logic clk_div;
  logic [1:0] mode;
  clk_divide divide(.*);

  

  // Output and next-state generator
  always_comb begin
    
    unique case (state)

      WAIT: begin
      end

      default: begin
      end

    endcase
  end


endmodule: vce_HuC6260;


