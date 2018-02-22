`default_nettype none

/*
 * Standard Register module from 18-240
 *
 * (C) 2018 Ford Seidel and Amolak Nagi
 */

module Register
    #(parameter WIDTH=16)
    (input logic [WIDTH-1:0] D,
     output logic [WIDTH-1:0] Q,
     input logic load, clear, clock, reset_n);

    always_ff @(posedge clock, negedge reset_n)
        if (~reset_n)
            Q <= 0;
        else if (clear)
            Q <= 0;
        else if (load)
            Q <= D;

endmodule: Register
