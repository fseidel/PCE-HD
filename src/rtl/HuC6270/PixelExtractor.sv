`default_nettype none

/*
 * Helper module to extract pixel data from planar bytes
 *
 * (C) 2018 Ford Seidel and Amolak Nagi
 */

module PixelExtractor(input logic [3:0][7:0] line_bytes,
                      output logic [7:0][3:0] pixel_indices
                     );

  // For each pixel index (of which there are 8), 
  // we will fetch each of the 4 bits from the 4 bytes of
  // the line. Pixel 0 (the leftmost pixel of the line) will fetch
  // its bits from the most significant bits of each pixel (bit 7).
  // Pixel 1 will fetch from bit 6, pixel 2 from bit 5, etc.
  genvar i;
  generate
    for (i = 0; i < 8; i++) begin : pix_extract

      assign pixel_indices[i] = {
                                  line_bytes[3][7-i],
                                  line_bytes[2][7-i],
                                  line_bytes[1][7-i],
                                  line_bytes[0][7-i]
                                };

    end : pix_extract
  endgenerate

endmodule: PixelExtractor
