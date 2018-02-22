/**
 * VDCDefines.vh
 * 
 * HuC6270 VDC
 *
 * (C) 2018 Ford Seidel and Amolak Nagi
 */

`ifndef VDC_DEFINES_VH_
`define VDC_DEFINES_VH_

// Incoming values on A[1:0] bus from CPU to VDC
typedef enum logic [1:0] {
    A_STATUS_REG = 2'd0,
    A_ADDR_REG   = 2'd1,
    A_DATA_LSB   = 2'd2,
    A_DATA_MSB   = 2'd3
} a_sel_t;

// Increment amount for writes to VRAM
typedef enum logic [1:0] {
    ADDR_INCR_1   = 2'd0,
    ADDR_INCR_32  = 2'd1,
    ADDR_INCR_64  = 2'd2,
    ADDR_INCR_128 = 2'd3
} addr_incr_t;

// 16-bit format of entry in the BAT
typedef struct packed {
  logic [3:0] palette_num; // Designated paletted for tile
  logic [11:0] tile_index; // Index for tile (VRAM address is tile_index << 5)
} bat_entry_t;



// Packed register definitions

// MWR - Memory-access Width Register [7:0]
typedef struct packed {
  logic CM;                         // CM (unknown - presumably 'Color Mode')
                                    // MagicKit: "Reserved: always set to 0"

  // These bits are called SCREEN; they control virtual map size
  logic virtual_screen_height;      // 0 = 256 pixels / 32 tiles
                                    // 1 = 512 pixels / 64 tiles
  logic [1:0] virtual_screen_width; // 00 = 256 pixels / 32 tiles
                                    // 01 = 512 pixels / 64 tiles
                                    // 1x = 1024 pixels / 128 tiles

  logic [1:0] sprite_dot_width;     // MagicKit: "Reserved - always set to 0"
  logic [1:0] VRAM_dot_width;       // MagicKit: "Reserved - always set to 0"
} MWR_t;



// $05 - CR - Control Register [12:0]
typedef struct packed {
  addr_incr_t addr_incr;              // Read/write address auto-increment
                                      // 00 = normal increment (+1)
                                      // 01 = +32
                                      // 10 = +64
                                      // 11 = +128
  logic DRAM_refresh;                 // MagicKit: "Reserved - always set to 0" DOUBLE CHECK THIS WITH FORD.
                                      // I think this changes with a select few games
  logic [1:0] disp_terminal_outputs;  // MagicKit: "Reserved - always set to 0"
  logic bgnd_en;                      // Background enable flag (1 = on)
  logic sprite_en;                    // Sprite enable flag (1 = on)
  logic [1:0] EX;                     // MagicKit: "Reserved - always set to 0" DOUBLE CHECK THIS
  logic vblank_int;                   // Vertical blanking interrupt enable flag
  logic scan_int;                     // Scanline interrupt enable flag
  logic sprite_overflow;              // Excess number of sprites in line (MagicKit says 16)
  logic sprite_collision;             // Sprite collision interrupt enable flag
} CR_t;



// $00 - MAWR - Memory Address Write Register [15:0]
// Specifies word offset into VRAM for writing.
// Every MSB write to VWR will store data at address specified by MAWR.
// When writes occur to MSB of VWR data, MAWR addr auto-increments
// by value specified in CR.addr_incr 
typedef struct packed {
  logic [15:0] addr;
} MAWR_t; 



// $01 - MARR - Memory Address Read Register [15:0]
// When upper byte of starting address is written into MARR, data are
// begun to be read from the starting address of the VRAM into this VRR.
// From then on, MARR is auto-incremented by CR.addr_incr
typedef struct packed {
  logic [15:0] addr;
} MARR_t;



// $02 - VRR - VRAM Data Read Register [15:0]
// When upper byte of starting address is written into MARR, data are
// begun to be read from the starting address of the VRAM into this VRR.
// From then on, MARR is auto-incremented.
typedef struct packed {
  logic [15:0] data;
} VRR_t;



// $02 - VWR - VRAM Data Write Register [15:0]
// When the MSB is written to, the latched LSB and new MSB data are stored to
// VRAM at address specified by MAWR. Writes to LSB will not initiate
// a write and will only store LSB in a latch.
typedef struct packed {
  logic [15:0] data;
} VWR_t;



// $03 - 
`endif /* VDC_DEFINES_VH */
