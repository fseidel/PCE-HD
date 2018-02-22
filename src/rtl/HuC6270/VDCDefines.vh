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


`endif /* VDC_DEFINES_VH */
