// =============================================================================
//  Title      : Portable dual‑port memory for DSMC DRAM interface
//  File       : dsmc_dram.v
//  Description: This module replaces the vendor specific Pango memory IP
//               (ipml_sdpram_v1_7_dsmc_dram) used in the original
//               dsmc_rockchip project.  It implements a simple dual‑port RAM
//               with separate read and write clocks using plain Verilog.
//
//  The goal of this version is to provide the same functionality as the
//  original Pango IP without relying on vendor specific primitives.  It
//  therefore targets portability across FPGA vendors such as Gowin.  The
//  module parameters have been retained to ease drop‑in replacement in the
//  existing design, however only a subset of them are honoured because
//  generic Verilog does not support features like byte enable or clock
//  polarity inversion in a vendor agnostic way.  Where unsupported
//  parameters are present they are ignored but left in place to avoid
//  breaking existing parameter assignments.
//
//  Note: This memory is synchronous on both read and write sides.  The read
//        address and data are registered on the rising edge of rd_clk when
//        rd_clk_en is asserted.  Likewise the write occurs on the rising
//        edge of wr_clk when wr_en is asserted.  If asynchronous reset is
//        enabled via the RESET_TYPE parameter ("ASYNC") the memory and read
//        data registers are cleared when wr_rst or rd_rst are asserted.  For
//        synchronous reset ("SYNC") the registers are cleared on the next
//        clock edge.
//
//  Copyright (c) 2025.  This code is provided without warranty of any
//  kind and may be used and modified freely.
// =============================================================================

module dsmc_dram #(
    // Address and data widths for write and read ports
    parameter integer WR_ADDR_WIDTH  = 14,
    parameter integer WR_DATA_WIDTH  = 32,
    parameter integer RD_ADDR_WIDTH  = 14,
    parameter integer RD_DATA_WIDTH  = 32,

    // Optional output register.  When set to 1 the read data is
    // registered; otherwise it is combinatorial.  Only the registered
    // mode is implemented here.
    parameter integer OUTPUT_REG     = 0,
    // Read output enable (not implemented)
    parameter integer RD_OCE_EN      = 0,
    // Read clock polarity inversion (not implemented)
    parameter integer RD_CLK_OR_POL_INV = 0,
    // Reset type: "ASYNC", "SYNC" or "ASYNC_RESET_SYNC_RELEASE".
    // Only ASYNC and SYNC are recognised here.
    parameter [8*5-1:0] RESET_TYPE     = "ASYNC",
    // Power optimisation flag (ignored)
    parameter integer POWER_OPT      = 0,
    // Initialisation file (ignored)
    parameter [8*4-1:0] INIT_FILE      = "NONE",
    // Initialisation format (ignored)
    parameter [8*3-1:0] INIT_FORMAT    = "BIN",
    // Write byte enable flag (ignored)
    parameter integer WR_BYTE_EN     = 0,
    // Byte enable width (ignored)
    parameter integer BE_WIDTH       = 1,
    // Read byte enable width (ignored)
    parameter integer RD_BE_WIDTH    = 1,
    // Byte size (ignored)
    parameter integer BYTE_SIZE      = 8,
    // Enable initial contents (ignored)
    parameter integer INIT_EN        = 0,
    // Same read/write width flag (ignored)
    parameter integer SAMEWIDTH_EN   = 1,
    // Write clock enable flag (ignored)
    parameter integer WR_CLK_EN      = 0,
    // Read clock enable flag (when 0 read data is always updated)
    parameter integer RD_CLK_EN      = 1,
    // Write address strobe enable (ignored)
    parameter integer WR_ADDR_STROBE_EN = 0,
    // Read address strobe enable (ignored)
    parameter integer RD_ADDR_STROBE_EN = 0
)(
    // Write port
    input  wire [WR_DATA_WIDTH-1:0] wr_data,
    input  wire [WR_ADDR_WIDTH-1:0] wr_addr,
    input  wire                     wr_en,
    input  wire                     wr_clk,
    input  wire                     wr_rst,

    // Read port
    input  wire [RD_ADDR_WIDTH-1:0] rd_addr,
    output reg  [RD_DATA_WIDTH-1:0] rd_data,
    input  wire                     rd_clk,
    input  wire                     rd_clk_en,
    input  wire                     rd_rst
);

    // -------------------------------------------------------------------------
    // Memory declaration
    // A single memory array is used for both read and write ports.  The size
    // of the memory is determined by the larger of the read and write address
    // widths.  If the widths differ the unused upper bits will be ignored.
    // -------------------------------------------------------------------------
    localparam integer MEM_DEPTH = (1 << (WR_ADDR_WIDTH > RD_ADDR_WIDTH ? WR_ADDR_WIDTH : RD_ADDR_WIDTH));
    reg [WR_DATA_WIDTH-1:0] mem [0:MEM_DEPTH-1];

    integer i;

    // Optional asynchronous initialisation of memory from INIT_FILE could be
    // added here if desired.  For portability this feature is not
    // implemented.

    // Write process
    always @(posedge wr_clk or posedge wr_rst) begin
        if (RESET_TYPE == "ASYNC") begin
            if (wr_rst) begin
                // Clear memory on asynchronous reset.  This loop is
                // synthesised for simulation only; most FPGA tools remove
                // initialisation logic for real hardware.  If clearing the
                // memory on reset is undesired or unsupported it may be
                // commented out.
                // synopsys translate_off
                for (i = 0; i < MEM_DEPTH; i = i + 1) begin
                    mem[i] <= {WR_DATA_WIDTH{1'b0}};
                end
                // synopsys translate_on
                // synthesis translate_on
            end else begin
                if (wr_en) begin
                    mem[wr_addr] <= wr_data;
                end
            end
        end else begin
            // Synchronous reset
            if (wr_rst) begin
                // synopsys translate_off
                for (i = 0; i < MEM_DEPTH; i = i + 1) begin
                    mem[i] <= {WR_DATA_WIDTH{1'b0}};
                end
                // synopsys translate_on
                // synthesis translate_on
            end else if (wr_en) begin
                mem[wr_addr] <= wr_data;
            end
        end
    end

    // Read process.  Read address is registered and data is presented at
    // rd_data on the next rising clock edge when rd_clk_en is asserted.  If
    // rd_clk_en is deasserted the previous value of rd_data is held.
    always @(posedge rd_clk or posedge rd_rst) begin
        if (RESET_TYPE == "ASYNC") begin
            if (rd_rst) begin
                rd_data <= {RD_DATA_WIDTH{1'b0}};
            end else begin
                if (RD_CLK_EN != 0) begin
                    // When RD_CLK_EN is zero the clock enable is ignored and
                    // rd_data is always updated.
                    if (rd_clk_en) begin
                        rd_data <= mem[rd_addr];
                    end
                end else begin
                    rd_data <= mem[rd_addr];
                end
            end
        end else begin
            // Synchronous reset
            if (rd_rst) begin
                rd_data <= {RD_DATA_WIDTH{1'b0}};
            end else if (RD_CLK_EN != 0) begin
                if (rd_clk_en) begin
                    rd_data <= mem[rd_addr];
                end
            end else begin
                rd_data <= mem[rd_addr];
            end
        end
    end

endmodule

