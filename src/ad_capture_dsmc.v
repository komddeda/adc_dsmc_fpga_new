`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
//  Title      : Top‑level ADC capture with automatic sampling
//  File       : adc_capture_dsmc_auto.v
//  Description: This top‑level module ties together the AD7606 capture front
//               end, the memory write control logic and the DSMC local bus
//               interface.  Unlike the original design, there is no software
//               controlled enable signal: the ADC continuously samples and
//               writes data to the dual‑port RAM.  The DSMC master can read
//               the latest samples from the reserved ADC window (0x3F8–0x3FC)
//               or read historical data from the RAM address space.  No
//               interrupt is generated; polling mode is used.
//
//  The module instantiates the modified sc1467_auto and adc_control_auto
//  modules which do not depend on an external enable.  It also instantiates
//  the dual‑port RAM and the DSMC interface (unchanged from the original
//  design).
//
//  Ports are identical to the original adc_capture_dsmc module except that
//  there is no adc_enable interface.
// -----------------------------------------------------------------------------

module adc_capture_dsmc_auto (
    // FPGA system clock and reset
    input  wire        sys_clk,      // System clock (e.g., 100 MHz)
    input  wire        rst_n,        // Global reset, active low

    // AD7606 ADC interface signals
    output wire        adc_reset,    // AD7606 RESET (active high, tied to ~rst_n)
    output wire        adc_convst_a, // AD7606 CONVST_A
    output wire        adc_convst_b, // AD7606 CONVST_B
    output wire [2:0]  adc_os,       // AD7606 oversampling setting
    output wire        adc_range,    // AD7606 range select
    input  wire        adc_busy,     // AD7606 BUSY indicator
    output wire        adc_cs_n,     // AD7606 Chip Select (active low)
    output wire        adc_rd_n,     // AD7606 RD (active low)
    input  wire [15:0] adc_data,     // AD7606 16‑bit parallel data output

    // DSMC local bus interface signals (to CPU)
    input  wire        DSMC_CLKP,    // DSMC differential clock (positive)
    input  wire        DSMC_CLKN,    // DSMC differential clock (negative, not used internally)
    input  wire        DSMC_CS,      // DSMC chip select (active low)
    input  wire        DSMC_CS1,     // DSMC second chip select (if any)
    input  wire        DSMC_RESENT,  // DSMC reset signal from CPU (if provided)
    inout  wire        DSMC_DQS,     // DSMC data strobe
    inout  wire        DSMC_DQS1,    // DSMC data strobe (upper byte)
    inout  wire [7:0]  DSMC_DQ,      // DSMC data bus [7:0]
    inout  wire [7:0]  DSMC_DQ1,     // DSMC data bus [15:8]
    output wire        dsmc_int      // DSMC interrupt request to CPU (unused, tied low)
);

    // Wires for inter‑module connections
    wire adc_read_done;
    wire [15:0] adc_ch1, adc_ch2, adc_ch3, adc_ch4;
    wire [15:0] adc_ch5, adc_ch6, adc_ch7, adc_ch8;

    // Wires for DSMC localbus <-> memory interface
    wire [13:0] lb_wr_addr;
    wire [31:0] lb_wr_data;
    wire        lb_wr_en;
    wire [13:0] lb_rd_addr;
    wire        lb_rd_clk_en;
    wire [31:0] lb_rd_data;

    // Instantiate AD7606 interface and capture module (always enabled)
    sc1467_auto #(
        .FPGA_CLOCK_FREQ   (100),   // (MHz) adjust as needed
        .ADC_SAMPLING_RATE (20)    // (kSPS) adjust as needed
    ) u_sc1467_auto (
        .sys_clk            (sys_clk),
        .rst_n              (rst_n),
        .adc_reset          (adc_reset),
        .adc_convst_a       (adc_convst_a),
        .adc_convst_b       (adc_convst_b),
        .adc_os             (adc_os),
        .adc_range          (adc_range),
        .adc_busy           (adc_busy),
        .adc_cs_n           (adc_cs_n),
        .adc_rd_n           (adc_rd_n),
        .adc_data           (adc_data),
        // ADC channel outputs
        .adc_ch1_data_out   (adc_ch1),
        .adc_ch2_data_out   (adc_ch2),
        .adc_ch3_data_out   (adc_ch3),
        .adc_ch4_data_out   (adc_ch4),
        .adc_ch5_data_out   (adc_ch5),
        .adc_ch6_data_out   (adc_ch6),
        .adc_ch7_data_out   (adc_ch7),
        .adc_ch8_data_out   (adc_ch8),
        .adc_read_done      (adc_read_done)
    );

    // Instantiate ADC control module for memory writing (no enable input)
    adc_control_auto u_adc_control (
        .sys_clk        (sys_clk),
        .rst_n          (rst_n),
        .adc_read_done  (adc_read_done),
        .adc_ch1        (adc_ch1),
        .adc_ch2        (adc_ch2),
        .adc_ch3        (adc_ch3),
        .adc_ch4        (adc_ch4),
        .adc_ch5        (adc_ch5),
        .adc_ch6        (adc_ch6),
        .adc_ch7        (adc_ch7),
        .adc_ch8        (adc_ch8),
        // Memory write port outputs
        .mem_wr_addr    (lb_wr_addr),
        .mem_wr_data    (lb_wr_data),
        .mem_wr_en      (lb_wr_en)
    );

    // Instantiate DSMC local bus interface module (connects to CPU memory bus).
    // The DSMC interface itself manages address decoding and will overlay the
    // latest ADC samples at addresses 0x3F8–0x3FC.  This module does not
    // provide an enable output in this configuration.
    dram_dsmc_localbus u_dsmc_bus (
        .sys_clk        (sys_clk),
        .DSMC_CLKP      (DSMC_CLKP),
        .DSMC_CLKN      (DSMC_CLKN),
        .DSMC_CS        (DSMC_CS),
        .DSMC_CS1       (DSMC_CS1),
        .DSMC_RESENT    (DSMC_RESENT),
        .DSMC_DQS       (DSMC_DQS),
        .DSMC_DQS1      (DSMC_DQS1),
        .DSMC_DQ        (DSMC_DQ),
        .DSMC_DQ1       (DSMC_DQ1),
        .rest_n         (rst_n),
        // External memory interface connections
        .wr_addr_ext    (lb_wr_addr),
        .wr_data_ext    (lb_wr_data),
        .wr_en_ext      (lb_wr_en),
        .rd_addr_ext    (lb_rd_addr),
        .rd_data_ext    (lb_rd_data),
        .rd_clk_en_ext  (lb_rd_clk_en)
        // Note: no adc_enable_out port is connected in this configuration
    );

    // Instantiate dual‑port RAM for data buffering between ADC (write) and CPU (read)
    dsmc_dram #(
        .WR_ADDR_WIDTH  (14),
        .WR_DATA_WIDTH  (32),
        .RD_ADDR_WIDTH  (14),
        .RD_DATA_WIDTH  (32),
        .RESET_TYPE     ("ASYNC"),
        .RD_CLK_EN      (1)
    ) u_dsmc_ram (
        .wr_data    (lb_wr_data),
        .wr_addr    (lb_wr_addr),
        .wr_en      (lb_wr_en),
        .wr_clk     (sys_clk),
        .wr_rst     (~rst_n),       // asynchronous reset for write domain
        .rd_addr    (lb_rd_addr),
        .rd_data    (lb_rd_data),
        .rd_clk     (DSMC_CLKP),    // use positive DSMC clock for read domain
        .rd_clk_en  (lb_rd_clk_en),
        .rd_rst     (~rst_n)        // asynchronous reset for read domain
    );

    // Disable interrupt output (polling mode)
    assign dsmc_int = 1'b0;

endmodule
