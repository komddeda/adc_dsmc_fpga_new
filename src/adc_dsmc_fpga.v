`timescale 1ns / 1ps
module adc_dsmc_fpga (
    // System clock and reset
    input  wire        sys_clk,
    input  wire        rest_n,

    // AD7606 interface
    output wire        adc_reset,
    output wire        adc_convst_a,
    output wire        adc_convst_b,
    output wire [2:0]  adc_os,
    output wire        adc_range,
    input  wire        adc_busy,
    output wire        adc_cs_n,
    output wire        adc_rd_n,
    input  wire [15:0] adc_data,

    // DSMC local bus interface
    input  wire        DSMC_CLKP,
    input  wire        DSMC_CLKN,
    input  wire        DSMC_CS,
    input  wire        DSMC_CS1,
    input  wire        DSMC_RESENT,
    inout  wire        DSMC_DQS,
    inout  wire        DSMC_DQS1,
    inout  wire [7:0]  DSMC_DQ,
    inout  wire [7:0]  DSMC_DQ1,

    // CPU interrupt (currently unused, keep for future use)
    output wire        dsmc_int
);
    // Bridge the external reset naming (rest_n) to the internal expectation (rst_n)
    wire rst_n = rest_n;

    adc_capture_dsmc u_adc_capture_dsmc (
        .sys_clk     (sys_clk),
        .rst_n       (rst_n),
        .adc_reset   (adc_reset),
        .adc_convst_a(adc_convst_a),
        .adc_convst_b(adc_convst_b),
        .adc_os      (adc_os),
        .adc_range   (adc_range),
        .adc_busy    (adc_busy),
        .adc_cs_n    (adc_cs_n),
        .adc_rd_n    (adc_rd_n),
        .adc_data    (adc_data),
        .DSMC_CLKP   (DSMC_CLKP),
        .DSMC_CLKN   (DSMC_CLKN),
        .DSMC_CS     (DSMC_CS),
        .DSMC_CS1    (DSMC_CS1),
        .DSMC_RESENT (DSMC_RESENT),
        .DSMC_DQS    (DSMC_DQS),
        .DSMC_DQS1   (DSMC_DQS1),
        .DSMC_DQ     (DSMC_DQ),
        .DSMC_DQ1    (DSMC_DQ1),
        .dsmc_int    (dsmc_int)
    );

endmodule
