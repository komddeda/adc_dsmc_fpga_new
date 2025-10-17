// =============================================================================
//  Title      : DSMC local bus with AD7606 ADC integration
//  File       : dram_dsmc_localbus_with_adc.v
//  Description: This module extends the original dram_dsmc_localbus design by
//               instantiating an AD7606 data acquisition core (sc1467) and
//               exposing the captured ADC samples to the DSMC master via
//               dedicated address locations.  New samples overwrite the
//               previous values and a status flag indicates when fresh data
//               is available.  When the master reads from the reserved
//               addresses the ADC channel data is returned instead of the
//               contents of the on‑chip dual port RAM.
//
//               Address mapping for ADC data (word address space):
//                 ADC_ADDR_BASE     : {CH2, CH1}  (bits [31:16] = CH2, [15:0] = CH1)
//                 ADC_ADDR_BASE + 1 : {CH4, CH3}
//                 ADC_ADDR_BASE + 2 : {CH6, CH5}
//                 ADC_ADDR_BASE + 3 : {CH8, CH7}
//                 ADC_ADDR_BASE + 4 : {31'b0, valid}  (bit0 = 1 when data is fresh)
//
//  Note: The remainder of the design is based on the dram_dsmc_localbus
//        implementation from the dsmc_fpga_localbus project.  The DSMC
//        interface logic is unchanged except that read data is multiplexed
//        between the dual port RAM and the ADC registers.  External logic
//        must avoid writing to the ADC reserved addresses to prevent
//        corruption of the captured samples.
//
//  Copyright (c) 2025.  This code is provided without warranty of any
//  kind and may be used and modified freely.
// =============================================================================

// ------------------------------
// Bidirectional IO buffer
// ------------------------------
(* keep_hierarchy = "yes" *) module GTP_IOBUF #(
    parameter IOSTANDARD     = "DEFAULT",
    parameter SLEW_RATE      = "FAST",
    parameter DRIVE_STRENGTH = "8"
) (
    inout  wire IO,  // Pad
    input  wire I,   // Core -> Pad
    output wire O,   // Pad -> Core
    input  wire T    // 1: High-Z pad, 0: drive
);
    assign IO = T ? 1'bz : I;
    assign O  = IO;
endmodule

// ------------------------------
// Input DDR register (edge capture)
// Q0 captures at posedge, Q1 at negedge
// ------------------------------
(* keep_hierarchy = "yes" *) module GTP_IDDR_E2 #(
    parameter GRS_EN = "TRUE"
) (
    output reg  Q0,
    output reg  Q1,
    input  wire CLK,
    input  wire D,
    input  wire RS   // active-high reset
);
    always @(posedge CLK or posedge RS) begin
        if (RS) Q0 <= 1'b0;
        else    Q0 <= D;
    end
    always @(negedge CLK or posedge RS) begin
        if (RS) Q1 <= 1'b0;
        else    Q1 <= D;
    end
endmodule

// ------------------------------
// Output DDR register
// Drives Q with D0 on posedge and D1 on negedge.
// T acts like tri-enable (mirrored to TQ).
// ------------------------------
(* keep_hierarchy = "yes" *) module GTP_ODDR_E2 #(
    parameter GRS_EN = "TRUE"
) (
    output wire Q,    // DDR output toward pad
    output wire TQ,   // Tri-state indicator (mirrors T)
    input  wire CLK,
    input  wire D0,   // Data for posedge
    input  wire D1,   // Data for negedge
    input  wire RS,   // active-high reset
    input  wire T     // 1: tri/disable, 0: enable
);
    reg q_pos /* synthesis keep */;
    reg q_neg /* synthesis keep */;

    always @(posedge CLK or posedge RS) begin
        if (RS) q_pos <= 1'b0;
        else    q_pos <= D0;
    end
    always @(negedge CLK or posedge RS) begin
        if (RS) q_neg <= 1'b0;
        else    q_neg <= D1;
    end

    // Mux between edge-captured values. Many tools will map this to native ODDR.
    // Using CLK as select is a common portable idiom for DDR emulation.
    assign Q  = (CLK ? q_neg : q_pos);
    assign TQ = T;
endmodule

// ------------------------------
// Generic I/O delay placeholder
// This module passes DI to DO without analog delay.
// Parameters are kept for compatibility.
// If you need discrete-cycle delay, insert pipelining at the call site.
// ------------------------------
(* keep_hierarchy = "yes" *)
module GTP_IODELAY #(
    parameter [6:0] DELAY_STEP  = 7'd0,
    parameter [6:0] DELAY_DEPTH = 7'd0
) (
    input  wire DI,
    output wire DO,
    input  wire LOAD_N,
    input  wire MOVE,
    input  wire DIRECTION,
    output wire DELAY_OB
);
    // Create a kept buffer chain so instances are not swept,
    // still functionally a pass-through (no cycle latency).
    generate
        if (DELAY_DEPTH == 0) begin : g_passthru
            (* keep = "true" *) wire di_keep /* synthesis keep */ = DI;
            assign DO = di_keep;
        end else begin : g_chain
            (* keep = "true" *) wire [DELAY_DEPTH:0] chain /* synthesis keep */;
            assign chain[0] = DI;
            genvar i;
            for (i = 0; i < DELAY_DEPTH; i = i + 1) begin : g_bufs
                (* keep = "true" *) wire stage /* synthesis keep */;
                assign stage = chain[i];
                assign chain[i+1] = stage;
            end
            assign DO = chain[DELAY_DEPTH];
        end
    endgenerate
    assign DELAY_OB = 1'b0;
endmodule

// ============================================================================
// END of primitive replacements
// ============================================================================

`timescale 1ns/1ps

module dram_dsmc_localbus_with_adc (
    // System clock/reset
    input        sys_clk,
    input        rest_n,

    // DSMC differential clock and control signals
    input        DSMC_CLKP,
    input        DSMC_CLKN,
    input        DSMC_CS,
    input        DSMC_CS1,
    input        DSMC_RESENT,
    inout        DSMC_DQS,
    inout        DSMC_DQS1,
    inout  [7:0] DSMC_DQ,
    inout  [7:0] DSMC_DQ1,

    // AD7606 ADC interface
    input        adc_busy,
    input  [15:0] adc_data,
    output       adc_cs_n,
    output       adc_rd_n,
    output       adc_reset,
    output       adc_convst_a,
    output       adc_convst_b,
    output [2:0] adc_os,
    output       adc_range
);

    // -------------------------------------------------------------------------
    // Internal parameters
    // -------------------------------------------------------------------------
    // Reserve upper addresses of the dual port RAM for ADC data.  The dual port
    // RAM exposes 14 address bits (16k 32‑bit words) so any unused address
    // towards the top of this space can be used.  Choosing 0x3F8 allows space
    // for five 32‑bit words at addresses 0x3F8–0x3FC.
    localparam [13:0] ADC_ADDR_BASE = 14'h3F8;

    // -------------------------------------------------------------------------
    // Wire/reg declarations
    // -------------------------------------------------------------------------
    wire        dsmc_clk_single;
    wire        cs_n;
    reg [15:0]  clk_in_cnt;
    wire        rw_flag_wire;
    reg         rw_flag;
    wire [7:0]  DSMC_DQ_delay;
    wire [7:0]  DSMC_DQ1_delay;
    wire [7:0]  DSMC_DQ_I;
    wire [7:0]  DSMC_DQ_O;
    wire [7:0]  DSMC_DQ1_I;
    wire [7:0]  DSMC_DQ1_O;
    wire [7:0]  dsmc_d_a;
    wire [7:0]  dsmc_d_b;
    wire [7:0]  dsmc_d_h_a;
    wire [7:0]  dsmc_d_h_b;
    reg  [15:0] command_info;
    reg  [31:0] address_info;
    reg  [31:0] data_info;
    reg         dram_wr_clk_en;
    reg  [13:0] dram_wr_addr;
    reg  [31:0] dram_wr_data;
    reg         dram_rd_clk_en;
    reg  [13:0] dram_rd_addr;
    wire [31:0] dram_rd_data;
    wire [31:0] bus_read_data;

    // ADC capture registers
    // These wires are driven directly by the AD7606 controller (sc1467).
    // They represent the most recent 16‑bit result for each of the 8 channels.
    wire [15:0] adc_ch1_reg, adc_ch2_reg, adc_ch3_reg, adc_ch4_reg;
    wire [15:0] adc_ch5_reg, adc_ch6_reg, adc_ch7_reg, adc_ch8_reg;
    // Valid flag toggled when a new sample set is available.  Cleared by a
    // write to the status word from software.
    reg         adc_data_valid;
    wire        adc_read_done;

    // Unused inputs tied off
    (* keep = "true" *) wire _unused_inputs = sys_clk ^ DSMC_CLKN ^ DSMC_CS1 ^ DSMC_RESENT;

    // -------------------------------------------------------------------------
    // Clock and chip select handling
    // -------------------------------------------------------------------------
    assign dsmc_clk_single = DSMC_CLKP;
    assign cs_n            = ~DSMC_CS;
    assign rw_flag_wire    = rw_flag;
    // Output DQS clocks when reading (rw_flag=0) and after the address phase
    assign DSMC_DQS  = (!DSMC_CS && rw_flag == 1'b0 && clk_in_cnt >= 16'd9) ? dsmc_clk_single : 1'bz;
    assign DSMC_DQS1 = (!DSMC_CS && rw_flag == 1'b0 && clk_in_cnt >= 16'd9) ? dsmc_clk_single : 1'bz;

    // -------------------------------------------------------------------------
    // Bidirectional data buffers and delay chains
    // -------------------------------------------------------------------------
    genvar i;
    generate
        for (i = 0; i < 8; i = i + 1) begin : gen_dsmc_io
            // DSMC_DQ[7:0]
            GTP_IOBUF #(
                .IOSTANDARD    ("DEFAULT"),
                .SLEW_RATE     ("FAST"),
                .DRIVE_STRENGTH(8)
            ) iobuf_dq_inst (
                .IO  (DSMC_DQ[i]),
                .I   (DSMC_DQ_I[i]),
                .O   (DSMC_DQ_O[i]),
                .T   (rw_flag_wire)
            );

            // DSMC_DQ1[7:0]
            GTP_IOBUF #(
                .IOSTANDARD    ("DEFAULT"),
                .SLEW_RATE     ("FAST"),
                .DRIVE_STRENGTH(8)
            ) iobuf_dq1_inst (
                .IO  (DSMC_DQ1[i]),
                .I   (DSMC_DQ1_I[i]),
                .O   (DSMC_DQ1_O[i]),
                .T   (rw_flag_wire)
            );

            // Input delay for DQ (low byte)
            GTP_IODELAY #(
                .DELAY_STEP (7'd30),
                .DELAY_DEPTH(7)
            ) iodelay_dq_inst (
                .DI     (DSMC_DQ_O[i]),
                .DO     (DSMC_DQ_delay[i]),
                .LOAD_N (1'b0),
                .MOVE   (1'b0),
                .DIRECTION(1'b0),
                .DELAY_OB()
            );

            // Input delay for DQ1 (high byte)
            GTP_IODELAY #(
                .DELAY_STEP (7'd30),
                .DELAY_DEPTH(7)
            ) iodelay_dq1_inst (
                .DI     (DSMC_DQ1_O[i]),
                .DO     (DSMC_DQ1_delay[i]),
                .LOAD_N (1'b0),
                .MOVE   (1'b0),
                .DIRECTION(1'b0),
                .DELAY_OB()
            );

            // DDR input capture for low byte
            GTP_IDDR_E2 #(
                .GRS_EN("TRUE")
            ) iddr_dq_inst (
                .Q0 (dsmc_d_a[i]),
                .Q1 (dsmc_d_b[i]),
                .CLK(dsmc_clk_single),
                .D  (DSMC_DQ_delay[i]),
                .RS (~rest_n)
            );

            // DDR input capture for high byte
            GTP_IDDR_E2 #(
                .GRS_EN("TRUE")
            ) iddr_dq1_inst (
                .Q0 (dsmc_d_h_a[i]),
                .Q1 (dsmc_d_h_b[i]),
                .CLK(dsmc_clk_single),
                .D  (DSMC_DQ1_delay[i]),
                .RS (~rest_n)
            );

            // DDR output drivers for low byte; use bus_read_data rather than dram_rd_data
            GTP_ODDR_E2 #(
                .GRS_EN("TRUE")
            ) oddr_dq_inst (
                .Q  (DSMC_DQ_I[i]),
                .TQ (),
                .CLK(dsmc_clk_single),
                .D0 (bus_read_data[i+8]),
                .D1 (bus_read_data[i]),
                .RS (~rest_n),
                .T  ((rw_flag_wire || ~(clk_in_cnt >= 12'd5)))
            );

            // DDR output drivers for high byte; use bus_read_data rather than dram_rd_data
            GTP_ODDR_E2 #(
                .GRS_EN("TRUE")
            ) oddr_dq1_inst (
                .Q  (DSMC_DQ1_I[i]),
                .TQ (),
                .CLK(dsmc_clk_single),
                .D0 (bus_read_data[i+24]),
                .D1 (bus_read_data[i+16]),
                .RS (~rest_n),
                .T  ((rw_flag_wire || ~(clk_in_cnt >= 12'd5)))
            );
        end
    endgenerate

    // -------------------------------------------------------------------------
    // DSMC command/address state machine
    // -------------------------------------------------------------------------
    // Count the number of half cycles within a chip select burst.  At the start
    // of each burst the counter resets.  The command and address information
    // are captured on specific counts.
    always @(posedge dsmc_clk_single or negedge cs_n) begin
        if (~cs_n) begin
            clk_in_cnt <= 16'd0;
        end else begin
            clk_in_cnt <= clk_in_cnt + 16'd1;
        end
    end

    // Capture command (read/write) and address from the master during the
    // address phase.  command_info[15] indicates read (0) or write (1).
    always @(posedge dsmc_clk_single or negedge cs_n) begin
        if (~cs_n) begin
            command_info <= 16'd0;
            address_info <= 32'd0;
            data_info    <= 32'd0;
            rw_flag      <= 1'b1; // default to write when idle
        end else begin
            case (clk_in_cnt)
                16'd2: begin
                    if (rw_flag == 1'b1) begin
                        command_info[15:8] <= dsmc_d_a;
                        command_info[7:0]  <= dsmc_d_b;
                    end
                end
                16'd3: begin
                    rw_flag           <= command_info[15];
                    address_info[31:24] <= dsmc_d_a;
                    address_info[23:16] <= dsmc_d_b;
                end
                16'd4: begin
                    address_info[15:8] <= dsmc_d_a;
                    address_info[7:0]  <= dsmc_d_b;
                end
                default: ;
            endcase
        end
    end

    // Handle DSMC write transactions.  Write data is latched and written
    // sequentially into the dual port RAM.  The address is taken from
    // address_info[15:2] and incremented for each 32‑bit word.
    always @(posedge dsmc_clk_single or negedge cs_n) begin
        if (~cs_n) begin
            // reset
        end else if (clk_in_cnt == 16'd0) begin
            dram_wr_addr   <= 14'd0;
            dram_wr_clk_en <= 1'b0;
        end else if (clk_in_cnt == 16'd5) begin
            dram_wr_addr   <= address_info[15:2];
        end else if ((clk_in_cnt >= 16'd6) && (rw_flag == 1'b1)) begin
            dram_wr_data[31:24] <= dsmc_d_h_a;
            dram_wr_data[23:16] <= dsmc_d_h_b;
            dram_wr_data[15:8]  <= dsmc_d_a;
            dram_wr_data[7:0]   <= dsmc_d_b;
            dram_wr_clk_en      <= 1'b1;
            if (clk_in_cnt >= 16'd7) begin
                dram_wr_addr <= dram_wr_addr + 1'b1;
            end
        end else begin
            dram_wr_clk_en <= 1'b0;
        end
    end

    // Handle DSMC read transactions.  dram_rd_addr is loaded during the
    // address phase and incremented for each burst.  dram_rd_clk_en gates
    // updates to the read port of the dual port RAM.
    always @(posedge dsmc_clk_single or negedge cs_n) begin
        if (~cs_n) begin
            dram_rd_clk_en <= 1'b0;
        end else if (clk_in_cnt == 16'd0) begin
            dram_rd_addr <= 14'd0;
        end else if ((clk_in_cnt == 16'd5) && (rw_flag == 1'b0)) begin
            dram_rd_clk_en <= 1'b1;
            dram_rd_addr   <= address_info[15:2];
        end else if ((clk_in_cnt >= 16'd6) && (rw_flag == 1'b0)) begin
            dram_rd_addr <= dram_rd_addr + 1'b1;
        end else begin
            // hold current values
        end
    end

    // -------------------------------------------------------------------------
    // ADC acquisition core
    // -------------------------------------------------------------------------
    // Instantiate the AD7606 controller.  The sampling rate and system clock
    // frequency parameters should be adjusted to suit the target platform.  A
    // 100 MHz system clock and 20 kSPS sampling rate are used here to match
    // the example code provided.
    sc1467 #(
        .FPGA_CLOCK_FREQ  (100),
        .ADC_SAMPLING_RATE(20)
    ) u_adc (
        .sys_clk          (sys_clk),
        .rst_n            (rest_n),
        .adc_busy         (adc_busy),
        .adc_cs_n         (adc_cs_n),
        .adc_rd_n         (adc_rd_n),
        .adc_data         (adc_data),
        .adc_ch1_data_out (adc_ch1_reg),
        .adc_ch2_data_out (adc_ch2_reg),
        .adc_ch3_data_out (adc_ch3_reg),
        .adc_ch4_data_out (adc_ch4_reg),
        .adc_ch5_data_out (adc_ch5_reg),
        .adc_ch6_data_out (adc_ch6_reg),
        .adc_ch7_data_out (adc_ch7_reg),
        .adc_ch8_data_out (adc_ch8_reg),
        .adc_read_done    (adc_read_done),
        .adc_convst_a     (adc_convst_a),
        .adc_convst_b     (adc_convst_b),
        .adc_os           (adc_os),
        .adc_range        (adc_range),
        .adc_reset        (adc_reset)
    );

    // Latch validity flag on completion of a conversion.  The channel data
    // themselves are driven directly by the sc1467 instance, so no latching
    // is required here.  When adc_read_done asserts, mark the data as fresh.
    // Software can clear this flag by writing to the status word (address
    // ADC_ADDR_BASE + 4).
    always @(posedge sys_clk or negedge rest_n) begin
        if (!rest_n) begin
            adc_data_valid <= 1'b0;
        end else begin
            if (adc_read_done) begin
                adc_data_valid <= 1'b1;
            end
            // Clear validity flag on software write to status word
            if (dram_wr_clk_en && (dram_wr_addr == ADC_ADDR_BASE + 14'd4)) begin
                adc_data_valid <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Dual port RAM instance
    // -------------------------------------------------------------------------
    dsmc_dram U0 (
        .wr_data   (dram_wr_data),
        .wr_addr   (dram_wr_addr),
        .wr_en     (dram_wr_clk_en),
        .wr_clk    (dsmc_clk_single),
        .wr_rst    (1'b0),
        .rd_addr   (dram_rd_addr),
        .rd_data   (dram_rd_data),
        .rd_clk    (dsmc_clk_single),
        .rd_clk_en (dram_rd_clk_en),
        .rd_rst    (1'b0)
    );

    // -------------------------------------------------------------------------
    // Read data multiplexer
    // -------------------------------------------------------------------------
    // On read cycles, multiplex between the dual port RAM data and ADC data
    // registers.  For addresses outside the reserved ADC window the RAM data
    // is returned unchanged.  The multiplexer is purely combinatorial.
    assign bus_read_data = (rw_flag_wire == 1'b0 && dram_rd_addr == ADC_ADDR_BASE)     ? {adc_ch2_reg, adc_ch1_reg} :
                           (rw_flag_wire == 1'b0 && dram_rd_addr == ADC_ADDR_BASE + 14'd1) ? {adc_ch4_reg, adc_ch3_reg} :
                           (rw_flag_wire == 1'b0 && dram_rd_addr == ADC_ADDR_BASE + 14'd2) ? {adc_ch6_reg, adc_ch5_reg} :
                           (rw_flag_wire == 1'b0 && dram_rd_addr == ADC_ADDR_BASE + 14'd3) ? {adc_ch8_reg, adc_ch7_reg} :
                           (rw_flag_wire == 1'b0 && dram_rd_addr == ADC_ADDR_BASE + 14'd4) ? {31'b0, adc_data_valid} :
                           dram_rd_data;

endmodule

