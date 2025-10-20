`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
//  Title      : ADC control without enable gate
//  File       : adc_control_auto.v
//  Description: This module writes a full set of eight 16‑bit ADC channel
//               samples into a dual‑port RAM.  Unlike the original design it
//               does not include an external enable input; sampling and
//               memory writes run continuously whenever new data arrives.
//               The write address wraps at 0x3F7 to avoid overwriting the
//               reserved ADC register window (0x3F8–0x3FC).  The first word
//               (address 0) remains unused for control.
//
//  The write process is triggered on the rising edge of adc_read_done.  Four
//  32‑bit words are written sequentially into the RAM, combining pairs of
//  16‑bit ADC samples into each 32‑bit word.
//
//  Ports:
//    sys_clk        : system clock
//    rst_n          : asynchronous active‑low reset
//    adc_read_done  : asserted when a full set of eight samples is ready
//    adc_ch[1..8]   : 16‑bit channels from the ADC front end
//    mem_wr_addr    : write address for the dual‑port RAM (14‑bit)
//    mem_wr_data    : write data for the dual‑port RAM (32‑bit)
//    mem_wr_en      : write enable pulse for the dual‑port RAM
//
//  Copyright (c) 2025.  This code is provided without warranty of any kind
//  and may be used and modified freely.
// -----------------------------------------------------------------------------

module adc_control_auto (
    input  wire        sys_clk,        // FPGA系统时钟
    input  wire        rst_n,          // 全局复位，低有效
    input  wire        adc_read_done,  // AD7606一次8通道数据采集完成标志

    input  wire [15:0] adc_ch1,        // 8个ADC通道数据输入
    input  wire [15:0] adc_ch2,
    input  wire [15:0] adc_ch3,
    input  wire [15:0] adc_ch4,
    input  wire [15:0] adc_ch5,
    input  wire [15:0] adc_ch6,
    input  wire [15:0] adc_ch7,
    input  wire [15:0] adc_ch8,

    output reg  [13:0] mem_wr_addr,    // 双口RAM写地址（14位，地址0保留不用）
    output reg  [31:0] mem_wr_data,    // 双口RAM写数据（32位）
    output reg         mem_wr_en       // 双口RAM写使能脉冲
);

    // 内部状态寄存器
    reg        writing;        // 指示当前是否在写4个字的数据块
    reg  [2:0] write_count;    // 当前已写入字计数（0~7）
    reg        last_read_done; // 前一时钟周期的adc_read_done，用于边沿检测

    // 在时钟域内检测adc_read_done的上升沿
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            last_read_done <= 1'b0;
        end else begin
            last_read_done <= adc_read_done;
        end
    end

    // 主状态机：监听adc_read_done，上升沿触发写数据4字块
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            // 复位状态：清零写地址和控制信号
            mem_wr_addr <= 14'd1;    // 数据区起始地址设为1（0用作控制寄存器）
            mem_wr_data <= 32'd0;
            mem_wr_en   <= 1'b0;
            writing     <= 1'b0;
            write_count <= 3'd0;
        end else begin
            // 当采集完成标志从0变为1且当前不在写入过程中时，启动一次写数据过程
            if ((adc_read_done == 1'b1) && (last_read_done == 1'b0) && !writing) begin
                writing     <= 1'b1;
                write_count <= 3'd0;
            end

            if (writing) begin
                case (write_count)
                    3'd0: begin
                        // 第0个32位数据：通道1(低16位) + 通道2(高16位)
                        mem_wr_data <= {adc_ch2, adc_ch1};
                        mem_wr_en   <= 1'b1;
                        // 地址不变（写第一个字到当前mem_wr_addr）
                        write_count <= 3'd1;
                    end
                    3'd1: begin
                        // 拉低写使能一个周期，准备下一个数据
                        mem_wr_en   <= 1'b0;
                        // 递增地址，达到0x3F7则回绕至1，避免占用ADC保留地址0x3F8–0x3FC
                        mem_wr_addr <= (mem_wr_addr >= 14'h3F7) ? 14'd1 : (mem_wr_addr + 14'd1);
                        write_count <= 3'd2;
                    end
                    3'd2: begin
                        // 第二个32位数据：通道3 + 通道4
                        mem_wr_data <= {adc_ch4, adc_ch3};
                        mem_wr_en   <= 1'b1;
                        write_count <= 3'd3;
                    end
                    3'd3: begin
                        mem_wr_en   <= 1'b0;
                        mem_wr_addr <= (mem_wr_addr >= 14'h3F7) ? 14'd1 : (mem_wr_addr + 14'd1);
                        write_count <= 3'd4;
                    end
                    3'd4: begin
                        // 第三个32位数据：通道5 + 通道6
                        mem_wr_data <= {adc_ch6, adc_ch5};
                        mem_wr_en   <= 1'b1;
                        write_count <= 3'd5;
                    end
                    3'd5: begin
                        mem_wr_en   <= 1'b0;
                        mem_wr_addr <= (mem_wr_addr >= 14'h3F7) ? 14'd1 : (mem_wr_addr + 14'd1);
                        write_count <= 3'd6;
                    end
                    3'd6: begin
                        // 第四个32位数据：通道7 + 通道8
                        mem_wr_data <= {adc_ch8, adc_ch7};
                        mem_wr_en   <= 1'b1;
                        write_count <= 3'd7;
                    end
                    3'd7: begin
                        // 最后一个写周期结束：拉低写使能，并更新下一个写入地址
                        mem_wr_en   <= 1'b0;
                        mem_wr_addr <= (mem_wr_addr >= 14'h3F7) ? 14'd1 : (mem_wr_addr + 14'd1);
                        // 完成4个32位写入，退出写入状态
                        writing     <= 1'b0;
                        write_count <= 3'd0;
                    end
                    default: begin
                        mem_wr_en <= 1'b0;
                    end
                endcase
            end else begin
                // 非writing状态下，确保不拉高写使能
                mem_wr_en <= 1'b0;
            end
        end
    end

endmodule
