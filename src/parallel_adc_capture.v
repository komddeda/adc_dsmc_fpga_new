`timescale 1ns / 1ps
module parallel_adc_capture #(
    parameter FPGA_CLOCK_FREQ   = 100,  // FPGA主时钟频率 (MHz)
    parameter ADC_SAMPLING_RATE = 20    // ADC采样速率 (kSPS)
)(
    // 时钟与复位
    input  wire        sys_clk,
    input  wire        rst_n,

    // ADC 控制接口
    output wire        adc_convst,   // AD7606 转换开始脉冲输出
    input  wire        adc_busy,     // AD7606 BUSY 信号输入

    // ADC 并行数据接口
    output wire        adc_cs_n,     // AD7606 片选信号
    output wire        adc_rd_n,     // AD7606 读控制信号
    input  wire        adc_wr_n,     // AD7606 写控制信号（未用，高电平）
    input  wire [15:0] adc_data,     // AD7606 并行数据输入总线

    // ADC 转换使能输入
    input  wire        adc_convst_en,

    // 分别输出8个通道的16位数据
    output reg [15:0]  adc_ch1_data_out,
    output reg [15:0]  adc_ch2_data_out,
    output reg [15:0]  adc_ch3_data_out,
    output reg [15:0]  adc_ch4_data_out,
    output reg [15:0]  adc_ch5_data_out,
    output reg [15:0]  adc_ch6_data_out,
    output reg [15:0]  adc_ch7_data_out,
    output reg [15:0]  adc_ch8_data_out,

    // 8通道数据读取完成标志
    output reg         adc_read_done
);

    // 根据ADC采样率计算转换周期计数（FPGA_CLOCK_FREQ需转换为Hz再计算）
    localparam [31:0] ADC_CYCLE_CNT   = FPGA_CLOCK_FREQ * 1000000 / (ADC_SAMPLING_RATE * 1000);
    // 用于读取ADC数据的时钟频率（采样频率的50倍，用于产生adc_rd_n脉冲）
    localparam [31:0] ADC_PAR_CLK_CNT = ADC_CYCLE_CNT / 50;

    reg [31:0] cycle_cnt;
    reg        clk_convst;
    reg        clk_adc_par;
    reg        start_read_data;
    reg [3:0]  adc_channel_read;

    // 根据使能状态选择CS引脚功能：当采集使能时，adc_cs_n受adc_rd_n控制；禁止时，adc_cs_n与adc_wr_n同电平（均为高）
    assign adc_cs_n = adc_convst_en ? adc_rd_n : adc_wr_n;
    // 控制adc_rd_n：在开始读取数据且尚未读完8个通道时，输出由clk_adc_par提供的脉冲，否则保持为高阻读停止状态
    assign adc_rd_n = (start_read_data && (adc_channel_read < 8)) ? clk_adc_par : 1'b1;
    // 产生ADC启动脉冲：当使能时，输出clk_convst时钟作为CONVST，否则保持为低（不触发转换）
    assign adc_convst = adc_convst_en ? clk_convst : 1'b0;

    // 利用FPGA系统时钟产生采样时序的计数器和时钟信号
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            cycle_cnt  <= ADC_CYCLE_CNT - 1;
            clk_convst <= 1'b1;
            clk_adc_par <= 1'b0;
        end else if (adc_convst_en) begin
            // 每个时钟周期递减计数，达到0则重新加载
            cycle_cnt <= cycle_cnt - 1;
            if (cycle_cnt == 0)
                cycle_cnt <= ADC_CYCLE_CNT - 1;
            // 产生用于读取ADC数据的clk_adc_par时钟（频率为采样频率的50倍，用于驱动adc_rd_n）
            if (cycle_cnt % (ADC_PAR_CLK_CNT / 2) == 0)
                clk_adc_par <= ~clk_adc_par;
            // 产生ADC采样触发时钟clk_convst（周期为ADC_CYCLE_CNT，对称翻转产生方波脉冲）
            if ((cycle_cnt == ADC_CYCLE_CNT / 2) || (cycle_cnt == 0))
                clk_convst <= ~clk_convst;
        end
    end

    // 在adc_convst_en为低（禁能）或复位时，复位读取状态机和数据寄存器
    // 在adc_convst_en为高且ADC空闲时启动数据读取过程
    always @(posedge clk_adc_par or negedge rst_n) begin
        if (!rst_n || !adc_convst_en) begin
            // 复位：通道计数清零，停止读取标志复位，数据完成标志复位，各通道数据清零
            adc_channel_read <= 4'd0;
            start_read_data  <= 1'b0;
            adc_read_done    <= 1'b0;
            adc_ch1_data_out <= 16'b0;
            adc_ch2_data_out <= 16'b0;
            adc_ch3_data_out <= 16'b0;
            adc_ch4_data_out <= 16'b0;
            adc_ch5_data_out <= 16'b0;
            adc_ch6_data_out <= 16'b0;
            adc_ch7_data_out <= 16'b0;
            adc_ch8_data_out <= 16'b0;
        end else begin
            // 当ADC未忙且尚未读取完上一批数据时，开始读取本次ADC转换数据
            if (!adc_busy && !adc_read_done) begin
                start_read_data <= 1'b1;
            end

            // 如果处于读数状态，每来一个clk_adc_par沿就读取一个通道数据
            if (start_read_data) begin
                adc_channel_read <= adc_channel_read + 1'b1;
                case (adc_channel_read)
                    4'd0: adc_ch1_data_out <= adc_data;
                    4'd1: adc_ch2_data_out <= adc_data;
                    4'd2: adc_ch3_data_out <= adc_data;
                    4'd3: adc_ch4_data_out <= adc_data;
                    4'd4: adc_ch5_data_out <= adc_data;
                    4'd5: adc_ch6_data_out <= adc_data;
                    4'd6: adc_ch7_data_out <= adc_data;
                    4'd7: adc_ch8_data_out <= adc_data;
                endcase
            end

            // 当8个通道的数据全部读完时，拉高adc_read_done标志
            if (adc_channel_read > 4'd7) begin
                start_read_data  <= 1'b0;
                adc_channel_read <= 4'd0;
                adc_read_done    <= 1'b1;
            end

            // 若检测到ADC再次变为忙碌（开始下一次转换）且此时刚好通道计数归零，表示新一轮转换开始，清除读取完成标志，为下一次采集做准备
            if (adc_busy && adc_channel_read == 4'd0) begin
                adc_read_done <= 1'b0;
            end
        end
    end

endmodule

