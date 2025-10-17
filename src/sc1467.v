 `timescale 1ns / 1ps
module sc1467 #(
    parameter FPGA_CLOCK_FREQ   = 100,  // FPGA主时钟频率 (MHz)
    parameter ADC_SAMPLING_RATE = 20    // ADC采样速率 (kSPS)
)(
    // 时钟与复位
    input  wire        sys_clk,       // 系统时钟输入
    input  wire        rst_n,         // 全局复位，低有效

    // AD7606 控制信号输出
    output wire        adc_reset,     // AD7606 RESET，高有效
    output wire        adc_convst_a,  // AD7606 转换开始 A
    output wire        adc_convst_b,  // AD7606 转换开始 B
    output wire [2:0]  adc_os,        // AD7606 过采样比特设置
    output wire        adc_range,     // AD7606 输入电压量程选择

    // AD7606 并行数据接口
    input  wire        adc_busy,      // AD7606 BUSY 标志
    output wire        adc_cs_n,      // AD7606 片选，低有效
    output wire        adc_rd_n,      // AD7606 读控制，低有效
    input  wire [15:0] adc_data,      // AD7606 16位并行数据总线

    // AD7606 八通道数据输出
    output wire [15:0] adc_ch1_data_out,
    output wire [15:0] adc_ch2_data_out,
    output wire [15:0] adc_ch3_data_out,
    output wire [15:0] adc_ch4_data_out,
    output wire [15:0] adc_ch5_data_out,
    output wire [15:0] adc_ch6_data_out,
    output wire [15:0] adc_ch7_data_out,
    output wire [15:0] adc_ch8_data_out,

    // 采集完成标志输出
    output wire        adc_read_done,

    // 新增：ADC采集使能控制输入
    input  wire        adc_enable
);

    // 内部信号
    wire adc_convst;
    wire adc_wr_n;
    wire adc_convst_en;

    // 将单一转换启动信号同时连接到AD7606的CONVST_A和CONVST_B，引脚
    assign adc_convst_a = adc_convst;
    assign adc_convst_b = adc_convst;
    // 配置AD7606：范围选择0 = ±5V，过采样率OS[2:0] = 000（不启用过采样）
    assign adc_range    = 1'b0;
    assign adc_os       = 3'b000;
    // AD7606复位信号：FPGA复位时拉高复位ADC
    assign adc_reset    = ~rst_n;
    // AD7606并行接口写控制不使用，保持高电平
    assign adc_wr_n     = 1'b1;
    // 将外部enable控制赋给内部转换使能
    assign adc_convst_en = adc_enable;

    // 实例化并行ADC采集子模块
    parallel_adc_capture #(
        .FPGA_CLOCK_FREQ   (FPGA_CLOCK_FREQ),
        .ADC_SAMPLING_RATE (ADC_SAMPLING_RATE)
    ) u_parallel_adc_capture (
        .sys_clk        (sys_clk),
        .rst_n          (rst_n),
        .adc_convst     (adc_convst),
        .adc_busy       (adc_busy),
        .adc_cs_n       (adc_cs_n),
        .adc_rd_n       (adc_rd_n),
        .adc_wr_n       (adc_wr_n),
        .adc_data       (adc_data),
        .adc_ch1_data_out (adc_ch1_data_out),
        .adc_ch2_data_out (adc_ch2_data_out),
        .adc_ch3_data_out (adc_ch3_data_out),
        .adc_ch4_data_out (adc_ch4_data_out),
        .adc_ch5_data_out (adc_ch5_data_out),
        .adc_ch6_data_out (adc_ch6_data_out),
        .adc_ch7_data_out (adc_ch7_data_out),
        .adc_ch8_data_out (adc_ch8_data_out),
        .adc_convst_en  (adc_convst_en),
        .adc_read_done  (adc_read_done)
    );

endmodule
