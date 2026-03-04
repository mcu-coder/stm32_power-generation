#include "voltage.h"

/**
 * @brief  ADC初始化（PA0 -> ADC1通道0）
 * 步骤：1.使能GPIO和ADC时钟  2.配置PA0为模拟输入  3.配置ADC工作模式  4.ADC校准
 */
void Voltage_PA0_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    ADC_InitTypeDef ADC_InitStruct;

    /************************** 1. 使能时钟 **************************/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // 使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);   // 使能ADC1时钟
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);  // ADC时钟分频（APB2=72MHz -> 72/6=12MHz，不超过14MHz上限）

    /************************** 2. 配置PA0为模拟输入 **************************/
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;          // PA0引脚
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;      // 模拟输入模式（无上下拉、无推挽）
    GPIO_Init(GPIOA, &GPIO_InitStruct);             // 初始化GPIOA

    /************************** 3. 配置ADC工作模式 **************************/
    ADC_InitStruct.ADC_Mode = ADC_Mode_Independent; // 独立模式（仅使用ADC1）
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;      // 关闭扫描模式（单通道采集）
    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;// 关闭连续转换（单次触发单次转换）
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 禁止外部触发（软件触发）
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right; // 数据右对齐（12位有效数据）
    ADC_InitStruct.ADC_NbrOfChannel = 1;            // 采集通道数量（1个通道）
    ADC_Init(ADC1, &ADC_InitStruct);                // 初始化ADC1

    /************************** 4. 配置ADC通道采样时间 **************************/
    // ADC1 -> 通道0（PA0）-> 采样时间55.5周期（采样时间越长，精度越高，速度越慢）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);

    /************************** 5. ADC校准（提高精度） **************************/
    ADC_Cmd(ADC1, ENABLE);                          // 使能ADC1
    ADC_ResetCalibration(ADC1);                     // 复位校准寄存器
    while(ADC_GetResetCalibrationStatus(ADC1));     // 等待复位完成
    ADC_StartCalibration(ADC1);                     // 开始校准
    while(ADC_GetCalibrationStatus(ADC1));          // 等待校准完成
}

/**
 * @brief  读取PA0引脚电压
 * 逻辑：软件触发ADC转换 -> 等待转换完成 -> 读取ADC原始值 -> 换算为实际电压
 */
float Voltage_PA0_Read(void)
{
    uint16_t adc_raw_val = 0;  // ADC原始采集值（0~4095）
    float voltage = 0.0f;      // 实际电压值

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);         // 软件触发ADC转换
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));  // 等待转换完成（EOC=1表示完成）
    adc_raw_val = ADC_GetConversionValue(ADC1);     // 读取ADC原始值

    // 电压换算公式：实际电压 = (ADC原始值 / ADC最大值) * 参考电压
    voltage = (adc_raw_val * VREF) / ADC_RESOLUTION;

    return voltage;
}

