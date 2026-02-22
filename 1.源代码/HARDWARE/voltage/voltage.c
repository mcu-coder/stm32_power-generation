#include "voltage.h"
#include "delay.h"

// 内部 ADC 单次采集（8次平均滤波）
static float ADC_ReadOnce(void)
{
    uint16_t adc_raw = 0;
    float raw_volt = 0.0f;
    uint8_t sample_cnt = 8;

    // 配置 ADC 通道（固定采集 PA0，无需切换）
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL, 1, ADC_SampleTime_239Cycles5);
    for(uint8_t i=0; i<sample_cnt; i++)
    {
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)); // 等待采集完成
        adc_raw += ADC_GetConversionValue(ADC1);
        delay_us(10);
    }
    // 计算原始电压（平均滤波 + 电压转换）
    raw_volt = ((adc_raw / sample_cnt) * VREF) / ADC_RESOLUTION;
    return raw_volt;
}

// 光伏采集初始化（仅初始化 PA0 和 ADC1，删除 PB0 配置）
void PV_Collect_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    ADC_InitTypeDef ADC_InitStruct;

    // 使能 GPIOA 和 ADC1 时钟（删除 GPIOB 时钟）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6); // ADC 时钟分频（72MHz/6=12MHz，符合 ADC 要求）

    // 初始化 PA0 为模拟输入（采集通道）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN; // 模拟输入模式
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // ADC1 配置
    ADC_InitStruct.ADC_Mode = ADC_Mode_Independent; // 独立模式
    ADC_InitStruct.ADC_ScanConvMode = DISABLE; // 非扫描模式（单通道）
    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE; // 单次转换模式
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 软件触发
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right; // 右对齐
    ADC_InitStruct.ADC_NbrOfChannel = 1; // 1个通道
    ADC_Init(ADC1, &ADC_InitStruct);

    // 使能 ADC1 并校准
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1)); // 等待复位校准完成
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1)); // 等待校准完成
}

// 光伏参数采集（删除 PB0 通道切换，根据硬件调整采集逻辑）
void PV_Collect_Params(void)
{
    float volt_raw = 0.0f;
    float curr_raw = 0.0f;
    float sense_volt = 0.0f;

    /************************** 电压采集（PA0 直接采集）**************************/
    volt_raw = ADC_ReadOnce(); // 直接采集 PA0 电压
    sensorData.pv_voltage = volt_raw * VOLT_DIV_RATIO; // 分压转换（根据硬件分压比）
    // 电压范围限制（过滤异常值）
    sensorData.pv_voltage = (sensorData.pv_voltage < 0.1f) ? 0.0f : sensorData.pv_voltage;
    sensorData.pv_voltage = (sensorData.pv_voltage > 5.0f) ? 5.0f : sensorData.pv_voltage;

    /************************** 电流采集（需根据硬件调整！）**************************/

    curr_raw = ADC_ReadOnce(); // 若电流通道与电压通道相同，直接采集；否则需修改
    sense_volt = curr_raw * VOLT_DIV_RATIO; // 电流采样电压转换
    if (SENSE_RESISTOR > 0.01f)
    {
        sensorData.pv_current = (sense_volt / SENSE_RESISTOR) * 1000.0f; // 计算电流（mA）
        // 电流范围限制（过滤异常值）
        sensorData.pv_current = (sensorData.pv_current < 0.5f) ? 0.0f : sensorData.pv_current;
        sensorData.pv_current = (sensorData.pv_current > 50.0f) ? 50.0f : sensorData.pv_current;
    }

    /************************** 功率计算（电压 × 电流）**************************/
    sensorData.pv_power = sensorData.pv_voltage * (sensorData.pv_current / 1000.0f); // 功率（W）
}

