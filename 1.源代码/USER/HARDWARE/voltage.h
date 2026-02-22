#ifndef __VOLTAGE_H
#define __VOLTAGE_H

#include "stm32f10x.h"  // STM32F1系列标准库头文件（根据实际型号调整）

/************************** 宏定义配置 **************************/
#define VREF        3.3f    // ADC参考电压（单位：V，默认外部3.3V供电）
#define ADC_RESOLUTION  4095 // 12位ADC的最大数值（2^12 - 1）

/************************** 函数声明 **************************/
/**
 * @brief  ADC初始化（PA0引脚配置为模拟输入）
 * @param  无
 * @retval 无
 */
void Voltage_PA0_Init(void);

/**
 * @brief  读取PA0引脚的实际电压值
 * @param  无
 * @retval float: 检测到的电压值（单位：V，精度约0.81mV）
 */
float Voltage_PA0_Read(void);

#endif // __VOLTAGE_H

