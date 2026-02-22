#ifndef __VOLTAGE_H
#define __VOLTAGE_H

#include "stm32f10x.h"
#include "Modules.h"

/************************** 光伏采集硬件参数配置 **************************/
#define VREF            3.3f       // ADC 参考电压
#define ADC_RESOLUTION  4095       // 12位 ADC 分辨率
#define SENSE_RESISTOR  10.0f      // 电流采样电阻（Ω），需与硬件一致
#define VOLT_DIV_RATIO  5.0f       // 电压分压比，需与硬件一致
#define ADC_CHANNEL     ADC_Channel_0  // ADC 采集通道（PA0）

/************************** 函数声明 **************************/
static float ADC_ReadOnce(void);  // 内部 ADC 采集函数
void PV_Collect_Init(void);       // 光伏采集初始化
void PV_Collect_Params(void);     // 光伏参数采集（电压/电流/功率）

#endif // __VOLTAGE_H

