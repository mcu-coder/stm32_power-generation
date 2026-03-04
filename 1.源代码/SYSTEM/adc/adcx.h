#ifndef __ADCX_H
#define __ADCX_H

#include "stm32f10x.h"                  // Device header
#include "adcx.h"


/**
  * 函    数：ADC初始化
  * 参    数：无
  * 返 回 值：无
  */
void ADCX_Init(void);
void Get_Average_LDR_LUX(uint16_t *lux);
void Get_Average_MQ2_PPM(uint16_t *ppm);
//void Get_Average_MQ135_PPM(uint16_t *ppm);
uint16_t LDR_LuxData(void);
uint16_t MQ2_GetData_PPM(void);
//uint16_t MQ7_GetData_PPM(void);
	
uint16_t MQ135_GetData_PPM(void);

/**
  * 函    数：读取MQ-7 DO口数字量（0/1）
  * 参    数：无
  * 返 回 值：0（浓度超限）或1（浓度正常）
  */
uint8_t MQ7_Get_DO_State(void);

#endif
