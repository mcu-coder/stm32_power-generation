#ifndef __USART3_H
#define __USART3_H

#include "stm32f10x.h"                  // Device header
#include "oled.h"
#include "usart.h"


void USART3_Config(void); //串口 3 初始化
void USART3_SendByte(uint8_t data);    // 发送单个字节
void USART3_SendString(uint8_t *str);  // 发送字符串

extern u8 Flag_dakai, dakai;
#endif


