#ifndef __DELAY_H
#define __DELAY_H

#include "sys.h"

void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);

// 添加系统滴答计时器功能
uint32_t delay_get_tick(void);
void delay_inc_tick(void);

#endif

