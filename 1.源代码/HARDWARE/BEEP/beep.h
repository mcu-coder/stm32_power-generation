#ifndef __BEEP_H
#define	__BEEP_H
#include "stm32f10x.h"
#include "delay.h"
#include "sys.h"



/***************根据自己需求更改****************/
// 蜂鸣器 GPIO宏定义

#define	BEEP_CLK							RCC_APB2Periph_GPIOC

#define BEEP_GPIO_PIN 				GPIO_Pin_13

#define BEEP_GPIO_PROT 			GPIOC


//#define BEEP_OFF 		GPIO_SetBits(BEEP_GPIO_PROT,BEEP_GPIO_PIN)
//#define BEEP_ON 	GPIO_ResetBits(BEEP_GPIO_PROT,BEEP_GPIO_PIN)
#define BEEP_ON 		GPIO_SetBits(BEEP_GPIO_PROT,BEEP_GPIO_PIN)
#define BEEP_OFF 	GPIO_ResetBits(BEEP_GPIO_PROT,BEEP_GPIO_PIN)


/*********************END**********************/

void BEEP_Init(void);

#endif



