#ifndef __RELAY_H
#define	__RELAY_H
#include "stm32f10x.h"
#include "delay.h"
#include "sys.h"

 

/***************根据自己需求更改****************/
// 继电器 GPIO宏定义

#define	RELAY_CLK							RCC_APB2Periph_GPIOB

#define RELAY_GPIO_PIN 				GPIO_Pin_1

#define RELAY_GPIO_PROT 			GPIOB

#define RELAY_ON 		GPIO_SetBits(RELAY_GPIO_PROT,RELAY_GPIO_PIN)
#define RELAY_OFF 	GPIO_ResetBits(RELAY_GPIO_PROT,RELAY_GPIO_PIN)


/*********************END**********************/

void RELAY_Init(void);

#endif



