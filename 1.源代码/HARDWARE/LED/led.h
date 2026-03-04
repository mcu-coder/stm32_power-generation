#ifndef	__LED_H
#define	__LED_H

#include "stm32f10x.h"                  // Device header

#define LED_GPIO_PORT		GPIOB
#define LED_GPIO_PIN	  GPIO_Pin_0

//#define LED1_GPIO_PORT		GPIOC
//#define LED1_GPIO_PIN	  GPIO_Pin_13

void LED_Init(void);
void LED_Toggle(void);
void LED_On(void);
void LED_Off(void);
void LED_Twinkle(void);

//void LED1_Init(void);
//void LED1_Toggle(void);
//void LED1_On(void);
//void LED1_Off(void);
//void LED1_Twinkle(void);
#endif
