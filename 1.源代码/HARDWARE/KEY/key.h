#ifndef	__KEY_H
#define __KEY_H
#include "stm32f10x.h"                  								// Device header




/*按键定义处*/
#define KEY1_GPIO_PIN	GPIO_Pin_12
#define KEY2_GPIO_PIN	GPIO_Pin_13
#define KEY3_GPIO_PIN	GPIO_Pin_14
#define KEY4_GPIO_PIN	GPIO_Pin_15

#define KEY_PORT	GPIOB
 
#define KEY1	GPIO_ReadInputDataBit(GPIOB, KEY1_GPIO_PIN) // 读取按键0
#define KEY2	GPIO_ReadInputDataBit(GPIOB, KEY2_GPIO_PIN) // 读取按键1
#define KEY3	GPIO_ReadInputDataBit(GPIOB, KEY3_GPIO_PIN) // 读取按键2
#define KEY4	GPIO_ReadInputDataBit(GPIOB, KEY4_GPIO_PIN) // 读取按键2

/*按键阈值*/
#define KEY_DELAY_TIME							10										  //消抖延时			   												10ms
#define KEY_LONG_TIME								1000								    //长按-阈值		   												1000ms

#define KEY_Continue_TIME					  500								  //短按最长停留时长--超过进行连击阈值判断	1000ms		
#define KEY_Continue_Trigger_TIME		5											//连击阈值																10ms		

/*按键码*/
#define KEY1_Short      1		//KEY1短按
#define KEY1_Long       11		//KEY1长按
#define KEY2_Short      2		//KEY2短按
#define KEY2_Long       22		//KEY2长按
#define KEY3_Short      3		//KEY3短按
#define KEY3_Long       33		//KEY3长按
#define KEY4_Short      4		//KEY4短按
#define KEY4_Long       44		//KEY4长按


extern u8 KeyNum;

void Key_Init(void);
void Key_scan(void);

#endif
