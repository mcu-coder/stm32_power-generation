#include "led.h"
#include "delay.h"

void LED_Init(void)
{
	//开启GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//配置LED引脚为模拟输出模式
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = LED_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(LED_GPIO_PORT, LED_GPIO_PIN);
}

void LED_Toggle(void)
{
	GPIO_WriteBit(LED_GPIO_PORT, LED_GPIO_PIN, (BitAction)((1-GPIO_ReadOutputDataBit(LED_GPIO_PORT, LED_GPIO_PIN))));//led电平翻转
}
void LED_On()
{
	GPIO_WriteBit(LED_GPIO_PORT, LED_GPIO_PIN,(BitAction)1);
}
void LED_Off()
{
	GPIO_WriteBit(LED_GPIO_PORT, LED_GPIO_PIN,(BitAction)0);
}

void LED_Twinkle()
{
	LED_On();
	delay_ms(10);
	LED_Off();
}


//void LED1_Init(void)
//{
//	//开启GPIOB时钟
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
//	//配置LED引脚为模拟输出模式
//	GPIO_InitTypeDef GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Pin = LED1_GPIO_PIN;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);
//	GPIO_ResetBits(LED1_GPIO_PORT, LED1_GPIO_PIN);
//}

//void LED1_Toggle(void)
//{
//	GPIO_WriteBit(LED1_GPIO_PORT, LED1_GPIO_PIN, (BitAction)((1-GPIO_ReadOutputDataBit(LED1_GPIO_PORT, LED1_GPIO_PIN))));//led电平翻转
//}
//void LED1_On()
//{
//	GPIO_WriteBit(LED1_GPIO_PORT, LED1_GPIO_PIN,(BitAction)0);
//}
//void LED1_Off()
//{
//	GPIO_WriteBit(LED1_GPIO_PORT, LED1_GPIO_PIN,(BitAction)1);
//}

//void LED1_Twinkle()
//{
//	LED1_On();
//	delay_ms(10);
//	LED1_Off();
//}

