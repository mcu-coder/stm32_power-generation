#include "delay.h"
#include "timer.h"
#include "adcx.h"
#include "led.h"
#include "string.h"


//extern u16 msHcCount;
void TIM4_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 			//时钟使能
	
	//定时器TIM4初始化
	TIM_TimeBaseStructure.TIM_Period = arr; 						//设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 						//设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 		//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 				//根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);  
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); 						//使能指定的TIM4中断,允许更新中断
	
	//中断优先级NVIC设置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  				//TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 		//先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  			//从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure); 								//初始化NVIC寄存器

	TIM_Cmd(TIM4,DISABLE);     //禁用TIM4					 
}

//定时器4中断服务程序
void TIM4_IRQHandler(void)   //TIM4中断		
{
   if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  
   {
       TIM_ClearITPendingBit(TIM4, TIM_IT_Update  ); 
      // msHcCount++;
   }
}


