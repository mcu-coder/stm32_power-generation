#include "stm32f10x.h"                  // Device header
#include <math.h>
#include "delay.h"

static uint16_t AD_Value[3];					//存放AD转换结果


#define LDR				GPIO_Pin_0
//#define MQ2				GPIO_Pin_4
//#define MQ135			GPIO_Pin_5

#define		MQ7_DO_GPIO_CLK								RCC_APB2Periph_GPIOA
#define 	MQ7_DO_GPIO_PORT							GPIOA
#define 	MQ7_DO_GPIO_PIN								GPIO_Pin_1		


/**
  * 函    数：ADC初始化
  * 参    数：无
  * 返 回 值：无
  */
void ADCX_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);	//开启ADC1的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//开启GPIOA的时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		//开启DMA1的时钟
	RCC_APB2PeriphClockCmd(MQ7_DO_GPIO_CLK, ENABLE);			//开启MQ-7 DO口的GPIO时钟
	
	/*设置ADC时钟*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);						//选择6分频
	
	/*GPIO初始化：分2部分——ADC模拟输入（LDR、MQ2、MQ135） + DO口数字输入（MQ7）*/
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// 第一部分：ADC模拟输入配置（Pin0、Pin1、Pin5）
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;			//模拟输入模式
	GPIO_InitStructure.GPIO_Pin = LDR ;		//保留3个ADC引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//初始化ADC引脚
	
	// 第二部分：MQ-7 DO口配置（数字上拉输入）
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;			//上拉输入
	GPIO_InitStructure.GPIO_Pin = MQ7_DO_GPIO_PIN	;				//DO口引脚（Pin2）
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MQ7_DO_GPIO_PORT, &GPIO_InitStructure);			//初始化DO口引脚
	
	/*规则组通道配置：仅保留3个通道*/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);	//LDR
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);	//MQ2
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, ADC_SampleTime_55Cycles5);	//MQ135
	
	/*ADC初始化*/
	ADC_InitTypeDef ADC_InitStructure;											//定义结构体变量
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;							//独立模式
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;						//右对齐
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;			//软件触发
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;							//连续转换
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;								//扫描模式
	ADC_InitStructure.ADC_NbrOfChannel = 1;										//通道数改为3
	ADC_Init(ADC1, &ADC_InitStructure);											//配置ADC1
	
	/*DMA初始化*/
	DMA_InitTypeDef DMA_InitStructure;											//定义结构体变量
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;				//外设基地址
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//外设数据宽度
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设地址自增，选择失能
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)AD_Value;					//存储器基地址
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;			//存储器数据宽度
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						//存储器地址自增，选择使能
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;							//数据传输方向
	DMA_InitStructure.DMA_BufferSize = 1;										//转运次数改为3
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;								//循环模式
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;								//存储器到存储器，选择失能
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;						//优先级，选择中等
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);								//配置DMA1的通道1
	
	/*DMA和ADC使能*/
	DMA_Cmd(DMA1_Channel1, ENABLE);							//DMA1的通道1使能
	ADC_DMACmd(ADC1, ENABLE);								//ADC1触发DMA1的信号使能
	ADC_Cmd(ADC1, ENABLE);									//ADC1使能
	
	/*ADC校准*/
	ADC_ResetCalibration(ADC1);								
	while (ADC_GetResetCalibrationStatus(ADC1) == SET);
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) == SET);
	
	/*ADC触发*/
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	//软件触发ADC开始工作
}

/**
  * 函    数：获取LUX数值
  * 参    数：无
  * 返 回 值：输出lux数据
  */
static uint16_t Get_LDR_LUX(void)
{
	float voltage = 0;	
	float R = 0;	
	voltage = AD_Value[0];
	voltage  = voltage / 4096 * 3.3f;	//计算出当前电压
	
	/*转换成LUX*/
	R = voltage / (3.3f - voltage) * 10000;
	return (40000 * pow(R, -0.6021));
}

uint16_t LDR_LuxData()
{
	float voltage = 0;	
	float R = 0;	
	uint16_t Lux = 0;
	voltage = AD_Value[0];
	voltage  = voltage / 4096 * 3.3f;
	
	R = voltage / (3.3f - voltage) * 10000;
		
	Lux = 40000 * pow(R, -0.6021);
	
	if (Lux > 9999)
	{
		Lux = 9999;
	}
	return Lux;
}


uint16_t MQ7_GetData(void)
{
    // 由于我们已配置为数字模式，直接读取DO口状态
    // 读取电平并取反（根据模块特性：通常低电平表示超限）
    	uint16_t tempData;
	tempData = !GPIO_ReadInputDataBit(MQ7_DO_GPIO_PORT, MQ7_DO_GPIO_PIN);
	return tempData;
}


/**
  * 函    数：获取MQ2传感器PPM
  * 参    数：无
  * 返 回 值：输出ppm数据
  */
//uint16_t MQ2_GetData_PPM(void)
//{
//	float  tempData = 0;
//	
//	tempData = AD_Value[1];
//	
//	float Vol = (tempData*5/4096);
//	float RS = (5-Vol)/(Vol*0.5);
//	float R0=6.64;
//	
//	float ppm = pow(11.5428*R0/RS, 0.6549f);
//	
//	return ppm;
//}

//uint16_t MQ135_GetData_PPM(void)
//{
//	float  tempData = 0;
//	
//	tempData = AD_Value[2];  // 修改为索引2，因为移除了MQ7的AD通道
//	
//	float Vol = (tempData*5/4096);
//	float RS = (5-Vol)/(Vol*0.5);
//	float R0=6.64;
//	
//	float ppm = pow(11.5428*R0/RS, 0.6549f);
//	
//	return ppm;
//}

/**
  * 函    数：获取滤波后的LUX数值
  * 参    数：输出lux数据
  * 返 回 值：无
  */
void Get_Average_LDR_LUX(uint16_t *lux)
{
	uint32_t temp = 0;  // 初始化temp为0

	for (uint8_t i = 0; i < 10; i++)
	{
		temp += Get_LDR_LUX();
		delay_ms(5);
	}
	
	temp /= 10;
	*lux = temp;
}

/**
  * 函    数：获取滤波后的MQ2传感器的PPM
  * 参    数：输出ppm数据
  * 返 回 值：无
  */
//void Get_Average_MQ2_PPM(uint16_t *ppm)
//{
//	uint32_t temp = 0;  // 初始化temp为0

//	for (uint8_t i = 0; i < 10; i++)
//	{
//		temp += MQ2_GetData_PPM();
//		delay_ms(5);
//	}
//	temp /= 10;
//	*ppm = temp;
//}

///**
//  * 函    数：获取滤波后的MQ135传感器的PPM
//  * 参    数：输出ppm数据
//  * 返 回 值：无
//  */
//void Get_Average_MQ135_PPM(uint16_t *ppm)
//{
//	uint32_t temp = 0;  // 初始化temp为0

//	for (uint8_t i = 0; i < 10; i++)
//	{
//		temp += MQ135_GetData_PPM();
//		delay_ms(5);
//	}
//	
//	temp /= 10;
//	*ppm = temp;
//}

/**
  * 函    数：读取MQ-7 DO口状态
  * 参    数：无
  * 返 回 值：0（浓度超限）或1（浓度正常）
  */
uint8_t MQ7_Get_DO_State(void)
{
	/*读取GPIO引脚电平：GPIO_ReadInputDataBit返回Bit_RESET（0）或Bit_SET（1）*/
	if (GPIO_ReadInputDataBit(MQ7_DO_GPIO_PORT, MQ7_DO_GPIO_PIN) == Bit_SET)
	{
		return 1;	// 高电平：浓度正常
	}
	else
	{
		return 0;	// 低电平：浓度超限
	}
}




