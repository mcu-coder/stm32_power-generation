#include "stm32f10x.h"                  // STM32F10x系列MCU核心头文件
#include "led.h"                        // LED驱动头文件
#include "beep.h"                       // 蜂鸣器驱动头文件
#include "usart.h"                      // 串口1驱动头文件（PC通信）
#include "delay.h"                      // 延时函数头文件
#include "oled.h"                       // OLED显示屏驱动头文件
#include "key.h"                        // 按键驱动头文件
#include "Modules.h"                    // 模块核心头文件（传感器、阈值、驱动器结构体定义）
#include "adcx.h"                       // ADC采集驱动头文件
#include "flash.h"                      // Flash存储驱动头文件
#include "usart2.h"                     // 串口2驱动头文件（蓝牙通信）
#include "usart3.h"                     // 串口3驱动头文件（语音模块通信）
#include "voltage.h"                    // 光伏电压/电流采集头文件
#include "ds18b20.h"                    // DS18B20温度传感器驱动头文件
#include "string.h"                     // 字符串处理标准库头文件
#include "relay.h"                      // 继电器驱动头文件
#include "timer.h"                      // 定时器通用头文件
#include "TIM3.h"                       // 定时器3驱动头文件
#include "TIM2.h"                       // 定时器2驱动头文件
#include "gizwits_product.h"            // 机智云产品相关头文件（数据点定义）
#include "gizwits_protocol.h"           // 机智云协议头文件（WiFi配网、数据通信）

 

#define KEY_Long1	11                  // 按键1长按标识值
#define KEY_1	1                      // 按键1短按标识值（模式切换）
#define KEY_2	2                      // 按键2短按标识值（菜单切换/光标移动）
#define KEY_3	3                      // 按键3短按标识值（参数增加/设备开启）
#define KEY_4	4                      // 按键4短按标识值（参数减少/设备关闭）


#define AUTO_MODE     1   // 自动模式
#define MANUAL_MODE   2   // 手动模式
#define SETTINGS_MODE 3   // 设置模式


#define FLASH_START_ADDR	0x0801f000	// Flash阈值存储起始地址（STM32F103C8T6 Flash尾部区域）

// 将float类型拆分为高16位uint16_t
#define FLOAT_TO_U16_HIGH(f)  ( (uint16_t)( ( *(uint32_t*)&f ) >> 16 ) )
// 将float类型拆分为低16位uint16_t
#define FLOAT_TO_U16_LOW(f)   ( (uint16_t)( *(uint32_t*)&f ) )
//#define WIFI_INVALID_MODE 0  

#ifndef GIZWITS_MODE_NORMAL
#define GIZWITS_MODE_NORMAL    0   // 正常模式
#endif
#ifndef GIZWITS_MODE_SOFTAP
#define GIZWITS_MODE_SOFTAP    1   // 软AP配网模式
#endif
#ifndef GIZWITS_MODE_AIRLINK
#define GIZWITS_MODE_AIRLINK   2   // 一键配网模式
#endif




//static uint32_t sys_millis = 0;
// 全局变量定义
extern  SensorModules sensorData;								// 传感器数据结构体（存储温度、光伏电压/电流/功率/转换率）
extern  SensorThresholdValue Sensorthreshold;		// 传感器阈值结构体（存储温度、电压、电流阈值）
extern  DriveModules driveData;									// 驱动器状态结构体（存储LED、蜂鸣器、继电器开关状态）

uint8_t mode = 1;	// 系统模式：1=自动模式，2=手动模式，3=设置模式（默认自动模式）
u8 dakai;           // 串口3通信传递变量
u8 Flag_dakai;      // 串口3接收完成标志位
uint8_t is_secondary_menu = 0;  // 二级菜单标志位：0=一级菜单，1=二级菜单
uint8_t secondary_pos = 1;      // 二级菜单光标位置：
uint8_t secondary_type = 0;     // 二级菜单类型
//float pa0_voltage = 0.0f;       
uint8_t count_m = 1;  // 手动模式菜单计数
uint8_t count_s = 1;	 // 设置模式菜单计数
extern unsigned char p[16];     // 温度显示字符串缓冲区

uint8_t last_mode = 0;  // 上一次系统模式（用于模式切换时清屏刷新显示）
extern dataPoint_t currentDataPoint;




/**
  * @brief  自动模式第一页固定菜单显示（仅显示标题，不包含动态数据）
  * @param  无
  * @retval 无
  */
void OLED_autoPage1(void)		//自动模式菜单第一页
{	
	// 显示"温度：  C"（0,0坐标开始，16x16字体）
	OLED_ShowChinese(0,0,0,16,1);	// 在(0,0)位置显示中文"温"（16x16字体，开启显示）
	OLED_ShowChinese(16,0,1,16,1);	// 在(16,0)位置显示中文"度"（16x16字体，开启显示）
	OLED_ShowChar(32,0,':',16,1);  // 在(32,0)位置显示冒号":"（16x16字体，开启显示）
	
	// 显示"电压：  V"（0,16坐标开始，16x16字体）
	OLED_ShowChinese(0,16,2,16,1);  // 在(0,16)位置显示中文"电"（16x16字体，开启显示）
	OLED_ShowChinese(16,16,3,16,1); // 在(16,16)位置显示中文"压"（16x16字体，开启显示）
	OLED_ShowChar(32,16,':',16,1);  // 在(32,16)位置显示冒号":"（16x16字体，开启显示）
	 
}

void OLED_autoPage2(void)   // 自动模式菜单第二页（预留扩展，暂未实现）
{
	

}

/**
  * @brief  自动模式第一页动态传感器数据显示（温度、电压、电流、功率、转换率）
  * @param  无
  * @retval 无
  */
void SensorDataDisplay1(void)		//传感器数据显示第一页
{
	// printf("=== SensorDataDisplay1 Executed ===\r\n");  // 打印函数执行日志（串口1输出，用于调试）
    SensorScan();	// 调用传感器扫描函数，获取最新温度、光伏电压/电流/功率数据
	 Update_Display_Parts(); // 调用数据拆分函数，将浮点型传感器数据拆分为整数+小数部分（用于OLED显示）

  OLED_autoPage1();  // 绘制自动模式固定菜单标题
    // 显示温度数据（p数组存储"XX.XC"格式字符串）
		OLED_ShowString(40,0,(u8*)p ,16,1);  // 在(40,0)位置显示温度字符串（16x16字体，开启显示）

    // 显示光伏电压（整数部分+小数点+小数部分+单位V）
    OLED_ShowNum(40,16, displayParts.volt_int, 1,16,1);  // 在(40,16)位置显示电压整数部分（1位，16x16字体）
    OLED_ShowChar(48,16,'.',16,1);                      // 在(48,16)位置显示小数点"."（16x16字体）
    OLED_ShowNum(56,16, displayParts.volt_dec, VOLT_DEC,16,1);  // 在(56,16)位置显示电压小数部分（VOLT_DEC位，16x16字体）
    OLED_ShowChar(72,16,'V',16,1);                      // 在(72,16)位置显示电压单位"V"（16x16字体）
 
  //  printf("温度：%s\r\n", p);  // p是"XX.XC"格式字符串，用%s打印
	//	printf("电压：%d.%0*dV\r\n", displayParts.volt_int, VOLT_DEC, displayParts.volt_dec);  // 整数+固定小数位（补零）
	//	printf("电流：%d.%0*dmA\r\n", displayParts.curr_int, CURR_DEC, displayParts.curr_dec);  // 同上，单位mA
	//	printf("功率：%d.%0*dW\r\n", displayParts.power_int, POWER_DEC, displayParts.power_dec);  // 同上，单位W
	//	printf("转换率：%d.%0*d%%\r\n", displayParts.eff_int, EFF_DEC, displayParts.eff_dec);  // %%打印%符号，固定小数位
}

void SensorDataDisplay2(void)		// 自动模式第二页传感器数据显示
{


}

/**
  * @brief  手动模式第一页固定菜单显示（仅显示标题，不包含设备状态）
  * @param  无
  * @retval 无
  */
void OLED_manualPage1(void)
{
	// 显示"灯光：  "（16,0坐标开始，16x16字体）
	OLED_ShowChinese(16,0,23,16,1);	// 在(16,0)位置显示中文"灯"（16x16字体，开启显示）
	OLED_ShowChinese(32,0,24,16,1);	// 在(32,0)位置显示中文"光"（16x16字体，开启显示）
	OLED_ShowChinese(48,0,25,16,1);	// 在(48,0)位置显示中文"："（16x16字体，开启显示）
	OLED_ShowChar(64,0,':',16,1);    // 在(64,0)位置显示冒号":"（16x16字体，开启显示）
 
}

/**
  * @brief  手动模式第一页设备状态显示（灯光、蜂鸣器的"开/关"状态）
  * @param  无
  * @retval 无
  */
void ManualSettingsDisplay1(void)
{
	// 显示灯光状态（根据RELAY_Flag标志位）
	if(driveData.RELAY_Flag ==1)
	{
		OLED_ShowChinese(96,0,19,16,1); 	// 灯光开启：在(96,0)位置显示中文"开"（16x16字体）
	}
	 
}

/**
  * @brief  设置模式第一页固定菜单显示（仅显示阈值标题，不包含阈值数值）
  * @param  无
  * @retval 无
  */
void OLED_settingsPage1(void)
{
	// 显示"温度阈值：  "（16,0坐标开始，16x16字体）
	OLED_ShowChinese(16,0,0,16,1);	// 在(16,0)位置显示中文"温"（16x16字体，开启显示）
	OLED_ShowChinese(32,0,1,16,1);	// 在(32,0)位置显示中文"度"（16x16字体，开启显示）
	OLED_ShowChinese(48,0,17,16,1);	// 在(48,0)位置显示中文"阈"（16x16字体，开启显示）
	OLED_ShowChinese(64,0,18,16,1);	// 在(64,0)位置显示中文"值"（16x16字体，开启显示）
	OLED_ShowChar(80,0,':',16,1);    // 在(80,0)位置显示冒号":"（16x16字体，开启显示）

	 
}

void OLED_settingsPage2(void)// 设置模式第二页固定菜单显示
{


}
void OLED_settingsPage3(void)// 设置模式第三页固定菜单显示
{

}

/**
  * @brief  设置模式第一页阈值数值显示（温度、电压、电流阈值的具体数值）
  * @param  无
  * @retval 无
  */
void SettingsThresholdDisplay1(void)//实际阈值1
{
	// 显示温度阈值数值（2位整数，单位℃）
	OLED_ShowNum(90,0, Sensorthreshold.tempValue, 2,16,1);
	
	// 显示电压阈值数值（1位整数+1位小数，单位V）
  uint8_t volt_int = (uint8_t)Sensorthreshold.pv_voltageValue;  // 电压阈值整数部分（强制类型转换）
	uint8_t volt_dec = (uint8_t)((Sensorthreshold.pv_voltageValue - volt_int) * 10 + 0.5); // 电压阈值小数部分（四舍五入）
 
	
	// 显示电流阈值数值（3位整数，单位mA）
	OLED_ShowNum(90, 32, Sensorthreshold.pv_currentValue , 3,16,1);
}

void SettingsThresholdDisplay2(void)// 设置模式第二页阈值数值显示
{

}

void SettingsThresholdDisplay3(void)// 设置模式第三页阈值数值显示
{

}

/**
  * @brief  自动模式下按键2短按计数
  * @param  无
  * @retval 返回自动模式菜单页数
  */
uint8_t SetAuto(void)  
{
 return 1;  // 自动模式只有一页，始终返回1
}

/**
  * @brief  手动模式下按键2短按计数（切换灯光/蜂鸣器控制菜单）
  * @param  无
  * @retval 返回当前手动模式菜单页码（1=灯光控制，2=蜂鸣器控制）
  */
uint8_t SetManual(void)  
{
	if(KeyNum == KEY_2)  // 检测按键2短按（菜单切换）
	{
		KeyNum = 0;        // 清空按键标识，避免重复触发
		count_m++;         // 手动模式菜单页码+1
		if (count_m > 2)   // 菜单页码超过最大页数（2页）
		{
			OLED_Clear();    // 清屏，准备重新显示第一页
			count_m = 1;     // 页码重置为1（回到灯光控制菜单）
		}
	}
	return count_m;      // 返回当前手动模式菜单页码
}

/**
  * @brief  设置模式下按键2短按计数（切换温度/电压/电流阈值设置菜单）
  * @param  无
  * @retval 返回当前设置模式菜单页码（1=温度阈值，2=电压阈值，3=电流阈值）
  */
uint8_t SetSelection(void)
{
    if(KeyNum == KEY_2 && is_secondary_menu == 0)  // 按键2短按且处于一级菜单
    {
        KeyNum = 0;                                // 清空按键标识，避免重复触发
        count_s++;                                 // 设置模式菜单页码+1
        if (count_s > 3)                           // 菜单页码超过最大页数（3页）
        {
            count_s = 1;                           // 页码重置为1（回到温度阈值设置）
        }
    }
    return count_s;                                // 返回当前设置模式菜单页码
}

/**
  * @brief  手动模式菜单光标显示（指示当前选中的控制项）
  * @param  num：光标位置（1=灯光控制，2=蜂鸣器控制）
  * @retval 无
  */
void OLED_manualOption(uint8_t num)
{
	switch(num)
	{
		case 1:	// 光标选中灯光控制
			OLED_ShowChar(0, 0,'>',16,1);  // 在(0,0)位置显示光标">"（16x16字体）
			OLED_ShowChar(0,16,' ',16,1);  // 清除蜂鸣器控制行光标（显示空格）
			OLED_ShowChar(0,32,' ',16,1);  // 清除预留行光标（显示空格）
			OLED_ShowChar(0,48,' ',16,1);  // 清除预留行光标（显示空格）
			break;
		 
	}
}

/**
  * @brief  设置模式菜单光标显示（指示当前选中的阈值设置项）
  * @param  num：光标位置（1=温度阈值，2=电压阈值，3=电流阈值，4=预留）
  * @retval 无
  */
void OLED_settingsOption(uint8_t num)
{
	static uint8_t prev_num = 1;  // 上一次光标位置（用于清除历史光标）

    // 清除上一次光标位置（仅操作光标，不影响阈值数据显示）
    switch(prev_num)
    {
        case 1: OLED_ShowChar(0, 0, ' ', 16, 1); break; // 清除温度阈值行光标（显示空格）
        case 2: OLED_ShowChar(0, 16, ' ', 16, 1); break; // 清除电压阈值行光标（显示空格）
        case 3: OLED_ShowChar(0, 32, ' ', 16, 1); break; // 清除电流阈值行光标（显示空格）
        case 4: OLED_ShowChar(0, 48, ' ', 16, 1); break; // 清除预留行光标（显示空格）
        default: break;
    }
	switch(num)
	{
		case 1:	// 光标选中温度阈值
			OLED_ShowChar(0, 0,'>',16,1);  // 在(0,0)位置显示光标">"（16x16字体）
			OLED_ShowChar(0,16,' ',16,1);  // 清除电压阈值行光标（显示空格）
			OLED_ShowChar(0,32,' ',16,1);  // 清除电流阈值行光标（显示空格）
			OLED_ShowChar(0,48,' ',16,1);  // 清除预留行光标（显示空格）
			break;
		case 2:	// 光标选中电压阈值
			OLED_ShowChar(0, 0,' ',16,1);  // 清除温度阈值行光标（显示空格）
			OLED_ShowChar(0,16,'>',16,1);  // 在(0,16)位置显示光标">"（16x16字体）
			OLED_ShowChar(0,32,' ',16,1);  // 清除电流阈值行光标（显示空格）
			OLED_ShowChar(0,48,' ',16,1);  // 清除预留行光标（显示空格）
			break;
	 
		default: break;
	}
	 prev_num = num;  // 更新上一次光标位置为当前位置
    OLED_Refresh(); // 仅刷新光标区域，数据区域保持不变
}

/**
  * @brief  自动模式控制逻辑（根据传感器数据与阈值对比，自动控制LED、蜂鸣器、继电器）
  * @param  无
  * @retval 无
  */
void AutoControl(void)//自动控制
{
	// 温度或电压超过设定阈值时，开启蜂鸣器报警
	if( (sensorData.temp > Sensorthreshold.tempValue)    // 温度超过温度阈值
 || (sensorData.pv_voltage > Sensorthreshold.pv_voltageValue)    // 电压超过电压阈值
  )                           
  {
   driveData.BEEP_Flag =1; // 任一阈值超标，蜂鸣器开启报警
  }
  else
	driveData.BEEP_Flag =0; // 无阈值超标，蜂鸣器关闭
	
    // 根据电流判断光照强度，控制LED（电流低于阈值=光照弱=LED开启；反之关闭）
	 if(sensorData.pv_current<Sensorthreshold.pv_currentValue)
		driveData.LED_Flag =0;  // LED_Flag=0对应LED_On（硬件逻辑：低电平点亮）
    else
		driveData.LED_Flag =1;  // LED_Flag=1对应LED_Off（硬件逻辑：高电平熄灭）
	 
}

/**
  * @brief  手动模式控制逻辑（根据按键操作，手动控制LED、蜂鸣器）
  * @param  num：当前手动模式菜单页码（1=灯光控制，2=蜂鸣器控制）
  * @retval 无
  */
void ManualControl(uint8_t num)
{
	switch(num)
	{
		case 1:  // 灯光控制菜单（控制继电器，对应灯光）
            if(KeyNum == KEY_3)  // 按键3短按=开启灯光
            {
                driveData.RELAY_Flag = 1;  // 继电器开启标志位设1
							  currentDataPoint.valuerelay = driveData.RELAY_Flag; // 同步到机智云
                KeyNum = 0;                // 清空按键标识，避免重复触发
               
            }
            if(KeyNum == KEY_4)  // 按键4短按=关闭灯光
            {
                driveData.RELAY_Flag = 0;  // 继电器关闭标志位设0
							  currentDataPoint.valuerelay = driveData.RELAY_Flag; // 同步到机智云
                KeyNum = 0;                // 清空按键标识，避免重复触发
               
            }
            break;
 
		default: break;
	}
}

/**
  * @brief  驱动器执行函数（根据driveData结构体标志位，控制LED、蜂鸣器、继电器硬件）
  * @param  无
  * @retval 无
  */
void Control_Manager(void)
{
    // 控制LED（LED_Flag=1=熄灭，LED_Flag=0=点亮，硬件逻辑相反）
    if(driveData.LED_Flag )
    {	
      LED_Off();  // LED熄灭
    }
    else 
    {
      LED_On ();  // LED点亮
    }
		
		 
	 
}

/**
  * @brief  阈值设置函数（根据按键操作，修改温度、电压、电流阈值，并同步到云端）
  * @param  num：当前设置模式菜单页码（1=温度阈值，2=电压阈值，3=电流阈值）
  * @retval 无
  */
void ThresholdSettings(uint8_t num)
{
	switch (num)
	{
		// 温度阈值修改（范围20-50℃，步长1℃）
		case 1:
			if (KeyNum == KEY_3)  // 按键3短按=温度阈值+1
			{
				KeyNum = 0;  // 清空按键标识，避免重复触发
				Sensorthreshold.tempValue += 1;  // 温度阈值+1
				if (Sensorthreshold.tempValue > 50)  // 超过最大值50℃
				{
					Sensorthreshold.tempValue = 20;  // 循环到最小值20℃
				}
				// 同步温度阈值到云端数据点
				currentDataPoint.valuetemp_value = Sensorthreshold.tempValue;
			}
		 
			break;
		
	 
        default: break;
	}   
}

/**
  * @brief  从Flash读取阈值数据（系统上电时加载掉电保存的阈值）
  * @param  无
  * @retval 无
  */
void FLASH_ReadThreshold()
{
	   // 1. 读取温度阈值（Flash中存储为u16，读取后强转为u8）
    Sensorthreshold.tempValue = (uint8_t)FLASH_R(FLASH_START_ADDR);
    
    // 2. 读取电压阈值（Flash中存储为高16位+低16位，拼接为32位后转回float）
    uint16_t volt_high = FLASH_R(FLASH_START_ADDR + 2);  // 读取电压阈值高16位
    uint16_t volt_low = FLASH_R(FLASH_START_ADDR + 4);   // 读取电压阈值低16位
    uint32_t volt_32bit = ( (uint32_t)volt_high << 16 ) | volt_low;  // 拼接为32位整数
    Sensorthreshold.pv_voltageValue = *(float*)&volt_32bit;  // 32位整数转回float类型电压阈值
    
    // 3. 读取电流阈值（Flash中直接存储为u16，直接读取）
    Sensorthreshold.pv_currentValue = FLASH_R(FLASH_START_ADDR + 6);
}




// 配网模式扫描函数（仅上电3秒内长按KEY3/KEY4触发）
void ScanGizwitsMode(void)
{
    static uint8_t key3_flag = 0;
    static uint8_t key4_flag = 0;
  //  static uint8_t last_mode = WIFI_INVALID_MODE; 

    if(!KEY3)
    {
        delay_ms(20); // 消抖
        if(!KEY3 && key3_flag == 0)
        {
            key3_flag = 1;
            // 清屏显示配网提示
            OLED_Clear();
            OLED_ShowChinese(32,16,26,16,1); //热
            OLED_ShowChinese(48,16,27,16,1); //点                               
           
            OLED_Refresh();
					 delay_ms(1000); 
            gizwitsSetMode(WIFI_SOFTAP_MODE);
          

        }
    }
    else
    {
        key3_flag = 0;
    }
    
    // ===== KEY4（一键配网）检测 =====
    if(!KEY4)
    {
        delay_ms(20); // 消抖
        if(!KEY4 && key4_flag == 0)
        {
            key4_flag = 1;
            // 清屏显示配网提示
            OLED_Clear();
            OLED_ShowChinese(32,16,28,16,1); //一
            OLED_ShowChinese(48,16,29,16,1); //键
            OLED_ShowChinese(64,16,30,16,1); //配
            

        }
    }
    else
    {
        key4_flag = 0;
    }
}


/**
  * @brief  主函数（系统初始化、主循环逻辑）
  * @param  无
  * @retval int：返回值（实际无意义，符合C语言标准）
  */
int main(void)
{ 
    SystemInit();    // 配置系统时钟为72MHz（STM32F103C8T6最大支持时钟）
    delay_init(72);  // 初始化延时函数（基于72MHz系统时钟）
    ADCX_Init();     // 初始化ADC（用于光伏电压/电流采集）
    LED_Init();      // 初始化LED（指示灯）
    BEEP_Init();     // 初始化蜂鸣器（报警）
    BEEP_OFF;        // 初始状态：蜂鸣器关闭
    

    // 从Flash读取阈值（上电加载掉电保存的阈值）
    delay_ms(100);   // 延时100ms，确保Flash稳定
    FLASH_ReadThreshold();

    TIM2_Init(72-1,1000-1);  // 初始化定时器2（2ms定时中断，用于系统滴答计数）
	  TIM3_Int_Init(1000-1,72-1);		// 初始化定时器3（1ms定时中断，用于按键扫描/数据刷新）
    // 状态管理静态变量（主循环中保持状态）
    static uint32_t last_sensor_time = 0; // 传感器扫描时间戳（控制扫描频率）
    static uint32_t last_display_time = 0; // 显示刷新时间戳（控制显示频率）
    
    // 阈值合法性校验：若Flash中阈值超出合理范围，初始化默认阈值并写入Flash
if (Sensorthreshold.tempValue < 20 || Sensorthreshold.tempValue > 50 ||
    Sensorthreshold.pv_voltageValue < 1.0f || Sensorthreshold.pv_voltageValue > 5.0f ||
    Sensorthreshold.pv_currentValue < 10 || Sensorthreshold.pv_currentValue > 50)
{
    // 初始化默认阈值
    Sensorthreshold.tempValue = 30;        // 温度默认阈值：30℃
    float default_volt = 3.0f;             // 电压默认阈值：3.0V（临时变量，避免字面量直接赋值）
    Sensorthreshold.pv_voltageValue = default_volt; // 电压阈值赋值
    Sensorthreshold.pv_currentValue = 30;  // 电流默认阈值：30mA
    
    // 将默认阈值拆分为高16位+低16位（用于Flash存储）
    uint16_t volt_high = FLOAT_TO_U16_HIGH(default_volt);
    uint16_t volt_low = FLOAT_TO_U16_LOW(default_volt);
    
    // 写入默认阈值到Flash
    FLASH_W(FLASH_START_ADDR, 30, volt_high, volt_low);
    FLASH_Unlock();
    FLASH_ProgramHalfWord(FLASH_START_ADDR + 6, 30);
    FLASH_Lock();
  
}
    
  userInit();       // 机智云用户初始化（数据点初始化等）
  gizwitsInit();    // 机智云协议初始化（WiFi配网、数据通信初始化）
	gizwitsSetMode(WIFI_AIRLINK_MODE); //默认一键配网
	delay_ms(200);
	
  printf("Start \n");  

   delay_ms(200);
    while (1)  // 主循环（无限循环，处理系统所有逻辑）
 {	
	 
	 
         gizwitsHandle((dataPoint_t *)&currentDataPoint);

        // ==================== 获取当前系统时间（基于定时器2滴答计数）====================
         uint32_t current_time = delay_get_tick();
	 
	 
	  

        
        // ==================== 控制传感器扫描频率（每200ms扫描一次）====================
        if(current_time - last_sensor_time > 100) // 100个定时器2滴答 = 100*2ms=200ms
        {
            SensorScan(); 	// 扫描传感器，获取最新温度、光伏电压/电流/功率数据
					  Update_Display_Parts(); // 拆分传感器数据为整数+小数部分（用于OLED显示）
            last_sensor_time = current_time; // 更新传感器扫描时间戳
        }
        
        // ==================== 立即处理按键（模式切换、参数调整等）====================
        uint8_t current_key_num = KeyNum; // 保存当前按键值（避免被后续逻辑修改）
        
        // 模式切换按键处理（仅响应模式切换相关按键）
        if(current_key_num != 0)
        {
            switch(mode)
            {
                case AUTO_MODE: // 当前为自动模式
                    if(current_key_num == KEY_1) // 按键1短按：切换到手动模式
                    {
                        mode = MANUAL_MODE;
                        count_m = 1; // 手动模式菜单页码重置为1（灯光控制）
                        // 切换到手动模式时，关闭LED和蜂鸣器
                        driveData.LED_Flag = 0;
                        driveData.BEEP_Flag = 0;
											 
                        KeyNum = 0; // 清空按键标识
                    }
                    else if(current_key_num == KEY_Long1) // 按键1长按：切换到设置模式
                    {
                        mode = SETTINGS_MODE;
                        count_s = 1; // 设置模式菜单页码重置为1（温度阈值）
                        KeyNum = 0; // 清空按键标识
                    }
                    break;
                    
                case MANUAL_MODE: // 当前为手动模式
                    if(current_key_num == KEY_1) // 按键1短按：切换到自动模式
                    {
                        mode = AUTO_MODE;
                        KeyNum = 0; // 清空按键标识
                    }
                    break;
                    
                case SETTINGS_MODE: // 当前为设置模式
                    // 设置模式内部按键在模式处理中单独处理
                    break;
            }
        }
        
        // ==================== 模式切换时清屏并刷新菜单 ====================
        if(last_mode != mode)
        {
            OLED_Clear(); // 清屏，避免不同模式菜单重叠
            
            // 绘制新模式的固定菜单标题
            switch(mode)
            {
                case AUTO_MODE:
                    OLED_autoPage1(); // 绘制自动模式第一页固定菜单
                    break;
                case MANUAL_MODE:
                    OLED_manualPage1(); // 绘制手动模式第一页固定菜单
                    break;
                case SETTINGS_MODE:
                    OLED_settingsPage1(); // 绘制设置模式第一页固定菜单
                    break;
            }
            OLED_Refresh(); // 立即刷新OLED显示
			      last_mode = mode; // 更新上一次模式为当前模式
        }
        
        // ==================== 各模式核心逻辑处理 ====================
        switch(mode)
        {
            case AUTO_MODE: // 自动模式
                // 显示传感器动态数据
                SensorDataDisplay1();	// 显示温度、电压、电流、功率、转换率
                AutoControl(); // 自动控制LED、蜂鸣器、继电器
                Control_Manager(); // 执行驱动器控制（硬件操作）
                break;
                
            case MANUAL_MODE: // 手动模式
            {
                static uint8_t manual_page_initialized = 0; // 手动模式页面初始化标志
                static uint8_t last_manual_count = 0; // 上一次手动模式菜单页码
                static uint8_t last_LED_Flag = 0; // 上一次LED状态
                static uint8_t last_BEEP_Flag = 0; // 上一次蜂鸣器状态
                static uint8_t force_refresh = 0;  // 强制刷新标志（模式切换时使用）
                
                // 模式切换到手动模式时，初始化状态
                if(last_mode != mode)
                {
                    manual_page_initialized = 0;
                    last_manual_count = 0;
                    last_LED_Flag = driveData.LED_Flag;
                    last_BEEP_Flag = driveData.BEEP_Flag;
                    force_refresh = 1;  // 设置强制刷新标志
                    
                    count_m = 1; // 光标默认选中灯光控制
                    // 初始状态：关闭LED和蜂鸣器
                    driveData.LED_Flag = 0;
                    driveData.BEEP_Flag = 0;
									  driveData.RELAY_Flag = 0;
									    // 同步初始状态到机智云
                    currentDataPoint.valueled = driveData.LED_Flag;
                    currentDataPoint.valuebeep = driveData.BEEP_Flag;
                    currentDataPoint.valuerelay = driveData.RELAY_Flag;
                }
                
                uint8_t current_manual_count = SetManual(); // 获取当前手动模式菜单页码
                
                // 检测设备状态是否变化（变化则需要刷新显示）
                uint8_t need_refresh = 0;
                if(driveData.LED_Flag != last_LED_Flag || driveData.BEEP_Flag != last_BEEP_Flag)
                {
                    need_refresh = 1;
                    last_LED_Flag = driveData.LED_Flag; // 更新LED状态
                    last_BEEP_Flag = driveData.BEEP_Flag; // 更新蜂鸣器状态
                }
                
                // 页面未初始化、页码变化、设备状态变化或强制刷新时，重绘页面
                if(!manual_page_initialized || current_manual_count != last_manual_count || need_refresh || force_refresh)
                {
                    OLED_manualPage1();          // 绘制固定菜单标题
                    OLED_manualOption(current_manual_count); // 显示当前光标位置
                    ManualSettingsDisplay1();    // 显示设备状态（开/关）
                    manual_page_initialized = 1; // 标记页面已初始化
                    last_manual_count = current_manual_count; // 更新上一次页码
                    force_refresh = 0;  // 清除强制刷新标志
                    OLED_Refresh(); // 强制刷新显示
                }
                
                // 处理手动模式按键（控制设备开关）
                if(current_key_num != 0)
                {
                    ManualControl(current_manual_count); // 执行手动控制逻辑
                    OLED_manualPage1();          // 重绘固定菜单标题
                    OLED_manualOption(current_manual_count); // 重绘光标
                    ManualSettingsDisplay1();    // 重绘设备状态
                    OLED_Refresh(); // 按键操作后立即刷新显示
                    KeyNum = 0; // 清空按键标识
                }
                
                // 确保显示内容始终正确    
                OLED_manualPage1();          // 绘制固定菜单标题
                OLED_manualOption(current_manual_count); // 显示光标
                ManualSettingsDisplay1();    // 显示设备状态
                
                Control_Manager(); // 执行驱动器控制（硬件操作）
                break;
            }
                
            case SETTINGS_MODE: // 设置模式
            {
                static uint8_t is_threshold_page_inited = 0; // 设置模式页面初始化标志
                uint8_t curr_count_s = SetSelection(); // 获取当前设置模式菜单页码
                
                 
                
                // 正常显示逻辑（一级菜单）
                if (is_secondary_menu == 1)
                {
                    // 二级菜单显示（暂未实现）
                }
                else
                {
                    // 一级菜单显示
                    if (curr_count_s >= 1 && curr_count_s <= 4) // 页码在有效范围内
                    {
                        if (is_threshold_page_inited == 0) // 页面未初始化
                        {
                            OLED_settingsPage1(); // 绘制固定菜单标题
                            is_threshold_page_inited = 1; // 标记页面已初始化
                        }
                    }
                    OLED_settingsOption(curr_count_s); // 显示当前光标
                    SettingsThresholdDisplay1(); // 显示阈值数值
                }
                break;
            }
        }
        
        // ==================== 控制OLED显示刷新频率（每50ms刷新一次）====================
        if(current_time - last_display_time > 25) // 25个定时器2滴答 = 25*2ms=50ms
        {
            OLED_Refresh(); // 刷新OLED显示
            last_display_time = current_time; // 更新显示刷新时间戳
        }
		 
				
    }
}

