/**
************************************************************
* @file         gizwits_product.c
* @brief        Gizwits control protocol processing, and platform-related hardware initialization 
* @author       Gizwits
* @date         2017-07-19
* @version      V03030000
* @copyright    Gizwits
*
* @note         Gizwits is only for smart hardware
*               Gizwits Smart Cloud for Smart Products
*               Links | Value Added | Open | Neutral | Safety | Own | Free | Ecology
*               www.gizwits.com
*
***********************************************************/
#include <stdio.h>
#include <string.h>
#include "gizwits_product.h"
#include "usart2.h"
#include "Modules.h"
#include "oled.h"
#include "usart.h"

#define FLASH_START_ADDR	0x0801f000	// 与 main.c 中的地址保持统一
#define FLOAT_TO_U16_HIGH(f)  ( (uint16_t)( ( *(uint32_t*)&f ) >> 16 ) )
#define FLOAT_TO_U16_LOW(f)   ( (uint16_t)( *(uint32_t*)&f ) )
void FLASH_W(uint32_t addr, uint16_t dat, uint16_t dat2, uint16_t dat3);

extern SensorModules sensorData;								//声明传感器数据结构体变量
extern SensorThresholdValue Sensorthreshold;		//声明传感器阈值结构体变量（已存在，无需新增）
extern DriveModules driveData;									//声明驱动器状态结构体变量
extern uint8_t mode;                  // 全局模式变量（1=自动，2=手动，3=设置）
extern uint8_t count_m;               // 手动模式计数
extern uint8_t last_mode;             
static uint32_t timerMsCount;

/** Current datapoint */
dataPoint_t currentDataPoint;

/**@} */
/**@name Gizwits User Interface
* @{
*/

/**
* @brief Event handling interface

* Description:

* 1. Users can customize the changes in WiFi module status

* 2. Users can add data points in the function of event processing logic, such as calling the relevant hardware peripherals operating interface

* @param [in] info: event queue
* @param [in] data: protocol data
* @param [in] len: protocol data length
* @return NULL
* @ref gizwits_protocol.h
*/
int8_t gizwitsEventProcess(eventInfo_t *info, uint8_t *gizdata, uint32_t len)
{
    uint8_t i = 0;
    dataPoint_t *dataPointPtr = (dataPoint_t *)gizdata;
  //  moduleStatusInfo_t *wifiData = (moduleStatusInfo_t *)gizdata;
   // protocolTime_t *ptime = (protocolTime_t *)gizdata;
    
#if MODULE_TYPE
    gprsInfo_t *gprsInfoData = (gprsInfo_t *)gizdata;
#else
  //  moduleInfo_t *ptModuleInfo = (moduleInfo_t *)gizdata;
#endif

    if((NULL == info) || (NULL == gizdata))
    {
        return -1;
    }

    for(i=0; i<info->num; i++)
    {
switch(info->event[i])
{

case EVENT_beep:
{
    currentDataPoint.valuebeep = dataPointPtr->valuebeep;
    GIZWITS_LOG("Evt: EVENT_beep %d \n", currentDataPoint.valuebeep);
    if(0x01 == currentDataPoint.valuebeep)
    {
        driveData.BEEP_Flag =1;
    }
    else
    {
        driveData.BEEP_Flag =0;
    }
    break;
}
case EVENT_led:
{
    currentDataPoint.valueled = dataPointPtr->valueled;
   // GIZWITS_LOG("Evt: EVENT_led %d \n", currentDataPoint.valueled);
    if(0x01 == currentDataPoint.valueled)
    {
        driveData.LED_Flag = 1;
    }
    else
    {
        driveData.LED_Flag = 0;
    }
    break;
}
case EVENT_relay:
{
    currentDataPoint.valuerelay = dataPointPtr->valuerelay;
  //  GIZWITS_LOG("Evt: EVENT_relay %d \n", currentDataPoint.valuerelay);
    if(0x01 == currentDataPoint.valuerelay)
    {
        driveData.RELAY_Flag = 1;
    }
    else
    {
        driveData.RELAY_Flag = 0 ;
    }
    break;
}
case EVENT_mode:
{
    currentDataPoint.valuemode = dataPointPtr->valuemode;
    switch(currentDataPoint.valuemode)
    {
    case mode_VALUE0:  // 云端下发「自动模式」指令
        mode = AUTO_MODE;
        last_mode = mode;
        OLED_Clear();
        count_m = 1;
        break;
    case mode_VALUE1:  // 云端下发「手动模式」指令
        mode = MANUAL_MODE;
        last_mode = mode;
        OLED_Clear();
        count_m = 1;
        driveData.LED_Flag = 0;
        driveData.BEEP_Flag = 0;
        driveData.RELAY_Flag = 0;
        break;
    default:
        break;
    }
    break;
}
// 温度阈值远程修改（EVENT_temp_value）
case EVENT_temp_value:
{
    currentDataPoint.valuetemp_value = dataPointPtr->valuetemp_value;
    uint8_t new_temp = currentDataPoint.valuetemp_value;  // 局部变量
    // 合法性校验：和手动设置一致，超出则取边界值
    if (new_temp < 20) new_temp = 20;
    if (new_temp > 50) new_temp = 50;
    // 更新阈值结构体
    Sensorthreshold.tempValue = new_temp;
    // 写入Flash
    FLASH_W(FLASH_START_ADDR, Sensorthreshold.tempValue, Sensorthreshold.pv_voltageValue, Sensorthreshold.pv_currentValue);
    break;
}
// 电压阈值远程修改（EVENT_voltageValue）
case EVENT_voltageValue:
{
    currentDataPoint.valuevoltageValue = dataPointPtr->valuevoltageValue;
    float new_volt = currentDataPoint.valuevoltageValue;  // 局部变量
    // 合法性校验
    if (new_volt < 1.0f) new_volt = 1.0f;
    if (new_volt > 5.0f) new_volt = 5.0f;
    new_volt = ( (uint16_t)(new_volt * 2 + 0.5f) ) / 2.0f;  // 0.5V 步长对齐
    // 更新阈值结构体
    Sensorthreshold.pv_voltageValue = new_volt;
    currentDataPoint.valuevoltageValue = new_volt;
    // 拆分电压写入Flash
    uint16_t volt_high = FLOAT_TO_U16_HIGH(new_volt);  // 局部变量
    uint16_t volt_low = FLOAT_TO_U16_LOW(new_volt);    // 局部变量
    FLASH_W(FLASH_START_ADDR,
            (uint16_t)Sensorthreshold.tempValue,
            volt_high,
            volt_low);
    FLASH_Unlock();
    FLASH_ProgramHalfWord(FLASH_START_ADDR + 6, Sensorthreshold.pv_currentValue);
    FLASH_Lock();
    break;
}
// 电流阈值远程修改（EVENT_pv_currentValue）
case EVENT_pv_currentValue:
{
    currentDataPoint.valuepv_currentValue = dataPointPtr->valuepv_currentValue;
    uint16_t new_curr = currentDataPoint.valuepv_currentValue;  // 局部变量
    // 合法性校验：和手动设置一致（10-50mA），超出取边界
    if (new_curr < 10) new_curr = 10;
    if (new_curr > 70) new_curr = 70;
    // 步长对齐：5mA倍数
    new_curr = ((new_curr + 2) / 5) * 5;
    // 更新阈值结构体
    Sensorthreshold.pv_currentValue = new_curr;
    // 写入Flash
    FLASH_W(FLASH_START_ADDR, Sensorthreshold.tempValue, Sensorthreshold.pv_voltageValue, Sensorthreshold.pv_currentValue);
    break;
}
// 以下其他case（WIFI_SOFTAP、WIFI_AIRLINK等）也建议加{}，保持风格一致
case WIFI_SOFTAP:
{
    break;
}
case WIFI_AIRLINK:
{
    break;
}
case WIFI_STATION:
{
    break;
}
case WIFI_CON_ROUTER:
{
    break;
}
case WIFI_DISCON_ROUTER:
{
    break;
}
case WIFI_CON_M2M:
{
    break;
}
case WIFI_DISCON_M2M:
{
    break;
}
case WIFI_RSSI:
{
   // GIZWITS_LOG("RSSI %d\n", wifiData->rssi);
    break;
}
case TRANSPARENT_DATA:
{
   // GIZWITS_LOG("TRANSPARENT_DATA \n");
    break;
}
case WIFI_NTP:
{
  //  GIZWITS_LOG("WIFI_NTP : [%d-%d-%d %02d:%02d:%02d][%d] \n",ptime->year,ptime->month,ptime->day,ptime->hour,ptime->minute,ptime->second,ptime->ntp);
    break;
}
case MODULE_INFO:
{
  //  GIZWITS_LOG("MODULE INFO ...\n");
#if MODULE_TYPE
  //  GIZWITS_LOG("GPRS MODULE ...\n");
  //  GIZWITS_LOG("moduleType : [%d] \n",gprsInfoData->Type);
#else
   // GIZWITS_LOG("WIF MODULE ...\n");
   // GIZWITS_LOG("moduleType : [%d] \n",ptModuleInfo->moduleType);
#endif
    break;
}
default:
{
    break;
}
}
				
				
    }

    return 0;
}

/**
* User data acquisition

* Here users need to achieve in addition to data points other than the collection of data collection, can be self-defined acquisition frequency and design data filtering algorithm

* @param none
* @return none
*/
void userHandle(void)
{
    currentDataPoint.valuetemp =  sensorData.temp;//Add Sensor Data Collection
    currentDataPoint.valuepv_voltage = sensorData.pv_voltage ;//Add Sensor Data Collection
    currentDataPoint.valuepv_efficiency =  sensorData.pv_efficiency;//Add Sensor Data Collection
    currentDataPoint.valuepv_power =  sensorData.pv_power;//Add Sensor Data Collection
    
	
	  currentDataPoint.valuetemp_value = Sensorthreshold.tempValue;       // 温度阈值
    currentDataPoint.valuevoltageValue = Sensorthreshold.pv_voltageValue; // 电压阈值
    currentDataPoint.valuepv_currentValue = Sensorthreshold.pv_currentValue; // 电流阈值
	
 //  驱动器（蜂鸣器、LED、继电器）状态同步（核心：手动/自动模式均生效）
    currentDataPoint.valuebeep = driveData.BEEP_Flag;
    currentDataPoint.valueled = driveData.LED_Flag;
    currentDataPoint.valuerelay = driveData.RELAY_Flag;
	
	if(mode == AUTO_MODE)
    {
        currentDataPoint.valuemode = mode_VALUE0;  // 自动→对应云端mode_VALUE0
    }
    else if(mode == MANUAL_MODE)
    {
        currentDataPoint.valuemode = mode_VALUE1;  // 手动→对应云端mode_VALUE1
    }
    
}

/**
* Data point initialization function

* In the function to complete the initial user-related data
* @param none
* @return none
* @note The developer can add a data point state initialization value within this function
*/
void userInit(void)
{
     memset((uint8_t*)&currentDataPoint, 0, sizeof(dataPoint_t));
    /** Warning !!! DataPoint Variables Init , Must Within The Data Range **/ 
    currentDataPoint.valuebeep = 0;
    currentDataPoint.valueled = 0;
    currentDataPoint.valuerelay = 0;
    currentDataPoint.valuemode = mode_VALUE0;  
    currentDataPoint.valuetemp_value = Sensorthreshold.tempValue;       // 温度阈值（从Flash读取后的值）
    currentDataPoint.valuevoltageValue = Sensorthreshold.pv_voltageValue; // 电压阈值（从Flash读取后的值）
    currentDataPoint.valuepv_currentValue = Sensorthreshold.pv_currentValue; // 电流阈值（从Flash读取后的值）
    currentDataPoint.valuetemp = 0 ;
    currentDataPoint.valuepv_voltage = 0;
    currentDataPoint.valuepv_efficiency = 0 ;
    currentDataPoint.valuepv_power = 0;

}


/**
* @brief  gizTimerMs

* millisecond timer maintenance function ,Millisecond increment , Overflow to zero

* @param none
* @return none
*/
void gizTimerMs(void)
{
    timerMsCount++;
}

/**
* @brief gizGetTimerCount

* Read system time, millisecond timer

* @param none
* @return System time millisecond
*/
uint32_t gizGetTimerCount(void)
{
    return timerMsCount;
}

/**
* @brief mcuRestart

* MCU Reset function

* @param none
* @return none
*/
void mcuRestart(void)
{
  __set_FAULTMASK(1);
   NVIC_SystemReset();
}
/**@} */

/**
* @brief TIMER_IRQ_FUN

* Timer Interrupt handler function

* @param none
* @return none
*/
void TIMER_IRQ_FUN(void)
{
  gizTimerMs();
}

/**
* @brief UART_IRQ_FUN

* UART Serial interrupt function ，For Module communication

* Used to receive serial port protocol data between WiFi module

* @param none
* @return none
*/
void UART_IRQ_FUN(void)
{
  uint8_t value = 0;
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  // 增加接收中断判断（避免误触发）
  {
      value = USART_ReceiveData(USART2);  // 取消注释，接收云端数据
      gizPutData(&value, 1);              // 传入机智云协议栈解析
		 USART_ClearITPendingBit(USART2, USART_IT_RXNE); // 新增：清除中断标志位（避免漏收）
  }
}


/**
* @brief uartWrite

* Serial write operation, send data to the WiFi module

* @param buf      : Data address
* @param len       : Data length
*
* @return : Not 0,Serial send success;
*           -1，Input Param Illegal
*/
int32_t uartWrite(uint8_t *buf, uint32_t len)
{
    uint32_t i = 0;
    
    if(NULL == buf)
    {
        return -1;
    }
    
    #ifdef PROTOCOL_DEBUG
   // GIZWITS_LOG("MCU2WiFi[%4d:%4d]: ", gizGetTimerCount(), len);
    for(i=0; i<len; i++)
    {
      //  GIZWITS_LOG("%02x ", buf[i]);
    }
   // GIZWITS_LOG("\n");
    #endif

    for(i=0; i<len; i++)
    {
        USART_SendData(USART2, buf[i]);//STM32 test demo
			while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
        //Serial port to achieve the function, the buf[i] sent to the module
        if(i >=2 && buf[i] == 0xFF)
        {
					USART_SendData(USART2,0x55);
          while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
          //Serial port to achieve the function, the 0x55 sent to the module
          //USART_SendData(UART, 0x55);//STM32 test demo
        }
    }

    
    return len;
}



