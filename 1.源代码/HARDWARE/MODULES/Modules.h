#ifndef	__MODULES_H_
#define __MODULES_H_

#include "stm32f10x.h"                  // Device header

#define PV_PANEL_AREA  0.0036f  // 光伏板面积0.06m×0.06m
/************************** 显示配置（全局统一宏定义） **************************/
#define VOLT_DEC  2   // 电压小数位数（2位）
#define CURR_DEC  2   // 电流小数位数（2位）
#define POWER_DEC 3   // 功率小数位数（3位）
#define TEMP_DEC  1   // 温度小数位数（1位）
#define EFF_DEC   1   // 转化率小数位数（1位，百分比显示）


typedef enum
{
    AUTO_MODE = 1,    // 自动模式
    MANUAL_MODE,      // 手动模式（值=2）
    SETTINGS_MODE     // 设置模式（值=3）
} MODE_PAGES;  // 枚举类型名，所有文件可直接使用

/************************** 显示拆分结果结构体（存储整数/小数部分） **************************/
typedef struct
{
    // 光伏参数拆分结果
    uint32_t volt_int;   // 电压整数部分
    uint32_t volt_dec;   // 电压小数部分
    uint32_t curr_int;   // 电流整数部分
    uint32_t curr_dec;   // 电流小数部分
    uint32_t power_int;  // 功率整数部分
    uint32_t power_dec;  // 功率小数部分
    // 温度参数拆分结果
    uint32_t temp_int;   // 温度整数部分
    uint32_t temp_dec;   // 温度小数部分
	  uint32_t eff_int;   // 转化率整数部分
    uint32_t eff_dec;   // 转化率小数部分
} DisplayPartsTypeDef;

/************************** 传感器数据结构体（含光伏参数） **************************/
typedef struct
{

    float temp;         // 温度（DS18B20）
    // 光伏板参数
    float pv_voltage;   // 光伏板电压（V）
    float pv_current;   // 光伏板电流（mA）
    float pv_power;     // 光伏板功率（W）
	  float pv_efficiency; // 光伏板转化率（%，计算后结果）
} SensorModules;

/************************** 传感器阈值结构体 **************************/
typedef struct
{
    float tempValue;//温度阈值
	  float pv_voltageValue;//电压阈值
	  float pv_currentValue;//电流阈值

} SensorThresholdValue;

/************************** 驱动器状态结构体 **************************/
typedef struct
{
    uint8_t LED_Flag;
    uint8_t BEEP_Flag;
	  uint8_t RELAY_Flag;

} DriveModules;


void Float_To_Int_Dec(float value, uint8_t decimal_len, uint32_t *int_part, uint32_t *dec_part);
/************************** 全局变量声明 **************************/
extern SensorModules sensorData;                // 传感器原始数据
extern SensorThresholdValue Sensorthreshold;    // 传感器阈值
extern DriveModules driveData;                  // 驱动器状态
extern DisplayPartsTypeDef displayParts;        // 显示拆分结果（整数/小数）

/************************** 函数声明 **************************/
void SensorScan(void);                          // 传感器总扫描（含光伏采集）
void PV_Collect_Init(void);                     // 光伏采集初始化（ADC+模拟开关）
void Update_Display_Parts(void);                // 更新显示拆分结果（核心接口）

#endif

