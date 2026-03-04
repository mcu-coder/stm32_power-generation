#include "Modules.h"
#include "ds18b20.h"
#include "voltage.h"
#include "delay.h"
#include <stdlib.h>
#include "stdio.h"

/************************** 全局变量定义 **************************/
SensorModules sensorData = {0};
SensorThresholdValue Sensorthreshold = {0};
DriveModules driveData = {0};
DisplayPartsTypeDef displayParts = {0};  // 显示拆分结果全局变量
unsigned char p[16] = " ";


/************************** 状态变量 **************************/
static uint8_t temp_read_state = 0;
static uint8_t pv_read_state = 0;
static uint32_t temp_start_time = 0;
static uint32_t last_sensor_update[7] = {0};


/************************** 传感器更新间隔 **************************/
#define TEMP_UPDATE_INTERVAL    400   // 800ms（温度）
#define PV_UPDATE_INTERVAL      200    // 400ms（光伏）


/************************** 内部函数：浮点拆分（原FloatToIntDec） **************************/
void Float_To_Int_Dec(float value, uint8_t decimal_len, uint32_t *int_part, uint32_t *dec_part)
{

    
    float multiplier = 1.0f;
    for (uint8_t i = 0; i < decimal_len; i++)
    {
        multiplier *= 10.0f;
    }
  
    // 四舍五入计算
    uint32_t temp = (uint32_t)(value * multiplier + 0.5f);
 
    
    *int_part = temp / (uint32_t)multiplier;
    *dec_part = temp % (uint32_t)multiplier;
  
}

/************************** 对外接口：更新显示拆分结果 **************************/
void Update_Display_Parts(void)
{
	//转换效率（%）=（实际功率 × 温度校正系数）/（1000 × 光伏板面积）× 100
	  /************************** 计算光伏板转化率（用宏定义代入面积） **************************/
   const float TEMP_COEFF = -0.0037f;  
    const float DENOMINATOR = 1000.0f * PV_PANEL_AREA; 
    float temp_correction = 1.0f + (sensorData.temp - 25.0f) * TEMP_COEFF;
    
    // 计算转化率
    sensorData.pv_efficiency = (sensorData.pv_power * temp_correction) / DENOMINATOR * 100.0f;
    // 限制范围
    sensorData.pv_efficiency = (sensorData.pv_efficiency < 0.0f) ? 0.0f : sensorData.pv_efficiency;
    sensorData.pv_efficiency = (sensorData.pv_efficiency > 100.0f) ? 100.0f : sensorData.pv_efficiency;
    
    // 拆分光伏电压（2位小数）
    Float_To_Int_Dec(sensorData.pv_voltage, VOLT_DEC, &displayParts.volt_int, &displayParts.volt_dec);
    // 拆分光伏电流（2位小数）
    Float_To_Int_Dec(sensorData.pv_current, CURR_DEC, &displayParts.curr_int, &displayParts.curr_dec);
    // 拆分光伏功率（3位小数）
    Float_To_Int_Dec(sensorData.pv_power, POWER_DEC, &displayParts.power_int, &displayParts.power_dec);
    // 拆分温度（1位小数）
    Float_To_Int_Dec(sensorData.temp, TEMP_DEC, &displayParts.temp_int, &displayParts.temp_dec);
		Float_To_Int_Dec(sensorData.pv_efficiency, EFF_DEC, &displayParts.eff_int, &displayParts.eff_dec);
  
}



/************************** 传感器总扫描函数 **************************/
void SensorScan(void)
{
    static short temperature = 0;
    uint32_t current_time = delay_get_tick();
    
    // ==================== DS18B20温度读取 ====================
    switch(temp_read_state)
    {
        case 0:
            if(current_time - last_sensor_update[0] > TEMP_UPDATE_INTERVAL)
            {
                DS18B20_Start();
                temp_start_time = current_time;
                temp_read_state = 1;
                last_sensor_update[0] = current_time;
            }
            break;
        case 1:
            if(current_time - temp_start_time > 400)
            {
                temp_read_state = 2;
            }
            break;
        case 2: // 读取温度值
    temperature = DS18B20_Get_Temp();
    sensorData.temp = (float)temperature / 10;
				 Update_Display_Parts(); //先拆分温度数据
    sprintf((char*)p, "%d.%dC", displayParts.temp_int, displayParts.temp_dec); // 新增：更新p数组
    last_sensor_update[0] = current_time;
    temp_read_state = 0;
    break;
    } 
    
		
		
    // ==================== 光伏采集 ====================
    switch(pv_read_state)
    {
        case 0:
            if(current_time - last_sensor_update[6] > PV_UPDATE_INTERVAL)
            {
                PV_Collect_Params();
                last_sensor_update[6] = current_time;
                pv_read_state = 1;
            }
            break;
        case 1:
            if(current_time - last_sensor_update[6] > 50)
            {
                pv_read_state = 0;
            }
            break;
    }
}

