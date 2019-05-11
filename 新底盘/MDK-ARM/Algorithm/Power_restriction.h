#ifndef __POWER_RESTRICTION_H
#define __POWER_RESTRICTION_H
/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "String.h"
#include "gpio.h"
#include "usart.h"
#include "protocol.h"

/* 本模块向外部提供的宏定义 --------------------------------------------------*/
//电容板开关
#define Switch1_On HAL_GPIO_WritePin(Switch_GPIO_Port,Switch1_Pin,GPIO_PIN_SET);	//放电开关
#define Switch2_On HAL_GPIO_WritePin(Switch_GPIO_Port,Switch2_Pin,GPIO_PIN_SET);//充电开关
#define Switch3_On HAL_GPIO_WritePin(Switch_GPIO_Port,Switch3_Pin,GPIO_PIN_SET);	//裁判系统直接供电开关

#define	Switch1_Off HAL_GPIO_WritePin(Switch_GPIO_Port,Switch1_Pin,GPIO_PIN_RESET);
#define	Switch2_Off HAL_GPIO_WritePin(Switch_GPIO_Port,Switch2_Pin,GPIO_PIN_RESET);
#define	Switch3_Off HAL_GPIO_WritePin(Switch_GPIO_Port,Switch3_Pin,GPIO_PIN_RESET);

/* 本模块向外部提供的数据类型定义 --------------------------------------------*/
typedef struct{
	
 //电池电流
 float  CurrentCalculat;			 			//电流计算结果
 float  Current_Offset;						  //电流偏置量(和裁判系统比较)
 float  Current_Referee;						//来自裁判系统的电流值
 int16_t Current_Offset_num; 
 //电容电压
 float Capacitance_Volt;						//电容电压
 //电机电流
 float	Chassis_Current;						//底盘电机电流

}Current_GET;   //电流采集

typedef struct{
		
 float  Volt_Referee;								//来自裁判系统的电压值
 float  Power_Referee;							//来自裁判系统的功率
 float  Power_Referee_Last;         
 float  Power_Calculat;							//自行计算的功率值
 float  Power_Chassis_Calculat;			//自行计算的电机端的功率值
 float  PowerRemain_Referee;				//来自裁判系统的剩余功率
 float  PowerRemain_Referee_last;		
 float  PowerRemain_Calculat;				//自行计算的裁判系统的剩余功率
 float  PowerRemain_Calculat_Last;
 float  PowerRemain_Calculat_Next;
 float  PowerLimit;									//最终限制值
 
}Limit;				 //电流限制




typedef struct{

	uint32_t time_now;
	uint32_t time_last;
	int32_t  time;
	int32_t  total_time_ms;
	double  total_time_s;
	
}MyTimeTick;
/* 本模块向外部提供的接口常量声明 --------------------------------------------*/
extern  uint32_t  uhADC1ConvertedValue[10];  //ADC1缓存数据
extern  uint32_t  uhADC2ConvertedValue[10];  //ADC2缓存数据
extern  uint32_t  uhADC3ConvertedValue[10];  //ADC3缓存数据

extern  Current_GET  current_get;
extern  Limit  limit;
extern  MyTimeTick  time_for_limit;
/* 本模块向外部提供的接口函数原型声明 ----------------------------------------*/
void power_limit(float  Current_get[4]);
void Super_Capacitance(float  Current_get[4]);
float	Show_CapVolt(void);
float Limit_filter(float oldData,float newData,float val);
float LPF_1st(float oldData, float newData, float lpf_factor);


#endif

