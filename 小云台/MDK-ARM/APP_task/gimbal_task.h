#ifndef __gimbal_task_H
#define __gimbal_task_H
/* 包含头文件----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "pid.h"
#include "communication.h "
#include "Motor_USE_CAN.h"
#include "chassis_task.h"
#include "atom_imu.h"
#include "decode.h"

/* 本模块向外部提供的数据类型定义--------------------------------------------*/

typedef struct{
		//int16_t expect;
		float expect;
    float expect_last;
    float ture_value;
		uint8_t	step;
		uint8_t mode;
		int16_t expect_pc;
} Pos_Set;

typedef enum
{
  SleepMode = 0,//休眠模式
  PatrolMode,//巡逻模式
  SnipeMode,//狙击模式
  Manual_Mode//手动模式
}Gimbal_Mode;

/* 本模块向外部提供的宏定义--------------------------------------------------*/

/* 本模块向外部提供的接口常量声明--------------------------------------------*/

extern Pos_Set  yaw_set;
extern Pos_Set  yaw_set_follow;
extern Pos_Set  pit_set;
extern int8_t gimbal_disable_flg;
extern uint8_t gimbal_mode;

/* 本模块向外部提供的接口函数原型声明----------------------------------------*/
void Gimbal_Task(void const * argument);

/* 全局配置区----------------------------------------------------------------*/

#endif
