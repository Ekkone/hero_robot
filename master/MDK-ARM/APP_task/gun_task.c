/* 包含头文件----------------------------------------------------------------*/
#include "gun_task.h"
#include "math.h"
#include "SystemState.h"
/* 内部宏定义----------------------------------------------------------------*/

/* 内部自定义数据类型--------------------------------------------------------*/

/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* 内部常量定义--------------------------------------------------------------*/
#define GUN_PERIOD  10
/* 外部变量声明--------------------------------------------------------------*/
Heat_Gun_t  ptr_heat_gun_t;
extern uint8_t shot_frequency;
//Power_Heat * power_heat;
/* 外部函数原型声明-----------------------------------------------------------
float pid_calc(pid_t* pid, float get, float set);
void Friction_Wheel_Motor(uint32_t wheelone,uint32_t wheeltwo);
void Allocate_Motor(CAN_HandleTypeDef * hcan,int16_t value);
-----------------------------------------------------------------------------
-*/
/* 内部变量------------------------------------------------------------------*/

pid_t pid_dial_pos  = {0};  //拨盘电机位置环
pid_t pid_dial_spd  = {0};	//拨盘电机速度环
pid_t pid_shot_spd[2]  = {0};	//摩擦轮速度环
/* 内部函数原型声明----------------------------------------------------------*/
void Gun_Pid_Init()
{
  /**/
		PID_struct_init(&pid_dial_pos, POSITION_PID, 6000, 5000,
									1.0f,	0.0000f,	0.0f);  
		//pid_pos[i].deadband=500;
		PID_struct_init(&pid_dial_spd, POSITION_PID, 6000, 5000,
									1.5f,	0.0f,	0.0f	);  
		pid_pit_spd.deadband=10;//2.5f,	0.03f,	1.0f	
  /*摩擦轮pid初始化*/
    PID_struct_init(&pid_shot_spd[0], POSITION_PID, 6000, 5000,
									1.5f,	0.0f,	0.0f	);  
    PID_struct_init(&pid_shot_spd[1], POSITION_PID, 6000, 5000,
									1.5f,	0.0f,	0.0f	); 
		pid_pit_spd.deadband=10;//2.5f,	0.03f,	1.0f
}
/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gun_Task(void const * argument)
	*	@param
	*	@supplement	枪口热量限制任务
	*	@retval	
****************************************************************************************/
void Gun_Task(void const * argument)
{ 

	osDelay(100);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	Gun_Pid_Init();
  /*设定发弹*/


	for(;;)
	{
		RefreshTaskOutLineTime(GunTask_ON);
		
 /*判断发射模式*/
    switch(ptr_heat_gun_t.sht_flg)
    {
      case 0://停止
      {
        
      }break;
      case 1://单发模式
      {
        
      }break;
      case 2://连发模式
      {

      }break;
    }
    
    
        osDelayUntil(&xLastWakeTime,GUN_PERIOD);
	}
}


