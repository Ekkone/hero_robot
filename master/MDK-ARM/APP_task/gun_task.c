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
volatile uint8_t finish_flag = 0;
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
/* 内部函数原型声明----------------------------------------------------------*/
void Gun_Pid_Init()
{
		PID_struct_init(&pid_dial_pos, POSITION_PID, 6000, 5000,
									0.2f,	0.0000f,	2.0f);  
		//pid_pos[i].deadband=500;
		PID_struct_init(&pid_dial_spd, POSITION_PID, 6000, 5000,
									1.5f,	0.1f,	0.0f	);  
		pid_pit_spd.deadband=10;//2.5f,	0.03f,	1.0f	
	
//    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);   //电源引脚 _待续
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
	
	uint8_t motor_stop_flag=0;
	int32_t set_angle = 0;
  uint8_t key_flag = 0;
  uint16_t check_count = 0;
  uint8_t check_flag = 0;
  uint8_t back_flag = 0;
  
  /*设定发弹*/
  int32_t one_angle = -215;    //单个角度 -122  60
  uint8_t frequency = 40;     //单个频率，越小越快
  float   all_speed = -8000;  //连续速度-7000 

	for(;;)
	{
		RefreshTaskOutLineTime(GunTask_ON);
		
 /*判断发射模式*/
    switch(ptr_heat_gun_t.sht_flg)
    {
      case 0://按键单发模式
      {
//        /*停止状态，位置环*/
//        if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == 1 && key_flag ==0)
//        {
//          moto_dial_get.round_cnt=0;
//          moto_dial_get.offset_angle= moto_dial_get.angle;
//          moto_dial_get.total_angle=0;
//          pid_calc(&pid_dial_pos, moto_dial_get.total_angle,0);	//设定角度值为0
//        }
//        /*按下按键瞬间，进入下一状态*/
//        else if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == 0 && key_flag ==0)
//        {
//          key_flag = 1;
//        }
        /*停止状态，位置环*/
        if(key_flag ==0)
        {
          moto_dial_get.round_cnt=0;
          moto_dial_get.offset_angle= moto_dial_get.angle;
          moto_dial_get.total_angle=0;
          pid_calc(&pid_dial_pos, moto_dial_get.total_angle,0);	//设定角度值为0
        }
        /*按下按键瞬间，进入下一状态*/
        else if(key_flag ==0)
        {
          key_flag = 1;
        }
        /*按键之后*/
//        if(key_flag == 1 || key_flag == 2)
//        {
//          /*匀速过程，未发弹完成*/
//          if(finish_flag == 0)
//          {
//            key_flag = 2;
//            pid_dial_pos.pos_out = all_speed;
//          }
//          /*发弹完成，进入停止状态*/
//          else
//          {
//            key_flag = 0;
//            finish_flag = 0;
//          }
//        }
      }break;
      case 1://固定角度模式
      {
        if(motor_stop_flag == 1)//更新角度
        {
          set_angle = one_angle * 819.1;
          shot_frequency = frequency;
          moto_dial_get.round_cnt=0;
          moto_dial_get.offset_angle= moto_dial_get.angle;
          moto_dial_get.total_angle=0;
          ptr_heat_gun_t.sht_flg=0;
        }
        /*单个发弹停止判断*/
        if(one_angle < 0)//反转
        {
          if(moto_dial_get.round_cnt <= (one_angle * 0.1 + 1) || set_angle==0)
          {
            motor_stop_flag = 1;
          }
          else  
          {
            motor_stop_flag = 0;
          }
        }
        else             //正转
        {
          if(moto_dial_get.round_cnt >= (one_angle * 0.1 - 3) || set_angle==0)
          {
            motor_stop_flag = 1;
          }
          else  
          {
            motor_stop_flag = 0;
          }
        }
        /*pid位置环*/
        pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	
      }break;
      case 2://连发模式
      {
          check_count++;
        if(check_count < 1000)//初始时一直保持匀速
        {
          pid_dial_pos.pos_out = all_speed;
        }
        else//开始检测堵转
        {
          if(check_count == 2000) //匀速过程循环1000-2000
            check_count = 1000;
          else if(back_flag == 0 && moto_dial_get.speed_rpm < 10)//堵转条件，转子速度<100
          {
            check_count = 4000;  
            check_flag = 1;
            back_flag = 1;
          }
          else if(check_count == 4200)
          {
            back_flag = 0;
            check_count = 1000;
          }
          else if(check_count > 3100) //堵转置为3000后增至    返回匀速过程
          {
            check_count = 1000;
            check_flag = 0;
          }
          switch(check_flag)
          {
            case 0:
              pid_dial_pos.pos_out = all_speed;//匀速过程
              break;
            case 1:
              pid_dial_pos.pos_out = -all_speed;//反转过程
              break;
          }
        }

      }break;
    }
     /*速度环*/
     pid_calc(&pid_dial_spd,moto_dial_get.speed_rpm ,pid_dial_pos.pos_out);
     /*驱动拨弹电机*/
		 Allocate_Motor(&hcan1,pid_dial_spd.pos_out);
					
        osDelayUntil(&xLastWakeTime,GUN_PERIOD);
	}
}


