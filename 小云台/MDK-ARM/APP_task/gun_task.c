/* 包含头文件----------------------------------------------------------------*/
#include "gun_task.h"
#include "math.h"
#include "SystemState.h"
#include "user_lib.h"
#include "protocol.h"
/* 内部宏定义----------------------------------------------------------------*/

/* 内部自定义数据类型--------------------------------------------------------*/

/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* 内部常量定义--------------------------------------------------------------*/
#define GUN_PERIOD  10
#define BLOCK_TIME 5000
#define REVERSE_TIME 2000
#define prepare_flag HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_0)
#define READY    1
#define NO_READY 0
/* 外部变量声明--------------------------------------------------------------*/
Heat_Gun_t  ptr_heat_gun_t;
uint8_t MoCa_Flag = 0;
ramp_function_source_t shoot;
extern uint8_t shot_frequency;
extern float Golf_speed;
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
  /*拨弹电机*/
		PID_struct_init(&pid_dial_pos, POSITION_PID, 6000, 5000,
									0.7f,	0.0f,	1.5f);  
		//pid_dial_pos.deadband = 10;
		PID_struct_init(&pid_dial_spd, POSITION_PID, 6000, 5000,
									1.5f,	0.0f,	0.5f	);  
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
  ramp_init(&shoot,0.05,150,100);//抹茶轮斜坡
	Gun_Pid_Init();
  /*设定发弹*/
  uint8_t motor_stop_flag=0;
	static int32_t set_angle = 0;
	int32_t set_speed = 0;
  int32_t set_M_speed = 0;
	static uint8_t set_cnt = 0;
  static uint8_t block_flag;
  static uint16_t remain_heat = 0;
  static uint8_t contiue_flag = 0;
	for(;;)
	{
    /*刷新断线时间*/
		RefreshTaskOutLineTime(GunTask_ON);
    switch(MoCa_Flag)
    {
      case Stop:
      {
        /*摩擦轮停止*/
        set_M_speed = 100;
        ptr_heat_gun_t.sht_flg = 11;
      }break;
      case Init:
      {
        /*摩擦轮初始化*/
        set_M_speed = 105;
        ptr_heat_gun_t.sht_flg = 11;
      }break;
      case LowSpeed:
      {
        /*摩擦轮低速*/
        set_M_speed = 130;
      }break;
      case MiddleSpeed:
      {
        /*摩擦轮中速*/
        set_M_speed = 160;
      }break;
      case HighSpeed:
      {
        /*摩擦轮速低速*/
        set_M_speed = 190;
      }break;
    }
    ramp_calc(&shoot,set_M_speed);
    Friction_Wheel_Motor(shoot.out,shoot.out);
    /*热量限制*/
    remain_heat = Robot.heat.shoot_17_cooling_limit - Robot.heat.shoot_17_heat;
    if(remain_heat < 30)
      ptr_heat_gun_t.sht_flg = GunStop;
    /*判断发射模式*/
    switch(ptr_heat_gun_t.sht_flg)
    {
			case GunStop://停止58982
			{
        
        switch(contiue_flag)
        {
          case 0:
          {
            /*设定角度*/
            set_angle=moto_dial_get.total_angle;	
            contiue_flag = 1;
          }break;
          case 1:
          {
            goto position;
          }break;
        }
			}break;
      case GunOne://单发模式
      {
        /*设定角度*/
				moto_dial_get.cmd_time=GetSystemTimer();
				set_cnt=1;
				set_angle=58982*set_cnt;
        /*清零*/
				moto_dial_get.round_cnt=0;
				moto_dial_get.offset_angle=moto_dial_get.angle;
				moto_dial_get.total_angle=0;	
        /*进入位置环*/
        ptr_heat_gun_t.sht_flg = GunHold;
        contiue_flag = 0;
        
      }break;
      case GunFire://3连发模式
      {
				set_speed = 1000;
      }break;
      case GunHold:
      {
        /*pid位置环*/
       position: pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	
				set_speed=pid_dial_pos.pos_out;
      }break;
			default :break;
    }
    //ptr_heat_gun_t.sht_flg = GunHold;//默认位置环
     /*速度环*/
     pid_calc(&pid_dial_spd,moto_dial_get.speed_rpm ,set_speed);
     /*驱动拨弹电机*/
		 //Shot_Motor(&hcan1,pid_dial_spd.pos_out);
    /*清零标志位*/
		 minipc_rx_small.state_flag=0;
		 set_speed = 0;	   
    
        osDelayUntil(&xLastWakeTime,GUN_PERIOD);
	}
}


