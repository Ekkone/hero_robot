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
#define BLOCK_TIME 2000
#define STIR_BLOCK_TIME 1000
#define prepare_flag HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_0)
#define READY    1
#define NO_READY 0
/* 外部变量声明--------------------------------------------------------------*/
Heat_Gun_t  ptr_heat_gun_t;
volatile uint8_t MoCa_Flag = Init;
uint16_t remain_heat = 0;
ramp_function_source_t shoot;
extern uint8_t shot_frequency;
extern float Golf_speed;
extern uint8_t stir_motor_flag;
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
pid_t pid_stir_spd;
/* 内部函数原型声明----------------------------------------------------------*/
void Gun_Pid_Init()
{
  /*拨弹电机*/
//		PID_struct_init(&pid_dial_pos, POSITION_PID, 10000, 5000,
//									5.0f,	0.0f,	3.0f);  
		//pid_dial_pos.deadband = 10;
		PID_struct_init(&pid_dial_spd, POSITION_PID, 10000, 6000,
									5.0f,	0.5f,	0.0f	);  
  PID_struct_init(&pid_stir_spd, POSITION_PID,15000,1000,
	                4.0f, 0.01f , 0.0f  );
}
/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gun_Task(void const * argument)
	*	@param
	*	@supplement	枪口热量限制任务
	*	@retval	
****************************************************************************************/
uint32_t locktime = 0;
uint32_t stir_locktime = 0;
void Gun_Task(void const * argument)
{ 

	osDelay(100);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  ramp_init(&shoot,0.05,190,100);//抹茶轮斜坡
	Gun_Pid_Init();
  /*设定发弹*/
  uint8_t motor_stop_flag=0;
  uint8_t set_cnt = 0;
	static int32_t set_angle = 0;
	int32_t set_speed = 0;
  int32_t set_M_speed = 103;
  static uint8_t block_flag;
  static uint16_t remain_heat = 0;
  static uint8_t contiue_flag = 0;

  int16_t set_stir_speed = 0;
	for(;;)
	{
    /*刷新断线时间*/
		RefreshTaskOutLineTime(GunTask_ON);
    switch(MoCa_Flag)
    {
      case Init:
      {
        /*摩擦轮中速*/
        shoot.max_value=103;
        ramp_calc(&shoot,103);
        Friction_Wheel_Motor(shoot.out,shoot.out);
      }
      case HighSpeed:
      {
        /*摩擦轮中速*/
        shoot.max_value=117;
        ramp_calc(&shoot,103);
        Friction_Wheel_Motor(shoot.out,shoot.out);
      }
    }
    
    /*热量限制*/
    if(remain_heat < 30)
//      ptr_heat_gun_t.sht_flg = GunStop;
    /*判断发射模式*/
    switch(ptr_heat_gun_t.sht_flg)
    {
			case GunStop://停止58982
			{
        set_speed = 0;
        set_cnt = 0;
        moto_dial_get.run_time=GetSystemTimer();//未堵转时间
        locktime = 0;
			}break;
      case GunFire://连发模式
      {
        if(Check_locked() || locktime)//堵转检测
        {
          locktime--;
          set_speed = -800;
        }
        else//未堵转
          set_speed = 2000;
      }break;
			default :break; 
    }
    /*传送电机*/
    if(stir_motor_flag)
    {
      if(Check_stir_locked() || stir_locktime)//堵转
      {
        stir_locktime--;
        set_stir_speed = 500;
      }
      else set_stir_speed = -500;
    }
    else
      set_stir_speed = 0;
    
       pid_calc(&pid_stir_spd,moto_stir_get.speed_rpm ,set_stir_speed);
     /*拨弹电机*/
     pid_calc(&pid_dial_spd,moto_dial_get.speed_rpm ,-set_speed);
    
     /*驱动电机*/
    Shot_Motor(&hcan1,pid_dial_spd.pos_out,pid_stir_spd.pos_out);
    /*清零标志位*/
		 minipc_rx_small.state_flag=0;
		 set_speed = 0;	   
    
        osDelayUntil(&xLastWakeTime,GUN_PERIOD);
	}
}
uint8_t Check_locked(void)
{
  moto_dial_get.cmd_time = GetSystemTimer();//刷新当前时间
  if(!locktime)
  {
		     /*判断拨盘是否转到位置*/			
					if(my_abs(moto_dial_get.round_cnt) >=2)
						{
							
										moto_dial_get.round_cnt=0;
										moto_dial_get.offset_angle=moto_dial_get.angle;
										moto_dial_get.total_angle=0;
										moto_dial_get.run_time=GetSystemTimer();//未堵转时间
                    return 0;//未堵转
						}
						else if( my_abs(moto_dial_get.run_time-moto_dial_get.cmd_time)>BLOCK_TIME )//堵转判定
						{
                    locktime = 100;
									  return 1;	//堵转
            }
            else return 0;//堵转时间继续加力
   }
    else return 0;
}
uint8_t Check_stir_locked(void)
{
  moto_stir_get.cmd_time = GetSystemTimer();//刷新当前时间
  if(!stir_locktime)
  {
		     /*判断拨盘是否转到位置*/			
					if(my_abs(moto_stir_get.total_angle) >= 70)
						{
							
										moto_stir_get.round_cnt=0;
										moto_stir_get.offset_angle=moto_stir_get.angle;
										moto_stir_get.total_angle=0;
										moto_stir_get.run_time=GetSystemTimer();//未堵转时间
                    return 0;//未堵转
						}
						else if( my_abs(moto_stir_get.run_time-moto_stir_get.cmd_time)>STIR_BLOCK_TIME )//堵转判定
						{
                    stir_locktime = 60;
									  return 1;	//堵转
            }
            else return 0;//堵转时间继续加力
   }
    else return 0;
}

