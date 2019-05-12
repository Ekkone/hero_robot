/* 包含头文件----------------------------------------------------------------*/
#include "remote_task.h"
#include "SystemState.h"
/* 内部宏定义----------------------------------------------------------------*/
#define press_times  20
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\
//extern osSemaphoreId Dubs_BinarySemHandle;
/* 内部自定义数据类型--------------------------------------------------------*/
//moto3508_type  moto_3508_set = {.flag = 0};
/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* 内部常量定义--------------------------------------------------------------*/
pid_t pid_stir_spd;
#define REMOTE_PERIOD 2
#define MINIPC_PERIOD 2
#define REMOTE_MODE 0
#define MOUSE_MODE 1
/* 外部变量声明--------------------------------------------------------------*/

/* 调用的外部函数原型声明------------------------------------------------------
	uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);
	uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
------------------------------------------------------------------------------
*/
/* 内部变量------------------------------------------------------------------*/
int16_t XY_speed_max = 6000;
int16_t XY_speed_min = -6000; 
int16_t W_speed_max = 3000;
int16_t W_speed_min = -3000; 
uint8_t press_counter;
uint8_t shot_anjian_counter=0;
uint8_t shot_frequency = 100;
int8_t chassis_gimble_Mode_flg;
//volatile float remain_power=0.0;   //底盘功率 _待续
//float power; 				 //底盘功率 _测试

//float chassis_Current; 
//float	 chassis_Volt; 
/* 内部函数原型声明-----------------------------------------------------------*/



/***************************************************************************************
**
	*	@brief	ManualMode()
	*	@param
	*	@supplement	手动调试模式
	*	@retval	
****************************************************************************************/
void ManualMode()
{
   gimbal_mode = Manual_Mode;
   pit_set.expect = pit_set.expect -(0x400-RC_Ctl.rc.ch3)/20;	
   yaw_set.expect = yaw_set.expect + (0x400-RC_Ctl.rc.ch2)/20;	

  if(press_counter >= press_times)//左按键延迟，时间由press_time控制
	{
		press_counter=press_times+1;
    switch(RC_Ctl.rc.s1)
      {
        case 1://上
        {
            MoCa_Flag = Init;
          ptr_heat_gun_t.sht_flg=GunStop;
        }break;
        case 3://中,开启摩擦轮低速
        {
            MoCa_Flag = Init;     
            ptr_heat_gun_t.sht_flg=GunFire;
        }break;
        case 2://下，开启摩擦轮高速与拨盘电机
        {
         
          ptr_heat_gun_t.sht_flg=GunFire;
            MoCa_Flag = Init; 
           /*拨盘电机*/
        }break;
        
        default:break;
    }
  }
}

/***************************************************************************************
**
	*	@brief	SleepMode()
	*	@param
	*	@supplement	睡眠模式
	*	@retval	
****************************************************************************************/
void Sleep_Mode(uint8_t mode)
{ 
  gimbal_mode = SleepMode;
  MoCa_Flag = 0;//摩擦轮关闭
  if(mode)
  {
    /*键鼠控制*/
    switch(communication_message)
    {
      case 2:
        Open_Door();
        break;
      case 3:
        Close_Door();
        break;
      default:break;
    }
  }
  else
  {
    /*遥控控制*/
  if(press_counter >= press_times)//左按键延迟，时间由press_time控制
	{
		press_counter=press_times+1;
    switch(RC_Ctl.rc.s1)
      {
        case 1://上,关闭仓门
        {
            Close_Door();
        }break;
        case 3://中,打开仓门
        {
            Open_Door();           
        }break;
        case 2://下
        {
            
        }break;
        
        default:break;
    }
  }
  }
  
  
}

/***************************************************************************************
**
	*	@brief	AutoMode()
	*	@param
	*	@supplement	自动模式
	*	@retval	
****************************************************************************************/
void AutoMode()
{
		if(minipc_rx_small.state_flag)  
    {
      gimbal_mode = SnipeMode;
      switch(minipc_rx_small.state_flag)
      {
        case 1:
        {
          MoCa_Flag = LowSpeed;
          ptr_heat_gun_t.sht_flg=GunStop;
        }break;
        case 2:
        {
          MoCa_Flag = LowSpeed;
          ptr_heat_gun_t.sht_flg=GunFire;
        }break;
        case 3:
        {
          MoCa_Flag = MiddleSpeed;
          ptr_heat_gun_t.sht_flg=GunFire;
        }break;
        case 4:
        {
          MoCa_Flag = HighSpeed;
          ptr_heat_gun_t.sht_flg=GunFire;
        }break;
      }
    }
    else
    {
      gimbal_mode = PatrolMode;
//      gimbal_mode = SleepMode;
      MoCa_Flag = Init;
      ptr_heat_gun_t.sht_flg=GunStop;
    }
}

/* 任务主体部分 -------------------------------------------------------------*/
/***************************************************************************************
**
	*	@brief	Data_Pro_task(void const * argument)
	*	@param
	*	@supplement	遥控数据接收及处理任务
	*	@retval	
****************************************************************************************/
extern volatile uint8_t RemoteData_flag;
extern volatile uint8_t Communication_flag;
uint8_t stir_motor_flag=0;
void Remote_Data_Task(void const * argument)
{
  int set_stir_speed = 0;
    uint32_t NotifyValue;
		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
  Close_Door();
  PID_struct_init(&pid_stir_spd, POSITION_PID,15000,1000,
	                4.0f, 0.01f , 0.0f  );
	for(;;)
	{
		/*刷新断线时间*/
    RefreshTaskOutLineTime(RemoteDataTask_ON);
    if(RemoteData_flag)//遥控控制
		{
			RemoteData_flag = 0;
			Remote_Ctrl();//遥控数据接收
				switch(RC_Ctl.rc.s2)
				{
          /*上*/
					case 1: ManualMode();break; 
          /*中*/
					case 3: Sleep_Mode(REMOTE_MODE);break;
          /*下*/
					case 2: AutoMode();break;
          
					default :break;
				}					
            press_counter++;
		}
    else if(Communication_flag)//自动控制
    {
      Communication_flag = 0;
      if(!communication_message)AutoMode();//自动模式
      else          Sleep_Mode(MOUSE_MODE);//休眠模式
    }
    if(stir_motor_flag)
      set_stir_speed = -700;
    else
      set_stir_speed = 0;
      /*速度环*/
    //set_stir_speed = -700;
       pid_calc(&pid_stir_spd,moto_stir_get.speed_rpm ,set_stir_speed);
       Stir_Motor(&hcan1,pid_stir_spd.pos_out);
			osDelayUntil(&xLastWakeTime, REMOTE_PERIOD);
	}
}
