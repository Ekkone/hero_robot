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
pid_t pid_minipc_yaw={0};
pid_t pid_minipc_pit={0};

#define REMOTE_PERIOD 2
#define MINIPC_PERIOD 2
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


void Minipc_Pid_Init()
{
		PID_struct_init(&pid_minipc_yaw, POSITION_PID, 6000, 5000,
									1.0f,	0.01f, 1.0f);  
		//pid_pos[i].deadband=500;
		PID_struct_init(&pid_minipc_pit, POSITION_PID, 6000, 5000,
									1.0f,	0.01f, 1.0f	);   
		pid_pit_spd.deadband=10;//2.5f,	0.03f,	1.0f	
	
//    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);   //电源引脚 _待续
}
void ChassisModeProcess()
{
   if(chassis_gimble_Mode_flg==1) //XY运动，底盘跟随云台
   {
      pit_set.expect = pit_set.expect +(0x400-RC_Ctl.rc.ch3)/20;	
      yaw_set_follow.expect = yaw_set_follow.expect +(0x400-RC_Ctl.rc.ch2)/20;	
     
     yaw_set.expect = yaw_get.total_angle;//更新分离编码器期望
   }
   else//WY运动，底盘云台分离
   {
      pit_set.expect = pit_set.expect +(0x400-RC_Ctl.rc.ch3)/20;	
      yaw_set.expect = yaw_set.expect +(0x400-RC_Ctl.rc.ch2)/20;	
     
     yaw_set_follow.expect = ptr_jy61_t_yaw.final_angle;//更新跟随陀螺仪期望
   }
   if(press_counter >= press_times)//左按键延迟，时间由press_time控制
	{
		press_counter=press_times+1;
   switch(RC_Ctl.rc.s1)
    {
      case 1://上,急停
      {
        /*底盘急停*/
        CAN_Send_YK(&hcan1,0,0,0,RC_Ctl.rc.s1,RC_Ctl.rc.s2);
      }break;
      case 2://下，底盘跟随
      {
        chassis_gimble_Mode_flg = 1;
        
        CAN_Send_YK(&hcan1,RC_Ctl.key.v,RC_Ctl.rc.ch0,RC_Ctl.rc.ch1,RC_Ctl.rc.s1,RC_Ctl.rc. s2);
      }break;
      case 3://中,底盘分离
      {
        chassis_gimble_Mode_flg = 0;  
        CAN_Send_YK(&hcan1,RC_Ctl.key.v,RC_Ctl.rc.ch0,RC_Ctl.rc.ch1,RC_Ctl.rc.s1,RC_Ctl.rc.s2);        
      }break;
      default:break;
    
    }
    MoCa_Flag = 0; 
  }
  stir_motor_flag = 0;
  //chassis_gimble_Mode_flg = 0; 
}
void ShotProcess()
{
  /*底盘模式默认分离*/
  chassis_gimble_Mode_flg = 0;
  pit_set.expect = pit_set.expect +(0x400-RC_Ctl.rc.ch3)/20;	
  yaw_set.expect = yaw_set.expect +(0x400-RC_Ctl.rc.ch2)/20;	
     
  yaw_set_follow.expect = ptr_jy61_t_yaw.final_angle;//更新跟随陀螺仪期望
  
  MoCa_Flag = 1; 
  if(press_counter >= press_times)//左按键延迟，时间由press_time控制
	{
		press_counter=press_times+1;
    switch(RC_Ctl.rc.s1)
      {
        case 1://上,只传送电机开
        {
          stir_motor_flag = 1;
        }break;
        case 3://中,只拨盘单发
        {
          /*拨盘单发*/
           shot_anjian_counter++;
            if(shot_anjian_counter > shot_frequency)//非连续触发信号
            {
              ptr_heat_gun_t.sht_flg=GunOne;//单发
              press_counter=0;
              shot_anjian_counter=0;
            }
           stir_motor_flag = 0;           
        }break;
        case 2://下，传送电机和拨盘一起
        {
          /*拨盘单发*/
          shot_anjian_counter++;
            if(shot_anjian_counter > shot_frequency)//非连续触发信号
            {
              ptr_heat_gun_t.sht_flg = GunOne;//单发
              press_counter = 0;
              shot_anjian_counter = 0;
            }
            
            /*传送电机*/
            stir_motor_flag = 1;
        }break;
        
        default:break;
    }
  }
  /**/
  CAN_Send_YK(&hcan1,RC_Ctl.key.v,RC_Ctl.rc.ch0,RC_Ctl.rc.ch1,RC_Ctl.rc.s1,RC_Ctl.rc.s2);
}

/***************************************************************************************
**
	*	@brief	MouseKeyControlProcess()
	*	@param
	*	@supplement	对键鼠的数据进行处理
	*	@retval	
****************************************************************************************/
void MouseKeyControlProcess()
{
  static uint16_t delay = 0;
  chassis_gimble_Mode_flg = 0;
  CAN_Send_YK(&hcan1,RC_Ctl.key.v,0,0,RC_Ctl.rc.s1,RC_Ctl.rc.s2);
  /*弹舱空则传送电机开*/
  if(BULLTE_EMPTY)
    stir_motor_flag = 1;
  else
    stir_motor_flag = 0;
  /*鼠标云台控制*/
  if(chassis_gimble_Mode_flg==1) //XY运动，底盘跟随云台
   {
     if(CTRL_Press&&R_Press&&minipc_rx_big.state_flag)//辅助瞄准
       yaw_set_follow.expect = yaw_set_follow.expect + minipc_rx_big.angle_yaw;
     else
      yaw_set_follow.expect = yaw_set_follow.expect-RC_Ctl.mouse.x/2;	
    
     yaw_set.expect = yaw_get.total_angle;//更新分离编码器期望
   }
   else//WY运动，底盘云台分离
   {	
     if(CTRL_Press&&R_Press&&minipc_rx_big.state_flag)//辅助瞄准
       yaw_set.expect = yaw_set.expect + minipc_rx_big.angle_yaw;
     else
      yaw_set.expect = yaw_set.expect-RC_Ctl.mouse.x/2;
     
     yaw_set_follow.expect = ptr_jy61_t_yaw.final_angle;//更新跟随陀螺仪期望
   }
   if(CTRL_Press&&R_Press&&minipc_rx_big.state_flag)//辅助瞄准
     pit_set.expect = pit_set.expect + minipc_rx_big.angle_pit;
   else
     pit_set.expect = pit_set.expect+RC_Ctl.mouse.y/2;	//鼠标（移动速度*1000/50）
   /*CTRL+鼠标右键关闭摩擦轮*/
   if(CTRL_Press&&Right_Press)
   {
     MoCa_Flag = 0;
   }
   /*鼠标右键开启摩擦轮*/
   else if(Right_Press)
   {
     MoCa_Flag = 1;
   }
  
  /*CTRL+C键底盘跟随*/ 
  if(CTRL_Press&&C_Press)
  {
    chassis_gimble_Mode_flg = 1;
  }
  /*C键底盘分离*/
  else if(C_Press) 
  {
    chassis_gimble_Mode_flg = 0;
  }
  /*摩擦轮开启时*/
  if(MoCa_Flag == 1)
  {
    /*发弹控制*/
    if(Left_Press)        //鼠标左键单发
    {
      if(delay > PRESS_DELAY && Left_Press)
      {
        ptr_heat_gun_t.sht_flg = GunOne;
        delay = 0;
      }
      delay++; 
    }
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
void Remote_Data_Task(void const * argument)
{
    uint32_t NotifyValue;
		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		/*刷新断线时间*/
    RefreshTaskOutLineTime(RemoteDataTask_ON);
		   //NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    if(RemoteData_flag)
		{
			RemoteData_flag = 0;
			//NotifyValue=0;
			Remote_Ctrl();//遥控数据接收
				switch(RC_Ctl.rc.s2)
				{
          /*上*/
					case 1: ChassisModeProcess();break; 
          /*中*/
					case 3: MouseKeyControlProcess();break;
          /*下*/
					case 2: ShotProcess();break;
          
					default :break;
				}					
            press_counter++;
		}
			osDelayUntil(&xLastWakeTime, REMOTE_PERIOD);
	}
}

