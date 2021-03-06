/* 包含头文件----------------------------------------------------------------*/
#include "gimbal_task.h"
#include "Power_restriction.h"
#include "SystemState.h"
/* 内部宏定义----------------------------------------------------------------*/

/* 内部自定义数据类型--------------------------------------------------------*/
static  int16_t Yaw_Current_Value = 0;
static  int16_t Pitch_Current_Value = 0;
static  int8_t  Direction = 1;
/* 任务相关信息定义----------------------------------------------------------*/

/* 内部常量定义--------------------------------------------------------------*/
#define GIMBAL_PERIOD 5
/* 外部变量声明--------------------------------------------------------------*/
Pos_Set  yaw_set;
Pos_Set  yaw_set_follow;
Pos_Set  pit_set;
int8_t gimbal_disable_flg;
uint8_t gimbal_mode;
/* 调用的外部函数原型声明------------------------------------------------------------
void Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch);
float pid_calc(pid_t* pid, float get, float set);
------------------------------------------------------------------------------
*/
/* 内部变量------------------------------------------------------------------*/

pid_t pid_yaw       = {0};  //yaw轴位置环
pid_t pid_pit       = {0};	//pit轴位置环
pid_t pid_yaw_spd   = {0};	//yaw轴速度环
pid_t pid_pit_spd   = {0};	//pit轴速度环
pid_t pid_minipc_pit    = {0};
pid_t pid_minipc_yaw    = {0};


pid_t pid_yaw_jy61 = {0};  //外接陀螺仪 /*目前只用于位置环*/ 
pid_t pid_yaw_jy61_follow = {0}; 
pid_t pid_pit_jy61 = {0};
pid_t pid_yaw_jy61_spd = {0};
pid_t pid_yaw_jy61_follow_spd = {0};
pid_t pid_pit_jy61_spd = {0};


/* 内部函数原型声明----------------------------------------------------------*/
/**                                                           //待续
	**************************************************************
	** Descriptions: 云台pid初始化
	** Input:  NULL
	** Output: NULL
	**************************************************************
**/

void gimbal_pid_init(void)
{
		/*pitch axis motor pid parameter*/
  PID_struct_init(&pid_minipc_pit, POSITION_PID, 800, 500,
									0.03f, 0.0f, 0.0f);
  PID_struct_init(&pid_minipc_yaw, POSITION_PID, 800, 500,
                  0.10f, 0.00003f, 0.0f);
  #if imu
  /*imu pid parameter*/
  /*暂时稳定版*/
  /*50*/
	PID_struct_init(&pid_pit, POSITION_PID, 29000, 10000,
									35.0f, 0.05f, 120.0f); 
	PID_struct_init(&pid_pit_spd, POSITION_PID, 29000, 1000,
                  4.0f, 0.0f, 0.0f );
  /*在调版*/
  /* yaw axis motor pid parameter */
	 PID_struct_init(&pid_yaw, POSITION_PID, 30000, 1000,
                  10.0f, 0.02f, 20.0f); 
	 PID_struct_init(&pid_yaw_spd, POSITION_PID, 10000, 1000,
                  4.0f, 0.0f, 0.0f );
  #endif
  #if jy61
/*暂时稳定版*/
#define YAW_JY 0
#define PIT_JY 0

#if PIT_JY
/*pit陀螺仪反馈*/
  PID_struct_init(&pid_pit_jy61, POSITION_PID, 5000, 1000,
                  6.0f, 0.05f, 28.0f); //	
  PID_struct_init(&pid_pit_jy61_spd, POSITION_PID, 5000, 1000,
                  2.2f, 0.0f, 0.0f );
#else
/*pit编码器反馈*/
  PID_struct_init(&pid_pit_jy61, POSITION_PID, 5000, 1000,
                  11.0f, 0.07f, 0.0f); //	
  PID_struct_init(&pid_pit_jy61_spd, POSITION_PID, 5000, 1000,
                  2.5f, 0.0f, 0.0f ); 
#endif

/*yaw陀螺仪反馈*/
  PID_struct_init(&pid_yaw_jy61_follow, POSITION_PID, 5000, 300,
                  7.0f, 0.1f, 40.0f); //	
  PID_struct_init(&pid_yaw_jy61_follow_spd, POSITION_PID, 5000, 100,
                  2.0f, 0.0f, 0.0f );

/*yaw编码器反馈*/
  PID_struct_init(&pid_yaw_jy61, POSITION_PID, 5000, 300,
                  12.0f, 0.1f, 12.0f); //	
  PID_struct_init(&pid_yaw_jy61_spd, POSITION_PID, 5000, 100,
                  2.5f, 0.0f, 0.0f );
  #endif

	
}
/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gimbal_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	云台电机控制
	*	@retval	
****************************************************************************************/
void Gimbal_Contrl_Task(void const * argument)
{
	yaw_set.expect = 0; 
	pit_set.expect = 0;
	gimbal_disable_flg=0;
	Pitch_Current_Value=0;
	Yaw_Current_Value=0;
	gimbal_pid_init();
  /*云台保护*/
	uint16_t pit_protect_correct_1 = 0;
	uint16_t pit_protect_correct_2 = 0;
  if(pit_get.offset_angle < 5000)
  {
    pit_protect_correct_1 = 8192;
    pit_protect_correct_2 = 0;
  }
  else
  {
    pit_protect_correct_1 = 0;
    pit_protect_correct_2 = 8192;
  }
	osDelay(200);//延时200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();	
			
	for(;;)		
    {
      /*刷新断线时间*/
	   RefreshTaskOutLineTime(GimbalContrlTask_ON);
        IMU_Get_Data();
      
      /*云台模式判断*/
      switch(gimbal_mode)
      {
        case Manual_Mode://手动模式，遥控器控制
        {
          
        }break;
        case SleepMode://休眠模式，复位
        {
            yaw_set.expect = 2700 ;
            pit_set.expect = 250 - pit_get.offset_angle;
        }break;
        case PatrolMode://巡逻模式，yaw轴周期转动
        {
          pit_set.expect = 250 - pit_get.offset_angle;
          if((yaw_set.expect) > 3500 \
            || (yaw_set.expect ) < -1350)
            {
              Direction = -Direction;
            }
          yaw_set.expect += Direction * 4;
          
        }break;
        case SnipeMode://狙击模式
        {
          pid_calc(&pid_minipc_pit,0, minipc_rx_small.angle_pit);
          pid_calc(&pid_minipc_yaw,0, minipc_rx_small.angle_yaw);
          yaw_set.expect += pid_minipc_yaw.pos_out;
          pit_set.expect -= pid_minipc_pit.pos_out;
          minipc_rx_small.angle_yaw = 0;
          minipc_rx_small.angle_pit = 0;
          minipc_rx_small.state_flag = 0;
        }break;
      }
      /*云台限位保护*/
      /*pit正常65-500*/
      if((pit_set.expect + pit_get.offset_angle) > 500)
      {
        if(pit_set.expect <= pit_set.expect_last)
          goto pit_last;
        pit_set.expect = 500 - pit_get.offset_angle;
      }
      if((pit_set.expect + pit_get.offset_angle) < 65)
      {
        if(pit_set.expect >= pit_set.expect_last)
          goto pit_last;
        pit_set.expect = 65 - pit_get.offset_angle;
      }
      //pit轴编码器
      pit_last:
      pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
      pid_calc(&pid_pit_spd,(imu_data.gy), pid_pit.pos_out);
      Pitch_Current_Value=(pid_pit_spd.pos_out); 
        
      /*yaw轴云台保护-1400~3600*/
      if((yaw_set.expect) > 3600)
      {
        if(yaw_set.expect <= yaw_set.expect_last)
          goto yaw_last;
        yaw_set.expect = 3600;
      }
      if((yaw_set.expect) < -1400)
      {
        if(yaw_set.expect >= yaw_set.expect_last)
          goto yaw_last;
        yaw_set.expect = -1400;
      }
      yaw_last:
      pid_calc(&pid_yaw, yaw_get.total_angle,yaw_set.expect);
      pid_calc(&pid_yaw_spd,pit_get.speed_rpm, pid_yaw.pos_out);
      Yaw_Current_Value= (pid_yaw_spd.pos_out);
     
      #if jy61
      IMU_Get_Data();
      yaw_set.expect = minipc_rx.angle_yaw + yaw_set.expect;
      pit_set.expect = minipc_rx.angle_pit + pit_set.expect;
      minipc_rx.angle_yaw = 0;
      minipc_rx.angle_pit = 0;
      /*云台限位保护*/
      /*pit正常0-670（前），7500（后）-8192*/
      if((pit_set.expect + pit_get.offset_angle) > (630 + pit_protect_correct_2) &&\
          (pit_set.expect + pit_get.offset_angle) < (2000 + pit_protect_correct_2))
      {
        pit_set.expect = (630 + pit_protect_correct_2) - pit_get.offset_angle;
      }
      if((pit_set.expect + pit_get.offset_angle) > (6500 - pit_protect_correct_1) &&\
          (pit_set.expect + pit_get.offset_angle) < (7500 - pit_protect_correct_1))
      {
        pit_set.expect = (7500 - pit_protect_correct_1) - pit_get.offset_angle;
      }
      /*yaw轴模式判断*/
      switch(chassis_gimble_Mode_flg)
      {
        case 0://分离，yaw使用编码器
        {
          /*yaw轴云台保护*/
          if((yaw_set.expect + yaw_get.offset_angle) > 2400)
          {
            yaw_set.expect = 2380 - yaw_get.offset_angle;
          }
          if((yaw_set.expect + yaw_get.offset_angle) < 1100)
          {
            yaw_set.expect = 1115 - yaw_get.offset_angle;
          }
          pid_calc(&pid_yaw_jy61,(yaw_get.total_angle),yaw_set.expect);
          pid_calc(&pid_yaw_jy61_spd,(ptr_jy61_t_angular_velocity.vz), pid_yaw_jy61.pos_out);
          Yaw_Current_Value= (-pid_yaw_jy61_spd.pos_out);
        }break;
        case 1://跟随，yaw使用陀螺仪
        {
          /*yaw轴云台保护*/
          
          if(yaw_get.angle > 2400)
          {
            if(yaw_set_follow.expect <= yaw_set_follow.expect_last)
            goto last;
            yaw_set_follow.expect = ptr_jy61_t_yaw.final_angle-5;
          }

          if(yaw_get.angle < 1100)
          {
            if(yaw_set_follow.expect >= yaw_set_follow.expect_last)
            goto last;
            yaw_set_follow.expect = ptr_jy61_t_yaw.final_angle;
          }
      last:pid_calc(&pid_yaw_jy61_follow,(ptr_jy61_t_yaw.final_angle),yaw_set_follow.expect);
          pid_calc(&pid_yaw_jy61_follow_spd,(ptr_jy61_t_angular_velocity.vz), pid_yaw_jy61_follow.pos_out);
          Yaw_Current_Value= (-pid_yaw_jy61_follow_spd.pos_out);
          
          
        }break;
      }

        #if PIT_JY
        //pit轴陀螺仪
        pid_calc(&pid_pit_jy61, (ptr_jy61_t_pit.final_angle*22.76), pit_set.expect);
        pid_calc(&pid_pit_jy61_spd,(ptr_jy61_t_angular_velocity.vy), pid_pit_jy61.pos_out);
        #else
        //pit轴编码器
        pid_calc(&pid_pit_jy61, pit_get.total_angle, pit_set.expect);
        pid_calc(&pid_pit_jy61_spd,(-ptr_jy61_t_angular_velocity.vy), pid_pit_jy61.pos_out);
        #endif
        Pitch_Current_Value=(-pid_pit_jy61_spd.pos_out); 
		    
      #endif
//        Pitch_Current_Value = 0;
//        Yaw_Current_Value = 0;
        /*驱动电机*/
				if(gimbal_disable_flg==1)//失能
				{
					Cloud_Platform_Motor_Disable(&hcan1);
				}
				else Cloud_Platform_Motor(&hcan1,Yaw_Current_Value,Pitch_Current_Value);
        
        yaw_set.expect_last = yaw_set.expect;
        pit_set.expect_last = pit_set.expect;
        
			osDelayUntil(&xLastWakeTime, GIMBAL_PERIOD);
			
      }
 
}
