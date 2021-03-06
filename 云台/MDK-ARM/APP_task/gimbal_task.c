/* 包含头文件----------------------------------------------------------------*/
#include "gimbal_task.h"
#include "Power_restriction.h"
#include "SystemState.h"
/* 内部宏定义----------------------------------------------------------------*/

/* 内部自定义数据类型--------------------------------------------------------*/
static  int16_t Yaw_Current_Value = 0;
static  int16_t Pitch_Current_Value = 0;
/* 任务相关信息定义----------------------------------------------------------*/

/* 内部常量定义--------------------------------------------------------------*/
#define GIMBAL_PERIOD 5
/* 外部变量声明--------------------------------------------------------------*/
Pos_Set  yaw_set;
Pos_Set  yaw_set_follow;
Pos_Set  pit_set;
int8_t gimbal_disable_flg;
extern uint8_t back_flag;
extern uint8_t round_flag;
uint8_t stir_motor_flag = 0;
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
  #if imu
  /*imu pid parameter*/
  /*暂时稳定版*/
  /*50*/
	PID_struct_init(&pid_pit, POSITION_PID, 5000, 1000,
									5.0f, 0.0f, 14.8f); 
	PID_struct_init(&pid_pit_spd, POSITION_PID, 5000, 1000,
                  2.0f, 0.0f, 0.0f );
  /*在调版*/
  /*35*/
//  PID_struct_init(&pid_pit, POSITION_PID, 5000, 1000,
//									4.0f, 0.05f, 15.5f); 
//	PID_struct_init(&pid_pit_spd, POSITION_PID, 5000, 1000,
//                  2.0f, 0.0f, 0.0f );

	
  /* yaw axis motor pid parameter */
	 PID_struct_init(&pid_yaw, POSITION_PID, 5000, 1000,
                  10.0f, 0.02f, 10.0f); 
	 PID_struct_init(&pid_yaw_spd, POSITION_PID, 5000, 1000,
                  2.0f, 0.0f, 0.0f );
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
/*操作稳定*/
//  PID_struct_init(&pid_pit_jy61, POSITION_PID, 5000, 100,
//                  20.0f, 0.03f, 22.0f); //	
//  PID_struct_init(&pid_pit_jy61_spd, POSITION_PID, 5000, 1000,
//                  2.5f, 0.0f, 0.0f  ); 
  PID_struct_init(&pid_pit_jy61, POSITION_PID, 5000, 100,
                  20.0f, 0.03f, 32.0f); //	
  PID_struct_init(&pid_pit_jy61_spd, POSITION_PID, 5000, 1000,
                  2.5f, 0.0f, 0.0f  );
#endif

/*yaw陀螺仪反馈*/
  PID_struct_init(&pid_yaw_jy61_follow, POSITION_PID, 5000, 300,
                  7.0f, 0.1f, 75.0f); //	
  PID_struct_init(&pid_yaw_jy61_follow_spd, POSITION_PID, 5000, 100,
                  2.0f, 0.0f, 0.0f );

/*yaw编码器反馈*/
  PID_struct_init(&pid_yaw_jy61, POSITION_PID, 5000, 300,
                  12.0f, 0.1f, 20.0f); //	
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
	yaw_set.mode   = 0;
  yaw_set_follow.expect = 0; 
	yaw_set_follow.mode   = 0;
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
      #if imu
        IMU_Get_Data();
        //yaw轴
      
        pid_calc(&pid_yaw, yaw_get.total_angle,yaw_set.expect);
        pid_calc(&pid_yaw_spd,(imu_data.gz), pid_yaw.pos_out);
        //pit轴
        pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
        pid_calc(&pid_pit_spd,(imu_data.gy), pid_pit.pos_out);
//        pid_calc(&pid_pit_spd,(imu_data.gy), 0);
      
        Pitch_Current_Value=(-pid_pit_spd.pos_out); 
		    Yaw_Current_Value= (-pid_yaw_spd.pos_out);
      #endif
      #if jy61
      IMU_Get_Data();
      
      /*云台限位保护*/
      /*pit正常200-1300*/
      if((pit_set.expect + pit_get.offset_angle) > 1300)
      {
        if(pit_set.expect <= pit_set.expect_last)
          goto pit_last;
        pit_set.expect = 1300 - pit_get.offset_angle;
      }
      if((pit_set.expect + pit_get.offset_angle) < 200)
      {
        if(pit_set.expect >= pit_set.expect_last)
          goto pit_last;
        pit_set.expect = 200 - pit_get.offset_angle;
      }
      //pit轴编码器
      pit_last:pid_calc(&pid_pit_jy61, pit_get.total_angle, pit_set.expect);
      pid_calc(&pid_pit_jy61_spd,(ptr_jy61_t_angular_velocity.vy), pid_pit_jy61.pos_out);
      Pitch_Current_Value=(-pid_pit_jy61_spd.pos_out); 
      /*yaw轴模式判断*/
      if(chassis_gimble_Mode_flg == 0 || round_flag)//分离
      {
        if(back_flag)
          goto back;
        /*yaw轴云台保护3600-4700*/
        if((yaw_get.offset_angle - yaw_set.expect) > 4700)
        {
          if(yaw_set.expect >= yaw_set.expect_last)
            goto yaw_last;
            yaw_set.expect = yaw_get.offset_angle - 4700;
        }
        if((yaw_get.offset_angle - yaw_set.expect) < 3600)
        {
          if(yaw_set.expect <= yaw_set.expect_last)
            goto yaw_last;
            yaw_set.expect = yaw_get.offset_angle - 3600;
        }
          yaw_last:pid_calc(&pid_yaw_jy61,(-yaw_get.total_angle),yaw_set.expect);
          pid_calc(&pid_yaw_jy61_spd,(-ptr_jy61_t_angular_velocity.vz), pid_yaw_jy61.pos_out);
          Yaw_Current_Value= (-pid_yaw_jy61_spd.pos_out);
      }
      else//跟随
      {
        /*yaw轴云台保护*/
        back:
          if(yaw_get.angle > 4840)
          {
            if(yaw_set_follow.expect >= yaw_set_follow.expect_last)
              goto yaw_follow_last;
              yaw_set_follow.expect = ptr_jy61_t_yaw.final_angle;
          }
          if(yaw_get.angle < 3430)
          {
            if(yaw_set_follow.expect <= yaw_set_follow.expect_last)
              goto yaw_follow_last;
              yaw_set_follow.expect = ptr_jy61_t_yaw.final_angle;
          }
      yaw_follow_last:pid_calc(&pid_yaw_jy61_follow,(ptr_jy61_t_yaw.final_angle),yaw_set_follow.expect);
          pid_calc(&pid_yaw_jy61_follow_spd,(ptr_jy61_t_angular_velocity.vz), pid_yaw_jy61_follow.pos_out);
          Yaw_Current_Value= (pid_yaw_jy61_follow_spd.pos_out);
      } 
      #endif
        //Pitch_Current_Value = 0;
        //Yaw_Current_Value = 0;
        /*驱动电机*/
				if(gimbal_disable_flg)//失能
				{
					Cloud_Platform_Motor_Disable(&hcan1);
				}
				else Cloud_Platform_Motor(&hcan1,Yaw_Current_Value,Pitch_Current_Value);
        
        yaw_set_follow.expect_last = yaw_set_follow.expect;
        yaw_set.expect_last = yaw_set.expect;
        pit_set.expect_last = pit_set.expect;
        
      minipc_rx_big.angle_yaw = 0;
      minipc_rx_big.angle_pit = 0;
        /*发送底盘*/
      CAN_Send_YT(&hcan1,yaw_get.angle,(int16_t)(ptr_jy61_t_angular_velocity.vz),chassis_gimble_Mode_flg,stir_motor_flag);
			osDelayUntil(&xLastWakeTime, GIMBAL_PERIOD);
			
   }
 
}
