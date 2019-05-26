/* 包含头文件----------------------------------------------------------------*/
#include "chassis_task.h"
#include "SystemState.h"
#include "data_pro_task.h"
/* 内部宏定义----------------------------------------------------------------*/

/* 内部自定义数据类型--------------------------------------------------------*/

/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId Chassis_QueueHandle;

/* 内部常量定义--------------------------------------------------------------
void Chassis_Motor( CAN_HandleTypeDef * hcan,
									  int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
float pid_calc(pid_t* pid, float get, float set);
void motor_move_setvmmps(float  wheel[4],float dstVmmps_X,
													float dstVmmps_Y,float dstVmmps_W);
----------------------------------------------------------------------------
*/
/* 外部变量声明--------------------------------------------------------------*/
moto3508_type  moto_3508_set = {.flag = 0}; 
extern float power;		//功率  	_测试变量
extern uint8_t back_flag;
extern uint8_t round_flag;
int8_t chassis_disable_flg;
/* 调用的外部函数原型声明----------------------------------------------------------*/

/* 内部变量------------------------------------------------------------------*/
pid_t pid_3508_pos;     		 //底盘电机位置环
pid_t pid_3508_spd[4];			 //底盘电机速度环
pid_t pid_3508_current[4];	 //底盘电机电流环节	
pid_t pid_chassis_follow = {0};//底盘跟随位置环
pid_t pid_chassis_follow_spd = {0};//底盘跟随速度环

static float Current_set[4] = {0};  //传递给功率限制的缓存

//测试变量
int16_t angle[2];

extern int16_t yaw_speed;
#define CHASSIS_PERIOD 5
#define Middle_angle 4144
/* 内部函数原型声明----------------------------------------------------------*/
void Chassis_pid_init(void)
{
	
	 PID_struct_init(&pid_3508_pos, POSITION_PID, 10000, 5000,
									1.5f,	0.0f,	20.0f);  // motos angular rate closeloop.pid:1.5,0.0,20.0
	 pid_3508_pos.deadband=150;
	
	 PID_struct_init(&pid_chassis_follow, POSITION_PID,3000,1000,
	               2.9f, 0.01f , 30.0f  );
//	  pid_chassis_follow.deadband=10;  
	 PID_struct_init(&pid_chassis_follow_spd, POSITION_PID,3000,1000,
	                1.0f, 0.0f , 0.0f  );
	
		for(int i=0; i<4; i++)
		{ 
			PID_struct_init(&pid_3508_spd[i], POSITION_PID, 15000, 5000,
										1.5f,	0.1f,	0.1f	);  //4 motos angular rate closeloop.
		}
    
}
/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Chassis_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	底盘控制任务
	*	@retval	
****************************************************************************************/
void Chassis_Contrl_Task(void const * argument)
{
  /*数据初始化*/
	static float  wheel[4]={0,0,0,0};
  static float Angle_gap;
	osDelay(200);//延时200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  chassis_disable_flg=0;
  
  /*功能初始化*/
	Chassis_pid_init();
  
	for(;;)
	{

    RefreshTaskOutLineTime(ChassisContrlTask_ON);
    /*底盘模式*/
    Angle_gap = yaw_get.angle-Middle_angle;
    if(chassis_gimble_Mode_flg == 0 || round_flag)//分离
    {
      if(back_flag)
        goto back;
       /*麦轮解算得出wheel[4]*/
			motor_move_setvmmps(wheel,moto_3508_set.dstVmmps_X,moto_3508_set.dstVmmps_Y,moto_3508_set.dstVmmps_W);
    }
    else
    {
      back:
      /*跟随位置环*/
			pid_calc(&pid_chassis_follow,yaw_get.angle,Middle_angle);
      /*跟随速度环*/ 
			pid_calc(&pid_chassis_follow_spd,-yaw_speed,pid_chassis_follow.pos_out);
      moto_3508_set.dstVmmps_W = pid_chassis_follow_spd.pos_out;
      if(abs(Angle_gap)<50)  moto_3508_set.dstVmmps_W=0;
        /*麦轮解算得出wheel[4]*/
			motor_move_setvmmps(wheel,moto_3508_set.dstVmmps_X,moto_3508_set.dstVmmps_Y,moto_3508_set.dstVmmps_W); 	
    }
    /*速度环计算*/
		for(int i=0; i<4; i++)
			{		
				pid_calc(&pid_3508_spd[i], moto_chassis_get[i].speed_rpm, wheel[i]);
//        pid_calc(&pid_3508_spd[i], moto_chassis_get[i].speed_rpm, 600);
			}
		
		/**********功率限制*********/

			Current_set[0] = pid_3508_spd[0].pos_out;
			Current_set[1] = pid_3508_spd[1].pos_out;
			Current_set[2] = pid_3508_spd[2].pos_out;
			Current_set[3] = pid_3508_spd[3].pos_out;			
			
//			printf("befeor:%f \n  ",Current_set[0]);
		  Super_Capacitance(Current_set);
//			printf("after:%f\n",Current_set[0]);
			
			pid_3508_spd[0].pos_out = Current_set[0];			
			pid_3508_spd[1].pos_out = Current_set[1];
			pid_3508_spd[2].pos_out = Current_set[2];
			pid_3508_spd[3].pos_out = Current_set[3];

							
	  /************end***********/	
			
      /*驱动电机*/
      if(chassis_disable_flg==1)//失能
			{
				  Chassis_Motor_Disable(&hcan2);
			}
			else
			{
					Chassis_Motor(&hcan2,
												pid_3508_spd[0].pos_out,
												pid_3508_spd[1].pos_out, 
												pid_3508_spd[2].pos_out, 
												pid_3508_spd[3].pos_out);

      } 
			osDelayUntil(&xLastWakeTime, CHASSIS_PERIOD);
  }
}


