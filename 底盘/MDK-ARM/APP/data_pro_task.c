/* 包含头文件----------------------------------------------------------------*/
#include "data_pro_task.h"
#include "SystemState.h"
#include "Motor_USE_CAN.h"
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

/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* 内部常量定义--------------------------------------------------------------*/
pid_t pid_minipc_yaw={0};
pid_t pid_minipc_pit={0};
pid_t pid_stir_spd;

#define REMOTE_PERIOD 5
#define MINIPC_PERIOD 2
#define REFEREE_PERIOD 5
/* 外部变量声明--------------------------------------------------------------*/

/* 调用的外部函数原型声明------------------------------------------------------
	uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);
	uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
------------------------------------------------------------------------------
*/
/* 内部变量------------------------------------------------------------------*/
int16_t XY_speed_max = 12000;
int16_t XY_speed_min = -12000; 
int16_t W_speed_max = 2000;
int16_t W_speed_min = -2000; 
uint8_t press_counter;
uint8_t shot_anjian_counter=0;
uint8_t shot_frequency = 100;
uint8_t chassis_gimble_Mode_flg = 0;
uint8_t communication_message = 1;
uint8_t stir_flag = 0;
extern int16_t yaw_speed;
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
		//pid_pit_spd.deadband=10;//2.5f,	0.03f,	1.0f	
	
//    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);   //电源引脚 _待续
}
/***************************************************************************************
**
	*	@brief	RemoteControlProcess()
	*	@param
	*	@supplement	与遥控器进行对接，对遥控器的数据进行处理，实现对底盘、云台、发射机构的控制
	*	@retval	
****************************************************************************************/
void ChassisModeProcess()
{
  /*遥控杆数据处理*/
	         if(chassis_gimble_Mode_flg==1) //XY运动，底盘跟随云台
					 {
							moto_3508_set.dstVmmps_X=-((RC_Ctl.rc.ch0-0x400)*5);
							moto_3508_set.dstVmmps_Y=-((RC_Ctl.rc.ch1-0x400)*5);
             
					 }
					 else//WY运动，底盘云台分离
					 {
							moto_3508_set.dstVmmps_W=((RC_Ctl.rc.ch0-0x400)*5);
							moto_3508_set.dstVmmps_Y=-((RC_Ctl.rc.ch1-0x400)*5);
					 }
           if(RC_Ctl.rc.s1 == 1)
             hard_brak();
}
void ShotProcess()
{	
  moto_3508_set.dstVmmps_W=((RC_Ctl.rc.ch0-0x400)*5);
	moto_3508_set.dstVmmps_Y=-((RC_Ctl.rc.ch1-0x400)*5);
}
/***************************************************************************************
**
	*	@brief	MouseKeyControlProcess()
	*	@param
	*	@supplement	对键鼠的数据进行处理
	*	@retval	
****************************************************************************************/
uint8_t back_flag = 0;
uint8_t round_flag = 0;
void MouseKeyControlProcess()
{
	
  if(F_Press)//暂时切换为跟随
   {
     back_flag = 1;
   } 
   else back_flag = 0;
  if(E_Press || Q_Press)//暂时切换为分离
  {
    round_flag = 1;
  }
  else round_flag = 0;
	if(SHIFT_Press)//最高速度
      {
        XY_speed_max = 5000;//(NORMAL_SPEED_MAX)*3.5;
        XY_speed_min = -5000;//(NORMAL_SPEED_MIN)*3.5;
        W_speed_max = 3000;
        W_speed_min = -3000; 
      }
  else if(G_Press)//慢速
    {
      XY_speed_max = 500;//(NORMAL_SPEED_MAX)*3.5;
      XY_speed_min = -500;//(NORMAL_SPEED_MIN)*3.5;
      W_speed_max = 500;
      W_speed_min = -500; 
    }
  else//正常速度
  {
     XY_speed_max = 4000;//(NORMAL_SPEED_MAX)*3.5;
     XY_speed_min = -4000;//(NORMAL_SPEED_MIN)*3.5;
     W_speed_max = 2000;
     W_speed_min = -2000;
  }
	/*Y向速度*/
  if(W_Press)                       moto_3508_set.dstVmmps_Y -= ACC_SPEED;//按下W键
  else if(S_Press)                  moto_3508_set.dstVmmps_Y += ACC_SPEED;//按下S键
  else{  
        if(moto_3508_set.dstVmmps_Y>-DEC_SPEED&&moto_3508_set.dstVmmps_Y<DEC_SPEED) 	 moto_3508_set.dstVmmps_Y = 0;
        if(moto_3508_set.dstVmmps_Y>0) 	                   moto_3508_set.dstVmmps_Y -= DEC_SPEED;
        if(moto_3508_set.dstVmmps_Y<0) 		                 moto_3508_set.dstVmmps_Y += DEC_SPEED;
  }
  /*W向速度*/
  if(Q_Press)                       moto_3508_set.dstVmmps_W -= ACC_SPEED;//按下A键
  else if(E_Press)                  moto_3508_set.dstVmmps_W += ACC_SPEED;//按下D键
  else{  
        if(moto_3508_set.dstVmmps_W>-DEC_SPEED&&moto_3508_set.dstVmmps_W<DEC_SPEED) 	 moto_3508_set.dstVmmps_W = 0;
        if(moto_3508_set.dstVmmps_W>0) 	                   moto_3508_set.dstVmmps_W -= DEC_SPEED;
        if(moto_3508_set.dstVmmps_W<0) 		                 moto_3508_set.dstVmmps_W += DEC_SPEED;
    }
  /*分离时*/
	if(chassis_gimble_Mode_flg == 0 || round_flag)
  {
    if(F_Press)
    {
      pid_calc(&pid_chassis_follow,-yaw_get.total_angle,0);
        /*跟随速度环*/ 
			pid_calc(&pid_chassis_follow_spd,-yaw_speed,pid_chassis_follow.pos_out);
      moto_3508_set.dstVmmps_W = -pid_chassis_follow_spd.pos_out;
    }
    /*X向速度*/
    if(A_Press)                        moto_3508_set.dstVmmps_X += ACC_SPEED; //按下Q键
    else if(D_Press)    		           moto_3508_set.dstVmmps_X -= ACC_SPEED;//按下E键
    else{
            if(moto_3508_set.dstVmmps_X>-DEC_SPEED&&moto_3508_set.dstVmmps_X<DEC_SPEED) 		moto_3508_set.dstVmmps_X = 0;		
            if(moto_3508_set.dstVmmps_X>0) 	                   moto_3508_set.dstVmmps_X -= DEC_SPEED;
            if(moto_3508_set.dstVmmps_X<0) 		                 moto_3508_set.dstVmmps_X += DEC_SPEED;
    }
  }
  /*跟随时*/
  else if(chassis_gimble_Mode_flg == 1 || back_flag)
  {
    /*X向速度*/
    if(D_Press)                        moto_3508_set.dstVmmps_X -= ACC_SPEED; //按下D键
    else if(A_Press)    		           moto_3508_set.dstVmmps_X += ACC_SPEED;//按下A键
    else{
            if(moto_3508_set.dstVmmps_X>-DEC_SPEED&&moto_3508_set.dstVmmps_X<DEC_SPEED) 		moto_3508_set.dstVmmps_X = 0;		
            if(moto_3508_set.dstVmmps_X>0) 	                   moto_3508_set.dstVmmps_X -= DEC_SPEED;
            if(moto_3508_set.dstVmmps_X<0) 		                 moto_3508_set.dstVmmps_X += DEC_SPEED;
      }
    
  }
  /*小云台控制*/
  if(B_Press)
  {
    communication_message = 1;//睡眠模式
  }
  else if(V_Press)
  {
    communication_message = 0;//自动模式
  }
  if(communication_message)
  {
    if(Z_Press && CTRL_Press) communication_message = 3;//关闭仓门
    else if(Z_Press)          communication_message = 2;//打开仓门 
  }
}


/***************************************************************************************
**
	*	@brief	hard_brak()
	*	@param
	*	@supplement	紧急停止函数
	*	@retval	
****************************************************************************************/
void hard_brak()
{
		moto_3508_set.dstVmmps_X=0;
		moto_3508_set.dstVmmps_Y=0;
		moto_3508_set.dstVmmps_W=0;
}


/* 任务主体部分 -------------------------------------------------------------*/
/***************************************************************************************
**
	*	@brief	Data_Pro_task(void const * argument)
	*	@param
	*	@supplement	遥控数据接收及处理任务
	*	@retval	
****************************************************************************************/
void Remote_Data_Task(void const * argument)
{
		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
	
	PID_struct_init(&pid_stir_spd, POSITION_PID,15000,1000,
	                4.0f, 0.01f , 0.0f  );
	for(;;)
	{
    /*发送给操作界面*/
    sendata(Show_CapVolt(),0,0,stir_motor_flag,1,1,1,1,1);
			RefreshTaskOutLineTime(RemoteDataTask_ON);
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
				
			VAL_LIMIT(moto_3508_set.dstVmmps_X, XY_speed_min, XY_speed_max);
			VAL_LIMIT(moto_3508_set.dstVmmps_Y, XY_speed_min, XY_speed_max);	
			VAL_LIMIT(moto_3508_set.dstVmmps_W, W_speed_min, W_speed_max);
        
      CAN_Send_Referee_B(&hcan1);
      CAN_Send_Referee_S(&hcan1);       
      
      press_counter++;
        osDelayUntil(&xLastWakeTime, REMOTE_PERIOD);
	}

}

