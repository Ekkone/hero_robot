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

/***************************************************************************************
**
	*	@brief	ManualMode()
	*	@param
	*	@supplement	手动调试模式
	*	@retval	
****************************************************************************************/
void ManualMode()
{
   pit_set.expect = pit_set.expect +(0x400-RC_Ctl.rc.ch3)/20;	
   yaw_set.expect = yaw_set.expect +(0x400-RC_Ctl.rc.ch2)/20;	

  if(press_counter >= press_times)//左按键延迟，时间由press_time控制
	{
		press_counter=press_times+1;
    switch(RC_Ctl.rc.s1)
      {
        case 1://上,只传送电机开
        {
          MoCa_Flag = 1; 
        }break;
        case 3://中,只拨盘单发
        {
          /*拨盘单发*/
           shot_anjian_counter++;
            if(shot_anjian_counter > shot_frequency)//非连续触发信号
            {
              ptr_heat_gun_t.sht_flg=1;//单发
              press_counter=0;
              shot_anjian_counter=0;
            }
            MoCa_Flag = 1;            
        }break;
        case 2://下，传送电机和拨盘一起
        {
          /*拨盘单发*/
          shot_anjian_counter++;
            if(shot_anjian_counter > shot_frequency)//非连续触发信号
            {
              ptr_heat_gun_t.sht_flg=1;//单发
              press_counter=0;
              shot_anjian_counter=0;
            }
            MoCa_Flag = 1; 
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
void Sleep_Mode()
{
  
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
    if(RemoteData_flag==1)
		{
			RemoteData_flag = 0;
			//NotifyValue=0;
			Remote_Ctrl();//遥控数据接收
				switch(RC_Ctl.rc.s2)
				{
          /*上*/
					case 1: ManualMode();break; 
          /*中*/
					case 3: Sleep_Mode();break;
          /*下*/
					case 2: AutoMode();break;
          
					default :break;
				}					
            press_counter++;
		}
			osDelayUntil(&xLastWakeTime, REMOTE_PERIOD);
	}
}
/***************************************************************************************
**
	*	@brief	MiniPC_Data_task(void const * argument)
	*	@param
	*	@supplement	视觉数据处理任务
	*	@retval	
****************************************************************************************/
void MiniPC_Data_task(void const * argument)
{
	minipc_rx.state_flag = 0;
	minipc_rx.angle_pit  = 0;
	minipc_rx.angle_yaw  = 0;
  uint32_t NotifyValue;
	Minipc_Pid_Init();
  
  portTickType xLastWakeTime;
		 xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		
		 
		RefreshTaskOutLineTime(MiniPCTask_ON);
	   NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    if(NotifyValue==1)
		{
			NotifyValue=0;
			Get_MiniPC_Data();
				
//			pid_calc(&pid_minipc_yaw, (int16_t)minipc_rx.angle_yaw, 0);
//			pid_calc(&pid_minipc_pit, (int16_t)minipc_rx.angle_pit, 0);
//			pid_minipc_yaw.pos_out=-(pid_minipc_yaw.pos_out);
//			pid_minipc_pit.pos_out=-(pid_minipc_pit.pos_out);
//			
//			yaw_set.expect_pc += pid_minipc_yaw.pos_out;
//			pit_set.expect_pc += pid_minipc_pit.pos_out;

			yaw_set.expect=minipc_rx.angle_yaw+yaw_get.total_angle;
			pit_set.expect=minipc_rx.angle_pit+pit_get.total_angle;
			yaw_set.mode = minipc_rx.state_flag;
			
			osDelayUntil(&xLastWakeTime, MINIPC_PERIOD);
		}
	}
}

