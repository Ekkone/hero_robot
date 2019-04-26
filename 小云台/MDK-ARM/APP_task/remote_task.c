/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "remote_task.h"
#include "SystemState.h"
/* �ڲ��궨��----------------------------------------------------------------*/
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
/* �ڲ��Զ�����������--------------------------------------------------------*/
//moto3508_type  moto_3508_set = {.flag = 0};
/* ���������Ϣ����----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* �ڲ���������--------------------------------------------------------------*/
pid_t pid_minipc_yaw={0};
pid_t pid_minipc_pit={0};

#define REMOTE_PERIOD 2
#define MINIPC_PERIOD 2
#define REMOTE_MODE 0
#define MOUSE_MODE 1
/* �ⲿ��������--------------------------------------------------------------*/

/* ���õ��ⲿ����ԭ������------------------------------------------------------
	uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);
	uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
------------------------------------------------------------------------------
*/
/* �ڲ�����------------------------------------------------------------------*/
int16_t XY_speed_max = 6000;
int16_t XY_speed_min = -6000; 
int16_t W_speed_max = 3000;
int16_t W_speed_min = -3000; 
uint8_t press_counter;
uint8_t shot_anjian_counter=0;
uint8_t shot_frequency = 100;
int8_t chassis_gimble_Mode_flg;
//volatile float remain_power=0.0;   //���̹��� _����
//float power; 				 //���̹��� _����

//float chassis_Current; 
//float	 chassis_Volt; 
/* �ڲ�����ԭ������-----------------------------------------------------------*/


void Minipc_Pid_Init()
{
		PID_struct_init(&pid_minipc_yaw, POSITION_PID, 6000, 5000,
									1.0f,	0.01f, 1.0f);  
		//pid_pos[i].deadband=500;
		PID_struct_init(&pid_minipc_pit, POSITION_PID, 6000, 5000,
									1.0f,	0.01f, 1.0f	);   
		pid_pit_spd.deadband=10;//2.5f,	0.03f,	1.0f	
	
//    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);   //��Դ���� _����
}

/***************************************************************************************
**
	*	@brief	ManualMode()
	*	@param
	*	@supplement	�ֶ�����ģʽ
	*	@retval	
****************************************************************************************/
void ManualMode()
{
   gimbal_mode = Manual_Mode;
   pit_set.expect = pit_set.expect +(0x400-RC_Ctl.rc.ch3)/20;	
   yaw_set.expect = yaw_set.expect +(0x400-RC_Ctl.rc.ch2)/20;	

  if(press_counter >= press_times)//�󰴼��ӳ٣�ʱ����press_time����
	{
		press_counter=press_times+1;
    switch(RC_Ctl.rc.s1)
      {
        case 1://��
        {
            MoCa_Flag = 0;
        }break;
        case 3://��,����Ħ���ֵ���
        {
            MoCa_Flag = 1;            
        }break;
        case 2://�£�����Ħ���ָ����벦�̵��
        {
          /*���̵���*/
//          shot_anjian_counter++;
//            if(shot_anjian_counter > shot_frequency)//�����������ź�
//            {
//              ptr_heat_gun_t.sht_flg=1;//����
//              press_counter=0;
//              shot_anjian_counter=0;
//            }
          ptr_heat_gun_t.sht_flg=3;
            MoCa_Flag = 2; 
           /*���̵��*/
        }break;
        
        default:break;
    }
  }
}

/***************************************************************************************
**
	*	@brief	SleepMode()
	*	@param
	*	@supplement	˯��ģʽ
	*	@retval	
****************************************************************************************/
void Sleep_Mode(uint8_t mode)
{ 
  gimbal_mode = SleepMode;
  MoCa_Flag = 0;//Ħ���ֹر�
  if(mode)
  {
    /*�������*/
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
    /*ң�ؿ���*/
  if(press_counter >= press_times)//�󰴼��ӳ٣�ʱ����press_time����
	{
		press_counter=press_times+1;
    switch(RC_Ctl.rc.s1)
      {
        case 1://��,�رղ���
        {
            Close_Door();
        }break;
        case 3://��,�򿪲���
        {
            Open_Door();           
        }break;
        case 2://��
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
	*	@supplement	�Զ�ģʽ
	*	@retval	
****************************************************************************************/
void AutoMode()
{
		if(minipc_rx_small.state_flag)  
    {
      gimbal_mode = SnipeMode;
      MoCa_Flag = 1;//����
    }
    else
    {
      gimbal_mode = PatrolMode;
      MoCa_Flag = 2;//����
      ptr_heat_gun_t.sht_flg=3;
    }
}

/* �������岿�� -------------------------------------------------------------*/
/***************************************************************************************
**
	*	@brief	Data_Pro_task(void const * argument)
	*	@param
	*	@supplement	ң�����ݽ��ռ���������
	*	@retval	
****************************************************************************************/
extern volatile uint8_t RemoteData_flag;
extern volatile uint8_t Communication_flag;
void Remote_Data_Task(void const * argument)
{
    uint32_t NotifyValue;
		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		/*ˢ�¶���ʱ��*/
    RefreshTaskOutLineTime(RemoteDataTask_ON);
    if(RemoteData_flag)//ң�ؿ���
		{
			RemoteData_flag = 0;
			Remote_Ctrl();//ң�����ݽ���
				switch(RC_Ctl.rc.s2)
				{
          /*��*/
					case 1: ManualMode();break; 
          /*��*/
					case 3: Sleep_Mode(REMOTE_MODE);break;
          /*��*/
					case 2: AutoMode();break;
          
					default :break;
				}					
            press_counter++;
		}
    else if(Communication_flag)//�Զ�����
    {
      Communication_flag = 0;
      if(!communication_message)AutoMode();//�Զ�ģʽ
      else          Sleep_Mode(MOUSE_MODE);//����ģʽ
    }
			osDelayUntil(&xLastWakeTime, REMOTE_PERIOD);
	}
}
/***************************************************************************************
**
	*	@brief	MiniPC_Data_task(void const * argument)
	*	@param
	*	@supplement	�Ӿ����ݴ�������
	*	@retval	
****************************************************************************************/
void MiniPC_Data_task(void const * argument)
{
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
	//		Get_MiniPC_Data();
				
//			pid_calc(&pid_minipc_yaw, (int16_t)minipc_rx.angle_yaw, 0);
//			pid_calc(&pid_minipc_pit, (int16_t)minipc_rx.angle_pit, 0);
//			pid_minipc_yaw.pos_out=-(pid_minipc_yaw.pos_out);
//			pid_minipc_pit.pos_out=-(pid_minipc_pit.pos_out);
//			
//			yaw_set.expect_pc += pid_minipc_yaw.pos_out;
//			pit_set.expect_pc += pid_minipc_pit.pos_out;

//			yaw_set.expect=minipc_rx.angle_yaw+yaw_get.total_angle;
//			pit_set.expect=minipc_rx.angle_pit+pit_get.total_angle;
	//		yaw_set.mode = minipc_rx.state_flag;
			
			osDelayUntil(&xLastWakeTime, MINIPC_PERIOD);
		}
	}
}

