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
uint8_t gun_check_flag = 0;
uint8_t gun_ready_flag = 0;
extern uint8_t heat_limit;
//volatile float remain_power=0.0;   //���̹��� _����
//float power; 				 //���̹��� _����

//float chassis_Current; 
//float	 chassis_Volt; 
/* �ڲ�����ԭ������-----------------------------------------------------------*/



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
   pit_set.expect = pit_set.expect - (0x400-RC_Ctl.rc.ch3)/40;	
   yaw_set.expect = yaw_set.expect + (0x400-RC_Ctl.rc.ch2)/40;	

  if(press_counter >= press_times)//�󰴼��ӳ٣�ʱ����press_time����
	{
		press_counter=press_times+1;
    switch(RC_Ctl.rc.s1)
      {
        case 1://��
        {
            MoCa_Flag = Init;
          ptr_heat_gun_t.sht_flg=GunStop;
        }break;
        case 3://��,����Ħ���ֵ���
        {
            MoCa_Flag = HighSpeed;     
            ptr_heat_gun_t.sht_flg=GunFire;
        }break;
        case 2://�£�����Ħ���ָ����벦�̵��
        {
         
          ptr_heat_gun_t.sht_flg=GunFire;
            MoCa_Flag = HighSpeed; 
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
  MoCa_Flag = Init;
  gun_ready_flag = 0;//δ����
  gimbal_mode = SleepMode;
  heat_limit = 1;
  ptr_heat_gun_t.sht_flg = GunStop;
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
extern volatile uint16_t remain_heat;
uint16_t remain_heat_last;
uint32_t delay_time = 0;
void AutoMode()
{
  MoCa_Flag = HighSpeed;
  heat_limit = 1;
  if(!gun_ready_flag)
  {
    gimbal_mode = SleepMode;
    ptr_heat_gun_t.sht_flg=GunFire;//����
    if(remain_heat < remain_heat_last)
    {
      gun_ready_flag = 1;//��ʼ�������
    }
  }
  else 
    if(minipc_rx_small.state_flag)  
    {
      gimbal_mode = SnipeMode;
      switch(minipc_rx_small.state_flag)
      {
        case 1:
        {
          ptr_heat_gun_t.sht_flg=GunStop;
        }break;
        case 2:
        {
          ptr_heat_gun_t.sht_flg=GunFire;
        }break;
        case 3:
        {
          ptr_heat_gun_t.sht_flg=GunFire;
        }break;
        case 4:
        {
          ptr_heat_gun_t.sht_flg=GunFire;
        }break;
      }
      delay_time = 0;
    }
    else 
    {
      delay_time++;
      ptr_heat_gun_t.sht_flg=GunStop;
      if(delay_time > 50)//
      {
        delay_time = 51;
        gimbal_mode = PatrolMode;
      }
      else
        gimbal_mode = SnipeMode;
    }
    remain_heat_last = remain_heat;
}
void CleanMode(void)
{
  heat_limit = 0;
  gimbal_mode = SleepMode;
  ptr_heat_gun_t.sht_flg = GunFire;
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
uint8_t stir_motor_flag=0;
void Remote_Data_Task(void const * argument)
{
  int set_stir_speed = 0;
    uint32_t NotifyValue;
		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
  Close_Door();
  
	for(;;)
	{
		/*ˢ�¶���ʱ��*/
    RefreshTaskOutLineTime(RemoteDataTask_ON);
//    RemoteData_flag = 1;
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
      if(communication_message == 0) AutoMode();//�Զ�ģʽ
      else if(communication_message == 4)//�嵯ģʽ
        CleanMode();
      else  Sleep_Mode(MOUSE_MODE);//����ģʽ
    }
    
			osDelayUntil(&xLastWakeTime, REMOTE_PERIOD);
	}
}
