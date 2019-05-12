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
pid_t pid_stir_spd;
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
   pit_set.expect = pit_set.expect -(0x400-RC_Ctl.rc.ch3)/20;	
   yaw_set.expect = yaw_set.expect + (0x400-RC_Ctl.rc.ch2)/20;	

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
            MoCa_Flag = Init;     
            ptr_heat_gun_t.sht_flg=GunFire;
        }break;
        case 2://�£�����Ħ���ָ����벦�̵��
        {
         
          ptr_heat_gun_t.sht_flg=GunFire;
            MoCa_Flag = Init; 
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
  PID_struct_init(&pid_stir_spd, POSITION_PID,15000,1000,
	                4.0f, 0.01f , 0.0f  );
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
    if(stir_motor_flag)
      set_stir_speed = -700;
    else
      set_stir_speed = 0;
      /*�ٶȻ�*/
    //set_stir_speed = -700;
       pid_calc(&pid_stir_spd,moto_stir_get.speed_rpm ,set_stir_speed);
       Stir_Motor(&hcan1,pid_stir_spd.pos_out);
			osDelayUntil(&xLastWakeTime, REMOTE_PERIOD);
	}
}
