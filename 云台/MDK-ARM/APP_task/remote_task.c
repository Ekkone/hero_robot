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
void ChassisModeProcess()
{
   if(chassis_gimble_Mode_flg==1) //XY�˶������̸�����̨
   {
      pit_set.expect = pit_set.expect +(0x400-RC_Ctl.rc.ch3)/20;	
      yaw_set_follow.expect = yaw_set_follow.expect +(0x400-RC_Ctl.rc.ch2)/20;	
     
     yaw_set.expect = yaw_get.total_angle;//���·������������
   }
   else//WY�˶���������̨����
   {
      pit_set.expect = pit_set.expect +(0x400-RC_Ctl.rc.ch3)/20;	
      yaw_set.expect = yaw_set.expect +(0x400-RC_Ctl.rc.ch2)/20;	
     
     yaw_set_follow.expect = ptr_jy61_t_yaw.final_angle;//���¸�������������
   }
   if(press_counter >= press_times)//�󰴼��ӳ٣�ʱ����press_time����
	{
		press_counter=press_times+1;
   switch(RC_Ctl.rc.s1)
    {
      case 1://��,��ͣ
      {
        /*���̼�ͣ*/
        CAN_Send_YK(&hcan1,0,0,0,RC_Ctl.rc.s1,RC_Ctl.rc.s2);
      }break;
      case 2://�£����̸���
      {
        chassis_gimble_Mode_flg = 1;
        
        CAN_Send_YK(&hcan1,RC_Ctl.key.v,RC_Ctl.rc.ch0,RC_Ctl.rc.ch1,RC_Ctl.rc.s1,RC_Ctl.rc. s2);
      }break;
      case 3://��,���̷���
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
  /*����ģʽĬ�Ϸ���*/
  chassis_gimble_Mode_flg = 0;
  pit_set.expect = pit_set.expect +(0x400-RC_Ctl.rc.ch3)/20;	
  yaw_set.expect = yaw_set.expect +(0x400-RC_Ctl.rc.ch2)/20;	
     
  yaw_set_follow.expect = ptr_jy61_t_yaw.final_angle;//���¸�������������
  
  MoCa_Flag = 1; 
  if(press_counter >= press_times)//�󰴼��ӳ٣�ʱ����press_time����
	{
		press_counter=press_times+1;
    switch(RC_Ctl.rc.s1)
      {
        case 1://��,ֻ���͵����
        {
          stir_motor_flag = 1;
        }break;
        case 3://��,ֻ���̵���
        {
          /*���̵���*/
           shot_anjian_counter++;
            if(shot_anjian_counter > shot_frequency)//�����������ź�
            {
              ptr_heat_gun_t.sht_flg=GunOne;//����
              press_counter=0;
              shot_anjian_counter=0;
            }
           stir_motor_flag = 0;           
        }break;
        case 2://�£����͵���Ͳ���һ��
        {
          /*���̵���*/
          shot_anjian_counter++;
            if(shot_anjian_counter > shot_frequency)//�����������ź�
            {
              ptr_heat_gun_t.sht_flg = GunOne;//����
              press_counter = 0;
              shot_anjian_counter = 0;
            }
            
            /*���͵��*/
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
	*	@supplement	�Լ�������ݽ��д���
	*	@retval	
****************************************************************************************/
void MouseKeyControlProcess()
{
  static uint16_t delay = 0;
  chassis_gimble_Mode_flg = 0;
  CAN_Send_YK(&hcan1,RC_Ctl.key.v,0,0,RC_Ctl.rc.s1,RC_Ctl.rc.s2);
  /*���տ����͵����*/
  if(BULLTE_EMPTY)
    stir_motor_flag = 1;
  else
    stir_motor_flag = 0;
  /*�����̨����*/
  if(chassis_gimble_Mode_flg==1) //XY�˶������̸�����̨
   {
     if(CTRL_Press&&R_Press&&minipc_rx_big.state_flag)//������׼
       yaw_set_follow.expect = yaw_set_follow.expect + minipc_rx_big.angle_yaw;
     else
      yaw_set_follow.expect = yaw_set_follow.expect-RC_Ctl.mouse.x/2;	
    
     yaw_set.expect = yaw_get.total_angle;//���·������������
   }
   else//WY�˶���������̨����
   {	
     if(CTRL_Press&&R_Press&&minipc_rx_big.state_flag)//������׼
       yaw_set.expect = yaw_set.expect + minipc_rx_big.angle_yaw;
     else
      yaw_set.expect = yaw_set.expect-RC_Ctl.mouse.x/2;
     
     yaw_set_follow.expect = ptr_jy61_t_yaw.final_angle;//���¸�������������
   }
   if(CTRL_Press&&R_Press&&minipc_rx_big.state_flag)//������׼
     pit_set.expect = pit_set.expect + minipc_rx_big.angle_pit;
   else
     pit_set.expect = pit_set.expect+RC_Ctl.mouse.y/2;	//��꣨�ƶ��ٶ�*1000/50��
   /*CTRL+����Ҽ��ر�Ħ����*/
   if(CTRL_Press&&Right_Press)
   {
     MoCa_Flag = 0;
   }
   /*����Ҽ�����Ħ����*/
   else if(Right_Press)
   {
     MoCa_Flag = 1;
   }
  
  /*CTRL+C�����̸���*/ 
  if(CTRL_Press&&C_Press)
  {
    chassis_gimble_Mode_flg = 1;
  }
  /*C�����̷���*/
  else if(C_Press) 
  {
    chassis_gimble_Mode_flg = 0;
  }
  /*Ħ���ֿ���ʱ*/
  if(MoCa_Flag == 1)
  {
    /*��������*/
    if(Left_Press)        //����������
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

/* �������岿�� -------------------------------------------------------------*/
/***************************************************************************************
**
	*	@brief	Data_Pro_task(void const * argument)
	*	@param
	*	@supplement	ң�����ݽ��ռ���������
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
		/*ˢ�¶���ʱ��*/
    RefreshTaskOutLineTime(RemoteDataTask_ON);
		   //NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    if(RemoteData_flag)
		{
			RemoteData_flag = 0;
			//NotifyValue=0;
			Remote_Ctrl();//ң�����ݽ���
				switch(RC_Ctl.rc.s2)
				{
          /*��*/
					case 1: ChassisModeProcess();break; 
          /*��*/
					case 3: MouseKeyControlProcess();break;
          /*��*/
					case 2: ShotProcess();break;
          
					default :break;
				}					
            press_counter++;
		}
			osDelayUntil(&xLastWakeTime, REMOTE_PERIOD);
	}
}

