/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "gun_task.h"
#include "math.h"
#include "SystemState.h"
#include "user_lib.h"
/* �ڲ��궨��----------------------------------------------------------------*/

/* �ڲ��Զ�����������--------------------------------------------------------*/

/* ���������Ϣ����----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* �ڲ���������--------------------------------------------------------------*/
#define ABS(x)		((x>0)? (x): (-x)) 
#define GUN_PERIOD  10
#define BLOCK_TIME 5000
#define REVERSE_TIME 2000
#define prepare_flag HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_0)
#define READY    1
#define NO_READY 0
/* �ⲿ��������--------------------------------------------------------------*/
Heat_Gun_t  ptr_heat_gun_t;
uint8_t MoCa_Flag = 0;
extern uint8_t shot_frequency;
extern float Golf_speed;
//Power_Heat * power_heat;
/* �ⲿ����ԭ������-----------------------------------------------------------
float pid_calc(pid_t* pid, float get, float set);
void Friction_Wheel_Motor(uint32_t wheelone,uint32_t wheeltwo);
void Allocate_Motor(CAN_HandleTypeDef * hcan,int16_t value);
-----------------------------------------------------------------------------
-*/
/* �ڲ�����------------------------------------------------------------------*/

pid_t pid_dial_pos  = {0};  //���̵��λ�û�
pid_t pid_dial_spd  = {0};	//���̵���ٶȻ�
pid_t pid_shot_spd[2]  = {0};	//Ħ�����ٶȻ�
/* �ڲ�����ԭ������----------------------------------------------------------*/
void Gun_Pid_Init()
{
  /*�������*/
		PID_struct_init(&pid_dial_pos, POSITION_PID, 6000, 5000,
									0.7f,	0.0f,	1.8f);  
		//pid_dial_pos.deadband = 10;
		PID_struct_init(&pid_dial_spd, POSITION_PID, 6000, 5000,
									1.5f,	0.0f,	0.15f	);  
   /*Ħ����*/
  for(uint8_t i = 0;i<2;i++)
  {
    PID_struct_init(&pid_shot_spd[i], POSITION_PID, 6000, 5000,
									1.8f,	0.0f,	0.0f	); 
  }
}
/* �������岿�� -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gun_Task(void const * argument)
	*	@param
	*	@supplement	ǹ��������������
	*	@retval	
****************************************************************************************/
  uint8_t motor_stop_flag=0;
  int32_t set_angle = 0;
	int32_t set_speed = 0;
  int32_t set_M_speed = 0;
  uint8_t set_cnt = 0;
  uint8_t block_flag;
  uint8_t bochi_count = 0;//���ݼ��� 0-4
  uint8_t contiue_flag = 0;
void Gun_Task(void const * argument)
{ 

	osDelay(100);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	Gun_Pid_Init();
  /*�趨����*/

	for(;;)
	{
    /*ˢ�¶���ʱ��*/
		RefreshTaskOutLineTime(GunTask_ON);
    switch(MoCa_Flag)
    {
      case 0:
      {
        /*Ħ�����ٶ�*/
        set_M_speed = 000;
        ptr_heat_gun_t.sht_flg = GunHold;
      }break;
      case 1:
      {
        /*Ħ�����ٶ�*/
        set_M_speed = 2000;
      }break;
    }
 /*�жϷ���ģʽ*/
    switch(ptr_heat_gun_t.sht_flg)
    {
			case GunStop://ֹͣ58982
			{
        switch(contiue_flag)
        {
          case 0:
          {
            /*�趨�Ƕ�*/
            set_angle = moto_dial_get.total_angle;	
            contiue_flag = 1;
          }break;
          case 1:
          {
            goto position;
          }break;
        }
        
			}break;
      case GunOne://����ģʽ
      {
        /*�趨�Ƕ�*/
        if(bochi_count != 4)//ǰ�ĸ��Ƕ�
        {
          moto_dial_get.cmd_time=GetSystemTimer();
          set_angle += 58982;//819.2*x
          if(bochi_count > 4)
          {
            bochi_count = 0;
            set_angle = 58982;
            /*����*/
            moto_dial_get.round_cnt=0;
            moto_dial_get.total_angle = moto_dial_get.angle - moto_dial_get.offset_angle;
          }
        }
        else//���һ���Ƕ�
        {
          moto_dial_get.cmd_time=GetSystemTimer();
          set_angle = 294912;//8192*36
        }
        /*����λ�û�*/
        bochi_count++;
        ptr_heat_gun_t.sht_flg = 11;
        contiue_flag = 0;
        
      }break;
      case GunFire://����ģʽ
      {
        set_speed = 1500;
      }break;
      case GunHold:
      {
        /*pidλ�û�*/
        position:
        pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	
				set_speed=pid_dial_pos.pos_out;
      }break;
      

			default :break;
    }
     ptr_heat_gun_t.sht_flg = GunHold;//Ĭ��λ�û�
     /*�ٶȻ�*/
     pid_calc(&pid_dial_spd,moto_dial_get.speed_rpm ,set_speed);
     pid_calc(&pid_shot_spd[0],moto_M_get[0].speed_rpm ,set_M_speed);
     pid_calc(&pid_shot_spd[1],moto_M_get[1].speed_rpm ,-set_M_speed);
    //printf("%d\t%d\r\n",moto_M_get[0].speed_rpm,moto_M_get[1].speed_rpm);
     /*�����������,Ħ����*/
		 Shot_Motor(&hcan2,pid_dial_spd.pos_out,pid_shot_spd[0].pos_out,pid_shot_spd[1].pos_out);
//		 Shot_Motor(&hcan2,0,0,0);
//     Shot_Motor(&hcan2,pid_dial_spd.pos_out,0,0);
		 minipc_rx_big.state_flag=0;
		 set_speed = 0;	   
    
        osDelayUntil(&xLastWakeTime,GUN_PERIOD);
	}
}


