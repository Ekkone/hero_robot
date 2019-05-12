/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "gun_task.h"
#include "math.h"
#include "SystemState.h"
#include "user_lib.h"
#include "protocol.h"
/* �ڲ��궨��----------------------------------------------------------------*/

/* �ڲ��Զ�����������--------------------------------------------------------*/

/* ���������Ϣ����----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* �ڲ���������--------------------------------------------------------------*/
#define GUN_PERIOD  10
#define BLOCK_TIME 5000
#define REVERSE_TIME 2000
#define prepare_flag HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_0)
#define READY    1
#define NO_READY 0
/* �ⲿ��������--------------------------------------------------------------*/
Heat_Gun_t  ptr_heat_gun_t;
uint8_t MoCa_Flag = 0;
uint16_t remain_heat = 0;
ramp_function_source_t shoot;
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
/* �ڲ�����ԭ������----------------------------------------------------------*/
void Gun_Pid_Init()
{
  /*�������*/
//		PID_struct_init(&pid_dial_pos, POSITION_PID, 10000, 5000,
//									5.0f,	0.0f,	3.0f);  
		//pid_dial_pos.deadband = 10;
		PID_struct_init(&pid_dial_spd, POSITION_PID, 10000, 5000,
									4.0f,	0.05f,	0.0f	);  
}
/* �������岿�� -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gun_Task(void const * argument)
	*	@param
	*	@supplement	ǹ��������������
	*	@retval	
****************************************************************************************/
void Gun_Task(void const * argument)
{ 

	osDelay(100);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  ramp_init(&shoot,0.05,190,100);//Ĩ����б��
	Gun_Pid_Init();
  /*�趨����*/
  uint8_t motor_stop_flag=0;
	static int32_t set_angle = 0;
	int32_t set_speed = 0;
  int32_t set_M_speed = 103;
	static uint8_t set_cnt = 0;
  static uint8_t block_flag;
  static uint16_t remain_heat = 0;
  static uint8_t contiue_flag = 0;
	for(;;)
	{
    /*ˢ�¶���ʱ��*/
		RefreshTaskOutLineTime(GunTask_ON);
    switch(MoCa_Flag)
    {
      case Init:
      {
        /*Ħ���ֳ�ʼ��*/
        shoot.max_value=103;
        ramp_calc(&shoot,103);
        Friction_Wheel_Motor(shoot.out,shoot.out);
      }break;
      case LowSpeed:
      {
        /*Ħ���ֵ���*/
        shoot.max_value = 130;
        ramp_calc(&shoot,130);
        Friction_Wheel_Motor(shoot.out,shoot.out);
      }break;
      case MiddleSpeed:
      {
        /*Ħ��������*/
        shoot.max_value = 160;
        ramp_calc(&shoot,160);
        Friction_Wheel_Motor(shoot.out,shoot.out);
      }break;
      case HighSpeed:
      {
        /*Ħ�����ٵ���*/
        shoot.max_value = 190;
        ramp_calc(&shoot,190);
        Friction_Wheel_Motor(shoot.out,shoot.out);
      }break;
    }
    
    /*��������*/
    if(remain_heat < 30)
      //ptr_heat_gun_t.sht_flg = GunStop;
    /*�жϷ���ģʽ*/
    switch(ptr_heat_gun_t.sht_flg)
    {
			case GunStop://ֹͣ58982
			{
        set_speed = 000;
        
//        switch(contiue_flag)
//        {
//          case 0:
//          {
//            /*�趨�Ƕ�*/
//            set_angle=moto_dial_get.total_angle;	
//            contiue_flag = 1;
//          }break;
//          case 1:
//          {
//            goto position;
//          }break;
//        }
			}break;
      case GunFire://����ģʽ
      {
				set_speed = 800;
//        contiue_flag = 0;
      }break;
//      case GunHold:
//      {
//        /*pidλ�û�*/
//       position: pid_calc(&pid_dial_pos,-moto_dial_get.total_angle,set_angle);	
//				set_speed=pid_dial_pos.pos_out;
//      }break;
			default :break; 
    }
     /*�ٶȻ�*/
    set_speed = 800;
     pid_calc(&pid_dial_spd,moto_dial_get.speed_rpm ,-set_speed);
     /*�����������*/
		 Shot_Motor(&hcan1,pid_dial_spd.pos_out);
//    Shot_Motor(&hcan1,5000);
    /*�����־λ*/
		 minipc_rx_small.state_flag=0;
		 set_speed = 0;	   
    
        osDelayUntil(&xLastWakeTime,GUN_PERIOD);
	}
}


