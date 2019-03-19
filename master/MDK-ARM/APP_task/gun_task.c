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
#define GUN_PERIOD  10
#define Mocha_PERIOD  1
#define BLOCK_TIME 5000
#define REVERSE_TIME 2000
#define prepare_flag HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_0)
#define READY    1
#define NO_READY 0
/* �ⲿ��������--------------------------------------------------------------*/
Heat_Gun_t  ptr_heat_gun_t;
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
pid_t pid_stir_spd = {0};
/* �ڲ�����ԭ������----------------------------------------------------------*/
void Gun_Pid_Init()
{
  /*�������*/
		PID_struct_init(&pid_dial_pos, POSITION_PID, 6000, 5000,
									1.5f,	0.0f,	3.0f);  
		//pid_dial_pos.deadband = 10;
		PID_struct_init(&pid_dial_spd, POSITION_PID, 6000, 5000,
									1.0f,	0.0f,	0.1f	);  
		/*���̵��*/
		PID_struct_init(&pid_stir_spd, POSITION_PID, 6000, 5000,
									1.5f,	0.0f,	0.0f	); 	
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

	Gun_Pid_Init();
  /*�趨����*/
  uint8_t motor_stop_flag=0;
	static int32_t set_angle = 0;
	int32_t set_speed = 0;
  int32_t set_stir_speed = 0;
	static uint8_t set_cnt = 0;
  static uint8_t block_flag;
  static float check_time = 0;

	for(;;)
	{
		RefreshTaskOutLineTime(GunTask_ON);

 /*�жϷ���ģʽ*/
    switch(ptr_heat_gun_t.sht_flg)
    {
			case 0://ֹͣ58982
			{
				set_angle=0;
				set_speed=0;
        set_stir_speed = 0;
				set_cnt=0;
				moto_dial_get.cmd_time=GetSystemTimer();
			}break;
      case 1://����ģʽ
      {
        /*�趨�Ƕ�*/
				moto_dial_get.cmd_time=GetSystemTimer();
				set_cnt=1;
				set_angle=58982*set_cnt;
        /*����*/
				moto_dial_get.round_cnt=0;
				moto_dial_get.offset_angle=moto_dial_get.angle;
				moto_dial_get.total_angle=0;	
        /*����λ�û�*/
        ptr_heat_gun_t.sht_flg = 11;
      }break;
      case 11:
      {
        /*pidλ�û�*/
        pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	
				set_speed=pid_dial_pos.pos_out;
        set_stir_speed = 1000;
        //set_speed = 5000;
      }break;
      case 2://3����ģʽ
      {
				moto_dial_get.cmd_time=GetSystemTimer();
				set_cnt=3;
				set_angle=58982*set_cnt;
        set_stir_speed = 1000;
        /*����*/
				moto_dial_get.round_cnt=0;
				moto_dial_get.offset_angle=moto_dial_get.angle;
				moto_dial_get.total_angle=0;
        /*����λ�û�*/
        ptr_heat_gun_t.sht_flg = 11;
      }break;
      case 3://����ģʽ
      { 
				moto_dial_get.cmd_time=GetSystemTimer();
        set_speed = 5000;
        set_stir_speed = 1000;
        set_cnt=1;
				
      }break;
			case 10://��ת
			{
				set_speed=-1000;
				moto_dial_get.reverse_time=GetSystemTimer();
        /*pidλ�û�*/
			}break;
			default :break;
    }
     /*�ٶȻ�*/
     pid_calc(&pid_dial_spd,moto_dial_get.speed_rpm ,set_speed);
    pid_calc(&pid_stir_spd,moto_stir_get.speed_rpm ,500);
      
    printf("speed=%4f\r\n",Golf_speed);
     /*�����������*/
		 Allocate_Motor(&hcan1,pid_dial_spd.pos_out);
     Stir_Motor(&hcan1,2000);
		 minipc_rx.state_flag=0;
		 set_speed=0;	   
    
        osDelayUntil(&xLastWakeTime,GUN_PERIOD);
	}
}


