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
#define BLOCK_TIME 2000
#define STIR_BLOCK_TIME 1000
#define prepare_flag HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_0)
#define READY    1
#define NO_READY 0
/* �ⲿ��������--------------------------------------------------------------*/
Heat_Gun_t  ptr_heat_gun_t;
volatile uint8_t MoCa_Flag = Init;
uint16_t remain_heat = 0;
ramp_function_source_t shoot;
extern uint8_t shot_frequency;
extern float Golf_speed;
extern uint8_t stir_motor_flag;
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
pid_t pid_stir_spd;
/* �ڲ�����ԭ������----------------------------------------------------------*/
void Gun_Pid_Init()
{
  /*�������*/
//		PID_struct_init(&pid_dial_pos, POSITION_PID, 10000, 5000,
//									5.0f,	0.0f,	3.0f);  
		//pid_dial_pos.deadband = 10;
		PID_struct_init(&pid_dial_spd, POSITION_PID, 10000, 6000,
									5.0f,	0.5f,	0.0f	);  
  PID_struct_init(&pid_stir_spd, POSITION_PID,15000,1000,
	                4.0f, 0.01f , 0.0f  );
}
/* �������岿�� -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gun_Task(void const * argument)
	*	@param
	*	@supplement	ǹ��������������
	*	@retval	
****************************************************************************************/
uint32_t locktime = 0;
uint32_t stir_locktime = 0;
void Gun_Task(void const * argument)
{ 

	osDelay(100);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  ramp_init(&shoot,0.05,190,100);//Ĩ����б��
	Gun_Pid_Init();
  /*�趨����*/
  uint8_t motor_stop_flag=0;
  uint8_t set_cnt = 0;
	static int32_t set_angle = 0;
	int32_t set_speed = 0;
  int32_t set_M_speed = 103;
  static uint8_t block_flag;
  static uint16_t remain_heat = 0;
  static uint8_t contiue_flag = 0;

  int16_t set_stir_speed = 0;
	for(;;)
	{
    /*ˢ�¶���ʱ��*/
		RefreshTaskOutLineTime(GunTask_ON);
    switch(MoCa_Flag)
    {
      case Init:
      {
        /*Ħ��������*/
        shoot.max_value=103;
        ramp_calc(&shoot,103);
        Friction_Wheel_Motor(shoot.out,shoot.out);
      }
      case HighSpeed:
      {
        /*Ħ��������*/
        shoot.max_value=117;
        ramp_calc(&shoot,103);
        Friction_Wheel_Motor(shoot.out,shoot.out);
      }
    }
    
    /*��������*/
    if(remain_heat < 30)
//      ptr_heat_gun_t.sht_flg = GunStop;
    /*�жϷ���ģʽ*/
    switch(ptr_heat_gun_t.sht_flg)
    {
			case GunStop://ֹͣ58982
			{
        set_speed = 0;
        set_cnt = 0;
        moto_dial_get.run_time=GetSystemTimer();//δ��תʱ��
        locktime = 0;
			}break;
      case GunFire://����ģʽ
      {
        if(Check_locked() || locktime)//��ת���
        {
          locktime--;
          set_speed = -800;
        }
        else//δ��ת
          set_speed = 2000;
      }break;
			default :break; 
    }
    /*���͵��*/
    if(stir_motor_flag)
    {
      if(Check_stir_locked() || stir_locktime)//��ת
      {
        stir_locktime--;
        set_stir_speed = 500;
      }
      else set_stir_speed = -500;
    }
    else
      set_stir_speed = 0;
    
       pid_calc(&pid_stir_spd,moto_stir_get.speed_rpm ,set_stir_speed);
     /*�������*/
     pid_calc(&pid_dial_spd,moto_dial_get.speed_rpm ,-set_speed);
    
     /*�������*/
    Shot_Motor(&hcan1,pid_dial_spd.pos_out,pid_stir_spd.pos_out);
    /*�����־λ*/
		 minipc_rx_small.state_flag=0;
		 set_speed = 0;	   
    
        osDelayUntil(&xLastWakeTime,GUN_PERIOD);
	}
}
uint8_t Check_locked(void)
{
  moto_dial_get.cmd_time = GetSystemTimer();//ˢ�µ�ǰʱ��
  if(!locktime)
  {
		     /*�жϲ����Ƿ�ת��λ��*/			
					if(my_abs(moto_dial_get.round_cnt) >=2)
						{
							
										moto_dial_get.round_cnt=0;
										moto_dial_get.offset_angle=moto_dial_get.angle;
										moto_dial_get.total_angle=0;
										moto_dial_get.run_time=GetSystemTimer();//δ��תʱ��
                    return 0;//δ��ת
						}
						else if( my_abs(moto_dial_get.run_time-moto_dial_get.cmd_time)>BLOCK_TIME )//��ת�ж�
						{
                    locktime = 100;
									  return 1;	//��ת
            }
            else return 0;//��תʱ���������
   }
    else return 0;
}
uint8_t Check_stir_locked(void)
{
  moto_stir_get.cmd_time = GetSystemTimer();//ˢ�µ�ǰʱ��
  if(!stir_locktime)
  {
		     /*�жϲ����Ƿ�ת��λ��*/			
					if(my_abs(moto_stir_get.total_angle) >= 70)
						{
							
										moto_stir_get.round_cnt=0;
										moto_stir_get.offset_angle=moto_stir_get.angle;
										moto_stir_get.total_angle=0;
										moto_stir_get.run_time=GetSystemTimer();//δ��תʱ��
                    return 0;//δ��ת
						}
						else if( my_abs(moto_stir_get.run_time-moto_stir_get.cmd_time)>STIR_BLOCK_TIME )//��ת�ж�
						{
                    stir_locktime = 60;
									  return 1;	//��ת
            }
            else return 0;//��תʱ���������
   }
    else return 0;
}

