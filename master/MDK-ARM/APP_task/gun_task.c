/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "gun_task.h"
#include "math.h"
#include "SystemState.h"
/* �ڲ��궨��----------------------------------------------------------------*/

/* �ڲ��Զ�����������--------------------------------------------------------*/

/* ���������Ϣ����----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* �ڲ���������--------------------------------------------------------------*/
#define GUN_PERIOD  10
/* �ⲿ��������--------------------------------------------------------------*/
Heat_Gun_t  ptr_heat_gun_t;
extern uint8_t shot_frequency;
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
  /**/
		PID_struct_init(&pid_dial_pos, POSITION_PID, 6000, 5000,
									1.0f,	0.0000f,	0.0f);  
		//pid_pos[i].deadband=500;
		PID_struct_init(&pid_dial_spd, POSITION_PID, 6000, 5000,
									1.5f,	0.0f,	0.0f	);  
		pid_pit_spd.deadband=10;//2.5f,	0.03f,	1.0f	
  /*Ħ����pid��ʼ��*/
    PID_struct_init(&pid_shot_spd[0], POSITION_PID, 6000, 5000,
									1.5f,	0.0f,	0.0f	);  
    PID_struct_init(&pid_shot_spd[1], POSITION_PID, 6000, 5000,
									1.5f,	0.0f,	0.0f	); 
		pid_pit_spd.deadband=10;//2.5f,	0.03f,	1.0f
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


	for(;;)
	{
		RefreshTaskOutLineTime(GunTask_ON);
		
 /*�жϷ���ģʽ*/
    switch(ptr_heat_gun_t.sht_flg)
    {
      case 0://ֹͣ
      {
        
      }break;
      case 1://����ģʽ
      {
        
      }break;
      case 2://����ģʽ
      {

      }break;
    }
    
    
        osDelayUntil(&xLastWakeTime,GUN_PERIOD);
	}
}


