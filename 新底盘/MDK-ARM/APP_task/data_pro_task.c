/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "data_pro_task.h"
#include "SystemState.h"
#include "Motor_USE_CAN.h"
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

/* ���������Ϣ����----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* �ڲ���������--------------------------------------------------------------*/
pid_t pid_minipc_yaw={0};
pid_t pid_minipc_pit={0};
pid_t pid_stir_spd;

#define REMOTE_PERIOD 5
#define MINIPC_PERIOD 2
#define REFEREE_PERIOD 5
/* �ⲿ��������--------------------------------------------------------------*/

/* ���õ��ⲿ����ԭ������------------------------------------------------------
	uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);
	uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
------------------------------------------------------------------------------
*/
/* �ڲ�����------------------------------------------------------------------*/
int16_t Y_speed_max = 6000;
int16_t Y_speed_min = -6000; 
int16_t X_speed_max = 4000;
int16_t X_speed_min = -4000; 
int16_t W_speed_max = 2000;
int16_t W_speed_min = -2000; 
uint8_t press_counter;
uint8_t shot_anjian_counter=0;
uint8_t shot_frequency = 100;
uint8_t chassis_gimble_Mode_flg = 0;
uint8_t communication_message = 1;
uint8_t stir_flag = 0;
uint8_t gun_num = 0;
uint8_t camera_flag = 0;
extern int16_t yaw_speed;
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
		//pid_pit_spd.deadband=10;//2.5f,	0.03f,	1.0f	
	
//    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);   //��Դ���� _����
}
/***************************************************************************************
**
	*	@brief	RemoteControlProcess()
	*	@param
	*	@supplement	��ң�������жԽӣ���ң���������ݽ��д���ʵ�ֶԵ��̡���̨����������Ŀ���
	*	@retval	
****************************************************************************************/
void ChassisModeProcess()
{
  /*ң�ظ����ݴ���*/
	         if(chassis_gimble_Mode_flg==1) //XY�˶������̸�����̨
					 {
							moto_3508_set.dstVmmps_X=-((RC_Ctl.rc.ch0-0x400)*5);
							moto_3508_set.dstVmmps_Y=-((RC_Ctl.rc.ch1-0x400)*5);
             
					 }
					 else//WY�˶���������̨����
					 {
							moto_3508_set.dstVmmps_W=((RC_Ctl.rc.ch0-0x400)*5);
							moto_3508_set.dstVmmps_Y=-((RC_Ctl.rc.ch1-0x400)*5);
					 }
           if(RC_Ctl.rc.s1 == 1)
             hard_brak();
           communication_message = 1;//˯��ģʽ
}
void ShotProcess()
{	
  switch(RC_Ctl.rc.s1)
  {
    case 1:
    {
      communication_message = 1;//˯��ģʽ
    }break;
    case 3:
    {
      communication_message = 4;//�嵯ģʽ
    }break;
    case 2:
    {
      communication_message = 0;//�Զ�ģʽ
    }break;
    default: break;
  }
  moto_3508_set.dstVmmps_W=((RC_Ctl.rc.ch0-0x400)*5);
	moto_3508_set.dstVmmps_Y=-((RC_Ctl.rc.ch1-0x400)*5);
}
/***************************************************************************************
**
	*	@brief	MouseKeyControlProcess()
	*	@param
	*	@supplement	�Լ�������ݽ��д���
	*	@retval	
****************************************************************************************/
uint8_t back_flag = 0;
uint8_t round_flag = 0;
void MouseKeyControlProcess()
{
	
  if(F_Press)//��ʱ�л�Ϊ����
   {
     back_flag = 1;
   } 
   else back_flag = 0;
  if(E_Press || Q_Press)//��ʱ�л�Ϊ����
  {
    round_flag = 1;
  }
  else round_flag = 0;
	if(SHIFT_Press)//����ٶ�
      {
        Y_speed_max = 7000;//(NORMAL_SPEED_MAX)*3.5;
        Y_speed_min = -7000;//(NORMAL_SPEED_MIN)*3.5;
        X_speed_max = 5000;//(NORMAL_SPEED_MAX)*3.5;
        X_speed_min = -5000;//(NORMAL_SPEED_MIN)*3.5;
        W_speed_max = 4000;
        W_speed_min = -4000; 
      }
  else if(G_Press)//����
    {
      Y_speed_max = 500;//(NORMAL_SPEED_MAX)*3.5;
      Y_speed_min = -500;//(NORMAL_SPEED_MIN)*3.5;
      X_speed_max = 500;//(NORMAL_SPEED_MAX)*3.5;
      X_speed_min = -500;//(NORMAL_SPEED_MIN)*3.5;
      W_speed_max = 500;
      W_speed_min = -500; 
    }
  else//�����ٶ�
  {
     Y_speed_max = 5500;//(NORMAL_SPEED_MAX)*3.5;
     Y_speed_min = -5500;//(NORMAL_SPEED_MIN)*3.5;
     X_speed_max = 4000;//(NORMAL_SPEED_MAX)*3.5;
     X_speed_min = -4000;//(NORMAL_SPEED_MIN)*3.5;
     W_speed_max = 3000;  
     W_speed_min = -3000;
  }
	/*Y���ٶ�*/
  if(W_Press)                       moto_3508_set.dstVmmps_Y -= ACC_SPEED;//����W��
  else if(S_Press)                  moto_3508_set.dstVmmps_Y += ACC_SPEED;//����S��
  else{  
        if(moto_3508_set.dstVmmps_Y>-DEC_SPEED&&moto_3508_set.dstVmmps_Y<DEC_SPEED) 	 moto_3508_set.dstVmmps_Y = 0;
        if(moto_3508_set.dstVmmps_Y>0) 	                   moto_3508_set.dstVmmps_Y -= DEC_SPEED;
        if(moto_3508_set.dstVmmps_Y<0) 		                 moto_3508_set.dstVmmps_Y += DEC_SPEED;
  }
  /*W���ٶ�*/
  if(Q_Press)                       moto_3508_set.dstVmmps_W -= ACC_SPEED;//����A��
  else if(E_Press)                  moto_3508_set.dstVmmps_W += ACC_SPEED;//����D��
  else{  
        if(moto_3508_set.dstVmmps_W>-DEC_SPEED&&moto_3508_set.dstVmmps_W<DEC_SPEED) 	 moto_3508_set.dstVmmps_W = 0;
        if(moto_3508_set.dstVmmps_W>0) 	                   moto_3508_set.dstVmmps_W -= DEC_SPEED;
        if(moto_3508_set.dstVmmps_W<0) 		                 moto_3508_set.dstVmmps_W += DEC_SPEED;
    }
  /*����ʱ*/
	if(chassis_gimble_Mode_flg == 0 || round_flag)
  {
    if(F_Press)
    {
      pid_calc(&pid_chassis_follow,-yaw_get.total_angle,0);
        /*�����ٶȻ�*/ 
			pid_calc(&pid_chassis_follow_spd,-yaw_speed,pid_chassis_follow.pos_out);
      moto_3508_set.dstVmmps_W = -pid_chassis_follow_spd.pos_out;
    }
    /*X���ٶ�*/
    if(A_Press)                        moto_3508_set.dstVmmps_X += ACC_SPEED; //����Q��
    else if(D_Press)    		           moto_3508_set.dstVmmps_X -= ACC_SPEED;//����E��
    else{
            if(moto_3508_set.dstVmmps_X>-DEC_SPEED&&moto_3508_set.dstVmmps_X<DEC_SPEED) 		moto_3508_set.dstVmmps_X = 0;		
            if(moto_3508_set.dstVmmps_X>0) 	                   moto_3508_set.dstVmmps_X -= DEC_SPEED;
            if(moto_3508_set.dstVmmps_X<0) 		                 moto_3508_set.dstVmmps_X += DEC_SPEED;
    }
  }
  /*����ʱ*/
  else if(chassis_gimble_Mode_flg == 1 || back_flag)
  {
    /*X���ٶ�*/
    if(D_Press)                        moto_3508_set.dstVmmps_X -= ACC_SPEED; //����D��
    else if(A_Press)    		           moto_3508_set.dstVmmps_X += ACC_SPEED;//����A��
    else{
            if(moto_3508_set.dstVmmps_X>-DEC_SPEED&&moto_3508_set.dstVmmps_X<DEC_SPEED) 		moto_3508_set.dstVmmps_X = 0;		
            if(moto_3508_set.dstVmmps_X>0) 	                   moto_3508_set.dstVmmps_X -= DEC_SPEED;
            if(moto_3508_set.dstVmmps_X<0) 		                 moto_3508_set.dstVmmps_X += DEC_SPEED;
      }
    
  }
  /*С��̨����*/
  if(B_Press)
  {
    communication_message = 1;//˯��ģʽ
  }
  else if(V_Press)
  {
    communication_message = 0;//�Զ�ģʽ
  }
  static uint8_t PatrolFlag = 0;
  if(communication_message)//����ģʽ
  {
    if(Z_Press && CTRL_Press) communication_message = 3;//�رղ���
    else if(Z_Press)          communication_message = 2;//�򿪲���

    if(PatrolFlag)
    {
      if(!R_Press)
      {
        communication_message = 0;
        PatrolFlag = 0;
      }
    }
  }
  else//�Զ�ģʽ
  {
    if(R_Press)
    {
      PatrolFlag = 1;
      communication_message = 1;//˯��ģʽ
    }
  }
  if(R_Press)//������׼
    gun_num = 2;
  else 
    gun_num = 1;
  if(CTRL_Press&&X_Press)
    camera_flag = 0;
  else if(X_Press)
    camera_flag = 1;
   
}


/***************************************************************************************
**
	*	@brief	hard_brak()
	*	@param
	*	@supplement	����ֹͣ����
	*	@retval	
****************************************************************************************/
void hard_brak()
{
		moto_3508_set.dstVmmps_X=0;
		moto_3508_set.dstVmmps_Y=0;
		moto_3508_set.dstVmmps_W=0;
}


/* �������岿�� -------------------------------------------------------------*/
/***************************************************************************************
**
	*	@brief	Data_Pro_task(void const * argument)
	*	@param
	*	@supplement	ң�����ݽ��ռ���������
	*	@retval	
****************************************************************************************/
float capvolt;
uint8_t small_gun_flag;
uint8_t big_gun_flag;
void Remote_Data_Task(void const * argument)
{
		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
	
	PID_struct_init(&pid_stir_spd, POSITION_PID,15000,1000,
	                4.0f, 0.01f , 0.0f  );
	for(;;)
	{
    /*���͸���������*/
    if(minipc_rx_small.state_flag != 0 &&  minipc_rx_small.state_flag != 1)
         small_gun_flag = 1;
    else small_gun_flag = 0;
    
    if(minipc_rx_big.state_flag != 0 &&  minipc_rx_big.state_flag != 1)
         big_gun_flag = 1;
    else big_gun_flag = 0;
    capvolt = Show_CapVolt();
//    sendata(1,0,0,1,0,0,1,1,0);
    sendata(capvolt,0,0,!stir_motor_flag,small_gun_flag,0,0,0,big_gun_flag);
			RefreshTaskOutLineTime(RemoteDataTask_ON);
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
				
			VAL_LIMIT(moto_3508_set.dstVmmps_X, X_speed_min, X_speed_max);
			VAL_LIMIT(moto_3508_set.dstVmmps_Y, Y_speed_min, Y_speed_max);	
			VAL_LIMIT(moto_3508_set.dstVmmps_W, W_speed_min, W_speed_max);
      CAN_Send_B(&hcan1);
      CAN_Send_S(&hcan1);
      Send_MiniPC_Data(gun_num,camera_flag);
      minipc_rx_small.angle_yaw  = 0;
      minipc_rx_small.angle_pit  = 0;
      minipc_rx_small.state_flag = 0;
      minipc_rx_big.angle_yaw  = 0;
      minipc_rx_big.angle_pit  = 0;
      minipc_rx_big.state_flag = 0;
      press_counter++;
        osDelayUntil(&xLastWakeTime, REMOTE_PERIOD);
	}

}

