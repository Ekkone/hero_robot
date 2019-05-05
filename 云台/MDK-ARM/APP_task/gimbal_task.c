/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "gimbal_task.h"
#include "Power_restriction.h"
#include "SystemState.h"
/* �ڲ��궨��----------------------------------------------------------------*/

/* �ڲ��Զ�����������--------------------------------------------------------*/
static  int16_t Yaw_Current_Value = 0;
static  int16_t Pitch_Current_Value = 0;
/* ���������Ϣ����----------------------------------------------------------*/

/* �ڲ���������--------------------------------------------------------------*/
#define GIMBAL_PERIOD 5
/* �ⲿ��������--------------------------------------------------------------*/
Pos_Set  yaw_set;
Pos_Set  yaw_set_follow;
Pos_Set  pit_set;
int8_t gimbal_disable_flg;
uint8_t stir_motor_flag = 0;
/* ���õ��ⲿ����ԭ������------------------------------------------------------------
void Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch);
float pid_calc(pid_t* pid, float get, float set);
------------------------------------------------------------------------------
*/
/* �ڲ�����------------------------------------------------------------------*/

pid_t pid_yaw       = {0};  //yaw��λ�û�
pid_t pid_pit       = {0};	//pit��λ�û�
pid_t pid_yaw_spd   = {0};	//yaw���ٶȻ�
pid_t pid_pit_spd   = {0};	//pit���ٶȻ�


pid_t pid_yaw_jy61 = {0};  //��������� /*Ŀǰֻ����λ�û�*/ 
pid_t pid_yaw_jy61_follow = {0}; 
pid_t pid_pit_jy61 = {0};
pid_t pid_yaw_jy61_spd = {0};
pid_t pid_yaw_jy61_follow_spd = {0};
pid_t pid_pit_jy61_spd = {0};


/* �ڲ�����ԭ������----------------------------------------------------------*/
/**                                                           //����
	**************************************************************
	** Descriptions: ��̨pid��ʼ��
	** Input:  NULL
	** Output: NULL
	**************************************************************
**/

void gimbal_pid_init(void)
{
		/*pitch axis motor pid parameter*/
  #if imu
  /*imu pid parameter*/
  /*��ʱ�ȶ���*/
  /*50*/
	PID_struct_init(&pid_pit, POSITION_PID, 5000, 1000,
									5.0f, 0.0f, 14.8f); 
	PID_struct_init(&pid_pit_spd, POSITION_PID, 5000, 1000,
                  2.0f, 0.0f, 0.0f );
  /*�ڵ���*/
  /*35*/
//  PID_struct_init(&pid_pit, POSITION_PID, 5000, 1000,
//									4.0f, 0.05f, 15.5f); 
//	PID_struct_init(&pid_pit_spd, POSITION_PID, 5000, 1000,
//                  2.0f, 0.0f, 0.0f );

	
  /* yaw axis motor pid parameter */
	 PID_struct_init(&pid_yaw, POSITION_PID, 5000, 1000,
                  10.0f, 0.02f, 10.0f); 
	 PID_struct_init(&pid_yaw_spd, POSITION_PID, 5000, 1000,
                  2.0f, 0.0f, 0.0f );
  #endif
  #if jy61
/*��ʱ�ȶ���*/
#define YAW_JY 0
#define PIT_JY 0

#if PIT_JY
/*pit�����Ƿ���*/
  PID_struct_init(&pid_pit_jy61, POSITION_PID, 5000, 1000,
                  6.0f, 0.05f, 28.0f); //	
  PID_struct_init(&pid_pit_jy61_spd, POSITION_PID, 5000, 1000,
                  2.2f, 0.0f, 0.0f );
#else
/*pit����������*/
/*�����ȶ�*/
//  PID_struct_init(&pid_pit_jy61, POSITION_PID, 5000, 1000,
//                  11.0f, 0.07f, 9.0f); //	
//  PID_struct_init(&pid_pit_jy61_spd, POSITION_PID, 5000, 1000,
//                  2.5f, 0.0f, 0.0f ); 
  PID_struct_init(&pid_pit_jy61, POSITION_PID, 5000, 100,
                  20.0f, 0.00f, 22.0f); //	
  PID_struct_init(&pid_pit_jy61_spd, POSITION_PID, 5000, 1000,
                  2.5f, 0.0f, 0.0f  ); 
#endif

/*yaw�����Ƿ���*/
  PID_struct_init(&pid_yaw_jy61_follow, POSITION_PID, 5000, 300,
                  7.0f, 0.1f, 40.0f); //	
  PID_struct_init(&pid_yaw_jy61_follow_spd, POSITION_PID, 5000, 100,
                  2.0f, 0.0f, 0.0f );

/*yaw����������*/
  PID_struct_init(&pid_yaw_jy61, POSITION_PID, 5000, 300,
                  12.0f, 0.1f, 12.0f); //	
  PID_struct_init(&pid_yaw_jy61_spd, POSITION_PID, 5000, 100,
                  2.5f, 0.0f, 0.0f );
  #endif

	
}
/* �������岿�� -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gimbal_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	��̨�������
	*	@retval	
****************************************************************************************/
void Gimbal_Contrl_Task(void const * argument)
{
	yaw_set.expect = 0; 
	pit_set.expect = 0;
	yaw_set.mode   = 0;
  yaw_set_follow.expect = 0; 
	yaw_set_follow.mode   = 0;
	gimbal_disable_flg=0;
	Pitch_Current_Value=0;
	Yaw_Current_Value=0;
	gimbal_pid_init();
  /*��̨����*/
	uint16_t pit_protect_correct_1 = 0;
	uint16_t pit_protect_correct_2 = 0;
  if(pit_get.offset_angle < 5000)
  {
    pit_protect_correct_1 = 8192;
    pit_protect_correct_2 = 0;
  }
  else
  {
    pit_protect_correct_1 = 0;
    pit_protect_correct_2 = 8192;
  }
	osDelay(200);//��ʱ200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();	
			
	for(;;)		
    {
      /*ˢ�¶���ʱ��*/
	   RefreshTaskOutLineTime(GimbalContrlTask_ON);
      #if imu
        IMU_Get_Data();
        //yaw��
      
        pid_calc(&pid_yaw, yaw_get.total_angle,yaw_set.expect);
        pid_calc(&pid_yaw_spd,(imu_data.gz), pid_yaw.pos_out);
        //pit��
        pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
        pid_calc(&pid_pit_spd,(imu_data.gy), pid_pit.pos_out);
//        pid_calc(&pid_pit_spd,(imu_data.gy), 0);
      
        Pitch_Current_Value=(-pid_pit_spd.pos_out); 
		    Yaw_Current_Value= (-pid_yaw_spd.pos_out);
      #endif
      #if jy61
      IMU_Get_Data();
      
      /*��̨��λ����*/
      /*pit����280-800*/
      if((pit_set.expect + pit_get.offset_angle) > 800)
      {
        if(pit_set.expect <= pit_set.expect_last)
          goto pit_last;
        pit_set.expect = 780 - pit_get.offset_angle;
      }
      if((pit_set.expect + pit_get.offset_angle) < 280)
      {
        if(pit_set.expect >= pit_set.expect_last)
          goto pit_last;
        pit_set.expect = 300 - pit_get.offset_angle;
      }
      #if PIT_JY
      //pit��������
      pid_calc(&pid_pit_jy61, (ptr_jy61_t_pit.final_angle*22.76), pit_set.expect);
      pid_calc(&pid_pit_jy61_spd,(ptr_jy61_t_angular_velocity.vy), pid_pit_jy61.pos_out);
      #else
      //pit�������
      pit_last:pid_calc(&pid_pit_jy61, pit_get.total_angle, pit_set.expect);
      pid_calc(&pid_pit_jy61_spd,(ptr_jy61_t_angular_velocity.vy), pid_pit_jy61.pos_out);
      #endif
      Pitch_Current_Value=(-pid_pit_jy61_spd.pos_out); 
      /*yaw��ģʽ�ж�*/
      switch(chassis_gimble_Mode_flg)
      {
        case 0://���룬yawʹ�ñ�����
        {
          /*yaw����̨����3100-4300*/
          if((yaw_set.expect + yaw_get.offset_angle) > 4300)
          {
            if(yaw_set.expect <= yaw_set.expect_last)
              goto yaw_last;
            yaw_set.expect = 4300 - yaw_get.offset_angle;
          }
          if((yaw_set.expect + yaw_get.offset_angle) < 3100)
          {
            if(yaw_set.expect >= yaw_set.expect_last)
              goto yaw_last;
            yaw_set.expect = 3120 - yaw_get.offset_angle;
          }
          yaw_last:pid_calc(&pid_yaw_jy61,(-yaw_get.total_angle),yaw_set.expect);
          pid_calc(&pid_yaw_jy61_spd,(-ptr_jy61_t_angular_velocity.vz), pid_yaw_jy61.pos_out);
          Yaw_Current_Value= (pid_yaw_jy61_spd.pos_out);
        }break;
        case 1://���棬yawʹ��������
        {
          /*yaw����̨����*/
          
          if(yaw_get.angle > 4300)
          {
            if(yaw_set_follow.expect <= yaw_set_follow.expect_last)
            goto yaw_follow_last;
            yaw_set_follow.expect = ptr_jy61_t_yaw.final_angle;
          }

          if(yaw_get.angle < 3100)
          {
            if(yaw_set_follow.expect >= yaw_set_follow.expect_last)
            goto yaw_follow_last;
            yaw_set_follow.expect = ptr_jy61_t_yaw.final_angle;
          }
      yaw_follow_last:pid_calc(&pid_yaw_jy61_follow,(ptr_jy61_t_yaw.final_angle),yaw_set_follow.expect);
          pid_calc(&pid_yaw_jy61_follow_spd,(ptr_jy61_t_angular_velocity.vz), pid_yaw_jy61_follow.pos_out);
          Yaw_Current_Value= (-pid_yaw_jy61_follow_spd.pos_out);
          
          
        }break;
      }  
		    
      #endif
        //Pitch_Current_Value = 0;
        //Yaw_Current_Value = 0;
        /*�������*/
				if(gimbal_disable_flg)//ʧ��
				{
					Cloud_Platform_Motor_Disable(&hcan1);
				}
				else Cloud_Platform_Motor(&hcan1,Yaw_Current_Value,Pitch_Current_Value);
        
        yaw_set_follow.expect_last = yaw_set_follow.expect;
        yaw_set.expect_last = yaw_set.expect;
        pit_set.expect_last = pit_set.expect;
        
      minipc_rx_big.angle_yaw = 0;
      minipc_rx_big.angle_pit = 0;
        /*���͵���*/
      CAN_Send_YT(&hcan1,yaw_get.total_angle,0,chassis_gimble_Mode_flg,stir_motor_flag);
			osDelayUntil(&xLastWakeTime, GIMBAL_PERIOD);
			
   }
 
}
