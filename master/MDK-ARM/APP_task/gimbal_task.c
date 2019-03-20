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
Pos_Set  pit_set;
int8_t gimbal_disable_flg;

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
pid_t pid_pit_jy61 = {0};
pid_t pid_yaw_jy61_spd = {0};
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
	PID_struct_init(&pid_pit, POSITION_PID, 5000, 1000,
									10.0f, 0.0f, 28.0f); 
	PID_struct_init(&pid_pit_spd, POSITION_PID, 5000, 1000,
                  1.0f, 0.0f, 0.0f );
  /*�ڵ���*/
//  PID_struct_init(&pid_pit, POSITION_PID, 5000, 1000,
//									15.0f, 0.0f, 10.0f); 
//	PID_struct_init(&pid_pit_spd, POSITION_PID, 5000, 1000,
//                  1.7f, 0.0f, 0.0f );

	
  /* yaw axis motor pid parameter */
	 PID_struct_init(&pid_yaw, POSITION_PID, 5000, 1000,
                  10.0f, 0.02f, 12.0f); 
	 PID_struct_init(&pid_yaw_spd, POSITION_PID, 5000, 1000,
                  2.0f, 0.0f, 0.0f );
  #endif
  #if jy61
	//use jy61
  PID_struct_init(&pid_pit_jy61, POSITION_PID, 5000, 1000,
                  5.0f, 0.1f, 25.0f); //	
  PID_struct_init(&pid_pit_jy61_spd, POSITION_PID, 5000, 1000,
                  2.5f, 0.0f, 1.0f ); 
  
  PID_struct_init(&pid_yaw_jy61, POSITION_PID, 5000, 100,
                  3.0f, 0.02f, 5.0f); //	
  PID_struct_init(&pid_yaw_jy61_spd, POSITION_PID, 5000, 100,
                  2.0f, 0.0f, 0.5f ); 
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
	gimbal_disable_flg=0;
	Pitch_Current_Value=0;
	Yaw_Current_Value=0;
	gimbal_pid_init();
	
	osDelay(200);//��ʱ200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();	
			
	for(;;)		
    {
	
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
        yaw_set.expect = minipc_rx.angle_yaw + yaw_set.expect;
        pit_set.expect = minipc_rx.angle_pit + pit_set.expect;
        minipc_rx.angle_yaw = 0;
        minipc_rx.angle_pit = 0;
        //yaw��
        pid_calc(&pid_yaw_jy61,(ptr_jy61_t_yaw.final_angle),yaw_set.expect);
        pid_calc(&pid_yaw_jy61_spd,(ptr_jy61_t_angular_velocity.vz), pid_yaw_jy61.pos_out);
        //pit��
        pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
        pid_calc(&pid_pit_jy61_spd,(ptr_jy61_t_angular_velocity.vy), pid_pit.pos_out);
      
        Pitch_Current_Value=(-pid_pit_jy61_spd.pos_out); 
		    Yaw_Current_Value= (-pid_yaw_jy61_spd.pos_out);
      #endif
        
//        printf("%f\r\n",ptr_jy61_t_yaw.final_angle);
        Pitch_Current_Value=0; 
		    Yaw_Current_Value= 0;
        /*�������*/
				if(gimbal_disable_flg==1)//ʧ��
				{
					Cloud_Platform_Motor_Disable(&hcan1);
				}
				else Cloud_Platform_Motor(&hcan1,Yaw_Current_Value,Pitch_Current_Value);

			osDelayUntil(&xLastWakeTime, GIMBAL_PERIOD);
			
   }
 
}
