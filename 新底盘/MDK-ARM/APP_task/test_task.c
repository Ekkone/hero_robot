/*************************************************************************************
*	@file			test_task.c
* @author	 	
*	@version 	V1.0
*	@date			
* @brief		NONE
*************************************************************************************//* Includes ------------------------------------------------------------------------*/
#include "test_task.h"
/* External variables --------------------------------------------------------------*/
/* Internal variables --------------------------------------------------------------*/
#define Check_PERIOD  100
/* Private function prototypes ---------------------------------------------------*/

/*测速模块*/
#define GunLength 0.05
#define MicroTime 0.000005

uint32_t Micro_Tick; //单位0.005ms
uint32_t Photoelectric_gate1 = 0,Photoelectric_gate2 = 0;
uint16_t gate1_counter = 0,gate2_counter = 0;
 float Golf_speed = 0;
int16_t Golf_counter = 0;
/*测速down*/

void testTask(void const * argument)
{
	
//	char InfoBuffer[1000];

	static double micro_timenow = 0;
	static double micro_timelast = 0;
	static double micro_time = 0;
	int16_t angle = 0;
  int16_t angle_yaw = 0;
  int16_t speed_set = 0;
	osDelay(200);//延时200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  float speed[30];
	
	
	for(;;)
	{ 				
		static uint16_t counter_last;
		static uint8_t count = 0;
    /*刷新断线时间*/
		RefreshTaskOutLineTime(testTask_ON);
    #if printf_power
    printf("%4f\r\n",limit.Power_Calculat);
    #endif
    #if printf_speed
    
			Golf_speed = (float)(GunLength / MicroTime / (Photoelectric_gate1 - Photoelectric_gate2));

			if(counter_last != gate1_counter)
			{  
				printf("Golf_speed = %4f\n",Golf_speed);
			}
			counter_last = gate1_counter;
		
    #endif 
  #if printf_sendware
		  int16_t  *ptr = &angle; //初始化指针
      int16_t  *p1  = &speed_set;
      float  *p2  = &(limit.Power_Calculat);
      float speed__= 0;
      float *p3 = &speed__;
			angle	= (pit_get.total_angle);
			/*用虚拟示波器，发送数据*/
//			vcan_sendware((uint8_t *)ptr,sizeof(angle));
		
//		printf("  pit=%d \n\t",pit_get.total_angle);
//	  printf("  yaw=%d \n\t",yaw_get.angle);
		
	#endif
		osDelayUntil(&xLastWakeTime,100);
	}
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/