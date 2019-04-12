/*************************************************************************************
*	@file			color.c
* @author	 	
*	@version 	V1.0
*	@date			
* @brief		NONE
*************************************************************************************//* Includes ------------------------------------------------------------------------*/
#include "status_task.h"
/* External variables --------------------------------------------------------------*/
#define Check_PERIOD  100
/* Internal variables --------------------------------------------------------------*/
/* Private function prototypes ---------------------------------------------------*/
void Status_Task(void const * argument)
{
  osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for(;;)
  {
    osDelayUntil(&xLastWakeTime,100);
  }
}
void Check_Task(void const * argument)
{
	osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
	if((SystemState.task_OutLine_Flag&0x01))
				{
					printf("testTask GG \n\t");
					osDelayUntil(&xLastWakeTime,100);
				}
				
				
				if((SystemState.task_OutLine_Flag&0x02))
				{
					printf("ChassisContrlTask GG \n\t");
					Chassis_Motor_Disable(&hcan2);
					osDelayUntil(&xLastWakeTime,100);
				} 
				
				
				if((SystemState.task_OutLine_Flag&0x04))
				{
						printf("RemoteDataTask GG \n\t");
						HAL_UART_DMAPause(&huart1);
				    *USART1_RX_DATA = 0;
						osDelayUntil(&xLastWakeTime,100);
				} 
				
				
				if((SystemState.task_OutLine_Flag&0x10))
				{
						printf("GunTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 
				

				if((SystemState.task_OutLine_Flag&0x40))
				{
						printf("vOutLineCheckTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 

		
				osDelayUntil(&xLastWakeTime,Check_PERIOD);
	
	}
	
	
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/