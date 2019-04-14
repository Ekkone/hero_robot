/*************************************************************************************
*	@file			status_task.c
* @author	 	
*	@version 	V1.0
*	@date			
* @brief		NONE
*************************************************************************************//* Includes ------------------------------------------------------------------------*/
#include "status_task.h"
/* External variables --------------------------------------------------------------*/
#define Check_PERIOD  13
#define BLINK_PERIOD  100
#define OFF_PERIOD    200
#define BLINK_GREEN();  {GREEN_LED(1);osDelayUntil(&xLastWakeTime,BLINK_PERIOD);GREEN_LED(0);osDelayUntil(&xLastWakeTime,OFF_PERIOD);}
#define BLINK_RED();    {RED_LED(1);osDelayUntil(&xLastWakeTime,BLINK_PERIOD);RED_LED(0);osDelayUntil(&xLastWakeTime,OFF_PERIOD);}
/* Internal variables --------------------------------------------------------------*/
/* Private function prototypes ---------------------------------------------------*/
void Status_Task(void const * argument)
{
  osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for(;;)
  {
    while(0)
    {
      OFF:RED_LED(0);
      osDelayUntil(&xLastWakeTime,2000);
    }
    if((SystemState.OutLine_Flag&0x01))//REMOTE
    {
      BLINK_RED();
      goto OFF;
    }
    
    if((SystemState.OutLine_Flag&0x02))//MOTOR_YAW
    {
      BLINK_RED();
      BLINK_RED();
      goto OFF;
    }
    
    if((SystemState.OutLine_Flag&0x04))//MOTOR_PIT
    {
      BLINK_RED();
      BLINK_RED();
      BLINK_RED();
      goto OFF;
    }
    
    if((SystemState.OutLine_Flag&0x08))//MOTOR_B
    {
      BLINK_RED();
      BLINK_RED();
      BLINK_RED();
      BLINK_RED();
      goto OFF;
    }
    else 
    {
      RED_LED(0);
      osDelayUntil(&xLastWakeTime,Check_PERIOD);
    }
  }
}
void Check_Task(void const * argument)
{
	osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
    while(0)
    {
      OFF:GREEN_LED(0);
      osDelayUntil(&xLastWakeTime,2000);
    }
    if((SystemState.task_OutLine_Flag&0x01))//测试任务
    {
      BLINK_GREEN();
      goto OFF;
    }
    else if((SystemState.task_OutLine_Flag&0x02))//遥控任务
    {
      BLINK_GREEN();
      BLINK_GREEN();
      goto OFF;
    }
    else if((SystemState.task_OutLine_Flag&0x04))//云台任务
    {
      BLINK_GREEN();
      BLINK_GREEN();
      BLINK_GREEN();
      goto OFF;
    }
    else if((SystemState.task_OutLine_Flag&0x08))//发射任务
    {
      BLINK_GREEN();
      BLINK_GREEN();
      BLINK_GREEN();
      BLINK_GREEN();
      goto OFF;
    }
    else if((SystemState.task_OutLine_Flag&0x10))//掉线任务
    {
      BLINK_GREEN();
      BLINK_GREEN();
      BLINK_GREEN();
      BLINK_GREEN();
      BLINK_GREEN();
      goto OFF;
    }
    else 
    {
      GREEN_LED(1);
      //GREEN_Blink();
      osDelayUntil(&xLastWakeTime,Check_PERIOD);
    }
    
  }
	
	
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/