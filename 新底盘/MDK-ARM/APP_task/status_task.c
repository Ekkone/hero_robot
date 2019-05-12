/*************************************************************************************
*	@file			status_task.c
* @author	 	
*	@version 	V1.0
*	@date			
* @brief		NONE
*************************************************************************************//* Includes ------------------------------------------------------------------------*/
#include "status_task.h"
/* External variables --------------------------------------------------------------*/
#define Check_PERIOD  16
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
      OFF:GREEN_LED(0);
      osDelayUntil(&xLastWakeTime,2000);
    }
    if((SystemState.task_OutLine_Flag&0x01))//测试任务
    {
      BLINK_GREEN();
      goto OFF;
    }
    else if((SystemState.task_OutLine_Flag&0x02))//底盘任务
    {
      BLINK_GREEN();
      BLINK_GREEN();
      goto OFF;
    }
    else if((SystemState.task_OutLine_Flag&0x04))//通信任务
    {
      BLINK_GREEN();
      BLINK_GREEN();
      BLINK_GREEN();
      goto OFF;
    }
    else if((SystemState.task_OutLine_Flag&0x08))//掉线任务
    {
      BLINK_GREEN();
      BLINK_GREEN();
      BLINK_GREEN();
      BLINK_GREEN();
      goto OFF;
    }
    else 
    {
      GREEN_Blink();
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
    #if BoardOld
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
    
    if((SystemState.OutLine_Flag&0x08))//MOTOR_M1
    {
      BLINK_RED();
      BLINK_RED();
      BLINK_RED();
      BLINK_RED();
      goto OFF;
    }
    
    if((SystemState.OutLine_Flag&0x10))//REMOT_M2
    {
      BLINK_RED();
      BLINK_RED();
      BLINK_RED();
      BLINK_RED();
      BLINK_RED();
      goto OFF;
    }
    
    if((SystemState.OutLine_Flag&0x20))//JY61
    {
      BLINK_RED();
      BLINK_RED();
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
    #else
    if(!(SystemState.OutLine_Flag&0x01))//MOTOR1
      LED1_Blink();
    else LED1(0);
    
    if(!(SystemState.OutLine_Flag&0x02))//MOTOR2
      LED2_Blink();
    else LED2(0);
    
    if(!(SystemState.OutLine_Flag&0x04))//MOTOR3
      LED3_Blink();
    else LED3(0);
    
    if(!(SystemState.OutLine_Flag&0x08))//MOTOR4
      LED4_Blink();
    else LED4(0);
    
    if(!(SystemState.OutLine_Flag&0x10))//REMOTE
      LED5_Blink();
    else LED5(0);
    
    if(!(SystemState.OutLine_Flag&0x20))//REFEREE
      LED6_Blink();
    else LED6(0);
    
    if(!(SystemState.OutLine_Flag&0x40))//MINI
      LED7_Blink();
    else LED7(0);
    printf("1");
    osDelayUntil(&xLastWakeTime,200);
    #endif
    
    
  }
	
	
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/