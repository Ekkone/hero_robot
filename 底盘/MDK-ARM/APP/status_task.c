/*************************************************************************************
*	@file			status_task.c
* @author	 	
*	@version 	V1.0
*	@date			
* @brief		NONE
*************************************************************************************//* Includes ------------------------------------------------------------------------*/
#include "status_task.h"
/* External variables --------------------------------------------------------------*/
#define Check_PERIOD  100
#define BLINK_PERIOD  100
#define OFF_PERIOD    200
#define BLINK(); {LED8(1);osDelayUntil(&xLastWakeTime,BLINK_PERIOD);LED8(0);osDelayUntil(&xLastWakeTime,OFF_PERIOD);}
/* Internal variables --------------------------------------------------------------*/
/* Private function prototypes ---------------------------------------------------*/
/**
	**************************************************************
	** Descriptions:Status_Task
	** Input: 检测各模块掉线情况，正常则闪烁，不正常则灭	
  **						
	**					
	**					
	** Output: NULL
	**************************************************************
**/
void Status_Task(void const * argument)
{
  osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for(;;)
  {
    if(!(SystemState.OutLine_Flag&0x01) ||!(SystemState.OutLine_Flag&0x02) ||\
        !(SystemState.OutLine_Flag&0x04) || !(SystemState.OutLine_Flag&0x08) )//底盘四个电机
      LED1_Blink();
    else LED1(0);
    if(!(SystemState.OutLine_Flag&0x10))//REMOTE
      LED2_Blink();
    else LED2(0);
    if(!(SystemState.OutLine_Flag&0x20))//MOTORS
      LED3_Blink();
    else LED3(0);
    if(!(SystemState.OutLine_Flag&0x40))//REFEREE
      LED4_Blink();
    else LED4(0);
    if(!(SystemState.OutLine_Flag&0x80))//MINI_B
      LED5_Blink();
    else LED5(0);
    if(!(SystemState.OutLine_Flag&0x100))//MINI_S
      LED6_Blink();
    else LED6(0);
    
    osDelayUntil(&xLastWakeTime,300);
  }
}
/**
	**************************************************************
	** Descriptions:自检任务，任务正常运行则常亮，若闪烁则闪烁次数为不正常任务ID
	** Input: 	
  **						
	**					
	**					
	** Output: NULL
	**************************************************************
**/
void Check_Task(void const * argument)
{
	osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
    while(0)
    {
      OFF:LED8(0);
      osDelayUntil(&xLastWakeTime,2000);
    }
    if((SystemState.task_OutLine_Flag&0x01))//测试任务
    {
      BLINK();
      goto OFF;
    }
    else if((SystemState.task_OutLine_Flag&0x02))//底盘任务
    {
      BLINK();
      BLINK();
      goto OFF;
    }
    else if((SystemState.task_OutLine_Flag&0x04))//通信任务
    {
      BLINK();
      BLINK();
      BLINK();
      goto OFF;
    }
    else if((SystemState.task_OutLine_Flag&0x08))//掉线任务
    {
      BLINK();
      BLINK();
      BLINK();
      BLINK();
      goto OFF;
    }
    else 
    {
      LED8_Blink();
      osDelayUntil(&xLastWakeTime,16);
    }
    
  }
	
	
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/