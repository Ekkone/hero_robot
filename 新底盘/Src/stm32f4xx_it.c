/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"
#include "usart.h"
#include "Motor_USE_CAN.h"
#include "communication.h "
#include "tim.h"
#include "can.h"
#include "adc.h"
/* USER CODE BEGIN 0 */
#include "pidwireless.h"
#include "communication.h "
#include "protocol.h"
#include "minipc.h"
#include "SystemState.h"
/* USER CODE END 0 */
extern  osThreadId RemoteDataTaskHandle;

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim12;

extern DMA_HandleTypeDef hdma_adc1;

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;

//测速模块
volatile uint8_t RemoteData_flag = 0;
extern uint32_t Micro_Tick;
extern uint32_t Photoelectric_gate1,Photoelectric_gate2;
extern uint16_t gate1_counter,gate2_counter;
/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
//  HAL_IncTick();
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}


/**
* @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
*/
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);

  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

//定时器6中断服务函数
void   TIM6_DAC_IRQHandler(void)
{
	    HAL_TIM_IRQHandler(&htim6);
}
/**
* @brief This function handles TIM8 break interrupt and TIM12 global interrupt.
*/
void TIM8_BRK_TIM12_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 0 */
  HAL_TIM_IRQHandler(&htim12);
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
}
/**
* @brief This function handles EXTI line2 interrupt.
*/
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}
/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA2 stream4 global interrupt.
*/
void DMA2_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream4_IRQn 0 */

  /* USER CODE END DMA2_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream4_IRQn 1 */

  /* USER CODE END DMA2_Stream4_IRQn 1 */
}
/******************************************************************************/

/**
* @brief This function handles DMA2 stream4 global interrupt.
*/
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream4_IRQn 0 */

  /* USER CODE END DMA2_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc2);
  /* USER CODE BEGIN DMA2_Stream4_IRQn 1 */

  /* USER CODE END DMA2_Stream4_IRQn 1 */
}
/******************************************************************************/

/**
* @brief This function handles DMA2 stream4 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream4_IRQn 0 */

  /* USER CODE END DMA2_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA2_Stream4_IRQn 1 */

  /* USER CODE END DMA2_Stream4_IRQn 1 */
}
/**
* @brief This function handles DMA2 stream1 global interrupt.
*/
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */
	
  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}
/**
* @brief This function handles DMA2 stream1 global interrupt.
*/
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */
	
  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}
/**
* @brief This function handles DMA2 stream2 global interrupt.
*/
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart8_rx);
  /* USER CODE END DMA2_Stream2_IRQn 1 */
}
void USART2_IRQHandler (void)
{
	 static  BaseType_t  pxHigherPriorityTaskWoken;
  uint8_t tmp1,tmp2;
    tmp1 = __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE);   //空闲中断中将已收字节数取出后，停止DMA
    tmp2 = __HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_IDLE);
    if((tmp1 != RESET)&&(tmp2 != RESET))
    {
      __HAL_DMA_DISABLE(&hdma_usart2_rx);
      __HAL_UART_CLEAR_IDLEFLAG(&huart2);
		
		UART8_RX_NUM=(SizeofMinipc)-(hdma_usart2_rx.Instance->NDTR);
    Get_MiniPC_Handle();
    __HAL_DMA_SET_COUNTER(&hdma_usart2_rx,SizeofMinipc);
    __HAL_DMA_ENABLE(&hdma_usart2_rx);	
    }
  /* USER CODE END UART8_IRQn 1 */
}

void USART6_IRQHandler(void)
{
  static  BaseType_t  pxHigherPriorityTaskWoken;
  uint8_t tmp1,tmp2;
    tmp1 = __HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE);   //空闲中断中将已收字节数取出后，停止DMA
    tmp2 = __HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_IDLE);
    if((tmp1 != RESET)&&(tmp2 != RESET))
    {
      RefreshDeviceOutLineTime(Referee_NO);
      __HAL_DMA_DISABLE(&hdma_usart6_rx);
      __HAL_UART_CLEAR_IDLEFLAG(&huart6);
		
		USART6_RX_NUM=(SizeofReferee)-(hdma_usart6_rx.Instance->NDTR);
    Referee_Data_Handler();
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx,SizeofReferee);
    __HAL_DMA_ENABLE(&hdma_usart6_rx);	
    }
}
void UART8_IRQHandler(void)
{
  static  BaseType_t  pxHigherPriorityTaskWoken;
  uint8_t tmp1,tmp2;
    tmp1 = __HAL_UART_GET_FLAG(&huart8, UART_FLAG_IDLE);   //空闲中断中将已收字节数取出后，停止DMA
    tmp2 = __HAL_UART_GET_IT_SOURCE(&huart8, UART_IT_IDLE);
    if((tmp1 != RESET)&&(tmp2 != RESET))
    {
      __HAL_DMA_DISABLE(&hdma_uart8_rx);
      __HAL_UART_CLEAR_IDLEFLAG(&huart8);
		
		UART8_RX_NUM=(SizeofMinipc)-(hdma_uart8_rx.Instance->NDTR);
    Get_MiniPC_Handle();
    __HAL_DMA_SET_COUNTER(&hdma_uart8_rx,SizeofMinipc);
    __HAL_DMA_ENABLE(&hdma_uart8_rx);	
    }
}

/**
* @brief This function handles CAN1 RX0 interrupts.
*/
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);

  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
* @brief This function handles CAN2 RX0 interrupts.
*/
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}
extern volatile unsigned long long FreeRTOSRunTimeTicks;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) 
	{
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if (htim->Instance == TIM5) 
	{
		__HAL_TIM_ENABLE(&htim5);
		__HAL_TIM_ENABLE_IT(&htim5,TIM_IT_UPDATE);
  }
  /* USER CODE END Callback 1 */
	 else if(htim==(&htim3))
	{
		 FreeRTOSRunTimeTicks++;  //时间节拍计数器加一
	}	
	else if(htim == (&htim12))
	{
		Micro_Tick++;
	}
	else if(htim == (&htim6))
	{
		RefreshSysTime();
	}
}
/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //接收完成            暂时不加任务通知，后续讨论　　_待续
{

		if(huart == &huart1)
	{
  	
//		  __HAL_UART_CLEAR_OREFLAG(&huart1);
//			__HAL_DMA_SET_COUNTER(&hdma_usart1_rx,SizeofRemote);
//			__HAL_DMA_ENABLE(&hdma_usart1_rx);
//      HAL_UART_Receive_DMA(&huart1,USART1_RX_DATA,SizeofRemote); 
//      vTaskNotifyGiveFromISR(RemoteDataTaskHandle,&pxHigherPriorityTaskWoken);
//			portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);			


	}else if(huart == &huart2)
	{ 
/*
		__HAL_DMA_SET_COUNTER(&hdma_usart2_rx,SizeofMinipc);
			__HAL_DMA_ENABLE(&hdma_usart2_rx);
		  vTaskNotifyGiveFromISR(MiniPCDataTaskHandle,&pxHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);	
*/
	}else if(huart == &huart3)
	{
//			uint8_t Res = 0;
//			HAL_UART_Receive(&huart3, &Res, 1,1);//读取接收到的数据
//		  /*无线调参的处理函数*/
//			PID_UART_IRQHandler(&huart3, Res);
	}
	else if(huart == &huart6)
	{
//			__HAL_DMA_SET_COUNTER(&hdma_usart6_rx,SizeofReferee);
//			__HAL_DMA_ENABLE(&hdma_usart6_rx);
//			vTaskNotifyGiveFromISR(RefereeDataTaskHandle,&pxHigherPriorityTaskWoken);
//			portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);	
		
	}
	else if(huart == &huart8)
	{
		
	}
	
}
/**
	**************************************************************
	** Descriptions:CAN接收回调函数
	** Input: 	
  **						
	**					
	**					
	** Output: NULL
	**************************************************************
**/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan == &hcan1)
	{
		switch(hcan1.pRxMsg->StdId)
		{
			case CAN_YT: 
			{
				RefreshDeviceOutLineTime(Remote_NO);
				CAN_RX_YT(&hcan1);         //云台数据接收
			}break;
			
			case CAN_YK: 
			{
				RefreshDeviceOutLineTime(Remote_NO);
				CAN_RX_YK(&hcan1);         //遥控器数据接收
				RemoteData_flag = 1;       //数据任务
			}break;		
		}
		if( HAL_BUSY == HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0))//开启中断接收
		{
			/* Enable FIFO 0 overrun and message pending Interrupt */
			__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
		}
	}else if(hcan == &hcan2)
	{
//		HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);
		switch(hcan->pRxMsg->StdId)
		{
			case CAN_3510Moto1_ID:     RefreshDeviceOutLineTime(Motor1_NO);break;
			case CAN_3510Moto2_ID:     RefreshDeviceOutLineTime(Motor2_NO);break;
			case CAN_3510Moto3_ID:     RefreshDeviceOutLineTime(Motor3_NO);break;
			case CAN_3510Moto4_ID:     RefreshDeviceOutLineTime(Motor4_NO);break;
		}
    static uint8_t i;
				i = hcan->pRxMsg->StdId - CAN_3510Moto1_ID;
				if(moto_chassis_get[i].msg_cnt++ <= 50)	
				{
					get_moto_offset(&moto_chassis_get[i],&hcan2);
				}
				else{		
					moto_chassis_get[i].msg_cnt=51;	
					get_moto_measure_3508(&moto_chassis_get[i], &hcan2);
				}
		if( HAL_BUSY == HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0))//开启中断接收
		{
			/* Enable FIFO 0 overrun and message pending Interrupt */
			__HAL_CAN_ENABLE_IT(&hcan2,CAN_IT_FMP0);
		}	
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
