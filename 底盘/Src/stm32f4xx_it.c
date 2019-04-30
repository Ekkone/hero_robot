/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "Motor_USE_CAN.h"
#include "SystemState.h"
#include "FreeRTOS.h"
#include "dma.h"
#include "usart.h"
#include "communication.h "
/* USER CODE BEGIN 0 */
uint8_t rx_date[8];
int can_count=0;
int can_count1=0;

volatile uint8_t RemoteData_flag = 0;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
CAN_RxHeaderTypeDef  Rx_Header;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim1;

//DMA
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart4_rx;
extern DMA_HandleTypeDef hdma_usart5_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;

extern osThreadId RefereeTaskHandle;
extern osThreadId MINIPCBIGTaskHandle;
extern osThreadId MINIPCSMATaskHandle;

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
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
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
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
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
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
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
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
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
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line1 interrupt.
*/
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
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
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
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

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

//定时器6中断服务函数
void   TIM6_DAC_IRQHandler(void)
{
	    HAL_TIM_IRQHandler(&htim6);
}

/**
* @brief This function handles DMA2 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
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

/* USER CODE BEGIN 1 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
	
	
	
}
void DMA2_Stream2_IRQHandler(void)
{
	/* USER CODE BEGIN DMA2_Stream2_IRQn 0 */
	//__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_TC);
  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}
/**
* @brief This function handles DMA1 stream1 global interrupt.
*/
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */
	//__HAL_DMA_DISABLE_IT(&hdma_usart5_rx,DMA_IT_TC);
  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart5_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}
/**
* @brief This function handles DMA1 stream1 global interrupt.
*/
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */
	//__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_TC);
  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}
/**
* @brief This function handles DMA1 stream1 global interrupt.
*/
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

	//__HAL_DMA_DISABLE_IT(&hdma_usart4_rx,DMA_IT_TC);
  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart4_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}
/**
* @brief This function handles DMA1 stream1 global interrupt.
*/
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */
	//__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_TC);
  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

void USART3_IRQHandler(void)
{
  static  BaseType_t  pxHigherPriorityTaskWoken;
  uint8_t tmp1,tmp2;
    tmp1 = __HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE);   //空闲中断中将已收字节数取出后，停止DMA
    tmp2 = __HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_IDLE);
    if((tmp1 != RESET)&&(tmp2 != RESET))
    {
      RefreshDeviceOutLineTime(Referee_NO);
      __HAL_DMA_DISABLE(&hdma_usart3_rx);
      __HAL_UART_CLEAR_IDLEFLAG(&huart3);
		
		USART3_RX_NUM=(SizeofReferee)-(hdma_usart3_rx.Instance->NDTR);
    __HAL_DMA_SET_COUNTER(&hdma_usart3_rx,SizeofReferee);
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
      
     vTaskNotifyGiveFromISR(RefereeTaskHandle,&pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);		
    }
}
void USART4_IRQHandler(void)
{
  static  BaseType_t  pxHigherPriorityTaskWoken;
  uint8_t tmp1,tmp2;
    tmp1 = __HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE);   //空闲中断中将已收字节数取出后，停止DMA
    tmp2 = __HAL_UART_GET_IT_SOURCE(&huart4, UART_IT_IDLE);
    if((tmp1 != RESET)&&(tmp2 != RESET))
    {
      RefreshDeviceOutLineTime(MINI_B_NO);
      __HAL_DMA_DISABLE(&hdma_usart4_rx);
      __HAL_UART_CLEAR_IDLEFLAG(&huart4);
		
		UART4_RX_NUM=(SizeofMinipc)-(hdma_usart4_rx.Instance->NDTR);
    __HAL_DMA_SET_COUNTER(&hdma_usart4_rx,SizeofMinipc);
    __HAL_DMA_ENABLE(&hdma_usart4_rx);
      
     vTaskNotifyGiveFromISR(MINIPCBIGTaskHandle,&pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);		
    }
}
void USART5_IRQHandler(void)
{
  static  BaseType_t  pxHigherPriorityTaskWoken;
  uint8_t tmp1,tmp2;
    tmp1 = __HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE);   //空闲中断中将已收字节数取出后，停止DMA
    tmp2 = __HAL_UART_GET_IT_SOURCE(&huart5, UART_IT_IDLE);
    if((tmp1 != RESET)&&(tmp2 != RESET))
    {
      RefreshDeviceOutLineTime(MINI_S_NO);
      __HAL_DMA_DISABLE(&hdma_usart5_rx);
      __HAL_UART_CLEAR_IDLEFLAG(&huart5);
		
		UART5_RX_NUM=(SizeofMinipc)-(hdma_usart5_rx.Instance->NDTR);
    __HAL_DMA_SET_COUNTER(&hdma_usart5_rx,SizeofMinipc);
    __HAL_DMA_ENABLE(&hdma_usart5_rx);
      
     vTaskNotifyGiveFromISR(MINIPCSMATaskHandle,&pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);		
    }
}
/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static  BaseType_t  pxHigherPriorityTaskWoken;
	
  if(huart->Instance == USART3)  //裁判系统
	{

	}
  else if(huart->Instance == UART4)  //MINIPC大枪管
  {

 }
  else if(huart->Instance == UART5)  //MINIPC小枪管
  {

  }
}




void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan == &hcan1)     
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,&Rx_Header,rx_date);
		switch(Rx_Header.StdId)
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
			case CAN_ERROR: 
			{
				RefreshDeviceOutLineTime(Remote_NO);
				CAN_RX_ERROR(&hcan1);
			}break;
			
      case CAN_3508_STIR:
      {
        RefreshDeviceOutLineTime(MotorS_NO);
        if(moto_stir_get.msg_cnt++ <= 50)	
				{
					get_moto_offset(&moto_stir_get,&hcan1);
				}
				else
				{		
					moto_stir_get.msg_cnt=51;	
					get_moto_measure_3508(&moto_stir_get, &hcan1);
				}
      }break;
			default: break;
		}
	}
		
//		if( HAL_BUSY == HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0))//开启中断接收
//		{
//			/* Enable FIFO 0 overrun and message pending Interrupt */
//			__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
//		}
	else
      if(hcan == &hcan2)  
	{
			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,&Rx_Header,rx_date);
		switch(Rx_Header.StdId)
		{
			case CAN_3510Moto1_ID:     RefreshDeviceOutLineTime(Motor1_NO);break;
			case CAN_3510Moto2_ID:     RefreshDeviceOutLineTime(Motor2_NO);break;
			case CAN_3510Moto3_ID:     RefreshDeviceOutLineTime(Motor3_NO);break;
			case CAN_3510Moto4_ID:     RefreshDeviceOutLineTime(Motor4_NO);break;
			
		}
    {
				static uint8_t i;
				i = Rx_Header.StdId - CAN_3510Moto1_ID;
				if(moto_chassis_get[i].msg_cnt++ <= 50)	
				{
					get_moto_offset(&moto_chassis_get[i],&hcan2);
				}
				else{		
					moto_chassis_get[i].msg_cnt=51;	
					get_moto_measure_3508(&moto_chassis_get[i], &hcan2);
				}
			}
//		if( HAL_BUSY == HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0))//开启中断接收
//		{
//			/* Enable FIFO 0 overrun and message pending Interrupt */
//			__HAL_CAN_ENABLE_IT(&hcan2,CAN_IT_FMP0);
//		}	
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) 
	{
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* USER CODE END Callback 1 */
 if(htim == (&htim6))
	{
		RefreshSysTime();
	}
/* USER CODE END 1 */
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
