/*************************************************************************************
*	@file			BSP.c
* @author	 	
*	@version 	V1.0
*	@date			
* @brief		NONE
*************************************************************************************//* Includes ------------------------------------------------------------------------*/
#include "BSP.h"
/* External variables --------------------------------------------------------------*/
volatile unsigned long long FreeRTOSRunTimeTicks;
extern CAN_HandleTypeDef hcan1;
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int count=28;
uint8_t kkk[10]="asdfghjk";
uint8_t usart_date[32];
uint32_t value_ad;
/* Internal variables --------------------------------------------------------------*/
/* Private function prototypes ---------------------------------------------------*/
void ConfigureTimerForRunTimeStats(void)  
{
	FreeRTOSRunTimeTicks = 0;
	//MX_TIM3_Init(); //周期50us，频率20K
}

/**
	**************************************************************
	** Descriptions:初始化
	** Input: 	
  **						
	**					
	**					
	** Output: NULL
	**************************************************************
**/
void BSP_Init(void)
{
	
	/*引脚和引脚时钟*/
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN2_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);
	 
	 HAL_ADC_Start(&hadc1);
//	 HAL_ADC_Start(&hadc2);
//	 HAL_ADC_Start(&hadc3);
	 
	CanFilter_Init(&hcan1);
	CanFilter_Init(&hcan2);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_UART_Receive_IT(&huart1,usart_date,10);
	HAL_Delay(1000);

}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/




