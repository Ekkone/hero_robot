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

extern DMA_HandleTypeDef  hdma_usart1_rx;
extern DMA_HandleTypeDef  hdma_usart3_rx;
extern DMA_HandleTypeDef  hdma_usart2_rx;
extern DMA_HandleTypeDef  hdma_usart4_rx;
extern DMA_HandleTypeDef  hdma_usart5_rx;
extern DMA_HandleTypeDef  hdma_usart6_rx;
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
	/*引脚*/
  MX_GPIO_Init();
  MX_DMA_Init();
  /*CAN*/
  MX_CAN2_Init();
  MX_CAN1_Init();
  /*串口*/
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  /*ADC*/
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /*定时器*/
  MX_TIM4_Init();
  MX_TIM2_Init();
  /*开启  */
  HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);

  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)uhADC1ConvertedValue, 10);//底盘总电流adc数据
  HAL_ADC_Start_DMA(&hadc2,(uint32_t*)uhADC2ConvertedValue, 10);//电容总电压Hadc数据
  HAL_ADC_Start_DMA(&hadc3,(uint32_t*)uhADC3ConvertedValue, 10);//电容总电压Ladc数据
  __HAL_DMA_DISABLE_IT(&hdma_adc1,DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_FE | DMA_IT_DME);
  __HAL_DMA_DISABLE_IT(&hdma_adc2,DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_FE | DMA_IT_DME);
  __HAL_DMA_DISABLE_IT(&hdma_adc3,DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_FE | DMA_IT_DME);
	/*开启中断*/ 
  HAL_UART_Receive_DMA(&huart3,USART3_RX_DATA,SizeofReferee);  	
  HAL_UART_Receive_DMA(&huart4,UART4_RX_DATA,SizeofMinipc); 				 	     
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);//
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);//
 
	/*关闭半传输完成中断*/
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);	//关闭串口3半传输完成中断
	__HAL_DMA_DISABLE_IT(&hdma_usart4_rx,DMA_IT_HT);	//关闭串口4半传输完成中断
  
	CanFilter_Init(&hcan1);
	CanFilter_Init(&hcan2);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_Delay(1000);

}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/




