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
/* Internal variables --------------------------------------------------------------*/
/* Private function prototypes ---------------------------------------------------*/
/**
	**************************************************************
	** Descriptions:	新板子电源初始化
	** Input:	huart  
  **						
	**					
	**					
	** Output: NULL
	**************************************************************
**/
void Power_Init(void)
{
  #if BoardNew

  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET);   //power1
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_SET);   //power2
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET);   //power3
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);   //power4

  #endif
	HAL_Delay(50);
}

/**
	**************************************************************
	** Descriptions:时间统计
	** Input: 	
  **						
	**					
	**					
	** Output: NULL
	**************************************************************
**/
void ConfigureTimerForRunTimeStats(void)  
{
	FreeRTOSRunTimeTicks = 0;
	MX_TIM3_Init(); //周期50us，频率20K
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
	Power_Init();
	/*dma*/
  MX_DMA_Init();
	/*can*/
	MX_CAN1_Init();
	MX_CAN2_Init();	
	CanFilter_Init(&hcan1);
	CanFilter_Init(&hcan2);
	SystemState_Inite();
  /*ADC*/
	MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
	/*串口*/
	MX_USART2_UART_Init();;
  MX_USART6_UART_Init();
  MX_UART8_Init();
	/*SPI*/
	MX_SPI5_Init();

	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
  __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);
	
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)uhADC1ConvertedValue, 10);//底盘总电流adc数据
  HAL_ADC_Start_DMA(&hadc2,(uint32_t*)uhADC2ConvertedValue, 10);//电容总电压Hadc数据
  HAL_ADC_Start_DMA(&hadc3,(uint32_t*)uhADC3ConvertedValue, 10);//电容总电压Ladc数据
  __HAL_DMA_DISABLE_IT(&hdma_adc1,DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_FE | DMA_IT_DME);
  __HAL_DMA_DISABLE_IT(&hdma_adc2,DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_FE | DMA_IT_DME);
  __HAL_DMA_DISABLE_IT(&hdma_adc3,DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_FE | DMA_IT_DME);
	/*使能DMA中断*/
  HAL_UART_Receive_DMA(&huart6,USART6_RX_DATA,SizeofReferee);
  HAL_UART_Receive_DMA(&huart8,UART8_RX_DATA,SizeofMinipc);
	/*使能can中断*/
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0); 
  HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
	HAL_Delay(1000);

}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/




