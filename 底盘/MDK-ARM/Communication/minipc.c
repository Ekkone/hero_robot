/*******************************************************************************
                      版权所有 (C), 2017-,NCUROBOT
 *******************************************************************************
  文 件 名   : minipc.c
  版 本 号   : 初稿
  作    者   : NCUERM
  生成日期   : 2018年7月
  最近修改   :
  功能描述   : 麦轮结算
  函数列表   :void Get_MiniPC_Data(void)
							void Send_MiniPC_Data(unsigned char cmd1,
																		unsigned char cmd2,
																		unsigned char state)
*******************************************************************************/

/* 包含头文件 ----------------------------------------------------------------*/
#include "minipc.h"
/* 内部自定义数据类型 --------------------------------------------------------*/

/* 内部宏定义 ----------------------------------------------------------------*/

/* 任务相关信息定义-----------------------------------------------------------*/

/* 内部常量定义---------------------------------------------------------------*/

/* 外部变量声明 --------------------------------------------------------------*/

/* 调用的外部函数原型声明 ----------------------------------------------------------
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, 
																		uint8_t *pData, uint16_t Size, uint32_t Timeout);
*/
/* 内部变量 ------------------------------------------------------------------*/
Minipc_Rx minipc_rx_big;
Minipc_Rx minipc_rx_small;
//Minipc_Tx minipc_tx;
/* 函数原型声明 ----------------------------------------------------------*/

/**
	**************************************************************
	** Descriptions: 获取来自minipc的数据
	** Input: NULL
	**			  
	** Output: NULL
	**************************************************************
**/
void Get_MiniPC_Data_Big(void)
{
	uint8_t *buff = UART4_RX_DATA;
		
	minipc_rx_big.frame_header = buff[0];
	minipc_rx_big.frame_tail 	 = buff[8];
	if((minipc_rx_big.frame_header == 0xFD) && (minipc_rx_big.frame_tail == 0xFC))
	{
		minipc_rx_big.angle_yaw  = (int16_t)(buff[1]<<8|buff[2]);
		minipc_rx_big.angle_pit  = (int16_t)(buff[3]<<8|buff[4]);
		minipc_rx_big.state_flag = buff[5];//0无目标，1有目标，2低速，3中速，4高速
	}
}
void Get_MiniPC_Data_Small(void)
{
	uint8_t *buff = UART5_RX_DATA;
		
	minipc_rx_small.frame_header = buff[0];
	minipc_rx_small.frame_tail 	 = buff[8];
	if((minipc_rx_small.frame_header == 0xFF) && (minipc_rx_small.frame_tail == 0xFE))
	{
		minipc_rx_small.angle_yaw  = (int16_t)(buff[1]<<8|buff[2]);
		minipc_rx_small.angle_pit  = (int16_t)(buff[3]<<8|buff[4]);
		minipc_rx_small.state_flag = buff[5];
	}
}
//void Send_MiniPC_Data(unsigned char cmd1,unsigned char cmd2,unsigned char state)
//{
//	minipc_tx.frame_header = 0xFF;
//	minipc_tx.cmd1 			   = cmd1;
//	minipc_tx.cmd1 				 = cmd2;
//	minipc_tx.frame_tail   = 0xFE;
//	
//	HAL_UART_Transmit(&huart2,(uint8_t *)&minipc_tx,sizeof(minipc_tx),10);

//}

