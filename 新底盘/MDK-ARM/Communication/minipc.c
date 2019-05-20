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
#include "can.h"
#include "Motor_USE_CAN.h"
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
Minipc_Tx minipc_tx;
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
void Get_MiniPC_Handle(void)
{
  uint8_t *buff = UART8_RX_DATA;
  uint8_t frame_header = buff[0];
	uint8_t frame_tail 	 = buff[6];
  RefreshDeviceOutLineTime(MINI_NO);
	if((frame_header == 0xFD) && (frame_tail == 0xFC))
	{
    RefreshDeviceOutLineTime(MINI_NO);
		minipc_rx_big.angle_yaw  = (int16_t)(buff[1]<<8|buff[2]);
		minipc_rx_big.angle_pit  = (int16_t)(buff[3]<<8|buff[4]);
		minipc_rx_big.state_flag = buff[5];//0无目标，1有目标，2低速，3中速，4高速       
      
	}
  else if((frame_header == 0xFF) && (frame_tail == 0xFE))
	{
    RefreshDeviceOutLineTime(MINI_NO);
		minipc_rx_small.angle_yaw  = (int16_t)(buff[1]<<8|buff[2]);
		minipc_rx_small.angle_pit  = (int16_t)(buff[3]<<8|buff[4]);
		minipc_rx_small.state_flag = buff[5];//0无目标，1有目标，2低速，3中速，4高速
	}
}
void Send_MiniPC_Data(uint8_t gun,uint8_t camera)
{
	minipc_tx.frame_header = 0xFF;
	minipc_tx.cmd1 			   = camera << 2 | gun;//0:关闭，1:开启,1:小枪口，2:大枪口
	minipc_tx.frame_tail   = 0xFE;
	
	HAL_UART_Transmit(&huart8,(uint8_t *)&minipc_tx,sizeof(minipc_tx),10);

}

