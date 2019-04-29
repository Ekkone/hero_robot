/*******************************************************************************
                      ��Ȩ���� (C), 2017-,NCUROBOT
 *******************************************************************************
  �� �� ��   : minipc.c
  �� �� ��   : ����
  ��    ��   : NCUERM
  ��������   : 2018��7��
  ����޸�   :
  ��������   : ���ֽ���
  �����б�   :void Get_MiniPC_Data(void)
							void Send_MiniPC_Data(unsigned char cmd1,
																		unsigned char cmd2,
																		unsigned char state)
*******************************************************************************/

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "minipc.h"
/* �ڲ��Զ����������� --------------------------------------------------------*/

/* �ڲ��궨�� ----------------------------------------------------------------*/

/* ���������Ϣ����-----------------------------------------------------------*/

/* �ڲ���������---------------------------------------------------------------*/

/* �ⲿ�������� --------------------------------------------------------------*/

/* ���õ��ⲿ����ԭ������ ----------------------------------------------------------
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, 
																		uint8_t *pData, uint16_t Size, uint32_t Timeout);
*/
/* �ڲ����� ------------------------------------------------------------------*/
Minipc_Rx minipc_rx_big;
Minipc_Rx minipc_rx_small;
//Minipc_Tx minipc_tx;
/* ����ԭ������ ----------------------------------------------------------*/

/**
	**************************************************************
	** Descriptions: ��ȡ����minipc������
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
		minipc_rx_big.state_flag = buff[5];//0��Ŀ�꣬1��Ŀ�꣬2���٣�3���٣�4����
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

