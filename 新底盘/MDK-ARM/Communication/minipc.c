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
#include "can.h"
#include "Motor_USE_CAN.h"
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
Minipc_Tx minipc_tx;
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
		minipc_rx_big.state_flag = buff[5];//0��Ŀ�꣬1��Ŀ�꣬2���٣�3���٣�4����       
      
	}
  else if((frame_header == 0xFF) && (frame_tail == 0xFE))
	{
    RefreshDeviceOutLineTime(MINI_NO);
		minipc_rx_small.angle_yaw  = (int16_t)(buff[1]<<8|buff[2]);
		minipc_rx_small.angle_pit  = (int16_t)(buff[3]<<8|buff[4]);
		minipc_rx_small.state_flag = buff[5];//0��Ŀ�꣬1��Ŀ�꣬2���٣�3���٣�4����
	}
}
void Send_MiniPC_Data(uint8_t gun,uint8_t camera)
{
	minipc_tx.frame_header = 0xFF;
	minipc_tx.cmd1 			   = camera << 2 | gun;//0:�رգ�1:����,1:Сǹ�ڣ�2:��ǹ��
	minipc_tx.frame_tail   = 0xFE;
	
	HAL_UART_Transmit(&huart8,(uint8_t *)&minipc_tx,sizeof(minipc_tx),10);

}

