#ifndef __minipc_H
#define __minipc_H
#include "stm32f4xx.h"
#include "main.h"
#include "communication.h "
#include "SystemState.h"

typedef struct{

unsigned char 		frame_header; 		  //֡ͷ0xFD
 int16_t 					angle_yaw;     			//yaw angle
 int16_t 					angle_pit;     			//pitch angle 
unsigned char 		state_flag;     		//��ǰ״̬����0 δ��׼Ŀ�� ���� 1 ����������Ŀ�꡿��2 Զ��������Ŀ�꡿��3 ���ģʽ��
unsigned char 		frame_tail; 	  	  //֡β0xFC
}Minipc_Rx;

typedef struct{

unsigned char 		frame_header; 		  //֡ͷ0xFD
unsigned char 		cmd1;     					//cmd1
unsigned char 		cmd2;     					//cmd2 
unsigned char 		frame_tail; 	  	  //֡β0xFC
}Minipc_Tx;


extern Minipc_Rx minipc_rx_big;
extern Minipc_Rx minipc_rx_small;

extern uint8_t USART4_RX_DATA[(SizeofMinipc)];		//MiniPC
extern uint16_t USART4_RX_NUM;

void Get_MiniPC_Handle(void);
void Send_MiniPC_Data(unsigned char cmd1,unsigned char cmd2,unsigned char state);


#endif
