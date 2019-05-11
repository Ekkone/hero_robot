#ifndef __minipc_H
#define __minipc_H
#include "stm32f4xx.h"
#include "main.h"
#include "communication.h "
#include "SystemState.h"

typedef struct{

unsigned char 		frame_header; 		  //帧头0xFD
 int16_t 					angle_yaw;     			//yaw angle
 int16_t 					angle_pit;     			//pitch angle 
unsigned char 		state_flag;     		//当前状态：【0 未瞄准目标 】【 1 近距离锁定目标】【2 远距离锁定目标】【3 打符模式】
unsigned char 		frame_tail; 	  	  //帧尾0xFC
}Minipc_Rx;

typedef struct{

unsigned char 		frame_header; 		  //帧头0xFD
unsigned char 		cmd1;     					//cmd1
unsigned char 		cmd2;     					//cmd2 
unsigned char 		frame_tail; 	  	  //帧尾0xFC
}Minipc_Tx;


extern Minipc_Rx minipc_rx_big;
extern Minipc_Rx minipc_rx_small;

extern uint8_t USART4_RX_DATA[(SizeofMinipc)];		//MiniPC
extern uint16_t USART4_RX_NUM;

void Get_MiniPC_Handle(void);
void Send_MiniPC_Data(unsigned char cmd1,unsigned char cmd2,unsigned char state);


#endif
