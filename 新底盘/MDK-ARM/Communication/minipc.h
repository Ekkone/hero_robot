#ifndef __minipc_H
#define __minipc_H
#include "stm32f4xx.h"
#include "main.h"
#include "communication.h "
#include "SystemState.h"

typedef struct{
 int16_t 					angle_yaw;     			//yaw angle
 int16_t 					angle_pit;     			//pitch angle 
unsigned char 		state_flag;     		//当前状态：【0 未瞄准目标 】【 1 近距离锁定目标】【2 远距离锁定目标】【3 打符模式】
}Minipc_Rx;

typedef struct{

unsigned char 		frame_header; 		  //帧头0xFD
unsigned char 		cmd1;     					//cmd1
unsigned char 		frame_tail; 	  	  //帧尾0xFC
}Minipc_Tx;


extern Minipc_Rx minipc_rx_big;
extern Minipc_Rx minipc_rx_small;

void Get_MiniPC_Handle(void);
void Send_MiniPC_Data(uint8_t gun,uint8_t camera);



#endif
