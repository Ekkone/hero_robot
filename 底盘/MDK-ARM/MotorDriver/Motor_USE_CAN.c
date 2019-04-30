/*******************************************************************************
*                     版权所有 (C), 2017-,NCUROBOT
********************************************************************************
* 文 件 名   : Motor_USE_CAN.c
* 版 本 号   : 初稿
* 作    者   : NCURM
* 生成日期   : 2018年7月
* 最近修改   :
* 功能描述   : 电机库模块中使用CAN进行控制的电机
* 函数列表   :
*使用CAN通讯的电机：云台电机   		 底盘电机	 	 	  拨弹电机
*				 	对应型号： c620						3508					 C2000
*接口函数：
*					Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch)
*					Chassis_Motor( CAN_HandleTypeDef * hcan,
*								  int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
*******************************************************************************/
/* 包含头文件 ----------------------------------------------------------------*/
#include "Motor_USE_CAN.h"
#include "communication.h "
#include "protocol.h"
#include "minipc.h"
/* 内部自定义数据类型 --------------------------------------------------------*/

/* 内部宏定义 ----------------------------------------------------------------*/

/* 任务相关信息定义-----------------------------------------------------------*/

/* 内部常量定义---------------------------------------------------------------*/

/* 外部变量声明 --------------------------------------------------------------*/
/*******************摩擦轮电机和底盘电机的参数变量***************************/
moto_measure_t   moto_chassis_get[4] = {0};//4 个 3508
moto_measure_t   moto_stir_get = {0};  //3508
moto_measure_t   moto_dial_get = {0};  //c2006
moto_measure_t   pit_get;
moto_measure_t   yaw_get;
/* 外部函数原型声明 ----------------------------------------------------------*/

/* 内部变量 ------------------------------------------------------------------*/
//为can发送分别创建缓存，防止串口发送的时候因只有一段内存而相互覆盖
static CAN_TxHeaderTypeDef  Cloud_Platform_Data;
static CAN_TxHeaderTypeDef  Allocate_Motor_Data;
static CAN_TxHeaderTypeDef  CANSend_DP;
static CAN_TxHeaderTypeDef  CANSend_Referee_B;
static CAN_TxHeaderTypeDef  CANSend_MINI_B;
static CAN_TxHeaderTypeDef  CANSend_Referee_S;
static CAN_TxHeaderTypeDef  CANSend_MINI_S;

static CAN_TxHeaderTypeDef	 Chassis_Motor_Header;
static CAN_TxHeaderTypeDef	 Stir_Motor_Header;

extern uint8_t rx_date[8];
uint8_t tx_date[8];
/* 函数原型声明 ----------------------------------------------------------*/
extern RC_Ctl_t RC_Ctl;
extern uint8_t chassis_gimble_Mode_flg;
uint8_t stir_motor_flag = 0;
int16_t yaw_speed;
int16_t OutLine_Flag;
int16_t task_OutLine_Flag;
int16_t R_key_v;
int16_t R_rc_ch0;
int16_t R_rc_ch1;
uint8_t R_rc_s1;
uint8_t R_rc_s2;
/**
	**************************************************************
	** Descriptions: 底盘电机驱动函数
	** Input: 	
	**			   hcan:要使用的CAN2
	**					iqn:第n个底盘电机的电流值
	** Output: NULL
	**************************************************************
**/
void Chassis_Motor( CAN_HandleTypeDef * hcan,int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
			Chassis_Motor_Header.DLC = 0x08;
			Chassis_Motor_Header.IDE = CAN_ID_STD;
			Chassis_Motor_Header.RTR = CAN_RTR_DATA;
			Chassis_Motor_Header.StdId = 0x200;

			tx_date[0] = iq1>>8;
	    tx_date[1] = iq1;
	    tx_date[2] = iq2>>8;
      tx_date[3] = iq2;
	    tx_date[4] = iq3>>8;
	    tx_date[5] = iq3;
	    tx_date[6] = iq4>>8;
	    tx_date[7] = iq4;
			HAL_CAN_AddTxMessage(hcan, &Chassis_Motor_Header, tx_date, (uint32_t *)CAN_TX_MAILBOX0 );
}	

/**
	**************************************************************
	** Descriptions: 底盘电机失能函数
	** Input: 	
	**			   hcan:要使用的CAN2
	**					iqn:第n个底盘电机的电流值
	** Output: NULL
	**************************************************************
**/
void Chassis_Motor_Disable( CAN_HandleTypeDef * hcan)
{
      Chassis_Motor_Header.DLC = 0x08;
			Chassis_Motor_Header.IDE = CAN_ID_STD;
			Chassis_Motor_Header.RTR = CAN_RTR_DATA;
			Chassis_Motor_Header.StdId = 0x200;

			tx_date[0] = 0x00;
	    tx_date[1] = 0x00;
	    tx_date[2] = 0x00;
      tx_date[3] = 0x00;
	    tx_date[4] = 0x00;
	    tx_date[5] = 0x00;
	    tx_date[6] = 0x00;
	    tx_date[7] = 0x00;
			HAL_CAN_AddTxMessage(hcan, &Chassis_Motor_Header, tx_date, (uint32_t *)CAN_TX_MAILBOX0 );
}	
/**
	**************************************************************
	** Descriptions: 搅拌电机驱动函数
	** Input: 	
	**			   hcan:要使用的CAN1
	**				value:搅拌电机的电流值
	** Output: NULL
	**************************************************************
**/
void Stir_Motor(CAN_HandleTypeDef * hcan,int16_t value)
{
      Stir_Motor_Header.DLC = 0x08;
			Stir_Motor_Header.IDE = CAN_ID_STD;
			Stir_Motor_Header.RTR = CAN_RTR_DATA;
			Stir_Motor_Header.StdId = 0x200;

      tx_date[0] = 0x00;
	    tx_date[1] = 0x00;
	    tx_date[2] = value>>8;
      tx_date[3] = value;
	    tx_date[4] = 0x00;
	    tx_date[5] = 0x00;
	    tx_date[6] = 0x00;
	    tx_date[7] = 0x00;
	
			HAL_CAN_AddTxMessage(hcan, &Stir_Motor_Header, tx_date, (uint32_t *)CAN_TX_MAILBOX0 );
}
/**                                                           //待续
	**************************************************************
	** Descriptions: 获取CAN通讯的3508电机的返回值
	** Input: 	
	**			  ptr:目标数据的内存地址
	**				hcan->pRxMsg->Data:保存的来自CAN的数据的数组
	** Output: NULL
	**************************************************************
**/
void get_moto_measure_3508(moto_measure_t *ptr,CAN_HandleTypeDef * hcan)
{
	/*BUG!!! dont use this para code*/

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(rx_date[0]<<8 | rx_date[1]) ;
	ptr->speed_rpm  = (int16_t)(rx_date[2]<<8 | rx_date[3]);
	ptr->real_current = (int16_t)(rx_date[4]<<8 | rx_date[5]);
	ptr->hall = rx_date[6];
	
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}
/**
	**************************************************************
	** Descriptions:获取电机返回值的偏差值
	** Input: 	
	**			  ptr:目标数据的内存地址
	**				hcan->pRxMsg->Data:保存的来自CAN的数据的数组
	** Output: NULL
	**************************************************************
**/
/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr,CAN_HandleTypeDef * hcan)
{
	ptr->angle = (uint16_t)(rx_date[0]<<8 |rx_date[1]) ;
	ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
/**
	**************************************************************
	** Descriptions: 获取电机的总角度值
	** Input: 	
	**			   *P:需要获取总角度值的地址
	**				
	** Output: NULL
	**************************************************************
**/
void get_total_angle(moto_measure_t *p){
	
	int res1, res2, delta;
	if(p->angle < p->last_angle){			//?????
		res1 = p->angle + 8192 - p->last_angle;	//??,delta=+
		res2 = p->angle - p->last_angle;				//??	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//??	delta -
		res2 = p->angle - p->last_angle;				//??	delta +
	}
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}

//接收，大云台
void CAN_RX_YT(CAN_HandleTypeDef * hcan)
{
	yaw_get.total_angle = (int16_t)(rx_date[0]<<8 |rx_date[1]) ;
	yaw_speed = (int16_t) (rx_date[2]<<8 | rx_date[3]) ;
	chassis_gimble_Mode_flg = (int16_t)(rx_date[4]<<8);
	stir_motor_flag = (int16_t)(rx_date[5]<<8);
	
}
void CAN_RX_YK(CAN_HandleTypeDef * hcan)
{
 RC_Ctl.key.v=(int16_t)(rx_date[0]<<8 |rx_date[1]) ;
 RC_Ctl.rc.ch0=(int16_t) (rx_date[2]<<8 |rx_date[3]) ;
 RC_Ctl.rc.ch1=(int16_t) (rx_date[4]<<8 |rx_date[5]) ;
 RC_Ctl.rc.s1=rx_date[6];
 RC_Ctl.rc.s2=rx_date[7];
	
}
void CAN_RX_ERROR(CAN_HandleTypeDef * hcan)
{
	OutLine_Flag = (int16_t)(rx_date[0]<<8 |rx_date[1]);
	task_OutLine_Flag = (int16_t) (rx_date[2]<<8 |rx_date[3]);
}
//发送，大云台
uint8_t TX_DATA_B[8];
void CAN_Send_chassis( CAN_HandleTypeDef * hcan,int16_t speed_rpm1, int16_t speed_rpm2, 
	int16_t speed_rpm3, int16_t speed_rpm4)
{
	    CANSend_DP.DLC = 0x08;
			CANSend_DP.IDE = CAN_ID_STD;
			CANSend_DP.RTR = CAN_RTR_DATA;
			CANSend_DP.StdId = CAN_Chassis;
  
      TX_DATA_B[0] = OutLine_Flag>>8;;
	    TX_DATA_B[1] = OutLine_Flag;
	    TX_DATA_B[2] = task_OutLine_Flag>>8;;
      TX_DATA_B[3] = task_OutLine_Flag;
	    TX_DATA_B[4] = 0x00;
	    TX_DATA_B[5] = 0x00;
	    TX_DATA_B[6] = 0x00;
	    TX_DATA_B[7] = 0x00;
	
			HAL_CAN_AddTxMessage(hcan, &CANSend_DP, TX_DATA_B, (uint32_t *)CAN_TX_MAILBOX0 );
}
uint8_t TX_REFEREE_B[9];
void CAN_Send_Referee_B( CAN_HandleTypeDef * hcan)
{
	    CANSend_Referee_B.DLC = 0x08;
			CANSend_Referee_B.IDE = CAN_ID_STD;
			CANSend_Referee_B.RTR = CAN_RTR_DATA;
			CANSend_Referee_B.StdId = 0x011;
  
      TX_REFEREE_B[0] = Robot.remainHp >> 8;
      TX_REFEREE_B[1] = Robot.remainHp;
      TX_REFEREE_B[2] = (uint16_t)Robot.heat.shoot_42_speed>>8;	//float型数据，扩大100倍后强制转换成uint16_t型
      TX_REFEREE_B[3] = (uint16_t)Robot.heat.shoot_42_speed;
      TX_REFEREE_B[4] = Robot.heat.shoot_42_heat>>8;	//42mm枪口热
      TX_REFEREE_B[5] = Robot.heat.shoot_42_heat;
      TX_REFEREE_B[6] = (uint16_t)Robot.heat.shoot_42_cooling_limit>>8;
      TX_REFEREE_B[7] = (uint16_t)Robot.heat.shoot_42_cooling_limit;


	
			HAL_CAN_AddTxMessage(hcan, &CANSend_Referee_B, TX_REFEREE_B, (uint32_t *)CAN_TX_MAILBOX0 );
}
uint8_t TX_DATA_MINI_B[8];
void CAN_Send_MINI_B( CAN_HandleTypeDef * hcan)
{
	    CANSend_MINI_B.DLC = 0x08;
			CANSend_MINI_B.IDE = CAN_ID_STD;
			CANSend_MINI_B.RTR = CAN_RTR_DATA;
			CANSend_MINI_B.StdId = CAN_Mini_B;
  
      TX_DATA_MINI_B[0] = minipc_rx_big.angle_yaw >> 8;
      TX_DATA_MINI_B[1] = minipc_rx_big.angle_yaw;
      TX_DATA_MINI_B[2] = minipc_rx_big.angle_pit>>8;
      TX_DATA_MINI_B[3] = minipc_rx_big.angle_pit;
      TX_DATA_MINI_B[4] = minipc_rx_big.state_flag;
      TX_DATA_MINI_B[5] = Robot.level;	
      TX_DATA_MINI_B[6] = 0;
      TX_DATA_MINI_B[7] = 0;
  
			HAL_CAN_AddTxMessage(hcan, &CANSend_MINI_B, TX_DATA_MINI_B, (uint32_t *)CAN_TX_MAILBOX0 );
}
//小云台裁判系统数据
uint8_t TX_DATA_S[8];
extern uint8_t communication_message;
void CAN_Send_Referee_S( CAN_HandleTypeDef * hcan)
{
	    CANSend_Referee_S.DLC = 0x08;
			CANSend_Referee_S.IDE = CAN_ID_STD;
			CANSend_Referee_S.RTR = CAN_RTR_DATA;
			CANSend_Referee_S.StdId = CAN_Referee_S;
  
      TX_DATA_S[0] = Robot.level;
      TX_DATA_S[1] = Robot.remainHp >> 8;
      TX_DATA_S[2] = Robot.remainHp;
      TX_DATA_S[3] = (uint16_t)Robot.heat.shoot_17_speed>>8;	//float型数据，扩大100倍后强制转换成uint16_t型
      TX_DATA_S[4] = (uint16_t)Robot.heat.shoot_17_speed;
      TX_DATA_S[5] = Robot.heat.shoot_17_heat>>8;	//17mm枪口热
      TX_DATA_S[6] = Robot.heat.shoot_17_heat;
      TX_DATA_S[7] = communication_message;
	
			HAL_CAN_AddTxMessage(hcan, &CANSend_Referee_S, TX_DATA_S, (uint32_t *)CAN_TX_MAILBOX0 );
}
//小云台视觉数据以及控制数据
uint8_t TX_DATA_MINI_S[8];

void CAN_Send_MINI_S( CAN_HandleTypeDef * hcan)
{
	    CANSend_MINI_S.DLC = 0x08;
			CANSend_MINI_S.IDE = CAN_ID_STD;
			CANSend_MINI_S.RTR = CAN_RTR_DATA;
			CANSend_MINI_S.StdId = CAN_Mini_S;
  
      TX_DATA_MINI_S[0] = minipc_rx_small.angle_yaw >> 8;
      TX_DATA_MINI_S[1] = minipc_rx_small.angle_yaw;
      TX_DATA_MINI_S[2] = minipc_rx_small.angle_pit>>8;
      TX_DATA_MINI_S[3] = minipc_rx_small.angle_pit;
      TX_DATA_MINI_S[4] = minipc_rx_small.state_flag;
      TX_DATA_MINI_S[5] = 0;	
      TX_DATA_MINI_S[6] = (uint16_t)Robot.heat.shoot_17_cooling_limit>>8;
      TX_DATA_MINI_S[7] = (uint16_t)Robot.heat.shoot_17_cooling_limit;

	
			HAL_CAN_AddTxMessage(hcan, &CANSend_MINI_S, TX_DATA_MINI_S, (uint32_t *)CAN_TX_MAILBOX0 );
}