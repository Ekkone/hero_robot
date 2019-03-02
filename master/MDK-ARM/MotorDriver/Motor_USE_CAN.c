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
/* 内部自定义数据类型 --------------------------------------------------------*/

/* 内部宏定义 ----------------------------------------------------------------*/

/* 任务相关信息定义-----------------------------------------------------------*/

/* 内部常量定义---------------------------------------------------------------*/

/* 外部变量声明 --------------------------------------------------------------*/
/*******************摩擦轮电机和底盘电机的参数变量***************************/
moto_measure_t   moto_chassis_get[4] = {0};//4 个 3508
moto_measure_t   moto_dial_get = {0};  //c2006
moto_measure_t   pit_get;
moto_measure_t   yaw_get;
/* 外部函数原型声明 ----------------------------------------------------------*/

/* 内部变量 ------------------------------------------------------------------*/
//为can发送分别创建缓存，防止串口发送的时候因只有一段内存而相互覆盖
static CanTxMsgTypeDef  Cloud_Platform_Data;
static CanTxMsgTypeDef	 Chassis_Motor_Data;
static CanTxMsgTypeDef  Allocate_Motor_Data;
/* 函数原型声明 ----------------------------------------------------------*/

/**
	**************************************************************
	** Descriptions: 云台电机驱动函数
	** Input: 	
	**			   hcan:要使用的CAN1
	**					yaw:yaw轴电流值
	**				pitch:pitch电流值
	** Output: NULL
	**************************************************************
**/
void Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch)
{
		Cloud_Platform_Data.StdId = 0x1FF;
		Cloud_Platform_Data.IDE = CAN_ID_STD;
		Cloud_Platform_Data.RTR = CAN_RTR_DATA;
		Cloud_Platform_Data.DLC = 0X08;
		
		Cloud_Platform_Data.Data[0] = yaw>>8;
		Cloud_Platform_Data.Data[1] = yaw;
		Cloud_Platform_Data.Data[2] = pitch>>8;
		Cloud_Platform_Data.Data[3] = pitch;
		Cloud_Platform_Data.Data[4] = 0x00;
		Cloud_Platform_Data.Data[5] = 0x00;
		Cloud_Platform_Data.Data[6] = 0x00;
		Cloud_Platform_Data.Data[7] = 0x00;

  	hcan->pTxMsg = &Cloud_Platform_Data;
		HAL_CAN_Transmit(hcan,0);
}

/**
	**************************************************************
	** Descriptions: 云台电机校准函数
	** Input: 	
	**			   hcan:要使用的CAN1
	**					
	**				
	** Output: NULL
	**************************************************************
**/
void Cloud_Platform_Motor_jiaozhun(CAN_HandleTypeDef * hcan)
{
	
		Cloud_Platform_Data.StdId = 0x3F0;
		Cloud_Platform_Data.IDE = CAN_ID_STD;
		Cloud_Platform_Data.RTR = CAN_RTR_DATA;
		Cloud_Platform_Data.DLC = 0X08;
		
		Cloud_Platform_Data.Data[0] = 'c' ;
		Cloud_Platform_Data.Data[1] = 0x00;
		Cloud_Platform_Data.Data[2] = 0x00;
		Cloud_Platform_Data.Data[3] = 0x00;
		Cloud_Platform_Data.Data[4] = 0x00;
		Cloud_Platform_Data.Data[5] = 0x00;
		Cloud_Platform_Data.Data[6] = 0x00;
		Cloud_Platform_Data.Data[7] = 0x00;

  	hcan->pTxMsg = &Cloud_Platform_Data;
		HAL_CAN_Transmit(hcan,10);
}

/**
	**************************************************************
	** Descriptions: 云台电机失能函数
	** Input: 	
	**			   hcan:要使用的CAN1
	**					
	**				
	** Output: NULL
	**************************************************************
**/
void Cloud_Platform_Motor_Disable(CAN_HandleTypeDef * hcan)
{
	
		Cloud_Platform_Data.StdId = 0x1FF;
		Cloud_Platform_Data.IDE = CAN_ID_STD;
		Cloud_Platform_Data.RTR = CAN_RTR_DATA;
		Cloud_Platform_Data.DLC = 0X08;
		
		Cloud_Platform_Data.Data[0] = 0x00;
		Cloud_Platform_Data.Data[1] = 0x00;
		Cloud_Platform_Data.Data[2] = 0x00;
		Cloud_Platform_Data.Data[3] = 0x00;
		Cloud_Platform_Data.Data[4] = 0x00;
		Cloud_Platform_Data.Data[5] = 0x00;
		Cloud_Platform_Data.Data[6] = 0x00;
		Cloud_Platform_Data.Data[7] = 0x00;

  	hcan->pTxMsg = &Cloud_Platform_Data;
		HAL_CAN_Transmit(hcan,10);
}

/**
	**************************************************************
	** Descriptions: 底盘电机驱动函数
	** Input: 	
	**			   hcan:要使用的CAN2
	**					iqn:第n个底盘电机的电流值
	** Output: NULL
	**************************************************************
**/
void Chassis_Motor( CAN_HandleTypeDef * hcan,
									  int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
			Chassis_Motor_Data.DLC = 0x08;
			Chassis_Motor_Data.IDE = CAN_ID_STD;
			Chassis_Motor_Data.RTR = CAN_RTR_DATA;
			Chassis_Motor_Data.StdId = 0x200;

			Chassis_Motor_Data.Data[0]=iq1>>8;
			Chassis_Motor_Data.Data[1]=iq1;
			Chassis_Motor_Data.Data[2]=iq2>>8;
			Chassis_Motor_Data.Data[3]=iq2;
			Chassis_Motor_Data.Data[4]=iq3>>8;
			Chassis_Motor_Data.Data[5]=iq3;
			Chassis_Motor_Data.Data[6]=iq4>>8;
			Chassis_Motor_Data.Data[7]=iq4;
	
			hcan->pTxMsg = &Chassis_Motor_Data;
			HAL_CAN_Transmit(hcan,0);
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
			Chassis_Motor_Data.DLC = 0x08;
			Chassis_Motor_Data.IDE = CAN_ID_STD;
			Chassis_Motor_Data.RTR = CAN_RTR_DATA;
			Chassis_Motor_Data.StdId = 0x200;

			Chassis_Motor_Data.Data[0]=0x00;
			Chassis_Motor_Data.Data[1]=0x00;
			Chassis_Motor_Data.Data[2]=0x00;
			Chassis_Motor_Data.Data[3]=0x00;
			Chassis_Motor_Data.Data[4]=0x00;
			Chassis_Motor_Data.Data[5]=0x00;
			Chassis_Motor_Data.Data[6]=0x00;
			Chassis_Motor_Data.Data[7]=0x00;
	
			hcan->pTxMsg = &Chassis_Motor_Data;
			HAL_CAN_Transmit(hcan,5);
}	
/**
	**************************************************************
	** Descriptions: 拨弹电机驱动函数
	** Input: 	
	**			   hcan:要使用的CAN1
	**				value:拨弹电机的电流值
	** Output: NULL
	**************************************************************
**/
void Allocate_Motor(CAN_HandleTypeDef * hcan,int16_t value)
{

			Allocate_Motor_Data.DLC = 0x08;
			Allocate_Motor_Data.IDE = CAN_ID_STD;
			Allocate_Motor_Data.RTR = CAN_RTR_DATA;
			Allocate_Motor_Data.StdId = 0x200;

			Allocate_Motor_Data.Data[0]=value>>8;
			Allocate_Motor_Data.Data[1]=value;
			Allocate_Motor_Data.Data[2]=0;
			Allocate_Motor_Data.Data[3]=0;
			Allocate_Motor_Data.Data[4]=0;
			Allocate_Motor_Data.Data[5]=0;
			Allocate_Motor_Data.Data[6]=0;
			Allocate_Motor_Data.Data[7]=0;
	
			hcan->pTxMsg = &Allocate_Motor_Data;
			HAL_CAN_Transmit(hcan,0);
}
/**                                                           //待续
	**************************************************************
	** Descriptions: 获取CAN通讯的6623电机的返回值
	** Input: 	
	**			  ptr:目标数据的内存地址
	**				hcan->pRxMsg->Data:保存的来自CAN的数据的数组
	** Output: NULL
	**************************************************************
**/
void get_moto_measure_6623(moto_measure_t *ptr,CAN_HandleTypeDef * hcan)
{
	/*BUG!!! dont use this para code*/

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	ptr->real_current  = (int16_t)(hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]);
	ptr->given_current = (int16_t)(hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5]);
	ptr->speed_rpm = ptr->real_current;
//	ptr->hall = hcan->pRxMsg->Data[6];
	
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
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
	ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	ptr->speed_rpm  = (int16_t)(hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]);
	ptr->real_current = (int16_t)(hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5]);
	ptr->hall = hcan->pRxMsg->Data[6];
	
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
	ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 |hcan->pRxMsg->Data[1]) ;
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
