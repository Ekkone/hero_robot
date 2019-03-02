#ifndef __MOTOR_USE_CAN_H
#define __MOTOR_USE_CAN_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "can.h"

#define FILTER_BUF_LEN		5

typedef enum
{
	//add by langgo
	CAN_3510Moto_ALL_ID = 0x200,
	CAN_3510Moto1_ID = 0x201,
	CAN_3510Moto2_ID = 0x202,
	CAN_3510Moto3_ID = 0x203,
	CAN_3510Moto4_ID = 0x204,
		
}CAN_Message_ID;

typedef struct{
	int16_t	 			speed_rpm;
	int16_t  			real_current;
	int16_t  			given_current;
	uint8_t  			hall;
	uint16_t 			angle;				//abs angle range:[0,8191]
	uint16_t 			last_angle;	//abs angle range:[0,8191]
	uint16_t			offset_angle;
	int32_t				round_cnt;
	int32_t				total_angle;
	uint8_t				buf_idx;
	uint16_t			angle_buf[FILTER_BUF_LEN];
	uint16_t			fited_angle;	
	uint32_t			msg_cnt;
}moto_measure_t;


extern moto_measure_t   moto_chassis_get[];
extern moto_measure_t   moto_dial_get;   //					_×¢ÊÍ
extern moto_measure_t   pit_get;
extern moto_measure_t   yaw_get;


void Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch);
void Chassis_Motor( CAN_HandleTypeDef * hcan,int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void Allocate_Motor(CAN_HandleTypeDef * hcan,int16_t value);
void get_moto_measure_3508(moto_measure_t *ptr,CAN_HandleTypeDef * hcan);
void get_moto_measure_6623(moto_measure_t *ptr,CAN_HandleTypeDef * hcan);
void get_moto_offset(moto_measure_t *ptr,CAN_HandleTypeDef * hcan);
void get_total_angle(moto_measure_t *p);
void Cloud_Platform_Motor_jiaozhun(CAN_HandleTypeDef * hcan);
void Cloud_Platform_Motor_Disable(CAN_HandleTypeDef * hcan);
void Chassis_Motor_Disable( CAN_HandleTypeDef * hcan);


#endif
