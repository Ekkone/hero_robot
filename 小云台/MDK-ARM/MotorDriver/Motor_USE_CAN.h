/*************************************************************************************
*	@file			Motor_USE_CAN.h
* @author	 	
*	@version 	V1.0
*	@date			
* @brief		NONE
*************************************************************************************/
#ifndef __MOTOR_USE_CAN_H
#define __MOTOR_USE_CAN_H

/* Includes ------------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "can.h"
/* Exported macro ------------------------------------------------------------*/
#define FILTER_BUF_LEN		5
/* Exported types --------------------------------------------------------*/
typedef enum
{
  /*CAN1*/
  CAN_3508_STIR = 0X203,
  CAN_2006_B = 0X201,
  CAN_GM3510_PIT = 0X206,
  CAN_GM6020_YAW = 0X205,
  /*CAN2*/
	/*µ×ÅÌ->Ð¡ÔÆÌ¨*/
  CAN_RX_S = 0x013
  
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
  int32_t       total_ture_angle;
	uint8_t				buf_idx;
	uint16_t			angle_buf[FILTER_BUF_LEN];
	uint16_t			fited_angle;	
	uint32_t			msg_cnt;
  int32_t      run_time;
	int32_t      cmd_time;
	int32_t      reverse_time;
  int32_t      REVE_time;
}moto_measure_t;

/* Exported constants------------------------------------------------------------*/
extern moto_measure_t   moto_dial_get; 
extern moto_measure_t   moto_stir_get;
extern moto_measure_t   pit_get;
extern moto_measure_t   yaw_get;
/* Internal functions ------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch);
void Cloud_Platform_Motor_Correct(CAN_HandleTypeDef * hcan);
void Cloud_Platform_Motor_Disable(CAN_HandleTypeDef * hcan);

void Shot_Motor(CAN_HandleTypeDef * hcan,int16_t bo_value,int16_t stir_value);
void get_moto_measure_GM6020(moto_measure_t *ptr,CAN_HandleTypeDef * hcan);
void get_moto_measure_GM3510(moto_measure_t *ptr,CAN_HandleTypeDef * hcan);
void get_moto_measure_3508(moto_measure_t *ptr,CAN_HandleTypeDef * hcan);
void get_moto_offset(moto_measure_t *ptr,CAN_HandleTypeDef * hcan);
void get_total_angle(moto_measure_t *p);

void CAN_Receive_S( CAN_HandleTypeDef * hcan);
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

