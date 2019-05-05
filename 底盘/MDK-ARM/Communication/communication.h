#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "string.h"


#define SizeofReferee 200
#define SizeofMinipc  7
extern uint8_t USART3_RX_DATA[(SizeofReferee)];//����ϵͳ
extern uint16_t USART3_RX_NUM;
extern uint8_t UART4_RX_DATA[(SizeofMinipc)];//���PC����̨
extern uint16_t UART4_RX_NUM;
extern uint8_t UART5_RX_DATA[(SizeofMinipc)];//���PCС��̨
extern uint16_t UART5_RX_NUM;

/* ��ģ�����ⲿ�ṩ���������Ͷ��� --------------------------------------------*/

///////////////ң��/////////////////////
typedef struct //ң����������ͨ��
		{ 
			int16_t x; //!< Byte 6-7 
			int16_t y; //!< Byte 8-9 
			int16_t z; //!< Byte 10-11 
			uint8_t press_l; //!< Byte 12 
			uint8_t press_r; //!< Byte 13 
    }Mouse; 
	typedef 	struct 
		{ 
	 uint16_t ch0; 
	 uint16_t ch1; 
	 uint16_t ch2; 
	 uint16_t ch3; 
	 uint8_t s1; 
	 uint8_t s2; 
		}Rc; 
	typedef struct 
		{ 
		  uint16_t v; //!< Byte 14-15 
		}Key; 
		typedef struct 
{ 
  Rc rc; 
  Mouse mouse; 
  Key key; 
}RC_Ctl_t; 
////////////////ң��/////////////////////
/*******************mpu6500*********************************/
typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  
  int16_t temp;
  
  int16_t gx;
  int16_t gy;
  int16_t gz;
  
  int16_t mx;
  int16_t my;
  int16_t mz;
}IMUDataTypedef;


/* ��ģ�����ⲿ�ṩ�ĺ궨�� --------------------------------------------------*/

/* ��ģ�����ⲿ�ṩ�Ľӿڳ������� --------------------------------------------*/
/****************ң��********************/
extern RC_Ctl_t RC_Ctl; //ң������

/* ��ģ�����ⲿ�ṩ�Ľӿں���ԭ������ ----------------------------------------*/
//////����ϵͳ

#endif
