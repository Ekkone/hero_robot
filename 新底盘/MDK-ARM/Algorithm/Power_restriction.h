#ifndef __POWER_RESTRICTION_H
#define __POWER_RESTRICTION_H
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "String.h"
#include "gpio.h"
#include "usart.h"
#include "protocol.h"

/* ��ģ�����ⲿ�ṩ�ĺ궨�� --------------------------------------------------*/
//���ݰ忪��
#define Switch1_On HAL_GPIO_WritePin(Switch_GPIO_Port,Switch1_Pin,GPIO_PIN_SET);	//�ŵ翪��
#define Switch2_On HAL_GPIO_WritePin(Switch_GPIO_Port,Switch2_Pin,GPIO_PIN_SET);//��翪��
#define Switch3_On HAL_GPIO_WritePin(Switch_GPIO_Port,Switch3_Pin,GPIO_PIN_SET);	//����ϵͳֱ�ӹ��翪��

#define	Switch1_Off HAL_GPIO_WritePin(Switch_GPIO_Port,Switch1_Pin,GPIO_PIN_RESET);
#define	Switch2_Off HAL_GPIO_WritePin(Switch_GPIO_Port,Switch2_Pin,GPIO_PIN_RESET);
#define	Switch3_Off HAL_GPIO_WritePin(Switch_GPIO_Port,Switch3_Pin,GPIO_PIN_RESET);

/* ��ģ�����ⲿ�ṩ���������Ͷ��� --------------------------------------------*/
typedef struct{
	
 //��ص���
 float  CurrentCalculat;			 			//����������
 float  Current_Offset;						  //����ƫ����(�Ͳ���ϵͳ�Ƚ�)
 float  Current_Referee;						//���Բ���ϵͳ�ĵ���ֵ
 int16_t Current_Offset_num; 
 //���ݵ�ѹ
 float Capacitance_Volt;						//���ݵ�ѹ
 //�������
 float	Chassis_Current;						//���̵������

}Current_GET;   //�����ɼ�

typedef struct{
		
 float  Volt_Referee;								//���Բ���ϵͳ�ĵ�ѹֵ
 float  Power_Referee;							//���Բ���ϵͳ�Ĺ���
 float  Power_Referee_Last;         
 float  Power_Calculat;							//���м���Ĺ���ֵ
 float  Power_Chassis_Calculat;			//���м���ĵ���˵Ĺ���ֵ
 float  PowerRemain_Referee;				//���Բ���ϵͳ��ʣ�๦��
 float  PowerRemain_Referee_last;		
 float  PowerRemain_Calculat;				//���м���Ĳ���ϵͳ��ʣ�๦��
 float  PowerRemain_Calculat_Last;
 float  PowerRemain_Calculat_Next;
 float  PowerLimit;									//��������ֵ
 
}Limit;				 //��������




typedef struct{

	uint32_t time_now;
	uint32_t time_last;
	int32_t  time;
	int32_t  total_time_ms;
	double  total_time_s;
	
}MyTimeTick;
/* ��ģ�����ⲿ�ṩ�Ľӿڳ������� --------------------------------------------*/
extern  uint32_t  uhADC1ConvertedValue[10];  //ADC1��������
extern  uint32_t  uhADC2ConvertedValue[10];  //ADC2��������
extern  uint32_t  uhADC3ConvertedValue[10];  //ADC3��������

extern  Current_GET  current_get;
extern  Limit  limit;
extern  MyTimeTick  time_for_limit;
/* ��ģ�����ⲿ�ṩ�Ľӿں���ԭ������ ----------------------------------------*/
void power_limit(float  Current_get[4]);
void Super_Capacitance(float  Current_get[4]);
float	Show_CapVolt(void);
float Limit_filter(float oldData,float newData,float val);
float LPF_1st(float oldData, float newData, float lpf_factor);


#endif

