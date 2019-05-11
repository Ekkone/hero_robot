/*******************************************************************************
*                     ��Ȩ���� (C), 2017-,NCUROBOT
********************************************************************************
* �� �� ��   : Motor_USE_CAN.c
* �� �� ��   : ����
* ��    ��   : NCURM
* ��������   : 2018��7��
* ����޸�   :
* ��������   : �����ģ����ʹ��CAN���п��Ƶĵ��
* �����б�   :
*ʹ��CANͨѶ�ĵ������̨���   		 ���̵��	 	 	  �������
*				 	��Ӧ�ͺţ� c620						3508					 C2000
*�ӿں�����
*					Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch)
*					Chassis_Motor( CAN_HandleTypeDef * hcan,
*								  int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
*******************************************************************************/
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "Motor_USE_CAN.h"
#include "communication.h "
#include "protocol.h"
#include "minipc.h"
/* �ڲ��Զ����������� --------------------------------------------------------*/

/* �ڲ��궨�� ----------------------------------------------------------------*/

/* ���������Ϣ����-----------------------------------------------------------*/

/* �ڲ���������---------------------------------------------------------------*/

/* �ⲿ�������� --------------------------------------------------------------*/
/*******************Ħ���ֵ���͵��̵���Ĳ�������***************************/
moto_measure_t   moto_chassis_get[4] = {0};//4 �� 3508
moto_measure_t   moto_dial_get = {0};  //c2006
moto_measure_t   moto_M_get[2] = {0};    
moto_measure_t   pit_get;
moto_measure_t   yaw_get;
/* �ⲿ����ԭ������ ----------------------------------------------------------*/

/* �ڲ����� ------------------------------------------------------------------*/
//Ϊcan���ͷֱ𴴽����棬��ֹ���ڷ��͵�ʱ����ֻ��һ���ڴ���໥����
static CanTxMsgTypeDef  CANSend_B;
static CanTxMsgTypeDef	CANSend_S;
static CanTxMsgTypeDef  Chassis_Motor_Header;

/* ����ԭ������ ----------------------------------------------------------*/
/* ����ԭ������ ----------------------------------------------------------*/
extern RC_Ctl_t RC_Ctl;
extern uint8_t chassis_gimble_Mode_flg;
uint8_t stir_motor_flag = 0;
int16_t yaw_speed;
int16_t R_key_v;
int16_t R_rc_ch0;
int16_t R_rc_ch1;
uint8_t R_rc_s1;
uint8_t R_rc_s2;
/**
	**************************************************************
	** Descriptions: ���̵����������
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN2
	**					iqn:��n�����̵���ĵ���ֵ
	** Output: NULL
	**************************************************************
**/
void Chassis_Motor( CAN_HandleTypeDef * hcan,
									  int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
			Chassis_Motor_Header.DLC = 0x08;
			Chassis_Motor_Header.IDE = CAN_ID_STD;
			Chassis_Motor_Header.RTR = CAN_RTR_DATA;
			Chassis_Motor_Header.StdId = 0x200;

			Chassis_Motor_Header.Data[0]=iq1>>8;
			Chassis_Motor_Header.Data[1]=iq1;
			Chassis_Motor_Header.Data[2]=iq2>>8;
			Chassis_Motor_Header.Data[3]=iq2;
			Chassis_Motor_Header.Data[4]=iq3>>8;
			Chassis_Motor_Header.Data[5]=iq3;
			Chassis_Motor_Header.Data[6]=iq4>>8;
			Chassis_Motor_Header.Data[7]=iq4;
	
			hcan->pTxMsg = &Chassis_Motor_Header;
			HAL_CAN_Transmit(hcan,0);
}	

/**
	**************************************************************
	** Descriptions: ���̵��ʧ�ܺ���
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN2
	**					iqn:��n�����̵���ĵ���ֵ
	** Output: NULL
	**************************************************************
**/
void Chassis_Motor_Disable( CAN_HandleTypeDef * hcan)
{
			Chassis_Motor_Header.DLC = 0x08;
			Chassis_Motor_Header.IDE = CAN_ID_STD;
			Chassis_Motor_Header.RTR = CAN_RTR_DATA;
			Chassis_Motor_Header.StdId = 0x200;

			Chassis_Motor_Header.Data[0]=0x00;
			Chassis_Motor_Header.Data[1]=0x00;
			Chassis_Motor_Header.Data[2]=0x00;
			Chassis_Motor_Header.Data[3]=0x00;
			Chassis_Motor_Header.Data[4]=0x00;
			Chassis_Motor_Header.Data[5]=0x00;
			Chassis_Motor_Header.Data[6]=0x00;
			Chassis_Motor_Header.Data[7]=0x00;
	
			hcan->pTxMsg = &Chassis_Motor_Header;
			HAL_CAN_Transmit(hcan,5);
}	
/**                                                           //����
	**************************************************************
	** Descriptions: ��ȡCANͨѶ��3508����ķ���ֵ
	** Input: 	
	**			  ptr:Ŀ�����ݵ��ڴ��ַ
	**				hcan->pRxMsg->Data:���������CAN�����ݵ�����
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
	** Descriptions:��ȡ�������ֵ��ƫ��ֵ
	** Input: 	
	**			  ptr:Ŀ�����ݵ��ڴ��ַ
	**				hcan->pRxMsg->Data:���������CAN�����ݵ�����
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
	** Descriptions: ��ȡ������ܽǶ�ֵ
	** Input: 	
	**			   *P:��Ҫ��ȡ�ܽǶ�ֵ�ĵ�ַ
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
/**
	**************************************************************
	** Descriptions: ����ͨ��
	** Input: 	ң��������
	**			   *P:�ṹ��
	**				
	** Output: NULL
	**************************************************************
**/

void CAN_Send_B( CAN_HandleTypeDef * hcan)
{
			CANSend_B.DLC = 0x08;
			CANSend_B.IDE = CAN_ID_STD;
			CANSend_B.RTR = CAN_RTR_DATA;
			CANSend_B.StdId = CAN_TX_B;

			CANSend_B.Data[0]=minipc_rx_big.angle_yaw >>8;
			CANSend_B.Data[1]=minipc_rx_big.angle_yaw;
			CANSend_B.Data[2]=minipc_rx_big.angle_pit>>8;
			CANSend_B.Data[3]=minipc_rx_big.angle_pit;
			CANSend_B.Data[4]=minipc_rx_big.state_flag;
			CANSend_B.Data[5]=(uint16_t)(Robot.heat.shoot_42_cooling_limit - Robot.heat.shoot_42_heat)>>8;;
			CANSend_B.Data[6]=(uint16_t)(Robot.heat.shoot_42_cooling_limit - Robot.heat.shoot_42_heat);;
			CANSend_B.Data[7]=0;
	
			hcan->pTxMsg = &CANSend_B;
			HAL_CAN_Transmit(hcan,30);
}	
extern uint8_t communication_message;
void CAN_Send_S( CAN_HandleTypeDef * hcan)
{
			CANSend_S.DLC = 0x08;
			CANSend_S.IDE = CAN_ID_STD;
			CANSend_S.RTR = CAN_RTR_DATA;
			CANSend_S.StdId = CAN_TX_S;

			CANSend_S.Data[0]=minipc_rx_small.angle_yaw >> 8;
			CANSend_S.Data[1]=minipc_rx_small.angle_yaw;
			CANSend_S.Data[2]=minipc_rx_small.angle_pit>>8;
			CANSend_S.Data[3]=minipc_rx_small.angle_pit;
			CANSend_S.Data[4]=minipc_rx_small.state_flag;
			CANSend_S.Data[5]=(stir_motor_flag << 7) | (communication_message & 0x7f);
			CANSend_S.Data[6]=(uint16_t)(Robot.heat.shoot_17_cooling_limit - Robot.heat.shoot_17_heat)>>8;
			CANSend_S.Data[7]=(uint16_t)(Robot.heat.shoot_17_cooling_limit - Robot.heat.shoot_17_heat);
	
			hcan->pTxMsg = &CANSend_S;
			HAL_CAN_Transmit(hcan,30);
}	
//���գ�����̨
void CAN_RX_YT(CAN_HandleTypeDef * hcan)
{
	yaw_get.angle = (int16_t)(hcan->pRxMsg->Data[0]<<8 |hcan->pRxMsg->Data[1]) ;
	yaw_speed = (int16_t) (hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]) ;
	chassis_gimble_Mode_flg = (uint8_t)(hcan->pRxMsg->Data[4]);
	stir_motor_flag = (uint8_t)(hcan->pRxMsg->Data[5]);
	
}
void CAN_RX_YK(CAN_HandleTypeDef * hcan)
{
 RC_Ctl.key.v=(int16_t)(hcan->pRxMsg->Data[0]<<8 |hcan->pRxMsg->Data[1]) ;
 RC_Ctl.rc.ch0=(int16_t) (hcan->pRxMsg->Data[2]<<8 |hcan->pRxMsg->Data[3]) ;
 RC_Ctl.rc.ch1=(int16_t) (hcan->pRxMsg->Data[4]<<8 |hcan->pRxMsg->Data[5]) ;
 RC_Ctl.rc.s1=hcan->pRxMsg->Data[6];
 RC_Ctl.rc.s2=hcan->pRxMsg->Data[7];
	
}