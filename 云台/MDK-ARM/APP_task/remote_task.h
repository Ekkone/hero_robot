#ifndef  __remote_task_H
#define  __remote_task_H
#ifdef __cplusplus
 extern "C" {
#endif
/* 包含头文件----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
	 

#include "pid.h"
#include "string.h"
#include "minipc.h"
#include "protocol.h"
#include "gun_task.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "communication.h"
#include "Power_restriction.h"	 
/* 本模块向外部提供的数据类型定义--------------------------------------------*/	

/* 本模块向外部提供的宏定义--------------------------------------------------*/	 
#define  NORMAL_SPEED_MAX 	1000
#define  NORMAL_SPEED_MIN  -1000
//#define  HIGH_SPEED_MAX 		4000
//#define  HIGH_SPEED_MIN    -4000`
#define  ACC_SPEED    30 
#define  DEC_SPEED    300 
/*按键*/
#define PRESS_DELAY  17
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)

#define Left_Press          (RC_Ctl.mouse.press_l==1)
#define Left__NoPress       (RC_Ctl.mouse.press_l==0)
#define Right_Press         (RC_Ctl.mouse.press_r==1)
#define Right_NoPress       (RC_Ctl.mouse.press_r==0)

#define W_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_W)
#define S_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_S)
#define A_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_A)
#define D_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_D)

#define SHIFT_Press         (RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT)
#define CTRL_Press          (RC_Ctl.key.v & KEY_PRESSED_OFFSET_CTRL)

#define Q_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_Q)
#define E_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_E)
#define R_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_R)
#define F_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_F)
#define G_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_G)
#define Z_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_Z)
#define X_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_X)
#define C_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_C)
#define V_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_V)
#define B_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_B)
typedef struct
{
	float dstVmmps_Y;
	float dstVmmps_X;
	float dstVmmps_W;
	char  flag;
}moto3508_type;
/* 本模块向外部提供的接口常量声明--------------------------------------------*/	
extern float power; 				 //底盘功率 _测试
extern uint8_t chassis_gimble_Mode_flg;
extern float chassis_Current; 
extern float	 chassis_Volt; 
//extern moto3508_type  moto_3508_set; 
/* 本模块向外部提供的接口函数原型声明----------------------------------------*/		 
void Remote_Data_Task(void const * argument); 
void RemoteControlProcess(void);
void ChassisModeProcess(void);
void MouseKeyControlProcess(void);
void ShotProcess(void);
void hard_brak(void);
void Remote_Ctrl(void);
void Minipc_Pid_Init(void);

/* 全局配置区----------------------------------------------------------------*/
	
#ifdef __cplusplus
}
#endif
#endif /*__ usart_task_H */

