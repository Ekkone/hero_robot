#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "USART.h"
#include "String.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

#define UP_REG_ID    0xA0  //up layer regional id
#define DN_REG_ID    0xA5  //down layer regional id
#define HEADER_LEN   sizeof(frame_header_t)
#define CMD_LEN      2    //cmdid bytes
#define CRC_LEN      2    //crc16 bytes

#define PROTOCAL_FRAME_MAX_SIZE  200

/** 
  * @brief  frame header structure definition
  */
//内部变量声明
typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;
//外部变量声明

typedef __packed struct
{
	uint8_t SOF;          //数据起始字节，固定为0xA5          
	uint16_t DataLength;  //数据长度
	uint8_t Seq;          //包序号
	uint8_t CRC8;         //帧头CRC校验
}tFrameHeader;//帧头

typedef enum                 //枚举类型，命令id_变更
{
	game_state = 0x0001,      
	game_result = 0x0002,            
	game_robot_survivors = 0x0003,                
	event_data = 0x0101,							
	supply_projectile_action = 0x0102,								
	game_robot_state = 0x0201,								
	power_heat_data = 0x0202,					
	game_robot_pos = 0x0203,						
	buff_musk =0x0204,  
  robot_energy=0x0205,
  robot_hurt=0x0206,
  shoot_data=0x0207,
  student_interactive_header_data = 0x0301,
  custom_data = 0x0301,
  interactive_data = 0x0301,
	Wrong = 0x1301       
}tCmdID; 

typedef __packed struct
{
  uint8_t game_type : 4;   //比赛类型
  uint8_t game_progress : 4;//比赛阶段
  uint16_t stage_remain_time;//当前阶段剩余时间
}ext_game_state_t; //比赛机器人状态（0x0001）

typedef __packed struct
{

	uint8_t winner ;
	
}ext_game_result_t;   //比赛结果数据（0x0002）

typedef __packed struct
{
	uint16_t robot_legion;
	
}ext_game_robot_survivors_t;   //机器人存活数据（0x0003）

typedef __packed struct
{

	uint32_t event_type;
	
} ext_event_data_t;  //场地事件数据（0x0101）


typedef __packed struct
{
	
	uint8_t supply_projectile_id; 
  uint8_t supply_robot_id; 
  uint8_t supply_projectile_step; 
  uint8_t supply_projectile_num;
  
}ext_supply_projectile_action_t;							//场地补给站动作识别数据（0x0102）

//typedef __packed struct
//{
//	
//	uint8_t supply_projectile_id;
//  uint8_t supply_robot_id; 
//  uint8_t supply_num;

//} ext_supply_projectile_booking_t;					//请求补给站补蛋(0x0103)

typedef __packed struct
{
	
 uint8_t robot_id;
 uint8_t robot_level;
 uint16_t remain_HP;
 uint16_t max_HP;
 uint16_t shooter_heat0_cooling_rate;
 uint16_t shooter_heat0_cooling_limit;
 uint16_t shooter_heat1_cooling_rate;
 uint16_t shooter_heat1_cooling_limit;
 uint8_t mains_power_gimbal_output : 1;
 uint8_t mains_power_chassis_output : 1;
 uint8_t mains_power_shooter_output : 1;

} ext_game_robot_state_t;				//比赛机器人状态(x00201)

typedef __packed struct
{
	
 uint16_t chassis_volt; 
 uint16_t chassis_current; 
 float chassis_power; 
 uint16_t chassis_power_buffer; 
 uint16_t shooter_heat0; 
 uint16_t shooter_heat1; 
	
} ext_power_heat_data_t;						//实时功率热量数据(0x0202)

typedef __packed struct
{
	float x;
 float y;
 float z;
 float yaw;
} ext_game_robot_pos_t;                     //机器人位置(0x0203)

typedef __packed struct
{
 uint8_t power_rune_buff;
}ext_buff_musk_t;                      //机器人增益(x00204)

typedef __packed struct
{
  uint8_t energy_point;
  uint8_t attack_time;
} aerial_robot_energy_t;              //空中机器人能量状态(0x0205)

typedef __packed struct
{
 uint8_t armor_id : 4;
 uint8_t hurt_type : 4;
} ext_robot_hurt_t;                  //伤害状态(0x0206)

typedef __packed struct
{
 uint8_t bullet_type;
 uint8_t bullet_freq;
 float bullet_speed;
} ext_shoot_data_t;                    //实时射击信息(0x0207)

typedef __packed struct
{
 uint16_t data_cmd_id;
 uint16_t send_ID;
 uint16_t receiver_ID;
}ext_student_interactive_header_data_t;    //交互数据接收信息(0x0301)


typedef __packed struct
{
float data1;
float data2;
float data3;
uint8_t masks;
} client_custom_data_t;                    //客户端自定义数据(0x0301)

typedef __packed struct
{
   uint8_t data[10];
} robot_interactive_data_t;                 //交互数据


typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	__packed union 
	{
		ext_game_state_t    			game_state;  				  //
		ext_game_result_t  		    game_result;          //
		ext_game_robot_survivors_t     		game_robot_survivors;          //
		ext_event_data_t			            event_data;				//
		ext_supply_projectile_action_t		supply_projectile_action;						//
		ext_game_robot_state_t		game_robot_state;			//
		ext_power_heat_data_t			power_heat_data;				//
		ext_game_robot_pos_t       game_robot_pos; 		//    
    ext_buff_musk_t            buff_musk;
    aerial_robot_energy_t      robot_energy;
    ext_robot_hurt_t           robot_hurt;
    ext_shoot_data_t           shoot_data;
    ext_student_interactive_header_data_t student_interactive_header_data;
    client_custom_data_t       custom_data;
    robot_interactive_data_t   interactive_data;
	}Data;
	uint16_t        CRC16;         //之前所有数据CRC校验   注意此数据和之前的数据可能不连续，所以不要直接使用，若需要直接使用，必须在此赋值
}tFrame;  //数据帧


//typedef __packed struct
//{
//	tFrameHeader    FrameHeader;
//	tCmdID          CmdID;
//  tSelfDefine     SelfDefine;
//	uint16_t        CRC16;         //之前所有数据CRC校验   注意此数据和之前的数据可能不连续，所以不要直接使用，若需要直接使用，必须在此赋值
//}tFrame;  //数据帧


typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_game_state_t    			game_state;    
	uint16_t        CRC16;         //数据CRC校验
}game_state_tFrame;  //
typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_game_result_t  		    game_result;   
	uint16_t        CRC16;         //数据CRC校验
}game_resultFrame; //?
typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_game_robot_survivors_t     		game_robot_survivors;   
	uint16_t        CRC16;         //数据CRC校验
}game_robot_survivorsFrame;    //

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_event_data_t	  event_data;   
	uint16_t        CRC16;         //数据CRC校验
}event_dataFrame;   //   

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_supply_projectile_action_t		supply_projectile_action;	   
	uint16_t        CRC16;         //数据CRC校验
}supply_projectile_actionFrame;			//

//typedef __packed struct
//{
//	tFrameHeader    FrameHeader;
//	tCmdID          CmdID;
//	ext_supply_projectile_booking_t		supply_projectile_booking;   
//	uint16_t        CRC16;         //数据CRC校验
//}supply_projectile_bookingFrame;								//比赛结果数据(0x006)

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_game_robot_state_t		game_robot_state;   
	uint16_t        CRC16;         //数据CRC校验
}game_robot_stateFrame;					//buff状态任意buff状态改变后发送一次(0x007);	

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_power_heat_data_t			power_heat_data;   
	uint16_t        CRC16;         //数据CRC校验
}power_heat_dataFrame;					 	//机器人位置信息和枪口朝向位置(0x008)

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_game_robot_pos_t       game_robot_pos;    
	uint16_t        CRC16;         //数据CRC校验
}game_robot_posFrame;               //

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_buff_musk_t            buff_musk;    
	uint16_t        CRC16;         //数据CRC校验
}buff_muskFrame;               //

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	aerial_robot_energy_t      robot_energy;    
	uint16_t        CRC16;         //数据CRC校验
}robot_energyFrame;     

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_robot_hurt_t           robot_hurt;   
	uint16_t        CRC16;         //数据CRC校验
}robot_hurtFrame;   

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_shoot_data_t           shoot_data;   
	uint16_t        CRC16;         //数据CRC校验
}shoot_dataFrame; 

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_student_interactive_header_data_t student_interactive_header_data;   
	uint16_t        CRC16;         //数据CRC校验
}student_interactive_header_dataFrame; 

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	client_custom_data_t       custom_data;   
	uint16_t        CRC16;         //数据CRC校验
}custom_dataFrame; 

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	robot_interactive_data_t   interactive_data;  
	uint16_t        CRC16;         //数据CRC校验
}interactive_dataFrame; 


/* 裁判系统数据缓冲区--------------------------------------------*/	
typedef struct
{
  uint16_t shoot_17_cooling_rate; //17每秒冷却值
  uint16_t shoot_42_cooling_rate; 
  uint16_t shoot_17_cooling_limit; //17冷却上限
  uint16_t shoot_42_cooling_limit;
  uint16_t shoot_17_heat;         //17枪口热量
  uint16_t shoot_42_heat;
  uint8_t  shoot_17_freq;         //17射频
  uint8_t  shoot_42_freq;
  float    shoot_17_speed;        //17射速
  float    shoot_42_speed; 
}_HEAT;  
   
typedef struct
{
  float x;
  float y;
  float z;
  float yaw; 
}_POS;     

typedef struct
{
 uint16_t Chassis_Volt;//底盘输出电压
 uint16_t Chassis_Current;//底盘输出电流
 float    chassis_Power;//底盘功率
 uint16_t Chassis_Power_buffer;//功率缓冲
}_POWER;  

   typedef  struct   //
{
  uint8_t id;              //id号
  uint8_t level;           //等级
  uint16_t remainHp;       //剩余血量
  uint16_t maxHp;          //最大血量
  uint8_t  buff;           //增益
  _HEAT    heat;           //枪口热量
  _POS     postion;        //位置
  _POWER   Chassis_Power;  //底盘功率
 
}ROBOT;



extern ROBOT Robot;
#endif
