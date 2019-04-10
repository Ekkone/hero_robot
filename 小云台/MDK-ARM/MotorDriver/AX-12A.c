/*************************************************************************************
*	@file			AX12-A.c
* @author	 	
*	@version 	V1.0
*	@date			2018/10/8
* @brief		数字总线舵机驱动程序
*************************************************************************************/
/* Includes ------------------------------------------------------------------------*/
#include "AX-12A.h"
/* External variables --------------------------------------------------------------*/
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
/* Internal variables --------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------------*/

void AX_Init(void)
{
  uint8_t id[8] = {0x03,0x0e,0x12,0x10,0x09,0x0b,0x06,0x04};
  for(uint8_t i = 0;i < 8;i++)
    {
      uart_t(0xff);
      uart_t(0xff);
      uart_t(0xfe);
      uart_t(0x09);
      uart_t(0x83);
      uart_t(0x06);
      uart_t(0x04);
      uart_t(id[i]);
      uart_t(0x00);
      uart_t(0x00);  
      uart_t(0xff);
      uart_t(0x03);
      uart_t(~((0x94 + 0x03 + 0xff + (id[i])) & 0xff));
    }
}
void uart_t(uint8_t data)
{
  
  HAL_UART_Transmit(&huart3,&data,1,1);
  

}



void Set_AX11(uint16_t angle,uint16_t speed)
{
  uint8_t id = 0x03;
  uint8_t low = angle;
  uint8_t high = angle >> 8;
  uint8_t low2 = speed;
  uint8_t high2 = speed >> 8;

  uart_t(0xff);
  uart_t(0xff);
  uart_t(0xfe);
  uart_t(0x0b);
  uart_t(0x83);
  uart_t(0x1c);
  uart_t(0x06);
  uart_t(id);
  uart_t(0x20);
  uart_t(0x20);  
  uart_t(low);
  uart_t(high);
  uart_t(low2);
  uart_t(high2);
  uart_t(~((0xee + (id) + (high) + (low) + (high2) + (low2)) & 0xff));

}

void Set_AX6(uint16_t angle,uint16_t speed)
{
  uint8_t id = 0x0e;
  uint8_t low = angle;
  uint8_t high = angle >> 8;
  uint8_t low2 = speed;
  uint8_t high2 = speed >> 8;

  uart_t(0xff);
  uart_t(0xff);
  uart_t(0xfe);
  uart_t(0x0b);
  uart_t(0x83);
  uart_t(0x1c);
  uart_t(0x06);
  uart_t(id);
  uart_t(0x20);
  uart_t(0x20);  
  uart_t(low);
  uart_t(high);
  uart_t(low2);
  uart_t(high2);
  uart_t(~((0xee + (id) + (high) + (low) + (high2) + (low2)) & 0xff));

}

void Set_AX18(uint16_t angle,uint16_t speed)
{
  uint8_t id = 0x12;
  uint8_t low = angle;
  uint8_t high = angle >> 8;
  uint8_t low2 = speed;
  uint8_t high2 = speed >> 8;

  uart_t(0xff);
  uart_t(0xff);
  uart_t(0xfe);
  uart_t(0x0b);
  uart_t(0x83);
  uart_t(0x1c);
  uart_t(0x06);
  uart_t(id);
  uart_t(0x20);
  uart_t(0x20);  
  uart_t(low);
  uart_t(high);
  uart_t(low2);
  uart_t(high2);
  uart_t(~((0xee + (id) + (high) + (low) + (high2) + (low2)) & 0xff));

}

void Set_AX8(uint16_t angle,uint16_t speed)
{
  uint8_t id = 0x10;
  uint8_t low = angle;
  uint8_t high = angle >> 8;
  uint8_t low2 = speed;
  uint8_t high2 = speed >> 8;

  uart_t(0xff);
  uart_t(0xff);
  uart_t(0xfe);
  uart_t(0x0b);
  uart_t(0x83);
  uart_t(0x1c);
  uart_t(0x06);
  uart_t(id);
  uart_t(0x20);
  uart_t(0x20);  
  uart_t(low);
  uart_t(high);
  uart_t(low2);
  uart_t(high2);
  uart_t(~((0xee + (id) + (high) + (low) + (high2) + (low2)) & 0xff));

}





void Set_AX9(uint16_t angle,uint16_t speed)
{
  uint8_t id = 0x09;
  uint8_t low = angle;
  uint8_t high = angle >> 8;
  uint8_t low2 = speed;
  uint8_t high2 = speed >> 8;

  uart_t(0xff);
  uart_t(0xff);
  uart_t(0xfe);
  uart_t(0x0b);
  uart_t(0x83);
  uart_t(0x1c);
  uart_t(0x06);
  uart_t(id);
  uart_t(0x20);
  uart_t(0x20);  
  uart_t(low);
  uart_t(high);
  uart_t(low2);
  uart_t(high2);
  uart_t(~((0xee + (id) + (high) + (low) + (high2) + (low2)) & 0xff));

}


void Set_AX5(uint16_t angle,uint16_t speed)
{
  uint8_t id = 0x0b;
  uint8_t low = angle;
  uint8_t high = angle >> 8;
  uint8_t low2 = speed;
  uint8_t high2 = speed >> 8;

  uart_t(0xff);
  uart_t(0xff);
  uart_t(0xfe);
  uart_t(0x0b);
  uart_t(0x83);
  uart_t(0x1c);
  uart_t(0x06);
  uart_t(id);
  uart_t(0x20);
  uart_t(0x20);  
  uart_t(low);
  uart_t(high);
  uart_t(low2);
  uart_t(high2);
  uart_t(~((0xee + (id) + (high) + (low) + (high2) + (low2)) & 0xff));

}

void Set_AX14(uint16_t angle,uint16_t speed)
{
  uint8_t id = 0x06;
  uint8_t low = angle;
  uint8_t high = angle >> 8;
  uint8_t low2 = speed;
  uint8_t high2 = speed >> 8;

  uart_t(0xff);
  uart_t(0xff);
  uart_t(0xfe);
  uart_t(0x0b);
  uart_t(0x83);
  uart_t(0x1c);
  uart_t(0x06);
  uart_t(id);
  uart_t(0x20);
  uart_t(0x20);  
  uart_t(low);
  uart_t(high);
  uart_t(low2);
  uart_t(high2);
  uart_t(~((0xee + (id) + (high) + (low) + (high2) + (low2)) & 0xff));

}

void Set_AX7(uint16_t angle,uint16_t speed)
{
  uint8_t id = 0x04;
  uint8_t low = angle;
  uint8_t high = angle >> 8;
  uint8_t low2 = speed;
  uint8_t high2 = speed >> 8;

  uart_t(0xff);
  uart_t(0xff);
  uart_t(0xfe);
  uart_t(0x0b);
  uart_t(0x83);
  uart_t(0x1c);
  uart_t(0x06);
  uart_t(id);
  uart_t(0x20);
  uart_t(0x20);  
  uart_t(low);
  uart_t(high);
  uart_t(low2);
  uart_t(high2);
  uart_t(~((0xee + (id) + (high) + (low) + (high2) + (low2)) & 0xff));

}


void Set_AX2(uint16_t angle,uint16_t speed)
{
  uint8_t id = 0x01;
  uint8_t low = angle;
  uint8_t high = angle >> 8;
  uint8_t low2 = speed;
  uint8_t high2 = speed >> 8;

  uart_t(0xff);
  uart_t(0xff);
  uart_t(0xfe);
  uart_t(0x0b);
  uart_t(0x83);
  uart_t(0x1c);
  uart_t(0x06);
  uart_t(id);
  uart_t(0x20);
  uart_t(0x20);  
  uart_t(low);
  uart_t(high);
  uart_t(low2);
  uart_t(high2);
  uart_t(~((0xee + (id) + (high) + (low) + (high2) + (low2)) & 0xff));
}

void Set_AX(uint8_t id,uint16_t angle,uint16_t speed)
{
  uint8_t low = angle;
  uint8_t high = angle >> 8;
  uint8_t low2 = speed;
  uint8_t high2 = speed >> 8;

  uart_t(0xff);
  uart_t(0xff);
  uart_t(0xfe);
  uart_t(0x0b);
  uart_t(0x83);
  uart_t(0x1c);
  uart_t(0x06);
  uart_t(id);
  uart_t(0x20);
  uart_t(0x20);  
  uart_t(low);
  uart_t(high);
  uart_t(low2);
  uart_t(high2);
  uart_t(~((0xee + (id) + (high) + (low) + (high2) + (low2)) & 0xff));
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

