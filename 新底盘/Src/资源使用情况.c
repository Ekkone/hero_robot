
 /****************************************************************************************************************************************************************
 **************************************************************************line**********************************************************************************
 ****************************************************************************************************************************************************************/
#if BoardNew
新板子：  
  
  -时钟          输入					PLLCK系数					输出					                                         引脚
			   HSE 12MHz			M=12，N=336，P=2			SYSCLK					168MHz
																APB1-peripheral clock	42MHz
																APB1-timer clock		84MHz
																APB2-peripheral clock	84MHz
																APB2-timer clock		168MHz

-ADC
  ADC1_IN4    PA4
  ADC2_IN11   PC1
  ADC3_IN10   PC0
-电容开关
  switch1 PD12
  switch2 PD13
  switch3 PD14
-串口：			-	模块			            DMA_request       DMA-stream             波特率           RX/TX         

  -USART1       -              USART1_RX         DMA2-Stream2             115200         PB7/PB6
  -USART3       -               USART3_RX         DMA1-Stream1             115200         PD9/PD8
  -USART6       -   裁判系统        USART6_RX					DMA2-Stream1          	 115200         PG9/PG14
  -USRT8        -	MINIPC		          UART8_RX					DMA1_Stream6						 115200         PE0/PE1
  -USART2       -    		     USART2_RX					DMA1_Stream5						 115200					PD6/PD5
  
	         NSS/SCK/MISO/MOSI
-SPI       PF6/PF7/PF8/PF9                                                                                                                         


-定时器                             模式						  分频系数			    重载系数		    计数模式            CH1/CH2/CH3/CH4
  
  -TIM5	   	    -摩擦轮电调         PWM输出模式       839+1				    1999+1		           	UP              PH10/PH11/PH12/PI0
  
  
-CAN通讯                                            RX/TX            

-CAN1           -底盘电机           CAN正常模式     PD0/PD1
                                                                                                                     
-CAN2           -云台电机           CAN正常模式     PB12/PB13       


-LED
GREEN                   PF14          
RED                     PE11          
按键							    	PB2		
IST初始化					    	PE3 
IST重置						    	PE2																											
激光							    	PG13
定时器4通道2 //激光引脚	PD13

  #endif
  