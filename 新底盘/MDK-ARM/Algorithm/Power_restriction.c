/*******************************************************************************
                      版权所有 (C), 2017-,NCUROBOT
 *******************************************************************************
  文 件 名   : Power_restriction.c
  版 本 号   : 初稿
  作    者   : NCUERM
  生成日期   : 2018年7月
  最近修改   :
  功能描述   : 底盘功率限制
  函数列表   :void power_limit(float  Current_get[4])


*******************************************************************************/
/*
* 电压输出与电流输入的关系:
*			VOUT = VOUT(Q) + Vsens*I;
*	VOUT(Q)为静态输出电压，即VCC/2，也就是无输入时，输出的电压。Vsens是传感器灵敏度
*（使用的型号的系数是40MV/A）I的方向是从IP+流向IP-的电流
*	eg:
*			VCC为5V，I电流为10A，那么输出即为5V / 2 + 40MV/A * 10A = 2.9V
*			VCC为3.3V，I电流为10A，那么输出即为3.3V / 2 + 40MV/A*10A = 2.05V
* 结果：
*			I = (VOUT - VOUT(Q))/Vsens
*			I = (V_get - 2.5)/0.04; 	  //接5V电压
*			I = (V_get - 1.65)/0.04;		//接3.3V电压
*/
/* 包含头文件 ----------------------------------------------------------------*/
#include "Power_restriction.h"
/* 内部自定义数据类型 --------------------------------------------------------*/
#include "Motor_USE_CAN.h"

/* 内部宏定义 ----------------------------------------------------------------*/
#define MyAbs(x) ((x > 0) ? (x) : (-x))
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

#define SET_BITS(x,n,m)    (x | ~(~0U<<(m-n+1))<<(n-1))

#define Predict_RemainPower 30

//#define Switch1_On HAL_GPIO_WritePin(Switch_GPIO_Port,Switch1_Pin,GPIO_PIN_SET);	//放电开关
//#define Switch2_On HAL_GPIO_WritePin(Switch_GPIO_Port,Switch2_Pin,GPIO_PIN_SET); //充电开关
//#define Switch3_On HAL_GPIO_WritePin(Switch_GPIO_Port,Switch3_Pin,GPIO_PIN_SET);//裁判系统直接供电开关


#define Charge_switch()						\
{																	\
	Switch1_Off;										\
	Switch2_On;											\
	Switch3_On;											\
}	
#define	Normal_switch()						\
{																	\
	Switch1_Off;										\
	Switch2_Off;										\
	Switch3_On;											\
}

//在切换状态的过程中，从正常切换到放电，会有一瞬间形成电容放电同时24v供电的状态，这种状态是不是有益的？
//需不需要刻意避免？如果避免之后，同样会有一瞬间电机电调断电的状态，这种状态是否会导致电调重启，或者损坏电调。
#define	Discharge_switch()				\
{																	\
	Switch2_Off;										\
	Switch3_Off;										\
	Switch1_On;											\
}

//一级状态	
#define	Judge				1
#define	UnJudge 		0
//二级状态
#define	Charge			1
#define	Normal			2
#define Discharge  	3
//判据
#define BoundaryOFCharge	40		//进入充电状态的最大功率
#define	RatedPower				80		//额定功率
#define	BoundaryOFNormal	75		//退出充电进入正常状态的最大功率
//ABS（BoundaryOFCharge - BoundaryOFNormal）是最大的电容充电功率35
#define MinVoltOFCap			16.0f	//电容放电的最小允许电压                       16.0
#define	RatedCurrent			6000.0//额定功率下对应的额定电流值（发送给电调的值） 5000.0
#define SoftLimitRP				55.0f		//进入软件限制的剩余能量值
#define	Filed							1.5f		//屏蔽范围                                    2.0
#define QuitSoftLimitPower 10     //防止低功率下的功率限制 
/* 任务相关信息定义-----------------------------------------------------------*/

/* 内部常量定义---------------------------------------------------------------*/

/* 外部变量声明 --------------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ROBOT Robot;

/* 外部函数原型声明 ----------------------------------------------------------*/

/* 内部变量 ------------------------------------------------------------------*/

/*
*
*	数据说明:ADC缓存数据 , 乘以系数0.8058608{3300/4095}
*  			  得到实际电压值
*	采样时间:采集一次数据的时间是(转换周期+采样周期)/时钟(12位采集是15个周期)
*				  所以配置的采样时间是(12+3)/(84/4)M = 0.7142us
*	   Note:实际调用数据的间隔最小5ms，所以创建一个长度为10的数组，每次使用数据的时候
*		   		对10个数据取平均值，然后再创建一个10位的数组存储历史数据，进行窗口滑动滤
*					波 (因为现在还没有硬件设计，暂时用这种方法简单滤波),数据有效时长延长到0.7ms
*/
uint32_t  uhADC1ConvertedValue[10] = {0}; 
uint32_t  uhADC2ConvertedValue[10] = {0};  
uint32_t  uhADC3ConvertedValue[10] = {0};  

Limit  limit = {.PowerRemain_Calculat = 60};
Current_GET  current_get = {0};
MyTimeTick  time_for_limit = {0}; //限制时间计时
MyTimeTick  time_for_RP = {0};  //remain power 缓存能量计算计时



//状态变量
uint8_t state1 = Judge;
uint8_t	state2 = Charge;

/* 函数原型声明 ----------------------------------------------------------*/


/*
** Descriptions: 交换两变量值
** Input: 需要交换的两个变量float型
** Output: NULL
*/
void swap(float *a,float *b)
{
 float c;
 c=*a;*a=*b;*b=c;
}
/*
** Descriptions: 将数组分成两部分，前一部分的值均比后一部分值小
** Input: 要求的数据的开头和末尾
** Output: 返回分界点
*/
int Partition(float data[],int low,int high)
{
 float pivokey;
 pivokey=data[low];
 while(low<high)
 {
  while(low<high&&data[high]>=pivokey)
   high--;
  swap(&data[low],&data[high]);

  while(low<high&&data[low]<=pivokey)
   low++;
  swap(&data[low],&data[high]);
 }
 return low;
}
/*
** Descriptions: 进行的递归的调用，达到排序的目的
** Input: 需要进行排序的数组指针，以及对应的范围
** Output: NULL
*/
void QSort(float data[],int low,int high)
{
 if(low<high)
 {
  int pivokey=Partition(data,low,high);
  QSort(data,low,pivokey-1);
  QSort(data,pivokey+1,high);
 }
}
/*
** Descriptions: 求得缓存的中值
** Input:对应的缓存指针，缓存的数组长度需要为10
** Output:中值
*/
float Median_value_fliter(uint32_t *buff,int length)
{	
	uint32_t mybuff[length];
	memcpy(mybuff,buff,length);
	QSort((float*)mybuff, 0, length-1);
	return buff[(int)((length-1)/2)];
}
/*
** Descriptions: 中值平均滤波
** Input:对应的缓存指针，缓存的数组长度需要为10
** Output:滤去最大最小值的平均值
*/
float Median_average_fliter(uint32_t *buff,int length)
{
	int16_t sum = 0;
	uint32_t mybuff[length];
	memcpy(mybuff,buff,length);
	QSort((float*)mybuff, 0, length-1);
	for(uint8_t i = 1;i < length-1;i++)
	{
		sum += mybuff[i];
	}
	sum = sum/(length-2);
	return sum;
}
/*
** Descriptions: 均值滤波
** Input: 需要滤波的缓存的指针
** Output: 滤波结果
*/
float Average_value_fliter(uint32_t *buff)
{
	float sum = 0;
	uint32_t mybuff[10];
	memcpy(mybuff,buff,10);
	for(uint8_t i = 0;i < 10;i++ )
	{
		sum += mybuff[i];
	}
	sum *= 0.1f;
	return sum;
}
/*
** Descriptions: 窗口滑动滤波
** Input: 需要滤波的缓存的指针
** Output: 滤波结果
*/
float Window_sliding_filter(float *buff)
{
	float sum = 0;
	for(uint8_t i = 0; i < 10; i++) {
	buff[i] = buff[i+1]; // 所有数据左移，低位仍掉
	sum += buff[i];
  }
	
	return sum;
}

/*
** Descriptions: 一阶低通滤波
** Input: 
** Output: 滤波结果
*/
float LPF_1st(float oldData, float newData, float lpf_factor)
{
	return oldData * (1 - lpf_factor) + newData * lpf_factor;
}

/*
** Descriptions: 限幅滤波
** Input:   相邻的两次数据
** Output: 滤波结果
*/
float Limit_filter(float oldData,float newData,float val)
{
	if(MyAbs(newData-oldData)>val)
		{
			return oldData;
		}
	else
		{
			return newData;
		}
}


/*
** Descriptions: 粗略测量任务时间(s)
** Input: 时间结构体的指针
** Output: NULL
*/
void	MyTime_statistics(MyTimeTick *time_tick)
{

	time_tick->time_now = HAL_GetTick();
	if(time_tick->time_last == 0)//避免第一次计数的时候last为零
	{
		time_tick->time = time_tick->time_now - time_tick->time_now;
	}else
	{
		time_tick->time = time_tick->time_now - time_tick->time_last;		
	}
	time_tick->time_last = time_tick->time_now;
	//切记清零总时间
	time_tick->total_time_ms += time_tick->time;
	time_tick->total_time_s = time_tick->total_time_ms * 0.001f;
}
/*
** Descriptions: 清空时间结构体
** Input: 
**				MyTimeTick *: 		时间结构体的指针
**				flag:  选择清空的内容
**								flag = 1  清空全部
**								flag = 2	清空总时间(防止连续调用时的计时溢1出错误)
** Output: NULL
*/
void MyTime_memset(MyTimeTick *time_tick ,char flag)
{
	if(flag == 1)
	{
		memset(time_tick,0,sizeof(MyTimeTick));
	}else if(flag == 2)
	{
		time_tick->total_time_ms = 0;
		time_tick->total_time_s = 0;
	}
	
}



/*
** Descriptions: 屏蔽函数
** Input: NULL
** Output: NULL
*/
//屏蔽waterlin(除过本身)附近field范围以内的数值
float Shield(float num,float waterline,float filed)
{
		float difer = 0;
	
		difer = num - waterline;
		if (MyAbs(difer) < filed)
		{
	
				if (difer < 0)
				{
						return num;
				}
				else
				{
						return waterline;
				}
		}
		
		return num;
}

/*
** Descriptions: ADC数据采集并滤波
** Input: NULL
** Output: 数据采集值
*/
void Get_ADC_Value(void)
{
		static uint32_t *buff2 = uhADC2ConvertedValue; //电容总电压Hadc数据
		static uint32_t *buff3 = uhADC3ConvertedValue; //电容总电压Ladc数据

		//电容电压ADC数据处理,暂做均值处理
		static uint32_t sum_voltH = 0,sum_voltL = 0;
		static float buff[6] = {0};
		for(uint8_t i = 0;i < 10;i++)
		{
				sum_voltH += buff2[i];
				sum_voltL += buff3[i]; 
		}		

		sum_voltH = sum_voltH / 10;
		sum_voltL = sum_voltL / 10;
		
		current_get.Capacitance_Volt = 10.3023256f * (((float)sum_voltH - (float)sum_voltL) * 0.0008056f);
		
		buff[0] = buff[1];
		buff[1] = buff[2];
		buff[2] = buff[3];
		buff[3] = buff[4];
		buff[4] = current_get.Capacitance_Volt;
		
		buff[5] = buff[0] + buff[1] + buff[2] + buff[3] + buff[4];
		
		current_get.Capacitance_Volt = buff[5] / 5;
		
}

/*
** Descriptions: 功率计算
** Input: NULL
** Output: NULL
*/

void Power_Calculate(void)
{
		limit.Power_Referee =  Robot.Chassis_Power.chassis_Power;

    limit.Power_Calculat = limit.Power_Referee;
}

/*
** Descriptions: 缓存能量计算
** Input: NULL
** Output: NULL
*/
void Remain_Power_Calculate(void)
{	
		/*有裁判系统更新数据，就使用裁判系统数据*/
		limit.PowerRemain_Referee = Robot.Chassis_Power.Chassis_Power_buffer;

    limit.PowerRemain_Calculat = limit.PowerRemain_Referee;
}
/*
** Descriptions:电容电压处理
** Input:waterline:水平线，filed：范围
** Output:
** Note: 向下屏蔽
*/
float Down_Shield(float num,float waterline ,float filed)
{
	float dir = 0;
	dir = num - waterline;
	if (MyAbs(dir) < filed)//在filed范围内
	{
		if (num >= waterline)
		{
				return num;
		}
		else 
		{
				return waterline - filed;
		}
	}
}
void	Cap_Volt_Treatment(float waterline ,float filed)
{
		current_get.Capacitance_Volt = Down_Shield(current_get.Capacitance_Volt, waterline,filed);
}

/*
*	如何充分利用裁判系统传回的功率和能量缓存
*
*/
void power_limit(float  * Current_get)
{
    static uint8_t soft_limit_flag = 0;
    
		float total_current = 0;
	
		total_current = MyAbs(Current_get[0]) + MyAbs(Current_get[1])+\
										MyAbs(Current_get[2]) + MyAbs(Current_get[3]);
	
		/*功率限制*/
		limit.PowerRemain_Calculat_Next = Robot.Chassis_Power.Chassis_Power_buffer;//limit.PowerRemain_Calculat;////预测能量缓存没用到
      
    if (limit.PowerRemain_Calculat_Next > SoftLimitRP)
    {
        soft_limit_flag = 0;
    }
    else if(limit.PowerRemain_Calculat_Next < Predict_RemainPower)//30
    {
        soft_limit_flag = 1;
    }
  
		if(soft_limit_flag == 1 && (limit.Power_Calculat > QuitSoftLimitPower || limit.PowerRemain_Calculat_Next < Predict_RemainPower))//20 10
		{      
			if(limit.PowerRemain_Calculat_Next <0) 
			{
				limit.PowerRemain_Calculat_Next = 0;
			}
			
			limit.PowerLimit = RatedCurrent;
			limit.PowerLimit = ((limit.PowerRemain_Calculat_Next * limit.PowerRemain_Calculat_Next) / 3025) * limit.PowerLimit;		
			
			/*电流限制*/
			Current_get[0] = (Current_get[0]/(total_current + 1.0f)) * limit.PowerLimit; 
			Current_get[1] = (Current_get[1]/(total_current + 1.0f)) * limit.PowerLimit; 
			Current_get[2] = (Current_get[2]/(total_current + 1.0f)) * limit.PowerLimit; 
			Current_get[3] = (Current_get[3]/(total_current + 1.0f)) * limit.PowerLimit; 
		
		}
		
		
		if(limit.Power_Calculat < 80)
		{
			limit.PowerLimit = 0;
		}
	
		limit.PowerRemain_Calculat_Last = limit.PowerRemain_Calculat;

}



/*注意：两种板子有一个不能同时开24v供电和电容让供电 */
void Super_Capacitance(float * Current_get)
{
		static float total_current = 0;
		static uint8_t flag = 0;
		//计算总电流
//		total_current = MyAbs(moto_chassis_get[0].real_current) + MyAbs(moto_chassis_get[1].real_current)\
//										+ MyAbs(moto_chassis_get[2].real_current) + MyAbs(moto_chassis_get[3].real_current);

		/*ADC数据处理*/
		Get_ADC_Value();
	
		/*计算功率*/
		Power_Calculate();
	
		/*计算剩余能量*/
		Remain_Power_Calculate();
		
		/*处理电容电压*/
		Cap_Volt_Treatment(MinVoltOFCap,Filed);
	
		/*判断一级状态*/
		if (state1)
		{
			
			//指定当前二级状态
			if (limit.Power_Calculat < BoundaryOFCharge)
			{
					state2 = Charge;
			}
			else if (limit.Power_Calculat <= RatedPower)
			{
					state2 = Normal;
			}
			else if (limit.Power_Calculat > RatedPower)
			{
					state2 = Discharge;
			}
		
			state1 = UnJudge;
		}

		/*二级状态下的动作执行*/
		static uint8_t discharge_flag = 0,soft_limit_flag = 0;
		switch (state2)
		{
				case Charge:
				{
						Charge_switch();
						
						if (limit.Power_Calculat >= BoundaryOFNormal)
						{
								state1 = Judge;
								Normal_switch();
						}
				}
				break;
				case Normal:
				{
						Normal_switch();
						state1 = Judge;
				}
				break;
				case Discharge:
				{ 	
						if (current_get.Capacitance_Volt < MinVoltOFCap || soft_limit_flag)//电容电量不足
						{
                soft_limit_flag = 1;
              
								Normal_switch();
								//软件功率限制
								power_limit(Current_get);
							
								if (limit.PowerRemain_Calculat > SoftLimitRP)
								{
										state1 = Judge;
                    soft_limit_flag = 0;
								}
									
								discharge_flag = 0;								
																									flag = 2;
								break;
						}
							
						if (discharge_flag)
						{
								if (limit.PowerRemain_Calculat > 50)
								{
										Normal_switch();
										state1 = Judge;
										discharge_flag = 0;	
																									flag = 0;
								}
								else
								{
										Discharge_switch();					
																									flag = 1;
								}
						}
						else if(limit.PowerRemain_Calculat < 30)//剩余能量做判据
						{
                discharge_flag = 1;		
                Discharge_switch();				
						}
						else
						{
								Normal_switch();
								state1 = Judge;
						}
				}
				break;
			}
		

//			printf("power_referee:%f,power:%f,capVolt:%f,limit.PowerRemain_Calculat:%d,state2:%d,discharge_flag:%d,flag:%d\n\r",
//			limit.Power_Referee,limit.Power_Calculat,current_get.Capacitance_Volt, Robot.Chassis_Power.Chassis_Power_buffer,state2,discharge_flag,flag);
}
/*
** Descriptions: 电容电量显示
** Input:	
** Output:
** Note:
*/
uint8_t set_bits(uint8_t n, uint8_t m)
{
		uint8_t buff = 0;
	
		for(uint8_t num = n;num < m + 1;num++)
		{
				buff |= (1 << num);
		}
		
		return buff;
}

float	Show_CapVolt(void)
{
    return (current_get.Capacitance_Volt - MinVoltOFCap)*12.5;	
}
