/*******************************************************************************
                      ��Ȩ���� (C), 2017-,NCUROBOT
 *******************************************************************************
  �� �� ��   : Power_restriction.c
  �� �� ��   : ����
  ��    ��   : NCUERM
  ��������   : 2018��7��
  ����޸�   :
  ��������   : ���̹�������
  �����б�   :void power_limit(float  Current_get[4])


*******************************************************************************/
/*
* ��ѹ������������Ĺ�ϵ:
*			VOUT = VOUT(Q) + Vsens*I;
*	VOUT(Q)Ϊ��̬�����ѹ����VCC/2��Ҳ����������ʱ������ĵ�ѹ��Vsens�Ǵ�����������
*��ʹ�õ��ͺŵ�ϵ����40MV/A��I�ķ����Ǵ�IP+����IP-�ĵ���
*	eg:
*			VCCΪ5V��I����Ϊ10A����ô�����Ϊ5V / 2 + 40MV/A * 10A = 2.9V
*			VCCΪ3.3V��I����Ϊ10A����ô�����Ϊ3.3V / 2 + 40MV/A*10A = 2.05V
* �����
*			I = (VOUT - VOUT(Q))/Vsens
*			I = (V_get - 2.5)/0.04; 	  //��5V��ѹ
*			I = (V_get - 1.65)/0.04;		//��3.3V��ѹ
*/
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "Power_restriction.h"
/* �ڲ��Զ����������� --------------------------------------------------------*/
#include "Motor_USE_CAN.h"

/* �ڲ��궨�� ----------------------------------------------------------------*/
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

#define Predict_Time  0.05f
#define Predict_RemainPower 20

#define Switch1_On HAL_GPIO_WritePin(Switch_GPIO_Port,Switch1_Pin,GPIO_PIN_SET);	//�ŵ翪��
#define Switch2_On HAL_GPIO_WritePin(Switch_GPIO_Port,Switch2_Pin,GPIO_PIN_SET); //��翪��
#define Switch3_On HAL_GPIO_WritePin(Switch_GPIO_Port,Switch3_Pin,GPIO_PIN_SET);	//����ϵͳֱ�ӹ��翪��


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

//���л�״̬�Ĺ����У��������л����ŵ磬����һ˲���γɵ��ݷŵ�ͬʱ24v�����״̬������״̬�ǲ�������ģ�
//�費��Ҫ������⣿�������֮��ͬ������һ˲��������ϵ��״̬������״̬�Ƿ�ᵼ�µ�������������𻵵����
#define	Discharge_switch()				\
{																	\
	Switch2_Off;										\
	Switch3_Off;										\
	Switch1_On;											\
}

//һ��״̬	
#define	Judge				1
#define	UnJudge 		0
//����״̬
#define	Charge			1
#define	Normal			2
#define Discharge  	3
//�о�
#define BoundaryOFCharge	39		//������״̬�������
#define	RatedPower				80		//�����
#define	BoundaryOFNormal	75		//�˳�����������״̬�������
//ABS��BoundaryOFCharge - BoundaryOFNormal�������ĵ��ݳ�繦��36
#define MinVoltOFCap			15.0f	//���ݷŵ����С�����ѹ
#define	RatedCurrent			2900.0//������¶�Ӧ�Ķ����ֵ�����͸������ֵ��
#define SoftLimitRP				55.0f		//����������Ƶ�ʣ������ֵ
#define	MinCapPower				40.0f		//��С�ĵ��ݷŵ繦��
#define	Filed							1.5f		//���η�Χ
/* ���������Ϣ����-----------------------------------------------------------*/

/* �ڲ���������---------------------------------------------------------------*/

/* �ⲿ�������� --------------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ROBOT Robot;

/* �ⲿ����ԭ������ ----------------------------------------------------------*/

/* �ڲ����� ------------------------------------------------------------------*/

/*
*
*	����˵��:ADC�������� , ����ϵ��0.8058608{3300/4095}
*  			  �õ�ʵ�ʵ�ѹֵ
*	����ʱ��:�ɼ�һ�����ݵ�ʱ����(ת������+��������)/ʱ��(12λ�ɼ���15������)
*				  �������õĲ���ʱ����(12+3)/(84/4)M = 0.7142us
*	   Note:ʵ�ʵ������ݵļ����С5ms�����Դ���һ������Ϊ10�����飬ÿ��ʹ�����ݵ�ʱ��
*		   		��10������ȡƽ��ֵ��Ȼ���ٴ���һ��10λ������洢��ʷ���ݣ����д��ڻ�����
*					�� (��Ϊ���ڻ�û��Ӳ����ƣ���ʱ�����ַ������˲�),������Чʱ���ӳ���0.7ms
*/
uint32_t  uhADC1ConvertedValue[10] = {0}; 
uint32_t  uhADC2ConvertedValue[10] = {0};  
uint32_t  uhADC3ConvertedValue[10] = {0};  

Limit  limit = {.PowerRemain_Calculat = 60};
Current_GET  current_get = {0};
MyTimeTick  time_for_limit = {0}; //����ʱ���ʱ
MyTimeTick  time_for_RP = {0};  //remain power �������������ʱ



//״̬����
uint8_t state1 = Judge;
uint8_t	state2 = Charge;

/* ����ԭ������ ----------------------------------------------------------*/


/*
** Descriptions: ����������ֵ
** Input: ��Ҫ��������������float��
** Output: NULL
*/
void swap(float *a,float *b)
{
 float c;
 c=*a;*a=*b;*b=c;
}
/*
** Descriptions: ������ֳ������֣�ǰһ���ֵ�ֵ���Ⱥ�һ����ֵС
** Input: Ҫ������ݵĿ�ͷ��ĩβ
** Output: ���طֽ��
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
** Descriptions: ���еĵݹ�ĵ��ã��ﵽ�����Ŀ��
** Input: ��Ҫ�������������ָ�룬�Լ���Ӧ�ķ�Χ
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
** Descriptions: ��û������ֵ
** Input:��Ӧ�Ļ���ָ�룬��������鳤����ҪΪ10
** Output:��ֵ
*/
float Median_value_fliter(uint32_t *buff,int length)
{	
	uint32_t mybuff[length];
	memcpy(mybuff,buff,length);
	QSort((float*)mybuff, 0, length-1);
	return buff[(int)((length-1)/2)];
}
/*
** Descriptions: ��ֵƽ���˲�
** Input:��Ӧ�Ļ���ָ�룬��������鳤����ҪΪ10
** Output:��ȥ�����Сֵ��ƽ��ֵ
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
** Descriptions: ��ֵ�˲�
** Input: ��Ҫ�˲��Ļ����ָ��
** Output: �˲����
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
** Descriptions: ���ڻ����˲�
** Input: ��Ҫ�˲��Ļ����ָ��
** Output: �˲����
*/
float Window_sliding_filter(float *buff)
{
	float sum = 0;
	for(uint8_t i = 0; i < 10; i++) {
	buff[i] = buff[i+1]; // �����������ƣ���λ�Ե�
	sum += buff[i];
  }
	
	return sum;
}

/*
** Descriptions: һ�׵�ͨ�˲�
** Input: 
** Output: �˲����
*/
float LPF_1st(float oldData, float newData, float lpf_factor)
{
	return oldData * (1 - lpf_factor) + newData * lpf_factor;
}

/*
** Descriptions: �޷��˲�
** Input:   ���ڵ���������
** Output: �˲����
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
** Descriptions: ���Բ�������ʱ��(s)
** Input: ʱ��ṹ���ָ��
** Output: NULL
*/
void	MyTime_statistics(MyTimeTick *time_tick)
{

	time_tick->time_now = HAL_GetTick();
	if(time_tick->time_last == 0)//�����һ�μ�����ʱ��lastΪ��
	{
		time_tick->time = time_tick->time_now - time_tick->time_now;
	}else
	{
		time_tick->time = time_tick->time_now - time_tick->time_last;		
	}
	time_tick->time_last = time_tick->time_now;
	//�м�������ʱ��
	time_tick->total_time_ms += time_tick->time;
	time_tick->total_time_s = time_tick->total_time_ms * 0.001f;
}
/*
** Descriptions: ���ʱ��ṹ��
** Input: 
**				MyTimeTick *: 		ʱ��ṹ���ָ��
**				flag:  ѡ����յ�����
**								flag = 1  ���ȫ��
**								flag = 2	�����ʱ��(��ֹ��������ʱ�ļ�ʱ��1������)
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
** Descriptions: ���κ���
** Input: NULL
** Output: NULL
*/
//����waterlin(��������)����field��Χ���ڵ���ֵ
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
** Descriptions: ADC���ݲɼ����˲�
** Input: NULL
** Output: ���ݲɼ�ֵ
*/
void Get_ADC_Value(void)
{
		static uint32_t *buff1 = uhADC1ConvertedValue; //�����ܵ���adc����
		static uint32_t *buff2 = uhADC2ConvertedValue; //�����ܵ�ѹHadc����
		static uint32_t *buff3 = uhADC3ConvertedValue; //�����ܵ�ѹLadc����

		//�ܵ���adc���ݴ���
		float sum1 = 0;
		
		for(uint8_t i = 0;i < 10;i++)
		{
			sum1 += buff1[i];
		}
	
		sum1 =  sum1 / 10;
		
		if(current_get.Current_Offset_num > 200)
		{
			current_get.CurrentCalculat = (sum1 * (0.00080566f) - 2.50f) * 25.0f -
																		 current_get.Current_Offset;
		}
		else
		{
			current_get.Current_Offset_num++;
			
			current_get.CurrentCalculat = (sum1 * (0.00080566f) - 2.50f) * 25.0f;
			
			if(current_get.Current_Offset_num > 50)
			{
				current_get.Current_Offset += current_get.CurrentCalculat - current_get.Current_Referee;
			}
			if(current_get.Current_Offset_num > 200)
			{
				current_get.Current_Offset = current_get.Current_Offset/150.0f;
			}
		}
		
		//���ݵ�ѹADC���ݴ���,������ֵ����
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
** Descriptions: ���ʼ���
** Input: NULL
** Output: NULL
*/

void Power_Calculate(void)
{
		limit.Power_Referee =  Robot.Chassis_Power.chassis_Power;
		if(limit.Volt_Referee != 0)//��ֹ����ϵͳʧЧ
		{
			limit.Power_Calculat = current_get.CurrentCalculat * limit.Volt_Referee;

		}else
		{
			limit.Power_Calculat = current_get.CurrentCalculat * 23.3f;		
		}
		
		limit.Power_Chassis_Calculat = current_get.Chassis_Current * 23.3f;//current_get.Capacitance_Volt;
		
		VAL_LIMIT(limit.Power_Calculat,0,300);
		VAL_LIMIT(limit.Power_Chassis_Calculat,0,300);

}

/*
** Descriptions: ������������
** Input: NULL
** Output: NULL
*/
void Remain_Power_Calculate(void)
{
		/*�ɼ�ʱ��*/
		MyTime_statistics(&time_for_RP);
	
		/*�����������*/
		if(limit.PowerRemain_Calculat_Last < 60 || limit.Power_Calculat > 80)
		{
			limit.PowerRemain_Calculat -= (limit.Power_Calculat - 80) * time_for_RP.time * 0.001f;
		}
		if(limit.Power_Calculat < 80 && limit.PowerRemain_Calculat_Last == 60)
		{
			limit.PowerRemain_Calculat = 60 - (limit.Power_Calculat - 80) * time_for_RP.time * 0.001f;
		}
		
		/*�в���ϵͳ�������ݣ���ʹ�ò���ϵͳ����*/
		limit.PowerRemain_Referee = Robot.Chassis_Power.Chassis_Power_buffer;
		if(limit.PowerRemain_Referee != limit.PowerRemain_Referee_last)
		{
			limit.PowerRemain_Calculat = limit.PowerRemain_Referee;
		}
		limit.PowerRemain_Referee_last = limit.PowerRemain_Referee;	
		
		/*�����ʱ��*/
		MyTime_memset(&time_for_RP,2);		
		
		/*�ָ���������*/
		VAL_LIMIT(limit.PowerRemain_Calculat, 0.0f, 60.0f);
}
/*
** Descriptions:���ݵ�ѹ����
** Input:waterline:ˮƽ�ߣ�filed����Χ
** Output:
** Note:���ε��ݵ�ѹ��waterline����+-field������
*/
void	Cap_Volt_Treatment(float waterline ,float filed)
{
		current_get.Capacitance_Volt = Shield(current_get.Capacitance_Volt, waterline,filed);
}

/*
*	��γ�����ò���ϵͳ���صĹ��ʺ���������
*
*/
void power_limit(float  Current_get[4])
{
		
		float total_current = 0;
	
		total_current = MyAbs(Current_get[0]) + MyAbs(Current_get[1])+\
										MyAbs(Current_get[2]) + MyAbs(Current_get[3]);
	
		/*��������*/
		limit.PowerRemain_Calculat_Next = limit.PowerRemain_Calculat;//Ԥ����������û�õ�
		
		if(limit.PowerRemain_Calculat_Next < Predict_RemainPower)
		{
			if(limit.PowerRemain_Calculat_Next <0) 
			{
				limit.PowerRemain_Calculat_Next = 0;
			}
			
			limit.PowerLimit = RatedCurrent;//ԭ����5000��Ч��������
			limit.PowerLimit = ((limit.PowerRemain_Calculat_Next * limit.PowerRemain_Calculat_Next) / 400) * limit.PowerLimit;		
			
			/*��������*/
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



/*ע�⣺���ְ�����һ������ͬʱ��24v����͵����ù��� */
void Super_Capacitance(float  Current_get[4])
{
		static float total_current = 0;
		static uint8_t flag = 0;
		//�����ܵ���
		total_current = MyAbs(moto_chassis_get[0].real_current) + MyAbs(moto_chassis_get[1].real_current)\
										+ MyAbs(moto_chassis_get[2].real_current) + MyAbs(moto_chassis_get[3].real_current);

		/*ADC���ݴ���*/
		Get_ADC_Value();
	
		/*���㹦��*/
		Power_Calculate();
	
		/*����ʣ������*/
		Remain_Power_Calculate();
		
		/*������ݵ�ѹ*/
		Cap_Volt_Treatment(MinVoltOFCap,Filed);
	
	
		/*�ȴ����̵���������ģ��ɼ�ƫ����*/
		if(/*current_get.Current_Offset_num < 200*/0)
		{
				printf("current:%f,power:%f,capVolt:%f,ChassisP:%f��ChassisI:%f,state2:%d,%d\n\r",current_get.CurrentCalculat,limit.Power_Calculat,current_get.Capacitance_Volt,limit.Power_Chassis_Calculat,current_get.Chassis_Current,state2,flag);

				return;
		}
		
		/*�ж�һ��״̬*/
		if (state1)
		{
			
			//ָ����ǰ����״̬
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

		/*����״̬�µĶ���ִ��*/
		static uint8_t discharge_flag = 0;
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
					
						if (current_get.Capacitance_Volt < MinVoltOFCap)//���ݵ�������
						{
								Normal_switch();
								//�����������
								power_limit(Current_get);
							
								if (limit.PowerRemain_Calculat > SoftLimitRP)
								{
										state1 = Judge;
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
						else if(limit.PowerRemain_Calculat < 20)//ʣ���������о�
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
		
//	 float *buff;
//	buff = &limit.Power_Calculat;
//	buff[1] = &total_current;
//	buff[2] = &Capacitance_Volt;
//	vcan_sendware((uint8_t *)buff,sizeof(buff));
			printf("power_referee:%f,power:%f,capVolt:%f,limit.PowerRemain_Calculat:%f,state2:%d,discharge_flag:%d,flag:%d\n\r",
			limit.Power_Referee,limit.Power_Calculat,current_get.Capacitance_Volt,limit.PowerRemain_Calculat,state2,discharge_flag,flag);
//	printf("current:%f\n",current_get.CurrentCalculat);
//	printf("power:%f,capVolt:%f,current:%f\n\r",limit.Power_Calculat,current_get.Capacitance_Volt,total_current);

//		printf("capVolt:%f\n\r",current_get.Capacitance_Volt);

}
