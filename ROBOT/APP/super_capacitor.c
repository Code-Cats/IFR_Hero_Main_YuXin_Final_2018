#include "super_capacitor.h"
#include "pwm.h"

extern u32 time_1ms_count;

extern Error_check_t Error_Check;
#define POWERLIMIT 120 	//120w��������
#define POWERBUFFER 60	//60J���ʻ���
#define POWERRELEASE	50	//��ʣ50jʱ�������������һͬ�ŵ磬�Ա�����ϵ��̲��費����ֿ�϶
#define POWERSAVING	20	//��ʣ20jʱ��ʼ���ݵ����ŵ磬���̶Ͽ��Ի�������

#define SUPERC_INPUT_POWER	50	//����50Wʱ��ʼ��磬���߼����Զ���	��
u8 stateeeeeee=0;

u8 Super_Capacitor_DealState=0;	//һ����״̬
u32 SuperC_OutputTime=0;	//��¼������δ�ظ����͹�������·ŵ�ʱ�䣬���ܹ���
u8 SuperC_Output_Enable=1;	//�����Ƿ��ܷŵ�
void Super_Capacitor_Task(float power,float powerbuffer)
{
	static u32 time_record_cut_C=0;	//�л�ʱ���¼	//Ŀ���Ǵ���һ��ͬʱ�����ʱ��	//��Ϊ������ֵ��һ�£����ܲ���Ҫ�����ʱ
	static u32 time_record_cut_J=0;	//�л�ʱ���¼	//Ŀ���Ǵ���һ��ͬʱ�����ʱ��	//��Ϊ������ֵ��һ�£����ܲ���Ҫ�����ʱ
	static u32 time_record_off_judge=0;	//�ϲ���ʱ���¼	//���ܶ�̫��ʱ�䣬���ʺ���ʱ������һ�������������лص��̹���
	
	if(Error_Check.statu[LOST_REFEREE]==0&&time_1ms_count>3000)	//����δlost
	{
		switch(Super_Capacitor_DealState)	//���̿ɿذ汾����Ӧ���ʿ���ҲҪ�ſ�
		{
			case 0:	//���̹���С���ɳ�磬ֹͣ�ŵ�	//�°�ĳ�罻���Զ�
			{
				if(SuperC_OutputTime>0)
				{
					if(time_1ms_count%2==0)
					{
						SuperC_OutputTime--;	//������ݷŵ����
					}
				}
				
				SUPERCAPACITOR_JUDGE=PWM_IO_ON;
				SUPERCAPACITOR_INPUT=PWM_IO_ON;
				
				if(time_1ms_count-time_record_cut_J>18)	//���ٹ�18ms������
				{
					SUPERCAPACITOR_OUTPUT=PWM_IO_OFF;
				}
				
				if(power>SUPERC_INPUT_POWER)
				{
					Super_Capacitor_DealState=1;	//�ص����̹��ʴ�״̬
				}
				if(powerbuffer<POWERRELEASE)
				{
					Super_Capacitor_DealState=2;	//����������ʼ��״̬
				}
				break;
			}
			case 1:	//���̹��ʴ�����������ֹͣ�ŵ�Ҳֹͣ���
			{
				SUPERCAPACITOR_JUDGE=PWM_IO_ON;
				SUPERCAPACITOR_INPUT=PWM_IO_OFF;
				if(time_1ms_count-time_record_cut_J>18)
				{
					SUPERCAPACITOR_OUTPUT=PWM_IO_OFF;
				}
				
				if(power<SUPERC_INPUT_POWER)
				{
					Super_Capacitor_DealState=0;	//�ص����̹���С״̬
				}
				if(powerbuffer<POWERRELEASE)
				{
					Super_Capacitor_DealState=2;	//����������ʼ��״̬
					time_record_cut_C=time_1ms_count;	//��¼��������ʱ�䣬����18ms��ſ��Թرյ���
				}
				break;
			}
			case 2:	//���̹��ʴ�������ʹ��һ���֣���ʼ�ŵ磬���̲���
			{
				SUPERCAPACITOR_JUDGE=PWM_IO_ON;
				if(SuperC_Output_Enable==1)
				{
					SUPERCAPACITOR_OUTPUT=PWM_IO_ON;
				}
				else if(time_1ms_count-time_record_cut_J>18)
				{
					SUPERCAPACITOR_OUTPUT=PWM_IO_OFF;
				}
				SUPERCAPACITOR_INPUT=PWM_IO_OFF;
				
				if(powerbuffer>POWERRELEASE)	//�����ظ�
				{
					Super_Capacitor_DealState=1;	//�ص����̹��ʴ�״̬
				}
				if(powerbuffer<POWERSAVING&&(time_1ms_count-time_record_cut_C)>18&&SuperC_Output_Enable==1)
				{
					Super_Capacitor_DealState=3;	//������̶Ͽ���������״̬
				}
				break;
			}
			case 3:	//���̹��ʴ������۽��þ�����ʼ�ŵ磬���̶Ͽ��Ի�������
			{
				SuperC_OutputTime++;
				SUPERCAPACITOR_JUDGE=PWM_IO_OFF;
				SUPERCAPACITOR_INPUT=PWM_IO_OFF;
				SUPERCAPACITOR_OUTPUT=PWM_IO_ON;
				if(powerbuffer>POWERRELEASE)	//ֱ��������������ָ������˳���״̬
				{
					time_record_cut_J=time_1ms_count;	//��¼���п���ʱ�䣬����18ms��ſ��Թص���
					Super_Capacitor_DealState=2;	//������̹���״̬
				}
				if(SuperC_Output_Enable==0)	//���ݲ�������
				{
					time_record_cut_J=time_1ms_count;	//��¼���п���ʱ�䣬����18ms��ſ��Թص���
					Super_Capacitor_DealState=2;	//���ݿ��þ���ǿ�ƽ�����̹���״̬
				}
				break;
			}
	//		case 4:	//���̹��ʴ������۽��þ�����ʼ�ŵ磬���̶Ͽ��Ի�������,���̹��ʽϵͿ���ͬʱ���
	//		{
	//			SUPERCAPACITOR_JUDGE=PWM_IO_OFF;
	////			SUPERCAPACITOR_INPUT=PWM_IO_OFF;
	//			SUPERCAPACITOR_OUTPUT=PWM_IO_ON;
	//			break;
	//		}
		}
		
//		if(power>SUPERC_INPUT_POWER)	//���ݳ�絥�����ƣ�������Ϊ�˷�ֹ���ݷŵ���ȣ�������ʱʱ�̿��ʵ��ĳ��
//		{
//			SUPERCAPACITOR_INPUT=PWM_IO_OFF;
//		}
//		else
//		{
//			SUPERCAPACITOR_INPUT=PWM_IO_ON;
//		}
		
//////////////////		if(SuperC_OutputTime>6300)	//����6000ms��6s������Ƶ��1000HZ
//////////////////		{
//////////////////			SuperC_Output_Enable=0;
//////////////////		}
//////////////////		else if(SuperC_OutputTime<3000)
//////////////////		{
//////////////////			SuperC_Output_Enable=1;
//////////////////		}

	}
	else	//����lost
	{
		SuperC_Output_Enable=0;	//���ݲ���ʹ��
		SUPERCAPACITOR_JUDGE=PWM_IO_ON;
		SUPERCAPACITOR_INPUT=PWM_IO_OFF;
		SUPERCAPACITOR_OUTPUT=PWM_IO_ON;
	}
	
	SuperC_Output_Enable=0;	//����
	
//	if(powerbuffer<POWERBUFFER)	//20	//�����̿ɿصİ汾
//	{
////		time_record_off_judge=time_1ms_count;
//		SUPERCAPACITOR_JUDGE=PWM_IO_ON;
//		SUPERCAPACITOR_OUTPUT=PWM_IO_ON;
//		SUPERCAPACITOR_INPUT=0;
//		stateeeeeee=1;
//	}
//	else if(powerbuffer==POWERBUFFER&&power>50)
//	{
//		SUPERCAPACITOR_JUDGE=PWM_IO_ON;
//		SUPERCAPACITOR_OUTPUT=0;
//		SUPERCAPACITOR_INPUT=0;
//		stateeeeeee=2;
//	}
//	else if(power<50)
//	{
//		SUPERCAPACITOR_JUDGE=PWM_IO_ON;
//		SUPERCAPACITOR_OUTPUT=0;
//		SUPERCAPACITOR_INPUT=PWM_IO_ON;
//		stateeeeeee=3;
//	}
//	
//	if(Error_Check.statu[LOST_REFEREE]==1)	//����lost
//	{
//		SUPERCAPACITOR_JUDGE=PWM_IO_ON;
//		SUPERCAPACITOR_OUTPUT=PWM_IO_ON;
//		SUPERCAPACITOR_INPUT=0;
//		stateeeeeee=4;
//	}
	
//	if(powerbuffer<POWERBUFFER)	//20	//�������̿ɿصİ汾
//	{
////		time_record_off_judge=time_1ms_count;
//		SUPERCAPACITOR_JUDGE=PWM_IO_ON;
//		SUPERCAPACITOR_OUTPUT=PWM_IO_ON;
//		SUPERCAPACITOR_INPUT=0;
//		stateeeeeee=1;
//	}
//	else if(powerbuffer==POWERBUFFER&&power>50)
//	{
//		SUPERCAPACITOR_JUDGE=PWM_IO_ON;
//		SUPERCAPACITOR_OUTPUT=0;
//		SUPERCAPACITOR_INPUT=0;
//		stateeeeeee=2;
//	}
//	else if(power<50)
//	{
//		SUPERCAPACITOR_JUDGE=PWM_IO_ON;
//		SUPERCAPACITOR_OUTPUT=0;
//		SUPERCAPACITOR_INPUT=PWM_IO_ON;
//		stateeeeeee=3;
//	}
//	
//	if(Error_Check.statu[LOST_REFEREE]==1)	//����lost
//	{
//		SUPERCAPACITOR_JUDGE=PWM_IO_ON;
//		SUPERCAPACITOR_OUTPUT=PWM_IO_ON;
//		SUPERCAPACITOR_INPUT=0;
//		stateeeeeee=4;
//	}
}



