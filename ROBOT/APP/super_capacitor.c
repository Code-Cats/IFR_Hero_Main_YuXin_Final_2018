#include "super_capacitor.h"
#include "pwm.h"

extern u32 time_1ms_count;

extern Error_check_t Error_Check;
#define POWERLIMIT 120 	//120w功率限制
#define POWERBUFFER 60	//60J功率缓冲
#define POWERRELEASE	50	//还剩50j时开启电容与底盘一同放电，以便后续断底盘步骤不会出现空隙
#define POWERSAVING	20	//还剩20j时开始电容单独放电，底盘断开以积蓄能量

#define SUPERC_INPUT_POWER	50	//低于50W时开始充电，该逻辑可以独立	？
u8 stateeeeeee=0;

u8 Super_Capacitor_DealState=0;	//一般充电状态
u32 SuperC_OutputTime=0;	//记录电容在未回复到低功率情况下放电时间，不能过长
u8 SuperC_Output_Enable=1;	//电容是否能放电
void Super_Capacitor_Task(float power,float powerbuffer)
{
	static u32 time_record_cut_C=0;	//切换时间记录	//目的是存在一个同时供电的时间	//因为触发阈值不一致，可能不需要这个延时
	static u32 time_record_cut_J=0;	//切换时间记录	//目的是存在一个同时供电的时间	//因为触发阈值不一致，可能不需要这个延时
	static u32 time_record_off_judge=0;	//断裁判时间记录	//不能断太长时间，功率和延时有任意一个符合条件就切回底盘供电
	
	if(Error_Check.statu[LOST_REFEREE]==0&&time_1ms_count>3000)	//裁判未lost
	{
		switch(Super_Capacitor_DealState)	//底盘可控版本，相应功率控制也要放宽
		{
			case 0:	//底盘功率小，可充电，停止放电	//新版的充电交给自动
			{
				if(SuperC_OutputTime>0)
				{
					if(time_1ms_count%2==0)
					{
						SuperC_OutputTime--;	//清零电容放电计数
					}
				}
				
				SUPERCAPACITOR_JUDGE=PWM_IO_ON;
				SUPERCAPACITOR_INPUT=PWM_IO_ON;
				
				if(time_1ms_count-time_record_cut_J>18)	//至少过18ms开电容
				{
					SUPERCAPACITOR_OUTPUT=PWM_IO_OFF;
				}
				
				if(power>SUPERC_INPUT_POWER)
				{
					Super_Capacitor_DealState=1;	//回到底盘功率大状态
				}
				if(powerbuffer<POWERRELEASE)
				{
					Super_Capacitor_DealState=2;	//进入能量开始用状态
				}
				break;
			}
			case 1:	//底盘功率大，能量槽满。停止放电也停止充电
			{
				SUPERCAPACITOR_JUDGE=PWM_IO_ON;
				SUPERCAPACITOR_INPUT=PWM_IO_OFF;
				if(time_1ms_count-time_record_cut_J>18)
				{
					SUPERCAPACITOR_OUTPUT=PWM_IO_OFF;
				}
				
				if(power<SUPERC_INPUT_POWER)
				{
					Super_Capacitor_DealState=0;	//回到底盘功率小状态
				}
				if(powerbuffer<POWERRELEASE)
				{
					Super_Capacitor_DealState=2;	//进入能量开始用状态
					time_record_cut_C=time_1ms_count;	//记录开启电容时间，至少18ms后才可以关闭底盘
				}
				break;
			}
			case 2:	//底盘功率大，能量槽使用一部分，开始放电，底盘不断
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
				
				if(powerbuffer>POWERRELEASE)	//能量回复
				{
					Super_Capacitor_DealState=1;	//回到底盘功率大状态
				}
				if(powerbuffer<POWERSAVING&&(time_1ms_count-time_record_cut_C)>18&&SuperC_Output_Enable==1)
				{
					Super_Capacitor_DealState=3;	//进入底盘断开积蓄能量状态
				}
				break;
			}
			case 3:	//底盘功率大，能量槽将用尽，开始放电，底盘断开以积蓄能量
			{
				SuperC_OutputTime++;
				SUPERCAPACITOR_JUDGE=PWM_IO_OFF;
				SUPERCAPACITOR_INPUT=PWM_IO_OFF;
				SUPERCAPACITOR_OUTPUT=PWM_IO_ON;
				if(powerbuffer>POWERRELEASE)	//直到底盘能量缓冲恢复满才退出该状态
				{
					time_record_cut_J=time_1ms_count;	//记录裁判开启时间，至少18ms后才可以关电容
					Super_Capacitor_DealState=2;	//进入底盘供电状态
				}
				if(SuperC_Output_Enable==0)	//电容不能用了
				{
					time_record_cut_J=time_1ms_count;	//记录裁判开启时间，至少18ms后才可以关电容
					Super_Capacitor_DealState=2;	//电容快用尽，强制进入底盘供电状态
				}
				break;
			}
	//		case 4:	//底盘功率大，能量槽将用尽，开始放电，底盘断开以积蓄能量,底盘功率较低可以同时充电
	//		{
	//			SUPERCAPACITOR_JUDGE=PWM_IO_OFF;
	////			SUPERCAPACITOR_INPUT=PWM_IO_OFF;
	//			SUPERCAPACITOR_OUTPUT=PWM_IO_ON;
	//			break;
	//		}
		}
		
//		if(power>SUPERC_INPUT_POWER)	//电容充电单独控制，这里是为了防止电容放电过度，而进行时时刻刻适当的充电
//		{
//			SUPERCAPACITOR_INPUT=PWM_IO_OFF;
//		}
//		else
//		{
//			SUPERCAPACITOR_INPUT=PWM_IO_ON;
//		}
		
//////////////////		if(SuperC_OutputTime>6300)	//大于6000ms即6s，处理频率1000HZ
//////////////////		{
//////////////////			SuperC_Output_Enable=0;
//////////////////		}
//////////////////		else if(SuperC_OutputTime<3000)
//////////////////		{
//////////////////			SuperC_Output_Enable=1;
//////////////////		}

	}
	else	//裁判lost
	{
		SuperC_Output_Enable=0;	//电容不能使用
		SUPERCAPACITOR_JUDGE=PWM_IO_ON;
		SUPERCAPACITOR_INPUT=PWM_IO_OFF;
		SUPERCAPACITOR_OUTPUT=PWM_IO_ON;
	}
	
	SuperC_Output_Enable=0;	//试验
	
//	if(powerbuffer<POWERBUFFER)	//20	//带底盘可控的版本
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
//	if(Error_Check.statu[LOST_REFEREE]==1)	//裁判lost
//	{
//		SUPERCAPACITOR_JUDGE=PWM_IO_ON;
//		SUPERCAPACITOR_OUTPUT=PWM_IO_ON;
//		SUPERCAPACITOR_INPUT=0;
//		stateeeeeee=4;
//	}
	
//	if(powerbuffer<POWERBUFFER)	//20	//不带底盘可控的版本
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
//	if(Error_Check.statu[LOST_REFEREE]==1)	//裁判lost
//	{
//		SUPERCAPACITOR_JUDGE=PWM_IO_ON;
//		SUPERCAPACITOR_OUTPUT=PWM_IO_ON;
//		SUPERCAPACITOR_INPUT=0;
//		stateeeeeee=4;
//	}
}



