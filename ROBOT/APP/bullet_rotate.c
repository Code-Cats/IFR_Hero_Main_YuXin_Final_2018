#include "bullet_rotate.h"
#include "math.h"
#include "pid.h"

extern Error_check_t Error_Check;

BULLETROTATE_DATA BulletRotate_Data=BULLETROTATE_DATA_DEFAULT;	//取弹旋转电机数据

PID_GENERAL   PID_BulletRotate_Position=PID_BULLETROTATE_POSITION_DEFAULT;
PID_GENERAL   PID_BulletRotate_Speed=PID_BULLETROTATE_SPEED_DEFAULT;

extern u32 time_1ms_count;
extern ViceControlDataTypeDef ViceControlData;

extern u8 BulletRotate_Cali_Statu;	//标定状态
void BulletRotate_Task(void)
{
//	BulletRotate_Data.tarP=(s16)(12*(RC_Ctl.rc.ch3-1024)/600.0);
	if(GetWorkState()!=CALI_STATE&&BulletRotate_Cali_Statu==2)	//标定状态
	{
			BulletRotate_Data.tarV=PID_General(BulletRotate_Data.tarP,BulletRotate_Data.fdbP,&PID_BulletRotate_Position);
	}
//	BulletRotate_Data.tarV=PID_General(BulletRotate_Data.tarP,BulletRotate_Data.fdbP,&PID_BulletRotate_Position);
	
	BulletRotate_Data.output=PID_General(BulletRotate_Data.tarV,BulletRotate_Data.fdbV,&PID_BulletRotate_Speed);
	
}


u8 BulletRotate_OffSetInit(void)	//初始OFFSET	//在cali(output为0时)状态下进行标定
{
	static u8 offset_statu=0;
	static s32 fdbp_record[2]={0};
	static u8 record_i=0;
	
	if(offset_statu==0)	//标定状态
	{
		if(Error_Check.statu[LOST_BULLETROTATE1]==0&&time_1ms_count%4==0)	//还在
		{
			fdbp_record[0]=BulletRotate_Data.fdbP;
			if(fdbp_record[0]==fdbp_record[1])
			{
				record_i++;
			}
			else
			{
				record_i=0;
			}
			fdbp_record[1]=fdbp_record[0];	//迭代
		}
		else if(Error_Check.statu[LOST_BULLETROTATE1]==1)
		{
			record_i=0;
		}
		
		if(record_i>10)
		{
			offset_statu=1;
			BulletRotate_Data.offsetP=BulletRotate_Data.fdbP;
		}
	}
		
	return offset_statu;
}

extern LIFT_POSITION_ENCODER bulletrotate_position_encoder;	//清零圈数
u8 BulletRotate_Cali_Statu=0;
u8 BulletRotate_Cali(void)	//初始位置标定	//在有输出状态下进行标定
{
	u8 cali_state=0;
	if(Error_Check.statu[LOST_BULLETROTATE1]==0)	//还在
	{
		switch(BulletRotate_Cali_Statu)
		{
			case 0:
			{
				static u16 time_count=0;
				time_count++;	//1ms累加一次
				BulletRotate_Data.tarV=-2000;
				if(time_count>400)	//这个延时目的是确保电机已经启动
				{
					time_count=0;
					BulletRotate_Data.tarV=-1500;
					BulletRotate_Cali_Statu=1;
				}
				break;
			}
			case 1:
			{
				static u16 time_count=0;
				
				BulletRotate_Data.tarV=-1500;
				if(abs(BulletRotate_Data.fdbV)<20)
				{
					time_count++;	//1ms累加一次
					bulletrotate_position_encoder.turns=0;	//清零圈数
				}
				else
				{
					time_count=0;
				}
				
				if(time_count>500)	//标定完成
				{
					time_count=0;
					BulletRotate_Cali_Statu=2;
					//清零圈数
					BulletRotate_Data.offsetP=BulletRotate_Data.fdbP;
				}
				break;
			}
			case 2:
			{
				cali_state=1;
				break;
			}
		}
	}
	else if(GetWorkState()==CALI_STATE)//丢失反馈
	{
//		static u16 time_count=0;
//		time_count++;	//1ms累加一次
//		if(time_count>)
	}
	return cali_state;
}