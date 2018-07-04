#include "bullet_rotate.h"
#include "math.h"
#include "pid.h"

extern Error_check_t Error_Check;

BULLETROTATE_DATA BulletRotate_Data=BULLETROTATE_DATA_DEFAULT;	//取弹旋转电机数据

PID_GENERAL   PID_BulletRotate_Position=PID_BULLETROTATE_POSITION_DEFAULT;
PID_GENERAL   PID_BulletRotate_Speed=PID_BULLETROTATE_SPEED_DEFAULT;

extern u32 time_1ms_count;
extern ViceControlDataTypeDef ViceControlData;

void BulletRotate_Task(void)
{
//	BulletRotate_Data.tarP=(s16)(12*(RC_Ctl.rc.ch3-1024)/600.0);
	
	BulletRotate_Data.tarV=PID_General(BulletRotate_Data.tarP,BulletRotate_Data.fdbP,&PID_BulletRotate_Position);
	
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

