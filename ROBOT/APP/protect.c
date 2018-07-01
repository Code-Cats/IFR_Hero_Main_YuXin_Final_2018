#include "protect.h"
#include "control.h"
/*
该文件用途：提供传感器冗余算法，传感器切换，以及车体实时状态检测，保护状态切换
预定义功能：
1.云台姿态，底盘姿态多来源冗余算法
2.车体误操作检测及有效制止
3.车体各模块在线检测检错
4.防疯防抢防盗
5.待续...
增加一个记录帧率、周期的运算？
*/
extern GYRO_DATA Gyro_Data;

#define LOST_THRESHOLD 5

Error_check_t Error_Check={LOST_CYCLE,{0},{0}};

u8 Error_check_workstate=1;

s16 t_error_record=0;
void LostCountAdd(u16* lostcount)	//无需改变
{
	if(*lostcount<0xFFFE)
	(*lostcount)++;
}

void LostCountFeed(u16* lostcount)	//无需改变
{
	*lostcount=0;
}

u8 LostCountCheck(u16 lostcount,u8* statu,const u16 cycle)	//无需改变
{
	if(lostcount>LOST_THRESHOLD*cycle)
		*statu=1;
	else
		*statu=0;
	return *statu;
}

s32 test_error_Satrt=0;
void Check_Task(void)
{
	for(int i=0;i<LOST_TYPE_NUM;i++)	//无需改变
	{
		LostCountAdd(&Error_Check.count[i]);
		LostCountCheck(Error_Check.count[i],&Error_Check.statu[i],Error_Check.cycle[i]);
	}
	
	
	if(Error_check_workstate==1)	//工作状态
	{
//		if(Error_Check.statu[LOST_IMU]==1)
//		{
//			test_error_Satrt=1;
//			t_error_record=LOST_IMU;
//			SetWorkState(ERROR_STATE);
//		}
//		
//		for(int i=5;i<LOST_TYPE_NUM-2;i++)	//电机比控更重要
//		{
//			if(Error_Check.statu[i]==1)
//			{
//				test_error_Satrt=-1;
//				t_error_record=i;
//				SetWorkState(ERROR_STATE);
//			}
//				
//		}

	}
	
	
////////////////////	if(Error_Check.statu[LOST_DBUS]==1)
////////////////////	{
////////////////////		if(GetWorkState()==CHECK_STATE)
////////////////////		{
////////////////////			SetWorkState(LOST_STATE);	//启动时没有遥控信号的选择
////////////////////		}
////////////////////		else
////////////////////		{
////////////////////			SetWorkState(PROTECT_STATE);
////////////////////		}
////////////////////		
////////////////////	}
	
	if(RC_Ctl.key.v_h!=0||RC_Ctl.key.v_l!=0||abs(RC_Ctl.mouse.x)>3)
	{
		Error_check_workstate=0;
	}

}

u8 IMU_Check_Useless_State=0;	//陀螺仪失效检测位	//临时设置成1
void IMU_Check_Useless(void)	//陀螺仪检测失效
{
	static u16 IMU_check_useless_count=0;
	if(abs(Gyro_Data.angle[0])<0.01f&&abs(Gyro_Data.angle[1])<0.01f&&abs(Gyro_Data.angle[2])<0.01f)
	{
		IMU_check_useless_count++;
		if(IMU_check_useless_count>200)	//200ms数据异常，则认为陀螺仪疯
		{
			IMU_Check_Useless_State=1;
		}
		
	}
	else
	{
		IMU_Check_Useless_State=0;	//认为陀螺仪恢复
		IMU_check_useless_count=0;
	}
}


