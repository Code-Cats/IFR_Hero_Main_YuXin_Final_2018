#include "imu_deal.h"

extern GYRO_DATA Gyro_Data;

u8 Imu_CaliOK_State=0;	//未校准
void Imu_Offset(float angvel[])	//和陀螺仪角速度更新频率一致
{
	static u16 cali_count=0;
	static float imu_calidata[3]={0};
	static float offset[3]={0};
	
	if(Imu_CaliOK_State==0)
	{
		imu_calidata[0]+=angvel[0];//+offset[0];
		imu_calidata[1]+=angvel[1];//+offset[1];
		imu_calidata[2]+=angvel[2];//+offset[2];
		cali_count++;
		if(cali_count==100)	//采样100次
		{
			offset[0]=imu_calidata[0]/100;
			offset[1]=imu_calidata[1]/100;
			offset[2]=imu_calidata[2]/100;
			Imu_CaliOK_State=1;
		}
	}
	else if(Imu_CaliOK_State==1)
	{
		angvel[0]-=offset[0];
		angvel[1]-=offset[1];
		angvel[2]-=offset[2];
	}
}

u8 Imu_Cali_State(void)
{
	return Imu_CaliOK_State;
}
