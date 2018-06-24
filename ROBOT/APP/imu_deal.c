#include "imu_deal.h"

extern GYRO_DATA Gyro_Data;

u8 Imu_CaliOK_State=0;	//未校准
int Imu_Calibration(int angvel[])	//和陀螺仪角速度更新频率一致
{
	static u16 cali_count=0;
	static int imu_calidata[3]={0};
	static int offset[3]={0};
	
	if(Imu_CaliOK_State==0)
	{
		imu_calidata[0]+=angvel[0]+offset[0];
		imu_calidata[1]+=angvel[1]+offset[1];
		imu_calidata[2]+=angvel[2]+offset[2];
		cali_count++;
		if(cali_count==20)	//采样20次
		{
			offset[0]=imu_calidata[0]/20;
			offset[1]=imu_calidata[1]/20;
			offset[2]=imu_calidata[2]/20;
			Imu_CaliOK_State=1;
		}
	}
}


