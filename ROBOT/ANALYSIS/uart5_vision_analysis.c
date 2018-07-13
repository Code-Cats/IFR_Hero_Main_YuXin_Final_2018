#include "uart5_vision_analysis.h"
#include "vision.h"
//#include "stm32f4xx.h"

extern YUN_MOTOR_DATA 			yunMotorData;
extern GYRO_DATA Gyro_Data;

VisionDataTypeDef	VisionData={0};
VisionReceiveDataTypeDef VisionReceiveData={0};

#define DATA_LEN 10
u32 t_vision_count=0;
void VisionData_Receive(u8 data)	//从主板传过来的数据解析（主副板通用）
{
//	LostCountFeed(&Error_Check.count[LOST_VICEBOARD]);
	if(data==0x5A&&VisionReceiveData.headOK_state==0)
	{
		VisionReceiveData.valid_state=0;	//数据接受期间不进行数据解析
		VisionReceiveData.headOK_state=1;	
		VisionReceiveData.count=0;	//重置count
	}
	
	if(VisionReceiveData.headOK_state==1)	//帧头已找到
	{
		VisionReceiveData.databuffer[VisionReceiveData.count]=data;
		VisionReceiveData.count++;
		if((data==0xA5&&VisionReceiveData.count!=DATA_LEN)||(VisionReceiveData.count>DATA_LEN))	//失效
		{
			VisionReceiveData.valid_state=0;
			VisionReceiveData.headOK_state=0;
			VisionReceiveData.count=0;	//重置count
		}
		else if(data==0xA5&&VisionReceiveData.count==DATA_LEN)
		{
			VisionReceiveData.valid_state=1;
			VisionReceiveData.headOK_state=0;
			VisionReceiveData.count=0;	//重置count
		}
	}
	t_vision_count++;
	//////////////////////////////这里放数据解析函数-->解析为真实数据
	if(VisionReceiveData.valid_state==1)	//数据正常有效
	{
//		t_vision_count++;
		LostCountFeed(&Error_Check.count[LOST_VISION]);
		VisionData_Deal(VisionReceiveData.databuffer);
	}
	
}




#define VISION_TARX 1020

void VisionData_Deal(volatile u8 *pData)	//传感器数据在除了帧头的第1帧
{
	VisionData.armor_sign=*(pData+1)>>(7)&0x01;
	VisionData.armor_type=*(pData+1)>>(6)&0x01;
	VisionData.armor_dis=*(pData+2);
	VisionData.tar_x=*(pData+3)<<8|*(pData+4);
	VisionData.tar_y=*(pData+5)<<8|*(pData+6);
	VisionData.pix_x_v=*(pData+7)<<8|*(pData+8);
	
	
//	if(RC_Ctl.rc.switch_right==RC_SWITCH_UP&&GetWorkState()==NORMAL_STATE )	//放在中断中运行
//	{
		Vision_Task(&yunMotorData.yaw_tarP,&yunMotorData.pitch_tarP);	//控制键位集成再内部
//	}
}
