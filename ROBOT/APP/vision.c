#include "vision.h"

extern YUN_MOTOR_DATA 			yunMotorData;
extern GYRO_DATA Gyro_Data;
extern VisionDataTypeDef	VisionData;

float Pixel_to_angle(s16 pix_error)	//将像素误差转换成角度
{
	float angel_error=0;
	angel_error=0.04f*pix_error;
	return angel_error;
}



#define VISION_TARX 640
#define VISION_TARY 480
void Vision_Task(float* yaw_tarP,float* pitch_tarP)	//处理目标角度
{
	if(VisionData.armor_sign==1)
	{
//		t_yaw_error=Pixel_to_angle((s16)(VisionData.error_x-VISION_TARX))*10;
//		t_pitch_error=Pixel_to_angle((s16)(VisionData.error_y-VISION_TARY))*8192/360;
		
//		t_yaw_error=(float)Gyro_Data.angle[2]*10-Pixel_to_angle((s16)(VisionData.error_x-VISION_TARX))*10;
//		t_pitch_error=(float)yunMotorData.pitch_fdbP+Pixel_to_angle((s16)(VisionData.error_y-VISION_TARY))*8192/360;
		*yaw_tarP=(float)Gyro_Data.angle[2]*10-Pixel_to_angle((s16)(VisionData.error_x-VISION_TARX))*10;
		*pitch_tarP=(float)yunMotorData.pitch_fdbP+Pixel_to_angle((s16)(VisionData.error_y-VISION_TARY))*8192/360;
	}
}
