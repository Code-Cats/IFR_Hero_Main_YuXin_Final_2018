#include "vision.h"
//#include "arm_math.h"
#include "math.h"
//PITCH_GYRO_INIT 2720

extern YUN_MOTOR_DATA 			yunMotorData;
extern GYRO_DATA Gyro_Data;
extern VisionDataTypeDef	VisionData;
extern GYRO_DATA Gyro_Data;	//融合用

float t_yaw_error=0;	//临时测试

float t_gravity_ballistic_set_angel=0;
	
float Pixel_to_angle(s16 pix_error)	//将像素误差转换成角度
{
	float angel_error=0;
//	angel_error=0.036f*pix_error;
//	t_yaw_angel_error=atan(pix_error/938.2f)*57.3f;
angel_error=atan(pix_error/1855.2f)*57.3f;	//arm_atan_f32为DSP
	t_yaw_error=angel_error;
	return angel_error;
}

#define CAMERA_D	1855	//相机镜片到感光片等效像素量
float Pixel_V_to_angle_V(s16 pix_v,s16 pix_error)	//从最原始的数据进行计算可以减少单片机浮点运算的精度丢失（误差增加）
{
	int camera_d_2=CAMERA_D*CAMERA_D;	//距离平方
	int r_2=camera_d_2+pix_error*pix_error;	//等效半径平方
	float cos_angel_2=(float)camera_d_2/(float)r_2;
	float angel_v=0;
	angel_v=pix_v*cos_angel_2/(float)CAMERA_D;
	angel_v=angel_v*57.3f;//*2*PI;	//进行还原处理
	
	return angel_v;
}

#define VISION_TARX 1020//580	//左上原点	640
#define VISION_TARY	540//560//360//410//440	//左上原点	480
void Vision_Task(float* yaw_tarP,float* pitch_tarP)	//处理目标角度
{
	if(Error_Check.statu[LOST_VISION]==1)	VisionData.armor_sign=0;
	//t_yaw_angel_v=Pixel_V_to_angle_V(VisionData.pix_x_v,(s16)(VisionData.error_x-VISION_TARX));
//	t_target_v=t_yaw_angel_v+Gyro_Data
	VisionData.vision_control_state=VisionData.armor_sign;
	
	if(VisionData.armor_sign==1)
	{
//		t_yaw_error=Pixel_to_angle((s16)(VisionData.error_x-VISION_TARX))*10;
//		t_pitch_error=Pixel_to_angle((s16)(VisionData.error_y-VISION_TARY))*8192/360;
		
//		t_yaw_error=(float)Gyro_Data.angle[2]*10-Pixel_to_angle((s16)(VisionData.error_x-VISION_TARX))*10;
//		t_pitch_error=(float)yunMotorData.pitch_fdbP+Pixel_to_angle((s16)(VisionData.error_y-VISION_TARY))*8192/360;
		*yaw_tarP=(float)Gyro_Data.angle[2]*10+Pixel_to_angle((s16)(VisionData.tar_x-VISION_TARX))*10;
		*pitch_tarP=(float)yunMotorData.pitch_fdbP-Pixel_to_angle((s16)(VisionData.tar_y-VISION_TARY))*8192/360;
//		t_gravity_ballistic_set_angel=Gravity_Ballistic_Set(pitch_tarP,(float)(VisionData.armor_dis/10.0f+0.2));	//重力补偿
		Tar_Move_Set(yaw_tarP,(float)(VisionData.armor_dis/10.0f+0.2f),VisionData.angel_x_v);
	}
	
}

#define SHOOT_V	14	//14M/s
#define SHOOT_V_2 (SHOOT_V*SHOOT_V)
#define G	9.8f	//重力加速度
float Gravity_Ballistic_Set(float* pitch_tarP,float dis_m)	//重力补偿坐标系中，向下为正
{
	static float tar_angle_rad_fliter=0;
	float tar_angle_rad=(*pitch_tarP-PITCH_GYRO_INIT)*0.000767f;	//弧度制简化计算2pi/8192
	tar_angle_rad_fliter=0.9f*tar_angle_rad_fliter+0.1f*tar_angle_rad;
	float sin_tar_angle=sin(tar_angle_rad);
	float gravity_ballistic_angle_rad=0;	//补偿角 弧度制
	float gravity_ballistic_angle=0;	//补偿角 角度制
	gravity_ballistic_angle_rad=0.5f*(-asin((G*dis_m*(1-sin_tar_angle*sin_tar_angle)-sin_tar_angle*SHOOT_V_2)/SHOOT_V_2)+tar_angle_rad);
	gravity_ballistic_angle=gravity_ballistic_angle_rad*57.3f;

	*pitch_tarP=PITCH_GYRO_INIT-gravity_ballistic_angle*8192.0f/360;

	return gravity_ballistic_angle;
}

void Tar_Move_Set(float* yaw_tarP,float dis_m,float tar_v)
{
	static float tar_v_fliter=0;
	if(abs(tar_v)<8)	tar_v=0;
	
	//tar_v_fliter=0.9f*tar_v_fliter+0.1f*tar_v;
	if (dis_m>4)
	{
		dis_m=4;
	}
	float shoot_delay=dis_m/SHOOT_V;	//以秒为单位
	float pre_angle=tar_v*shoot_delay;
	if(pre_angle>10)	pre_angle=10;
	if(pre_angle<-10)	pre_angle=-10;
	*yaw_tarP-=pre_angle;
}
