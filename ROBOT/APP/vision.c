#include "vision.h"
//#include "arm_math.h"
#include "math.h"
//PITCH_GYRO_INIT 2720

extern YUN_MOTOR_DATA 			yunMotorData;
extern GYRO_DATA Gyro_Data;
extern VisionDataTypeDef	VisionData;
extern GYRO_DATA Gyro_Data;	//�ں���

float t_yaw_error=0;	//��ʱ����

float t_gravity_ballistic_set_angel=0;
	
float Pixel_to_angle(s16 pix_error)	//���������ת���ɽǶ�
{
	float angel_error=0;
//	angel_error=0.036f*pix_error;
//	t_yaw_angel_error=atan(pix_error/938.2f)*57.3f;
angel_error=atan(pix_error/1855.2f)*57.3f;	//arm_atan_f32ΪDSP
	t_yaw_error=angel_error;
	return angel_error;
}

#define CAMERA_D	1855	//�����Ƭ���й�Ƭ��Ч������
float Pixel_V_to_angle_V(s16 pix_v,s16 pix_error)	//����ԭʼ�����ݽ��м�����Լ��ٵ�Ƭ����������ľ��ȶ�ʧ��������ӣ�
{
	int camera_d_2=CAMERA_D*CAMERA_D;	//����ƽ��
	int r_2=camera_d_2+pix_error*pix_error;	//��Ч�뾶ƽ��
	float cos_angel_2=(float)camera_d_2/(float)r_2;
	float angel_v=0;
	angel_v=pix_v*cos_angel_2/(float)CAMERA_D;
	angel_v=angel_v*57.3f;//*2*PI;	//���л�ԭ����
	
	return angel_v;
}


//#define PITCH_INIT         3098	//2018.7.10	//��λ��
#define YUN_DOWN_VALUELIMIT 2765	//������λ
#define YUN_UP_VALUELIMIT 3650	//������λ
#define YUN_UP_DISLIMIT 552	//�����Ļ��Χ��UPΪ��
#define YUN_DOWN_DISLIMIT 333	//�����Ļ��Χ��DOWNΪ��


#define VISION_TARX 1035//1035��������װƫ��1020//580	//����ԭ��	640
#define VISION_TARY	480//490//500//520//540//560//360//410//440	//����ԭ��	480
void Vision_Task(float* yaw_tarP,float* pitch_tarP)	//����Ŀ��Ƕ�
{
	if(Error_Check.statu[LOST_VISION]==1)	VisionData.armor_sign=0;	//���޷���=����Task�����ж��������У�������yun.c���Խ���Ƶ�ʱ�������
	//t_yaw_angel_v=Pixel_V_to_angle_V(VisionData.pix_x_v,(s16)(VisionData.error_x-VISION_TARX));
//	t_target_v=t_yaw_angel_v+Gyro_Data
	if(RC_Ctl.mouse.press_r==1&&VisionData.armor_sign==1)
	{
		VisionData.vision_control_state=1;	//���տ���λ
	}
	else
	{
		VisionData.vision_control_state=0;	//���տ���λ
	}
	
	if(VisionData.vision_control_state==1)
	{
//		t_yaw_error=Pixel_to_angle((s16)(VisionData.error_x-VISION_TARX))*10;
//		t_pitch_error=Pixel_to_angle((s16)(VisionData.error_y-VISION_TARY))*8192/360;
		
//		t_yaw_error=(float)Gyro_Data.angle[2]*10-Pixel_to_angle((s16)(VisionData.error_x-VISION_TARX))*10;
//		t_pitch_error=(float)yunMotorData.pitch_fdbP+Pixel_to_angle((s16)(VisionData.error_y-VISION_TARY))*8192/360;
		*yaw_tarP=(float)Gyro_Data.angle[2]*10+Pixel_to_angle((s16)(VisionData.tar_x-VISION_TARX))*10;
		*pitch_tarP=(float)yunMotorData.pitch_fdbP-Pixel_to_angle((s16)(VisionData.tar_y-VISION_TARY))*8192/360;
//		t_gravity_ballistic_set_angel=Gravity_Ballistic_Set(pitch_tarP,(float)(VisionData.armor_dis/10.0f+0.2));	//��������
//		Tar_Move_Set(yaw_tarP,(float)(VisionData.armor_dis/10.0f+0.2f),VisionData.angel_x_v);	//Ԥ�� ������
		
		*pitch_tarP=*pitch_tarP>(PITCH_INIT+YUN_UP_DISLIMIT)?(PITCH_INIT+YUN_UP_DISLIMIT):*pitch_tarP;	//�����г�
		*pitch_tarP=*pitch_tarP<(PITCH_INIT-YUN_DOWN_DISLIMIT)?(PITCH_INIT-YUN_DOWN_DISLIMIT):*pitch_tarP;	//�����г�
	}
	
}

#define SHOOT_V	15	//14M/s
#define SHOOT_V_2 (SHOOT_V*SHOOT_V)
#define G	9.8f	//�������ٶ�
float Gravity_Ballistic_Set(float* pitch_tarP,float dis_m)	//������������ϵ�У�����Ϊ��
{
	static float tar_angle_rad_fliter=0;
	float tar_angle_rad=(*pitch_tarP-PITCH_GYRO_INIT)*0.000767f;	//�����Ƽ򻯼���2pi/8192
	tar_angle_rad_fliter=0.9f*tar_angle_rad_fliter+0.1f*tar_angle_rad;
	float sin_tar_angle=sin(tar_angle_rad);
	float gravity_ballistic_angle_rad=0;	//������ ������
	float gravity_ballistic_angle=0;	//������ �Ƕ���
	gravity_ballistic_angle_rad=0.5f*(-asin((G*dis_m*(1-sin_tar_angle*sin_tar_angle)-sin_tar_angle*SHOOT_V_2)/SHOOT_V_2)+tar_angle_rad);
	gravity_ballistic_angle=gravity_ballistic_angle_rad*57.3f;

	*pitch_tarP=PITCH_GYRO_INIT-gravity_ballistic_angle*8192.0f/360;

	return gravity_ballistic_angle;
}

void Tar_Move_Set(float* yaw_tarP,float dis_m,float tar_v)
{
	static float tar_v_fliter=0;
//	if(abs(tar_v)<2)	tar_v=0;
	
	//tar_v_fliter=0.9f*tar_v_fliter+0.1f*tar_v;
	if (dis_m>1.9f)
	{
		dis_m=1.9f;
	}
	float shoot_delay=dis_m/SHOOT_V;	//����Ϊ��λ
	float pre_angle=tar_v*shoot_delay;
	if(pre_angle>65)	pre_angle=65;
	if(pre_angle<-65)	pre_angle=-65;
	*yaw_tarP+=pre_angle;
}


u8 Auto_Shoot_Aimfdb(void)	//��׼״̬
{
	if(abs(VisionData.tar_y-VISION_TARY)<30&&abs(VisionData.tar_x-VISION_TARX)<5&&Error_Check.statu[LOST_VISION]==0&&abs(VisionData.angel_x_v)<40)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
