#ifndef __VISION_H_
#define __VISION_H_
#include "bsp.h"
#include "uart5_vision_analysis.h"

float Pixel_to_angle(s16 pix_error);	//���������ת���ɽǶ�
void Vision_Task(float* yaw_tarP,float* pitch_tarP);	//����Ŀ��Ƕ�

float Pixel_V_to_angle_V(s16 pix_v,s16 pix_error);	//����ԭʼ�����ݽ��м�����Լ��ٵ�Ƭ����������ľ��ȶ�ʧ��������ӣ�

void Tar_Relative_V_Mix(float yaw_angvel,s16 pix_x_v);	//Ŀ���ٶ��ں�

float Gravity_Ballistic_Set(float* pitch_tarP,float dis_m);	//������������ϵ�У�����Ϊ��
void Tar_Move_Set(float* yaw_tarP,float dis_m,float tar_v);

u8 Auto_Shoot_Aimfdb(void);	//��׼״̬
u8 Auto_Shoot_AimAppraisal_Dynamic(float relative_v,s16 dis_dm,s16 pix_error);	//��̬��׼��������

#endif
