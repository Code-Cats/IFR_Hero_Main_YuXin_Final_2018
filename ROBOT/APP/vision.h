#ifndef __VISION_H_
#define __VISION_H_
#include "bsp.h"
#include "uart5_vision_analysis.h"

float Pixel_to_angle(s16 pix_error);	//���������ת���ɽǶ�
void Vision_Task(float* yaw_tarP,float* pitch_tarP);	//����Ŀ��Ƕ�

float Pixel_V_to_angle_V(s16 pix_v,s16 pix_error);	//����ԭʼ�����ݽ��м�����Լ��ٵ�Ƭ����������ľ��ȶ�ʧ��������ӣ�

float Gravity_Ballistic_Set(float* pitch_tarP,float dis_m);	//������������ϵ�У�����Ϊ��
void Tar_Move_Set(float* yaw_tarP,float dis_m,float tar_v);
#endif
