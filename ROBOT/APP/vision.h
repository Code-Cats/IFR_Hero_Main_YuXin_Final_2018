#ifndef __VISION_H_
#define __VISION_H_
#include "bsp.h"
#include "usart6_vision_analysis.h"

float Pixel_to_angle(s16 pix_error);	//���������ת���ɽǶ�
void Vision_Task(float* yaw_tarP,float* pitch_tarP);	//����Ŀ��Ƕ�

#endif
