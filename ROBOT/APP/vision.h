#ifndef __VISION_H_
#define __VISION_H_
#include "bsp.h"
#include "usart6_vision_analysis.h"

float Pixel_to_angle(s16 pix_error);	//将像素误差转换成角度
void Vision_Task(float* yaw_tarP,float* pitch_tarP);	//处理目标角度

#endif
