#ifndef __VISION_H_
#define __VISION_H_
#include "bsp.h"
#include "uart5_vision_analysis.h"

float Pixel_to_angle(s16 pix_error);	//将像素误差转换成角度
void Vision_Task(float* yaw_tarP,float* pitch_tarP);	//处理目标角度

float Pixel_V_to_angle_V(s16 pix_v,s16 pix_error);	//从最原始的数据进行计算可以减少单片机浮点运算的精度丢失（误差增加）

float Gravity_Ballistic_Set(float* pitch_tarP,float dis_m);	//重力补偿坐标系中，向下为正
void Tar_Move_Set(float* yaw_tarP,float dis_m,float tar_v);
#endif
