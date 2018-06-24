#ifndef _FILTER_H_
#define _FILTER_H_

#include "main.h"

#define     PI                     3.1415926f

typedef __packed struct
{
		const float ratio;	//一阶滞后滤波系数
	  const char    up_flag;//上升时是否需要滤波
	  const char    down_flag;//下降时是否需要滤波
	  const int16_t tiny;	//计算死区
    float Data_Save[2];
	  float output;	  
}FIRST_ORDER_FILTER;

//typedef __packed struct
//{
//    const float   former;
//	  const float   later;
//	  const char    up_flag;//上升时是否需要滤波
//	  const char    down_flag;//下降时是否需要滤波
//	  const int16_t   back;//归零区间
//	  const int16_t   input_min;
//		const int16_t   input_max;
//	  const int16_t   output_min;
//	  const int16_t   output_max;
//    float Data_Save[2];
//	  float output;	  
//}FIRST_ORDER_FILTER;
float FirstOrder_General(const s32 cur_val,FIRST_ORDER_FILTER*filter);	//2018.3.6


float Filter_firstOrder(int16_t input,FIRST_ORDER_FILTER *ray);
float	Filter_MPU6050_Accel(float Accel,float param);
float Filter_Yun_FirstOrder(float tar_val,int16_t cur_val,int init_val,float param);
#endif

