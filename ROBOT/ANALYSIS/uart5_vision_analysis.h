#ifndef __UART5_VISION_ANALYSIS_H
#define __UART5_VISION_ANALYSIS_H

#include "stm32f4xx.h"
#include "bsp.h"

typedef struct
{
	volatile u8 headOK_state;
	volatile u8 valid_state;	//数据帧有效标志位
	volatile u8 databuffer[10];	//最新的增加了速度10位//由副板的5位变为视觉8位（带帧头帧尾）
	volatile u8 count;
}VisionReceiveDataTypeDef;	//副板数据处理数据结构体


typedef struct
{
	u8 armor_sign;	//是否有有效装甲
	u8 armor_type;	//装甲类型
	u8 armor_dis;	//深度信息
	u16 tar_x;	//x坐标
	u16 tar_y;	//y坐标
	s16 shooterror_x;	//实际目标偏差x	作为自动射击
	s16 shooterror_y;	//实际目标偏差y	作为自动射击
	s16 pix_x_v;
	float angel_x_v;	//经融合的到的相对速度	//单位为0.1度每秒
	float angle_x_v_filter;
	u8 vision_control_state;	//是否受控
}VisionDataTypeDef;	//经解析得到的传感器数据

void VisionData_Receive(u8 data);	//从视觉传过来的数据接受校验主函数
void VisionData_Deal(volatile u8 *pData);	//视觉解析

#endif
