#ifndef __UART5_VISION_ANALYSIS_H
#define __UART5_VISION_ANALYSIS_H

#include "stm32f4xx.h"
#include "bsp.h"

typedef struct
{
	volatile u8 headOK_state;
	volatile u8 valid_state;	//数据帧有效标志位
	volatile u8 databuffer[8];	//由副板的5位变为视觉8位（带帧头帧尾）
	volatile u8 count;
}VisionReceiveDataTypeDef;	//副板数据处理数据结构体


typedef struct
{
	u8 armor_sign;	//是否有有效装甲
	u8 armor_type;	//装甲类型
	u8 armor_dis;	//深度信息
	u16 error_x;	//x坐标差
	u16 error_y;	//y坐标差
}VisionDataTypeDef;	//经解析得到的传感器数据

void VisionData_Receive(u8 data);	//从视觉传过来的数据接受校验主函数
void VisionData_Deal(volatile u8 *pData);	//视觉解析

#endif
