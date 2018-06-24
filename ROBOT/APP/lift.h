#ifndef __LIFT_H_
#define __LIFT_H_

#include "stm32f4xx.h"



void AutoChassisAttitude_Lift(float chassis_pitch);	//自动调整姿态	//pitch正方向为前上	//注意放在lift_task前面
void AutoChassisAttitude_Lift_V2(float chassis_pitch_raw);	//自动调整姿态	//pitch正方向为前上	//注意放在lift_task前面


#endif

