#ifndef __IMU_DEAL_H_
#define __IMU_DEAL_H_

#include "uart4.h"
#include "stm32f4xx.h"

u8 Imu_Cali_State(void);
void Imu_Offset(float angvel[]);	//和陀螺仪角速度更新频率一致
#endif
