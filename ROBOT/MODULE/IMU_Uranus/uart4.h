#ifndef __UART4_H
#define __UART4_H

#include "stdio.h"
#include "stm32f4xx.h"

#include "delay.h"

#include "imu_data_decode.h"
#include "packet.h"


typedef struct GYRO_Struct
{
	int16_t acc[3];
	int16_t angvel[3];
	float angle[3];
}GYRO_DATA;
extern GYRO_DATA Gyro_Data;


void uart4_init(void);

#endif
