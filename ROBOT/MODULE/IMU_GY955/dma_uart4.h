#ifndef __DMA_UART4_H
#define __DMA_UART4_H
#include "stm32f4xx.h"
#include "uart4.h"
#include "main.h"

void dma_uart4_init(void);
void packet_dec(void);
typedef struct __GYRO_
{
	int16_t acc[3];
	int16_t mag[3];
	int16_t angvel[3];
	float angle[3];//pitch roll yaw
}GYRO_DATA;
extern GYRO_DATA Gyro_Data;

/************************
µÚÒ»°æÍÓÂÝÒÇ
typedef struct GYRO_Struct
{
	int16_t acc[3];
	int16_t angvel[3];
	float angle[3];
}GYRO_DATA;

extern GYRO_DATA Gyro_Data;

************************/

#endif 
