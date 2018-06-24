#ifndef __DMA_UART4_H
#define __DMA_UART4_H

#include "uart4.h"
#include "packet.h"
#include "imu_data_decode.h"

typedef struct GYRO_Struct
{
	int16_t acc[3];
	int16_t angvel[3];
	float angle[3];
}GYRO_DATA;
extern GYRO_DATA Gyro_Data;

void dma_uart4_init(void);
void DMA1_Stream2_IRQHandler(void);

#endif 
