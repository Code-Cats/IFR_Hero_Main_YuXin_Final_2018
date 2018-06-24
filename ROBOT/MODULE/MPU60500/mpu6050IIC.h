#ifndef __MPU6050IIC_H__
#define __MPU6050IIC_H__

#include "main.h"

void IIC_GPIO_Init(void);
int IIC_WriteData(u8 deviceAddr,u8 regAddr,u8 data);
int IIC_ReadData(u8 deviceAddr,u8 regAddr,u8 *pdata,u8 count);
void HEAT_Configuration(void);

#endif

