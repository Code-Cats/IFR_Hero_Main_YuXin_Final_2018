#ifndef __UART4_H_
#define __UART4_H_

#include "stdio.h"
#include "stm32f4xx.h"
#include "delay.h"

void data_decode(uint16_t ch);
void uart4_init(void);
float Data_Boundary_Transform(float input);	//数据从360分界变换到180分界

typedef struct __GYRO_DATA
{
	float	Hx;
	float	Hy;
	float Hz;
	float T;
	
	float acc[3];
	float angvel[3];
	float angle[3];
}GYRO_DATA;

#define PITCH 0	//x
#define ROLL 1	//y
#define YAW 2		//z
#define ACC_X	0
#define ACC_Y	1
#define ACC_Z	2
#endif
