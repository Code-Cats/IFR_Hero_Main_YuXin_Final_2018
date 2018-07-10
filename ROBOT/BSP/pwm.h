#ifndef __PWM_H__
#define __PWM_H__

#include "main.h"

#define PWM3_1  TIM3->CCR1    //
#define PWM3_2  TIM3->CCR2    //
#define PWM3_3  TIM3->CCR3    //摩擦轮1
#define PWM3_4  TIM3->CCR4    //摩擦轮2
//#define PWM5  TIM14->CCR1   //弹仓盖舵机

#define SetFrictionWheelSpeed(x) \
        PWM3_3 = x;                \
        PWM3_4 = x;
				
				
#define PWM5_1 TIM5->CCR1
#define PWM5_2 TIM5->CCR2
#define PWM5_3 TIM5->CCR3
#define PWM5_4 TIM5->CCR4


#define STEER_UP_L PWM3_1
#define STEER_UP_R PWM3_2
#define STEER_DOWN_L PWM3_3	//保持与工程兼容
#define STEER_DOWN_R PWM3_4

//////#define STEER_IMAGE PWM5_4
//////#define IMAGE_START	PWM5_3
//////#define STEER_RFID	PWM5_2
//////#define t_AV_CUT PWM5_2	//临时的，给切换图传用的
#define IMAGE_START	PWM5_4

#define SUPERCAPACITOR_JUDGE	PWM5_1
#define SUPERCAPACITOR_INPUT	PWM5_2
#define SUPERCAPACITOR_OUTPUT	PWM5_3
				
#define FRICTION_INIT      1000

#define PWM_IO_ON	20000
#define PWM_IO_OFF	0

void PWM_Config(void);


#endif


