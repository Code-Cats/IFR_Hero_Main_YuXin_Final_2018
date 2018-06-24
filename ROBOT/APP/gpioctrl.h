#ifndef __GPIOCTRL_H__
#define __GPIOCTRL_H__
#include "main.h"

#define GREEN_LED_ON()      GPIO_ResetBits(GPIOC, GPIO_Pin_1)
#define GREEN_LED_OFF()     GPIO_SetBits(GPIOC, GPIO_Pin_1)
#define GREEN_LED_TOGGLE()      GPIO_ToggleBits(GPIOC, GPIO_Pin_1)

#define RED_LED_ON()            GPIO_ResetBits(GPIOC, GPIO_Pin_0)
#define RED_LED_OFF()           GPIO_SetBits(GPIOC, GPIO_Pin_0)
#define RED_LED_TOGGLE()        GPIO_ToggleBits(GPIOC, GPIO_Pin_0)

#define LASER_ON()  GPIO_SetBits(GPIOC, GPIO_Pin_5)
#define LASER_OFF()  GPIO_ResetBits(GPIOC, GPIO_Pin_5)

#define DELAY_CNT  200000    //闪烁时间间隔

#define BLINK_CYCLE 150
#define BLINK_INTERVAL (BLINK_CYCLE*2*8)
void LED_Blink_Run(void);
void LED_Blink_Set(u8 frequency_green,u8 frequency_red);	//2s内闪烁次数//LED闪烁运行函数10h处理频率

#endif

