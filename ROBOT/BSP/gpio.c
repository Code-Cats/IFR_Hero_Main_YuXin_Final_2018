#include "gpio.h"

/*************************
函数名：LED_Configuration
函数功能：LED灯的GPIO配置
函数输入：无
函数返回值： 无
描述：配置的引脚 PC0；PC1
*************************/
void LED_Config(void)
{
   GPIO_InitTypeDef gpio;  
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
		
	 gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0 ;
	 gpio.GPIO_Mode = GPIO_Mode_OUT;
	 gpio.GPIO_OType = GPIO_OType_PP;
	 gpio.GPIO_Speed = GPIO_Speed_100MHz;
	 GPIO_Init(GPIOC,&gpio);
			
	 GPIO_SetBits(GPIOC, GPIO_Pin_1);	//绿 灭
	 GPIO_SetBits(GPIOC, GPIO_Pin_0);	//红 灭
}
/*************************
函数名：Laser_Config
函数功能：激光的GPIO配置
函数输入：无
函数返回值： 无
描述：配置的引脚 PC5
*************************/
void Laser_Config(void)
{
   GPIO_InitTypeDef gpio;
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	 gpio.GPIO_Pin = GPIO_Pin_5;
	 gpio.GPIO_Mode = GPIO_Mode_OUT;
	 gpio.GPIO_OType = GPIO_OType_PP;
	 gpio.GPIO_Speed = GPIO_Speed_100MHz;
	 GPIO_Init(GPIOC, &gpio);
	 GPIO_ResetBits(GPIOC, GPIO_Pin_5);
}

