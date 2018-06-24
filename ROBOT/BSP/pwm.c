#include "pwm.h"

#define STEER_UP_L_INIT 560//500//1210	//2500
#define STEER_UP_R_INIT 2500//1950	//500
/***********************************
函数名称：PWM_Config
函数功能：配置定时器3的通道3和通道4为摩擦轮PWM
          配置定时器14的通道1为弹仓盖舵机PWM
函数参数： 无
函数返回值： 无
描述：定时器3通道3，通道4；定时器14通道1；
************************************/
void PWM_Config(void)
{
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB ,ENABLE);
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);   //PCLK1=42MHz,TIM5 clk =84MHz
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);   
//		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE); 
	
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1  ;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB,&gpio);
		
		gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA,&gpio);//定时器5通道3

    GPIO_PinAFConfig(GPIOB,GPIO_PinSource0,GPIO_AF_TIM3);//定时器3 通道3
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource1,GPIO_AF_TIM3);//定时器3 通道4
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);//定时器3 通道1
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);//定时器3 通道2
		
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5);//定时器5 通道3
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5);//定时器5 通道3
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM5);//定时器5 通道3
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM5);//定时器5 通道4
	
    tim.TIM_Prescaler = 84-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 20000;   //2ms
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3,&tim);
		
//	  tim.TIM_Prescaler = 84-1;
//    tim.TIM_CounterMode = TIM_CounterMode_Up;
//    tim.TIM_Period = 5000;   //2ms
//    tim.TIM_ClockDivision = TIM_CKD_DIV1;
//    TIM_TimeBaseInit(TIM14,&tim);
		
		    /* TIM5 */
		tim.TIM_Prescaler = 84-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 20000;   //2ms
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM5,&tim);
		
    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 1000;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
		TIM_OC1Init(TIM3,&oc);	//PA6
    TIM_OC2Init(TIM3,&oc);	//PA7
    TIM_OC3Init(TIM3,&oc);	//PB0
    TIM_OC4Init(TIM3,&oc);	//PB1
		oc.TIM_Pulse = 1000;
		TIM_OC1Init(TIM5,&oc);//定时器5 通道1
		TIM_OC2Init(TIM5,&oc);//定时器5 通道2
		TIM_OC3Init(TIM5,&oc);//定时器5 通道3
		TIM_OC4Init(TIM5,&oc);//定时器5 通道4
		
//		TIM_OC1Init(TIM14,&oc);
//		oc.TIM_Pulse = 0;
//		TIM_OC1Init(TIM14,&oc);//定时器5 通道3
		
//    TIM_OC1PreloadConfig(TIM14,TIM_OCPreload_Enable);		
		TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
		
		TIM_OC1PreloadConfig(TIM5,TIM_OCPreload_Enable);
		TIM_OC2PreloadConfig(TIM5,TIM_OCPreload_Enable);
		TIM_OC3PreloadConfig(TIM5,TIM_OCPreload_Enable);
		TIM_OC4PreloadConfig(TIM5,TIM_OCPreload_Enable);
           
//		TIM_ARRPreloadConfig(TIM14,ENABLE);
    TIM_ARRPreloadConfig(TIM3,ENABLE);
    TIM_ARRPreloadConfig(TIM5,ENABLE);
		
//		TIM_Cmd(TIM14,ENABLE);
    TIM_Cmd(TIM3,ENABLE);
		TIM_Cmd(TIM5,ENABLE);
		
		PWM3_1=STEER_UP_L_INIT;
		PWM3_2=STEER_UP_R_INIT;
		
		IMAGE_START=PWM_IO_ON;
		t_AV_CUT=PWM_IO_OFF;
}



