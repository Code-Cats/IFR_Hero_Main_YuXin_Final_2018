#include "heartbeat.h"


/*********************
函数名：  TIM6_Configuration
函数功能：定时器6配置
函数输入：无
函数返回值： 无
描述：定时器周期为1ms
**********************/
void TIM6_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);

    nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tim.TIM_Prescaler = 84-1;//cyq:APB1=42MHz,APB2=84MHz
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 1000;//cyq:1Khz
    TIM_TimeBaseInit(TIM6,&tim);
}
/*********************
函数名：  TIM6_Start_Heartbeat
函数功能：开启定时器6，清除定时器状态标志位
函数输入：无
函数返回值： 无
*********************/
void TIM6_Start(void)
{
    TIM_Cmd(TIM6, ENABLE);	 
    TIM_ITConfig(TIM6, TIM_IT_Update,ENABLE);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);	
}
/***************************************
函数名：IM6_DAC_IRQHandler
函数功能：执行系统响相应指令
函数参数： 无
函数返回值： 无
描述： 定时器中断服务函数
****************************************/
void TIM6_DAC_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET) 
	{
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);
		Control_Task();
   }
}

