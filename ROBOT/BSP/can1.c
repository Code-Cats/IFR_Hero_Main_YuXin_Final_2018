#include "can1.h"

/*********************************
函数名：  CAN1_Motors_Config
函数功能：CAN1电机通信配置 
函数参数：无
函数返回值：无
描述：初始化CAN1配置为1M波特率
      TX*****PA12
	    RX*****PA11
*********************************/
void CAN1_Motors_Config(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  canFilter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

    gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &gpio);
    
    nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    CAN_DeInit(CAN1);
    CAN_StructInit(&can);
		
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = ENABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN1, &can);

	  canFilter.CAN_FilterNumber=1;
	  canFilter.CAN_FilterMode=CAN_FilterMode_IdMask;
	  canFilter.CAN_FilterScale=CAN_FilterScale_32bit;
	  canFilter.CAN_FilterIdHigh=0x0000;
	  canFilter.CAN_FilterIdLow=0x0000;
	  canFilter.CAN_FilterMaskIdHigh=0x0000;
	  canFilter.CAN_FilterMaskIdLow=0x0000;
	  canFilter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
	  canFilter.CAN_FilterActivation=ENABLE;
	  CAN_FilterInit(&canFilter);
    
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
}

/****************************
函数名：CAN1_RX0_IRQHandler
函数功能：接收CAN数据并解析得到
          yunMotorData.Fire_Cp yunMotorData.Fire_Cv
函数输入：无
函数返回值：无
描述： CAN接收中断服务函数
****************************/
void CAN1_RX0_IRQHandler(void)
{    
	 CanRxMsg rx_message;
   if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
	 {	   
      CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		  CAN1_Feedback_Analysis(&rx_message);
   }
}

