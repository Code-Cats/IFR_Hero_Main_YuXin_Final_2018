#include "usart1_remote.h"

unsigned char sbus_rx_buffer[25];
/*************************************
函数名称：USART1_DMA_Config
函数功能：串口1DMA配置
函数参数：波特率：baudrate
函数返回值： 无
描述：DANHUANCHONG
      配置引脚为PB7
			DMA选择：DMA2_Stream2
**************************************/
void USART1_DMA_Config(uint32_t baudrate)	//沿用自董陪伦
{
  GPIO_InitTypeDef GPIO_InitStructure; 
  USART_InitTypeDef usart1;
	
			//结构体初始化
  NVIC_InitTypeDef nvic;
	DMA_InitTypeDef dma;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	USART_DeInit(USART1);
	USART_StructInit(&usart1);
	usart1.USART_BaudRate = 100000;
	usart1.USART_WordLength = USART_WordLength_8b;
	usart1.USART_StopBits = USART_StopBits_1;
	usart1.USART_Parity = USART_Parity_Even;
	usart1.USART_Mode = USART_Mode_Rx ;
	usart1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1,&usart1);
	
	USART_Cmd(USART1,ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
	
	//////////////////////////////////////////////////////////////
	
	
  DMA_DeInit(DMA2_Stream2);
	//DMA2 CLOCK ENABLE
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	//DMA CONFIG
  dma.DMA_Channel = DMA_Channel_4;
  dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
  dma.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer;
  dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
  dma.DMA_BufferSize = 18;
  dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
  dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  dma.DMA_Mode = DMA_Mode_Circular;
  dma.DMA_Priority = DMA_Priority_VeryHigh;
  dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
  dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  dma.DMA_MemoryBurst = DMA_Mode_Normal;
  dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream2,&dma);
	//DMA STREAM2 TRANSPORT COMPLETE INTERRUPT ENABLE
  DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);
	//DMA STREAM2 ENABLE
  DMA_Cmd(DMA2_Stream2,ENABLE); 
	//DMA中断
  nvic.NVIC_IRQChannel = DMA2_Stream2_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 0;	//2
  nvic.NVIC_IRQChannelSubPriority = 0;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
}

//DMA2数据流2中断服务函数
void DMA2_Stream2_IRQHandler(void)
{ 
  if(DMA_GetITStatus(DMA2_Stream2,DMA_IT_TCIF2))
  {

			//清除DMA接收中断
			DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2);
      DMA_ClearITPendingBit(DMA2_Stream2,DMA_IT_TCIF2);
			
			//数据解码
		RemoteData_analysis(sbus_rx_buffer);
		LostCountFeed(&Error_Check.count[LOST_DBUS]);
		}
}

