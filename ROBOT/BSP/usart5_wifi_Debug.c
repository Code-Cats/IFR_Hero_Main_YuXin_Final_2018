#include "usart5_wifi_Debug.h"


void USART5_WIFIDEBUG_Init(uint32_t baud_rate)	//待修改
{

    GPIO_InitTypeDef gpio;
	  USART_InitTypeDef usart;
	  NVIC_InitTypeDef nvic;
//    DMA_InitTypeDef dma;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
//     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); 
    
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); 

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC,&gpio);
    
    USART_DeInit(USART6);
    USART_StructInit(&usart);
    usart.USART_BaudRate = baud_rate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
	  usart.USART_Parity = USART_Parity_No;
   // usart.USART_Parity = USART_Parity_Even;
    usart.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART6, &usart);
//		
//		USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
//		
//	  DMA_DeInit(DMA1_Stream1);
//    DMA_StructInit(&dma);
//    dma.DMA_Channel = DMA_Channel_4;
//    dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
//    dma.DMA_Memory0BaseAddr = (uint32_t)&_USART3_DMA_RX_BUF[0][0];
//    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
//    dma.DMA_BufferSize = sizeof(_USART3_DMA_RX_BUF)/2;
//    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
//    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//    dma.DMA_Mode = DMA_Mode_Circular;
//    dma.DMA_Priority = DMA_Priority_Medium;
//    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
//    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//    DMA_Init(DMA1_Stream1, &dma);
//		
//		
//		    //配置Memory1,Memory0是第一个使用的Memory
//    DMA_DoubleBufferModeConfig(DMA1_Stream1, (uint32_t)&_USART3_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
//    DMA_DoubleBufferModeCmd(DMA1_Stream1, ENABLE);
//    
//    DMA_Cmd(DMA1_Stream1, ENABLE);
		
    nvic.NVIC_IRQChannel = USART6_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&nvic);
		
		USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);        //usart rx idle interrupt  enabled
    USART_Cmd(USART6, ENABLE);
}


u8 USART5_Res=0;
void USART5_IRQHandler(void)
{
////		if(Vision_Flag==0)
////		{
//	static uint32_t this_time_rx_len = 0;


	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{
		
		USART5_Res=USART_ReceiveData(UART5);
		//Data_Receive(USART6_Res);
		//clear the idle pending flag 
		(void)UART5->SR;
		(void)UART5->DR;
	}

}


