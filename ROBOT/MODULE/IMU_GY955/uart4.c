#include "uart4.h"
 /**
 * @brief  UART4 INIT
 * @note   UART4 INIT
 * @param  void
 * @retval void
 */

void uart4_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure; 
  USART_InitTypeDef uart4;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_11 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	USART_DeInit(UART4);
	USART_StructInit(&uart4);
	uart4.USART_BaudRate = 115200;
	uart4.USART_WordLength = USART_WordLength_8b;
	uart4.USART_StopBits = USART_StopBits_1;
	uart4.USART_Parity = USART_Parity_No;
	uart4.USART_Mode = USART_Mode_Rx;
	uart4.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(UART4,&uart4);
	USART_Cmd(UART4,ENABLE);
	USART_DMACmd(UART4,USART_DMAReq_Rx, ENABLE); 
}
