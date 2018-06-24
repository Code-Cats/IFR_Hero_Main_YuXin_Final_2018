#ifndef __USART1_REMOTE_H__
#define __USART1_REMOTE_H__

#include "main.h"

#define  USART1_DMA_RC_BUF_LEN               30u  
#define  RC_FRAME_LENGTH                     18u

void USART1_IRQHandler(void);
void USART1_DMA_Config(uint32_t baudrate);

#endif

