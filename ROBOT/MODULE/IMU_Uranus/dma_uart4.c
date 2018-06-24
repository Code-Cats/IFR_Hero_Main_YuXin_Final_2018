#include "dma_uart4.h"
#include "protect.h"

#define lenth_uart4 34	//34
uint8_t  buffer_UART4[lenth_uart4];
uint8_t  buf2[lenth_uart4];
uint8_t  buf3[lenth_uart4];


GYRO_DATA Gyro_Data={0};
 /**
 * @brief  DMA初始化函数
 * @note   UART4 DMA INIT
 * @param  void
 * @retval void
 */

void dma_uart4_init(void)
{
	//结构体初始化
  NVIC_InitTypeDef nvic_uart4;
	DMA_InitTypeDef dma_uart4;
  DMA_DeInit(DMA1_Stream2);
	//DMA2 CLOCK ENABLE
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	//DMA CONFIG
  dma_uart4.DMA_Channel = DMA_Channel_4;
  dma_uart4.DMA_PeripheralBaseAddr = (uint32_t)&(UART4->DR);
  dma_uart4.DMA_Memory0BaseAddr = (uint32_t)buffer_UART4;
  dma_uart4.DMA_DIR = DMA_DIR_PeripheralToMemory;
  dma_uart4.DMA_BufferSize = lenth_uart4;
  dma_uart4.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  dma_uart4.DMA_MemoryInc = DMA_MemoryInc_Enable;
  dma_uart4.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  dma_uart4.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  dma_uart4.DMA_Mode = DMA_Mode_Circular;
  dma_uart4.DMA_Priority = DMA_Priority_VeryHigh;
  dma_uart4.DMA_FIFOMode = DMA_FIFOMode_Disable;
  dma_uart4.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  dma_uart4.DMA_MemoryBurst = DMA_Mode_Normal;
  dma_uart4.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream2,&dma_uart4);

  DMA_ITConfig(DMA1_Stream2,DMA_IT_TC,ENABLE);

  DMA_Cmd(DMA1_Stream2,ENABLE); 

  nvic_uart4.NVIC_IRQChannel = DMA1_Stream2_IRQn;
  nvic_uart4.NVIC_IRQChannelPreemptionPriority = 0;
  nvic_uart4.NVIC_IRQChannelSubPriority = 0;
  nvic_uart4.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic_uart4);
}


 /**
 * @brief  DMA Interrupt Service Function
 * @note   DMA1_Stream2 Interrupt Service Function
 * @param  void
 * @retval void
 */

void DMA1_Stream2_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream2,DMA_IT_TCIF2))
  {
			DMA_ClearFlag(DMA1_Stream2,DMA_FLAG_TCIF2);
      DMA_ClearITPendingBit(DMA1_Stream2,DMA_IT_TCIF2);
		    //DATA SHIFTING 
		LostCountFeed(&Error_Check.count[LOST_IMU]);//////////////////////////////////////
//				if(buffer_UART4[0]==0x5A&&buffer_UART4[1]==0xA5)
//				{
//					int u;
//					for(u = 0;u < lenth_uart4;u++) Packet_Decode(buffer_UART4[u]);
//				}else
//				{
//					int i,j,k,bit,p=0;
//					if(buf2[lenth_uart4-1]==0x5A&&buffer_UART4[0]==0xA5)
//					{
//						bit = lenth_uart4-1;
//					}else 
//					{
//						for(i = 0;i <lenth_uart4;i++)
//						{
//							if(buf2[i]==0x5A && buf2[i+1]==0xA5) bit = i;
//						}
//					}
//					for(j = 0;j <lenth_uart4-bit;j++)
//					{
//						buf3[j]=buf2[j+bit];
//					}
//					for(k = 0;k < bit;k++)
//					{
//						buf3[k+lenth_uart4-bit]=buffer_UART4[k];
//					}
//					for(j = 0;j <lenth_uart4;j++)
//					{
//						buf2[j]=buffer_UART4[j];
//					}
//					for( p = 0 ; p < lenth_uart4 ; p++) Packet_Decode(buf3[p]);
//				}
				//GET GYROSCOPE DATA
				for(int p = 0 ; p < lenth_uart4 ; p++) Packet_Decode(buffer_UART4[p]);
				get_raw_acc(Gyro_Data.acc);
        get_raw_gyo(Gyro_Data.angvel);
        get_eular(Gyro_Data.angle);
	}
}

