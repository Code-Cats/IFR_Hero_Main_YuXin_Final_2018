#include "dma_uart4.h"

#define lenth_uart4 30
u8  buffer_UART4[lenth_uart4];
u8  buf2_xxx[lenth_uart4];
u8  buf3_xxx[lenth_uart4];
GYRO_DATA Gyro_Data={0};
 /**
 * @brief  DMA CONFIG
 * @note   UART4 DMA INIT
 * @param  void
 * @retval void
 */

void dma_uart4_init(void)
{
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
  dma_uart4.DMA_BufferSize = lenth_uart4*2;
  dma_uart4.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  dma_uart4.DMA_MemoryInc = DMA_MemoryInc_Enable;
  dma_uart4.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  dma_uart4.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  dma_uart4.DMA_Mode = DMA_Mode_Circular;
  dma_uart4.DMA_Priority = DMA_Priority_VeryHigh;
  dma_uart4.DMA_FIFOMode = DMA_FIFOMode_Disable;
  dma_uart4.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  dma_uart4.DMA_MemoryBurst = DMA_Mode_Normal;
  dma_uart4.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream2,&dma_uart4);
	
  DMA_ITConfig(DMA1_Stream2,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA1_Stream2,ENABLE); 
	
  nvic_uart4.NVIC_IRQChannel = DMA1_Stream2_IRQn;
  nvic_uart4.NVIC_IRQChannelPreemptionPriority = 1;
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
				if(buffer_UART4[0]==0x5A&&buffer_UART4[1]==0x5A)				//帧头恰好处于buffer头，直接解析
				{
					int u;
					for(u = 0;u < lenth_uart4;u++) buf3_xxx[u]=buffer_UART4[u];
					packet_dec();
				}else
				{
					int i,j,k,bit;
					if(buf2_xxx[lenth_uart4-1]==0x5A&&buffer_UART4[0]==0x5A)	//帧头位于buffer底部
					{
						bit = lenth_uart4-1;
					}else 
					{
						for(i = 0;i < lenth_uart4;i++)											//找出帧头位置，并计算偏移量 bit
						{
							if(buf2_xxx[i]==0x5A && buf2_xxx[i+1]==0x5A) bit = i;		
						}
					}
					for(j = 0;j < lenth_uart4-bit;j++)										//把 buf2 的 bit~（lenth_uart4-1）位赋值给 
					{																											//					buf3 0~（lenth_uart4-1-bit）位
						buf3_xxx[j]=buf2_xxx[j+bit];
					}
					for(k = 0;k < bit;k++)														 		//把 buffer_UART4 的 0~（bit-1）位 赋值给 
					{																											//	buf3 (lenth_uart4-bit)~lenth_uart4 位
						buf3_xxx[k+lenth_uart4-bit]=buffer_UART4[k];
					}
					for(j = 0;j <lenth_uart4;j++)													//把 buffer_UART4 赋值给 buf2
					{
						buf2_xxx[j]=buffer_UART4[j];
					}
					packet_dec();
				}
	}
}

void packet_dec(void)
{
		Gyro_Data.acc[0] = buf3_xxx[5]|(buf3_xxx[4]<<8);
		Gyro_Data.acc[1] = buf3_xxx[7]|(buf3_xxx[6]<<8);
		Gyro_Data.acc[2] = buf3_xxx[9]|(buf3_xxx[8]<<8);

		Gyro_Data.mag[0] = (int16_t)(buf3_xxx[11]|(buf3_xxx[10]<<8))/16;
		Gyro_Data.mag[1] = (int16_t)(buf3_xxx[13]|(buf3_xxx[12]<<8))/16;
		Gyro_Data.mag[2] = (int16_t)(buf3_xxx[15]|(buf3_xxx[14]<<8))/16;
	
		Gyro_Data.angvel[0] = (int16_t)(buf3_xxx[17]|(buf3_xxx[16]<<8))/16;
		Gyro_Data.angvel[1] = (int16_t)(buf3_xxx[19]|(buf3_xxx[18]<<8))/16;
		Gyro_Data.angvel[2] = (int16_t)(buf3_xxx[21]|(buf3_xxx[20]<<8))/16;

		Gyro_Data.angle[0] = (float)((uint16_t)(buf3_xxx[23]|(buf3_xxx[22]<<8)))/100;			//yaw
		Gyro_Data.angle[1] = (float)((int16_t)(buf3_xxx[25]|(buf3_xxx[24]<<8)))/100;			//roll
		Gyro_Data.angle[2] = (float)((int16_t)(buf3_xxx[27]|(buf3_xxx[26]<<8)))/100;				//pitch
}
