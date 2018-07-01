#include "uart4.h"
#include "protect.h"
 /**
 * @brief  UART4 INIT
 * @note   UART4 INIT
 * @param  void
 * @retval void
 */
 
 s32 t_gyro_count=0;
 
GYRO_DATA Gyro_Data;
void uart4_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure; 
  USART_InitTypeDef uart4;
	NVIC_InitTypeDef uart4_nvic;
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
	
	uart4_nvic.NVIC_IRQChannel = UART4_IRQn;
  uart4_nvic.NVIC_IRQChannelPreemptionPriority = 0;
  uart4_nvic.NVIC_IRQChannelSubPriority = 0;
  uart4_nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&uart4_nvic);
	
	USART_ITConfig(UART4,USART_IT_RXNE,ENABLE);
}

unsigned char ch;
void UART4_IRQHandler(void)
{
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) 
	{
		USART_ClearITPendingBit(UART4,USART_IT_RXNE);
		ch=USART_ReceiveData(UART4);
		data_decode(ch);
	}	
}
enum {READY,DATE,ACC,GYRO,ANGLE,MAG,IDLE};
int STATUS = IDLE;
uint16_t offset = 0;
uint16_t AxL,AyL,AzL,AxH,AyH,AzH;
uint16_t wxL,wxH,wyL,wyH,wzL,wzH;
uint16_t TL,TH;
uint16_t RollL,RollH,PitchL,PitchH,YawL,YawH;
uint16_t HxL,HxH,HyL,HyH,HzL,HzH;
uint16_t sum = 0,sum_cal = 0;

void data_decode(uint16_t ch)
{
	switch(STATUS)
	{
		case IDLE:
			offset = 0;
			if(ch == 0x55) STATUS = READY;
			else STATUS = IDLE;
		break;
		case READY:
		{
			switch(ch)
			{
				case 0x51:
					STATUS = ACC;
					offset = 0;
					break;
				case 0x52:
					STATUS = GYRO;
					offset = 0;
					break;
				case 0x53:
					STATUS = ANGLE;
					offset = 0;
					break;
				case 0x54:
					STATUS = MAG;
					offset = 0;
					break;
				default:
					STATUS = IDLE;
			}
			break;
		}	
		case ACC:
			switch(offset++)
			{
				case 0:AxL = ch;
					break;
				case 1:AxH = ch;
					break;
				case 2:AyL = ch;
					break;
				case 3:AyH = ch;
					break;
				case 4:AzL = ch;
					break;
				case 5:AzH = ch;
					break;
				case 6:TL = ch;
					break;
				case 7:TH = ch;
					break;
				case 8:
					sum = ch;
					sum_cal = 0x55+0x51+AxL+AxH+AyL+AyH+AzL+AzH+TL+TH;
					if((sum<<8) & (sum_cal<<8) )
					{
						Gyro_Data.T	 =(float)(((TH<<8)|TL)/100.0f);
						////////////////////////////////////////////////////////////
						Gyro_Data.acc[ACC_X]= ((float)(short)(((short)AxH<<8)|AxL)/32768.0f*16.0f);
						Gyro_Data.acc[ACC_Y]= ((float)(short)(((short)AyH<<8)|AyL)/32768.0f*16.0f);
						Gyro_Data.acc[ACC_Z]= ((float)(short)(((short)AzH<<8)|AzL)/32768.0f*16.0f);
						////////////////////////////////////////////////////////////
						STATUS = IDLE;
					}else STATUS = IDLE;
					break;
				default:
					STATUS = IDLE;
					offset = 0;
			}
		break;
		case GYRO:
			switch(offset++)
			{
				case 0:wxL = ch;
					break;
				case 1:wxH = ch;
					break;
				case 2:wyL = ch;
					break;
				case 3:wyH = ch;
					break;
				case 4:wzL = ch;
					break;
				case 5:wzH = ch;
					break;
				case 6:TL = ch;
					break;
				case 7:TH = ch;
					break;
				case 8:
					sum = ch;
					sum_cal = 0x55+0x52+wxL+wxH+wyL+wyH+wzL+wzH+TL+TH;
					if((sum<<8) & (sum_cal<<8))
					{
						Gyro_Data.T	 =(float)((((short)TH<<8)|TL)/100.0f);
						/////////////////////////////////////////////////////////////
						Gyro_Data.angvel[PITCH]= ((float)(short)(((short)wxH<<8)|(short)wxL)/32768.0f*2000.0f);
						Gyro_Data.angvel[ROLL]= ((float)(short)(((short)wyH<<8)|(short)wyL)/32768.0f*2000.0f);
						Gyro_Data.angvel[YAW]= -((float)(short)(((short)wzH<<8)|(short)wzL)/32768.0f*2000.0f);
						/////////////////////////////////////////////////////////////
						STATUS = IDLE;
					}else STATUS = IDLE;
					break;
				default:
					STATUS = IDLE;
					offset = 0;
			}
		break;
		case ANGLE:
			switch(offset++)
			{
				case 0:RollL = ch;
					break;
				case 1:RollH = ch;
					break;
				case 2:PitchL = ch;
					break;
				case 3:PitchH = ch;
					break;
				case 4:YawL = ch;
					break;
				case 5:YawH = ch;
					break;
				case 6:TL = ch;
					break;
				case 7:TH = ch;
					break;
				case 8:
					sum = ch;
					sum_cal = 0x55+0x53+RollL+RollH+PitchL+PitchH+YawL+YawH+TL+TH;
					if((sum<<8) & (sum_cal<<8))
					{
						Gyro_Data.T			=	(float)((((short)TH<<8)|TL)/100.0f);
						////////////////////////////////////////////////////
						Gyro_Data.angle[ROLL]= (float)((((short)PitchH<<8)|PitchL)/32768.0f*180.0f);	//安装上和陀螺仪规定方向差90°
						Gyro_Data.angle[PITCH]= (float)((((short)RollH<<8)|RollL)/32768.0f*180.0f);
						Gyro_Data.angle[YAW]= (float)((((short)YawH<<8)|YawL)/32768.0f*180.0f);
						
						Gyro_Data.angle[PITCH]=Data_Boundary_Transform(Gyro_Data.angle[PITCH]);
						Gyro_Data.angle[ROLL]=Data_Boundary_Transform(Gyro_Data.angle[ROLL]);
						Gyro_Data.angle[YAW]=-Data_Boundary_Transform(Gyro_Data.angle[YAW]);
						
						LostCountFeed(&Error_Check.count[LOST_IMU]);
						////////////////////////////////////////////////////
						STATUS = IDLE;
					}else STATUS = IDLE;
					break;
				default:
					STATUS = IDLE;
					offset = 0;
			}
		break;
		case MAG:	//磁场
			switch(offset++)
			{
				case 0:HxL = ch;
					break;
				case 1:HxH = ch;
					break;
				case 2:HyL = ch;
					break;
				case 3:HyH = ch;
					break;
				case 4:HzL = ch;
					break;
				case 5:HzH = ch;
					break;
				case 6:TL = ch;
					break;
				case 7:TH = ch;
					break;
				case 8:
					sum = ch;
					sum_cal = 0x55+0x54+HxL+HxH+HyL+HyH+HzL+HzH+TL+TH;
					if((sum<<8) & (sum_cal<<8))
					{
						Gyro_Data.Hx = ((float)(short)(((short)HxH<<8)|HxL)/32768.0f*2000.0f);
						Gyro_Data.Hy = ((float)(short)(((short)HyH<<8)|HyL)/32768.0f*2000.0f);
						Gyro_Data.Hz = ((float)(short)(((short)HzH<<8)|HzL)/32768.0f*2000.0f);
						Gyro_Data.T	 = (float)((((short)TH<<8)|TL)/100.0f);
						STATUS = IDLE;
					}else STATUS = IDLE;
					break;
				default:
					STATUS = IDLE;
					offset = 0;
			}
		break;
		default:
			STATUS = IDLE;
	}
}

	
float Data_Boundary_Transform(float input)	//数据从360分界变换到180分界
{
	float output=0;
	if(input>180)
	{
		output=input-360;
	}
	else
	{
		output=input;
	}
	return output;
}

