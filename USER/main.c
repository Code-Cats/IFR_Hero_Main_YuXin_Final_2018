#include "main.h"
#include "math.h"

#define PITCH 0	//�Ƶ�����ȥ��
#define ROLL 1
#define YAW 2


u8 Judge_Send_Statu=0;	//ˢ�±�־

int main(void)
{
	SetWorkState(CHECK_STATE);
	Data_Init();
	delay_ms(500);
	BSP_Init();
	delay_ms(500);
	while(1)
	 {
		 Screen_Start();
	 }
}





//////////////////////////////////////////////////////////////////
//??????,??printf??,??????use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//??????????                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//??_sys_exit()??????????    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//???fputc?? 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//????,??????   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
