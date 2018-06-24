#include "main.h"
#include "math.h"

#define PITCH 0	//移到上面去了
#define ROLL 1
#define YAW 2


u8 Judge_Send_Statu=0;	//刷新标志

int main(void)
{
	delay_ms(1500);
	while(1)
	 {
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
