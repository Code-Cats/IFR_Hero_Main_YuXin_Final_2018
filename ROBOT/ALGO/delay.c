#include "delay.h"

/*****************************
函数名称：delay_ms
函数功能：延时，毫秒级
函数参数：累加值：t
函数返回值：无
******************************/
void delay_ms(unsigned int t)
{
	 int i;
	 for( i=0;i<t;i++)
	 {
		  int a=42000; //at 168MHz 42000 is ok
		  while(a--);
	 }
}
/*****************************
函数名称：delay_us
函数功能：延时，微秒级
函数参数：累加值：t
函数返回值：无
******************************/
void delay_us(unsigned int t)
{
	 int i;
	 for( i=0;i<t;i++)
	 {
		  int a=40;  //at 168MHz 40 is ok,the higher the number the more timing precise
		  while(a--);
	 }
}

