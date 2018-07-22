#include "main.h"
#include "math.h"

#define PITCH 0	//移到上面去了
#define ROLL 1
#define YAW 2


u8 Judge_Send_Statu=0;	//刷新标志

extern RobotHeatDataSimuTypeDef RobotHeatDataSimu42;
extern GYRO_DATA Gyro_Data;

extern u8 Guiding_Lights_Data;
float Judge_Data_Send_A=0.0;	//裁判发送数据
float Judge_Data_Send_B=0.0;
float Judge_Data_Send_C=0.0;

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
		 
		 ViceBoard_SendDataRun();
		 
		 if(Judge_Send_Statu==1)
		 {
			 Judge_Data_Send_A=RobotHeatDataSimu42.bullet_num;	//发弹量
//			 Judge_Data_Send_B=(lift_Data.lf_lift_fdbP+lift_Data.rf_lift_fdbP)/2.0;	//前升降
			 Judge_Data_Send_C=Gyro_Data.angle[YAW];//(lift_Data.lb_lift_fdbP+lift_Data.rb_lift_fdbP)/2.0;	//后升降
			 Guiding_Lights_Data=Judagement_Send_Guiding_lights(0,0,!Error_Check.statu[LOST_REFEREE],0,0,IMU_Check_Useless_State);
			 Judgement_DataSend(Judge_Data_Send_A,Judge_Data_Send_B,Judge_Data_Send_C,Guiding_Lights_Data);
		 }
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
