#include "main.h"
#include "math.h"

#define PITCH 0	//移到上面去了
#define ROLL 1
#define YAW 2

float test_tarP=PITCH_GYRO_INIT;
float test_tarP_set=PITCH_GYRO_INIT;
float teat_dis=1;
float teat_out=0;
#define SHOOT_V	15	//14M/s
#define SHOOT_V_2 (SHOOT_V*SHOOT_V)
#define G	9.8f	//重力加速度
float testGravity_Ballistic_Set(float* pitch_tarP,float dis_m)	//重力补偿坐标系中，向下为正
{
//	static float tar_angle_rad_fliter=0;
	float tar_angle_rad=(PITCH_GYRO_INIT-*pitch_tarP)*0.000767f;	//弧度制简化计算2pi/8192//////////////////////////////////////////
//	tar_angle_rad_fliter=0.9f*tar_angle_rad_fliter+0.1f*tar_angle_rad;
	float sin_tar_angle=sin(tar_angle_rad);
	float gravity_ballistic_angle_rad=0;	//补偿角 弧度制
	float gravity_ballistic_angle=0;	//补偿角 角度制
	gravity_ballistic_angle_rad=0.5f*(-asin((G*dis_m*(1-sin_tar_angle*sin_tar_angle)-sin_tar_angle*SHOOT_V_2)/SHOOT_V_2)+tar_angle_rad);
	gravity_ballistic_angle=gravity_ballistic_angle_rad*57.3f;

	*pitch_tarP=PITCH_GYRO_INIT-gravity_ballistic_angle*8192.0f/360;//////////////////////////////

	return gravity_ballistic_angle;
}


u8 Judge_Send_Statu=0;	//刷新标志

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
//		 test_tarP=test_tarP_set;
//		 teat_out=testGravity_Ballistic_Set(&test_tarP,teat_dis);
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
