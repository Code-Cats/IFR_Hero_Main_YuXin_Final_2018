#include "mpu6050_it.h"

extern MPU6050_RAW_DATA    MPU6050_Raw_Data; 
extern MPU6050_REAL_DATA    MPU6050_Real_Data; 
extern int gyroADC_X_offset,gyroADC_Y_offset,gyroADC_Z_offset;
extern GYRO_RADIAN_DATA     Gyro_Radian_Data;
extern ACCEL_AVERAGE_DATA   Accel_Raw_Average_Data;

void MPU6050_IntConfiguration(void)
{
		GPIO_InitTypeDef    gpio;
    NVIC_InitTypeDef    nvic;
    EXTI_InitTypeDef    exti;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,  ENABLE);   
		gpio.GPIO_Pin = GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &gpio);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,GPIO_PinSource4); 
    exti.EXTI_Line = EXTI_Line4;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿中断
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);
    
    nvic.NVIC_IRQChannel = EXTI4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

//MPU6050 外部中断处理函数
void EXTI4_IRQHandler(void)         //中断频率1KHz
{
	  

    if(EXTI_GetITStatus(EXTI_Line4) == SET)
    {
        
        //读取MPU6050数据,为了使云台的控制更平滑，
        //使用MPU6050的陀螺仪输出作为速度环反馈
        //单纯使用电调板返回机械角度值做速度环反馈，会有明显振荡现象
       MPU6050_ReadData(); 
//			MPU6050_Data_Filter();
//			MPU6050_Angle_Calculate(Gyro_Radian_Data.X,Gyro_Radian_Data.Y,Gyro_Radian_Data.Z,Accel_Raw_Average_Data.X,Accel_Raw_Average_Data.Y,Accel_Raw_Average_Data.Z);	//误差太大
     
       EXTI_ClearFlag(EXTI_Line4);          
  		 EXTI_ClearITPendingBit(EXTI_Line4);
    }
}
