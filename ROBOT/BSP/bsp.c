#include "bsp.h"
/******************************************
函数名称;BSP_Init
函数功能：对板级设备初始化配置
函数参数：无
函数返回值：无
函数描述：无
******************************************/
s16 t_error_iic=0;
void BSP_Init(void)
{  
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
   LED_Config();
	 PWM_Config();
	 Laser_Config();
	 USART1_DMA_Config(100000);  
   USART6_ViceBoard_Init(115200);
	 USART3_Configuration(115200);
//	 USART5_WIFIDEBUG_Init(115200);
//	USART6_WIFIDEBUG_Init(115200);
//////	 t_error_iic=MPU6050_Initialization();//
//////   MPU6050_Gyro_calibration();//
//////	 MPU6050_IntConfiguration(); //
	 uart4_init();              										  //UART4 INIT UART4->GYRO PORT
	 imu_data_decode_init();
	 delay_ms(1000);
	 TIM6_Configuration();
	 CAN1_Motors_Config();
	 CAN2_Motors_Config();
  delay_ms(1600+0);
	 TIM6_Start();			
}

