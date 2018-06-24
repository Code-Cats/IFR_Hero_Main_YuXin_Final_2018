#ifndef __CAN2_ANALYSIS__
#define __CAN2_ANALYSIS__

#include "main.h"

#define    MOTOR_FEEDBACK_DATA_DEFAULT    {0,0,0,0};

typedef struct  __MOTOR_FEEDBACK_DATA__
{
    int16_t chassis_lf_v; //201 
		int16_t chassis_rf_v; //202
		int16_t chassis_lb_v; //203
		int16_t chassis_rb_v;	//204	  

}MOTOR_FEEDBACK_DATA;  //底盘电机反馈数据

void CAN2_Feedback_Analysis(CanRxMsg *rx_message);
void CAN2_Chassis_SendMsg(int motor_201,int motor_202,int motor_203,int motor_204);
void CAN2_Shoot_SendMsg(int16_t motor_205,int16_t motor_206);
	
void Pitch_Board_Remove(void);
void Yaw_Board_Remove(void);
#endif

