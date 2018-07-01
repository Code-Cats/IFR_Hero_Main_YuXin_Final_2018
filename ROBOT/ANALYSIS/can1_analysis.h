#ifndef __CAN1_ANALYSIS__
#define __CAN1_ANALYSIS__

#include "main.h"


typedef struct
{
	s16 real[8];
//	s16 sum;
	s16 calc;	//	两次一平均的角度数值
	s16 calc_diff;
	s16 calc_last;	//上一次计算的角度值
	s16 turns;	//监视两次角度值之差得出的圈数相对值
}LIFT_POSITION_ENCODER;	//CAN1


void CAN1_Feedback_Analysis(CanRxMsg *rx_message);
//void CAN1_Lift_SendMsg(int motor_201,int motor_202,int motor_203,int motor_204);
void CAN1_Yun_SendMsg(int16_t motor_205,int16_t motor_206);	//yaw  pitch
void CAN2_Shoot_Bullet_SendMsg(int16_t motor_201,int16_t motor_202,int16_t motor_203,int16_t motor_204);
void CAN_Motor6623_calibration(void);

void Position_Data_deal_DIV8(s32 * value,LIFT_POSITION_ENCODER *Receive,CanRxMsg * msg);	//分辨率转子1/8圈
void Position_Data_deal_DIV81(s32 * value,LIFT_POSITION_ENCODER *Receive,CanRxMsg * msg);	//分辨率转子1/81圈
void Position_To_Turns(LIFT_POSITION_ENCODER *Receive);	//按照6倍采样来计算，机械角度共8192个挡位，则过界表现差值为6826
void Speed_Data_deal(s32 * fdbV,CanRxMsg * msg);
#endif

