#ifndef _PID_H_
#define _PID_H_

#include "main.h"

typedef __packed struct
{
		float   k_p;
		float   k_i;
		float   k_d;
		float input_maxerror;
    const int32_t input_min;
    const int32_t input_max;
    const int32_t output_min;
    const int32_t output_max;
		float i_sum_max;
	  float Data_Save[2];
    float inte;
    float dire;
    float output;	
	  const int32_t tiny;
	s16 t_error;
	s16 t_dire;
	s16 t_inte;
}PID_GENERAL;


float PID_ChassisFollow_Variable_kp(float error);	//底盘电机8192为360度，15为341，
float PID_General(float target,float current,PID_GENERAL *pid);
float PID_ChassisFollow(float target,float current,PID_GENERAL *pid);
float PID_Robust(float target,float current,float differential,PID_GENERAL *pid);

#endif
