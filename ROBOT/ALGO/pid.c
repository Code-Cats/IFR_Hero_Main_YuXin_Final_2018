#include "pid.h"



/****************************************
函数名称：PID_General
函数功能：使被控对象更稳定的达到目标值
函数参数：目标值：target
          当前值：current
					PID结构体参数：PID_GENERAL
函数返回值：输出:output
描述：通用PID,传统PID
*****************************************/
float PID_General(float target,float current,PID_GENERAL *pid)
{
	  //目标数据大小限制
		if(target>pid->input_max){target=pid->input_max;}
		if(target<pid->input_min){target=pid->input_min;}
    
    pid->Data_Save[0] = pid->Data_Save[1];
	  pid->Data_Save[1] = target - current;
		
		if(target - current>pid->input_maxerror)pid->Data_Save[1]=pid->input_maxerror;
		else if(target - current<-pid->input_maxerror)pid->Data_Save[1]=-pid->input_maxerror;
		
		if(abs(target - current)<=pid->tiny)pid->Data_Save[1]=0;
		
		pid->inte = 0.999f*pid->inte+pid->Data_Save[1];
		pid->dire = pid->k_d * (pid->Data_Save[1] - pid->Data_Save[0]);

		if(pid->inte>pid->i_sum_max){pid->inte=pid->i_sum_max;}
		if(pid->inte<-pid->i_sum_max){pid->inte=-pid->i_sum_max;}
pid->t_error=(s16)(pid->Data_Save[1]*pid->k_p);
pid->t_dire=(s16)pid->dire;
pid->t_inte=(s16)(pid->inte * pid->k_i);
		pid->output = pid->Data_Save[1] * pid->k_p + pid->inte * pid->k_i + pid->dire; 
	
		//输出数据大小限制
		if(pid->output>pid->output_max){pid->output=pid->output_max;}
		if(pid->output<pid->output_min){pid->output=pid->output_min;}
		return pid->output;
}


float PID_ChassisFollow_Variable_kp(float error)	//底盘电机8192为360度，15为341，
{
	float kp=1;
	if(abs(error)<400)
	{
		kp=abs(error)/400+0.4f;
		kp=kp>1?1:kp;
		kp=kp<0?0:kp;
	}
	return kp;
}

/****************************************
函数名称：PID_ChassisFollow
函数功能：使被控对象更稳定的达到目标值
函数参数：目标值：target
          当前值：current
					PID结构体参数：PID_ChassisFollow
函数返回值：输出:output
描述：底盘跟随用的PID
*****************************************/
float PID_ChassisFollow(float target,float current,PID_GENERAL *pid)
{
	  //目标数据大小限制
		if(target>pid->input_max){target=pid->input_max;}
		if(target<pid->input_min){target=pid->input_min;}
    
    pid->Data_Save[0] = pid->Data_Save[1];
	  pid->Data_Save[1] = target - current;
		
		if(target - current>pid->input_maxerror)pid->Data_Save[1]=pid->input_maxerror;
		else if(target - current<-pid->input_maxerror)pid->Data_Save[1]=-pid->input_maxerror;
		
		if(abs(target - current)<=pid->tiny)pid->Data_Save[1]=0;
		
		pid->inte = 0.999f*pid->inte+pid->Data_Save[1];
		pid->dire = pid->k_d * (pid->Data_Save[1] - pid->Data_Save[0]);

		if(pid->inte>pid->i_sum_max){pid->inte=pid->i_sum_max;}
		if(pid->inte<-pid->i_sum_max){pid->inte=-pid->i_sum_max;}
pid->t_error=(s16)(pid->Data_Save[1]*pid->k_p);
pid->t_dire=(s16)pid->dire;
pid->t_inte=(s16)(pid->inte * pid->k_i);
		pid->output = pid->Data_Save[1] * pid->k_p *PID_ChassisFollow_Variable_kp(pid->Data_Save[1]) + pid->inte * pid->k_i + pid->dire; 
	
		//输出数据大小限制
		if(pid->output>pid->output_max){pid->output=pid->output_max;}
		if(pid->output<pid->output_min){pid->output=pid->output_min;}
		return pid->output;
}



float PID_Robust(float target,float current,float differential,PID_GENERAL *pid)
{
	//目标数据大小限制
		if(target>pid->input_max){target=pid->input_max;}
		if(target<pid->input_min){target=pid->input_min;}
    
    pid->Data_Save[0] = pid->Data_Save[1];
	  pid->Data_Save[1] = target - current;
		
		if(target - current>pid->input_maxerror)pid->Data_Save[1]=pid->input_maxerror;
		else if(target - current<-pid->input_maxerror)pid->Data_Save[1]=-pid->input_maxerror;
		
		if(abs(target - current)<=pid->tiny)pid->Data_Save[1]=0;
		
		pid->inte = 0.999f*pid->inte+pid->Data_Save[1];
		pid->dire = pid->k_d * differential;

		if(pid->inte>pid->i_sum_max){pid->inte=pid->i_sum_max;}
		if(pid->inte<-pid->i_sum_max){pid->inte=-pid->i_sum_max;}
pid->t_error=(s16)(pid->Data_Save[1]*pid->k_p);
pid->t_dire=(s16)pid->dire;
pid->t_inte=(s16)(pid->inte * pid->k_i);
		pid->output = pid->Data_Save[1] * pid->k_p + pid->inte * pid->k_i + pid->dire; 
	
		//输出数据大小限制
		if(pid->output>pid->output_max){pid->output=pid->output_max;}
		if(pid->output<pid->output_min){pid->output=pid->output_min;}
		return pid->output;
}

