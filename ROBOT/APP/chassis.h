#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "main.h"

#define RC_CONTROL 0	//遥控器操纵模式
#define PC_CONTROL 1	//键盘操纵模式

#define CHASSIS_SPEED_PID_P 3.0f	//底盘PID参数
#define CHASSIS_SPEED_PID_I 0.035f
#define CHASSIS_SPEED_PID_D 0
#define CHASSIS_SPEED_MAXOUT 8000
#define CHASSIS_SPEED_MAXINPUT 8200
#define CHASSIS_SPEED_MER 3200
#define CHASSIS_SPEED_DEAD 0
#define CHASSIS_SPEED_I_MAX 2500/CHASSIS_SPEED_PID_I

//底盘电机速度环PID参数
#define PID_CHASSIS_SPEED_DEFAULT \
{\
	CHASSIS_SPEED_PID_P,\
	CHASSIS_SPEED_PID_I,\
  CHASSIS_SPEED_PID_D,\
	CHASSIS_SPEED_MER,\
	-CHASSIS_SPEED_MAXINPUT,\
	CHASSIS_SPEED_MAXINPUT,\
	-CHASSIS_SPEED_MAXOUT,\
	CHASSIS_SPEED_MAXOUT,\
	CHASSIS_SPEED_I_MAX,\
	{0.0,0.0},\
	0.0,\
	0.0,\
	0.0,\
	CHASSIS_SPEED_DEAD,\
}\

#define CHASSIS_FOLLOW_PID_P 0.57f//0.6f//0.6有一定超调2018.4.23
#define CHASSIS_FOLLOW_PID_I 0
#define CHASSIS_FOLLOW_PID_D 17
#define CHASSIS_FOLLOW_PID_MAXINPUT 7000//6000	//云台活动范围：4450-5350
#define CHASSIS_FOLLOW_PID_MAXOUT 400
#define CHASSIS_FOLLOW_PID_MER 600
#define CHASSIS_FOLLOW_PID_DEAD 6//8//10
#define CHASSIS_FOLLOW_PID_I_MAX 0

//底盘跟随速度环PID参数
#define PID_CHASSIS_FOLLOW_DEFAULT \
{\
	CHASSIS_FOLLOW_PID_P,\
	CHASSIS_FOLLOW_PID_I,\
  CHASSIS_FOLLOW_PID_D,\
	CHASSIS_FOLLOW_PID_MER,\
	-CHASSIS_FOLLOW_PID_MAXINPUT,\
	CHASSIS_FOLLOW_PID_MAXINPUT,\
	-CHASSIS_FOLLOW_PID_MAXOUT,\
	CHASSIS_FOLLOW_PID_MAXOUT,\
	CHASSIS_FOLLOW_PID_I_MAX,\
	{0.0,0.0},\
	0.0,\
	0.0,\
	0.0,\
	CHASSIS_FOLLOW_PID_DEAD,\
}\


#define YAW_FOLLOW_FILTER_DEFAULT \
{\
	0.5f,\
	1,\
	0,\
	7,\
	{0,0},\
	0.0,\
}\


typedef struct
{
	 int32_t lf_wheel_fdbV; //201 
	 int32_t rf_wheel_fdbV; //202
	 int32_t lb_wheel_fdbV; //203
	 int32_t rb_wheel_fdbV;	//204	  
	
   int32_t lf_wheel_tarV;
	 int32_t rf_wheel_tarV;
	 int32_t lb_wheel_tarV;
	 int32_t rb_wheel_tarV;
	  
   int32_t lf_wheel_fdbP;
	 int32_t rf_wheel_fdbP;
	 int32_t lb_wheel_fdbP;
	 int32_t rb_wheel_fdbP;
	 
	 int32_t lf_wheel_tarP;
	 int32_t rf_wheel_tarP;
	 int32_t lb_wheel_tarP;
	 int32_t rb_wheel_tarP;
	 
	 int32_t lf_wheel_output;
	 int32_t rf_wheel_output;	
	 int32_t lb_wheel_output;
	 int32_t rb_wheel_output;
	
}CHASSIS_DATA;	//CAN2


void Remote_Task(void);

void Chassis_Control_External_Solution(void);	//外置反馈方案
void Chassis_Control_Inscribe_Solution(void);	//内接反馈方案

void RC_Control_Chassis(void);
void PC_Control_Chassis(s16 * chassis_vx,s16 * chassis_vy);	//1000Hz
s16 chassis_Vw_filter(s16 now_V);
float Limit_Power(float power,float powerbuffer);	//英雄120W热量限制，直接限制总输出
void Overall_Motion_Ratio_Protect(CHASSIS_DATA* chassis_data);	//整体轮速比例保护,一定要放在整体轮速解算出来后
void Extended_Integral_PID(CHASSIS_DATA* chassis_data);	//扩展型整体PID，适用于任意动作场景	2018.4.19

#endif

