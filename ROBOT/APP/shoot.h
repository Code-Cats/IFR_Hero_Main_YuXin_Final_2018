#ifndef __SHOOT_H_
#define __SHOOT_H_
#include "stm32f4xx.h"
#include "main.h"

////				P  I D MAXIN MAXOUT MAX_I
////原参数：20 0 0 10890 12000 12000
////速度环：1 0.01 0 12000 8000 4900
//位置反馈变小，位置环计算参数变大，输入限制变小，输出不表，速度环不变
#define SHOOT_POSITION_PID_KP            80//120
#define SHOOT_POSITION_PID_KI            0.01f
#define SHOOT_POSITION_PID_KD            0.01	//0
#define SHOOT_POSITION_PID_MER					10000
#define SHOOT_POSITION_PID_I_MAX				0.0f
#define SHOOT_POSITION_PID_MAXIN        60000
#define SHOOT_POSITION_MAXOUT           12000	//12000

#define SHOOT_SPEED_PID_KP           2	//0.32
#define SHOOT_SPEED_PID_KI           0.01f	//0.002
#define SHOOT_SPEED_PID_KD           0.0f 
#define SHOOT_SPEED_PID_MER					7000	//4000
#define SHOOT_SPEED_PID_I_MAX 			3000.0f/SHOOT_SPEED_PID_KI//3000.0f/SHOOT_SPEED_PID_KI
#define SHOOT_SPEED_PID_MAXIN       12000
#define SHOOT_SPEED_MAXOUT      	  8000



//拨盘电机位置环PID参数
#define PID_SHOOT_POSITION_DEFAULT \
{\
	SHOOT_POSITION_PID_KP,\
	SHOOT_POSITION_PID_KI,\
  SHOOT_POSITION_PID_KD,\
	SHOOT_POSITION_PID_MER,\
	-SHOOT_POSITION_PID_MAXIN,\
	SHOOT_POSITION_PID_MAXIN,\
	-SHOOT_POSITION_MAXOUT,\
	SHOOT_POSITION_MAXOUT,\
	SHOOT_POSITION_PID_I_MAX,\
	{0.0,0.0},\
	0.0,\
	0.0,\
	0.0,\
	0,\
}\

//拨弹电机速度环PID参数
#define PID_SHOOT_SPEED_DEFAULT \
{\
	SHOOT_SPEED_PID_KP,\
	SHOOT_SPEED_PID_KI,\
  SHOOT_SPEED_PID_KD,\
	SHOOT_SPEED_PID_MER,\
	-SHOOT_SPEED_PID_MAXIN,\
	SHOOT_SPEED_PID_MAXIN,\
	-SHOOT_SPEED_MAXOUT,\
	SHOOT_SPEED_MAXOUT,\
	SHOOT_SPEED_PID_I_MAX,\
	{0.0,0.0},\
	0.0,\
	0.0,\
	0.0,\
	0,\
}\

typedef struct
{
	s16 count;
	s16 count_fdb;
	u32 last_time;	//上一次射击时间，以此可以限制射频
	u16 cycle;
	u16 frequency;
	float motor_tarP;
	struct
	{
		u16 distance;
		u8 priority;
	}Visual;
	struct
	{
		u16 rpm;
		u16 pwm_output;
	}Rate;
	struct
	{
		u8 sign;
		u32 count;
	}Jam;
}SHOOT_DATA;

#define SHOOT_DATA_INIT \
{\
	0,\
	0,\
	0,\
	0\
}

typedef struct
{
	/*fdbp:当前机械角度
  fdbv:当前转速
  Tarp:目标机械角度
  Tarv:目标转速*/
	s32 fdbP;	//处理后的数据
	
	s16 fdbP_raw;	//位置的原始数据
	s32 fdbP_raw_sum;
	s16 fdbP_raw_last;
	s16 fdbP_diff;
	
	s16 fdbV;
	
	s32 tarP;
	s16 tarV;

  float output;
}SHOOT_MOTOR_DATA;

void PC_Control_Shoot(u8* fri_state);
void RC_Control_Shoot(u8* fri_state);

void Shoot_Instruction(void);	//发弹指令模块
void Shoot_Task(void); 
void Shoot_Feedback_Deal(SHOOT_DATA *shoot_data,SHOOT_MOTOR_DATA *shoot_motor_data,CanRxMsg *msg);
void Prevent_Jam_Down(SHOOT_DATA * shoot_data,SHOOT_MOTOR_DATA * shoot_motor_Data);	//防卡弹程序	//同时包含防鸡蛋的功能	//放在tarP计算出之后
void Prevent_Jam_Up(SHOOT_DATA * shoot_data,SHOOT_MOTOR_DATA * shoot_motor_Data);	//防卡弹程序	//同时包含防鸡蛋的功能	//放在tarP计算出之后
void Shoot_Frequency_Limit(int* ferquency,u16 rate,u16 heat);	//m/s为单位

u8 Shoot_Heat_Limit(u16 heating,u8 level);	//限制热量
u8 Shoot_Heat_Lost_Fre_Limit(void);	//裁判lost情况对射频的限制，反返回1是OK

u16 Friction_Adjust_DependOn_Vol(float voltage);	//运算频率10HZ

#endif
