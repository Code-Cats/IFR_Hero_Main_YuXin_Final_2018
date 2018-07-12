#ifndef __YUN_H__

#define __YUN_H__

#include "main.h"

#define  YAW_INIT_DEFINE 3525//3160
//s32 YAW_INIT=YAW_INIT_DEFINE;
extern s32 YAW_INIT;
#define PITCH_INIT         3045//3098
#define PITCH_GYRO_INIT 3045//3098	//云台陀螺仪零点对应电机位置，陀螺仪更改位置需要重调	//反馈的太小，说明这个太大需要调

#define 		Yaw_MAX 			   YAW_INIT+1000
#define 		Yaw_MIN				   YAW_INIT-1000
#define 		Pitch_MAX 		   PITCH_INIT+250
#define 		Pitch_MIN 		   PITCH_INIT-600

#define RC_CONTROL 0	//遥控器操纵模式
#define PC_CONTROL 1	//键盘操纵模式


#define YUN_MOTOR_DATA_DEFAULT \
{\
	0,\
	0,\
	0,\
	PITCH_INIT,\
	0,\
	0,\
	0,\
	0,\
	YAW_INIT_DEFINE,\
	0,\
}


#define YUN_DATA_DEFAULT  {0};


/*PID参数-外接陀螺仪*/
#define PITCH_POSITION_PID_P 0.80//0.83
#define PITCH_POSITION_PID_I 0.008f
#define PITCH_POSITION_PID_D 1.1f	//1.2
#define PITCH_POSITION_PID_MER 400	//最大偏差输入
#define PITCH_POSITION_PID_MAXINPUT 8190	//最大期望输入
#define PITCH_POSITION_PID_MAXOUTPUT 320	//最大输出
#define PITCH_POSITION_PID_I_MAX 22/PITCH_POSITION_PID_I	//I累加限制
#define PITCH_POSITION_PID_DEAD 0	//计算死区3
//参数记录：
#define PITCH_SPEED_PID_P 25
#define PITCH_SPEED_PID_I 0.01f//0.014	//0.026
#define PITCH_SPEED_PID_D 1	//0.08
#define PITCH_SPEED_PID_MER 260	//最大偏差输入240
#define PITCH_SPEED_PID_MAXINPUT 320	//最大期望输入
#define PITCH_SPEED_PID_MAXOUTPUT 6000	//超过7000电机反转
#define PITCH_SPEED_PID_I_MAX 600/PITCH_SPEED_PID_I	//I累加限制1200
#define PITCH_SPEED_PID_DEAD 0	//计算死区3

///*PID参数-板载陀螺仪*/
//#define PITCH_POSITION_PID_P 0.92	//0.95
//#define PITCH_POSITION_PID_I 0
//#define PITCH_POSITION_PID_D 2	//1.2
//#define PITCH_POSITION_PID_MER 250	//最大偏差输入
//#define PITCH_POSITION_PID_MAXINPUT 6700	//最大期望输入
//#define PITCH_POSITION_PID_MAXOUTPUT 250	//最大输出
//#define PITCH_POSITION_PID_I_MAX 0	//I累加限制
//#define PITCH_POSITION_PID_DEAD 1	//计算死区3
////参数记录：
//#define PITCH_SPEED_PID_P 17.5
//#define PITCH_SPEED_PID_I 0.028	//0.03
//#define PITCH_SPEED_PID_D 2	//0.08
//#define PITCH_SPEED_PID_MER 240	//最大偏差输入300
//#define PITCH_SPEED_PID_MAXINPUT 240	//最大期望输入
//#define PITCH_SPEED_PID_MAXOUTPUT 5300	//最大输出5500
//#define PITCH_SPEED_PID_I_MAX 450/PITCH_SPEED_PID_I	//I累加限制1200
//#define PITCH_SPEED_PID_DEAD 1	//计算死区3


#define YAW_SPEED_PID_P 45//44//45	//30  
#define YAW_SPEED_PID_I 0.025//0.03	//0.02
#define YAW_SPEED_PID_D 1
#define YAW_SPEED_PID_MER 260	//最大偏差输入
#define YAW_SPEED_PID_MAXINPUT 320	//最大期望输入
#define YAW_SPEED_PID_MAXOUTPUT 7000	//最大输出
#define YAW_SPEED_PID_I_MAX 1500/YAW_SPEED_PID_I	//I累加限制
#define YAW_SPEED_PID_DEAD 0	//计算死区

#define YAW_POSITION_PID_P 1.3f//1.4//1.2  
#define YAW_POSITION_PID_I 0.004f
#define YAW_POSITION_PID_D 1	//经过实验观察数据发现D过大引起tarV波动过大，但D对于减弱波形超调猜测有效用，采用非线性D	//原值6
#define YAW_POSITION_PID_MER 280	//最大偏差输入
#define YAW_POSITION_PID_MAXINPUT 5350	//最大期望输入
#define YAW_POSITION_PID_MAXOUTPUT 300	//最大输出
#define YAW_POSITION_PID_I_MAX 15/YAW_POSITION_PID_I	//I累加限制
#define YAW_POSITION_PID_DEAD 0	//计算死区

//PITCH电机位置环参数
#define PID_PITCH_POSITION_DEFAULT \
{\
	PITCH_POSITION_PID_P,\
	PITCH_POSITION_PID_I,\
  PITCH_POSITION_PID_D,\
	PITCH_POSITION_PID_MER,\
	-PITCH_POSITION_PID_MAXINPUT,\
	PITCH_POSITION_PID_MAXINPUT,\
	-PITCH_POSITION_PID_MAXOUTPUT,\
	PITCH_POSITION_PID_MAXOUTPUT,\
	PITCH_POSITION_PID_I_MAX,\
	{0.0,0.0},\
	0.0,\
	0.0,\
	0.0,\
	PITCH_POSITION_PID_DEAD,\
}\

//PITCH电机速度环参数
#define PID_PITCH_SPEED_DEFAULT \
{\
	PITCH_SPEED_PID_P,\
	PITCH_SPEED_PID_I,\
  PITCH_SPEED_PID_D,\
	PITCH_SPEED_PID_MER,\
	-PITCH_SPEED_PID_MAXINPUT,\
	PITCH_SPEED_PID_MAXINPUT,\
	-PITCH_SPEED_PID_MAXOUTPUT,\
	PITCH_SPEED_PID_MAXOUTPUT,\
	PITCH_SPEED_PID_I_MAX,\
	{0.0,0.0},\
	0.0,\
	0.0,\
	0.0,\
	PITCH_SPEED_PID_DEAD,\
}\

//YAW电机速度环参数
#define PID_YAW_SPEED_DEFAULT \
{\
	YAW_SPEED_PID_P,\
	YAW_SPEED_PID_I,\
  YAW_SPEED_PID_D,\
	YAW_SPEED_PID_MER,\
	-YAW_SPEED_PID_MAXINPUT,\
	YAW_SPEED_PID_MAXINPUT,\
	-YAW_SPEED_PID_MAXOUTPUT,\
	YAW_SPEED_PID_MAXOUTPUT,\
	YAW_SPEED_PID_I_MAX,\
	{0.0,0.0},\
	0.0,\
	0.0,\
	0.0,\
	YAW_SPEED_PID_DEAD,\
}\

//YAW电机位置环参数
#define PID_YAW_POSITION_DEFAULT \
{\
	YAW_POSITION_PID_P,\
	YAW_POSITION_PID_I,\
  YAW_POSITION_PID_D,\
	YAW_POSITION_PID_MER,\
	-YAW_POSITION_PID_MAXINPUT,\
	YAW_POSITION_PID_MAXINPUT,\
	-YAW_POSITION_PID_MAXOUTPUT,\
	YAW_POSITION_PID_MAXOUTPUT,\
	YAW_POSITION_PID_I_MAX,\
	{0.0,0.0},\
	0.0,\
	0.0,\
	0.0,\
	YAW_POSITION_PID_DEAD,\
}\

typedef struct  __YUN_MOTOR_DATA__
{
		int32_t   pitch_fdbP;  //俯仰轴电机
		int32_t   pitch_fdbV;
		int32_t   pitch_tarV;
		float   pitch_tarP;	//现在以角度为单位
    int32_t   pitch_output;

		int32_t   yaw_fdbP;  //航向轴电机
		int32_t   yaw_fdbV;
		int32_t   yaw_tarV;
		float   yaw_tarP;
    int32_t   yaw_output;
}YUN_MOTOR_DATA;

typedef struct  __YUN_DATA__
{
	float pitch_move;
	float yaw_move;

	char Turo_State;
	char Fire_State;
	char Right_State;
	char Left_State;
	int16_t value_mouse_X;
	int16_t value_mouse_Y;
}YUN_DATA;

s32 yun_pitch_tarV(s32 now_V);

float Yaw_Angle_Calculate(int16_t src_angle,int16_t Init);
void Yun_Init(void);
s32 Yaw_output_offset(s32 yaw_fbdP);	//克服云台yaw轴非线性力及非对称性的补偿 //虽然yaw云台阻尼曲线满足收敛，但因为参照物并非为云台电机，故应采用当前反馈位置做参照
s16 Pitch_output_offset(s32 pitch_tarP);	//克服云台pitch轴非线性力及非对称性的补偿	//因为云台pitch阻尼曲线满足收敛（在外部激励情况下只存在一个最小值），故采用tarP作为补偿参照可以提高间接反应速度

void Yun_Task(void);	//云台控制任务 
void Yun_Control_External_Solution(void);	//外置反馈方案
void Yun_Control_Inscribe_Solution(void);	//内接反馈方案
//void Yun_WorkState_Turn_Task(void);	//模式切换时云台转向任务

s32 t_yun_yaw_tarV(s32 now_V);	//对速度的一阶滞后滤波

void __yun_yaw_offset(void);

void RC_Control_Yun(float * yaw_tarp,float * pitch_tarp);	//1000Hz
void PC_Control_Yun(float * yaw_tarp,float * pitch_tarp);	//1000Hz


#endif

