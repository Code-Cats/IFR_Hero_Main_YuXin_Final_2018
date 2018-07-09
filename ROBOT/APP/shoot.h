#ifndef __SHOOT_H_
#define __SHOOT_H_
#include "stm32f4xx.h"
#include "main.h"

////				P  I D MAXIN MAXOUT MAX_I
////ԭ������20 0 0 10890 12000 12000
////�ٶȻ���1 0.01 0 12000 8000 4900
//λ�÷�����С��λ�û������������������Ʊ�С����������ٶȻ�����
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



//���̵��λ�û�PID����
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

//��������ٶȻ�PID����
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
	u32 last_time;	//��һ�����ʱ�䣬�Դ˿���������Ƶ
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
	/*fdbp:��ǰ��е�Ƕ�
  fdbv:��ǰת��
  Tarp:Ŀ���е�Ƕ�
  Tarv:Ŀ��ת��*/
	s32 fdbP;	//����������
	
	s16 fdbP_raw;	//λ�õ�ԭʼ����
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

void Shoot_Instruction(void);	//����ָ��ģ��
void Shoot_Task(void); 
void Shoot_Feedback_Deal(SHOOT_DATA *shoot_data,SHOOT_MOTOR_DATA *shoot_motor_data,CanRxMsg *msg);
void Prevent_Jam_Down(SHOOT_DATA * shoot_data,SHOOT_MOTOR_DATA * shoot_motor_Data);	//����������	//ͬʱ�����������Ĺ���	//����tarP�����֮��
void Prevent_Jam_Up(SHOOT_DATA * shoot_data,SHOOT_MOTOR_DATA * shoot_motor_Data);	//����������	//ͬʱ�����������Ĺ���	//����tarP�����֮��
void Shoot_Frequency_Limit(int* ferquency,u16 rate,u16 heat);	//m/sΪ��λ

u8 Shoot_Heat_Limit(u16 heat,u16 maxheat);	//��Ӧ��������Ƶ
u8 Shoot_Heat_Lost_Fre_Limit(void);	//����lost�������Ƶ�����ƣ�������1��OK

u16 Friction_Adjust_DependOn_Vol(float voltage);	//����Ƶ��10HZ

#endif
