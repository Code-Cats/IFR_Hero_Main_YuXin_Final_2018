#include "auto_takebullet.h"
#include "main.h"

#define LIFT_DISTANCE_BULLET 700

TakeBulletState_e TakeBulletState=BULLET_ACQUIRE;	//���Զ���ȡ��״̬λ

u8 take_bullet_auto_move_statu=0;	//ȡ���Զ�ƽ�Ʊ�־����ÿһ���Զ�ȡ���������ڽ���ʱ��������������ȡ��ʱ�������ڼ���Ҫ��ʱ����chassis��RC���ƣ�ִ������Զ�����	//��ʱδ�ӣ���Ϊ��ҩ���޷���֤��������λ
u8 take_bullet_end_state=0;	//һ��ȡ�������ı�־λ	//�����ǲ����г���ʲô״̬��ȡ���ܹ�˳����ɣ������������ȵȲ��ܸÿ��ƣ��г�ȡ������Ԥ����Ҫ�����޸�

extern u32 time_1ms_count;
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
extern RC_Ctl_t RC_Ctl;
extern ViceControlDataTypeDef ViceControlData;
extern PID_GENERAL PID_Chassis_Speed[4];
extern u8 Replenish_Bullet_Statu;	//����ģʽ������״̬

#define STEER_UP_L_INIT 560//
#define STEER_UP_R_INIT 2500//1950	//
#define STEER_UP_L_REVERSAL 1750//
#define STEER_UP_R_REVERSAL 1300//
float pwm_l_t=STEER_UP_L_INIT;
float pwm_r_t=STEER_UP_R_INIT;


u8 valve_fdbstate[6]={0};	//��¼�Ƿ�����ķ�����־
u8 servo_fdbstate[2]={0};
const u32 valve_GOODdelay[6]={300,1200,300,1000,1000,1000};	//�����룬��ʱ����
const u32 valve_POORdelay[6]={300,1200,300,1000,1000,1000};	//�����룬��ʱ����
const u32 servo_GOODdelay[2]={2500,800};	//��ʱ����	//��һ��Ϊ2500�ǽ��ӵ����µ���ʱҲ�ӽ�ȥ�ˣ���Ϊ�����ת���ӵ��������������һ���
const u32 servo_POORdelay[2]={500,500};	//��ʱ����


//#define VALVE_BULLET_HORIZONTAL1 0		//ԭ�ǵ�--��ƽ��1
//#define VALVE_BULLET_HORIZONTAL2 1	//ԭǰ��--��ƽ��2
//#define VALVE_BULLET_CLAMP 2	//�н�

void TakeBullet_Control_Center(void)
{
	static u8 swicth_Last_state=0;	//�Ҳ���
	
	static u8 valve_last[6]={0};	//��¼��һ����ֵ	//�����빤�̳�������
	static u8 servo_last[2]={0};	//��¼��һ����ֵ	//�����빤�̳�������
	
	static u32 valve_startGOOD_time[6]={0};	//��¼˳�򴥷�ʱ��	//�����빤�̳�������
	static u32 servo_startGOOD_time[2]={0};	//��¼˳�򴥷�ʱ��	//�����빤�̳�������
	static u32 valve_startPOOR_time[6]={0};	//��¼���򴥷�ʱ��	//�����빤�̳�������
	static u32 servo_startPOOR_time[2]={0};	//��¼���򴥷�ʱ��	//�����빤�̳�������
	
////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//�Զ�ȡ������λ
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	/******************************************************************
��������forΪ���������ⷽ��	//������ʱ�̶����м��
�ֱ�Ϊ��
1.�������½��صĴ���ʱ���¼
2.���ݴ���ʱ������ݷ���ֵ����
3.���ݵ���
******************************************************************/
	for(int i=0;i<6;i++)	//����ʱ���
	{
		if(valve_last[i]==0&&ViceControlData.valve[i]==1)	//�������
		{
			valve_startGOOD_time[i]=time_1ms_count;
		}
		else if(valve_last[i]==1&&ViceControlData.valve[i]==0)//�ջش���
		{
			valve_startPOOR_time[i]=time_1ms_count;
		}
		
		if(i<2)
		{
			if(servo_last[i]==0&&ViceControlData.servo[i]==1)
			{
				servo_startGOOD_time[i]=time_1ms_count;
			}
			else if(servo_last[i]==1&&ViceControlData.servo[i]==0)
			{
				servo_startPOOR_time[i]=time_1ms_count;
			}
		}
	}
	
	for(int i=0;i<6;i++)	//��������λ
	{
		if(ViceControlData.valve[i]==1&&time_1ms_count-valve_startGOOD_time[i]>valve_GOODdelay[i])	//����ֵΪ��������λ��ʱ����ͳһ��Ϊ1000ms
		{
			valve_fdbstate[i]=1;
		}
		else if(ViceControlData.valve[i]==0&&time_1ms_count-valve_startPOOR_time[i]>valve_POORdelay[i])	//����ֵΪ�ջ�����λ��ʱ����ͳһ��Ϊ1000ms
		{
			valve_fdbstate[i]=0;
		}
		
		if(i<2)
		{
			if(ViceControlData.servo[i]==1&&time_1ms_count-servo_startGOOD_time[i]>servo_GOODdelay[i])	//����ֵΪ��������λ��ʱ����ͳһ��Ϊ1000ms
			{
				servo_fdbstate[i]=1;
			}
			else if(ViceControlData.servo[i]==0&&time_1ms_count-servo_startPOOR_time[i]>servo_POORdelay[i])	//����ֵΪ�ջ�����λ��ʱ����ͳһ��Ϊ1000ms
			{
				servo_fdbstate[i]=0;
			}
		}
	}
	
	for(int i=0;i<6;i++)	//������
	{
		valve_last[i]=ViceControlData.valve[i];
		if(i<2)	servo_last[i]=ViceControlData.servo[i];
	}
////////////////////////////////////////////////////////////////////////////////////////////�������־
	
	
	//���ִ�п�	//��ŷ��ڸ���ִ��
	if(Replenish_Bullet_Statu!=1)	//��ȡ��ģʽ
	{
		if(ViceControlData.servo[0]==0)
		{
			if(pwm_l_t-STEER_UP_L_INIT>0.01f)
			{
				pwm_l_t-=5;
			}
			else
			{
				pwm_l_t=STEER_UP_L_INIT;
			}
			
			if(STEER_UP_R_INIT-pwm_r_t>0.01f)
			{
				pwm_r_t+=5;
			}
			else
			{
				pwm_r_t=STEER_UP_R_INIT;
			}
		}
		else
		{
			if(STEER_UP_L_REVERSAL-pwm_l_t>0.01f)
			{
				pwm_l_t+=5;
			}
			else
			{
				pwm_l_t=STEER_UP_L_REVERSAL;
			}
			
			if(pwm_r_t-STEER_UP_R_REVERSAL>0.01f)
			{
				pwm_r_t-=5;
			}
			else
			{
				pwm_r_t=STEER_UP_R_REVERSAL;
			}
		}

	}
	else	//����ģʽ���
	{
		pwm_l_t=1100;
		pwm_r_t=2100;
	}
	
	
	
	swicth_Last_state=RC_Ctl.rc.switch_right;
	
	PWM3_1=(u16)pwm_l_t;
  PWM3_2=(u16)pwm_r_t;
}

/********************************************************************
u8 SetCheck_TakeBullet_TakeBack_statu=0;	//�г�ȡ������ִ�б�־λ
void SetCheck_TakeBullet_TakeBack(void)	//�г�ȡ��������λ����
{
	if(SetCheck_TakeBullet_TakeBack_statu==1&&auto_takebullet_statu==0)//��״̬Ϊ���µ�1
	{
		ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
		ViceControlData.valve[VALVE_BULLET_PROTRACT]=0;
		
		if(valve_fdbstate[VALVE_BULLET_PROTRACT]==0)
		{
			SetCheck_FrontLift(0);//�г�����
			SetCheck_BackLift(0);
			if(SetCheck_FrontLift(0)==1&&SetCheck_BackLift(0)==1)
				SetCheck_TakeBullet_TakeBack_statu=0;
//			return 1;
		}
	}
//	return 0;
}



#define LIFT_DISTANCE_GRIPBULLET	490	//�е�ҩ��ʱ�߶�
#define LIFT_DISTANCE_DISGRIPBULLET	1400	//��������ҩ��߶�
#define LIFT_DISTANCE_SLOPEBACKBULLET	1400	//��бʱ���ȸ߶�
#define LIFT_DISTANCE_SLOPEFRONTBULLET	1400	//��бʱǰ�ȸ߶�
extern LIFT_DATA lift_Data;

u8 SetCheck_GripLift(u8 grip_state)	//�Ƿ��뵯ҩ��ƽ��,gripץס����˼	//0��ʾ��ץס������Ҫ����ҩ������ҩ��߶ȣ�1��ʾץס������Ҫ�н���ҩ��ʱ�ĸ߶�
{
	lift_Data.lf_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.rf_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.lb_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.rb_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_GRIPBULLET);
	
	return (abs(lift_Data.lf_lift_fdbP+lift_Data.rf_lift_fdbP-2*(LIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_GRIPBULLET)))<30);	//�����ǽ���ǰ����Ϊ�������ص�
}

u8 SetCheck_SlopeLift(u8 slope_state)	//��ʱֻ������	slope��б����˼	//0��ʾ����б�����ָ�������ҩ��߶ȣ�1��ʾ��б������б���ӵ�״̬
{
	lift_Data.lb_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(slope_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_SLOPEBACKBULLET);
	lift_Data.rb_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(slope_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_SLOPEBACKBULLET);
	
	lift_Data.lf_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(slope_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_SLOPEFRONTBULLET);
	lift_Data.rf_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(slope_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_SLOPEFRONTBULLET);
	
	return (abs(lift_Data.lb_lift_fdbP+lift_Data.rb_lift_fdbP-2*(LIFT_DISTANCE_DISGRIPBULLET-(slope_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_SLOPEBACKBULLET)))<30);	//�����ǽ���ǰ����Ϊ�������ص�,��Ϊǰ����Ϊ�������г̸���ʱ���������ǰ���ѵ�λ�����һ����λ
}


*****************************************************************/



