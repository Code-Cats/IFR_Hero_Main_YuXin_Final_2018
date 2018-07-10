#include "auto_takebullet.h"
#include "main.h"

#define LIFT_DISTANCE_BULLET 700

TakeBulletState_e TakeBulletState=BULLET_OTHER;	//���Զ���ȡ��״̬λ

#define BULLETROTATE_OTHER	0	//��ȡ��λ��
#define BULLETROTATE_WAITING	530//-750//650	//�ȴ�����λ��ʱλ��
#define BULLETROTATE_ACQUIRE	1040	//ȡ��λ��
#define BULLETROTATE_POUROUT	120	//����λ��
#define BULLETROTATE_THROWOUT	960//-280//310	//�׳�λ��


extern u32 time_1ms_count;
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
extern RC_Ctl_t RC_Ctl;
extern ViceControlDataTypeDef ViceControlData;
extern PID_GENERAL PID_Chassis_Speed[4];
extern u8 Replenish_Bullet_Statu;	//����ģʽ������״̬

extern BULLETROTATE_DATA BulletRotate_Data;	//������

#define STEER_UP_L_INIT 560//
#define STEER_UP_R_INIT 2500//1950	//
#define STEER_UP_L_REVERSAL 1750//
#define STEER_UP_R_REVERSAL 1300//
float pwm_l_t=STEER_UP_L_INIT;
float pwm_r_t=STEER_UP_R_INIT;


u8 valve_fdbstate[6]={0};	//��¼�Ƿ�����ķ�����־
u8 servo_fdbstate[2]={0};
const u32 valve_GOODdelay[6]={350,350,300,1000,1000,1000};	//0--1//�����룬��ʱ����
const u32 valve_POORdelay[6]={350,350,100,1000,1000,1000};	//1--0//�����룬��ʱ����
const u32 servo_GOODdelay[2]={2500,800};	//��ʱ����	//��һ��Ϊ2500�ǽ��ӵ����µ���ʱҲ�ӽ�ȥ�ˣ���Ϊ�����ת���ӵ��������������һ���
const u32 servo_POORdelay[2]={500,500};	//��ʱ����


//#define VALVE_BULLET_HORIZONTAL1 0		//ԭ�ǵ�--��ƽ��1
//#define VALVE_BULLET_HORIZONTAL2 1	//ԭǰ��--��ƽ��2
//#define VALVE_BULLET_CLAMP 2	//�н�

u8 t_statu=0;
void TakeBullet_Control_Center(void)
{
	static u8 swicth_Last_state=0;	//�Ҳ���
	
	static u8 valve_last[6]={0};	//��¼��һ����ֵ	//�����빤�̳�������
	static u8 servo_last[2]={0};	//��¼��һ����ֵ	//�����빤�̳�������
	
	static u32 valve_startGOOD_time[6]={0};	//��¼˳�򴥷�ʱ��	//�����빤�̳�������
	static u32 servo_startGOOD_time[2]={0};	//��¼˳�򴥷�ʱ��	//�����빤�̳�������
	static u32 valve_startPOOR_time[6]={0};	//��¼���򴥷�ʱ��	//�����빤�̳�������
	static u32 servo_startPOOR_time[2]={0};	//��¼���򴥷�ʱ��	//�����빤�̳�������
	
	static WorkState_e State_Record=CHECK_STATE;
	
	static TakeBulletState_e takebulletstate_last=BULLET_OTHER;
	
	if(GetWorkState()==TAKEBULLET_STATE)	//5.9����//��һ��--��//ȡ��������DOWN-MID��ǰ�����-�н�һ�׸�DOWN-MID-->DOWN-DOWN;�����ת��DOWN-MID-->DOWN-UP
	{
		if(State_Record!=TAKEBULLET_STATE)
		{
			TakeBulletState=BULLET_WAITING;
		}
		
		if(RC_Ctl.rc.ch3-1024>80&&TakeBulletState==BULLET_WAITING)	/////////////////////////////�޸Ĳ���ģʽʱ��Ҫ�޸�
		{
			TakeBulletState=BULLET_ACQUIRE1;
		}
		else if(RC_Ctl.rc.ch3-1024<-80)
		{
			TakeBulletState=BULLET_WAITING;
		}			
	}	
	else
	{
//		if(State_Record==TAKEBULLET_STATE)
//		{
//		}
		
		if(TakeBulletState==BULLET_WAITING)
		{
			TakeBulletState=BULLET_OTHER;
		}
	}
			
	State_Record=GetWorkState();
	
////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//�Զ�ȡ������λ
	switch(TakeBulletState)	//�Զ�ȡ������
	{
		case BULLET_WAITING:	//�ȴ�ȡ����������λ��״̬
		{
			ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
			
			if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)	//�����ɿ�
			{
				BulletRotate_Data.tarP=BULLETROTATE_WAITING;
				if(abs(BulletRotate_Data.fdbP-BULLETROTATE_WAITING)<30)	//����ջ�
				{
					ViceControlData.valve[VALVE_BULLET_HORIZONTAL1]=0;
					ViceControlData.valve[VALVE_BULLET_HORIZONTAL2]=1;
				}
			}
			
			break;
		}
		case BULLET_ACQUIRE1:	//ǰ�졢�н���̧����	��֮Ϊ��ù���
		{
			BulletRotate_Data.tarP=BULLETROTATE_ACQUIRE;
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_ACQUIRE)<45)
			{
				ViceControlData.valve[VALVE_BULLET_CLAMP]=1;
				if(valve_fdbstate[VALVE_BULLET_CLAMP]==1)
				{
					TakeBulletState=BULLET_POUROUT1;	//�л�������
				}
			}
			break;
		}
		case BULLET_POUROUT1:	//������б�������ת	��֮Ϊ��������
		{
			
			BulletRotate_Data.tarP=BULLETROTATE_POUROUT;
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_POUROUT)<50)
			{
				TakeBulletState=BULLET_THROWOUT1;	//�л����ӳ�
			}
			break;
		}
		case BULLET_THROWOUT1:	//������ء�����̧�𡢼н��ɿ�	��֮Ϊ�������
		{
			
			if(valve_fdbstate[VALVE_BULLET_CLAMP]==1)	//�н�ʱ���£����º����
			{
				BulletRotate_Data.tarP=BULLETROTATE_THROWOUT;
			}
			
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_THROWOUT)<40)	//�Ż�ԭλ
			{
				ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
			}
			
			if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)	//�Ѿ��ɿ�����ʼ����
			{
				BulletRotate_Data.tarP=BULLETROTATE_WAITING;
				
				if((BulletRotate_Data.fdbP-BULLETROTATE_THROWOUT)<-60)	//΢΢̧���ʼƽ��
				{
					ViceControlData.valve[VALVE_BULLET_HORIZONTAL1]=1;
					ViceControlData.valve[VALVE_BULLET_HORIZONTAL2]=0;
					t_statu++;
				}
			}
			
			
			
			if(valve_fdbstate[VALVE_BULLET_CLAMP]==0&&abs(BulletRotate_Data.fdbP-BULLETROTATE_WAITING)<40)	//�ɿ����ҵ�׼��λ�ã����Կ�ʼ��һ��
			{
				TakeBulletState=BULLET_ACQUIRE2;	//��һ��ȡ��
			}
			break;
		}
		case BULLET_ACQUIRE2:	//ǰ�졢�н���̧����	��֮Ϊ��ù���2
		{
			if(valve_fdbstate[VALVE_BULLET_HORIZONTAL1]==1)	//����ƽ��
			{
				BulletRotate_Data.tarP=BULLETROTATE_ACQUIRE;
				if(abs(BulletRotate_Data.fdbP-BULLETROTATE_ACQUIRE)<45)
				{
					ViceControlData.valve[VALVE_BULLET_CLAMP]=1;
					if(valve_fdbstate[VALVE_BULLET_CLAMP]==1)
					{
						TakeBulletState=BULLET_POUROUT2;	//�л�������
					}
				}
			}
			break;
		}
		case BULLET_POUROUT2:	//������б�������ת	��֮Ϊ��������2
		{
			
			BulletRotate_Data.tarP=BULLETROTATE_POUROUT;
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_POUROUT)<50)
			{
				TakeBulletState=BULLET_THROWOUT2;	//�л����ӳ�
			}
			break;
		}
		case BULLET_THROWOUT2:	//������ء�����̧�𡢼н��ɿ�	��֮Ϊ�������2
		{
			///////////////////////////////////////////
			
			if(valve_fdbstate[VALVE_BULLET_CLAMP]==1)	//�н�ʱ���£����º����
			{
				BulletRotate_Data.tarP=BULLETROTATE_THROWOUT;
			}
			
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_THROWOUT)<40)	//�Ż�ԭλ
			{
				ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
			}
			
			if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)	//�Ѿ��ɿ�����ʼ����
			{
				BulletRotate_Data.tarP=BULLETROTATE_WAITING;
				
////////////				if((BulletRotate_Data.fdbP-BULLETROTATE_THROWOUT)>60)	//΢΢̧���ʼƽ��
////////////				{
////////////					ViceControlData.valve[VALVE_BULLET_HORIZONTAL1]=0;
////////////					ViceControlData.valve[VALVE_BULLET_HORIZONTAL2]=1;
////////////					t_statu++;
////////////				}
			}
			
			if(valve_fdbstate[VALVE_BULLET_CLAMP]==0&&abs(BulletRotate_Data.fdbP-BULLETROTATE_WAITING)<40)	//�ɿ����ҵ�׼��λ�ã����Կ�ʼ��һ��
			{
				TakeBulletState=BULLET_WAITING;	//��һ��ȡ��
			}
			
			break;
		}
		case BULLET_OTHER:	//������ȡ��״̬--���񣺻��к����ȡ��
		{
			ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
			static u8 valve_horizontal_state=0;	//��¼��ŷ�λ��
			static u32 valve_getback_time_record=0;
			static u8 valve_getback_statu=0;
				
			if(takebulletstate_last!=BULLET_OTHER)
			{
				valve_getback_time_record=0;	//�л�ʱ���������һ�ε�Ӱ��
				valve_getback_statu=0;
			}
			
			if(valve_fdbstate[VALVE_BULLET_HORIZONTAL1]==0&&valve_fdbstate[VALVE_BULLET_HORIZONTAL2]==1&&valve_getback_statu!=1)	//�ڶ��л���0���ж�����ʱ������Ҫ����һ����ɱ�־λ������ʱ�л�״̬�����Ӱ��
			{
				valve_horizontal_state=1;
				//BulletRotate_Data.tarP=BULLETROTATE_WAITING;	//��Ϊ�߼�����ֻ�ܴ�WAITING����OTHER������tarP��ȻΪWAITING 
			}
			else if(valve_fdbstate[VALVE_BULLET_HORIZONTAL1]==1&&valve_fdbstate[VALVE_BULLET_HORIZONTAL2]==0&&valve_getback_statu!=1)
			{
				valve_horizontal_state=2;
			}
			
			if(valve_horizontal_state==1)	//��0 1����
			{
				if(valve_getback_time_record==0)
				{
					ViceControlData.valve[VALVE_BULLET_HORIZONTAL1]=1;
					ViceControlData.valve[VALVE_BULLET_HORIZONTAL2]=0;
					valve_getback_time_record=time_1ms_count;
				}					

				if(time_1ms_count-valve_getback_time_record>140)	//140��ʱ���м�
				{
					ViceControlData.valve[VALVE_BULLET_HORIZONTAL1]=0;
					ViceControlData.valve[VALVE_BULLET_HORIZONTAL2]=0;
					valve_getback_time_record=0;
					valve_horizontal_state=0;
					valve_getback_statu=1;
				}
			}

			if(valve_horizontal_state==2)	//��1 0����
			{
				if(valve_getback_time_record==0)
				{
					ViceControlData.valve[VALVE_BULLET_HORIZONTAL1]=0;
					ViceControlData.valve[VALVE_BULLET_HORIZONTAL2]=1;
					valve_getback_time_record=time_1ms_count;
				}					

				if(time_1ms_count-valve_getback_time_record>130)	//200��ʱ���м�
				{
					ViceControlData.valve[VALVE_BULLET_HORIZONTAL1]=0;
					ViceControlData.valve[VALVE_BULLET_HORIZONTAL2]=0;
					valve_getback_time_record=0;
					valve_horizontal_state=0;
					valve_getback_statu=1;
				}
			}

			
			
		if(valve_fdbstate[VALVE_BULLET_HORIZONTAL1]==0&&valve_fdbstate[VALVE_BULLET_HORIZONTAL2]==0)
		{
			BulletRotate_Data.tarP=BULLETROTATE_OTHER;
		}

			
			break;
		}
	}
	
	takebulletstate_last=TakeBulletState;
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



