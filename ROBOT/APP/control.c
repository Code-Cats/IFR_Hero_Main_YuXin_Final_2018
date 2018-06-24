#include "control.h"

#define PITCH 0
#define ROLL 1
#define YAW 2

extern u8 IMU_Check_Useless_State;	//������ʧЧ���λ

WorkState_e workState=PREPARE_STATE;

u16 t_up_sm_recorf=0;

LIFT_DATA lift_Data={0};


PID_GENERAL PID_Lift_Position[4]={PID_LIFT_POSITION_DEFAULT,PID_LIFT_POSITION_DEFAULT,PID_LIFT_POSITION_DEFAULT,PID_LIFT_POSITION_DEFAULT};
PID_GENERAL PID_Lift_Speed[4]={PID_LIFT_SPEED_DEFAULT,PID_LIFT_SPEED_DEFAULT,PID_LIFT_SPEED_DEFAULT,PID_LIFT_SPEED_DEFAULT};


extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
extern RC_Ctl_t RC_Ctl;
extern LIFT_POSITION_ENCODER lift_position_encoder[4];
extern GYRO_DATA Gyro_Data;
extern YUN_MOTOR_DATA 	yunMotorData;
extern CHASSIS_DATA chassis_Data;
extern ViceControlDataTypeDef ViceControlData;

extern SHOOT_DATA shoot_Data_Down;
extern SHOOT_MOTOR_DATA shoot_Motor_Data_Down;

extern SHOOT_DATA shoot_Data_Up;
extern SHOOT_MOTOR_DATA shoot_Motor_Data_Up;


extern s16 Chassis_Vx;
extern s16 Chassis_Vy;
extern s16 Chassis_Vw;
extern u8 cali_state_Entirety_PID;
extern u8 Replenish_Bullet_Statu;

extern u8 Judge_Send_Statu;	//ˢ�±�־λ

s16 lift_tem=0;
s16 LIFT_tarP=0;

u8 t_lift_time_start=0;
///////////////////////////////////////remoteData
s16 Vw_tem=0;
///////////////////////////////////////

u32 time_1ms_count=0;

float t_yaw_error=0;	//��ʱ����
float t_pitch_error=0;	//��ʱ����
void Control_Task(void)	//2ms
{
	time_1ms_count++;

if(time_1ms_count%100==0)
{
	Judge_Send_Statu=1; 
}


	BulletNum_Calculate();

	Lift_Time_Gauge(&t_lift_time_start);
	
	Check_Task();

	IMU_Check_Useless();	//�����Ǽ��ʧЧ

	KeyboardRetset();	//��ս�������������ʱ�����и�λ����		CTRL SHIFT Z X����	C VΪ0
	#ifdef USART6_WIFIDEBUG
	if(time_1ms_count%50==0)
	{
//		Debug_Send_OSC();
	}
	#endif
//	Vw_tem=Chassis_Attitude_Correct(Chassis_GYRO[2],Gyro_Data.angvel[2]+2);
//  Chassis_Vw+=Vw_tem;
	
	if(time_1ms_count%1==0)	//1000hz
	{
		for(int keyid=0;keyid<KEY_NUMS;keyid++)	//���ڶ�ʱ����
		{
			ButtonStatu_Verdict(&KeyBoardData[keyid]);
		}
	}
	
	
	Work_State_Change();
	
	Work_State_Change_BackProtect();
	
	Image_Cut_Task();	//����ͷ�л������
	
	switch (GetWorkState())	//2018.3.15	ִ�������
	{
		case CHECK_STATE:	//�Լ�ģʽ
		{	//���������ʼ���������Լ�ģʽ //��ʱ����ոտ�������ȴ�һ��ʱ��ȫ���Լ�δ��⵽�쳣��2-3���Լ촥���������ϣ�������Ϊʱ��������Ϊ��ʱ�������㣬���������ʱ����¼
			if(time_1ms_count>300)	//����LOST״̬�ص�CHECKģʽ����ִ�м����������
			{	//����ִ�е�����˵��LOSTCHECKͨ����������ֵ���
				RC_Calibration();	//self check
				if(1)	//selfcheck��־
				{
					SetWorkState(PREPARE_STATE);	//�˲���ζ�Լ�ͨ����һ��Ӳ��ģ������
					//���ݳ�ʼ����
					yunMotorData.pitch_tarP=PITCH_INIT;	//-50����Ϊ������ˮƽʱ��̨����	//��������������̨����
					yunMotorData.yaw_tarP=(Gyro_Data.angle[2]*10+(YAW_INIT-yunMotorData.yaw_fdbP)*3600/8192);	//�����Ŵ�10������Ŀ��λ����Ϊ�е�
				}
			}
			break;
		}
		case PREPARE_STATE:	//Ԥ��ģʽ
		{	//�ȴ�����״̬�ȶ��������ó�ֵ
			Yun_Task();	//��������
//			if(abs(Gyro_Data.angvel[0])<20&&abs(Gyro_Data.angvel[2])<20&&abs(yunMotorData.pitch_tarP-(Gyro_Data.angle[0]*8192/360.0f+PITCH_INIT))<50)	//��̨�Ѿ�λ	//λ�û������
			if(abs(Gyro_Data.angvel[0])<20)	//��̨�Ѿ�λ�����ٶȻ����
			{
				SetWorkState(CALI_STATE);
			}
			Shoot_Task();	//��ʱ����
			break;
		}
		case CALI_STATE:	//�궨ģʽ
		{
			if(Lift_Cali()==1)
			{
				SetWorkState(NORMAL_STATE);
			}
			Yun_Task();	//������̨����
			Lift_Task();	//��������
			Shoot_Task();	//��ʱ����
			TakeBullet_Control_Center();	//���м�����ʱ�����Ͷ��ִ�У��ʼ���
			break;
		}
		case NORMAL_STATE:	//��������ģʽ
		{
			Replenish_Bullet_Task(KeyBoardData[KEY_R].value);
			Teleconltroller_Data_protect();	//ң�������ݱ���
			Yun_Task();	//������̨����
			Remote_Task();	//ִ���ƶ�
			AutoChassisAttitude_Lift_V2(Chassis_GYRO[PITCH]);	//���ڲ���ģʽ���ƺͼ�λ����
			TakeBullet_Control_Center();	//ȡ����������
			Lift_Task();	//��������
			Shoot_Task();
			
			break;
		}
		case WAIST_STATE:
		{
			Teleconltroller_Data_protect();	//ң�������ݱ���
			Yun_Task();	//������̨����
			Remote_Task();	//ִ���ƶ�
//			AutoChassisAttitude_Lift_V2(Chassis_GYRO[PITCH]);
			Lift_Task();	//��������
			Shoot_Task();
			TakeBullet_Control_Center();	//���м�����ʱ�����Ͷ��ִ�У��ʼ���
			break;
		}
		case ASCEND_STATE:	//�Զ��ϵ�ģʽ
		{
			Ascend_Control_Center();
			Yun_Task();	//������̨����
			Remote_Task();	//ִ���ƶ�
			Lift_Task();	//��������
			TakeBullet_Control_Center();	//���м�����ʱ�����Ͷ��ִ�У��ʼ���
			break;
		}
		case DESCEND_STATE:	//�Զ��µ�ģʽ
		{
			ViceControlData.image_cut[0]=1;	//�ߵ�ƽ
			Descend_Control_Center();
			Yun_Task();	//������̨����
			Remote_Task();	//ִ���ƶ�
			Lift_Task();	//��������
			TakeBullet_Control_Center();	//���м�����ʱ�����Ͷ��ִ�У��ʼ���
			break;
		}
		case TAKEBULLET_STATE:
		{
			Teleconltroller_Data_protect();	//ң�������ݱ���
			TakeBullet_Control_Center();	//ȡ����������
			Yun_Task();	//������̨����
			Remote_Task();	//ִ���ƶ�
			Lift_Task();	//��������
			Shoot_Task();
			break;
		}
		case ERROR_STATE:	//����ģʽ
		{
			
			break;
		}
		case LOST_STATE:	//����ģʽ
		{
			break;
		}
		case STOP_STATE:	//ֹͣ״̬
		{
//////			Vision_Task(&yunMotorData.yaw_tarP,&yunMotorData.pitch_tarP);
//////			Yun_Task();	//��������
		//	Vision_Task(&t_yaw_error,&t_pitch_error);
			TakeBullet_Control_Center();	//���м�����ʱ�����Ͷ��ִ�У��ʼ���
			break;
		}
		case PROTECT_STATE:	//���ұ���ģʽ
		{
			break;
		}
	}
	
	LED_Indicate();
	
	Chassis_Attitude_Angle_Convert();
	
	Motor_Send();
	if(time_1ms_count%2==0)
	{
		ViceBoard_SendDataRefresh();
		ViceBoard_SendDataRun();
	}
	
}



extern AscendState_e AscendState;
extern DescendState_e DescendState;
/*************************************
RC��PC�Ի���״̬���л�
*************************************/
void Work_State_Change(void)
{
	static u8 Switch_Right_Last=0;
	static WorkState_e State_Record=CHECK_STATE;	
	State_Record=GetWorkState();
	
	
	switch (GetWorkState())	//2018.3.15
	{
		case CHECK_STATE:	//�Լ�ģʽ
		{	//���������ʼ���������Լ�ģʽ 
			
			break;
		}
		case PREPARE_STATE:	//Ԥ��ģʽ
		{	
			
			break;
		}
		case CALI_STATE:	//�궨ģʽ
		{
			
			break;
		}
		case NORMAL_STATE:	//��������ģʽ
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_MIDDLE)	//��������
			{
				SetWorkState(STOP_STATE);
			}
			
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE&&Switch_Right_Last==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_UP)
			{
				SetWorkState(ASCEND_STATE);
			}
			else if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE&&Switch_Right_Last==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)
			{
				SetWorkState(DESCEND_STATE);
//				SetWorkState(TAKEBULLET_STATE);	//������ģʽ//��ʱ���ԣ�ȡ��״̬
			}
			
			if(RC_Ctl.rc.switch_left==RC_SWITCH_DOWN)	//����ȡ��
			{
				SetWorkState(TAKEBULLET_STATE);	//������ģʽ//��ʱ���ԣ�ȡ��״̬
			}
			
			if(RC_Ctl.mouse.press_r==1&&RC_Ctl.rc.switch_left!=RC_SWITCH_MIDDLE)	//�м��ܲ���Ť����
			{
				SetWorkState(WAIST_STATE);
			}
			break;
		}
		case WAIST_STATE:
		{
			if(RC_Ctl.mouse.press_r!=1)
			{
				SetWorkState(NORMAL_STATE);
			}
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_MIDDLE)	//����
			{
				SetWorkState(STOP_STATE);
			}
			break;
		}
		case ASCEND_STATE:	//�Զ��ϵ�ģʽ
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_MIDDLE)	//����
			{
				AscendState=FULLRISE_GO1;	//���÷�ֹ��һ���쳣
				SetWorkState(STOP_STATE);
			}
			break;
		}
		case DESCEND_STATE:	//�Զ��µ�ģʽ
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_MIDDLE)	
			{
				DescendState=FULLFALL_DOWN1;	//���÷�ֹ��һ���쳣
				SetWorkState(STOP_STATE);
			}
			break;
		}
		case TAKEBULLET_STATE:	//ȡ��ģʽ
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_MIDDLE)	//����
			{
				SetWorkState(STOP_STATE);
			}
			
			if(RC_Ctl.rc.switch_left==RC_SWITCH_UP)	
			{
				SetWorkState(NORMAL_STATE);
			}
			break;
		}
		case ERROR_STATE:	//����ģʽ
		{
			if(RC_Ctl.key.v_h!=0||RC_Ctl.key.v_l!=0||abs(RC_Ctl.mouse.x)>3)	//�˳���ģʽ
			{
				SetWorkState(NORMAL_STATE);
			}
			break;
		}
		case LOST_STATE:	//����ģʽ
		{
			SetWorkState(CHECK_STATE);
			time_1ms_count=0;	//�����ʼ״̬�����Լ�
			break;
		}
		case STOP_STATE:	//ֹͣ״̬
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_UP)	
			{
				SetWorkState(NORMAL_STATE);
			}
			else if(RC_Ctl.rc.switch_left==RC_SWITCH_DOWN)
			{
				SetWorkState(TAKEBULLET_STATE);
			}
			
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE&&Switch_Right_Last==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_UP)
			{
				SetWorkState(ASCEND_STATE);
			}
			else if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE&&Switch_Right_Last==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)
			{
				SetWorkState(DESCEND_STATE);
//				SetWorkState(TAKEBULLET_STATE);	//������ģʽ//��ʱ���ԣ�ȡ��״̬
			}
			break;
		}
		case PROTECT_STATE:	//���ұ���ģʽ
		{
			static u32 time_count=0;
			time_count++;
			if(Error_Check.statu[LOST_DBUS]==0&&abs(RC_Ctl.rc.ch0+RC_Ctl.rc.ch1+RC_Ctl.rc.ch2+RC_Ctl.rc.ch3-1024*4)>8)
			{
				yunMotorData.yaw_tarP=(Gyro_Data.angle[2]*10+(YAW_INIT-yunMotorData.yaw_fdbP)*3600/8192);	//������̨Ŀ��λ��
				SetWorkState(NORMAL_STATE);
				time_count=0;
			}
			
			if(Error_Check.statu[LOST_DBUS]==0&&time_count>8000)	//�з�����Ϊ�޷��ָ�
			{
				NVIC_SystemReset();
			}
			break;
		}
	}
	Switch_Right_Last=RC_Ctl.rc.switch_right;
}


extern u8 descend_valve_prepare_state;	//�Զ��µ���ŷ���λ����
extern  u32 descend_valve_prepare_state_count;

extern u8 auto_takebullet_statu;
extern u8 SetCheck_TakeBullet_TakeBack_statu;	//�г�ȡ������ִ�б�־λ	//����ǰ��extern
void Work_State_Change_BackProtect(void)	//����ĳһ״̬�˳�ʱ��ȷ����״̬��һ���������ƶ���λ
{
	static WorkState_e State_Record=CHECK_STATE;
 
	if(State_Record!=DESCEND_STATE&&GetWorkState()==DESCEND_STATE)
	{
		descend_valve_prepare_state=0;
		descend_valve_prepare_state_count=0;
	}
	
	if(State_Record==TAKEBULLET_STATE&&GetWorkState()!=TAKEBULLET_STATE)	//�˳�ȡ��ģʽ
	{
		SetCheck_TakeBullet_TakeBack_statu=1;	//ˢ�´�
//		auto_takebullet_statu=0;	//����//��Ϊ5.15�����߼����ڵȴ�ȡ����ɣ���һ��Ϊ��⣬���Բ��ܶ�����в���
	}
	
	if(State_Record!=TAKEBULLET_STATE&&GetWorkState()==TAKEBULLET_STATE)
	{
		SetCheck_GripLift(1);	//������ȡ���߶�
		auto_takebullet_statu=0;	//�������Ӱ��Ӱ��
	}
	SetCheck_TakeBullet_TakeBack();	//ִ�д�
	State_Record=GetWorkState();
}



extern s16 t_error_record;
void LED_Indicate(void)
{
	if(time_1ms_count%BLINK_CYCLE==0)
	{
		switch (GetWorkState())	//2018.3.15
		{
			case CHECK_STATE:	//�Լ�ģʽ
			{	//���������ʼ���������Լ�ģʽ 
				LED_Blink_Set(10,10);
				break;
			}
			case PREPARE_STATE:	//Ԥ��ģʽ
			{	
				LED_Blink_Set(9,9);
				break;
			}
			case CALI_STATE:	//�궨ģʽ	�쿪����
			{
				LED_Blink_Set(9,9);
				break;
			}
			case NORMAL_STATE:	//��������ģʽ	�������
			{
				LED_Blink_Set(1,0);
				break;
			}
			case WAIST_STATE:
			{
				LED_Blink_Set(1,0);
				break;
			}
			case ASCEND_STATE:	//�Զ��ϵ�ģʽ	����
			{
				LED_Blink_Set(1,0);
				break;
			}
			case DESCEND_STATE:	//�Զ��µ�ģʽ	����
			{
				LED_Blink_Set(1,0);
				break;
			}
			case TAKEBULLET_STATE:
			{
				LED_Blink_Set(1,0);
				break;
			}
			case ERROR_STATE:	//����ģʽ
			{
				if(t_error_record==LOST_SM_DOWN)	//�²���
				{
					LED_Blink_Set(3,10);
				}
				else if(t_error_record==LOST_SM_UP)	//�ϲ���
				{
					LED_Blink_Set(4,10);
				}
				else if(t_error_record==LOST_CM1||t_error_record==LOST_CM2||t_error_record==LOST_CM3||t_error_record==LOST_CM4)	//���̵��
				{
					LED_Blink_Set(2,10);
				}
				else if(t_error_record==LOST_LIFT1||t_error_record==LOST_LIFT2||t_error_record==LOST_LIFT3||t_error_record==LOST_LIFT4)	//����
				{
					LED_Blink_Set(1,10);
				}
				else if(t_error_record==LOST_PITCH||t_error_record==LOST_YAW)	//��̨
				{
					LED_Blink_Set(5,10);
				}
				else
				{
					LED_Blink_Set(6,10);
				}
				break;
			}
			case LOST_STATE:	//����ģʽ
			{
				LED_Blink_Set(1,1);
				break;
			}
			case STOP_STATE:	//ֹͣ״̬	����
			{
				if(time_1ms_count%BLINK_INTERVAL==0)
				{
					LED_Blink_Set(0,10);
				}
				else if((time_1ms_count+BLINK_INTERVAL/2)%BLINK_INTERVAL==0)
				{
					LED_Blink_Set(10,0);
				}
				
				break;
			}
			case PROTECT_STATE:	//���ұ���ģʽ	˫��
			{
				LED_Blink_Set(1,1);
				break;
			}
		}
LED_Blink_Run();
	}
}

u8 lift_control_all_state=0;
s16 lift_k_tem=0;
#define LIFT_K 0.8
void Lift_Task(void)
{
	///////////////////////////////////////////////////////////////////
//	lift_Data.lf_lift_tarP=LIFT_tarP;
//	lift_Data.rf_lift_tarP=LIFT_tarP;
//	lift_Data.lb_lift_tarP=LIFT_tarP;
//	lift_Data.rb_lift_tarP=LIFT_tarP;
	if(GetWorkState()==NORMAL_STATE)
	{
		
		if(RC_Ctl.rc.switch_left==RC_SWITCH_UP)	//����
		{
			if(time_1ms_count%10==0)
			{
				switch (RC_Ctl.rc.switch_right)
				{
					case RC_SWITCH_UP:
					{
						lift_Data.lf_lift_tarP+=(s16)(12*(RC_Ctl.rc.ch3-1024)/660.0);
						lift_Data.rf_lift_tarP+=(s16)(12*(RC_Ctl.rc.ch3-1024)/660.0);
						lift_Data.lb_lift_tarP+=(s16)(12*(RC_Ctl.rc.ch3-1024)/660.0);
						lift_Data.rb_lift_tarP+=(s16)(12*(RC_Ctl.rc.ch3-1024)/660.0);
						break;
					}
					case RC_SWITCH_DOWN:
					{
						lift_Data.lf_lift_tarP+=(s16)(7*(RC_Ctl.rc.ch3-1024)/660.0);
						lift_Data.rf_lift_tarP+=(s16)(7*(RC_Ctl.rc.ch3-1024)/660.0);
							
						lift_Data.lb_lift_tarP-=(s16)(7*(RC_Ctl.rc.ch3-1024)/660.0);
						lift_Data.rb_lift_tarP-=(s16)(7*(RC_Ctl.rc.ch3-1024)/660.0);
						break;
					}
					case RC_SWITCH_MIDDLE:
					{
						
					}
				}
			}
//////////			SetCheck_FrontLift(lift_control_all_state);//�����������������������������Ǹ�ʲô��
//////////			SetCheck_BackLift(lift_control_all_state);
		}
	}
	
	
	if(GetWorkState()!=CALI_STATE)	//�궨״̬�²������г�
	{
		lift_Data.lf_lift_tarP=lift_Data.lf_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.lf_lift_tarP;	//�����г�
		lift_Data.lf_lift_tarP=lift_Data.lf_lift_tarP>LIFT_DISTANCE_LIMIT?LIFT_DISTANCE_LIMIT:lift_Data.lf_lift_tarP;
		
		lift_Data.rf_lift_tarP=lift_Data.rf_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.rf_lift_tarP;	//�����г�
		lift_Data.rf_lift_tarP=lift_Data.rf_lift_tarP>LIFT_DISTANCE_LIMIT?LIFT_DISTANCE_LIMIT:lift_Data.rf_lift_tarP;
		
		lift_Data.lb_lift_tarP=lift_Data.lb_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.lb_lift_tarP;	//�����г�
		lift_Data.lb_lift_tarP=lift_Data.lb_lift_tarP>LIFT_DISTANCE_LIMIT?LIFT_DISTANCE_LIMIT:lift_Data.lb_lift_tarP;
		
		lift_Data.rb_lift_tarP=lift_Data.rb_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.rb_lift_tarP;	//�����г�
		lift_Data.rb_lift_tarP=lift_Data.rb_lift_tarP>LIFT_DISTANCE_LIMIT?LIFT_DISTANCE_LIMIT:lift_Data.rb_lift_tarP;
	}
	
	lift_Data.lf_lift_tarV=(int32_t)PID_General(lift_Data.lf_lift_tarP,lift_Data.lf_lift_fdbP,&PID_Lift_Position[LF]);	//λ�û�PID����
	lift_Data.rf_lift_tarV=(int32_t)PID_General(lift_Data.rf_lift_tarP,lift_Data.rf_lift_fdbP,&PID_Lift_Position[RF]);
	lift_Data.lb_lift_tarV=(int32_t)PID_General((lift_Data.lb_lift_tarP),lift_Data.lb_lift_fdbP,&PID_Lift_Position[LB]);
	lift_Data.rb_lift_tarV=(int32_t)PID_General((lift_Data.rb_lift_tarP),lift_Data.rb_lift_fdbP,&PID_Lift_Position[RB]);
	
	lift_Data.lf_lift_output=PID_General(lift_Data.lf_lift_tarV,lift_Data.lf_lift_fdbV,&PID_Lift_Speed[LF]);	//�ٶȻ�PID����
	lift_Data.rf_lift_output=PID_General(lift_Data.rf_lift_tarV,lift_Data.rf_lift_fdbV,&PID_Lift_Speed[RF]);
	lift_Data.lb_lift_output=PID_General(lift_Data.lb_lift_tarV,lift_Data.lb_lift_fdbV,&PID_Lift_Speed[LB]);
	lift_Data.rb_lift_output=PID_General(lift_Data.rb_lift_tarV,lift_Data.rb_lift_fdbV,&PID_Lift_Speed[RB]);
	
//	CAN_Lift_SendMsg(lift_Data.lf_lift_output,lift_Data.rf_lift_output,lift_Data.lb_lift_output,lift_Data.rb_lift_output);
	
}


/*****************************************�������궨����**********************************************/
LiftCaliState_e liftcaliState=UP_STATE;
u8 Lift_Cali(void)
{
	switch (liftcaliState)
	{
		case UP_STATE:
		{
			PID_Lift_Speed[LF].k_i=2*LIFT_SPEED_PID_I;
			PID_Lift_Speed[RF].k_i=2*LIFT_SPEED_PID_I;
			PID_Lift_Speed[LB].k_i=2*LIFT_SPEED_PID_I;
			PID_Lift_Speed[RB].k_i=2*LIFT_SPEED_PID_I;
			
			lift_Data.lf_lift_tarP=70;
			lift_Data.rf_lift_tarP=70;
			lift_Data.lb_lift_tarP=70;
			lift_Data.rb_lift_tarP=70;

			if(lift_Data.lf_lift_fdbP>30&&lift_Data.rf_lift_fdbP>30&&lift_Data.lb_lift_fdbP>30&&lift_Data.rb_lift_fdbP>30)
			{
				liftcaliState=WAIT_STATE;
				cali_state_Entirety_PID=1;
			}
			break;
		}
		case WAIT_STATE:
		{
			PID_Lift_Speed[LF].k_i=LIFT_SPEED_PID_I;
			PID_Lift_Speed[RF].k_i=0;
			PID_Lift_Speed[LB].k_i=0;
			PID_Lift_Speed[RB].k_i=0;
				
			lift_Data.lf_lift_tarP=-10000;
			lift_Data.rf_lift_tarP=-10000;
			lift_Data.lb_lift_tarP=-10000;
			lift_Data.rb_lift_tarP=-10000;
			if((lift_Data.lf_lift_fdbV+lift_Data.rf_lift_fdbV+lift_Data.lb_lift_fdbV+lift_Data.rb_lift_fdbV)<-25)	//���ѹ��ٶ����
			{
				liftcaliState=DOWN_STATE;
			}
			break;
		}
		case DOWN_STATE:
		{
			if(abs(lift_Data.lf_lift_fdbV+lift_Data.rf_lift_fdbV+lift_Data.lb_lift_fdbV+lift_Data.rb_lift_fdbV)<10)	//��һ����һ��λ��ֵ֮��//��Ϊ��
			{
				lift_position_encoder[LF].turns=0;
				lift_position_encoder[RF].turns=0;
				lift_position_encoder[LB].turns=0;
				lift_position_encoder[RB].turns=0;
				
				lift_Data.lf_lift_tarP=0;
				lift_Data.rf_lift_tarP=0;
				lift_Data.lb_lift_tarP=0;
				lift_Data.rb_lift_tarP=0;
				
				PID_Lift_Speed[LF].k_i=LIFT_SPEED_PID_I;
				PID_Lift_Speed[RF].k_i=LIFT_SPEED_PID_I;
				PID_Lift_Speed[LB].k_i=LIFT_SPEED_PID_I;
				PID_Lift_Speed[RB].k_i=LIFT_SPEED_PID_I;
				
				cali_state_Entirety_PID=0;	//����
				return 1;
//				SetWorkState(NORMAL_STATE);	//3.16��ʱ��
			}
			break;
		}
	}
	return 0;
}



u8 cali_state_Entirety_PID=0;	//����PID�ڱ궨�����еĴ���ʱ����־λ
u8 Calibration_state=0;
s16 t_fdbV_sum=0;
u16 t_cali_count=0;
/*****************************************�ڶ����궨����**********************************************/
void Lift_Calibration(void)	//��������ϵ�궨
{
//	u32 Record_Last=0;	//�궨��������һ�νǶȼ�¼ֵ
	u8 Calibration_Time_Count=0;	//��ʱ
	
	
	SetWorkState(CALI_STATE);
	PID_Lift_Speed[LF].k_i=2*LIFT_SPEED_PID_I;
	PID_Lift_Speed[RF].k_i=2*LIFT_SPEED_PID_I;
	PID_Lift_Speed[LB].k_i=2*LIFT_SPEED_PID_I;
	PID_Lift_Speed[RB].k_i=2*LIFT_SPEED_PID_I;
	
	lift_Data.lf_lift_tarP=100;
	lift_Data.rf_lift_tarP=100;
	lift_Data.lb_lift_tarP=100;
	lift_Data.rb_lift_tarP=100;
	while(lift_Data.lf_lift_fdbP<40||lift_Data.rf_lift_fdbP<40||lift_Data.lb_lift_fdbP<40||lift_Data.rb_lift_fdbP<40)	//�ȴ�������һ���߶�
	{;}

	
	PID_Lift_Speed[LF].k_i=0;
	PID_Lift_Speed[RF].k_i=0;
	PID_Lift_Speed[LB].k_i=0;
	PID_Lift_Speed[RB].k_i=LIFT_SPEED_PID_I;
		
	lift_Data.lf_lift_tarP=-10000;
	lift_Data.rf_lift_tarP=-10000;
	lift_Data.lb_lift_tarP=-10000;
	lift_Data.rb_lift_tarP=-10000;
		
	cali_state_Entirety_PID=1;
	
	Calibration_Time_Count=time_1ms_count;	//	��¼��ǰϵͳʱ��
		
	while(Calibration_state!=1||lift_position_encoder[LF].turns!=0||lift_position_encoder[RF].turns!=0||lift_position_encoder[LB].turns!=0||lift_position_encoder[RB].turns!=0)
	{
		if((time_1ms_count-Calibration_Time_Count)>1000)	//����ʱ�����������������ԭ��
		{
			
			if(time_1ms_count%500==0)
			{
				t_cali_count++;
				t_fdbV_sum=abs(lift_Data.lf_lift_fdbV+lift_Data.rf_lift_fdbV+lift_Data.lb_lift_fdbV+lift_Data.rb_lift_fdbV);
				if(abs(lift_Data.lf_lift_fdbV+lift_Data.rf_lift_fdbV+lift_Data.lb_lift_fdbV+lift_Data.rb_lift_fdbV)<15)	//��һ����һ��λ��ֵ֮��//��Ϊ��
				{
					
					Calibration_state=1;
					lift_position_encoder[LF].turns=0;
					lift_position_encoder[RF].turns=0;
					lift_position_encoder[LB].turns=0;
					lift_position_encoder[RB].turns=0;
					
					lift_Data.lf_lift_tarP=0;
					lift_Data.rf_lift_tarP=0;
					lift_Data.lb_lift_tarP=0;
					lift_Data.rb_lift_tarP=0;
					
					PID_Lift_Speed[LF].k_i=LIFT_SPEED_PID_I;
					PID_Lift_Speed[RF].k_i=LIFT_SPEED_PID_I;
					PID_Lift_Speed[LB].k_i=LIFT_SPEED_PID_I;
					PID_Lift_Speed[RB].k_i=LIFT_SPEED_PID_I;
					
					cali_state_Entirety_PID=0;	//����
					SetWorkState(NORMAL_STATE);	//3.16��ʱ��
				}
				//Record_Last= lift_position_encoder[LF].calc+lift_position_encoder[RF].calc+lift_position_encoder[LB].calc+lift_position_encoder[RB].calc;
			}

		}
		Calibration_Time_Count=0;
	}
}


s32 t_yaw_send=0;
s32 t_pitch_send=0;
s16 t_yaw_16t=0;
s16 t_pitch_16t=0;

float cali_send[4]={0};
//extern YUN_MOTOR_DATA 			yunMotorData;

u32 t_send_count=0;
void Motor_Send(void)
{
	switch (GetWorkState())	//2018.3.15
	{	
		case CHECK_STATE:	//�Լ�ģʽ
		{	//���������ʼ���������Լ�ģʽ //��ʱ����ոտ�������ȴ�һ��ʱ��ȫ���Լ�δ��⵽�쳣��2-3���Լ촥���������ϣ�������Ϊʱ��������Ϊ��ʱ�������㣬���������ʱ����¼
			CAN1_Yun_SendMsg(0,0);	//CAN2	//yaw,pitch
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_Shoot_SendMsg(0,0);//�²������ϲ���
//			CAN2_Shoot_SendMsg((s16)shoot_Motor_Data_Down.output,0);	//�²������ϲ���
			break;
		}
		case PREPARE_STATE:	//Ԥ��ģʽ
		{	//�ȴ�����״̬�ȶ��������ó�ֵ
			CAN1_Yun_SendMsg(yunMotorData.yaw_output,yunMotorData.pitch_output);	//CAN2-1000	//ȡ����������
	//		CAN1_Yun_SendMsg(t_yaw_16t,t_pitch_16t);	//������ģʽ
//			CAN_Motor6623_calibration();
			//CAN1_Yun_SendMsg(yunMotorData.yaw_output+Yaw_output_offset(yunMotorData.yaw_fdbP),yunMotorData.pitch_output+Pitch_output_offset(yunMotorData.pitch_tarP));	//CAN2-1000
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			//CAN2_Shoot_SendMsg(0,0);//�²������ϲ���
			CAN2_Shoot_SendMsg((s16)shoot_Motor_Data_Down.output,0);	//�²������ϲ���
			break;
		}
		case CALI_STATE:	//�궨ģʽ
		{
			t_send_count++;
			SetFrictionWheelSpeed(FRICTION_INIT);
			
			Cali_Output_Limit(lift_Data.lf_lift_output,&cali_send[LF]);
			Cali_Output_Limit(lift_Data.rf_lift_output,&cali_send[RF]);
			Cali_Output_Limit(lift_Data.lb_lift_output,&cali_send[LB]);
			Cali_Output_Limit(lift_Data.rb_lift_output,&cali_send[RB]);
//		Entirety_PID(&lift_Data,cali_send);  	//����PID����
			if(IMU_Check_Useless_State==0)
			{
				Lift_Cali_GYRO_Compensate(cali_send);	//�����ǲ���.��������3.14
			}
//			CAN1_Yun_SendMsg(yunMotorData.yaw_output+Yaw_output_offset(yunMotorData.yaw_fdbP),yunMotorData.pitch_output+Pitch_output_offset(yunMotorData.pitch_tarP));	//CAN2-1000	//���뷴������
			CAN1_Yun_SendMsg(yunMotorData.yaw_output,yunMotorData.pitch_output);	//CAN2-1000	//ȡ����������
//			CAN1_Yun_SendMsg(0,0);
			CAN2_Chassis_SendMsg(0,0,0,0);
//		CAN1_Lift_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg((s16)cali_send[LF],(s16)cali_send[RF],(s16)cali_send[LB],(s16)cali_send[RB]);
//			CAN2_Shoot_SendMsg(0,0);//�²������ϲ���

			CAN2_Shoot_SendMsg((s16)shoot_Motor_Data_Down.output,0);	//�²������ϲ���
			break;
		}
		case NORMAL_STATE:	//��������ģʽ
		{
			CAN1_Yun_SendMsg((s16)yunMotorData.yaw_output,(s16)yunMotorData.pitch_output);	//CAN2-1000	//ȡ����������	��������ģʽ
//		CAN_Yun_SendMsg(0,0);
//			CAN1_Yun_SendMsg(t_yaw_16t,t_pitch_16t);	//������ģʽ
			
//		CAN_Chassis_SendMsg((s16)remote_tem,(s16)remote_tem,(s16)remote_tem,(s16)remote_tem);
//			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
			CAN2_Chassis_SendMsg(0,0,0,0);
			
//    CAN1_Lift_SendMsg((s16)lift_tem,(s16)lift_tem,(s16)lift_tem,(s16)lift_tem);
			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			CAN2_Shoot_SendMsg((s16)shoot_Motor_Data_Down.output,(s16)shoot_Motor_Data_Up.output);	//�²������ϲ���
			break;
		}
		case WAIST_STATE:
		{
			CAN1_Yun_SendMsg(yunMotorData.yaw_output,yunMotorData.pitch_output);	//CAN2-1000	//ȡ����������
//		CAN_Yun_SendMsg(0,0);
//		CAN_Chassis_SendMsg((s16)remote_tem,(s16)remote_tem,(s16)remote_tem,(s16)remote_tem);
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
//			CAN_Chassis_SendMsg(0,0,0,0);
//    CAN_Lift_SendMsg((s16)lift_tem,(s16)lift_tem,(s16)lift_tem,(s16)lift_tem);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_Shoot_SendMsg((s16)shoot_Motor_Data_Down.output,0);	//�²������ϲ���
			break;
		}
		case ERROR_STATE:	//����ģʽ
		{
			CAN1_Yun_SendMsg(0,0);	//CAN2	//yaw,pitch
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_Shoot_SendMsg(0,0);//�²������ϲ���
			break;
		}
		case STOP_STATE:	//ֹͣ״̬
		{
			CAN1_Yun_SendMsg(0,0);	//CAN2	//yaw,pitch
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_Shoot_SendMsg(0,0);//�²������ϲ���
			break;
		}
		case PROTECT_STATE:	//���ұ���ģʽ
		{
			CAN1_Yun_SendMsg(0,0);	//CAN2	//yaw,pitch
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_Shoot_SendMsg(0,0);//�²������ϲ���
			break;
		}
		case ASCEND_STATE:	//�Զ��ϵ�ģʽ
		{
			CAN1_Yun_SendMsg(yunMotorData.yaw_output+Yaw_output_offset(yunMotorData.yaw_fdbP),yunMotorData.pitch_output+Pitch_output_offset(yunMotorData.pitch_tarP));	//CAN2-1000
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			CAN2_Shoot_SendMsg(0,0);//�²������ϲ���
			break;
		}
		case DESCEND_STATE:	//�Զ��µ�ģʽ
		{
			CAN1_Yun_SendMsg(yunMotorData.yaw_output+Yaw_output_offset(yunMotorData.yaw_fdbP),yunMotorData.pitch_output+Pitch_output_offset(yunMotorData.pitch_tarP));	//CAN2-1000
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			CAN2_Shoot_SendMsg(0,0);//�²������ϲ���
			break;
		}
		case TAKEBULLET_STATE:
		{
			CAN1_Yun_SendMsg(yunMotorData.yaw_output,yunMotorData.pitch_output);	//CAN2-1000	//ȡ����������
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			CAN2_Shoot_SendMsg((s16)shoot_Motor_Data_Down.output,(s16)shoot_Motor_Data_Up.output);//�²������ϲ���
			break;
		}
		default:
		{
			CAN1_Yun_SendMsg(0,0);	//CAN2	//yaw,pitch
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_Shoot_SendMsg(0,0);//�²������ϲ���
			break;
		}
	}
	
/*	
	switch (GetWorkState())
	{
		case PREPARE_STATE:	//Ԥ��ģʽ
		{
			CAN_Yun_SendMsg(0,0);	//CAN2	//yaw,pitch
			CAN_Chassis_SendMsg(0,0,0,0);
			CAN_Lift_SendMsg(0,0,0,0);
			break;
		}
		case NORMAL_STATE:
		{
			t_pitch_offset=Pitch_output_offset(yunMotorData.pitch_tarP);
			t_pitch_outsend=t_pitch_offset+yunMotorData.pitch_output;

		//	t_pitch_outsend=-1000+yunMotorData.pitch_output;
			CAN_Yun_SendMsg(yunMotorData.yaw_output+Yaw_output_offset(yunMotorData.yaw_fdbP),yunMotorData.pitch_output+Pitch_output_offset(yunMotorData.pitch_tarP));	//CAN2-1000
//	CAN_Yun_SendMsg(0,0);
	//		CAN_Chassis_SendMsg((s16)remote_tem,(s16)remote_tem,(s16)remote_tem,(s16)remote_tem);
			CAN_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
	//    CAN_Lift_SendMsg((s16)lift_tem,(s16)lift_tem,(s16)lift_tem,(s16)lift_tem);
			CAN_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			break;
		}
		case STOP_STATE:	//��������״̬
		{
			CAN_Yun_SendMsg(0,0);	//CAN2
			CAN_Chassis_SendMsg(0,0,0,0);
			CAN_Lift_SendMsg(0,0,0,0);
			break;
		}
		case CALI_STATE:
		{
			float cali_send[4]={0};
			
			Cali_Output_Limit(lift_Data.lf_lift_output,&cali_send[LF]);
			Cali_Output_Limit(lift_Data.rf_lift_output,&cali_send[RF]);
			Cali_Output_Limit(lift_Data.lb_lift_output,&cali_send[LB]);
			Cali_Output_Limit(lift_Data.rb_lift_output,&cali_send[RB]);
//			Entirety_PID(&lift_Data,cali_send);  	//����PID����
			Lift_Cali_GYRO_Compensate(cali_send);	//�����ǲ���.��������3.14
			CAN_Chassis_SendMsg(0,0,0,0);
	//		CAN_Lift_SendMsg(0,0,0,0);
			CAN_Lift_SendMsg((s16)cali_send[LF],(s16)cali_send[RF],(s16)cali_send[LB],(s16)cali_send[RB]);
			CAN_Yun_SendMsg(yunMotorData.yaw_output+Yaw_output_offset(yunMotorData.yaw_fdbP),yunMotorData.pitch_output+Pitch_output_offset(yunMotorData.pitch_tarP));	//CAN2-1000
			break;
		}
		case ASCEND_STATE:	//�ǵ�״̬
		{
//		CAN_Chassis_SendMsg(0,0,0,0);
//			CAN_Lift_SendMsg(0,0,0,0);
			t_pitch_offset=Pitch_output_offset(yunMotorData.pitch_tarP);
			t_pitch_outsend=t_pitch_offset+yunMotorData.pitch_output;

			CAN_Yun_SendMsg(yunMotorData.yaw_output+Yaw_output_offset(yunMotorData.yaw_fdbP),t_pitch_outsend);	//CAN2-1000
			CAN_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
			CAN_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			break;
		}
		case DESCEND_STATE:
		{
			CAN_Chassis_SendMsg(0,0,0,0);
			CAN_Lift_SendMsg(0,0,0,0);
			CAN_Yun_SendMsg(0,0);	//CAN2
//			CAN_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
//			CAN_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			break;
		}
		case PROTECT_STATE:	//��������״̬
		{
			CAN_Yun_SendMsg(0,0);	//CAN2
			CAN_Chassis_SendMsg(0,0,0,0);
			CAN_Lift_SendMsg(0,0,0,0);
			break;
		}
		default:
		{
			CAN_Yun_SendMsg(0,0);	//CAN2	//yaw,pitch
			CAN_Chassis_SendMsg(0,0,0,0);
			CAN_Lift_SendMsg(0,0,0,0);
			break;
		}
	}
*/
}


void Cali_Output_Limit(float cm_out,float * cali_out)
{
	if(cm_out>LIFT_CALI_OUTPUT_MAX+4000)
	{
		*cali_out=LIFT_CALI_OUTPUT_MAX+4000;
	}
	else if(cm_out<-LIFT_CALI_OUTPUT_MAX)
	{
		*cali_out=-LIFT_CALI_OUTPUT_MAX;
	}
	else
	{
		*cali_out=cm_out;
	}
}


//#define PITCH 0	//�Ƶ�����ȥ��
//#define ROLL 1
//#define YAW 2
float Chassis_GYRO[3]={0};	//pitch roll yaw
/*************************************************
���ܣ��������ںϸ���������̬������ģ�����
���ݵ�λ���� Gyro_Data.angle
��̨����������������pitch:�� 	roll:����ҵ�	 yaw����ʱ��
���λ��������pitch����  yaw:��ʱ��
�����ֵ��YAW_INIT PITCH_INIT
�ںϺ�chassis����:pitch:��		roll����ҵ�		yaw:��ʱ��
**************************************************/
void Chassis_Attitude_Angle_Convert(void)	//�ۺϵó�������̬
{
	float deviation_pitch=PITCH_GYRO_INIT-yunMotorData.pitch_fdbP;	//���ڵ�����˵����̨��ֵ���ǵ�������̨����ϵ�ϵ�λ��
	float deviation_yaw=YAW_INIT-yunMotorData.yaw_fdbP;
	//��yaw��������ƣ���׼���㣨-180����+180��
	Chassis_GYRO[PITCH]=-Gyro_Data.angle[PITCH]-deviation_pitch*360.0f/8192;	//��Ϊ��̨���λ�÷������������������������෴pitch��-2
	Chassis_GYRO[ROLL]=Gyro_Data.angle[ROLL]-3;	//roll	-3Ϊ��ֹʱ����
	Chassis_GYRO[YAW]=Gyro_Data.angle[YAW]+deviation_yaw*360.0f/8192;	//��Ϊ��̨���λ�÷�������������������������ͬ
 
	//����-180_+180
	Chassis_GYRO[YAW]=Chassis_GYRO[YAW]>180?Chassis_GYRO[YAW]-360:Chassis_GYRO[YAW];
	Chassis_GYRO[YAW]=Chassis_GYRO[YAW]<-180?Chassis_GYRO[YAW]+360:Chassis_GYRO[YAW];
}



s32 t_entirety_lf=0;
s32 t_entirety_rf=0;
s32 t_entirety_lb=0;
s32 t_entirety_rb=0;
s32 t_out_lf=0;
s32 t_out_rf=0;
s32 t_out_lb=0;
s32 t_out_rb=0;
#define ENTIRETY_LIFT_P 8	//����PID����
void Entirety_PID(const LIFT_DATA * pliftdata,float cali_send[4])	//����PID����		//2018.2.26DEBUG��
{
	s32 lift_average=(s32)(pliftdata->lf_lift_fdbP+pliftdata->rf_lift_fdbP+pliftdata->lb_lift_fdbP+pliftdata->rb_lift_fdbP)/4;
	if(cali_state_Entirety_PID==1)
	{
		t_entirety_lf=(lift_average-pliftdata->lf_lift_fdbP)*ENTIRETY_LIFT_P;
		t_entirety_rf=(lift_average-pliftdata->rf_lift_fdbP)*ENTIRETY_LIFT_P;
		t_entirety_lb=(lift_average-pliftdata->lb_lift_fdbP)*ENTIRETY_LIFT_P;
		t_entirety_rb=(lift_average-pliftdata->rb_lift_fdbP)*ENTIRETY_LIFT_P;
		
		cali_send[LF]+=t_entirety_lf;
		cali_send[RF]+=t_entirety_rf;
		cali_send[LB]+=t_entirety_lb;
		cali_send[RB]+=t_entirety_rb;
		
		t_out_lf=cali_send[LF];
		t_out_rf=cali_send[RF];
		t_out_lb=cali_send[LB];
		t_out_rb=cali_send[RB];
	}
}

#define LIFT_GYRO_CALI_K 65
s16 t_cali_gyro_lf=0;
s16 t_cali_gyro_rf=0;
s16 t_cali_gyro_lb=0;
s16 t_cali_gyro_rb=0;
void Lift_Cali_GYRO_Compensate(float cali_send[4])	//���������ǵĵ��̱궨�������3.13���������
{	//��chassis_pitch>0ʱǰ�ߺ��  ��Ҫ��ǰ����LF RF���Ӻ�����LB RB
	//��roll>0ʱ����ҵ�	��Ҫ��������LF LB,��������RF RB
	if(cali_state_Entirety_PID==1)	
	{
		cali_send[LF]+=LIFT_GYRO_CALI_K*(-Chassis_GYRO[PITCH]-Chassis_GYRO[ROLL]);
		cali_send[RF]+=LIFT_GYRO_CALI_K*(-Chassis_GYRO[PITCH]+Chassis_GYRO[ROLL]);
		cali_send[LB]+=LIFT_GYRO_CALI_K*(Chassis_GYRO[PITCH]-Chassis_GYRO[ROLL]);
		cali_send[RB]+=LIFT_GYRO_CALI_K*(Chassis_GYRO[PITCH]+Chassis_GYRO[ROLL]);
		t_cali_gyro_lf=LIFT_GYRO_CALI_K*(-Chassis_GYRO[PITCH]-Chassis_GYRO[ROLL]);
		t_cali_gyro_rf=LIFT_GYRO_CALI_K*(-Chassis_GYRO[PITCH]+Chassis_GYRO[ROLL]);
		t_cali_gyro_lb=LIFT_GYRO_CALI_K*(Chassis_GYRO[PITCH]-Chassis_GYRO[ROLL]);
		t_cali_gyro_rb=LIFT_GYRO_CALI_K*(Chassis_GYRO[PITCH]+Chassis_GYRO[ROLL]);
	}
	  
}



u16 lift_time_gauge_count=0;
void Lift_Time_Gauge(u8 *trigger)	//����ʱ���Բ���
{
	if(*trigger==1)
	{
		lift_time_gauge_count++;
		if(SetCheck_FrontLift(1)==1||SetCheck_BackLift(1)==1)
		{
			*trigger=0;
			SetCheck_FrontLift(0);
			SetCheck_BackLift(0);
		}
	}
}


void KeyboardRetset(void)	//���ս���������⣬�ͽ��и�λ����
{
	if(KeyBoardData[KEY_CTRL].value==1&&KeyBoardData[KEY_SHIFT].value==1&&KeyBoardData[KEY_Z].value==0&&KeyBoardData[KEY_X].value==0&&KeyBoardData[KEY_C].value==1&&KeyBoardData[KEY_V].value==1)	//������Ƿ�ֹ��ʼ��ʱȫ��Ϊ0
	{
		NVIC_SystemReset();
	}
}

void Data_Init(void)	//�ں˸�λ����������
{
	RC_Ctl.rc.ch0=1024;
	RC_Ctl.rc.ch1=1024;
	RC_Ctl.rc.ch2=1024;
	RC_Ctl.rc.ch3=1024;
	RC_Ctl.rc.switch_left=3;
	RC_Ctl.rc.switch_right=3;
	time_1ms_count=0;
}

void RC_Calibration(void)	//�ϵ���ң��������ֵ����Ĭ�ϲ����Ƚϣ��ж��Ƿ�������������λ
{													//ע���������ң�������ճ�ʼ����
	if(abs(RC_Ctl.rc.ch0+RC_Ctl.rc.ch1+RC_Ctl.rc.ch2+RC_Ctl.rc.ch3-1024*4)>8)
	{
		NVIC_SystemReset();
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
/*********************************************��������״̬***********************************************/
RC_Ctl_t RC_DATA_ERROR={0};	//��¼����֡����
void Teleconltroller_Data_protect(void)	//ң���������Ա��� 
{
	u8 protect_state=0xC0;	//��λ��ʾ��ǰң���������Ƿ�����	//���2λΪ����λ����Ϊ1	//364-1024-1684
	protect_state|=(abs(RC_Ctl.rc.ch0-1024)<=662);
	protect_state|=(abs(RC_Ctl.rc.ch1-1024)<=662)<<1;
	protect_state|=(abs(RC_Ctl.rc.ch2-1024)<=662)<<2;
	protect_state|=(abs(RC_Ctl.rc.ch3-1024)<=662)<<3;
	protect_state|=(RC_Ctl.rc.switch_left==1||RC_Ctl.rc.switch_left==2||RC_Ctl.rc.switch_left==3)<<4;
	protect_state|=(RC_Ctl.rc.switch_right==1||RC_Ctl.rc.switch_right==2||RC_Ctl.rc.switch_right==3)<<5;
	
	if(protect_state!=0xFF)	{SetWorkState(PROTECT_STATE); RC_DATA_ERROR=RC_Ctl;}
}




////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

/***********************--����״̬--**********************/
void SetWorkState(WorkState_e state)
{
    workState = state;
}


WorkState_e GetWorkState()
{
	return workState;
}


