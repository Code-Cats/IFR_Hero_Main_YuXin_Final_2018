#include "control.h"

#define PITCH 0
#define ROLL 1
#define YAW 2

extern u8 IMU_Check_Useless_State;	//陀螺仪失效检测位

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

extern u8 Judge_Send_Statu;	//刷新标志位

s16 lift_tem=0;
s16 LIFT_tarP=0;

u8 t_lift_time_start=0;
///////////////////////////////////////remoteData
s16 Vw_tem=0;
///////////////////////////////////////

u32 time_1ms_count=0;

float t_yaw_error=0;	//临时测试
float t_pitch_error=0;	//临时测试
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

	IMU_Check_Useless();	//陀螺仪检测失效

	KeyboardRetset();	//对战场发生意外情况时，进行复位处理		CTRL SHIFT Z X按下	C V为0
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
		for(int keyid=0;keyid<KEY_NUMS;keyid++)	//放在定时器里
		{
			ButtonStatu_Verdict(&KeyBoardData[keyid]);
		}
	}
	
	
	Work_State_Change();
	
	Work_State_Change_BackProtect();
	
	Image_Cut_Task();	//摄像头切换、舵机
	
	switch (GetWorkState())	//2018.3.15	执行任务块
	{
		case CHECK_STATE:	//自检模式
		{	//板载外设初始化后便进入自检模式 //此时外设刚刚开启，需等待一段时间全局自检未检测到异常（2-3个自检触发周期以上），又因为时间计算起点为定时器启动点，故无需进行时间差记录
			if(time_1ms_count>300)	//若从LOST状态回到CHECK模式，则执行计数清零操作
			{	//若能执行到这里说明LOSTCHECK通过，进行数值检测
				RC_Calibration();	//self check
				if(1)	//selfcheck标志
				{
					SetWorkState(PREPARE_STATE);	//此步意味自检通过，一切硬件模块正常
					//数据初始化↓
					yunMotorData.pitch_tarP=PITCH_INIT;	//-50是因为陀螺仪水平时云台上扬	//陀螺仪正方向云台向下
					yunMotorData.yaw_tarP=(Gyro_Data.angle[2]*10+(YAW_INIT-yunMotorData.yaw_fdbP)*3600/8192);	//反馈放大10倍并将目标位置置为中点
				}
			}
			break;
		}
		case PREPARE_STATE:	//预备模式
		{	//等待车身状态稳定，并设置初值
			Yun_Task();	//开启底盘
//			if(abs(Gyro_Data.angvel[0])<20&&abs(Gyro_Data.angvel[2])<20&&abs(yunMotorData.pitch_tarP-(Gyro_Data.angle[0]*8192/360.0f+PITCH_INIT))<50)	//云台已就位	//位置环情况下
			if(abs(Gyro_Data.angvel[0])<20)	//云台已就位，单速度环情况
			{
				SetWorkState(CALI_STATE);
			}
			Shoot_Task();	//临时调试
			break;
		}
		case CALI_STATE:	//标定模式
		{
			if(Lift_Cali()==1)
			{
				SetWorkState(NORMAL_STATE);
			}
			Yun_Task();	//开启云台处理
			Lift_Task();	//开启升降
			Shoot_Task();	//临时调试
			TakeBullet_Control_Center();	//含有假设延时反馈和舵机执行，故加入
			break;
		}
		case NORMAL_STATE:	//正常操作模式
		{
			Replenish_Bullet_Task(KeyBoardData[KEY_R].value);
			Teleconltroller_Data_protect();	//遥控器数据保护
			Yun_Task();	//开启云台处理
			Remote_Task();	//执行移动
			AutoChassisAttitude_Lift_V2(Chassis_GYRO[PITCH]);	//再内部加模式限制和键位处理
			TakeBullet_Control_Center();	//取弹控制中心
			Lift_Task();	//开启升降
			Shoot_Task();
			
			break;
		}
		case WAIST_STATE:
		{
			Teleconltroller_Data_protect();	//遥控器数据保护
			Yun_Task();	//开启云台处理
			Remote_Task();	//执行移动
//			AutoChassisAttitude_Lift_V2(Chassis_GYRO[PITCH]);
			Lift_Task();	//开启升降
			Shoot_Task();
			TakeBullet_Control_Center();	//含有假设延时反馈和舵机执行，故加入
			break;
		}
		case ASCEND_STATE:	//自动上岛模式
		{
			Ascend_Control_Center();
			Yun_Task();	//开启云台处理
			Remote_Task();	//执行移动
			Lift_Task();	//开启升降
			TakeBullet_Control_Center();	//含有假设延时反馈和舵机执行，故加入
			break;
		}
		case DESCEND_STATE:	//自动下岛模式
		{
			ViceControlData.image_cut[0]=1;	//高电平
			Descend_Control_Center();
			Yun_Task();	//开启云台处理
			Remote_Task();	//执行移动
			Lift_Task();	//开启升降
			TakeBullet_Control_Center();	//含有假设延时反馈和舵机执行，故加入
			break;
		}
		case TAKEBULLET_STATE:
		{
			Teleconltroller_Data_protect();	//遥控器数据保护
			TakeBullet_Control_Center();	//取弹控制中心
			Yun_Task();	//开启云台处理
			Remote_Task();	//执行移动
			Lift_Task();	//开启升降
			Shoot_Task();
			break;
		}
		case ERROR_STATE:	//错误模式
		{
			
			break;
		}
		case LOST_STATE:	//错误模式
		{
			break;
		}
		case STOP_STATE:	//停止状态
		{
//////			Vision_Task(&yunMotorData.yaw_tarP,&yunMotorData.pitch_tarP);
//////			Yun_Task();	//开启底盘
		//	Vision_Task(&t_yaw_error,&t_pitch_error);
			TakeBullet_Control_Center();	//含有假设延时反馈和舵机执行，故加入
			break;
		}
		case PROTECT_STATE:	//自我保护模式
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
RC或PC对机器状态的切换
*************************************/
void Work_State_Change(void)
{
	static u8 Switch_Right_Last=0;
	static WorkState_e State_Record=CHECK_STATE;	
	State_Record=GetWorkState();
	
	
	switch (GetWorkState())	//2018.3.15
	{
		case CHECK_STATE:	//自检模式
		{	//板载外设初始化后便进入自检模式 
			
			break;
		}
		case PREPARE_STATE:	//预备模式
		{	
			
			break;
		}
		case CALI_STATE:	//标定模式
		{
			
			break;
		}
		case NORMAL_STATE:	//正常操作模式
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_MIDDLE)	//左中右中
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
//				SetWorkState(TAKEBULLET_STATE);	//增加新模式//临时测试，取弹状态
			}
			
			if(RC_Ctl.rc.switch_left==RC_SWITCH_DOWN)	//左下取弹
			{
				SetWorkState(TAKEBULLET_STATE);	//增加新模式//临时测试，取弹状态
			}
			
			if(RC_Ctl.mouse.press_r==1&&RC_Ctl.rc.switch_left!=RC_SWITCH_MIDDLE)	//中间能不能扭腰？
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
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_MIDDLE)	//左中
			{
				SetWorkState(STOP_STATE);
			}
			break;
		}
		case ASCEND_STATE:	//自动上岛模式
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_MIDDLE)	//左中
			{
				AscendState=FULLRISE_GO1;	//重置防止下一次异常
				SetWorkState(STOP_STATE);
			}
			break;
		}
		case DESCEND_STATE:	//自动下岛模式
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_MIDDLE)	
			{
				DescendState=FULLFALL_DOWN1;	//重置防止下一次异常
				SetWorkState(STOP_STATE);
			}
			break;
		}
		case TAKEBULLET_STATE:	//取弹模式
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_MIDDLE)	//左中
			{
				SetWorkState(STOP_STATE);
			}
			
			if(RC_Ctl.rc.switch_left==RC_SWITCH_UP)	
			{
				SetWorkState(NORMAL_STATE);
			}
			break;
		}
		case ERROR_STATE:	//错误模式
		{
			if(RC_Ctl.key.v_h!=0||RC_Ctl.key.v_l!=0||abs(RC_Ctl.mouse.x)>3)	//退出该模式
			{
				SetWorkState(NORMAL_STATE);
			}
			break;
		}
		case LOST_STATE:	//错误模式
		{
			SetWorkState(CHECK_STATE);
			time_1ms_count=0;	//进入初始状态重新自检
			break;
		}
		case STOP_STATE:	//停止状态
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
//				SetWorkState(TAKEBULLET_STATE);	//增加新模式//临时测试，取弹状态
			}
			break;
		}
		case PROTECT_STATE:	//自我保护模式
		{
			static u32 time_count=0;
			time_count++;
			if(Error_Check.statu[LOST_DBUS]==0&&abs(RC_Ctl.rc.ch0+RC_Ctl.rc.ch1+RC_Ctl.rc.ch2+RC_Ctl.rc.ch3-1024*4)>8)
			{
				yunMotorData.yaw_tarP=(Gyro_Data.angle[2]*10+(YAW_INIT-yunMotorData.yaw_fdbP)*3600/8192);	//重置云台目标位置
				SetWorkState(NORMAL_STATE);
				time_count=0;
			}
			
			if(Error_Check.statu[LOST_DBUS]==0&&time_count>8000)	//有反馈认为无法恢复
			{
				NVIC_SystemReset();
			}
			break;
		}
	}
	Switch_Right_Last=RC_Ctl.rc.switch_right;
}


extern u8 descend_valve_prepare_state;	//自动下岛电磁阀到位保护
extern  u32 descend_valve_prepare_state_count;

extern u8 auto_takebullet_statu;
extern u8 SetCheck_TakeBullet_TakeBack_statu;	//切出取弹保护执行标志位	//放在前面extern
void Work_State_Change_BackProtect(void)	//当从某一状态退出时，确保该状态的一切遗留控制都归位
{
	static WorkState_e State_Record=CHECK_STATE;
 
	if(State_Record!=DESCEND_STATE&&GetWorkState()==DESCEND_STATE)
	{
		descend_valve_prepare_state=0;
		descend_valve_prepare_state_count=0;
	}
	
	if(State_Record==TAKEBULLET_STATE&&GetWorkState()!=TAKEBULLET_STATE)	//退出取弹模式
	{
		SetCheck_TakeBullet_TakeBack_statu=1;	//刷新处
//		auto_takebullet_statu=0;	//重置//因为5.15现行逻辑是在等待取弹完成，又一次为检测，所以不能对其进行操作
	}
	
	if(State_Record!=TAKEBULLET_STATE&&GetWorkState()==TAKEBULLET_STATE)
	{
		SetCheck_GripLift(1);	//上升到取弹高度
		auto_takebullet_statu=0;	//这里待定影不影响
	}
	SetCheck_TakeBullet_TakeBack();	//执行处
	State_Record=GetWorkState();
}



extern s16 t_error_record;
void LED_Indicate(void)
{
	if(time_1ms_count%BLINK_CYCLE==0)
	{
		switch (GetWorkState())	//2018.3.15
		{
			case CHECK_STATE:	//自检模式
			{	//板载外设初始化后便进入自检模式 
				LED_Blink_Set(10,10);
				break;
			}
			case PREPARE_STATE:	//预备模式
			{	
				LED_Blink_Set(9,9);
				break;
			}
			case CALI_STATE:	//标定模式	红开绿闪
			{
				LED_Blink_Set(9,9);
				break;
			}
			case NORMAL_STATE:	//正常操作模式	红关绿闪
			{
				LED_Blink_Set(1,0);
				break;
			}
			case WAIST_STATE:
			{
				LED_Blink_Set(1,0);
				break;
			}
			case ASCEND_STATE:	//自动上岛模式	绿闪
			{
				LED_Blink_Set(1,0);
				break;
			}
			case DESCEND_STATE:	//自动下岛模式	绿闪
			{
				LED_Blink_Set(1,0);
				break;
			}
			case TAKEBULLET_STATE:
			{
				LED_Blink_Set(1,0);
				break;
			}
			case ERROR_STATE:	//错误模式
			{
				if(t_error_record==LOST_SM_DOWN)	//下拨弹
				{
					LED_Blink_Set(3,10);
				}
				else if(t_error_record==LOST_SM_UP)	//上拨弹
				{
					LED_Blink_Set(4,10);
				}
				else if(t_error_record==LOST_CM1||t_error_record==LOST_CM2||t_error_record==LOST_CM3||t_error_record==LOST_CM4)	//底盘电机
				{
					LED_Blink_Set(2,10);
				}
				else if(t_error_record==LOST_LIFT1||t_error_record==LOST_LIFT2||t_error_record==LOST_LIFT3||t_error_record==LOST_LIFT4)	//升降
				{
					LED_Blink_Set(1,10);
				}
				else if(t_error_record==LOST_PITCH||t_error_record==LOST_YAW)	//云台
				{
					LED_Blink_Set(5,10);
				}
				else
				{
					LED_Blink_Set(6,10);
				}
				break;
			}
			case LOST_STATE:	//错误模式
			{
				LED_Blink_Set(1,1);
				break;
			}
			case STOP_STATE:	//停止状态	红闪
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
			case PROTECT_STATE:	//自我保护模式	双闪
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
		
		if(RC_Ctl.rc.switch_left==RC_SWITCH_UP)	//左上
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
//////////			SetCheck_FrontLift(lift_control_all_state);//？？？？？？？？？？？忘了这是干什么的
//////////			SetCheck_BackLift(lift_control_all_state);
		}
	}
	
	
	if(GetWorkState()!=CALI_STATE)	//标定状态下不限制行程
	{
		lift_Data.lf_lift_tarP=lift_Data.lf_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.lf_lift_tarP;	//限制行程
		lift_Data.lf_lift_tarP=lift_Data.lf_lift_tarP>LIFT_DISTANCE_LIMIT?LIFT_DISTANCE_LIMIT:lift_Data.lf_lift_tarP;
		
		lift_Data.rf_lift_tarP=lift_Data.rf_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.rf_lift_tarP;	//限制行程
		lift_Data.rf_lift_tarP=lift_Data.rf_lift_tarP>LIFT_DISTANCE_LIMIT?LIFT_DISTANCE_LIMIT:lift_Data.rf_lift_tarP;
		
		lift_Data.lb_lift_tarP=lift_Data.lb_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.lb_lift_tarP;	//限制行程
		lift_Data.lb_lift_tarP=lift_Data.lb_lift_tarP>LIFT_DISTANCE_LIMIT?LIFT_DISTANCE_LIMIT:lift_Data.lb_lift_tarP;
		
		lift_Data.rb_lift_tarP=lift_Data.rb_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.rb_lift_tarP;	//限制行程
		lift_Data.rb_lift_tarP=lift_Data.rb_lift_tarP>LIFT_DISTANCE_LIMIT?LIFT_DISTANCE_LIMIT:lift_Data.rb_lift_tarP;
	}
	
	lift_Data.lf_lift_tarV=(int32_t)PID_General(lift_Data.lf_lift_tarP,lift_Data.lf_lift_fdbP,&PID_Lift_Position[LF]);	//位置环PID计算
	lift_Data.rf_lift_tarV=(int32_t)PID_General(lift_Data.rf_lift_tarP,lift_Data.rf_lift_fdbP,&PID_Lift_Position[RF]);
	lift_Data.lb_lift_tarV=(int32_t)PID_General((lift_Data.lb_lift_tarP),lift_Data.lb_lift_fdbP,&PID_Lift_Position[LB]);
	lift_Data.rb_lift_tarV=(int32_t)PID_General((lift_Data.rb_lift_tarP),lift_Data.rb_lift_fdbP,&PID_Lift_Position[RB]);
	
	lift_Data.lf_lift_output=PID_General(lift_Data.lf_lift_tarV,lift_Data.lf_lift_fdbV,&PID_Lift_Speed[LF]);	//速度环PID计算
	lift_Data.rf_lift_output=PID_General(lift_Data.rf_lift_tarV,lift_Data.rf_lift_fdbV,&PID_Lift_Speed[RF]);
	lift_Data.lb_lift_output=PID_General(lift_Data.lb_lift_tarV,lift_Data.lb_lift_fdbV,&PID_Lift_Speed[LB]);
	lift_Data.rb_lift_output=PID_General(lift_Data.rb_lift_tarV,lift_Data.rb_lift_fdbV,&PID_Lift_Speed[RB]);
	
//	CAN_Lift_SendMsg(lift_Data.lf_lift_output,lift_Data.rf_lift_output,lift_Data.lb_lift_output,lift_Data.rb_lift_output);
	
}


/*****************************************第三代标定程序**********************************************/
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
			if((lift_Data.lf_lift_fdbV+lift_Data.rf_lift_fdbV+lift_Data.lb_lift_fdbV+lift_Data.rb_lift_fdbV)<-25)	//即已过速度零点
			{
				liftcaliState=DOWN_STATE;
			}
			break;
		}
		case DOWN_STATE:
		{
			if(abs(lift_Data.lf_lift_fdbV+lift_Data.rf_lift_fdbV+lift_Data.lb_lift_fdbV+lift_Data.rb_lift_fdbV)<10)	//上一次这一次位置值之差//改为了
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
				
				cali_state_Entirety_PID=0;	//清零
				return 1;
//				SetWorkState(NORMAL_STATE);	//3.16暂时加
			}
			break;
		}
	}
	return 0;
}



u8 cali_state_Entirety_PID=0;	//整体PID在标定过程中的触发时机标志位
u8 Calibration_state=0;
s16 t_fdbV_sum=0;
u16 t_cali_count=0;
/*****************************************第二代标定程序**********************************************/
void Lift_Calibration(void)	//升降电机上电标定
{
//	u32 Record_Last=0;	//标定程序中上一次角度纪录值
	u8 Calibration_Time_Count=0;	//计时
	
	
	SetWorkState(CALI_STATE);
	PID_Lift_Speed[LF].k_i=2*LIFT_SPEED_PID_I;
	PID_Lift_Speed[RF].k_i=2*LIFT_SPEED_PID_I;
	PID_Lift_Speed[LB].k_i=2*LIFT_SPEED_PID_I;
	PID_Lift_Speed[RB].k_i=2*LIFT_SPEED_PID_I;
	
	lift_Data.lf_lift_tarP=100;
	lift_Data.rf_lift_tarP=100;
	lift_Data.lb_lift_tarP=100;
	lift_Data.rb_lift_tarP=100;
	while(lift_Data.lf_lift_fdbP<40||lift_Data.rf_lift_fdbP<40||lift_Data.lb_lift_fdbP<40||lift_Data.rb_lift_fdbP<40)	//等待上升到一定高度
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
	
	Calibration_Time_Count=time_1ms_count;	//	记录当前系统时间
		
	while(Calibration_state!=1||lift_position_encoder[LF].turns!=0||lift_position_encoder[RF].turns!=0||lift_position_encoder[LB].turns!=0||lift_position_encoder[RB].turns!=0)
	{
		if((time_1ms_count-Calibration_Time_Count)>1000)	//此延时作用是让升降电机过原点
		{
			
			if(time_1ms_count%500==0)
			{
				t_cali_count++;
				t_fdbV_sum=abs(lift_Data.lf_lift_fdbV+lift_Data.rf_lift_fdbV+lift_Data.lb_lift_fdbV+lift_Data.rb_lift_fdbV);
				if(abs(lift_Data.lf_lift_fdbV+lift_Data.rf_lift_fdbV+lift_Data.lb_lift_fdbV+lift_Data.rb_lift_fdbV)<15)	//上一次这一次位置值之差//改为了
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
					
					cali_state_Entirety_PID=0;	//清零
					SetWorkState(NORMAL_STATE);	//3.16暂时加
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
		case CHECK_STATE:	//自检模式
		{	//板载外设初始化后便进入自检模式 //此时外设刚刚开启，需等待一段时间全局自检未检测到异常（2-3个自检触发周期以上），又因为时间计算起点为定时器启动点，故无需进行时间差记录
			CAN1_Yun_SendMsg(0,0);	//CAN2	//yaw,pitch
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_Shoot_SendMsg(0,0);//下拨弹、上拨弹
//			CAN2_Shoot_SendMsg((s16)shoot_Motor_Data_Down.output,0);	//下拨弹、上拨弹
			break;
		}
		case PREPARE_STATE:	//预备模式
		{	//等待车身状态稳定，并设置初值
			CAN1_Yun_SendMsg(yunMotorData.yaw_output,yunMotorData.pitch_output);	//CAN2-1000	//取消反馈补偿
	//		CAN1_Yun_SendMsg(t_yaw_16t,t_pitch_16t);	//调试用模式
//			CAN_Motor6623_calibration();
			//CAN1_Yun_SendMsg(yunMotorData.yaw_output+Yaw_output_offset(yunMotorData.yaw_fdbP),yunMotorData.pitch_output+Pitch_output_offset(yunMotorData.pitch_tarP));	//CAN2-1000
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			//CAN2_Shoot_SendMsg(0,0);//下拨弹、上拨弹
			CAN2_Shoot_SendMsg((s16)shoot_Motor_Data_Down.output,0);	//下拨弹、上拨弹
			break;
		}
		case CALI_STATE:	//标定模式
		{
			t_send_count++;
			SetFrictionWheelSpeed(FRICTION_INIT);
			
			Cali_Output_Limit(lift_Data.lf_lift_output,&cali_send[LF]);
			Cali_Output_Limit(lift_Data.rf_lift_output,&cali_send[RF]);
			Cali_Output_Limit(lift_Data.lb_lift_output,&cali_send[LB]);
			Cali_Output_Limit(lift_Data.rb_lift_output,&cali_send[RB]);
//		Entirety_PID(&lift_Data,cali_send);  	//整体PID补偿
			if(IMU_Check_Useless_State==0)
			{
				Lift_Cali_GYRO_Compensate(cali_send);	//陀螺仪补偿.存在问题3.14
			}
//			CAN1_Yun_SendMsg(yunMotorData.yaw_output+Yaw_output_offset(yunMotorData.yaw_fdbP),yunMotorData.pitch_output+Pitch_output_offset(yunMotorData.pitch_tarP));	//CAN2-1000	//加入反馈补偿
			CAN1_Yun_SendMsg(yunMotorData.yaw_output,yunMotorData.pitch_output);	//CAN2-1000	//取消反馈补偿
//			CAN1_Yun_SendMsg(0,0);
			CAN2_Chassis_SendMsg(0,0,0,0);
//		CAN1_Lift_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg((s16)cali_send[LF],(s16)cali_send[RF],(s16)cali_send[LB],(s16)cali_send[RB]);
//			CAN2_Shoot_SendMsg(0,0);//下拨弹、上拨弹

			CAN2_Shoot_SendMsg((s16)shoot_Motor_Data_Down.output,0);	//下拨弹、上拨弹
			break;
		}
		case NORMAL_STATE:	//正常操作模式
		{
			CAN1_Yun_SendMsg((s16)yunMotorData.yaw_output,(s16)yunMotorData.pitch_output);	//CAN2-1000	//取消反馈补偿	正常程序模式
//		CAN_Yun_SendMsg(0,0);
//			CAN1_Yun_SendMsg(t_yaw_16t,t_pitch_16t);	//调试用模式
			
//		CAN_Chassis_SendMsg((s16)remote_tem,(s16)remote_tem,(s16)remote_tem,(s16)remote_tem);
//			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
			CAN2_Chassis_SendMsg(0,0,0,0);
			
//    CAN1_Lift_SendMsg((s16)lift_tem,(s16)lift_tem,(s16)lift_tem,(s16)lift_tem);
			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			CAN2_Shoot_SendMsg((s16)shoot_Motor_Data_Down.output,(s16)shoot_Motor_Data_Up.output);	//下拨弹、上拨弹
			break;
		}
		case WAIST_STATE:
		{
			CAN1_Yun_SendMsg(yunMotorData.yaw_output,yunMotorData.pitch_output);	//CAN2-1000	//取消反馈补偿
//		CAN_Yun_SendMsg(0,0);
//		CAN_Chassis_SendMsg((s16)remote_tem,(s16)remote_tem,(s16)remote_tem,(s16)remote_tem);
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
//			CAN_Chassis_SendMsg(0,0,0,0);
//    CAN_Lift_SendMsg((s16)lift_tem,(s16)lift_tem,(s16)lift_tem,(s16)lift_tem);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_Shoot_SendMsg((s16)shoot_Motor_Data_Down.output,0);	//下拨弹、上拨弹
			break;
		}
		case ERROR_STATE:	//错误模式
		{
			CAN1_Yun_SendMsg(0,0);	//CAN2	//yaw,pitch
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_Shoot_SendMsg(0,0);//下拨弹、上拨弹
			break;
		}
		case STOP_STATE:	//停止状态
		{
			CAN1_Yun_SendMsg(0,0);	//CAN2	//yaw,pitch
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_Shoot_SendMsg(0,0);//下拨弹、上拨弹
			break;
		}
		case PROTECT_STATE:	//自我保护模式
		{
			CAN1_Yun_SendMsg(0,0);	//CAN2	//yaw,pitch
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_Shoot_SendMsg(0,0);//下拨弹、上拨弹
			break;
		}
		case ASCEND_STATE:	//自动上岛模式
		{
			CAN1_Yun_SendMsg(yunMotorData.yaw_output+Yaw_output_offset(yunMotorData.yaw_fdbP),yunMotorData.pitch_output+Pitch_output_offset(yunMotorData.pitch_tarP));	//CAN2-1000
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			CAN2_Shoot_SendMsg(0,0);//下拨弹、上拨弹
			break;
		}
		case DESCEND_STATE:	//自动下岛模式
		{
			CAN1_Yun_SendMsg(yunMotorData.yaw_output+Yaw_output_offset(yunMotorData.yaw_fdbP),yunMotorData.pitch_output+Pitch_output_offset(yunMotorData.pitch_tarP));	//CAN2-1000
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			CAN2_Shoot_SendMsg(0,0);//下拨弹、上拨弹
			break;
		}
		case TAKEBULLET_STATE:
		{
			CAN1_Yun_SendMsg(yunMotorData.yaw_output,yunMotorData.pitch_output);	//CAN2-1000	//取消反馈补偿
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			CAN2_Shoot_SendMsg((s16)shoot_Motor_Data_Down.output,(s16)shoot_Motor_Data_Up.output);//下拨弹、上拨弹
			break;
		}
		default:
		{
			CAN1_Yun_SendMsg(0,0);	//CAN2	//yaw,pitch
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_Shoot_SendMsg(0,0);//下拨弹、上拨弹
			break;
		}
	}
	
/*	
	switch (GetWorkState())
	{
		case PREPARE_STATE:	//预备模式
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
		case STOP_STATE:	//主动保护状态
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
//			Entirety_PID(&lift_Data,cali_send);  	//整体PID补偿
			Lift_Cali_GYRO_Compensate(cali_send);	//陀螺仪补偿.存在问题3.14
			CAN_Chassis_SendMsg(0,0,0,0);
	//		CAN_Lift_SendMsg(0,0,0,0);
			CAN_Lift_SendMsg((s16)cali_send[LF],(s16)cali_send[RF],(s16)cali_send[LB],(s16)cali_send[RB]);
			CAN_Yun_SendMsg(yunMotorData.yaw_output+Yaw_output_offset(yunMotorData.yaw_fdbP),yunMotorData.pitch_output+Pitch_output_offset(yunMotorData.pitch_tarP));	//CAN2-1000
			break;
		}
		case ASCEND_STATE:	//登岛状态
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
		case PROTECT_STATE:	//被动保护状态
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


//#define PITCH 0	//移到上面去了
//#define ROLL 1
//#define YAW 2
float Chassis_GYRO[3]={0};	//pitch roll yaw
/*************************************************
功能：经数据融合给出底盘姿态供其他模块调用
数据单位：度 Gyro_Data.angle
云台陀螺仪数据正方向：pitch:下 	roll:左高右低	 yaw：逆时针
电机位置正方向：pitch：下  yaw:逆时针
电机中值：YAW_INIT PITCH_INIT
融合后chassis方向:pitch:上		roll左高右低		yaw:逆时针
**************************************************/
void Chassis_Attitude_Angle_Convert(void)	//综合得出底盘姿态
{
	float deviation_pitch=PITCH_GYRO_INIT-yunMotorData.pitch_fdbP;	//对于底盘来说，云台中值即是底盘在云台坐标系上的位置
	float deviation_yaw=YAW_INIT-yunMotorData.yaw_fdbP;
	//对yaw轴进行限制，标准过零（-180——+180）
	Chassis_GYRO[PITCH]=-Gyro_Data.angle[PITCH]-deviation_pitch*360.0f/8192;	//因为云台电机位置反馈正方向与陀螺仪正方向相反pitch？-2
	Chassis_GYRO[ROLL]=Gyro_Data.angle[ROLL]-3;	//roll	-3为静止时补偿
	Chassis_GYRO[YAW]=Gyro_Data.angle[YAW]+deviation_yaw*360.0f/8192;	//因为云台电机位置反馈正方向与陀螺仪正方向相同
 
	//限制-180_+180
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
#define ENTIRETY_LIFT_P 8	//整体PID参数
void Entirety_PID(const LIFT_DATA * pliftdata,float cali_send[4])	//整体PID补偿		//2018.2.26DEBUG版
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
void Lift_Cali_GYRO_Compensate(float cali_send[4])	//基于陀螺仪的底盘标定输出补偿3.13晚存在问题
{	//当chassis_pitch>0时前高后低  需要减前两个LF RF，加后两个LB RB
	//当roll>0时左高右低	需要减左两个LF LB,加右两个RF RB
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
void Lift_Time_Gauge(u8 *trigger)	//升降时间自测量
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


void KeyboardRetset(void)	//如果战场发生意外，就进行复位处理
{
	if(KeyBoardData[KEY_CTRL].value==1&&KeyBoardData[KEY_SHIFT].value==1&&KeyBoardData[KEY_Z].value==0&&KeyBoardData[KEY_X].value==0&&KeyBoardData[KEY_C].value==1&&KeyBoardData[KEY_V].value==1)	//后面的是防止初始化时全部为0
	{
		NVIC_SystemReset();
	}
}

void Data_Init(void)	//内核复位后数据重置
{
	RC_Ctl.rc.ch0=1024;
	RC_Ctl.rc.ch1=1024;
	RC_Ctl.rc.ch2=1024;
	RC_Ctl.rc.ch3=1024;
	RC_Ctl.rc.switch_left=3;
	RC_Ctl.rc.switch_right=3;
	time_1ms_count=0;
}

void RC_Calibration(void)	//上电检测遥控器接收值并与默认参数比较，判断是否正常，否则软复位
{													//注：必须放在遥控器接收初始化后
	if(abs(RC_Ctl.rc.ch0+RC_Ctl.rc.ch1+RC_Ctl.rc.ch2+RC_Ctl.rc.ch3-1024*4)>8)
	{
		NVIC_SystemReset();
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
/*********************************************主动保护状态***********************************************/
RC_Ctl_t RC_DATA_ERROR={0};	//记录错误帧数据
void Teleconltroller_Data_protect(void)	//遥控器数据自保护 
{
	u8 protect_state=0xC0;	//按位表示当前遥控器数据是否正常	//最高2位为保留位，常为1	//364-1024-1684
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

/***********************--工作状态--**********************/
void SetWorkState(WorkState_e state)
{
    workState = state;
}


WorkState_e GetWorkState()
{
	return workState;
}


