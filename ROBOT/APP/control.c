#include "control.h"

#define PITCH 0
#define ROLL 1
#define YAW 2

extern u8 IMU_Check_Useless_State;	//陀螺仪失效检测位

WorkState_e workState=PREPARE_STATE;


extern tGameRobotState         testGameRobotState;      //比赛机器人状态
extern tPowerHeatData 				  testPowerHeatData;      //实时功率热量数据
//LIFT_DATA lift_Data={0};
//PID_GENERAL PID_Lift_Position[4]={PID_LIFT_POSITION_DEFAULT,PID_LIFT_POSITION_DEFAULT,PID_LIFT_POSITION_DEFAULT,PID_LIFT_POSITION_DEFAULT};
//PID_GENERAL PID_Lift_Speed[4]={PID_LIFT_SPEED_DEFAULT,PID_LIFT_SPEED_DEFAULT,PID_LIFT_SPEED_DEFAULT,PID_LIFT_SPEED_DEFAULT};
//extern LIFT_POSITION_ENCODER lift_position_encoder[4];
//extern u8 cali_state_Entirety_PID;

extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
extern RC_Ctl_t RC_Ctl;

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

extern u8 Replenish_Bullet_Statu;

extern u8 Judge_Send_Statu;	//刷新标志位

u32 time_1ms_count=0;

void Control_Task(void)	//2ms
{
	time_1ms_count++;

	if(time_1ms_count%100==0)
	{
		Judge_Send_Statu=1; 
	}

	if(time_1ms_count%10==0)
	{
		Super_Capacitor_Task(testPowerHeatData.chassisPower,testPowerHeatData.chassisPowerBuffer);
	}
	
	BulletNum_Calculate();
	if(time_1ms_count%10==0)
	{
		Heat_Simulating(testPowerHeatData.shooterHeat1,testGameRobotState.remainHP);	//本地端热量模拟
	}
	
	
	Check_Task();

	IMU_Check_Useless();	//陀螺仪检测失效

	KeyboardRetset();	//对战场发生意外情况时，进行复位处理		CTRL SHIFT Z X按下	C V为0
	
	if(time_1ms_count%1==0)	//1000hz
	{
		for(int keyid=0;keyid<KEY_NUMS;keyid++)	//放在定时器里
		{
			ButtonStatu_Verdict(&KeyBoardData[keyid]);
		}
	}
	
	Work_State_Change();	//工作状态切换
	
	Work_State_Change_BackProtect();	//工作状态切换保护
	
	Work_Execute();	//工作状态执行
	
//	Image_Cut_Task();	//摄像头切换、舵机
	
	LED_Indicate();
	
	Motor_Send();
	if(time_1ms_count%2==0)
	{
		ViceBoard_SendDataRefresh();
		ViceBoard_SendDataRun();
	}
	
}

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
//				SetWorkState(ASCEND_STATE);
			}
			else if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE&&Switch_Right_Last==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)
			{
//				SetWorkState(DESCEND_STATE);
//				SetWorkState(TAKEBULLET_STATE);	//增加新模式//临时测试，取弹状态
			}
			
			if(RC_Ctl.rc.switch_left==RC_SWITCH_DOWN)	//左下取弹
			{
				SetWorkState(TAKEBULLET_STATE);	//增加新模式//临时测试，取弹状态
			}
			
			if(KeyBoardData[KEY_SHIFT].value==1&&RC_Ctl.rc.switch_left!=RC_SWITCH_MIDDLE)	//中间能不能扭腰？
			{
				SetWorkState(WAIST_STATE);
			}
			break;
		}
		case WAIST_STATE:
		{
			if(KeyBoardData[KEY_SHIFT].value!=1)
			{
				SetWorkState(NORMAL_STATE);
			}
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_MIDDLE)	//左中
			{
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
//				SetWorkState(ASCEND_STATE);
			}
			else if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE&&Switch_Right_Last==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)
			{
//				SetWorkState(DESCEND_STATE);
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


extern u8 SetCheck_TakeBullet_TakeBack_statu;	//切出取弹保护执行标志位	//放在前面extern
void Work_State_Change_BackProtect(void)	//当从某一状态退出时，确保该状态的一切遗留控制都归位	//新版取弹不需要，因为改写了结构，一切动作在自动取弹中都会顺延完成
{
	static WorkState_e State_Record=CHECK_STATE;
	
	if(State_Record==TAKEBULLET_STATE&&GetWorkState()!=TAKEBULLET_STATE)	//退出取弹模式
	{

	}
	
	if(State_Record!=TAKEBULLET_STATE&&GetWorkState()==TAKEBULLET_STATE)
	{

	}
	State_Record=GetWorkState();
}


void Work_Execute(void)	//工作执行2018.7.1
{
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
					yunMotorData.yaw_tarP=(Gyro_Data.angle[YAW]*10+(YAW_INIT-yunMotorData.yaw_fdbP)*3600/8192);	//反馈放大10倍并将目标位置置为中点
				}
			}
			break;
		}
		case PREPARE_STATE:	//预备模式
		{	//等待车身状态稳定，并设置初值
			Yun_Task();	//开启底盘
//			if(abs(Gyro_Data.angvel[0])<20&&abs(Gyro_Data.angvel[2])<20&&abs(yunMotorData.pitch_tarP-(Gyro_Data.angle[0]*8192/360.0f+PITCH_INIT))<50)	//云台已就位	//位置环情况下
			if(abs(Gyro_Data.angvel[YAW])<2&&Error_Check.statu[LOST_IMU]!=1)	//云台已就位，且有反馈
			{
				SetWorkState(CALI_STATE);
			}
			Shoot_Task();	//临时调试
			break;
		}
		case CALI_STATE:	//标定模式
		{
			if(BulletRotate_OffSetInit()==1&&Imu_Cali_State()==1)	//待写取弹电机初始标定
			{
				SetWorkState(NORMAL_STATE);
			}
			Yun_Task();	//开启云台处理
			Shoot_Task();	//临时调试
			TakeBullet_Control_Center();	//含有假设延时反馈和舵机执行，故加入
			break;
		}
		case NORMAL_STATE:	//正常操作模式
		{
	//		Replenish_Bullet_Task(KeyBoardData[KEY_R].value);
			Teleconltroller_Data_protect();	//遥控器数据保护
			Yun_Task();	//开启云台处理
			Remote_Task();	//执行移动
			TakeBullet_Control_Center();	//取弹控制中心
			Shoot_Task();
			BulletRotate_Task();
			break;
		}
		case WAIST_STATE:
		{
			Teleconltroller_Data_protect();	//遥控器数据保护
			Yun_Task();	//开启云台处理
			Remote_Task();	//执行移动
//			AutoChassisAttitude_Lift_V2(Chassis_GYRO[PITCH]);
			Shoot_Task();
			TakeBullet_Control_Center();	//含有假设延时反馈和舵机执行，故加入
			BulletRotate_Task();
			break;
		}
		case TAKEBULLET_STATE:
		{
			Teleconltroller_Data_protect();	//遥控器数据保护
			TakeBullet_Control_Center();	//取弹控制中心
			Yun_Task();	//开启云台处理
			Remote_Task();	//执行移动
			Shoot_Task();
			BulletRotate_Task();
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
			Yun_Task();	//开启云台
		//	Vision_Task(&t_yaw_error,&t_pitch_error);
			TakeBullet_Control_Center();	//含有假设延时反馈和舵机执行，故加入
			BulletRotate_Task();
			break;
		}
		case PROTECT_STATE:	//自我保护模式
		{
			break;
		}
	}
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
				else if(t_error_record==LOST_PITCH)	//云台||t_error_record==LOST_YAW
				{
					LED_Blink_Set(5,10);
				}
				else if(t_error_record==LOST_YAW)	//LOST_BULLETROTATE1
				{
					LED_Blink_Set(1,10);
				}
				else if(t_error_record==LOST_IMU)
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


//extern YUN_MOTOR_DATA 			yunMotorData;
extern BULLETROTATE_DATA BulletRotate_Data;
s16 t_yaw_send,t_pitch_send=0;

void Motor_Send(void)
{
	switch (GetWorkState())	//2018.3.15
	{	
		case CHECK_STATE:	//自检模式
		{	//板载外设初始化后便进入自检模式 //此时外设刚刚开启，需等待一段时间全局自检未检测到异常（2-3个自检触发周期以上），又因为时间计算起点为定时器启动点，故无需进行时间差记录
			CAN1_Yun_SendMsg(0,0);	//CAN2	//yaw,pitch
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN2_Shoot_Bullet_SendMsg(0,0,0,0);//取弹旋转、0、下拨弹、上拨弹
//			CAN2_Shoot_SendMsg((s16)shoot_Motor_Data_Down.output,0);	//下拨弹、上拨弹
			break;
		}
		case PREPARE_STATE:	//预备模式
		{	//等待车身状态稳定，并设置初值
			CAN1_Yun_SendMsg(0,0);	//CAN2	//yaw,pitch
////////			CAN1_Yun_SendMsg(yunMotorData.yaw_output,yunMotorData.pitch_output);	//CAN2-1000	//取消反馈补偿
//CAN1_Yun_SendMsg(0,0);	//CAN2-1000	//取消反馈补偿
			//			CAN1_Yun_SendMsg(t_yaw_send,t_pitch_send);	//调试用模式
//			CAN_Motor6623_calibration();
			//CAN1_Yun_SendMsg(yunMotorData.yaw_output+Yaw_output_offset(yunMotorData.yaw_fdbP),yunMotorData.pitch_output+Pitch_output_offset(yunMotorData.pitch_tarP));	//CAN2-1000
			CAN2_Chassis_SendMsg(0,0,0,0);
			//CAN2_Shoot_SendMsg(0,0);//下拨弹、上拨弹
			CAN2_Shoot_Bullet_SendMsg(0,0,0,0);//取弹旋转、0、下拨弹、上拨弹
			break;
		}
		case CALI_STATE:	//标定模式
		{
			SetFrictionWheelSpeed(FRICTION_INIT);
			
//			CAN1_Yun_SendMsg(yunMotorData.yaw_output+Yaw_output_offset(yunMotorData.yaw_fdbP),yunMotorData.pitch_output+Pitch_output_offset(yunMotorData.pitch_tarP));	//CAN2-1000	//加入反馈补偿
			CAN1_Yun_SendMsg(0,0);	//CAN2	//yaw,pitch
//////			CAN1_Yun_SendMsg(yunMotorData.yaw_output,yunMotorData.pitch_output);	//CAN2-1000	//取消反馈补偿
//			CAN1_Yun_SendMsg(0,0);
			CAN2_Chassis_SendMsg(0,0,0,0);
//			CAN2_Shoot_SendMsg(0,0);//下拨弹、上拨弹

			CAN2_Shoot_Bullet_SendMsg(0,0,0,0);//取弹旋转、0、下拨弹、上拨弹
			break;
		}
		case NORMAL_STATE:	//正常操作模式
		{
			CAN1_Yun_SendMsg((s16)yunMotorData.yaw_output,(s16)yunMotorData.pitch_output);	//CAN2-1000	//取消反馈补偿	正常程序模式
//		CAN_Yun_SendMsg(0,0);
//			CAN1_Yun_SendMsg(t_yaw_16t,t_pitch_16t);	//调试用模式
			
//		CAN_Chassis_SendMsg((s16)remote_tem,(s16)remote_tem,(s16)remote_tem,(s16)remote_tem);
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
////			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN2_Shoot_Bullet_SendMsg((s16)BulletRotate_Data.output,0,(s16)shoot_Motor_Data_Up.output,(s16)shoot_Motor_Data_Down.output);//取弹旋转、0、上拨弹、下拨弹
			break;
		}
		case WAIST_STATE:
		{
			CAN1_Yun_SendMsg(yunMotorData.yaw_output,yunMotorData.pitch_output);	//CAN2-1000	//取消反馈补偿
//		CAN_Yun_SendMsg(0,0);
//		CAN_Chassis_SendMsg((s16)remote_tem,(s16)remote_tem,(s16)remote_tem,(s16)remote_tem);
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
//			CAN_Chassis_SendMsg(0,0,0,0);
			CAN2_Shoot_Bullet_SendMsg((s16)BulletRotate_Data.output,0,(s16)shoot_Motor_Data_Up.output,(s16)shoot_Motor_Data_Down.output);//取弹旋转、0、上拨弹、下拨弹
			break;
		}
		case ERROR_STATE:	//错误模式
		{
			CAN1_Yun_SendMsg(0,0);	//CAN2	//yaw,pitch
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN2_Shoot_Bullet_SendMsg(0,0,0,0);//取弹旋转、0、下拨弹、上拨弹
			break;
		}
		case STOP_STATE:	//停止状态
		{
			CAN1_Yun_SendMsg(0,0);	//CAN2	//yaw,pitch
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN2_Shoot_Bullet_SendMsg(0,0,0,0);//取弹旋转、0、下拨弹、上拨弹
			break;
		}
		case PROTECT_STATE:	//自我保护模式
		{
			CAN1_Yun_SendMsg(0,0);	//CAN2	//yaw,pitch
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN2_Shoot_Bullet_SendMsg(0,0,0,0);//取弹旋转、0、下拨弹、上拨弹
			break;
		}
		case TAKEBULLET_STATE:
		{
			CAN1_Yun_SendMsg(yunMotorData.yaw_output,yunMotorData.pitch_output);	//CAN2-1000	//取消反馈补偿
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
			CAN2_Shoot_Bullet_SendMsg((s16)BulletRotate_Data.output,0,(s16)shoot_Motor_Data_Up.output,(s16)shoot_Motor_Data_Down.output);//取弹旋转、0、上拨弹、下拨弹
			break;
		}
		default:
		{
			CAN1_Yun_SendMsg(0,0);	//CAN2	//yaw,pitch
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN2_Shoot_Bullet_SendMsg(0,0,0,0);//取弹旋转、0、下拨弹、上拨弹
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



/***********************************************
//#define PITCH 0	//移到上面去了
//#define ROLL 1
//#define YAW 2
float Chassis_GYRO[3]={0};	//pitch roll yaw
*************************************************
功能：经数据融合给出底盘姿态供其他模块调用
数据单位：度 Gyro_Data.angle
云台陀螺仪数据正方向：pitch:下 	roll:左高右低	 yaw：逆时针
电机位置正方向：pitch：下  yaw:逆时针
电机中值：YAW_INIT PITCH_INIT
融合后chassis方向:pitch:上		roll左高右低		yaw:逆时针
**************************************************
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
***********************************************/

void KeyboardRetset(void)	//如果战场发生意外，就进行复位处理
{
	if(KeyBoardData[KEY_CTRL].value==1&&KeyBoardData[KEY_SHIFT].value==1&&KeyBoardData[KEY_X].value==0&&KeyBoardData[KEY_C].value==1&&KeyBoardData[KEY_V].value==1)	//后面的是防止初始化时全部为0
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


