#include "chassis.h"
#include "math.h"

CHASSIS_DATA chassis_Data={0};

PID_GENERAL PID_Chassis_Speed[4]={PID_CHASSIS_SPEED_DEFAULT,PID_CHASSIS_SPEED_DEFAULT,PID_CHASSIS_SPEED_DEFAULT,PID_CHASSIS_SPEED_DEFAULT};
PID_GENERAL PID_Chassis_Follow=PID_CHASSIS_FOLLOW_DEFAULT;

FIRST_ORDER_FILTER Yaw_Follow_Filter=YAW_FOLLOW_FILTER_DEFAULT;

s16 Chassis_Vx=0;
s16 Chassis_Vy=0;
s16 Chassis_Vw=0;


/************************外部数据引用**************************/
extern RC_Ctl_t RC_Ctl;	//遥控数据
extern GYRO_DATA Gyro_Data;
extern YUN_MOTOR_DATA 	yunMotorData;
extern tPowerHeatData 	testPowerHeatData;      //实时功率热量数据
extern u32 time_1ms_count;
extern IslandAttitudeCorrectState_e IslandAttitude_Correct_State;	//登岛姿态自校正
extern ViceControlDataTypeDef ViceControlData;

extern u8 Yun_WorkState_Turn180_statu;	//180旋转到位标志位，放在了上面
extern u8 Replenish_Bullet_Statu;
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
extern u8 IMU_Check_Useless_State;	//陀螺仪失效检测位
/*************************************************************/


/***************底盘相关控制位（其他函数会引用）*****************/
u8 Chassis_Follow_Statu=1;	//底盘跟随标志位，设立此标志位原因是方便的在传感器失效时切换底盘独立状态
float yaw_follow_error=0;	//弧度制必须浮点	//这里在云台那里扭腰部分会用到
/*************************************************************/


#define WAIST_RANGE 780//750
#define K_SPEED 10
s32 t_Vw_PID=0;
s32 yaw_follow_tarP=YAW_INIT_DEFINE;
volatile float yaw_follow_real_error=0;	//用于扭腰及普通状态下的90度

float t_Vy_k=0;
float t_Vx_k=0;
u8 Chassis_Control_RCorPC=RC_CONTROL;


void Remote_Task(void)
{
	if(IMU_Check_Useless_State==0)
	{
		Chassis_Control_External_Solution();	//正常的位置环
	}
	else
	{
		Chassis_Control_Inscribe_Solution();	//当陀螺仪位置反馈崩了，就采用速度环控制的云台方案对应的底盘方案
	}
}



/******************************************************/
void Chassis_Control_External_Solution(void)	//陀螺仪正常的底盘解决方案
{
	static u8 chassis_follow_statu_last=0;	//记录上一次状态、目的是消除按下键盘触发不跟随模式和键盘控制模式有Vw残留的问题
	
	if(GetWorkState()!=ASCEND_STATE&&GetWorkState()!=DESCEND_STATE)	//其他状态下均收起导轮	//需要改进，在切换到手动模式时不收导轮
	{
		ViceControlData.valve[VALVE_ISLAND]=0;
	}
	
	if(GetWorkState()==NORMAL_STATE)	//底盘PID复位
	{
		for(int i=0;i<4;i++)
		{
			PID_Chassis_Speed[i].k_i=CHASSIS_SPEED_PID_I;
			PID_Chassis_Speed[i].i_sum_max=CHASSIS_SPEED_I_MAX;
		}
	}
	
	if(GetWorkState()!=PREPARE_STATE&&GetWorkState()!=CALI_STATE)	//模式切换
	{
		if(RC_Ctl.key.v_h!=0||RC_Ctl.key.v_l!=0)
		{
			Chassis_Control_RCorPC=PC_CONTROL;
		}
		else if(abs(RC_Ctl.rc.ch0-1024)>3||abs(RC_Ctl.rc.ch1-1024)>3||abs(RC_Ctl.rc.ch2-1024)>3)
		{
			Chassis_Control_RCorPC=RC_CONTROL;
		}
	}
	
	if(Chassis_Control_RCorPC==RC_CONTROL)
	{
		RC_Control_Chassis();
	}
	else if(Chassis_Control_RCorPC==PC_CONTROL)
	{
		PC_Control_Chassis(&Chassis_Vx,&Chassis_Vy);
	}
	
	
	
	static u8 turn_flag=0;
	if(GetWorkState()==WAIST_STATE)
	{
		
		s32 yaw_init_def=YAW_INIT_DEFINE;
		switch(turn_flag)
		{
			case 0:
			{
				YAW_INIT=yaw_init_def-WAIST_RANGE;
				if(abs(YAW_INIT-yunMotorData.yaw_fdbP)<55)
				{
					turn_flag=1;
				}
				break;
			}
			case 1:
			{
				YAW_INIT=yaw_init_def+WAIST_RANGE;
				if(abs(YAW_INIT-yunMotorData.yaw_fdbP)<55)
				{
					turn_flag=0;
				}
				break;
			}
		}
		
		PID_Chassis_Follow.k_p=CHASSIS_FOLLOW_PID_P/1;
	}
	else
	{
		YAW_INIT=YAW_INIT_DEFINE;
		PID_Chassis_Follow.k_p=CHASSIS_FOLLOW_PID_P;
	}
	
	
	
//	yaw_follow_tarP=yunMotorData.yaw_fdbP;
	if(yunMotorData.yaw_tarP-Gyro_Data.angle[2]*10>1800)	//过零点
	{
		yaw_follow_tarP=yunMotorData.yaw_fdbP+(yunMotorData.yaw_tarP-(Gyro_Data.angle[2]*10+3600))*8192/3600;
	}
	else if(yunMotorData.yaw_tarP-Gyro_Data.angle[2]*10<-1800)
	{
		yaw_follow_tarP=yunMotorData.yaw_fdbP+(yunMotorData.yaw_tarP-(Gyro_Data.angle[2]*10-3600))*8192/3600;
	}
	else
	{
		yaw_follow_tarP=yunMotorData.yaw_fdbP+(yunMotorData.yaw_tarP-Gyro_Data.angle[2]*10)*8192/3600;
	}
	
	
	
	if((GetWorkState()==NORMAL_STATE||GetWorkState()==WAIST_STATE)&&Replenish_Bullet_Statu==0)	//底盘跟随标志位定义块	//取弹、补弹不受云台控
	{
		Chassis_Follow_Statu=1;
	}
	else if((GetWorkState()==ASCEND_STATE||GetWorkState()==DESCEND_STATE)&&IslandAttitude_Correct_State==CORRECT_CHASSIS_STATE)	//上下岛模式
	{
		Chassis_Follow_Statu=1;
	}
	else if((GetWorkState()==TAKEBULLET_STATE&&Yun_WorkState_Turn180_statu==1)||Replenish_Bullet_Statu==1)	//取弹转向后不受控+补弹位=1时
	{
		Chassis_Follow_Statu=0;
	}
//	else
//	{
//		Chassis_Follow_Statu=0;
//	}
	
	
	if(chassis_follow_statu_last!=Chassis_Follow_Statu)
	{
		Chassis_Vw=0;	//防止Vw残留的问题
		Chassis_Control_RCorPC=RC_CONTROL;
	}
	
	chassis_follow_statu_last=Chassis_Follow_Statu;
	
	
	if(Chassis_Follow_Statu==1)	//底盘跟随部分解算
	{
//////		Chassis_Vx=RC_Ctl.rc.ch1-1024;
		
//		Chassis_Vw=RC_Ctl.rc.ch2-1024;
		if((abs(RC_Ctl.rc.ch2-1024)<40&&abs(YAW_INIT-yunMotorData.yaw_fdbP)<180)||GetWorkState()==TAKEBULLET_STATE)//此处陀螺仪在加速度过大时反馈会有较大误差，因此采用低转向普通跟随，高转向智能跟随模式	//此处应把fdb改为tarP
		{
			if(YAW_INIT-yunMotorData.yaw_fdbP>8192/2)	//普通跟随块	
			{
				Chassis_Vw=PID_ChassisFollow(YAW_INIT,yunMotorData.yaw_fdbP+8192,&PID_Chassis_Follow);	//负过界状况下
				yaw_follow_error=YAW_INIT-(yunMotorData.yaw_fdbP+8192);
			}
			else if(YAW_INIT-yunMotorData.yaw_fdbP<-8192/2)
			{
				Chassis_Vw=PID_ChassisFollow(YAW_INIT,yunMotorData.yaw_fdbP-8192,&PID_Chassis_Follow);	//正过界状况下
				yaw_follow_error=YAW_INIT-(yunMotorData.yaw_fdbP-8192);
			}
			else
			{
				Chassis_Vw=PID_ChassisFollow(YAW_INIT,yunMotorData.yaw_fdbP,&PID_Chassis_Follow);	//正常状况下
				yaw_follow_error=YAW_INIT-yunMotorData.yaw_fdbP;
			}
		}
		else
		{
			if(YAW_INIT-yaw_follow_tarP>8192/2)	//智能跟随块	
			{
				Chassis_Vw=PID_ChassisFollow(YAW_INIT,yaw_follow_tarP+8192,&PID_Chassis_Follow);	//负过界状况下
				yaw_follow_error=YAW_INIT-(yaw_follow_tarP+8192);
			}
			else if(YAW_INIT-yaw_follow_tarP<-8192/2)
			{
				Chassis_Vw=PID_ChassisFollow(YAW_INIT,yaw_follow_tarP-8192,&PID_Chassis_Follow);	//正过界状况下
				yaw_follow_error=YAW_INIT-(yaw_follow_tarP-8192);
			}
			else
			{
				Chassis_Vw=PID_ChassisFollow(YAW_INIT,yaw_follow_tarP,&PID_Chassis_Follow);	//正常状况下
				yaw_follow_error=YAW_INIT-yaw_follow_tarP;
			}
		}
			


		Chassis_Vw=chassis_Vw_filter(Chassis_Vw);	//对速度一阶滤波
		
		
//		Chassis_Vw=(s16)(FirstOrder_General((YAW_INIT-yunMotorData.yaw_fdbP),&Yaw_Follow_Filter)*0.43f);
//		Chassis_Vw=(s16)((YAW_INIT-yunMotorData.yaw_fdbP)*0.6f);	//YUN_INIT为目标位置，故为YAW_INIT-
	}
	
	if(GetWorkState()==NORMAL_STATE||GetWorkState()==TAKEBULLET_STATE)	//取弹为暂时加入
	yaw_follow_error=yaw_follow_error/8192.0f*2*PI;	//每次都运算方便yun.c调用
	if(GetWorkState()==NORMAL_STATE&&abs(YAW_INIT-yunMotorData.yaw_fdbP)>200)	//过弯漂移	//发现过弯飘移会影响转向，解决方法1，转向驱动力达到极限触顶导致实际速度比例偏差预期，设置一函数检测任意输出值大于8000时限制整体使比例预期，方法二，减弱Vy
	{	//智能转向块
		s16 Vx_record=Chassis_Vx;
		float yaw_follow_error_deal=0.0;	//5.21加入的优化	防止转向过大直接停顿
		Chassis_Vx=0;
		
		yaw_follow_error_deal=yaw_follow_error*0.8f;	//5.21加入的优化	防止转向过大直接停顿
//		yaw_follow_error=yaw_follow_error/8192.0f*2*PI;
		Chassis_Vx+=(s16)(Vx_record*(cos(yaw_follow_error_deal)));	//还原只需将yaw_follow_error_deal换成yaw_follow_error
		Chassis_Vy+=(s16)(Vx_record*(sin(yaw_follow_error_deal))*1);
	}
	
	
	if(GetWorkState()==WAIST_STATE)	//扭腰前进	
	{
		s16 Vx_record=Chassis_Vx;
		if(YAW_INIT_DEFINE-yaw_follow_tarP>8192/2)	//防止过界
		{
			yaw_follow_real_error=YAW_INIT_DEFINE-(yaw_follow_tarP+8192);
		}
		else if(YAW_INIT_DEFINE-yaw_follow_tarP<-8192/2)
		{
			yaw_follow_real_error=YAW_INIT_DEFINE-(yaw_follow_tarP-8192);
		}
		else
		{
			yaw_follow_real_error=YAW_INIT_DEFINE-yaw_follow_tarP;
		}

		Chassis_Vx=0;
		yaw_follow_real_error=yaw_follow_real_error/8192.0f*2*PI;
		Chassis_Vx+=(s16)(Vx_record*(cos(yaw_follow_real_error)));
		Chassis_Vy+=(s16)(Vx_record*(sin(yaw_follow_real_error))*1);	
//		t_Vy_k=sin(yaw_follow_real_error);
//		t_Vx_k=cos(yaw_follow_real_error);
	}
	
	
	
	if(GetWorkState()==WAIST_STATE&&KeyBoardData[KEY_CTRL].value==1	)	//云台中心转向	//仅在扭腰时生效
	{	//等到CTRL写好了开启
		float chassis_vw_record=Chassis_Vw;
		Chassis_Vy-=(s16)(chassis_vw_record/1.7f);	//2
		Chassis_Vx+=21;	//对后退的补偿	//每次调节轮压需要重新调节		//思考？加在这里还是上面扭腰前进补偿之前合适
	}

				
	chassis_Data.lf_wheel_tarV=(Chassis_Vx+Chassis_Vy+Chassis_Vw)*K_SPEED;
	chassis_Data.rf_wheel_tarV=(-Chassis_Vx+Chassis_Vy+Chassis_Vw)*K_SPEED;
	chassis_Data.lb_wheel_tarV=(Chassis_Vx-Chassis_Vy+Chassis_Vw)*K_SPEED;
	chassis_Data.rb_wheel_tarV=(-Chassis_Vx-Chassis_Vy+Chassis_Vw)*K_SPEED;
	
	
	Overall_Motion_Ratio_Protect(&chassis_Data);	//整体速度保护
	

	
	chassis_Data.lf_wheel_output=PID_General(chassis_Data.lf_wheel_tarV,chassis_Data.lf_wheel_fdbV,&PID_Chassis_Speed[LF]);
	chassis_Data.rf_wheel_output=PID_General(chassis_Data.rf_wheel_tarV,chassis_Data.rf_wheel_fdbV,&PID_Chassis_Speed[RF]);
	chassis_Data.lb_wheel_output=PID_General(chassis_Data.lb_wheel_tarV,chassis_Data.lb_wheel_fdbV,&PID_Chassis_Speed[LB]);
	chassis_Data.rb_wheel_output=PID_General(chassis_Data.rb_wheel_tarV,chassis_Data.rb_wheel_fdbV,&PID_Chassis_Speed[RB]);

	
	if((GetWorkState()==NORMAL_STATE||GetWorkState()==ASCEND_STATE)&&RC_Ctl.rc.switch_left==RC_SWITCH_UP)	//扩展性整体比例PID补偿块
	{
		Extended_Integral_PID(&chassis_Data);
	}
	
	
	{	//功率限制块
		float limit_k=Limit_Power(testPowerHeatData.chassisPower,testPowerHeatData.chassisPowerBuffer);	//testPowerHeatData.chassisPowerBuffer
		float output_limit_lf=chassis_Data.lf_wheel_output*limit_k;
		float output_limit_rf=chassis_Data.rf_wheel_output*limit_k;
		float output_limit_lb=chassis_Data.lb_wheel_output*limit_k;
		float output_limit_rb=chassis_Data.rb_wheel_output*limit_k;
		chassis_Data.lf_wheel_output=(s32)output_limit_lf;
		chassis_Data.rf_wheel_output=(s32)output_limit_rf;
		chassis_Data.lb_wheel_output=(s32)output_limit_lb;
		chassis_Data.rb_wheel_output=(s32)output_limit_rb;
	}
}
/******************************************************/



/******************************************************/	//陀螺仪失效时的底盘跟随等解决方案
void Chassis_Control_Inscribe_Solution(void)	//陀螺仪正常的底盘解决方案	/////////////////////////////////////////仅跟随，扭腰可能不同
{
	static u8 chassis_follow_statu_last=0;	//记录上一次状态、目的是消除按下键盘触发不跟随模式和键盘控制模式有Vw残留的问题
	
	if(GetWorkState()!=ASCEND_STATE&&GetWorkState()!=DESCEND_STATE)	//其他状态下均收起导轮	//需要改进，在切换到手动模式时不收导轮
	{
		ViceControlData.valve[VALVE_ISLAND]=0;
	}
	
	if(GetWorkState()==NORMAL_STATE)	//底盘PID复位
	{
		for(int i=0;i<4;i++)
		{
			PID_Chassis_Speed[i].k_i=CHASSIS_SPEED_PID_I;
			PID_Chassis_Speed[i].i_sum_max=CHASSIS_SPEED_I_MAX;
		}
	}
	
	if(GetWorkState()!=PREPARE_STATE&&GetWorkState()!=CALI_STATE)	//模式切换
	{
		if(RC_Ctl.key.v_h!=0||RC_Ctl.key.v_l!=0)
		{
			Chassis_Control_RCorPC=PC_CONTROL;
		}
		else if(abs(RC_Ctl.rc.ch0-1024)>3||abs(RC_Ctl.rc.ch1-1024)>3||abs(RC_Ctl.rc.ch2-1024)>3)
		{
			Chassis_Control_RCorPC=RC_CONTROL;
		}
	}
	
	if(Chassis_Control_RCorPC==RC_CONTROL)
	{
		RC_Control_Chassis();
	}
	else if(Chassis_Control_RCorPC==PC_CONTROL)
	{
		PC_Control_Chassis(&Chassis_Vx,&Chassis_Vy);
	}
	
	
	
	static u8 turn_flag=0;
	if(GetWorkState()==WAIST_STATE)
	{
		
		s32 yaw_init_def=YAW_INIT_DEFINE;
		switch(turn_flag)
		{
			case 0:
			{
				YAW_INIT=yaw_init_def-WAIST_RANGE;
				if(abs(YAW_INIT-yunMotorData.yaw_fdbP)<55)
				{
					turn_flag=1;
				}
				break;
			}
			case 1:
			{
				YAW_INIT=yaw_init_def+WAIST_RANGE;
				if(abs(YAW_INIT-yunMotorData.yaw_fdbP)<55)
				{
					turn_flag=0;
				}
				break;
			}
		}
		
		PID_Chassis_Follow.k_p=CHASSIS_FOLLOW_PID_P/1;
	}
	else
	{
		YAW_INIT=YAW_INIT_DEFINE;
		PID_Chassis_Follow.k_p=CHASSIS_FOLLOW_PID_P;
	}
	
	
	
//////////////	yaw_follow_tarP=yunMotorData.yaw_fdbP;
////////////	if(yunMotorData.yaw_tarP-Gyro_Data.angle[2]*10>1800)	//过零点
////////////	{
////////////		yaw_follow_tarP=yunMotorData.yaw_fdbP+(yunMotorData.yaw_tarP-(Gyro_Data.angle[2]*10+3600))*8192/3600;
////////////	}
////////////	else if(yunMotorData.yaw_tarP-Gyro_Data.angle[2]*10<-1800)
////////////	{
////////////		yaw_follow_tarP=yunMotorData.yaw_fdbP+(yunMotorData.yaw_tarP-(Gyro_Data.angle[2]*10-3600))*8192/3600;
////////////	}
////////////	else
////////////	{
////////////		yaw_follow_tarP=yunMotorData.yaw_fdbP+(yunMotorData.yaw_tarP-Gyro_Data.angle[2]*10)*8192/3600;
////////////	}
	
	
	
	if((GetWorkState()==NORMAL_STATE||GetWorkState()==WAIST_STATE)&&Replenish_Bullet_Statu==0)	//底盘跟随标志位定义块	//取弹、补弹不受云台控
	{
		Chassis_Follow_Statu=1;
	}
	else if((GetWorkState()==ASCEND_STATE||GetWorkState()==DESCEND_STATE)&&IslandAttitude_Correct_State==CORRECT_CHASSIS_STATE)	//上下岛模式
	{
		Chassis_Follow_Statu=1;
	}
	else if(GetWorkState()==TAKEBULLET_STATE||Replenish_Bullet_Statu==1)	//当陀螺仪失效，所有模式陀螺仪都受控
	{
		Chassis_Follow_Statu=1;
	}
//	else
//	{
//		Chassis_Follow_Statu=0;
//	}
	
	
	if(chassis_follow_statu_last!=Chassis_Follow_Statu)
	{
		Chassis_Vw=0;	//防止Vw残留的问题
		Chassis_Control_RCorPC=RC_CONTROL;
	}
	
	chassis_follow_statu_last=Chassis_Follow_Statu;
	
	
	if(Chassis_Follow_Statu==1)	//底盘跟随部分解算
	{
//////		Chassis_Vx=RC_Ctl.rc.ch1-1024;
		
//		Chassis_Vw=RC_Ctl.rc.ch2-1024;
		if(1)//此处陀螺仪在加速度过大时反馈会有较大误差，因此采用低转向普通跟随，高转向智能跟随模式	//此处应把fdb改为tarP
		{
			if(YAW_INIT-yunMotorData.yaw_fdbP>8192/2)	//普通跟随块	
			{
				Chassis_Vw=PID_ChassisFollow(YAW_INIT,yunMotorData.yaw_fdbP+8192,&PID_Chassis_Follow);	//负过界状况下
				yaw_follow_error=YAW_INIT-(yunMotorData.yaw_fdbP+8192);
			}
			else if(YAW_INIT-yunMotorData.yaw_fdbP<-8192/2)
			{
				Chassis_Vw=PID_ChassisFollow(YAW_INIT,yunMotorData.yaw_fdbP-8192,&PID_Chassis_Follow);	//正过界状况下
				yaw_follow_error=YAW_INIT-(yunMotorData.yaw_fdbP-8192);
			}
			else
			{
				Chassis_Vw=PID_ChassisFollow(YAW_INIT,yunMotorData.yaw_fdbP,&PID_Chassis_Follow);	//正常状况下
				yaw_follow_error=YAW_INIT-yunMotorData.yaw_fdbP;
			}
		}
//////////		else	//含有4-5个注释的意思是和标准反馈方案的区别
//////////		{
//////////			if(YAW_INIT-yaw_follow_tarP>8192/2)	//智能跟随块	
//////////			{
//////////				Chassis_Vw=PID_ChassisFollow(YAW_INIT,yaw_follow_tarP+8192,&PID_Chassis_Follow);	//负过界状况下
//////////				yaw_follow_error=YAW_INIT-(yaw_follow_tarP+8192);
//////////			}
//////////			else if(YAW_INIT-yaw_follow_tarP<-8192/2)
//////////			{
//////////				Chassis_Vw=PID_ChassisFollow(YAW_INIT,yaw_follow_tarP-8192,&PID_Chassis_Follow);	//正过界状况下
//////////				yaw_follow_error=YAW_INIT-(yaw_follow_tarP-8192);
//////////			}
//////////			else
//////////			{
//////////				Chassis_Vw=PID_ChassisFollow(YAW_INIT,yaw_follow_tarP,&PID_Chassis_Follow);	//正常状况下
//////////				yaw_follow_error=YAW_INIT-yaw_follow_tarP;
//////////			}
//////////		}
			


		Chassis_Vw=chassis_Vw_filter(Chassis_Vw);	//对速度一阶滤波
		
		
//		Chassis_Vw=(s16)(FirstOrder_General((YAW_INIT-yunMotorData.yaw_fdbP),&Yaw_Follow_Filter)*0.43f);
//		Chassis_Vw=(s16)((YAW_INIT-yunMotorData.yaw_fdbP)*0.6f);	//YUN_INIT为目标位置，故为YAW_INIT-
	}
	
	if(GetWorkState()==NORMAL_STATE||GetWorkState()==TAKEBULLET_STATE)	//取弹为暂时加入
	yaw_follow_error=yaw_follow_error/8192.0f*2*PI;	//每次都运算方便yun.c调用
	if(GetWorkState()==NORMAL_STATE&&abs(YAW_INIT-yunMotorData.yaw_fdbP)>200)	//过弯漂移	//发现过弯飘移会影响转向，解决方法1，转向驱动力达到极限触顶导致实际速度比例偏差预期，设置一函数检测任意输出值大于8000时限制整体使比例预期，方法二，减弱Vy
	{	//智能转向块
		s16 Vx_record=Chassis_Vx;
		float yaw_follow_error_deal=0.0;	//5.21加入的优化	防止转向过大直接停顿
		Chassis_Vx=0;
		
		yaw_follow_error_deal=yaw_follow_error*0.8f;	//5.21加入的优化	防止转向过大直接停顿
//		yaw_follow_error=yaw_follow_error/8192.0f*2*PI;
		Chassis_Vx+=(s16)(Vx_record*(cos(yaw_follow_error_deal)));	//还原只需将yaw_follow_error_deal换成yaw_follow_error
		Chassis_Vy+=(s16)(Vx_record*(sin(yaw_follow_error_deal))*1);
	}
	
	
	if(GetWorkState()==WAIST_STATE)	//扭腰前进	//////////////////////////////////////////////////////////////////////////////////////？？？？？？？？？？？？OK？？？
	{
		s16 Vx_record=Chassis_Vx;
		if(YAW_INIT_DEFINE-yunMotorData.yaw_fdbP>8192/2)	//防止过界
		{
			yaw_follow_real_error=YAW_INIT_DEFINE-(yunMotorData.yaw_fdbP+8192);
		}
		else if(YAW_INIT_DEFINE-yunMotorData.yaw_fdbP<-8192/2)
		{
			yaw_follow_real_error=YAW_INIT_DEFINE-(yunMotorData.yaw_fdbP-8192);
		}
		else
		{
			yaw_follow_real_error=YAW_INIT_DEFINE-yunMotorData.yaw_fdbP;
		}

		Chassis_Vx=0;
		yaw_follow_real_error=yaw_follow_real_error/8192.0f*2*PI;
		Chassis_Vx+=(s16)(Vx_record*(cos(yaw_follow_real_error)));
		Chassis_Vy+=(s16)(Vx_record*(sin(yaw_follow_real_error))*1);	
//		t_Vy_k=sin(yaw_follow_real_error);
//		t_Vx_k=cos(yaw_follow_real_error);
	}
	
	
	
	if(GetWorkState()==WAIST_STATE&&KeyBoardData[KEY_CTRL].value==1	)	//云台中心转向	//仅在扭腰时生效
	{	//等到CTRL写好了开启
		float chassis_vw_record=Chassis_Vw;
		Chassis_Vy-=(s16)(chassis_vw_record/1.7f);	//2
		Chassis_Vx+=21;	//对后退的补偿	//每次调节轮压需要重新调节		//思考？加在这里还是上面扭腰前进补偿之前合适
	}

				
	chassis_Data.lf_wheel_tarV=(Chassis_Vx+Chassis_Vy+Chassis_Vw)*K_SPEED;
	chassis_Data.rf_wheel_tarV=(-Chassis_Vx+Chassis_Vy+Chassis_Vw)*K_SPEED;
	chassis_Data.lb_wheel_tarV=(Chassis_Vx-Chassis_Vy+Chassis_Vw)*K_SPEED;
	chassis_Data.rb_wheel_tarV=(-Chassis_Vx-Chassis_Vy+Chassis_Vw)*K_SPEED;
	
	
	Overall_Motion_Ratio_Protect(&chassis_Data);	//整体速度保护
	

	
	chassis_Data.lf_wheel_output=PID_General(chassis_Data.lf_wheel_tarV,chassis_Data.lf_wheel_fdbV,&PID_Chassis_Speed[LF]);
	chassis_Data.rf_wheel_output=PID_General(chassis_Data.rf_wheel_tarV,chassis_Data.rf_wheel_fdbV,&PID_Chassis_Speed[RF]);
	chassis_Data.lb_wheel_output=PID_General(chassis_Data.lb_wheel_tarV,chassis_Data.lb_wheel_fdbV,&PID_Chassis_Speed[LB]);
	chassis_Data.rb_wheel_output=PID_General(chassis_Data.rb_wheel_tarV,chassis_Data.rb_wheel_fdbV,&PID_Chassis_Speed[RB]);

	
	if((GetWorkState()==NORMAL_STATE||GetWorkState()==ASCEND_STATE)&&RC_Ctl.rc.switch_left==RC_SWITCH_UP)	//扩展性整体比例PID补偿块
	{
		Extended_Integral_PID(&chassis_Data);
	}
	
	
	{	//功率限制块
		float limit_k=Limit_Power(testPowerHeatData.chassisPower,testPowerHeatData.chassisPowerBuffer);	//testPowerHeatData.chassisPowerBuffer
		float output_limit_lf=chassis_Data.lf_wheel_output*limit_k;
		float output_limit_rf=chassis_Data.rf_wheel_output*limit_k;
		float output_limit_lb=chassis_Data.lb_wheel_output*limit_k;
		float output_limit_rb=chassis_Data.rb_wheel_output*limit_k;
		chassis_Data.lf_wheel_output=(s32)output_limit_lf;
		chassis_Data.rf_wheel_output=(s32)output_limit_rf;
		chassis_Data.lb_wheel_output=(s32)output_limit_lb;
		chassis_Data.rb_wheel_output=(s32)output_limit_rb;
	}
}




/******************************************************/



void RC_Control_Chassis(void)
{
	static s16 Chassis_Vx_last=0;
	static s16 Chassis_Vy_last=0;
	if((GetWorkState()==NORMAL_STATE||GetWorkState()==WAIST_STATE)&&Replenish_Bullet_Statu==0)	//如果在普通模式且补弹=0
	{
		if(time_1ms_count%1==0)
		{
			if(RC_Ctl.rc.ch1-1024-Chassis_Vx_last>1&&RC_Ctl.rc.ch1-1024>10)	//只在前进加速时生效，
			{
				Chassis_Vx+=1;
			}
			else if(RC_Ctl.rc.ch1-1024-Chassis_Vx_last<-1&&RC_Ctl.rc.ch1-1024<-10)	//只在后退加速时生效
			{
				Chassis_Vx-=1;
			}
			else
			{
				Chassis_Vx=RC_Ctl.rc.ch1-1024;
			}
		}
//		Chassis_Vx=RC_Ctl.rc.ch1-1024;	//代替为斜坡函数
		Chassis_Vx_last=Chassis_Vx;
	}
	else if(GetWorkState()==TAKEBULLET_STATE||Replenish_Bullet_Statu==1)	//取弹时反向，补弹时反向
	{
		if(time_1ms_count%1==0)
		{
			Chassis_Vx=-(RC_Ctl.rc.ch1-1024);
		}
//		Chassis_Vx=RC_Ctl.rc.ch1-1024;	//代替为斜坡函数
		Chassis_Vx_last=Chassis_Vx;
	}
	
	if(time_1ms_count%1==0)
	{
		if(GetWorkState()!=TAKEBULLET_STATE&&Replenish_Bullet_Statu==0)	//取弹时反向
		{
			if(RC_Ctl.rc.ch0-1024-Chassis_Vy_last>1&&RC_Ctl.rc.ch0-1024>10)
			{
				Chassis_Vy+=1;
			}
			else if(RC_Ctl.rc.ch0-1024-Chassis_Vy_last<-1&&RC_Ctl.rc.ch0-1024<-10)	//刹车按不缓冲
			{
				Chassis_Vy-=1;
			}
			else
			{
				Chassis_Vy=RC_Ctl.rc.ch0-1024;
			}
		}
		else	//取弹反向
		{
			Chassis_Vy=-(RC_Ctl.rc.ch0-1024);
		}

	}
	Chassis_Vy_last=Chassis_Vy;
	
	if((GetWorkState()==TAKEBULLET_STATE||Replenish_Bullet_Statu==1)&&IMU_Check_Useless_State==0)	//当陀螺仪正常时取弹模式不受云台控，当陀螺仪失效，受控
	{
		Chassis_Vw=(RC_Ctl.rc.ch2-1024)*300/660;
	}
//	Chassis_Vy=RC_Ctl.rc.ch0-1024;
}

extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
void PC_Control_Chassis(s16 * chassis_vx,s16 * chassis_vy)	//1000Hz
{
	static s16 chassis_vx_record=0;
	static s16 chassis_vy_record=0;
	if(GetWorkState()==NORMAL_STATE||GetWorkState()==WAIST_STATE)
	{
		if(time_1ms_count%2==0)
		{
			if(KeyBoardData[KEY_W].value!=0)
			{
				if(chassis_vx_record<660&&chassis_vx_record>=0)
				{
					chassis_vx_record++;
				}
				else if(chassis_vx_record<0)
				{
					chassis_vx_record=0;
				}
			}
			else if(KeyBoardData[KEY_S].value!=0)
			{
				if(chassis_vx_record>-660&&chassis_vx_record<=0)
				{
					chassis_vx_record--;
				}
				else if(chassis_vx_record>0)
				{
					chassis_vx_record=0;
				}
			}
			else
			{
				chassis_vx_record=0;
			}
			///////////////////////////////////////
			if(KeyBoardData[KEY_D].value!=0)
			{
				if(chassis_vy_record<660&&chassis_vy_record>=0)
				{
					chassis_vy_record++;
				}
				else if(*chassis_vy<0)
				{
					chassis_vy_record=0;
				}
			}
			else if(KeyBoardData[KEY_A].value!=0)
			{
				if(chassis_vy_record>-660&&chassis_vy_record<=0)
				{
					chassis_vy_record--;
				}
				else if(chassis_vy_record>0)
				{
					chassis_vy_record=0;
				}
			}
			else
			{
				chassis_vy_record=0;
			}
			*chassis_vx=chassis_vx_record;
			*chassis_vy=chassis_vy_record;
		}

	}
		
}


s16 chassis_Vw_filter(s16 now_V)
{
	static s32 Last_V=0;
	static s32 now_acc_V=0;
	static float a=0.25f;
	now_acc_V=(s32)(Last_V*(1-a)+now_V*a);
	Last_V=now_acc_V;
	return now_acc_V;
}

extern Error_check_t Error_Check;
#define POWERLIMIT 120 	//120w功率限制
#define POWERBUFFER 60	//60J功率缓冲
float Limit_Power(float power,float powerbuffer)	//英雄120J热量限制，直接限制总输出
{
	float limit_k=1;
//	if(power>POWERLIMIT*0.6)
//	{
		limit_k=3.0f*powerbuffer/200.0f+0.1f;	//0.4
		limit_k=limit_k>1?1:limit_k;
		limit_k=limit_k<0.1f?0.1f:limit_k;
//	}
//	limit_k=0.9;	//取消功率限制，屏蔽后取消
	
	if(Error_Check.statu[LOST_REFEREE]==1)	//裁判lost
	{
		limit_k=0.8;
	}
	
	return limit_k;
}


#define CHASSIS_SPEEDMAX 8500.0f
void Overall_Motion_Ratio_Protect(CHASSIS_DATA* chassis_data)	//整体轮速比例保护,一定要放在整体轮速解算出来后
{
	s32 chassis_tarV_max=0;
	float chassis_protect_k=1;
	
	chassis_tarV_max=chassis_data->lf_wheel_tarV>chassis_data->rf_wheel_tarV?chassis_data->lf_wheel_tarV:chassis_data->rf_wheel_tarV;
	chassis_tarV_max=chassis_tarV_max>chassis_data->lb_wheel_tarV?chassis_tarV_max:chassis_data->lb_wheel_tarV;
	chassis_tarV_max=chassis_tarV_max>chassis_data->rb_wheel_tarV?chassis_tarV_max:chassis_data->rb_wheel_tarV;
	
	if(chassis_tarV_max>CHASSIS_SPEEDMAX)	//计算出保护系数为了减少计算量将缩小计算放在IF内
	{
		chassis_protect_k=CHASSIS_SPEEDMAX/chassis_tarV_max;
		
		chassis_data->lf_wheel_tarV*=chassis_protect_k;
		chassis_data->rf_wheel_tarV*=chassis_protect_k;
		chassis_data->lb_wheel_tarV*=chassis_protect_k;
		chassis_data->rb_wheel_tarV*=chassis_protect_k;
	}
	
}



#define CHASSIS_INTEGRAL_PID_KP 3
#define CHASSIS_INTEGRAL_PID_KI 0.01f
#define CHASSIS_INTEGRAL_PID_I_SUM_LIM 1000
void Extended_Integral_PID(CHASSIS_DATA* chassis_data)	//扩展型整体PID，适用于任意动作场景	2018.4.19
{
	float tarv_sum=abs(chassis_data->lf_wheel_tarV)+abs(chassis_data->rf_wheel_tarV)+abs(chassis_data->lb_wheel_tarV)+abs(chassis_data->rb_wheel_tarV);
	float fdbv_sum=abs(chassis_data->lf_wheel_fdbV)+abs(chassis_data->rf_wheel_fdbV)+abs(chassis_data->lb_wheel_fdbV)+abs(chassis_data->rb_wheel_fdbV);
	float expect[4]={0};
	float error[4]={0};
	static float inte[4];
	s32 output_compensation[4];
	
	if(abs(tarv_sum)<0.1f)	//相当于被除数为0
	{
		expect[LF]=0;
		expect[RF]=0;
		expect[LB]=0;
		expect[RB]=0;
	}
	else
	{
		expect[LF]=fdbv_sum*chassis_data->lf_wheel_tarV/tarv_sum;
		expect[RF]=fdbv_sum*chassis_data->rf_wheel_tarV/tarv_sum;
		expect[LB]=fdbv_sum*chassis_data->lb_wheel_tarV/tarv_sum;
		expect[RB]=fdbv_sum*chassis_data->rb_wheel_tarV/tarv_sum;
	}
	
	error[LF]=expect[LF]-chassis_data->lf_wheel_fdbV;
	error[RF]=expect[RF]-chassis_data->rf_wheel_fdbV;
	error[LB]=expect[LB]-chassis_data->lb_wheel_fdbV;
	error[RB]=expect[RB]-chassis_data->rb_wheel_fdbV;
	
	inte[LF]+=error[LF]*CHASSIS_INTEGRAL_PID_KI;
	inte[LF]+=error[LF]*CHASSIS_INTEGRAL_PID_KI;
	inte[LF]+=error[LF]*CHASSIS_INTEGRAL_PID_KI;
	inte[LF]+=error[LF]*CHASSIS_INTEGRAL_PID_KI;
	
	for(int id=0;id<4;id++)
	{
		inte[id]=inte[id]>CHASSIS_INTEGRAL_PID_I_SUM_LIM?CHASSIS_INTEGRAL_PID_I_SUM_LIM:inte[id];
		inte[id]=inte[id]<-CHASSIS_INTEGRAL_PID_I_SUM_LIM?-CHASSIS_INTEGRAL_PID_I_SUM_LIM:inte[id];
	}
	
	output_compensation[LF]=(s32)(error[LF]*CHASSIS_INTEGRAL_PID_KP+inte[LF]);
	output_compensation[RF]=(s32)(error[RF]*CHASSIS_INTEGRAL_PID_KP+inte[RF]);
	output_compensation[LB]=(s32)(error[LB]*CHASSIS_INTEGRAL_PID_KP+inte[LB]);
	output_compensation[RB]=(s32)(error[RB]*CHASSIS_INTEGRAL_PID_KP+inte[RB]);
	
	for(int id=0;id<4;id++)
	{
		output_compensation[id]=output_compensation[id]>4000?4000:output_compensation[id];
		output_compensation[id]=output_compensation[id]<-4000?-4000:output_compensation[id];
	}
	
	chassis_data->lf_wheel_output+=output_compensation[LF];
	chassis_data->rf_wheel_output+=output_compensation[RF];
	chassis_data->lb_wheel_output+=output_compensation[LB];
	chassis_data->rb_wheel_output+=output_compensation[RB];
}

