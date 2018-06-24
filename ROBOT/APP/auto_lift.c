#include "auto_lift.h"
#include "common_definition.h"

extern LIFT_DATA lift_Data;

extern u8 valve_fdbstate[6];	//假设的电磁阀反馈
extern u8 Steer_Image_state;

extern ViceControlDataTypeDef ViceControlData;
AscendState_e AscendState=FULLRISE_GO1;	//登岛状态位

extern PID_GENERAL PID_Chassis_Speed[4];
//rise fall 
//
void Ascend_Control_Center(void)	//全自动登岛控制中心
{
	ViceControlData.image_cut[0]=1;	//高电平
	Steer_Image_state=1;
	for(int i=0;i<4;i++)
	{
		PID_Chassis_Speed[i].k_i=CHASSIS_SPEED_PID_I*3;
		PID_Chassis_Speed[i].i_sum_max=CHASSIS_SPEED_I_MAX*1.5f;
	}
	ViceControlData.valve[VALVE_ISLAND]=1;	//后面高优先级会覆盖
	switch(AscendState)
	{
		case FULLRISE_GO1:
		{
			if(Ascend_FullRise_GO1()==1)
				AscendState=BACKFALL_GO1;
			break;
		}
		case BACKFALL_GO1:
		{
			if(Ascend_BackFall_GO()==1)
				AscendState=FULLFALL_GO1;
			break;
		}
		case FULLFALL_GO1:
		{
			if(Ascend_FullFall_GO()==1)
				AscendState=FULLRISE_GO2;
			break;
		}
		case FULLRISE_GO2:
		{
			if(Ascend_FullRise_GO2()==1)
				AscendState=BACKFALL_GO2;
			break;
		}
		case BACKFALL_GO2:
		{
			if(Ascend_BackFall_GO()==1)
				AscendState=FULLFALL_GO2;
			break;
		}
		case FULLFALL_GO2:
		{
			if(Ascend_FullFall_GO()==1)
			{
				ViceControlData.image_cut[0]=0;	//低电平
			SetWorkState(NORMAL_STATE);
			AscendState=FULLRISE_GO1;	//重置防止下一次异常
			ViceControlData.valve[VALVE_ISLAND]=0;
			for(int i=0;i<4;i++)
			{
				PID_Chassis_Speed[i].k_i=CHASSIS_SPEED_PID_I;
				PID_Chassis_Speed[i].i_sum_max=CHASSIS_SPEED_I_MAX;
			}
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//姿态矫正函数保持自校准状态
			}
		//		AscendState=FULLFALL_GO1;
			break;
		}
	}
	
	
}


extern s16 Chassis_Vx;
extern s16 Chassis_Vy;
extern s16 Chassis_Vw;

extern SensorDataTypeDef SensorData;

extern u32 time_1ms_count;

#define VW_REDRESS 30
#define VX_SPEED_UP 110
#define VX_SPEED_NEAR 10
#define VX_PRESS_V0 40
#define VX_SPEED_RUSH 160	//冲台阶
#define VX_SPEED_WAITUP 70	//没有完全升起来时速度
#define VX_SPEED_WAIT_FRONTFALL	90	//后腿已经搭上后，前进撞前腿的过程

//红外开关亮为低（即近），即0
//限位
	//以下都是以车身前进方向为前方
u8 Ascend_FullRise_GO1(void)	//前进、调整、触发蹬腿函数
{
	Set_Attitude_Correct_State(CORRECT_CHASSIS_STATE);	//设置默认底盘矫正开启，后续IF语句执行将会覆盖
	
	SetCheck_FrontLift(1);
	SetCheck_BackLift(1);
	Chassis_Vw=0;
		if(SetCheck_FrontLift(1)!=1||SetCheck_BackLift(1)!=1)
		{
			Chassis_Vx=-VX_SPEED_WAITUP;	//没有完全升起来时速度
			Chassis_Vw=0;
		}
		else
		{
			Chassis_Vx=-VX_SPEED_UP;
			Chassis_Vw=0;
		}
		
		
		if(SetCheck_FrontLift(1)==1&&SensorData.Infrare[2]==0&&SensorData.Infrare[3]==0)	//进入到了
		{
			Chassis_Vx=-VX_SPEED_NEAR;
			Chassis_Vw=0;
		}
		else if(SetCheck_FrontLift(1)==1&&SensorData.Infrare[2]==1&&SensorData.Infrare[3]==0)	//数字小的在右边，1代表未触发
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);
			Chassis_Vx=-VX_SPEED_NEAR;
			Chassis_Vw=-VW_REDRESS;	//矫正 逆时针转动
		}
		else if(SetCheck_FrontLift(1)==1&&SensorData.Infrare[2]==0&&SensorData.Infrare[3]==1)	//近距离为0
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);
			Chassis_Vx=-VX_SPEED_NEAR;
			Chassis_Vw=VW_REDRESS;	//矫正	顺时针转动
		}
		
		
		if(SensorData.Limit[2]==1&&SensorData.Limit[3]==1)
		{
			Chassis_Vx=-VX_PRESS_V0;
			Chassis_Vw=0;
				///////////////////////设置到下一个状态
			return 1;
		}
		else if(SensorData.Limit[2]==0&&SensorData.Limit[3]==1)	//0代表未触发,
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);
			Chassis_Vx=-VX_PRESS_V0;
			Chassis_Vw=-VW_REDRESS;	//矫正 逆时针转动
		}
		else if(SensorData.Limit[2]==1&&SensorData.Limit[3]==0)
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//姿态矫正函数进行自校准
			Chassis_Vx=-VX_PRESS_V0;
			Chassis_Vw=VW_REDRESS;	//矫正 逆时针转动
		}
		
		return 0;
}



u8 Ascend_BackFall_GO(void)	//3，4号升降抬起前进的进程
{
	static u32 time_record=0;
	Set_Attitude_Correct_State(CORRECT_CHASSIS_STATE);	//设置默认底盘矫正开启，后续IF语句执行将会覆盖
	SetCheck_FrontLift(1);
	SetCheck_BackLift(0);
	Chassis_Vw=0;
	if(SetCheck_FrontLift(1)==1&&SetCheck_BackLift(0)==1)	//完成抬腿动作后
	{
		if(time_record==0)
		{
			time_record=time_1ms_count;
		}
		
		if(time_1ms_count-time_record<400)	//这里是向前冲以便于后腿冲上台阶
		{
			Chassis_Vx=-VX_SPEED_RUSH;
			Chassis_Vw=0;
		}
		else if(time_1ms_count-time_record>400&&time_record!=0)
		{
			Chassis_Vx=-VX_SPEED_WAIT_FRONTFALL;	//这个值在检测到后会被if中的值覆盖	//这里是撞前腿（登岛的后腿）的速度
			Chassis_Vw=0;
		}
		

		if(SensorData.Infrare[0]==0&&SensorData.Infrare[1]==0)
		{
			Chassis_Vx=-VX_SPEED_NEAR;
			Chassis_Vw=0;
		}
		else if(SensorData.Infrare[0]==1&&SensorData.Infrare[1]==0)	//数字小的在右边，1代表未触发
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//姿态矫正函数进行自校准
			Chassis_Vx=-VX_SPEED_NEAR;
			Chassis_Vw=-VW_REDRESS;	//矫正 逆时针转动
		}
		else if(SensorData.Infrare[0]==0&&SensorData.Infrare[1]==1)
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//姿态矫正函数进行自校准
			Chassis_Vx=-VX_SPEED_NEAR;
			Chassis_Vw=VW_REDRESS;	//矫正 顺时针转动
		}
		
		
		if(SensorData.Limit[0]==1&&SensorData.Limit[1]==1)
		{
			Chassis_Vx=-VX_PRESS_V0;
			Chassis_Vw=0;
			time_record=0;	//重置计时
			return 1;
			////////////////////////////////切换到下一个状态
			
		}
		else if(SensorData.Limit[0]==0&&SensorData.Limit[1]==1)
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//姿态矫正函数进行自校准
			Chassis_Vx=-VX_PRESS_V0;
			Chassis_Vw=-VW_REDRESS;	//矫正 逆时针转动
		}
		else if(SensorData.Limit[0]==1&&SensorData.Limit[1]==0)
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//姿态矫正函数进行自校准
			Chassis_Vx=-VX_PRESS_V0;
			Chassis_Vw=VW_REDRESS;	//矫正 顺时针转动
		}
			
	}
	return 0;
}



u8 Ascend_FullFall_GO(void)	//都抬起到都抬起后	//通过观察视频得知在碰撞一瞬间弹后，轮子瞬间升起，没有缓冲效果，故延时
{	//该阶段问题：如果左右偏移严重，会有影响
	static u32 time_record=0;	//这里可以用位置环来代替
	static u32 time_record_all=0;
	
	if(time_record_all==0)
	{
		time_record_all=time_1ms_count;
	}
	if(time_1ms_count-time_record_all>450)	//延时0.6s
	{
			Set_Attitude_Correct_State(CORRECT_CHASSIS_STATE);	//设置默认底盘矫正开启，后续IF语句执行将会覆盖
			SetCheck_FrontLift(0);
			SetCheck_BackLift(0);
			Chassis_Vw=0;
			
			if(SetCheck_FrontLift(0)==1&&SetCheck_BackLift(0)==1)	//轮已收起
			{
				if(time_record==0)	//time_record=0意思即为第一次执行
				{
					time_record=time_1ms_count;
					Chassis_Vx=-(VX_SPEED_RUSH-50);
					Chassis_Vw=0;
				}
				
				if(time_1ms_count-time_record>630)	//0.6s
				{
					Chassis_Vx=-VX_PRESS_V0;
					Chassis_Vw=0;
					/////////////////////////////设置到下一个状态
					time_record=0;
					time_record_all=0;
					return 1;
					
				}
			}
			else	//轮未收起过程中，实际测试会有偏移，故加上传感矫正
			{
				if(SensorData.Infrare[0]==0&&SensorData.Infrare[1]==0)
				{
					Chassis_Vx=-VX_PRESS_V0;
					Chassis_Vw=0;
				}
				else if(SensorData.Infrare[0]==1&&SensorData.Infrare[1]==0)	//数字小的在右边，1代表未触发
				{
		//			Set_Attitude_Correct_State(CALI_SELF_STATE);	//姿态矫正函数进行自校准
					Chassis_Vx=-VX_PRESS_V0;
					Chassis_Vw=-VW_REDRESS;	//矫正 逆时针转动
				}
				else if(SensorData.Infrare[0]==0&&SensorData.Infrare[1]==1)
				{
		//			Set_Attitude_Correct_State(CALI_SELF_STATE);	//姿态矫正函数进行自校准
					Chassis_Vx=-VX_PRESS_V0;
					Chassis_Vw=VW_REDRESS;	//矫正 顺时针转动
				}
			}
	}
	return 0;
}

u8 Ascend_FullRise_GO2(void)	//前进、调整、触发蹬腿函数
{ 
	Set_Attitude_Correct_State(CORRECT_CHASSIS_STATE);	//设置默认底盘矫正开启，后续IF语句执行将会覆盖
	SetCheck_FrontLift(1);
	SetCheck_BackLift(1);
	Chassis_Vw=0;
	if(SetCheck_FrontLift(1)!=1||SetCheck_BackLift(1)!=1)
		{
			Chassis_Vx=-VX_SPEED_WAITUP;	//没有完全升起来时
			Chassis_Vw=0;
		}
		else
		{
			Chassis_Vx=-VX_SPEED_UP;
			Chassis_Vw=0;
		}
		
		
		if(SetCheck_FrontLift(1)==1&&SensorData.Infrare[2]==0&&SensorData.Infrare[3]==0)	//进入到了
		{
			Chassis_Vx=-VX_SPEED_NEAR;
			Chassis_Vw=0;
		}
		else if(SetCheck_FrontLift(1)==1&&SensorData.Infrare[2]==1&&SensorData.Infrare[3]==0)	//数字小的在右边，1代表未触发
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//姿态矫正函数进行自校准
			Chassis_Vx=-VX_SPEED_NEAR;
			Chassis_Vw=-VW_REDRESS;	//矫正 逆时针转动
		}
		else if(SetCheck_FrontLift(1)==1&&SensorData.Infrare[2]==0&&SensorData.Infrare[3]==1)
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//姿态矫正函数进行自校准
			Chassis_Vx=-VX_SPEED_NEAR;
			Chassis_Vw=VW_REDRESS;	//矫正	顺时针转动
		}
		
		
		if(SensorData.Limit[2]==1&&SensorData.Limit[3]==1)
		{
			Chassis_Vx=-VX_PRESS_V0;
			Chassis_Vw=0;
				///////////////////////设置到下一个状态
			return 1;
		}
		else if(SensorData.Limit[2]==0&&SensorData.Limit[3]==1)	//0代表未触发
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//姿态矫正函数进行自校准
			Chassis_Vx=-VX_PRESS_V0;
			Chassis_Vw=-VW_REDRESS;	//矫正 逆时针转动
		}
		else if(SensorData.Limit[2]==1&&SensorData.Limit[3]==0)
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//姿态矫正函数进行自校准
			Chassis_Vx=-VX_PRESS_V0;
			Chassis_Vw=VW_REDRESS;	//矫正 逆时针转动
		}
		
	return 0;	
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////上为登岛，下为下岛///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////



DescendState_e DescendState=FULLFALL_DOWN1;	//下岛状态记录位

u8 descend_valve_prepare_state=0;	//自动下岛电磁阀到位保护
u32 descend_valve_prepare_state_count=0;
void Descend_Control_Center(void)
{
	ViceControlData.image_cut[0]=1;	//高电平
	for(int i=0;i<4;i++)
	{
		PID_Chassis_Speed[i].k_i=CHASSIS_SPEED_PID_I*3;
		PID_Chassis_Speed[i].i_sum_max=CHASSIS_SPEED_I_MAX*1.5f;
	}
	
	if(descend_valve_prepare_state==0)	//准备只需要上升然后触发电磁阀，直接可以交给程序，程序会自动下降。而且这样可以避免进行了误操作的风险（全部下落）	
	{
		if(Check_FrontLift()!=0||descend_valve_prepare_state_count>700)	//前腿升起，肯定是
		{
			descend_valve_prepare_state=1;
		}
		SetCheck_FrontLift(1);
		SetCheck_BackLift(1);
		descend_valve_prepare_state_count++;
	}
	else	//如果前腿不在最底部，只有两种可能：	1、准备进行下岛；2、前腿已经落到下一级准备下岛，这两种情况都可以直接执行
	{
	//	descend_valve_prepare_state=1;
	}
	
	ViceControlData.valve[VALVE_ISLAND]=1;	//后面高优先级会覆盖
	
	if(descend_valve_prepare_state==1)	//如果下岛准备
	{
		switch(DescendState)
		{
			case FULLFALL_DOWN1:
			{
				if(Descend_FullFall_Down()==1)
					DescendState=FRONTRISE_DOWM1;
				break;
			}
			case FRONTRISE_DOWM1:
			{
				if(Descend_FrontRise_Down()==1)
					DescendState=FULLRISE_DOWN1;
				break;
			}
			case FULLRISE_DOWN1:
			{
				if(Descend_FullRise_Down1()==1)
					DescendState=FULLFALL_DOWN2;
				break;
			}
			case FULLFALL_DOWN2:
			{
				if(Descend_FullFall_Down()==1)
					DescendState=FRONTRISE_DOWN2;
				break;
			}
			case FRONTRISE_DOWN2:
			{
				if(Descend_FrontRise_Down()==1)
					DescendState=FULLRISE_DOWN2;
				break;
			}
			case FULLRISE_DOWN2:
			{
				ViceControlData.image_cut[0]=0;	//低电平
				if(abs(lift_Data.lb_lift_fdbP+lift_Data.rb_lift_fdbP-2*(LIFT_DISTANCE_FALL-(1!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_ISLAND)))<100)	//提前收回导轮，防止降下去后收不回来		//在上升即将到顶部的时候收回导论
				{
					ViceControlData.valve[VALVE_ISLAND]=0;	//收回导轮
				}
				if(Descend_FullRise_Down1()==1)
				{
					ViceControlData.valve[VALVE_ISLAND]=0;
					DescendState=FULLFALL_DOWN1;	//重置防止下一次异常
					for(int i=0;i<4;i++)
					{
						PID_Chassis_Speed[i].k_i=CHASSIS_SPEED_PID_I;
						PID_Chassis_Speed[i].i_sum_max=CHASSIS_SPEED_I_MAX;
					}
					SetWorkState(NORMAL_STATE);
					Set_Attitude_Correct_State(CORRECT_CHASSIS_STATE);
				}
				break;
			}
		}
	}
}


/*
#define VX_WAIT_BACKRISE	-72	//等待不太危险的后腿前进红外触发，可以快一点
#define VX_NEAR_DOWN -40
#define VX_WAITALLFALL -70		//-90
#define VW_DESCEND 30
*/
#define VX_WAIT_BACKRISE	72	//等待不太危险的后腿前进红外触发，可以快一点
#define VX_NEAR_DOWN 40
#define VX_WAITALLFALL 70		//-90
#define VW_DESCEND 30
u8 Descend_FullFall_Down(void)	//全部落下准备前腿落到下一级动作
{
	Set_Attitude_Correct_State(CORRECT_CHASSIS_STATE);	//设置默认底盘矫正开启，后续IF语句执行将会覆盖
  Chassis_Vx=0;
	Chassis_Vw=0;
	
	SetCheck_FrontLift(0);
	SetCheck_BackLift(0);
	if(SetCheck_FrontLift(0)==1&&SetCheck_BackLift(0)==1)
	{
		Chassis_Vx=VX_NEAR_DOWN;
		
		if(SensorData.Infrare[0]==1&&SensorData.Infrare[1]==1)
		{
			Chassis_Vx=0;
			Chassis_Vw=0;
				///////////////////////设置到下一个状态
			return 1;
		}
		else if(SensorData.Infrare[0]==0&&SensorData.Infrare[1]==1)	//0代表未触发(还在岛上)
		{
			Chassis_Vx=0;
			Chassis_Vw=VW_DESCEND;	//矫正 顺时针转动
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//姿态矫正函数进行自校准
		}
		else if(SensorData.Infrare[0]==1&&SensorData.Infrare[1]==0)
		{
			Chassis_Vx=0;
			Chassis_Vw=-VW_DESCEND;	//矫正 逆时针转动
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//姿态矫正函数进行自校准
		}
		
	}
	
	return 0;
}
 

u8 Descend_FrontRise_Down(void)	//完成前轮下去后，最容易翻车的路段已经过去，可以加快速度
{
	Set_Attitude_Correct_State(CORRECT_CHASSIS_STATE);	//设置默认底盘矫正开启，后续IF语句执行将会覆盖
	Chassis_Vx=0;
	Chassis_Vw=0;
	
	SetCheck_FrontLift(1);
	SetCheck_BackLift(0);
	
	if(SetCheck_FrontLift(1)==1&&SetCheck_BackLift(0)==1)	//完成前轮伸腿后,加速
	{
			Chassis_Vx=VX_WAIT_BACKRISE;
			Chassis_Vw=0;
		if(SensorData.Infrare[2]==1&&SensorData.Infrare[3]==1)
		{
			Chassis_Vx=0;
			Chassis_Vw=0;
				///////////////////////设置到下一个状态
			return 1;
		}
		else if(SensorData.Infrare[2]==0&&SensorData.Infrare[3]==1)	//0代表未触发(还在岛上)（近距离）
		{
			Chassis_Vx=0;
			Chassis_Vw=VW_DESCEND;	//矫正 顺时针转动
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//姿态矫正函数进行自校准
		}
		else if(SensorData.Infrare[2]==1&&SensorData.Infrare[3]==0)
		{
			Chassis_Vx=0;
			Chassis_Vw=-VW_DESCEND;	//矫正 逆时针转动
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//姿态矫正函数进行自校准
		}
		
	}
	return 0;
}


u8 Descend_FullRise_Down1(void)	//5.10更新
{
	static u32 time_record=0;
	static u8 full_fall_statu=0;
	
	Set_Attitude_Correct_State(CORRECT_CHASSIS_STATE);	//设置默认底盘矫正开启，后续IF语句执行将会覆盖
	
	Chassis_Vx=0;
	Chassis_Vw=0;
	
	
	if(SetCheck_FrontLift(1)==1&&SetCheck_BackLift(1)==1&&full_fall_statu==0)	//完成后轮伸腿后full_fall_statu==1后不执行
	{
		full_fall_statu=1;
	}
	
	if(full_fall_statu==1)	//完成后轮伸腿后
	{
		if(time_record==0)	//time_record=0意思即为第一次执行
		{
			time_record=time_1ms_count;
		}
		
		Chassis_Vx=VX_WAITALLFALL;	//70	//这个速度是在中间层台阶上快速通过
		Chassis_Vw=0;
		
		if(time_1ms_count-time_record>200&&time_record!=0)	//这个延时是让前导轮离开上一级台阶
		{
			SetCheck_FrontLift(0);	//完全降落
			SetCheck_BackLift(0);
		}
		
		if(time_1ms_count-time_record>1200&&time_record!=0)	//这个延时作用是直接冲到边缘以便加快速度
		{
			Chassis_Vx=0;
			Chassis_Vw=0;
			time_record=0;
			full_fall_statu=0;
			return 1;
		}
		
	}
	return 0;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/*******************************姿态自校正函数***************************************/
#define ATTITUDE_CORRECT_KP 10	//yaw的单位为度，-180到+180，含分界线
#define ATTITUDE_CORRECT_KD 0.008f

extern GYRO_DATA Gyro_Data;
extern float Chassis_GYRO[3];	//融合后的底盘数据
IslandAttitudeCorrectState_e IslandAttitude_Correct_State=CALI_SELF_STATE;
float tar_attitude=0;
float Attitude_error=0;
s16 t_Vw_correct=0;
//陀螺仪的反馈逆时针为正，姿态旋转顺时针为正
s16 Chassis_Attitude_Correct(float fdbP,int16_t fdbV)	//将状态检测集成在了内部
{
	Attitude_error=fdbP-tar_attitude;  
	s16 Vw_Correct=0;
	
	Attitude_error=Attitude_error>180.0f?(Attitude_error-360.0f):(Attitude_error<-180.0f?(Attitude_error+360.0f):Attitude_error);
	
	switch (IslandAttitude_Correct_State)
	{
		case CALI_SELF_STATE:	//校准目标位置(自己)
		{
			tar_attitude=fdbP;
			Vw_Correct=0;
			break;
		}
		case CORRECT_CHASSIS_STATE:	//矫正底盘姿态
		{
			Vw_Correct=ATTITUDE_CORRECT_KP*(s16)(Attitude_error);	//+(s16)(ATTITUDE_CORRECT_KD*fdbV)
			break;
		}
	}
	
	Vw_Correct=Vw_Correct>100?100:Vw_Correct;
	Vw_Correct=Vw_Correct<-100?-100:Vw_Correct;
	t_Vw_correct=Vw_Correct;
	Vw_Correct=0;	//暂时屏蔽该函数
	return Vw_Correct;
}

void Set_Attitude_Correct_State(IslandAttitudeCorrectState_e state)
{
	IslandAttitude_Correct_State=state;
}





u8 Check_FrontLift(void)	//用于自动登岛状态识别	//返回1是升，0是降
{
	u8 rise_state=1;
	return (abs(lift_Data.lf_lift_fdbP+lift_Data.rf_lift_fdbP-2*(LIFT_DISTANCE_FALL-(rise_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_ISLAND)))<500);	//前（同英雄）升降位置检查，1为升起状态；0为底部状态	//以位置400为分界
}

u8 Check_BackLift(void)	//用于自动登岛状态识别
{
	u8 rise_state=1;
	return (abs(lift_Data.lb_lift_fdbP+lift_Data.rb_lift_fdbP-2*(LIFT_DISTANCE_FALL-(rise_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_ISLAND)))<500);	//后（同英雄）升降位置检查，1为升起状态；0为底部状态
}


AscendState_e Island_State_Recognize(void)	//辨识当前登岛状态
{
	AscendState_e islandstate;
	if(Check_FrontLift()==0&&Check_BackLift()==0)	//全部都是落下的，将状态置到初始登岛，以全部升起
	{
		islandstate=FULLRISE_GO1;	//全部升起
	}
	else if(Check_FrontLift()==1&&Check_BackLift()==0)	//后腿0收起来，前腿1伸出，说明在第2状态
	{
		islandstate=BACKFALL_GO1;
	}
	else	//一般情况不可能出现这种情况，暂时置为全部升起
	{
		islandstate=FULLRISE_GO1;	//全部升起
	}
	return islandstate;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////




u8 SetCheck_FrontLift(u8 rise_state)	//前升降轮升起/落下并检查	//0表示FALL，1表示ISLAND
{
	lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL-(rise_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_ISLAND);
	lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL-(rise_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_ISLAND);
	
	return (abs(lift_Data.lf_lift_fdbP+lift_Data.rf_lift_fdbP-2*(LIFT_DISTANCE_FALL-(rise_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_ISLAND)))<30);	//20
}

u8 SetCheck_BackLift(u8 rise_state)
{
	lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL-(rise_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_ISLAND);
	lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL-(rise_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_ISLAND);
	
	return (abs(lift_Data.lb_lift_fdbP+lift_Data.rb_lift_fdbP-2*(LIFT_DISTANCE_FALL-(rise_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_ISLAND)))<30);	//20
}

