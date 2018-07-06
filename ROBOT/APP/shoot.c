#include "shoot.h"
#include "math.h"

SHOOT_DATA shoot_Data_Down=SHOOT_DATA_INIT;
SHOOT_MOTOR_DATA shoot_Motor_Data_Down ={0};

SHOOT_DATA shoot_Data_Up=SHOOT_DATA_INIT;
SHOOT_MOTOR_DATA shoot_Motor_Data_Up ={0};

PID_GENERAL   PID_Shoot_Down_Position=PID_SHOOT_POSITION_DEFAULT;
PID_GENERAL   PID_Shoot_Down_Speed=PID_SHOOT_SPEED_DEFAULT;

PID_GENERAL   PID_Shoot_Up_Position=PID_SHOOT_POSITION_DEFAULT;
PID_GENERAL   PID_Shoot_Up_Speed=PID_SHOOT_SPEED_DEFAULT;

extern u32 time_1ms_count;

extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];

extern tGameRobotState         testGameRobotState;      //比赛机器人状态
extern u8 Robot_Level;

u8 Friction_State=0;	//初始化不开启
//const u16 FRICTION_INIT=800;
u16 FRICTION_SHOOT=1965;	//发弹的PWM	在检录处测的射速13米每秒
u16 Friction_Send=FRICTION_INIT;
void Shoot_Task(void)	//定时频率：1ms
{ 
	if(time_1ms_count%100==0)
	{
		FRICTION_SHOOT=Friction_Adjust_DependOn_Vol(testPowerHeatData.chassisVolt);	//自动调整输出
	}
	
	Shoot_Instruction();
	shoot_Motor_Data_Down.tarP=(s32)shoot_Data_Down.motor_tarP;
//	shoot_Motor_Data_Up.tarP=(s32)shoot_Data_Up.motor_tarP;
	
	shoot_Motor_Data_Down.tarV=PID_General(shoot_Motor_Data_Down.tarP,shoot_Motor_Data_Down.fdbP,&PID_Shoot_Down_Position);
//	shoot_Motor_Data_Up.tarV=PID_General(shoot_Motor_Data_Up.tarP,shoot_Motor_Data_Up.fdbP,&PID_Shoot_Up_Position);

	Friction_Send=FRICTION_INIT-(FRICTION_INIT-FRICTION_SHOOT)*Friction_State;	//1888对应射速20,1800-14	1830-14.7	1840-15.1（5.14）	1850最高16，最低15		//经过观察，可能和电压有关系，满电时1860为17.7，空电为15.7

	shoot_Motor_Data_Down.output=PID_General(shoot_Motor_Data_Down.tarV,shoot_Motor_Data_Down.fdbV,&PID_Shoot_Down_Speed);//down
	shoot_Motor_Data_Up.output=PID_General(shoot_Motor_Data_Up.tarV,shoot_Motor_Data_Up.fdbV,&PID_Shoot_Up_Speed);//Up
	SetFrictionWheelSpeed(Friction_Send);	//摩擦轮数值发送

}


u16 shoot_time_record=0;
u16 shoot_time_measure(const s16 tarP,const s16 fbdP,const u8 last_mouse_press_l)
{
	static u8 once_statu=0;
	static u16 time_count_start=0;
	static u16 time_count_end=0;
	static u16 time_count_tem=0;
	if(RC_Ctl.mouse.press_l==1&&last_mouse_press_l==0)
	{
		time_count_tem=time_1ms_count;
		once_statu=0;
	}
	if(abs(tarP-fbdP)<2&&once_statu!=1)
	{
		once_statu=1;
		time_count_end=time_1ms_count;
		time_count_start=time_count_tem;
	}
	
	return  time_count_end-time_count_start;
}




#define SINGLE_INCREMENT_OLD_2006 196.608f	//8192*96/4/1000	一圈的累加值8192*96除上一圈7个子弹除以编码器转换倍数=发射一颗子弹的位置增量
#define SINGLE_INCREMENT_NEW_2006 73//65.536f		//8192*32/4/1000
#define SINGLE_INCREMENT SINGLE_INCREMENT_NEW_2006	//5.11少
//输出为发弹量，单位颗
//注：应当在本函数或者另一指令解析函数中设置逻辑：切换状态就重置发弹指令（以免突发情况使程序具有滞后性）
//或者将发弹逻辑改为基于增量式方法的频率控制
void Shoot_Instruction(void)	//发弹指令模块
{
	static u8 auto_takebullet_statu_last=0;
	static WorkState_e State_Record=CHECK_STATE;
	PC_Control_Shoot(&Friction_State);
	RC_Control_Shoot(&Friction_State);
	
//	if((auto_takebullet_statu_last==0&&auto_takebullet_statu==1)||KeyBoardData[KEY_R].value==1)	//开摩擦轮
//	{
//		Friction_State=1;	//自动开摩擦轮
//	}
//	shoot_time_record=shoot_time_measure(shoot_Data_Down.count,shoot_Data_Down.count_fdb,last_mouse_press_l);////////////////////////////////
	
	shoot_Data_Down.motor_tarP=((float)shoot_Data_Down.count*SINGLE_INCREMENT);	//新2006
//	shoot_Data_Up.motor_tarP=((float)shoot_Data_Up.count*SINGLE_INCREMENT);	//新2006
	
	shoot_Motor_Data_Up.tarV=-3500;
	
	Prevent_Jam_Down(&shoot_Data_Down,&shoot_Motor_Data_Down);
	Prevent_Jam_Up(&shoot_Data_Up,&shoot_Motor_Data_Up);
	
	State_Record=GetWorkState();
}

u8 Shoot_RC_Control_State=1;	//当进行按键操作后，进行RC屏蔽
void RC_Control_Shoot(u8* fri_state)
{
	static u8 swicth_Last_state=0;	//右拨杆
	if(Shoot_RC_Control_State==1)
	{
		if(Shoot_Heat_Limit(testPowerHeatData.shooterHeat1,Robot_Level)==1&&Shoot_Heat_Lost_Fre_Limit()==1&&*fri_state==1)	//热量限制
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_UP&&swicth_Last_state==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)
			{
				shoot_Data_Down.count-=1;
				shoot_Data_Down.last_time=time_1ms_count;
			}

		}
		
		if(RC_Ctl.rc.switch_left==RC_SWITCH_UP&&swicth_Last_state==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_UP)
		{
			*fri_state=!*fri_state;
		}
	}
	swicth_Last_state=RC_Ctl.rc.switch_right;
}


void PC_Control_Shoot(u8* fri_state)
{
	static u8 last_mouse_press_l=0;
	static u8 last_keyX_state=0;

	if(RC_Ctl.mouse.press_l==1||RC_Ctl.mouse.press_r==1||RC_Ctl.mouse.x>1||RC_Ctl.mouse.y>1)
	{
		Shoot_RC_Control_State=0;	//屏蔽RC，后期加入若PC失效临时启用RC的操作
	}
	
	
	if(Shoot_Heat_Limit(testPowerHeatData.shooterHeat1,Robot_Level)==1&&Shoot_Heat_Lost_Fre_Limit()==1&&*fri_state==1)	//热量限制
	{
		if(RC_Ctl.mouse.press_l==1&&last_mouse_press_l==0)	//shoot_Motor_Data.tarP-shoot_Motor_Data.fdbP	//待加入
		{
			shoot_Data_Down.count--;
			shoot_Data_Down.last_time=time_1ms_count;
		}
	}
	
	if(last_keyX_state==0&&KeyBoardData[KEY_X].value==1)
	{
		*fri_state=1;
	}
	
	else if(KeyBoardData[KEY_X].statu==2)
	{
		*fri_state=0;
	}
	
	if(*fri_state==0)
	{
		LASER_OFF();
	}
	else
	{
		LASER_ON();
	}
		
	last_mouse_press_l=RC_Ctl.mouse.press_l;
	last_keyX_state=KeyBoardData[KEY_X].value;
}


const float Friction_Set_Record[2][2]={	\
{21.6f,1866},\
{24.4f,1838}\
};	//记录两个电压值端点最优摩擦轮参数，第一个是低，第二个是高
u16 Friction_Adjust_DependOn_Vol(float voltage)	//运算频率10HZ
{
	u16 pwm_set=800;
	static float voltage_deal=24.2f;
	float ratio_vol=0.0;	//记录电压点在总量程比例
	voltage_deal=0.96f*voltage_deal+0.04f*voltage;
	
	ratio_vol=(float)(voltage_deal-Friction_Set_Record[0][0])/(float)(Friction_Set_Record[1][0]-Friction_Set_Record[0][0]);	//待完善
	pwm_set=(u16)((Friction_Set_Record[1][1]-Friction_Set_Record[0][1])*ratio_vol+Friction_Set_Record[0][1]);
	
	if(pwm_set<1820)	pwm_set=1820;
	if(pwm_set>1865)	pwm_set=1865;
//	pwm_set=pwm_set<1840?1840:pwm_set;
//	pwm_set=pwm_set>1865?1865:pwm_set;
	
//	if(Error_Check.statu[LOST_REFEREE]==1)	//裁判lost
//	{
		pwm_set=1800;//1808;
//	}
	
	return pwm_set;
	//根据电压得出最优PWM
}


#define G 9.80151f
/**********************************
Visual_Pretreatment
deal the Visual data
output:disitance: dm
			 priority:0-10
**********************************/
void Visual_Pretreatment()
{
	shoot_Data_Down.Visual.distance=20;
	shoot_Data_Down.Visual.priority=10;
}

/*********************************
Shoot_Rate_Set
caculate the rate data based on the visual.distance data
用于自动射击
*********************************/
void Shoot_Rate_Set()
{
	
}

/*********************************
Shoot_Frequency_Set
*********************************/
void Shoot_Frequency_Set()
{
	
}



/*********************************
Shoot_Frequency_Limit	//老版的V^2已弃用
用于自动打击
*********************************/
#define UPPER_LIMIT_OF_HEAT 4500	//热量上限
#define COOLING_PER_SECOND 1500	//每秒冷却
void Shoot_Frequency_Limit(int* ferquency,u16 rate,u16 heat)	//m/s为单位
{
	u16 heating=rate*rate;
	s16 ferquency_safe=(s16)(COOLING_PER_SECOND/heating);
	if(*ferquency!=0)
	{
		if(heat<5*heating&&heat>=2*heating)	//4倍余量时开始缓冲，以防超出
		{
			*ferquency=(u16)ferquency_safe+1;
		}
		else if(heat<=heating)	//单发余量触发保护
		{
			*ferquency=0;
		}
		else if(heat>=heating&&heat<2*heating)
		{
			*ferquency=(u16)((ferquency_safe-1)>0?(ferquency_safe-1):0);
		}
	}

}



u8 Shoot_Heat_Limit(u16 heating,u8 level)	//还应当限制射频
{
	if((80*pow(2,level-1)-heating>46)&&time_1ms_count-shoot_Data_Down.last_time>250)	//testPowerHeatData.shooterHeat1
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

u8 Shoot_Heat_Lost_Fre_Limit(void)	//裁判lost情况对射频的限制，反返回1是OK
{
	u8 limit_state=0;
	if(Error_Check.statu[LOST_REFEREE]==1)	//裁判lost
	{
		if(time_1ms_count-shoot_Data_Down.last_time>250)	//大于1s	//临时测试，1秒4发	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		{
			limit_state=1;
		}
	}
	else
	{
		limit_state=1;
	}
	return limit_state;
}

void Shoot_Speed_Adjust(u16 * pwm , u16 speed_fdb)	//射速闭环
{
	
}

/*********************************
Shoot_Rate_Adjust
*********************************/
void Shoot_Rate_Adjust()
{
	
}
s32 jam_DownfdbP_record;	//这里必须是s32不然在开始时卡单会死循环
#define JAM_FALLBACK 34	//100	//往回走的距离
//对tarP的操作
void Prevent_Jam_Down(SHOOT_DATA * shoot_data,SHOOT_MOTOR_DATA * shoot_motor_Data)	//防卡弹程序	//同时包含防鸡蛋的功能	//放在tarP计算出之后
{
	static s32 deviation=0;	//偏差
	static u8 jam_deal_state=0;
//	static u16 ferquency_last=0;
	
	deviation=shoot_motor_Data->tarP-shoot_motor_Data->fdbP;
	
//	if(shoot_data->frequency!=ferquency_last)
//	{
//		shoot_data->Jam.count=0;	//重置count
//	}
//	ferquency_last=shoot_data->frequency;	//迭代
	
	
	if(abs(deviation)>6&&abs(shoot_motor_Data->fdbV)<10)	//期望速度不为0时位置未发生变化	//bug:频率刷新时需要刷新count	//手动射击将频率检测删除
	{
		shoot_data->Jam.count++;
	}
	else
	{
		shoot_data->Jam.count=0;
	}
	
//	if(shoot_data->cycle!=0)
//	{
		if(shoot_data->Jam.count>100&&shoot_data->Jam.sign==0)	//超出非正常时间	//且仅执行一次
		{
			 shoot_data->Jam.sign=1;	//标记卡弹
			 jam_deal_state=1;	//标记卡弹处理进程状态
		}
//	}
	
	if(shoot_data->Jam.sign==1)	//处理卡弹模块
	{
		switch (jam_deal_state)
		{
				case 1:
				{
					jam_DownfdbP_record=shoot_motor_Data->fdbP-JAM_FALLBACK;	//可能会在开始时候卡弹有危险？
					shoot_data->motor_tarP=jam_DownfdbP_record;
					jam_deal_state=2;
					break;
				}
				case 2:
				{
					shoot_data->motor_tarP=jam_DownfdbP_record;
					if(abs(shoot_motor_Data->fdbP-jam_DownfdbP_record)<40)	//认为已经执行了动作	//50
					{
						jam_deal_state=3;
					}
					break;
				}
				case 3:
				{
					shoot_data->Jam.sign=0;	//Reset
					jam_deal_state=0;	//
					shoot_data->count=shoot_data->count_fdb;	//重置子弹数据，防止鸡蛋	//？是否需要+-1？
					shoot_data->Jam.count=0;	//重置卡弹检测数据，防止误检测
					break;
				}
		}
	}
	
}   


void Prevent_Jam_Up(SHOOT_DATA * shoot_data,SHOOT_MOTOR_DATA * shoot_motor_Data)	//待加入从STOP切换的保护//防卡弹程序	//同时包含防鸡蛋的功能	//放在tarP计算出之后
{
	static u8 jam_deal_state=0;
	static u32 time_record_jam_start=0;
	static u32 time_record_jam_end=0;
	if(abs(shoot_motor_Data->fdbV)<100&&shoot_data->Jam.sign==0)	//卡弹了
	{
		shoot_data->Jam.count++;
	}
	else	//当速度不符合条件，进行清零，当在处理卡弹，也进行清零防止处理完后容易触发
	{
		shoot_data->Jam.count=0;
	}
	
	if(shoot_data->Jam.count>100)	//50ms
	{
		shoot_data->Jam.sign=1;
		jam_deal_state=1;
	}
	
	if(shoot_data->Jam.sign==1)	//处理卡弹模块
	{
		switch (jam_deal_state)
		{
				case 1:
				{
					shoot_motor_Data->tarV=2500;
					if(time_record_jam_start==0)
					{
						time_record_jam_start=time_1ms_count;	//记录处理时间
					}
					if(time_1ms_count-time_record_jam_start>300&&time_record_jam_start!=0)	//半秒
					{
						jam_deal_state=2;
						time_record_jam_start=0;
					}
					break;
				}
				case 2:
				{
					if(time_record_jam_end==0)
					{
						time_record_jam_end=time_1ms_count;
					}
					
					shoot_motor_Data->tarV=-3500;
					if(time_1ms_count-time_record_jam_end>200&&time_record_jam_end!=0)
					{
						jam_deal_state=1;
						shoot_data->Jam.sign=0;
						time_record_jam_end=0;
					}
					break;
				}
		}
	}
	
}

/*******************************分区赛版*******************************
以下为上拨弹防卡弹
s32 jam_UpfdbP_record;
//对tarP的操作
void Prevent_Jam_Up(SHOOT_DATA * shoot_data,SHOOT_MOTOR_DATA * shoot_motor_Data)	//防卡弹程序	//同时包含防鸡蛋的功能	//放在tarP计算出之后
{
	static s32 deviation=0;	//偏差
	static u8 jam_deal_state=0;
//	static u16 ferquency_last=0;
	
	deviation=shoot_motor_Data->tarP-shoot_motor_Data->fdbP;
	
//	if(shoot_data->frequency!=ferquency_last)
//	{
//		shoot_data->Jam.count=0;	//重置count
//	}
//	ferquency_last=shoot_data->frequency;	//迭代
	
	
	if(abs(deviation)>6&&abs(shoot_motor_Data->fdbV)<10)	//期望速度不为0时位置未发生变化	//bug:频率刷新时需要刷新count	//手动射击将频率检测删除
	{
		shoot_data->Jam.count++;
	}
	else
	{
		shoot_data->Jam.count=0;
	}
	
//	if(shoot_data->cycle!=0)
//	{
		if(shoot_data->Jam.count>100&&shoot_data->Jam.sign==0)	//超出非正常时间	//且仅执行一次
		{
			 shoot_data->Jam.sign=1;	//标记卡弹
			 jam_deal_state=1;	//标记卡弹处理进程状态
		}
//	}
	
	if(shoot_data->Jam.sign==1)	//处理卡弹模块
	{
		switch (jam_deal_state)
		{
				case 1:
				{
					jam_UpfdbP_record=shoot_motor_Data->fdbP-JAM_FALLBACK;
					shoot_data->motor_tarP=jam_UpfdbP_record;
					jam_deal_state=2;
					break;
				}
				case 2:
				{
					shoot_data->motor_tarP=jam_UpfdbP_record;
					if(abs(shoot_motor_Data->fdbP-jam_UpfdbP_record)<40)	//认为已经执行了动作	//50
					{
						jam_deal_state=3;
					}
					break;
				}
				case 3:
				{
					shoot_data->Jam.sign=0;	//Reset
					jam_deal_state=0;	//
					shoot_data->count=shoot_data->count_fdb;	//重置子弹数据，防止鸡蛋	//？是否需要+-1？
					shoot_data->Jam.count =0;	//重置卡弹检测数据，防止误检测
					break;
				}
		}
	}
	
} 
*************************************************/



/*****************************************
函数名称：Shoot_Feedback_Deal
函数功能：拨弹电机反馈数据解析+处理

*****************************************/
void Shoot_Feedback_Deal(SHOOT_DATA *shoot_data,SHOOT_MOTOR_DATA *shoot_motor_data,CanRxMsg *msg)
{
	shoot_motor_data->fdbP_raw=(msg->Data[0]<<8)|msg->Data[1];//接收到的真实数据值  处理频率1KHz
	shoot_motor_data->fdbV=(msg->Data[2]<<8)|msg->Data[3];
	
	shoot_motor_data->fdbP_diff=shoot_motor_data->fdbP_raw_last-shoot_motor_data->fdbP_raw;
	if(shoot_motor_data->fdbP_diff>4096)	//按照6倍采样来计算，机械角度共8192个挡位，则过界表现差值为6826
	{																			//注：此函数未对第一次运行时的可能的圈数直接为1的偏差做处理（处理方法在初始化中标定初始角度值）
		shoot_motor_data->fdbP_raw_sum+=8192;
	}
	else if(shoot_motor_data->fdbP_diff<-4096)
	{
		shoot_motor_data->fdbP_raw_sum-=8192;
	}
	
	shoot_motor_data->fdbP=(s32)((shoot_motor_data->fdbP_raw_sum+shoot_motor_data->fdbP_raw)/1000.0f);	//因为2006减速比过大 不便精确
	
	shoot_motor_data->fdbP_raw_last=shoot_motor_data->fdbP_raw;	//数据迭代
	
	shoot_data->count_fdb=(u16)(shoot_motor_data->fdbP/SINGLE_INCREMENT);	///////防止卡弹，刷新反馈的
}




//一圈7个
//极限15小

