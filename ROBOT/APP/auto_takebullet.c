#include "auto_takebullet.h"
#include "main.h"

#define LIFT_DISTANCE_BULLET 700

TakeBulletState_e TakeBulletState=BULLET_OTHER;	//（自动）取弹状态位
AutoAimBulletTypeDef AutoAimBulletData={0};

#define BULLETROTATE_OTHER	18//0	//非取弹位置
#define BULLETROTATE_WAITING	476//455//-750//650	//等待（对位）时位置
#define BULLETROTATE_ACQUIRE	1050//1050//1030	//取弹位置
#define BULLETROTATE_POUROUT	80//130//120	//倒弹位置
#define BULLETROTATE_THROWOUT	960//970//960//-280//310	//抛出位置

#define BULLETROTATE_POUROUT_DELAY	180	//200ms

extern u32 time_1ms_count;
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
extern RC_Ctl_t RC_Ctl;
extern ViceControlDataTypeDef ViceControlData;
extern PID_GENERAL PID_Chassis_Speed[4];
extern u8 Replenish_Bullet_Statu;	//补弹模式特殊舵机状态

extern BULLETROTATE_DATA BulletRotate_Data[2];	//国赛版

extern SensorDataTypeDef SensorData;

#define STEER_UP_L_INIT 560//
#define STEER_UP_R_INIT 2500//1950	//
#define STEER_UP_L_REVERSAL 1750//
#define STEER_UP_R_REVERSAL 1300//
float pwm_l_t=STEER_UP_L_INIT;
float pwm_r_t=STEER_UP_R_INIT;


u8 valve_fdbstate[6]={0};	//记录是否伸出的反馈标志
u8 servo_fdbstate[2]={0};
const u32 valve_GOODdelay[6]={370,370,300,1000,1000,1000};	//0--1//待加入，延时参数
const u32 valve_POORdelay[6]={370,370,90,1000,1000,1000};	//1--0//待加入，延时参数
const u32 servo_GOODdelay[2]={2500,800};	//延时参数	//第一段为2500是将子弹落下的延时也加进去了，因为舵机翻转和子弹下落必须是连在一体的
const u32 servo_POORdelay[2]={500,500};	//延时参数


//#define VALVE_BULLET_HORIZONTAL1 0		//原登岛--现平移1
//#define VALVE_BULLET_HORIZONTAL2 1	//原前伸--现平移2
//#define VALVE_BULLET_CLAMP 2	//夹紧

extern s16 Chassis_Vx;
extern s16 Chassis_Vy;
extern u8 BulletRotate_Cali_Statu;	//标定状态	//置0重新标定

u8 TakeBullet_AutoAimState=1;	//默认开启自动对位，单词取弹模式可取消，对本次有效
u8 t_statu=0;
void TakeBullet_Control_Center(void)
{
	static u32 time_record_pourout=0;	//用来记录倒弹延时等待
	
	static u8 swicth_Last_state=0;	//右拨杆
	
	static u8 valve_last[6]={0};	//记录上一次数值	//保持与工程车兼容性
	static u8 servo_last[2]={0};	//记录上一次数值	//保持与工程车兼容性
	
	static u32 valve_startGOOD_time[6]={0};	//记录顺向触发时间	//保持与工程车兼容性
	static u32 servo_startGOOD_time[2]={0};	//记录顺向触发时间	//保持与工程车兼容性
	static u32 valve_startPOOR_time[6]={0};	//记录逆向触发时间	//保持与工程车兼容性
	static u32 servo_startPOOR_time[2]={0};	//记录逆向触发时间	//保持与工程车兼容性
	
	static WorkState_e State_Record=CHECK_STATE;
	
	static TakeBulletState_e takebulletstate_last=BULLET_OTHER;
	
	if(GetWorkState()==TAKEBULLET_STATE)	//5.9更新//上一版--》//取弹升降给DOWN-MID，前伸出发-夹紧一套给DOWN-MID-->DOWN-DOWN;舵机旋转给DOWN-MID-->DOWN-UP
	{
		static u8 key_ctrl_last=0;
		static u8 key_shift_last=0;
			
		if(key_ctrl_last==0&&KeyBoardData[KEY_CTRL].value==1)	//取弹模式按了CTRL就取消自动取块
		{
			TakeBullet_AutoAimState=!TakeBullet_AutoAimState;	//屏蔽自动对位模块
		}
		key_ctrl_last=KeyBoardData[KEY_CTRL].value;

		if(key_shift_last==0&&KeyBoardData[KEY_SHIFT].value==1&&TakeBulletState==BULLET_WAITING)	//SHIFT重新开始取两块
		{
			AutoAimBulletData.take_count=0;
		}
		key_shift_last=KeyBoardData[KEY_SHIFT].value;
		
		if(State_Record!=TAKEBULLET_STATE)
		{
			TakeBulletState=BULLET_WAITING;
		}
		
		if(RC_Ctl.rc.ch3-1024>80&&TakeBulletState==BULLET_WAITING)	/////////////////////////////修改操作模式时需要修改
		{
			TakeBulletState=BULLET_ACQUIRE1;
		}
		else if(RC_Ctl.rc.ch3-1024<-80)
		{
			TakeBulletState=BULLET_WAITING;
		}			
		
		if(TakeBulletState==BULLET_WAITING&&KeyBoardData[KEY_Q].value==1&&KeyBoardData[KEY_E].value==1)	//qe同时按重新标定
		{
			BulletRotate_Cali_Statu=0;	//重新标定
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
	
	if(State_Record!=TAKEBULLET_STATE&&GetWorkState()==TAKEBULLET_STATE)	//进入取弹模式
	{
//		AutoAimBulletData.take_count=0;	//清零取块数量记录
		AutoAimBulletData.aim_state=0;	//back
		
//		if(AutoAimBulletData.take_count==0)	//第一次取块
//		{
//			AutoAimBulletData.control_state=1;	//第一次开启对位	//该变量影响自动对位底盘屏蔽
//			TakeBullet_AutoAimState=1;	//默认开启自动模式
//		}
//		else
//		{
			AutoAimBulletData.control_state=0;	//关闭对位	//该变量影响自动对位底盘屏蔽
			TakeBullet_AutoAimState=0;	//默认关闭自动模式
//		}
		time_record_pourout=0;	//防止意外退出导致BUG
	}
	
	if(State_Record==TAKEBULLET_STATE&&GetWorkState()!=TAKEBULLET_STATE)	//退出取弹模式
	{
		Chassis_Vx=0;
		Chassis_Vy=0;
		AutoAimBulletData.control_state=0;	//关闭对位//该变量影响自动对位底盘屏蔽
//		AutoAimBulletData.take_count=0;	//清零取块数量记录
		AutoAimBulletData.aim_state=0;	//back
		TakeBullet_AutoAimState=0;	//默认关闭自动模式
		
		time_record_pourout=0;	//防止意外退出导致BUG
	}
	
	
	if(AutoAimBulletData.control_state==1)	//自动对位开启，则底盘PID变高
	{
		PID_Chassis_Speed[0].k_p=CHASSIS_SPEED_PID_P*1.2f;
		PID_Chassis_Speed[0].k_i=CHASSIS_SPEED_PID_I*1.2f;
		PID_Chassis_Speed[0].i_sum_max=CHASSIS_SPEED_I_MAX*1.2f;
//		PID_Chassis_Speed[0].k_d=CHASSIS_SPEED_PID_P*1.2f;
	}
	else
	{
		PID_Chassis_Speed[0].k_p=CHASSIS_SPEED_PID_P;
		PID_Chassis_Speed[0].k_i=CHASSIS_SPEED_PID_I;
		PID_Chassis_Speed[0].i_sum_max=CHASSIS_SPEED_I_MAX;
	}
			
	State_Record=GetWorkState();
	
	
	if(TakeBullet_AutoAimState==0)	//一个保护
	{
		AutoAimBulletData.control_state=0;	//关闭对位
		AutoAimBulletData.aim_state=0;
	}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	if(AutoAimBulletData.take_count<2)	//第一次取弹（或者一次取两箱）
	{
		switch(TakeBulletState)	//自动取弹过程
		{
			case BULLET_WAITING:	//等待取弹动作（对位）状态
			{
				
				if(TakeBullet_AutoAimState==1&&valve_fdbstate[VALVE_BULLET_HORIZONTAL1]==0&&valve_fdbstate[VALVE_BULLET_HORIZONTAL2]==1)	//自动取弹模式开启
				{
					if(AutoAimBulletData.take_count==0)
					{
						AutoAimBulletData.control_state=1;	//启动对位//该变量影响自动对位底盘屏蔽
						if(AutoAimBullet_Task(&Chassis_Vx,&Chassis_Vy)==1)
						{
							TakeBulletState=BULLET_ACQUIRE1;	//启动夹块
							AutoAimBulletData.control_state=0;	//关闭对位//该变量影响自动对位底盘屏蔽
							AutoAimBulletData.aim_state=0;
						}
					}
//					else if(AutoAimBulletData.take_count==2)
//					{
//						AutoAimBulletData.control_state=1;	//启动对位
//						if(AutoAimBullet_Task(&Chassis_Vx,&Chassis_Vy)==1)
//						{
//							TakeBulletState=BULLET_ACQUIRE1;	//启动夹块
//							AutoAimBulletData.control_state=0;	//关闭对位
//							AutoAimBulletData.aim_state=0;
//						}
//					}
				}
				
				
				ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
				
				if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)	//气缸松开
				{
					BulletRotate_Data[BULLETROTATE_RIGHT].tarP=BULLETROTATE_WAITING;
					if(abs(BulletRotate_Data[BULLETROTATE_RIGHT].fdbP-BULLETROTATE_WAITING)<30)	//电机收回
					{
						ViceControlData.valve[VALVE_BULLET_HORIZONTAL1]=0;
						ViceControlData.valve[VALVE_BULLET_HORIZONTAL2]=1;
					}
				}
				
				break;
			}
			case BULLET_ACQUIRE1:	//前伸、夹紧、抬起动作	称之为获得过程
			{
				BulletRotate_Data[BULLETROTATE_RIGHT].tarP=BULLETROTATE_ACQUIRE;
				if(abs(BulletRotate_Data[BULLETROTATE_RIGHT].fdbP-BULLETROTATE_ACQUIRE)<45)
				{
					ViceControlData.valve[VALVE_BULLET_CLAMP]=1;
					if(valve_fdbstate[VALVE_BULLET_CLAMP]==1)
					{
						TakeBulletState=BULLET_POUROUT1;	//切换到倒弹
					}
				}
				break;
			}
			case BULLET_POUROUT1:	//车身倾斜、舵机旋转	称之为倒弹过程
			{
//				static u32 time_record=0;
				
				BulletRotate_Data[BULLETROTATE_RIGHT].tarP=BULLETROTATE_POUROUT;
				if(abs(BulletRotate_Data[BULLETROTATE_RIGHT].fdbP-BULLETROTATE_POUROUT)<50)
				{
					if(time_record_pourout==0)
					{
						time_record_pourout=time_1ms_count;	//记录进入的时间
					}
					
					if(time_record_pourout!=0&&time_1ms_count-time_record_pourout>BULLETROTATE_POUROUT_DELAY)	//300ms延时
					{
						TakeBulletState=BULLET_THROWOUT1;	//切换到扔出
						time_record_pourout=0;
					}
				}
				break;
			}
			case BULLET_THROWOUT1:	//舵机旋回、车身抬起、夹紧松开	称之为抛落过程
			{
				
				if(valve_fdbstate[VALVE_BULLET_CLAMP]==1)	//夹紧时放下，放下后回来
				{
					BulletRotate_Data[BULLETROTATE_RIGHT].tarP=BULLETROTATE_THROWOUT;
				}
				
				if(abs(BulletRotate_Data[BULLETROTATE_RIGHT].fdbP-BULLETROTATE_THROWOUT)<40)	//放回原位
				{
					ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
				}
				
				if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)	//已经松开，开始回来
				{
					BulletRotate_Data[BULLETROTATE_RIGHT].tarP=BULLETROTATE_WAITING;
					
					if((BulletRotate_Data[BULLETROTATE_RIGHT].fdbP-BULLETROTATE_THROWOUT)<-60)	//微微抬起后开始平移
					{
						ViceControlData.valve[VALVE_BULLET_HORIZONTAL1]=1;
						ViceControlData.valve[VALVE_BULLET_HORIZONTAL2]=0;
						t_statu++;
					}
				}
				
				
				
				if(valve_fdbstate[VALVE_BULLET_CLAMP]==0&&abs(BulletRotate_Data[BULLETROTATE_RIGHT].fdbP-BULLETROTATE_WAITING)<40)	//松开后且到准备位置，可以开始下一次
				{
					TakeBulletState=BULLET_ACQUIRE2;	//下一次取弹
					TakeBullet_AutoAimState=0;	//取到第一个块之后取消自动对位，好处是如果第一个块不成功就一直保持自动对位
				}
				break;
			}
			case BULLET_ACQUIRE2:	//前伸、夹紧、抬起动作	称之为获得过程2
			{
				if(valve_fdbstate[VALVE_BULLET_HORIZONTAL1]==1)	//气缸平移
				{
					BulletRotate_Data[BULLETROTATE_RIGHT].tarP=BULLETROTATE_ACQUIRE;
					if(abs(BulletRotate_Data[BULLETROTATE_RIGHT].fdbP-BULLETROTATE_ACQUIRE)<45)
					{
						ViceControlData.valve[VALVE_BULLET_CLAMP]=1;
						if(valve_fdbstate[VALVE_BULLET_CLAMP]==1)
						{
							TakeBulletState=BULLET_POUROUT2;	//切换到倒弹
						}
					}
				}
				break;
			}
			case BULLET_POUROUT2:	//车身倾斜、舵机旋转	称之为倒弹过程2
			{
//				static u32 time_record=0;
				
				BulletRotate_Data[BULLETROTATE_RIGHT].tarP=BULLETROTATE_POUROUT;
				if(abs(BulletRotate_Data[BULLETROTATE_RIGHT].fdbP-BULLETROTATE_POUROUT)<50)
				{
					if(time_record_pourout==0)
					{
						time_record_pourout=time_1ms_count;	//记录进入的时间
					}
					
					if(time_record_pourout!=0&&time_1ms_count-time_record_pourout>BULLETROTATE_POUROUT_DELAY)	//300ms延时
					{
						TakeBulletState=BULLET_THROWOUT2;	//切换到扔出
						time_record_pourout=0;
					}
					
				}
				break;
			}
			case BULLET_THROWOUT2:	//舵机旋回、车身抬起、夹紧松开	称之为抛落过程2
			{
				///////////////////////////////////////////
				
				if(valve_fdbstate[VALVE_BULLET_CLAMP]==1)	//夹紧时放下，放下后回来
				{
					BulletRotate_Data[BULLETROTATE_RIGHT].tarP=BULLETROTATE_THROWOUT;
				}
				
				if(abs(BulletRotate_Data[BULLETROTATE_RIGHT].fdbP-BULLETROTATE_THROWOUT)<40)	//放回原位
				{
					ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
				}
				
				if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)	//已经松开，开始回来
				{
					BulletRotate_Data[BULLETROTATE_RIGHT].tarP=BULLETROTATE_WAITING;
					
	////////////				if((BulletRotate_Data.fdbP-BULLETROTATE_THROWOUT)>60)	//微微抬起后开始平移
	////////////				{
	////////////					ViceControlData.valve[VALVE_BULLET_HORIZONTAL1]=0;
	////////////					ViceControlData.valve[VALVE_BULLET_HORIZONTAL2]=1;
	////////////					t_statu++;
	////////////				}
				}
				
				if(valve_fdbstate[VALVE_BULLET_CLAMP]==0&&abs(BulletRotate_Data[BULLETROTATE_RIGHT].fdbP-BULLETROTATE_WAITING)<40)	//松开后且到准备位置，可以开始下一次
				{
					TakeBulletState=BULLET_WAITING;	//下一次取弹
					AutoAimBulletData.take_count+=2;	//标记取弹数量+2
				}
				
				break;
			}
			case BULLET_OTHER:	//其他非取弹状态--任务：回中后放下取弹
			{
				ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
				static u8 valve_horizontal_state=0;	//记录电磁阀位置
				static u32 valve_getback_time_record=0;
				static u8 valve_getback_statu=0;
					
				if(takebulletstate_last!=BULLET_OTHER)
				{
					valve_getback_time_record=0;	//切换时清零清除上一次的影响
					valve_getback_statu=0;
				}
				
				if(valve_fdbstate[VALVE_BULLET_HORIZONTAL1]==0&&valve_fdbstate[VALVE_BULLET_HORIZONTAL2]==1&&valve_getback_statu!=1)	//在都切换到0后判断有延时所以需要增加一个完成标志位避免延时切换状态对其的影响
				{
					valve_horizontal_state=1;
					//BulletRotate_Data.tarP=BULLETROTATE_WAITING;	//因为逻辑限制只能从WAITING跳到OTHER，所以tarP必然为WAITING 
				}
				else if(valve_fdbstate[VALVE_BULLET_HORIZONTAL1]==1&&valve_fdbstate[VALVE_BULLET_HORIZONTAL2]==0&&valve_getback_statu!=1)
				{
					valve_horizontal_state=2;
				}
				
				if(valve_horizontal_state==1)	//从0 1回中
				{
					if(valve_getback_time_record==0)
					{
						ViceControlData.valve[VALVE_BULLET_HORIZONTAL1]=1;
						ViceControlData.valve[VALVE_BULLET_HORIZONTAL2]=0;
						valve_getback_time_record=time_1ms_count;
					}					

					if(time_1ms_count-valve_getback_time_record>140)	//140延时到中间
					{
						ViceControlData.valve[VALVE_BULLET_HORIZONTAL1]=0;
						ViceControlData.valve[VALVE_BULLET_HORIZONTAL2]=0;
						valve_getback_time_record=0;
						valve_horizontal_state=0;
						valve_getback_statu=1;
					}
				}

				if(valve_horizontal_state==2)	//从1 0回中
				{
					if(valve_getback_time_record==0)
					{
						ViceControlData.valve[VALVE_BULLET_HORIZONTAL1]=0;
						ViceControlData.valve[VALVE_BULLET_HORIZONTAL2]=1;
						valve_getback_time_record=time_1ms_count;
					}					

					if(time_1ms_count-valve_getback_time_record>130)	//200延时到中间
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
				BulletRotate_Data[BULLETROTATE_RIGHT].tarP=BULLETROTATE_OTHER;
			}

				
				break;
			}
		}
	}	//if结尾
	else	//第二次抓块（一次取一箱）
	{
		switch(TakeBulletState)	//自动取弹过程
		{
			case BULLET_WAITING:	//等待取弹动作（对位）状态
			{
				
				if(TakeBullet_AutoAimState==1&&valve_fdbstate[VALVE_BULLET_HORIZONTAL1]==1&&valve_fdbstate[VALVE_BULLET_HORIZONTAL2]==0)	//自动取弹模式开启
				{
					if(AutoAimBulletData.take_count>0)
					{
						AutoAimBulletData.control_state=1;	//启动对位//该变量影响自动对位底盘屏蔽
						if(AutoAimBullet_Task(&Chassis_Vx,&Chassis_Vy)==1)
						{
							TakeBulletState=BULLET_ACQUIRE1;	//启动夹块
							AutoAimBulletData.control_state=0;	//关闭对位//该变量影响自动对位底盘屏蔽
							AutoAimBulletData.aim_state=0;
						}
					}
//					else if(AutoAimBulletData.take_count==2)
//					{
//						AutoAimBulletData.control_state=1;	//启动对位
//						if(AutoAimBullet_Task(&Chassis_Vx,&Chassis_Vy)==1)
//						{
//							TakeBulletState=BULLET_ACQUIRE1;	//启动夹块
//							AutoAimBulletData.control_state=0;	//关闭对位
//							AutoAimBulletData.aim_state=0;
//						}
//					}
				}
				
				
				ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
				
				if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)	//气缸松开
				{
					BulletRotate_Data[BULLETROTATE_RIGHT].tarP=BULLETROTATE_WAITING;
					if(abs(BulletRotate_Data[BULLETROTATE_RIGHT].fdbP-BULLETROTATE_WAITING)<30)	//电机收回
					{
						ViceControlData.valve[VALVE_BULLET_HORIZONTAL1]=1;
						ViceControlData.valve[VALVE_BULLET_HORIZONTAL2]=0;
					}
				}
				
				break;
			}
			case BULLET_ACQUIRE1:	//前伸、夹紧、抬起动作	称之为获得过程
			{
				BulletRotate_Data[BULLETROTATE_RIGHT].tarP=BULLETROTATE_ACQUIRE;
				if(abs(BulletRotate_Data[BULLETROTATE_RIGHT].fdbP-BULLETROTATE_ACQUIRE)<45)
				{
					ViceControlData.valve[VALVE_BULLET_CLAMP]=1;
					if(valve_fdbstate[VALVE_BULLET_CLAMP]==1)
					{
						TakeBulletState=BULLET_POUROUT1;	//切换到倒弹
					}
				}
				break;
			}
			case BULLET_POUROUT1:	//车身倾斜、舵机旋转	称之为倒弹过程
			{
//				static u32 time_record=0;
				
				BulletRotate_Data[BULLETROTATE_RIGHT].tarP=BULLETROTATE_POUROUT;
				if(abs(BulletRotate_Data[BULLETROTATE_RIGHT].fdbP-BULLETROTATE_POUROUT)<50)
				{
					if(time_record_pourout==0)
					{
						time_record_pourout=time_1ms_count;	//记录进入的时间
					}
					
					if(time_record_pourout!=0&&time_1ms_count-time_record_pourout>BULLETROTATE_POUROUT_DELAY)	//300ms延时
					{
						TakeBulletState=BULLET_THROWOUT1;	//切换到扔出
						time_record_pourout=0;
					}
					
				}
				break;
			}
			case BULLET_THROWOUT1:	//舵机旋回、车身抬起、夹紧松开	称之为抛落过程
			{
				
				if(valve_fdbstate[VALVE_BULLET_CLAMP]==1)	//夹紧时放下，放下后回来
				{
					BulletRotate_Data[BULLETROTATE_RIGHT].tarP=BULLETROTATE_THROWOUT;
				}
				
				if(abs(BulletRotate_Data[BULLETROTATE_RIGHT].fdbP-BULLETROTATE_THROWOUT)<40)	//放回原位
				{
					ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
				}
				
				if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)	//已经松开，开始回来
				{
					BulletRotate_Data[BULLETROTATE_RIGHT].tarP=BULLETROTATE_WAITING;
					
					if((BulletRotate_Data[BULLETROTATE_RIGHT].fdbP-BULLETROTATE_THROWOUT)<-60)	//微微抬起后开始平移
					{
						ViceControlData.valve[VALVE_BULLET_HORIZONTAL1]=1;
						ViceControlData.valve[VALVE_BULLET_HORIZONTAL2]=0;
					}
				}
				
				
				
				if(valve_fdbstate[VALVE_BULLET_CLAMP]==0&&abs(BulletRotate_Data[BULLETROTATE_RIGHT].fdbP-BULLETROTATE_WAITING)<40)	//松开后且到准备位置，可以开始下一次
				{
					TakeBulletState=BULLET_WAITING;	//下一次取弹
					AutoAimBulletData.take_count+=1;	//标记取弹数量+1
					TakeBullet_AutoAimState=0;	//取到第一个块之后取消自动对位，好处是如果第一个块不成功就一直保持自动对位
				}
				break;
			}
			case BULLET_OTHER:	//其他非取弹状态--任务：回中后放下取弹
			{
				ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
				static u8 valve_horizontal_state=0;	//记录电磁阀位置
				static u32 valve_getback_time_record=0;
				static u8 valve_getback_statu=0;
					
				if(takebulletstate_last!=BULLET_OTHER)
				{
					valve_getback_time_record=0;	//切换时清零清除上一次的影响
					valve_getback_statu=0;
				}
				
				if(valve_fdbstate[VALVE_BULLET_HORIZONTAL1]==0&&valve_fdbstate[VALVE_BULLET_HORIZONTAL2]==1&&valve_getback_statu!=1)	//在都切换到0后判断有延时所以需要增加一个完成标志位避免延时切换状态对其的影响
				{
					valve_horizontal_state=1;
					//BulletRotate_Data.tarP=BULLETROTATE_WAITING;	//因为逻辑限制只能从WAITING跳到OTHER，所以tarP必然为WAITING 
				}
				else if(valve_fdbstate[VALVE_BULLET_HORIZONTAL1]==1&&valve_fdbstate[VALVE_BULLET_HORIZONTAL2]==0&&valve_getback_statu!=1)
				{
					valve_horizontal_state=2;
				}
				
				if(valve_horizontal_state==1)	//从0 1回中
				{
					if(valve_getback_time_record==0)
					{
						ViceControlData.valve[VALVE_BULLET_HORIZONTAL1]=1;
						ViceControlData.valve[VALVE_BULLET_HORIZONTAL2]=0;
						valve_getback_time_record=time_1ms_count;
					}					

					if(time_1ms_count-valve_getback_time_record>140)	//140延时到中间
					{
						ViceControlData.valve[VALVE_BULLET_HORIZONTAL1]=0;
						ViceControlData.valve[VALVE_BULLET_HORIZONTAL2]=0;
						valve_getback_time_record=0;
						valve_horizontal_state=0;
						valve_getback_statu=1;
					}
				}

				if(valve_horizontal_state==2)	//从1 0回中
				{
					if(valve_getback_time_record==0)
					{
						ViceControlData.valve[VALVE_BULLET_HORIZONTAL1]=0;
						ViceControlData.valve[VALVE_BULLET_HORIZONTAL2]=1;
						valve_getback_time_record=time_1ms_count;
					}					

					if(time_1ms_count-valve_getback_time_record>130)	//200延时到中间
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
				BulletRotate_Data[BULLETROTATE_RIGHT].tarP=BULLETROTATE_OTHER;
			}

				
				break;
			}
		}//switch结束
	}
	//自动取弹放置位
	
	takebulletstate_last=TakeBulletState;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	/******************************************************************
以下三个for为反馈假设检测方案	//在任意时刻都进行检测
分别为：
1.上升，下降沿的触发时间记录
2.根据触发时间的推演反馈值计算
3.数据迭代
******************************************************************/
	for(int i=0;i<6;i++)	//触发时间块
	{
		if(valve_last[i]==0&&ViceControlData.valve[i]==1)	//伸出触发
		{
			valve_startGOOD_time[i]=time_1ms_count;
		}
		else if(valve_last[i]==1&&ViceControlData.valve[i]==0)//收回触发
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
	
	for(int i=0;i<6;i++)	//反馈计算位
	{
		if(ViceControlData.valve[i]==1&&time_1ms_count-valve_startGOOD_time[i]>valve_GOODdelay[i])	//本数值为启动至到位延时，暂统一定为1000ms
		{
			valve_fdbstate[i]=1;
		}
		else if(ViceControlData.valve[i]==0&&time_1ms_count-valve_startPOOR_time[i]>valve_POORdelay[i])	//本数值为收回至到位延时，暂统一定为1000ms
		{
			valve_fdbstate[i]=0;
		}
		
		if(i<2)
		{
			if(ViceControlData.servo[i]==1&&time_1ms_count-servo_startGOOD_time[i]>servo_GOODdelay[i])	//本数值为启动至到位延时，暂统一定为1000ms
			{
				servo_fdbstate[i]=1;
			}
			else if(ViceControlData.servo[i]==0&&time_1ms_count-servo_startPOOR_time[i]>servo_POORdelay[i])	//本数值为收回至到位延时，暂统一定为1000ms
			{
				servo_fdbstate[i]=0;
			}
		}
	}
	
	for(int i=0;i<6;i++)	//迭代块
	{
		valve_last[i]=ViceControlData.valve[i];
		if(i<2)	servo_last[i]=ViceControlData.servo[i];
	}
////////////////////////////////////////////////////////////////////////////////////////////块结束标志
	
	
	//舵机执行块	//电磁阀在副板执行
	if(Replenish_Bullet_Statu!=1)	//非取弹模式
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
	else	//补弹模式舵机
	{
		pwm_l_t=1100;
		pwm_r_t=2100;
	}
	
	
	
	swicth_Last_state=RC_Ctl.rc.switch_right;
	
	PWM3_1=(u16)pwm_l_t;
  PWM3_2=(u16)pwm_r_t;
}


#define AUTOAIM_SPEENX	15	//向前压力
#define AUTOAIM_VOIDSPEEDY	150//130//110	//自动取块空时对位速度	//以向右为正方向
#define AUTOAIM_EXISTSPEEDY	120//75//55	//自动取块有障碍时速度
u8 AutoAimBullet_Task(s16* chassis_vx,s16* chassis_vy)	//自动对位任务
{
	static u8 aim_control_state_last=0;
	u8 aim_OK_statu=0;
	
	//移动状态辨识
	if(SensorData.Infrare[6]==0&&SensorData.Infrare[7]==1)	//[6]为左对位 [7]为右对位 0为有，1为无
	{	//向左移动状态
		AutoAimBulletData.relative_location=-1;
	}
	else if(SensorData.Infrare[6]==1&&SensorData.Infrare[7]==0)
	{
		AutoAimBulletData.relative_location=1;
	}
	else if(SensorData.Infrare[6]==1&&SensorData.Infrare[7]==1)
	{
		AutoAimBulletData.relative_location=0;
	}
	else if(SensorData.Infrare[6]==0&&SensorData.Infrare[7]==0)
	{
		AutoAimBulletData.relative_location=2;
	}
	
	if(aim_control_state_last==0&&AutoAimBulletData.control_state==1)
	{
		AutoAimBulletData.aim_state=0;
	}
	
	if(AutoAimBulletData.control_state==1)
	{
		//状态辨识之后移进来
		
		switch(AutoAimBulletData.aim_state)
		{
			case 0:	//空（需要移动）(向右移动)
			{
				*chassis_vx=AUTOAIM_SPEENX;
				*chassis_vy=AUTOAIM_VOIDSPEEDY;
				if(AutoAimBulletData.relative_location==-1)
				{
					AutoAimBulletData.aim_state=1;
				}
				else if(AutoAimBulletData.relative_location==1)
				{
					AutoAimBulletData.aim_state=2;
				}
				else if(AutoAimBulletData.relative_location==2)	//都有，速度减慢，方向不变
				{
					AutoAimBulletData.aim_state=4;
				}
				break;
			}
			case 1:	//向左移动
			{
				*chassis_vx=AUTOAIM_SPEENX;
				*chassis_vy=-AUTOAIM_EXISTSPEEDY;
				if(AutoAimBulletData.relative_location==0)
				{
					AutoAimBulletData.aim_state=3;
				}
				else if(AutoAimBulletData.relative_location==1)	//向右移动
				{
					AutoAimBulletData.aim_state=2;
				}
				break;
			}
			case 2:	//向右移动
			{
				*chassis_vx=AUTOAIM_SPEENX;
				*chassis_vy=AUTOAIM_EXISTSPEEDY;
				if(AutoAimBulletData.relative_location==0)
				{
					AutoAimBulletData.aim_state=3;
				}
				else if(AutoAimBulletData.relative_location==-1)	//向左移动
				{
					AutoAimBulletData.aim_state=1;
				}
				break;
			}
			case 3:	//空（可以取块）
			{
				*chassis_vx=0;
				*chassis_vy=0;
				aim_OK_statu=1;
				AutoAimBulletData.aim_state=0;
				break;
			}
			case 4:	//都有 和都空方向一致但速度减慢
			{
				*chassis_vx=AUTOAIM_SPEENX;
				*chassis_vy=AUTOAIM_EXISTSPEEDY;
				if(AutoAimBulletData.relative_location==-1)
				{
					AutoAimBulletData.aim_state=1;
				}
				else if(AutoAimBulletData.relative_location==1)
				{
					AutoAimBulletData.aim_state=2;
				}
				else if(AutoAimBulletData.relative_location==0)	//可以取块
				{
					AutoAimBulletData.aim_state=3;
				}
				break;
			}
		}
	}
	else
	{
		AutoAimBulletData.aim_state=0;	//清零
	}
	
	aim_control_state_last=AutoAimBulletData.control_state;
	
	return aim_OK_statu;
}


/********************************************************************
u8 SetCheck_TakeBullet_TakeBack_statu=0;	//切出取弹保护执行标志位
void SetCheck_TakeBullet_TakeBack(void)	//切出取弹机构回位保护
{
	if(SetCheck_TakeBullet_TakeBack_statu==1&&auto_takebullet_statu==0)//当状态为更新到1
	{
		ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
		ViceControlData.valve[VALVE_BULLET_PROTRACT]=0;
		
		if(valve_fdbstate[VALVE_BULLET_PROTRACT]==0)
		{
			SetCheck_FrontLift(0);//切出保护
			SetCheck_BackLift(0);
			if(SetCheck_FrontLift(0)==1&&SetCheck_BackLift(0)==1)
				SetCheck_TakeBullet_TakeBack_statu=0;
//			return 1;
		}
	}
//	return 0;
}



#define LIFT_DISTANCE_GRIPBULLET	490	//夹弹药箱时高度
#define LIFT_DISTANCE_DISGRIPBULLET	1400	//拔起来后弹药箱高度
#define LIFT_DISTANCE_SLOPEBACKBULLET	1400	//倾斜时后腿高度
#define LIFT_DISTANCE_SLOPEFRONTBULLET	1400	//倾斜时前腿高度
extern LIFT_DATA lift_Data;

u8 SetCheck_GripLift(u8 grip_state)	//是否与弹药箱平齐,grip抓住的意思	//0表示不抓住，即需要丢弹药箱或拔起弹药箱高度，1表示抓住，即需要夹紧弹药箱时的高度
{
	lift_Data.lf_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.rf_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.lb_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.rb_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_GRIPBULLET);
	
	return (abs(lift_Data.lf_lift_fdbP+lift_Data.rf_lift_fdbP-2*(LIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_GRIPBULLET)))<30);	//这里是仅以前两腿为反馈传回的
}

u8 SetCheck_SlopeLift(u8 slope_state)	//暂时只升后腿	slope倾斜的意思	//0表示不倾斜，即恢复到拔起弹药箱高度，1表示倾斜，即倾斜倒子弹状态
{
	lift_Data.lb_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(slope_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_SLOPEBACKBULLET);
	lift_Data.rb_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(slope_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_SLOPEBACKBULLET);
	
	lift_Data.lf_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(slope_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_SLOPEFRONTBULLET);
	lift_Data.rf_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(slope_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_SLOPEFRONTBULLET);
	
	return (abs(lift_Data.lb_lift_fdbP+lift_Data.rb_lift_fdbP-2*(LIFT_DISTANCE_DISGRIPBULLET-(slope_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_SLOPEBACKBULLET)))<30);	//这里是仅以前两腿为反馈传回的,因为前两腿为升高且行程更大时间更长，若前腿已到位则后腿一定到位
}


*****************************************************************/



