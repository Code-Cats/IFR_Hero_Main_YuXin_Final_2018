#include "lift.h"
#include "control.h"

extern LIFT_DATA lift_Data;
extern u32 time_1ms_count;

extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
extern u8 IMU_Check_Useless_State;	//陀螺仪失效检测位

#define AUTOCHASSIS_LIFT 12
void AutoChassisAttitude_Lift(float chassis_pitch_raw)	//自动调整姿态	//pitch正方向为前上	//注意放在lift_task前面
{
	static float chassis_pitch=0;
	static float ka=0.1f;
	static u16 steady_flat_count=0;

	chassis_pitch=chassis_pitch*(1-ka)+chassis_pitch_raw*ka;
	
	if(GetWorkState()==NORMAL_STATE)	//正常状态
	{
		
		if(abs(chassis_pitch)>8)	//8度阈值
		{
			steady_flat_count=0;
			
			lift_Data.lf_lift_tarP=-chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lf_lift_fdbP;
			lift_Data.rf_lift_tarP=-chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.rf_lift_fdbP;
			lift_Data.lb_lift_tarP=chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lb_lift_fdbP;
			lift_Data.rb_lift_tarP=chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.rb_lift_fdbP;
			
			if(lift_Data.lf_lift_fdbP!=lift_Data.rf_lift_fdbP)
			{
				s32 l_r_error=(lift_Data.lf_lift_fdbP-lift_Data.rf_lift_fdbP)/2;
				lift_Data.lf_lift_tarP-=l_r_error;
				lift_Data.rf_lift_tarP+=l_r_error;
			}
			
			if(lift_Data.lb_lift_fdbP!=lift_Data.rb_lift_fdbP)
			{
				s32 l_r_error=(lift_Data.lb_lift_fdbP-lift_Data.rb_lift_fdbP)/2;
				lift_Data.lb_lift_tarP-=l_r_error;
				lift_Data.rb_lift_tarP+=l_r_error;
			}
		}
		else
		{
			if(abs(lift_Data.lf_lift_tarP-lift_Data.lb_lift_tarP)<300)
			{
				if(steady_flat_count<0xFFFE)
				{
					steady_flat_count++;
				}    
			}
				
			if(abs(lift_Data.lf_lift_fdbP+lift_Data.rf_lift_fdbP-2*LIFT_DISTANCE_FALL)>18&&abs(lift_Data.lb_lift_fdbP+lift_Data.rb_lift_fdbP-2*LIFT_DISTANCE_FALL)>18)
			{
				if(time_1ms_count%40==0)	//1ms加的太快10MS
				{
					if(lift_Data.lf_lift_fdbP!=lift_Data.rf_lift_fdbP)
					{
						lift_Data.lf_lift_tarP=lift_Data.lf_lift_fdbP<lift_Data.rf_lift_fdbP?lift_Data.rf_lift_fdbP:lift_Data.rf_lift_fdbP;
						lift_Data.rf_lift_tarP=lift_Data.lf_lift_tarP;
					}
					
					if(lift_Data.lb_lift_fdbP!=lift_Data.rb_lift_fdbP)
					{
						lift_Data.lb_lift_tarP=lift_Data.lb_lift_fdbP<lift_Data.rb_lift_fdbP?lift_Data.lb_lift_fdbP:lift_Data.rb_lift_fdbP;
						lift_Data.rb_lift_tarP=lift_Data.lb_lift_tarP;
					}
					lift_Data.lf_lift_tarP-=5;
					lift_Data.rf_lift_tarP-=5;
					lift_Data.lb_lift_tarP-=5;
					lift_Data.rb_lift_tarP-=5;
				}
			}
			
		}
		
		if(steady_flat_count>800)
		{
			lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL;
			lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL;
			lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL;
			lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL;
		}
//		if(lift_Data.lf_lift_fdbP!=lift_Data.rf_lift_fdbP)
//		{
//			lift_Data.lf_lift_tarP=lift_Data.lf_lift_fdbP<lift_Data.rf_lift_fdbP?lift_Data.rf_lift_fdbP:lift_Data.rf_lift_fdbP;
//			lift_Data.rf_lift_tarP=lift_Data.lf_lift_tarP;
//		}
//		
//		if(lift_Data.lb_lift_fdbP!=lift_Data.rb_lift_fdbP)
//		{
//			lift_Data.lb_lift_tarP=lift_Data.lb_lift_fdbP<lift_Data.rb_lift_fdbP?lift_Data.lb_lift_fdbP:lift_Data.rb_lift_fdbP;
//			lift_Data.rb_lift_tarP=lift_Data.lb_lift_tarP;
//		}
		
		
		lift_Data.lf_lift_tarP=lift_Data.lf_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.lf_lift_tarP;	//限制行程
		lift_Data.lf_lift_tarP=lift_Data.lf_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.lf_lift_tarP;
		
		lift_Data.rf_lift_tarP=lift_Data.rf_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.rf_lift_tarP;	//限制行程
		lift_Data.rf_lift_tarP=lift_Data.rf_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.rf_lift_tarP;
		
		lift_Data.lb_lift_tarP=lift_Data.lb_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.lb_lift_tarP;	//限制行程
		lift_Data.lb_lift_tarP=lift_Data.lb_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.lb_lift_tarP;
		
		lift_Data.rb_lift_tarP=lift_Data.rb_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.rb_lift_tarP;	//限制行程
		lift_Data.rb_lift_tarP=lift_Data.rb_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.rb_lift_tarP;
	}
}



extern GYRO_DATA Gyro_Data;
	//经实验测得对于车身左边系来说在向前加速时后腿登起，说明向前加速时姿态误差为前仰后附，即正向误差
	//														向前减速时前腿登起，说明向前减速（加速度负向）时误差为负
	//经实验测得IMU_Uranus在芯片向前安放时向前加速度正方向为前，为acc第0位	//按照200加速度4°误差来计算，补偿系数为4/200
#define ATTITUDE_ACC_COMPENSATE (5.0f/200)	//加速度对于姿态补偿系数

#define TILT 1	//倾斜状态
#define STAEDY_REAL 0	//平稳状态
#define STAEDY_ADJUST 2	//经调整平稳状态
u8 Adjust_Statu=STAEDY_REAL;
void AutoChassisAttitude_Lift_V2(float chassis_pitch_raw)	//自动调整姿态	//pitch正方向为前上	//注意放在lift_task前面
{
	static float chassis_pitch=0;
	static float ka=0.05f;
	
//	chassis_pitch_raw-=Gyro_Data.acc[0]*ATTITUDE_ACC_COMPENSATE;	//对IMU_Uranus内置算法缺陷进行补偿
	
	chassis_pitch=chassis_pitch*(1-ka)+chassis_pitch_raw*ka;
	
	if(GetWorkState()==NORMAL_STATE&&KeyBoardData[KEY_CTRL].value==1&&IMU_Check_Useless_State==0)	//加入陀螺仪失效处理//加入CTRL控制   &&KeyBoardData[KEY_CTRL].value==1		//先不加入以便测试，后期必须加上
	{
		switch(Adjust_Statu)
		{
			case STAEDY_REAL:
			{
				static u16 tilt_change_count=0;	//若阈值检测效果不好，则使用消抖检测
				lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL;
				if(chassis_pitch>(8)&&tilt_change_count<0xFFFE)	//触发阈值	为7时有意外触发现象			+Gyro_Data.acc[0]*4.0f/200云台不水平影响较大
				{
					tilt_change_count++;
				}
				else if((chassis_pitch<(-8)&&tilt_change_count<0xFFFE))		//+Gyro_Data.acc[0]*4.0f/200
				{
					tilt_change_count++;
				}
				else
				{
					tilt_change_count=0;
				}
				
				if(tilt_change_count>100)
				{
					Adjust_Statu=TILT;
					tilt_change_count=0;
				}
				break;
			}
			case TILT:
			{
				static u16 staedy_adjust_count=0;
				if(chassis_pitch>0)	//前仰，可以通过下前解决，当下前无法解决，采用上后
				{
					if(lift_Data.lf_lift_tarP<=LIFT_DISTANCE_FALL)	//若下前至极限
					{
						lift_Data.lb_lift_tarP=chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lb_lift_fdbP;
						lift_Data.rb_lift_tarP=lift_Data.lb_lift_tarP;
					}
					else
					{
						lift_Data.lf_lift_tarP=-chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lf_lift_fdbP;
						lift_Data.rf_lift_tarP=lift_Data.lf_lift_tarP;	//以左前电机为基准
					}
					
				
				}
				else	//前俯，可以通过下后解决，若下后无法解决，采用上前
				{
					if(lift_Data.lb_lift_tarP<=LIFT_DISTANCE_FALL)	//下后
					{
						lift_Data.lf_lift_tarP=-chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lf_lift_fdbP;
						lift_Data.rf_lift_tarP=lift_Data.lf_lift_tarP;	//以左前电机为基准
					}
					else
					{
						lift_Data.lb_lift_tarP=chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lb_lift_fdbP;
						lift_Data.rb_lift_tarP=lift_Data.lb_lift_tarP;
					}
					
					
				}
				
//				lift_Data.lf_lift_tarP=-chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lf_lift_fdbP;
//				lift_Data.rf_lift_tarP=lift_Data.lf_lift_tarP;	//以左前电机为基准
//				lift_Data.lb_lift_tarP=chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lb_lift_fdbP;
//				lift_Data.rb_lift_tarP=lift_Data.lb_lift_tarP;
				
////////				if(chassis_pitch>0)	//前仰，可以通过下前解决，当下前无法解决，采用上后	待在此处写强制下降下前下后
////////				{
////////					if(abs(lift_Data.lf_lift_tarP-FALL)>)
////////				}
////////				else	//前俯，可以通过下后解决，若下后无法解决，采用上前
////////				{
////////					
////////				}
				
//				if(abs(lift_Data.lf_lift_tarP-FALL)>10&&abs(lift_Data.lb_lift_tarP-FALL)>10)	//未在重心最高点	无效
//				{
//					if(time_1ms_count%20==0)
//					{
//						lift_Data.lf_lift_tarP-=5;
//						lift_Data.rf_lift_tarP-=5;
//						lift_Data.lb_lift_tarP-=5;
//						lift_Data.rb_lift_tarP-=5;
//					}
//				}
				
				if(abs(chassis_pitch)<2.2f&&staedy_adjust_count<0xFFFE)
				{
					staedy_adjust_count++;
				}
				else
				{
					staedy_adjust_count=0;
				}
				
				if(staedy_adjust_count>300)	//稳定了	//待调整
				{
					Adjust_Statu=STAEDY_ADJUST;
					staedy_adjust_count=0;
				}
				break;
			}
			case STAEDY_ADJUST:
			{
				static u16 staedy_real_count=0;
				if(abs(chassis_pitch)>4)
				{
					Adjust_Statu=TILT;
				}
				else
				{
					if(abs(lift_Data.lf_lift_tarP-lift_Data.lb_lift_tarP)<300&&staedy_real_count<0xFFFE)
					{
						staedy_real_count++;
					}
				}
				
				if(staedy_real_count>150)	//0.3s
				{
					Adjust_Statu=STAEDY_REAL;
					staedy_real_count=0;
				}
				break;
			}
		}	
	}
	
	
	lift_Data.lf_lift_tarP=lift_Data.lf_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.lf_lift_tarP;	//限制行程
	lift_Data.lf_lift_tarP=lift_Data.lf_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.lf_lift_tarP;
	
	lift_Data.rf_lift_tarP=lift_Data.rf_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.rf_lift_tarP;	//限制行程
	lift_Data.rf_lift_tarP=lift_Data.rf_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.rf_lift_tarP;
	
	lift_Data.lb_lift_tarP=lift_Data.lb_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.lb_lift_tarP;	//限制行程
	lift_Data.lb_lift_tarP=lift_Data.lb_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.lb_lift_tarP;
	
	lift_Data.rb_lift_tarP=lift_Data.rb_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.rb_lift_tarP;	//限制行程
	lift_Data.rb_lift_tarP=lift_Data.rb_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.rb_lift_tarP;
}


/***********************************************************
对于V2版本，出现回中时重心升高问题解决思想
1.发生后及时处理，在检测到轮平面低于一阈值时及时降到最低平面（方案阈值检测鲁棒性延迟问题）
2.在发生阶段避免，当pitch>0,车身前仰，在TITL调整阶段
	无需上升的不上升，可通过下降解决的全通过下降解决
	
***********************************************************/

