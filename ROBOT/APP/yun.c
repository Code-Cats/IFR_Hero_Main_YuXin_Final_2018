#include "yun.h"
#include "usart1_remote_analysis.h"
s32 YAW_INIT=YAW_INIT_DEFINE;
/*
整体结构：yaw轴暂定单独速度环//后期计划增加外接陀螺仪位置环进行选择
pitch轴位置环

经过控制信息处理，传入参数为
yaw速度？或者位置
pitch位置


*/
YUN_MOTOR_DATA 			yunMotorData=YUN_MOTOR_DATA_DEFAULT;
YUN_DATA          	yunData=YUN_DATA_DEFAULT;

PID_GENERAL          PID_PITCH_POSITION=PID_PITCH_POSITION_DEFAULT;
PID_GENERAL          PID_PITCH_SPEED=PID_PITCH_SPEED_DEFAULT;
PID_GENERAL          PID_YAW_POSITION=PID_YAW_POSITION_DEFAULT;
PID_GENERAL          PID_YAW_SPEED=PID_YAW_SPEED_DEFAULT;

extern  FIRST_ORDER_FILTER   FILTER_MOUSE_YAW;
extern  FIRST_ORDER_FILTER   FILTER_WAIST_YAW;
extern  MPU6050_REAL_DATA    MPU6050_Real_Data;
extern	RC_Ctl_t RC_Ctl;
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
extern GYRO_DATA Gyro_Data;
extern u8 Chassis_Follow_Statu;	//底盘跟随标志位
extern volatile float yaw_follow_real_error;	//扭腰时的底盘跟随偏差
extern float yaw_follow_error;	//普通时的底盘跟随误差

extern u8 Replenish_Bullet_Statu;	//补弹状态位

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1*/
extern u8 IMU_Check_Useless_State;	//陀螺仪失效检测位
/*！！！！！！！！！！！！！！！！！！！！*/

extern u32 time_1ms_count;
s32 t_pitch____=0;
s32 t_yaw___=0;
void Yun_Task(void)	//云台控制任务 
{
	if(IMU_Check_Useless_State==0)
	{
		Yun_Control_External_Solution();	//正常的位置环
	}
	else
	{
		Yun_Control_Inscribe_Solution();	//当陀螺仪位置反馈崩了，就采用速度环控制
	}
}

u8 Yun_WorkState_Turn180_statu=0;	//180旋转到位标志位，放在了上面
u8 Yun_Control_RCorPC=RC_CONTROL;
u8 yun_control_pcorrc_last=RC_CONTROL;	//记录上一次控制模式，便于在切换时对某些数据进行处理	//这里时架构问题，更改架构可以不用该变量
void Yun_Control_External_Solution(void)	//外置反馈方案
{
	if(GetWorkState()!=PREPARE_STATE&&GetWorkState()!=CALI_STATE)	//模式切换
	{
		if(RC_Ctl.mouse.press_l==1||RC_Ctl.mouse.press_r==1||RC_Ctl.mouse.x>1||RC_Ctl.mouse.y>1)
		{
			Yun_Control_RCorPC=PC_CONTROL;
		}
		else if(abs(RC_Ctl.rc.ch2-1024)>3||abs(RC_Ctl.rc.ch3-1024)>3)
		{
			Yun_Control_RCorPC=RC_CONTROL;
		}
	}
	
//	Yun_WorkState_Turn_Task();	//新版无需取弹//取弹时云台转向标志位
	
	if(GetWorkState()==NORMAL_STATE||GetWorkState()==WAIST_STATE)	//仅在正常模式下受控	//取弹受控为暂时加入，之后以传感器自动进行	//取弹受控已取消，云台跟随底盘
	{
		if(Yun_Control_RCorPC==PC_CONTROL)
		{	//PC控制数据
			PC_Control_Yun(&yunMotorData.yaw_tarP,&yunMotorData.pitch_tarP);
		}
		else if(Yun_Control_RCorPC==RC_CONTROL)
		{	//RC控制数据
			RC_Control_Yun(&yunMotorData.yaw_tarP,&yunMotorData.pitch_tarP);
		}
	}
	
	yun_control_pcorrc_last=Yun_Control_RCorPC;
	
	if(GetWorkState()==TAKEBULLET_STATE)	//新版取消了旋转180//取弹模式且已经转了180°一直校准		//后续取消180°
	{
		yunMotorData.yaw_tarP=(Gyro_Data.angle[YAW]*10+(YAW_INIT-yunMotorData.yaw_fdbP)*3600/8192);	//反馈放大10倍并将目标位置置为中点
	}
	else if(GetWorkState()==NORMAL_STATE&&Replenish_Bullet_Statu==1)
	{
		yunMotorData.yaw_tarP=(Gyro_Data.angle[YAW]*10+(YAW_INIT-yunMotorData.yaw_fdbP)*3600/8192);	//反馈放大10倍并将目标位置置为中点
	}

	/////////////////////
//////////////	if(KeyBoardData[KEY_F].value==1)	//放在中断中运行
//////////////	{
//////////////		Vision_Task(&yunMotorData.yaw_tarP,&yunMotorData.pitch_tarP);
//////////////	}
	
	////////////////////////////
	
	yunMotorData.pitch_tarV=PID_General(yunMotorData.pitch_tarP,(yunMotorData.pitch_fdbP),&PID_PITCH_POSITION);
		
	if(yunMotorData.yaw_tarP-Gyro_Data.angle[YAW]*10>1800)	//过零点
	{
		yunMotorData.yaw_tarV=PID_General(yunMotorData.yaw_tarP,Gyro_Data.angle[YAW]*10+3600,&PID_YAW_POSITION);
	}
	else if(yunMotorData.yaw_tarP-Gyro_Data.angle[YAW]*10<-1800)
	{
		yunMotorData.yaw_tarV=PID_General(yunMotorData.yaw_tarP,Gyro_Data.angle[YAW]*10-3600,&PID_YAW_POSITION);
	}
	else
	{
		yunMotorData.yaw_tarV=PID_General(yunMotorData.yaw_tarP,Gyro_Data.angle[YAW]*10,&PID_YAW_POSITION);
	}
	
//	if(KeyBoardData[KEY_SHIFT].value==1)	//分区赛版本扩展云台范围
//	{
//		Yun_Pitch_Extension(yunMotorData.pitch_tarP);
//	}
	
	
	yunMotorData.pitch_output=-PID_General(yunMotorData.pitch_tarV,(Gyro_Data.angvel[PITCH]),&PID_PITCH_SPEED);
	yunMotorData.yaw_output=-PID_General(yunMotorData.yaw_tarV,(Gyro_Data.angvel[YAW]),&PID_YAW_SPEED);	//采用外界陀螺仪做反馈
}




void RC_Control_Yun(float * yaw_tarp,float * pitch_tarp)	//1000Hz
{
	if(time_1ms_count%15==0)	//66.67hz
	{
		yunMotorData.yaw_tarP+=((RC_Ctl.rc.ch2-1024)*35.0/660.0);
		yunMotorData.yaw_tarP=yunMotorData.yaw_tarP>1800?yunMotorData.yaw_tarP-3600:yunMotorData.yaw_tarP;	//过零点
		yunMotorData.yaw_tarP=yunMotorData.yaw_tarP<-1800?yunMotorData.yaw_tarP+3600:yunMotorData.yaw_tarP;	//过零点
	}
	
	yunMotorData.pitch_tarP=((RC_Ctl.rc.ch3-1024)*460.0/660.0)+PITCH_INIT;	//-50是因为陀螺仪水平时云台上扬
}


#define YUN_UPMAX 660	//正常的活动范围，UP为负
#define YUN_DOWNMAX 460	//正常的活动范围，DOWN为正

#define YUN_UPMAX_EXTENSION (YUN_UPMAX+200)	//补偿的活动范围，UP为负
#define YUN_DOWNMAX_EXTENSION (YUN_DOWNMAX+200)	//补偿的活动范围，DOWN为正
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
void PC_Control_Yun(float * yaw_tarp,float * pitch_tarp)	//1000Hz	
{
	static float yaw_tarp_float=0;
	static float pitch_tarp_float=PITCH_INIT;
	static u8 start_state=0;	//初始位置

	if(yun_control_pcorrc_last==RC_CONTROL&&Yun_Control_RCorPC==PC_CONTROL)
	{
		yaw_tarp_float=(float)*yaw_tarp;
	}
	
	if(start_state==0)	//初始时记录
	{
		yaw_tarp_float=(float)*yaw_tarp;
		start_state=1;
	}
	static u8 keyQ_last,keyE_last=0;	//暂时屏蔽
	if(keyQ_last==0&&KeyBoardData[KEY_Q].value==1&&abs(yaw_follow_error)<PI/10)
	{
		yaw_tarp_float+=900;
		yaw_tarp_float=yaw_tarp_float>1800?yaw_tarp_float-3600:yaw_tarp_float;	//过零点
		yaw_tarp_float=yaw_tarp_float<-1800?yaw_tarp_float+3600:yaw_tarp_float;	//过零点
	}
	keyQ_last=KeyBoardData[KEY_Q].value;
	
	if(keyE_last==0&&KeyBoardData[KEY_E].value==1&&abs(yaw_follow_error)<PI/10)
	{
		yaw_tarp_float-=900;
		yaw_tarp_float=yaw_tarp_float>1800?yaw_tarp_float-3600:yaw_tarp_float;	//过零点
		yaw_tarp_float=yaw_tarp_float<-1800?yaw_tarp_float+3600:yaw_tarp_float;	//过零点
	}
	keyE_last=KeyBoardData[KEY_E].value;
	
	if(time_1ms_count%10==0)
	{
		yaw_tarp_float+=RC_Ctl.mouse.x*15.0f/40.0f;
		pitch_tarp_float-=RC_Ctl.mouse.y*2.0f/3.0f;	//2/4
		
		yaw_tarp_float=yaw_tarp_float>1800?yaw_tarp_float-3600:yaw_tarp_float;	//过零点
		yaw_tarp_float=yaw_tarp_float<-1800?yaw_tarp_float+3600:yaw_tarp_float;	//过零点
		
		if(KeyBoardData[KEY_SHIFT].value!=1)
		{
			pitch_tarp_float=pitch_tarp_float>(PITCH_INIT+YUN_DOWNMAX)?(PITCH_INIT+YUN_DOWNMAX):pitch_tarp_float;	//限制行程
			pitch_tarp_float=pitch_tarp_float<(PITCH_INIT-YUN_UPMAX)?(PITCH_INIT-YUN_UPMAX):pitch_tarp_float;	//限制行程
		}
		else	//shift模式
		{
			pitch_tarp_float=pitch_tarp_float>(PITCH_INIT+YUN_DOWNMAX_EXTENSION)?(PITCH_INIT+YUN_DOWNMAX_EXTENSION):pitch_tarp_float;	//限制行程
			pitch_tarp_float=pitch_tarp_float<(PITCH_INIT-YUN_UPMAX_EXTENSION)?(PITCH_INIT-YUN_UPMAX_EXTENSION):pitch_tarp_float;	//限制行程
		}
		
		
		*yaw_tarp=yaw_tarp_float;
		*pitch_tarp=pitch_tarp_float;
	}
}



/********************************************************************************************/
void Yun_Control_Inscribe_Solution(void)	//当陀螺仪崩了时单速度反馈方案
{
	if(GetWorkState()!=PREPARE_STATE&&GetWorkState()!=CALI_STATE)	//模式切换
	{
		if(RC_Ctl.mouse.press_l==1||RC_Ctl.mouse.press_r==1||RC_Ctl.mouse.x>1||RC_Ctl.mouse.y>1)
		{
			Yun_Control_RCorPC=PC_CONTROL;
		}
		else if(abs(RC_Ctl.rc.ch2-1024)>3||abs(RC_Ctl.rc.ch3-1024)>3)
		{
			Yun_Control_RCorPC=RC_CONTROL;
		}
	}
	
	
	if(Yun_Control_RCorPC==PC_CONTROL)
	{	//PC控制方案
		static float pitch_tarp_float=PITCH_INIT;
		
		if(time_1ms_count%10==0)
		{
			pitch_tarp_float+=RC_Ctl.mouse.y*2.0f/4.0f;	//2/4
			
			if(KeyBoardData[KEY_SHIFT].value!=1)
			{
				pitch_tarp_float=pitch_tarp_float>(PITCH_INIT+YUN_DOWNMAX)?(PITCH_INIT+YUN_DOWNMAX):pitch_tarp_float;	//限制行程
				pitch_tarp_float=pitch_tarp_float<(PITCH_INIT-YUN_UPMAX)?(PITCH_INIT-YUN_UPMAX):pitch_tarp_float;	//限制行程
			}
			else	//shift模式
			{
				pitch_tarp_float=pitch_tarp_float>(PITCH_INIT+YUN_DOWNMAX_EXTENSION)?(PITCH_INIT+YUN_DOWNMAX_EXTENSION):pitch_tarp_float;	//限制行程
				pitch_tarp_float=pitch_tarp_float<(PITCH_INIT-YUN_UPMAX_EXTENSION)?(PITCH_INIT-YUN_UPMAX_EXTENSION):pitch_tarp_float;	//限制行程
			}
			
			yunMotorData.pitch_tarP=pitch_tarp_float;	//赋值位置
		}
		
		yunMotorData.yaw_tarV=RC_Ctl.mouse.x*5;	//赋值速度
	}
	
	
	if(Yun_Control_RCorPC==RC_CONTROL)
	{	//RC控制方案
		if(time_1ms_count%15==0)	//66.67hz
		{
			yunMotorData.yaw_tarV=((RC_Ctl.rc.ch2-1024)*300.0/660.0);
		}
		
		yunMotorData.pitch_tarP=((RC_Ctl.rc.ch3-1024)*460.0/660.0)+PITCH_INIT;	//-50是因为陀螺仪水平时云台上扬
	}
	
	
//	Yun_WorkState_Turn_Task();	//取弹时云台转向标志位	//内部有对于陀螺仪状态的区别
	
	yunMotorData.pitch_tarV=PID_General(yunMotorData.pitch_tarP,(yunMotorData.pitch_fdbP),&PID_PITCH_POSITION);	//pitch位置环
	
//	if(KeyBoardData[KEY_SHIFT].value==1)	//分区赛版本扩展云台范围
//	{
//		Yun_Pitch_Extension(yunMotorData.pitch_tarP);	//pitch扩展
//	}
	
	yunMotorData.pitch_output=-PID_General(yunMotorData.pitch_tarV,(Gyro_Data.angvel[PITCH]),&PID_PITCH_SPEED);
	yunMotorData.yaw_output=-PID_General(yunMotorData.yaw_tarV,(Gyro_Data.angvel[YAW]),&PID_YAW_SPEED);	//采用外界陀螺仪做反馈
}



/********************************老板英雄*******************************
extern LIFT_DATA lift_Data;
*******************************
云台电机反馈同陀螺仪：下为正，上为负
********************************
#define PITCH_EXTENSION_UP 550
#define PITCH_EXTENSION_DOWN	550
#define PITCH_EXTENSION_TRIGGER_UP	620	//实际活动范围：860
#define PITCH_EXTENSION_TRIGGER_DOWN	450	//实际活动范围：650
void Yun_Pitch_Extension(float pitch_tar)	//pitch轴扩展范围
{
	if(pitch_tar>PITCH_INIT+PITCH_EXTENSION_TRIGGER_DOWN)	//向下触发,即后腿升起
	{
		lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL;
		lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL;
		lift_Data.lb_lift_tarP=PITCH_EXTENSION_DOWN;
		lift_Data.rb_lift_tarP=PITCH_EXTENSION_DOWN;
	}
	else if(pitch_tar<PITCH_INIT-PITCH_EXTENSION_TRIGGER_UP)	//向上触发
	{
		lift_Data.lf_lift_tarP=PITCH_EXTENSION_UP;
		lift_Data.rf_lift_tarP=PITCH_EXTENSION_UP;
		lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL;
		lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL;
	}
}
**********************************************************************/

//u8 Yun_WorkState_Turn180_statu=0;	//180旋转到位标志位，放在了上面
void Yun_WorkState_Turn_Task(void)	//模式切换时云台转向任务	//当转向完成标志位为1时切换到云台跟随底盘
{
	static WorkState_e State_Record=CHECK_STATE;
	static u32 time_start_record=0;
	
	if(IMU_Check_Useless_State==0)	//陀螺仪正常
	{
		if(State_Record!=TAKEBULLET_STATE&&GetWorkState()==TAKEBULLET_STATE)
		{
			time_start_record=time_1ms_count;
			Yun_WorkState_Turn180_statu=0;
			yunMotorData.yaw_tarP-=1800;
			yunMotorData.yaw_tarP=yunMotorData.yaw_tarP>1800?yunMotorData.yaw_tarP-3600:yunMotorData.yaw_tarP;	//过零点
			yunMotorData.yaw_tarP=yunMotorData.yaw_tarP<-1800?yunMotorData.yaw_tarP+3600:yunMotorData.yaw_tarP;	//过零点
		}
		else if(State_Record==TAKEBULLET_STATE&&GetWorkState()!=TAKEBULLET_STATE)
		{
			Yun_WorkState_Turn180_statu=0;
		}
		
		if(abs(yunMotorData.yaw_tarP-Gyro_Data.angle[2]*10)<2&&time_1ms_count-time_start_record>1500&&time_start_record!=0)	//2度范围认为到位
		{
			Yun_WorkState_Turn180_statu=1;	//置为1
			time_start_record=0;	//5.21新增
		}
	}
	else	//陀螺仪异常
	{
		if(State_Record!=TAKEBULLET_STATE&&GetWorkState()==TAKEBULLET_STATE)
		{
			time_start_record=time_1ms_count;
			yunMotorData.yaw_tarV=260;
			Yun_WorkState_Turn180_statu=0;
		}
		else if(State_Record==TAKEBULLET_STATE&&GetWorkState()!=TAKEBULLET_STATE)
		{
			Yun_WorkState_Turn180_statu=0;
		}
		
		if(time_start_record!=0&&time_1ms_count-time_start_record<900)	//如果激活
		{
			yunMotorData.yaw_tarV=260;
		}
		
		if(time_1ms_count-time_start_record>900&&time_start_record!=0)	//2度范围认为到位
		{
			yunMotorData.yaw_tarV=0;
			Yun_WorkState_Turn180_statu=1;	//置为1
			time_start_record=0;
		}
		
	}
	
	State_Record=GetWorkState();
}

float yaw_move_optimize_PC(s16 mouse_x,s16 mouse_y)	//对鼠标处理
{
	return 0;
}

//s16 __t_yaw_offset=0;
//	void __yun_yaw_offset(void)
//	{
//	static s32 Last_V_yaw=0;
//	static float a=0.05f;
//	__t_yaw_offset=(s32)(Last_V_yaw*(1-a)+yunMotorData.yaw_output*a);
//	Last_V_yaw=__t_yaw_offset;
//	}

s32 yun_pitch_tarV(s32 now_V)
{
	static s32 Last_V=0;
	static s32 now_acc_V=0;
	static float a=0.97f;
	now_acc_V=(s32)(Last_V*(1-a)+now_V*a);
	Last_V=now_acc_V;
	return now_acc_V;
}



/***************************************
函数名称：Yaw_Angle_Calculate
函数功能：通过当前机械角度与中值机械角度比较得到实际差角
函数参数：当前机械角度：src_angle
          中值机械角度：Init
函数返回值：实际差角：output
描述：无
****************************************/
float Yaw_Angle_Calculate(int16_t src_angle,int16_t Init)
{
    float output=-(float)(src_angle-Init)/8192*2*PI;	
	  return output;
}
//记录补偿值曲线
#define YAW_OFFSET_COUNT 11
const s32 YAW_OFFSET_VALUE[YAW_OFFSET_COUNT][2]=\
{\
	{5310,-620},\
	{5200,-530},\
	{5100,-475},\
	{5000,-390},\
	{4900,-110},\
	{4800,70},\
	{4700,280},\
	{4600,400},\
	{4500,470},\
	{4400,530},\
	{4300,573},\
};	//3.6测得

//const s32 YAW_OFFSET_VALUE[YAW_OFFSET_COUNT][2]=\
{\
	{5310,-1000},\
	{5180,-800},\
	{5000,-700},\
	{4910,-600},\
	{4800,-350},\
	{4710,0},\
	{4600,20},\
	{4500,240},\
	{4450,420},\
};	//旧

#define PITCH_OFFSET_COUNT 12
const s32 PITCH_OFFSET_VALUE[PITCH_OFFSET_COUNT][2]=\
{\
	{6600,-3500},\
	{6500,-2800},\
	{6400,-2500},\
	{6300,-2300},\
	{6200,-1920},\
	{6100,-1465},\
	{6000,-1300},\
	{5900,-1186},\
	{5800,-1200},\
	{5700,-1000},\
	{5600,-900},\
	{5500,-800},\
};	//原版，过大
//const s32 PITCH_OFFSET_VALUE[PITCH_OFFSET_COUNT][2]=\
{\
	{6600,-3000},\
	{6500,-2400},\
	{6400,-2100},\
	{6300,-1900},\
	{6200,-1750},\
	{6100,-1400},\
	{6000,-1200},\
	{5900,-1186},\
	{5800,-1100},\
	{5700,-1000},\
	{5600,-900},\
	{5500,-800},\
};

s32 Yaw_output_offset(s32 yaw_fbdP)	//克服云台yaw轴非线性力及非对称性的补偿 //虽然yaw云台阻尼曲线满足收敛，但因为参照物并非为云台电机，故应采用当前反馈位置做参照
{
	s32 offset=0;
	int i=0;
	
	yaw_fbdP=yaw_fbdP>YAW_OFFSET_VALUE[0][0]?YAW_OFFSET_VALUE[0][0]:yaw_fbdP;
	yaw_fbdP=yaw_fbdP<YAW_OFFSET_VALUE[YAW_OFFSET_COUNT-1][0]?YAW_OFFSET_VALUE[YAW_OFFSET_COUNT-1][0]:yaw_fbdP;
	
	for(i=0;i<YAW_OFFSET_COUNT;i++)	//遍历数组寻找位置
	{
		if(yaw_fbdP>=YAW_OFFSET_VALUE[i][0]) break;
	}
	
	i=i>YAW_OFFSET_COUNT-2?YAW_OFFSET_COUNT-2:i;	//限制到倒数第二个元素的位置，以免下一步运算数组越界
	
	offset=YAW_OFFSET_VALUE[i][1]+(YAW_OFFSET_VALUE[i+1][1]-YAW_OFFSET_VALUE[i][1])*(YAW_OFFSET_VALUE[i][0]-yaw_fbdP)/(YAW_OFFSET_VALUE[i][0]-YAW_OFFSET_VALUE[i+1][0]);
	return offset;
}

s16 Pitch_output_offset(s32 pitch_tarP)	//克服云台pitch轴非线性力及非对称性的补偿	//因为云台pitch阻尼曲线满足收敛（在外部激励情况下只存在一个最小值），故采用tarP作为补偿参照可以提高间接反应速度
{
	s16 offset=0;
//	int i=0;
//	
//	pitch_tarP=pitch_tarP>PITCH_OFFSET_VALUE[0][0]?PITCH_OFFSET_VALUE[0][0]:pitch_tarP;
//	pitch_tarP=pitch_tarP<PITCH_OFFSET_VALUE[PITCH_OFFSET_COUNT-1][0]?PITCH_OFFSET_VALUE[PITCH_OFFSET_COUNT-1][0]:pitch_tarP;
//	
//	for(i=0;i<PITCH_OFFSET_COUNT;i++)	//遍历数组寻找位置
//	{
//		if(pitch_tarP>=PITCH_OFFSET_VALUE[i][0]) break;
//	}
//	
//	i=i>PITCH_OFFSET_COUNT-2?PITCH_OFFSET_COUNT-2:i;	//限制到倒数第二个元素的位置，以免下一步运算数组越界
//	
//	offset=PITCH_OFFSET_VALUE[i][1]+(PITCH_OFFSET_VALUE[i+1][1]-PITCH_OFFSET_VALUE[i][1])*(PITCH_OFFSET_VALUE[i][0]-pitch_tarP)/(PITCH_OFFSET_VALUE[i][0]-PITCH_OFFSET_VALUE[i+1][0]);
	return offset;
}




