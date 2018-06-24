#include "main.h"
#include "image_control.h"

#define IMAGE_CUTLIST_CHASSIS 1
#define IMAGE_CUTLIST_TAKEBULLET 0
const u8 Image_CutList[2][2]={\
{0,0},\
{1,0},\
};	//分别为：补弹、底盘

extern u32 time_1ms_count;
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
extern RC_Ctl_t RC_Ctl;
extern ViceControlDataTypeDef ViceControlData;

#define STEER_IMAGE_INIT	1640
#define STEER_IMAGE_REVERSAL	510

#define IMAGE_START_DELAY	(1000*5)	//5s后开始
void Screen_Start(void)	//屏幕启动切换到AV信道
{
	if(KeyBoardData[KEY_G].value!=1)
	{
		if(time_1ms_count<IMAGE_START_DELAY)	//5s后开始
		{
			IMAGE_START=PWM_IO_ON;
		}
		else if(time_1ms_count>IMAGE_START_DELAY&&time_1ms_count<IMAGE_START_DELAY+1000)
		{
			IMAGE_START=PWM_IO_OFF;
		}
		else
		{
			IMAGE_START=PWM_IO_ON;
		}
	}
	else
	{
		IMAGE_START=PWM_IO_OFF;
	}
		

}

extern u8 Replenish_Bullet_Statu;	//补弹状态位
//u8 av_cut=0;
u8 Steer_Image_state=0;
u16 steer_image=STEER_IMAGE_REVERSAL;
void Image_Cut_Task(void)	//摄像头切换、舵机
{
	static u8 key_c_last=0;
	static WorkState_e State_Record=CHECK_STATE;
	static u8 replenish_bullet_statu_last=0;
//	t_AV_CUT=av_cut*20000;	//一直在上面
	
////////////////	if(State_Record!=NORMAL_STATE&&GetWorkState()==NORMAL_STATE)
////////////////	{
////////////////		Image_Cut_Screen(IMAGE_CUTLIST_TAKEBULLET);	//低电平
////////////////		Steer_Image_state=0;
////////////////	}
	
	if(State_Record!=TAKEBULLET_STATE&&GetWorkState()==TAKEBULLET_STATE)
	{
		Image_Cut_Screen(IMAGE_CUTLIST_TAKEBULLET);	//低电平
		Steer_Image_state=1;
	}
	else if(State_Record==TAKEBULLET_STATE&&GetWorkState()!=TAKEBULLET_STATE)
	{
		Image_Cut_Screen(IMAGE_CUTLIST_TAKEBULLET);	//低电平
		Steer_Image_state=0;
	}
	
	if(State_Record!=NORMAL_STATE&&GetWorkState()==NORMAL_STATE)
	{
		Image_Cut_Screen(IMAGE_CUTLIST_TAKEBULLET);	//低电平
		Steer_Image_state=0;
	}
	
	if(replenish_bullet_statu_last==0&&Replenish_Bullet_Statu==1)	//进入取弹模式
	{
		Image_Cut_Screen(IMAGE_CUTLIST_TAKEBULLET);	//低电平
		Steer_Image_state=1;
	}
	else if(replenish_bullet_statu_last==1&&Replenish_Bullet_Statu==0)
	{
		Image_Cut_Screen(IMAGE_CUTLIST_TAKEBULLET);	//低电平
		Steer_Image_state=0;
	}
	
	if((State_Record!=ASCEND_STATE&&GetWorkState()==ASCEND_STATE)||(State_Record!=DESCEND_STATE&&GetWorkState()==DESCEND_STATE))	//自动上下岛模式
	{
		Image_Cut_Screen(IMAGE_CUTLIST_CHASSIS);	//低电平
		Steer_Image_state=1;
	}
	else if((State_Record==ASCEND_STATE&&GetWorkState()!=ASCEND_STATE)||(State_Record==DESCEND_STATE&&GetWorkState()!=DESCEND_STATE))
	{
		Image_Cut_Screen(IMAGE_CUTLIST_CHASSIS);	//低电平
		Steer_Image_state=0;
	}
	
//	if(key_r_last==0&&KeyBoardData[KEY_R].value==1)
//	{
//		ViceControlData.image_cut[0]=0;	//低电平
//		Steer_Image_state=!Steer_Image_state;
//		Replenish_Bullet_Statu=Steer_Image_state;
//	}
	if(key_c_last==0&&KeyBoardData[KEY_C].value==1)
	{
		Steer_Image_state=!Steer_Image_state;
	}
	
	
	STEER_IMAGE=STEER_IMAGE_INIT-Steer_Image_state*(STEER_IMAGE_INIT-STEER_IMAGE_REVERSAL);
	
	State_Record=GetWorkState();
	replenish_bullet_statu_last=Replenish_Bullet_Statu;
	key_c_last=KeyBoardData[KEY_C].value;
}


void Image_Cut_Screen(u8 state)
{
//	Chassis_Control_Move_Reverse(Steer_Image_state,state);
	switch(state)
	{
		case IMAGE_CUTLIST_CHASSIS:
		{
			ViceControlData.image_cut[0]=Image_CutList[IMAGE_CUTLIST_CHASSIS][0];
			ViceControlData.image_cut[1]=Image_CutList[IMAGE_CUTLIST_CHASSIS][1];
		}
		case IMAGE_CUTLIST_TAKEBULLET:
		{
			ViceControlData.image_cut[0]=Image_CutList[IMAGE_CUTLIST_TAKEBULLET][0];
			ViceControlData.image_cut[1]=Image_CutList[IMAGE_CUTLIST_TAKEBULLET][1];
		}
	}
	
}

void Chassis_Control_Move_Reverse(u8 image_steer,u8 image_cut[])	//机器前进方向设置
{
	if(image_steer==0)
	{
		
	}
	else if(image_cut[0]==0)
	{
		
	}
	else if(image_cut[0]==1)
	{
		
	}
}

