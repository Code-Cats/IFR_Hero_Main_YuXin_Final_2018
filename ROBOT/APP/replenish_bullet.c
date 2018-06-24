#include "replenish_bullet.h"

extern u32 time_1ms_count;
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
extern RC_Ctl_t RC_Ctl;
extern ViceControlDataTypeDef ViceControlData;


u8 Replenish_Bullet_Statu=0;
void Replenish_Bullet_Task(u8 key_r_state)	//从补给站补弹
{
	static u8 key_r_state_last;
	static u8 replenish_bullet_statu_last=0;

	if(key_r_state_last==0&&key_r_state==1)	//按下R键
	{
		Replenish_Bullet_Statu=!Replenish_Bullet_Statu;
	}
	
	replenish_bullet_statu_last=Replenish_Bullet_Statu;
	key_r_state_last=key_r_state;
}


