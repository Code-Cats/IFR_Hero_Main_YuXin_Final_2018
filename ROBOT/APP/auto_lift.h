#ifndef __AUTO_LIFT_H__
#define __AUTO_LIFT_H__

#include "bsp.h"

u8 SetCheck_FrontLift(u8 rise_state);	//前升降轮升起/落下并检查	//0表示FALL，1表示ISLAND
u8 SetCheck_BackLift(u8 rise_state);

void Ascend_Control_Center(void);	//上岛逻辑控制中心
u8 Ascend_FullRise_GO1(void);	//前进、调整、触发蹬腿函数
u8 Ascend_BackFall_GO(void);	//3，4号升降抬起前进的进程
u8 Ascend_FullFall_GO(void);	//都抬起到都抬起后
u8 Ascend_FullRise_GO2(void);	//前进、调整、触发蹬腿函数

void Descend_Control_Center(void);	//下岛逻辑控制中心
u8 Descend_FullFall_Down(void);
u8 Descend_FrontRise_Down(void);
u8 Descend_FullRise_Down1(void);

s16 Chassis_Attitude_Correct(float fdbP,s16 fdbV);	//上岛姿态自校正函数

u8 Check_FrontLift(void);
u8 Check_BackLift(void);


typedef enum
{
    FULLRISE_GO1,  		//第一次全部升起
    BACKFALL_GO1,			//第一次收起后腿（车身前进坐标系）
		FULLFALL_GO1,			//第一次收起前腿（全部收起）
		FULLRISE_GO2,  		//第二次
    BACKFALL_GO2,			//第二次
		FULLFALL_GO2,			//第二次
}AscendState_e;

typedef enum
{
    FULLFALL_DOWN1,  		//第一次全部落下以准备下岛
    FRONTRISE_DOWM1,			//第一次伸出前腿以着地（车身前进坐标系）
		FULLRISE_DOWN1,			//第一次伸出后腿以着地（全部伸出）
		FULLFALL_DOWN2,  		//第二次
    FRONTRISE_DOWN2,			//第二次
		FULLRISE_DOWN2,			//第二次
}DescendState_e;

typedef enum
{
    CORRECT_CHASSIS_STATE,  		//矫正底盘
    CALI_SELF_STATE,			//校准自身目标位置	//未进入登下岛模式时，不断校准自身位置
}IslandAttitudeCorrectState_e;

void Set_Attitude_Correct_State(IslandAttitudeCorrectState_e state);	//设置矫正状态的函数

AscendState_e Island_State_Recognize(void);	//切入自动登岛状态自动辨识

#endif
