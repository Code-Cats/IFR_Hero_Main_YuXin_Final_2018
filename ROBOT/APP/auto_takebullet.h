#ifndef __AUTO_TAKEBULLET_H
#define __AUTO_TAKEBULLET_H
#include "stm32f4xx.h"

#define VALVE_BULLET_HORIZONTAL1 0		//原登岛--现平移1
#define VALVE_BULLET_HORIZONTAL2 1	//原前伸--现平移2
#define VALVE_BULLET_CLAMP 2	//夹紧


void TakeBullet_Control_Center(void);

typedef enum
{
    BULLET_ACQUIRE1,  		//前伸、夹紧、抬起动作	称之为获得过程
    BULLET_POUROUT1,			//车身倾斜、舵机旋转	称之为倒弹过程
		BULLET_THROWOUT1,			//舵机旋回、车身抬起、夹紧松开	称之为抛落过程
		BULLET_ACQUIRE2,  		//2前伸、夹紧、抬起动作	称之为获得过程2
    BULLET_POUROUT2,			//2车身倾斜、舵机旋转	称之为倒弹过程2
		BULLET_THROWOUT2,			//2舵机旋回、车身抬起、夹紧松开	称之为抛落过程2
		BULLET_WAITING,	//待命状态
		BULLET_OTHER,	//非取弹状态
}TakeBulletState_e;

typedef struct
{
	u8 control_state;	//自动对位状态控制位	当其等于1开始自动对位且底盘停止RC输入以让自动程序运行
	u8 take_count;	//虚拟计算的弹药箱数量(定于与英雄兼容)
	s8 relative_location;	//相对位置-1表示需要向左移动（右遮(0)左无(1)） 相对位置+1表示需要向右移动（右无(1)左遮(0)）	0表示前方皆空（1需要移动，2正好）	其他表示前方都被遮住（需要移动）
	u8 aim_state;	//一套完整流程需要switch
}AutoAimBulletTypeDef;	//自动对位

u8 AutoAimBullet_Task(s16* chassis_vx,s16* chassis_vy);	//自动对位任务
//u8 SetCheck_GripLift(u8 grip_state);	//是否与弹药箱平齐,grip抓住的意思	//0表示不抓住，即需要丢弹药箱或拔起弹药箱高度，1表示抓住，即需要夹紧弹药箱时的高度
//u8 SetCheck_SlopeLift(u8 slope_state);	//暂时只升后腿	slope倾斜的意思	//0表示不倾斜，即恢复到拔起弹药箱高度，1表示倾斜，即倾斜倒子弹状态
//void SetCheck_TakeBullet_TakeBack(void);	//切出取弹机构回位保护

#endif
