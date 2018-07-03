#ifndef __AUTO_TAKEBULLET_H
#define __AUTO_TAKEBULLET_H
#include "stm32f4xx.h"

#define VALVE_BULLET_HORIZONTAL1 0		//原登岛--现平移1
#define VALVE_BULLET_HORIZONTAL2 1	//原前伸--现平移2
#define VALVE_BULLET_CLAMP 2	//夹紧


void TakeBullet_Control_Center(void);

typedef enum
{
    BULLET_ACQUIRE,  		//前伸、夹紧、抬起动作	称之为获得过程
    BULLET_POUROUT,			//车身倾斜、舵机旋转	称之为倒弹过程
		BULLET_THROWOUT,			//舵机旋回、车身抬起、夹紧松开	称之为抛落过程
}TakeBulletState_e;


//u8 SetCheck_GripLift(u8 grip_state);	//是否与弹药箱平齐,grip抓住的意思	//0表示不抓住，即需要丢弹药箱或拔起弹药箱高度，1表示抓住，即需要夹紧弹药箱时的高度
//u8 SetCheck_SlopeLift(u8 slope_state);	//暂时只升后腿	slope倾斜的意思	//0表示不倾斜，即恢复到拔起弹药箱高度，1表示倾斜，即倾斜倒子弹状态
//void SetCheck_TakeBullet_TakeBack(void);	//切出取弹机构回位保护

#endif
