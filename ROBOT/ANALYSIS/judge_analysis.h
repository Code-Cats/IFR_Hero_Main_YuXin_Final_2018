#ifndef __JUDGE_ANALYSIS_H_
#define __JUDGE_ANALYSIS_H_

#include "bsp.h"

typedef struct
{
	uint16_t bullet_num;	//发弹量	直接放在射速中断中自加
	s16 heat;
	u16 maxheat;
}RobotHeatDataSimuTypeDef;	//机器热量仿真-基于射速更新

void BulletNum_Calculate(void);
void Heat_MAX_COOL_calc(int* maxheat,float* coolheat,int maxhp);	//根据步兵最大血量推出等级，推出热量上限及冷却值

void Heat_Simulating(u16 heating,u8 level);	//热量仿真//运行频率1-10HZ
void BulletNum_Simu_ADD(void);
#endif
