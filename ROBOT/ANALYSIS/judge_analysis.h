#ifndef __JUDGE_ANALYSIS_H_
#define __JUDGE_ANALYSIS_H_

#include "bsp.h"
void BulletNum_Calculate(void);
void Heat_MAX_COOL_calc(int* maxheat,float* coolheat,int maxhp);	//根据步兵最大血量推出等级，推出热量上限及冷却值

#endif
