#include "judge_analysis.h"

extern tGameRobotState         testGameRobotState;      //比赛机器人状态
/**************************************************
函数名称：BulletNum_Calculate
函数功能：计算子弹射速与打出子弹数目
输入参数：无
函数返回值：无
描述：根据裁判系统的枪口热量检测频率和冷却值可计算其射速；
      100ms之内可能打出好几发，做相应处理即可计算出射出子弹数目
***************************************************/
u8 Robot_Level=1;


int realBulletNum=0;	//发弹量
int speedCnt=0;
float speedReal=16;	//射速
float realBulletFreq=0.0;

int maxHeat=80;	//最高热量
float coolHeat=20;	//每秒冷却
		
void BulletNum_Calculate(void)
{
	static u8 secondSpeed[2]={0,0};
    speedCnt++;
	if(speedCnt==20)
	{ 
		  speedCnt=0;
		
		Heat_MAX_COOL_calc(&maxHeat,&coolHeat,testGameRobotState.maxHP);
		
		  secondSpeed[0]=secondSpeed[1];
		  secondSpeed[1]=testPowerHeatData.shooterHeat1;  //记录第0ms和第100ms的枪口热量	//Heat1为大子弹
		  if(secondSpeed[1]-secondSpeed[0]>0)             //判断发射出了子弹
		  {	
			    speedReal=secondSpeed[1]-secondSpeed[0];//+coolHeat; //根据裁判热量计算方式间接计算出子弹速度
				realBulletFreq=speedReal/40;              //记录子弹打出数量，100ms内可能打出好几发子弹
				if(abs(realBulletFreq-1)<0.5f) {speedReal=speedReal;realBulletNum=realBulletNum+1;}
				else if(abs(realBulletFreq-2)<0.5f) {speedReal=speedReal/2;realBulletNum=realBulletNum+2;}
				else if(abs(realBulletFreq-3)<0.5f) {speedReal=speedReal/3;realBulletNum=realBulletNum+3;}
				else if(abs(realBulletFreq-4)<0.5f) {speedReal=speedReal/4;realBulletNum=realBulletNum+4;}
				else if(abs(realBulletFreq-5)<0.5f) {speedReal=speedReal/5;realBulletNum=realBulletNum+5;}
			    else if(abs(realBulletFreq-6)<0.5f) {speedReal=speedReal/5;realBulletNum=realBulletNum+6;}
			    else if(abs(realBulletFreq-7)<0.5f) {speedReal=speedReal/5;realBulletNum=realBulletNum+7;}
			    }
		  }
         // if(PWM5==MAGAZINE_OPEN)  realBulletNum=0;
}


void Heat_MAX_COOL_calc(int* maxheat,float* coolheat,int maxhp)	//根据步兵最大血量推出等级，推出热量上限及冷却值
{
	if(maxhp==1500)
	{
		*maxheat=80;
		*coolheat=20;
		Robot_Level=1;
	}
	else if(maxhp==2500)
	{
		*maxheat=160;
		*coolheat=40;
		Robot_Level=2;
	}
	else if(maxhp==3500)
	{
		*maxheat=320;
		*coolheat=80;
		Robot_Level=3;
	}
	else
	{
		*maxheat=80;
		*coolheat=20;
		Robot_Level=1;
	}
}
