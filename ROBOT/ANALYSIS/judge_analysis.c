#include "judge_analysis.h"

extern tGameRobotState         testGameRobotState;      //����������״̬
/**************************************************
�������ƣ�BulletNum_Calculate
�������ܣ������ӵ����������ӵ���Ŀ
�����������
��������ֵ����
���������ݲ���ϵͳ��ǹ���������Ƶ�ʺ���ȴֵ�ɼ��������٣�
      100ms֮�ڿ��ܴ���ü���������Ӧ�����ɼ��������ӵ���Ŀ
***************************************************/
u8 Robot_Level=1;


int realBulletNum=0;	//������
int speedCnt=0;
float speedReal=16;	//����
float realBulletFreq=0.0;

int maxHeat=80;	//�������
float coolHeat=20;	//ÿ����ȴ
		
void BulletNum_Calculate(void)
{
	static u8 secondSpeed[2]={0,0};
    speedCnt++;
	if(speedCnt==20)
	{ 
		  speedCnt=0;
		
		Heat_MAX_COOL_calc(&maxHeat,&coolHeat,testGameRobotState.maxHP);
		
		  secondSpeed[0]=secondSpeed[1];
		  secondSpeed[1]=testPowerHeatData.shooterHeat1;  //��¼��0ms�͵�100ms��ǹ������	//Heat1Ϊ���ӵ�
		  if(secondSpeed[1]-secondSpeed[0]>0)             //�жϷ�������ӵ�
		  {	
			    speedReal=secondSpeed[1]-secondSpeed[0];//+coolHeat; //���ݲ����������㷽ʽ��Ӽ�����ӵ��ٶ�
				realBulletFreq=speedReal/40;              //��¼�ӵ����������100ms�ڿ��ܴ���ü����ӵ�
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


void Heat_MAX_COOL_calc(int* maxheat,float* coolheat,int maxhp)	//���ݲ������Ѫ���Ƴ��ȼ����Ƴ��������޼���ȴֵ
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
