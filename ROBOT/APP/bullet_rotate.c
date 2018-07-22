#include "bullet_rotate.h"
#include "math.h"
#include "pid.h"

//#define BULLETROTATE_LEFT 1
//#define BULLETROTATE_RIGHT 0

extern Error_check_t Error_Check;

BULLETROTATE_DATA BulletRotate_Data[2]={BULLETROTATE_DATA_DEFAULT,BULLETROTATE_DATA_DEFAULT};	//ȡ����ת�������

PID_GENERAL   PID_BulletRotate_Position[2]={PID_BULLETROTATE_POSITION_DEFAULT,PID_BULLETROTATE_POSITION_DEFAULT};
PID_GENERAL   PID_BulletRotate_Speed[2]={PID_BULLETROTATE_SPEED_DEFAULT,PID_BULLETROTATE_SPEED_DEFAULT};

extern u32 time_1ms_count;
extern ViceControlDataTypeDef ViceControlData;

extern u8 BulletRotate_Cali_Statu;	//�궨״̬
void BulletRotate_Task(void)
{
	BulletRotate_Data[BULLETROTATE_LEFT].tarP=-BulletRotate_Data[BULLETROTATE_RIGHT].tarP;	//����ֵ
//	BulletRotate_Data.tarP=(s16)(12*(RC_Ctl.rc.ch3-1024)/600.0);
	if(GetWorkState()!=CALI_STATE&&BulletRotate_Cali_Statu==2)	//�궨״̬
	{
			BulletRotate_Data[BULLETROTATE_RIGHT].tarV=PID_General(BulletRotate_Data[BULLETROTATE_RIGHT].tarP,BulletRotate_Data[BULLETROTATE_RIGHT].fdbP,&PID_BulletRotate_Position[BULLETROTATE_RIGHT]);
			BulletRotate_Data[BULLETROTATE_LEFT].tarV=PID_General(BulletRotate_Data[BULLETROTATE_LEFT].tarP,BulletRotate_Data[BULLETROTATE_LEFT].fdbP,&PID_BulletRotate_Position[BULLETROTATE_LEFT]);
	}
//	BulletRotate_Data.tarV=PID_General(BulletRotate_Data.tarP,BulletRotate_Data.fdbP,&PID_BulletRotate_Position);
	
	BulletRotate_Data[BULLETROTATE_RIGHT].output=PID_General(BulletRotate_Data[BULLETROTATE_RIGHT].tarV,BulletRotate_Data[BULLETROTATE_RIGHT].fdbV,&PID_BulletRotate_Speed[BULLETROTATE_RIGHT]);
	BulletRotate_Data[BULLETROTATE_LEFT].output=PID_General(BulletRotate_Data[BULLETROTATE_LEFT].tarV,BulletRotate_Data[BULLETROTATE_LEFT].fdbV,&PID_BulletRotate_Speed[BULLETROTATE_LEFT]);
	
}


u8 BulletRotate_OffSetInit(void)	//��ʼOFFSET	//��cali(outputΪ0ʱ)״̬�½��б궨
{
	static u8 offset_statu=0;
	static s32 fdbp_record[2]={0};
	static u8 record_i=0;
	
	if(offset_statu==0)	//�궨״̬
	{
		if(Error_Check.statu[LOST_BULLETROTATE1]==0&&time_1ms_count%4==0)	//����
		{
			fdbp_record[0]=BulletRotate_Data[BULLETROTATE_RIGHT].fdbP;
			if(fdbp_record[0]==fdbp_record[1])
			{
				record_i++;
			}
			else
			{
				record_i=0;
			}
			fdbp_record[1]=fdbp_record[0];	//����
		}
		else if(Error_Check.statu[LOST_BULLETROTATE1]==1)
		{
			record_i=0;
		}
		
		if(record_i>10)
		{
			offset_statu=1;
			BulletRotate_Data[BULLETROTATE_RIGHT].offsetP=BulletRotate_Data[BULLETROTATE_RIGHT].fdbP;
		}
	}
		
	return offset_statu;
}

extern LIFT_POSITION_ENCODER bulletrotate_position_encoder[2];	//����Ȧ��
u8 BulletRotate_Cali_Statu=0;
u8 BulletRotate_Cali(void)	//��ʼλ�ñ궨	//�������״̬�½��б궨
{
	u8 cali_state=0;
	if(Error_Check.statu[LOST_BULLETROTATE1]==0)	//����
	{
		switch(BulletRotate_Cali_Statu)
		{
			case 0:
			{
				static u16 time_count=0;
				time_count++;	//1ms�ۼ�һ��
				BulletRotate_Data[BULLETROTATE_RIGHT].tarV=-2000;
				if(time_count>400)	//�����ʱĿ����ȷ������Ѿ�����
				{
					time_count=0;
					BulletRotate_Data[BULLETROTATE_RIGHT].tarV=-1500;
					BulletRotate_Cali_Statu=1;
				}
				break;
			}
			case 1:
			{
				static u16 time_count=0;
				
				BulletRotate_Data[BULLETROTATE_RIGHT].tarV=-1500;
				if(abs(BulletRotate_Data[BULLETROTATE_RIGHT].fdbV)<20)
				{
					time_count++;	//1ms�ۼ�һ��
					bulletrotate_position_encoder[BULLETROTATE_RIGHT].turns=0;	//����Ȧ��
					bulletrotate_position_encoder[BULLETROTATE_LEFT].turns=0;	//����Ȧ��	//���������
				}
				else
				{
					time_count=0;
				}
				
				if(time_count>500)	//�궨���
				{
					time_count=0;
					BulletRotate_Cali_Statu=2;
					//����Ȧ��	//��¼offset
					BulletRotate_Data[BULLETROTATE_RIGHT].offsetP=BulletRotate_Data[BULLETROTATE_RIGHT].fdbP;
					BulletRotate_Data[BULLETROTATE_LEFT].offsetP=BulletRotate_Data[BULLETROTATE_LEFT].fdbP;
				}
				break;
			}
			case 2:
			{
				cali_state=1;
				break;
			}
		}
	}
	else if(GetWorkState()==CALI_STATE)//��ʧ����
	{
//		static u16 time_count=0;
//		time_count++;	//1ms�ۼ�һ��
//		if(time_count>)
	}
	return cali_state;
}