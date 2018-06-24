#ifndef __AUTO_LIFT_H__
#define __AUTO_LIFT_H__

#include "bsp.h"

u8 SetCheck_FrontLift(u8 rise_state);	//ǰ����������/���²����	//0��ʾFALL��1��ʾISLAND
u8 SetCheck_BackLift(u8 rise_state);

void Ascend_Control_Center(void);	//�ϵ��߼���������
u8 Ascend_FullRise_GO1(void);	//ǰ�����������������Ⱥ���
u8 Ascend_BackFall_GO(void);	//3��4������̧��ǰ���Ľ���
u8 Ascend_FullFall_GO(void);	//��̧�𵽶�̧���
u8 Ascend_FullRise_GO2(void);	//ǰ�����������������Ⱥ���

void Descend_Control_Center(void);	//�µ��߼���������
u8 Descend_FullFall_Down(void);
u8 Descend_FrontRise_Down(void);
u8 Descend_FullRise_Down1(void);

s16 Chassis_Attitude_Correct(float fdbP,s16 fdbV);	//�ϵ���̬��У������

u8 Check_FrontLift(void);
u8 Check_BackLift(void);


typedef enum
{
    FULLRISE_GO1,  		//��һ��ȫ������
    BACKFALL_GO1,			//��һ��������ȣ�����ǰ������ϵ��
		FULLFALL_GO1,			//��һ������ǰ�ȣ�ȫ������
		FULLRISE_GO2,  		//�ڶ���
    BACKFALL_GO2,			//�ڶ���
		FULLFALL_GO2,			//�ڶ���
}AscendState_e;

typedef enum
{
    FULLFALL_DOWN1,  		//��һ��ȫ��������׼���µ�
    FRONTRISE_DOWM1,			//��һ�����ǰ�����ŵأ�����ǰ������ϵ��
		FULLRISE_DOWN1,			//��һ������������ŵأ�ȫ�������
		FULLFALL_DOWN2,  		//�ڶ���
    FRONTRISE_DOWN2,			//�ڶ���
		FULLRISE_DOWN2,			//�ڶ���
}DescendState_e;

typedef enum
{
    CORRECT_CHASSIS_STATE,  		//��������
    CALI_SELF_STATE,			//У׼����Ŀ��λ��	//δ������µ�ģʽʱ������У׼����λ��
}IslandAttitudeCorrectState_e;

void Set_Attitude_Correct_State(IslandAttitudeCorrectState_e state);	//���ý���״̬�ĺ���

AscendState_e Island_State_Recognize(void);	//�����Զ��ǵ�״̬�Զ���ʶ

#endif
