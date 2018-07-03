#ifndef __AUTO_TAKEBULLET_H
#define __AUTO_TAKEBULLET_H
#include "stm32f4xx.h"

#define VALVE_BULLET_HORIZONTAL1 0		//ԭ�ǵ�--��ƽ��1
#define VALVE_BULLET_HORIZONTAL2 1	//ԭǰ��--��ƽ��2
#define VALVE_BULLET_CLAMP 2	//�н�


void TakeBullet_Control_Center(void);

typedef enum
{
    BULLET_ACQUIRE,  		//ǰ�졢�н���̧����	��֮Ϊ��ù���
    BULLET_POUROUT,			//������б�������ת	��֮Ϊ��������
		BULLET_THROWOUT,			//������ء�����̧�𡢼н��ɿ�	��֮Ϊ�������
}TakeBulletState_e;


//u8 SetCheck_GripLift(u8 grip_state);	//�Ƿ��뵯ҩ��ƽ��,gripץס����˼	//0��ʾ��ץס������Ҫ����ҩ������ҩ��߶ȣ�1��ʾץס������Ҫ�н���ҩ��ʱ�ĸ߶�
//u8 SetCheck_SlopeLift(u8 slope_state);	//��ʱֻ������	slope��б����˼	//0��ʾ����б�����ָ�������ҩ��߶ȣ�1��ʾ��б������б���ӵ�״̬
//void SetCheck_TakeBullet_TakeBack(void);	//�г�ȡ��������λ����

#endif
