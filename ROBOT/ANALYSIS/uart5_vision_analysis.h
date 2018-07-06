#ifndef __UART5_VISION_ANALYSIS_H
#define __UART5_VISION_ANALYSIS_H

#include "stm32f4xx.h"
#include "bsp.h"

typedef struct
{
	volatile u8 headOK_state;
	volatile u8 valid_state;	//����֡��Ч��־λ
	volatile u8 databuffer[10];	//���µ��������ٶ�10λ//�ɸ����5λ��Ϊ�Ӿ�8λ����֡ͷ֡β��
	volatile u8 count;
}VisionReceiveDataTypeDef;	//�������ݴ������ݽṹ��


typedef struct
{
	u8 armor_sign;	//�Ƿ�����Чװ��
	u8 armor_type;	//װ������
	u8 armor_dis;	//�����Ϣ
	u16 tar_x;	//x����
	u16 tar_y;	//y����
	s16 shooterror_x;	//ʵ��Ŀ��ƫ��x	��Ϊ�Զ����
	s16 shooterror_y;	//ʵ��Ŀ��ƫ��y	��Ϊ�Զ����
	s16 pix_x_v;
	float angel_x_v;	//���ںϵĵ�������ٶ�	//��λΪ0.1��ÿ��
	u8 vision_control_state;	//�Ƿ��ܿ�
}VisionDataTypeDef;	//�������õ��Ĵ���������

void VisionData_Receive(u8 data);	//���Ӿ������������ݽ���У��������
void VisionData_Deal(volatile u8 *pData);	//�Ӿ�����

#endif
