#ifndef __VICEBOARD_ANALYSIS_H
#define __VICEBOARD_ANALYSIS_H

#include "bsp.h"

#define VALVE_ISLAND 0		//��ŷ�����λ����
#define VALVE_BULLET_PROTRACT 1
#define VALVE_BULLET_CLAMP 2



void ViceBoard_SendDataRun(void);
void ViceBoard_SendDataRefresh(void);
void ViceData_Receive(u8 data);	//�����崫���������ݽ�����������ͨ�ã�
void SensorData_Deal(volatile u8 *pData);

typedef struct
{
	u8 valve[6];
	u8 servo[2];
	u8 image_cut[2];	//Ӣ���ǵ�1λ��Ч
}ViceControlDataTypeDef;	//���Ƹ���


typedef struct
{
	u8 statu;
	u8 data[5];
	u8 count;
}ViceBoardSendTypeDef;	//���͸���������ݽṹ��

#define VICEBOARD_SENDDATA_DEFAULT \
{\
	0,\
	{0x5A,0,0,0,0xA5},\
	0,\
}\

typedef struct
{
	volatile u8 headOK_state;
	volatile u8 valid_state;	//����֡��Ч��־λ
	volatile u8 databuffer[5];
	volatile u8 count;
}ReceiveDataTypeDef;	//�������ݴ������ݽṹ��

typedef struct
{
	u8 Infrare[6];	//�¼�����������
	u8 Limit[4];
}SensorDataTypeDef;	//�������õ��Ĵ���������

extern ViceBoardSendTypeDef SendData;
extern ReceiveDataTypeDef ReceiveData;


#endif

