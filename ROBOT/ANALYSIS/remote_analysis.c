#include "remote_analysis.h"

RC_Ctl_t RC_Ctl=RC_DATA_DEFAULT;
KeyBoardTypeDef KeyBoardData[KEY_NUMS]={0};
/*****************************************
�������ƣ�RemoteData_analysis
�������ܣ��Դ�ң�������ݽ�����λ����
�������������ң�������ݵ�����
��������ֵ����
������ ��
*****************************************/
u16 t_dbus_count=0;
void RemoteData_analysis(uint8_t *sbus_rx_buffer)
{
	t_dbus_count++;
	if(sbus_rx_buffer == NULL)
	{
			return;
	}
	RC_Ctl.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff; //!< Channel 0
	RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff; //!< Channel 1
	RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;//!< Channel 2	
	RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
	RC_Ctl.rc.switch_left = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2; //!< Switch left
	RC_Ctl.rc.switch_right = ((sbus_rx_buffer[5] >> 4)& 0x0003); //!< Switch right
	RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8); //!< Mouse X axis
	RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8); //!< Mouse Y axis
	RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8); //!< Mouse Z axis
	RC_Ctl.mouse.press_l = sbus_rx_buffer[12]; //!< Mouse Left Is Press ?
	RC_Ctl.mouse.press_r = sbus_rx_buffer[13]; //!< Mouse Right Is Press ?
	RC_Ctl.key.v_l = sbus_rx_buffer[14]; //!< KeyBoard value
	RC_Ctl.key.v_h = sbus_rx_buffer[15];
	Key_Analysis();
}


void Key_Analysis(void)
{
	KeyBoardData[KEY_W].value=RC_Ctl.key.v_l&0x01;
	KeyBoardData[KEY_S].value=(RC_Ctl.key.v_l&0x02)>>1;
	KeyBoardData[KEY_A].value=(RC_Ctl.key.v_l&0x04)>>2;
	KeyBoardData[KEY_D].value=(RC_Ctl.key.v_l&0x08)>>3;
	KeyBoardData[KEY_SHIFT].value=(RC_Ctl.key.v_l&0x10)>>4;
	KeyBoardData[KEY_CTRL].value=(RC_Ctl.key.v_l&0x20)>>5;
	KeyBoardData[KEY_Q].value=(RC_Ctl.key.v_l&0x40)>>6;
	KeyBoardData[KEY_E].value=(RC_Ctl.key.v_l&0x80)>>7;

	KeyBoardData[KEY_R].value=RC_Ctl.key.v_h&0x01;
	KeyBoardData[KEY_F].value=(RC_Ctl.key.v_h&0x02)>>1;
	KeyBoardData[KEY_G].value=(RC_Ctl.key.v_h&0x04)>>2;
	KeyBoardData[KEY_Z].value=(RC_Ctl.key.v_h&0x08)>>3;
	KeyBoardData[KEY_X].value=(RC_Ctl.key.v_h&0x10)>>4;
	KeyBoardData[KEY_C].value=(RC_Ctl.key.v_h&0x20)>>5;
	KeyBoardData[KEY_V].value=(RC_Ctl.key.v_h&0x40)>>6;
	KeyBoardData[KEY_B].value=(RC_Ctl.key.v_h&0x80)>>7;
	
//	for(int keyid=0;keyid<KEY_NUMS;keyid++)	//���ڶ�ʱ����
//	{
//		ButtonStatu_Verdict(&KeyBoardData[keyid]);
//	}
	

}


//��λ������
//���ֵ��1:�̰� 2:���� 0:δ��
//���ֶ̰��Ĵ�����ʽ ����ע��ѡ��
//2017.4.29
void ButtonStatu_Verdict(KeyBoardTypeDef * Key)	//�����ּ�ⷽ����һ������ʱ��Ϊ�ֽ��ͺ����߼�Ϊ����ֵ��ԭ����һ���޸��˳���״̬ǰʼ�ջ������һ״̬��ȱ��
{																			//����Ƶ��100HZ�����Ƿ��ڶ�ʱ�����
	if(Key->last==1)
	{
		Key->count++;
	}
	else
	{
		Key->count=0;
	}
	
	if(Key->count>10)	//���������� 10ms
	{
		if(Key->count<1000)	//1s
		{
			if(Key->last==1&&Key->value==0)
			{
				Key->statu=1;
			}
		}
		else
		{
			Key->statu=2;
		}
	}
	else
	{
		Key->statu=0;
	}
	Key->last=Key->value;
}

