#include "remote_analysis.h"

RC_Ctl_t RC_Ctl=RC_DATA_DEFAULT;
KeyBoardTypeDef KeyBoardData[KEY_NUMS]={0};
/*****************************************
函数名称：RemoteData_analysis
函数功能：对大疆遥控器数据进行移位解析
函数参数：存放遥控器数据的数组
函数返回值：无
描述： 无
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
	
//	for(int keyid=0;keyid<KEY_NUMS;keyid++)	//放在定时器里
//	{
//		ButtonStatu_Verdict(&KeyBoardData[keyid]);
//	}
	

}


//键位处理函数
//结果值：1:短按 2:长按 0:未按
//两种短按的触发方式 自行注释选择
//2017.4.29
void ButtonStatu_Verdict(KeyBoardTypeDef * Key)	//有两种检测方法，一种是以时间为分界点和后来者即为最终值的原理。另一种修复了长按状态前始终会存在另一状态的缺点
{																			//处理频率100HZ，还是放在定时器里，，
	if(Key->last==1)
	{
		Key->count++;
	}
	else
	{
		Key->count=0;
	}
	
	if(Key->count>10)	//防抖动部分 10ms
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

