#ifndef __REMOTE_ANALYSIS_H__
#define __REMOTE_ANALYSIS_H__

#include "main.h"

#define REMOTE_DATA_DEFAULT              {1024,1024,1024,1024,3,3};

#define RC_SWITCH_UP 1
#define RC_SWITCH_MIDDLE 3
#define RC_SWITCH_DOWN 2

typedef struct
{
	struct
	{
		uint16_t ch0;
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
		uint8_t switch_left;
		uint8_t switch_right;
	}rc;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	}mouse;
	struct
	{
		uint8_t v_l;
		uint8_t v_h;
	}key;
}RC_Ctl_t;

typedef struct
{
	u16 count;
	u8 value;
	u8 last;
	u8 statu;
}KeyBoardTypeDef;

enum KEYBOARDID	//键位标识
{   
	KEY_W,
	KEY_S,
	KEY_A,
	KEY_D,
	KEY_SHIFT,
	KEY_CTRL,
	KEY_Q,
	KEY_E,\
	\
	KEY_R,
	KEY_F,
	KEY_G,
	KEY_Z,
	KEY_X,
	KEY_C,
	KEY_V,
	KEY_B,
	KEY_NUMS,
};

#define RC_DATA_DEFAULT \
{\
	{1024,1024,1024,1024,3,3},\
	{0},\
	{0},\
}\

extern RC_Ctl_t RC_Ctl;
void Key_Analysis(void);
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];

void RemoteData_analysis(uint8_t *djiData);
void ButtonStatu_Verdict(KeyBoardTypeDef * Key);	//有两种检测方法，一种是以时间为分界点和后来者即为最终值的原理。另一种修复了长按状态前始终会存在另一状态的缺点

#endif

