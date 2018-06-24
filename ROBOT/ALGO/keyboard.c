#include "keyboard.h"

KEY_SWITCH           Key_Switch_Shift=KEY_SWITCH_DEFAULT;
KEY_SWITCH           Key_Switch_R=KEY_SWITCH_DEFAULT;
KEY_POINT            Key_Spoint_Left_Mouse=KEY_PIONT_LEFT_MOUSE_DEFAULT;
KEY_CHIOCE           Key_Chioce_Nine=KEY_CHIOCE_DEFAULT;
KEYBOARD_DATA 			 keyData=KEYBOARD_DATA_DEFAULT;

/*************************************************	
//W S A D        0x01   0x02   0x04   0x08
//shift Ctrl Q E 0x10   0x20   0x40   0x80
//R F G Z        0x100  0x200  0x400  0x800
//X C V B        0x1000 0x2000 0x3000 0x4000*/
/***********************************************
函数名称：Keyboard_Response
函数功能：获得鼠标键盘的键值状态
函数参数：无
函数返回值：无
描述： 无
************************************************/
void Keyboard_Response(void)
{
		
		keyData.W_State=Shake_Keyboard_Remove((0x01));
		keyData.S_State=Shake_Keyboard_Remove((0x02));
		keyData.A_State=Shake_Keyboard_Remove((0x04));
		keyData.D_State=Shake_Keyboard_Remove((0x08));
		keyData.Shift_State=Shake_Keyboard_Remove((0x10));
		keyData.Ctrl_State=Shake_Keyboard_Remove((0x20));
		keyData.Q_State=Shake_Keyboard_Remove((0x40));
		keyData.E_State=Shake_Keyboard_Remove((0x80));
		keyData.R_State=Shake_Keyboard_Remove((0x100));
		keyData.F_State=Shake_Keyboard_Remove((0x200));
		keyData.G_State=Shake_Keyboard_Remove((0x400));
		keyData.Z_State=Shake_Keyboard_Remove((0x800));
		keyData.X_State=Shake_Keyboard_Remove((0x1000));
		keyData.C_State=Shake_Keyboard_Remove((0x2000));
		keyData.V_State=Shake_Keyboard_Remove((0x4000));
		keyData.B_State=Shake_Keyboard_Remove((0x8000));
}
