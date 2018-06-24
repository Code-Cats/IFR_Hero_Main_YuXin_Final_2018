#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_

#include "main.h"

#define KEY_SWITCH_SHAKE         3
#define KEY_SWITCH_POINT_TIME    24
#define KEY_CHIOCE_SHAKE         1
#define KEY_CHIOCE_POINT_TIME    24
#define KEY_POINT_TIME  25

#define DATA_GO_TO_ZERO          127
#define DATA_TURE                1
#define DATA_FALSE               0

#define KEYBOARD_DATA_DEFAULT  {0};

#define KEY_SWITCH_DEFAULT \
{\
	KEY_SWITCH_SHAKE,\
	KEY_SWITCH_POINT_TIME,\
	0,\
	0,\
	0,\
	0,\
	0,\
}\

#define KEY_CHIOCE_DEFAULT \
{\
	KEY_CHIOCE_SHAKE,\
	KEY_CHIOCE_POINT_TIME,\
	0,\
	0,\
	0,\
	0,\
	{0,0},\
	0,\
}\

#define KEY_PIONT_LEFT_MOUSE_DEFAULT \
{\
	KEY_SWITCH_SHAKE,\
	KEY_POINT_TIME,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
}\

typedef __packed struct
{
		const char   shake;
	  const char   time;
    char    up;
    char    down;
	  char    away;
	  char    state;
	  char    output;
}KEY_SWITCH;

typedef __packed struct
{
		const char   shake;
	  const char   time;
    char    up;
    char    down;
	  char    away;
	  char    state;
	  char    state_save[2];
	  char    output;
}KEY_CHIOCE;

typedef __packed struct
{
		const char   shake;
	  const char   point;
	  const char   advanced_flag;
	  const float  advanced_end;
    char    up;
    char    down;
	  char    away;
	  char    state;
	  char    output;
	  float   advanced;
		float   advanced_inte;
	  
}KEY_POINT;

typedef struct  __KEYBOARD_DATA__
{
	char W_State;
	char S_State;
	char A_State;	
	char D_State;
	char Shift_State;
	char Ctrl_State;	
	char Q_State;
	char E_State;
	char R_State;	
	char F_State;
	char G_State;
	char Z_State;	
	char X_State;
	char C_State;
	char V_State;
  char B_State;		
	
	char  Mouse_Left;
	char  Mouse_Right;
	
	int16_t  Mouse_X; 
	int16_t  Mouse_Y;	
	int16_t  Keyboard;
}KEYBOARD_DATA;


 char Key_Choice(char input,KEY_CHIOCE *key);
 char Key_Switch(char input,KEY_SWITCH *key);
 char Key_Point(char input,KEY_POINT *key);

 void Keyboard_Response(void);
 int16_t Shake_Keyboard_Remove(int16_t key_value);
 int16_t Shake_Keymouse_Remove(int16_t key_value);


#endif

