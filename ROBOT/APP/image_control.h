#ifndef __IMAGE_CONTROL_H_
#define __IMAGE_CONTROL_H_

#include "bsp.h"

void Screen_Start(void);	//屏幕启动切换到AV信道
void Image_Cut_Task(void);	//摄像头切换、舵机
void Image_Cut_Screen(u8 state);

#endif
