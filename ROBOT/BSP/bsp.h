#ifndef __BSP_H__
#define __BSP_H__

#include "common_definition.h"


#include "can1.h"
#include "can2.h"
#include "pwm.h"
#include "gpio.h"
#include "usart1_remote.h"
#include "heartbeat.h"
#include "usart6_viceboard.h"
#include "viceboard_analysis.h"
#include "usart5_wifi_Debug.h"
#include "usart6_wifi_Debug.h"
#include "usart3_judge.h"

#include "uart4.h"
#include "imu_data_decode.h"
#include "packet.h"

#include "main.h"

void BSP_Init(void);



#endif 

