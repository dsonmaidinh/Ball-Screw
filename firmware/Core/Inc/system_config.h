#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>
//#include "motor.h"

// TOÁN HỌC
#define SYS_PI 							3.1415
#define SYS_RAD2MM					2.0/SYS_PI
#define SYS_MM2RAD					SYS_PI/2.0
// HỆ VÍT ME
#define SYS_PB							4.0
#define SYS_MAXLENGTH				400.0
#define SYS_MINLENGTH				0.0
// ĐỘNG CƠ
#define MOTOR_PORT 			GPIOA
#define MOTOR_PIN_1			0
#define MOTOR_PIN_2			1
#define SYS_SAMPLETIME			0.005
// ENCODER
#define Encoder_Port		GPIOB
#define Encoder_A				GPIO_PIN_0
#define Encoder_B				GPIO_PIN_2
#define Encoder_ppr 		8000

#define SYS_MaxPWM				  1000
#define SYS_MaxPWM_P			  800
// THỜI GIAN
#define SYS_RUN_TIME				2.1
#define SYS_STOP_TIME				12.9
#endif
