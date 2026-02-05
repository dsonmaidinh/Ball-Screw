#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "encoder.h"
#include "trajectory.h"

#define MOTOR_VEL_CONVERSION_FACTOR_RAD (SYS_PI/20.0)
#define MOTOR_VEL_CONVERSION_FACTOR_MM  (1.0/10.0)

#define MOTOR_MAX_SPEED_TOLERANCE 			1.1 		// Sai số vận tốc so với vận tốc tối đa
#define MOTOR_VEL_THRESHOLD 						0.5     // Ngưỡng vận tốc để coi là dừng (mm/s)
#define MOTOR_STABILIZE_TIME 						0.9     // Thời gian chờ ổn định (s)
#define MOTOR_DEADZONE_VEL 							0.11    // Ngưỡng nhiễu vận tốc khi về 0 (mm/s)
#define MOTOR_ACCEL_START_THRESHOLD 		0.09    // Ngưỡng vận tốc đặt bắt đầu tăng tốc (mm/s)
#define MOTOR_MAXVEL_MM     						45
#define MOTOR_MINVEL_MM     						20
#define MOTOR_MAXACC_SF									0.8
#define MOTOR_RESPONSETIME							0.02


typedef enum {
    MOTOR_IDLE = 0,
    MOTOR_ACCEL,      
    MOTOR_CONSTANT,   
    MOTOR_DECEL,      
    MOTOR_STOPPED    
} MotorState;
typedef enum {
    MOTOR_SetPos = 0,
    MOTOR_P_PI,      
    MOTOR_P,   
} MotorMode;

typedef struct {
    float DesiredPos_mm, DesiredPos_mm_pre, AutoDesiredPos_mm, DesiredVel_mm, DesiredVel_rad;
    float CurPos_mm, CurVel_rad, CurVel_mm, CurVel_mm_pre, Acc_mm;	

    //--- BIẾN THỜI GIAN
//    float time_actual;        // tổng thời gian thực tế
//    float ta_actual;          // thời gian tăng tốc thực tế
//    float tc_actual;          // thời gian vận tốc không đổi thực tế
//    float td_actual;          // thời gian giảm tốc thực tế
//		float t_motor_adjust;		  // thời gian động cơ tự điều chỉnh (không mong muốn)
		uint16_t tick;
		uint8_t tick_enabled;
		uint8_t check;
	
    float vel_diff;
		uint8_t plot_active;
		uint8_t auto_run;
		uint8_t cycle_count;
		float cycle_error[2];
		uint8_t cycle_index;
	
    //--- TRẠNG THÁI ĐỘNG CƠ
    MotorState state;
    MotorState prev_state;
		MotorMode  mode;

    //--- THÔNG SỐ ĐỘNG CƠ
		float v_max_rad;
		float v_max_mm;
		float a_max_mm;
		float err_pos;
	
    int16_t pwm, pwm_real, dir;
    uint32_t pwmchannel;
    uint8_t mpin[2];
} motor;

void MOTOR_SetRun(motor *Motor);

void MOTOR_SetParams(motor *Motor, uint32_t ch, uint8_t pin1, uint8_t pin2, MotorMode mode, float max_speed_mm);
void MOTOR_ResetParams(motor *Motor);
void MOTOR_CalculateParams(motor *Motor, encoder *Encoder, float sign, MotionPhase_t phase);

void MOTOR_ResetTime(motor *Motor);
void MOTOR_CalculateTime(motor *Motor);
void MOTOR_CalculateTime_v1(motor *Motor, MotionPhase_t phase);

void MOTOR_UpdateMode(motor *Motor, MotorMode mode);
void MOTOR_UpdateVel(motor *Motor, float vel);
void MOTOR_UpdatePos(motor *Motor, float pos);

#endif
