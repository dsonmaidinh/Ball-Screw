#ifndef PID_H
#define PID_H

#include <stdint.h>
#include "trajectory.h"

typedef struct {
    //--- THÔNG SỐ PID CƠ BẢN
    float kp;
    float ki;
    float kd;
    float kb;
    float alpha;
    float output_limit;
    float sample_time;
    
		float ek_pre;
    float err_sat;
    float UI_pre;
	
    //--- THÔNG SỐ PID CẢI TIẾN
    // Hệ số bù
    float K_est;                    // Hệ số ước lượng
    float K_com;                    // Hệ số bù
    float accel_max;                // Gia tốc tối đa [MM/s²]
		float compensation_term1;       // Kpp*Kcom / (2*amax)
    float compensation_term2;       // 1 / ω^*m(tdec)
    
    // Biến
    float theta_err_init;           // Initial position error
    float omega_ref_prev;           // Previous velocity reference
    float omega_dec_estimated;      // Estimated velocity at deceleration point

    //--- Bộ giới hạn dải động
    float accel_limit_step;         // = accel_max * Ts
    float control_period;           // Chu kỳ điều khiển Ts [s]
    
    // Cờ điều khiển
    uint8_t compensator_enabled;     // Cờ bật bộ bù
    uint8_t limiter_enabled;        // Cờ bật bộ giới hạn dải động
    uint8_t first_call;             // Cờ lần gọi đầu (cho khởi tạo)
    uint8_t direction_locked;       // Cờ khóa hướng


    //--- Hướng chuyển động
    int8_t sign;                    // Hướng chuyển động: +1 hoặc -1
		
		float time;
		uint8_t err_state;
} pid_controller;


void PID_SetParams(pid_controller *pid, float kp, float ki, float kd, float kb, float alpha, float output_limit, float sample_time);
void PID_SetParamsImproved(pid_controller *pid, float kp, float output_limit, float sample_time, float K_est, float K_com, float accel_max);

float PID_Pos_Simple(pid_controller *pid, float DesiredValue, float CurrentValue);
float PID_Pos_Improved(pid_controller *pid, float DesiredValue, float CurrentValue);
int PID_Vel(pid_controller *pid, float DesiredValue, float CurrentValue, MotionPhase_t phase, float sign);

void PID_EnableImproved(pid_controller *pid);
void PID_DisableImproved(pid_controller *pid);

void PID_ResetState(pid_controller *pid);
void PID_ResetParams(pid_controller *pid);
	
int PID_Pos(pid_controller *pid, float DesiredValue, float CurrentValue);
#endif
