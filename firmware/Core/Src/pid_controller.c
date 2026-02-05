#include "pid_controller.h"
#include "trajectory.h"
#include "stm32f4xx.h"
#include "system_config.h"
#include <math.h>

void PID_ResetState(pid_controller *pid)
{
		pid->theta_err_init = 0.0f;
    pid->omega_ref_prev = 0.0f;
    pid->omega_dec_estimated = 0.0f;
    pid->compensation_term2 = 0.0f;
    pid->first_call = 1;
    pid->direction_locked = 0;
    pid->sign = 0;
}

void PID_ResetParams(pid_controller *pid){
		pid->ek_pre = 0;
    pid->err_sat = 0;
    pid->UI_pre = 0;
}

static inline void PID_ResetDirection(pid_controller *pid)
{
    pid->direction_locked = 0;
    pid->sign = 0;
}

static inline void PID_UpdateDirection(pid_controller *pid, float error)
{
    // Only update if not locked
    if (!pid->direction_locked) {
        if (fabsf(error) > 0.001f) {  // Deadzone threshold
            pid->sign = (error > 0.0f) ? 1 : -1;
            pid->direction_locked = 1;  // Lock direction
        }
    }
}

static inline float PID_ApplyDynamicLimiter(pid_controller *pid, float v_desired, float v_prev)
{
    if (!pid->limiter_enabled) {
        return v_desired;
    }
    
    // Tính delta_v
    float delta_v = v_desired - v_prev;
    
    // Giới hạn gia tốc tối đa: |Δv| = amax×Ts
    if (fabsf(delta_v) > pid->accel_limit_step) {
        // Nếu vượt quá ở giới hạn theo dấu của delta_v
        // delta_v > 0: đang muốn tăng tốc nhanh ở hướng +amax×Ts
        // delta_v < 0: đang muốn giảm tốc nhanh ở hướng -amax×Ts
        v_desired = v_prev + copysignf(pid->accel_limit_step, delta_v);
    }
    
    // Giới hạn output cuối cùng trong [-vmax, +vmax]
    if (v_desired > pid->output_limit) {
        v_desired = pid->output_limit;
    } else if (v_desired < -pid->output_limit) {
        v_desired = -pid->output_limit;
    }
    
    return v_desired;
}

void PID_SetParams(pid_controller *pid, float kp, float ki, float kd, float kb, float alpha, float output_limit, float sample_time)
{
    // PID Parameters
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kb = kb;
    pid->alpha = alpha;
    pid->output_limit = output_limit;
    pid->sample_time = sample_time;
    
    // Disable improved features
    pid->compensator_enabled = 0;
    pid->limiter_enabled = 0;
    
    // Reset state
    PID_ResetState(pid);
}

void PID_SetParamsImproved(pid_controller *pid, float kp, float output_limit, float sample_time, float K_est, float K_com, float accel_max)
{
    // Basic PID (P-only for position)
    pid->kp = kp;
    pid->ki = 0.0f;
    pid->kd = 0.0f;
    pid->kb = 0.0f;
    pid->alpha = 0.0f;
    pid->output_limit = output_limit;
    pid->sample_time = sample_time;
    
    // Improved Profile Generator Parameters
    pid->K_est = K_est;
    pid->K_com = K_com;
    pid->accel_max = accel_max;
    pid->control_period = sample_time;
    
    // Pre-calculate constant term for efficiency
    // Term 1: Kpp·Kcom / (2·amax)
    pid->compensation_term1 = (kp * K_com) / (2.0f * accel_max);
    
    // Pre-calculate for dynamic limiter
    pid->accel_limit_step = accel_max * sample_time * 0.1;
    
    // Enable improved features
    pid->compensator_enabled = 1;
    pid->limiter_enabled = 1;
    
    // Reset state
    PID_ResetState(pid);
}

// ============================================================
// POSITION CONTROLLER - IMPROVED (Heo et al. 2019)
// ============================================================
float PID_Pos_Improved(pid_controller *pid, float desired, float current)
{
    // 1. TÍnh err - Position error
    float theta_err = desired - current;
    if(fabs(theta_err) < 0.01) return 0;
    
    // === KHỞI TẠO LẦN ĐẦU ===
    if (pid->first_call && fabsf(theta_err) > 0.001f && pid->compensator_enabled) {
				PID_UpdateDirection(pid, theta_err);
				
        // Theta_err(0)
        pid->theta_err_init = theta_err;
        
        // Tính ω̂*m(tdec) = min(√(K_est·amax·|θerr(0)|), ωmax)
        float omega_est_raw = sqrtf(pid->K_est * pid->accel_max * fabsf(pid->theta_err_init));
        float omega_magnitude = fminf(omega_est_raw, pid->output_limit);
                
        // Tính term2 = 1 / |ω̂*m(tdec)|
        // Sử DỤNG MAGNITUDE (không dấu) để tránh term2 âm!
        if (omega_magnitude > 0.01f) {
            pid->compensation_term2 = 1.0f / omega_magnitude;
        } else {
            pid->compensation_term2 = 0.0f;
        }
        
        pid->first_call = 0;
    }
    
    // === FEEDBACK COMPENSATOR ===
    // 4. Tính ωmfd(t) = [Kpp·Kcom/(2·amax) - 1/|ω̂*m(tdec)|] · ω*m(t)
    float omega_compensation = 0.0f;
    
    if (pid->compensator_enabled && fabsf(pid->omega_ref_prev) > 0.001f) {
        // Bình phương vận tốc tham chiếu (LUÔN DƯƠNG)
        float omega_ref_squared = pid->omega_ref_prev * pid->omega_ref_prev;
        
        // Hệ số bù = [term1 - term2]
        float compensation_coefficient = pid->compensation_term1 - pid->compensation_term2;
        
        // Tính compensation (chưa có dấu)
        omega_compensation = compensation_coefficient * omega_ref_squared;
        
        omega_compensation *= (float)pid->sign;
    }

    float omega_p_output = pid->kp * theta_err;
    // Apply compensation: ω*mcom = Kpp·θerr - ωmfd
    float omega_compensated = omega_p_output - omega_compensation;
    
    // === DYNAMIC RANGE LIMITER ===
    if(pid->limiter_enabled) omega_compensated = PID_ApplyDynamicLimiter(pid, omega_compensated, pid->omega_ref_prev);
    // === FINAL OUTPUT SATURATION ===
    if (omega_compensated > pid->output_limit) {
        omega_compensated = pid->output_limit;
    } else if (omega_compensated < -pid->output_limit) {
        omega_compensated = -pid->output_limit;
    }
    
    // Update state: lưu ω*m(t) cho chu kỳ tiếp theo
    pid->omega_ref_prev = omega_compensated;
    
    return omega_compensated;
}

// ============================================================
// POSITION CONTROLLER - SIMPLE P-ONLY
// ============================================================
float PID_Pos_Simple(pid_controller *pid, float desired, float current)
{
    float error = desired - current;
		if(fabs(error) < 0.5) {
				
				return 0;
		}
    float output = pid->kp * error;
    
    // Apply output limit
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }
    
    return output;
}

int PID_Vel(pid_controller *pid, float DesiredValue, float CurrentValue, MotionPhase_t phase, float sign)
{
    static float UD_func_pre = 0;
    static float UI_backup = 0;  // Biến backup cho UI_pre
		static uint8_t first_time = 1;
    static uint8_t is_stopped = 1;  // Cờ trạng thái dừng
    
    float pidterm = 0, pid_sat = 0;
    float ek;
    float UP, UI, UD, UD_func;
    
    int16_t pidout;
    if(pid->ki == 0) pid->kb = 0;
    
    // TÍNH SAI SỐ HIỆN TẠI
    ek = DesiredValue - CurrentValue;
    
    // KIỂM TRA ĐIỀU KIỆN DỪNG PWM
    if((phase == TRAJ_PHASE_STOP && pid->time >= 1.0) || phase == TRAJ_PHASE_COMPLETE){
        // BACKUP UI_pre TRƯỚC KHI RESET
        if(!is_stopped) {
            UI_backup = pid->UI_pre;
            is_stopped = 1;
        }
        
        // RESET CÁC BIẾN KHÁC NHƯNG GIỮ LẠI UI_pre TRONG BACKUP
        pid->time = 0;
        pid->ek_pre = 0;
        pid->err_sat = 0;
        pid->UI_pre = 0;
        UD_func_pre = 0;
				first_time = 1;
        return 0;  // TRẢ VỀ PWM = 0 NGAY LẬP TỨC
    } 
    else if(phase == TRAJ_PHASE_ACCEL && first_time) {
        // KHI BẮT ĐẦU GIA TỐC, KHÔI PHỤC UI_pre TỪ BACKUP
        if(is_stopped) {
            pid->UI_pre = 0;
            is_stopped = 0;
        }
				first_time = 0;
    }
		else if(phase == TRAJ_PHASE_STOP && fabs(ek) < 0.009) pid->time += pid->sample_time;
    
    /* Khâu P */
    UP = pid->kp * ek;
    
//    /* Khâu D với lọc */
//    UD = pid->kd * (ek - ek_pre) / pid->sample_time;
//    UD_func = (1 - pid->alpha) * UD_func_pre + pid->alpha * UD;
    
    /* Khâu I với anti-windup */
    UI = pid->UI_pre + pid->ki * ek * pid->sample_time + pid->kb * pid->err_sat * pid->sample_time;
    
    /* Tổng PID output */
    if(pid->err_state) {
				pidterm = 0;
				PID_ResetParams(pid);
				return 0;
		}
		else pidterm = UP + UI;
    
    /* Saturation */
    if(pidterm >= pid->output_limit) {
        pid_sat = pid->output_limit;
        pid->err_sat = pid->output_limit - pidterm;
    }
    else if (pidterm <= -pid->output_limit) {
        pid_sat = -pid->output_limit;
        pid->err_sat = -pid->output_limit - pidterm;
    }
    else {
        pid_sat = pidterm;
        pid->err_sat = 0;
    }
    
    /* Cập nhật các biến static */
    pid->ek_pre = ek;
    pid->UI_pre = UI;
//    UD_func_pre = UD_func;
    
    pidout = (int16_t) pid_sat;
    return pidout;
}

void PID_DisableImproved(pid_controller *pid){
		pid->compensator_enabled = 0;
		pid->limiter_enabled = 0;
}
void PID_EnableImproved(pid_controller *pid){
		pid->compensator_enabled = 1;
		pid->limiter_enabled = 1;
}

// ============================================================
// POSITION CONTROLLER - FULL PID
// ============================================================
int PID_Pos(pid_controller *pid, float DesiredValue, float CurrentValue)
{
    static float ek_pre = 0;
    static float err_sat = 0;
    static float UI_pre = 0;
    static float UD_func_pre = 0;

    float pidterm = 0, pid_sat = 0;
    float ek;
    float UP, UI, UD, UD_func;
    
    int16_t pidout;
    if(pid->ki == 0) pid->kb = 0;
    
    // T�nh sai s? hi?n t?i
    ek = DesiredValue - CurrentValue;
    
    /* Kh�u P */
    UP = pid->kp * ek;
    
    /* Kh�u D v?i l?c */
    UD = pid->kd * (ek - ek_pre) / pid->sample_time;
    UD_func = (1 - pid->alpha) * UD_func_pre + pid->alpha * UD;
    
    /* Kh�u I v?i anti-windup */
    UI = UI_pre + pid->ki * ek * pid->sample_time + pid->kb * err_sat * pid->sample_time;
    
    /* T?ng PID output */
    pidterm = UP + UI + UD_func;
    
    /* Saturation */
    if(pidterm >= pid->output_limit) {
        pid_sat = pid->output_limit;
        err_sat = pid->output_limit - pidterm;
    }
    else if (pidterm <= -pid->output_limit) {
        pid_sat = -pid->output_limit;
        err_sat = -pid->output_limit - pidterm;
    }
    else {
        pid_sat = pidterm;
        err_sat = 0;
    }
    
    /* C?p nh?t c�c bi?n static */
    ek_pre = ek;
    UI_pre = UI;
    UD_func_pre = UD_func;
    
    pidout = (int16_t) pid_sat;
    return pidout;
}
 