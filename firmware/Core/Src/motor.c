#include "motor.h"
#include "stm32f4xx.h"
#include "system_config.h"
#include <stdbool.h>
#include <math.h>

extern TIM_HandleTypeDef htim2;

void MOTOR_SetRun(motor *Motor){
		int8_t next_dir = 0;
    
    // 1. Xác định hướng TƯƠNG LAI dựa trên PWM
    if(Motor->pwm >= 0) {
        next_dir = 1; // Ví dụ: -1 là chiều thuận
    } else {
        next_dir = -1;  // Ví dụ: 1 là chiều nghịch
    }

    // 2. So sánh hướng tương lai và hướng hiện tại
    if(next_dir != Motor->dir){
        // Nếu đảo chiều, Reset cả 2 chân trước để tránh xung đột
        MOTOR_PORT->ODR &= ~((1<<Motor->mpin[0]) | (1<<Motor->mpin[1]));
        Motor->dir = next_dir; // Cập nhật hướng mới
    }
    
    // 3. Thực hiện điều khiển
    Motor->pwm_real = (Motor->pwm >= 0) ? Motor->pwm : -Motor->pwm;
    static int8_t pre_dir = 0;
    if(Motor->pwm >= 0) {
        MOTOR_PORT->ODR |= 1<<Motor->mpin[0];
    }
    else if(Motor->pwm < 0) {
        MOTOR_PORT->ODR |= 1<<Motor->mpin[1];
    }
    __HAL_TIM_SetCompare(&htim2, Motor->pwmchannel, Motor->pwm_real);
}

void MOTOR_SetParams(motor *Motor, uint32_t ch, uint8_t pin1, uint8_t pin2, MotorMode mode, float max_speed_mm){
    Motor->pwmchannel = ch;
    Motor->mpin[0]    = pin1;
    Motor->mpin[1]    = pin2;
    
		Motor->cycle_error[0] = 0.1;
		Motor->cycle_error[1] = 0.9;
	
    Motor->state      = MOTOR_IDLE;
    Motor->prev_state = MOTOR_IDLE;
	
		Motor->v_max_mm	  =	max_speed_mm;
		Motor->a_max_mm	  = (max_speed_mm/MOTOR_RESPONSETIME)*MOTOR_MAXACC_SF;
		MOTOR_UpdateMode(Motor, mode);
}

void MOTOR_ResetParams(motor *Motor){
		Motor->DesiredPos_mm = 0;
		Motor->DesiredPos_mm_pre = 0;
		Motor->CurPos_mm = 0;
	
		Motor->DesiredVel_mm = 0;
		Motor->DesiredVel_rad = 0;
	
		Motor->plot_active = 0;
		Motor->auto_run = 0;
		Motor->cycle_count = 0;
		//Motor->cycle_index = 0;
		Motor->tick = 0;
		Motor->tick_enabled = 0;
		Motor->check = 0;
		MOTOR_PORT->ODR &= ~((1<<Motor->mpin[0]) | (1<<Motor->mpin[1]));
}

void MOTOR_UpdateMode(motor *Motor, MotorMode mode){
		Motor->mode       = mode;
		if(mode == MOTOR_P) Motor->err_pos = 0.5;
		else Motor->err_pos = 0.01;
}
void MOTOR_UpdateVel(motor *Motor, float vel){
		Motor->v_max_mm	  =	vel;
		if (Motor->v_max_mm >= MOTOR_MAXVEL_MM) Motor->v_max_mm = MOTOR_MAXVEL_MM;
    if (Motor->v_max_mm <= MOTOR_MINVEL_MM) Motor->v_max_mm = MOTOR_MINVEL_MM;
		Motor->a_max_mm	  = (Motor->v_max_mm/MOTOR_RESPONSETIME)*MOTOR_MAXACC_SF;
}
void MOTOR_UpdatePos(motor *Motor, float pos){
		Motor->DesiredPos_mm	  =	pos;
		if (Motor->DesiredPos_mm >= SYS_MAXLENGTH) Motor->DesiredPos_mm = SYS_MAXLENGTH;
    if (Motor->DesiredPos_mm <= SYS_MINLENGTH) Motor->DesiredPos_mm = SYS_MINLENGTH;
}

void MOTOR_CalculateParams(motor *Motor, encoder *Encoder, float sign, MotionPhase_t phase){
    Encoder->Cnttmp 	= Encoder->CntVel;
    Encoder->CntVel 	= 0;
    Motor->CurVel_rad = sign * Encoder->Cnttmp * MOTOR_VEL_CONVERSION_FACTOR_RAD;
    Motor->CurPos_mm 	= ((float)Encoder->PosCnt + (float)Encoder->CountValue / (float)Encoder_ppr) * SYS_PB;
    
    // Chuyển đổi vận tốc sang mm/s để so sánh
    if(Motor->mode == MOTOR_P) Motor->CurVel_mm = Motor->CurVel_rad;
    else Motor->CurVel_mm = Motor->CurVel_rad * SYS_RAD2MM;
    
    // Lọc nhiễu: nếu vận tốc thực tế < ngưỡng nhiễu thì coi như 0
    if(fabs(Motor->CurVel_mm) < MOTOR_DEADZONE_VEL) {
        Motor->CurVel_mm = 0.0f;
    }
//else Motor->state = MOTOR_ACCEL;
    
    Motor->vel_diff = Motor->CurVel_mm - Motor->CurVel_mm_pre;

    //MOTOR_CalculateTime_v1(Motor, time, phase);
    if(fabs(Motor->DesiredPos_mm - Motor->CurPos_mm) < Motor->err_pos && phase == TRAJ_PHASE_COMPLETE){
        Motor->plot_active = 0;
    }
		
    Motor->CurVel_mm_pre = Motor->CurVel_mm;
    
    // Tính gia tốc thực tế
    Motor->Acc_mm = 0.8f * Motor->Acc_mm + 0.2f * ((Motor->vel_diff) / SYS_SAMPLETIME);
}

// void MOTOR_CalculateTime(motor *Motor) {
//     static float t_stable_count = 0.0f;    // Thời gian ổn định
//     static float prev_pos = 0.0f;          // Vị trí trước để để kiểm tra
//     static bool is_stabilizing = false;    // Đang trong quá trình ổn định
    
//     float error_at_max_speed = fabs(Motor->v_max_mm - Motor->CurVel_mm);
    
//     // ========== GIAI ĐOẠN ỔN ĐỊNH (sau khi giảm tốc) ==========
//     // Phát hiện bắt đầu ổn định: DesiredSpeed về 0 Và CurVel gần 0
//     if(fabs(Motor->DesiredVel_mm) < MOTOR_ACCEL_START_THRESHOLD && 
//        fabs(Motor->CurVel_mm) < MOTOR_DEADZONE_VEL && 
//        Motor->state == MOTOR_DECEL && 
//        !is_stabilizing) {
//         is_stabilizing = true;
//         t_stable_count = 0.0f;
//         prev_pos = Motor->CurPos_mm;
//         Motor->state = MOTOR_IDLE;
//     }
//     if(fabs(Motor->DesiredPos_mm - Motor->CurPos_mm) < 0.01f){
//         is_stabilizing = false;
//         t_stable_count = 0.0f;
//         Motor->t_motor_adjust = 0.0f;
//         Motor->state = MOTOR_IDLE;
//         Motor->plot_active = 0;
//     }
//     // Đang trong giai đoạn ổn định
//     if(is_stabilizing) {
//         float pos_change = fabs(Motor->CurPos_mm - prev_pos);
        
//         // Nếu vị trí thay đổi -> reset bộ đếm (động cơ vẫn đang điều chỉnh)
//         if(pos_change > 0.001f)  {
//             t_stable_count = 0.0f;
//             prev_pos = Motor->CurPos_mm;
//             Motor->t_motor_adjust += SYS_SAMPLETIME;  // Tích lũy thời gian tự điều chỉnh
//         }
//         else {
//             // Vị trí không đổi -> tăng bộ đếm
//             t_stable_count += SYS_SAMPLETIME;
//         }
        
//         // Nếu đã ổn định đủ lâu
//         if(t_stable_count >= MOTOR_STABILIZE_TIME) {
//             is_stabilizing = false;
//             t_stable_count = 0.0f;
//             Motor->t_motor_adjust = 0.0f;
//             Motor->state = MOTOR_IDLE;
//             Motor->plot_active = 0;
//         }
        
//         // Trong lúc ổn định, không cho phép chuyển state
//         Motor->prev_state = Motor->state;
//         Motor->CurVel_mm_pre = Motor->CurVel_mm;
//         return;
//     }
    
//     // ========== LOGIC CHUYỂN TRẠNG THÁI CHÍNH ==========
//     // --- IDLE -> ACCEL: Bắt đầu tăng tốc ---
//     // Dựa vào DesiredSpeed (và CurVel có deadzone)
//     if(Motor->state == MOTOR_IDLE && 
//        fabs(Motor->DesiredVel_mm) > MOTOR_ACCEL_START_THRESHOLD && 
// 			 fabs(Motor->CurVel_mm) > MOTOR_DEADZONE_VEL &&
//        !is_stabilizing) {
//         Motor->state = MOTOR_ACCEL;
//     }
    
//     // --- ACCEL: Đang tăng tốc ---
//     if(Motor->state == MOTOR_ACCEL) {
//         // Chuyển sang CONSTANT khi:
//         // 1. Vận tốc thực tế đạt gần max
//         // 2. Vận tốc thay đổi nhỏ (ổn định)
//         if(error_at_max_speed <= MOTOR_MAX_SPEED_TOLERANCE && 
//            fabs(Motor->vel_diff) <= MOTOR_MAX_SPEED_TOLERANCE) {
//             Motor->state = MOTOR_CONSTANT;
//             Motor->tc_actual += SYS_SAMPLETIME;
//         }
//         else {
//             Motor->ta_actual += SYS_SAMPLETIME;
//         }
//     }
    
//     // --- CONSTANT: Vận tốc không đổi ---
//     else if(Motor->state == MOTOR_CONSTANT) {
//         // Chuyển sang DECEL khi:
//         // 1. DesiredSpeed giảm dần kề (dấu hiệu bắt đầu giảm tốc)
//         // 2. HOẶC vận tốc thực tế giảm dần kề
//         if(Motor->DesiredVel_mm < (error_at_max_speed) || 
//            Motor->vel_diff < -MOTOR_MAX_SPEED_TOLERANCE) {
//             Motor->state = MOTOR_DECEL;
//             Motor->td_actual += SYS_SAMPLETIME;
//         }
//         else {
//             Motor->tc_actual += SYS_SAMPLETIME;
//         }
//     }
    
//     // --- DECEL: Đang giảm tốc ---
//     else if(Motor->state == MOTOR_DECEL) {
//         Motor->td_actual += SYS_SAMPLETIME;
//         // Sẽ chuyển sang ổn định ở đầu hàm (để xử lý)
//     }
    
//     // Cập nhật tổng thời gian (chỉ khi không IDLE)
//     if(Motor->state != MOTOR_IDLE) {
//         Motor->time_actual += SYS_SAMPLETIME;
//     }
    
//     // ưu trạng thái vận tốc cho lần sau
//     Motor->prev_state = Motor->state;    
// }

// void MOTOR_CalculateTime_v1(motor *Motor, MotionPhase_t phase ){
// 		if(fabs(Motor->DesiredPos_mm - Motor->CurPos_mm) < 0.01f && phase == TRAJ_PHASE_COMPLETE){
//         Motor->t_motor_adjust = 0.0f;
//         Motor->plot_active = 0;
// 				//*time = Motor->time_actual;
// 				Motor->state = MOTOR_IDLE;
// 				MOTOR_ResetTime(Motor);
//     }
// 		else if(Motor->state != MOTOR_IDLE){
// 				 Motor->time_actual += SYS_SAMPLETIME;
// 		}
// }

// void MOTOR_ResetTime(motor *Motor) {
//     Motor->time_actual = 0.0f;
//     Motor->ta_actual = 0.0f;
//     Motor->tc_actual = 0.0f;
//     Motor->td_actual = 0.0f;
//     Motor->t_motor_adjust = 0.0f;
//     Motor->state = MOTOR_IDLE;
//     Motor->prev_state = MOTOR_IDLE;
//     Motor->CurVel_mm_pre = 0.0f;
// }


