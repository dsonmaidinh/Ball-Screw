#include <stdbool.h>
#include "trajectory.h"
#include "system_config.h"

// === PRIVATE FUNCTIONS ===
static void CalculateTrapezoidalProfile(Trajectory_t *traj, float q_start, float q_end);
static inline float clamp(float value, float min, float max);

static void CalculateTrapezoidalProfile(Trajectory_t *traj, float q_start, float q_end) {
    traj->q_i = q_start;
    traj->q_f = q_end;

    float dq = fabsf(q_end - q_start);  // |?q|
    
    // Gia t?c t?i thi?u: qc_2dot >= 4?q/t_f²
    float qc_2dot_min = 4.0f * dq / (traj->t_f * traj->t_f);
    
    // Ch?n gia t?c: min n?u d?, max n?u không d?
    float qc_2dot = (qc_2dot_min <= traj->accel_max_mm*0.8) ? qc_2dot_min : traj->accel_max_mm*0.8;
		        
    // Tính t_c t? gia t?c qc_2dot
    float discriminant = traj->t_f * traj->t_f - 4.0f * dq / qc_2dot;
    
    if (discriminant < 0.0f) {
        // Triangular profile (5.13)
        traj->t_c = traj->t_f * 0.5f;
    } else {
        // Trapezoidal profile
        traj->t_c = 0.5f * (traj->t_f - sqrtf(discriminant));
    }
    
    // Gi?i h?n: 0 <= t_c <= t_f/2
    traj->t_c = clamp(traj->t_c, 0.0f, traj->t_f * 0.5f);

    // V?n t?c d?nh và gia t?c
    traj->velocity_max_mm = qc_2dot * traj->t_c;
    traj->accel_desired = qc_2dot;
    traj->accel_desired_plot = qc_2dot;
}

void TRAJ_CalculateParamsTrapezoidal(Trajectory_t *traj, float target_pos, float current_pos) {
    // Xác d?nh hu?ng
    float delta = target_pos - current_pos;
    traj->sign = (delta >= 0.0f) ? 1.0f : -1.0f;
    
    // Reset state
    traj->q_desired_prev = current_pos;
    traj->v_desired_prev = 0.0f;
    traj->v_desired = 0.0f;
    traj->phase = TRAJ_PHASE_ACCEL;
    
    // Tính toán profile
    CalculateTrapezoidalProfile(traj, current_pos, target_pos);
    
    // Kích ho?t
    traj->move_active  = 1;
	  traj->plot_active  = 1;

    // Reset th?i gian
    traj->total_time    = 0.0f;
}

void TRAJ_DrawTrajectory(Trajectory_t *traj) {
    if (!traj->move_active) {
        if (traj->multi.auto_continue && traj->phase == TRAJ_PHASE_COMPLETE && TRAJ_ContinueNextSegment_100mm(traj)) {
            return;
        }
				traj->plot_active = 0;
        
        return;
    }
    // C?p nh?t th?i gian
    traj->total_time += traj->Ts;
    float t = traj->total_time;
    
    // === STATE MACHINE ===
    float accel_current;
    
    if (t < traj->t_c) {
        // PHASE 1: ACCELERATION
        traj->phase = TRAJ_PHASE_ACCEL;
        accel_current = traj->sign * traj->accel_desired;
				traj->accel_desired_plot = accel_current;
        
    } else if (t < (traj->t_f - traj->t_c)) {
        // PHASE 2: CONSTANT VELOCITY
        traj->phase = TRAJ_PHASE_CONSTANT;
        accel_current = 0.0f;
				traj->accel_desired_plot = accel_current;
        
    } else if (t <= traj->t_f) {
        // PHASE 3: DECELERATION
        traj->phase = TRAJ_PHASE_DECEL;
        accel_current = -traj->sign * traj->accel_desired;
				traj->accel_desired_plot = accel_current;
        
    } else if (t <= (traj->t_f + traj->t_stop)) {
        // PHASE 4: STOP
        traj->phase = TRAJ_PHASE_STOP;
        traj->v_desired = 0.0f;
        traj->q_desired = traj->q_f;
				traj->accel_desired_plot = 0.0;
        return;
        
    } else {
        // PHASE 5: COMPLETE
        traj->phase = TRAJ_PHASE_COMPLETE;
        traj->move_active = 0;
        return;
    }

    // === DISCRETE INTEGRATION (5.22) ===
    // v[k] = v[k-1] + a*Ts
    float v_next = traj->v_desired_prev + accel_current * traj->Ts;
    
    // Gi?i h?n v?n t?c
    float v_limit = traj->sign * traj->velocity_max_mm;
    if (traj->sign > 0.0f) {
        v_next = clamp(v_next, 0.0f, v_limit);
    } else {
        v_next = clamp(v_next, v_limit, 0.0f);
    }

    // q[k] = q[k-1] + v[k-1]*Ts + 0.5*a*Ts²
    float q_next = traj->q_desired_prev 
                 + traj->v_desired_prev * traj->Ts 
                 + 0.5f * accel_current * traj->Ts * traj->Ts;


    // C?p nh?t state
    traj->v_desired_prev = v_next;
    traj->q_desired_prev = q_next;
    traj->v_desired = v_next;
    traj->q_desired = q_next;
}

void TRAJ_SetupMultiSegmentMove_100mm(Trajectory_t *traj, float target_pos) {
    traj->multi.total_distance = target_pos - traj->q_desired_prev;
    traj->multi.max_segment_mm = 100.0f;  // Kho?ng cách t?i da m?i do?n
    traj->multi.starting_pos   = 0.0f;    // Ði?m b?t d?u c?a do?n hi?n t?i
    
    traj->multi.sign = (traj->multi.total_distance >= 0.0f) ? 1.0f : -1.0f;
    float abs_distance = fabsf(traj->multi.total_distance);
    
    // === CHIA ÐO?N THEO QUY T?C: 100mm + 100mm + ... + ph?n du ===
    if (abs_distance <= 100.0f) {
        // Nh? hon ho?c b?ng 100mm chia 1 do?n
        traj->multi.num_segments = 1;
        traj->multi.segment_length_mm = traj->multi.total_distance;
    } else {
        // L?n hon 100mm ? chia nhi?u do?n
        uint8_t full_segments = (uint8_t)(abs_distance / 100.0f);   // S? do?n 100mm d?y d?
        traj->multi.remaining_distance_mm = abs_distance - (full_segments * 100.0f);
        
        if (traj->multi.remaining_distance_mm > 0.0f) {
            // Có ph?n du -> full_segments + 1
            traj->multi.num_segments = full_segments + 1;
        } else {
            // Không có ph?n du
            traj->multi.num_segments = full_segments;
        }
    }
    
    traj->multi.current_segment = 0;
    traj->multi.auto_continue = 1;  // M?c d?nh t? d?ng chuy?n
}

uint8_t TRAJ_ContinueNextSegment_100mm(Trajectory_t *traj) {
    // Ki?m tra dã hoàn thành chua
    if (traj->multi.current_segment >= traj->multi.num_segments) {
        return 0;  // Ðã xong t?t c?
    }
    
    // Luu v? trí b?t d?u ? l?n d?u
    if (traj->multi.current_segment == 0) {
        traj->multi.starting_pos = traj->q_desired_prev;
    }
    float target;
    float accumulated_distance;
    // === TÍNH CHI?U DÀI ÐO?N HI?N T?I ===
    if (traj->multi.current_segment < traj->multi.num_segments - 1) {
        // Các do?n t? 0 d?n n-2: luôn là 100mm
        accumulated_distance = traj->multi.sign * (traj->multi.current_segment + 1) * 100.0f;
    }
    else {
        // Ðo?n cu?i: dùng ph?n du
        accumulated_distance = traj->multi.total_distance;
    }
    // Luu v? trí hi?n t?i
    float current = traj->q_desired_prev;
    target = traj->multi.starting_pos + accumulated_distance;
    // B?t d?u trajectory cho do?n này
    TRAJ_CalculateParamsTrapezoidal(traj, target, current);
    
    traj->multi.current_segment++;
    return 1;  // Ðang ch?y
}

void TRAJ_SetParams(Trajectory_t *traj, float Ts, float t_move, float t_stop, float accel_max, float v_max_mm) {
    traj->Ts = Ts;
    traj->t_f = t_move;
    traj->t_stop = t_stop;
    traj->accel_max_mm = accel_max;
    traj->multi.use_fixed_100mm = 1;
}

uint8_t TRAJ_ContinueNextSegment_100mm_P(Trajectory_t *traj, float target_pos_mm, float current_pos) {
    if(!traj->move_active) return 0;
    
    static uint8_t first_call = 1;
    static float start_pos_mm = 0;
    static float t = 0;
    static uint8_t waiting = 0;
    static uint8_t moving = 0;
    
    // === L?N Ð?U: SETUP ===
    if(first_call) {
        traj->multi.starting_pos = start_pos_mm;
        float abs_distance = fabsf(target_pos_mm - start_pos_mm);
        traj->multi.sign = ((target_pos_mm - start_pos_mm) >= 0.0f) ? 1.0f : -1.0f;
        
        // === CHIA ÐO?N THEO QUY T?C: 100mm + 100mm + ... + ph?n du ===
        if (abs_distance <= 100.0f) {
            // Nh? hon ho?c b?ng 100mm chia 1 do?n
            traj->multi.num_segments = 1;
            traj->multi.segment_length_mm = traj->multi.total_distance;
        } else {
            // L?n hon 100mm ? chia nhi?u do?n
            uint8_t full_segments = (uint8_t)(abs_distance / 100.0f);   // S? do?n 100mm d?y d?
            traj->multi.remaining_distance_mm = abs_distance - (full_segments * 100.0f);
            
            if (traj->multi.remaining_distance_mm > 0.0f) {
                // Có ph?n du -> full_segments + 1
                traj->multi.num_segments = full_segments + 1;
            } else {
                // Không có ph?n du
                traj->multi.num_segments = full_segments;
            }
        }
        
        traj->multi.current_segment = 0;
        first_call = 0;
        t = 0;
        waiting = 0;
        moving = 0;
    }
    
    // === KI?M TRA HOÀN THÀNH T?T C? ===
    if (traj->multi.current_segment >= traj->multi.num_segments) {
        start_pos_mm = target_pos_mm;
        traj->multi.starting_pos = target_pos_mm;
        traj->multi.current_segment = 0;
        traj->multi.num_segments = 0;
        traj->q_desired_prev = traj->q_desired;
        first_call = 1;
        t = 0;
        waiting = 0;
        moving = 0;
        traj->move_active = 0;
        return 0;
    }
    
    // === TÍNH TARGET CHO ÐO?N HI?N T?I ===
    float accumulated_distance;
    
    if (traj->multi.current_segment < traj->multi.num_segments - 1) {
        accumulated_distance = traj->multi.sign * (traj->multi.current_segment + 1) * 100.0f;
    } else {
        accumulated_distance = target_pos_mm - traj->multi.starting_pos;
    }
    
    traj->q_desired = traj->multi.starting_pos + accumulated_distance;
    
    // === KI?M TRA VÀ Ð?M TH?I GIAN ===
    float error = fabsf(traj->q_desired - current_pos);
    
    if(error > 0.009f) {
        moving = 1;
    }
    
    if(waiting) {
        t += traj->Ts;
        
        if(t >= traj->t_stop) {
            traj->multi.current_segment++;
            t = 0;
            waiting = 0;
            moving = 0;
        }
        
    } else {
        if(error < 0.09f && moving) {
            waiting = 1;
            t = 0;
        }
    }
    
    return 1;
}
static inline float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
