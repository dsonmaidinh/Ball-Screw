#include <stdbool.h>
#include "trajectory.h"
#include "system_config.h"
// === PRIVATE FUNCTIONS ===
static void CalculateTrapezoidalProfile(Trajectory_t *traj, float q_start, float q_end);
static inline float clamp(float value, float min, float max);

static inline void CalculateTrapezoidalProfile(Trajectory_t *traj, float q_start, float q_end) {
    traj->q_i = q_start;
    traj->q_f = q_end;

    float dq = fabsf(q_end - q_start);  // |Δq|
    
    // Gia tốc tối thiểu: qc_2dot >= 4Δq/t_f²
//    float qc_2dot_min = 4.0f * dq / (traj->t_f * traj->t_f);
    
    float qc_2dot = traj->accel_max_mm;
		        
    // Tính t_c từ gia tốc qc_2dot
    float discriminant = traj->t_f * traj->t_f - 4.0f * dq / qc_2dot;
    
    if (discriminant < 0.0f) {
        // Triangular profile (5.13)
        traj->t_c = traj->t_f * 0.5f;
    } else {
        // Trapezoidal profile
        traj->t_c = 0.5f * (traj->t_f - sqrtf(discriminant));
    }
    
    // Giới hạn: 0 <= t_c <= t_f/2
    traj->t_c = clamp(traj->t_c, 0.0f, traj->t_f * 0.5f);
		
		traj->dv = traj->velocity_max_mm*traj->Ts/traj->t_c;
		
    // Vận tốc đỉnh và gia tốc
    traj->accel_desired = qc_2dot;
    traj->accel_desired_plot = qc_2dot;
}

static inline void CalculateTrapezoidalProfile_v1(Trajectory_t *traj, float q_start, float q_end) {
    traj->q_i = q_start;
    traj->q_f = q_end;

    float dq = fabsf(q_end - q_start);  // |Δq|
    
    // Gia tốc tối thiểu: qc_2dot >= 4Δq/t_f²
    float qc_2dot_min = 4.0f * dq / (traj->t_f * traj->t_f);
    
    // Chọn gia tốc: min nếu đủ, max nếu không đủ
    float qc_2dot = qc_2dot_min;
		        
    // Tính t_c từ gia tốc qc_2dot
    float discriminant = traj->t_f * traj->t_f - 4.0f * dq / qc_2dot;
    
    if (discriminant < 0.0f) {
        // Triangular profile (5.13)
        traj->t_c = traj->t_f * 0.5f;
    } else {
        // Trapezoidal profile
        traj->t_c = 0.5f * (traj->t_f - sqrtf(discriminant));
    }
    
    // Giới hạn: 0 <= t_c <= t_f/2
    traj->t_c = clamp(traj->t_c, 0.0f, traj->t_f * 0.5f);

    // Vận tốc đỉnh và gia tốc
    traj->velocity_max_mm = qc_2dot * traj->t_c;
    traj->accel_desired = qc_2dot;
    traj->accel_desired_plot = qc_2dot;
}

static inline void TRAJ_CalculateParamsTrapezoidal(Trajectory_t *traj, float target_pos, float current_pos) {
    // Xác định hướng
    float delta = target_pos - current_pos;
    traj->sign = (delta >= 0.0f) ? 1.0f : -1.0f;
    
    // Reset state
    traj->q_desired_prev = current_pos;
    traj->v_desired_prev = 0.0f;
    traj->v_desired = 0.0f;
    traj->phase = TRAJ_PHASE_ACCEL;
    
    // Tính toán profile
    if(traj->use_v1) CalculateTrapezoidalProfile_v1(traj, current_pos, target_pos);
		else CalculateTrapezoidalProfile(traj, current_pos, target_pos);
    
    // Kích hoạt
    traj->move_active  = 1;
	  traj->plot_active  = 1;

    // Reset thời gian
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
    traj->count++;
    traj->total_time += traj->Ts;
    float t = traj->total_time;
    
    // === STATE MACHINE ===
    
    if (t <= traj->t_c) {
        // PHASE 1: ACCELERATION
        traj->phase = TRAJ_PHASE_ACCEL;
        traj->accel_desired_plot = traj->sign * traj->accel_desired;
        traj->q_desired = traj->q_i + 0.5 * traj->sign * traj->accel_desired * t * t;
        
        // Tích lũy vận tốc từng bước như Python
        traj->v_desired = traj->v_desired_prev + traj->sign * traj->dv;
        
        // Giới hạn v_desired không vượt quá v_peak
        if (fabsf(traj->v_desired) > traj->velocity_max_mm) {
            traj->v_desired = traj->sign * traj->velocity_max_mm;
        }
        
    } else if (t <= (traj->t_f - traj->t_c)) {
        // PHASE 2: CONSTANT VELOCITY
        traj->phase = TRAJ_PHASE_CONSTANT;
        traj->accel_desired_plot = 0.0;
        traj->q_desired = traj->q_i + traj->sign * traj->accel_desired * traj->t_c * (t - traj->t_c/2.0);
        traj->v_desired = traj->sign * traj->velocity_max_mm;  // v_peak
        
    } else if (t <= traj->t_f) {
        // PHASE 3: DECELERATION
        traj->phase = TRAJ_PHASE_DECEL;
        traj->accel_desired_plot = -traj->sign * traj->accel_desired;
        traj->q_desired = traj->q_f - 0.5 * traj->sign * traj->accel_desired * (traj->t_f - t) * (traj->t_f - t);
        
        // Giảm vận tốc từng bước như Python
        traj->v_desired = traj->v_desired_prev - traj->sign * traj->dv;
        
        // Giới hạn không âm
        if (fabsf(traj->v_desired) < traj->dv) {
            traj->v_desired = 0.0f;
        }
        
    } else if (t <= (traj->t_f + traj->t_stop)) {
        // PHASE 4: STOP
        traj->phase = TRAJ_PHASE_STOP;
        traj->v_desired = 0.0f;
        traj->q_desired = traj->q_f;
        traj->accel_desired_plot = 0.0;
        traj->v_desired_prev = traj->v_desired;
        traj->q_desired_prev = traj->q_desired;
        return;
        
    } else {
        // PHASE 5: COMPLETE
        traj->phase = TRAJ_PHASE_COMPLETE;
        traj->move_active = 0;
        return;
    }
}

void TRAJ_DrawTrajectory_v1(Trajectory_t *traj) {
    if (!traj->move_active) {
        if (traj->multi.auto_continue && traj->phase == TRAJ_PHASE_COMPLETE && TRAJ_ContinueNextSegment_100mm(traj)) {
            return;
        }
				traj->plot_active = 0;
        
        return;
    }
		traj->count++;
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
    traj->multi.max_segment_mm = 100.0f;  // Khoảng cách tối đa mỗi đoạn
    traj->multi.starting_pos   = 0.0f;    // Điểm bắt đầu của đoạn hiện tại
    
    traj->multi.sign = (traj->multi.total_distance >= 0.0f) ? 1.0f : -1.0f;
    float abs_distance = fabsf(traj->multi.total_distance);
    
    // === CHIA ĐOẠN THEO QUY TẮC: 100mm + 100mm + ... + phần dư ===
    if (abs_distance <= 100.0f) {
        // Nhỏ hơn hoặc bằng 100mm chia 1 đoạn
        traj->multi.num_segments = 1;
        traj->multi.segment_length_mm = traj->multi.total_distance;
    } else {
        // Lớn hơn 100mm ở chia nhiều đoạn
        uint8_t full_segments = (uint8_t)(abs_distance / 100.0f);   // Số đoạn 100mm đầy đủ
        traj->multi.remaining_distance_mm = abs_distance - (full_segments * 100.0f);
        
        if (traj->multi.remaining_distance_mm > 0.0f) {
            // Có phần dư -> full_segments + 1
            traj->multi.num_segments = full_segments + 1;
        } else {
            // Không có phần dư
            traj->multi.num_segments = full_segments;
        }
    }
    
    traj->multi.current_segment = 0;
    traj->multi.auto_continue = 1;  // Mặc định tự động chuyển
}

uint8_t TRAJ_ContinueNextSegment_100mm(Trajectory_t *traj) {
    // Kiểm tra đã hoàn thành chưa
    if (traj->multi.current_segment >= traj->multi.num_segments) {
        return 0;  // Đã xong tất cả
    }
    
    // Lưu vị trí bắt đầu ở lần đầu
    if (traj->multi.current_segment == 0) {
        traj->multi.starting_pos = traj->q_desired_prev;
    }
    float target;
    float accumulated_distance;
    // === TÍNH CHIỀU DÀI ĐOẠN HIỆN TẠI ===
    if (traj->multi.current_segment < traj->multi.num_segments - 1) {
        // Các đoạn từ 0 đến n-2: luôn là 100mm
        accumulated_distance = traj->multi.sign * (traj->multi.current_segment + 1) * 100.0f;
    }
    else {
        // Đoạn cuối: dùng phần dư
        accumulated_distance = traj->multi.total_distance;
    }
    // Lưu vị trí hiện tại
    float current = traj->q_desired_prev;
    target = traj->multi.starting_pos + accumulated_distance;
    // Bắt đầu trajectory cho đoạn này
    TRAJ_CalculateParamsTrapezoidal(traj, target, current);
    
    traj->multi.current_segment++;
    return 1;  // Đang chạy
}

void TRAJ_SetParams(Trajectory_t *traj, float Ts, float t_move, float t_stop, float a_max_mm, float v_max_mm) {
    traj->Ts = Ts;
    traj->t_f = t_move;
    traj->t_stop = t_stop;
    traj->accel_max_mm = a_max_mm;
		traj->velocity_max_mm = v_max_mm;
		traj->use_v1 = 1;
}

void TRAJ_ResetParams(Trajectory_t *traj) {
    // Reset các thông số được tính toán trong quá trình chạy trajectory
    traj->q_i = 0.0f;
    traj->q_f = 0.0f;
    traj->t_c = 0.0f;
    traj->dv = 0.0f;
    traj->accel_desired = 0.0f;
    traj->accel_desired_plot = 0.0f;
    traj->sign = 1.0f;
    traj->q_desired_prev = 0.0f;
    traj->v_desired_prev = 0.0f;
    traj->v_desired = 0.0f;
    traj->q_desired = 0.0f;
    traj->phase = TRAJ_PHASE_ACCEL;
    traj->move_active = 0;
    traj->plot_active = 0;
    traj->total_time = 0.0f;
    traj->count = 0;
    
    // Reset multi-segment parameters
    traj->multi.total_distance = 0.0f;
    traj->multi.max_segment_mm = 0.0f;
    traj->multi.starting_pos = 0.0f;
    traj->multi.sign = 1.0f;
    traj->multi.num_segments = 0;
    traj->multi.current_segment = 0;
    traj->multi.auto_continue = 0;
    traj->multi.remaining_distance_mm = 0.0f;
    traj->multi.segment_length_mm = 0.0f;
}

uint8_t TRAJ_ContinueNextSegment_100mm_P(Trajectory_t *traj, float target_pos_mm, float current_pos) {
    if(!traj->move_active) return 0;
    else traj->count++;
    static uint8_t first_call = 1;
    static float start_pos_mm = 0;
    static float t = 0;
    static uint8_t waiting = 0;
    static uint8_t moving = 0;
    
    // === LẦN ĐẦU: SETUP ===
    if(first_call) {
        traj->multi.starting_pos = start_pos_mm;
        float abs_distance = fabsf(target_pos_mm - start_pos_mm);
				traj->multi.total_distance = abs_distance;
        traj->multi.sign = ((target_pos_mm - start_pos_mm) >= 0.0f) ? 1.0f : -1.0f;
        traj->sign = traj->multi.sign;
				traj->phase = TRAJ_PHASE_ACCEL;
        // === CHIA ĐOẠN THEO QUY TẮC: 100mm + 100mm + ... + phần dư ===
        if (abs_distance <= 100.0f) {
            // Nhỏ hơn hoặc bằng 100mm chia 1 đoạn
            traj->multi.num_segments = 1;
            traj->multi.segment_length_mm = traj->multi.total_distance;
        } else {
            // Lớn hơn 100mm ở chia nhiều đoạn
            uint8_t full_segments = (uint8_t)(abs_distance / 100.0f);   // Số đoạn 100mm đầy đủ
            traj->multi.remaining_distance_mm = abs_distance - (full_segments * 100.0f);
            
            if (traj->multi.remaining_distance_mm > 0.0f) {
                // Có phần dư -> full_segments + 1
                traj->multi.num_segments = full_segments + 1;
            } else {
                // Không có phần dư
                traj->multi.num_segments = full_segments;
            }
        }
        
        traj->multi.current_segment = 0;
        first_call = 0;
        t = 0;
        waiting = 0;
        moving = 0;
    }
    
    // === KIỂM TRA HOÀN THÀNH TẤT CẢ ===
    if (traj->multi.current_segment >= traj->multi.num_segments) {
        start_pos_mm = target_pos_mm;
        traj->multi.starting_pos = target_pos_mm;
        traj->multi.current_segment = 0;
        traj->multi.num_segments = 0;
        traj->q_desired_prev = traj->q_desired;
        first_call = 1;
				traj->phase = TRAJ_PHASE_COMPLETE;
        t = 0;
        waiting = 0;
        moving = 0;
        traj->move_active = 0;
        return 0;
    }
    
    // === TÍNH TARGET CHO ĐOẠN HIỆN TẠI ===
    float accumulated_distance;
    
    if (traj->multi.current_segment < traj->multi.num_segments - 1) {
        accumulated_distance = traj->multi.sign * (traj->multi.current_segment + 1) * 100.0f;
    } else {
        accumulated_distance = target_pos_mm - traj->multi.starting_pos;
    }
    
    traj->q_desired = traj->multi.starting_pos + accumulated_distance;
    
    // === KIỂM TRA VÀ ĐẾM THỜI GIAN ===
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
        if(error < 0.9f && moving) {
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
