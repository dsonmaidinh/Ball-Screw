#ifndef TRAJECTORY_v1_H
#define TRAJECTORY_v1_H

#include <stdint.h>
#include <math.h>

// MOTION PHASES
typedef enum {
    PHASE_ACCEL = 0,
    PHASE_CONSTANT,
    PHASE_DECEL,
    PHASE_STOP,
    PHASE_COMPLETE
} MotionPhase_t;

// === MULTI-SEGMENT TRAJECTORY ===
typedef struct {
    float total_distance_mm;
    float max_segment_mm;
    float lead_screw_pitch;
    
    uint8_t num_segments;
    uint8_t current_segment;
    float segment_length_mm;
    
    float starting_pos_rad;
    uint8_t auto_continue;
    
    uint8_t use_fixed_100mm;  // C? d�ng h�m 100mm
} MultiSegment_t;
// === IMPROVED PROFILE GENERATOR (Based on Heo et al. 2019) ===
typedef struct {
    // �? l?i di?u khi?n
    float Kpp;              // P gain c?a position controller
    float Kest;             // Estimation gain (0.95-1.05)
    float Kcom;             // Compensation gain (0.95-1.05)

    // Trạng thái bộ bù
    float theta_err_init;   // Sai số vị trí ban đầu
    float omega_dec_est;    // Vận tốc ước lượng tại điểm giảm tốc
    float omega_ref_prev;   // Vận tốc tham chiếu trước đó

    uint8_t compensator_enabled;  // Bật/tắt bộ bù feedback
} ImprovedProfileGen_t;

// TRAJECTORY STATE
typedef struct {
    // === CONTROL FLAGS ===
    uint8_t move_active;      // Qu? d?o dang ch?y
    MotionPhase_t phase;      // Phase hi?n t?i
    
    // === DESIRED VALUES (OUTPUT) ===
    float q_desired;          // V? tr� d?t [rad]
    float v_desired;          // V?n t?c d?t [rad/s]
    
    // === STATE MEMORY ===
    float q_desired_prev;     // V? tr� chu k? tru?c
    float v_desired_prev;     // V?n t?c chu k? tru?c
    
    // === TRAJECTORY PARAMETERS ===
    float q_i;                // V? tr� b?t d?u [rad]
    float q_f;                // V? tr� k?t th�c [rad]
    float sign;               // Hu?ng: +1 xu�i, -1 ngu?c
    
    float t_f;                // Th?i gian ho�n th�nh [s]
    float t_c;                // Th?i gian gia t?c [s]
    float t_stop;             // Th?i gian d?ng [s]
    float Ts;                 // Chu k? m?u [s]
    
    float velocity_max;       // V?n t?c d?nh [rad/s]
    float accel_desired;      // Gia t?c (bi�n d? duong) [rad/s�]
    float accel_desired_plot; 
    
    // === HARDWARE LIMITS ===
    float accel_max;          // Gia t?c t?i da [rad/s�]
    
    float move_time;          // Th?i gian ch?y hi?n t?i
    float error_time;         // Th?i gian ph�t hi?n l?i
    uint8_t check_error;      // C? ki?m tra l?i
    uint8_t cycle_active;     // C? k�ch ho?t chu k?
    
    uint8_t test;
    uint8_t plot_active;
    MultiSegment_t multi;
    
    // === IMPROVED PROFILE GENERATOR ===
    ImprovedProfileGen_t improved;  // Th�m c?u tr�c b? t?o profile c?i ti?n
} Trajectory_t;

// === FUNCTION PROTOTYPES ===
void TrajParams(Trajectory_t *traj, float Ts, float t_move, float t_stop, float accel_max);
void SetMoveTrapezoidal(Trajectory_t *traj, float target_pos, float current_pos);
void DoMove(Trajectory_t *traj);

// === IMPROVED PROFILE FUNCTIONS ===
void TrajParams_Improved(Trajectory_t *traj, float Ts, float t_move, float t_stop, 
                         float accel_max, float Kpp, float Kest, float Kcom);
void SetMoveTrapezoidal_Improved(Trajectory_t *traj, float target_pos, float current_pos);
void DoMove_Improved(Trajectory_t *traj, float current_pos);

// === MULTI-SEGMENT FUNCTIONS ===
void SetupMultiSegmentMove(Trajectory_t *traj, float distance_mm, float pitch_mm, float max_seg_mm);
uint8_t ContinueNextSegment(Trajectory_t *traj);

// === FIXED 100mm SEGMENT ===
void SetupMultiSegmentMove_100mm(Trajectory_t *traj, float target_pos_mm, float pitch_mm);
uint8_t ContinueNextSegment_100mm(Trajectory_t *traj);

#endif