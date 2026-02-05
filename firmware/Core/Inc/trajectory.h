#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdint.h>
#include <math.h>

// === PHA CHUYỂN ĐỘNG ===
typedef enum {
    TRAJ_PHASE_ACCEL = 0,
    TRAJ_PHASE_CONSTANT,
    TRAJ_PHASE_DECEL,
    TRAJ_PHASE_STOP,
    TRAJ_PHASE_COMPLETE
} MotionPhase_t;
// === QUỸ ĐẠO NHIỀU ĐOẠN ===
typedef struct {
    float total_distance;
    float max_segment_mm;
    
    uint8_t num_segments;
    uint8_t current_segment;
    float remaining_distance_mm;
    float segment_length_mm;
    float sign;
    
    float starting_pos;
    uint8_t auto_continue;
    
    uint8_t use_fixed_100mm;
} MultiSegment_t;
// === TRẠNG THÁI QUỸ ĐẠO ===
typedef struct {
    //--- CỜ ĐIỀU KHIỂN
    uint8_t move_active;      // Quỹ đạo đang chạy
    uint8_t plot_active;
		uint8_t use_v1;
	
    //--- GIÁ TRỊ MONG MUỐN
    float q_desired;          // Vị trí đặt [mm]
    float v_desired;          // Vận tốc đặt [mm/s]
    
    //--- GIÁ TRỊ TRƯỚC ĐÓ
    float q_desired_prev;     // Vị trí chu kỳ trước
    float v_desired_prev;     // Vận tốc chu kỳ trước
		float v_desired_plot;     // Vận tốc chu kỳ trước
		float dv;
    
    //--- THÔNG SỐ QUỸ ĐẠO
    float q_i;                // Vị trí bắt đầu [mm]
    float q_f;                // Vị trí kết thúc [mm]
    float sign;               // Hướng: +1 xuôi, -1 ngược
    
    float total_time;         // Tổng thời gian chạy
    float t_f;                // Thời gian hoàn thành [s]
    float t_c;                // Thời gian gia tốc [s]
    float t_stop;             // Thời gian dừng [s]
    float Ts;                 // Chu kỳ mẫu [s]
    
    float accel_desired;      // Gia tốc [mm/s²]
		float accel_desired_plot; // Gia tốc hiển thị [mm/s²]
    
    //--- GIỚI HẠN PHẦN CỨNG
    float accel_max_mm;       // Gia tốc tối đa [mm/s²]
		float velocity_max_mm;    // Vận tốc tối đa [mm/s]

		
		MultiSegment_t multi;
    MotionPhase_t phase;      // Phase hiện tại
		int count;
} Trajectory_t;

// === FUNCTION PROTOTYPES ===
void TRAJ_SetParams(Trajectory_t *traj, float Ts, float t_move, float t_stop, float a_max_mm, float v_max_mm);
void TRAJ_ResetParams(Trajectory_t *traj);
//void TRAJ_CalculateParamsTrapezoidal(Trajectory_t *traj, float target_pos, float current_pos);
void TRAJ_DrawTrajectory(Trajectory_t *traj);
void TRAJ_DrawTrajectory_v1(Trajectory_t *traj);

// === NEW: FIXED 100mm SEGMENT ===
void TRAJ_SetupMultiSegmentMove_100mm(Trajectory_t *traj, float target_pos_mm);
uint8_t TRAJ_ContinueNextSegment_100mm(Trajectory_t *traj);
uint8_t TRAJ_ContinueNextSegment_100mm_P(Trajectory_t *traj, float target_pos_mm, float current_pos);

#endif
