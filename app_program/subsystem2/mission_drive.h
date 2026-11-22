/** @file mission_drive.h
 * @brief The mission: subsystem 2: drive, pivot and wheel-controller
 * settings; the calls the other parts make into mission_drive.c. Include
 * mission_private.h first.
 */
#ifndef MISSION_DRIVE_H
#define MISSION_DRIVE_H
void bench_sweep_start (void);
void bench_sweep_tick (uint8_t command);
void bench_sweep_finish (const char * p_reason);
void bench_step_place (uint32_t step, int8_t * p_duty_pct,
                              bool * p_down);
void bench_wheel_summary (int32_t direction, uint32_t wheel);
void bench_drive_start (void);
void bench_drive_request (uint32_t step);
void bench_drive_tick (uint8_t command);
void bench_pid_tick (uint64_t now_us, uint32_t dt_us,
                            const motion_output_t * p_output);
void bench_pid_sample (uint32_t wheel, double speed, double duty,
                              uint64_t t_us, uint32_t dt_us);
void bench_move_tick (uint64_t now_us,
                             const motion_output_t * p_output);
void bench_drive_finish (const char * p_reason);
void bench_pid_results (const char * p_reason);
void bench_pid_wheel (const bench_wheel_t * p_wheel, uint32_t wheel);
void bench_move_results (const char * p_reason);
bool bench_pivot_mode (bench_mode_t mode);
bool bench_pivot_tick (void);
void bench_pivot_results (void);
void bench_move_line (const char * p_label, const int32_t * p_counts,
                             double yaw);
bool bench_pose_text (uint64_t now_us);
void bench_gains_text (void);
void bench_marks (const char * p_text, const char * p_alphabet,
                         uint32_t bits);

#endif /* MISSION_DRIVE_H */

/* End of file. */

