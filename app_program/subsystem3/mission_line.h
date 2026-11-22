/** @file mission_line.h
 * @brief The mission: subsystem 3: line following, AUTO-CENTRE, calibration
 * and junction moves; the calls the other parts make into mission_line.c.
 * Include mission_private.h first.
 */
#ifndef MISSION_LINE_H
#define MISSION_LINE_H

void mission_ir_drain (void);
void bench_nav_start (void);
void bench_nav_poll_map (void);
void bench_nav_say_map (bench_kind_t kind);
void bench_nav_code (const bench_code_found_t * p_found);
void bench_nav_report (uint32_t who, int32_t from_mm, int32_t to_mm,
                              const char * p_code);
char bench_nav_decide (uint32_t * p_votes, uint32_t * p_readers);
void bench_nav_reads (void);
void bench_nav_say (void);
int32_t bench_nav_move_of (char letter);
void bench_nav_backup (const uint8_t * p_bytes);
bool bench_nav_junction (uint64_t now_us, int32_t centre,
                                motion_request_t * p_request);
void bench_nav_drive (uint64_t now_us, int32_t centre,
                             motion_request_t * p_request);
bool bench_nav_pending (bool b_open_too);
void bench_nav_force (void);
bool bench_nav_wait (uint64_t now_us, int32_t centre,
                            motion_request_t * p_request);
bool bench_nav_probe (uint64_t now_us, int32_t centre,
                             motion_request_t * p_request);
bool bench_is_line (bench_mode_t mode);
double bench_line_speed (bench_mode_t mode);
bool bench_line_start (void);
void bench_line_step (uint64_t now_us, motion_request_t * p_request);
bool bench_line_cross (const line_ctrl_output_t * p_output,
                              int32_t centre_mm, motion_request_t * p_request);
void bench_line_moving (void);
void bench_line_end (uint64_t now_us, const char * p_reason);
void bench_line_trace (void);
void bench_line_print (const char * p_label, uint32_t t_ms,
                              const bench_line_row_t * p_row);
bool bench_is_centre (bench_mode_t mode);
const bench_smooth_t * bench_smooth_set (bench_mode_t mode);
double bench_smooth_track (const bench_smooth_t * p_set, double error,
                                  double travel);
double bench_smooth_gate (uint32_t channel, double darkness,
                                 double travel);
bool bench_centre_straight (void);
bool bench_line_black (const line_ctrl_output_t * p_output);
double bench_centre_steer (const line_ctrl_output_t * p_output,
                                  int32_t centre, uint64_t now_us,
                                  bool * p_lost);
void bench_centre_results (void);
bool bench_straight_code (bench_mode_t mode);
void bench_auto_start (void);
void bench_auto_step (uint64_t now_us,
                             const line_ctrl_output_t * p_output,
                             motion_request_t * p_request);
void bench_auto_seek (uint64_t now_us,
                             const line_ctrl_output_t * p_output);
void bench_auto_found (uint64_t now_us);
void bench_auto_middle (uint64_t now_us, motion_request_t * p_request);
void bench_auto_settle (uint64_t now_us);
void bench_auto_phase (uint64_t now_us, bench_auto_phase_t phase);
void bench_auto_fail (uint64_t now_us, const char * p_reason);
void bench_auto_results (void);
void bench_raw_keep (void);
bool bench_raw_text (void);
void bench_cal_sample (const uint16_t * p_raw, int32_t centre_mm);
bool bench_cal_text (void);

#endif /* MISSION_LINE_H */

/* End of file. */

