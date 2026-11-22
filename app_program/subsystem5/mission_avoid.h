/** @file mission_avoid.h
 * @brief The mission: subsystem 5: the front sonar and the obstacle go-
 * around; the calls the other parts make into mission_avoid.c. Include
 * mission_private.h first.
 */
#ifndef MISSION_AVOID_H
#define MISSION_AVOID_H
void bench_servo (uint8_t command);
void bench_avoid_hold (double target, motion_request_t * p_request);
void bench_front_start (void);
bool bench_front_step (uint64_t now_us);
void bench_front_results (void);
void bench_avoid_drive (uint64_t now_us, bool b_range, int32_t centre,
                               motion_request_t * p_request);
void bench_avoid_begin (uint64_t now_us, int32_t centre);
void bench_avoid_rejoin (uint64_t now_us, int32_t centre);
bool bench_avoid_guard (bool b_range, motion_request_t * p_request);
bool bench_avoid_line_fix (void);
void bench_avoid_blocked (uint64_t now_us, double half,
                                 motion_request_t * p_request);
double bench_avoid_wrap (double angle);
bool bench_avoid_pivot (double target, motion_request_t * p_request);
bool bench_avoid_pivot_from (double home, double target,
                                    motion_request_t * p_request);
void bench_avoid_turn_begin (double target);
bool bench_avoid_turn_line (double target, double window,
                                   double * p_line_rad);
bool bench_avoid_turn_check (double target,
                                    motion_request_t * p_request);
void bench_avoid_home_fix (void);
void bench_avoid_ir_setup (void);
void bench_avoid_ir_say (void);
void bench_avoid_ir_peak (const uint16_t * p_raw);
void bench_avoid_peak_say (void);
bool bench_avoid_median (uint16_t * p_mm);
bool bench_avoid_edge (double outward);
bool bench_avoid_window_beyond (void);
double bench_avoid_shown (int32_t side);
void bench_avoid_far_found (motion_request_t * p_request, bool b_past,
                                   double past_mm);
void bench_avoid_creep (uint64_t now_us, double turned,
                               motion_request_t * p_request);
void bench_avoid_creep_found (uint64_t now_us,
                                     motion_request_t * p_request);
void bench_avoid_keep_beam (double angle, uint16_t range_mm);
bool bench_avoid_beam (double car_angle, double beam_angle,
                              uint16_t range_mm);
void bench_avoid_mark (double x_mm, double y_mm, uint8_t code);
bool bench_avoid_side_clear (int32_t side, double out_mm);
void bench_avoid_plan (uint64_t now_us, motion_request_t * p_request);
void bench_avoid_map_print (void);
bool bench_avoid_map_row (uint32_t row, int32_t car_row,
                                 int32_t car_col, char * p_line);
void bench_avoid_obst_text (void);
void bench_avoid_say (void);
bool bench_avoid_text (void);
void bench_avoid_live (uint64_t now_us);
bool bench_beams_text (void);
void bench_scan_start (void);
void bench_scan_tick (uint8_t command);
bool bench_scan_servo (int32_t angle_deg);
void bench_scan_finish (const char * p_reason);
void bench_scan_verdict (void);
void bench_scan_beams (void);
void bench_scan_mm (double metres);

#endif /* MISSION_AVOID_H */

/* End of file. */

