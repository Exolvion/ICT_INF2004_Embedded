/** @file mission_terrain.h
 * @brief The mission: subsystem 4: the hump (ramp), side leans and the lift
 * stop; the calls the other parts make into mission_terrain.c. Include
 * mission_private.h first.
 */
#ifndef MISSION_TERRAIN_H
#define MISSION_TERRAIN_H
bool bench_hold_lets_go (void);
void bench_ramp_start (void);
void bench_ramp_axes (const double * p_mg, double * p_mps2);
void bench_ramp_step (uint64_t now_us,
                             const motion_output_t * p_output);
void bench_lift_check (uint64_t now_us, const double * p_axes,
                              uint64_t dt_us,
                              const motion_output_t * p_output);
void bench_ramp_offset (double ds);
void bench_ramp_lean (uint64_t now_us, double ds, int32_t centre_mm);
double bench_lean_height (const bench_lean_t * p_lean);
bool bench_lean_wheel_up (const bench_lean_t * p_lean);
bool bench_rise_wheel_up (const bench_rise_t * p_rise);
void bench_ramp_lean_line (const char * p_head,
                                  const bench_lean_t * p_lean);
void bench_ramp_rise (uint64_t now_us, int32_t path,
                             int32_t centre_mm);
bool bench_ramp_on_straight (void);
bool bench_ramp_hold (void);
bool bench_ramp_text (void);
void bench_ramp_status (void);
void bench_ramp_deg (double rad);
void bench_ramp_side (double roll);
void bench_ramp_results (void);

#endif /* MISSION_TERRAIN_H */

/* End of file. */

