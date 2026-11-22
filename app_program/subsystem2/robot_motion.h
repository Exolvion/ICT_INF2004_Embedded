/** @file robot_motion.h
 * @brief The robot app: subsystem 2: the Motion task (wheel controller,
 * encoders, stalls, the approved reset); the calls the other parts make into
 * robot_motion.c. Include robot_app_private.h first.
 */
#ifndef ROBOT_MOTION_H
#define ROBOT_MOTION_H
bool robot_app_motion_init (void);
void robot_app_commit (const motion_output_t * p_output,
                              int32_t left_delta, int32_t right_delta,
                              uint32_t now_ms);
void robot_app_motion_health (motion_output_t * p_output,
    int32_t left_delta, int32_t right_delta, uint32_t now_ms);
void robot_app_reset (const robot_app_shared_t * p_copy);
int32_t robot_app_signed_count (int32_t count, int32_t sign);
int32_t robot_app_path_delta (uint16_t newer, uint16_t older);
#if !ROBOT_APP_MISSION
int32_t robot_app_count_delta (uint16_t newer, uint16_t older);
#endif
#if ROBOT_APP_MISSION
uint32_t robot_app_duty_pm (core_real_t duty);
#endif

void robot_app_motion_step (uint32_t now_ms);

#endif /* ROBOT_MOTION_H */

/* End of file. */

