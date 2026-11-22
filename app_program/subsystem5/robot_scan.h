/** @file robot_scan.h
 * @brief The robot app: subsystem 5: the Scan task (servo and sonar); the
 * calls the other parts make into robot_scan.c. Include robot_app_private.h
 * first.
 */
#ifndef ROBOT_SCAN_H
#define ROBOT_SCAN_H
#if !(ROBOT_APP_MISSION)
bool robot_app_range_valid (const robot_app_shared_t * p_copy);
void robot_app_scan_cycle (const robot_app_shared_t * p_copy,
                                  uint32_t now_ms, scanner_echo_t echo);
#endif
void robot_app_front_result (bool b_valid, uint32_t pulse_us);
bool robot_app_servo (int32_t angle_deg);
#if ROBOT_APP_MISSION

void robot_app_scan_step (uint32_t now_ms);
#endif
#if !(ROBOT_APP_MISSION)

void robot_app_scan_step (uint32_t now_ms);
#endif

#endif /* ROBOT_SCAN_H */

/* End of file. */

