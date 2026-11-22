/** @file mission_board.h
 * @brief The mission: the robot app's owners as the mission's board calls
 * (rings, requests, the Motion snapshot); the calls the other parts make
 * into mission_board.c. Include mission_private.h first.
 */
#ifndef MISSION_BOARD_H
#define MISSION_BOARD_H
#if !defined(MISSION_SIM)

INT mission_no_console (const UB * p_format, ...);
#endif

bool mission_read_ir (uint16_t samples[3]);

board_accel_status_t mission_read_accel (int16_t milligravity[3]);

bool mission_accel_waiting (void);

uint64_t mission_accel_sample_us (uint64_t now_us);

bool mission_sonar_trigger (void);

void mission_sonar_service (void);

bool mission_sonar_result (uint32_t * p_pulse_us, bool * p_valid);

bool mission_servo_us (uint16_t pulse_us);

void mission_publish (const motion_request_t * p_request);

void mission_publish_halt (void);

void mission_motor_brake (void);

void mission_motor_duty (core_real_t left_duty, core_real_t right_duty);

void mission_read_encoders (int32_t * p_left, int32_t * p_right,
                                   uint32_t * p_errors);

void mission_encoders_discard (void);

bool mission_watchdog_start (uint32_t timeout_ms);

void mission_watchdog_feed (void);

void mission_origin (void);

void mission_motion_output (motion_output_t * p_output);

#endif /* MISSION_BOARD_H */

/* End of file. */

