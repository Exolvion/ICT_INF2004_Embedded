/** @file robot_comm.h
 * @brief The robot app: subsystem 1: MQTT commands (Comm) and the status,
 * perception and diagnostic frames (Telemetry); the calls the other parts
 * make into robot_comm.c. Include robot_app_private.h first.
 */
#ifndef ROBOT_COMM_H
#define ROBOT_COMM_H
void robot_app_count_drop (void);
void robot_app_perception (const robot_app_shared_t * p_copy,
                                  uint32_t now_ms);
void robot_app_diagnostic_frame (const robot_app_shared_t * p_copy,
                                       uint32_t now_ms);
int16_t robot_app_wire_signed (uint32_t field, uint32_t width);

void robot_app_comm_step (uint32_t now_ms);

void robot_app_telemetry_step (uint32_t now_ms);

#endif /* ROBOT_COMM_H */

/* End of file. */

