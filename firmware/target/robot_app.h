/** @file robot_app.h
 * @brief Robot task bodies and deterministic application-cycle entry points.
 */
#ifndef ROBOT_APP_H
#define ROBOT_APP_H

#include <stdbool.h>
#include <stdint.h>

/** Stable diagnostic codes: first detected cause, not a physical diagnosis. */
typedef enum
{
    ROBOT_APP_FAULT_NONE = 0,
    ROBOT_APP_FAULT_LOCAL_STOP,
    ROBOT_APP_FAULT_REMOTE_STOP,
    ROBOT_APP_FAULT_OWNER_TIMEOUT,
    ROBOT_APP_FAULT_TILT,
    ROBOT_APP_FAULT_COLLISION,
    ROBOT_APP_FAULT_ENCODER,
    ROBOT_APP_FAULT_MOTION,
    ROBOT_APP_FAULT_OPTICAL,
    ROBOT_APP_FAULT_SUPERVISOR,
    ROBOT_APP_FAULT_IMU,
    ROBOT_APP_FAULT_SCAN,
    ROBOT_APP_FAULT_LEFT_STALL,
    ROBOT_APP_FAULT_RIGHT_STALL,
    ROBOT_APP_FAULT_MOTOR_OUTPUT,
    ROBOT_APP_FAULT_TILT_UNAVAILABLE
} robot_app_fault_t;

typedef struct
{
    robot_app_fault_t code;
    uint32_t first_fault_ms;
} robot_app_diagnostic_t;

/** Initialize all application state before starting any application task. */
bool robot_app_init (void);

/** Copy the first cause since boot/approved reset; NULL has no effect. */
void robot_app_diagnostic (robot_app_diagnostic_t * p_output);

/** Execute one task activation; each function has exactly one task owner. */
void robot_app_safety_step (uint32_t now_ms);
void robot_app_motion_step (uint32_t now_ms);
void robot_app_line_step (uint32_t now_ms);
void robot_app_vehicle_step (uint32_t now_ms);
void robot_app_imu_step (uint32_t now_ms);
void robot_app_scan_step (uint32_t now_ms);
void robot_app_comm_step (uint32_t now_ms);
void robot_app_telemetry_step (uint32_t now_ms);

/** Non-returning periodic tasks; kernel ABI wrappers live in app_main.c. */
void robot_app_safety_task (void);
void robot_app_motion_task (void);
void robot_app_line_task (void);
void robot_app_vehicle_task (void);
void robot_app_imu_task (void);
void robot_app_scan_task (void);
void robot_app_comm_task (void);
void robot_app_telemetry_task (void);

#endif /* ROBOT_APP_H */

/* End of file. */

