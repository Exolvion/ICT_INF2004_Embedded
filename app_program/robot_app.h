/** @file robot_app.h
 * @brief Robot task bodies and deterministic application-cycle entry points.
 */
#ifndef ROBOT_APP_H
#define ROBOT_APP_H

#include <stdbool.h>
#include <stdint.h>

/** Stable diagnostic codes: first detected cause, not a physical diagnosis.
 * Sixteen values in four bits; the numbering is part of the wire contract.
 */
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

/** Bit positions in the 16-bit active-fault mask. Stopping bits stay set
 * until an approved reset (soft) or reboot (hard). NO_ECHO and WIFI_LOSS
 * are non-stopping status bits that their owners set and clear. OPTICAL
 * is split into a bad optical sample and a second barcode decoded before
 * the vehicle consumed the first; both report first cause OPTICAL.
 * LOCAL and REMOTE STOP are not faults: they use the stop-source bits.
 */
typedef enum
{
    ROBOT_APP_BIT_OWNER_TIMEOUT = 0,
    ROBOT_APP_BIT_ENCODER,
    ROBOT_APP_BIT_MOTION,
    ROBOT_APP_BIT_LEFT_STALL,
    ROBOT_APP_BIT_RIGHT_STALL,
    ROBOT_APP_BIT_MOTOR_OUTPUT,
    ROBOT_APP_BIT_OPTICAL_SAMPLE,
    ROBOT_APP_BIT_BARCODE_OVERRUN,
    ROBOT_APP_BIT_SUPERVISOR,
    ROBOT_APP_BIT_IMU,
    ROBOT_APP_BIT_TILT,
    ROBOT_APP_BIT_TILT_UNAVAILABLE,
    ROBOT_APP_BIT_COLLISION,
    ROBOT_APP_BIT_SCAN,
    ROBOT_APP_BIT_NO_ECHO,
    ROBOT_APP_BIT_WIFI_LOSS
} robot_app_bit_t;

/** Mask of the non-stopping status bits; an approved reset keeps them. */
#define ROBOT_APP_STATUS_BITS \
    ((UINT16_C(1) << ROBOT_APP_BIT_NO_ECHO) | \
     (UINT16_C(1) << ROBOT_APP_BIT_WIFI_LOSS))

/** Stop-source bits, separate from the fault mask. */
#define ROBOT_APP_STOP_LOCAL  (1U)
#define ROBOT_APP_STOP_REMOTE (2U)

/** First cause and its time, plus the live mask and stop sources. */
typedef struct
{
    uint32_t          first_fault_ms;
    uint16_t          mask;
    robot_app_fault_t code : 4;
    uint32_t          stop_source : 2;
} robot_app_diagnostic_t;

/** The eight application tasks, in priority order (app_main.c). */
#define ROBOT_APP_TASKS (8U)

/** SYS-12 evidence for one task since the last robot_app_timing_restart():
 * its longest step and how many periods it skipped (a step that ran past
 * its next release), and its own CPU time: its steps' run time with the
 * steps of other tasks that ran inside them (higher priority, preempting)
 * taken out, and how many steps it ran (23 Nov: where the CPU goes).
 */
typedef struct
{
    uint32_t worst_us;
    uint32_t skipped;
    uint32_t own_us;    /* saturates at UINT32_MAX (71 min) */
    uint32_t steps;
} robot_app_task_time_t;

/** Initialize all application state before starting any application task. */
bool robot_app_init (void);

/** Copy each task's timing (Safety, Motion, Line, Vehicle, IMU, Scan, Comm,
 * Telemetry); NULL has no effect.
 */
void robot_app_timing (robot_app_task_time_t p_output[ROBOT_APP_TASKS]);

/** Start a new timing window: each task clears its own record at its next
 * step.
 */
void robot_app_timing_restart (void);

/** Milliseconds since the last robot_app_timing_restart(); for the task
 * that calls it (the Vehicle task).
 */
uint32_t robot_app_timing_window_ms (void);

/** The IMU task's evidence since the last robot_app_timing_restart(), for
 * the mission's end report. Counts saturate at 255.
 */
typedef struct
{
    uint32_t failed     : 8;   /* reads that failed on the I2C bus */
    uint32_t saturated  : 8;   /* samples with an axis at the 12-bit limit */
    uint32_t late       : 8;   /* fresh samples past terrain's dt bound */
    uint32_t refused    : 8;   /* other samples terrain refused */
    uint32_t longest_ms : 16;  /* longest gap between two fresh samples */
    uint32_t slowest_us : 16;  /* longest accelerometer read, wall time */
    uint32_t failed_us  : 16;  /* how long the last failed read took */
} robot_app_imu_evidence_t;

/** Copy the IMU evidence (robot_app_imu_evidence_t); NULL has no effect. */
void robot_app_imu_evidence (robot_app_imu_evidence_t * p_output);

/** Owners whose health Safety checks, in the order Motion, Line, Vehicle,
 * IMU, Scan, Comm (gates 20, 20, 100 in the mission else 40, 100, 100 and,
 * for the watchdog feed only, 100 ms).
 */
#define ROBOT_APP_OWNERS (6U)

/** Safety's evidence since the last robot_app_timing_restart(): each
 * owner's longest health gap, ms, held at 65535. The one over its gate is
 * the OWNER_TIMEOUT.
 */
typedef struct
{
    uint16_t longest_ms[ROBOT_APP_OWNERS];
} robot_app_owner_evidence_t;

/** Copy the owner evidence; NULL has no effect. */
void robot_app_owner_evidence (robot_app_owner_evidence_t * p_output);

/** Copy the first cause since boot/approved reset, the active mask and the
 * stop sources; NULL has no effect.
 */
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

