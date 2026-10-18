/** @file motion.h
 * @brief Single-owner, leased wheel control; SI units throughout.
 * Calls require serialization by the owning motion task. Calibration is a gate.
 */
#ifndef MOTION_H
#define MOTION_H

#include "core.h"
#include <stdbool.h>
#include <stdint.h>

#define MOTION_LEASE_MS (50U)
#define MOTION_SPEED_BUCKETS (11U)

typedef enum
{
    MOTION_IDLE = 0,
    MOTION_RUNNING,
    MOTION_COMPLETE,
    MOTION_INHIBITED
} motion_state_t;

typedef enum
{
    MOTION_VELOCITY = 0,
    MOTION_DISTANCE,
    MOTION_TURN,
    MOTION_HALT
} motion_kind_t;

typedef enum
{
    MOTION_OK = 0,
    MOTION_INVALID,
    MOTION_REJECTED,
    MOTION_EXPIRED,
    MOTION_TIMED_OUT,
    MOTION_SENSOR_FAULT,
    MOTION_STOPPED
} motion_status_t;

typedef struct
{
    uint32_t        command_id;
    motion_kind_t   kind;
    core_real_t     forward_mps;
    core_real_t     yaw_rps;
    core_real_t     target;
} motion_request_t;

typedef struct
{
    core_odometry_config_t  odometry;
    core_pid_config_t       wheel_pid;
    core_real_t             max_wheel_mps;
    core_real_t             max_yaw_rps;
    core_real_t             distance_tolerance_m;
    core_real_t             turn_tolerance_rad;
    core_real_t             stationary_mps;
    uint32_t                primitive_timeout_ms;
    bool                    b_calibration_confirmed;
} motion_config_t;

typedef struct
{
    motion_state_t  state;
    motion_status_t reason;
    uint32_t        command_id;
    core_pose_t     pose;
    core_real_t     left_duty;
    core_real_t     right_duty;
    core_real_t     left_speed_mps;
    core_real_t     right_speed_mps;
    core_real_t     signed_distance_m; /* Raw centre delta of latest step. */
    bool            b_stationary;
} motion_output_t;

typedef struct
{
    core_real_t left_distance_m;
    core_real_t right_distance_m;
    core_real_t elapsed_s;
} motion_speed_bucket_t;

typedef struct
{
    motion_speed_bucket_t buckets[MOTION_SPEED_BUCKETS];
    uint32_t              current;
} motion_speed_window_t;

typedef struct
{
    motion_config_t     config;
    motion_request_t    request;
    motion_output_t     output;
    core_pid_state_t    left_pid;
    core_pid_state_t    right_pid;
    motion_speed_window_t speed_window;
    core_real_t         progress;
    uint32_t            renewed_ms;
    uint32_t            started_ms;
    bool                b_initialized;
    bool                b_has_command;
    bool                b_stop_latched;
    bool                b_start_pending;
} motion_t;

/** Initialize zero drive output. Invalid config leaves an inhibited instance.
 */
motion_status_t motion_init (motion_t * p_motion,
                             const motion_config_t * p_config);

/** Submit/renew a request. IDs use serial order modulo 2^32 and cannot be zero.
 * A repeated running primitive ID renews without restarting. An identical
 * completed request succeeds as a no-op; its ID cannot restart the primitive.
 * A velocity request may replace its same-ID velocity values.
 * HALT is non-latching and always stops; it cannot clear a previous STOP.
 */
motion_status_t motion_submit (motion_t * p_motion,
                               const motion_request_t * p_request,
                               uint32_t now_ms);

/** Latch inhibition and zero commanded output immediately. */
void motion_stop (motion_t * p_motion, motion_status_t reason);

/** Explicitly acknowledge a stop only with independently confirmed conditions.
 * A new ID is still required after reset. Reset does not start the robot.
 */
motion_status_t motion_reset (motion_t * p_motion, bool b_stationary,
                              bool b_fault_clear);

/** Update odometry and wheel PID. Positive encoder deltas mean forward.
 * now_ms is a wrapping millisecond clock; call gaps must be below 2^31 ms.
 * dt_s comes from the actual sample interval, never a nominal period.
 * PID speed feedback averages 50--55 ms after warm-up, using bounded 5 ms
 * buckets. During warm-up it uses only elapsed observations. Counts spanning
 * bucket boundaries are apportioned uniformly over their actual interval.
 * Pose/progress, signed_distance_m and the stationary indication use raw
 * deltas. signed_distance_m is zero when the latest feedback is invalid;
 * stationary still requires independent confirmation/dwell before reset.
 * Submit a new request before step/applying its first duty. That first sample
 * updates pose but excludes pre-command counts from primitive progress.
 * Hardware must apply the returned zero duties even when status is non-OK.
 */
motion_status_t motion_step (motion_t * p_motion, uint32_t now_ms,
                             core_real_t dt_s, int32_t left_delta,
                             int32_t right_delta, motion_output_t * p_output);

#endif /* MOTION_H */

/** End of file. */

