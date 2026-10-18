/** @file core.h
 * @brief Hardware-independent geometry and bounded control arithmetic.
 * Units are metres, seconds and radians unless explicitly stated otherwise.
 * No function allocates memory or accesses hardware. Caller owns all state.
 * Unless stated otherwise, output/state is unchanged on any non-OK return.
 * Callers MUST inspect each return status before using a result.
 */

#ifndef CORE_H
#define CORE_H

#include <float.h>
#include <stdbool.h>
#include <stdint.h>

/* BARR-REAL-01: project prefix replaces the illustrative float64_t name.
 * ISO C99 does not define float64_t. Binary64-level precision is required.
 * The alias is a numerical contract, not a claim about sizeof(double).
 */
#if (DBL_MANT_DIG < 53) || (DBL_MAX_EXP < 1024)
#error "core_real_t requires binary64-level precision and range"
#endif

typedef double core_real_t;

#define CORE_PI (3.14159265358979323846264338327950288)

typedef enum
{
    CORE_OK = 0,
    CORE_INVALID_ARGUMENT,
    CORE_NUMERIC_ERROR,
    CORE_UNAVAILABLE,
    CORE_NO_ECHO,
    CORE_OUT_OF_RANGE
} core_status_t;

typedef struct
{
    core_real_t left_radius_m;
    core_real_t right_radius_m;
    core_real_t track_width_m;
    uint32_t    left_counts_per_revolution;
    uint32_t    right_counts_per_revolution;
} core_odometry_config_t;

typedef struct
{
    core_real_t x_m;
    core_real_t y_m;
    core_real_t yaw_rad;
    /* Accumulated absolute centre-path distance; an in-place turn adds zero. */
    core_real_t centre_path_m;
} core_pose_t;

typedef struct
{
    core_pose_t pose;
    core_real_t left_distance_m;
    core_real_t right_distance_m;
    core_real_t signed_centre_distance_m;
    core_real_t delta_yaw_rad;
} core_odometry_result_t;

/** @brief Integrate calibrated wheel distances into planar odometry.
 * Both signed encoder deltas are positive when the corresponding wheel moves
 * the robot forwards. Counts/revolution refers to the decoder's effective edge
 * count at the WHEEL, including any gearing, not the motor's label value.
 * Exact constant-curvature integration assumes no slip and planar motion.
 * The output yaw is normalized to [-pi, pi]. This does not compensate humps.
 */
core_status_t core_odometry_step (
    const core_odometry_config_t * p_config,
    const core_pose_t * p_previous_pose,
    int32_t left_delta_counts,
    int32_t right_delta_counts,
    core_odometry_result_t * p_result);

typedef struct
{
    core_real_t proportional_gain;
    core_real_t integral_gain;
    core_real_t derivative_gain;
    core_real_t output_min;
    core_real_t output_max;
    core_real_t max_dt_s;
} core_pid_config_t;

typedef struct
{
    core_real_t integral_output;
    core_real_t previous_measurement;
    bool        b_has_previous_measurement;
} core_pid_state_t;

/** @brief Reset a bounded PID controller.
 * Reset after a mode/configuration change and before the first step.
 * All three gains must be nonnegative; output_min < output_max; limits must
 * include zero; max_dt_s > 0. This is a zero-bias controller.
 */
core_status_t core_pid_reset (
    const core_pid_config_t * p_config,
    core_pid_state_t * p_state);

/** @brief Advance a bounded PID controller by one sample.
 * Derivative is taken on measurement (zero for the first sample).
 * Conditional integration prevents further windup into saturation; integral
 * contribution is also bounded by output limits. dt must be in (0, max_dt_s].
 * This discrete controller still requires hardware tuning and a stable period.
 * Reset when gains/limits change; do not share state across independent loops.
 * p_output must refer to storage separate from p_state.
 */
core_status_t core_pid_step (
    const core_pid_config_t * p_config,
    core_pid_state_t * p_state,
    core_real_t setpoint,
    core_real_t measurement,
    core_real_t dt_s,
    core_real_t * p_output);

/** @brief Integrate one trusted chassis-path height increment.
 * Geometry only: dz = signed forward path distance * sin(pitch).
 * Positive pitch means the front of the robot is uphill. Distance is along the
 * slope, not its horizontal projection. Pitch must be within [-pi/2, pi/2].
 * b_trusted_sample must be false for stale or uncalibrated IMU samples,
 * dynamically unreliable samples, slipping, and manoeuvres outside the
 * hump state machine's scope.
 * An unavailable sample is NOT a zero-height observation.
 */
core_status_t core_hump_height_increment (
    core_real_t signed_forward_distance_m,
    core_real_t pitch_rad,
    bool b_trusted_sample,
    core_real_t * p_delta_height_m);

typedef enum
{
    CORE_RANGE_UNKNOWN = 0,
    CORE_RANGE_VALID,
    CORE_RANGE_OUT_OF_RANGE
} core_range_state_t;

typedef struct
{
    core_range_state_t state;
    /* Valid only when state == CORE_RANGE_VALID. Otherwise NAN. */
    core_real_t distance_m;
} core_range_sample_t;

typedef struct
{
    core_real_t sound_speed_m_per_s;
    core_real_t min_distance_m;
    core_real_t max_distance_m;
} core_ultrasonic_config_t;

/** @brief Convert a completed ultrasonic pulse into a qualified range.
 * A completed ECHO pulse is round-trip time in seconds. Timeout/no echo is
 * UNKNOWN, never clear space. Output is deliberately invalidated on every
 * failure (if non-NULL), so a previous valid reading cannot survive a timeout.
 * Driver must separately enforce trigger timing, timeout and pulse integrity.
 */
core_status_t core_ultrasonic_range (
    const core_ultrasonic_config_t * p_config,
    bool b_echo_received,
    core_real_t echo_pulse_s,
    core_range_sample_t * p_sample);

typedef struct
{
    core_real_t x_m;
    core_real_t y_m;
} core_point_t;

/** @brief Project one qualified range into the sensor frame.
 * Converts a VALID range into sensor-frame coordinates. Angle 0 is forward,
 * positive angles point left; require [-pi, pi]. The hardware adapter must
 * calibrate servo zero/direction and apply the sensor-to-robot transform.
 * A point is not a certified obstacle boundary or a free-space certificate.
 */
core_status_t core_range_to_point (
    const core_range_sample_t * p_sample,
    core_real_t beam_angle_rad,
    core_point_t * p_point);

/** @brief Compute scalar forward stopping clearance.
 * Conservative scalar forward stopping clearance from the sensor origin:
 * v*latency + v^2/(2*deceleration) + sensor_to_front + margin.
 * speed is a nonnegative forward approach magnitude, latency includes the
 * age of the reading plus detection/scheduling/actuation delay, deceleration
 * is a positive experimentally justified lower bound, offsets are >= 0.
 * Does not prove swept-volume clearance for turns or reverse motion.
 */
core_status_t core_stopping_clearance (
    core_real_t forward_speed_m_per_s,
    core_real_t total_latency_s,
    core_real_t braking_deceleration_m_per_s2,
    core_real_t sensor_to_front_m,
    core_real_t margin_m,
    core_real_t * p_clearance_m);

#endif /* CORE_H */

/* End of file. */

