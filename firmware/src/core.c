/** @file core.c
 * @brief Portable geometry and bounded control arithmetic.
 * @details State belongs to the caller; there is no allocation or I/O.
 * BARR-RETURN-01: early error returns preserve unchanged-output contracts.
 */
#include "core.h"

#include <math.h>
#include <stddef.h>

/** @brief Clamp a finite value to inclusive finite limits. */
static core_real_t clamp_value (core_real_t value, core_real_t lower,
                                core_real_t upper);
/** @brief Report whether a value is finite and strictly positive. */
static bool is_positive_finite (core_real_t value);
/** @brief Report whether a value is finite and nonnegative. */
static bool is_nonnegative_finite (core_real_t value);
/** @brief Validate gains, zero-containing limits and maximum sample time. */
static bool pid_config_is_valid (const core_pid_config_t * p_config);
/** @brief Evaluate sin(value)/value using its local series near zero. */
static core_real_t sinc_value (core_real_t value);

/** @copydoc core_odometry_step */
core_status_t core_odometry_step (
    const core_odometry_config_t * p_config,
    const core_pose_t * p_previous_pose,
    int32_t left_delta_counts,
    int32_t right_delta_counts,
    core_odometry_result_t * p_result)
{
    core_odometry_result_t result;
    core_real_t            half_turn;
    core_real_t            midpoint_yaw;
    core_real_t            chord_length;

    if ((NULL == p_config) || (NULL == p_previous_pose) || (NULL == p_result))
    {
        return CORE_INVALID_ARGUMENT;
    }

    if ((!is_positive_finite(p_config->left_radius_m)) ||
        (!is_positive_finite(p_config->right_radius_m)) ||
        (!is_positive_finite(p_config->track_width_m)) ||
        (0U == p_config->left_counts_per_revolution) ||
        (0U == p_config->right_counts_per_revolution) ||
        (!isfinite(p_previous_pose->x_m)) ||
        (!isfinite(p_previous_pose->y_m)) ||
        (!isfinite(p_previous_pose->yaw_rad)) ||
        (!is_nonnegative_finite(p_previous_pose->centre_path_m)))
    {
        return CORE_INVALID_ARGUMENT;
    }

    /* Both signed 32-bit deltas and unsigned 32-bit resolutions are exactly
     * representable: core_real_t has at least 53 significand bits.
     */
    result.left_distance_m =
        ((2.0 * CORE_PI * p_config->left_radius_m) /
         (core_real_t) p_config->left_counts_per_revolution) *
        (core_real_t) left_delta_counts;
    result.right_distance_m =
        ((2.0 * CORE_PI * p_config->right_radius_m) /
         (core_real_t) p_config->right_counts_per_revolution) *
        (core_real_t) right_delta_counts;
    result.signed_centre_distance_m =
        (result.left_distance_m / 2.0) + (result.right_distance_m / 2.0);
    result.delta_yaw_rad =
        (result.right_distance_m - result.left_distance_m) /
        p_config->track_width_m;

    if ((!isfinite(result.left_distance_m)) ||
        (!isfinite(result.right_distance_m)) ||
        (!isfinite(result.signed_centre_distance_m)) ||
        (!isfinite(result.delta_yaw_rad)))
    {
        return CORE_NUMERIC_ERROR;
    }

    half_turn = result.delta_yaw_rad / 2.0;
    midpoint_yaw =
        remainder(p_previous_pose->yaw_rad, 2.0 * CORE_PI) + half_turn;
    chord_length    = result.signed_centre_distance_m * sinc_value(half_turn);
    result.pose.x_m = p_previous_pose->x_m + (chord_length * cos(midpoint_yaw));
    result.pose.y_m = p_previous_pose->y_m + (chord_length * sin(midpoint_yaw));
    result.pose.yaw_rad = remainder(
        remainder(p_previous_pose->yaw_rad, 2.0 * CORE_PI) +
        result.delta_yaw_rad,
        2.0 * CORE_PI);
    result.pose.centre_path_m =
        p_previous_pose->centre_path_m + fabs(result.signed_centre_distance_m);

    if ((!isfinite(result.pose.x_m)) || (!isfinite(result.pose.y_m)) ||
        (!isfinite(result.pose.yaw_rad)) ||
        (!isfinite(result.pose.centre_path_m)))
    {
        return CORE_NUMERIC_ERROR;
    }

    *p_result = result;
    return CORE_OK;
}

/** @copydoc core_pid_reset */
core_status_t core_pid_reset (
    const core_pid_config_t * p_config,
    core_pid_state_t * p_state)
{
    if ((!pid_config_is_valid(p_config)) || (NULL == p_state))
    {
        return CORE_INVALID_ARGUMENT;
    }

    p_state->integral_output =
        clamp_value(0.0, p_config->output_min, p_config->output_max);
    p_state->previous_measurement       = 0.0;
    p_state->b_has_previous_measurement = false;
    return CORE_OK;
}

/** @copydoc core_pid_step */
core_status_t core_pid_step (
    const core_pid_config_t * p_config,
    core_pid_state_t * p_state,
    core_real_t setpoint,
    core_real_t measurement,
    core_real_t dt_s,
    core_real_t * p_output)
{
    core_real_t      error;
    core_real_t      proportional;
    core_real_t      derivative = 0.0;
    core_real_t      integral_increment;
    core_real_t      integral_candidate;
    core_real_t      unconstrained;
    core_pid_state_t next_state;

    if ((!pid_config_is_valid(p_config)) ||
        (NULL == p_state) ||
        (NULL == p_output))
    {
        return CORE_INVALID_ARGUMENT;
    }

    if ((!isfinite(setpoint)) || (!isfinite(measurement)) ||
        (!is_positive_finite(dt_s)) || (dt_s > p_config->max_dt_s) ||
        (!isfinite(p_state->integral_output)) ||
        (p_state->integral_output < p_config->output_min) ||
        (p_state->integral_output > p_config->output_max) ||
        (!isfinite(p_state->previous_measurement)))
    {
        return CORE_INVALID_ARGUMENT;
    }

    next_state   = *p_state;
    error        = setpoint - measurement;
    proportional = p_config->proportional_gain * error;
    if (p_state->b_has_previous_measurement)
    {
        derivative = -p_config->derivative_gain *
                     ((measurement - p_state->previous_measurement) / dt_s);
    }
    integral_increment = p_config->integral_gain * error * dt_s;
    integral_candidate = p_state->integral_output + integral_increment;

    if ((!isfinite(error)) ||
        (!isfinite(proportional)) ||
        (!isfinite(derivative)) ||
        (!isfinite(integral_increment)) || (!isfinite(integral_candidate)))
    {
        return CORE_NUMERIC_ERROR;
    }

    integral_candidate = clamp_value(
        integral_candidate, p_config->output_min, p_config->output_max);
    unconstrained = proportional + integral_candidate + derivative;
    if (!isfinite(unconstrained))
    {
        return CORE_NUMERIC_ERROR;
    }

    if (((unconstrained > p_config->output_max) && (error > 0.0)) ||
        ((unconstrained < p_config->output_min) && (error < 0.0)))
    {
        /* Reject only integration that would drive further into saturation. */
        integral_candidate = p_state->integral_output;
    }

    unconstrained = proportional + integral_candidate + derivative;
    if (!isfinite(unconstrained))
    {
        return CORE_NUMERIC_ERROR;
    }

    next_state.integral_output            = integral_candidate;
    next_state.previous_measurement       = measurement;
    next_state.b_has_previous_measurement = true;
    *p_output =
        clamp_value(unconstrained, p_config->output_min, p_config->output_max);
    *p_state = next_state;
    return CORE_OK;
}

/** @copydoc core_hump_height_increment */
core_status_t core_hump_height_increment (
    core_real_t signed_forward_distance_m,
    core_real_t pitch_rad,
    bool b_trusted_sample,
    core_real_t * p_delta_height_m)
{
    core_real_t delta_height;

    if (NULL == p_delta_height_m)
    {
        return CORE_INVALID_ARGUMENT;
    }
    if (!b_trusted_sample)
    {
        return CORE_UNAVAILABLE;
    }
    if ((!isfinite(signed_forward_distance_m)) || (!isfinite(pitch_rad)) ||
        (fabs(pitch_rad) > (CORE_PI / 2.0)))
    {
        return CORE_INVALID_ARGUMENT;
    }

    delta_height = signed_forward_distance_m * sin(pitch_rad);
    if (!isfinite(delta_height))
    {
        return CORE_NUMERIC_ERROR;
    }

    *p_delta_height_m = delta_height;
    return CORE_OK;
}

/** @copydoc core_ultrasonic_range */
core_status_t core_ultrasonic_range (
    const core_ultrasonic_config_t * p_config,
    bool b_echo_received,
    core_real_t echo_pulse_s,
    core_range_sample_t * p_sample)
{
    core_real_t distance;

    if (NULL == p_sample)
    {
        return CORE_INVALID_ARGUMENT;
    }

    p_sample->state      = CORE_RANGE_UNKNOWN;
    p_sample->distance_m = NAN;

    if ((NULL == p_config) ||
        (!is_positive_finite(p_config->sound_speed_m_per_s)) ||
        (!is_nonnegative_finite(p_config->min_distance_m)) ||
        (!isfinite(p_config->max_distance_m)) ||
        (p_config->max_distance_m <= p_config->min_distance_m))
    {
        return CORE_INVALID_ARGUMENT;
    }
    if (!b_echo_received)
    {
        return CORE_NO_ECHO;
    }
    if (!is_positive_finite(echo_pulse_s))
    {
        return CORE_INVALID_ARGUMENT;
    }

    distance = (echo_pulse_s / 2.0) * p_config->sound_speed_m_per_s;
    if (!isfinite(distance))
    {
        return CORE_NUMERIC_ERROR;
    }
    if ((distance < p_config->min_distance_m) ||
        (distance > p_config->max_distance_m))
    {
        p_sample->state = CORE_RANGE_OUT_OF_RANGE;
        return CORE_OUT_OF_RANGE;
    }

    p_sample->state      = CORE_RANGE_VALID;
    p_sample->distance_m = distance;
    return CORE_OK;
}

/** @copydoc core_range_to_point */
core_status_t core_range_to_point (
    const core_range_sample_t * p_sample,
    core_real_t beam_angle_rad,
    core_point_t * p_point)
{
    core_point_t point;

    if ((NULL == p_sample) ||
        (NULL == p_point) ||
        (!isfinite(beam_angle_rad)) ||
        (fabs(beam_angle_rad) > CORE_PI))
    {
        return CORE_INVALID_ARGUMENT;
    }
    if (p_sample->state != CORE_RANGE_VALID)
    {
        return CORE_UNAVAILABLE;
    }
    if (!is_nonnegative_finite(p_sample->distance_m))
    {
        return CORE_INVALID_ARGUMENT;
    }

    point.x_m = p_sample->distance_m * cos(beam_angle_rad);
    point.y_m = p_sample->distance_m * sin(beam_angle_rad);
    if ((!isfinite(point.x_m)) || (!isfinite(point.y_m)))
    {
        return CORE_NUMERIC_ERROR;
    }

    *p_point = point;
    return CORE_OK;
}

/** @copydoc core_stopping_clearance */
core_status_t core_stopping_clearance (
    core_real_t forward_speed_m_per_s,
    core_real_t total_latency_s,
    core_real_t braking_deceleration_m_per_s2,
    core_real_t sensor_to_front_m,
    core_real_t margin_m,
    core_real_t * p_clearance_m)
{
    core_real_t braking_time_s;
    core_real_t braking_distance_m;
    core_real_t distance;

    if ((NULL == p_clearance_m) ||
        (!is_nonnegative_finite(forward_speed_m_per_s)) ||
        (!is_nonnegative_finite(total_latency_s)) ||
        (!is_positive_finite(braking_deceleration_m_per_s2)) ||
        (!is_nonnegative_finite(sensor_to_front_m)) ||
        (!is_nonnegative_finite(margin_m)))
    {
        return CORE_INVALID_ARGUMENT;
    }

    braking_time_s     = forward_speed_m_per_s / braking_deceleration_m_per_s2;
    braking_distance_m = (forward_speed_m_per_s / 2.0) * braking_time_s;
    distance = (forward_speed_m_per_s * total_latency_s) + braking_distance_m +
               sensor_to_front_m + margin_m;

    if ((!isfinite(braking_time_s)) ||
        (!isfinite(braking_distance_m)) ||
        (!isfinite(distance)))
    {
        return CORE_NUMERIC_ERROR;
    }

    *p_clearance_m = distance;
    return CORE_OK;
}

static core_real_t clamp_value (core_real_t value, core_real_t lower,
                                core_real_t upper)
{
    core_real_t result = value;

    if (result < lower)
    {
        result = lower;
    }
    else if (result > upper)
    {
        result = upper;
    }
    else
    {
        /* Value is already in range. */
    }

    return result;
}

static bool is_positive_finite (core_real_t value)
{
    return ((isfinite(value)) && (value > 0.0));
}

static bool is_nonnegative_finite (core_real_t value)
{
    return ((isfinite(value)) && (value >= 0.0));
}

static bool pid_config_is_valid (const core_pid_config_t * p_config)
{
    return ((p_config != NULL) &&
            (is_nonnegative_finite(p_config->proportional_gain)) &&
            (is_nonnegative_finite(p_config->integral_gain)) &&
            (is_nonnegative_finite(p_config->derivative_gain)) &&
            (isfinite(p_config->output_min)) &&
            (isfinite(p_config->output_max)) &&
            (p_config->output_min < p_config->output_max) &&
            (p_config->output_min <= 0.0) &&
            (p_config->output_max >= 0.0) &&
            (is_positive_finite(p_config->max_dt_s)));
}

static core_real_t sinc_value (core_real_t value)
{
    core_real_t result;

    if (fabs(value) < 1.0e-6)
    {
        const core_real_t square = value * value;
        result = 1.0 - (square / 6.0) + ((square * square) / 120.0);
    }
    else
    {
        result = sin(value) / value;
    }

    return result;
}

/* End of file. */

