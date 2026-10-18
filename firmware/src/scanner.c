/** @file scanner.c
 * @brief Fixed-memory coarse/fine ultrasonic scan scheduling and profile.
 */
#include "scanner.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#if DBL_MANT_DIG < 53
#error "Scanner geometry requires at least 53 double mantissa bits."
#endif

#define SCANNER_MIN_ANGLE_DEG (-60)
#define SCANNER_MAX_ANGLE_DEG (60)
#define SCANNER_COARSE_STEP_DEG (10)
#define SCANNER_FINE_STEP_DEG (2)
#define SCANNER_SIDE_ANGLE_DEG (20)
#define SCANNER_MAX_DELAY_MS (60000U)
#define SCANNER_MIN_INTERVAL_MS (65U)

typedef struct
{
    double minimum_y_m;
    double maximum_y_m;
    double left_distance_m;
    double right_distance_m;
    uint32_t left_samples;
    uint32_t right_samples;
    bool b_left_valid;
    bool b_right_valid;
} scanner_profile_t;

static bool scanner_config_valid (const scanner_config_t * p_config);
static void scanner_result_reset (scanner_result_t * p_result);
static void scanner_capture (scanner_state_t * p_state,
                             const scanner_echo_t * p_echo);
static bool scanner_advance (scanner_state_t * p_state);
static bool scanner_refine (scanner_state_t * p_state);
static bool scanner_latest (const scanner_state_t * p_state, uint32_t index);
static void scanner_profile (scanner_state_t * p_state);
static void scanner_accumulate (scanner_state_t * p_state,
                                scanner_profile_t * p_profile,
                                const scanner_sample_t * p_sample);
static void scanner_choose_side (scanner_state_t * p_state,
                                 const scanner_profile_t * p_profile);

/** Default timing exceeds the HC-SR04 recommendation of 60 ms per cycle. */
void scanner_default_config (scanner_config_t * p_config)
{
    if (NULL != p_config)
    {
        p_config->range.sound_speed_m_per_s = 343.0;
        p_config->range.min_distance_m      = 0.02;
        p_config->range.max_distance_m      = 4.0;
        p_config->obstacle_distance_m       = 0.60;
        p_config->robot_half_width_m        = 0.08;
        p_config->uncertainty_margin_m      = 0.05;
        p_config->required_clearance_m      = 0.50;
        p_config->settle_ms                 = 150U;
        p_config->trigger_interval_ms       = SCANNER_MIN_INTERVAL_MS;
        p_config->echo_timeout_ms           = 30U;
        p_config->b_calibrated              = false;
    }
}

/** Configuration validation is independent of later calibration approval. */
bool scanner_init (scanner_state_t * p_state,
                   const scanner_config_t * p_config)
{
    scanner_config_t config_copy;

    if ((NULL == p_state) || (!scanner_config_valid(p_config)))
    {
        return false;
    }
    config_copy = *p_config;
    memset(p_state, 0, sizeof(*p_state));
    p_state->config        = config_copy;
    p_state->b_initialized = true;
    p_state->phase         = SCANNER_IDLE;
    scanner_result_reset(&p_state->result);
    return true;
}

/** Acquire only after calibration and stationary-pose confirmation. */
bool scanner_start (scanner_state_t * p_state, uint32_t now_ms,
                    bool b_stationary)
{
    if ((NULL == p_state) || (!p_state->b_initialized)
        || (!p_state->config.b_calibrated) || (!b_stationary))
    {
        return false;
    }
    if ((SCANNER_POSITION == p_state->phase)
        || (SCANNER_SETTLE == p_state->phase)
        || (SCANNER_WAIT_ECHO == p_state->phase))
    {
        return false;
    }
    p_state->sample_count     = 0U;
    p_state->angle_deg        = SCANNER_MIN_ANGLE_DEG;
    p_state->fine_end_deg     = SCANNER_MAX_ANGLE_DEG;
    p_state->phase_started_ms = now_ms;
    p_state->b_fine           = false;
    p_state->phase            = SCANNER_POSITION;
    scanner_result_reset(&p_state->result);
    return true;
}

/** Service one bounded transition; board pulse capture owns microseconds. */
bool scanner_step (scanner_state_t * p_state, uint32_t now_ms,
                   bool b_stationary, const scanner_echo_t * p_echo,
                   scanner_output_t * p_output)
{
    uint32_t elapsed_ms = 0U;

    if (NULL == p_output)
    {
        return false;
    }
    memset(p_output, 0, sizeof(*p_output));
    if ((NULL == p_state) || (NULL == p_echo)
        || (!p_state->b_initialized))
    {
        return false;
    }
    if ((SCANNER_IDLE == p_state->phase)
        || (SCANNER_COMPLETE == p_state->phase)
        || (SCANNER_ABORTED == p_state->phase))
    {
        return true;
    }
    if (!b_stationary)
    {
        scanner_result_reset(&p_state->result);
        p_state->result.b_aborted = true;
        p_state->phase            = SCANNER_ABORTED;
        p_output->b_set_servo     = true;
        p_output->b_done          = true;
        return true;
    }
    elapsed_ms = now_ms - p_state->phase_started_ms;
    switch (p_state->phase)
    {
        case SCANNER_IDLE:
        case SCANNER_COMPLETE:
        case SCANNER_ABORTED:
            /* Terminal phases returned above before stationary validation. */
        break;
        case SCANNER_POSITION:
            p_output->b_set_servo     = true;
            p_output->angle_deg       = p_state->angle_deg;
            p_state->phase_started_ms = now_ms;
            p_state->phase            = SCANNER_SETTLE;
        break;
        case SCANNER_SETTLE:
            if ((elapsed_ms >= p_state->config.settle_ms)
                && ((!p_state->b_triggered)
                    || ((now_ms - p_state->last_trigger_ms)
                        >= p_state->config.trigger_interval_ms)))
            {
                p_output->b_trigger      = true;
                p_state->last_trigger_ms = now_ms;
                p_state->b_triggered     = true;
                p_state->phase_started_ms = now_ms;
                p_state->phase            = SCANNER_WAIT_ECHO;
            }
        break;
        case SCANNER_WAIT_ECHO:
            if (p_echo->b_ready
                || (elapsed_ms >= p_state->config.echo_timeout_ms))
            {
                scanner_capture(p_state, p_echo);
                if (!scanner_advance(p_state))
                {
                    scanner_profile(p_state);
                    p_state->phase       = SCANNER_COMPLETE;
                    p_output->b_set_servo = true;
                    p_output->b_done      = true;
                }
            }
        break;
        default:
            scanner_result_reset(&p_state->result);
            p_state->phase            = SCANNER_ABORTED;
            p_state->result.b_aborted = true;
            p_output->b_set_servo     = true;
            p_output->b_done          = true;
            return false;
        break;
    }
    return true;
}

/** Bound times below half the clock wrap and reject nonphysical geometry. */
static bool scanner_config_valid (const scanner_config_t * p_config)
{
    if (NULL == p_config)
    {
        return false;
    }
    return (isfinite(p_config->range.sound_speed_m_per_s))
           && (p_config->range.sound_speed_m_per_s > 0.0)
           && (isfinite(p_config->range.min_distance_m))
           && (p_config->range.min_distance_m >= 0.02)
           && (isfinite(p_config->range.max_distance_m))
           && (p_config->range.max_distance_m <= 4.0)
           && (p_config->range.max_distance_m
               > p_config->range.min_distance_m)
           && (isfinite(p_config->obstacle_distance_m))
           && (p_config->obstacle_distance_m
               >= p_config->range.min_distance_m)
           && (p_config->obstacle_distance_m < p_config->range.max_distance_m)
           && (isfinite(p_config->robot_half_width_m))
           && (p_config->robot_half_width_m > 0.0)
           && (isfinite(p_config->uncertainty_margin_m))
           && (p_config->uncertainty_margin_m >= 0.0)
           && (isfinite(p_config->required_clearance_m))
           && (p_config->required_clearance_m > 0.0)
           && (p_config->settle_ms > 0U)
           && (p_config->settle_ms <= SCANNER_MAX_DELAY_MS)
           && (p_config->trigger_interval_ms >= SCANNER_MIN_INTERVAL_MS)
           && (p_config->trigger_interval_ms <= SCANNER_MAX_DELAY_MS)
           && (p_config->echo_timeout_ms > 0U)
           && (p_config->echo_timeout_ms < p_config->trigger_interval_ms);
}

/** Unknown distances use NaN; no previous profile survives a new request. */
static void scanner_result_reset (scanner_result_t * p_result)
{
    memset(p_result, 0, sizeof(*p_result));
    p_result->nearest_distance_m = NAN;
    p_result->apparent_width_m   = NAN;
    p_result->left_clearance_m   = NAN;
    p_result->right_clearance_m  = NAN;
}

/** Capture the current pulse or timeout; store no guessed distance. */
static void scanner_capture (scanner_state_t * p_state,
                             const scanner_echo_t * p_echo)
{
    scanner_sample_t * p_sample = NULL;
    core_status_t status = CORE_UNAVAILABLE;
    double pulse_s = 0.0;

    if (p_state->sample_count >= SCANNER_MAX_SAMPLES)
    {
        return;
    }
    p_sample = &p_state->samples[p_state->sample_count];
    p_sample->angle_deg = p_state->angle_deg;
    p_sample->b_fine    = p_state->b_fine;
    /* uint32_t is exactly representable by the required 53-bit double. */
    pulse_s = ((double) p_echo->pulse_us) / 1000000.0;
    status = core_ultrasonic_range(&p_state->config.range,
                                    p_echo->b_ready, pulse_s, &p_sample->range);
    if ((CORE_OK != status) && (CORE_OUT_OF_RANGE != status))
    {
        p_sample->range.state      = CORE_RANGE_UNKNOWN;
        p_sample->range.distance_m = NAN;
    }
    p_state->sample_count++;
}

/** Continue bounded sweeps; at most 13 coarse plus 61 fine samples fit. */
static bool scanner_advance (scanner_state_t * p_state)
{
    const int32_t step_deg = p_state->b_fine ? SCANNER_FINE_STEP_DEG
                                            : SCANNER_COARSE_STEP_DEG;
    const int32_t end_deg  = p_state->b_fine ? p_state->fine_end_deg
                                            : SCANNER_MAX_ANGLE_DEG;

    if ((p_state->angle_deg < end_deg)
        && (p_state->sample_count < SCANNER_MAX_SAMPLES))
    {
        p_state->angle_deg += step_deg;
        p_state->phase = SCANNER_POSITION;
        return true;
    }
    if ((!p_state->b_fine) && (scanner_refine(p_state)))
    {
        p_state->phase = SCANNER_POSITION;
        return true;
    }
    return false;
}

/** Refine all uncertain/near coarse beams and their immediate neighbours. */
static bool scanner_refine (scanner_state_t * p_state)
{
    int32_t first_deg = SCANNER_MAX_ANGLE_DEG;
    int32_t last_deg  = SCANNER_MIN_ANGLE_DEG;
    uint32_t index   = 0U;
    bool b_found     = false;

    for (index = 0U; index < p_state->sample_count; index++)
    {
        const scanner_sample_t * p_sample = &p_state->samples[index];

        if ((CORE_RANGE_VALID != p_sample->range.state)
            || (p_sample->range.distance_m
                <= p_state->config.obstacle_distance_m))
        {
            if (!b_found)
            {
                first_deg = p_sample->angle_deg;
            }
            last_deg = p_sample->angle_deg;
            b_found  = true;
        }
    }
    if (b_found)
    {
        first_deg -= SCANNER_COARSE_STEP_DEG;
        last_deg  += SCANNER_COARSE_STEP_DEG;
        p_state->angle_deg = (first_deg < SCANNER_MIN_ANGLE_DEG)
                             ? SCANNER_MIN_ANGLE_DEG : first_deg;
        p_state->fine_end_deg = (last_deg > SCANNER_MAX_ANGLE_DEG)
                                ? SCANNER_MAX_ANGLE_DEG : last_deg;
        p_state->b_fine = true;
    }
    return b_found;
}

/** A fine observation replaces the coarse beam at the same angle. */
static bool scanner_latest (const scanner_state_t * p_state, uint32_t index)
{
    uint32_t later = 0U;

    for (later = index + 1U; later < p_state->sample_count; later++)
    {
        if (p_state->samples[later].angle_deg
            == p_state->samples[index].angle_deg)
        {
            return false;
        }
    }
    return true;
}

/** Assemble visible endpoint span and explicitly qualified sector margins. */
static void scanner_profile (scanner_state_t * p_state)
{
    scanner_profile_t profile;
    uint32_t index = 0U;

    memset(&profile, 0, sizeof(profile));
    profile.minimum_y_m      = DBL_MAX;
    profile.maximum_y_m      = -DBL_MAX;
    profile.left_distance_m  = DBL_MAX;
    profile.right_distance_m = DBL_MAX;
    profile.b_left_valid     = true;
    profile.b_right_valid    = true;
    scanner_result_reset(&p_state->result);
    for (index = 0U; index < p_state->sample_count; index++)
    {
        if (scanner_latest(p_state, index))
        {
            scanner_accumulate(p_state, &profile, &p_state->samples[index]);
        }
    }
    p_state->result.b_profile_valid = (p_state->result.valid_samples > 0U)
        && (0U == p_state->result.unknown_samples);
    if (p_state->result.b_obstacle_seen)
    {
        p_state->result.apparent_width_m = profile.maximum_y_m
                                          - profile.minimum_y_m;
    }
    scanner_choose_side(p_state, &profile);
}

/** Count all unknowns, retaining no-clear semantics even for maximum range. */
static void scanner_accumulate (scanner_state_t * p_state,
                                scanner_profile_t * p_profile,
                                const scanner_sample_t * p_sample)
{
    scanner_result_t * p_result = &p_state->result;
    const bool b_valid = (CORE_RANGE_VALID == p_sample->range.state);
    const bool b_left  = (p_sample->angle_deg >= SCANNER_SIDE_ANGLE_DEG);
    const bool b_right = (p_sample->angle_deg <= -SCANNER_SIDE_ANGLE_DEG);
    double lateral_m = 0.0;
    double angle_rad = 0.0;

    if (b_left)
    {
        p_profile->left_samples++;
        p_profile->b_left_valid = p_profile->b_left_valid && b_valid;
    }
    if (b_right)
    {
        p_profile->right_samples++;
        p_profile->b_right_valid = p_profile->b_right_valid && b_valid;
    }
    if (!b_valid)
    {
        p_result->unknown_samples++;
        return;
    }
    p_result->valid_samples++;
    if ((!isfinite(p_result->nearest_distance_m))
        || (p_sample->range.distance_m < p_result->nearest_distance_m))
    {
        p_result->nearest_distance_m = p_sample->range.distance_m;
    }
    if (b_left)
    {
        p_profile->left_distance_m = fmin(p_profile->left_distance_m,
                                         p_sample->range.distance_m);
    }
    if (b_right)
    {
        p_profile->right_distance_m = fmin(p_profile->right_distance_m,
                                          p_sample->range.distance_m);
    }
    if (p_sample->range.distance_m <= p_state->config.obstacle_distance_m)
    {
        /* The bounded integral degree angle is represented exactly. */
        angle_rad = ((double) p_sample->angle_deg) * CORE_PI / 180.0;
        lateral_m = p_sample->range.distance_m * sin(angle_rad);
        p_profile->minimum_y_m = fmin(p_profile->minimum_y_m, lateral_m);
        p_profile->maximum_y_m = fmax(p_profile->maximum_y_m, lateral_m);
        p_result->b_obstacle_seen = true;
    }
}

/** Prefer a side only with complete observations and sufficient margin. */
static void scanner_choose_side (scanner_state_t * p_state,
                                 const scanner_profile_t * p_profile)
{
    scanner_result_t * p_result = &p_state->result;
    const double allowance_m = p_state->config.robot_half_width_m
                                + p_state->config.uncertainty_margin_m;
    bool b_left_clear  = false;
    bool b_right_clear = false;

    p_result->b_left_observed = p_profile->b_left_valid
                               && (p_profile->left_samples > 0U);
    p_result->b_right_observed = p_profile->b_right_valid
                                && (p_profile->right_samples > 0U);
    if (p_result->b_left_observed)
    {
        p_result->left_clearance_m = fmax(0.0,
                                     p_profile->left_distance_m - allowance_m);
        b_left_clear = p_result->left_clearance_m
                       >= p_state->config.required_clearance_m;
    }
    if (p_result->b_right_observed)
    {
        p_result->right_clearance_m = fmax(0.0,
                                    p_profile->right_distance_m - allowance_m);
        b_right_clear = p_result->right_clearance_m
                        >= p_state->config.required_clearance_m;
    }
    if (!p_result->b_profile_valid)
    {
        p_result->preferred_side = SCANNER_SIDE_NONE;
    }
    else if (b_left_clear && ((!b_right_clear)
             || (p_result->left_clearance_m >= p_result->right_clearance_m)))
    {
        p_result->preferred_side = SCANNER_SIDE_LEFT;
    }
    else if (b_right_clear)
    {
        p_result->preferred_side = SCANNER_SIDE_RIGHT;
    }
    else
    {
        p_result->preferred_side = SCANNER_SIDE_NONE;
    }
}

/** End of file. */

