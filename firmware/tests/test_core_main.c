/** @file test_core_main.c
 * @brief Hosted regression checks for numerical contracts.
 * @details Five analytical fixture groups; checks survive -DNDEBUG.
 */
#include "test_core_main.h"

#include "core.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

static unsigned int g_checks   = 0U;
static unsigned int g_failures = 0U;

/* Tests deliberately do not use assert(): -DNDEBUG must not disable checks. */
/* BARR-TEST-01: source location/stringification requires a macro; the
 * condition is evaluated once and the full expression is parenthesized.
 */
#define CHECK(b_condition) \
    (check_result((b_condition), #b_condition, __LINE__))

#define TEST_PID_SATURATION_STEPS (100U)
#define TEST_HUMP_TOTAL_STEPS (20U)
#define TEST_HUMP_ASCENT_STEPS (10U)

/** @brief Check result. */
static void check_result (bool b_condition, const char * p_expression,
                          int line);
/** @brief Near value. */
static bool near_value (core_real_t actual, core_real_t expected);
/** @brief Test odometry. */
static void test_odometry (void);
/** @brief Test pid. */
static void test_pid (void);
/** @brief Reject invalid PID input while preserving state/output. */
static void test_pid_invalid (void);
/** @brief Test hump geometry. */
static void test_hump_geometry (void);
/** @brief Test ultrasonic and points. */
static void test_ultrasonic_and_points (void);
/** @brief Test stopping clearance. */
static void test_stopping_clearance (void);

/** @brief Execute numerical checks; return failure on any failed check. */
int main (void)
{
    test_odometry();
    test_pid();
    test_hump_geometry();
    test_ultrasonic_and_points();
    test_stopping_clearance();

    /* Output failure is handled independently from the numeric checks. */
    if (0 > printf("Checks: %u; failures: %u\n", g_checks, g_failures))
    {
        return EXIT_FAILURE;
    }
    if (g_failures != 0U)
    {
        return EXIT_FAILURE;
    }

    if (EOF == puts("PASS: numerical core; no hardware/RTOS validation."))
    {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

static void check_result (bool b_condition, const char * p_expression, int line)
{
    ++g_checks;
    if (!b_condition)
    {
        ++g_failures;
        /* Cast discards int status: the numeric failure is already recorded. */
        (void) fprintf(stderr, "FAIL line %d: %s\n", line, p_expression);
    }
}

static bool near_value (core_real_t actual, core_real_t expected)
{
    return ((isfinite(actual)) &&
            (fabs(actual - expected) <= (1.0e-12 * fmax(1.0, fabs(expected)))));
}

static void test_odometry (void)
{
    core_odometry_config_t config =
    {
        1.0 / (2.0 * CORE_PI), 1.0 / (2.0 * CORE_PI), 0.2, 1000U, 1000U
    };
    core_pose_t start =
    {
        0.0, 0.0, 0.0, 0.0
    };
    core_odometry_result_t result =
    {
        {
            0.0, 0.0, 0.0, 0.0
        },
        0.0, 0.0, 0.0, 0.0
    };

    CHECK(CORE_OK == core_odometry_step(&config, &start, 1000, 1000, &result));
    CHECK(near_value(result.pose.x_m, 1.0));
    CHECK(near_value(result.pose.y_m, 0.0));
    CHECK(near_value(result.pose.yaw_rad, 0.0));
    CHECK(near_value(result.pose.centre_path_m, 1.0));
    CHECK(near_value(result.left_distance_m, 1.0));

    CHECK(CORE_OK ==
          core_odometry_step(&config, &start, -1000, -1000, &result));
    CHECK(near_value(result.pose.x_m, -1.0));
    CHECK(near_value(result.signed_centre_distance_m, -1.0));
    CHECK(near_value(result.pose.centre_path_m, 1.0));

    start.yaw_rad = CORE_PI / 2.0;
    CHECK(CORE_OK == core_odometry_step(&config, &start, 1000, 1000, &result));
    CHECK(near_value(result.pose.x_m, 0.0));
    CHECK(near_value(result.pose.y_m, 1.0));
    start.yaw_rad = 0.0;

    /* Quarter-circle: left wheel fixed, right wheel travels exactly 1 m. */
    config.track_width_m = 2.0 / CORE_PI;
    CHECK(CORE_OK == core_odometry_step(&config, &start, 0, 1000, &result));
    CHECK(near_value(result.pose.x_m, 1.0 / CORE_PI));
    CHECK(near_value(result.pose.y_m, 1.0 / CORE_PI));
    CHECK(near_value(result.pose.yaw_rad, CORE_PI / 2.0));
    CHECK(near_value(result.pose.centre_path_m, 0.5));

    CHECK(CORE_OK == core_odometry_step(&config, &start, -500, 500, &result));
    CHECK(near_value(result.pose.x_m, 0.0));
    CHECK(near_value(result.pose.y_m, 0.0));
    CHECK(near_value(result.pose.yaw_rad, CORE_PI / 2.0));
    CHECK(near_value(result.pose.centre_path_m, 0.0));
    CHECK(CORE_OK == core_odometry_step(&config, &start, 500, -500, &result));
    CHECK(near_value(result.pose.yaw_rad, -CORE_PI / 2.0));

    start.yaw_rad = 3.0 * CORE_PI / 4.0;
    CHECK(CORE_OK == core_odometry_step(&config, &start, -500, 500, &result));
    CHECK(near_value(result.pose.yaw_rad, -3.0 * CORE_PI / 4.0));
    start.yaw_rad = 0.0;

    /* Unequal wheel radius and decoder resolution cancel when calibrated. */
    config.right_radius_m *= 2.0;
    config.right_counts_per_revolution = 2000U;
    CHECK(CORE_OK == core_odometry_step(&config, &start, 1000, 1000, &result));
    CHECK(near_value(result.pose.x_m, 1.0));
    CHECK(near_value(result.pose.yaw_rad, 0.0));

    config.track_width_m = 1.0e9;
    CHECK(CORE_OK == core_odometry_step(&config, &start, 0, 1, &result));
    CHECK(near_value(result.pose.x_m, 0.0005));
    CHECK(CORE_OK ==
          core_odometry_step(&config, &start, INT32_MIN, INT32_MAX, &result));
    CHECK(isfinite(result.pose.x_m));

    result.pose.x_m                   = 123.0;
    config.left_counts_per_revolution = 0U;
    CHECK(CORE_INVALID_ARGUMENT ==
          core_odometry_step(&config, &start, 1, 1, &result));
    CHECK(near_value(result.pose.x_m, 123.0));
    config.left_counts_per_revolution = 1000U;
    config.track_width_m              = NAN;
    CHECK(CORE_INVALID_ARGUMENT ==
          core_odometry_step(&config, &start, 1, 1, &result));
    config.track_width_m = -1.0;
    CHECK(CORE_INVALID_ARGUMENT ==
          core_odometry_step(&config, &start, 1, 1, &result));
    config.track_width_m = 0.2;
    start.x_m            = INFINITY;
    CHECK(CORE_INVALID_ARGUMENT ==
          core_odometry_step(&config, &start, 1, 1, &result));
    start.x_m            = 0.0;
    config.left_radius_m = DBL_MAX;
    CHECK(CORE_NUMERIC_ERROR ==
          core_odometry_step(&config, &start, 1, 1, &result));
    CHECK(near_value(result.pose.x_m, 123.0));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_odometry_step(NULL, &start, 0, 0, &result));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_odometry_step(&config, NULL, 0, 0, &result));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_odometry_step(&config, &start, 0, 0, NULL));
}

static void test_pid (void)
{
    core_pid_config_t config =
    {
        2.0, 1.0, 0.0, -1.0, 1.0, 0.1
    };
    core_pid_state_t state =
    {
        0.0, 0.0, false
    };
    core_real_t  output = 123.0;
    unsigned int index;

    CHECK(CORE_OK == core_pid_reset(&config, &state));
    for (index = 0U; index < TEST_PID_SATURATION_STEPS; ++index)
    {
        CHECK(CORE_OK ==
              core_pid_step(&config, &state, 1.0, 0.0, 0.1, &output));
        CHECK(near_value(output, 1.0));
    }
    CHECK(near_value(state.integral_output, 0.0));
    CHECK(CORE_OK == core_pid_step(&config, &state, 0.0, 0.0, 0.1, &output));
    CHECK(near_value(output, 0.0));
    CHECK(CORE_OK == core_pid_step(&config, &state, -1.0, 0.0, 0.1, &output));
    CHECK(near_value(output, -1.0));
    CHECK(near_value(state.integral_output, 0.0));

    config.proportional_gain = 0.0;
    config.integral_gain     = 1.0;
    CHECK(CORE_OK == core_pid_reset(&config, &state));
    CHECK(CORE_OK == core_pid_step(&config, &state, 0.5, 0.0, 0.1, &output));
    CHECK(near_value(output, 0.05));
    CHECK(CORE_OK == core_pid_step(&config, &state, 0.5, 0.0, 0.1, &output));
    CHECK(near_value(output, 0.1));

    /* An existing integral unwinds when the error reverses. */
    state.integral_output = 0.9;
    CHECK(CORE_OK == core_pid_step(&config, &state, -1.0, 0.0, 0.1, &output));
    CHECK(near_value(output, 0.8));

    config.integral_gain   = 0.0;
    config.derivative_gain = 0.5;
    CHECK(CORE_OK == core_pid_reset(&config, &state));
    CHECK(CORE_OK == core_pid_step(&config, &state, 0.0, 0.0, 0.1, &output));
    CHECK(near_value(output, 0.0));
    CHECK(CORE_OK == core_pid_step(&config, &state, 100.0, 0.0, 0.1, &output));
    CHECK(near_value(output, 0.0)); /* No setpoint derivative kick. */
    CHECK(CORE_OK == core_pid_step(&config, &state, 100.0, 0.1, 0.1, &output));
    CHECK(near_value(output, -0.5));

    test_pid_invalid();
}

static void test_pid_invalid (void)
{
    /* State left by the measurement-derivative fixture above. */
    core_pid_config_t config =
    {
        0.0, 0.0, 0.5, -1.0, 1.0, 0.1
    };
    core_pid_state_t state =
    {
        0.0, 0.1, true
    };
    core_real_t output = 123.0;
    CHECK(CORE_INVALID_ARGUMENT ==
          core_pid_step(&config, &state, 1.0, 0.2, 0.0, &output));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_pid_step(&config, &state, 1.0, 0.2, -0.1, &output));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_pid_step(&config, &state, 1.0, 0.2, 0.1001, &output));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_pid_step(&config, &state, 1.0, 0.2, NAN, &output));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_pid_step(&config, &state, NAN, 0.2, 0.1, &output));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_pid_step(&config, &state, 1.0, INFINITY, 0.1, &output));
    CHECK(near_value(output, 123.0));
    CHECK(near_value(state.previous_measurement, 0.1));

    config.proportional_gain = DBL_MAX;
    CHECK(CORE_OK == core_pid_reset(&config, &state));
    CHECK(CORE_NUMERIC_ERROR ==
          core_pid_step(&config, &state, DBL_MAX, 0.0, 0.1, &output));
    CHECK(!state.b_has_previous_measurement);
    CHECK(near_value(output, 123.0));
    config.proportional_gain = -1.0;
    CHECK(CORE_INVALID_ARGUMENT == core_pid_reset(&config, &state));
    config.proportional_gain = 1.0;
    config.output_max        = config.output_min;
    CHECK(CORE_INVALID_ARGUMENT == core_pid_reset(&config, &state));
    config.output_min = 10.0;
    config.output_max = 100.0;
    CHECK(CORE_INVALID_ARGUMENT == core_pid_reset(&config, &state));
    config.output_min = -100.0;
    config.output_max = -10.0;
    CHECK(CORE_INVALID_ARGUMENT == core_pid_reset(&config, &state));
    config.output_min      = 0.0;
    config.output_max      = 1.0;
    config.integral_gain   = 0.0;
    config.derivative_gain = 0.0;
    CHECK(CORE_OK == core_pid_reset(&config, &state));
    CHECK(CORE_OK == core_pid_step(&config, &state, 0.5, 0.0, 0.1, &output));
    CHECK(near_value(output, 0.5));
    config.output_min = -1.0;
    config.output_max = 1.0;
    CHECK(CORE_INVALID_ARGUMENT == core_pid_reset(NULL, &state));
    CHECK(CORE_INVALID_ARGUMENT == core_pid_reset(&config, NULL));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_pid_step(&config, NULL, 0.0, 0.0, 0.1, &output));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_pid_step(&config, &state, 0.0, 0.0, 0.1, NULL));
    state.integral_output = NAN;
    CHECK(CORE_INVALID_ARGUMENT ==
          core_pid_step(&config, &state, 0.0, 0.0, 0.1, &output));
}

static void test_hump_geometry (void)
{
    const core_real_t rise_m = 0.1;
    const core_real_t horizontal_half_length_m = 0.5;
    const core_real_t pitch_rad = atan2(rise_m, horizontal_half_length_m);
    const core_real_t step_distance_m =
        hypot(rise_m, horizontal_half_length_m) / 10.0;
    core_real_t  height_m = 0.0;
    core_real_t  peak_m = 0.0;
    core_real_t  increment_m = 0.0;
    unsigned int index;

    /* An ideal triangular ramp has analytically known rise 0.1 m. */
    for (index = 0U; index < TEST_HUMP_TOTAL_STEPS; ++index)
    {
        const core_real_t signed_pitch =
            (index < TEST_HUMP_ASCENT_STEPS) ? pitch_rad : -pitch_rad;
        CHECK(CORE_OK ==
              core_hump_height_increment(step_distance_m, signed_pitch, true,
                                         &increment_m));
        height_m += increment_m;
        peak_m = fmax(peak_m, height_m);
    }
    CHECK(near_value(peak_m, rise_m));
    CHECK(near_value(height_m, 0.0));
    CHECK(CORE_OK ==
          core_hump_height_increment(-step_distance_m, pitch_rad, true,
                                     &increment_m));
    CHECK(near_value(increment_m, -0.01));
    CHECK(CORE_OK == core_hump_height_increment(1.0, 0.0, true, &increment_m));
    CHECK(near_value(increment_m, 0.0));

    increment_m = 123.0;
    CHECK(CORE_UNAVAILABLE ==
          core_hump_height_increment(1.0, 0.0, false, &increment_m));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_hump_height_increment(NAN, 0.0, true, &increment_m));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_hump_height_increment(1.0, INFINITY, true, &increment_m));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_hump_height_increment(1.0, CORE_PI, true, &increment_m));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_hump_height_increment(1.0, 0.0, true, NULL));
    CHECK(near_value(increment_m, 123.0));
}

static void test_ultrasonic_and_points (void)
{
    core_ultrasonic_config_t config =
    {
        343.0, 0.02, 4.0
    };
    core_range_sample_t sample =
    {
        CORE_RANGE_UNKNOWN, NAN
    };
    core_point_t point =
    {
        123.0, 456.0
    };

    CHECK(CORE_OK == core_ultrasonic_range(&config, true, 0.002, &sample));
    CHECK(CORE_RANGE_VALID == sample.state);
    CHECK(near_value(sample.distance_m, 0.343));
    CHECK(CORE_OK == core_range_to_point(&sample, 0.0, &point));
    CHECK(near_value(point.x_m, 0.343));
    CHECK(near_value(point.y_m, 0.0));
    CHECK(CORE_OK == core_range_to_point(&sample, CORE_PI / 2.0, &point));
    CHECK(near_value(point.x_m, 0.0));
    CHECK(near_value(point.y_m, 0.343));
    CHECK(CORE_OK == core_range_to_point(&sample, -CORE_PI / 2.0, &point));
    CHECK(near_value(point.y_m, -0.343));

    /* A timeout must destroy the previous valid reading. */
    CHECK(CORE_NO_ECHO == core_ultrasonic_range(&config, false, NAN, &sample));
    CHECK(CORE_RANGE_UNKNOWN == sample.state);
    CHECK(isnan(sample.distance_m));
    point.x_m = 123.0;
    CHECK(CORE_UNAVAILABLE == core_range_to_point(&sample, 0.0, &point));
    CHECK(near_value(point.x_m, 123.0));

    CHECK(CORE_OUT_OF_RANGE ==
          core_ultrasonic_range(&config, true, 0.001 / 343.0, &sample));
    CHECK(CORE_RANGE_OUT_OF_RANGE == sample.state);
    CHECK(isnan(sample.distance_m));
    CHECK(CORE_OUT_OF_RANGE ==
          core_ultrasonic_range(&config, true, 10.0 / 343.0, &sample));
    CHECK(CORE_RANGE_OUT_OF_RANGE == sample.state);
    CHECK(CORE_UNAVAILABLE == core_range_to_point(&sample, 0.0, &point));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_ultrasonic_range(&config, true, 0.0, &sample));
    CHECK(CORE_RANGE_UNKNOWN == sample.state);
    CHECK(CORE_INVALID_ARGUMENT ==
          core_ultrasonic_range(&config, true, -0.001, &sample));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_ultrasonic_range(&config, true, NAN, &sample));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_ultrasonic_range(&config, true, INFINITY, &sample));
    CHECK(CORE_NUMERIC_ERROR ==
          core_ultrasonic_range(&config, true, DBL_MAX, &sample));
    CHECK(isnan(sample.distance_m));

    /* Exactly representable boundary check, independent of round-off at 343. */
    config.sound_speed_m_per_s = 2.0;
    config.min_distance_m      = 0.125;
    config.max_distance_m      = 4.0;
    CHECK(CORE_OK == core_ultrasonic_range(&config, true, 0.125, &sample));
    CHECK(near_value(sample.distance_m, 0.125));
    CHECK(CORE_OK == core_ultrasonic_range(&config, true, 4.0, &sample));
    CHECK(near_value(sample.distance_m, 4.0));
    config.sound_speed_m_per_s = NAN;
    CHECK(CORE_INVALID_ARGUMENT ==
          core_ultrasonic_range(&config, true, 0.001, &sample));
    CHECK(CORE_RANGE_UNKNOWN == sample.state);
    CHECK(CORE_INVALID_ARGUMENT ==
          core_ultrasonic_range(NULL, true, 0.001, &sample));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_ultrasonic_range(&config, true, 0.001, NULL));

    sample.state      = CORE_RANGE_VALID;
    sample.distance_m = 1.0;
    CHECK(CORE_INVALID_ARGUMENT == core_range_to_point(&sample, NAN, &point));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_range_to_point(&sample, 2.0 * CORE_PI, &point));
    CHECK(CORE_INVALID_ARGUMENT == core_range_to_point(NULL, 0.0, &point));
    CHECK(CORE_INVALID_ARGUMENT == core_range_to_point(&sample, 0.0, NULL));
    sample.distance_m = NAN;
    CHECK(CORE_INVALID_ARGUMENT == core_range_to_point(&sample, 0.0, &point));
    sample.distance_m = -1.0;
    CHECK(CORE_INVALID_ARGUMENT == core_range_to_point(&sample, 0.0, &point));
}

static void test_stopping_clearance (void)
{
    core_real_t clearance_m = 0.0;
    core_real_t faster_clearance_m = 0.0;

    /* 0.5*0.1 + 0.5^2/(2*1) + 0.08 + 0.02 = 0.275 m. */
    CHECK(CORE_OK ==
          core_stopping_clearance(0.5, 0.1, 1.0, 0.08, 0.02, &clearance_m));
    CHECK(near_value(clearance_m, 0.275));
    CHECK(CORE_OK ==
          core_stopping_clearance(1.0, 0.1, 1.0, 0.08, 0.02,
                                  &faster_clearance_m));
    CHECK(faster_clearance_m > clearance_m);
    CHECK(CORE_OK ==
          core_stopping_clearance(0.0, 0.1, 1.0, 0.08, 0.02, &clearance_m));
    CHECK(near_value(clearance_m, 0.1));

    clearance_m = 123.0;
    CHECK(CORE_INVALID_ARGUMENT ==
          core_stopping_clearance(-1.0, 0.1, 1.0, 0.08, 0.02, &clearance_m));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_stopping_clearance(1.0, -0.1, 1.0, 0.08, 0.02, &clearance_m));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_stopping_clearance(1.0, 0.1, 0.0, 0.08, 0.02, &clearance_m));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_stopping_clearance(1.0, 0.1, 1.0, -0.08, 0.02, &clearance_m));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_stopping_clearance(1.0, 0.1, 1.0, 0.08, -0.02, &clearance_m));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_stopping_clearance(NAN, 0.1, 1.0, 0.08, 0.02, &clearance_m));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_stopping_clearance(1.0, INFINITY, 1.0, 0.08, 0.02,
                                  &clearance_m));
    CHECK(CORE_INVALID_ARGUMENT ==
          core_stopping_clearance(1.0, 0.1, 1.0, 0.08, 0.02, NULL));
    CHECK(CORE_NUMERIC_ERROR ==
          core_stopping_clearance(DBL_MAX, 1.0, 1.0, 0.08, 0.02, &clearance_m));
    CHECK(near_value(clearance_m, 123.0));
}

/* End of file. */

