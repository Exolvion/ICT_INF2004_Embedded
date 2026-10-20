/** @file test_core_main.c
 * @brief Hosted regression checks for numerical contracts.
 * @details Five analytical fixture groups plus the shared pose and range
 * records; checks survive -DNDEBUG.
 */
#include "test_core_main.h"

#include "bitpack.h"
#include "core.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned int g_checks   = 0U;
static unsigned int g_failures = 0U;
/* Fixed seed: every run replays the same pseudo-random poses and ranges. */
static uint32_t     g_random   = 0x2545F491U;

/* Tests deliberately do not use assert(): -DNDEBUG must not disable checks. */
/* BARR-TEST-01: source location/stringification requires a macro; the
 * condition is evaluated once and the full expression is parenthesized.
 */
#define CHECK(b_condition) \
    (check_result((b_condition), #b_condition, __LINE__))

#define TEST_PID_SATURATION_STEPS (100U)
#define TEST_HUMP_TOTAL_STEPS (20U)
#define TEST_HUMP_ASCENT_STEPS (10U)
#define TEST_POSE_PAIRS (20000U)
/* 0.02..4.00 m in 0.1 mm steps, plus pseudo-random distances to 4.2 m. */
#define TEST_RANGE_STEPS (39800U)
#define TEST_RANGE_RANDOM (20000U)
#define TEST_PULSE_MAX_US (70000U)
/* One rounding of the double products in a bound check. */
#define TEST_ROUNDING_M (1.0e-9)

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
/** @brief Shared pose fields: nearest mm/mrad and the unshared path. */
static void test_pose_pack (void);
/** @brief Modulo-2^16 x/y, normalised yaw, invalid code and NULLs. */
static void test_pose_wrap (void);
/** @brief Wrap-safe pose differences, invalid poses and NULL outputs. */
static void test_pose_delta (void);
/** @brief The packed range/pose gate moves only inside its stated bound. */
static void test_pose_gate_bound (void);
/** @brief The packed corridor check moves only inside its stated bound. */
static void test_pose_corridor_bound (void);
/** @brief Shared range: states, CEIL millimetres and saturation. */
static void test_range_pack (void);
/** @brief Shared range unpacking, invalid codes and NULL arguments. */
static void test_range_unpack (void);
/** @brief CEIL millimetres keep the original front-stop decisions. */
static void test_range_equivalence (void);
static void test_range_pulses (void);
/** @brief Pack a pose holding only x; return the signed x field. */
static int32_t pose_x_mm (core_real_t x_m);
/** @brief Pack a pose holding only yaw; return the signed yaw field. */
static int32_t pose_yaw_mrad (core_real_t yaw_rad);
/** @brief Report whether a pose packs and unpacks as the invalid pose. */
static bool pose_packs_invalid (const core_pose_t * p_pose);
/** @brief Pack one range sample and return the packed record. */
static core_range_packed_t range_packed (core_range_state_t state,
                                         core_real_t distance_m);
/** @brief True when both front-stop decisions agree with the original. */
static bool range_decisions_match (core_real_t distance_m);
/** @brief Deterministic value in [low, high) from a fixed-seed LCG. */
static core_real_t test_random (core_real_t low, core_real_t high);

/** @brief Execute numerical checks; return failure on any failed check. */
int main (void)
{
    test_odometry();
    test_pid();
    test_hump_geometry();
    test_ultrasonic_and_points();
    test_stopping_clearance();
    test_pose_pack();
    test_pose_wrap();
    test_pose_delta();
    test_pose_gate_bound();
    test_pose_corridor_bound();
    test_range_pack();
    test_range_unpack();
    test_range_equivalence();
    test_range_pulses();

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

static void test_pose_pack (void)
{
    core_pose_t pose =
    {
        1.2344, -2.3456, 0.7854, 7.0
    };
    core_pose_t unpacked =
    {
        123.0, 123.0, 123.0, 123.0
    };
    core_pose_packed_t packed =
    {
        0U, 0U, 0U
    };
    int32_t yaw_mrad = 0;

    core_pose_pack(&pose, &packed);
    CHECK(1234 == bitpack_extend(packed.x_mm, CORE_POSE_XY_BITS));
    CHECK(-2346 == bitpack_extend(packed.y_mm, CORE_POSE_XY_BITS));
    CHECK(785 == bitpack_extend(packed.yaw_mrad, CORE_POSE_YAW_BITS));
    core_pose_unpack(&packed, &unpacked);
    CHECK(near_value(unpacked.x_m, 1.234));
    CHECK(near_value(unpacked.y_m, -2.346));
    CHECK(near_value(unpacked.yaw_rad, 0.785));
    /* The odometer is Motion-private: not shared, so it unpacks as 0. */
    CHECK(near_value(unpacked.centre_path_m, 0.0));

    /* Nearest rounding either side of each half millimetre/milliradian. */
    CHECK(5 == pose_x_mm(0.0054));
    CHECK(6 == pose_x_mm(0.0056));
    CHECK(-5 == pose_x_mm(-0.0054));
    CHECK(-6 == pose_x_mm(-0.0056));
    CHECK(0 == pose_x_mm(-0.0004));
    CHECK(-1 == pose_x_mm(-0.0006));
    CHECK(5 == pose_yaw_mrad(0.0054));
    CHECK(-6 == pose_yaw_mrad(-0.0056));

    /* Yaw is normalised to [-pi, pi] first: +/-3142 never saturates. */
    CHECK(3142 == pose_yaw_mrad(CORE_PI));
    CHECK(-3142 == pose_yaw_mrad(-CORE_PI));
    CHECK(-1571 == pose_yaw_mrad(1.5 * CORE_PI));
    CHECK(0 == pose_yaw_mrad(2.0 * CORE_PI));
    CHECK(717 == pose_yaw_mrad(7.0));
    CHECK(-717 == pose_yaw_mrad(-7.0));
    yaw_mrad = pose_yaw_mrad(DBL_MAX);
    CHECK((yaw_mrad >= -3142) && (yaw_mrad <= 3142));
}

static void test_pose_wrap (void)
{
    const core_real_t bad[] =
    {
        NAN, INFINITY, -INFINITY, DBL_MAX, -DBL_MAX
    };
    core_pose_t pose =
    {
        1.0, 1.0, 0.5, 2.0
    };
    core_pose_t unpacked =
    {
        9.0, 9.0, 9.0, 9.0
    };
    core_pose_packed_t packed =
    {
        0x1234U, 0x5678U, 0x0123U
    };
    unsigned int index;

    /* x/y keep the low 16 bits of the whole millimetre count. */
    CHECK(32767 == pose_x_mm(32.767));
    CHECK(-32768 == pose_x_mm(-32.768));
    CHECK(-32768 == pose_x_mm(32.768));
    CHECK(-32767 == pose_x_mm(32.769));
    CHECK(0 == pose_x_mm(65.536));
    CHECK(-31072 == pose_x_mm(100.0));
    CHECK(31072 == pose_x_mm(-100.0));
    CHECK(4096 == pose_x_mm(1.0e9));

    /* Never produced by core_odometry_step, but stored deterministically.
     * DBL_MAX is finite, but its millimetre value overflows.
     */
    for (index = 0U; index < (sizeof(bad) / sizeof(bad[0])); ++index)
    {
        pose.x_m = bad[index];
        CHECK(pose_packs_invalid(&pose));
        pose.x_m = 1.0;
        pose.y_m = bad[index];
        CHECK(pose_packs_invalid(&pose));
        pose.y_m = 1.0;
    }
    pose.yaw_rad = NAN;
    CHECK(pose_packs_invalid(&pose));
    pose.yaw_rad = -INFINITY;
    CHECK(pose_packs_invalid(&pose));
    pose.yaw_rad = 0.5;

    /* NULL arguments leave every output untouched. */
    core_pose_pack(NULL, &packed);
    CHECK((0x1234U == packed.x_mm) && (0x5678U == packed.y_mm) &&
          (0x0123U == packed.yaw_mrad));
    core_pose_pack(&pose, NULL);
    core_pose_unpack(NULL, &unpacked);
    CHECK(near_value(unpacked.x_m, 9.0) && near_value(unpacked.yaw_rad, 9.0));
    core_pose_unpack(&packed, NULL);
}

static void test_pose_delta (void)
{
    core_pose_t first =
    {
        65.530, 1.0, -3.0, 0.0
    };
    core_pose_t second =
    {
        65.540, -2.5, 3.0, 0.0
    };
    core_pose_packed_t from =
    {
        0x7FFFU, 0U, 0U
    };
    core_pose_packed_t to =
    {
        0x8000U, 0U, 0U
    };
    core_real_t dx_m     = 9.0;
    core_real_t dy_m     = 9.0;
    core_real_t dyaw_rad = 9.0;

    /* Crossing the +32.767 m seam is +1 mm; 0 minus 1 mm is -1 mm. */
    core_pose_delta(&to, &from, &dx_m, &dy_m, &dyaw_rad);
    CHECK(near_value(dx_m, 0.001));
    CHECK(near_value(dy_m, 0.0) && near_value(dyaw_rad, 0.0));
    core_pose_delta(&from, &to, &dx_m, NULL, NULL);
    CHECK(near_value(dx_m, -0.001));
    from.x_mm = 0U;
    to.x_mm   = 0xFFFFU;
    core_pose_delta(&to, &from, &dx_m, NULL, NULL);
    CHECK(near_value(dx_m, -0.001));
    to.x_mm = 0x7FFFU;
    core_pose_delta(&to, &from, &dx_m, NULL, NULL);
    CHECK(near_value(dx_m, 32.767));
    to.x_mm = 0x8001U;
    core_pose_delta(&to, &from, &dx_m, NULL, NULL);
    CHECK(near_value(dx_m, -32.767));

    /* Absolute poses past 32.767 m still difference exactly. */
    core_pose_pack(&first, &from);
    core_pose_pack(&second, &to);
    core_pose_delta(&to, &from, &dx_m, &dy_m, &dyaw_rad);
    CHECK(near_value(dx_m, 0.010));
    CHECK(near_value(dy_m, -3.5));
    CHECK(near_value(dyaw_rad, 6.0 - (2.0 * CORE_PI)));
    core_pose_delta(&from, &to, NULL, NULL, &dyaw_rad);
    CHECK(near_value(dyaw_rad, (2.0 * CORE_PI) - 6.0));

    /* +pi against -pi is a 0.8 mrad step, not a full turn. */
    first.yaw_rad  = -CORE_PI;
    second.yaw_rad = CORE_PI;
    core_pose_pack(&first, &from);
    core_pose_pack(&second, &to);
    core_pose_delta(&to, &from, NULL, NULL, &dyaw_rad);
    CHECK(fabs(dyaw_rad) < 0.001);
    core_pose_delta(&from, &to, NULL, NULL, &dyaw_rad);
    CHECK(fabs(dyaw_rad) < 0.001);

    /* An invalid or missing pose gives NAN, which fails every <= gate. */
    first.x_m = NAN;
    core_pose_pack(&first, &from);
    core_pose_delta(&to, &from, &dx_m, &dy_m, &dyaw_rad);
    CHECK(isnan(dx_m) && isnan(dy_m) && isnan(dyaw_rad));
    dx_m = 9.0;
    core_pose_delta(&from, &to, &dx_m, NULL, NULL);
    CHECK(isnan(dx_m));
    dx_m = 9.0;
    core_pose_delta(NULL, &to, &dx_m, NULL, NULL);
    CHECK(isnan(dx_m));
    dx_m = 9.0;
    core_pose_delta(&to, NULL, &dx_m, NULL, NULL);
    CHECK(isnan(dx_m));
    core_pose_delta(&to, &to, NULL, NULL, NULL);
}

static void test_pose_gate_bound (void)
{
    core_pose_t        front         = {0.0, 0.0, 0.0, 0.0};
    core_pose_t        motion        = {0.0, 0.0, 0.0, 0.0};
    core_pose_packed_t front_packed  = {0U, 0U, 0U};
    core_pose_packed_t motion_packed = {0U, 0U, 0U};
    core_real_t        dx_m          = 0.0;
    core_real_t        dy_m          = 0.0;
    core_real_t        dyaw_rad      = 0.0;
    core_real_t        exact_m       = 0.0;
    core_real_t        exact_rad     = 0.0;
    core_real_t        xy_error      = 0.0;
    core_real_t        yaw_error     = 0.0;
    const core_real_t  xy_bound  = (sqrt(2.0) * 0.001) + TEST_ROUNDING_M;
    const core_real_t  yaw_bound = 0.001 + TEST_ROUNDING_M;
    unsigned int       outside   = 0U;
    unsigned int       index;
    bool               b_old;
    bool               b_new;

    for (index = 0U; index < TEST_POSE_PAIRS; ++index)
    {
        front.x_m      = test_random(-40.0, 40.0);
        front.y_m      = test_random(-40.0, 40.0);
        front.yaw_rad  = test_random(-CORE_PI, CORE_PI);
        motion.x_m     = front.x_m + test_random(-0.035, 0.035);
        motion.y_m     = front.y_m + test_random(-0.035, 0.035);
        motion.yaw_rad = remainder(front.yaw_rad + test_random(-0.14, 0.14),
                                   2.0 * CORE_PI);
        core_pose_pack(&front, &front_packed);
        core_pose_pack(&motion, &motion_packed);
        core_pose_delta(&motion_packed, &front_packed, &dx_m, &dy_m,
                        &dyaw_rad);
        /* The range gate exactly as robot_app_vehicle_input wrote it. */
        exact_rad = fabs(atan2(sin(motion.yaw_rad - front.yaw_rad),
                               cos(motion.yaw_rad - front.yaw_rad)));
        exact_m = hypot(motion.x_m - front.x_m, motion.y_m - front.y_m);
        b_old   = (exact_rad <= 0.12) && (exact_m <= 0.03);
        b_new   = (fabs(dyaw_rad) <= 0.12) && (hypot(dx_m, dy_m) <= 0.03);
        xy_error  = fmax(xy_error, fabs(hypot(dx_m, dy_m) - exact_m));
        yaw_error = fmax(yaw_error, fabs(fabs(dyaw_rad) - exact_rad));
        if ((b_old != b_new) && (fabs(exact_m - 0.03) > xy_bound) &&
            (fabs(exact_rad - 0.12) > yaw_bound))
        {
            ++outside;
        }
    }
    /* Each axis rounds twice (+/-0.5 mm): sqrt(2) mm and 1 mrad. */
    CHECK(xy_error <= xy_bound);
    CHECK(yaw_error <= yaw_bound);
    CHECK(0U == outside);
}

static void test_pose_corridor_bound (void)
{
    core_pose_t        anchor   = {0.0, 0.0, 0.0, 0.0};
    core_pose_t        motion   = {0.0, 0.0, 0.0, 0.0};
    core_pose_t        unpacked = {0.0, 0.0, 0.0, 0.0};
    core_pose_packed_t anchor_packed = {0U, 0U, 0U};
    core_pose_packed_t motion_packed = {0U, 0U, 0U};
    core_real_t        along_m  = 0.0;
    core_real_t        side_m   = 0.0;
    core_real_t        old_m    = 0.0;
    core_real_t        new_m    = 0.0;
    core_real_t        old_rad  = 0.0;
    core_real_t        new_rad  = 0.0;
    core_real_t        dx_m     = 0.0;
    core_real_t        dy_m     = 0.0;
    core_real_t        bound_m  = 0.0;
    core_real_t        worst    = -1.0;
    const core_real_t  yaw_bound = 0.001 + TEST_ROUNDING_M;
    unsigned int       outside   = 0U;
    unsigned int       index;

    for (index = 0U; index < TEST_POSE_PAIRS; ++index)
    {
        anchor.x_m     = test_random(-40.0, 40.0);
        anchor.y_m     = test_random(-40.0, 40.0);
        anchor.yaw_rad = test_random(-CORE_PI, CORE_PI);
        along_m        = test_random(0.0, 3.4);
        side_m         = test_random(-0.045, 0.045);
        motion.x_m = anchor.x_m + (along_m * cos(anchor.yaw_rad)) -
                     (side_m * sin(anchor.yaw_rad));
        motion.y_m = anchor.y_m + (along_m * sin(anchor.yaw_rad)) +
                     (side_m * cos(anchor.yaw_rad));
        motion.yaw_rad = remainder(anchor.yaw_rad + test_random(-0.24, 0.24),
                                   2.0 * CORE_PI);
        core_pose_pack(&anchor, &anchor_packed);
        core_pose_pack(&motion, &motion_packed);
        /* robot_app_original_line as written before the change. */
        dx_m    = motion.x_m - anchor.x_m;
        dy_m    = motion.y_m - anchor.y_m;
        old_m   = (-sin(anchor.yaw_rad) * dx_m) + (cos(anchor.yaw_rad) * dy_m);
        old_rad = atan2(sin(motion.yaw_rad - anchor.yaw_rad),
                        cos(motion.yaw_rad - anchor.yaw_rad));
        /* sqrt(2) mm from x/y, 0.5 mrad of anchor yaw over the distance. */
        bound_m = (sqrt(2.0) * 0.001) + (0.0005 * hypot(dx_m, dy_m)) +
                  TEST_ROUNDING_M;
        core_pose_unpack(&anchor_packed, &unpacked);
        core_pose_delta(&motion_packed, &anchor_packed, &dx_m, &dy_m,
                        &new_rad);
        new_m = (-sin(unpacked.yaw_rad) * dx_m) +
                (cos(unpacked.yaw_rad) * dy_m);
        worst = fmax(worst, fabs(new_m - old_m) - bound_m);
        worst = fmax(worst, fabs(fabs(new_rad) - fabs(old_rad)) - yaw_bound);
        if ((((fabs(old_m) <= 0.03) && (fabs(old_rad) <= 0.20)) !=
             ((fabs(new_m) <= 0.03) && (fabs(new_rad) <= 0.20))) &&
            (fabs(fabs(old_m) - 0.03) > bound_m) &&
            (fabs(fabs(old_rad) - 0.20) > yaw_bound))
        {
            ++outside;
        }
    }
    CHECK(worst <= 0.0);
    CHECK(0U == outside);
}

static void test_range_pack (void)
{
    core_range_packed_t packed = range_packed(CORE_RANGE_VALID, 0.25);

    CHECK((CORE_RANGE_VALID == packed.state) && (250U == packed.distance_mm));
    packed = range_packed(CORE_RANGE_VALID, 0.0);
    CHECK((CORE_RANGE_VALID == packed.state) && (0U == packed.distance_mm));
    /* CEIL: any fraction of a millimetre rounds up. */
    packed = range_packed(CORE_RANGE_VALID, 0.2495);
    CHECK((CORE_RANGE_VALID == packed.state) && (250U == packed.distance_mm));
    packed = range_packed(CORE_RANGE_VALID, 0.2501);
    CHECK(251U == packed.distance_mm);
    packed = range_packed(CORE_RANGE_VALID, nextafter(0.25, INFINITY));
    CHECK(251U == packed.distance_mm);
    packed = range_packed(CORE_RANGE_VALID, 4.0);
    CHECK(4000U == packed.distance_mm);
    packed = range_packed(CORE_RANGE_VALID, 4.095);
    CHECK(4095U == packed.distance_mm);
    /* Above 4.095 m the field saturates and the reading stays VALID. */
    packed = range_packed(CORE_RANGE_VALID, 4.0955);
    CHECK((CORE_RANGE_VALID == packed.state) && (4095U == packed.distance_mm));
    packed = range_packed(CORE_RANGE_VALID, DBL_MAX);
    CHECK((CORE_RANGE_VALID == packed.state) && (4095U == packed.distance_mm));
    packed = range_packed(CORE_RANGE_VALID, -0.5);
    CHECK((CORE_RANGE_VALID == packed.state) && (0U == packed.distance_mm));
    packed = range_packed(CORE_RANGE_VALID, -DBL_MAX);
    CHECK((CORE_RANGE_VALID == packed.state) && (0U == packed.distance_mm));
    /* VALID without a finite distance is never a measurement. */
    packed = range_packed(CORE_RANGE_VALID, NAN);
    CHECK((CORE_RANGE_UNKNOWN == packed.state) && (0U == packed.distance_mm));
    packed = range_packed(CORE_RANGE_VALID, INFINITY);
    CHECK((CORE_RANGE_UNKNOWN == packed.state) && (0U == packed.distance_mm));
    /* Other states store distance 0; an undefined state reads UNKNOWN. */
    packed = range_packed(CORE_RANGE_OUT_OF_RANGE, NAN);
    CHECK((CORE_RANGE_OUT_OF_RANGE == packed.state) &&
          (0U == packed.distance_mm));
    packed = range_packed(CORE_RANGE_OUT_OF_RANGE, 1.0);
    CHECK((CORE_RANGE_OUT_OF_RANGE == packed.state) &&
          (0U == packed.distance_mm));
    packed = range_packed(CORE_RANGE_UNKNOWN, 1.0);
    CHECK((CORE_RANGE_UNKNOWN == packed.state) && (0U == packed.distance_mm));
    packed = range_packed((core_range_state_t) 7, 1.0);
    CHECK((CORE_RANGE_UNKNOWN == packed.state) && (0U == packed.distance_mm));
}

static void test_range_unpack (void)
{
    core_ultrasonic_config_t config =
    {
        2.0, 0.125, 4.0
    };
    core_range_sample_t sample =
    {
        CORE_RANGE_UNKNOWN, NAN
    };
    core_range_packed_t packed = range_packed(CORE_RANGE_VALID, 4.095);

    core_range_unpack(&packed, &sample);
    CHECK((CORE_RANGE_VALID == sample.state) &&
          near_value(sample.distance_m, 4.095));
    /* A qualified range round-trips through the shared record. */
    CHECK(CORE_OK == core_ultrasonic_range(&config, true, 0.25, &sample));
    core_range_pack(&sample, &packed);
    sample.distance_m = 9.0;
    core_range_unpack(&packed, &sample);
    CHECK((CORE_RANGE_VALID == sample.state) &&
          near_value(sample.distance_m, 0.25));
    /* A timeout stays UNKNOWN with NAN, as core_ultrasonic_range makes it. */
    CHECK(CORE_NO_ECHO == core_ultrasonic_range(&config, false, NAN, &sample));
    core_range_pack(&sample, &packed);
    sample.distance_m = 9.0;
    core_range_unpack(&packed, &sample);
    CHECK((CORE_RANGE_UNKNOWN == sample.state) && isnan(sample.distance_m));
    /* A stored distance never leaks out of a non-VALID state. */
    packed.state       = CORE_RANGE_OUT_OF_RANGE;
    packed.distance_mm = 77U;
    core_range_unpack(&packed, &sample);
    CHECK((CORE_RANGE_OUT_OF_RANGE == sample.state) &&
          isnan(sample.distance_m));
    /* All ones: the unused fourth state code reads as UNKNOWN. */
    memset(&packed, 0xFF, sizeof(packed));
    core_range_unpack(&packed, &sample);
    CHECK((CORE_RANGE_UNKNOWN == sample.state) && isnan(sample.distance_m));
    /* NULL arguments have no effect. */
    sample.state      = CORE_RANGE_VALID;
    sample.distance_m = 1.5;
    core_range_unpack(NULL, &sample);
    CHECK((CORE_RANGE_VALID == sample.state) &&
          near_value(sample.distance_m, 1.5));
    core_range_unpack(&packed, NULL);
    packed = range_packed(CORE_RANGE_VALID, 0.123);
    core_range_pack(NULL, &packed);
    CHECK((CORE_RANGE_VALID == packed.state) && (123U == packed.distance_mm));
    core_range_pack(&sample, NULL);
}

/** @brief Every integer echo pulse 0..70,000 us through the real range
 * conversion (as robot_app_front_result uses it): the packed front-stop
 * decisions equal the original double predicates, and a reading that is
 * not valid never packs as valid.
 */
static void test_range_pulses (void)
{
    const core_ultrasonic_config_t config = {343.0, 0.02, 4.0};
    core_range_sample_t sample;
    core_range_packed_t packed;
    unsigned int pulse_us = 0U;
    unsigned int valid = 0U;
    unsigned int mismatches = 0U;

    for (pulse_us = 0U; pulse_us <= TEST_PULSE_MAX_US; ++pulse_us)
    {
        (void)core_ultrasonic_range(&config, true,
                                    (core_real_t)pulse_us / 1000000.0,
                                    &sample);
        core_range_pack(&sample, &packed);
        if (CORE_RANGE_VALID == sample.state)
        {
            ++valid;
            if ((CORE_RANGE_VALID != packed.state) ||
                ((sample.distance_m <= 0.25) !=
                 (packed.distance_mm <= 250U)) ||
                ((sample.distance_m > 0.25) != (packed.distance_mm > 250U)))
            {
                ++mismatches;
            }
        }
        else if (CORE_RANGE_VALID == packed.state)
        {
            ++mismatches;
        }
    }
    CHECK(valid > 20000U);
    CHECK(0U == mismatches);
}

static void test_range_equivalence (void)
{
    const core_real_t edges[] =
    {
        0.0, 0.02, 0.249, 0.2495, 0.2499, 0.2501, 0.2505, 0.251, 4.0, 4.095,
        4.2
    };
    core_real_t  distance_m = 0.0;
    unsigned int mismatches = 0U;
    unsigned int index;

    /* The threshold and its double neighbours (spec rule 5). */
    CHECK(range_decisions_match(0.25));
    CHECK(range_decisions_match(nextafter(0.25, INFINITY)));
    CHECK(range_decisions_match(nextafter(0.25, -INFINITY)));
    for (index = 0U; index < (sizeof(edges) / sizeof(edges[0])); ++index)
    {
        CHECK(range_decisions_match(edges[index]));
    }
    for (index = 0U; index <= TEST_RANGE_STEPS; ++index)
    {
        distance_m = 0.02 + ((core_real_t) index * 1.0e-4);
        if ((!range_decisions_match(distance_m)) ||
            (!range_decisions_match(nextafter(distance_m, INFINITY))) ||
            (!range_decisions_match(nextafter(distance_m, -INFINITY))))
        {
            ++mismatches;
        }
    }
    for (index = 0U; index < TEST_RANGE_RANDOM; ++index)
    {
        if (!range_decisions_match(test_random(0.0, 4.2)))
        {
            ++mismatches;
        }
    }
    CHECK(0U == mismatches);
}

static int32_t pose_x_mm (core_real_t x_m)
{
    core_pose_t pose =
    {
        0.0, 0.0, 0.0, 0.0
    };
    core_pose_packed_t packed =
    {
        0U, 0U, 0U
    };

    pose.x_m = x_m;
    core_pose_pack(&pose, &packed);
    return bitpack_extend(packed.x_mm, CORE_POSE_XY_BITS);
}

static int32_t pose_yaw_mrad (core_real_t yaw_rad)
{
    core_pose_t pose =
    {
        0.0, 0.0, 0.0, 0.0
    };
    core_pose_packed_t packed =
    {
        0U, 0U, 0U
    };

    pose.yaw_rad = yaw_rad;
    core_pose_pack(&pose, &packed);
    return bitpack_extend(packed.yaw_mrad, CORE_POSE_YAW_BITS);
}

static bool pose_packs_invalid (const core_pose_t * p_pose)
{
    core_pose_packed_t packed =
    {
        1U, 1U, 1U
    };
    core_pose_t unpacked =
    {
        0.0, 0.0, 0.0, 1.0
    };

    core_pose_pack(p_pose, &packed);
    core_pose_unpack(&packed, &unpacked);
    return (0U == packed.x_mm) && (0U == packed.y_mm) &&
           (bitpack_invalid(CORE_POSE_YAW_BITS) == packed.yaw_mrad) &&
           isnan(unpacked.x_m) && isnan(unpacked.y_m) &&
           isnan(unpacked.yaw_rad) && near_value(unpacked.centre_path_m, 0.0);
}

static core_range_packed_t range_packed (core_range_state_t state,
                                         core_real_t distance_m)
{
    core_range_sample_t sample =
    {
        CORE_RANGE_UNKNOWN, NAN
    };
    core_range_packed_t packed =
    {
        CORE_RANGE_OUT_OF_RANGE, 1U
    };

    sample.state      = state;
    sample.distance_m = distance_m;
    core_range_pack(&sample, &packed);
    return packed;
}

static bool range_decisions_match (core_real_t distance_m)
{
    core_range_packed_t packed = range_packed(CORE_RANGE_VALID, distance_m);

    /* Left of each ==: robot_app.c before the change, on the double and
     * the 0.25 literal (ROBOT_FRONT_STOP_DISTANCE_M). Right: the packed
     * compare on ROBOT_FRONT_STOP_DISTANCE_MM (250U).
     */
    return (CORE_RANGE_VALID == packed.state) &&
           ((distance_m <= 0.25) == (packed.distance_mm <= 250U)) &&
           ((distance_m > 0.25) == (packed.distance_mm > 250U));
}

static core_real_t test_random (core_real_t low, core_real_t high)
{
    /* Numerical Recipes LCG; uint32_t arithmetic wraps modulo 2^32. */
    g_random = (g_random * 1664525U) + 1013904223U;
    return low + ((high - low) * ((core_real_t) g_random / 4294967296.0));
}

/* End of file. */

