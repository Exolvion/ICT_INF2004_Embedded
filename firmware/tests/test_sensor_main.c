/** @file test_sensor_main.c
 * @brief Deterministic acquisition timing, unknown-range and hump regressions.
 */
#include "test_sensor_main.h"

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "scanner.h"
#include "terrain.h"

#define SENSOR_TEST_LIMIT (30000U)
#define SENSOR_SLOPE_SAMPLES (20U)
#define SENSOR_FLAT_SAMPLES (6U)
#define SENSOR_TOLERANCE (0.000001)
#define SENSOR_RANGE_CASES (5U)

typedef enum
{
    SENSOR_SCENE_CLEAR = 0,
    SENSOR_SCENE_CENTER,
    SENSOR_SCENE_LEFT_BLOCKED,
    SENSOR_SCENE_UNKNOWN,
    SENSOR_SCENE_OUT_OF_RANGE,
    SENSOR_SCENE_RECOVERY
} test_sensor_main_scene_t;

static uint32_t m_checks = 0U;
static uint32_t m_failures = 0U;

static void test_sensor_main_check (bool b_condition, const char * p_label);
static bool test_sensor_main_near (double actual, double expected);
static void test_sensor_main_config (void);
static void test_sensor_main_timing (void);
static void test_sensor_main_ranges (void);
static void test_sensor_main_scan (test_sensor_main_scene_t scene);
static void test_sensor_main_abort (void);
static void test_sensor_main_terrain (void);
static void test_sensor_main_invalid (void);
static void test_sensor_main_motion (void);
static void test_sensor_main_plateau (void);
static void test_sensor_main_slope (terrain_state_t * p_state,
                                    double angle_rad, double distance_m,
                                    uint32_t count, bool b_valid);
static uint32_t test_sensor_main_echo (test_sensor_main_scene_t scene,
                                      int32_t angle_deg, bool b_fine);

/** Execute each independent scenario and return a CI-compatible status. */
int main (void)
{
    int printed = 0;

    test_sensor_main_config();
    test_sensor_main_timing();
    test_sensor_main_ranges();
    test_sensor_main_scan(SENSOR_SCENE_CLEAR);
    test_sensor_main_scan(SENSOR_SCENE_CENTER);
    test_sensor_main_scan(SENSOR_SCENE_LEFT_BLOCKED);
    test_sensor_main_scan(SENSOR_SCENE_UNKNOWN);
    test_sensor_main_scan(SENSOR_SCENE_OUT_OF_RANGE);
    test_sensor_main_scan(SENSOR_SCENE_RECOVERY);
    test_sensor_main_abort();
    test_sensor_main_terrain();
    test_sensor_main_invalid();
    test_sensor_main_motion();
    test_sensor_main_plateau();
    printed = printf("Sensor checks: %" PRIu32 "; failures: %" PRIu32 "\n",
                     m_checks, m_failures);
    return ((printed >= 0) && (0U == m_failures)) ? EXIT_SUCCESS : EXIT_FAILURE;
}

/** Record one runtime assertion with a meaningful failing diagnostic. */
static void test_sensor_main_check (bool b_condition, const char * p_label)
{
    m_checks++;
    if (!b_condition)
    {
        int printed = 0;

        m_failures++;
        printed = fprintf(stderr, "FAIL: %s\n", p_label);
        if (printed < 0)
        {
            m_failures++;
        }
    }
}

/** Comparison excludes NaN and infinity instead of accepting accidental NaN. */
static bool test_sensor_main_near (double actual, double expected)
{
    return (isfinite(actual)) && (isfinite(expected))
           && (fabs(actual - expected) < SENSOR_TOLERANCE);
}

/** Defaults remain disabled; invalid configuration cannot start acquisition. */
static void test_sensor_main_config (void)
{
    scanner_config_t scan_config;
    scanner_state_t scan_state;
    terrain_config_t terrain_config;
    terrain_state_t terrain_state;

    scanner_default_config(&scan_config);
    terrain_default_config(&terrain_config);
    test_sensor_main_check(scanner_init(&scan_state, &scan_config),
                           "scanner defaults structurally valid");
    test_sensor_main_check(!scanner_start(&scan_state, 0U, true),
                           "uncalibrated scanner cannot start");
    scan_config.b_calibrated = true;
    scan_config.trigger_interval_ms = 60U;
    test_sensor_main_check(!scanner_init(&scan_state, &scan_config),
                           "60 ms trigger interval rejected");
    scanner_default_config(&scan_config);
    scan_config.range.min_distance_m = 0.001;
    test_sensor_main_check(!scanner_init(&scan_state, &scan_config),
                           "below specified range rejected");
    test_sensor_main_check(terrain_init(&terrain_state, &terrain_config),
                           "terrain defaults structurally valid");
    test_sensor_main_check(!terrain_reference(&terrain_state, 0.0, 0.0,
                           terrain_config.gravity_mps2, true),
                           "uncalibrated terrain refuses reference");
    terrain_config.slope_enter_rad = NAN;
    test_sensor_main_check(!terrain_init(&terrain_state, &terrain_config),
                           "nonfinite terrain threshold rejected");
    test_sensor_main_check(!scanner_init(NULL, &scan_config),
                           "null scanner state rejected");
    test_sensor_main_check(!terrain_init(NULL, &terrain_config),
                           "null terrain state rejected");
}

/** Independently verify echo timeout, servo settling and clock wrapping. */
static void test_sensor_main_timing (void)
{
    scanner_config_t config;
    scanner_state_t state;
    scanner_output_t output;
    scanner_echo_t echo = {false, 0U};
    const uint32_t start_ms = UINT32_MAX - 100U;

    scanner_default_config(&config);
    config.b_calibrated = true;
    test_sensor_main_check(scanner_init(&state, &config), "timing init");
    test_sensor_main_check(scanner_start(&state, start_ms, true),
                           "start near uint32 wrap");
    test_sensor_main_check(scanner_step(&state, start_ms, true, &echo, &output),
                           "position service");
    test_sensor_main_check(output.b_set_servo && (!output.b_trigger)
                           && (-60 == output.angle_deg), "first servo angle");
    test_sensor_main_check(scanner_step(&state, start_ms + 149U, true,
                           &echo, &output), "pre-settle service");
    test_sensor_main_check(!output.b_trigger, "no early trigger");
    test_sensor_main_check(scanner_step(&state, start_ms + 150U, true,
                           &echo, &output), "settled service over wrap");
    test_sensor_main_check(output.b_trigger, "trigger after servo settle");
    test_sensor_main_check(scanner_step(&state, start_ms + 179U, true,
                           &echo, &output), "pre-timeout service");
    test_sensor_main_check(0U == state.sample_count, "no early timeout");
    test_sensor_main_check(scanner_step(&state, start_ms + 180U, true,
                           &echo, &output), "timeout service");
    test_sensor_main_check((1U == state.sample_count)
                           && (CORE_RANGE_UNKNOWN
                               == state.samples[0].range.state)
                           && (isnan(state.samples[0].range.distance_m)),
                           "timeout is unknown and distance invalidated");
    test_sensor_main_check(!scanner_start(&state, 200U, true),
                           "active scan cannot be overwritten");
}

/** Quantized pulse widths straddle both stated sensor range boundaries. */
static void test_sensor_main_ranges (void)
{
    const uint32_t pulses[SENSOR_RANGE_CASES] =
    {
        116U, 117U, 23323U, 23324U, 0U
    };
    const core_range_state_t expected[SENSOR_RANGE_CASES] =
    {
        CORE_RANGE_OUT_OF_RANGE, CORE_RANGE_VALID, CORE_RANGE_VALID,
        CORE_RANGE_OUT_OF_RANGE, CORE_RANGE_UNKNOWN
    };
    scanner_config_t config;
    scanner_state_t state;
    scanner_output_t output;
    scanner_echo_t echo = {false, 0U};
    uint32_t index = 0U;
    bool b_service_valid = true;

    scanner_default_config(&config);
    config.b_calibrated = true;
    for (index = 0U; index < SENSOR_RANGE_CASES; index++)
    {
        echo.b_ready = false;
        echo.pulse_us = 0U;
        b_service_valid = scanner_init(&state, &config) && b_service_valid;
        b_service_valid = scanner_start(&state, 0U, true) && b_service_valid;
        b_service_valid = scanner_step(&state, 0U, true, &echo, &output)
                          && b_service_valid;
        b_service_valid = scanner_step(&state, 150U, true, &echo, &output)
                          && b_service_valid;
        echo.b_ready = (pulses[index] > 0U);
        echo.pulse_us = pulses[index];
        b_service_valid = scanner_step(&state, 180U, true, &echo, &output)
                          && b_service_valid;
        test_sensor_main_check((1U == state.sample_count)
                               && (expected[index]
                                   == state.samples[0].range.state),
                               "pulse boundary state is explicit");
    }
    test_sensor_main_check(b_service_valid, "range boundary services valid");
}

/** Drive a sweep with echo completion delayed by each pulse duration. */
static void test_sensor_main_scan (test_sensor_main_scene_t scene)
{
    scanner_config_t config;
    scanner_state_t state;
    scanner_output_t output;
    scanner_echo_t echo = {false, 0U};
    uint32_t now_ms = 0U;
    uint32_t previous_trigger_ms = 0U;
    uint32_t echo_due_ms = 0U;
    uint32_t triggers = 0U;
    bool b_done = false;
    bool b_timing_valid = true;
    bool b_service_valid = true;

    scanner_default_config(&config);
    config.b_calibrated = true;
    test_sensor_main_check(scanner_init(&state, &config), "scene init");
    test_sensor_main_check(scanner_start(&state, 0U, true), "scene start");
    for (now_ms = 0U; (now_ms < SENSOR_TEST_LIMIT) && (!b_done); now_ms++)
    {
        echo.b_ready = (echo.pulse_us > 0U) && (now_ms >= echo_due_ms);
        b_service_valid = scanner_step(&state, now_ms, true, &echo, &output)
                          && b_service_valid;
        if (echo.b_ready)
        {
            echo.pulse_us = 0U;
        }
        if (output.b_trigger)
        {
            if (triggers > 0U)
            {
                b_timing_valid = b_timing_valid
                    && ((now_ms - previous_trigger_ms) >= 65U);
            }
            previous_trigger_ms = now_ms;
            triggers++;
            echo.pulse_us = test_sensor_main_echo(scene, state.angle_deg,
                                                  state.b_fine);
            echo_due_ms = now_ms + ((echo.pulse_us + 999U) / 1000U);
        }
        b_done = output.b_done;
    }
    test_sensor_main_check(b_service_valid, "complete scan service valid");
    test_sensor_main_check(b_timing_valid, "all trigger intervals >=65 ms");
    test_sensor_main_check(b_done && (SCANNER_COMPLETE == state.phase),
                           "scan completes within 30 s bound");
    test_sensor_main_check(output.b_set_servo && (0 == output.angle_deg),
                           "completed scan returns servo forward");
    test_sensor_main_check(state.sample_count <= SCANNER_MAX_SAMPLES,
                           "fixed sample buffer respected");
    if ((SENSOR_SCENE_UNKNOWN == scene) || (SENSOR_SCENE_OUT_OF_RANGE == scene))
    {
        test_sensor_main_check(!state.result.b_profile_valid,
                               "unknown scan cannot validate profile");
        test_sensor_main_check(SCANNER_SIDE_NONE == state.result.preferred_side,
                               "unknown never becomes clear route");
        test_sensor_main_check(74U == state.sample_count,
                               "unknown coarse scan triggers full fine sweep");
    }
    else
    {
        test_sensor_main_check(state.result.b_profile_valid,
                               "all valid final observations validate profile");
        test_sensor_main_check(state.result.valid_samples >= 13U,
                               "full angular coverage acquired");
        if (SENSOR_SCENE_CENTER == scene)
        {
            test_sensor_main_check(state.result.b_obstacle_seen
                                   && (state.sample_count > 13U),
                                   "center obstacle receives fine scan");
            test_sensor_main_check(fabs(state.result.apparent_width_m
                                   - (0.6 * sin(10.0 * CORE_PI / 180.0)))
                                   < 0.001, "visible endpoint width geometry");
        }
        else if (SENSOR_SCENE_LEFT_BLOCKED == scene)
        {
            test_sensor_main_check(SCANNER_SIDE_RIGHT
                                   == state.result.preferred_side,
                                   "blocked left prefers observed right");
        }
        else
        {
            test_sensor_main_check(!state.result.b_obstacle_seen,
                                   "far scene has no near obstacle");
        }
    }
}

/** A moving platform invalidates the shared-pose scan immediately. */
static void test_sensor_main_abort (void)
{
    scanner_config_t config;
    scanner_state_t state;
    scanner_output_t output;
    scanner_echo_t echo = {false, 0U};

    scanner_default_config(&config);
    config.b_calibrated = true;
    test_sensor_main_check(scanner_init(&state, &config), "abort init");
    test_sensor_main_check(!scanner_start(&state, 0U, false),
                           "moving start rejected");
    test_sensor_main_check(scanner_start(&state, 0U, true), "abort start");
    test_sensor_main_check(scanner_step(&state, 1U, false, &echo, &output),
                           "motion abort service");
    test_sensor_main_check(state.result.b_aborted && output.b_done
                           && (!state.result.b_profile_valid)
                           && (SCANNER_SIDE_NONE
                               == state.result.preferred_side),
                           "motion abort gives no trusted route");
}

/** Exact synthetic triangular humps establish integration/maxima behavior. */
static void test_sensor_main_terrain (void)
{
    terrain_config_t config;
    terrain_state_t state;
    const double angle = 0.2;
    const double expected = 0.2 * sin(angle);

    terrain_default_config(&config);
    config.b_calibrated = true;
    config.filter_time_s = 0.0;
    test_sensor_main_check(terrain_init(&state, &config), "hump init");
    test_sensor_main_check(terrain_reference(&state, 0.0, 0.0,
                           config.gravity_mps2, true), "flat reference");
    test_sensor_main_slope(&state, angle, 0.01, SENSOR_SLOPE_SAMPLES, true);
    test_sensor_main_check(test_sensor_main_near(state.output.hump_peak_m,
                           expected), "uphill chassis-path peak");
    test_sensor_main_slope(&state, -angle, 0.01, SENSOR_SLOPE_SAMPLES, true);
    test_sensor_main_slope(&state, 0.0, 0.002, SENSOR_FLAT_SAMPLES, true);
    test_sensor_main_check((1U == state.output.completed_humps)
                           && state.output.b_last_hump_valid,
                           "closed triangle commits one valid hump");
    test_sensor_main_check(test_sensor_main_near(state.output.run_maximum_m,
                           expected), "run max equals first valid peak");
    test_sensor_main_slope(&state, 0.1, 0.01, SENSOR_SLOPE_SAMPLES, true);
    test_sensor_main_slope(&state, -0.1, 0.01, SENSOR_SLOPE_SAMPLES, true);
    test_sensor_main_slope(&state, 0.0, 0.002, SENSOR_FLAT_SAMPLES, true);
    test_sensor_main_check((2U == state.output.completed_humps)
                           && (test_sensor_main_near(state.output.run_maximum_m,
                               expected)), "smaller hump preserves maximum");
    test_sensor_main_slope(&state, angle, 0.01, SENSOR_SLOPE_SAMPLES, true);
    test_sensor_main_slope(&state, -angle, 0.01, 1U, false);
    test_sensor_main_slope(&state, -angle, 0.01, SENSOR_SLOPE_SAMPLES, true);
    test_sensor_main_slope(&state, 0.0, 0.002, SENSOR_FLAT_SAMPLES, true);
    test_sensor_main_check((2U == state.output.completed_humps)
                           && (!state.output.b_last_hump_valid)
                           && (test_sensor_main_near(state.output.run_maximum_m,
                               expected)), "invalid hump preserves maximum");
}

/** Invalid timing and dynamic acceleration invalidate current estimates. */
static void test_sensor_main_invalid (void)
{
    terrain_config_t config;
    terrain_state_t state;
    terrain_output_t output;
    terrain_input_t input = {0.0, 0.0, 9.80665, 0.0, 0.0, 0.01,
                              true, true, true};

    terrain_default_config(&config);
    config.b_calibrated = true;
    test_sensor_main_check(terrain_init(&state, &config), "invalid-case init");
    test_sensor_main_check(!terrain_step(&state, &input, &output),
                           "reference required before processing");
    test_sensor_main_check(!terrain_reference(&state, 0.0, 0.0, 0.0, true),
                           "free-fall reference rejected");
    test_sensor_main_check(!terrain_reference(&state, 0.0, 0.0, 9.80665, false),
                           "moving reference rejected");
    test_sensor_main_check(terrain_reference(&state, 0.0, 0.0, 9.80665, true),
                           "valid stationary reference accepted");
    input.dt_s = 0.0;
    test_sensor_main_check(!terrain_step(&state, &input, &output)
                           && (!output.b_tilt_valid), "zero dt rejected");
    input.dt_s = 1.0;
    test_sensor_main_check(!terrain_step(&state, &input, &output),
                           "stale dt rejected");
    input.dt_s = 0.01;
    input.accel_x_mps2 = NAN;
    test_sensor_main_check(!terrain_step(&state, &input, &output),
                           "NaN acceleration rejected");
    input.accel_x_mps2 = 0.0;
    input.accel_z_mps2 = 30.0;
    test_sensor_main_check(terrain_step(&state, &input, &output)
                           && (!output.b_tilt_valid)
                           && (isnan(output.pitch_rad)),
                           "dynamic acceleration gives unavailable tilt");
}

/** Event labels distinguish encoder yaw and unverified collision candidates. */
static void test_sensor_main_motion (void)
{
    terrain_config_t config;
    terrain_state_t state;
    terrain_output_t output;
    terrain_input_t input = {0.0, 0.0, 9.80665, 0.004, 0.0, 0.01,
                              true, true, true};

    terrain_default_config(&config);
    config.b_calibrated = true;
    test_sensor_main_check(terrain_init(&state, &config), "motion init");
    test_sensor_main_check(terrain_reference(&state, 0.0, 0.0, 9.80665, true),
                           "motion reference");
    test_sensor_main_check(terrain_step(&state, &input, &output),
                           "initial speed observation");
    input.signed_distance_m = 0.0;
    input.accel_x_mps2 = -4.0;
    input.b_quasistatic = false;
    test_sensor_main_check(terrain_step(&state, &input, &output)
                           && output.b_collision_candidate
                           && (TERRAIN_COLLISION_CANDIDATE == output.motion)
                           && (!output.b_tilt_valid),
                           "combined deceleration gives impact candidate");
    input.accel_x_mps2 = 0.0;
    input.encoder_yaw_rate_rps = 0.5;
    input.b_quasistatic = true;
    test_sensor_main_check(terrain_step(&state, &input, &output)
                           && (TERRAIN_TURNING == output.motion)
                           && (test_sensor_main_near(
                               output.encoder_yaw_rate_rps,
                               0.5)), "turn rate is an encoder estimate");
    input.b_encoder_valid = false;
    test_sensor_main_check(terrain_step(&state, &input, &output)
                           && (TERRAIN_INVALID == output.motion)
                           && isnan(output.encoder_yaw_rate_rps),
                           "invalid encoder clears previously valid yaw");
}

/** A flat hump crest does not close the event before the descending ramp. */
static void test_sensor_main_plateau (void)
{
    terrain_config_t config;
    terrain_state_t state;
    const double angles[] = {0.1, 0.2};
    uint32_t scenario;
    double expected;

    for (scenario = 0U; scenario < 2U; ++scenario)
    {
        expected = 0.2 * sin(angles[scenario]);
        terrain_default_config(&config);
        config.b_calibrated = true;
        config.filter_time_s = 0.0;
        test_sensor_main_check(terrain_init(&state, &config), "plateau init");
        test_sensor_main_check(terrain_reference(&state, 0.0, 0.0,
                               config.gravity_mps2, true), "plateau reference");
        test_sensor_main_slope(&state, angles[scenario], 0.01,
                               SENSOR_SLOPE_SAMPLES, true);
        test_sensor_main_slope(&state, 0.0, 0.002, SENSOR_FLAT_SAMPLES, true);
        test_sensor_main_check(state.output.b_hump_active
                               && (0U == state.output.completed_humps)
                               && (!state.output.b_hump_completed),
                               "flat crest preserves incomplete uphill hump");
        test_sensor_main_slope(&state, -angles[scenario], 0.01,
                               SENSOR_SLOPE_SAMPLES, true);
        test_sensor_main_slope(&state, 0.0, 0.002, SENSOR_FLAT_SAMPLES, true);
        test_sensor_main_check((1U == state.output.completed_humps)
                               && state.output.b_last_hump_valid
                               && test_sensor_main_near(
                                   state.output.run_maximum_m, expected),
                               "flat-top hump closes after descending ramp");
    }
}

/** Feed a bounded fixture with gravity matching the declared mounting axes. */
static void test_sensor_main_slope (terrain_state_t * p_state,
                                    double angle_rad, double distance_m,
                                    uint32_t count, bool b_valid)
{
    terrain_input_t input;
    terrain_output_t output;
    uint32_t index = 0U;
    bool b_all_valid = true;

    input.accel_x_mps2 = p_state->config.gravity_mps2 * sin(angle_rad);
    input.accel_y_mps2 = 0.0;
    input.accel_z_mps2 = p_state->config.gravity_mps2 * cos(angle_rad);
    input.signed_distance_m = distance_m;
    input.encoder_yaw_rate_rps = 0.0;
    input.dt_s = 0.04;
    input.b_sensor_valid = b_valid;
    input.b_encoder_valid = true;
    input.b_quasistatic = true;
    for (index = 0U; index < count; index++)
    {
        b_all_valid = terrain_step(p_state, &input, &output) && b_all_valid;
    }
    test_sensor_main_check(b_all_valid, "synthetic slope fixture processed");
}

/** Return measured-pulse fixtures; zero means no echo, not far distance. */
static uint32_t test_sensor_main_echo (test_sensor_main_scene_t scene,
                                      int32_t angle_deg, bool b_fine)
{
    double distance_m = 1.5;

    if ((SENSOR_SCENE_UNKNOWN == scene)
        || ((SENSOR_SCENE_RECOVERY == scene) && (!b_fine)))
    {
        return 0U;
    }
    if (SENSOR_SCENE_OUT_OF_RANGE == scene)
    {
        return 30000U;
    }
    if ((SENSOR_SCENE_CENTER == scene)
        && (angle_deg >= -10) && (angle_deg <= 10))
    {
        distance_m = 0.3;
    }
    else if ((SENSOR_SCENE_LEFT_BLOCKED == scene) && (angle_deg >= 0))
    {
        distance_m = 0.3;
    }
    else
    {
        distance_m = 1.5;
    }
    /* Fixture is 0.3..1.5 m: rounded pulse is 1749..8746 us, fitting uint32. */
    return (uint32_t) ((distance_m * 2000000.0 / 343.0) + 0.5);
}

/** End of file. */

