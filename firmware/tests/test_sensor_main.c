/** @file test_sensor_main.c
 * @brief Deterministic acquisition timing, unknown-range and hump regressions.
 */
#include "test_sensor_main.h"

#include <float.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bitpack.h"
#include "scanner.h"
#include "terrain.h"

#define SENSOR_TEST_LIMIT (30000U)
#define SENSOR_SLOPE_SAMPLES (20U)
#define SENSOR_FLAT_SAMPLES (6U)
#define SENSOR_TOLERANCE (0.000001)
#define SENSOR_RANGE_CASES (5U)
/* Consumer literals: ROBOT_TILT_LIMIT_RAD (target/robot_config.h) and the
 * whole-mrad limit the packed comparisons use instead. */
#define SENSOR_TILT_LIMIT_RAD (0.60)
#define SENSOR_TILT_LIMIT_MRAD (600U)
#define SENSOR_TILT_ULPS (200000U)
#define SENSOR_TILT_SWEEP (50000U)
#define SENSOR_ANGLE_CASES (14U)
#define SENSOR_HEIGHT_CASES (11U)
#define SENSOR_YAW_CASES (15U)
#define SENSOR_HEIGHT_SWEEP (511000U)
/* Private scanner.c constants, copied for the legacy reference model. */
#define SENSOR_LEGACY_MIN_ANGLE_DEG (-60)
#define SENSOR_LEGACY_MAX_ANGLE_DEG (60)
#define SENSOR_LEGACY_COARSE_STEP_DEG (10)
#define SENSOR_LEGACY_FINE_STEP_DEG (2)
#define SENSOR_LEGACY_SIDE_ANGLE_DEG (20)
#define SENSOR_LEGACY_MAX_DELAY_MS (60000U)
#define SENSOR_LEGACY_MIN_INTERVAL_MS (65U)
#define SENSOR_RANDOM_SCENES (240U)
#define SENSOR_RANDOM_SCANS (3U)
#define SENSOR_PULSE_CASES (26U)
#define SENSOR_OUTCOMES_ALL (0xFFU)
/* Consumer literals: ROBOT_ROBOT_WIDTH_M, ROBOT_OBSTACLE_MARGIN_M,
 * ROBOT_MAX_OBSTACLE_DEPTH_M, ROBOT_ROBOT_LENGTH_M (target/robot_config.h)
 * and max_bypass_offset_m / max_bypass_length_m (target/app_cfg.c). */
#define SENSOR_ROBOT_WIDTH_M (0.18)
#define SENSOR_OBSTACLE_MARGIN_M (0.06)
#define SENSOR_MAX_OBSTACLE_DEPTH_M (0.30)
#define SENSOR_ROBOT_LENGTH_M (0.22)
#define SENSOR_MAX_BYPASS_OFFSET_M (0.80)
#define SENSOR_MAX_BYPASS_LENGTH_M (1.20)
#define SENSOR_PULSE_SWEEP (40000U)
#define SENSOR_WIDTH_SWEEP (1310000U)
#define SENSOR_GATE_ULPS (8U)
#define SENSOR_ULP_SLACK (1.0e-12)
/* test_sensor_main_bypass_compare outcomes. */
#define SENSOR_SAME (0U)
#define SENSOR_STRICTER (1U)
#define SENSOR_DIFFERENT (2U)
#define SENSOR_NEAREST_CASES (16U)
#define SENSOR_WIDTH_CASES (9U)
#define SENSOR_COUNT_CASES (6U)
#define SENSOR_ENTRY_CASES (14U)

typedef enum
{
    SENSOR_SCENE_CLEAR = 0,
    SENSOR_SCENE_CENTER,
    SENSOR_SCENE_LEFT_BLOCKED,
    SENSOR_SCENE_UNKNOWN,
    SENSOR_SCENE_OUT_OF_RANGE,
    SENSOR_SCENE_RECOVERY,
    SENSOR_SCENE_RANDOM
} test_sensor_main_scene_t;

/** Reference copy of the pre-change scanner state: the 80-entry list. */
typedef struct
{
    scanner_config_t config;
    scanner_sample_t samples[SCANNER_MAX_SAMPLES];
    scanner_result_t result;
    scanner_phase_t  phase;
    uint32_t         sample_count;
    uint32_t         phase_started_ms;
    uint32_t         last_trigger_ms;
    int32_t          angle_deg;
    int32_t          fine_end_deg;
    bool             b_initialized;
    bool             b_triggered;
    bool             b_fine;
} test_sensor_main_legacy_t;

/** Reference copy of the pre-change private profile accumulator. */
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
} test_sensor_main_profile_t;

/** One driven scan: echo source, abort time and pending echo. */
typedef struct
{
    test_sensor_main_scene_t scene;
    uint32_t seed;
    uint32_t start_ms;
    uint32_t abort_after_ms;
    uint32_t bad_in_16;
    uint32_t pulse_us;
    uint32_t due_ms;
    bool     b_pending;
    bool     b_deliver;
} test_sensor_main_plan_t;

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
static void test_sensor_main_output (terrain_output_t * p_output);
static void test_sensor_main_pack_angles (void);
static void test_sensor_main_pack_invalid (void);
static bool test_sensor_main_tilt_same (double pitch_rad, double roll_rad,
                                        bool b_valid);
static void test_sensor_main_tilt_equivalence (void);
static void test_sensor_main_pack_heights (void);
static int32_t test_sensor_main_milli (double value);
static void test_sensor_main_height_equivalence (void);
static void test_sensor_main_pack_yaw (void);
static void test_sensor_main_pack_classes (void);
static bool test_sensor_main_legacy_init (
    test_sensor_main_legacy_t * p_state, const scanner_config_t * p_config);
static bool test_sensor_main_legacy_start (
    test_sensor_main_legacy_t * p_state, uint32_t now_ms, bool b_stationary);
static bool test_sensor_main_legacy_step (
    test_sensor_main_legacy_t * p_state, uint32_t now_ms, bool b_stationary,
    const scanner_echo_t * p_echo, scanner_output_t * p_output);
static bool test_sensor_main_legacy_valid (
    const scanner_config_t * p_config);
static void test_sensor_main_legacy_reset (scanner_result_t * p_result);
static void test_sensor_main_legacy_capture (
    test_sensor_main_legacy_t * p_state, const scanner_echo_t * p_echo);
static bool test_sensor_main_legacy_advance (
    test_sensor_main_legacy_t * p_state);
static bool test_sensor_main_legacy_refine (
    test_sensor_main_legacy_t * p_state);
static bool test_sensor_main_legacy_latest (
    const test_sensor_main_legacy_t * p_state, uint32_t index);
static void test_sensor_main_legacy_profile (
    test_sensor_main_legacy_t * p_state);
static void test_sensor_main_legacy_accumulate (
    test_sensor_main_legacy_t * p_state,
    test_sensor_main_profile_t * p_profile,
    const scanner_sample_t * p_sample);
static void test_sensor_main_legacy_side (
    test_sensor_main_legacy_t * p_state,
    const test_sensor_main_profile_t * p_profile);
static uint32_t test_sensor_main_random (uint32_t * p_seed);
static void test_sensor_main_random_config (uint32_t * p_seed,
                                            scanner_config_t * p_config);
static void test_sensor_main_plan_echo (test_sensor_main_plan_t * p_plan,
                                        int32_t angle_deg, bool b_fine,
                                        uint32_t now_ms);
static bool test_sensor_main_same_real (double first, double second);
static bool test_sensor_main_same_result (const scanner_result_t * p_first,
                                          const scanner_result_t * p_second);
static bool test_sensor_main_same_step (
    const scanner_state_t * p_state,
    const test_sensor_main_legacy_t * p_legacy,
    const scanner_output_t * p_output, const scanner_output_t * p_reference);
static bool test_sensor_main_compare_scan (
    scanner_state_t * p_state, test_sensor_main_legacy_t * p_legacy,
    test_sensor_main_plan_t * p_plan);
static void test_sensor_main_equivalence (void);
static uint32_t test_sensor_main_outcome (
    const test_sensor_main_legacy_t * p_legacy);
static bool test_sensor_main_same_samples (
    const scanner_state_t * p_state,
    const test_sensor_main_legacy_t * p_legacy);
static bool test_sensor_main_bypass (const scanner_result_t * p_scan);
static bool test_sensor_main_same_shared (const scanner_result_t * p_legacy,
                                          const scanner_result_t * p_result);
static void test_sensor_main_bypass_base (scanner_result_t * p_result);
static uint32_t test_sensor_main_bypass_compare (
    const scanner_result_t * p_result);
static uint32_t test_sensor_main_capture_one (scanner_state_t * p_state,
                                              bool b_ready,
                                              uint32_t pulse_us);
static void test_sensor_main_sample_api (void);
static void test_sensor_main_entries (void);
static void test_sensor_main_saturation (void);
static void test_sensor_main_stale_table (void);
static void test_sensor_main_shared_nearest (void);
static void test_sensor_main_shared_width (void);
static void test_sensor_main_shared_fields (void);
static void test_sensor_main_bypass_equivalence (void);
static void test_sensor_main_hump_saturation (void);
static void test_sensor_main_corrupt_angle (void);

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
    test_sensor_main_pack_angles();
    test_sensor_main_pack_invalid();
    test_sensor_main_tilt_equivalence();
    test_sensor_main_pack_heights();
    test_sensor_main_height_equivalence();
    test_sensor_main_pack_yaw();
    test_sensor_main_pack_classes();
    test_sensor_main_equivalence();
    test_sensor_main_sample_api();
    test_sensor_main_entries();
    test_sensor_main_saturation();
    test_sensor_main_stale_table();
    test_sensor_main_shared_nearest();
    test_sensor_main_shared_width();
    test_sensor_main_shared_fields();
    test_sensor_main_bypass_equivalence();
    test_sensor_main_hump_saturation();
    test_sensor_main_corrupt_angle();
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
    /* Starts VALID so a sample that is never filled cannot pass. */
    scanner_sample_t sample = {0, {CORE_RANGE_VALID, 0.0}, false};
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
                           && scanner_sample(&state, -60, &sample)
                           && (CORE_RANGE_UNKNOWN == sample.range.state)
                           && (isnan(sample.range.distance_m)),
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
    scanner_sample_t sample = {0, {CORE_RANGE_UNKNOWN, 0.0}, false};
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
        sample.range.state = CORE_RANGE_UNKNOWN;
        test_sensor_main_check((1U == state.sample_count)
                               && scanner_sample(&state, -60, &sample)
                               && (expected[index] == sample.range.state),
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

/** A finite, valid output; each pack test changes only what it checks. */
static void test_sensor_main_output (terrain_output_t * p_output)
{
    memset(p_output, 0, sizeof(*p_output));
    p_output->motion       = TERRAIN_STEADY;
    p_output->b_tilt_valid = true;
}

/** Boundary set: AWAY rounding, symmetric saturation and both signs. */
static void test_sensor_main_pack_angles (void)
{
    const double angles[SENSOR_ANGLE_CASES] =
    {
        0.6, nextafter(0.6, INFINITY), nextafter(0.6, -INFINITY),
        -0.6, nextafter(-0.6, -INFINITY), nextafter(-0.6, INFINITY),
        2.5, -2.5, 0.0, 1.0e-9, -1.0e-9, 2.047, 2.0465, 2.048
    };
    const int32_t expected[SENSOR_ANGLE_CASES] =
    {
        600, 601, 600, -600, -601, -600, 2047, -2047, 0, 1, -1, 2047, 2047,
        2047
    };
    terrain_output_t output;
    terrain_shared_t shared;
    uint32_t index = 0U;
    uint32_t magnitude = 0U;

    for (index = 0U; index < SENSOR_ANGLE_CASES; index++)
    {
        test_sensor_main_output(&output);
        output.pitch_rad = angles[index];
        output.roll_rad  = -angles[index];
        terrain_shared_pack(&output, 0U, false, &shared);
        /* Expected codes are within +/-2047, so the magnitude is exact. */
        magnitude = (expected[index] < 0) ? (uint32_t)(-expected[index])
                                          : (uint32_t)expected[index];
        test_sensor_main_check(shared.b_tilt_valid
            && (expected[index]
                == bitpack_extend(shared.pitch_mrad, TERRAIN_ANGLE_BITS))
            && (-expected[index]
                == bitpack_extend(shared.roll_mrad, TERRAIN_ANGLE_BITS))
            && (magnitude == terrain_angle_mrad(shared.pitch_mrad))
            && (magnitude == terrain_angle_mrad(shared.roll_mrad)),
            "pitch/roll boundary code rounds the magnitude up");
    }
}

/** Invalid tilt never publishes an angle; null arguments are ignored. */
static void test_sensor_main_pack_invalid (void)
{
    const uint32_t reserved = bitpack_invalid(TERRAIN_ANGLE_BITS);
    terrain_output_t output;
    terrain_shared_t shared;
    terrain_shared_t before;

    test_sensor_main_output(&output);
    output.b_tilt_valid = false;
    output.pitch_rad    = NAN;
    output.roll_rad     = NAN;
    terrain_shared_pack(&output, 0U, false, &shared);
    test_sensor_main_check((!shared.b_tilt_valid)
                           && (reserved == shared.pitch_mrad)
                           && (reserved == shared.roll_mrad),
                           "invalid tilt stores the reserved angle code");
    test_sensor_main_check(TERRAIN_ANGLE_INVALID_MRAD
                           == terrain_angle_mrad(shared.roll_mrad),
                           "reserved angle reads beyond every limit");
    output.pitch_rad = 0.7;
    output.roll_rad  = 0.0;
    terrain_shared_pack(&output, 0U, false, &shared);
    test_sensor_main_check((!shared.b_tilt_valid)
                           && (reserved == shared.pitch_mrad),
                           "clear tilt flag never publishes a stale angle");
    test_sensor_main_output(&output);
    output.pitch_rad = NAN;
    terrain_shared_pack(&output, 0U, false, &shared);
    test_sensor_main_check((!shared.b_tilt_valid)
                           && (reserved == shared.pitch_mrad)
                           && (reserved == shared.roll_mrad),
                           "non-finite pitch clears the shared tilt flag");
    test_sensor_main_output(&output);
    output.roll_rad = INFINITY;
    terrain_shared_pack(&output, 0U, false, &shared);
    test_sensor_main_check((!shared.b_tilt_valid)
                           && (reserved == shared.pitch_mrad)
                           && (reserved == shared.roll_mrad),
                           "non-finite roll clears the shared tilt flag");
    before = shared;
    terrain_shared_pack(NULL, 0U, false, &shared);
    terrain_shared_pack(&output, 0U, false, NULL);
    test_sensor_main_check(0 == memcmp(&before, &shared, sizeof(shared)),
                           "null pack arguments are ignored");
}

/** Original Safety and ready predicates against the packed comparisons. */
static bool test_sensor_main_tilt_same (double pitch_rad, double roll_rad,
                                        bool b_valid)
{
    terrain_output_t output;
    terrain_shared_t shared;
    bool b_fault_old = false;
    bool b_fault_new = false;
    bool b_ready_old = false;
    bool b_ready_new = false;

    test_sensor_main_output(&output);
    output.pitch_rad    = pitch_rad;
    output.roll_rad     = roll_rad;
    output.b_tilt_valid = b_valid;
    terrain_shared_pack(&output, 0U, false, &shared);
    /* robot_app.c TILT fault and robot_app_ready, as written today. */
    b_fault_old = output.b_tilt_valid
        && ((fabs(output.pitch_rad) > SENSOR_TILT_LIMIT_RAD)
            || (fabs(output.roll_rad) > SENSOR_TILT_LIMIT_RAD));
    b_ready_old = output.b_tilt_valid
        && (fabs(output.pitch_rad) <= SENSOR_TILT_LIMIT_RAD)
        && (fabs(output.roll_rad) <= SENSOR_TILT_LIMIT_RAD);
    b_fault_new = shared.b_tilt_valid
        && ((terrain_angle_mrad(shared.pitch_mrad) > SENSOR_TILT_LIMIT_MRAD)
            || (terrain_angle_mrad(shared.roll_mrad)
                > SENSOR_TILT_LIMIT_MRAD));
    b_ready_new = shared.b_tilt_valid
        && (terrain_angle_mrad(shared.pitch_mrad) <= SENSOR_TILT_LIMIT_MRAD)
        && (terrain_angle_mrad(shared.roll_mrad) <= SENSOR_TILT_LIMIT_MRAD);
    return (b_fault_old == b_fault_new) && (b_ready_old == b_ready_new);
}

/** 200k ulps either side of +/-0.60 rad, a +/-2.5 rad sweep and every
 * neighbour pair give the original TILT and ready decisions. */
static void test_sensor_main_tilt_equivalence (void)
{
    const double edges[2] = {SENSOR_TILT_LIMIT_RAD, -SENSOR_TILT_LIMIT_RAD};
    const double near[6] =
    {
        nextafter(0.6, -INFINITY), 0.6, nextafter(0.6, INFINITY),
        nextafter(-0.6, INFINITY), -0.6, nextafter(-0.6, -INFINITY)
    };
    const double directions[2] = {INFINITY, -INFINITY};
    double angle = 0.0;
    uint32_t edge = 0U;
    uint32_t way = 0U;
    uint32_t index = 0U;
    uint32_t other = 0U;
    uint32_t mismatches = 0U;

    for (edge = 0U; edge < 4U; edge++)
    {
        angle = edges[edge / 2U];
        way   = edge % 2U;
        for (index = 0U; index < SENSOR_TILT_ULPS; index++)
        {
            mismatches += test_sensor_main_tilt_same(angle, 0.0, true)
                          ? 0U : 1U;
            mismatches += test_sensor_main_tilt_same(0.0, angle, true)
                          ? 0U : 1U;
            angle = nextafter(angle, directions[way]);
        }
    }
    for (index = 0U; index <= SENSOR_TILT_SWEEP; index++)
    {
        /* The index is exact in double; the sweep covers +/-2.5 rad. */
        angle = -2.5 + (((double)index) * 0.0001);
        mismatches += test_sensor_main_tilt_same(angle, -angle, true)
                      ? 0U : 1U;
        mismatches += test_sensor_main_tilt_same(angle, angle, false)
                      ? 0U : 1U;
    }
    for (index = 0U; index < 36U; index++)
    {
        other = index % 6U;
        mismatches += test_sensor_main_tilt_same(near[index / 6U],
                                                 near[other], true)
                      ? 0U : 1U;
    }
    test_sensor_main_check(0U == mismatches,
                           "packed tilt fault and ready decisions identical");
}

/** Heights FLOOR to whole mm in 0..511; non-finite and negative give 0. */
static void test_sensor_main_pack_heights (void)
{
    const double heights[SENSOR_HEIGHT_CASES] =
    {
        0.0, 0.005, 0.300, 0.5115, 0.6, NAN, -0.004, -INFINITY, INFINITY,
        0.0009999, 0.001
    };
    const uint32_t expected[SENSOR_HEIGHT_CASES] =
    {
        0U, 5U, 300U, 511U, 511U, 0U, 0U, 0U, 0U, 0U, 1U
    };
    const uint32_t missions[7] = {0U, 5U, 300U, 511U, 512U, 600U, UINT32_MAX};
    const uint32_t stored[7] = {0U, 5U, 300U, 511U, 511U, 511U, 511U};
    terrain_output_t output;
    terrain_shared_t shared;
    uint32_t index = 0U;

    for (index = 0U; index < SENSOR_HEIGHT_CASES; index++)
    {
        test_sensor_main_output(&output);
        output.run_maximum_m = heights[index];
        terrain_shared_pack(&output, 0U, false, &shared);
        test_sensor_main_check((expected[index]
                                == terrain_height_mm(heights[index]))
                               && (expected[index] == shared.run_maximum_mm),
                               "height floors to whole mm in 9 bits");
    }
    for (index = 0U; index < 7U; index++)
    {
        test_sensor_main_output(&output);
        terrain_shared_pack(&output, missions[index], (0U != index),
                            &shared);
        test_sensor_main_check((stored[index] == shared.mission_maximum_mm)
                               && ((0U != index) == shared.b_mission_valid),
                               "mission maximum saturates at 511 mm");
    }
}

/** robot_app_milli as written today (wire pk/mp before this change). */
static int32_t test_sensor_main_milli (double value)
{
    if (!isfinite(value))
    {
        return 0;
    }
    if (value > 1000.0)
    {
        return 1000000;
    }
    if (value < -1000.0)
    {
        return -1000000;
    }
    /* Finite result is within +/-1,000,000; truncation is intentional. */
    return (int32_t)(value * 1000.0);
}

/** FLOOR equals today's truncation for 0..0.511 m, and the integer mission
 * maximum equals the truncated fmax used today. */
static void test_sensor_main_height_equivalence (void)
{
    const double peaks[6] = {0.0, 0.005, 0.0055, 0.2 * sin(0.2), 0.2999999,
                             0.300};
    double height = 0.0;
    uint32_t index = 0U;
    uint32_t newer = 0U;
    uint32_t older = 0U;
    uint32_t mismatches = 0U;

    for (index = 0U; index <= SENSOR_HEIGHT_SWEEP; index++)
    {
        /* 0..0.511 m in 1 um steps; the index is exact in double. */
        height = ((double)index) / 1000000.0;
        /* Heights here floor to 0..511, so the int32_t view is exact. */
        mismatches += ((int32_t)terrain_height_mm(height)
                       == test_sensor_main_milli(height)) ? 0U : 1U;
    }
    for (index = 1U; index <= TERRAIN_HEIGHT_MAX_MM; index++)
    {
        height = ((double)index) / 1000.0;
        mismatches += ((int32_t)terrain_height_mm(nextafter(height, 0.0))
                       == test_sensor_main_milli(nextafter(height, 0.0)))
                      ? 0U : 1U;
        mismatches += ((int32_t)terrain_height_mm(nextafter(height, 1.0))
                       == test_sensor_main_milli(nextafter(height, 1.0)))
                      ? 0U : 1U;
    }
    for (index = 0U; index < 36U; index++)
    {
        newer = terrain_height_mm(peaks[index / 6U]);
        older = terrain_height_mm(peaks[index % 6U]);
        newer = (older > newer) ? older : newer;
        mismatches += ((int32_t)newer == test_sensor_main_milli(
                           fmax(peaks[index % 6U], peaks[index / 6U])))
                      ? 0U : 1U;
    }
    test_sensor_main_check(0U == mismatches,
                           "pk/mp heights equal today's truncation");
}

/** Yaw rate rounds to nearest mrad/s, saturates at +/-4095 and reserves
 * the most negative code for non-finite input. */
static void test_sensor_main_pack_yaw (void)
{
    const double rates[SENSOR_YAW_CASES] =
    {
        0.0, 0.25, -0.25, 2.1, -2.1, 0.0005, -0.0005, 0.0004999, 4.0954,
        4.0955, 5.0, -5.0, NAN, INFINITY, -INFINITY
    };
    const int32_t expected[SENSOR_YAW_CASES] =
    {
        0, 250, -250, 2100, -2100, 1, -1, 0, 4095, 4095, 4095, -4095,
        -4096, -4096, -4096
    };
    terrain_output_t output;
    terrain_shared_t shared;
    uint32_t index = 0U;

    for (index = 0U; index < SENSOR_YAW_CASES; index++)
    {
        test_sensor_main_output(&output);
        output.encoder_yaw_rate_rps = rates[index];
        terrain_shared_pack(&output, 0U, false, &shared);
        test_sensor_main_check(expected[index]
                               == bitpack_extend(shared.yaw_mrad_s,
                                                 TERRAIN_RATE_BITS),
                               "yaw rate code is nearest, saturating");
    }
    test_sensor_main_check(bitpack_invalid(TERRAIN_RATE_BITS)
                           == shared.yaw_mrad_s,
                           "non-finite yaw uses the reserved code");
}

/** Every motion class and each flag survive packing on their own. */
static void test_sensor_main_pack_classes (void)
{
    const terrain_motion_t classes[9] =
    {
        TERRAIN_INVALID, TERRAIN_STATIONARY, TERRAIN_STEADY,
        TERRAIN_ACCELERATING, TERRAIN_BRAKING, TERRAIN_TURNING,
        TERRAIN_CLIMBING, TERRAIN_DESCENDING, TERRAIN_COLLISION_CANDIDATE
    };
    terrain_output_t output;
    terrain_shared_t shared;
    uint32_t index = 0U;

    for (index = 0U; index < 9U; index++)
    {
        test_sensor_main_output(&output);
        output.motion = classes[index];
        terrain_shared_pack(&output, 0U, false, &shared);
        test_sensor_main_check(classes[index] == shared.motion,
                               "motion class packs in 4 bits");
    }
    test_sensor_main_output(&output);
    output.b_collision_candidate = true;
    terrain_shared_pack(&output, 0U, false, &shared);
    test_sensor_main_check(shared.b_collision_candidate
                           && (!shared.b_hump_active)
                           && (!shared.b_last_hump_valid)
                           && (!shared.b_mission_valid),
                           "collision flag packs alone");
    test_sensor_main_output(&output);
    output.b_hump_active = true;
    terrain_shared_pack(&output, 0U, false, &shared);
    test_sensor_main_check(shared.b_hump_active
                           && (!shared.b_collision_candidate)
                           && (!shared.b_last_hump_valid),
                           "hump-active flag packs alone");
    test_sensor_main_output(&output);
    output.b_last_hump_valid = true;
    terrain_shared_pack(&output, 0U, true, &shared);
    test_sensor_main_check(shared.b_last_hump_valid && shared.b_mission_valid
                           && (!shared.b_hump_active)
                           && (!shared.b_collision_candidate),
                           "last-hump and mission flags pack");
    test_sensor_main_check(8U == sizeof(terrain_shared_t),
                           "shared terrain record is 64 bits");
}

/* Legacy reference: the pre-change scanner.c list algorithm, copied
 * verbatim apart from names. Do not edit it to match the new code. */

/** Legacy scanner_init. */
static bool test_sensor_main_legacy_init (
    test_sensor_main_legacy_t * p_state, const scanner_config_t * p_config)
{
    scanner_config_t config_copy;

    if ((NULL == p_state) || (!test_sensor_main_legacy_valid(p_config)))
    {
        return false;
    }
    config_copy = *p_config;
    memset(p_state, 0, sizeof(*p_state));
    p_state->config        = config_copy;
    p_state->b_initialized = true;
    p_state->phase         = SCANNER_IDLE;
    test_sensor_main_legacy_reset(&p_state->result);
    return true;
}

/** Legacy scanner_start. */
static bool test_sensor_main_legacy_start (
    test_sensor_main_legacy_t * p_state, uint32_t now_ms, bool b_stationary)
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
    p_state->angle_deg        = SENSOR_LEGACY_MIN_ANGLE_DEG;
    p_state->fine_end_deg     = SENSOR_LEGACY_MAX_ANGLE_DEG;
    p_state->phase_started_ms = now_ms;
    p_state->b_fine           = false;
    p_state->phase            = SCANNER_POSITION;
    test_sensor_main_legacy_reset(&p_state->result);
    return true;
}

/** Legacy scanner_step. */
static bool test_sensor_main_legacy_step (
    test_sensor_main_legacy_t * p_state, uint32_t now_ms, bool b_stationary,
    const scanner_echo_t * p_echo, scanner_output_t * p_output)
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
        test_sensor_main_legacy_reset(&p_state->result);
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
                test_sensor_main_legacy_capture(p_state, p_echo);
                if (!test_sensor_main_legacy_advance(p_state))
                {
                    test_sensor_main_legacy_profile(p_state);
                    p_state->phase       = SCANNER_COMPLETE;
                    p_output->b_set_servo = true;
                    p_output->b_done      = true;
                }
            }
        break;
        default:
            test_sensor_main_legacy_reset(&p_state->result);
            p_state->phase            = SCANNER_ABORTED;
            p_state->result.b_aborted = true;
            p_output->b_set_servo     = true;
            p_output->b_done          = true;
            return false;
        break;
    }
    return true;
}

/** Legacy scanner_config_valid. */
static bool test_sensor_main_legacy_valid (
    const scanner_config_t * p_config)
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
           && (p_config->settle_ms <= SENSOR_LEGACY_MAX_DELAY_MS)
           && (p_config->trigger_interval_ms >= SENSOR_LEGACY_MIN_INTERVAL_MS)
           && (p_config->trigger_interval_ms <= SENSOR_LEGACY_MAX_DELAY_MS)
           && (p_config->echo_timeout_ms > 0U)
           && (p_config->echo_timeout_ms < p_config->trigger_interval_ms);
}

/** Legacy scanner_result_reset. */
static void test_sensor_main_legacy_reset (scanner_result_t * p_result)
{
    memset(p_result, 0, sizeof(*p_result));
    p_result->nearest_distance_m = NAN;
    p_result->apparent_width_m   = NAN;
    p_result->left_clearance_m   = NAN;
    p_result->right_clearance_m  = NAN;
}

/** Legacy scanner_capture. */
static void test_sensor_main_legacy_capture (
    test_sensor_main_legacy_t * p_state, const scanner_echo_t * p_echo)
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

/** Legacy scanner_advance. */
static bool test_sensor_main_legacy_advance (
    test_sensor_main_legacy_t * p_state)
{
    const int32_t step_deg = p_state->b_fine ? SENSOR_LEGACY_FINE_STEP_DEG
                                            : SENSOR_LEGACY_COARSE_STEP_DEG;
    const int32_t end_deg  = p_state->b_fine ? p_state->fine_end_deg
                                            : SENSOR_LEGACY_MAX_ANGLE_DEG;

    if ((p_state->angle_deg < end_deg)
        && (p_state->sample_count < SCANNER_MAX_SAMPLES))
    {
        p_state->angle_deg += step_deg;
        p_state->phase = SCANNER_POSITION;
        return true;
    }
    if ((!p_state->b_fine) && (test_sensor_main_legacy_refine(p_state)))
    {
        p_state->phase = SCANNER_POSITION;
        return true;
    }
    return false;
}

/** Legacy scanner_refine. */
static bool test_sensor_main_legacy_refine (
    test_sensor_main_legacy_t * p_state)
{
    int32_t first_deg = SENSOR_LEGACY_MAX_ANGLE_DEG;
    int32_t last_deg  = SENSOR_LEGACY_MIN_ANGLE_DEG;
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
        first_deg -= SENSOR_LEGACY_COARSE_STEP_DEG;
        last_deg  += SENSOR_LEGACY_COARSE_STEP_DEG;
        p_state->angle_deg = (first_deg < SENSOR_LEGACY_MIN_ANGLE_DEG)
                             ? SENSOR_LEGACY_MIN_ANGLE_DEG : first_deg;
        p_state->fine_end_deg = (last_deg > SENSOR_LEGACY_MAX_ANGLE_DEG)
                                ? SENSOR_LEGACY_MAX_ANGLE_DEG : last_deg;
        p_state->b_fine = true;
    }
    return b_found;
}

/** Legacy scanner_latest. */
static bool test_sensor_main_legacy_latest (
    const test_sensor_main_legacy_t * p_state, uint32_t index)
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

/** Legacy scanner_profile. */
static void test_sensor_main_legacy_profile (
    test_sensor_main_legacy_t * p_state)
{
    test_sensor_main_profile_t profile;
    uint32_t index = 0U;

    memset(&profile, 0, sizeof(profile));
    profile.minimum_y_m      = DBL_MAX;
    profile.maximum_y_m      = -DBL_MAX;
    profile.left_distance_m  = DBL_MAX;
    profile.right_distance_m = DBL_MAX;
    profile.b_left_valid     = true;
    profile.b_right_valid    = true;
    test_sensor_main_legacy_reset(&p_state->result);
    for (index = 0U; index < p_state->sample_count; index++)
    {
        if (test_sensor_main_legacy_latest(p_state, index))
        {
            test_sensor_main_legacy_accumulate(p_state, &profile,
                                               &p_state->samples[index]);
        }
    }
    p_state->result.b_profile_valid = (p_state->result.valid_samples > 0U)
        && (0U == p_state->result.unknown_samples);
    if (p_state->result.b_obstacle_seen)
    {
        p_state->result.apparent_width_m = profile.maximum_y_m
                                          - profile.minimum_y_m;
    }
    test_sensor_main_legacy_side(p_state, &profile);
}

/** Legacy scanner_accumulate. */
static void test_sensor_main_legacy_accumulate (
    test_sensor_main_legacy_t * p_state,
    test_sensor_main_profile_t * p_profile,
    const scanner_sample_t * p_sample)
{
    scanner_result_t * p_result = &p_state->result;
    const bool b_valid = (CORE_RANGE_VALID == p_sample->range.state);
    const bool b_left  = (p_sample->angle_deg >= SENSOR_LEGACY_SIDE_ANGLE_DEG);
    const bool b_right = (p_sample->angle_deg <= -SENSOR_LEGACY_SIDE_ANGLE_DEG);
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

/** Legacy scanner_choose_side. */
static void test_sensor_main_legacy_side (
    test_sensor_main_legacy_t * p_state,
    const test_sensor_main_profile_t * p_profile)
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

/* End of the legacy reference. */

/** Deterministic xorshift32; the seed must be nonzero. */
static uint32_t test_sensor_main_random (uint32_t * p_seed)
{
    uint32_t value = *p_seed;

    value ^= value << 13;
    value ^= value >> 17;
    value ^= value << 5;
    *p_seed = value;
    return value;
}

/** Valid configurations around the production one, with 0.25 m and exact
 * echo-distance obstacle gates and an exact-echo clearance tie. */
static void test_sensor_main_random_config (uint32_t * p_seed,
                                            scanner_config_t * p_config)
{
    const uint32_t draw = test_sensor_main_random(p_seed);

    scanner_default_config(p_config);
    p_config->b_calibrated         = true;
    p_config->settle_ms            = 1U + (draw % 200U);
    p_config->robot_half_width_m   = 0.09;
    p_config->uncertainty_margin_m = 0.06;
    switch ((draw >> 8) % 4U)
    {
        case 0U:
            p_config->obstacle_distance_m = 0.60;
        break;
        case 1U:
            p_config->obstacle_distance_m = 0.25;
        break;
        case 2U:
            /* The exact range of a 3498 us echo: a distance <= gate tie. */
            p_config->obstacle_distance_m = ((3498.0 / 1000000.0) / 2.0)
                                            * 343.0;
        break;
        default:
            /* A 8746 us echo leaves exactly the required clearance. */
            p_config->required_clearance_m = (((8746.0 / 1000000.0) / 2.0)
                                              * 343.0) - (0.09 + 0.06);
        break;
    }
}

/** Arm the echo for a new trigger: scene fixtures, or random in-range
 * pulses (incl. the 0.02/0.25/0.60/4.0 m edges) mixed with bad echoes
 * (0, below/above range, saturated, late or never answered). */
static void test_sensor_main_plan_echo (test_sensor_main_plan_t * p_plan,
                                        int32_t angle_deg, bool b_fine,
                                        uint32_t now_ms)
{
    static const uint32_t pulses[SENSOR_PULSE_CASES] =
    {
        117U, 1451U, 1452U, 1457U, 1458U, 1463U, 1464U, 3492U, 3493U,
        3498U, 3499U, 3504U, 3505U, 8746U, 23323U,
        0U, 1U, 116U, 23324U, 30000U, 32766U, 32767U, 32768U, 65535U,
        100000U, UINT32_MAX
    };
    uint32_t draw = 0U;

    p_plan->b_pending = true;
    if (SENSOR_SCENE_RANDOM != p_plan->scene)
    {
        p_plan->pulse_us  = test_sensor_main_echo(p_plan->scene, angle_deg,
                                                  b_fine);
        p_plan->due_ms    = now_ms + ((p_plan->pulse_us + 999U) / 1000U);
        p_plan->b_deliver = (p_plan->pulse_us > 0U);
        return;
    }
    draw = test_sensor_main_random(&p_plan->seed);
    if ((draw % 16U) < p_plan->bad_in_16)
    {
        /* Entries 15.. are not valid ranges; a quarter never answer. */
        p_plan->pulse_us  = pulses[15U + ((draw >> 4) % 11U)];
        p_plan->b_deliver = (0U != ((draw >> 8) % 4U));
        p_plan->due_ms    = now_ms + ((draw >> 12) % 36U);
        return;
    }
    /* Valid ranges: an edge, else 0.02..0.62 m or 0.02..4.0 m. */
    p_plan->pulse_us = (0U == ((draw >> 4) % 3U))
                       ? pulses[(draw >> 6) % 15U]
                       : (117U + (test_sensor_main_random(&p_plan->seed)
                                  % ((0U != ((draw >> 10) % 2U))
                                     ? 3500U : 23207U)));
    p_plan->b_deliver = true;
    p_plan->due_ms    = now_ms + ((p_plan->pulse_us + 999U) / 1000U);
}

/** Bit-identical doubles, or both NaN. */
static bool test_sensor_main_same_real (double first, double second)
{
    return (isnan(first) && isnan(second))
           || ((!isnan(first)) && (!isnan(second))
               && (0 == memcmp(&first, &second, sizeof(first))));
}

/** Every result field equal, distances bit for bit. */
static bool test_sensor_main_same_result (const scanner_result_t * p_first,
                                          const scanner_result_t * p_second)
{
    return test_sensor_main_same_real(p_first->nearest_distance_m,
                                      p_second->nearest_distance_m)
           && test_sensor_main_same_real(p_first->apparent_width_m,
                                         p_second->apparent_width_m)
           && test_sensor_main_same_real(p_first->left_clearance_m,
                                         p_second->left_clearance_m)
           && test_sensor_main_same_real(p_first->right_clearance_m,
                                         p_second->right_clearance_m)
           && (p_first->preferred_side == p_second->preferred_side)
           && (p_first->valid_samples == p_second->valid_samples)
           && (p_first->unknown_samples == p_second->unknown_samples)
           && (p_first->b_profile_valid == p_second->b_profile_valid)
           && (p_first->b_obstacle_seen == p_second->b_obstacle_seen)
           && (p_first->b_left_observed == p_second->b_left_observed)
           && (p_first->b_right_observed == p_second->b_right_observed)
           && (p_first->b_aborted == p_second->b_aborted);
}

/** Same outputs and the same schedule state after one step. */
static bool test_sensor_main_same_step (
    const scanner_state_t * p_state,
    const test_sensor_main_legacy_t * p_legacy,
    const scanner_output_t * p_output, const scanner_output_t * p_reference)
{
    return (p_output->b_set_servo == p_reference->b_set_servo)
           && (p_output->angle_deg == p_reference->angle_deg)
           && (p_output->b_trigger == p_reference->b_trigger)
           && (p_output->b_done == p_reference->b_done)
           && (p_state->phase == p_legacy->phase)
           && (p_state->sample_count == p_legacy->sample_count)
           && (p_state->angle_deg == p_legacy->angle_deg)
           && (p_state->fine_end_deg == p_legacy->fine_end_deg)
           && (p_state->b_fine == p_legacy->b_fine)
           && (p_state->b_triggered == p_legacy->b_triggered);
}

/** Drive the scanner and the legacy list with identical inputs; false at
 * the first different output, schedule state or final result. */
static bool test_sensor_main_compare_scan (
    scanner_state_t * p_state, test_sensor_main_legacy_t * p_legacy,
    test_sensor_main_plan_t * p_plan)
{
    scanner_output_t output;
    scanner_output_t reference;
    scanner_echo_t echo = {false, 0U};
    uint32_t elapsed = 0U;
    uint32_t now_ms = p_plan->start_ms;
    bool b_stationary = true;
    bool b_same = false;
    bool b_done = false;

    b_same = (scanner_start(p_state, now_ms, true)
              == test_sensor_main_legacy_start(p_legacy, now_ms, true));
    p_plan->b_pending = false;
    p_plan->pulse_us  = 0U;
    for (elapsed = 0U; b_same && (!b_done) && (elapsed < SENSOR_TEST_LIMIT);
         elapsed++)
    {
        /* Wraps modulo 2^32 exactly like the millisecond clock. */
        now_ms = p_plan->start_ms + elapsed;
        b_stationary = (elapsed < p_plan->abort_after_ms);
        echo.pulse_us = p_plan->pulse_us;
        echo.b_ready = p_plan->b_pending && p_plan->b_deliver
                       && ((now_ms - p_plan->due_ms) < UINT32_C(0x80000000));
        b_same = (scanner_step(p_state, now_ms, b_stationary, &echo, &output)
                  == test_sensor_main_legacy_step(p_legacy, now_ms,
                                                  b_stationary, &echo,
                                                  &reference))
                 && test_sensor_main_same_step(p_state, p_legacy, &output,
                                               &reference);
        if (echo.b_ready)
        {
            /* Each echo is delivered once; fixtures then clear the pulse. */
            p_plan->b_pending = false;
            p_plan->pulse_us  = (SENSOR_SCENE_RANDOM == p_plan->scene)
                                ? p_plan->pulse_us : 0U;
        }
        if (output.b_trigger)
        {
            test_sensor_main_plan_echo(p_plan, p_state->angle_deg,
                                       p_state->b_fine, now_ms);
        }
        b_done = output.b_done;
    }
    return b_same && b_done
           && test_sensor_main_same_result(&p_state->result,
                                           &p_legacy->result)
           && test_sensor_main_same_samples(p_state, p_legacy)
           && test_sensor_main_same_shared(&p_legacy->result,
                                           &p_state->result);
}

/** Every fixture scene and randomised scenes (repeated scans on one state,
 * clock wrap, aborts) give the legacy list's outputs and results. */
static void test_sensor_main_equivalence (void)
{
    const test_sensor_main_scene_t scenes[6] =
    {
        SENSOR_SCENE_CLEAR, SENSOR_SCENE_CENTER, SENSOR_SCENE_LEFT_BLOCKED,
        SENSOR_SCENE_UNKNOWN, SENSOR_SCENE_OUT_OF_RANGE, SENSOR_SCENE_RECOVERY
    };
    const uint32_t bad_rates[5] = {0U, 0U, 1U, 4U, 8U};
    scanner_config_t config;
    scanner_state_t state;
    test_sensor_main_legacy_t legacy;
    test_sensor_main_plan_t plan;
    uint32_t seed = 0x2545F491U;
    uint32_t index = 0U;
    uint32_t scan = 0U;
    uint32_t differences = 0U;
    uint32_t reached = 0U;

    memset(&plan, 0, sizeof(plan));
    scanner_default_config(&config);
    config.b_calibrated = true;
    for (index = 0U; index < 6U; index++)
    {
        plan.scene          = scenes[index];
        plan.abort_after_ms = UINT32_MAX;
        test_sensor_main_check(scanner_init(&state, &config)
                               && test_sensor_main_legacy_init(&legacy,
                                                               &config)
                               && test_sensor_main_compare_scan(&state,
                                   &legacy, &plan),
                               "fixture scene equals the legacy list");
    }
    for (index = 0U; index < SENSOR_RANDOM_SCENES; index++)
    {
        test_sensor_main_random_config(&seed, &config);
        differences += (scanner_init(&state, &config)
                        && test_sensor_main_legacy_init(&legacy, &config))
                       ? 0U : 1U;
        plan.scene    = SENSOR_SCENE_RANDOM;
        plan.seed     = test_sensor_main_random(&seed);
        plan.start_ms = (0U == (index % 4U)) ? (UINT32_MAX - 3000U)
                                             : test_sensor_main_random(&seed);
        for (scan = 0U; scan < SENSOR_RANDOM_SCANS; scan++)
        {
            plan.bad_in_16 = bad_rates[test_sensor_main_random(&seed) % 5U];
            plan.abort_after_ms = (0U == (test_sensor_main_random(&seed)
                                          % 6U))
                                  ? (test_sensor_main_random(&seed) % 9000U)
                                  : UINT32_MAX;
            differences += test_sensor_main_compare_scan(&state, &legacy,
                                                         &plan) ? 0U : 1U;
            reached |= test_sensor_main_outcome(&legacy);
            plan.start_ms += SENSOR_TEST_LIMIT;
        }
    }
    test_sensor_main_check(0U == differences,
                           "randomised scenes equal the legacy list");
    test_sensor_main_check(SENSOR_OUTCOMES_ALL == reached,
                           "randomised scenes reach every scan outcome");
}

/** Outcome bits of one finished legacy scan, for the coverage check. */
static uint32_t test_sensor_main_outcome (
    const test_sensor_main_legacy_t * p_legacy)
{
    const scanner_result_t * p_result = &p_legacy->result;
    uint32_t outcome = 0U;

    outcome |= p_result->b_aborted ? 0x01U : 0U;
    outcome |= (p_legacy->b_fine && (SCANNER_COMPLETE == p_legacy->phase))
               ? 0x02U : 0U;
    outcome |= p_result->b_profile_valid ? 0x04U : 0U;
    outcome |= (SCANNER_SIDE_LEFT == p_result->preferred_side) ? 0x08U : 0U;
    outcome |= (SCANNER_SIDE_RIGHT == p_result->preferred_side) ? 0x10U : 0U;
    outcome |= (p_result->b_profile_valid
                && (SCANNER_SIDE_NONE == p_result->preferred_side))
               ? 0x20U : 0U;
    outcome |= p_result->b_obstacle_seen ? 0x40U : 0U;
    outcome |= ((p_result->unknown_samples > 0U)
                && (SCANNER_COMPLETE == p_legacy->phase)) ? 0x80U : 0U;
    return outcome;
}

/** Per angle, scanner_sample equals the legacy list's newest sample there
 * (state, distance bit for bit, fine flag) and is false exactly where the
 * list has none. */
static bool test_sensor_main_same_samples (
    const scanner_state_t * p_state,
    const test_sensor_main_legacy_t * p_legacy)
{
    scanner_sample_t sample;
    const scanner_sample_t * p_newest = NULL;
    int32_t angle = 0;
    uint32_t index = 0U;
    bool b_found = false;
    bool b_same = true;

    for (angle = -60; angle <= 60; angle += 2)
    {
        p_newest = NULL;
        for (index = 0U; index < p_legacy->sample_count; index++)
        {
            if (angle == p_legacy->samples[index].angle_deg)
            {
                p_newest = &p_legacy->samples[index];
            }
        }
        b_found = scanner_sample(p_state, angle, &sample);
        b_same = b_same && (b_found == (NULL != p_newest));
        if (b_found && (NULL != p_newest))
        {
            b_same = b_same && (angle == sample.angle_deg)
                     && (p_newest->range.state == sample.range.state)
                     && test_sensor_main_same_real(p_newest->range.distance_m,
                                                   sample.range.distance_m)
                     && (p_newest->b_fine == sample.b_fine);
        }
    }
    return b_same;
}

/** robot_app_vehicle_input bypass inputs and vehicle_scan_valid as written
 * before this change (corridor confirmed), on one scan result. */
static bool test_sensor_main_bypass (const scanner_result_t * p_scan)
{
    double offset_m = 0.0;
    double length_m = 0.0;
    int32_t side = 0;
    bool b_clearance = false;

    offset_m = (p_scan->apparent_width_m / 2.0) +
               (SENSOR_ROBOT_WIDTH_M / 2.0) + SENSOR_OBSTACLE_MARGIN_M;
    length_m = p_scan->nearest_distance_m + SENSOR_MAX_OBSTACLE_DEPTH_M
               + SENSOR_ROBOT_LENGTH_M + SENSOR_OBSTACLE_MARGIN_M;
    side = (SCANNER_SIDE_LEFT == p_scan->preferred_side) ? 1 : -1;
    b_clearance = (p_scan->b_profile_valid) && (p_scan->b_obstacle_seen) &&
        (SCANNER_SIDE_NONE != p_scan->preferred_side) &&
        (0U == p_scan->unknown_samples);
    return (b_clearance) &&
           ((1 == side) || (-1 == side)) &&
           (isfinite(offset_m)) &&
           (offset_m > 0.0) &&
           (offset_m <= SENSOR_MAX_BYPASS_OFFSET_M) &&
           (isfinite(length_m)) &&
           (length_m > 0.0) &&
           (length_m <= SENSOR_MAX_BYPASS_LENGTH_M);
}

/** The packed record of the new result hands the consumers the legacy
 * result's side, counts, flags, distance validity and bypass decision. */
static bool test_sensor_main_same_shared (const scanner_result_t * p_legacy,
                                          const scanner_result_t * p_result)
{
    scanner_shared_t shared;
    scanner_result_t unpacked;

    memset(&shared, 0, sizeof(shared));
    scanner_shared_pack(p_result, &shared);
    scanner_shared_unpack(&shared, &unpacked);
    return (p_legacy->preferred_side == unpacked.preferred_side)
           && (p_legacy->valid_samples == unpacked.valid_samples)
           && (p_legacy->unknown_samples == unpacked.unknown_samples)
           && (p_legacy->b_profile_valid == unpacked.b_profile_valid)
           && (p_legacy->b_obstacle_seen == unpacked.b_obstacle_seen)
           && (p_legacy->b_left_observed == unpacked.b_left_observed)
           && (p_legacy->b_right_observed == unpacked.b_right_observed)
           && (p_legacy->b_aborted == unpacked.b_aborted)
           && ((!isnan(p_legacy->nearest_distance_m))
               == (!isnan(unpacked.nearest_distance_m)))
           && ((!isnan(p_legacy->apparent_width_m))
               == (!isnan(unpacked.apparent_width_m)))
           && isnan(unpacked.left_clearance_m)
           && isnan(unpacked.right_clearance_m)
           && (test_sensor_main_bypass(p_legacy)
               == test_sensor_main_bypass(&unpacked));
}

/** A bypass-eligible result: valid profile, an obstacle, LEFT preferred. */
static void test_sensor_main_bypass_base (scanner_result_t * p_result)
{
    memset(p_result, 0, sizeof(*p_result));
    p_result->nearest_distance_m = 0.3;
    p_result->apparent_width_m   = 0.2;
    p_result->left_clearance_m   = NAN;
    p_result->right_clearance_m  = NAN;
    p_result->preferred_side     = SCANNER_SIDE_LEFT;
    p_result->valid_samples      = 13U;
    p_result->b_profile_valid    = true;
    p_result->b_obstacle_seen    = true;
    p_result->b_left_observed    = true;
    p_result->b_right_observed   = true;
}

/** Original bypass decision on a result against the decision on its packed
 * copy, whose distances must be larger by less than 1 mm: SENSOR_SAME,
 * SENSOR_STRICTER (only the original accepts) or SENSOR_DIFFERENT. */
static uint32_t test_sensor_main_bypass_compare (
    const scanner_result_t * p_result)
{
    scanner_shared_t shared;
    scanner_result_t unpacked;
    double nearest_up = 0.0;
    double width_up = 0.0;
    bool b_original = false;
    bool b_packed = false;

    memset(&shared, 0, sizeof(shared));
    scanner_shared_pack(p_result, &shared);
    scanner_shared_unpack(&shared, &unpacked);
    nearest_up = unpacked.nearest_distance_m - p_result->nearest_distance_m;
    width_up   = unpacked.apparent_width_m - p_result->apparent_width_m;
    if ((nearest_up <= -SENSOR_ULP_SLACK)
        || (nearest_up >= (0.001 + SENSOR_ULP_SLACK))
        || (width_up <= -SENSOR_ULP_SLACK)
        || (width_up >= (0.001 + SENSOR_ULP_SLACK)))
    {
        return SENSOR_DIFFERENT;
    }
    b_original = test_sensor_main_bypass(p_result);
    b_packed   = test_sensor_main_bypass(&unpacked);
    if (b_original == b_packed)
    {
        return SENSOR_SAME;
    }
    return b_original ? SENSOR_STRICTER : SENSOR_DIFFERENT;
}

/** One capture at -60 degrees with the calibrated defaults; returns the
 * raw table entry of that beam. */
static uint32_t test_sensor_main_capture_one (scanner_state_t * p_state,
                                              bool b_ready,
                                              uint32_t pulse_us)
{
    scanner_config_t config;
    scanner_output_t output;
    scanner_echo_t echo = {false, 0U};
    bool b_valid = false;

    memset(p_state, 0, sizeof(*p_state));
    scanner_default_config(&config);
    config.b_calibrated = true;
    b_valid = scanner_init(p_state, &config)
              && scanner_start(p_state, 0U, true)
              && scanner_step(p_state, 0U, true, &echo, &output)
              && scanner_step(p_state, 150U, true, &echo, &output);
    echo.b_ready  = b_ready;
    echo.pulse_us = pulse_us;
    b_valid = scanner_step(p_state, 180U, true, &echo, &output) && b_valid;
    test_sensor_main_check(b_valid && (1U == p_state->sample_count),
                           "single capture serviced");
    return p_state->table[0];
}

/** scanner_sample: NULL, odd, out-of-sector and never-sampled angles are
 * false and leave the output alone; a sampled beam is returned. */
static void test_sensor_main_sample_api (void)
{
    const int32_t rejected[6] = {-59, 61, -62, 62, INT32_MIN, INT32_MAX};
    scanner_config_t config;
    scanner_state_t state;
    scanner_sample_t sample = {7, {CORE_RANGE_VALID, 1.0}, true};
    int32_t angle = 0;
    uint32_t index = 0U;
    uint32_t found = 0U;

    scanner_default_config(&config);
    config.b_calibrated = true;
    /* A zeroed static state, also after a refused init, has no beams. */
    memset(&state, 0, sizeof(state));
    config.range.sound_speed_m_per_s = 200.0;
    test_sensor_main_check(!scanner_init(&state, &config),
                           "slow sound speed refused");
    for (angle = -60; angle <= 60; angle += 2)
    {
        found += scanner_sample(&state, angle, &sample) ? 1U : 0U;
    }
    test_sensor_main_check((0U == found) && (7 == sample.angle_deg),
                           "uninitialised state has no beam");
    config.range.sound_speed_m_per_s = 343.0;
    test_sensor_main_check(scanner_init(&state, &config), "sample api init");
    for (angle = -60; angle <= 60; angle += 2)
    {
        found += scanner_sample(&state, angle, &sample) ? 1U : 0U;
    }
    test_sensor_main_check(0U == found, "no beam is sampled after init");
    (void)test_sensor_main_capture_one(&state, true, 1749U);
    for (index = 0U; index < 6U; index++)
    {
        found += scanner_sample(&state, rejected[index], &sample) ? 1U : 0U;
    }
    found += scanner_sample(&state, -58, &sample) ? 1U : 0U;
    found += scanner_sample(NULL, -60, &sample) ? 1U : 0U;
    test_sensor_main_check((0U == found) && (7 == sample.angle_deg)
                           && (CORE_RANGE_VALID == sample.range.state)
                           && test_sensor_main_same_real(
                               sample.range.distance_m, 1.0)
                           && sample.b_fine,
                           "rejected angles leave the sample unchanged");
    test_sensor_main_check(!scanner_sample(&state, -60, NULL),
                           "null sample output rejected");
    test_sensor_main_check(scanner_sample(&state, -60, &sample)
                           && (-60 == sample.angle_deg)
                           && (CORE_RANGE_VALID == sample.range.state)
                           && (!sample.b_fine)
                           && test_sensor_main_near(sample.range.distance_m,
                                  ((1749.0 / 1000000.0) / 2.0) * 343.0),
                           "sampled beam decodes its echo");
}

/** Entries keep pulses up to 32766 us and store 0 for no echo or a 0 us
 * echo; the decoded beam equals the list's capture of the original pulse,
 * also for saturated and timed-out pulses. */
static void test_sensor_main_entries (void)
{
    const uint32_t pulses[SENSOR_ENTRY_CASES] =
    {
        0U, 1U, 116U, 117U, 23323U, 23324U, 30000U, 32765U, 32766U, 32767U,
        32768U, 65535U, 100000U, UINT32_MAX
    };
    const uint32_t codes[SENSOR_ENTRY_CASES] =
    {
        0U, 1U, 116U, 117U, 23323U, 23324U, 30000U, 32765U, 32766U, 32766U,
        32766U, 32766U, 32766U, 32766U
    };
    scanner_state_t state;
    scanner_sample_t sample;
    core_range_sample_t reference;
    core_status_t status = CORE_OK;
    uint32_t index = 0U;
    uint32_t pulse = 0U;
    uint32_t entry = 0U;
    uint32_t code = 0U;
    bool b_ready = false;

    for (index = 0U; index < (2U * SENSOR_ENTRY_CASES); index++)
    {
        /* Second half: the same pulses after an echo timeout. */
        b_ready = (index < SENSOR_ENTRY_CASES);
        pulse   = pulses[index % SENSOR_ENTRY_CASES];
        code    = b_ready ? codes[index % SENSOR_ENTRY_CASES] : 0U;
        entry   = test_sensor_main_capture_one(&state, b_ready, pulse);
        /* The legacy capture arithmetic on the original pulse. */
        status = core_ultrasonic_range(&state.config.range, b_ready,
                                       ((double) pulse) / 1000000.0,
                                       &reference);
        if ((CORE_OK != status) && (CORE_OUT_OF_RANGE != status))
        {
            reference.state      = CORE_RANGE_UNKNOWN;
            reference.distance_m = NAN;
        }
        test_sensor_main_check((code == entry)
                               && scanner_sample(&state, -60, &sample)
                               && (reference.state == sample.range.state)
                               && test_sensor_main_same_real(
                                   reference.distance_m,
                                   sample.range.distance_m)
                               && (!sample.b_fine),
                               "table entry decodes as the list captured");
    }
}

/** Speeds that would leave the saturated 32766 us code inside the range are
 * refused, although the list accepted them; real speeds are accepted. */
static void test_sensor_main_saturation (void)
{
    const double speeds[4] = {200.0, 244.0, 245.0, 343.0};
    const bool accepted[4] = {false, false, true, true};
    scanner_config_t config;
    scanner_state_t state;
    test_sensor_main_legacy_t legacy;
    uint32_t index = 0U;

    for (index = 0U; index < 4U; index++)
    {
        scanner_default_config(&config);
        config.range.sound_speed_m_per_s = speeds[index];
        test_sensor_main_check(
            (accepted[index] == scanner_init(&state, &config))
            && test_sensor_main_legacy_init(&legacy, &config),
            "saturated pulse code must lie beyond the maximum range");
    }
    scanner_default_config(&config);
    config.range.sound_speed_m_per_s = 200.0;
    config.range.max_distance_m      = 3.0;
    test_sensor_main_check(scanner_init(&state, &config),
                           "a shorter maximum range admits a slower speed");
}

/** A coarse-only scan after a refined one on the same state keeps none of
 * the earlier fine beams; both scans equal the legacy list. */
static void test_sensor_main_stale_table (void)
{
    scanner_config_t config;
    scanner_state_t state;
    scanner_sample_t sample;
    test_sensor_main_legacy_t legacy;
    test_sensor_main_plan_t plan;

    memset(&plan, 0, sizeof(plan));
    scanner_default_config(&config);
    config.b_calibrated = true;
    plan.scene          = SENSOR_SCENE_CENTER;
    plan.abort_after_ms = UINT32_MAX;
    test_sensor_main_check(scanner_init(&state, &config)
                           && test_sensor_main_legacy_init(&legacy, &config)
                           && test_sensor_main_compare_scan(&state, &legacy,
                                                            &plan),
                           "refined scan equals the legacy list");
    test_sensor_main_check(scanner_sample(&state, 0, &sample)
                           && sample.b_fine
                           && (0U != (state.table[30] & SCANNER_ENTRY_FINE))
                           && scanner_sample(&state, -60, &sample)
                           && (!sample.b_fine),
                           "fine beams carry the fine-pass bit");
    plan.scene    = SENSOR_SCENE_CLEAR;
    plan.start_ms = SENSOR_TEST_LIMIT;
    test_sensor_main_check(test_sensor_main_compare_scan(&state, &legacy,
                                                         &plan)
                           && (13U == state.sample_count)
                           && (!scanner_sample(&state, -18, &sample))
                           && scanner_sample(&state, -20, &sample)
                           && (!sample.b_fine),
                           "a new scan forgets earlier fine beams");
}

/** Nearest: CEIL to whole mm with 4095 saturation; 4095 for a non-finite
 * distance flagged valid; 0 and NaN without a valid sample. */
static void test_sensor_main_shared_nearest (void)
{
    const double distances[SENSOR_NEAREST_CASES] =
    {
        0.02, 0.25, nextafter(0.25, INFINITY), nextafter(0.25, -INFINITY),
        0.6, nextafter(0.6, INFINITY), 0.62, nextafter(0.62, INFINITY),
        4.0, 4.095, 4.0951, 1000.0, NAN, INFINITY, -0.001, 0.0
    };
    const uint32_t expected[SENSOR_NEAREST_CASES] =
    {
        20U, 250U, 251U, 250U, 600U, 601U, 620U, 621U, 4000U, 4095U, 4095U,
        4095U, 4095U, 4095U, 0U, 0U
    };
    scanner_result_t result;
    scanner_result_t unpacked;
    scanner_shared_t shared;
    uint32_t index = 0U;

    for (index = 0U; index < SENSOR_NEAREST_CASES; index++)
    {
        test_sensor_main_bypass_base(&result);
        result.nearest_distance_m = distances[index];
        scanner_shared_pack(&result, &shared);
        scanner_shared_unpack(&shared, &unpacked);
        test_sensor_main_check((expected[index] == shared.nearest_mm)
            && test_sensor_main_same_real(unpacked.nearest_distance_m,
                                          ((double) expected[index])
                                          / 1000.0),
            "nearest packs to ceil mm and unpacks to metres");
    }
    test_sensor_main_bypass_base(&result);
    result.valid_samples = 0U;
    scanner_shared_pack(&result, &shared);
    scanner_shared_unpack(&shared, &unpacked);
    test_sensor_main_check((0U == shared.nearest_mm)
                           && isnan(unpacked.nearest_distance_m),
                           "nearest without a valid sample unpacks NaN");
}

/** Width: CEIL to whole mm with 4095 saturation; a zero width with an
 * obstacle stays 0.0 m; no obstacle stores 0 and unpacks NaN. */
static void test_sensor_main_shared_width (void)
{
    const double widths[SENSOR_WIDTH_CASES] =
    {
        0.0, 0.104, nextafter(0.104, INFINITY), 1.0392, 1.3,
        nextafter(1.3, INFINITY), 4.095, 5.0, NAN
    };
    const uint32_t expected[SENSOR_WIDTH_CASES] =
    {
        0U, 104U, 105U, 1040U, 1300U, 1301U, 4095U, 4095U, 4095U
    };
    scanner_result_t result;
    scanner_result_t unpacked;
    scanner_shared_t shared;
    uint32_t index = 0U;

    for (index = 0U; index < SENSOR_WIDTH_CASES; index++)
    {
        test_sensor_main_bypass_base(&result);
        result.apparent_width_m = widths[index];
        scanner_shared_pack(&result, &shared);
        scanner_shared_unpack(&shared, &unpacked);
        test_sensor_main_check((expected[index] == shared.width_mm)
            && test_sensor_main_same_real(unpacked.apparent_width_m,
                                          ((double) expected[index])
                                          / 1000.0),
            "width packs to ceil mm and unpacks to metres");
    }
    test_sensor_main_bypass_base(&result);
    result.b_obstacle_seen = false;
    scanner_shared_pack(&result, &shared);
    scanner_shared_unpack(&shared, &unpacked);
    test_sensor_main_check((0U == shared.width_mm)
                           && isnan(unpacked.apparent_width_m)
                           && isnan(unpacked.left_clearance_m)
                           && isnan(unpacked.right_clearance_m),
                           "width without an obstacle unpacks NaN");
}

/** Counts saturate at 63; sides and each flag round-trip on their own; the
 * record is 8 bytes; NULL arguments are ignored. */
static void test_sensor_main_shared_fields (void)
{
    const uint32_t counts[SENSOR_COUNT_CASES] =
    {
        0U, 1U, 61U, 63U, 64U, UINT32_MAX
    };
    const uint32_t stored[SENSOR_COUNT_CASES] = {0U, 1U, 61U, 63U, 63U, 63U};
    const scanner_side_t sides[3] =
    {
        SCANNER_SIDE_NONE, SCANNER_SIDE_LEFT, SCANNER_SIDE_RIGHT
    };
    scanner_result_t result;
    scanner_result_t unpacked;
    scanner_shared_t shared;
    uint32_t index = 0U;
    uint32_t other = 0U;
    uint32_t flags = 0U;

    for (index = 0U; index < SENSOR_COUNT_CASES; index++)
    {
        other = (index + 1U) % SENSOR_COUNT_CASES;
        test_sensor_main_bypass_base(&result);
        result.valid_samples   = counts[index];
        result.unknown_samples = counts[other];
        scanner_shared_pack(&result, &shared);
        scanner_shared_unpack(&shared, &unpacked);
        test_sensor_main_check((stored[index] == shared.valid_samples)
                               && (stored[other] == shared.unknown_samples)
                               && (stored[index] == unpacked.valid_samples)
                               && (stored[other] == unpacked.unknown_samples),
                               "sample counts saturate at 63 in 6 bits");
    }
    for (index = 0U; index < 3U; index++)
    {
        test_sensor_main_bypass_base(&result);
        result.preferred_side = sides[index];
        scanner_shared_pack(&result, &shared);
        scanner_shared_unpack(&shared, &unpacked);
        test_sensor_main_check((sides[index] == shared.preferred_side)
                               && (sides[index] == unpacked.preferred_side),
                               "preferred side round-trips in 2 bits");
    }
    for (index = 0U; index < 5U; index++)
    {
        memset(&result, 0, sizeof(result));
        result.b_profile_valid  = (0U == index);
        result.b_obstacle_seen  = (1U == index);
        result.b_left_observed  = (2U == index);
        result.b_right_observed = (3U == index);
        result.b_aborted        = (4U == index);
        scanner_shared_pack(&result, &shared);
        scanner_shared_unpack(&shared, &unpacked);
        flags = (shared.b_profile_valid ? 0x01U : 0U)
                | (shared.b_obstacle_seen ? 0x02U : 0U)
                | (shared.b_left_observed ? 0x04U : 0U)
                | (shared.b_right_observed ? 0x08U : 0U)
                | (shared.b_aborted ? 0x10U : 0U)
                | (unpacked.b_profile_valid ? 0x20U : 0U)
                | (unpacked.b_obstacle_seen ? 0x40U : 0U)
                | (unpacked.b_left_observed ? 0x80U : 0U)
                | (unpacked.b_right_observed ? 0x100U : 0U)
                | (unpacked.b_aborted ? 0x200U : 0U);
        test_sensor_main_check((0x21U << index) == flags,
                               "each scan flag packs alone");
    }
    test_sensor_main_check(8U == sizeof(scanner_shared_t),
                           "shared scan record is 8 bytes");
    test_sensor_main_bypass_base(&result);
    scanner_shared_pack(&result, &shared);
    scanner_shared_pack(NULL, &shared);
    scanner_shared_pack(&result, NULL);
    unpacked.valid_samples = 99U;
    scanner_shared_unpack(NULL, &unpacked);
    scanner_shared_unpack(&shared, NULL);
    test_sensor_main_check((300U == shared.nearest_mm)
                           && (200U == shared.width_mm)
                           && (13U == shared.valid_samples)
                           && (99U == unpacked.valid_samples),
                           "null shared-record arguments are ignored");
}

/** Length gate (nearest + 0.58 <= 1.20 m): identical for every pulse and
 * for SENSOR_GATE_ULPS doubles either side of 0.62 m. Offset gate
 * (width / 2 + 0.15 <= 0.80 m): identical on a 1 um grid to 1.31 m; around
 * 1.30 m only widths within 1 pm above it may differ, and only by the
 * packed path rejecting (the original sum rounds down to 0.80 there). */
static void test_sensor_main_bypass_equivalence (void)
{
    const double gates[2] = {0.62, 1.30};
    scanner_config_t config;
    scanner_result_t result;
    core_range_sample_t range;
    double value = 0.0;
    uint32_t index = 0U;
    uint32_t gate = 0U;
    uint32_t valid = 0U;
    uint32_t outcome = 0U;
    uint32_t mismatches = 0U;
    uint32_t stricter = 0U;

    scanner_default_config(&config);
    test_sensor_main_bypass_base(&result);
    for (index = 1U; index <= SENSOR_PULSE_SWEEP; index++)
    {
        /* The scanner's capture arithmetic for one completed echo. */
        (void)core_ultrasonic_range(&config.range, true,
                                    ((double) index) / 1000000.0, &range);
        if (CORE_RANGE_VALID == range.state)
        {
            valid++;
            result.nearest_distance_m = range.distance_m;
            mismatches += (SENSOR_SAME
                           == test_sensor_main_bypass_compare(&result))
                          ? 0U : 1U;
        }
    }
    test_sensor_main_bypass_base(&result);
    for (index = 0U; index <= SENSOR_WIDTH_SWEEP; index++)
    {
        /* The index is exact in double: 0..1.31 m in 1 um steps. */
        result.apparent_width_m = ((double) index) / 1000000.0;
        mismatches += (SENSOR_SAME == test_sensor_main_bypass_compare(&result))
                      ? 0U : 1U;
    }
    for (gate = 0U; gate < 2U; gate++)
    {
        value = gates[gate];
        for (index = 0U; index < SENSOR_GATE_ULPS; index++)
        {
            value = nextafter(value, -INFINITY);
        }
        for (index = 0U; index <= (2U * SENSOR_GATE_ULPS); index++)
        {
            test_sensor_main_bypass_base(&result);
            result.nearest_distance_m = (0U == gate) ? value : 0.3;
            result.apparent_width_m   = (0U == gate) ? 0.2 : value;
            outcome = test_sensor_main_bypass_compare(&result);
            if ((SENSOR_STRICTER == outcome) && (1U == gate)
                && (value > 1.30) && (value <= (1.30 + SENSOR_ULP_SLACK)))
            {
                stricter++;
            }
            else
            {
                mismatches += (SENSOR_SAME == outcome) ? 0U : 1U;
            }
            value = nextafter(value, INFINITY);
        }
    }
    test_sensor_main_check((23207U == valid) && (0U == mismatches),
                           "bypass length gate identical, offset gate "
                           "identical outside 1 pm above 1.30 m");
    test_sensor_main_check(1U == stricter,
                           "offset gate differs at one double above "
                           "1.30 m, only by rejecting");
}

/** The 8-bit hump counter saturates at 255 instead of wrapping to 0, so
 * the IMU task's (0 != completed_humps) mission flag stays set. */
static void test_sensor_main_hump_saturation (void)
{
    terrain_config_t config;
    terrain_state_t state;
    uint32_t hump = 0U;

    terrain_default_config(&config);
    config.b_calibrated  = true;
    config.filter_time_s = 0.0;
    test_sensor_main_check(terrain_init(&state, &config)
                           && terrain_reference(&state, 0.0, 0.0,
                                                config.gravity_mps2, true),
                           "hump counter init");
    /* 254 fits uint8_t exactly. */
    state.output.completed_humps = (uint8_t) (UINT8_MAX - 1U);
    for (hump = 0U; hump < 2U; hump++)
    {
        test_sensor_main_slope(&state, 0.2, 0.01, SENSOR_SLOPE_SAMPLES, true);
        test_sensor_main_slope(&state, -0.2, 0.01, SENSOR_SLOPE_SAMPLES,
                               true);
        test_sensor_main_slope(&state, 0.0, 0.002, SENSOR_FLAT_SAMPLES,
                               true);
        test_sensor_main_check((UINT8_MAX == state.output.completed_humps)
                               && state.output.b_last_hump_valid,
                               "completed humps saturate at 255");
    }
}

/** A corrupted beam angle (never produced) captures nothing instead of
 * writing outside the table; ASan would report such a write. */
static void test_sensor_main_corrupt_angle (void)
{
    const int8_t angles[3] = {-100, 62, INT8_MAX};
    scanner_config_t config;
    scanner_state_t state;
    scanner_output_t output;
    scanner_echo_t echo = {true, 1749U};
    uint32_t index = 0U;
    uint32_t entry = 0U;
    uint32_t untouched = 0U;
    bool b_valid = true;

    scanner_default_config(&config);
    config.b_calibrated = true;
    for (index = 0U; index < 3U; index++)
    {
        b_valid = scanner_init(&state, &config)
                  && scanner_start(&state, 0U, true)
                  && scanner_step(&state, 0U, true, &echo, &output)
                  && scanner_step(&state, 150U, true, &echo, &output)
                  && b_valid;
        state.angle_deg = angles[index];
        b_valid = scanner_step(&state, 151U, true, &echo, &output)
                  && (0U == state.sample_count) && b_valid;
        for (entry = 0U; entry < SCANNER_TABLE_SIZE; entry++)
        {
            untouched += (SCANNER_ENTRY_NOT_SAMPLED == state.table[entry])
                         ? 1U : 0U;
        }
    }
    test_sensor_main_check(b_valid && ((3U * SCANNER_TABLE_SIZE) == untouched),
                           "corrupted beam angle never writes the table");
}

/** End of file. */

