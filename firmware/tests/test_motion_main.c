/** @file test_motion_main.c
 * @brief Motion leases, primitives, inhibition and supervisor scenarios.
 * Synthetic encoder/event fixtures do not establish physical accuracy.
 */
#include "test_motion_main.h"

#include "motion.h"
#include "vehicle.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_DISTANCE_STEPS (5U)
#define TEST_TURN_STEPS (63U)
#define TEST_TIMEOUT_STEPS (10U)
#define TEST_BYPASS_STEPS (7U)
#define TEST_INTEGRATION_LIMIT (2000U)
/* Junction geometry: 18 mm line, sensors 60 mm ahead of the axle (the
 * ROBOT_SENSOR_AXLE_OFFSET_M hypothesis that app_cfg.c configures as
 * junction_distance_m), and an edge once the sensors are 2 mm inside.
 * Production cases instead start at the near edge (0 mm inside) and add
 * the travel during the production line_ctrl junction debounce (20 ms,
 * checked in test_line_main.c) at ROBOT_LINE_SPEED_MPS (0.08 m/s): 1.6 mm
 * (NI-MOT-1). The plant moves 2 mm per 10 ms sample, coarser than the
 * 0.4 mm entry difference, so the geometry test also evaluates the pivot
 * from the exact edge crossing (test_vehicle_junction_geometry).
 */
#define TEST_LINE_WIDTH_M (0.018)
#define TEST_SENSOR_AXLE_M (0.060)
#define TEST_CROSSBAR_M (0.500)
#define TEST_EDGE_ENTRY_M (0.002)
#define TEST_PLANT_STEP_M (0.002)
#define TEST_LINE_SPEED_MPS (0.08)
#define TEST_JUNCTION_DEBOUNCE_S (0.020)

static unsigned int g_checks = 0U;
static unsigned int g_failures = 0U;

/* BARR-TEST-01: single evaluation plus expression/line diagnostics. */
#define CHECK(b_condition) \
    (check_result((b_condition), #b_condition, __LINE__))

static void check_result (bool b_condition, const char * p_expression,
                          int line);
static bool near_value (core_real_t actual, core_real_t expected);
static motion_config_t motion_fixture (void);
static vehicle_config_t vehicle_fixture (void);
static vehicle_input_t vehicle_inputs (void);
static void start_vehicle (vehicle_t * p_vehicle,
                           vehicle_input_t * p_input,
                           vehicle_output_t * p_output);
static void test_motion_stop (void);
static void test_motion_lease (void);
static void test_motion_distance (void);
static void test_motion_turn (void);
static void test_motion_timeout (void);
static void test_motion_invalid (void);
static void test_motion_speed_window (void);
static void test_motion_speed_reset (void);
static void test_motion_speed_intervals (void);
static void test_vehicle_start_stop (void);
static void test_vehicle_bypass (int32_t side, vehicle_nav_t navigation);
static void test_vehicle_scan_faults (void);
static void test_vehicle_navigation (vehicle_nav_t navigation,
                                     core_real_t expected_angle);
static void test_vehicle_recovery (void);
static void test_vehicle_turn_guard (void);
static void test_interrupted_navigation (void);
static void test_motion_command_boundary (void);
static void test_motion_completion_renewal (void);
static void test_vehicle_line_wait (void);
static void test_vehicle_line_loss_precedence (void);
static void test_vehicle_range_recovery (void);
static void test_vehicle_range_bounds (void);
static void test_vehicle_range_guards (void);
static void test_vehicle_range_primitives (void);
static void test_vehicle_junction_geometry (vehicle_nav_t navigation,
                                            core_real_t expected_angle,
                                            core_real_t edge_travel_m,
                                            core_real_t approach_m,
                                            core_real_t tolerance_m);
static void test_junction_production (void);
static void test_integrated_sequence (void);
static bool integrated_motion_tick (motion_t * p_motion,
                                    const vehicle_output_t * p_output,
                                    uint32_t now_ms);

/** @brief Return nonzero on any persistent release-mode test failure. */
int main (void)
{
    test_motion_stop();
    test_motion_lease();
    test_motion_distance();
    test_motion_turn();
    test_motion_timeout();
    test_motion_invalid();
    test_motion_speed_window();
    test_motion_speed_reset();
    test_motion_speed_intervals();
    test_vehicle_start_stop();
    test_vehicle_bypass(1, VEHICLE_NAV_NONE);
    test_vehicle_bypass(-1, VEHICLE_NAV_NONE);
    test_vehicle_bypass(1, VEHICLE_NAV_LEFT);
    test_vehicle_bypass(-1, VEHICLE_NAV_RIGHT);
    test_vehicle_scan_faults();
    test_vehicle_navigation(VEHICLE_NAV_LEFT, 0.5 * CORE_PI);
    test_vehicle_navigation(VEHICLE_NAV_RIGHT, -0.5 * CORE_PI);
    test_vehicle_navigation(VEHICLE_NAV_UTURN, CORE_PI);
    test_vehicle_navigation(VEHICLE_NAV_STRAIGHT, 0.0);
    test_vehicle_recovery();
    test_vehicle_turn_guard();
    test_interrupted_navigation();
    test_motion_command_boundary();
    test_motion_completion_renewal();
    test_vehicle_line_wait();
    test_vehicle_line_loss_precedence();
    test_vehicle_range_recovery();
    test_vehicle_range_bounds();
    test_vehicle_range_guards();
    test_vehicle_range_primitives();
    test_vehicle_junction_geometry(VEHICLE_NAV_LEFT, 0.5 * CORE_PI,
        TEST_EDGE_ENTRY_M, TEST_SENSOR_AXLE_M, 0.5 * TEST_LINE_WIDTH_M);
    test_vehicle_junction_geometry(VEHICLE_NAV_RIGHT, -0.5 * CORE_PI,
        TEST_EDGE_ENTRY_M, TEST_SENSOR_AXLE_M, 0.5 * TEST_LINE_WIDTH_M);
    test_vehicle_junction_geometry(VEHICLE_NAV_UTURN, CORE_PI,
        TEST_EDGE_ENTRY_M, TEST_SENSOR_AXLE_M, 0.5 * TEST_LINE_WIDTH_M);
    test_junction_production();
    test_integrated_sequence();
    if (0 > printf("Checks: %u; failures: %u\n", g_checks, g_failures))
    {
        return EXIT_FAILURE;
    }
    if (0U != g_failures)
    {
        return EXIT_FAILURE;
    }
    if (EOF == puts("PASS: motion/supervisor synthetic host scenarios."))
    {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

/** @brief Count checks even with NDEBUG and retain source diagnostics. */
static void check_result (bool b_condition, const char * p_expression,
                          int line)
{
    g_checks += 1U;
    if (!b_condition)
    {
        g_failures += 1U;
        /* Discard diagnostic I/O result: failure is already counted. */
        (void) fprintf(stderr, "FAIL line %d: %s\n", line, p_expression);
    }
}

/** @brief Compare numerical fixtures without floating equality operators. */
static bool near_value (core_real_t actual, core_real_t expected)
{
    return (isfinite(actual)) && (fabs(actual - expected) < 1.0e-10);
}

/** @brief One count is exactly 1 mm in this synthetic geometry. */
static motion_config_t motion_fixture (void)
{
    motion_config_t config;

    memset(&config, 0, sizeof(config));
    config.odometry.left_radius_m = 1.0 / (2.0 * CORE_PI);
    config.odometry.right_radius_m = 1.0 / (2.0 * CORE_PI);
    config.odometry.track_width_m = 0.2;
    config.odometry.left_counts_per_revolution = 1000U;
    config.odometry.right_counts_per_revolution = 1000U;
    config.wheel_pid.proportional_gain = 10.0;
    config.wheel_pid.integral_gain = 0.2;
    config.wheel_pid.derivative_gain = 0.0;
    config.wheel_pid.output_min = -1.0;
    config.wheel_pid.output_max = 1.0;
    config.wheel_pid.max_dt_s = 0.1;
    config.max_wheel_mps = 1.0;
    config.max_yaw_rps = 3.0;
    config.distance_tolerance_m = 0.001;
    config.turn_tolerance_rad = 0.01;
    config.stationary_mps = 0.01;
    config.primitive_timeout_ms = 2000U;
    config.b_calibration_confirmed = true;
    return config;
}

/** @brief Deliberately small, bounded synthetic course configuration. */
static vehicle_config_t vehicle_fixture (void)
{
    vehicle_config_t config;

    memset(&config, 0, sizeof(config));
    config.forward_mps = 0.2;
    config.turn_rps = 1.0;
    config.junction_distance_m = 0.15;
    config.reacquire_distance_m = 0.2;
    config.max_bypass_offset_m = 0.5;
    config.max_bypass_length_m = 1.0;
    config.scan_timeout_ms = 1000U;
    config.reacquire_timeout_ms = 2000U;
    config.range_max_age_ms = 150U;
    config.range_wait_timeout_ms = 1000U;
    config.pending_nav_timeout_ms = 10000U;
    config.b_calibration_confirmed = true;
    config.b_course_confirmed = true;
    return config;
}

/** @brief Fresh valid sensors with no autonomous start event. */
static vehicle_input_t vehicle_inputs (void)
{
    vehicle_input_t input;

    memset(&input, 0, sizeof(input));
    input.motion.state = MOTION_IDLE;
    input.motion.b_stationary = true;
    input.b_fault_clear = true;
    input.b_sensors_ready = true;
    input.b_line_valid = true;
    input.b_range_valid = true;
    input.b_front_clear = true;
    input.line_forward_mps = 0.2;
    return input;
}

/** @brief Exercise INIT, IDLE, explicit START and the first FOLLOW request. */
static void start_vehicle (vehicle_t * p_vehicle,
                           vehicle_input_t * p_input,
                           vehicle_output_t * p_output)
{
    vehicle_config_t config = vehicle_fixture();

    CHECK(vehicle_init(p_vehicle, &config));
    CHECK(vehicle_step(p_vehicle, 0U, p_input, p_output));
    CHECK(VEHICLE_IDLE == p_output->state);
    p_input->b_start = true;
    CHECK(vehicle_step(p_vehicle, 10U, p_input, p_output));
    CHECK(VEHICLE_FOLLOW == p_output->state);
    p_input->b_start = false;
    CHECK(vehicle_step(p_vehicle, 20U, p_input, p_output));
    CHECK(p_output->b_issue_motion);
    CHECK(MOTION_VELOCITY == p_output->request.kind);
}

/** @brief STOP preempts drive, survives HALT, and requires stationary reset. */
static void test_motion_stop (void)
{
    motion_t motion;
    motion_config_t config = motion_fixture();
    motion_request_t request = {1U, MOTION_VELOCITY, 0.2, 0.0, 0.0};
    motion_request_t halt = {0U, MOTION_HALT, 0.0, 0.0, 0.0};
    motion_output_t output;

    CHECK(MOTION_OK == motion_init(&motion, &config));
    CHECK(MOTION_OK == motion_submit(&motion, &request, 0U));
    CHECK(MOTION_OK == motion_step(&motion, 10U, 0.01, 0, 0, &output));
    CHECK(near_value(output.left_duty, 1.0));
    CHECK(near_value(output.right_duty, 1.0));
    motion_stop(&motion, MOTION_STOPPED);
    CHECK(near_value(motion.output.left_duty, 0.0));
    CHECK(MOTION_INHIBITED == motion.output.state);
    CHECK(MOTION_REJECTED == motion_submit(&motion, &request, 11U));
    CHECK(MOTION_OK == motion_submit(&motion, &halt, 11U));
    CHECK(MOTION_INHIBITED == motion.output.state);
    CHECK(MOTION_REJECTED == motion_reset(&motion, false, true));
    CHECK(MOTION_REJECTED == motion_reset(&motion, true, false));
    CHECK(MOTION_STOPPED ==
          motion_step(&motion, 20U, 0.01, 1, 1, &output));
    CHECK(MOTION_REJECTED == motion_reset(&motion, true, true));
    CHECK(MOTION_STOPPED ==
          motion_step(&motion, 30U, 0.01, 0, 0, &output));
    CHECK(MOTION_OK == motion_reset(&motion, true, true));
    CHECK(MOTION_IDLE == motion.output.state);
    CHECK(MOTION_REJECTED == motion_submit(&motion, &request, 31U));
    request.command_id = 2U;
    CHECK(MOTION_OK == motion_submit(&motion, &request, 31U));
    CHECK(MOTION_OK == motion_submit(&motion, &halt, 32U));
    CHECK(MOTION_IDLE == motion.output.state);
    CHECK(!motion.b_stop_latched);
}

/** @brief Expiration works across clock wrap and cannot be revived by submit.
 */
static void test_motion_lease (void)
{
    motion_t motion;
    motion_config_t config = motion_fixture();
    motion_request_t request = {UINT32_MAX, MOTION_VELOCITY, 0.2, 0.0, 0.0};
    motion_output_t output;
    uint32_t started = UINT32_MAX - 20U;

    CHECK(MOTION_OK == motion_init(&motion, &config));
    CHECK(MOTION_OK == motion_submit(&motion, &request, started));
    CHECK(MOTION_OK ==
          motion_step(&motion, started + 49U, 0.01, 0, 0, &output));
    CHECK(MOTION_EXPIRED ==
          motion_step(&motion, started + 50U, 0.01, 0, 0, &output));
    CHECK(near_value(output.left_duty, 0.0));
    CHECK(MOTION_REJECTED ==
          motion_submit(&motion, &request, started + 51U));
    CHECK(MOTION_OK == motion_reset(&motion, true, true));
    request.command_id = 1U;
    CHECK(MOTION_OK == motion_submit(&motion, &request, started + 52U));
    request.command_id = UINT32_MAX;
    CHECK(MOTION_REJECTED ==
          motion_submit(&motion, &request, started + 53U));
    request.command_id = 1U;
    CHECK(MOTION_EXPIRED ==
          motion_submit(&motion, &request, started + 102U));
    CHECK(MOTION_INHIBITED == motion.output.state);
}

/** @brief Same-ID renewal preserves distance; completion cannot restart. */
static void test_motion_distance (void)
{
    motion_t motion;
    motion_config_t config = motion_fixture();
    motion_request_t request = {1U, MOTION_DISTANCE, 0.2, 0.0, 0.1};
    motion_output_t output;
    uint32_t sample = 0U;

    CHECK(MOTION_OK == motion_init(&motion, &config));
    CHECK(MOTION_OK == motion_submit(&motion, &request, 0U));
    CHECK(MOTION_OK == motion_step(&motion, 1U, 0.01, 0, 0, &output));
    for (sample = 1U; sample <= TEST_DISTANCE_STEPS; sample += 1U)
    {
        CHECK(MOTION_OK == motion_submit(&motion, &request, sample * 10U));
        CHECK(MOTION_OK ==
              motion_step(&motion, sample * 10U, 0.01, 20, 20, &output));
        CHECK(near_value(output.pose.x_m, 0.02 * sample));
    }
    CHECK(MOTION_COMPLETE == output.state);
    CHECK(near_value(output.left_duty, 0.0));
    CHECK(MOTION_OK == motion_submit(&motion, &request, 60U));
    CHECK(MOTION_COMPLETE == motion.output.state);
    CHECK(near_value(motion.output.left_duty, 0.0));
    request.command_id = 2U;
    request.target = -0.1;
    CHECK(MOTION_OK == motion_submit(&motion, &request, 60U));
    CHECK(MOTION_OK == motion_step(&motion, 61U, 0.01, 0, 0, &output));
    CHECK(output.left_duty < 0.0);
    CHECK(MOTION_OK == motion_step(&motion, 70U, 0.01, -100, -100, &output));
    CHECK(MOTION_COMPLETE == output.state);
    CHECK(near_value(output.pose.x_m, 0.0));
    request.command_id = 3U;
    CHECK(MOTION_OK == motion_submit(&motion, &request, 80U));
    request.target = -0.2;
    CHECK(MOTION_REJECTED == motion_submit(&motion, &request, 90U));
}

/** @brief Turn progress is unwrapped even when the reported pose wraps. */
static void test_motion_turn (void)
{
    motion_t motion;
    motion_config_t config = motion_fixture();
    motion_request_t request = {1U, MOTION_TURN, 0.0, 1.0, 2.0 * CORE_PI};
    motion_output_t output;
    uint32_t sample = 0U;

    CHECK(MOTION_OK == motion_init(&motion, &config));
    CHECK(MOTION_OK == motion_submit(&motion, &request, 0U));
    CHECK(MOTION_OK == motion_step(&motion, 1U, 0.01, 0, 0, &output));
    CHECK(output.left_duty < 0.0);
    CHECK(output.right_duty > 0.0);
    for (sample = 1U; sample <= TEST_TURN_STEPS; sample += 1U)
    {
        CHECK(MOTION_OK == motion_submit(&motion, &request, sample * 10U));
        CHECK(MOTION_OK ==
              motion_step(&motion, sample * 10U, 0.01, -10, 10, &output));
    }
    CHECK(MOTION_COMPLETE == output.state);
    CHECK(fabs(output.pose.yaw_rad) < 0.02);
    CHECK(near_value(output.pose.centre_path_m, 0.0));
    CHECK(near_value(output.left_duty, 0.0));
    request.command_id = 2U;
    request.target = -0.1;
    CHECK(MOTION_OK == motion_submit(&motion, &request, 640U));
    CHECK(MOTION_OK == motion_step(&motion, 641U, 0.01, 0, 0, &output));
    CHECK(MOTION_OK == motion_step(&motion, 650U, 0.01, 10, -10, &output));
    CHECK(MOTION_COMPLETE == output.state);
}

/** @brief Healthy lease renewal cannot hide a stalled primitive deadline. */
static void test_motion_timeout (void)
{
    motion_t motion;
    motion_config_t config = motion_fixture();
    motion_request_t request = {1U, MOTION_DISTANCE, 0.2, 0.0, 1.0};
    motion_output_t output;
    uint32_t sample = 0U;
    motion_status_t expected = MOTION_OK;

    config.primitive_timeout_ms = 200U;
    CHECK(MOTION_OK == motion_init(&motion, &config));
    CHECK(MOTION_OK == motion_submit(&motion, &request, 0U));
    for (sample = 1U; sample <= TEST_TIMEOUT_STEPS; sample += 1U)
    {
        CHECK(MOTION_OK == motion_submit(&motion, &request, sample * 20U));
        expected = (TEST_TIMEOUT_STEPS == sample) ?
                   MOTION_TIMED_OUT : MOTION_OK;
        CHECK(expected ==
              motion_step(&motion, sample * 20U, 0.02, 0, 0, &output));
    }
    CHECK(MOTION_INHIBITED == output.state);
    CHECK(near_value(output.right_duty, 0.0));
}

/** @brief Invalid calibration, input and speed envelope never enable drive. */
static void test_motion_invalid (void)
{
    motion_t motion;
    motion_config_t config = motion_fixture();
    motion_request_t request = {1U, MOTION_VELOCITY, 1.0, 3.0, 0.0};
    motion_output_t output;

    config.b_calibration_confirmed = false;
    CHECK(MOTION_REJECTED == motion_init(&motion, &config));
    CHECK(MOTION_REJECTED == motion_reset(&motion, true, true));
    CHECK(MOTION_REJECTED == motion_submit(&motion, &request, 0U));
    config.b_calibration_confirmed = true;
    CHECK(MOTION_OK == motion_init(&motion, &config));
    CHECK(MOTION_INVALID == motion_submit(&motion, &request, 0U));
    request.forward_mps = NAN;
    CHECK(MOTION_INVALID == motion_submit(&motion, &request, 0U));
    request.forward_mps = 0.2;
    request.yaw_rps = 0.0;
    request.command_id = 0U;
    CHECK(MOTION_INVALID == motion_submit(&motion, &request, 0U));
    request.command_id = 1U;
    CHECK(MOTION_OK == motion_submit(&motion, &request, 0U));
    CHECK(MOTION_SENSOR_FAULT ==
          motion_step(&motion, 10U, NAN, 0, 0, &output));
    CHECK(!output.b_stationary);
    CHECK(near_value(output.left_duty, 0.0));
    config.wheel_pid.output_max = 1.1;
    CHECK(MOTION_INVALID == motion_init(&motion, &config));
    CHECK(MOTION_INHIBITED == motion.output.state);
    CHECK(MOTION_INVALID == motion_init(NULL, &config));
}

/** @brief Explicit start/reset, duplicate velocity IDs and dominant STOP.
 */
static void test_vehicle_start_stop (void)
{
    vehicle_t vehicle;
    vehicle_config_t config = vehicle_fixture();
    vehicle_input_t input = vehicle_inputs();
    vehicle_output_t output;
    uint32_t first_id = 0U;

    config.b_course_confirmed = false;
    CHECK(vehicle_init(&vehicle, &config));
    input.b_start = true;
    CHECK(vehicle_step(&vehicle, 0U, &input, &output));
    CHECK(VEHICLE_INIT == output.state);
    input.b_start = false;
    start_vehicle(&vehicle, &input, &output);
    first_id = output.request.command_id;
    input.line_yaw_rps = 0.3;
    CHECK(vehicle_step(&vehicle, 30U, &input, &output));
    CHECK(first_id == output.request.command_id);
    CHECK(near_value(output.request.yaw_rps, 0.3));
    input.b_stop = true;
    CHECK(vehicle_step(&vehicle, 40U, &input, &output));
    CHECK(VEHICLE_FAULT == output.state);
    CHECK(output.b_stop);
    input.b_stop = false;
    input.b_start = true;
    CHECK(vehicle_step(&vehicle, 50U, &input, &output));
    CHECK(VEHICLE_FAULT == output.state);
    input.b_start = false;
    input.b_reset = true;
    input.motion.b_stationary = false;
    CHECK(vehicle_step(&vehicle, 60U, &input, &output));
    CHECK(!output.b_reset_motion);
    input.motion.b_stationary = true;
    CHECK(vehicle_step(&vehicle, 70U, &input, &output));
    CHECK(VEHICLE_IDLE == output.state);
    CHECK(output.b_reset_motion);
    CHECK(!output.b_issue_motion);
    input.b_reset = false;
    CHECK(vehicle_step(&vehicle, 80U, &input, &output));
    CHECK(VEHICLE_IDLE == output.state);
    input.b_stop = true;
    input.b_start = true;
    CHECK(vehicle_step(&vehicle, 90U, &input, &output));
    CHECK(VEHICLE_FAULT == output.state);
}

/** @brief Both detour sides and pending navigation preserve primitive IDs. */
static void test_vehicle_bypass (int32_t side, vehicle_nav_t navigation)
{
    vehicle_t vehicle;
    vehicle_input_t input = vehicle_inputs();
    vehicle_output_t output;
    uint32_t phase = 0U;
    uint32_t command_id = 0U;
    uint32_t now_ms = 100U;
    core_real_t direction = (side > 0) ? 1.0 : -1.0;
    const motion_kind_t kinds[TEST_BYPASS_STEPS] =
    {
        MOTION_TURN, MOTION_DISTANCE, MOTION_TURN, MOTION_DISTANCE,
        MOTION_TURN, MOTION_DISTANCE, MOTION_TURN
    };
    core_real_t targets[TEST_BYPASS_STEPS] =
    {
        direction * 0.5 * CORE_PI, 0.3, -direction * 0.5 * CORE_PI, 0.7,
        -direction * 0.5 * CORE_PI, 0.3, direction * 0.5 * CORE_PI
    };

    start_vehicle(&vehicle, &input, &output);
    input.b_obstacle = true;
    input.navigation = navigation;
    CHECK(vehicle_step(&vehicle, 30U, &input, &output));
    CHECK(VEHICLE_SCAN == output.state);
    CHECK(MOTION_HALT == output.request.kind);
    CHECK(!output.b_stop);
    input.navigation = VEHICLE_NAV_NONE;
    input.motion.b_stationary = false;
    CHECK(vehicle_step(&vehicle, 40U, &input, &output));
    CHECK(VEHICLE_SCAN_NONE == output.scan);
    input.motion.b_stationary = true;
    CHECK(vehicle_step(&vehicle, 50U, &input, &output));
    CHECK(VEHICLE_SCAN_COARSE == output.scan);
    input.b_coarse_ready = true;
    CHECK(vehicle_step(&vehicle, 60U, &input, &output));
    CHECK(VEHICLE_SCAN_FINE == output.scan);
    input.b_coarse_ready = false;
    input.b_fine_ready = true;
    input.b_bypass_clearance = true;
    input.bypass_side = side;
    input.bypass_offset_m = 0.3;
    input.bypass_length_m = 0.7;
    CHECK(vehicle_step(&vehicle, 70U, &input, &output));
    CHECK(VEHICLE_BYPASS == output.state);
    input.b_fine_ready = false;
    input.b_obstacle = false;
    for (phase = 0U; phase < TEST_BYPASS_STEPS; phase += 1U)
    {
        CHECK(vehicle_step(&vehicle, now_ms, &input, &output));
        CHECK(VEHICLE_BYPASS == output.state);
        CHECK(kinds[phase] == output.request.kind);
        CHECK(near_value(output.request.target, targets[phase]));
        command_id = output.request.command_id;
        input.motion.state = MOTION_RUNNING;
        input.motion.command_id = command_id;
        CHECK(vehicle_step(&vehicle, now_ms + 10U, &input, &output));
        CHECK(command_id == output.request.command_id);
        input.motion.state = MOTION_COMPLETE;
        now_ms += 20U;
    }
    CHECK(vehicle_step(&vehicle, now_ms, &input, &output));
    CHECK(VEHICLE_REACQUIRE == output.state);
    input.b_original_line_verified = true;
    CHECK(vehicle_step(&vehicle, now_ms + 10U, &input, &output));
    CHECK(VEHICLE_FOLLOW == output.state);
    CHECK(navigation == vehicle.pending_nav);
    CHECK(MOTION_HALT == output.request.kind);
    if (VEHICLE_NAV_NONE != navigation)
    {
        input.b_junction_edge = true;
        CHECK(vehicle_step(&vehicle, now_ms + 20U, &input, &output));
        CHECK(VEHICLE_NAVIGATE == output.state);
    }
}

/** @brief Fine-scan unknown, stale front range and scan timeout fail closed. */
static void test_vehicle_scan_faults (void)
{
    vehicle_t vehicle;
    vehicle_input_t input = vehicle_inputs();
    vehicle_output_t output;

    start_vehicle(&vehicle, &input, &output);
    input.b_obstacle = true;
    CHECK(vehicle_step(&vehicle, 30U, &input, &output));
    CHECK(vehicle_step(&vehicle, 40U, &input, &output));
    input.b_coarse_ready = true;
    CHECK(vehicle_step(&vehicle, 50U, &input, &output));
    input.b_coarse_ready = false;
    input.b_fine_ready = true;
    input.b_bypass_clearance = false;
    CHECK(vehicle_step(&vehicle, 60U, &input, &output));
    CHECK(VEHICLE_FAULT == output.state);
    CHECK(output.b_stop);
    input = vehicle_inputs();
    start_vehicle(&vehicle, &input, &output);
    input.b_obstacle = true;
    CHECK(vehicle_step(&vehicle, UINT32_MAX - 20U, &input, &output));
    CHECK(vehicle_step(&vehicle, 979U, &input, &output));
    CHECK(VEHICLE_FAULT == output.state);
    input = vehicle_inputs();
    start_vehicle(&vehicle, &input, &output);
    input.range_age_ms = 150U;
    CHECK(vehicle_step(&vehicle, 30U, &input, &output));
    CHECK(VEHICLE_RANGE_WAIT == output.state);
    CHECK(MOTION_HALT == output.request.kind);
    CHECK(!output.b_stop);
}

/** @brief All four required navigation actions and their recovery transition.
 */
static void test_vehicle_navigation (vehicle_nav_t navigation,
                                     core_real_t expected_angle)
{
    vehicle_t vehicle;
    vehicle_input_t input = vehicle_inputs();
    vehicle_output_t output;

    start_vehicle(&vehicle, &input, &output);
    input.navigation = navigation;
    CHECK(vehicle_step(&vehicle, 30U, &input, &output));
    CHECK(VEHICLE_FOLLOW == output.state);
    CHECK(MOTION_VELOCITY == output.request.kind);
    input.navigation = VEHICLE_NAV_NONE;
    input.motion.pose.x_m += 0.15;
    CHECK(vehicle_step(&vehicle, 31U, &input, &output));
    CHECK(VEHICLE_FOLLOW == output.state);
    input.b_junction_edge = true;
    CHECK(vehicle_step(&vehicle, 32U, &input, &output));
    CHECK(VEHICLE_NAVIGATE == output.state);
    input.b_junction_edge = false;
    /* The sensor-to-axle approach precedes any pivot at the junction. */
    CHECK(MOTION_DISTANCE == output.request.kind);
    CHECK(near_value(output.request.target, 0.15));
    input.motion.state = MOTION_COMPLETE;
    input.motion.command_id = output.request.command_id;
    CHECK(vehicle_step(&vehicle, 40U, &input, &output));
    if (VEHICLE_NAV_STRAIGHT != navigation)
    {
        CHECK(VEHICLE_NAVIGATE == output.state);
        CHECK(MOTION_TURN == output.request.kind);
        CHECK(near_value(output.request.target, expected_angle));
        input.motion.state = MOTION_COMPLETE;
        input.motion.command_id = output.request.command_id;
        CHECK(vehicle_step(&vehicle, 50U, &input, &output));
    }
    CHECK(VEHICLE_REACQUIRE == output.state);
    CHECK(VEHICLE_NAV_NONE == vehicle.pending_nav);
    input.b_original_line_verified = true;
    CHECK(vehicle_step(&vehicle, 60U, &input, &output));
    CHECK(VEHICLE_FOLLOW == output.state);
    input.navigation = navigation;
    CHECK(vehicle_step(&vehicle, UINT32_MAX - 20U, &input, &output));
    input.navigation = VEHICLE_NAV_NONE;
    CHECK(vehicle_step(&vehicle, 9979U, &input, &output));
    CHECK(VEHICLE_FAULT == output.state);
    CHECK(output.b_stop);
}

/** @brief A black sample without original-line evidence cannot end recovery. */
static void test_vehicle_recovery (void)
{
    vehicle_t vehicle;
    vehicle_input_t input = vehicle_inputs();
    vehicle_output_t output;

    start_vehicle(&vehicle, &input, &output);
    input.navigation = VEHICLE_NAV_STRAIGHT;
    CHECK(vehicle_step(&vehicle, 30U, &input, &output));
    input.navigation = VEHICLE_NAV_NONE;
    input.b_junction_edge = true;
    CHECK(vehicle_step(&vehicle, 31U, &input, &output));
    input.b_junction_edge = false;
    input.motion.state = MOTION_COMPLETE;
    input.motion.command_id = output.request.command_id;
    CHECK(vehicle_step(&vehicle, 40U, &input, &output));
    CHECK(VEHICLE_REACQUIRE == output.state);
    CHECK(vehicle_step(&vehicle, 50U, &input, &output));
    CHECK(VEHICLE_REACQUIRE == output.state);
    CHECK(MOTION_DISTANCE == output.request.kind);
    CHECK(near_value(output.request.target, 0.2));
    input.motion.command_id = output.request.command_id;
    CHECK(vehicle_step(&vehicle, 60U, &input, &output));
    CHECK(VEHICLE_FAULT == output.state);
    CHECK(output.b_stop);
}

/** @brief A scanned turn may face the obstacle; forward segments may not. */
static void test_vehicle_turn_guard (void)
{
    vehicle_t vehicle;
    vehicle_input_t input = vehicle_inputs();
    vehicle_output_t output;

    start_vehicle(&vehicle, &input, &output);
    input.b_obstacle = true;
    input.b_front_clear = false;
    CHECK(vehicle_step(&vehicle, 30U, &input, &output));
    CHECK(vehicle_step(&vehicle, 40U, &input, &output));
    input.b_coarse_ready = true;
    CHECK(vehicle_step(&vehicle, 50U, &input, &output));
    input.b_coarse_ready = false;
    input.b_fine_ready = true;
    input.b_bypass_clearance = true;
    input.bypass_side = 1;
    input.bypass_offset_m = 0.3;
    input.bypass_length_m = 0.7;
    CHECK(vehicle_step(&vehicle, 60U, &input, &output));
    input.b_fine_ready = false;
    CHECK(vehicle_step(&vehicle, 70U, &input, &output));
    CHECK(VEHICLE_BYPASS == output.state);
    CHECK(MOTION_TURN == output.request.kind);
    input.motion.state = MOTION_COMPLETE;
    input.motion.command_id = output.request.command_id;
    CHECK(vehicle_step(&vehicle, 80U, &input, &output));
    CHECK(VEHICLE_FAULT == output.state);
    CHECK(output.b_stop);
}

/** @brief Use real motion completion rather than injecting completion events.
 * Wheel counts follow the requested geometry, not a simulated motor plant.
 * This validates API/lease/state coordination, not PID tracking accuracy.
 */
static void test_integrated_sequence (void)
{
    motion_t motion;
    vehicle_t vehicle;
    motion_config_t motion_config = motion_fixture();
    vehicle_config_t config = vehicle_fixture();
    vehicle_input_t input = vehicle_inputs();
    vehicle_output_t output;
    uint32_t sample = 0U;
    uint32_t completions = 0U;
    uint32_t previous_complete = 0U;
    bool b_sequence_ok = true;
    bool b_returned_follow = false;
    bool b_saw_bypass = false;

    motion_config.primitive_timeout_ms = 10000U;
    /* The synthetic bypass takes over 10 s at 2 mm per 10 ms sample. */
    config.pending_nav_timeout_ms = 20000U;
    CHECK(MOTION_OK == motion_init(&motion, &motion_config));
    CHECK(vehicle_init(&vehicle, &config));
    memset(&output, 0, sizeof(output));
    input.b_bypass_clearance = true;
    input.bypass_side = 1;
    input.bypass_offset_m = 0.3;
    input.bypass_length_m = 0.7;
    for (sample = 1U; sample <= TEST_INTEGRATION_LIMIT; sample += 1U)
    {
        input.motion = motion.output;
        input.b_start = (2U == sample);
        input.b_obstacle = (4U == sample);
        input.navigation = (4U == sample) ?
                           VEHICLE_NAV_UTURN : VEHICLE_NAV_NONE;
        input.b_coarse_ready = (VEHICLE_SCAN_COARSE == output.scan);
        input.b_fine_ready = (VEHICLE_SCAN_FINE == output.scan);
        input.b_original_line_verified =
            (VEHICLE_REACQUIRE == vehicle.state);
        input.b_junction_edge = b_saw_bypass &&
            (VEHICLE_FOLLOW == vehicle.state) &&
            (VEHICLE_NAV_NONE != vehicle.pending_nav);
        if ((!vehicle_step(&vehicle, sample * 10U, &input, &output)) ||
            (!integrated_motion_tick(&motion, &output, sample * 10U)))
        {
            b_sequence_ok = false;
            break;
        }
        if ((MOTION_COMPLETE == motion.output.state) &&
            (previous_complete != motion.output.command_id))
        {
            previous_complete = motion.output.command_id;
            completions += 1U;
        }
        if (VEHICLE_BYPASS == output.state)
        {
            b_saw_bypass = true;
        }
        if (b_saw_bypass && (VEHICLE_FOLLOW == output.state) &&
            (VEHICLE_NAV_NONE == vehicle.pending_nav))
        {
            b_returned_follow = true;
            break;
        }
    }
    CHECK(b_sequence_ok);
    CHECK(b_saw_bypass);
    CHECK(b_returned_follow);
    CHECK(9U == completions);
    CHECK(motion.output.pose.centre_path_m >= 1.4);
    CHECK(!motion.b_stop_latched);
    CHECK(VEHICLE_NAV_NONE == vehicle.pending_nav);
}

/** @brief Feed 1 mm/count encoder samples consistent with primitive
 * direction.
 */
static bool integrated_motion_tick (motion_t * p_motion,
                                    const vehicle_output_t * p_output,
                                    uint32_t now_ms)
{
    motion_output_t feedback;
    int32_t left_delta = 0;
    int32_t right_delta = 0;
    int32_t direction = 1;

    if (p_output->b_stop)
    {
        motion_stop(p_motion, MOTION_STOPPED);
        return false;
    }
    if (MOTION_RUNNING == p_motion->output.state)
    {
        direction = (p_motion->request.target < 0.0) ? -1 : 1;
        switch (p_motion->request.kind)
        {
            case MOTION_TURN:
                left_delta = -direction;
                right_delta = direction;
            break;
            case MOTION_DISTANCE:
                left_delta = 2 * direction;
                right_delta = 2 * direction;
            break;
            case MOTION_VELOCITY:
                left_delta = 2;
                right_delta = 2;
            break;
            case MOTION_HALT:
                left_delta = 0;
                right_delta = 0;
            break;
            default:
                return false;
        }
    }
    if (p_output->b_issue_motion &&
        (MOTION_OK != motion_submit(p_motion, &p_output->request, now_ms)))
    {
        return false;
    }
    return (MOTION_OK == motion_step(p_motion, now_ms, 0.01,
                                     left_delta, right_delta, &feedback));
}

/** @brief A partly executed junction turn must not be replayed after bypass.
 * Without a confirmed junction/map anchor, the correct outcome is a fault.
 */
static void test_interrupted_navigation (void)
{
    vehicle_t vehicle;
    vehicle_input_t input;
    vehicle_output_t output;
    uint32_t scenario = 0U;

    /* Scenario 0 interrupts the pivot; scenario 1 interrupts the approach. */
    for (scenario = 0U; scenario < 2U; scenario += 1U)
    {
        input = vehicle_inputs();
        start_vehicle(&vehicle, &input, &output);
        input.navigation = VEHICLE_NAV_LEFT;
        CHECK(vehicle_step(&vehicle, 30U, &input, &output));
        CHECK(VEHICLE_FOLLOW == output.state);
        input.navigation = VEHICLE_NAV_NONE;
        input.b_junction_edge = true;
        CHECK(vehicle_step(&vehicle, 31U, &input, &output));
        CHECK(MOTION_DISTANCE == output.request.kind);
        input.b_junction_edge = false;
        if (0U == scenario)
        {
            input.motion.state = MOTION_COMPLETE;
            input.motion.command_id = output.request.command_id;
            CHECK(vehicle_step(&vehicle, 35U, &input, &output));
            CHECK(MOTION_TURN == output.request.kind);
            input.motion.pose.yaw_rad = 0.25 * CORE_PI;
        }
        input.motion.state = MOTION_RUNNING;
        input.motion.command_id = output.request.command_id;
        input.b_obstacle = true;
        CHECK(vehicle_step(&vehicle, 40U, &input, &output));
        CHECK(VEHICLE_FAULT == output.state);
        CHECK(output.b_stop);
        CHECK(!output.b_issue_motion);
        CHECK(VEHICLE_SCAN_NONE == output.scan);
        input.b_obstacle = false;
        input.b_original_line_verified = true;
        CHECK(vehicle_step(&vehicle, 50U, &input, &output));
        CHECK(VEHICLE_FAULT == output.state);
        CHECK(!output.b_issue_motion);
    }
}

/** @brief Encoder counts preceding first actuation cannot shorten a target. */
static void test_motion_command_boundary (void)
{
    motion_t motion;
    motion_config_t config = motion_fixture();
    motion_request_t request = {1U, MOTION_DISTANCE, 0.2, 0.0, 0.1};
    motion_output_t output;

    CHECK(MOTION_OK == motion_init(&motion, &config));
    CHECK(MOTION_OK == motion_submit(&motion, &request, 0U));
    CHECK(MOTION_OK == motion_step(&motion, 1U, 0.01, 70, 70, &output));
    CHECK(near_value(output.pose.x_m, 0.07));
    CHECK(near_value(motion.progress, 0.0));
    CHECK(MOTION_RUNNING == output.state);
    CHECK(MOTION_OK == motion_step(&motion, 10U, 0.01, 30, 30, &output));
    CHECK(MOTION_RUNNING == output.state);
    CHECK(near_value(motion.progress, 0.03));
    CHECK(MOTION_OK == motion_step(&motion, 20U, 0.01, 70, 70, &output));
    CHECK(MOTION_COMPLETE == output.state);
    CHECK(near_value(output.pose.x_m, 0.17));
    CHECK(near_value(output.signed_distance_m, 0.07));
}

/** @brief Junction debounce halts without losing the ability to resume. */
static void test_vehicle_line_wait (void)
{
    vehicle_t vehicle;
    vehicle_input_t input = vehicle_inputs();
    vehicle_output_t output;
    uint32_t old_id = 0U;

    start_vehicle(&vehicle, &input, &output);
    old_id = output.request.command_id;
    input.b_line_valid = false;
    input.b_line_wait = true;
    CHECK(vehicle_step(&vehicle, 30U, &input, &output));
    CHECK(VEHICLE_FOLLOW == output.state);
    CHECK(MOTION_HALT == output.request.kind);
    CHECK(!output.b_stop);
    CHECK(vehicle_step(&vehicle, 40U, &input, &output));
    CHECK(MOTION_HALT == output.request.kind);
    input.b_line_valid = true;
    input.b_line_wait = false;
    CHECK(vehicle_step(&vehicle, 50U, &input, &output));
    CHECK(VEHICLE_FOLLOW == output.state);
    CHECK(MOTION_VELOCITY == output.request.kind);
    CHECK(output.request.command_id != old_id);
    input.b_line_valid = false;
    CHECK(vehicle_step(&vehicle, 60U, &input, &output));
    CHECK(VEHICLE_FAULT == output.state);
    CHECK(output.b_stop);
}

/** @brief A delayed supervisor renewal cannot fault a finished primitive.
 * Model preemption between the supervisor's input snapshot and its publish.
 */
static void test_motion_completion_renewal (void)
{
    motion_t motion;
    vehicle_t vehicle;
    motion_config_t config = motion_fixture();
    vehicle_input_t input = vehicle_inputs();
    vehicle_output_t output;
    motion_output_t feedback;
    motion_request_t completed;

    CHECK(MOTION_OK == motion_init(&motion, &config));
    start_vehicle(&vehicle, &input, &output);
    input.navigation = VEHICLE_NAV_STRAIGHT;
    CHECK(vehicle_step(&vehicle, 29U, &input, &output));
    CHECK(VEHICLE_FOLLOW == output.state);
    input.navigation = VEHICLE_NAV_NONE;
    input.b_junction_edge = true;
    CHECK(vehicle_step(&vehicle, 30U, &input, &output));
    CHECK(MOTION_DISTANCE == output.request.kind);
    CHECK(MOTION_OK == motion_submit(&motion, &output.request, 30U));
    CHECK(MOTION_OK == motion_step(&motion, 30U, 0.01, 0, 0, &feedback));
    input.navigation = VEHICLE_NAV_NONE;
    input.motion = feedback;
    CHECK(MOTION_RUNNING == input.motion.state);
    CHECK(MOTION_OK == motion_step(&motion, 40U, 0.01, 150, 150,
                                   &feedback));
    CHECK(MOTION_COMPLETE == feedback.state);
    completed = motion.request;
    CHECK(vehicle_step(&vehicle, 40U, &input, &output));
    CHECK(VEHICLE_NAVIGATE == output.state);
    CHECK(output.request.command_id == completed.command_id);
    CHECK(MOTION_OK == motion_submit(&motion, &output.request, 41U));
    CHECK(MOTION_COMPLETE == motion.output.state);
    CHECK(near_value(motion.output.left_duty, 0.0));
    CHECK(near_value(motion.output.right_duty, 0.0));
    CHECK(30U == motion.started_ms);
    CHECK(30U == motion.renewed_ms);
    CHECK(MOTION_OK == motion_step(&motion, 50U, 0.01, 0, 0, &feedback));
    CHECK(MOTION_COMPLETE == feedback.state);
    input.motion = feedback;
    CHECK(vehicle_step(&vehicle, 50U, &input, &output));
    CHECK(VEHICLE_REACQUIRE == output.state);
    completed.target += 0.1;
    CHECK(MOTION_REJECTED == motion_submit(&motion, &completed, 51U));
    CHECK(MOTION_COMPLETE == motion.output.state);
    CHECK(!motion.b_stop_latched);
}

/** @brief Terminal line loss wins over simultaneous route-changing events. */
static void test_vehicle_line_loss_precedence (void)
{
    vehicle_t vehicle;
    vehicle_input_t input = vehicle_inputs();
    vehicle_output_t output;

    start_vehicle(&vehicle, &input, &output);
    input.b_line_valid = false;
    input.b_line_wait = false;
    input.navigation = VEHICLE_NAV_LEFT;
    CHECK(vehicle_step(&vehicle, 30U, &input, &output));
    CHECK(VEHICLE_FAULT == output.state);
    CHECK(output.b_stop);
    CHECK(!output.b_issue_motion);
    CHECK(VEHICLE_SCAN_NONE == output.scan);
    input = vehicle_inputs();
    start_vehicle(&vehicle, &input, &output);
    input.b_line_valid = false;
    input.b_line_wait = false;
    input.b_obstacle = true;
    CHECK(vehicle_step(&vehicle, 30U, &input, &output));
    CHECK(VEHICLE_FAULT == output.state);
    CHECK(output.b_stop);
    CHECK(!output.b_issue_motion);
    CHECK(VEHICLE_SCAN_NONE == output.scan);
}

/** @brief A paused FOLLOW needs fresh range after rest and a new motion ID. */
static void test_vehicle_range_recovery (void)
{
    vehicle_t vehicle;
    vehicle_input_t input = vehicle_inputs();
    vehicle_output_t output;
    uint32_t old_id = 0U;

    start_vehicle(&vehicle, &input, &output);
    old_id = output.request.command_id;
    input.navigation = VEHICLE_NAV_LEFT;
    CHECK(vehicle_step(&vehicle, 25U, &input, &output));
    input.navigation = VEHICLE_NAV_NONE;
    input.b_range_valid = false;
    input.motion.b_stationary = false;
    CHECK(vehicle_step(&vehicle, 30U, &input, &output));
    CHECK(VEHICLE_RANGE_WAIT == output.state);
    CHECK(MOTION_HALT == output.request.kind);
    CHECK(!output.b_stop);
    CHECK(vehicle_step(&vehicle, 40U, &input, &output));
    CHECK(VEHICLE_RANGE_WAIT == output.state);
    input.b_range_valid = true;
    input.motion.b_stationary = true;
    input.motion.state = MOTION_RUNNING;
    CHECK(vehicle_step(&vehicle, 50U, &input, &output));
    CHECK(VEHICLE_RANGE_WAIT == output.state);
    input.motion.state = MOTION_IDLE;
    CHECK(vehicle_step(&vehicle, 60U, &input, &output));
    CHECK(VEHICLE_RANGE_WAIT == output.state);
    input.range_age_ms = 10U;
    CHECK(vehicle_step(&vehicle, 70U, &input, &output));
    CHECK(VEHICLE_RANGE_WAIT == output.state);
    CHECK(MOTION_HALT == output.request.kind);
    input.range_age_ms = 0U;
    CHECK(vehicle_step(&vehicle, 80U, &input, &output));
    CHECK(VEHICLE_FOLLOW == output.state);
    CHECK(MOTION_HALT == output.request.kind);
    CHECK(vehicle_step(&vehicle, 90U, &input, &output));
    CHECK(MOTION_VELOCITY == output.request.kind);
    CHECK(old_id != output.request.command_id);
    CHECK(VEHICLE_NAV_LEFT == vehicle.pending_nav);
    CHECK(25U == vehicle.pending_nav_ms);
}

/** @brief Missing range stays stopped; wrap-safe expiry needs local reset. */
static void test_vehicle_range_bounds (void)
{
    vehicle_t vehicle;
    vehicle_input_t input = vehicle_inputs();
    vehicle_output_t output;
    vehicle_config_t config = vehicle_fixture();

    config.range_wait_timeout_ms = 0U;
    CHECK(!vehicle_init(&vehicle, &config));
    config.range_wait_timeout_ms = UINT32_C(2147483648);
    CHECK(!vehicle_init(&vehicle, &config));
    start_vehicle(&vehicle, &input, &output);
    input.b_range_valid = false;
    CHECK(vehicle_step(&vehicle, UINT32_MAX - 99U, &input, &output));
    CHECK(VEHICLE_RANGE_WAIT == output.state);
    CHECK(vehicle_step(&vehicle, 899U, &input, &output));
    CHECK(VEHICLE_RANGE_WAIT == output.state);
    CHECK(MOTION_HALT == output.request.kind);
    input.b_range_valid = true;
    CHECK(vehicle_step(&vehicle, 900U, &input, &output));
    CHECK(VEHICLE_FAULT == output.state);
    CHECK(output.b_stop);
    input.b_start = true;
    CHECK(vehicle_step(&vehicle, 910U, &input, &output));
    CHECK(VEHICLE_FAULT == output.state);
    input.b_reset = true;
    CHECK(vehicle_step(&vehicle, 920U, &input, &output));
    CHECK(VEHICLE_IDLE == output.state);
    CHECK(output.b_reset_motion);
    CHECK(!output.b_issue_motion);
    input.b_start = false;
    input.b_reset = false;
    CHECK(vehicle_step(&vehicle, 930U, &input, &output));
    CHECK(VEHICLE_IDLE == output.state);

    input = vehicle_inputs();
    start_vehicle(&vehicle, &input, &output);
    input.navigation = VEHICLE_NAV_LEFT;
    CHECK(vehicle_step(&vehicle, UINT32_MAX - 499U, &input, &output));
    input.navigation = VEHICLE_NAV_NONE;
    input.b_range_valid = false;
    CHECK(vehicle_step(&vehicle, 9400U, &input, &output));
    CHECK(VEHICLE_RANGE_WAIT == output.state);
    CHECK(vehicle_step(&vehicle, 9499U, &input, &output));
    CHECK(VEHICLE_RANGE_WAIT == output.state);
    CHECK(vehicle_step(&vehicle, 9500U, &input, &output));
    CHECK(VEHICLE_FAULT == output.state);
    CHECK(output.b_stop);

    input = vehicle_inputs();
    start_vehicle(&vehicle, &input, &output);
    input.b_range_valid = false;
    CHECK(vehicle_step(&vehicle, UINT32_MAX - 19U, &input, &output));
    CHECK(vehicle_step(&vehicle, UINT32_MAX - 9U, &input, &output));
    input.b_range_valid = true;
    CHECK(vehicle_step(&vehicle, 5U, &input, &output));
    CHECK(VEHICLE_FOLLOW == output.state);
    CHECK(MOTION_HALT == output.request.kind);
}

/** @brief STOP, health, route ambiguity and unclear readings fail closed. */
static void test_vehicle_range_guards (void)
{
    vehicle_t vehicle;
    vehicle_input_t input;
    vehicle_output_t output;
    uint32_t scenario = 0U;

    for (scenario = 0U; scenario < 7U; scenario += 1U)
    {
        input = vehicle_inputs();
        start_vehicle(&vehicle, &input, &output);
        input.navigation = VEHICLE_NAV_LEFT;
        CHECK(vehicle_step(&vehicle, 25U, &input, &output));
        input.navigation = VEHICLE_NAV_NONE;
        input.b_range_valid = false;
        CHECK(vehicle_step(&vehicle, 30U, &input, &output));
        CHECK(vehicle_step(&vehicle, 40U, &input, &output));
        input.b_range_valid = true;
        input.b_stop = (0U == scenario);
        input.b_reset = (0U == scenario);
        input.b_sensors_ready = (1U != scenario);
        input.b_line_valid = (2U != scenario);
        input.b_junction_edge = (3U == scenario);
        input.b_front_clear = (4U != scenario);
        input.motion.state = (5U == scenario) ?
                              MOTION_INHIBITED : MOTION_IDLE;
        input.navigation = (6U == scenario) ?
                            VEHICLE_NAV_RIGHT : VEHICLE_NAV_NONE;
        CHECK(vehicle_step(&vehicle, 50U, &input, &output));
        CHECK(VEHICLE_FAULT == output.state);
        CHECK(output.b_stop);
        CHECK(!output.b_issue_motion);
        CHECK(!output.b_reset_motion);
    }
    input = vehicle_inputs();
    start_vehicle(&vehicle, &input, &output);
    input.b_range_valid = false;
    CHECK(vehicle_step(&vehicle, 30U, &input, &output));
    CHECK(vehicle_step(&vehicle, 40U, &input, &output));
    input.b_range_valid = true;
    input.b_obstacle = true;
    input.b_front_clear = false;
    CHECK(vehicle_step(&vehicle, 50U, &input, &output));
    CHECK(VEHICLE_SCAN == output.state);
    CHECK(MOTION_HALT == output.request.kind);
    CHECK(VEHICLE_SCAN_NONE == output.scan);
    CHECK(vehicle_step(&vehicle, 60U, &input, &output));
    CHECK(VEHICLE_SCAN_COARSE == output.scan);

    input = vehicle_inputs();
    start_vehicle(&vehicle, &input, &output);
    input.navigation = VEHICLE_NAV_LEFT;
    CHECK(vehicle_step(&vehicle, 25U, &input, &output));
    input.navigation = VEHICLE_NAV_NONE;
    input.b_junction_edge = true;
    input.b_range_valid = false;
    CHECK(vehicle_step(&vehicle, 30U, &input, &output));
    CHECK(VEHICLE_FAULT == output.state);
    CHECK(output.b_stop);
}

/** @brief Missing range never restarts partly executed navigation or bypass. */
static void test_vehicle_range_primitives (void)
{
    vehicle_t vehicle;
    vehicle_input_t input;
    vehicle_output_t output;
    uint32_t scenario = 0U;

    for (scenario = 0U; scenario < 2U; scenario += 1U)
    {
        input = vehicle_inputs();
        start_vehicle(&vehicle, &input, &output);
        if (0U == scenario)
        {
            input.navigation = VEHICLE_NAV_LEFT;
            CHECK(vehicle_step(&vehicle, 30U, &input, &output));
            input.navigation = VEHICLE_NAV_NONE;
            input.b_junction_edge = true;
            CHECK(vehicle_step(&vehicle, 40U, &input, &output));
            CHECK(VEHICLE_NAVIGATE == output.state);
            CHECK(MOTION_DISTANCE == output.request.kind);
            input.b_junction_edge = false;
            input.motion.state = MOTION_COMPLETE;
            input.motion.command_id = output.request.command_id;
            CHECK(vehicle_step(&vehicle, 50U, &input, &output));
            CHECK(VEHICLE_NAVIGATE == output.state);
        }
        else
        {
            input.b_obstacle = true;
            CHECK(vehicle_step(&vehicle, 30U, &input, &output));
            CHECK(vehicle_step(&vehicle, 40U, &input, &output));
            input.b_coarse_ready = true;
            CHECK(vehicle_step(&vehicle, 50U, &input, &output));
            input.b_fine_ready = true;
            input.b_bypass_clearance = true;
            input.bypass_side = 1;
            input.bypass_offset_m = 0.3;
            input.bypass_length_m = 0.7;
            CHECK(vehicle_step(&vehicle, 60U, &input, &output));
            CHECK(vehicle_step(&vehicle, 70U, &input, &output));
            CHECK(VEHICLE_BYPASS == output.state);
        }
        CHECK(MOTION_TURN == output.request.kind);
        input.motion.state = MOTION_RUNNING;
        input.motion.pose.yaw_rad = 0.25 * CORE_PI;
        input.b_range_valid = false;
        CHECK(vehicle_step(&vehicle, 80U, &input, &output));
        CHECK(VEHICLE_FAULT == output.state);
        CHECK(output.b_stop);
        input.b_range_valid = true;
        CHECK(vehicle_step(&vehicle, 90U, &input, &output));
        CHECK(VEHICLE_FAULT == output.state);
        CHECK(!output.b_issue_motion);
    }
}

/** @brief Kinematic regression: a junction pivot starts with the axle, the
 * in-place turn centre, within tolerance_m of the crossbar centre. The edge
 * event fires on the first sample with the sensors edge_travel_m or more
 * past the crossbar's near edge; approach_m is the configured
 * junction_distance_m. Real motion completion drives 1 mm counts; the plant
 * is ideal and moves TEST_PLANT_STEP_M per sample. The simulated pivot
 * therefore includes up to one step of edge-sampling lag and one step of
 * command latency (Motion does not count the first sample after a submit),
 * and both only add travel. The minimum-travel pivot, the exact edge
 * crossing plus the travel the approach primitive counted, must also land
 * within tolerance_m, and each added travel is bounded to one plant step.
 */
static void test_vehicle_junction_geometry (vehicle_nav_t navigation,
                                            core_real_t expected_angle,
                                            core_real_t edge_travel_m,
                                            core_real_t approach_m,
                                            core_real_t tolerance_m)
{
    motion_t motion;
    vehicle_t vehicle;
    motion_config_t motion_config = motion_fixture();
    vehicle_config_t config = vehicle_fixture();
    vehicle_input_t input = vehicle_inputs();
    vehicle_output_t output;
    uint32_t sample = 0U;
    core_real_t edge_x = TEST_CROSSBAR_M - (0.5 * TEST_LINE_WIDTH_M) +
                         edge_travel_m;
    core_real_t pivot_x = -1.0;
    core_real_t edge_pose_x = -1.0;
    core_real_t approach_x = -1.0;
    core_real_t exact_pivot_x = -1.0;
    core_real_t yaw_error = 0.0;
    bool b_edge_sent = false;
    bool b_approach = false;
    bool b_ok = true;

    /* A U-turn at 2 mm per 10 ms sample takes about 3.2 s. */
    motion_config.primitive_timeout_ms = 10000U;
    config.junction_distance_m = approach_m;
    CHECK(MOTION_OK == motion_init(&motion, &motion_config));
    CHECK(vehicle_init(&vehicle, &config));
    memset(&output, 0, sizeof(output));
    for (sample = 1U; (sample <= TEST_INTEGRATION_LIMIT) && b_ok &&
                      (VEHICLE_REACQUIRE != output.state); sample += 1U)
    {
        input.motion = motion.output;
        input.b_start = (2U == sample);
        input.navigation = (5U == sample) ? navigation : VEHICLE_NAV_NONE;
        input.b_junction_edge = (!b_edge_sent) &&
            ((motion.output.pose.x_m + TEST_SENSOR_AXLE_M) >= edge_x);
        if (input.b_junction_edge)
        {
            edge_pose_x = motion.output.pose.x_m;
        }
        b_edge_sent = b_edge_sent || input.b_junction_edge;
        b_ok = vehicle_step(&vehicle, sample * 10U, &input, &output);
        b_approach = b_edge_sent && (approach_x < 0.0) &&
                     output.b_issue_motion &&
                     (MOTION_DISTANCE == output.request.kind);
        if ((pivot_x < 0.0) && (VEHICLE_NAVIGATE == output.state) &&
            (MOTION_TURN == output.request.kind))
        {
            pivot_x = motion.output.pose.x_m;
        }
        b_ok = b_ok && integrated_motion_tick(&motion, &output,
                                              sample * 10U);
        if (b_approach)
        {
            /* The approach counts from the end of its uncounted sample. */
            approach_x = motion.output.pose.x_m;
        }
    }
    yaw_error = atan2(sin(motion.output.pose.yaw_rad - expected_angle),
                      cos(motion.output.pose.yaw_rad - expected_angle));
    exact_pivot_x = (edge_x - TEST_SENSOR_AXLE_M) + (pivot_x - approach_x);
    CHECK(b_ok);
    CHECK(b_edge_sent);
    CHECK(VEHICLE_REACQUIRE == output.state);
    CHECK(fabs(pivot_x - TEST_CROSSBAR_M) <= tolerance_m);
    CHECK(fabs(motion.output.pose.x_m - pivot_x) < 1.0e-9);
    CHECK(fabs(yaw_error) < 0.02);
    /* Edge-sampling lag and command latency: at most one plant step each. */
    CHECK(((edge_pose_x + TEST_SENSOR_AXLE_M) >= edge_x) &&
          ((edge_pose_x + TEST_SENSOR_AXLE_M) <
           (edge_x + TEST_PLANT_STEP_M)));
    CHECK((approach_x > edge_pose_x) &&
          ((approach_x - edge_pose_x) <= (TEST_PLANT_STEP_M + 1.0e-9)));
    CHECK(fabs((pivot_x - approach_x) - approach_m) <=
          motion_config.distance_tolerance_m);
    CHECK(fabs(exact_pivot_x - TEST_CROSSBAR_M) <= tolerance_m);
}

/** @brief NI-MOT-1 with production detection travel: the edge fires after
 * the 20 ms debounce at line speed from the near edge (0 mm entry), the
 * shortest travel before the approach starts (Vehicle and Motion latency
 * only add travel), not after an assumed 2 mm. With the raw 60 mm
 * sensor-to-axle offset as the approach, the pivot must still land inside
 * the crossbar on every navigation. The binding check is the minimum-travel
 * pivot from the exact 1.6 mm edge crossing, 7.4 mm short of the centre
 * against the 9 mm half-width tolerance. The sampled pivot (4.0 mm short)
 * also includes 1.4 mm of edge-sampling lag and 2 mm of command latency
 * from the 2 mm plant step. Shortening the approach to 55.5 mm (or 58.5 mm,
 * which Motion completes 1 mm early within its tolerance) fails the check.
 * A real robot's value is the measured detection-to-crossbar-centre travel.
 */
static void test_junction_production (void)
{
    core_real_t travel_m = TEST_LINE_SPEED_MPS * TEST_JUNCTION_DEBOUNCE_S;

    test_vehicle_junction_geometry(VEHICLE_NAV_LEFT, 0.5 * CORE_PI,
        travel_m, TEST_SENSOR_AXLE_M, 0.5 * TEST_LINE_WIDTH_M);
    test_vehicle_junction_geometry(VEHICLE_NAV_RIGHT, -0.5 * CORE_PI,
        travel_m, TEST_SENSOR_AXLE_M, 0.5 * TEST_LINE_WIDTH_M);
    test_vehicle_junction_geometry(VEHICLE_NAV_UTURN, CORE_PI,
        travel_m, TEST_SENSOR_AXLE_M, 0.5 * TEST_LINE_WIDTH_M);
}

/** @brief Sub-count 5 ms samples converge without changing raw odometry. */
static void test_motion_speed_window (void)
{
    motion_t motion;
    motion_config_t config = motion_fixture();
    motion_request_t request = {1U, MOTION_VELOCITY, 0.08, 0.0, 0.0};
    motion_output_t output;
    uint32_t sample = 0U;
    uint32_t counts = 0U;
    int32_t delta = 0;

    config.wheel_pid.proportional_gain = 1.0;
    config.wheel_pid.integral_gain = 0.0;
    config.wheel_pid.max_dt_s = 0.025;
    CHECK(MOTION_OK == motion_init(&motion, &config));
    CHECK(MOTION_OK == motion_submit(&motion, &request, 0U));
    for (sample = 1U; sample <= 60U; sample += 1U)
    {
        /* Two counts per 25 ms = 0.08 m/s, with 0/0.2 m/s raw feedback. */
        delta = ((3U == (sample % 5U)) || (0U == (sample % 5U))) ? 1 : 0;
        counts += (uint32_t) delta;
        CHECK(MOTION_OK == motion_submit(&motion, &request, sample * 5U));
        CHECK(MOTION_OK == motion_step(&motion, sample * 5U, 0.005,
                                       delta, delta, &output));
        CHECK(near_value(output.pose.x_m, 0.001 * counts));
        CHECK(near_value(output.signed_distance_m, 0.001 * delta));
        if (sample >= 10U)
        {
            CHECK(near_value(output.left_speed_mps, 0.08));
            CHECK(near_value(output.right_speed_mps, 0.08));
            CHECK(near_value(output.left_duty, 0.0));
            CHECK(near_value(output.right_duty, 0.0));
        }
    }
    /* STOP is not issued: the moving history must expire by observation. */
    for (sample = 61U; sample <= 70U; sample += 1U)
    {
        CHECK(MOTION_OK == motion_submit(&motion, &request, sample * 5U));
        CHECK(MOTION_OK == motion_step(&motion, sample * 5U, 0.005,
                                       0, 0, &output));
    }
    CHECK(near_value(output.left_speed_mps, 0.0));
    CHECK(near_value(output.right_speed_mps, 0.0));
    CHECK(near_value(output.pose.x_m, 0.024));
}

/** @brief STOP/HALT/reset and wheel-direction changes discard old feedback. */
static void test_motion_speed_reset (void)
{
    motion_t motion;
    motion_config_t config = motion_fixture();
    motion_request_t request = {1U, MOTION_VELOCITY, 0.2, 0.0, 0.0};
    motion_request_t halt = {0U, MOTION_HALT, 0.0, 0.0, 0.0};
    motion_output_t output;

    CHECK(MOTION_OK == motion_init(&motion, &config));
    CHECK(MOTION_OK == motion_submit(&motion, &request, 0U));
    CHECK(MOTION_OK == motion_step(&motion, 5U, 0.005, 1, 1, &output));
    CHECK(near_value(output.left_speed_mps, 0.2));
    request.forward_mps = 0.1;
    CHECK(MOTION_OK == motion_submit(&motion, &request, 6U));
    CHECK(near_value(motion.output.left_speed_mps, 0.2));
    request.forward_mps = -0.1;
    CHECK(MOTION_OK == motion_submit(&motion, &request, 7U));
    CHECK(near_value(motion.output.left_speed_mps, 0.0));
    CHECK(!motion.left_pid.b_has_previous_measurement);
    CHECK(MOTION_OK == motion_step(&motion, 10U, 0.005, -1, -1, &output));
    CHECK(near_value(output.left_speed_mps, -0.2));
    CHECK(!output.b_stationary);
    CHECK(near_value(output.pose.x_m, 0.0));
    CHECK(near_value(output.signed_distance_m, -0.001));
    motion_stop(&motion, MOTION_STOPPED);
    CHECK(near_value(motion.output.left_speed_mps, 0.0));
    CHECK(!motion.output.b_stationary);
    CHECK(MOTION_REJECTED == motion_reset(&motion, true, true));
    CHECK(MOTION_STOPPED == motion_step(&motion, 15U, 0.005,
                                       1, 1, &output));
    CHECK(near_value(output.left_speed_mps, 0.2));
    CHECK(MOTION_REJECTED == motion_reset(&motion, true, true));
    CHECK(MOTION_STOPPED == motion_step(&motion, 20U, 0.005,
                                       0, 0, &output));
    CHECK(MOTION_OK == motion_reset(&motion, true, true));
    CHECK(near_value(motion.output.left_speed_mps, 0.0));
    CHECK(MOTION_REJECTED == motion_submit(&motion, &request, 21U));
    request.command_id = 2U;
    request.forward_mps = 0.1;
    CHECK(MOTION_OK == motion_submit(&motion, &request, 21U));
    CHECK(MOTION_OK == motion_step(&motion, 25U, 0.005, 0, 0, &output));
    CHECK(near_value(output.left_speed_mps, 0.0));
    CHECK(MOTION_OK == motion_step(&motion, 30U, 0.005, 1, 1, &output));
    request.yaw_rps = 2.0;
    CHECK(MOTION_OK == motion_submit(&motion, &request, 31U));
    CHECK(near_value(motion.output.left_speed_mps, 0.0));
    CHECK(near_value(motion.output.right_speed_mps, 0.0));
    CHECK(MOTION_OK == motion_step(&motion, 35U, 0.005, -1, 1, &output));
    CHECK(near_value(output.left_speed_mps, -0.2));
    CHECK(near_value(output.right_speed_mps, 0.2));
    CHECK(MOTION_OK == motion_submit(&motion, &halt, 36U));
    CHECK(near_value(motion.output.left_speed_mps, 0.0));
    CHECK(!motion.output.b_stationary);
    CHECK(MOTION_IDLE == motion.output.state);
}

/** @brief Actual elapsed time, short samples and invalid dt remain bounded. */
static void test_motion_speed_intervals (void)
{
    const core_real_t invalid_dt[] = {0.0, -0.005, NAN, INFINITY, 0.025001};
    motion_t motion;
    motion_config_t config = motion_fixture();
    motion_output_t output;
    uint32_t sample = 0U;
    uint32_t index = 0U;
    int32_t delta = 0;

    config.wheel_pid.max_dt_s = 0.025;
    CHECK(MOTION_OK == motion_init(&motion, &config));
    for (sample = 1U; sample <= 500U; sample += 1U)
    {
        /* A sample-count ring would incorrectly shrink to a few ms here. */
        delta = (0U == (sample % 100U)) ? 1 : 0;
        CHECK(MOTION_OK == motion_step(&motion, sample / 10U, 0.0001,
                                       delta, delta, &output));
    }
    CHECK(near_value(output.left_speed_mps, 0.1));
    CHECK(near_value(output.pose.x_m, 0.005));
    CHECK(MOTION_OK == motion_init(&motion, &config));
    for (sample = 1U; sample <= 5U; sample += 1U)
    {
        CHECK(MOTION_OK == motion_step(&motion, (sample * 10U) - 7U,
                                       0.003, 0, 0, &output));
        CHECK(MOTION_OK == motion_step(&motion, sample * 10U,
                                       0.007, 1, 1, &output));
    }
    CHECK(near_value(output.left_speed_mps, 0.1));
    CHECK(near_value(output.pose.x_m, 0.005));
    CHECK(MOTION_OK == motion_step(&motion, 75U, 0.025, 0, 0, &output));
    CHECK(output.left_speed_mps > 0.0);
    CHECK(MOTION_OK == motion_step(&motion, 100U, 0.025, 0, 0, &output));
    CHECK(near_value(output.left_speed_mps, 0.0));
    for (index = 0U; index < (sizeof(invalid_dt) / sizeof(invalid_dt[0]));
         index += 1U)
    {
        CHECK(MOTION_OK == motion_init(&motion, &config));
        CHECK(MOTION_SENSOR_FAULT == motion_step(&motion, 1U,
                    invalid_dt[index], 1, 1, &output));
        CHECK(MOTION_INHIBITED == output.state);
        CHECK(!output.b_stationary);
        CHECK(near_value(output.left_duty, 0.0));
        CHECK(near_value(output.pose.x_m, 0.0));
        CHECK(near_value(output.signed_distance_m, 0.0));
    }
    config.wheel_pid.max_dt_s = 0.1;
    CHECK(MOTION_OK == motion_init(&motion, &config));
    CHECK(MOTION_OK == motion_step(&motion, 100U, 0.1, 10, 10, &output));
    CHECK(near_value(output.left_speed_mps, 0.1));
    CHECK(near_value(output.pose.x_m, 0.01));
}

/** End of file. */

