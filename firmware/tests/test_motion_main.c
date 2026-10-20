/** @file test_motion_main.c
 * @brief Motion leases, primitives, inhibition and supervisor scenarios.
 * Synthetic encoder/event fixtures do not establish physical accuracy.
 */
#include "test_motion_main.h"

#include "bitpack.h"
#include "motion.h"
#include "vehicle.h"

#include <float.h>
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
/* More primitives than one 8-bit ID cycle (1..255), within a step budget. */
#define TEST_WRAP_PRIMITIVES (300U)
#define TEST_WRAP_LIMIT (8000U)
/* Q7 line steering codes -127..127, as robot_app.c scales them. */
#define TEST_STEER_CODES (127)
/* D5 consumers: robot_config.h ROBOT_TRACK_WIDTH_M, the IMU task's
 * quasistatic speed limit and terrain_default_config's turning threshold.
 */
#define TEST_TRACK_WIDTH_M (0.140)
#define TEST_QUASI_LIMIT_MPS (0.15)
#define TEST_TURNING_RPS (0.25)
#define TEST_BOUND_STEPS (20000U)
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

/* One request shape and the status the original double request gets. */
typedef struct
{
    motion_kind_t   kind;
    core_real_t     forward_mps;
    core_real_t     yaw_rps;
    core_real_t     target;
    motion_status_t expected;
} test_request_case_t;

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
static void test_integrated_sequence (bool b_packed);
static bool integrated_motion_tick (motion_t * p_motion,
                                    const vehicle_output_t * p_output,
                                    uint32_t now_ms);
static bool packed_motion_tick (motion_t * p_motion,
                                const vehicle_output_t * p_output,
                                uint32_t now_ms);
static motion_config_t motion_production (void);
static motion_request_t request_shared (const motion_request_t * p_request);
static motion_request_packed_t request_packed (motion_kind_t kind,
                                               core_real_t forward_mps,
                                               core_real_t yaw_rps,
                                               core_real_t target);
static void request_codes (motion_kind_t kind, core_real_t forward_mps,
                           core_real_t yaw_rps, core_real_t target,
                           int32_t * p_codes);
static int32_t speed_code (core_real_t speed_mps);
static motion_status_t submit_fresh (const motion_config_t * p_config,
                                     const motion_request_t * p_request);
static void test_motion_ids (void);
static void test_request_fields (void);
static void test_request_targets (void);
static void test_request_equivalence (void);
static void test_request_bounded (void);
static void test_request_renewal (void);
static void test_snapshot_pack (void);
static void test_snapshot_speeds (void);
static void test_snapshot_speed_bound (void);
static void test_vehicle_id_wrap (void);
static void test_completion_bound (void);
static uint32_t completion_steps (const motion_request_t * p_request,
                                  int32_t left_delta, int32_t right_delta);
static void test_request_direction (void);
static int32_t renewal_resets (const motion_request_t * p_first,
                               const motion_request_t * p_renewal);

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
    test_integrated_sequence(false);
    test_integrated_sequence(true);
    test_motion_ids();
    test_request_fields();
    test_request_targets();
    test_request_equivalence();
    test_request_bounded();
    test_request_renewal();
    test_snapshot_pack();
    test_snapshot_speeds();
    test_snapshot_speed_bound();
    test_vehicle_id_wrap();
    test_completion_bound();
    test_request_direction();
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
 * The 8-bit ID also wraps (255 -> 1 is newer; 255 after 1 is stale).
 */
static void test_motion_lease (void)
{
    motion_t motion;
    motion_config_t config = motion_fixture();
    motion_request_t request = {UINT8_MAX, MOTION_VELOCITY, 0.2, 0.0, 0.0};
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
    request.command_id = UINT8_MAX;
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
    uint8_t command_id = 0U;
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
 * With b_packed both shared messages are packed as robot_app.c packs them
 * (request: Vehicle -> Motion; snapshot: Motion -> Vehicle), and the same
 * scenario must end the same way.
 */
static void test_integrated_sequence (bool b_packed)
{
    motion_t motion;
    vehicle_t vehicle;
    motion_config_t motion_config = motion_fixture();
    vehicle_config_t config = vehicle_fixture();
    vehicle_input_t input = vehicle_inputs();
    vehicle_output_t output;
    motion_snapshot_t snapshot;
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
    memset(&snapshot, 0, sizeof(snapshot));
    input.b_bypass_clearance = true;
    input.bypass_side = 1;
    input.bypass_offset_m = 0.3;
    input.bypass_length_m = 0.7;
    for (sample = 1U; sample <= TEST_INTEGRATION_LIMIT; sample += 1U)
    {
        input.motion = motion.output;
        if (b_packed)
        {
            motion_snapshot_pack(&motion.output, &snapshot);
            motion_snapshot_unpack(&snapshot, &input.motion);
        }
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
            (!(b_packed ?
               packed_motion_tick(&motion, &output, sample * 10U) :
               integrated_motion_tick(&motion, &output, sample * 10U))))
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

/** @brief integrated_motion_tick through the shared request message:
 * Vehicle packs its own request and Motion submits the unpacked copy, as
 * robot_app_vehicle_step and robot_app_motion_step do.
 */
static bool packed_motion_tick (motion_t * p_motion,
                                const vehicle_output_t * p_output,
                                uint32_t now_ms)
{
    vehicle_output_t shared = *p_output;
    motion_request_packed_t packed;

    memset(&packed, 0, sizeof(packed));
    motion_request_pack(&p_output->request, &packed);
    motion_request_unpack(&packed, &shared.request);
    return integrated_motion_tick(p_motion, &shared, now_ms);
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

/** @brief Production limits (app_cfg.c with robot_config.h): 0.15 m/s,
 * 1.0 rad/s, 0.005 m and 0.03 rad tolerances, 140 mm track. Saturated
 * request fields stay rejected only below 0.255 m/s and 1.023 rad/s, so
 * equivalence is shown on these limits, not on the wide fixture.
 */
static motion_config_t motion_production (void)
{
    motion_config_t config;

    memset(&config, 0, sizeof(config));
    config.odometry.left_radius_m = 0.0325;
    config.odometry.right_radius_m = 0.0325;
    config.odometry.track_width_m = 0.140;
    config.odometry.left_counts_per_revolution = 360U;
    config.odometry.right_counts_per_revolution = 360U;
    config.wheel_pid.proportional_gain = 1.0;
    config.wheel_pid.integral_gain = 0.3;
    config.wheel_pid.derivative_gain = 0.0;
    config.wheel_pid.output_min = -0.35;
    config.wheel_pid.output_max = 0.35;
    config.wheel_pid.max_dt_s = 0.025;
    config.max_wheel_mps = 0.15;
    config.max_yaw_rps = 1.0;
    config.distance_tolerance_m = 0.005;
    config.turn_tolerance_rad = 0.03;
    config.stationary_mps = 0.012;
    config.primitive_timeout_ms = 15000U;
    config.b_calibration_confirmed = true;
    return config;
}

/** @brief The request Motion receives: the shared message, unpacked. */
static motion_request_t request_shared (const motion_request_t * p_request)
{
    motion_request_packed_t packed;
    motion_request_t shared;

    memset(&packed, 0, sizeof(packed));
    memset(&shared, 0, sizeof(shared));
    motion_request_pack(p_request, &packed);
    motion_request_unpack(&packed, &shared);
    return shared;
}

/** @brief Pack one request with ID 1 and return the shared record. */
static motion_request_packed_t request_packed (motion_kind_t kind,
                                               core_real_t forward_mps,
                                               core_real_t yaw_rps,
                                               core_real_t target)
{
    motion_request_t request = {1U, MOTION_VELOCITY, 0.0, 0.0, 0.0};
    motion_request_packed_t packed;

    memset(&packed, 0, sizeof(packed));
    request.kind = kind;
    request.forward_mps = forward_mps;
    request.yaw_rps = yaw_rps;
    request.target = target;
    motion_request_pack(&request, &packed);
    return packed;
}

/** @brief Signed forward, yaw and target codes of one packed request. */
static void request_codes (motion_kind_t kind, core_real_t forward_mps,
                           core_real_t yaw_rps, core_real_t target,
                           int32_t * p_codes)
{
    motion_request_packed_t packed =
        request_packed(kind, forward_mps, yaw_rps, target);

    p_codes[0] = bitpack_extend(packed.forward_mm_s, MOTION_FORWARD_BITS);
    p_codes[1] = bitpack_extend(packed.yaw_mrad_s, MOTION_YAW_BITS);
    p_codes[2] = bitpack_extend(packed.target, MOTION_TARGET_BITS);
}

/** @brief Signed snapshot code of one speed on both wheels; INT32_MIN if
 * the two wheels disagree.
 */
static int32_t speed_code (core_real_t speed_mps)
{
    motion_output_t output;
    motion_snapshot_t snapshot;
    int32_t left = 0;

    memset(&output, 0, sizeof(output));
    memset(&snapshot, 0, sizeof(snapshot));
    output.left_speed_mps = speed_mps;
    output.right_speed_mps = speed_mps;
    motion_snapshot_pack(&output, &snapshot);
    left = bitpack_extend(snapshot.left_mm_s, MOTION_SPEED_BITS);
    return (left == bitpack_extend(snapshot.right_mm_s, MOTION_SPEED_BITS)) ?
           left : INT32_MIN;
}

/** @brief Status of the first submit to a freshly initialised owner;
 * STOPPED (never expected here) if the configuration is refused.
 */
static motion_status_t submit_fresh (const motion_config_t * p_config,
                                     const motion_request_t * p_request)
{
    motion_t motion;

    if (MOTION_OK != motion_init(&motion, p_config))
    {
        return MOTION_STOPPED;
    }
    return motion_submit(&motion, p_request, 0U);
}

/** @brief 8-bit IDs: serial order modulo 256 with a 128 half-range; 0 is
 * reserved; a same-ID renewal keeps the running primitive.
 */
static void test_motion_ids (void)
{
    motion_t motion;
    motion_config_t config = motion_fixture();
    motion_request_t request = {200U, MOTION_VELOCITY, 0.1, 0.0, 0.0};

    CHECK(MOTION_OK == motion_init(&motion, &config));
    CHECK(MOTION_OK == motion_submit(&motion, &request, 0U));
    CHECK(MOTION_OK == motion_submit(&motion, &request, 5U));
    CHECK((0U == motion.started_ms) && (5U == motion.renewed_ms));
    /* 72 - 200 = 128 (mod 256): exactly half-range, so stale. */
    request.command_id = 72U;
    CHECK(MOTION_REJECTED == motion_submit(&motion, &request, 10U));
    CHECK(200U == motion.output.command_id);
    /* 71 - 200 = 127 (mod 256): the newest ID still accepted. */
    request.command_id = 71U;
    CHECK(MOTION_OK == motion_submit(&motion, &request, 15U));
    CHECK((71U == motion.output.command_id) && (15U == motion.started_ms));
    /* 200 is now 129 ahead of 71: stale. 0 is never accepted. */
    request.command_id = 200U;
    CHECK(MOTION_REJECTED == motion_submit(&motion, &request, 20U));
    request.command_id = 0U;
    CHECK(MOTION_INVALID == motion_submit(&motion, &request, 25U));
    /* 71 -> 198 -> 255 -> 1: the wrap skips 0 and is two ahead. */
    request.command_id = 198U;
    CHECK(MOTION_OK == motion_submit(&motion, &request, 30U));
    request.command_id = 255U;
    CHECK(MOTION_OK == motion_submit(&motion, &request, 35U));
    request.command_id = 1U;
    CHECK(MOTION_OK == motion_submit(&motion, &request, 40U));
    CHECK((1U == motion.output.command_id) && (40U == motion.started_ms));
    /* 255 after 1 is 254 ahead: stale. Renewing 1 still works. */
    request.command_id = 255U;
    CHECK(MOTION_REJECTED == motion_submit(&motion, &request, 45U));
    request.command_id = 1U;
    CHECK(MOTION_OK == motion_submit(&motion, &request, 50U));
    CHECK((40U == motion.started_ms) && (50U == motion.renewed_ms));
    CHECK(MOTION_RUNNING == motion.output.state);
}

/** @brief Speed fields: magnitudes round up (AWAY), saturate symmetrically
 * at +/-255 mm/s and +/-1023 mrad/s, non-finite takes the reserved code
 * and decodes as NAN; every ID and kind copies through unchanged.
 */
static void test_request_fields (void)
{
    const motion_kind_t kinds[] =
    {
        MOTION_VELOCITY, MOTION_DISTANCE, MOTION_TURN, MOTION_HALT
    };
    const core_real_t tiny = nextafter(0.0, INFINITY);
    const int32_t forward_reserved = bitpack_extend(
        bitpack_invalid(MOTION_FORWARD_BITS), MOTION_FORWARD_BITS);
    const int32_t yaw_reserved = bitpack_extend(
        bitpack_invalid(MOTION_YAW_BITS), MOTION_YAW_BITS);
    motion_request_t request = {255U, MOTION_DISTANCE, 0.255, -1.023, 1.0};
    motion_request_t shared = request_shared(&request);
    int32_t codes[3] = {0, 0, 0};
    uint32_t index = 0U;
    bool b_copied = true;

    CHECK((255U == shared.command_id) && (MOTION_DISTANCE == shared.kind));
    CHECK(near_value(shared.forward_mps, 0.255));
    CHECK(near_value(shared.yaw_rps, -1.023));
    CHECK(near_value(shared.target, 1.0));
    request_codes(MOTION_VELOCITY, 0.0804, -0.6501, 0.0, codes);
    CHECK((81 == codes[0]) && (-651 == codes[1]));
    request_codes(MOTION_VELOCITY, tiny, -tiny, 0.0, codes);
    CHECK((1 == codes[0]) && (-1 == codes[1]));
    request_codes(MOTION_VELOCITY, 0.0, 0.0, 0.0, codes);
    CHECK((0 == codes[0]) && (0 == codes[1]));
    request_codes(MOTION_VELOCITY, 0.08, 0.65, 0.0, codes);
    CHECK((80 == codes[0]) && (650 == codes[1]));
    request_codes(MOTION_VELOCITY, 0.2551, -1.0231, 0.0, codes);
    CHECK((255 == codes[0]) && (-1023 == codes[1]));
    request_codes(MOTION_VELOCITY, -DBL_MAX, DBL_MAX, 0.0, codes);
    CHECK((-255 == codes[0]) && (1023 == codes[1]));
    request_codes(MOTION_VELOCITY, NAN, INFINITY, 0.0, codes);
    CHECK((forward_reserved == codes[0]) && (yaw_reserved == codes[1]));
    request.forward_mps = -INFINITY;
    request.yaw_rps = NAN;
    shared = request_shared(&request);
    CHECK(isnan(shared.forward_mps) && isnan(shared.yaw_rps));
    for (index = 0U; index <= UINT8_MAX; index += 1U)
    {
        /* index <= 255, so the narrowing is exact. */
        request.command_id = (uint8_t) index;
        shared = request_shared(&request);
        b_copied = b_copied && (index == shared.command_id);
    }
    for (index = 0U; index < (sizeof(kinds) / sizeof(kinds[0])); index += 1U)
    {
        request.kind = kinds[index];
        shared = request_shared(&request);
        b_copied = b_copied && (kinds[index] == shared.kind);
    }
    CHECK(b_copied);
}

/** @brief Targets: DISTANCE mm and TURN mrad with AWAY rounding, a TURN
 * within 2*pi capped at 6283 mrad, +/-8191 saturation, 0 for HALT and a
 * finite VELOCITY, and the reserved code for other non-finite targets.
 */
static void test_request_targets (void)
{
    const int32_t reserved = bitpack_extend(
        bitpack_invalid(MOTION_TARGET_BITS), MOTION_TARGET_BITS);
    const core_real_t over_turn = nextafter(2.0 * CORE_PI, INFINITY);
    motion_request_t request = {3U, MOTION_DISTANCE, 0.08, 0.0, 8.191};
    motion_request_t shared = request_shared(&request);
    motion_request_packed_t packed =
        request_packed(MOTION_HALT, 0.0, 0.0, 0.0);
    int32_t codes[3] = {0, 0, 0};

    CHECK(near_value(shared.target, 8.191));
    request_codes(MOTION_DISTANCE, 0.08, 0.0, 0.060, codes);
    CHECK(60 == codes[2]);
    request_codes(MOTION_DISTANCE, 0.08, 0.0, 0.0601, codes);
    CHECK(61 == codes[2]);
    request_codes(MOTION_DISTANCE, 0.08, 0.0, -0.0601, codes);
    CHECK(-61 == codes[2]);
    request_codes(MOTION_DISTANCE, 0.08, 0.0, 8.1911, codes);
    CHECK(8191 == codes[2]);
    request_codes(MOTION_DISTANCE, 0.08, 0.0, -100.0, codes);
    CHECK(-8191 == codes[2]);
    request_codes(MOTION_DISTANCE, 0.08, 0.0, NAN, codes);
    CHECK(reserved == codes[2]);
    request_codes(MOTION_TURN, 0.0, 0.65, 0.5 * CORE_PI, codes);
    CHECK(1571 == codes[2]);
    request_codes(MOTION_TURN, 0.0, 0.65, -CORE_PI, codes);
    CHECK(-3142 == codes[2]);
    request_codes(MOTION_TURN, 0.0, 0.65, 2.0 * CORE_PI, codes);
    CHECK(6283 == codes[2]);
    request_codes(MOTION_TURN, 0.0, 0.65, -2.0 * CORE_PI, codes);
    CHECK(-6283 == codes[2]);
    request_codes(MOTION_TURN, 0.0, 0.65, over_turn, codes);
    CHECK(6284 == codes[2]);
    request_codes(MOTION_TURN, 0.0, 0.65, 9.0, codes);
    CHECK(8191 == codes[2]);
    request_codes(MOTION_TURN, 0.0, 0.65, -INFINITY, codes);
    CHECK(reserved == codes[2]);
    request_codes(MOTION_HALT, 0.0, 0.0, 5.0, codes);
    CHECK(0 == codes[2]);
    request_codes(MOTION_HALT, 0.0, 0.0, NAN, codes);
    CHECK(0 == codes[2]);
    request_codes(MOTION_VELOCITY, 0.0, 0.0, 5.0, codes);
    CHECK(0 == codes[2]);
    request_codes(MOTION_VELOCITY, 0.0, 0.0, NAN, codes);
    CHECK(reserved == codes[2]);
    request.target = NAN;
    shared = request_shared(&request);
    CHECK(isnan(shared.target));
    /* Two bits would alias kind 4 onto VELOCITY: reserved target instead. */
    packed = request_packed((motion_kind_t) 4, 0.08, 0.0, 0.0);
    CHECK((MOTION_VELOCITY == packed.kind) &&
          (bitpack_invalid(MOTION_TARGET_BITS) == packed.target));
    /* NULL arguments have no effect. */
    shared.target = 9.0;
    motion_request_unpack(NULL, &shared);
    CHECK(near_value(shared.target, 9.0));
    motion_request_unpack(&packed, NULL);
    packed = request_packed(MOTION_TURN, 0.0, 0.65, CORE_PI);
    motion_request_pack(NULL, &packed);
    CHECK((MOTION_TURN == packed.kind) && (650U == packed.yaw_mrad_s));
    motion_request_pack(&request, NULL);
}

/** @brief Under production limits the unpacked copy gets exactly the
 * original's submit status at every validity threshold, its double
 * neighbours, saturated and non-finite values (spec rule 5).
 */
static void test_request_equivalence (void)
{
    const motion_config_t config = motion_production();
    const test_request_case_t cases[] =
    {
        {MOTION_VELOCITY, 0.0, 1.0, 0.0, MOTION_OK},
        {MOTION_VELOCITY, 0.0, nextafter(1.0, INFINITY), 0.0, MOTION_INVALID},
        {MOTION_VELOCITY, 0.0, -1.0, 0.0, MOTION_OK},
        {MOTION_VELOCITY, 0.0, nextafter(-1.0, -INFINITY), 0.0,
         MOTION_INVALID},
        {MOTION_VELOCITY, 0.0, 1.0004, 0.0, MOTION_INVALID},
        {MOTION_VELOCITY, 0.15, 0.0, 0.0, MOTION_OK},
        {MOTION_VELOCITY, nextafter(0.15, INFINITY), 0.0, 0.0,
         MOTION_INVALID},
        {MOTION_VELOCITY, -0.15, 0.0, 0.0, MOTION_OK},
        {MOTION_VELOCITY, 0.1504, 0.0, 0.0, MOTION_INVALID},
        {MOTION_VELOCITY, 0.08, 0.65, 0.0, MOTION_OK},
        {MOTION_VELOCITY, 1.0, 0.0, 0.0, MOTION_INVALID},
        {MOTION_VELOCITY, -DBL_MAX, 0.0, 0.0, MOTION_INVALID},
        {MOTION_VELOCITY, 0.0, 5.0, 0.0, MOTION_INVALID},
        {MOTION_VELOCITY, NAN, 0.0, 0.0, MOTION_INVALID},
        {MOTION_VELOCITY, 0.0, INFINITY, 0.0, MOTION_INVALID},
        {MOTION_VELOCITY, 0.08, 0.0, NAN, MOTION_INVALID},
        {MOTION_DISTANCE, 0.08, 0.0, 0.005, MOTION_INVALID},
        {MOTION_DISTANCE, 0.08, 0.0, nextafter(0.005, INFINITY), MOTION_OK},
        {MOTION_DISTANCE, 0.08, 0.0, -0.005, MOTION_INVALID},
        {MOTION_DISTANCE, 0.08, 0.0, nextafter(-0.005, -INFINITY),
         MOTION_OK},
        {MOTION_DISTANCE, 0.08, 0.0, 0.0054, MOTION_OK},
        {MOTION_DISTANCE, 0.08, 0.0, 0.0049996, MOTION_INVALID},
        {MOTION_DISTANCE, nextafter(0.0, INFINITY), 0.0, 0.06, MOTION_OK},
        {MOTION_DISTANCE, 0.0, 0.0, 0.06, MOTION_INVALID},
        {MOTION_DISTANCE, -0.08, 0.0, 0.06, MOTION_INVALID},
        {MOTION_DISTANCE, 0.08, 0.0, 0.60, MOTION_OK},
        {MOTION_DISTANCE, 0.08, 0.0, 1.20, MOTION_OK},
        {MOTION_DISTANCE, 0.08, 0.0, 100.0, MOTION_OK},
        {MOTION_TURN, 0.0, 0.65, 0.03, MOTION_INVALID},
        {MOTION_TURN, 0.0, 0.65, nextafter(0.03, INFINITY), MOTION_OK},
        {MOTION_TURN, 0.0, 0.65, 0.0304, MOTION_OK},
        {MOTION_TURN, 0.0, 0.65, -0.5 * CORE_PI, MOTION_OK},
        {MOTION_TURN, 0.0, 0.65, CORE_PI, MOTION_OK},
        {MOTION_TURN, 0.0, 0.65, 2.0 * CORE_PI, MOTION_OK},
        {MOTION_TURN, 0.0, 0.65, nextafter(2.0 * CORE_PI, INFINITY),
         MOTION_INVALID},
        {MOTION_TURN, 0.0, 0.65, -2.0 * CORE_PI, MOTION_OK},
        {MOTION_TURN, 0.0, 0.65, nextafter(-2.0 * CORE_PI, -INFINITY),
         MOTION_INVALID},
        {MOTION_TURN, 0.0, 0.65, 6.2834, MOTION_INVALID},
        {MOTION_TURN, 0.0, 0.65, 9.0, MOTION_INVALID},
        {MOTION_TURN, 0.0, nextafter(0.0, INFINITY), CORE_PI, MOTION_OK},
        {MOTION_TURN, 0.0, -0.65, CORE_PI, MOTION_INVALID},
        {MOTION_TURN, 0.0, 0.0, CORE_PI, MOTION_INVALID},
        {MOTION_HALT, NAN, NAN, NAN, MOTION_OK},
        {(motion_kind_t) 4, 0.08, 0.0, 0.0, MOTION_INVALID}
    };
    motion_request_t request = {1U, MOTION_VELOCITY, 0.0, 0.0, 0.0};
    motion_request_t shared = request;
    unsigned int index = 0U;
    bool b_match = true;

    for (index = 0U; index < (sizeof(cases) / sizeof(cases[0])); index += 1U)
    {
        request.kind = cases[index].kind;
        request.forward_mps = cases[index].forward_mps;
        request.yaw_rps = cases[index].yaw_rps;
        request.target = cases[index].target;
        shared = request_shared(&request);
        b_match = (cases[index].expected ==
                   submit_fresh(&config, &request)) &&
                  (cases[index].expected == submit_fresh(&config, &shared));
        CHECK(b_match);
        if (!b_match)
        {
            /* Diagnostic only: the failure is already counted. */
            (void) fprintf(stderr, "request case %u\n", index);
        }
    }
    request.kind = MOTION_DISTANCE;
    request.forward_mps = 0.08;
    request.target = 0.06;
    request.command_id = 0U;
    shared = request_shared(&request);
    CHECK((MOTION_INVALID == submit_fresh(&config, &request)) &&
          (MOTION_INVALID == submit_fresh(&config, &shared)));
}

/** @brief The bounded side of AWAY (INTEGRATION.md (d)): a valid request
 * within one unit of a limit may become INVALID, never the reverse; every
 * production follow request is far from it.
 */
static void test_request_bounded (void)
{
    const motion_config_t config = motion_production();
    motion_request_t request = {1U, MOTION_VELOCITY, 0.0795, 1.0, 0.0};
    motion_request_t shared = request_shared(&request);
    core_real_t line = 0.0;
    int32_t code = 0;
    bool b_valid = true;

    /* Envelope: 0.0795 + 0.07 * 1.0 < 0.15, but 0.080 + 0.070 > 0.15. */
    CHECK(MOTION_OK == submit_fresh(&config, &request));
    CHECK(MOTION_INVALID == submit_fresh(&config, &shared));
    /* A cross component in (0, 1e-12) becomes one whole unit. */
    request.kind = MOTION_DISTANCE;
    request.forward_mps = 0.08;
    request.yaw_rps = 1.0e-13;
    request.target = 0.06;
    shared = request_shared(&request);
    CHECK(MOTION_OK == submit_fresh(&config, &request));
    CHECK(MOTION_INVALID == submit_fresh(&config, &shared));
    request.kind = MOTION_TURN;
    request.forward_mps = 1.0e-13;
    request.yaw_rps = 0.65;
    request.target = 0.5 * CORE_PI;
    shared = request_shared(&request);
    CHECK(MOTION_OK == submit_fresh(&config, &request));
    CHECK(MOTION_INVALID == submit_fresh(&config, &shared));
    /* robot_app_vehicle_input: Q7 steering, 0.08 m/s, gain 0.75 rad/s,
     * clamp 0.65 rad/s. Its envelope peaks at 0.0908 m/s, so every follow
     * request stays valid either way.
     */
    request.kind = MOTION_VELOCITY;
    request.target = 0.0;
    for (code = -TEST_STEER_CODES; code <= TEST_STEER_CODES; code += 1)
    {
        line = (core_real_t) code / 127.0;
        request.forward_mps = 0.08 * (1.0 - (0.5 * fabs(line)));
        request.yaw_rps = -fmax(-0.65, fmin(0.65, line * 0.75));
        shared = request_shared(&request);
        b_valid = b_valid && (MOTION_OK == submit_fresh(&config, &request)) &&
                  (MOTION_OK == submit_fresh(&config, &shared));
    }
    CHECK(b_valid);
}

/** @brief Renewals through the shared message stay one primitive: a
 * producer request always packs to the same bits, and Motion compares
 * decoded values with decoded values (robot_app.c submits only those).
 */
static void test_request_renewal (void)
{
    const motion_config_t config = motion_production();
    motion_t motion;
    motion_request_t request = {7U, MOTION_TURN, 0.0, 0.65, 0.5 * CORE_PI};
    motion_request_t shared = request_shared(&request);
    motion_request_packed_t first =
        request_packed(MOTION_TURN, 0.0, 0.65, 0.5 * CORE_PI);
    motion_request_packed_t second =
        request_packed(MOTION_TURN, 0.0, 0.65, 0.5 * CORE_PI);

    CHECK((first.forward_mm_s == second.forward_mm_s) &&
          (first.yaw_mrad_s == second.yaw_mrad_s) &&
          (first.command_id == second.command_id) &&
          (first.kind == second.kind) && (first.target == second.target));
    CHECK(MOTION_OK == motion_init(&motion, &config));
    CHECK(MOTION_OK == motion_submit(&motion, &shared, 0U));
    shared = request_shared(&request);
    CHECK(MOTION_OK == motion_submit(&motion, &shared, 10U));
    CHECK((MOTION_RUNNING == motion.output.state) &&
          (0U == motion.started_ms) && (10U == motion.renewed_ms));
    /* Mixing paths is no renewal: 1.571 differs from pi/2 by 2e-4. */
    CHECK(MOTION_REJECTED == motion_submit(&motion, &request, 20U));
    /* An off-grid DISTANCE (a computed bypass length) renews the same way.
     */
    request.command_id = 8U;
    request.kind = MOTION_DISTANCE;
    request.forward_mps = 0.08;
    request.yaw_rps = 0.0;
    request.target = 0.3004;
    shared = request_shared(&request);
    CHECK(MOTION_OK == motion_submit(&motion, &shared, 30U));
    shared = request_shared(&request);
    CHECK(MOTION_OK == motion_submit(&motion, &shared, 40U));
    CHECK((30U == motion.started_ms) && (40U == motion.renewed_ms));
    CHECK(near_value(motion.request.target, 0.301));
    /* A same-ID VELOCITY renewal may still change its speed. */
    request.command_id = 9U;
    request.kind = MOTION_VELOCITY;
    request.forward_mps = 0.05;
    request.target = 0.0;
    shared = request_shared(&request);
    CHECK(MOTION_OK == motion_submit(&motion, &shared, 50U));
    request.forward_mps = 0.0604;
    shared = request_shared(&request);
    CHECK(MOTION_OK == motion_submit(&motion, &shared, 60U));
    CHECK(near_value(motion.request.forward_mps, 0.061));
}

/** @brief Snapshot: every state and reason, the 8-bit ID, the flag and the
 * pose round-trip; duties, centre path and step distance unpack as 0.
 */
static void test_snapshot_pack (void)
{
    const motion_state_t states[] =
    {
        MOTION_IDLE, MOTION_RUNNING, MOTION_COMPLETE, MOTION_INHIBITED
    };
    const motion_status_t reasons[] =
    {
        MOTION_OK, MOTION_INVALID, MOTION_REJECTED, MOTION_EXPIRED,
        MOTION_TIMED_OUT, MOTION_SENSOR_FAULT, MOTION_STOPPED
    };
    motion_output_t output;
    motion_output_t shared;
    motion_snapshot_t snapshot;
    uint32_t state = 0U;
    uint32_t reason = 0U;
    bool b_enums = true;

    memset(&output, 0, sizeof(output));
    memset(&shared, 0, sizeof(shared));
    memset(&snapshot, 0, sizeof(snapshot));
    output.command_id = 255U;
    output.pose.x_m = 1.2344;
    output.pose.y_m = -0.5;
    output.pose.yaw_rad = 0.7854;
    output.pose.centre_path_m = 12.0;
    output.left_duty = 0.3;
    output.right_duty = -0.2;
    output.left_speed_mps = 0.1234;
    output.right_speed_mps = -0.0456;
    output.signed_distance_m = 0.002;
    output.b_stationary = true;
    for (state = 0U; state < (sizeof(states) / sizeof(states[0])); ++state)
    {
        for (reason = 0U; reason < (sizeof(reasons) / sizeof(reasons[0]));
             ++reason)
        {
            output.state = states[state];
            output.reason = reasons[reason];
            motion_snapshot_pack(&output, &snapshot);
            motion_snapshot_unpack(&snapshot, &shared);
            b_enums = b_enums && (states[state] == shared.state) &&
                      (reasons[reason] == shared.reason);
        }
    }
    CHECK(b_enums);
    CHECK((255U == shared.command_id) && shared.b_stationary);
    CHECK(near_value(shared.pose.x_m, 1.234));
    CHECK(near_value(shared.pose.y_m, -0.5));
    CHECK(near_value(shared.pose.yaw_rad, 0.785));
    CHECK(near_value(shared.left_speed_mps, 0.123));
    CHECK(near_value(shared.right_speed_mps, -0.046));
    CHECK(near_value(shared.left_duty, 0.0));
    CHECK(near_value(shared.right_duty, 0.0));
    CHECK(near_value(shared.pose.centre_path_m, 0.0));
    CHECK(near_value(shared.signed_distance_m, 0.0));
    output.command_id = 0U;
    output.b_stationary = false;
    output.pose.yaw_rad = NAN;
    motion_snapshot_pack(&output, &snapshot);
    motion_snapshot_unpack(&snapshot, &shared);
    CHECK((0U == shared.command_id) && (!shared.b_stationary));
    CHECK(isnan(shared.pose.x_m) && isnan(shared.pose.yaw_rad));
    /* NULL arguments have no effect. */
    motion_snapshot_unpack(NULL, &shared);
    CHECK(0U == shared.command_id);
    motion_snapshot_unpack(&snapshot, NULL);
    motion_snapshot_pack(NULL, &snapshot);
    motion_snapshot_pack(&output, NULL);
    CHECK(0U == snapshot.command_id);
}

/** @brief Wheel speeds: nearest mm/s, symmetric +/-511 saturation, and the
 * reserved code for non-finite speeds, which decodes as NAN.
 */
/** @brief D5: the IMU task's two consumers of the packed wheel speeds,
 * |mean - imu| < 0.15 m/s and |(vr - vl) / 0.14| > 0.25 rad/s, can change
 * only inside the rounding band (0.5 mm/s and 1 mm/s / 0.14 m), in either
 * direction. Each sweep must cross its threshold, so the test is not
 * vacuous.
 */
static void test_snapshot_speed_bound (void)
{
    const core_real_t imu_mps = 0.05;
    const core_real_t yaw_band = (0.001 / TEST_TRACK_WIDTH_M) + 1e-12;
    motion_output_t raw;
    motion_output_t decoded;
    motion_snapshot_t snapshot;
    core_real_t raw_value = 0.0;
    core_real_t packed_value = 0.0;
    uint32_t step = 0U;
    uint32_t seen = 0U;
    bool b_bounded = true;

    memset(&raw, 0, sizeof(raw));
    for (step = 0U; step <= TEST_BOUND_STEPS; ++step)
    {
        /* Mean speed crosses imu + 0.15 with unequal wheels. */
        raw.left_speed_mps = 0.1948 + ((0.01 * step) / TEST_BOUND_STEPS);
        raw.right_speed_mps = raw.left_speed_mps + 0.0003;
        motion_snapshot_pack(&raw, &snapshot);
        motion_snapshot_unpack(&snapshot, &decoded);
        raw_value = ((raw.left_speed_mps + raw.right_speed_mps) / 2.0) -
                    imu_mps;
        packed_value = ((decoded.left_speed_mps + decoded.right_speed_mps) /
                        2.0) - imu_mps;
        seen |= (fabs(raw_value) < TEST_QUASI_LIMIT_MPS) ? 1U : 2U;
        if ((fabs(raw_value) < TEST_QUASI_LIMIT_MPS) !=
            (fabs(packed_value) < TEST_QUASI_LIMIT_MPS))
        {
            b_bounded = b_bounded &&
                (fabs(fabs(raw_value) - TEST_QUASI_LIMIT_MPS) <=
                 (0.0005 + 1e-12));
        }
        /* Yaw rate crosses +0.25 and -0.25 rad/s from a moving base. */
        raw.left_speed_mps = 0.0004 + ((0.05 * step) / TEST_BOUND_STEPS);
        raw.right_speed_mps = raw.left_speed_mps + 0.0340 +
            ((0.002 * step) / TEST_BOUND_STEPS);
        if (0U != (step & 1U))
        {
            raw.right_speed_mps = raw.left_speed_mps -
                (raw.right_speed_mps - raw.left_speed_mps);
        }
        motion_snapshot_pack(&raw, &snapshot);
        motion_snapshot_unpack(&snapshot, &decoded);
        raw_value = (raw.right_speed_mps - raw.left_speed_mps) /
                    TEST_TRACK_WIDTH_M;
        packed_value = (decoded.right_speed_mps - decoded.left_speed_mps) /
                       TEST_TRACK_WIDTH_M;
        seen |= (fabs(raw_value) > TEST_TURNING_RPS) ? 4U : 8U;
        if ((fabs(raw_value) > TEST_TURNING_RPS) !=
            (fabs(packed_value) > TEST_TURNING_RPS))
        {
            b_bounded = b_bounded &&
                (fabs(fabs(raw_value) - TEST_TURNING_RPS) <= yaw_band);
        }
    }
    CHECK(15U == seen);
    CHECK(b_bounded);
}

static void test_snapshot_speeds (void)
{
    const int32_t reserved = bitpack_extend(
        bitpack_invalid(MOTION_SPEED_BITS), MOTION_SPEED_BITS);
    motion_output_t output;
    motion_snapshot_t snapshot;

    CHECK(123 == speed_code(0.1234));
    CHECK(124 == speed_code(0.1236));
    CHECK(-123 == speed_code(-0.1234));
    CHECK(-124 == speed_code(-0.1236));
    CHECK(0 == speed_code(-0.0004));
    CHECK(150 == speed_code(0.15));
    CHECK(511 == speed_code(0.511));
    CHECK(511 == speed_code(0.5114));
    CHECK(511 == speed_code(0.5116));
    CHECK(-511 == speed_code(-0.6));
    CHECK(511 == speed_code(DBL_MAX));
    CHECK(reserved == speed_code(NAN));
    CHECK(reserved == speed_code(-INFINITY));
    memset(&output, 0, sizeof(output));
    memset(&snapshot, 0, sizeof(snapshot));
    output.left_speed_mps = INFINITY;
    output.right_speed_mps = 0.511;
    motion_snapshot_pack(&output, &snapshot);
    output.left_speed_mps = 0.0;
    motion_snapshot_unpack(&snapshot, &output);
    CHECK(isnan(output.left_speed_mps));
    CHECK(near_value(output.right_speed_mps, 0.511));
}

/** @brief Over 255 primitives through Vehicle, both shared messages and
 * Motion: STRAIGHT junction cycles issue a VELOCITY and a DISTANCE ID each.
 * IDs run 1..255 and wrap to 1, never 0; Motion accepts every one, and
 * completion still matches on the 8-bit ID after the wrap.
 */
static void test_vehicle_id_wrap (void)
{
    motion_t motion;
    vehicle_t vehicle;
    motion_config_t motion_config = motion_fixture();
    vehicle_config_t config = vehicle_fixture();
    vehicle_input_t input = vehicle_inputs();
    vehicle_output_t output;
    motion_snapshot_t snapshot;
    uint32_t sample = 0U;
    uint32_t issued = 0U;
    uint32_t approaches = 0U;
    uint8_t last_id = 0U;
    bool b_ok = true;
    bool b_zero = false;
    bool b_wrapped = false;

    /* A 20 mm approach completes in about ten 10 ms samples. */
    config.junction_distance_m = 0.02;
    CHECK(MOTION_OK == motion_init(&motion, &motion_config));
    CHECK(vehicle_init(&vehicle, &config));
    memset(&output, 0, sizeof(output));
    memset(&snapshot, 0, sizeof(snapshot));
    for (sample = 1U; b_ok && (sample <= TEST_WRAP_LIMIT) &&
                      (issued < TEST_WRAP_PRIMITIVES); sample += 1U)
    {
        motion_snapshot_pack(&motion.output, &snapshot);
        motion_snapshot_unpack(&snapshot, &input.motion);
        input.b_start = (2U == sample);
        input.navigation = ((VEHICLE_FOLLOW == vehicle.state) &&
                            vehicle.b_active &&
                            (VEHICLE_NAV_NONE == vehicle.pending_nav)) ?
                           VEHICLE_NAV_STRAIGHT : VEHICLE_NAV_NONE;
        input.b_junction_edge = (VEHICLE_FOLLOW == vehicle.state) &&
                                (VEHICLE_NAV_NONE != vehicle.pending_nav);
        input.b_original_line_verified =
            (VEHICLE_REACQUIRE == vehicle.state);
        b_ok = vehicle_step(&vehicle, sample * 10U, &input, &output) &&
               (VEHICLE_FAULT != output.state);
        if (output.b_issue_motion && (MOTION_HALT != output.request.kind) &&
            (last_id != output.request.command_id))
        {
            b_zero = b_zero || (0U == output.request.command_id);
            b_wrapped = b_wrapped || ((UINT8_MAX == last_id) &&
                                      (1U == output.request.command_id));
            approaches += (MOTION_DISTANCE == output.request.kind) ? 1U : 0U;
            last_id = output.request.command_id;
            issued += 1U;
        }
        b_ok = b_ok && packed_motion_tick(&motion, &output, sample * 10U);
    }
    CHECK(b_ok);
    CHECK(issued >= TEST_WRAP_PRIMITIVES);
    CHECK(!b_zero);
    CHECK(b_wrapped);
    /* Every second ID was a junction approach that Motion completed. */
    CHECK(approaches >= ((TEST_WRAP_PRIMITIVES / 2U) - 1U));
    CHECK(!motion.b_stop_latched);
}

/** @brief The bounded completion change (INTEGRATION.md (d)) on production
 * geometry, one count per wheel per sample: whole-unit targets finish on
 * the same sample; an off-grid target rounded up finishes at most one
 * sample later, never earlier. pi -> 3142 mrad is the U-turn case.
 */
static void test_completion_bound (void)
{
    motion_request_t request = {1U, MOTION_TURN, 0.0, 0.65, 0.5 * CORE_PI};
    motion_request_t shared = request_shared(&request);
    uint32_t original = completion_steps(&request, -1, 1);

    CHECK((0U != original) && (original == completion_steps(&shared, -1, 1)));
    request.target = -CORE_PI;
    shared = request_shared(&request);
    original = completion_steps(&request, 1, -1);
    CHECK((0U != original) &&
          ((original + 1U) == completion_steps(&shared, 1, -1)));
    request.kind = MOTION_DISTANCE;
    request.forward_mps = 0.08;
    request.yaw_rps = 0.0;
    request.target = 0.060;
    shared = request_shared(&request);
    original = completion_steps(&request, 1, 1);
    CHECK((0U != original) && (original == completion_steps(&shared, 1, 1)));
    request.target = 0.3004;
    shared = request_shared(&request);
    original = completion_steps(&request, 1, 1);
    CHECK((0U != original) &&
          ((original + 1U) == completion_steps(&shared, 1, 1)));
}

/** @brief Samples until a renewed primitive completes under production
 * limits with fixed deltas each 10 ms; 0 if it never completes.
 */
static uint32_t completion_steps (const motion_request_t * p_request,
                                  int32_t left_delta, int32_t right_delta)
{
    const motion_config_t config = motion_production();
    motion_t motion;
    motion_output_t output;
    uint32_t sample = 0U;

    memset(&output, 0, sizeof(output));
    if ((MOTION_OK != motion_init(&motion, &config)) ||
        (MOTION_OK != motion_submit(&motion, p_request, 0U)))
    {
        return 0U;
    }
    for (sample = 1U; sample <= TEST_WRAP_LIMIT; sample += 1U)
    {
        if ((MOTION_OK != motion_submit(&motion, p_request, sample * 10U)) ||
            (MOTION_OK != motion_step(&motion, sample * 10U, 0.01,
                                      left_delta, right_delta, &output)))
        {
            return 0U;
        }
        if (MOTION_COMPLETE == output.state)
        {
            return sample;
        }
    }
    return 0U;
}

/** @brief The bounded direction-change case (INTEGRATION.md (d)): a wheel
 * target within about 1.07 mm/s of zero can change sign class only after
 * quantisation. Its one effect is a reset of the wheel loops and speed
 * window on a same-ID VELOCITY renewal; no safety decision depends on it.
 */
static void test_request_direction (void)
{
    motion_request_t first = {5U, MOTION_VELOCITY, 0.0698, 1.0, 0.0};
    motion_request_t renewal = {5U, MOTION_VELOCITY, 0.0689, 1.0, 0.0};
    motion_request_t first_shared = request_shared(&first);
    motion_request_t renewal_shared = request_shared(&renewal);

    /* Left wheel -0.2 -> -1.1 mm/s: same sign, no reset. Decoded:
     * 0 -> -1 mm/s, which motion_direction_changed treats as a reversal.
     */
    CHECK(0 == renewal_resets(&first, &renewal));
    CHECK(1 == renewal_resets(&first_shared, &renewal_shared));
    /* Away from zero the decoded renewal behaves as the original: the
     * right wheel reverses (+120 -> -5 mm/s), then no wheel does.
     */
    first.forward_mps = 0.05;
    renewal.forward_mps = -0.075;
    first_shared = request_shared(&first);
    renewal_shared = request_shared(&renewal);
    CHECK(1 == renewal_resets(&first, &renewal));
    CHECK(1 == renewal_resets(&first_shared, &renewal_shared));
    renewal.forward_mps = 0.06;
    renewal_shared = request_shared(&renewal);
    CHECK(0 == renewal_resets(&first, &renewal));
    CHECK(0 == renewal_resets(&first_shared, &renewal_shared));
}

/** @brief 1 if a same-ID renewal reset the wheel loops, 0 if it kept them,
 * -1 if it was not an accepted renewal (production limits; one sample
 * between submit and renewal).
 */
static int32_t renewal_resets (const motion_request_t * p_first,
                               const motion_request_t * p_renewal)
{
    const motion_config_t config = motion_production();
    motion_t motion;
    motion_output_t output;

    memset(&output, 0, sizeof(output));
    if ((MOTION_OK != motion_init(&motion, &config)) ||
        (MOTION_OK != motion_submit(&motion, p_first, 0U)) ||
        (MOTION_OK != motion_step(&motion, 5U, 0.005, 0, 0, &output)) ||
        (!motion.left_pid.b_has_previous_measurement) ||
        (MOTION_OK != motion_submit(&motion, p_renewal, 10U)))
    {
        return -1;
    }
    return motion.left_pid.b_has_previous_measurement ? 0 : 1;
}

/** End of file. */

