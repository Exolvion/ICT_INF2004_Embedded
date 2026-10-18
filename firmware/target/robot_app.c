/** @file robot_app.c
 * @brief Single-core RTOS application: sensor owners and one motion writer.
 * Shared records are copied inside bounded interrupt-masked regions. No I2C,
 * formatting, scan, network or floating-point computation occurs while locked.
 * BARR-NUMERIC-01: direct double uses the core.h binary64 precision contract.
 */
#include "robot_app.h"
#include "app_main.h"
#include "app_cfg.h"
#include "board.h"
#include "course_cfg.h"
#include "net_mqtt.h"
#include "robot_config.h"
#include <math.h>
#include <stddef.h>
#include "telemetry.h"
#include <string.h>

#define ROBOT_APP_TASKS (8U)
#define ROBOT_APP_MG_TO_MPS2 (0.00980665)
#define ROBOT_APP_SERIAL_HALF (UINT32_C(2147483648))
#define ROBOT_APP_TILT_TIMEOUT_MS (100U)

typedef struct
{
    motion_output_t  motion;
    line_ctrl_output_t line;
    terrain_output_t terrain;
    scanner_result_t scan;
    core_range_sample_t front;
    motion_request_t request;
    vehicle_state_t vehicle;
    vehicle_nav_t navigation;
    vehicle_scan_t scan_request;
    robot_app_diagnostic_t diagnostic;
    uint32_t health[ROBOT_APP_TASKS];
    uint32_t line_ms;
    uint32_t imu_ms;
    uint32_t tilt_ms;
    uint32_t range_ms;
    uint32_t request_version;
    uint32_t stop_epoch;
    uint32_t reset_epoch;
    uint32_t run_id;
    uint32_t scan_ms;
    int32_t barcode_label;
    barcode_action_t barcode_action;
    core_pose_t front_pose;
    uint32_t barcode_unknown;
    uint32_t barcode_rejected;
    double mission_maximum_m;
    bool b_mission_maximum_valid;
    bool b_junction_edge;
    uint32_t ignored_commands;
    uint32_t dropped_telemetry;
    /* Motion publishes each path with its release stamp, and keeps the
     * previous pair, in one critical section (robot_app_commit).
     */
    double signed_path_m;
    double previous_path_m;
    uint32_t path_ms;
    uint32_t previous_path_ms;
    bool b_inhibit;
    bool b_stop_latched;
    bool b_hard_fault;
    bool b_reset_motion;
    bool b_start;
    bool b_reset;
    bool b_coarse_ready;
    bool b_fine_ready;
} robot_app_shared_t;

static robot_app_shared_t g_shared;
static motion_t g_motion;
static vehicle_t g_vehicle;
static line_ctrl_config_t g_line_config;
static line_ctrl_state_t g_line;
static barcode_config_t g_barcode_config;
static barcode_state_t g_barcode;
static terrain_state_t g_terrain;
static scanner_state_t g_scanner;
static core_pose_t g_route_anchor;
static uint64_t g_motion_us;
static uint32_t g_motion_version;
static uint32_t g_scan_centred_ms;
static uint32_t g_scan_started_ms;
static uint32_t g_left_stall_ms;
static uint32_t g_right_stall_ms;
static uint32_t g_stationary_since_ms;
static uint32_t g_line_left_total;
static uint32_t g_line_right_total;
static uint32_t g_imu_window_ms;
static uint32_t g_imu_run_id;
static double g_imu_window_path_m;
static core_pose_t g_front_trigger_pose;
static uint32_t g_front_trigger_ms;
static core_pose_t g_optical_pose;
static double g_optical_signed_m;
static bool gb_front_pending;
static bool gb_fine_requested;
static uint32_t g_line_lock_ms;
static uint32_t g_button_since_ms;
static uint32_t g_telemetry_sequence;
static double g_imu_path_m;
static double g_imu_speed_mps;
static bool gb_scan_busy;
static bool gb_scan_result_ready;
static bool gb_button_down;
static bool gb_button_latched;
static bool gb_line_lock;

static void robot_app_copy (robot_app_shared_t * p_copy);
static void robot_app_fault (bool b_hard, robot_app_fault_t reason);
static bool robot_app_active (vehicle_state_t state);
static bool robot_app_ready (const robot_app_shared_t * p_copy,
                             uint32_t now_ms);
static bool robot_app_motion_init (void);
static void robot_app_commit (motion_output_t * p_output,
                              double distance_m, uint32_t now_ms);
static void robot_app_motion_health (motion_output_t * p_output,
    int32_t left_delta, int32_t right_delta, uint32_t now_ms);
static void robot_app_reset (const robot_app_shared_t * p_copy);
static int32_t robot_app_signed_count (int32_t count, int32_t sign);
static int32_t robot_app_count_delta (uint32_t newer, uint32_t older);
static bool robot_app_optical_path (void);
static bool robot_app_codebook_ready (void);
static void robot_app_terrain_run (const robot_app_shared_t * p_copy,
                                   double path_m, uint32_t now_ms);
static void robot_app_perception (const robot_app_shared_t * p_copy,
                                  uint32_t now_ms);
static void robot_app_diagnostic_frame (const robot_app_shared_t * p_copy,
                                       uint32_t now_ms);
static bool robot_app_sensors_init (void);
static void robot_app_health (uint32_t index, uint32_t now_ms);
static void robot_app_periodic (void (* p_step)(uint32_t), uint32_t period_ms);
static uint32_t robot_app_now (void);
static void robot_app_button (uint32_t now_ms, vehicle_state_t state);
static vehicle_nav_t robot_app_nav (barcode_action_t action);
static void robot_app_vehicle_input (const robot_app_shared_t * p_copy,
                                     uint32_t now_ms,
                                     vehicle_input_t * p_input);
static bool robot_app_original_line (const robot_app_shared_t * p_copy,
                                     uint32_t now_ms);
static void robot_app_scan_cycle (const robot_app_shared_t * p_copy,
                                  uint32_t now_ms, scanner_echo_t echo);
static bool robot_app_servo (int32_t angle_deg);
static int32_t robot_app_milli (double value);
static uint32_t robot_app_age (uint32_t now_ms, uint32_t stamp_ms);
static bool robot_app_imu_path (const robot_app_shared_t * p_copy,
                                uint32_t now_ms, double * p_path_m);

/** @brief Initialize statically-owned modules, then bounded network queues. */
bool robot_app_init (void)
{
    vehicle_config_t config = {0};
    uint32_t now_ms = robot_app_now();
    uint32_t index = 0U;
    bool b_ready = true;

    memset(&g_shared, 0, sizeof(g_shared));
    g_shared.front.state = CORE_RANGE_UNKNOWN;
    g_shared.front.distance_m = NAN;
    g_shared.vehicle = VEHICLE_INIT;
    g_shared.b_inhibit = true;
    for (index = 0U; index < ROBOT_APP_TASKS; ++index)
    {
        g_shared.health[index] = now_ms;
    }
    course_cfg_load(&g_barcode_config);
    barcode_init(&g_barcode);
    app_cfg_vehicle(&config,
        robot_app_codebook_ready());
    b_ready = vehicle_init(&g_vehicle, &config);
    b_ready = robot_app_motion_init() && b_ready;
    b_ready = robot_app_sensors_init() && b_ready;
    b_ready = net_mqtt_start() && b_ready;
    g_shared.b_inhibit = !g_motion.config.b_calibration_confirmed;
    g_shared.line_ms = now_ms;
    g_shared.imu_ms = now_ms;
    g_shared.tilt_ms = now_ms;
    g_shared.path_ms = now_ms;
    g_shared.previous_path_ms = now_ms;
    g_stationary_since_ms = now_ms;
    g_imu_window_ms = now_ms;
    gb_button_latched = board_start_pressed();
    gb_button_down = gb_button_latched;
    board_encoder_totals(&g_line_left_total, &g_line_right_total);
    g_motion_us = board_time_us();
    g_scan_centred_ms = now_ms;
    return b_ready;
}

/** @brief A bounded snapshot preserves the first detected fault reason. */
void robot_app_diagnostic (robot_app_diagnostic_t * p_output)
{
    uint32_t saved = 0U;

    if (NULL != p_output)
    {
        saved = app_main_lock();
        *p_output = g_shared.diagnostic;
        app_main_unlock(saved);
    }
}

/** @brief STOP preempts normal output; stale local owners latch a hard
 * fault. */
void robot_app_safety_step (uint32_t now_ms)
{
    robot_app_shared_t copy = {0};
    bool b_stale = false;

    robot_app_copy(&copy);
    if (board_stop_pressed())
    {
        robot_app_fault(false, ROBOT_APP_FAULT_LOCAL_STOP);
    }
    /* A valid measured limit violation remains hard even after a soft halt. */
    if (copy.terrain.b_tilt_valid &&
        ((fabs(copy.terrain.pitch_rad) > ROBOT_TILT_LIMIT_RAD) ||
         (fabs(copy.terrain.roll_rad) > ROBOT_TILT_LIMIT_RAD)))
    {
        robot_app_fault(true, ROBOT_APP_FAULT_TILT);
    }
    if (robot_app_active(copy.vehicle))
    {
        b_stale = (robot_app_age(now_ms, copy.health[1]) > 20U) ||
                  (robot_app_age(now_ms, copy.health[2]) > 20U) ||
                  (robot_app_age(now_ms, copy.health[3]) > 40U) ||
                  (robot_app_age(now_ms, copy.health[4]) > 100U) ||
                  (robot_app_age(now_ms, copy.health[5]) > 100U);
        if (b_stale || copy.terrain.b_collision_candidate)
        {
            robot_app_fault(true, b_stale ? ROBOT_APP_FAULT_OWNER_TIMEOUT :
                ROBOT_APP_FAULT_COLLISION);
        }
        else if ((!copy.terrain.b_tilt_valid) ||
                 (robot_app_age(now_ms, copy.tilt_ms) >
                  ROBOT_APP_TILT_TIMEOUT_MS))
        {
            robot_app_fault(false, ROBOT_APP_FAULT_TILT_UNAVAILABLE);
        }
    }
    robot_app_health(0U, now_ms);
    /* Motion, Line, Vehicle, IMU, Scan and Comm (1..6) gate the feed.
     * Telemetry and the radio owner that receives MQTT STOP (cyw43_utk_task,
     * P9) do not: remote STOP is best-effort.
     */
    if ((robot_app_age(now_ms, copy.health[1]) <= 20U) &&
        (robot_app_age(now_ms, copy.health[2]) <= 20U) &&
        (robot_app_age(now_ms, copy.health[3]) <= 40U) &&
        (robot_app_age(now_ms, copy.health[4]) <= 100U) &&
        (robot_app_age(now_ms, copy.health[5]) <= 100U) &&
        (robot_app_age(now_ms, copy.health[6]) <= 100U))
    {
        board_watchdog_feed();
    }
}

/** @brief One encoder/PID owner; a renewal is accepted only once per
 * publish. */
void robot_app_motion_step (uint32_t now_ms)
{
    robot_app_shared_t copy = {0};
    motion_output_t output = {0};
    int32_t left_delta = 0;
    int32_t right_delta = 0;
    uint32_t errors = 0U;
    uint64_t now_us = board_time_us();
    double dt_s = (double)(now_us - g_motion_us) / 1000000.0;
    double signed_distance_m = 0.0;
    motion_status_t status = MOTION_OK;

    /* A bounded microsecond interval is accurately rounded to binary64. */
    g_motion_us = now_us;
    robot_app_copy(&copy);
    board_read_encoders(&left_delta, &right_delta, &errors);
    left_delta = robot_app_signed_count(left_delta,
                                        ROBOT_LEFT_ENCODER_SIGN);
    right_delta = robot_app_signed_count(right_delta,
                                         ROBOT_RIGHT_ENCODER_SIGN);
    if (0U != errors)
    {
        robot_app_fault(true, ROBOT_APP_FAULT_ENCODER);
        copy.b_inhibit = true;
    }
    if ((copy.b_inhibit) &&
        (MOTION_INHIBITED != g_motion.output.state))
    {
        motion_stop(&g_motion, MOTION_STOPPED);
    }
    if (copy.b_reset_motion)
    {
        robot_app_reset(&copy);
    }
    if (g_motion_version != copy.request_version)
    {
        g_motion_version = copy.request_version;
        status = motion_submit(&g_motion, &copy.request, now_ms);
        if ((MOTION_OK != status) && (!copy.b_inhibit))
        {
            robot_app_fault(true, ROBOT_APP_FAULT_MOTION);
        }
    }
    status = motion_step(&g_motion, now_ms, dt_s,
                         left_delta, right_delta, &output);
    if ((MOTION_OK != status) && (MOTION_STOPPED != status) &&
        (MOTION_REJECTED != status))
    {
        robot_app_fault(true, ROBOT_APP_FAULT_MOTION);
    }
    signed_distance_m = output.signed_distance_m;
    robot_app_motion_health(&output, left_delta, right_delta, now_ms);
    robot_app_commit(&output, signed_distance_m, now_ms);
}

/** @brief Decode only during forward following; reverse/turn resets framing.
 * A frame still open when FOLLOW ends (for example at a RANGE_WAIT pause) is
 * discarded and counted as rejected, so br accounts for the lost code. This
 * includes a frame opened by a dark non-barcode feature, such as a junction
 * crossbar under the side sensor as NAVIGATE starts; Buddy 3 decides whether
 * that should count (T-55, T-06).
 */
void robot_app_line_step (uint32_t now_ms)
{
    robot_app_shared_t copy = {0};
    line_ctrl_output_t output = {0};
    barcode_output_t code = {0};
    uint16_t raw[LINE_CTRL_CHANNELS] = {0};
    uint32_t saved = 0U;
    bool b_valid = board_read_ir(raw);
    bool b_frame_lost = false;

    robot_app_copy(&copy);
    b_valid = line_ctrl_step(&g_line_config, &g_line, raw, now_ms,
                             &output) && b_valid;
    b_valid = robot_app_optical_path() && b_valid;
    if (!b_valid)
    {
        output.status = LINE_CTRL_FAULT;
    }
    if (b_valid && (VEHICLE_FOLLOW == copy.vehicle))
    {
        b_valid = barcode_step(&g_barcode_config, &g_barcode,
            output.darkness[2], g_optical_signed_m, now_ms, &code);
    }
    else
    {
        b_frame_lost = g_barcode.b_frame;
        barcode_init(&g_barcode);
    }
    saved = app_main_lock();
    g_shared.line = output;
    if (((VEHICLE_FOLLOW == copy.vehicle) ||
         (VEHICLE_RANGE_WAIT == copy.vehicle)) && output.b_junction_edge)
    {
        /* The 2 ms producer must retain an edge for the 10 ms consumer. */
        g_shared.b_junction_edge = true;
    }
    if ((BARCODE_EVENT_UNKNOWN == code.event) &&
        (UINT32_MAX != g_shared.barcode_unknown))
    {
        ++g_shared.barcode_unknown;
    }
    if (((BARCODE_EVENT_REJECTED == code.event) || b_frame_lost) &&
        (UINT32_MAX != g_shared.barcode_rejected))
    {
        ++g_shared.barcode_rejected;
    }
    if (b_valid)
    {
        g_shared.line_ms = now_ms;
    }
    if (BARCODE_EVENT_DECODED == code.event)
    {
        if (VEHICLE_NAV_NONE != g_shared.navigation)
        {
            robot_app_fault(true, ROBOT_APP_FAULT_OPTICAL);
        }
        else
        {
            g_shared.navigation = robot_app_nav(code.action);
            g_shared.barcode_action = code.action;
            /* Character code is nonnegative ASCII in the validated codebook. */
            g_shared.barcode_label = (unsigned char)code.label;
        }
    }
    g_shared.health[2] = now_ms;
    app_main_unlock(saved);
    /* A valid optical sample with an unmatched frame is not sensor failure. */
    if ((!b_valid) && (robot_app_active(copy.vehicle)))
    {
        robot_app_fault(true, ROBOT_APP_FAULT_OPTICAL);
    }
}

/** @brief Only local button edges start/reset; barcodes provide navigation. */
void robot_app_vehicle_step (uint32_t now_ms)
{
    robot_app_shared_t copy = {0};
    vehicle_input_t input = {0};
    vehicle_output_t output = {0};
    uint32_t saved = 0U;

    robot_app_copy(&copy);
    robot_app_button(now_ms, copy.vehicle);
    saved = app_main_lock();
    copy = g_shared;
    g_shared.b_start = false;
    g_shared.b_reset = false;
    g_shared.navigation = VEHICLE_NAV_NONE;
    g_shared.b_junction_edge = false;
    g_shared.b_coarse_ready = false;
    g_shared.b_fine_ready = false;
    app_main_unlock(saved);
    robot_app_vehicle_input(&copy, now_ms, &input);
    if (!vehicle_step(&g_vehicle, now_ms, &input, &output))
    {
        robot_app_fault(true, ROBOT_APP_FAULT_SUPERVISOR);
    }
    else if ((VEHICLE_FAULT == output.state) &&
             (VEHICLE_FAULT != copy.vehicle))
    {
        robot_app_fault(false, ROBOT_APP_FAULT_SUPERVISOR);
    }
    if ((VEHICLE_SCAN == output.state) &&
        (VEHICLE_SCAN != copy.vehicle))
    {
        g_route_anchor = copy.motion.pose;
    }
    if ((VEHICLE_REACQUIRE == output.state) &&
        (VEHICLE_NAVIGATE == copy.vehicle))
    {
        /* Completed turn defines the new branch's expected local heading. */
        g_route_anchor = copy.motion.pose;
    }
    saved = app_main_lock();
    if ((VEHICLE_FOLLOW == output.state) &&
        (VEHICLE_IDLE == copy.vehicle))
    {
        ++g_shared.run_id;
        g_shared.barcode_action = BARCODE_ACTION_UNKNOWN;
        g_shared.barcode_label = 0;
        g_shared.scan.b_profile_valid = false;
    }
    g_shared.vehicle = output.state;
    g_shared.scan_request = output.scan;
    if (output.b_issue_motion)
    {
        g_shared.request = output.request;
        ++g_shared.request_version;
    }
    if (output.b_reset_motion)
    {
        g_shared.b_reset_motion = true;
        g_shared.reset_epoch = copy.stop_epoch;
    }
    if (output.b_stop)
    {
        g_shared.b_inhibit = true;
        board_motor_brake();
    }
    g_shared.health[3] = now_ms;
    app_main_unlock(saved);
}

/** @brief Fresh calibrated accelerometer sample with encoder path interval. */
void robot_app_imu_step (uint32_t now_ms)
{
    robot_app_shared_t copy = {0};
    terrain_input_t input = {0};
    terrain_output_t output = {0};
    int16_t raw[3] = {0};
    double axes[3] = {0.0};
    double speed_mps = 0.0;
    double path_m = 0.0;
    uint32_t saved = 0U;
    board_accel_status_t sample_status = board_read_accel(raw);
    bool b_valid = BOARD_ACCEL_FRESH == sample_status;
    double mission_maximum_m = 0.0;

    robot_app_copy(&copy);
    /* Independent sensor clock: PENDING keeps its age; an overrun is FRESH,
     * dt spanning the lost sample. Gaps beyond the dt bound cannot drive.
     */
    input.dt_s = (double)(now_ms - copy.imu_ms) / 1000.0;
    if ((BOARD_ACCEL_PENDING == sample_status) &&
        (input.dt_s <= g_terrain.config.maximum_dt_s))
    {
        robot_app_health(4U, now_ms);
        return;
    }
    if (!b_valid)
    {
        saved = app_main_lock();
        g_shared.terrain.b_tilt_valid = false;
        g_shared.terrain.b_height_valid = false;
        g_shared.health[4] = now_ms;
        app_main_unlock(saved);
        if (robot_app_active(copy.vehicle))
        {
            robot_app_fault(BOARD_ACCEL_PENDING != sample_status,
                (BOARD_ACCEL_PENDING == sample_status) ?
                ROBOT_APP_FAULT_TILT_UNAVAILABLE : ROBOT_APP_FAULT_IMU);
        }
        return;
    }
    input.b_encoder_valid = robot_app_imu_path(&copy, now_ms, &path_m);
    robot_app_terrain_run(&copy, path_m, now_ms);
    /* int16_t mg and bias hypotheses are exactly representable in double. */
    axes[0] = ((double)raw[0] - ROBOT_ACCEL_BIAS_X_MG)
              * ROBOT_ACCEL_SCALE_X * ROBOT_APP_MG_TO_MPS2;
    axes[1] = ((double)raw[1] - ROBOT_ACCEL_BIAS_Y_MG)
              * ROBOT_ACCEL_SCALE_Y * ROBOT_APP_MG_TO_MPS2;
    axes[2] = ((double)raw[2] - ROBOT_ACCEL_BIAS_Z_MG)
              * ROBOT_ACCEL_SCALE_Z * ROBOT_APP_MG_TO_MPS2;
    input.accel_x_mps2 = axes[ROBOT_IMU_FORWARD_AXIS]
                        * ROBOT_IMU_FORWARD_SIGN;
    input.accel_y_mps2 = axes[ROBOT_IMU_LEFT_AXIS] * ROBOT_IMU_LEFT_SIGN;
    input.accel_z_mps2 = axes[ROBOT_IMU_UP_AXIS] * ROBOT_IMU_UP_SIGN;
    /* PENDING returned before the baseline moved: no interval is lost. */
    input.signed_distance_m = path_m - g_imu_path_m;
    input.encoder_yaw_rate_rps =
        (copy.motion.right_speed_mps - copy.motion.left_speed_mps)
        / ROBOT_TRACK_WIDTH_M;
    speed_mps = (copy.motion.left_speed_mps + copy.motion.right_speed_mps)
                / 2.0;
    input.b_sensor_valid = b_valid;
    /* A 200ms displacement window reduces count quantization in speed.
     * The acceleration threshold is a tuning hypothesis, not gyro evidence.
     */
    if ((now_ms - g_imu_window_ms) >= 200U)
    {
        g_imu_speed_mps = (path_m - g_imu_window_path_m) /
                          ((double)(now_ms - g_imu_window_ms) / 1000.0);
        g_imu_window_path_m = path_m;
        g_imu_window_ms = now_ms;
    }
    input.b_quasistatic = copy.motion.b_stationary ||
        ((fabs(speed_mps - g_imu_speed_mps) < 0.15) &&
         (fabs(input.accel_x_mps2) < 4.0));
    g_imu_path_m = path_m;
    b_valid = terrain_step(&g_terrain, &input, &output) && b_valid;
    mission_maximum_m = fmax(copy.mission_maximum_m, output.run_maximum_m);
    saved = app_main_lock();
    g_shared.terrain = output;
    g_shared.mission_maximum_m = mission_maximum_m;
    g_shared.b_mission_maximum_valid = copy.b_mission_maximum_valid ||
        (0U != output.completed_humps);
    g_shared.imu_ms = now_ms;
    if (b_valid && output.b_tilt_valid)
    {
        g_shared.tilt_ms = now_ms;
    }
    g_shared.health[4] = now_ms;
    app_main_unlock(saved);
    if ((!b_valid) && (robot_app_active(copy.vehicle)))
    {
        robot_app_fault(true, ROBOT_APP_FAULT_IMU);
    }
    else if ((!output.b_tilt_valid) && robot_app_active(copy.vehicle))
    {
        /* Unusable orientation brakes; recovery needs local reset. */
        robot_app_fault(false, ROBOT_APP_FAULT_TILT_UNAVAILABLE);
    }
}

/** @brief One servo/sonar owner; frontal polling resumes after scan
 * settling. */
void robot_app_scan_step (uint32_t now_ms)
{
    robot_app_shared_t copy = {0};
    scanner_echo_t echo = {0};
    core_range_sample_t range = {0};
    uint32_t saved = 0U;
    bool b_valid = false;

    robot_app_copy(&copy);
    if (VEHICLE_SCAN_FINE == copy.scan_request)
    {
        gb_fine_requested = true;
    }
    board_ultrasonic_service();
    echo.b_ready = board_ultrasonic_result(&echo.pulse_us, &b_valid);
    if (!b_valid)
    {
        echo.pulse_us = 0U;
    }
    if ((!gb_scan_busy) &&
        (VEHICLE_SCAN_COARSE == copy.scan_request) &&
        (!gb_scan_result_ready))
    {
        gb_scan_busy = scanner_start(&g_scanner, now_ms,
                                      copy.motion.b_stationary);
        g_scan_started_ms = now_ms;
        gb_front_pending = false;
    }
    if ((copy.b_stop_latched) || (copy.b_hard_fault))
    {
        if (gb_scan_busy)
        {
            gb_scan_busy = false;
            g_scanner.phase = SCANNER_ABORTED;
            g_scan_centred_ms = now_ms;
            (void)robot_app_servo(0);
        }
    }
    if (gb_scan_busy)
    {
        robot_app_scan_cycle(&copy, now_ms, echo);
    }
    else if ((now_ms - g_scan_centred_ms) >= ROBOT_SERVO_SETTLE_MS)
    {
        if (echo.b_ready && gb_front_pending)
        {
            gb_front_pending = false;
            /* uint32_t microseconds convert exactly, then scale to seconds. */
            (void)core_ultrasonic_range(&g_scanner.config.range, b_valid,
                (double)echo.pulse_us / 1000000.0, &range);
            saved = app_main_lock();
            g_shared.front = range;
            g_shared.range_ms = g_front_trigger_ms;
            g_shared.front_pose = g_front_trigger_pose;
            app_main_unlock(saved);
        }
        /* A false return means the board's bounded interval is not ready. */
        if (board_ultrasonic_trigger())
        {
            gb_front_pending = true;
            g_front_trigger_pose = copy.motion.pose;
            g_front_trigger_ms = now_ms;
        }
    }
    if (gb_fine_requested && gb_scan_result_ready &&
        ((now_ms - g_scan_centred_ms) >= ROBOT_SERVO_SETTLE_MS) &&
        (CORE_RANGE_VALID == copy.front.state) &&
        ((now_ms - copy.range_ms) < 150U))
    {
        saved = app_main_lock();
        g_shared.b_fine_ready = true;
        app_main_unlock(saved);
    }
    if (VEHICLE_SCAN != copy.vehicle)
    {
        gb_scan_result_ready = false;
        gb_fine_requested = false;
    }
    robot_app_health(5U, now_ms);
}

/** @brief Receive at most one bounded command; only STOP changes mission. */
void robot_app_comm_step (uint32_t now_ms)
{
    char command[33] = {0};
    size_t length = net_mqtt_receive(command, sizeof(command));
    uint32_t saved = 0U;

    if (0U != length)
    {
        if (0 == strcmp(command, "STOP"))
        {
            robot_app_fault(false, ROBOT_APP_FAULT_REMOTE_STOP);
        }
        else
        {
            saved = app_main_lock();
            ++g_shared.ignored_commands;
            app_main_unlock(saved);
        }
    }
    robot_app_health(6U, now_ms);
}

/** @brief Compact integer units avoid floating printf and preserve validity. */
void robot_app_telemetry_step (uint32_t now_ms)
{
    robot_app_shared_t copy = {0};
    telemetry_status_t status = {0};
    char frame[TELEMETRY_BUFFER_SIZE] = {0};
    uint32_t saved = 0U;
    size_t length = 0U;

    robot_app_copy(&copy);
    ++g_telemetry_sequence;
    status.seq = g_telemetry_sequence;
    status.now_ms = now_ms;
    status.dropped = copy.dropped_telemetry;
    status.ignored = copy.ignored_commands;
    status.range_age_ms = robot_app_age(now_ms, copy.range_ms);
    /* Enum values are small, nonnegative and exactly fit int32_t. */
    status.state = (int32_t)copy.vehicle;
    status.vleft_mm_s = robot_app_milli(copy.motion.left_speed_mps);
    status.vright_mm_s = robot_app_milli(copy.motion.right_speed_mps);
    status.pitch_mrad = robot_app_milli(copy.terrain.pitch_rad);
    status.peak_mm = robot_app_milli(copy.terrain.run_maximum_m);
    status.range_mm = robot_app_milli(copy.front.distance_m);
    status.b_stop = copy.b_inhibit;
    status.b_hardfault = copy.b_hard_fault;
    /* hv reports the last completed hump; pk > 0 is already a valid peak. */
    status.b_heightvalid = copy.terrain.b_last_hump_valid;
    status.b_rangevalid = CORE_RANGE_VALID == copy.front.state;
    length = telemetry_format(&status, frame, sizeof(frame));
    if ((0U == length) || (!net_mqtt_send(frame, length)))
    {
        saved = app_main_lock();
        ++g_shared.dropped_telemetry;
        app_main_unlock(saved);
    }
    if (0U == (g_telemetry_sequence % 5U))
    {
        robot_app_perception(&copy, now_ms);
        robot_app_diagnostic_frame(&copy, now_ms);
    }
    robot_app_health(7U, now_ms);
}

/** @brief Safety owner with absolute-deadline periodic release. */
void robot_app_safety_task (void)
{
    robot_app_periodic(robot_app_safety_step, 2U);
}

/** @brief Motion owner with absolute-deadline periodic release. */
void robot_app_motion_task (void)
{
    robot_app_periodic(robot_app_motion_step, 5U);
}

/** @brief Line owner with absolute-deadline periodic release. */
void robot_app_line_task (void)
{
    robot_app_periodic(robot_app_line_step, 2U);
}

/** @brief Vehicle owner with absolute-deadline periodic release. */
void robot_app_vehicle_task (void)
{
    robot_app_periodic(robot_app_vehicle_step, 10U);
}

/** @brief Imu owner with absolute-deadline periodic release. */
void robot_app_imu_task (void)
{
    robot_app_periodic(robot_app_imu_step, 10U);
}

/** @brief Scan owner with absolute-deadline periodic release. */
void robot_app_scan_task (void)
{
    robot_app_periodic(robot_app_scan_step, 2U);
}

/** @brief Comm owner with absolute-deadline periodic release. */
void robot_app_comm_task (void)
{
    robot_app_periodic(robot_app_comm_step, 5U);
}

/** @brief Telemetry owner with absolute-deadline periodic release. */
void robot_app_telemetry_task (void)
{
    robot_app_periodic(robot_app_telemetry_step, 200U);
}

/** @brief Snapshot copy has no kernel wait or peripheral transaction. */
static void robot_app_copy (robot_app_shared_t * p_copy)
{
    uint32_t saved = app_main_lock();

    *p_copy = g_shared;
    app_main_unlock(saved);
}

/** @brief Sticky software stop and immediate output commit share one lock. */
static void robot_app_fault (bool b_hard, robot_app_fault_t reason)
{
    uint32_t now_ms = robot_app_now();
    uint32_t saved = app_main_lock();

    if (ROBOT_APP_FAULT_NONE == g_shared.diagnostic.code)
    {
        g_shared.diagnostic.code = reason;
        g_shared.diagnostic.first_fault_ms = now_ms;
    }
    g_shared.b_hard_fault = g_shared.b_hard_fault || b_hard;
    g_shared.b_inhibit = true;
    g_shared.b_stop_latched = true;
    ++g_shared.stop_epoch;
    board_motor_brake();
    app_main_unlock(saved);
}

/** @brief Active missions, including stopped retries, retain health gates. */
static bool robot_app_active (vehicle_state_t state)
{
    return (VEHICLE_FOLLOW == state) || (VEHICLE_NAVIGATE == state) ||
           (VEHICLE_SCAN == state) || (VEHICLE_BYPASS == state) ||
           (VEHICLE_REACQUIRE == state) || (VEHICLE_RANGE_WAIT == state);
}

/** @brief Every drive authorization requires a fresh usable orientation. */
static bool robot_app_ready (const robot_app_shared_t * p_copy,
                             uint32_t now_ms)
{
    return (0 != ROBOT_CALIBRATION_CONFIRMED) &&
           (0 != ROBOT_IMU_MOUNT_CONFIRMED) &&
           (robot_app_age(now_ms, p_copy->line_ms) <= 20U) &&
           (robot_app_age(now_ms, p_copy->imu_ms) <= 100U) &&
           (robot_app_age(now_ms, p_copy->tilt_ms) <=
            ROBOT_APP_TILT_TIMEOUT_MS) &&
           p_copy->terrain.b_tilt_valid &&
           (fabs(p_copy->terrain.pitch_rad) <= ROBOT_TILT_LIMIT_RAD) &&
           (fabs(p_copy->terrain.roll_rad) <= ROBOT_TILT_LIMIT_RAD) &&
           (robot_app_age(now_ms, p_copy->health[1]) <= 20U) &&
           (LINE_CTRL_FAULT != p_copy->line.status) &&
           (LINE_CTRL_INHIBITED != p_copy->line.status) &&
           (!p_copy->b_hard_fault);
}

/** @brief Unconfirmed calibration is a normal inhibited configuration. */
static bool robot_app_motion_init (void)
{
    motion_config_t config = {0};
    motion_status_t result = MOTION_INVALID;

    app_cfg_motion(&config);
    result = motion_init(&g_motion, &config);
    return (MOTION_OK == result) || (MOTION_REJECTED == result);
}

/** @brief Load measured reference parameters; never auto-calibrate on a
 * hump. */
static bool robot_app_sensors_init (void)
{
    terrain_config_t terrain_config = {0};
    scanner_config_t scan_config = {0};
    bool b_ready = true;

    app_cfg_line(&g_line_config);
    line_ctrl_init(&g_line);
    app_cfg_terrain(&terrain_config);
    app_cfg_scanner(&scan_config);
    b_ready = terrain_init(&g_terrain, &terrain_config);
    b_ready = scanner_init(&g_scanner, &scan_config) && b_ready;
    b_ready = line_ctrl_config_valid(&g_line_config) && b_ready;
    b_ready = barcode_config_valid(&g_barcode_config) && b_ready;
    if (terrain_config.b_calibrated)
    {
        /* Reference is a stored, reviewed stationary mean in vehicle axes. */
        b_ready = terrain_reference(&g_terrain,
            ROBOT_IMU_REFERENCE_X_MG * ROBOT_APP_MG_TO_MPS2,
            ROBOT_IMU_REFERENCE_Y_MG * ROBOT_APP_MG_TO_MPS2,
            ROBOT_IMU_REFERENCE_Z_MG * ROBOT_APP_MG_TO_MPS2, true)
            && b_ready;
    }
    if (0 != ROBOT_SERVO_CONFIRMED)
    {
        b_ready = robot_app_servo(0) && b_ready;
    }
    return b_ready;
}

/** @brief Publish liveness only after one complete activation. */
static void robot_app_health (uint32_t index, uint32_t now_ms)
{
    uint32_t saved = app_main_lock();

    g_shared.health[index] = now_ms;
    app_main_unlock(saved);
}

/** @brief Absolute releases skip missed periods instead of a catch-up loop. */
static void robot_app_periodic (void (* p_step)(uint32_t), uint32_t period_ms)
{
    uint32_t next_ms = robot_app_now();
    uint32_t now_ms = next_ms;
    uint32_t remaining_ms = 0U;

    for (;;)
    {
        p_step(now_ms);
        next_ms += period_ms;
        now_ms = robot_app_now();
        remaining_ms = next_ms - now_ms;
        if ((0U == remaining_ms) ||
            (remaining_ms >= ROBOT_APP_SERIAL_HALF))
        {
            /* A missed period is skipped, not counted: the owner freshness
             * gates see the late step, which a tight catch-up dispatch would
             * hide. Measure jitter on the board.
             */
            next_ms = now_ms + period_ms;
            remaining_ms = period_ms;
        }
        app_main_delay(remaining_ms);
        now_ms = robot_app_now();
    }
}

/** @brief Millisecond serial clock deliberately wraps every 2^32 ms. */
static uint32_t robot_app_now (void)
{
    /* Unsigned narrowing intentionally implements modulo-2^32 milliseconds. */
    return (uint32_t)(board_time_us() / 1000U);
}

/** @brief One action per debounced press, with release required to rearm. */
static void robot_app_button (uint32_t now_ms, vehicle_state_t state)
{
    bool b_down = board_start_pressed();
    uint32_t saved = 0U;

    if (b_down != gb_button_down)
    {
        gb_button_down = b_down;
        g_button_since_ms = now_ms;
    }
    if ((now_ms - g_button_since_ms) < 30U)
    {
        return;
    }
    if (!b_down)
    {
        gb_button_latched = false;
    }
    else if (!gb_button_latched)
    {
        gb_button_latched = true;
        saved = app_main_lock();
        if (VEHICLE_FAULT == state)
        {
            g_shared.b_reset = true;
        }
        else if (VEHICLE_IDLE == state)
        {
            g_shared.b_start = true;
        }
        else
        {
            /* A press during a manoeuvre cannot change autonomous intent. */
        }
        app_main_unlock(saved);
    }
    else
    {
        /* Held button cannot start or reset repeatedly. */
    }
}

/** @brief Mapping is from decoded action, never from illustrative A/B/C/D. */
static vehicle_nav_t robot_app_nav (barcode_action_t action)
{
    vehicle_nav_t navigation = VEHICLE_NAV_NONE;

    switch (action)
    {
        case BARCODE_ACTION_LEFT:
            navigation = VEHICLE_NAV_LEFT;
        break;
        case BARCODE_ACTION_RIGHT:
            navigation = VEHICLE_NAV_RIGHT;
        break;
        case BARCODE_ACTION_STRAIGHT:
            navigation = VEHICLE_NAV_STRAIGHT;
        break;
        case BARCODE_ACTION_UTURN:
            navigation = VEHICLE_NAV_UTURN;
        break;
        case BARCODE_ACTION_UNKNOWN:
        default:
            navigation = VEHICLE_NAV_NONE;
        break;
    }
    return navigation;
}

/** @brief Construct evidence gates and bounded geometric bypass hypotheses. */
static void robot_app_vehicle_input (const robot_app_shared_t * p_copy,
                                     uint32_t now_ms,
                                     vehicle_input_t * p_input)
{
    double offset_m = 0.0;
    double length_m = 0.0;
    double steering = p_copy->line.steering * ROBOT_LINE_GAIN_RPS;

    if (steering > ROBOT_TURN_RATE_RPS)
    {
        steering = ROBOT_TURN_RATE_RPS;
    }
    if (steering < -ROBOT_TURN_RATE_RPS)
    {
        steering = -ROBOT_TURN_RATE_RPS;
    }
    p_input->motion = p_copy->motion;
    p_input->navigation = p_copy->navigation;
    p_input->b_junction_edge = p_copy->b_junction_edge;
    p_input->line_forward_mps = ROBOT_LINE_SPEED_MPS *
                               (1.0 - (0.5 * fabs(p_copy->line.steering)));
    /* Optical positive-right becomes differential-drive positive-left yaw. */
    p_input->line_yaw_rps = -steering;
    p_input->b_start = p_copy->b_start;
    p_input->b_reset = p_copy->b_reset;
    p_input->b_stop = (p_copy->b_hard_fault) || (board_stop_pressed()) ||
        ((p_copy->b_stop_latched) && (VEHICLE_FAULT != p_copy->vehicle));
    p_input->b_fault_clear = (!p_copy->b_hard_fault) &&
                             (!board_stop_pressed());
    p_input->b_sensors_ready = robot_app_ready(p_copy, now_ms);
    p_input->b_line_valid = (LINE_CTRL_TRACKING == p_copy->line.status) ||
                            (LINE_CTRL_JUNCTION == p_copy->line.status);
    p_input->b_line_wait = LINE_CTRL_SEARCHING == p_copy->line.status;
    p_input->b_range_valid = (CORE_RANGE_VALID == p_copy->front.state) &&
        (fabs(atan2(sin(p_copy->motion.pose.yaw_rad -
                       p_copy->front_pose.yaw_rad),
                    cos(p_copy->motion.pose.yaw_rad -
                       p_copy->front_pose.yaw_rad))) <= 0.12) &&
        (hypot(p_copy->motion.pose.x_m - p_copy->front_pose.x_m,
               p_copy->motion.pose.y_m - p_copy->front_pose.y_m) <= 0.03);
    p_input->range_age_ms = robot_app_age(now_ms, p_copy->range_ms);
    p_input->b_obstacle = p_input->b_range_valid &&
        (p_copy->front.distance_m <= ROBOT_FRONT_STOP_DISTANCE_M);
    p_input->b_front_clear = p_input->b_range_valid &&
        (p_copy->front.distance_m > ROBOT_FRONT_STOP_DISTANCE_M);
    p_input->b_coarse_ready = p_copy->b_coarse_ready;
    p_input->b_fine_ready = p_copy->b_fine_ready;
    offset_m = (p_copy->scan.apparent_width_m / 2.0) +
               (ROBOT_ROBOT_WIDTH_M / 2.0) + ROBOT_OBSTACLE_MARGIN_M;
    length_m = p_copy->scan.nearest_distance_m + ROBOT_MAX_OBSTACLE_DEPTH_M
               + ROBOT_ROBOT_LENGTH_M + ROBOT_OBSTACLE_MARGIN_M;
    p_input->bypass_offset_m = offset_m;
    p_input->bypass_length_m = length_m;
    p_input->bypass_side = SCANNER_SIDE_LEFT == p_copy->scan.preferred_side
                           ? 1 : -1;
    p_input->b_bypass_clearance = (0 != ROBOT_CORRIDOR_CONFIRMED) &&
        (p_copy->scan.b_profile_valid) && (p_copy->scan.b_obstacle_seen) &&
        (SCANNER_SIDE_NONE != p_copy->scan.preferred_side) &&
        (0U == p_copy->scan.unknown_samples);
    p_input->b_original_line_verified =
        robot_app_original_line(p_copy, now_ms);
}

/** @brief Require known course corridor, heading and sustained optical lock. */
static bool robot_app_original_line (const robot_app_shared_t * p_copy,
                                     uint32_t now_ms)
{
    double delta_x = p_copy->motion.pose.x_m - g_route_anchor.x_m;
    double delta_y = p_copy->motion.pose.y_m - g_route_anchor.y_m;
    double lateral_m = (-sin(g_route_anchor.yaw_rad) * delta_x) +
                        (cos(g_route_anchor.yaw_rad) * delta_y);
    double yaw_error = atan2(
        sin(p_copy->motion.pose.yaw_rad - g_route_anchor.yaw_rad),
        cos(p_copy->motion.pose.yaw_rad - g_route_anchor.yaw_rad));
    bool b_matches = (0 != ROBOT_COURSE_CONFIRMED) &&
        (LINE_CTRL_TRACKING == p_copy->line.status) &&
        (fabs(lateral_m) <= ROBOT_LINE_UNIQUE_CORRIDOR_M) &&
        (fabs(yaw_error) <= ROBOT_LINE_HEADING_TOL_RAD);

    if ((!b_matches) || (VEHICLE_REACQUIRE != p_copy->vehicle))
    {
        gb_line_lock = false;
        return false;
    }
    if (!gb_line_lock)
    {
        gb_line_lock = true;
        g_line_lock_ms = now_ms;
    }
    return (now_ms - g_line_lock_ms) >= ROBOT_LINE_LOCK_MS;
}

/** @brief Combined scanner completes both passes before staged IPC events. */
static void robot_app_scan_cycle (const robot_app_shared_t * p_copy,
                                  uint32_t now_ms, scanner_echo_t echo)
{
    scanner_output_t output = {0};
    uint32_t saved = 0U;
    bool b_ok = scanner_step(&g_scanner, now_ms,
                              p_copy->motion.b_stationary, &echo, &output);

    if (output.b_set_servo)
    {
        b_ok = robot_app_servo(output.angle_deg) && b_ok;
    }
    if (output.b_trigger)
    {
        b_ok = board_ultrasonic_trigger() && b_ok;
    }
    if ((!b_ok) || (g_scanner.result.b_aborted) ||
        ((now_ms - g_scan_started_ms) > 20000U))
    {
        robot_app_fault(true, ROBOT_APP_FAULT_SCAN);
        gb_scan_busy = false;
        return;
    }
    if (output.b_done)
    {
        gb_scan_busy = false;
        gb_scan_result_ready = true;
        g_scan_centred_ms = now_ms;
        saved = app_main_lock();
        g_shared.scan = g_scanner.result;
        g_shared.scan_ms = now_ms;
        g_shared.b_coarse_ready = true;
        g_shared.front.state = CORE_RANGE_UNKNOWN;
        app_main_unlock(saved);
    }
}

/** @brief Calibrated zero and sign map a -60..60 beam sector to PWM. */
static bool robot_app_servo (int32_t angle_deg)
{
    int32_t pulse_us = 0;
    int32_t physical_deg = 0;
    int32_t span_us = 0;

    if ((angle_deg < -60) || (angle_deg > 60) ||
        ((1 != ROBOT_SERVO_ANGLE_SIGN) && (-1 != ROBOT_SERVO_ANGLE_SIGN)) ||
        (0U == ROBOT_SERVO_MIN_US) ||
        (ROBOT_SERVO_MIN_US >= ROBOT_SERVO_CENTRE_US) ||
        (ROBOT_SERVO_CENTRE_US >= ROBOT_SERVO_MAX_US) ||
        (ROBOT_SERVO_MAX_US > UINT16_MAX) ||
        (ROBOT_SERVO_MAX_US >= ROBOT_SERVO_FRAME_US))
    {
        return false;
    }
    physical_deg = angle_deg * ROBOT_SERVO_ANGLE_SIGN;
    /* Calibrated negative and positive endpoint spans need not be equal. */
    span_us = (physical_deg < 0) ?
        (int32_t)(ROBOT_SERVO_CENTRE_US - ROBOT_SERVO_MIN_US) :
        (int32_t)(ROBOT_SERVO_MAX_US - ROBOT_SERVO_CENTRE_US);
    pulse_us = (int32_t)ROBOT_SERVO_CENTRE_US +
        ((physical_deg * span_us) / 60);
    /* Validated endpoints bound the result within the uint16_t board API. */
    return board_servo_us((uint16_t)pulse_us);
}

/** @brief Bound SI-to-milli telemetry conversion before signed narrowing. */
static int32_t robot_app_milli (double value)
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

/** @brief Wrap-safe age of a shared stamp on the millisecond serial clock.
 * A step reads now_ms at release; a higher-priority owner may publish a
 * newer stamp before the step's snapshot (for example MotionTask or LineTask
 * during VehicleTask's button read). That stamp is fresh (age 0), not about
 * 2^32 ms stale. Genuine staleness below the serial half is unchanged. The
 * IMU step instead selects an encoder path sample (robot_app_imu_path).
 */
static uint32_t robot_app_age (uint32_t now_ms, uint32_t stamp_ms)
{
    uint32_t age = now_ms - stamp_ms;

    return (age >= ROBOT_APP_SERIAL_HALF) ? 0U : age;
}

/** @brief Encoder path at the last Motion publish at or before the IMU
 * release now_ms, so the path interval matches the release-based dt. A
 * publish during this step's blocking I2C read is newer than the release:
 * the previous sample is used instead, so one preemption creates no
 * path/time skew and no spurious deceleration. If even the previous sample
 * is newer (two publishes during one step), this step's encoder interval is
 * invalid and that sample becomes the next baseline. The following step is
 * still marked valid but can under-estimate its path by 5 ms (5 ms of path
 * over a 10 ms dt). After an invalid step it has no acceleration of its
 * own, and with forward-only motion the next step's recovery reads as
 * acceleration, so it cannot latch a false COLLISION; it can misclassify
 * that recovery step and dull its collision test. A sample older than
 * 20 ms is invalid. False means the terrain must not use this interval.
 * Residuals (T-53): the step after a double publish, as above; now_ms is
 * the step's start, so higher-priority work that delays the start past a
 * Motion publish still skews one interval; and a 10-20 ms Motion publish
 * gap gives a valid zero-distance interval, then a valid double-distance
 * one, which with a coincident longitudinal transient below -3 m/s^2 can
 * latch a false COLLISION.
 */
static bool robot_app_imu_path (const robot_app_shared_t * p_copy,
                                uint32_t now_ms, double * p_path_m)
{
    uint32_t stamp_ms = p_copy->path_ms;

    *p_path_m = p_copy->signed_path_m;
    if ((now_ms - stamp_ms) >= ROBOT_APP_SERIAL_HALF)
    {
        stamp_ms = p_copy->previous_path_ms;
        *p_path_m = p_copy->previous_path_m;
    }
    /* A stamp still newer than now_ms wraps to at least the serial half. */
    return (now_ms - stamp_ms) <= 20U;
}


/** @brief Software STOP always wins over an older reset authorization. */
static void robot_app_reset (const robot_app_shared_t * p_copy)
{
    motion_status_t status = MOTION_REJECTED;
    uint32_t saved = 0U;
    bool b_approved = false;

    status = motion_reset(&g_motion, p_copy->motion.b_stationary,
                           (!p_copy->b_hard_fault) &&
                           (!board_stop_pressed()));
    saved = app_main_lock();
    b_approved = (MOTION_OK == status) &&
        (p_copy->reset_epoch == g_shared.stop_epoch) &&
        (!g_shared.b_hard_fault) && (!board_stop_pressed());
    g_shared.b_reset_motion = false;
    if (b_approved)
    {
        g_shared.b_inhibit = false;
        g_shared.b_stop_latched = false;
        g_shared.diagnostic.code = ROBOT_APP_FAULT_NONE;
        g_shared.diagnostic.first_fault_ms = 0U;
    }
    app_main_unlock(saved);
    if (!b_approved)
    {
        motion_stop(&g_motion, MOTION_STOPPED);
    }
}

/** @brief Separate wheels cannot hide each other's stalls; rest needs dwell. */
static void robot_app_motion_health (motion_output_t * p_output,
    int32_t left_delta, int32_t right_delta, uint32_t now_ms)
{
    if ((0 != left_delta) ||
        (fabs(p_output->left_duty) < ROBOT_STALL_DUTY))
    {
        g_left_stall_ms = now_ms;
    }
    if ((0 != right_delta) ||
        (fabs(p_output->right_duty) < ROBOT_STALL_DUTY))
    {
        g_right_stall_ms = now_ms;
    }
    if (((now_ms - g_left_stall_ms) > ROBOT_STALL_MS) ||
        ((now_ms - g_right_stall_ms) > ROBOT_STALL_MS))
    {
        robot_app_fault(true,
            ((now_ms - g_left_stall_ms) > ROBOT_STALL_MS) ?
            ROBOT_APP_FAULT_LEFT_STALL : ROBOT_APP_FAULT_RIGHT_STALL);
    }
    if ((0 != left_delta) || (0 != right_delta) ||
        (fabs(p_output->left_duty) > 0.01) ||
        (fabs(p_output->right_duty) > 0.01))
    {
        g_stationary_since_ms = now_ms;
    }
    p_output->b_stationary =
        (now_ms - g_stationary_since_ms) >= 200U;
}

/** @brief Prepare soft-float PWM outside masking, commit with latest
 * inhibit. */
static void robot_app_commit (motion_output_t * p_output,
                              double distance_m, uint32_t now_ms)
{
    board_motor_command_t command = {0};
    uint32_t saved = 0U;
    double new_path_m = g_shared.signed_path_m + distance_m;
    bool b_prepared = board_motor_prepare(p_output->left_duty,
                                          p_output->right_duty, &command);

    /* Unconfirmed calibration is a normal inhibited configuration (for
     * example the wiring-only commissioning stage): a rejected command is
     * braked below without a hard fault. With the gate open, a preparation
     * failure is a genuine output fault.
     */
    if ((!b_prepared) && g_motion.config.b_calibration_confirmed)
    {
        robot_app_fault(true, ROBOT_APP_FAULT_MOTOR_OUTPUT);
    }
    saved = app_main_lock();
    g_shared.motion = *p_output;
    g_shared.previous_path_m = g_shared.signed_path_m;
    g_shared.previous_path_ms = g_shared.path_ms;
    g_shared.signed_path_m = new_path_m;
    g_shared.path_ms = now_ms;
    if (g_shared.b_inhibit || (!b_prepared))
    {
        board_motor_brake();
    }
    else
    {
        board_motor_commit(&command);
    }
    g_shared.health[1] = now_ms;
    app_main_unlock(saved);
}

/** @brief A measured sign can reverse a count without signed-overflow UB. */
static int32_t robot_app_signed_count (int32_t count, int32_t sign)
{
    if (((1 != sign) && (-1 != sign)) || (INT32_MIN == count))
    {
        robot_app_fault(true, ROBOT_APP_FAULT_ENCODER);
        return 0;
    }
    return (1 == sign) ? count : -count;
}

/** @brief Convert a modulo position difference with an explicit half-range. */
static int32_t robot_app_count_delta (uint32_t newer, uint32_t older)
{
    uint32_t difference = newer - older;

    if (ROBOT_APP_SERIAL_HALF == difference)
    {
        robot_app_fault(true, ROBOT_APP_FAULT_ENCODER);
        return 0;
    }
    if (difference < ROBOT_APP_SERIAL_HALF)
    {
        /* Positive difference is <=INT32_MAX. */
        return (int32_t)difference;
    }
    /* Negative magnitude is 1..INT32_MAX and is safe to negate. */
    return -(int32_t)(UINT32_MAX - difference + 1U);
}

/** @brief Two-millisecond optical sampling has independent encoder totals. */
static bool robot_app_optical_path (void)
{
    uint32_t left_total = 0U;
    uint32_t right_total = 0U;
    int32_t left_delta = 0;
    int32_t right_delta = 0;
    core_odometry_result_t result = {0};
    core_status_t status = CORE_INVALID_ARGUMENT;

    board_encoder_totals(&left_total, &right_total);
    left_delta = robot_app_signed_count(
        robot_app_count_delta(left_total, g_line_left_total),
        ROBOT_LEFT_ENCODER_SIGN);
    right_delta = robot_app_signed_count(
        robot_app_count_delta(right_total, g_line_right_total),
        ROBOT_RIGHT_ENCODER_SIGN);
    g_line_left_total = left_total;
    g_line_right_total = right_total;
    status = core_odometry_step(&g_motion.config.odometry, &g_optical_pose,
                                left_delta, right_delta, &result);
    if (CORE_OK != status)
    {
        return false;
    }
    g_optical_pose = result.pose;
    g_optical_signed_m += result.signed_centre_distance_m;
    return isfinite(g_optical_signed_m);
}

/** @brief Reject a codebook whose narrowest run cannot be sampled spatially. */
static bool robot_app_codebook_ready (void)
{
    double quantum_m = 0.0;
    double worst_step_m = 0.0;
    double counts_per_revolution = ROBOT_ENCODER_COUNTS_PER_REV;

    if (counts_per_revolution <= 0.0)
    {
        return false;
    }
    quantum_m = (2.0 * CORE_PI * ROBOT_WHEEL_RADIUS_M) /
                counts_per_revolution;
    /* Qualify the largest accepted gap, not the nominal 2 ms task period.
     * The line-speed bound and encoder calibration still need bench proof.
     */
    worst_step_m = quantum_m + (ROBOT_LINE_SPEED_MPS *
        ((double)g_barcode_config.maximum_gap_ms / 1000.0));

    return (g_barcode_config.b_codebook_confirmed) &&
        (0U != g_barcode_config.symbol_count) &&
        (barcode_config_valid(&g_barcode_config)) &&
        (g_barcode_config.maximum_step_m >= worst_step_m) &&
        (g_barcode_config.minimum_run_m >=
         (5.0 * g_barcode_config.maximum_step_m));
}

/** @brief One-Hz perception summary complements five-Hz motion status. */
static void robot_app_perception (const robot_app_shared_t * p_copy,
                                  uint32_t now_ms)
{
    telemetry_perception_t status = {0};
    char frame[TELEMETRY_BUFFER_SIZE] = {0};
    uint32_t saved = 0U;
    size_t length = 0U;

    status.seq = g_telemetry_sequence;
    status.now_ms = now_ms;
    status.run_id = p_copy->run_id;
    status.cal_version = ROBOT_CALIBRATION_VERSION;
    /* Enum values are bounded and exactly representable in int32_t. */
    status.barcode_action = (int32_t)p_copy->barcode_action;
    status.barcode_label = p_copy->barcode_label;
    status.yaw_mrad_s = robot_app_milli(
        p_copy->terrain.encoder_yaw_rate_rps);
    status.motion_class = (int32_t)p_copy->terrain.motion;
    status.scan_width_mm = robot_app_milli(p_copy->scan.apparent_width_m);
    status.scan_nearest_mm =
        robot_app_milli(p_copy->scan.nearest_distance_m);
    status.scan_side = (int32_t)p_copy->scan.preferred_side;
    status.scan_valid = p_copy->scan.valid_samples;
    status.scan_unknown = p_copy->scan.unknown_samples;
    status.b_tilt_valid = p_copy->terrain.b_tilt_valid;
    status.b_hump_active = p_copy->terrain.b_hump_active;
    status.b_scan_valid = p_copy->scan.b_profile_valid;
    length = telemetry_format_perception(&status, frame, sizeof(frame));
    if ((0U == length) || (!net_mqtt_send(frame, length)))
    {
        saved = app_main_lock();
        ++g_shared.dropped_telemetry;
        app_main_unlock(saved);
    }
}

/** @brief One-Hz first-cause frame preserves bounded sensor frame sizes. */
static void robot_app_diagnostic_frame (const robot_app_shared_t * p_copy,
                                       uint32_t now_ms)
{
    telemetry_diagnostic_t status = {0};
    char frame[TELEMETRY_BUFFER_SIZE] = {0};
    uint32_t saved = 0U;
    size_t length = 0U;

    status.seq = g_telemetry_sequence;
    status.now_ms = now_ms;
    status.run_id = p_copy->run_id;
    status.first_fault_ms = p_copy->diagnostic.first_fault_ms;
    status.barcode_unknown = p_copy->barcode_unknown;
    status.barcode_rejected = p_copy->barcode_rejected;
    status.mission_peak_mm = robot_app_milli(p_copy->mission_maximum_m);
    status.b_mission_valid = p_copy->b_mission_maximum_valid;
    /* Enum codes are 0..15, exactly representable in the wire integer. */
    status.fault_code = (int32_t)p_copy->diagnostic.code;
    length = telemetry_format_diagnostic(&status, frame, sizeof(frame));
    if ((0U == length) || (!net_mqtt_send(frame, length)))
    {
        saved = app_main_lock();
        ++g_shared.dropped_telemetry;
        app_main_unlock(saved);
    }
}

/** @brief IMU owner resets run history while retaining its measured mount.
 * path_m is the encoder path robot_app_imu_path selected for the release
 * now_ms. Both 200 ms speed-window bases refer to that same instant.
 */
static void robot_app_terrain_run (const robot_app_shared_t * p_copy,
                                   double path_m, uint32_t now_ms)
{
    terrain_config_t config = g_terrain.config;
    double pitch_reference = g_terrain.pitch_reference_rad;
    double roll_reference = g_terrain.roll_reference_rad;
    bool b_reference_valid = g_terrain.b_reference_valid;

    if (p_copy->run_id != g_imu_run_id)
    {
        if (!terrain_init(&g_terrain, &config))
        {
            robot_app_fault(true, ROBOT_APP_FAULT_IMU);
        }
        g_terrain.pitch_reference_rad = pitch_reference;
        g_terrain.roll_reference_rad = roll_reference;
        g_terrain.b_reference_valid = b_reference_valid;
        g_imu_run_id = p_copy->run_id;
        g_imu_path_m = path_m;
        g_imu_window_path_m = path_m;
        g_imu_speed_mps = 0.0;
        g_imu_window_ms = now_ms;
    }
}

/* End of file. */

