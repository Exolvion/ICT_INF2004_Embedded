/** @file robot_scan.c
 * @brief The robot app: subsystem 5: the Scan task (servo and sonar). Split
 * out of the one file by tools/reorg/split_c.py (22 Nov 2026); what the
 * parts share is in robot_app_private.h.
 */
#include "robot_app_private.h"
#include "robot_scan.h"

/* The sonar's echo time field (15 bits, up to 32.767 ms). */
#define ROBOT_APP_PULSE_MAX_US (32767U)

/** Scan task flags; one owner, so plain bit-field writes are safe. */
typedef struct
{
    bool b_busy          : 1;
    bool b_result_ready  : 1;
    bool b_front_pending : 1;
    bool b_fine_requested : 1;
    bool b_fine_reported : 1;
    bool b_no_echo       : 1;
} robot_app_scan_flags_t;
#if !ROBOT_APP_MISSION
static uint32_t g_scan_started_ms;
#endif
static core_pose_packed_t g_front_trigger_pose;
static uint32_t g_front_trigger_ms;
static robot_app_scan_flags_t g_scan_flags;
#if ROBOT_APP_MISSION
           /* Vehicle: last 1 ms given away */
static uint32_t g_scan_servo_seq;
     /* Scan: servo request applied */
static uint32_t g_scan_trigger_seq;
         /* Vehicle: START until reset/run */
static bool gb_scan_echo;

/** @brief One servo/sonar owner for the mission: a new servo request sets
 * the pulse, a new trigger request triggers as soon as the board allows,
 * and that trigger's echo goes back with its sequence number. Every echo
 * also updates the front range and NO_ECHO for telemetry. An echo missing
 * after 100 ms is handed back as no echo.
 */
void robot_app_scan_step (uint32_t now_ms)
{
    robot_app_shared_t copy = {0};
    uint32_t pulse_us = 0U;
    uint32_t saved = 0U;
    bool b_valid = false;
    bool b_done = false;

    robot_app_copy(&copy);
    board_ultrasonic_service();
    if (board_ultrasonic_result(&pulse_us, &b_valid))
    {
        if (!b_valid)
        {
            pulse_us = 0U;
        }
        robot_app_front_result(b_valid, pulse_us);
        b_done = gb_scan_echo;
    }
    else if (gb_scan_echo && ((now_ms - g_front_trigger_ms) > 100U))
    {
        b_done = true;
        b_valid = false;
        pulse_us = 0U;
    }
    else
    {
        /* No echo yet. */
    }
    if (b_done)
    {
        gb_scan_echo = false;
        saved = app_main_lock();
        g_shared.sonar.done_seq = g_scan_trigger_seq & MISSION_SEQ_MASK;
        g_shared.sonar.pulse_us = ((pulse_us > ROBOT_APP_PULSE_MAX_US) ?
                                   ROBOT_APP_PULSE_MAX_US : pulse_us) &
                                  ROBOT_APP_PULSE_MAX_US;
        g_shared.sonar.b_valid = b_valid;
        app_main_unlock(saved);
    }
    if (copy.sonar.servo_seq != g_scan_servo_seq)
    {
        g_scan_servo_seq = copy.sonar.servo_seq;
        if (0U != copy.sonar.servo_us)
        {
            /* 12 bits: the pulse fits uint16_t. */
            (void)board_servo_us((uint16_t)copy.sonar.servo_us);
        }
    }
    if ((copy.sonar.trigger_seq != g_scan_trigger_seq) && (!gb_scan_echo) &&
        board_ultrasonic_trigger())
    {
        g_scan_trigger_seq = copy.sonar.trigger_seq;
        gb_scan_echo = true;
        g_front_trigger_pose = copy.motion.pose;
        g_front_trigger_ms = now_ms;
    }
    robot_app_health(ROBOT_APP_HEALTH_SCAN, now_ms);
}
#endif
#if !(ROBOT_APP_MISSION)

/** @brief One servo/sonar owner; frontal polling resumes after scan
 * settling. */
void robot_app_scan_step (uint32_t now_ms)
{
    robot_app_shared_t copy = {0};
    scanner_echo_t echo = {0};
    uint32_t saved = 0U;
    bool b_valid = false;

    robot_app_copy(&copy);
    if (VEHICLE_SCAN_FINE == copy.vehicle.scan_request)
    {
        g_scan_flags.b_fine_requested = true;
    }
    board_ultrasonic_service();
    echo.b_ready = board_ultrasonic_result(&echo.pulse_us, &b_valid);
    if (!b_valid)
    {
        echo.pulse_us = 0U;
    }
    if ((!g_scan_flags.b_busy) &&
        (VEHICLE_SCAN_COARSE == copy.vehicle.scan_request) &&
        (!g_scan_flags.b_result_ready))
    {
        g_scan_flags.b_busy = scanner_start(&g_scanner, now_ms,
                                            copy.motion.b_stationary);
        g_scan_started_ms = now_ms;
        g_scan_flags.b_front_pending = false;
    }
    if ((copy.faults.b_stop_latched) || (copy.faults.b_hard))
    {
        if (g_scan_flags.b_busy)
        {
            g_scan_flags.b_busy = false;
            g_scanner.phase = SCANNER_ABORTED;
            g_scan_centred_ms = now_ms;
            (void)robot_app_servo(0);
        }
    }
    if (g_scan_flags.b_busy)
    {
        robot_app_scan_cycle(&copy, now_ms, echo);
    }
    else if ((now_ms - g_scan_centred_ms) >= ROBOT_SERVO_SETTLE_MS)
    {
        if (echo.b_ready && g_scan_flags.b_front_pending)
        {
            g_scan_flags.b_front_pending = false;
            robot_app_front_result(b_valid, echo.pulse_us);
        }
        /* A false return means the board's bounded interval is not ready. */
        if (board_ultrasonic_trigger())
        {
            g_scan_flags.b_front_pending = true;
            g_front_trigger_pose = copy.motion.pose;
            g_front_trigger_ms = now_ms;
        }
    }
    else
    {
        /* The servo is still settling after centring: no front trigger yet. */
    }
    if (g_scan_flags.b_fine_requested && g_scan_flags.b_result_ready &&
        (!g_scan_flags.b_fine_reported) &&
        ((now_ms - g_scan_centred_ms) >= ROBOT_SERVO_SETTLE_MS) &&
        (CORE_RANGE_VALID == copy.front.range.state) &&
        ((now_ms - copy.range_ms) < 150U))
    {
        /* One event per request: a 2-bit counter bumped every 2 ms could
         * alias back to the value Vehicle last saw between its 5-10 ms reads.
         */
        g_scan_flags.b_fine_reported = true;
        saved = app_main_lock();
        g_shared.front.fine_done =
            (g_shared.front.fine_done + 1U) & ROBOT_APP_DONE_MASK;
        app_main_unlock(saved);
    }
    if (VEHICLE_SCAN != copy.vehicle.state)
    {
        g_scan_flags.b_result_ready = false;
        g_scan_flags.b_fine_requested = false;
        g_scan_flags.b_fine_reported = false;
    }
    robot_app_health(ROBOT_APP_HEALTH_SCAN, now_ms);
}
#endif
#if !ROBOT_APP_MISSION

/** @brief A qualified range taken at (nearly) the current pose. */
bool robot_app_range_valid (const robot_app_shared_t * p_copy)
{
    core_real_t dx_m = 0.0;
    core_real_t dy_m = 0.0;
    core_real_t dyaw_rad = 0.0;

    core_pose_delta(&p_copy->motion.pose, &p_copy->front.pose, &dx_m,
                    &dy_m, &dyaw_rad);
    return (CORE_RANGE_VALID == p_copy->front.range.state) &&
           (fabs(dyaw_rad) <= 0.12) && (hypot(dx_m, dy_m) <= 0.03);
}

/** @brief Combined scanner completes both passes before staged events. */
void robot_app_scan_cycle (const robot_app_shared_t * p_copy,
                                  uint32_t now_ms, scanner_echo_t echo)
{
    scanner_output_t output = {0};
    scanner_shared_t shared;
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
        robot_app_fault_set(true, ROBOT_APP_BIT_SCAN);
        g_scan_flags.b_busy = false;
        return;
    }
    if (output.b_done)
    {
        g_scan_flags.b_busy = false;
        g_scan_flags.b_result_ready = true;
        g_scan_centred_ms = now_ms;
        scanner_shared_pack(&g_scanner.result, &shared);
        saved = app_main_lock();
        g_shared.scan = shared;
        g_shared.front.coarse_done =
            (g_shared.front.coarse_done + 1U) & ROBOT_APP_DONE_MASK;
        g_shared.front.range.state = CORE_RANGE_UNKNOWN;
        g_shared.front.range.distance_mm = 0U;
        app_main_unlock(saved);
    }
}
#endif

/** @brief Publish one front reading with its trigger pose; NO_ECHO follows
 * the latest reading and changes only through the locked status helper.
 */
void robot_app_front_result (bool b_valid, uint32_t pulse_us)
{
    core_range_sample_t range = {CORE_RANGE_UNKNOWN, 0.0};
    core_range_packed_t packed;
    uint32_t saved = 0U;

    /* uint32_t microseconds convert exactly, then scale to seconds. */
    (void)core_ultrasonic_range(&g_scanner.config.range, b_valid,
        (double)pulse_us / 1000000.0, &range);
    core_range_pack(&range, &packed);
    saved = app_main_lock();
    g_shared.front.range = packed;
    g_shared.range_ms = g_front_trigger_ms;
    g_shared.front.pose = g_front_trigger_pose;
    app_main_unlock(saved);
    if (b_valid == g_scan_flags.b_no_echo)
    {
        g_scan_flags.b_no_echo = !b_valid;
        robot_app_status_set(ROBOT_APP_BIT_NO_ECHO, !b_valid);
    }
}

/** @brief Calibrated zero and sign map the +/-ROBOT_SCAN_HALF_ANGLE_DEG beam
 * sector to PWM. */
bool robot_app_servo (int32_t angle_deg)
{
    int32_t pulse_us = 0;
    int32_t physical_deg = 0;
    int32_t span_us = 0;

    if ((angle_deg < -ROBOT_SCAN_HALF_ANGLE_DEG) ||
        (angle_deg > ROBOT_SCAN_HALF_ANGLE_DEG) ||
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
        ((physical_deg * span_us) / ROBOT_SCAN_HALF_ANGLE_DEG);
    /* Validated endpoints bound the result within the uint16_t board API. */
    return board_servo_us((uint16_t)pulse_us);
}

/* End of file. */

