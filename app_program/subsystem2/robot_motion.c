/** @file robot_motion.c
 * @brief The robot app: subsystem 2: the Motion task (wheel controller,
 * encoders, stalls, the approved reset). Split out of the one file by
 * tools/reorg/split_c.py (22 Nov 2026); what the parts share is in
 * robot_app_private.h.
 */
#include "robot_app_private.h"
#include "robot_motion.h"
#define ROBOT_APP_COUNT_HALF (UINT32_C(0x8000))
#define ROBOT_APP_COUNT_RANGE (INT32_C(0x10000))
#define ROBOT_APP_ENCODER_RUN (3U)   /* Motion, 5 ms: 15 ms of edge errors */
#define ROBOT_APP_MOTION_RUN (3U)    /* Motion: 3 refusals/errors in a row */
#define ROBOT_APP_OUTPUT_RUN (3U)    /* Motion: 3 unprepared outputs in a row */
         /* Safety: OWNER_TIMEOUT */
static uint8_t g_noise_encoder;
       /* Motion: ENCODER */
static uint8_t g_noise_submit;
        /* Motion: refused request (MOTION) */
static uint8_t g_noise_motion;
        /* Motion: controller error (MOTION) */
static uint8_t g_noise_output;
static uint32_t g_left_stall_ms;
static uint32_t g_right_stall_ms;
static uint8_t g_motion_version;

/** @brief One encoder/PID owner; a renewal is accepted only once per
 * publish. */
void robot_app_motion_step (uint32_t now_ms)
{
    robot_app_shared_t copy = {0};
    motion_output_t output = {0};
    motion_request_t request = {0};
    int32_t left_delta = 0;
    int32_t right_delta = 0;
    uint32_t errors = 0U;
    uint64_t now_us = board_time_us();
    double dt_s = (double)(now_us - g_motion_us) / 1000000.0;
    motion_status_t status = MOTION_OK;

    /* A bounded microsecond interval is accurately rounded to binary64. */
    g_motion_us = now_us;
    robot_app_copy(&copy);
    board_read_encoders(&left_delta, &right_delta, &errors);
    left_delta = robot_app_signed_count(left_delta,
                                        ROBOT_LEFT_ENCODER_SIGN);
    right_delta = robot_app_signed_count(right_delta,
                                         ROBOT_RIGHT_ENCODER_SIGN);
    /* Noise control: an edge error in one read is noise (the board counts
     * no displacement for it and resynchronises); three reads in a row
     * latch. Mission: soft (the team's choice, 22 Nov). It still brakes and
     * ends a run at once; the car goes back to IDLE by itself once it
     * stands still, unless the errors keep coming (each one re-latches, so
     * the approved reset is never clean).
     */
    if (robot_app_noise(&g_noise_encoder, 0U != errors,
                        ROBOT_APP_NOISE_LIMIT(ROBOT_APP_ENCODER_RUN)))
    {
        robot_app_fault_set(0 == ROBOT_APP_MISSION, ROBOT_APP_BIT_ENCODER);
        copy.faults.b_inhibit = true;
    }
    if ((copy.faults.b_inhibit) &&
        (MOTION_INHIBITED != g_motion.output.state))
    {
        motion_stop(&g_motion, MOTION_STOPPED);
    }
    if (copy.vehicle.b_reset_motion)
    {
        robot_app_reset(&copy);
    }
    if (g_motion_version != copy.vehicle.request_version)
    {
        /* The version field holds four bits, which fit uint8_t. */
        g_motion_version = (uint8_t)copy.vehicle.request_version;
        motion_request_unpack(&copy.vehicle.request, &request);
        status = motion_submit(&g_motion, &request, now_ms);
        /* Noise control: three refused requests in a row. */
        if (robot_app_noise(&g_noise_submit, (MOTION_OK != status) &&
                            (!copy.faults.b_inhibit),
                            ROBOT_APP_NOISE_LIMIT(ROBOT_APP_MOTION_RUN)))
        {
            robot_app_fault_set(true, ROBOT_APP_BIT_MOTION);
        }
    }
    status = motion_step(&g_motion, now_ms, dt_s,
                         left_delta, right_delta, &output);
    /* Noise control: three controller errors in a row (a controller that
     * stopped itself keeps reporting its reason, so that still latches).
     */
    if (robot_app_noise(&g_noise_motion, (MOTION_OK != status) &&
                        (MOTION_STOPPED != status) &&
                        (MOTION_REJECTED != status),
                        ROBOT_APP_NOISE_LIMIT(ROBOT_APP_MOTION_RUN)))
    {
        robot_app_fault_set(true, ROBOT_APP_BIT_MOTION);
    }
    robot_app_motion_health(&output, left_delta, right_delta, now_ms);
    robot_app_commit(&output, left_delta, right_delta, now_ms);
}

/** @brief Unconfirmed calibration is a normal inhibited configuration. */
bool robot_app_motion_init (void)
{
    motion_config_t config = {0};
    motion_status_t result = MOTION_INVALID;
    core_pose_t pose = {0.0, 0.0, 0.0, 0.0};
    core_odometry_result_t odometry;

#if ROBOT_APP_MISSION
    /* app_cfg_motion() with the mission's PID gains, step limit and wheel
     * speed cap (the car-tested set, mission.c bench_motion_config).
     */
    mission_motion_config(&config);
#else
    app_cfg_motion(&config);
#endif
    result = motion_init(&g_motion, &config);
    /* Metres per half-count (left + right counts) for the IMU path; the
     * optical path stays valid only with a usable odometry configuration.
     */
    gb_optical_valid = CORE_OK == core_odometry_step(
        &g_motion.config.odometry, &pose, 0, 0, &odometry);
    g_half_count_m = gb_optical_valid ?
        ((CORE_PI * (g_motion.config.odometry.left_radius_m +
                     g_motion.config.odometry.right_radius_m)) /
         (2.0 * (double)g_motion.config.odometry.left_counts_per_revolution))
        : 0.0;
    return (MOTION_OK == result) || (MOTION_REJECTED == result);
}

/** @brief Software STOP always wins over an older reset authorization.
 * The clean flag is read live and the stopping bits are cleared in the
 * same critical section, so a concurrent stop cannot be erased.
 */
void robot_app_reset (const robot_app_shared_t * p_copy)
{
    motion_status_t status = MOTION_REJECTED;
    uint32_t saved = 0U;
    bool b_approved = false;
    /* Mission RESET: a hard fault may be cleared as well (b_reset_hard). A
     * fault raised after Vehicle's snapshot still clears b_reset_clean.
     */
    bool b_hard_ok = p_copy->vehicle.b_reset_hard;

    status = motion_reset(&g_motion, p_copy->motion.b_stationary,
                           ((!p_copy->faults.b_hard) || b_hard_ok) &&
                           (!board_stop_pressed()));
    saved = app_main_lock();
    b_approved = (MOTION_OK == status) && g_shared.faults.b_reset_clean &&
        ((!g_shared.faults.b_hard) || b_hard_ok) && (!board_stop_pressed());
    g_shared.vehicle.b_reset_motion = false;
    g_shared.vehicle.b_reset_hard = false;
    if (b_approved)
    {
        g_shared.faults.b_hard = false;
        g_shared.faults.b_inhibit = false;
        g_shared.faults.b_stop_latched = false;
        g_shared.faults.first = ROBOT_APP_FAULT_NONE;
        g_shared.faults.stop_source = 0U;
        /* Stopping bits clear; their owners keep NO_ECHO/WIFI_LOSS current. */
        g_shared.faults.mask = g_shared.faults.mask & ROBOT_APP_STATUS_BITS;
        g_shared.first_fault_ms = 0U;
    }
    app_main_unlock(saved);
    if (!b_approved)
    {
        motion_stop(&g_motion, MOTION_STOPPED);
    }
}

/** @brief Separate wheels cannot hide each other's stalls; rest needs dwell. */
void robot_app_motion_health (motion_output_t * p_output,
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
        robot_app_fault_set(true,
            ((now_ms - g_left_stall_ms) > ROBOT_STALL_MS) ?
            ROBOT_APP_BIT_LEFT_STALL : ROBOT_APP_BIT_RIGHT_STALL);
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

/** @brief Prepare soft-float PWM and pack the snapshot outside masking,
 * then commit with the latest inhibit. A valid step's centre path is
 * (left + right) half-counts; an invalid step (signed distance stored as
 * exactly 0.0 by motion_step) adds nothing, as before.
 */
void robot_app_commit (const motion_output_t * p_output,
                              int32_t left_delta, int32_t right_delta,
                              uint32_t now_ms)
{
    board_motor_command_t command = {0};
    motion_snapshot_t snapshot;
    uint32_t saved = 0U;
    int32_t path_step = (0.0 != p_output->signed_distance_m) ?
                        (left_delta + right_delta) : 0;
    bool b_prepared = board_motor_prepare(p_output->left_duty,
                                          p_output->right_duty, &command);
#if ROBOT_APP_MISSION
    uint32_t left_pm = robot_app_duty_pm(p_output->left_duty);
    uint32_t right_pm = robot_app_duty_pm(p_output->right_duty);
#endif

    motion_snapshot_pack(p_output, &snapshot);
    /* Unconfirmed calibration is a normal inhibited configuration (for
     * example the wiring-only commissioning stage): a rejected command is
     * braked below without a hard fault. With the gate open, a preparation
     * failure is a genuine output fault.
     */
    /* Noise control: three unprepared outputs in a row (each one is braked
     * below meanwhile).
     */
    if (robot_app_noise(&g_noise_output, (!b_prepared) &&
                        g_motion.config.b_calibration_confirmed,
                        ROBOT_APP_NOISE_LIMIT(ROBOT_APP_OUTPUT_RUN)))
    {
        robot_app_fault_set(true, ROBOT_APP_BIT_MOTOR_OUTPUT);
    }
    saved = app_main_lock();
    g_shared.motion = snapshot;
#if ROBOT_APP_MISSION
    /* Already ROBOT_APP_DUTY_BITS wide (robot_app_duty_pm). */
    g_shared.left_duty_pm = left_pm & 0x3FFU;
    g_shared.right_duty_pm = right_pm & 0x3FFU;
#endif
    g_shared.previous_path_hc = g_shared.path_hc;
    g_shared.previous_path_ms = g_shared.path_ms;
    /* Modulo-2^16 addition keeps a wrapping path that the IMU differences. */
    g_shared.path_hc = (uint16_t)(g_shared.path_hc + (uint32_t)path_step);
    g_shared.path_ms = now_ms;
    if (g_shared.faults.b_inhibit || (!b_prepared))
    {
        board_motor_brake();
    }
    else
    {
        board_motor_commit(&command);
    }
    g_shared.health[ROBOT_APP_HEALTH_MOTION] = now_ms;
    app_main_unlock(saved);
}
#if ROBOT_APP_MISSION

/** @brief A duty as a ROBOT_APP_DUTY_BITS two's complement field in per
 * mille, nearest, saturated; non-finite is 0.
 */
uint32_t robot_app_duty_pm (core_real_t duty)
{
    int32_t limit = bitpack_signed_max(ROBOT_APP_DUTY_BITS);
    double scaled = isfinite(duty) ? (duty * 1000.0) : 0.0;
    int32_t value = 0;

    if (scaled > (double)limit)
    {
        value = limit;
    }
    else if (scaled < -(double)limit)
    {
        value = -limit;
    }
    else
    {
        /* Within +/-511 after the checks, so the conversion is exact. */
        value = (int32_t)lround(scaled);
    }
    return bitpack_signed(value, ROBOT_APP_DUTY_BITS) &
           bitpack_mask(ROBOT_APP_DUTY_BITS);
}
#endif

/** @brief A measured sign can reverse a count without signed-overflow UB. */
int32_t robot_app_signed_count (int32_t count, int32_t sign)
{
    if (((1 != sign) && (-1 != sign)) || (INT32_MIN == count))
    {
        robot_app_fault_set(true, ROBOT_APP_BIT_ENCODER);
        return 0;
    }
    return (1 == sign) ? count : -count;
}
#if !ROBOT_APP_MISSION

/** @brief Convert a 16-bit modulo position difference with an explicit
 * half-range. 32,767 counts (18.6 m) between 2 ms reads is unreachable, so
 * the ambiguous half-range value is an encoder fault.
 */
int32_t robot_app_count_delta (uint16_t newer, uint16_t older)
{
    uint32_t difference = (uint32_t)(uint16_t)(newer - older);

    if (ROBOT_APP_COUNT_HALF == difference)
    {
        robot_app_fault_set(true, ROBOT_APP_BIT_ENCODER);
        return 0;
    }
    return robot_app_path_delta(newer, older);
}
#endif

/** @brief Signed modulo-2^16 difference of two wrapping counts. */
int32_t robot_app_path_delta (uint16_t newer, uint16_t older)
{
    /* The masked difference is below 2^16 and fits int32_t exactly. */
    int32_t difference = (int32_t)(uint16_t)(newer - older);

    return (difference >= (int32_t)ROBOT_APP_COUNT_HALF) ?
           (difference - ROBOT_APP_COUNT_RANGE) : difference;
}

/* End of file. */

