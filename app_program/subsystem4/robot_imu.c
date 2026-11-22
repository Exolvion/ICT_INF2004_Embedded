/** @file robot_imu.c
 * @brief The robot app: subsystem 4: the IMU task (accelerometer, terrain,
 * tilt and collision, its evidence). Split out of the one file by
 * tools/reorg/split_c.py (22 Nov 2026); what the parts share is in
 * robot_app_private.h.
 */
#include "robot_app_private.h"
#include "robot_imu.h"

/* Mission: the IMU task, which owns the I2C bus, also polls the compass for
 * the mission's report every this many steps (100 ms).
 */
#define ROBOT_APP_MAG_STEPS (10U)
#define ROBOT_APP_IMU_RUN (3U)       /* IMU, 10 ms: 30 ms of failed reads */
#define ROBOT_APP_COLLISION_RUN (2U) /* IMU: 20 ms of collision deceleration */

/* IMU task: its evidence (under app_main_lock) and the window it is for. */
static robot_app_imu_evidence_t g_imu_evidence;
static uint32_t g_imu_evidence_window;
static uint8_t g_noise_imu;
           /* IMU: failed or saturated read */
static uint8_t g_noise_collision;
static double g_imu_speed_mps;
static uint16_t g_imu_path_hc;
static uint16_t g_imu_window_path_hc;
static uint8_t g_imu_run_id;
#if ROBOT_APP_MISSION
   /* Scan: trigger request taken */
static uint8_t g_mag_steps;
#endif

/** @brief Fresh calibrated accelerometer sample with encoder path interval. */
void robot_app_imu_step (uint32_t now_ms)
{
    robot_app_shared_t copy = {0};
    terrain_input_t input = {0};
    terrain_output_t output = {0};
    motion_output_t motion = {0};
    int16_t raw[3] = {0};
    double axes[3] = {0.0};
    double speed_mps = 0.0;
    uint16_t path_hc = 0U;
    /* The read's wall time, preemption included (the evidence). */
    uint64_t start_us = board_time_us();
    board_accel_status_t sample_status = board_read_accel(raw);
    uint32_t read_us = (uint32_t)(board_time_us() - start_us);
    bool b_valid = BOARD_ACCEL_FRESH == sample_status;

    robot_app_copy(&copy);
    robot_app_imu_note(ROBOT_APP_IMU_READ, read_us);
#if ROBOT_APP_MISSION
    /* This task owns the I2C bus, so the mission's compass is read here too:
     * a read from the higher-priority Vehicle task could cut into this
     * task's accelerometer read on the same controller and fail it.
     */
    ++g_mag_steps;
    if (g_mag_steps >= ROBOT_APP_MAG_STEPS)
    {
        g_mag_steps = 0U;
        mission_mag_poll();
    }
#endif
    /* Independent sensor clock: PENDING keeps its age; an overrun is FRESH,
     * dt spanning the lost sample. Gaps beyond the dt bound cannot drive.
     */
    input.dt_s = (double)(now_ms - copy.imu_ms) / 1000.0;
    if ((BOARD_ACCEL_PENDING == sample_status) &&
        (input.dt_s <= g_terrain.config.maximum_dt_s))
    {
        robot_app_health(ROBOT_APP_HEALTH_IMU, now_ms);
        return;
    }
    if (!b_valid)
    {
        robot_app_imu_missed(sample_status, raw, read_us, copy.vehicle.state,
                             now_ms);
        return;
    }
    robot_app_imu_note(ROBOT_APP_IMU_GAP, now_ms - copy.imu_ms);
    /* A good read drains the failed-read level (noise control). */
    (void)robot_app_noise(&g_noise_imu, false, 0U);
#if ROBOT_APP_MISSION
    /* The mission's hump and lift checks read every fresh sample too. */
    mission_accel_push(raw, (uint32_t)(start_us + read_us));
#endif
    input.b_encoder_valid = robot_app_imu_path(&copy, now_ms, &path_hc);
    robot_app_terrain_run(&copy, path_hc, now_ms);
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
    input.signed_distance_m =
        (double)robot_app_path_delta(path_hc, g_imu_path_hc) * g_half_count_m;
    motion_snapshot_unpack(&copy.motion, &motion);
    input.encoder_yaw_rate_rps =
        (motion.right_speed_mps - motion.left_speed_mps) / ROBOT_TRACK_WIDTH_M;
    speed_mps = (motion.left_speed_mps + motion.right_speed_mps) / 2.0;
    input.b_sensor_valid = b_valid;
    /* A 200ms displacement window reduces count quantization in speed.
     * The acceleration threshold is a tuning hypothesis, not gyro evidence.
     */
    if ((now_ms - g_imu_window_ms) >= 200U)
    {
        g_imu_speed_mps = ((double)robot_app_path_delta(path_hc,
                               g_imu_window_path_hc) * g_half_count_m) /
                          ((double)(now_ms - g_imu_window_ms) / 1000.0);
        g_imu_window_path_hc = path_hc;
        g_imu_window_ms = now_ms;
    }
    input.b_quasistatic = motion.b_stationary ||
        ((fabs(speed_mps - g_imu_speed_mps) < 0.15) &&
         (fabs(input.accel_x_mps2) < 4.0));
    g_imu_path_hc = path_hc;
    b_valid = terrain_step(&g_terrain, &input, &output) && b_valid;
    /* Noise control: a collision candidate on two samples in a row (20 ms)
     * is published; a single one is a jolt.
     */
    output.b_collision_candidate = robot_app_noise(&g_noise_collision,
        output.b_collision_candidate,
        ROBOT_APP_NOISE_LIMIT(ROBOT_APP_COLLISION_RUN));
    robot_app_imu_publish(&output, &copy, b_valid, now_ms);
    robot_app_imu_judge(b_valid, output.b_tilt_valid, input.dt_s,
                        copy.vehicle.state);
}

/** @brief A read that gave no fresh sample: PENDING past terrain's dt bound
 * (no new sample for that long), a failed bus read or a saturated sample.
 * The tilt is unusable either way. During a run a failed or saturated read
 * is a hard IMU fault. A pending sample is a soft TILT_UNAVAILABLE at once
 * in the supervisor; the mission leaves it to Safety, which stops the run
 * after ROBOT_APP_TILT_TIMEOUT_MS without a valid tilt.
 */
void robot_app_imu_missed (board_accel_status_t status,
                                  const int16_t raw[3], uint32_t read_us,
                                  vehicle_state_t state, uint32_t now_ms)
{
    uint32_t saved = app_main_lock();
    bool b_pending = BOARD_ACCEL_PENDING == status;
    bool b_saturated = false;
    bool b_fault = false;
    uint32_t axis = 0U;

    g_shared.terrain.b_tilt_valid = false;
    g_shared.health[ROBOT_APP_HEALTH_IMU] = now_ms;
    app_main_unlock(saved);
    if (!b_pending)
    {
        /* board_read_accel writes the axes only once the data read worked,
         * so a bus failure leaves them all zero (inside the range).
         */
        for (axis = 0U; axis < 3U; ++axis)
        {
            b_saturated = b_saturated || (raw[axis] <= -2048) ||
                          (raw[axis] >= 2047);
        }
        robot_app_imu_note(b_saturated ? ROBOT_APP_IMU_SATURATED :
                           ROBOT_APP_IMU_FAILED, read_us);
        /* Noise control: three failed or saturated reads in a row (30 ms);
         * a single one only leaves the tilt invalid for that sample.
         */
        b_fault = robot_app_noise(&g_noise_imu, true,
                                  ROBOT_APP_NOISE_LIMIT(ROBOT_APP_IMU_RUN));
    }
#if ROBOT_APP_MISSION
    if (b_fault && robot_app_active(state))
#else
    if ((b_pending || b_fault) && robot_app_active(state))
#endif
    {
        robot_app_fault_set(!b_pending, b_pending ?
            ROBOT_APP_BIT_TILT_UNAVAILABLE : ROBOT_APP_BIT_IMU);
    }
}

/** @brief After terrain_step: a sample terrain refused (b_valid false; late,
 * past its dt bound, or unusable) is a hard IMU fault during a supervisor
 * run, and a sample without tilt a soft TILT_UNAVAILABLE. The mission only
 * counts the refusal: the tilt stays invalid, and Safety stops the run after
 * ROBOT_APP_TILT_TIMEOUT_MS without a valid one (a jolt, or a late sample
 * after a long higher-priority step, is not a sensor fault).
 */
void robot_app_imu_judge (bool b_valid, bool b_tilt_valid,
                                 double dt_s, vehicle_state_t state)
{
    if (!b_valid)
    {
        robot_app_imu_note((dt_s > g_terrain.config.maximum_dt_s) ?
                           ROBOT_APP_IMU_LATE : ROBOT_APP_IMU_REFUSED, 0U);
    }
#if ROBOT_APP_MISSION
    (void)b_tilt_valid;
    (void)state;
#else
    if ((!b_valid) && robot_app_active(state))
    {
        robot_app_fault_set(true, ROBOT_APP_BIT_IMU);
    }
    else if ((!b_tilt_valid) && robot_app_active(state))
    {
        /* Unusable orientation brakes; recovery needs local reset. */
        robot_app_fault_set(false, ROBOT_APP_BIT_TILT_UNAVAILABLE);
    }
    else
    {
        /* A valid sample with tilt. */
    }
#endif
}

/** @brief Add one event to this window's IMU evidence (IMU task); the first
 * event of a new robot_app_timing_restart() window clears the old record.
 */
void robot_app_imu_note (robot_app_imu_event_t event, uint32_t value)
{
    robot_app_imu_evidence_t evidence = {0};
    /* GAP gives ms, READ and FAILED us; 16-bit fields, held at the top. */
    uint16_t value16 = (uint16_t)((value > UINT16_MAX) ? UINT16_MAX : value);
    uint32_t saved = app_main_lock();

    if (g_imu_evidence_window == g_timing_window)
    {
        evidence = g_imu_evidence;
    }
    switch (event)
    {
        case ROBOT_APP_IMU_READ:
            if (value16 > evidence.slowest_us)
            {
                evidence.slowest_us = value16;
            }
            break;
        case ROBOT_APP_IMU_FAILED:
            evidence.failed = robot_app_count8(evidence.failed);
            evidence.failed_us = value16;
            break;
        case ROBOT_APP_IMU_SATURATED:
            evidence.saturated = robot_app_count8(evidence.saturated);
            break;
        case ROBOT_APP_IMU_LATE:
            evidence.late = robot_app_count8(evidence.late);
            break;
        case ROBOT_APP_IMU_REFUSED:
            evidence.refused = robot_app_count8(evidence.refused);
            break;
        case ROBOT_APP_IMU_GAP:
            /* The interval since the last fresh sample. */
            if (value16 > evidence.longest_ms)
            {
                evidence.longest_ms = value16;
            }
            break;
        default:
            /* No other event. */
            break;
    }
    g_imu_evidence = evidence;
    g_imu_evidence_window = g_timing_window;
    app_main_unlock(saved);
}

/** @brief An 8-bit count plus one, held at 255. */
uint8_t robot_app_count8 (uint32_t count)
{
    /* Below 255 the sum fits; at or above it the count holds at 255. */
    return (count < UINT8_MAX) ? (uint8_t)(count + 1U) : (uint8_t)UINT8_MAX;
}

/** @brief The IMU evidence of the current window (zero before its first
 * event).
 */
void robot_app_imu_evidence (robot_app_imu_evidence_t * p_output)
{
    robot_app_imu_evidence_t evidence = {0};
    uint32_t saved = 0U;

    if (NULL == p_output)
    {
        return;
    }
    saved = app_main_lock();
    if (g_imu_evidence_window == g_timing_window)
    {
        evidence = g_imu_evidence;
    }
    app_main_unlock(saved);
    *p_output = evidence;
}

/** @brief Imu owner with absolute-deadline periodic release. */
void robot_app_imu_task (void)
{
    robot_app_periodic(robot_app_imu_step, 10U, 4U);
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
bool robot_app_imu_path (const robot_app_shared_t * p_copy,
                                uint32_t now_ms, uint16_t * p_path_hc)
{
    uint32_t stamp_ms = p_copy->path_ms;

    *p_path_hc = p_copy->path_hc;
    if ((now_ms - stamp_ms) >= ROBOT_APP_SERIAL_HALF)
    {
        stamp_ms = p_copy->previous_path_ms;
        *p_path_hc = p_copy->previous_path_hc;
    }
    /* A stamp still newer than now_ms wraps to at least the serial half. */
    return (0.0 < g_half_count_m) && ((now_ms - stamp_ms) <= 20U);
}

/** @brief Pack the terrain record outside the lock, then publish it with
 * the IMU stamps in one critical section. The mission maximum is an
 * integer maximum of floor-mm values, equal to the floor of the old fmax.
 */
void robot_app_imu_publish (const terrain_output_t * p_output,
                                   const robot_app_shared_t * p_copy,
                                   bool b_valid, uint32_t now_ms)
{
    terrain_shared_t shared;
    uint32_t mission_mm = terrain_height_mm(p_output->run_maximum_m);
    uint32_t saved = 0U;

    if (p_copy->terrain.mission_maximum_mm > mission_mm)
    {
        mission_mm = p_copy->terrain.mission_maximum_mm;
    }
    terrain_shared_pack(p_output, mission_mm,
                        p_copy->terrain.b_mission_valid ||
                        (0U != p_output->completed_humps), &shared);
    saved = app_main_lock();
    g_shared.terrain = shared;
    g_shared.imu_ms = now_ms;
    if (b_valid && p_output->b_tilt_valid)
    {
        g_shared.tilt_ms = now_ms;
    }
    g_shared.health[ROBOT_APP_HEALTH_IMU] = now_ms;
    app_main_unlock(saved);
}

/** @brief IMU owner resets run history while retaining its measured mount.
 * path_hc is the encoder path robot_app_imu_path selected for the release
 * now_ms. Both 200 ms speed-window bases refer to that same instant.
 */
void robot_app_terrain_run (const robot_app_shared_t * p_copy,
                                   uint16_t path_hc, uint32_t now_ms)
{
    terrain_config_t config = g_terrain.config;
    double pitch_reference = g_terrain.pitch_reference_rad;
    double roll_reference = g_terrain.roll_reference_rad;
    bool b_reference_valid = g_terrain.b_reference_valid;

    if (p_copy->vehicle.run_id != g_imu_run_id)
    {
        if (!terrain_init(&g_terrain, &config))
        {
            robot_app_fault_set(true, ROBOT_APP_BIT_IMU);
        }
        g_terrain.pitch_reference_rad = pitch_reference;
        g_terrain.roll_reference_rad = roll_reference;
        g_terrain.b_reference_valid = b_reference_valid;
        /* The run counter field holds eight bits. */
        g_imu_run_id = (uint8_t)p_copy->vehicle.run_id;
        g_imu_path_hc = path_hc;
        g_imu_window_path_hc = path_hc;
        g_imu_speed_mps = 0.0;
        g_imu_window_ms = now_ms;
    }
}

/* End of file. */

