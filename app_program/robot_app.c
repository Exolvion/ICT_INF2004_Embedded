/** @file robot_app.c
 * @brief The robot app: the shared record and its locks, the fault latch and
 * noise control, Safety, Vehicle (the mission's glue), task timing and the
 * task entries. Split out of the one file by tools/reorg/split_c.py (22 Nov
 * 2026); what the parts share is in robot_app_private.h.
 */
#include "robot_app.h"
#include "robot_app_private.h"
#if ROBOT_APP_MISSION
#define ROBOT_APP_TILT_TIMEOUT_MS (500U)
#endif
#if !(ROBOT_APP_MISSION)
#define ROBOT_APP_TILT_TIMEOUT_MS (100U)
#endif
#if ROBOT_APP_MISSION
#define ROBOT_APP_VEHICLE_GATE_MS (100U)
#endif
#if !(ROBOT_APP_MISSION)
#define ROBOT_APP_VEHICLE_GATE_MS (40U)
#endif
#define ROBOT_APP_OWNER_RUN (3U)     /* Safety, 2 ms: 6 ms over a gate */
#define ROBOT_APP_STOP_RUN (3U)      /* Safety, 2 ms: GP20 held 6 ms */
#define ROBOT_APP_TILT_RUN (50U)     /* Safety, 2 ms: 100 ms over the limit */
#define ROBOT_APP_STEER_SCALE (127.0)
#define ROBOT_APP_MASK_BITS (16U)
#define ROBOT_APP_SOURCE_MASK (3U)
#define ROBOT_APP_VERSION_MASK (0xFU)

/* A START waits this long for its reset to clear a soft stop. */
#define ROBOT_APP_START_WAIT_MS (2000U)

/* First cause reported for each mask bit; status bits never stop. */
static const robot_app_fault_t g_robot_app_causes[ROBOT_APP_MASK_BITS] =
{
    ROBOT_APP_FAULT_OWNER_TIMEOUT, ROBOT_APP_FAULT_ENCODER,
    ROBOT_APP_FAULT_MOTION, ROBOT_APP_FAULT_LEFT_STALL,
    ROBOT_APP_FAULT_RIGHT_STALL, ROBOT_APP_FAULT_MOTOR_OUTPUT,
    ROBOT_APP_FAULT_OPTICAL, ROBOT_APP_FAULT_OPTICAL,
    ROBOT_APP_FAULT_SUPERVISOR, ROBOT_APP_FAULT_IMU, ROBOT_APP_FAULT_TILT,
    ROBOT_APP_FAULT_TILT_UNAVAILABLE, ROBOT_APP_FAULT_COLLISION,
    ROBOT_APP_FAULT_SCAN, ROBOT_APP_FAULT_NONE, ROBOT_APP_FAULT_NONE
};

/** One task's timing record (robot_app_periodic): only that task writes it;
 * window follows g_timing_window, which only the caller of
 * robot_app_timing_restart (the Vehicle task) advances.
 */
typedef struct
{
    uint32_t worst_us;
    uint32_t skipped;
    uint32_t own_us;
    uint32_t steps;
    uint32_t window;
} robot_app_timing_t;

static robot_app_timing_t g_timing[ROBOT_APP_TASKS];
uint32_t g_timing_window;
/* Own time of every finished step, all tasks (us, wraps): a step's growth
 * of it is the time of the steps that ran inside it. Changed only under
 * app_main_lock().
 */
static uint32_t g_timing_nested_us;
/* When the window began (robot_app_timing_restart, the Vehicle task). */
static uint64_t g_timing_start_us;

/* Safety task: the owners' longest health gaps, and their window. */
static robot_app_owner_evidence_t g_owner_evidence;
static uint32_t g_owner_evidence_window;

/* Noise levels (robot_app_noise), each written by its one owner task. */
static uint8_t g_noise_stop;
          /* Safety: GP20 STOP */
static uint8_t g_noise_tilt;
          /* Safety: TILT */
static uint8_t g_noise_owner;
     /* IMU: COLLISION */
robot_app_shared_t g_shared;
motion_t g_motion;
static vehicle_t g_vehicle;
line_ctrl_runtime_t g_line_ctrl_runtime;
line_ctrl_state_t g_line_ctrl;
barcode_runtime_t g_barcode_runtime;
barcode_state_t g_barcode;
terrain_state_t g_terrain;
scanner_state_t g_scanner;
#if !ROBOT_APP_MISSION
core_pose_packed_t g_route_anchor;
#endif
uint64_t g_motion_us;
double g_half_count_m;
uint32_t g_scan_centred_ms;
uint32_t g_stationary_since_ms;
uint32_t g_imu_window_ms;
static uint32_t g_button_since_ms;
uint16_t g_line_left_total;
uint16_t g_line_right_total;
robot_app_vehicle_flags_t g_vehicle_flags;
#if ROBOT_APP_MISSION
static mission_input_t g_mission_in;
  /* Vehicle: the mission's input */
static uint32_t g_start_since_ms;
     /* Vehicle: a START waiting */
static uint32_t g_yield_ms;
           /* IMU: steps since the compass poll */
static bool gb_start_pending;
#endif

/** @brief Initialize statically-owned modules, then bounded network queues.
 * Runs before any task starts, so g_shared needs no lock here; the GPIO ISR
 * never writes g_shared.
 */
bool robot_app_init (void)
{
    vehicle_config_t config = {0};
    uint32_t now_ms = robot_app_now();
    uint32_t index = 0U;
    bool b_ready = true;

    memset(&g_shared, 0, sizeof(g_shared));
    /* All-zero fields already mean VEHICLE_INIT, an UNKNOWN front range,
     * invalid tilt, no fault, no stop source and no pending event.
     */
    g_shared.faults.b_inhibit = true;
    for (index = 0U; index < (uint32_t)ROBOT_APP_HEALTH_COUNT; ++index)
    {
        g_shared.health[index] = now_ms;
    }
    barcode_init(&g_barcode);
    app_cfg_vehicle(&config,
        robot_app_codebook_ready());
    b_ready = vehicle_init(&g_vehicle, &config);
    b_ready = robot_app_motion_init() && b_ready;
    b_ready = robot_app_sensors_init() && b_ready;
    b_ready = net_mqtt_start() && b_ready;
    g_shared.faults.b_inhibit = !g_motion.config.b_calibration_confirmed;
    g_shared.line_ms = now_ms;
    g_shared.imu_ms = now_ms;
    g_shared.tilt_ms = now_ms;
    g_shared.path_ms = now_ms;
    g_shared.previous_path_ms = now_ms;
    g_stationary_since_ms = now_ms;
    g_imu_window_ms = now_ms;
    g_vehicle_flags.b_button_latched = board_start_pressed();
    g_vehicle_flags.b_button_down = g_vehicle_flags.b_button_latched;
    board_encoder_totals(&g_line_left_total, &g_line_right_total);
    /* Counts since board_init() would otherwise saturate the 6-bit ISR
     * deltas into a false hard ENCODER fault on the first Motion read.
     */
    board_encoders_discard();
    g_motion_us = board_time_us();
    g_scan_centred_ms = now_ms;
#if ROBOT_APP_MISSION
    /* Its BOOT line and the servo ahead; the tasks have not started. */
    mission_init();
    g_yield_ms = now_ms;
#endif
    return b_ready;
}

/** @brief A bounded snapshot of the first cause, mask and stop sources. */
void robot_app_diagnostic (robot_app_diagnostic_t * p_output)
{
    robot_app_faults_t faults;
    uint32_t first_fault_ms = 0U;
    uint32_t saved = 0U;

    if (NULL != p_output)
    {
        saved = app_main_lock();
        faults = g_shared.faults;
        first_fault_ms = g_shared.first_fault_ms;
        app_main_unlock(saved);
        memset(p_output, 0, sizeof(*p_output));
        p_output->first_fault_ms = first_fault_ms;
        /* The mask field holds exactly sixteen bits. */
        p_output->mask = (uint16_t)faults.mask;
        p_output->code = faults.first;
        p_output->stop_source = faults.stop_source & ROBOT_APP_SOURCE_MASK;
    }
}

/** @brief STOP preempts normal output; stale local owners latch a hard
 * fault. */
void robot_app_safety_step (uint32_t now_ms)
{
    terrain_shared_t terrain;
    uint32_t health[ROBOT_APP_HEALTH_COUNT] = {0U};
    uint32_t tilt_ms = 0U;
    vehicle_state_t state = VEHICLE_IDLE;
    uint32_t saved = 0U;
    bool b_stale = false;

    /* Only the fields Safety checks, in one short masked region: this runs
     * every 2 ms at the highest priority, so it does not copy the record.
     */
    saved = app_main_lock();
    terrain = g_shared.terrain;
    (void)memcpy(health, g_shared.health, sizeof(health));
    tilt_ms = g_shared.tilt_ms;
    state = g_shared.vehicle.state;
    app_main_unlock(saved);
    /* Noise control: a spike on GP20 is not a press (held 6 ms is). */
    if (robot_app_noise(&g_noise_stop, board_stop_pressed(),
                        ROBOT_APP_NOISE_LIMIT(ROBOT_APP_STOP_RUN)))
    {
        robot_app_stop_set(ROBOT_APP_STOP_LOCAL);
    }
    /* A valid measured limit violation remains hard even after a soft halt.
     * Mission: soft (the team's choice, 22 Nov): it still brakes and ends a
     * run at once, and a car lifted or carried tilted goes back to IDLE by
     * itself once it stands level and still again (robot_app_vehicle_step).
     */
    /* Noise control: over the limit for 100 ms (about ten IMU samples). */
    if (robot_app_noise(&g_noise_tilt, terrain.b_tilt_valid &&
                        robot_app_tilt_exceeded(&terrain),
                        ROBOT_APP_NOISE_LIMIT(ROBOT_APP_TILT_RUN)))
    {
#if ROBOT_APP_MISSION
        robot_app_fault_set(false, ROBOT_APP_BIT_TILT);
#else
        robot_app_fault_set(true, ROBOT_APP_BIT_TILT);
#endif
    }
    robot_app_owner_note(health, now_ms);
    if (robot_app_active(state))
    {
        b_stale = (robot_app_age(now_ms,
                       health[ROBOT_APP_HEALTH_MOTION]) > 20U) ||
                  (robot_app_age(now_ms,
                       health[ROBOT_APP_HEALTH_LINE]) > 20U) ||
                  (robot_app_age(now_ms,
                       health[ROBOT_APP_HEALTH_VEHICLE]) >
                   ROBOT_APP_VEHICLE_GATE_MS) ||
                  (robot_app_age(now_ms,
                       health[ROBOT_APP_HEALTH_IMU]) > 100U) ||
                  (robot_app_age(now_ms,
                       health[ROBOT_APP_HEALTH_SCAN]) > 100U);
        /* Noise control: over a gate on three checks in a row. COLLISION
         * is filtered where each sample is judged (robot_app_imu_step).
         */
        b_stale = robot_app_noise(&g_noise_owner, b_stale,
                                  ROBOT_APP_NOISE_LIMIT(ROBOT_APP_OWNER_RUN));
        if (b_stale || terrain.b_collision_candidate)
        {
            robot_app_fault_set(true, b_stale ?
                ROBOT_APP_BIT_OWNER_TIMEOUT : ROBOT_APP_BIT_COLLISION);
        }
#if ROBOT_APP_MISSION
        /* The last valid tilt is the gate: a jolt's samples are skipped. */
        else if (robot_app_age(now_ms, tilt_ms) > ROBOT_APP_TILT_TIMEOUT_MS)
#else
        else if ((!terrain.b_tilt_valid) ||
                 (robot_app_age(now_ms, tilt_ms) >
                  ROBOT_APP_TILT_TIMEOUT_MS))
#endif
        {
            robot_app_fault_set(false, ROBOT_APP_BIT_TILT_UNAVAILABLE);
        }
        else
        {
            /* Owners fresh and tilt current: nothing to latch. */
        }
    }
    else
    {
        /* No run: the owner gates do not apply, so the level drains. */
        (void)robot_app_noise(&g_noise_owner, false, 0U);
    }
    /* Motion, Line, Vehicle, IMU, Scan and Comm gate the feed.
     * Telemetry and the radio owner that receives MQTT STOP (cyw43_utk_task,
     * P9) do not: remote STOP is best-effort.
     */
    if ((robot_app_age(now_ms, health[ROBOT_APP_HEALTH_MOTION]) <=
         20U) &&
        (robot_app_age(now_ms, health[ROBOT_APP_HEALTH_LINE]) <= 20U) &&
        (robot_app_age(now_ms, health[ROBOT_APP_HEALTH_VEHICLE]) <=
         ROBOT_APP_VEHICLE_GATE_MS) &&
        (robot_app_age(now_ms, health[ROBOT_APP_HEALTH_IMU]) <= 100U) &&
        (robot_app_age(now_ms, health[ROBOT_APP_HEALTH_SCAN]) <=
         100U) &&
        (robot_app_age(now_ms, health[ROBOT_APP_HEALTH_COMM]) <= 100U))
    {
        board_watchdog_feed();
    }
}
#if ROBOT_APP_MISSION

/** @brief The mission's step. START, from the button or MQTT (a deviation
 * from FR10 the team chose: remote START as well as remote STOP), runs it;
 * after a soft stop the START first asks Motion for the approved reset
 * (FR11) and waits for it up to ROBOT_APP_START_WAIT_MS. A latched stop or
 * fault (the fault mask) ends a run. After the run (the team's choice, 22
 * Nov) a soft stop clears itself once the car stands still and level (STOP
 * from the button or MQTT, a lift, a soft TILT or TILT_UNAVAILABLE): IDLE
 * again, waiting for START. A hard fault stays (FAULT) until RESET, from
 * MQTT or the button, which clears it too: no reboot.
 */
void robot_app_vehicle_step (uint32_t now_ms)
{
    robot_app_shared_t copy = {0};
    motion_output_t motion = {0};
    const mission_output_t * p_mission = mission_output();
    uint32_t saved = 0U;
    bool b_start = false;
    bool b_reset = false;
    bool b_hard_reset = false;
    bool b_level = false;

    robot_app_copy(&copy);
    robot_app_button(now_ms, copy.vehicle.state);
    saved = app_main_lock();
    copy = g_shared;
    b_start = g_shared.vehicle.b_start;
    b_reset = g_shared.vehicle.b_reset;
    g_shared.vehicle.b_start = false;
    g_shared.vehicle.b_reset = false;
    g_shared.faults.b_stop_since_snapshot = false;
    app_main_unlock(saved);
    if (b_start && (!p_mission->b_running))
    {
        gb_start_pending = true;
        g_start_since_ms = now_ms;
    }
    if (gb_start_pending &&
        ((now_ms - g_start_since_ms) > ROBOT_APP_START_WAIT_MS))
    {
        gb_start_pending = false;
    }
    /* Level: a fresh valid tilt inside the limit, so a soft TILT's cause has
     * gone and Safety will not latch it again at once.
     */
    b_level = copy.terrain.b_tilt_valid &&
              (!robot_app_tilt_exceeded(&copy.terrain)) &&
              (robot_app_age(now_ms, copy.tilt_ms) <= 100U);
    b_hard_reset = b_reset && copy.faults.b_hard;
    if (copy.faults.b_stop_latched && (!p_mission->b_running) &&
        (!copy.vehicle.b_reset_motion) &&
        (b_hard_reset ||
         ((!copy.faults.b_hard) && (b_reset || gb_start_pending || b_level))))
    {
        /* Clean only if no stop was raised since this step's snapshot; a
         * refused reset (the car still moving) is asked again next step,
         * except a hard one, which needs RESET again.
         */
        saved = app_main_lock();
        g_shared.vehicle.b_reset_motion = true;
        g_shared.vehicle.b_reset_hard = b_hard_reset;
        g_shared.faults.b_reset_clean = !g_shared.faults.b_stop_since_snapshot;
        app_main_unlock(saved);
    }
    robot_app_mission_input(&copy);
    motion_snapshot_unpack(&copy.motion, &motion);
    g_mission_in.command = MISSION_CMD_NONE;
    if (p_mission->b_running && copy.faults.b_inhibit)
    {
        g_mission_in.command = MISSION_CMD_STOP;
    }
    else if (gb_start_pending && (!p_mission->b_running) &&
             (!copy.faults.b_inhibit) && (!copy.faults.b_hard) &&
             (MOTION_INHIBITED != motion.state) && (!board_stop_pressed()))
    {
        g_mission_in.command = MISSION_CMD_START;
        gb_start_pending = false;
    }
    else
    {
        /* No command this step. */
    }
    mission_step(&g_mission_in);
    robot_app_mission_publish(now_ms);
}

/** @brief The other owners' data the mission reads this step. */
void robot_app_mission_input (const robot_app_shared_t * p_copy)
{
    g_mission_in.motion = p_copy->motion;
    g_mission_in.left_duty_pm = bitpack_extend(p_copy->left_duty_pm,
                                               ROBOT_APP_DUTY_BITS);
    g_mission_in.right_duty_pm = bitpack_extend(p_copy->right_duty_pm,
                                                ROBOT_APP_DUTY_BITS);
    g_mission_in.sonar_seq = p_copy->sonar.done_seq;
    g_mission_in.sonar_pulse_us = p_copy->sonar.pulse_us;
    g_mission_in.b_sonar_valid = p_copy->sonar.b_valid;
}

/** @brief Publish the mission's requests: a renewed motion request (a new
 * version every call, so Motion's lease never lapses), the servo and sonar
 * requests, the state (FAULT while a stop is latched and no run is under
 * way) and the Vehicle task's liveness.
 */
void robot_app_mission_publish (uint32_t now_ms)
{
    const mission_output_t * p_mission = mission_output();
    vehicle_state_t state = (vehicle_state_t)p_mission->state;
    uint32_t saved = app_main_lock();

    if ((!p_mission->b_running) && g_shared.faults.b_stop_latched)
    {
        state = VEHICLE_FAULT;
    }
    g_shared.vehicle.state = state;
    g_shared.vehicle.request = p_mission->request;
    g_shared.vehicle.request_version =
        (g_shared.vehicle.request_version + 1U) & ROBOT_APP_VERSION_MASK;
    g_shared.sonar.servo_us = p_mission->servo_us;
    g_shared.sonar.servo_seq = p_mission->servo_seq;
    g_shared.sonar.trigger_seq = p_mission->trigger_seq;
    g_shared.health[ROBOT_APP_HEALTH_VEHICLE] = now_ms;
    app_main_unlock(saved);
}

/** @brief The mission in a wait or a long decode (mission.h): publish now,
 * then sleep a period (b_wait) or give the lower tasks 1 ms every 10 ms, so
 * IMU, Scan, Comm and the radio stay fresh and the watchdog is fed.
 */
void mission_hook_alive (bool b_wait)
{
    uint32_t now_ms = robot_app_now();

    robot_app_mission_publish(now_ms);
    if (b_wait)
    {
        app_main_delay(5U);
        g_yield_ms = robot_app_now();
    }
    else if ((now_ms - g_yield_ms) >= 10U)
    {
        app_main_delay(1U);
        g_yield_ms = robot_app_now();
    }
    else
    {
        /* Ran less than 10 ms since the last pause. */
    }
}
#endif
#if !(ROBOT_APP_MISSION)

/** @brief Only local button edges start/reset; barcodes provide navigation. */
void robot_app_vehicle_step (uint32_t now_ms)
{
    robot_app_shared_t copy = {0};
    vehicle_input_t input = {0};
    vehicle_output_t output = {0};
    motion_request_packed_t request = {0};
    uint32_t saved = 0U;

    robot_app_copy(&copy);
    robot_app_button(now_ms, copy.vehicle.state);
    saved = app_main_lock();
    copy = g_shared;
    g_shared.vehicle.b_start = false;
    g_shared.vehicle.b_reset = false;
    g_shared.line.navigation = VEHICLE_NAV_NONE;
    g_shared.line.b_junction_edge = false;
    g_shared.faults.b_stop_since_snapshot = false;
    app_main_unlock(saved);
    robot_app_vehicle_input(&copy, now_ms, &input);
    /* Completion events are consumed by remembering the counters acted on. */
    g_vehicle_flags.coarse_seen = copy.front.coarse_done & ROBOT_APP_DONE_MASK;
    g_vehicle_flags.fine_seen = copy.front.fine_done & ROBOT_APP_DONE_MASK;
    if (!vehicle_step(&g_vehicle, now_ms, &input, &output))
    {
        robot_app_fault_set(true, ROBOT_APP_BIT_SUPERVISOR);
    }
    else if ((VEHICLE_FAULT == output.state) &&
             (VEHICLE_FAULT != copy.vehicle.state))
    {
        saved = app_main_lock();
        if (input.b_stop && (!copy.faults.b_stop_latched) &&
            (!copy.faults.b_hard))
        {
            /* Only GP20 explains the stop: Vehicle read it before Safety's
             * next poll, so record the local STOP itself.
             */
            robot_app_latch_locked(false, ROBOT_APP_FAULT_LOCAL_STOP, 0U,
                                   ROBOT_APP_STOP_LOCAL);
        }
        else
        {
            /* FAULT entered because a stop is already latched keeps the old
             * first-cause rule but is not a supervisor fault: no mask bit.
             */
            robot_app_latch_locked(false, ROBOT_APP_FAULT_SUPERVISOR,
                input.b_stop ? 0U :
                (UINT32_C(1) << (uint32_t)ROBOT_APP_BIT_SUPERVISOR), 0U);
        }
        app_main_unlock(saved);
    }
    else
    {
        /* Stepped, and FAULT was not newly entered: nothing to latch. */
    }
    if (((VEHICLE_SCAN == output.state) &&
         (VEHICLE_SCAN != copy.vehicle.state)) ||
        ((VEHICLE_REACQUIRE == output.state) &&
         (VEHICLE_NAVIGATE == copy.vehicle.state)))
    {
        /* Scan entry, or a completed turn, defines the expected heading. */
        g_route_anchor = copy.motion.pose;
    }
    if (output.b_issue_motion)
    {
        motion_request_pack(&output.request, &request);
    }
    saved = app_main_lock();
    if ((VEHICLE_FOLLOW == output.state) &&
        (VEHICLE_IDLE == copy.vehicle.state))
    {
        g_shared.vehicle.run_id = (g_shared.vehicle.run_id + 1U) & UINT8_MAX;
        g_shared.line.barcode_action = BARCODE_ACTION_UNKNOWN;
        g_shared.line.barcode_symbol = 0U;
        g_shared.scan.b_profile_valid = false;
    }
    g_shared.vehicle.state = output.state;
    g_shared.vehicle.scan_request = output.scan;
    if (output.b_issue_motion)
    {
        g_shared.vehicle.request = request;
        g_shared.vehicle.request_version =
            (g_shared.vehicle.request_version + 1U) & ROBOT_APP_VERSION_MASK;
    }
    if (output.b_reset_motion)
    {
        /* Clean only if no stop was raised since this step's snapshot. */
        g_shared.vehicle.b_reset_motion = true;
        g_shared.faults.b_reset_clean =
            !g_shared.faults.b_stop_since_snapshot;
    }
    if (output.b_stop)
    {
        /* Supervisor stop: inhibit without recording a fault cause. */
        g_shared.faults.b_inhibit = true;
        board_motor_brake();
    }
    g_shared.health[ROBOT_APP_HEALTH_VEHICLE] = now_ms;
    app_main_unlock(saved);
}
#endif

/** @brief Noise control for one sample-driven stop (ROBOT_APP_NOISE_UP):
 * true only for a bad sample that brings the level to limit. A good sample
 * never latches.
 */
bool robot_app_noise (uint8_t * p_level, bool b_bad, uint32_t limit)
{
    if (b_bad)
    {
        *p_level = (*p_level >= (UINT8_MAX - ROBOT_APP_NOISE_UP)) ?
                   (uint8_t)UINT8_MAX :
                   (uint8_t)(*p_level + ROBOT_APP_NOISE_UP);
    }
    else if (0U != *p_level)
    {
        *p_level = (uint8_t)(*p_level - 1U);
    }
    else
    {
        /* Level already zero. */
    }
    return b_bad && ((uint32_t)*p_level >= limit);
}

/** @brief Safety: each owner's health age now, kept as the window's longest
 * (ROBOT_APP_OWNERS of them; the first robot_app_health_t entries). The
 * first step of a new robot_app_timing_restart() window starts afresh.
 */
void robot_app_owner_note (const uint32_t health[ROBOT_APP_HEALTH_COUNT],
                                  uint32_t now_ms)
{
    robot_app_owner_evidence_t evidence = {{0U}};
    uint32_t age = 0U;
    uint32_t index = 0U;
    uint32_t saved = app_main_lock();

    if (g_owner_evidence_window == g_timing_window)
    {
        evidence = g_owner_evidence;
    }
    for (index = 0U; index < ROBOT_APP_OWNERS; ++index)
    {
        age = robot_app_age(now_ms, health[index]);
        if (age > UINT16_MAX)
        {
            age = UINT16_MAX;
        }
        if (age > evidence.longest_ms[index])
        {
            evidence.longest_ms[index] = (uint16_t)age;
        }
    }
    g_owner_evidence = evidence;
    g_owner_evidence_window = g_timing_window;
    app_main_unlock(saved);
}

/** @brief The owner evidence of the current window (zero before Safety's
 * first step in it).
 */
void robot_app_owner_evidence (robot_app_owner_evidence_t * p_output)
{
    robot_app_owner_evidence_t evidence = {{0U}};
    uint32_t saved = 0U;

    if (NULL == p_output)
    {
        return;
    }
    saved = app_main_lock();
    if (g_owner_evidence_window == g_timing_window)
    {
        evidence = g_owner_evidence;
    }
    app_main_unlock(saved);
    *p_output = evidence;
}

/** @brief Safety owner with absolute-deadline periodic release. */
void robot_app_safety_task (void)
{
    robot_app_periodic(robot_app_safety_step, 2U, 0U);
}

/** @brief Motion owner with absolute-deadline periodic release. */
void robot_app_motion_task (void)
{
    robot_app_periodic(robot_app_motion_step, 5U, 1U);
}

/** @brief Line owner with absolute-deadline periodic release. */
void robot_app_line_task (void)
{
    robot_app_periodic(robot_app_line_step, 2U, 2U);
}

/** @brief Vehicle owner with absolute-deadline periodic release. The
 * mission steps every 5 ms, as its bench loop did (its steering, holds and
 * timings were tuned there); the original supervisor every 10 ms.
 */
void robot_app_vehicle_task (void)
{
#if ROBOT_APP_MISSION
    robot_app_periodic(robot_app_vehicle_step, 5U, 3U);
#else
    robot_app_periodic(robot_app_vehicle_step, 10U, 3U);
#endif
}

/** @brief Scan owner with absolute-deadline periodic release. */
void robot_app_scan_task (void)
{
    robot_app_periodic(robot_app_scan_step, 2U, 5U);
}

/** @brief Comm owner with absolute-deadline periodic release. */
void robot_app_comm_task (void)
{
    robot_app_periodic(robot_app_comm_step, 5U, 6U);
}

/** @brief Telemetry owner with absolute-deadline periodic release. */
void robot_app_telemetry_task (void)
{
    robot_app_periodic(robot_app_telemetry_step, 200U, 7U);
}

/** @brief Snapshot copy has no kernel wait or peripheral transaction. */
void robot_app_copy (robot_app_shared_t * p_copy)
{
    uint32_t saved = app_main_lock();

    *p_copy = g_shared;
    app_main_unlock(saved);
}

/** @brief Record one stop or stopping fault; the caller holds the lock.
 * First cause and time if none yet, mask and stop-source bits, the hard
 * latch, inhibition, the stop latch and the reset handshake flags, then an
 * immediate brake.
 */
void robot_app_latch_locked (bool b_hard, robot_app_fault_t cause,
                                    uint32_t mask, uint32_t source)
{
    if (ROBOT_APP_FAULT_NONE == g_shared.faults.first)
    {
        g_shared.faults.first = cause;
        g_shared.first_fault_ms = robot_app_now();
    }
    g_shared.faults.mask = (g_shared.faults.mask | mask) & UINT16_MAX;
    g_shared.faults.stop_source =
        (g_shared.faults.stop_source | source) & ROBOT_APP_SOURCE_MASK;
    g_shared.faults.b_hard = g_shared.faults.b_hard || b_hard;
    g_shared.faults.b_inhibit = true;
    g_shared.faults.b_stop_latched = true;
    g_shared.faults.b_stop_since_snapshot = true;
    g_shared.faults.b_reset_clean = false;
    board_motor_brake();
}

/** @brief Sticky software stop and immediate output commit share one lock;
 * the first cause comes from the bit's entry in g_robot_app_causes.
 */
void robot_app_fault_set (bool b_hard, robot_app_bit_t bit)
{
    uint32_t saved = app_main_lock();

    robot_app_latch_locked(b_hard, g_robot_app_causes[bit],
                           UINT32_C(1) << (uint32_t)bit, 0U);
    app_main_unlock(saved);
}

/** @brief LOCAL or REMOTE STOP is a soft stop recorded as its source. */
void robot_app_stop_set (uint32_t source)
{
    uint32_t saved = app_main_lock();

    robot_app_latch_locked(false, (ROBOT_APP_STOP_LOCAL == source) ?
                           ROBOT_APP_FAULT_LOCAL_STOP :
                           ROBOT_APP_FAULT_REMOTE_STOP, 0U, source);
    app_main_unlock(saved);
}

/** @brief Non-stopping status bit set or cleared by its single owner; never
 * touches the first cause, inhibition, the stop latch or the handshake.
 */
void robot_app_status_set (robot_app_bit_t bit, bool b_active)
{
    uint32_t bit_mask = UINT32_C(1) << (uint32_t)bit;
    uint32_t saved = app_main_lock();

    if (b_active)
    {
        g_shared.faults.mask = (g_shared.faults.mask | bit_mask) & UINT16_MAX;
    }
    else
    {
        g_shared.faults.mask = g_shared.faults.mask & (~bit_mask & UINT16_MAX);
    }
    app_main_unlock(saved);
}

/** @brief Active missions, including stopped retries, retain health gates. */
bool robot_app_active (vehicle_state_t state)
{
    return (VEHICLE_FOLLOW == state) || (VEHICLE_NAVIGATE == state) ||
           (VEHICLE_SCAN == state) || (VEHICLE_BYPASS == state) ||
           (VEHICLE_REACQUIRE == state) || (VEHICLE_RANGE_WAIT == state);
}

/** @brief Integer form of fabs(angle) > ROBOT_TILT_LIMIT_RAD. Angles are
 * packed with magnitudes rounded up, which keeps that decision exact.
 */
bool robot_app_tilt_exceeded (const terrain_shared_t * p_terrain)
{
    int32_t pitch = bitpack_extend(p_terrain->pitch_mrad,
                                   ROBOT_APP_ANGLE_BITS);
    int32_t roll = bitpack_extend(p_terrain->roll_mrad, ROBOT_APP_ANGLE_BITS);

    /* Fields are at least -2048, so negation cannot overflow. */
    return (((pitch < 0) ? -pitch : pitch) > ROBOT_TILT_LIMIT_MRAD) ||
           (((roll < 0) ? -roll : roll) > ROBOT_TILT_LIMIT_MRAD);
}
#if !ROBOT_APP_MISSION

/** @brief Every drive authorization requires a fresh usable orientation. */
bool robot_app_ready (const robot_app_shared_t * p_copy,
                             uint32_t now_ms)
{
    return (0 != ROBOT_CALIBRATION_CONFIRMED) &&
           (0 != ROBOT_IMU_MOUNT_CONFIRMED) &&
           (robot_app_age(now_ms, p_copy->line_ms) <= 20U) &&
           (robot_app_age(now_ms, p_copy->imu_ms) <= 100U) &&
           (robot_app_age(now_ms, p_copy->tilt_ms) <=
            ROBOT_APP_TILT_TIMEOUT_MS) &&
           p_copy->terrain.b_tilt_valid &&
           (!robot_app_tilt_exceeded(&p_copy->terrain)) &&
           (robot_app_age(now_ms,
                          p_copy->health[ROBOT_APP_HEALTH_MOTION]) <= 20U) &&
           (LINE_CTRL_FAULT != p_copy->line.status) &&
           (LINE_CTRL_INHIBITED != p_copy->line.status) &&
           (!p_copy->faults.b_hard);
}
#endif

/** @brief Load measured reference parameters; never auto-calibrate on a
 * hump. */
bool robot_app_sensors_init (void)
{
    terrain_config_t terrain_config = {0};
    scanner_config_t scan_config = {0};
    line_ctrl_config_t line_config = {0};
    bool b_ready = true;

    app_cfg_line(&line_config);
    line_ctrl_init(&g_line_ctrl);
    app_cfg_terrain(&terrain_config);
    app_cfg_scanner(&scan_config);
    b_ready = terrain_init(&g_terrain, &terrain_config);
    b_ready = scanner_init(&g_scanner, &scan_config) && b_ready;
    b_ready = line_ctrl_derive(&line_config, &g_line_ctrl_runtime) && b_ready;
    /* Needs the half-count quantum from robot_app_motion_init(). */
    b_ready = barcode_derive(course_cfg_codebook(),
        g_half_count_m * (double)ROBOT_APP_BARCODE_UNIT_HC,
        &line_config, &g_barcode_runtime) && b_ready;
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
void robot_app_health (robot_app_health_t index, uint32_t now_ms)
{
    uint32_t saved = app_main_lock();

    g_shared.health[index] = now_ms;
    app_main_unlock(saved);
}

/** @brief Absolute releases skip missed periods instead of a catch-up loop.
 * Each step's run time and each skipped period go to the task's own timing
 * record (index), cleared at the first step of a new window. So does its
 * own time: the run time less the own time of the steps that finished
 * inside it (they preempted it), which g_timing_nested_us collects; this
 * step then adds its own for any step it preempted.
 */
void robot_app_periodic (void (* p_step)(uint32_t), uint32_t period_ms,
                                uint32_t index)
{
    uint32_t next_ms = robot_app_now();
    uint32_t now_ms = next_ms;
    uint32_t remaining_ms = 0U;
    uint64_t start_us = 0U;
    uint32_t used_us = 0U;
    uint32_t nested_us = 0U;
    uint32_t own_us = 0U;
    uint32_t saved = 0U;
    robot_app_timing_t * p_timing = &g_timing[index % ROBOT_APP_TASKS];

    for (;;)
    {
        if (p_timing->window != g_timing_window)
        {
            p_timing->worst_us = 0U;
            p_timing->skipped = 0U;
            p_timing->own_us = 0U;
            p_timing->steps = 0U;
            p_timing->window = g_timing_window;
        }
        /* The start and the nested total read together: a step that
         * preempts between them is counted once.
         */
        saved = app_main_lock();
        start_us = board_time_us();
        nested_us = g_timing_nested_us;
        app_main_unlock(saved);
        p_step(now_ms);
        saved = app_main_lock();
        /* A step is far below 2^32 us, so the difference fits. */
        used_us = (uint32_t)(board_time_us() - start_us);
        nested_us = g_timing_nested_us - nested_us;   /* wraps: modulo */
        own_us = (nested_us < used_us) ? (used_us - nested_us) : 0U;
        g_timing_nested_us += own_us;
        app_main_unlock(saved);
        if (used_us > p_timing->worst_us)
        {
            p_timing->worst_us = used_us;
        }
        p_timing->own_us = (p_timing->own_us > (UINT32_MAX - own_us)) ?
                           UINT32_MAX : (p_timing->own_us + own_us);
        ++p_timing->steps;
        next_ms += period_ms;
        now_ms = robot_app_now();
        remaining_ms = next_ms - now_ms;
        if ((0U == remaining_ms) ||
            (remaining_ms >= ROBOT_APP_SERIAL_HALF))
        {
            /* A missed period is skipped, not caught up: the owner freshness
             * gates see the late step, which a tight catch-up dispatch would
             * hide. Counted for SYS-12 (robot_app_timing).
             */
            next_ms = now_ms + period_ms;
            remaining_ms = period_ms;
            ++p_timing->skipped;
        }
        app_main_delay(remaining_ms);
        now_ms = robot_app_now();
    }
}

/** @brief Each task's longest step and skipped periods this window. Each
 * 32-bit field is written by its own task and read whole.
 */
void robot_app_timing (robot_app_task_time_t p_output[ROBOT_APP_TASKS])
{
    uint32_t index = 0U;

    if (NULL == p_output)
    {
        return;
    }
    for (index = 0U; index < ROBOT_APP_TASKS; ++index)
    {
        p_output[index].worst_us = g_timing[index].worst_us;
        p_output[index].skipped = g_timing[index].skipped;
        p_output[index].own_us = g_timing[index].own_us;
        p_output[index].steps = g_timing[index].steps;
    }
}

/** @brief A new window; each task clears its own record at its next step. */
void robot_app_timing_restart (void)
{
    ++g_timing_window;
    g_timing_start_us = board_time_us();
}

/** @brief The window's length so far; the Vehicle task writes and reads
 * g_timing_start_us, so the 64-bit value is never torn.
 */
uint32_t robot_app_timing_window_ms (void)
{
    return (uint32_t)((board_time_us() - g_timing_start_us) / 1000U);
}

/** @brief Millisecond serial clock deliberately wraps every 2^32 ms. */
uint32_t robot_app_now (void)
{
    /* Unsigned narrowing intentionally implements modulo-2^32 milliseconds. */
    return (uint32_t)(board_time_us() / 1000U);
}

/** @brief One action per debounced press, with release required to rearm. */
void robot_app_button (uint32_t now_ms, vehicle_state_t state)
{
    bool b_down = board_start_pressed();
    uint32_t saved = 0U;

    if (b_down != g_vehicle_flags.b_button_down)
    {
        g_vehicle_flags.b_button_down = b_down;
        g_button_since_ms = now_ms;
    }
    if ((now_ms - g_button_since_ms) < 30U)
    {
        return;
    }
    if (!b_down)
    {
        g_vehicle_flags.b_button_latched = false;
    }
    else if (!g_vehicle_flags.b_button_latched)
    {
        g_vehicle_flags.b_button_latched = true;
        saved = app_main_lock();
        if (VEHICLE_FAULT == state)
        {
            g_shared.vehicle.b_reset = true;
        }
        else if (VEHICLE_IDLE == state)
        {
            g_shared.vehicle.b_start = true;
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
#if !ROBOT_APP_MISSION

/** @brief Construct evidence gates from the packed shared records. */
void robot_app_vehicle_input (const robot_app_shared_t * p_copy,
                                     uint32_t now_ms,
                                     vehicle_input_t * p_input)
{
    /* Q7 steering: -127..127 represents -1..1. */
    double line_steering = (double)bitpack_extend(p_copy->line.steering_q7,
                               ROBOT_APP_STEER_BITS) / ROBOT_APP_STEER_SCALE;
    double steering = line_steering * ROBOT_LINE_GAIN_RPS;

    if (steering > ROBOT_TURN_RATE_RPS)
    {
        steering = ROBOT_TURN_RATE_RPS;
    }
    if (steering < -ROBOT_TURN_RATE_RPS)
    {
        steering = -ROBOT_TURN_RATE_RPS;
    }
    motion_snapshot_unpack(&p_copy->motion, &p_input->motion);
    p_input->navigation = p_copy->line.navigation;
    p_input->b_junction_edge = p_copy->line.b_junction_edge;
    p_input->line_forward_mps = ROBOT_LINE_SPEED_MPS *
                               (1.0 - (0.5 * fabs(line_steering)));
    /* Optical positive-right becomes differential-drive positive-left yaw. */
    p_input->line_yaw_rps = -steering;
    p_input->b_start = p_copy->vehicle.b_start;
    p_input->b_reset = p_copy->vehicle.b_reset;
    p_input->b_stop = (p_copy->faults.b_hard) || (board_stop_pressed()) ||
        ((p_copy->faults.b_stop_latched) &&
         (VEHICLE_FAULT != p_copy->vehicle.state));
    p_input->b_fault_clear = (!p_copy->faults.b_hard) &&
                             (!board_stop_pressed());
    p_input->b_sensors_ready = robot_app_ready(p_copy, now_ms);
    p_input->b_line_valid = (LINE_CTRL_TRACKING == p_copy->line.status) ||
                            (LINE_CTRL_JUNCTION == p_copy->line.status);
    p_input->b_line_wait = LINE_CTRL_SEARCHING == p_copy->line.status;
    p_input->b_range_valid = robot_app_range_valid(p_copy);
    p_input->range_age_ms = robot_app_age(now_ms, p_copy->range_ms);
    /* Ceil-mm ranges keep distance_m <= 0.25 and > 0.25 decisions exact. */
    p_input->b_obstacle = p_input->b_range_valid &&
        (p_copy->front.range.distance_mm <= ROBOT_FRONT_STOP_DISTANCE_MM);
    p_input->b_front_clear = p_input->b_range_valid &&
        (p_copy->front.range.distance_mm > ROBOT_FRONT_STOP_DISTANCE_MM);
    p_input->b_coarse_ready =
        p_copy->front.coarse_done != g_vehicle_flags.coarse_seen;
    p_input->b_fine_ready =
        p_copy->front.fine_done != g_vehicle_flags.fine_seen;
    robot_app_bypass_input(p_copy, p_input);
    p_input->b_original_line_verified =
        robot_app_original_line(p_copy, now_ms);
}

/** @brief Bounded geometric bypass hypotheses from the packed profile. */
void robot_app_bypass_input (const robot_app_shared_t * p_copy,
                                    vehicle_input_t * p_input)
{
    scanner_result_t result;

    scanner_shared_unpack(&p_copy->scan, &result);
    p_input->bypass_offset_m = (result.apparent_width_m / 2.0) +
        (ROBOT_ROBOT_WIDTH_M / 2.0) + ROBOT_OBSTACLE_MARGIN_M;
    p_input->bypass_length_m = result.nearest_distance_m +
        ROBOT_MAX_OBSTACLE_DEPTH_M + ROBOT_ROBOT_LENGTH_M +
        ROBOT_OBSTACLE_MARGIN_M;
    p_input->bypass_side = (SCANNER_SIDE_LEFT == result.preferred_side) ?
                           1 : -1;
    p_input->b_bypass_clearance = (0 != ROBOT_CORRIDOR_CONFIRMED) &&
        (result.b_profile_valid) && (result.b_obstacle_seen) &&
        (SCANNER_SIDE_NONE != result.preferred_side) &&
        (0U == result.unknown_samples);
}
#endif

/** @brief Wrap-safe age of a shared stamp on the millisecond serial clock.
 * A step reads now_ms at release; a higher-priority owner may publish a
 * newer stamp before the step's snapshot (for example MotionTask or LineTask
 * during VehicleTask's button read). That stamp is fresh (age 0), not about
 * 2^32 ms stale. Genuine staleness below the serial half is unchanged. The
 * IMU step instead selects an encoder path sample (robot_app_imu_path).
 */
uint32_t robot_app_age (uint32_t now_ms, uint32_t stamp_ms)
{
    uint32_t age = now_ms - stamp_ms;

    return (age >= ROBOT_APP_SERIAL_HALF) ? 0U : age;
}


/* End of file. */

