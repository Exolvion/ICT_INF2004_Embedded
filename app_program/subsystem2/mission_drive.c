/** @file mission_drive.c
 * @brief The mission: subsystem 2: drive, pivot and wheel-controller
 * settings. Split out of the one file by tools/reorg/split_c.py (22 Nov
 * 2026); what the parts share is in mission_private.h.
 */
#include "mission_private.h"
#include "mission_drive.h"
#define BENCH_PIVOT_TURNS (2U)  /* full turns per START, by the wheels */
#define BENCH_PIVOT_MARKS (8U)  /* line marks kept per front sensor */

/* The pivot test on the car (3 Nov 19:15-19:18, track mat, axle beside a
 * straight line): one real full turn counted 314.0 and 314.8 deg left (4
 * spans each), 314.3 and 313.9 deg right (2 spans each) on the wheels, the
 * mean 314.3: the car turns 14.5 % more than its wheels say, the same both
 * ways (our videos: the go-around's quarter turns came out past 90
 * deg). The go-around and the pivot test steer by g_drive.heading, the
 * wheels' turn x BENCH_YAW_SCALE; line following, AUTO-CENTRE, the P lines
 * and the motion module's own pose keep the wheels' heading (g_drive.yaw).
 * A car constant, measured on this mat: run the pivot test again on another
 * surface.
 */
#define BENCH_PIVOT_WHEEL_DEG (314.3)
#define BENCH_YAW_SCALE (360.0 / BENCH_PIVOT_WHEEL_DEG)
#define board_motor_duty          mission_motor_duty

/* The motion controller's longest step (wheel_pid.max_dt_s). The robot's
 * 25 ms (app_cfg.c) is for its 5 ms Motion task; this loop runs every ~7 ms
 * and a barcode decode step can take longer: 22 Nov the dark hump at the
 * start read as a "card" under the side sensor, its decode made one step
 * 25-26 ms, and the controller stopped the car (SENSOR_FAULT) 0.6 s after
 * the hump, twice. 50 ms, as line_ctrl's own gap limit (maximum_gap_ms),
 * still stops a loop that hangs. The "ENCODER ERRORS" beside it were the
 * ISR's 6-bit counts per read saturating over the long step (31 counts,
 * ~22 ms at 115 mm/s); the bench's distance comes from the 16-bit totals,
 * so no count was lost.
 */
#define BENCH_MOTION_MAX_DT_S (0.050)
#define BENCH_SETTLE_US (500000U)   /* at each duty before counting */
#define BENCH_MEASURE_US (500000U)  /* then counted */
#define BENCH_REST_US (1000000U)    /* braked between directions */

/* counts/s (about 8 mm/s): the wheel turns. Twitches of a sticking wheel
 * stay below it; the slowest real turning on 27 Oct was about 160.
 */
#define BENCH_MOVING_RATE (100)
#define BENCH_RATE_MAX (32767)
#define BENCH_PID_STEP_US (4000000U)    /* each speed step */
#define BENCH_PID_STEADY_US (2000000U)  /* its last 2 s: mean and ripple */
#define BENCH_TRACE_US (100000U)
#define BENCH_RISE (0.90)               /* rise: 90% of the target */
#define BENCH_BAND (0.10)               /* settled: within 10% of it */
#define BENCH_DRIVE_M (1.0)             /* S2-U09 */
#define BENCH_AFTER_US (1000000U)       /* drive tests report again 1 s on */
#define BENCH_STATUS_NAMES (7U)
#define BENCH_POSE_US (500000U)

typedef struct
{
    int8_t  duty_pct;     /* signed: + forward */
    bool    b_down;       /* measured on the way down */
    bool    b_errors;     /* the encoder ISR rejected an edge or saturated */
    int16_t rate[BENCH_WHEELS];  /* counts/s, + forward */
} bench_step_t;

typedef struct
{
    double proportional;
    double integral;
} bench_gains_t;

/* Gain set 0 is the configured one; the others are candidates bracketing a
 * faster PI loop (about 8.2 mm/s per 1% duty, S2-U07). D stays configured.
 * The car always uses set 4 (g_gain), the one every 180 mm/s run passed on.
 */
static const bench_gains_t g_gains[] =
{
    {ROBOT_PID_PROPORTIONAL, ROBOT_PID_INTEGRAL},
    {1.0, 3.0}, {2.0, 5.0}, {2.0, 10.0}, {3.0, 15.0}, {4.0, 20.0}
};

static const char * const g_mode_names[BENCH_MODES] =
{
    "PID steps (stand)", "DIST 1 m (floor)", "LEFT 90 (floor)",
    "RIGHT 90 (floor)", "TURN 180 (floor)", "LINE follow 80 mm/s (floor)",
    "LINE follow 120 mm/s (floor)", "LINE follow 150 mm/s (floor)",
    "LINE + stop within 10 cm of an obstacle, 120 mm/s (floor)",
    "SCAN obstacle (floor, car still)",
    "LINE centre 120 mm/s (track, runs until STOP)",
    "LINE centre 150 mm/s (track, runs until STOP)",
    "LINE smooth A 180 mm/s, soft (track, runs until STOP)",
    "LINE smooth B 150 mm/s, soft (track, runs until STOP)",
    "LINE smooth C 130 mm/s, soft (track, runs until STOP)",
    "BARCODE read, smooth A steering, 80 mm/s (track, bars on the left)",
    "AUTO-CENTRE only (track, pivots on the spot, no driving)",
    "MISSION: smooth A, barcodes and ramp read while driving (track, runs"
    " until STOP)",
    "AVOID an obstacle and return to course, 80 mm/s (floor, no line,"
    " until STOP)"
};

/* Why a drive ended (bench_move_results: the reasons bench_line_end and
 * bench_drive_finish are given, then g_status_names), sent as the index.
 */
#define BENCH_END_NAMES (24U)
static const char * const g_end_names[BENCH_END_NAMES] =
{
    "aborted", "done", "centred", "stopped: IR sensor fault",
    "stopped: obstacle ahead",
    "stopped: all three sensors dark (lifted, or off the mat)",
    "stopped: both sensors black past a junction", "stopped: line lost",
    "stopped: line sensor fault", "stopped at the distance limit",
    "stopped at the time limit", "stopped: AUTO-CENTRE failed",
    "stopped: the way along was blocked",
    "stopped: the way back to the line was blocked",
    "stopped: the obstacle's edge was out of reach",
    "stopped: no clear way round the obstacle", "stopped: lifted (IMU)",
    "OK", "INVALID", "REJECTED", "EXPIRED", "TIMED_OUT", "SENSOR_FAULT",
    "STOPPED"
};

/* Indexed by motion_status_t. */
static const char * const g_status_names[BENCH_STATUS_NAMES] =
{
    "OK", "INVALID", "REJECTED", "EXPIRED", "TIMED_OUT", "SENSOR_FAULT",
    "STOPPED"
};

static bench_step_t  g_steps[BENCH_STEPS];

  /* BENCH_PIVOT_TEST: STARTs so far */
/** BENCH_PIVOT_TEST: one spin's line marks (bench_pivot_tick). */
typedef struct
{
    double   turned;      /* the wheels' turn this START, rad (+ left) */
    double   last_yaw;    /* pose heading last tick */
    bool     dark[2];     /* front sensors' class (go-around hysteresis) */
    bool     known[2];    /* and that dark stretch's start was seen */
    double   from[2];     /* turned where it went dark */
    uint16_t peak[2];     /* highest reading this spin */
    uint32_t marks[2];
    double   mark[2][BENCH_PIVOT_MARKS]; /* dark stretches' middles */
} bench_pivot_t;
static bench_pivot_t g_pivot;
static const uint32_t g_gain = 4U;
uint64_t            g_pose_due_us;

/** @brief Begin the sweep from rest; refuses while STOP is held. */
void bench_sweep_start (void)
{
    if (board_stop_pressed())
    {
        bench_text("SWEEP refused: STOP is pressed");
        bench_flush();
        return;
    }
    if ((!gb_watchdog) && board_watchdog_start(BENCH_WATCHDOG_MS))
    {
        gb_watchdog = true;
    }
    g_sweep.b_running = true;
    g_sweep.step = 0U;
    g_sweep.b_rest = false;
    g_sweep.b_counting = false;
    g_steps_done = 0U;
    bench_text("SWEEP start: about 2.5 min, wheels must be off the ground;"
               " STOP aborts");
    bench_flush();
    g_sweep.step_us = board_time_us();
}

/** @brief Hold one duty (re-committed each tick, so STOP brakes at once),
 * count the second half of the step over its measured time, then move on.
 */
void bench_sweep_tick (uint8_t command)
{
    int8_t   duty_pct = 0;
    bool     b_down = false;
    uint16_t total[BENCH_WHEELS] = {0U, 0U};
    uint64_t now_us = board_time_us();
    uint32_t wheel;
    double   duty;
    double   rate;

    if (BENCH_CMD_STOP == command)
    {
        bench_sweep_finish("aborted");
        return;
    }
    if (g_sweep.b_rest)
    {
        board_motor_brake();
        if ((now_us - g_sweep.step_us) >= BENCH_REST_US)
        {
            g_sweep.b_rest = false;
            g_sweep.step_us = now_us;
        }
        return;
    }
    bench_step_place(g_sweep.step, &duty_pct, &b_down);
    duty = (double)duty_pct / 100.0;
    board_motor_duty(duty, duty);
    if ((!g_sweep.b_counting) &&
        ((now_us - g_sweep.step_us) >= BENCH_SETTLE_US))
    {
        board_encoder_totals(&g_sweep.start[BENCH_LEFT],
                             &g_sweep.start[BENCH_RIGHT]);
        g_sweep.count_us = now_us;
        g_sweep.b_counting = true;
        g_sweep.b_errors = false;
        return;
    }
    if ((!g_sweep.b_counting) ||
        ((now_us - g_sweep.count_us) < BENCH_MEASURE_US))
    {
        return;
    }
    board_encoder_totals(&total[BENCH_LEFT], &total[BENCH_RIGHT]);
    g_steps[g_sweep.step].duty_pct = duty_pct;
    g_steps[g_sweep.step].b_down = b_down;
    g_steps[g_sweep.step].b_errors = g_sweep.b_errors;
    for (wheel = 0U; wheel < BENCH_WHEELS; ++wheel)
    {
        rate = (double)bench_signed_delta(total[wheel], g_sweep.start[wheel],
                                          wheel) * 1000000.0 /
               (double)(now_us - g_sweep.count_us);
        if (rate > (double)BENCH_RATE_MAX)
        {
            rate = (double)BENCH_RATE_MAX;
        }
        if (rate < -(double)BENCH_RATE_MAX)
        {
            rate = -(double)BENCH_RATE_MAX;
        }
        g_steps[g_sweep.step].rate[wheel] = (int16_t)rate;
    }
    ++g_sweep.step;
    g_steps_done = g_sweep.step;
    g_sweep.step_us = now_us;
    g_sweep.b_counting = false;
    if (g_sweep.step >= BENCH_STEPS)
    {
        bench_sweep_finish("done");
    }
    else if ((2U * BENCH_DUTIES) == g_sweep.step)
    {
        g_sweep.b_rest = true;
    }
    else
    {
        /* Next duty. */
    }
}

/** @brief Brake, then print every finished step and the summary. */
void bench_sweep_finish (const char * p_reason)
{
    uint32_t step;
    uint32_t wheel;
    int32_t  rate;

    board_motor_brake();
    board_encoders_discard();
    g_sweep.b_running = false;
    /* The motors are stopped: wait for the queue rather than lose rows. */
    g_send_tries = BENCH_TABLE_SEND_TRIES;
    bench_text("SWEEP ");
    bench_text(p_reason);
    bench_text(": duty%, counts/s and mm/s per wheel, E = encoder errors");
    bench_flush();
    for (step = 0U; step < g_steps_done; ++step)
    {
        bench_signed((int32_t)g_steps[step].duty_pct);
        bench_text(g_steps[step].b_down ? " dn" : " up");
        for (wheel = 0U; wheel < BENCH_WHEELS; ++wheel)
        {
            rate = (int32_t)g_steps[step].rate[wheel];
            bench_text((BENCH_LEFT == wheel) ? " L " : " R ");
            bench_signed(rate);
            bench_text(" ");
            bench_signed(bench_mm(rate));
        }
        if (g_steps[step].b_errors)
        {
            bench_text(" E");
        }
        bench_flush();
    }
    bench_summary();
    g_send_tries = BENCH_SEND_TRIES;
}

/** @brief Step order: forward up, forward down, backward up, backward down.
 */
void bench_step_place (uint32_t step, int8_t * p_duty_pct,
                              bool * p_down)
{
    uint32_t within = step % (2U * BENCH_DUTIES);
    uint32_t index = (within < BENCH_DUTIES) ? within :
                     ((2U * BENCH_DUTIES) - 1U - within);
    int32_t  pct = (int32_t)g_duties[index];

    *p_down = within >= BENCH_DUTIES;
    *p_duty_pct = (int8_t)((step < (2U * BENCH_DUTIES)) ? pct : -pct);
}

/** @brief First duty that moves the wheel at all, the duty from which every
 * higher step turns it (steady: the wheels stick at low duty and can twitch
 * below it), keep-turning duty (lowest step down that still turns), top
 * speed and the sign at the top duty. ROBOT_STALL_DUTY must not be below
 * the largest steady duty, or a slow start latches a stall fault. The sign
 * verdict trusts the encoder signs: check those first by pushing the car
 * forward on the floor (both ENC distances must rise).
 */
void bench_wheel_summary (int32_t direction, uint32_t wheel)
{
    uint32_t first = (direction > 0) ? 0U : (2U * BENCH_DUTIES);
    uint32_t top = first + BENCH_DUTIES - 1U;
    uint32_t step;
    int32_t  start_pct = -1;
    int32_t  steady_pct = -1;
    int32_t  keep_pct = -1;
    int32_t  forward;
    int32_t  top_rate = (int32_t)g_steps[top].rate[wheel] * direction;

    for (step = first; (step <= top) && (start_pct < 0); ++step)
    {
        forward = (int32_t)g_steps[step].rate[wheel] * direction;
        if (forward >= BENCH_MOVING_RATE)
        {
            start_pct = (int32_t)g_duties[step - first];
        }
    }
    /* Steady: walk down from the top duty while the wheel still turns. */
    for (step = top + 1U; step > first; --step)
    {
        forward = (int32_t)g_steps[step - 1U].rate[wheel] * direction;
        if (forward < BENCH_MOVING_RATE)
        {
            break;
        }
        steady_pct = (int32_t)g_duties[step - 1U - first];
    }
    if (steady_pct > g_stall_pct)
    {
        g_stall_pct = steady_pct;
    }
    /* The down pass runs from the top duty back to 0. */
    for (step = top + 1U; step < (first + (2U * BENCH_DUTIES)); ++step)
    {
        forward = (int32_t)g_steps[step].rate[wheel] * direction;
        if (forward < BENCH_MOVING_RATE)
        {
            break;
        }
        keep_pct = (int32_t)g_duties[(2U * BENCH_DUTIES) - 1U -
                                     (step - first)];
    }
    bench_text((direction > 0) ? "FWD " : "BWD ");
    bench_text((BENCH_LEFT == wheel) ? "L" : "R");
    if ((start_pct < 0) || (steady_pct < 0))
    {
        bench_text(" does not turn steadily up to 60%");
    }
    else
    {
        bench_text(" first moves ");
        bench_dec(start_pct);
        bench_text("%, steady from ");
        bench_dec(steady_pct);
        bench_text("%, keeps turning to ");
        if (keep_pct < 0)
        {
            bench_text("?");
        }
        else
        {
            bench_dec(keep_pct);
        }
        bench_text("%, top ");
        bench_dec(bench_mm(top_rate));
        bench_text(" mm/s");
    }
    bench_text("; sign ");
    if (top_rate >= BENCH_MOVING_RATE)
    {
        bench_text("ok");
    }
    else if (top_rate <= -BENCH_MOVING_RATE)
    {
        bench_text((BENCH_LEFT == wheel) ?
                   "WRONG: flip ROBOT_LEFT_MOTOR_SIGN" :
                   "WRONG: flip ROBOT_RIGHT_MOTOR_SIGN");
    }
    else
    {
        bench_text("no motion");
    }
    bench_flush();
}

/** @brief Begin the selected test from rest with a fresh controller and the
 * selected gains; refuses while STOP is held.
 */
/** @brief The wheel controller for the selected test: the robot's settings
 * (app_cfg_motion) with g_gains[g_gain], BENCH_MOTION_MAX_DT_S, the robot's
 * duty limit and, for the smooth steering, BENCH_SMOOTH_WHEEL_MPS.
 */
static void bench_motion_config (motion_config_t * p_config)
{
    app_cfg_motion(p_config);
    p_config->wheel_pid.max_dt_s = BENCH_MOTION_MAX_DT_S;
    p_config->wheel_pid.proportional_gain = g_gains[g_gain].proportional;
    p_config->wheel_pid.integral_gain = g_gains[g_gain].integral;
    /* The bench copy raises ROBOT_MAX_DUTY for the sweep: use the robot's. */
    p_config->wheel_pid.output_min = -BENCH_ROBOT_MAX_DUTY;
    p_config->wheel_pid.output_max = BENCH_ROBOT_MAX_DUTY;
    if (NULL != bench_smooth_set(g_mode))
    {
        p_config->max_wheel_mps = BENCH_SMOOTH_WHEEL_MPS;
    }
}

void bench_drive_start (void)
{
    motion_config_t config;
    motion_status_t status = MOTION_OK;
    uint32_t        wheel;

    if (board_stop_pressed())
    {
        bench_text("MOTION refused: STOP is pressed");
        bench_flush();
        return;
    }
    if (BENCH_MODE_SCAN == g_mode)
    {
        bench_scan_start();
        return;
    }
    if (bench_is_line(g_mode) && (!bench_line_start()))
    {
        return;
    }
    /* The Motion task's controller has these settings from boot
     * (mission_motion_config); the robot app starts a run only once a stop is
     * reset. Nothing to set up here.
     */
    (void)config;
    (void)status;
    if ((!gb_watchdog) && board_watchdog_start(BENCH_WATCHDOG_MS))
    {
        gb_watchdog = true;
    }
    (void)memset(&g_drive, 0, sizeof(g_drive));
    g_drive.mode = g_mode;
    g_drive.errors = g_encoder_errors;
    if ((0 != BENCH_PIVOT_TEST) && bench_pivot_mode(g_drive.mode))
    {
        (void)memset(&g_pivot, 0, sizeof(g_pivot));
        g_pivot.dark[0] = true; /* what is under a sensor now is no mark */
        g_pivot.dark[1] = true;
        bench_avoid_ir_setup();
        bench_text("PIVOT test: axle on a straight stretch of the line; ");
        bench_dec((int32_t)BENCH_PIVOT_TURNS);
        bench_text(" turns and a quarter ");
        bench_text((BENCH_MODE_RIGHT_90 == g_drive.mode) ? "right" : "left");
        bench_text(", dark from L ");
        bench_dec((int32_t)g_avoid.ir_hyst[0].on_adc);
        bench_text(" R ");
        bench_dec((int32_t)g_avoid.ir_hyst[1].on_adc);
        bench_text(", heading = wheels x ");
        bench_fixed(bench_round(BENCH_YAW_SCALE * 100.0), 2U);
        bench_flush();
    }
    for (wheel = 0U; wheel < BENCH_WHEELS; ++wheel)
    {
        g_drive.start[wheel] = g_distance[wheel];
    }
    if (BENCH_MODE_MISSION == g_mode)
    {
        bench_ramp_start();
    }
    bench_line(BK_DRIVE_START);
    bench_text(bench_flag(BENCH_MODE_PID == g_mode) ?
               "PID start: about 15 s, wheels must be off the ground;"
               " STOP aborts" :
               "DRIVE start: car on the floor, path clear; STOP aborts");
    if (BENCH_MODE_PID != g_mode)
    {
        /* Which steering this image has, so a run's log names it. */
        bench_text(bench_flag(0 != BENCH_EDGE_FOLLOW) ?
                   "; steering EDGE (left edge)" : "; steering SMOOTH");
    }
    bench_flush();
    if ((0 != BENCH_STRAIGHT_CODE) && (BENCH_MODE_MISSION == g_mode))
    {
        bench_line(BK_STRAIGHT);   /* speed 0-511 mm/s */
        bench_text("STRAIGHT barcode: no line following, straight on at ");
        bench_u((uint32_t)bench_round(BENCH_MISSION_MPS * 1000.0), 9U);
        bench_text(" mm/s until STOP");
        bench_flush();
    }
    if (bench_has_auto(g_mode))
    {
        bench_line(BK_AUTO_FIRST);
        bench_text("AUTO-CENTRE first: pivots on the spot, up to 70 deg"
                   " each way, until both front sensors find the line");
        bench_flush();
    }
    /* The text above can outlast line_ctrl's 50 ms sample gap. */
    line_ctrl_init(&g_line_state);
    bench_drive_request(0U);
    /* The Motion task's controller lives on between runs: the first move
     * (bench_line_moving) numbers its primitive after the last one used, so a
     * new run never reuses a finished ID (motion_submit would refuse it).
     */
    g_drive.request.command_id = g_motion_id;
    mission_origin();
    robot_app_timing_restart();     /* the run's task timing, end report */
    g_drive.b_running = true;
    g_drive.step_us = board_time_us();
    g_drive.last_us = g_drive.step_us;
}

/** @brief The request of one step, with a fresh command ID (a new ID resets
 * the controller, as for every new primitive on the robot).
 */
void bench_drive_request (uint32_t step)
{
    motion_request_t request;
    double           half_track = 0.5 * ROBOT_TRACK_WIDTH_M;
    uint32_t         wheel;

    (void)memset(&request, 0, sizeof(request));
    request.command_id = (uint8_t)(step + 1U);
    switch (g_drive.mode)
    {
        case BENCH_MODE_DISTANCE:
            request.kind = MOTION_DISTANCE;
            request.forward_mps = ROBOT_LINE_SPEED_MPS;
            request.target = BENCH_DRIVE_M;
        break;
        case BENCH_MODE_LEFT_90:
        case BENCH_MODE_RIGHT_90:
        case BENCH_MODE_TURN_180:
            request.kind = MOTION_TURN;
            request.yaw_rps = ROBOT_TURN_RATE_RPS;
            request.target = (BENCH_MODE_TURN_180 == g_drive.mode) ?
                             CORE_PI : (0.5 * CORE_PI);
            if (BENCH_MODE_RIGHT_90 == g_drive.mode)
            {
                request.target = -request.target;
            }
            if ((0 != BENCH_PIVOT_TEST) && bench_pivot_mode(g_drive.mode))
            {
                /* A steady spin; the line marks end it (bench_pivot_tick). */
                request.kind = MOTION_VELOCITY;
                request.yaw_rps = (BENCH_MODE_RIGHT_90 == g_drive.mode) ?
                                  -ROBOT_TURN_RATE_RPS : ROBOT_TURN_RATE_RPS;
                request.target = 0.0;
            }
        break;
        case BENCH_MODE_LINE:
        case BENCH_MODE_LINE_FAST:
        case BENCH_MODE_LINE_MAX:
        case BENCH_MODE_CENTRE:
        case BENCH_MODE_CENTRE_MAX:
        case BENCH_MODE_SMOOTH_A:
        case BENCH_MODE_SMOOTH_B:
        case BENCH_MODE_SMOOTH_C:
        case BENCH_MODE_BARCODE:
        case BENCH_MODE_AUTO:
        case BENCH_MODE_MISSION:
            /* bench_line_step() sets the speeds and a new ID per start. */
            request.kind = MOTION_VELOCITY;
        break;
        default:
            /* PID: line speed, top speed, then the pivot-turn speed. */
            request.kind = MOTION_VELOCITY;
            if (0U == step)
            {
                request.forward_mps = ROBOT_LINE_SPEED_MPS;
            }
            else if (1U == step)
            {
                request.forward_mps = ROBOT_MAX_SPEED_MPS;
            }
            else
            {
                request.yaw_rps = ROBOT_TURN_RATE_RPS;
            }
            g_drive.wheel[step][BENCH_LEFT].target =
                request.forward_mps - (half_track * request.yaw_rps);
            g_drive.wheel[step][BENCH_RIGHT].target =
                request.forward_mps + (half_track * request.yaw_rps);
            for (wheel = 0U; wheel < BENCH_WHEELS; ++wheel)
            {
                g_drive.wheel[step][wheel].rise_ms = -1;
                g_drive.wheel[step][wheel].out_ms = -1;
            }
        break;
    }
    g_drive.request = request;
}

/** @brief One controller period as the robot's motion task runs it: renew
 * the request, step the controller on this period's counts, apply its
 * duties. No text is sent until the motors stop.
 */
void bench_drive_tick (uint8_t command)
{
    motion_output_t  output;
    motion_request_t halt;
    motion_request_t line;
    motion_status_t  status = MOTION_OK;
    uint64_t         now_us = board_time_us();
    uint32_t         dt_us = (uint32_t)(now_us - g_drive.last_us);
    uint32_t         now_ms = (uint32_t)(now_us / 1000U);
    uint32_t         ramp;

    g_drive.last_us = now_us;
    if (dt_us > g_drive.dt_max_us)
    {
        g_drive.dt_max_us = dt_us;
        g_drive.dt_max_mm = bench_round(g_centre.path_m * 1000.0);
    }
    /* The Motion task runs the controller: publish this step's request and
     * read its latest snapshot. A STOP is already latched by the Safety or
     * Comm task (the fault mask); the run ends and reports.
     */
    (void)halt;
    (void)status;
    (void)now_ms;
    if (BENCH_CMD_STOP == command)
    {
        bench_drive_finish("aborted");
        return;
    }
    if (bench_is_line(g_drive.mode))
    {
        bench_line_step(now_us, &line);
        mission_publish(&line);
    }
    else
    {
        /* Only MISSION (a line mode) runs in the robot app. */
        mission_publish_halt();
    }
    mission_motion_output(&output);
    if (!g_drive.b_running)
    {
        /* The step ended the run (bench_line_end via a lift or the like). */
        return;
    }
    g_drive.yaw = output.pose.yaw_rad;
    g_drive.heading = bench_avoid_wrap(g_drive.heading +
                                       (BENCH_YAW_SCALE *
                                        bench_avoid_wrap(output.pose.yaw_rad -
                                                         g_drive.pose_yaw)));
    g_drive.pose_yaw = output.pose.yaw_rad;
    g_drive.x_m = output.pose.x_m;
    g_drive.y_m = output.pose.y_m;
    if (BENCH_MODE_MISSION == g_drive.mode)
    {
        /* Every sample queued since the last step (a long step brings
         * several), each at its own time; at most a ring's worth, and none
         * once the run has ended (a lift).
         */
        for (ramp = 0U; (ramp < MISSION_ACCEL_RING) && g_drive.b_running &&
                        (!g_drive.b_done) && mission_accel_waiting(); ++ramp)
        {
            bench_ramp_step(now_us, &output);
        }
    }
    if (bench_is_line(g_drive.mode))
    {
        /* For the next line sample's trace row. */
        g_line.wheel_mms[BENCH_LEFT] =
            (int16_t)bench_round(output.left_speed_mps * 1000.0);
        g_line.wheel_mms[BENCH_RIGHT] =
            (int16_t)bench_round(output.right_speed_mps * 1000.0);
        g_line.duty_pm[BENCH_LEFT] =
            (int16_t)bench_round(output.left_duty * 1000.0);
        g_line.duty_pm[BENCH_RIGHT] =
            (int16_t)bench_round(output.right_duty * 1000.0);
    }
    if (MOTION_INHIBITED == output.state)
    {
        bench_drive_finish(g_status_names[(uint32_t)output.reason %
                                          BENCH_STATUS_NAMES]);
        return;
    }
    if ((0 != BENCH_PIVOT_TEST) && bench_pivot_mode(g_drive.mode) &&
        bench_pivot_tick())
    {
        bench_drive_finish("done");
        return;
    }
    if (BENCH_MODE_PID == g_drive.mode)
    {
        bench_pid_tick(now_us, dt_us, &output);
    }
    else
    {
        bench_move_tick(now_us, &output);
    }
}

/** @brief PID steps: statistics, trace, then rest 1 s braked between steps.
 */
void bench_pid_tick (uint64_t now_us, uint32_t dt_us,
                            const motion_output_t * p_output)
{
    uint64_t t_us = now_us - g_drive.step_us;
    uint32_t row = g_drive.traced;
    bench_wheel_t * p_wheel = g_drive.wheel[g_drive.step % BENCH_PID_STEPS];

    if (g_drive.b_rest)
    {
        if (t_us >= BENCH_REST_US)
        {
            g_drive.b_rest = false;
            g_drive.traced = 0U;
            g_drive.step_us = now_us;
            bench_drive_request(g_drive.step);
        }
        return;
    }
    bench_pid_sample(BENCH_LEFT, p_output->left_speed_mps,
                     p_output->left_duty, t_us, dt_us);
    bench_pid_sample(BENCH_RIGHT, p_output->right_speed_mps,
                     p_output->right_duty, t_us, dt_us);
    if ((row < BENCH_TRACE_ROWS) &&
        (t_us >= ((uint64_t)(row + 1U) * BENCH_TRACE_US)))
    {
        p_wheel[BENCH_LEFT].trace_mm_s[row] =
            (int16_t)bench_round(p_output->left_speed_mps * 1000.0);
        p_wheel[BENCH_RIGHT].trace_mm_s[row] =
            (int16_t)bench_round(p_output->right_speed_mps * 1000.0);
        p_wheel[BENCH_LEFT].trace_permille[row] =
            (int16_t)bench_round(p_output->left_duty * 1000.0);
        p_wheel[BENCH_RIGHT].trace_permille[row] =
            (int16_t)bench_round(p_output->right_duty * 1000.0);
        g_drive.traced = row + 1U;
    }
    if (t_us >= BENCH_PID_STEP_US)
    {
        ++g_drive.step;
        if (g_drive.step >= BENCH_PID_STEPS)
        {
            bench_drive_finish("done");
        }
        else
        {
            g_drive.b_rest = true;
            g_drive.step_us = now_us;
        }
    }
}

/** @brief One wheel's tick of a PID step, in m/s toward its target. */
void bench_pid_sample (uint32_t wheel, double speed, double duty,
                              uint64_t t_us, uint32_t dt_us)
{
    bench_wheel_t * p_wheel = &g_drive.wheel[g_drive.step][wheel];
    bool            b_reverse = p_wheel->target < 0.0;
    double          size = fabs(p_wheel->target);
    double          toward = b_reverse ? -speed : speed;
    int32_t         t_ms = (int32_t)(t_us / 1000U);

    if ((p_wheel->rise_ms < 0) && (toward >= (BENCH_RISE * size)))
    {
        p_wheel->rise_ms = t_ms;
    }
    if (fabs(speed - p_wheel->target) > (BENCH_BAND * size))
    {
        p_wheel->out_ms = t_ms;
    }
    if (toward > p_wheel->peak)
    {
        p_wheel->peak = toward;
    }
    if (fabs(duty) >= (BENCH_ROBOT_MAX_DUTY - 1.0e-9))
    {
        ++p_wheel->saturated;
    }
    /* The robot latches a stall after ROBOT_STALL_MS of this. */
    if ((fabs(duty) >= ROBOT_STALL_DUTY) && (0 == g_delta[wheel]))
    {
        p_wheel->stall_us += dt_us;
        if (p_wheel->stall_us > p_wheel->stall_max_us)
        {
            p_wheel->stall_max_us = p_wheel->stall_us;
        }
    }
    else
    {
        p_wheel->stall_us = 0U;
    }
    if (t_us < (BENCH_PID_STEP_US - BENCH_PID_STEADY_US))
    {
        return;
    }
    if ((0U == p_wheel->steady_ticks) || (toward < p_wheel->low))
    {
        p_wheel->low = toward;
    }
    if ((0U == p_wheel->steady_ticks) || (toward > p_wheel->high))
    {
        p_wheel->high = toward;
    }
    ++p_wheel->steady_ticks;
    p_wheel->counts += b_reverse ? -g_delta[wheel] : g_delta[wheel];
    p_wheel->duty_s += fabs(duty) * ((double)dt_us / 1000000.0);
    p_wheel->steady_s += (double)dt_us / 1000000.0;
}

/** @brief Drive tests: note the primitive's completion, report 1 s later. */
void bench_move_tick (uint64_t now_us,
                             const motion_output_t * p_output)
{
    uint32_t wheel;

    if (!g_drive.b_done)
    {
        if (MOTION_COMPLETE == p_output->state)
        {
            g_drive.b_done = true;
            g_drive.done_s = (double)(now_us - g_drive.step_us) / 1000000.0;
            g_drive.step_us = now_us;
            g_drive.stop_yaw = p_output->pose.yaw_rad;
            for (wheel = 0U; wheel < BENCH_WHEELS; ++wheel)
            {
                g_drive.stop[wheel] = g_distance[wheel] - g_drive.start[wheel];
            }
        }
        return;
    }
    /* MISSION reports at once: no second look 1 s on. */
    if ((BENCH_MODE_MISSION == g_drive.mode) ||
        ((now_us - g_drive.step_us) >= BENCH_AFTER_US))
    {
        bench_drive_finish((bench_is_line(g_drive.mode)) ?
                           g_line.p_reason : "done");
    }
}

/** @brief Brake, then print the results; motors stay stopped while sending.
 */
void bench_drive_finish (const char * p_reason)
{
    board_motor_brake();
    board_encoders_discard();
    g_drive.b_running = false;
    /* The robot's tasks see the run over at once: the report below (with
     * the REC rows) takes many seconds of mission_hook_alive calls, which
     * publish g_out. Left as it was, Safety kept the run's owner gates and
     * IMU rule on a stopped car (22 Nov 22:33: 24 s of FOLLOW).
     */
    g_out.state = (uint32_t)VEHICLE_IDLE;
    g_out.b_running = false;
    g_send_tries = BENCH_TABLE_SEND_TRIES;
    if (BENCH_MODE_PID == g_drive.mode)
    {
        bench_pid_results(p_reason);
    }
    else
    {
        bench_move_results(p_reason);
    }
    g_send_tries = BENCH_SEND_TRIES;
}

/** @brief Every step reached: targets, per-wheel summary, then the trace. */
void bench_pid_results (const char * p_reason)
{
    uint32_t steps = g_drive.step;
    uint32_t step;
    uint32_t row;
    uint32_t rows;
    const bench_wheel_t * p_wheel;

    /* An abort inside a step reports that step too, as far as it got. */
    if ((!g_drive.b_rest) && (steps < BENCH_PID_STEPS))
    {
        ++steps;
    }
    bench_text("PID ");
    bench_text(p_reason);
    bench_text(": ");
    bench_gains_text();
    bench_text(", duty limit ");
    bench_dec(bench_round(BENCH_ROBOT_MAX_DUTY * 100.0));
    bench_text("%, dt max ");
    bench_fixed(bench_round((double)g_drive.dt_max_us / 100.0), 1U);
    bench_text(" ms");
    if (g_encoder_errors != g_drive.errors)
    {
        bench_text(", ENCODER ERRORS");
    }
    bench_flush();
    for (step = 0U; step < steps; ++step)
    {
        p_wheel = g_drive.wheel[step];
        bench_text("STEP ");
        bench_dec((int32_t)step + 1);
        bench_text(" target L ");
        bench_signed(bench_round(p_wheel[BENCH_LEFT].target * 1000.0));
        bench_text(" R ");
        bench_signed(bench_round(p_wheel[BENCH_RIGHT].target * 1000.0));
        bench_text(" mm/s");
        bench_flush();
        bench_pid_wheel(&p_wheel[BENCH_LEFT], BENCH_LEFT);
        bench_pid_wheel(&p_wheel[BENCH_RIGHT], BENCH_RIGHT);
        bench_text(" trace t ms: L mm/s duty%, R mm/s duty%");
        bench_flush();
        rows = (step == g_drive.step) ? g_drive.traced : BENCH_TRACE_ROWS;
        for (row = 0U; row < rows; ++row)
        {
            bench_text(" ");
            bench_dec((int32_t)((row + 1U) * (BENCH_TRACE_US / 1000U)));
            bench_text(" L ");
            bench_signed((int32_t)p_wheel[BENCH_LEFT].trace_mm_s[row]);
            bench_text(" ");
            bench_fixed((int32_t)p_wheel[BENCH_LEFT].trace_permille[row],
                        1U);
            bench_text(" R ");
            bench_signed((int32_t)p_wheel[BENCH_RIGHT].trace_mm_s[row]);
            bench_text(" ");
            bench_fixed((int32_t)p_wheel[BENCH_RIGHT].trace_permille[row],
                        1U);
            bench_flush();
        }
    }
}

/** @brief One wheel: rise, settling, peak, steady mean and ripple toward
 * the target, mean |duty|, ticks at the duty limit, longest stall-like run.
 */
void bench_pid_wheel (const bench_wheel_t * p_wheel, uint32_t wheel)
{
    double metres_per_count = (2.0 * CORE_PI * ROBOT_WHEEL_RADIUS_M) /
                              (double)ROBOT_ENCODER_COUNTS_PER_REV;
    int32_t steady_ms =
        (int32_t)((BENCH_PID_STEP_US - BENCH_PID_STEADY_US) / 1000U);

    bench_text((BENCH_LEFT == wheel) ? " L rise " : " R rise ");
    if (p_wheel->rise_ms < 0)
    {
        bench_text("never");
    }
    else
    {
        bench_dec(p_wheel->rise_ms);
        bench_text(" ms");
    }
    bench_text(", settled ");
    if (p_wheel->out_ms >= steady_ms)
    {
        bench_text("no");
    }
    else
    {
        bench_dec(p_wheel->out_ms + 1);
        bench_text(" ms");
    }
    bench_text(", peak ");
    bench_signed(bench_round(p_wheel->peak * 1000.0));
    if (0U == p_wheel->steady_ticks)
    {
        bench_text(" mm/s, no steady window");
    }
    else
    {
        bench_text(", steady ");
        bench_signed(bench_round(((double)p_wheel->counts * metres_per_count *
                                  1000.0) / p_wheel->steady_s));
        bench_text(" (");
        bench_signed(bench_round(p_wheel->low * 1000.0));
        bench_text("..");
        bench_signed(bench_round(p_wheel->high * 1000.0));
        bench_text(") mm/s, duty ");
        bench_fixed(bench_round((p_wheel->duty_s / p_wheel->steady_s) *
                                1000.0), 1U);
        bench_text("%");
    }
    bench_text(", sat ");
    bench_dec((int32_t)p_wheel->saturated);
    bench_text(", stall ");
    bench_dec((int32_t)(p_wheel->stall_max_us / 1000U));
    bench_text(" ms");
    bench_flush();
}

/** @brief Drive tests: time to completion, then distance and heading at
 * completion and 1 s later (or where an abort or fault left them).
 */
void bench_move_results (const char * p_reason)
{
    int32_t  now[BENCH_WHEELS];
    uint32_t wheel;

    for (wheel = 0U; wheel < BENCH_WHEELS; ++wheel)
    {
        now[wheel] = g_distance[wheel] - g_drive.start[wheel];
    }
    /* Frame: the mode (5 bits unless MISSION), why (g_end_names, 5 bits),
     * the time 0-6553.5 s, dt max 0-6553.5 ms. The test modes' gains are
     * text only (those lines are not sent; START runs MISSION).
     */
    bench_line(BK_END);
    if (bench_flag(BENCH_MODE_MISSION == g_drive.mode))
    {
        bench_text("MISSION");
    }
    else
    {
        bench_name(g_mode_names, (uint32_t)g_drive.mode, 5U);
    }
    bench_text(" ");
    bench_name_of(g_end_names, BENCH_END_NAMES, p_reason, 5U);
    if (bench_flag(g_drive.b_done))
    {
        bench_text(bench_flag(bench_is_line(g_drive.mode)) ? " after " :
                                                       ", complete after ");
        bench_ux((uint32_t)bench_round(g_drive.done_s * 10.0), 1U, 16U);
        bench_text(" s");
    }
    bench_text(": ");
    if (BENCH_MODE_MISSION != g_drive.mode)
    {
        bench_gains_text();
        bench_text(", ");
    }
    bench_text("dt max ");
    bench_ux((uint32_t)bench_round((double)g_drive.dt_max_us / 100.0), 1U,
             16U);
    bench_text(" ms");
    if (bench_flag(g_encoder_errors != g_drive.errors))
    {
        bench_text(", ENCODER ERRORS");
    }
    bench_flush();
    if (BENCH_MODE_MISSION == g_drive.mode)
    {
        /* Where the slowest step was, and each part's slowest (0.1 ms,
         * 0-6553.5; the path as the live line).
         */
        bench_line(BK_SLOW);
        bench_text(" slowest step at ");
        bench_i(g_drive.dt_max_mm, 21U);
        bench_text(" mm; most in one step: Decoder 1 tries ");
        bench_ux(g_drive.slow_tries_us / 100U, 1U, 16U);
        bench_text(" ms (the Decoder 2 fit ");
        bench_ux(g_drive.slow_fit_us / 100U, 1U, 16U);
        bench_text(" ms of it), live lines ");
        bench_ux(g_drive.slow_live_us / 100U, 1U, 16U);
        bench_text(" ms");
        bench_flush();
        bench_tasks_text();
    }
    if (BENCH_MODE_MISSION == g_drive.mode)
    {
        /* The rest went out live: only totals, the link is slow. */
        if (g_auto.b_active || g_auto.b_failed)
        {
            bench_auto_results();
        }
        else
        {
            bench_centre_results();
            bench_code_mission_results();
            bench_ramp_results();
            bench_rec_text();
        }
        return;
    }
    if (g_drive.b_done)
    {
        bench_move_line(" at completion:", g_drive.stop, g_drive.stop_yaw);
        bench_move_line(" 1 s later:", now, g_drive.yaw);
        if ((!bench_is_centre(g_drive.mode)) &&
            (BENCH_MODE_AUTO != g_drive.mode))
        {
            bench_text(" Now measure the real ");
            if (bench_is_line(g_drive.mode))
            {
                bench_text("stop: sensors to the line and the T.");
            }
            else
            {
                bench_text((BENCH_MODE_DISTANCE == g_drive.mode) ?
                           "distance." : "angle.");
            }
            bench_flush();
        }
    }
    else
    {
        bench_move_line(" at the stop:", now, g_drive.yaw);
    }
    if ((0 != BENCH_PIVOT_TEST) && bench_pivot_mode(g_drive.mode))
    {
        bench_pivot_results();
    }
    if (BENCH_MODE_LINE_STOP == g_drive.mode)
    {
        bench_front_results();
    }
    if (bench_has_auto(g_drive.mode))
    {
        bench_auto_results();
    }
    if ((BENCH_MODE_AUTO == g_drive.mode) || g_auto.b_active ||
        g_auto.b_failed)
    {
        /* Centring alone, or it did not finish: no line test followed. */
        return;
    }
    if (bench_is_centre(g_drive.mode))
    {
        bench_centre_results();
    }
    if (BENCH_MODE_BARCODE == g_drive.mode)
    {
        bench_code_results();
    }
    if (bench_is_line(g_drive.mode))
    {
        bench_line_trace();
    }
}

/** @brief ENC distance per wheel and the odometry heading. */
void bench_move_line (const char * p_label, const int32_t * p_counts,
                             double yaw)
{
    bench_text(p_label);
    bench_text(" ENC mm L ");
    bench_signed(bench_mm(p_counts[BENCH_LEFT]));
    bench_text(" R ");
    bench_signed(bench_mm(p_counts[BENCH_RIGHT]));
    bench_text(", heading ");
    if (yaw >= 0.0)
    {
        bench_text("+");
    }
    bench_fixed(bench_round(yaw * 1800.0 / CORE_PI), 1U);
    bench_text(" deg");
    bench_flush();
}

/** @brief BENCH_PIVOT_TEST: the spins (LEFT 90 / RIGHT 90 there). */
bool bench_pivot_mode (bench_mode_t mode)
{
    return (BENCH_MODE_LEFT_90 == mode) || (BENCH_MODE_RIGHT_90 == mode);
}

/** @brief BENCH_PIVOT_TEST, each tick of a spin: the wheels' turn so far,
 * and each front sensor's dark stretches across the line (the go-around's
 * hysteresis, bench_avoid_ir_setup). A stretch whose start was seen and that
 * lasted half to twice BENCH_TURN_LINE_RAD of turn is a mark at its middle.
 * True once BENCH_PIVOT_TURNS turns and a quarter are done.
 */
bool bench_pivot_tick (void)
{
    uint16_t raw[LINE_CTRL_CHANNELS] = {0U, 0U, 0U};
    uint32_t channel;

    g_pivot.turned += bench_avoid_wrap(g_drive.heading - g_pivot.last_yaw);
    g_pivot.last_yaw = g_drive.heading;
    if (board_read_ir(raw))
    {
        for (channel = 0U; channel < 2U; ++channel)
        {
            line_ctrl_class_t cls = line_ctrl_classify(
                                        &g_avoid.ir_hyst[channel],
                                        raw[channel]);

            if (raw[channel] > g_pivot.peak[channel])
            {
                g_pivot.peak[channel] = raw[channel];
            }
            if ((LINE_CTRL_DARK == cls) && (!g_pivot.dark[channel]))
            {
                g_pivot.dark[channel] = true;
                g_pivot.known[channel] = true;
                g_pivot.from[channel] = g_pivot.turned;
            }
            else if ((LINE_CTRL_LIGHT == cls) && g_pivot.dark[channel])
            {
                double swept = fabs(g_pivot.turned - g_pivot.from[channel]);

                g_pivot.dark[channel] = false;
                if (g_pivot.known[channel] &&
                    (swept >= (0.5 * BENCH_TURN_LINE_RAD)) &&
                    (swept <= (2.0 * BENCH_TURN_LINE_RAD)) &&
                    (g_pivot.marks[channel] < BENCH_PIVOT_MARKS))
                {
                    g_pivot.mark[channel][g_pivot.marks[channel]] =
                        0.5 * (g_pivot.from[channel] + g_pivot.turned);
                    ++g_pivot.marks[channel];
                }
                g_pivot.known[channel] = false;
            }
            else
            {
                /* No change, in the band, or invalid: keep the class. */
            }
        }
    }
    return fabs(g_pivot.turned) >=
           (((double)BENCH_PIVOT_TURNS + 0.25) * 2.0 * CORE_PI);
}

/** @brief BENCH_PIVOT_TEST, after a spin: each sensor's marks (degrees of
 * g_drive.heading from the start), then the mean of every span two marks
 * apart (one real full turn) against 360 deg.
 */
void bench_pivot_results (void)
{
    static const char * const names[2] = {"L", "R"};
    double   sum = 0.0;
    double   turn_deg;
    double   pct;
    uint32_t spans = 0U;
    uint32_t channel;
    uint32_t i;

    for (channel = 0U; channel < 2U; ++channel)
    {
        bench_text("PIVOT ");
        bench_text(names[channel]);
        bench_text(": line at");
        for (i = 0U; i < g_pivot.marks[channel]; ++i)
        {
            bench_text(" ");
            bench_fixed(bench_round(fabs(g_pivot.mark[channel][i]) * 1800.0 /
                                    CORE_PI), 1U);
        }
        bench_text(" deg (peak ");
        bench_dec((int32_t)g_pivot.peak[channel]);
        bench_text(")");
        bench_flush();
        for (i = 0U; (i + 2U) < g_pivot.marks[channel]; ++i)
        {
            sum += fabs(g_pivot.mark[channel][i + 2U] -
                        g_pivot.mark[channel][i]);
            ++spans;
        }
    }
    if (0U == spans)
    {
        bench_text("PIVOT: no full turn between line marks; put the axle on"
                   " a straight stretch of the line");
        bench_flush();
        return;
    }
    turn_deg = (sum / (double)spans) * 180.0 / CORE_PI;
    pct = ((turn_deg / 360.0) - 1.0) * 100.0;
    bench_text((BENCH_MODE_RIGHT_90 == g_drive.mode) ? "PIVOT right" :
                                                       "PIVOT left");
    bench_text(": a real full turn is ");
    bench_fixed(bench_round(turn_deg * 10.0), 1U);
    bench_text(" deg by the heading (");
    bench_dec((int32_t)spans);
    bench_text(" spans), ");
    if (pct >= 0.0)
    {
        bench_text("+");
    }
    bench_fixed(bench_round(pct * 10.0), 1U);
    bench_text(" %: turns by the heading come out ");
    bench_text((pct >= 0.0) ? "short" : "long");
    bench_flush();
}

/** @brief MISSION: every BENCH_POSE_US, the car's odometry pose as one line
 * (not flushed) for the dashboard's map: P x y mm and heading deg, + left,
 * from the run's start, then C and the compass's raw X Y Z (trial, report
 * only) when it reads. True if it was due.
 */
bool bench_pose_text (uint64_t now_us)
{
    int16_t  xyz[3] = {0, 0, 0};
    uint32_t axis;

    if (now_us < g_pose_due_us)
    {
        return false;
    }
    g_pose_due_us = now_us + BENCH_POSE_US;
    /* x, y +-65535 mm from the start (17 bits: odometry drifts over a long
     * run; 15 saturated at 16 m in the sim); heading +-180 deg (9 bits,
     * the odometry wraps it); compass raw +-4095 (13 bits; LSM303DLHC
     * counts are 12-bit).
     */
    bench_line(BK_POSE);
    bench_text(" P ");
    bench_i(bench_round(g_drive.x_m * 1000.0), 17U);
    bench_text(" ");
    bench_i(bench_round(g_drive.y_m * 1000.0), 17U);
    bench_text(" ");
    bench_i(bench_round(g_drive.yaw * 180.0 / CORE_PI), 9U);
    if (bench_flag(bench_mag_read(xyz)))
    {
        bench_text(" C");
        for (axis = 0U; axis < 3U; ++axis)
        {
            bench_text(" ");
            bench_s((int32_t)xyz[axis], 13U);
        }
    }
    return true;
}

/** @brief The selected gain set, without a line end. */
void bench_gains_text (void)
{
    bench_text("gains ");
    bench_dec((int32_t)g_gain);
    bench_text((0U == g_gain) ? " (configured) P " : " P ");
    bench_fixed(bench_round(g_gains[g_gain].proportional * 100.0), 2U);
    bench_text(" I ");
    bench_fixed(bench_round(g_gains[g_gain].integral * 100.0), 2U);
    bench_text(" D ");
    bench_fixed(bench_round(ROBOT_PID_DERIVATIVE * 100.0), 2U);
}

/** @brief Text of a fixed length from a small alphabet (map rows): each
 * character's index in p_alphabet, bits each, no length (the template knows
 * it). A character not in it has no code: the line is not sent.
 */
void bench_marks (const char * p_text, const char * p_alphabet,
                         uint32_t bits)
{
    uint32_t index;
    uint32_t code;

    for (index = 0U; '\0' != p_text[index]; ++index)
    {
        code = 0U;
        while (('\0' != p_alphabet[code]) &&
               (p_alphabet[code] != p_text[index]))
        {
            ++code;
        }
        if ('\0' == p_alphabet[code])
        {
            bench_untyped();
        }
        bench_code(code, bits);
    }
    bench_text(p_text);
}

/** @brief The Motion task's controller as MISSION drives it. */
void mission_motion_config (motion_config_t * p_config)
{
    bench_mode_t mode = g_mode;

    g_mode = BENCH_MODE_MISSION;
    bench_motion_config(p_config);
    g_mode = mode;
}

/* End of file. */

