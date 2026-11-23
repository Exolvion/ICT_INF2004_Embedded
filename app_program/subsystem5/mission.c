/** @file mission.c
 * @brief The mission: the mission's core: commands, the run's start and end,
 * the idle report, the step. Split out of the one file by
 * tools/reorg/split_c.py (22 Nov 2026); what the parts share is in
 * mission_private.h.
 */
#include "mission.h"
#include "mission_private.h"
#define BENCH_MOTION (1)
#if !defined(MISSION_SIM)
#define MISSION_CONSOLE (0)
#endif
#if !(!defined(MISSION_SIM))
#define MISSION_CONSOLE (1)
#endif
#if !defined(BENCH_MOTION)
#define BENCH_MOTION (0)
#endif
uint16_t         g_ir_now_total[2];
mission_input_t  g_in;
               /* this step's input */
mission_output_t g_out;

/* A 5 ms delay plus the sensor reads takes about 6.75 ms, so every window
 * is timed with board_time_us(), not counted in ticks.
 */
#define BENCH_TICK_MS (5U)
#define BENCH_REPORT_US (1000000U)  /* sensor report period */
#define BENCH_BUTTON_TICKS (4U)     /* debounce on START */
#define BENCH_STALL_MARGIN_PCT (2)

/* Go-around checks added 3 Nov (every value comes from the car's
 * own runs):
 *   STEPIN line fix: driving square at the line, the front sensors cross it
 *     while the pivot is still BENCH_AVOID_FRONT_MM from it. Both front
 *     sensors dark together (this run's calibrated thresholds, with their
 *     hysteresis) over a line-sized stretch, BENCH_AVOID_LINE_MM / 2 to
 *     1.5 x BENCH_AVOID_LINE_MM of travel, put the line's centre at the
 *     stretch's middle: the sideways offset the wheels drifted over the
 *     go-around is reset there and the car stops ON the line. The two front
 *     sensors sit side by side, so square on they cross the line together.
 *     A card beside the line must not pass for it: its bars start 29 mm
 *     from the line's centre (half the line + the 20 mm gap, spec) and run
 *     away from it, so a crossing is believed only within
 *     BENCH_AVOID_LINE_WIN_MM (those 29 mm) of where odometry expects the
 *     line, a stretch already dark when STEPIN starts (its start unseen:
 *     the sensors may stop over a card) is not used, and a bar crossed
 *     along its length is dark longer than a line. A bigger drift than the
 *     window leaves the offset to odometry, as before.
 *   Path guard: on ALONG, BACKPIVOT and STEPIN the sonar looks straight
 *     ahead (servo centred). BENCH_AVOID_CONFIRM echoes in a row within
 *     BENCH_AVOID_GUARD_MM (the car's front within BENCH_AVOID_CLEAR_MM of
 *     something) mean the obstacle, or something else, is in the way.
 *     BACKPIVOT and STEPIN then turn back parallel and run on by half the
 *     car + BENCH_AVOID_CLEAR_MM (the margin ALONG keeps past the back)
 *     before trying again, the object being deeper than the given depth, at
 *     most BENCH_AVOID_EXTENDS times; ALONG, or a spent retry, stops the car
 *     instead of driving into it. No reversing. BACKPIVOT turns the sonar
 *     from ahead toward the line, never back at the obstacle's back face,
 *     so an echo then is something beside the car's way back. The sonar's
 *     cone is narrower than the car, so a clear look after a blocked one
 *     does not show the flank is clear: the object's end is somewhere
 *     between the two looks. The first clear BACKPIVOT after a block turns
 *     back parallel once more and runs on half the car +
 *     BENCH_AVOID_CLEAR_MM past it before stepping in.
 */
#define BENCH_AVOID_LINE_WIN_MM (29.0)  /* bars from the line centre (spec) */
#define BENCH_CODE_POLL_TMO_MS (1)
#define BENCH_CODE_TRY_MAX (12U)

/* Mission command bytes (mqttcmd.h), given bench meanings. */
#define BENCH_CMD_NONE (0x00U)
#define BENCH_CMD_START (0xAAU)
#define BENCH_CMD_RESET (0xABU)
#define BENCH_CMD_RIGHT (0xADU)

/* Duty in percent: 1% steps where a wheel starts, coarse above. */
const uint8_t g_duties[] =
{
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U, 13U, 14U, 15U,
    16U, 17U, 18U, 19U, 20U, 21U, 22U, 23U, 24U, 25U, 26U, 27U, 28U, 29U,
    30U, 35U, 40U, 50U, 60U
};

typedef struct
{
    uint32_t count;
    uint32_t sum[3];
    uint16_t minimum[3];
    uint16_t maximum[3];
} bench_ir_t;
uint32_t      g_steps_done;
bench_sweep_t g_sweep;
static bench_ir_t    g_ir;
static bench_accel_t g_accel;
bench_ramp_t  g_ramp;
uint32_t         g_frame_kind;
    /* BK_NONE: no frame for the line */
bool             gb_frame_text;
           /* MISSION: the barcode D0 */
bool          gb_mqtt;
bool          gb_accel;
int32_t       g_distance[BENCH_WHEELS];
uint16_t      g_last_total[BENCH_WHEELS];
uint32_t      g_encoder_errors;
static uint32_t      g_start_ticks;
static uint64_t      g_report_us;
int32_t       g_stall_pct;
   /* largest steady duty of the sweep */
int32_t       g_delta[BENCH_WHEELS];
  /* this tick, + forward */
bench_drive_t g_drive;
bench_mode_t  g_mode = BENCH_MODE_MISSION;
  /* what START runs */
static uint32_t      g_pivot_starts;
bench_scan_t        g_scan;
bench_avoid_t       g_avoid;
bench_nav_t         g_nav;

   /* bench_smooth_track's keep-turning rule acted */

/* ==== Kernel tasks (14 Nov 2026): a safety task preempts control ========
 * Rebuilt on the proven single-loop. The CONTROL task runs the old loop
 * body unchanged, so its inline barcode reads and telemetry keep the exact
 * pacing that makes the bars read and the log stream live (the control task
 * yields to the Wi-Fi task in bench_tick / bench_code_wait, as the single
 * loop did). A separate SAFETY task, above control, brakes the moment STOP
 * is pressed or sent, and relays every other command to control through a
 * message buffer; it is the sole reader of the buttons and MQTT, so no
 * command is read twice.
 *   1 init     bench_loop: create objects, start tasks, then sleep forever
 *   2 safety   5 ms cyclic handler: instant STOP brake + event flag; other
 *              commands to control by message buffer; START debounce
 *   5 control  the old loop body (bench_tick / track / command / dispatch)
 *   9 Wi-Fi    cyw43_utk_task (unchanged)
 * Nothing timing-sensitive to the drive loop is a separate task, so live
 * telemetry and the barcode decode cannot regress. In the host sim no task
 * is started (gb_rtos stays false): bench_command then polls the button and
 * MQTT exactly as the single loop did, so sim output is unchanged.
 */
#define BENCH_PRI_SAFETY   (2)
#define BENCH_PRI_CONTROL  (5)
#define BENCH_SAFETY_MS    (5U)
#define BENCH_FLG_STOP     (1U << 0U)
#define BENCH_CTRL_STACK   (8192U)
#define BENCH_SAFETY_STACK (1024U)
#define BENCH_CMD_DEPTH    (8U)
#define BENCH_CMD_WORDS(m) ((((m) + 3U) / 4U) + 1U)

static bool     gb_rtos;
         /* the tasks were started */
static ID       g_ctrl_tid;
static ID       g_safety_tid;
static ID       g_stop_flg;
      /* safety -> control: STOP */
static ID       g_cmd_mbf;
       /* safety -> control: other commands */
static ID       g_safety_cyc;
static uint32_t g_ctrl_stack[BENCH_CTRL_STACK / 4U] __attribute__((aligned(8)));
static uint32_t g_safety_stack[BENCH_SAFETY_STACK / 4U]
                __attribute__((aligned(8)));
static uint32_t g_cmd_store[BENCH_CMD_DEPTH * BENCH_CMD_WORDS(1U)];

void bench_loop (void)
{
    uint16_t     samples[3] = {0U, 0U, 0U};
    bool         b_board = false;
    const char * p_what = "";
    int32_t      error = 0;

    (void)MISSION_PRINTF((const UB *)"\nBENCH IMAGE: not the robot firmware;"
                    " wheels must be off the ground.\n");
    b_board = board_init();
    /* An accelerometer failure still leaves the other measurements. */
    if ((!b_board) && (!mission_read_ir(samples)))
    {
        (void)MISSION_PRINTF((const UB *)"board_init refused: %s\n",
                        board_init_diagnostic());
        for (;;)
        {
            (void)tk_dly_tsk(1000);
        }
    }
    gb_accel = b_board;
#if ROBOT_MQTT
    gb_mqtt = net_mqtt_ipc_start();
#else
    /* GP0/GP1 go back to UART0: the sonar is not used here. */
    board_console_restore();
#endif
    board_encoder_totals(&g_last_total[BENCH_LEFT],
                         &g_last_total[BENCH_RIGHT]);
    mission_encoders_discard();
    bench_line(BK_BOOT);
    if (bench_flag(0 != BENCH_MOTION))
    {
        bench_text("BENCH MOTION ready: START runs MISSION (line-follow +"
                   " barcode + obstacle go-around), STOP stops. Accel ");
    }
    else
    {
        bench_text("BENCH ready: START = duty sweep, STOP = abort. Accel ");
    }
    bench_text(bench_flag(gb_accel) ? "ok" : "FAILED");
    bench_flush();
    if (0 != BENCH_MOTION)
    {
        /* The sonar faces ahead from boot, for the obstacle tests. */
        (void)bench_scan_servo(0);
    }
    g_report_us = board_time_us();
    /* Create the kernel objects and start the safety and control tasks;
     * any failure brakes and refuses to run (nothing may drive).
     */
    if (!bench_rtos_start(&p_what, &error))
    {
        mission_motor_brake();
        (void)MISSION_PRINTF((const UB *)"BENCH refused: kernel %s failed,"
                        " error %d\n", p_what, (int)error);
        for (;;)
        {
            mission_motor_brake();
            (void)tk_dly_tsk(1000);
        }
    }
    /* The tasks run from here; this initial task (priority 1) only sleeps. */
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

/** @brief The old single-loop body, now run by the control task (and by the
 * host sim through bench_loop_sim). Unchanged except that bench_command
 * takes STOP/commands from the safety task when the tasks are running.
 */
void bench_run_loop (void)
{
    uint8_t command = BENCH_CMD_NONE;

    for (;;)
    {
        bench_tick();
        bench_track();
        bench_nav_poll_map();
        command = bench_command();
        if (g_sweep.b_running)
        {
            bench_sweep_tick(command);
        }
        else if ((0 != BENCH_MOTION) && g_drive.b_running)
        {
            bench_drive_tick(command);
        }
        else if ((0 != BENCH_MOTION) && g_scan.b_running)
        {
            bench_scan_tick(command);
        }
        else
        {
            bench_idle(command);
        }
    }
}

/** @brief Create the event flag, command buffer and cyclic handler, then
 * create and start the control and safety tasks. The tasks are lower
 * priority than this initial task, so none runs until bench_loop sleeps.
 * False with *pp_what / *p_err on the first failure.
 */
bool bench_rtos_start (const char ** pp_what, int32_t * p_err)
{
    T_CFLG flag;
    T_CMBF mbf;
    T_CCYC cyc;
    T_CTSK ctsk;
    ER     result;

    *p_err = 0;
    (void)memset(&flag, 0, sizeof(flag));
    flag.flgatr = TA_TFIFO | TA_WSGL;
    g_stop_flg = tk_cre_flg(&flag);
    *pp_what = "event flag";
    if (g_stop_flg <= 0)
    {
        *p_err = (int32_t)g_stop_flg;
        return false;
    }

    (void)memset(&mbf, 0, sizeof(mbf));
    mbf.mbfatr = TA_TFIFO | TA_USERBUF;
    mbf.bufsz  = (SZ)sizeof(g_cmd_store);
    mbf.maxmsz = 1;
    mbf.bufptr = g_cmd_store;
    g_cmd_mbf = tk_cre_mbf(&mbf);
    *pp_what = "command buffer";
    if (g_cmd_mbf <= 0)
    {
        *p_err = (int32_t)g_cmd_mbf;
        return false;
    }

    (void)memset(&cyc, 0, sizeof(cyc));
    cyc.cycatr = TA_HLNG;
    cyc.cychdr = (FP)bench_safety_cyc;
    cyc.cyctim = (RELTIM)BENCH_SAFETY_MS;
    g_safety_cyc = tk_cre_cyc(&cyc);
    *pp_what = "cyclic handler";
    if (g_safety_cyc <= 0)
    {
        *p_err = (int32_t)g_safety_cyc;
        return false;
    }

    (void)memset(&ctsk, 0, sizeof(ctsk));
    ctsk.tskatr = TA_HLNG | TA_RNG0 | TA_USERBUF;
    ctsk.task   = (FP)bench_control_task;
    ctsk.itskpri = BENCH_PRI_CONTROL;
    ctsk.stksz  = (SZ)sizeof(g_ctrl_stack);
    ctsk.bufptr = g_ctrl_stack;
    g_ctrl_tid = tk_cre_tsk(&ctsk);
    *pp_what = "control task";
    if (g_ctrl_tid <= 0)
    {
        *p_err = (int32_t)g_ctrl_tid;
        return false;
    }

    (void)memset(&ctsk, 0, sizeof(ctsk));
    ctsk.tskatr = TA_HLNG | TA_RNG0 | TA_USERBUF;
    ctsk.task   = (FP)bench_safety_task;
    ctsk.itskpri = BENCH_PRI_SAFETY;
    ctsk.stksz  = (SZ)sizeof(g_safety_stack);
    ctsk.bufptr = g_safety_stack;
    g_safety_tid = tk_cre_tsk(&ctsk);
    *pp_what = "safety task";
    if (g_safety_tid <= 0)
    {
        *p_err = (int32_t)g_safety_tid;
        return false;
    }

    gb_rtos = true;   /* bench_command now takes commands from safety */
    result = tk_sta_tsk(g_ctrl_tid, 0);
    if (E_OK == result)
    {
        result = tk_sta_tsk(g_safety_tid, 0);
    }
    if (E_OK == result)
    {
        result = tk_sta_cyc(g_safety_cyc);
    }
    *pp_what = "task start";
    if (E_OK != result)
    {
        gb_rtos = false;
        *p_err = (int32_t)result;
        return false;
    }
    return true;
}

/** @brief Control task: the proven loop body. */
void bench_control_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;
    bench_run_loop();
}

/** @brief Safety task, woken every BENCH_SAFETY_MS: read the buttons and
 * MQTT; STOP brakes at once and raises the flag; any other command goes to
 * control. It is the only reader of the command sources.
 */
void bench_safety_task (INT stacd, void * exinf)
{
    uint8_t command;

    (void)stacd;
    (void)exinf;
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
        command = bench_poll_command();
        if (BENCH_CMD_STOP == command)
        {
            mission_motor_brake();
            (void)tk_set_flg(g_stop_flg, BENCH_FLG_STOP);
        }
        else if (BENCH_CMD_NONE != command)
        {
            (void)tk_snd_mbf(g_cmd_mbf, &command, 1, TMO_POL);
        }
        else
        {
            /* Nothing pressed or received. */
        }
    }
}

/** @brief Wake the safety task each period. */
void bench_safety_cyc (void * exinf)
{
    (void)exinf;
    (void)tk_wup_tsk(g_safety_tid);
}

/** @brief STOP raised by safety and not yet handled by control: the drive
 * tick keeps the motors braked so a step cannot re-power them in the gap.
 * Always false before the tasks run and in the host sim.
 */

/** @brief One 5 ms period, read by the side sensor every 1 ms while a
 * BARCODE run follows the line. The watchdog runs from the first sweep on;
 * feeding does nothing before it starts.
 */
void bench_tick (void)
{
    /* A send waiting for room: the robot app renews the request, marks the
     * task alive and sleeps one period; the IR the Line task took meanwhile
     * goes to the barcode store as the 1 ms polls did.
     */
    mission_hook_alive(true);
    mission_ir_drain();
}

/** @brief Poll the command sources: STOP first, then MQTT, then START.
 * The safety task calls this on the car; the host sim (and the pre-task
 * path) calls it directly through bench_command.
 */
uint8_t bench_poll_command (void)
{
    uint8_t command = BENCH_CMD_NONE;

    if (board_stop_pressed())
    {
        return BENCH_CMD_STOP;
    }
#if ROBOT_MQTT
    if (gb_mqtt && net_mqtt_ipc_receive(&command))
    {
        return command;
    }
#endif
    /* START counts once per press, after a short debounce. */
    if (board_start_pressed())
    {
        if (g_start_ticks < BENCH_BUTTON_TICKS)
        {
            ++g_start_ticks;
            if (BENCH_BUTTON_TICKS == g_start_ticks)
            {
                command = BENCH_CMD_START;
            }
        }
    }
    else
    {
        g_start_ticks = 0U;
    }
    return command;
}

/** @brief The control task's command each tick. On the car the safety task
 * owns the buttons and MQTT: STOP arrives as an event flag (handled at
 * once, braked already), other commands through the message buffer. Before
 * the tasks run, and in the host sim (gb_rtos false), poll directly so the
 * behaviour is identical to the single loop.
 */
uint8_t bench_command (void)
{
    uint8_t command = BENCH_CMD_NONE;
    UINT    pattern = 0U;

    if (!gb_rtos)
    {
        return bench_poll_command();
    }
    if (E_OK == tk_wai_flg(g_stop_flg, BENCH_FLG_STOP,
                           TWF_ORW | TWF_BITCLR, &pattern, TMO_POL))
    {
        return BENCH_CMD_STOP;
    }
    if (1 == tk_rcv_mbf(g_cmd_mbf, &command, TMO_POL))
    {
        return command;
    }
    return BENCH_CMD_NONE;
}

/** @brief Wheel distance from the 16-bit totals; consume the ISR deltas so
 * their error bits mean rejected edges or saturation in this period only.
 */
void bench_track (void)
{
    uint16_t total[BENCH_WHEELS] = {0U, 0U};
    int32_t  left = 0;
    int32_t  right = 0;
    uint32_t errors = 0U;
    uint32_t wheel;

    mission_read_encoders(&left, &right, &errors);
    board_encoder_totals(&total[BENCH_LEFT], &total[BENCH_RIGHT]);
    for (wheel = 0U; wheel < BENCH_WHEELS; ++wheel)
    {
        g_delta[wheel] = bench_signed_delta(total[wheel],
                                            g_last_total[wheel], wheel);
        g_distance[wheel] += g_delta[wheel];
        g_last_total[wheel] = total[wheel];
    }
    if (0U != errors)
    {
        ++g_encoder_errors;
        g_sweep.b_errors = true;
    }
}

/** @brief Sensors, servo and distance while no sweep runs. */
void bench_idle (uint8_t command)
{
    uint64_t now_us = 0U;

    switch (command)
    {
        case BENCH_CMD_START:
            if (0 != BENCH_MOTION)
            {
                if (0 != BENCH_PIVOT_TEST)
                {
                    g_mode = (0U == (g_pivot_starts % 2U)) ?
                             BENCH_MODE_LEFT_90 : BENCH_MODE_RIGHT_90;
                    ++g_pivot_starts;
                }
                bench_drive_start();
            }
            else
            {
                bench_sweep_start();
            }
            return;
        case BENCH_CMD_STOP:
            mission_motor_brake();
        break;
        case BENCH_CMD_RESET:
            g_distance[BENCH_LEFT] = 0;
            g_distance[BENCH_RIGHT] = 0;
            g_encoder_errors = 0U;
            bench_line(BK_ENC_ZERO);
            bench_text("ENC distance zeroed");
            bench_flush();
        break;
        case BENCH_CMD_LEFT:
        case BENCH_CMD_RIGHT:
        case BENCH_CMD_STRAIGHT:
        case BENCH_CMD_UTURN:
            /* Motion image: no test or gain selection, so ignored. */
            if (0 == BENCH_MOTION)
            {
                bench_servo(command);
            }
        break;
        default:
        break;
    }
    bench_sample();
    now_us = board_time_us();
    if ((now_us - g_report_us) >= BENCH_REPORT_US)
    {
        g_report_us = now_us;
        bench_report();
    }
}

/** @brief Per wheel and direction, then the stall-duty lower bound. */
void bench_summary (void)
{
    if (g_steps_done < BENCH_STEPS)
    {
        bench_text("Summary needs a full sweep.");
        bench_flush();
        return;
    }
    g_stall_pct = 0;
    bench_wheel_summary(1, BENCH_LEFT);
    bench_wheel_summary(1, BENCH_RIGHT);
    bench_wheel_summary(-1, BENCH_LEFT);
    bench_wheel_summary(-1, BENCH_RIGHT);
    bench_text("ROBOT_STALL_DUTY >= 0.");
    if ((g_stall_pct + BENCH_STALL_MARGIN_PCT) < 10)
    {
        bench_text("0");
    }
    bench_dec(g_stall_pct + BENCH_STALL_MARGIN_PCT);
    bench_text(" (largest steady duty + 2%)");
    bench_flush();
}

/** @brief True for the tests that AUTO-CENTRE first: LINE smooth A-C,
 * BARCODE, and AUTO-CENTRE alone.
 */
bool bench_has_auto (bench_mode_t mode)
{
    return ((BENCH_MODE_AUTO == mode) || (NULL != bench_smooth_set(mode))) &&
           (!bench_straight_code(mode));
}

/** @brief Radians as signed degrees to 0.1, without a unit. */
void bench_deg (double radians)
{
    if (radians >= 0.0)
    {
        bench_text("+");
    }
    bench_x(bench_round(radians * 1800.0 / CORE_PI), 1U, 12U); /* +-204.7 */
}

/** @brief IR every tick; the 100 Hz accelerometer when a new set is ready.
 */
void bench_sample (void)
{
    uint16_t samples[3] = {0U, 0U, 0U};
    int16_t  milligravity[3] = {0, 0, 0};
    uint32_t axis;

    if (mission_read_ir(samples))
    {
        for (axis = 0U; axis < ROBOT_IMU_AXES; ++axis)
        {
            if ((0U == g_ir.count) || (samples[axis] < g_ir.minimum[axis]))
            {
                g_ir.minimum[axis] = samples[axis];
            }
            if ((0U == g_ir.count) || (samples[axis] > g_ir.maximum[axis]))
            {
                g_ir.maximum[axis] = samples[axis];
            }
            g_ir.sum[axis] += samples[axis];
        }
        ++g_ir.count;
    }
    if (gb_accel &&
        (BOARD_ACCEL_FRESH == mission_read_accel(milligravity)))
    {
        for (axis = 0U; axis < ROBOT_IMU_AXES; ++axis)
        {
            if ((0U == g_accel.count) ||
                (milligravity[axis] < g_accel.minimum[axis]))
            {
                g_accel.minimum[axis] = milligravity[axis];
            }
            if ((0U == g_accel.count) ||
                (milligravity[axis] > g_accel.maximum[axis]))
            {
                g_accel.maximum[axis] = milligravity[axis];
            }
            g_accel.sum[axis] += milligravity[axis];
        }
        ++g_accel.count;
    }
}

/** @brief Once a second: IR, accelerometer, compass (trial) and wheel
 * distance lines. */
void bench_report (void)
{
    static const char * const ir_names[3] = {"IR L ", " R ", " B "};
    static const char * const axis_names[3] = {" x ", " y ", " z "};
    int16_t  compass[3] = {0, 0, 0};
    uint32_t axis;

    /* Frames: IR ADC 0-4095; samples 0-1023 a second; acceleration
     * +-4095 mg (13 bits); compass raw as P lines; encoder counts
     * +-16777215 (25 bits: over an hour of driving), mm +-2097151 (22 bits);
     * errors 0-65535.
     */
    bench_line(BK_IR);
    (void)bench_flag(0U != g_ir.count);
    for (axis = 0U; (axis < ROBOT_IMU_AXES) && (0U != g_ir.count); ++axis)
    {
        bench_text(ir_names[axis]);
        bench_u(g_ir.sum[axis] / g_ir.count, 12U);
        bench_text(" ");
        bench_u((uint32_t)g_ir.minimum[axis], 12U);
        bench_text("-");
        bench_u((uint32_t)g_ir.maximum[axis], 12U);
    }
    bench_flush();
    bench_line(BK_ACC);
    bench_text("ACC mg n ");
    bench_u(g_accel.count, 10U);
    for (axis = 0U; (axis < ROBOT_IMU_AXES) && (0U != g_accel.count); ++axis)
    {
        bench_text(axis_names[axis]);
        bench_s(g_accel.sum[axis] / (int32_t)g_accel.count, 13U);
        bench_text(" ");
        bench_s((int32_t)g_accel.minimum[axis], 13U);
        bench_text("..");
        bench_s((int32_t)g_accel.maximum[axis], 13U);
    }
    bench_flush();
    if (bench_mag_read(compass))
    {
        bench_line(BK_MAG);
        bench_text("MAG");
        for (axis = 0U; axis < ROBOT_IMU_AXES; ++axis)
        {
            bench_text(axis_names[axis]);
            bench_s((int32_t)compass[axis], 13U);
        }
        bench_flush();
    }
    bench_line(BK_ENC);
    bench_text("ENC counts L ");
    bench_s(g_distance[BENCH_LEFT], 25U);
    bench_text(" R ");
    bench_s(g_distance[BENCH_RIGHT], 25U);
    bench_text(" mm L ");
    bench_s(bench_mm(g_distance[BENCH_LEFT]), 22U);
    bench_text(" R ");
    bench_s(bench_mm(g_distance[BENCH_RIGHT]), 22U);
    bench_text(" err ");
    bench_u(g_encoder_errors, 16U);
    bench_flush();
    g_ramp.flat = g_accel;
    g_ir.count = 0U;
    g_ir.sum[0] = 0U;
    g_ir.sum[1] = 0U;
    g_ir.sum[2] = 0U;
    g_accel.count = 0U;
    g_accel.sum[0] = 0;
    g_accel.sum[1] = 0;
    g_accel.sum[2] = 0;
}

/** @brief Wheel travel in whole millimetres for a signed count. */
int32_t bench_mm (int32_t counts)
{
    double mm = (double)counts * 2.0 * CORE_PI * ROBOT_WHEEL_RADIUS_M *
                1000.0 / (double)ROBOT_ENCODER_COUNTS_PER_REV;

    return (int32_t)((mm >= 0.0) ? (mm + 0.5) : (mm - 0.5));
}

/** @brief Nearest whole number, halves away from zero. */
int32_t bench_round (double value)
{
    return (int32_t)((value >= 0.0) ? (value + 0.5) : (value - 0.5));
}

/** @brief *p_most = the longest of it and the time since start_us. */
void bench_slowest (uint32_t * p_most, uint64_t start_us)
{
    uint32_t took = (uint32_t)(board_time_us() - start_us);

    if (took > *p_most)
    {
        *p_most = took;
    }
}

/** @brief A number printed with no width: the line cannot be a frame
 * (bench_frame_close counts it and nothing is sent).
 */
void bench_untyped (void)
{
    if (BK_NONE != g_frame_kind)
    {
        gb_frame_text = true;
    }
}

/* ==== The robot app's entry points (22 Nov 2026) =========================
 * robot_app.c: mission_init before the tasks; the Line and IMU tasks push
 * their samples; the Vehicle task runs mission_step every 5 ms and publishes
 * mission_output(). The bench image has these too (robot_app.c is in its
 * build), but nothing calls them there.
 */

/** @brief What bench_loop set up before its tasks: the encoder baseline,
 * the BOOT line, the servo ahead.
 */
void mission_init (void)
{
    (void)memset(&g_out, 0, sizeof(g_out));
    mission_publish_halt();
    g_out.state = (uint32_t)VEHICLE_IDLE;
    gb_accel = true;
    gb_mqtt = (0 != ROBOT_MQTT);
    board_encoder_totals(&g_last_total[BENCH_LEFT],
                         &g_last_total[BENCH_RIGHT]);
    g_ir_now_total[BENCH_LEFT] = g_last_total[BENCH_LEFT];
    g_ir_now_total[BENCH_RIGHT] = g_last_total[BENCH_RIGHT];
    g_mode = BENCH_MODE_MISSION;
    /* The BOOT frame (bench_codec.py's template: the motion flag, then the
     * accelerometer's).
     */
    bench_line(BK_BOOT);
    (void)bench_flag(true);
    bench_text("BENCH MOTION ready: START runs MISSION (line-follow +"
               " barcode + obstacle go-around), STOP stops. Accel ");
    bench_text(bench_flag(gb_accel) ? "ok" : "FAILED");
    bench_flush();
    (void)bench_scan_servo(0);
    g_report_us = board_time_us();
}

/** @brief The vehicle state the mission is in, for the robot's telemetry
 * and its Safety task (an active state keeps the owners' health gates).
 */
static vehicle_state_t mission_state (void)
{
    if (!g_drive.b_running)
    {
        return VEHICLE_IDLE;
    }
    if (BENCH_AVOID_STRAIGHT != g_avoid.phase)
    {
        return VEHICLE_BYPASS;
    }
    if ((uint8_t)BENCH_NAV_IDLE != g_nav.phase)
    {
        return VEHICLE_NAVIGATE;
    }
    return VEHICLE_FOLLOW;
}

/** @brief Vehicle task: one step of bench_run_loop's body on this step's
 * input: the IR since the last step, the wheels, a new junction map, then
 * the run (or the idle reports).
 */
void mission_step (const mission_input_t * p_input)
{
    uint8_t command = BENCH_CMD_NONE;

    g_in = *p_input;
    mission_ir_drain();
    bench_track();
    bench_nav_poll_map();
    if (MISSION_CMD_STOP == p_input->command)
    {
        command = BENCH_CMD_STOP;
    }
    else if (MISSION_CMD_START == p_input->command)
    {
        command = BENCH_CMD_START;
    }
    else
    {
        /* None. */
    }
    if (g_drive.b_running)
    {
        bench_drive_tick(command);
    }
    else
    {
        /* Idle: the sensor report each second, the brake held. A STOP when
         * idle needs nothing more (the Safety task braked).
         */
        bench_idle((BENCH_CMD_STOP == command) ? BENCH_CMD_NONE : command);
        if (!g_drive.b_running)
        {
            mission_publish_halt();
        }
    }
    g_out.state = (uint32_t)mission_state();
    g_out.b_running = g_drive.b_running;
}

/** @brief The mission's current requests. */
const mission_output_t * mission_output (void)
{
    return &g_out;
}


/* End of file. */

