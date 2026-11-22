/** @file mission_line.c
 * @brief The mission: subsystem 3: line following, AUTO-CENTRE, calibration
 * and junction moves. Split out of the one file by tools/reorg/split_c.py
 * (22 Nov 2026); what the parts share is in mission_private.h.
 */
#include "mission_private.h"
#include "mission_line.h"

/* Which build (see the file comment). The bench copy of robot_config.h
 * (build.sh) defines BENCH_IMAGE and the BENCH_* settings below; the robot
 * app takes the robot's own limits and runs MISSION only.
 */
#define MISSION_ROBOT (1)

/* ==== The robot app's owners (22 Nov 2026) ===============================
 * In the robot app this file is the Vehicle task's controller (priority 4,
 * every 5 ms). The board belongs to the other tasks, so the board calls the
 * mission makes are redirected here:
 *   IR          Line task (2 ms) pushes each sample with the wheels' 16-bit
 *               totals (mission_ir_push); a step feeds all but the newest to
 *               the barcode store as the bench's 1 ms polls did, and the
 *               newest is the step's own reading (board_read_ir).
 *   accel       IMU task (10 ms) pushes each fresh sample (mission_accel_push);
 *               board_read_accel takes the oldest, FRESH, else PENDING.
 *   sonar/servo Scan task (2 ms) sets the pulse and triggers when asked
 *               (mission_output_t servo_seq/trigger_seq) and hands back the
 *               echo of that trigger (mission_input_t sonar_seq).
 *   wheels      Motion task (5 ms) runs the controller on the request the
 *               mission publishes each step; the mission reads its snapshot.
 *               Braking also asks for a HALT.
 *   watchdog    Safety task. A long decode feeds it through
 *               mission_hook_alive, which keeps the request renewed and lets
 *               the lower tasks run.
 * The encoder totals and the clock are read directly (no state is consumed).
 * Every record shared with another task is bit-packed (Change Requirement 1)
 * and passed under app_main_lock().
 */
/* One IR sample: three 12-bit readings, the 16-bit totals, valid. */
typedef struct
{
    uint32_t left_raw    : 12;
    uint32_t right_raw   : 12;
    bool     b_valid     : 1;
    uint32_t side_raw    : 12;
    uint32_t left_total  : 16;
    uint32_t right_total : 16;
} mission_ir_t;

static mission_ir_t     g_ir_ring[MISSION_IR_RING];
static uint8_t          g_ir_head;
          /* Line task writes */
static uint8_t          g_ir_tail;
          /* Vehicle task reads */
static uint8_t          g_ir_dropped;
static bool             gb_ir_fresh;
        /* g_ir_now is the newest read */
static uint8_t          g_ir_failed;

        /* failed reads in a row */
/* Noise control: failed IR reads in a row the last good sample covers. */
#define MISSION_IR_HOLD (5U)

/* line test: track runs cross junctions */
#define BENCH_LINE_MAX_M (5.0)

/* Straight on past a junction: at least BENCH_LINE_CROSS_MM from where it was
 * seen (the 18 mm track line, then clear of it), and on while both sensors
 * stay black, up to BENCH_LINE_CROSS_MAX_MM.
 */
#define BENCH_LINE_CROSS_MM (50)
#define BENCH_LINE_CROSS_MAX_MM (150)
#define BENCH_LINE_MAX_US (60000000U)
#define BENCH_LINE_TRACE_US (250000U)

/* LINE centre and smooth run twice as long, for two laps (29 Oct), with half
 * as many trace rows per second: the trace takes as long to send.
 */
#define BENCH_CENTRE_MAX_US (120000000U)
#define BENCH_CENTRE_TRACE_US (500000U)

/* Faster line tests; with the FOLLOW law (half speed at full steering, yaw
 * clamped to ROBOT_TURN_RATE_RPS) no wheel exceeds ROBOT_MAX_SPEED_MPS.
 */
#define BENCH_LINE_FAST_MPS (0.12)
#define BENCH_LINE_LOST_MS (500U)       /* white gaps of the uneven tape */
#define BENCH_ID_LAST (255U)

/* LINE centre, on line_ctrl's calibrated darkness d (0 white, 1 black) of
 * the left (d0) and right (d1) sensors. Their lenses sit wider apart than
 * the line, so it is often under neither; FOLLOW's (d1 - d0) / (d0 + d1)
 * is then full steering whenever only one sees it.
 */
#define BENCH_CENTRE_MPS (0.12)
#define BENCH_CENTRE_SEEN (0.10)     /* a sensor sees the line */
#define BENCH_CENTRE_SIDE (0.05)     /* |d1 - d0| that names the line's side */
#define BENCH_CENTRE_CROSS (0.80)    /* both this dark: a crossing line */
#define BENCH_CENTRE_KP (1.5)        /* steering per unit of d1 - d0 */
#define BENCH_CENTRE_SEARCH (1.0)    /* after a long white gap */
#define BENCH_CENTRE_GAP_M (0.05)    /* straight on between the sensors */
#define BENCH_CENTRE_CLEAR_M (0.015) /* straight on past a crossing */
#define BENCH_CENTRE_LOST_US (1500000U)
#define BENCH_CENTRE_MAX_M (30.0)

/* line_ctrl's LOST is not used. 60 s of searching in a row is line_ctrl's
 * largest (120 s refused every START, 29 Oct), and enough for a 120 s run.
 */
#define BENCH_CENTRE_LOST_MS (60000U)

/* LINE smooth A-C: LINE centre with steering that starts gently when a
 * sensor first sees the line and grows only while it stays there, changes
 * at a bounded rate, and keeps the inner wheel at BENCH_SMOOTH_INNER_MPS or
 * more. On the track (29 Oct) LINE centre steered at full lock 63% of the
 * time: the sensors go from white to black within a few mm, so
 * KP * (d1 - d0) jumps to the limit at every touch, and the car zig-zagged.
 */
#define BENCH_SMOOTH_SETS (3U)

/* Turns wide on the track (29 Oct) at a 0.65 rad/s cap: now full steering is
 * up to 0.95 rad/s (motion allows 1.0) at 65% speed, about a 9-10 cm radius,
 * the inner wheel at 20 mm/s or more and the outer within
 * ROBOT_MAX_SPEED_MPS.
 */
#define BENCH_SMOOTH_INNER_MPS (0.020)  /* inner wheel floor while steering */
#define BENCH_SMOOTH_SLOW (0.15)        /* speed cut at full steering */
#define BENCH_SMOOTH_TURN_MPS (0.085)
#define BENCH_SMOOTH_YAW_RPS (0.95)     /* yaw at full steering, and its cap */

/* Both front sensors white: steer back toward the side the line was last
 * seen, from nothing, growing by this much per metre of travel, to full at
 * 1/BENCH_SMOOTH_SEARCH_PER_M. Their lenses are wider apart than the 18 mm
 * line, so both white is also the line centred between them: that drives on
 * almost straight; a line that has gone gets more and more steering. No
 * distance is set by the track. Straight on for 50 mm and then full lock
 * swung the car off straights it was centred on (29 Oct, 180 mm/s: half the
 * run searching), and 250 mm straight on missed a bend after a crossing. A
 * line lost outward in a bend gets full steering at once (BENCH_SMOOTH_OUTWARD,
 * BENCH_SMOOTH_TURN_M).
 */
#define BENCH_SMOOTH_SEARCH_PER_M (10.0)

/* The bend test above takes the steering averaged over this much travel
 * (signed, so a zig-zag averages out), not the latest steering. On 29 Oct the
 * line passed from the right sensor to the left in a hard right bend, the
 * steering dropped below BENCH_SMOOTH_BEND within one swing and both sensors
 * dark then read as a crossing: straight on, off the loop, a 90 degree search.
 * In a sustained bend, lost with both white is outward too (search at once).
 */
#define BENCH_SMOOTH_TURN_M (0.04)
#define BENCH_SMOOTH_OUTWARD (0.8)      /* lost at this steering: a bend */
#define BENCH_SMOOTH_LOST_US (2500000U)

/* A barcode bar beside the line is at most 9 mm wide along the travel; the
 * line, and a crossing line, 18 mm. On the track (29 Oct) bars under the left
 * sensor read as the line and as a crossing. A sensor steers only after
 * BENCH_SMOOTH_DARK_M of travel at BENCH_SMOOTH_DARK or darker. A
 * 13 mm steering filter with a 3/s steering rate lagged a quarter swing
 * behind the line and the car swung across it (29 Oct): the steering filter
 * is short and the rates fast.
 */
#define BENCH_SMOOTH_DARK (0.5)

/* All three sensors (the barcode one too) dark this far: lifted, or off the
 * mat (its floor reads dark), not a line (18 mm); stop. On 29 Oct the car,
 * lifted after it stalled, kept driving its wheels for 4 s.
 */
#define BENCH_SMOOTH_ALL_DARK_M (0.08)
#define BENCH_SMOOTH_DARK_M (0.006)

/* Junctions (19 Nov: junctions are only on a straight line, never at
 * a turn). Both front sensors dark (BENCH_SMOOTH_DARK, each for at least
 * BENCH_CROSS_MIN_M) is a junction while the car is going straight, and
 * never while it is turning: going straight is the wheels' heading turned
 * less than BENCH_STRAIGHT_RAD over the last BENCH_STRAIGHT_SLOTS x
 * BENCH_STRAIGHT_STEP_M of travel. The 13 mm minimum dark length
 * (BENCH_SMOOTH_CROSS_M) and the per-sensor skew test are gone
 * (19 Nov: junctions were missed with them). With no minimum at all the sim
 * counted 3-4 junctions on a straight entered 8-12 mm off centre (both
 * sensors brushing the line); 3 mm took those out and still caught every
 * real junction (0-35 deg skew, mid-correction starts).
 */
#define BENCH_CROSS_MIN_M (0.003)
#define BENCH_EDGE_TARGET (0.5)
#define BENCH_EDGE_KP (2.0)
#define BENCH_EDGE_CROSS (0.7)
#define BENCH_STRAIGHT_STEP_M (0.005)
#define BENCH_STRAIGHT_RAD (20.0 * CORE_PI / 180.0)
#define BENCH_CODE_MPS (0.08)
#define BENCH_CODE_MAX_M (0.8)

/* MISSION also sends, while it drives, the stored samples of each cut that
 * made a group (bench_raw_keep, bench_raw_text): a RAW header, then lines
 * "R k:" of one character per unit, '#' + (raw - white + BENCH_RAW_BELOW) /
 * BENCH_RAW_STEP below level BENCH_RAW_KNEE, coarser by BENCH_RAW_HIGH from
 * it up to '~', and BENCH_RAW_GAP for a unit without a reading. The fine
 * part keeps the spaces and print; the coarse one shows bar peaks and the
 * tape (30 Oct at the lowest mount: bars past white + 172, the old ceiling).
 * The status line then comes every BENCH_LIVE_RAW_US. A cut decoded while the
 * last is still going out is skipped and counted; one left at the stop is
 * dropped: nothing long goes out after the stop.
 */
#define BENCH_RAW_CHARS (4096U)
#define BENCH_RAW_LINE (64U)   /* 7 bits a character: 64 fill a 64 B frame */
#define BENCH_RAW_ZERO ('#')
#define BENCH_RAW_TOP ('~')
#define BENCH_RAW_GAP ('!')
#define BENCH_RAW_BELOW (10)
#define BENCH_RAW_STEP (2)
#define BENCH_RAW_KNEE (64)    /* 'c': white + 118 */
#define BENCH_RAW_HIGH (140)   /* '~': white + 3898 and above */

/* START calibration: each IR channel's
 * lowest and highest reading over AUTO-CENTRE's pivots and the first
 * BENCH_CAL_M of line, sent once (CAL) beside the configured white and
 * black. Measured only: the front spans scale darkness, so applied they
 * would change the tuned steering strength.
 */
#define BENCH_CAL_M (4.0 * ROBOT_ROBOT_LENGTH_M)

/* AUTO-CENTRE: pivots at the S2-U10 turn rate (wheels at 42 mm/s, which the
 * PID holds; slower wheels stall on the floor) within BENCH_AUTO_SWEEP_RAD of
 * the START heading. A sensor finds the line when its darkness reaches
 * BENCH_SMOOTH_DARK after being below BENCH_AUTO_WHITE in this sweep, and
 * stays there for BENCH_AUTO_CONFIRM_RAD of turn.
 */
#define BENCH_AUTO_YAW_RPS (ROBOT_TURN_RATE_RPS)
#define BENCH_AUTO_SWEEP_RAD (70.0 * CORE_PI / 180.0)
#define BENCH_AUTO_WHITE (0.25)
#define BENCH_AUTO_CONFIRM_RAD (1.0 * CORE_PI / 180.0)
#define BENCH_AUTO_SPAN_RAD (50.0 * CORE_PI / 180.0) /* edges apart, at most */
#define BENCH_AUTO_LEAD_RAD (0.5 * CORE_PI / 180.0)  /* halt this early */
#define BENCH_AUTO_TOLERANCE_RAD (2.0 * CORE_PI / 180.0)
#define BENCH_AUTO_CORRECTIONS (2U)
#define BENCH_AUTO_PAUSE_US (300000U)   /* halted between pivots */
#define BENCH_AUTO_SETTLE_US (500000U)  /* halted at the end, at least */
#define BENCH_AUTO_STILL_MMS (12)       /* app_cfg_motion stationary_mps */
#define BENCH_AUTO_STALL_RAD (1.0 * CORE_PI / 180.0)
#define BENCH_AUTO_STALL_US (1500000U)  /* a pivot turning less: stalled */
#define BENCH_AUTO_MAX_US (30000000U)

/* LINE smooth A-C, in mode order, all with the soft steering. Before it a
 * touch held 0.3 s reached full steering and in the loop the line went from
 * one sensor to the other every 250 ms (29 Oct: 6.1 m in 60 s, 102 mm/s
 * mean, at 0.20 + 2.5/s). Soft, 60 s: 6.0 m at 110 mm/s, 6.5 m at 120 and
 * 6.9 m at 130 (C). In bends the speed falls toward BENCH_SMOOTH_TURN_MPS
 * (bench step). The ramp is now per metre: 11.5/m is the soft 1.5/s at
 * 130 mm/s, for every set. 23/m for A (30 Oct) searched more (36% of the
 * time against 28%), turned onto crossing lines and read no code.
 */
static const bench_smooth_t g_smooth[BENCH_SMOOTH_SETS] =
{
    { 0.18, 0.15, 11.5, 12.0 },
    { 0.15, 0.15, 11.5, 12.0 },
    { 0.13, 0.15, 11.5, 12.0 }
};

/* Indexed by bench_centre_phase_t. */
static const char * const g_centre_names[BENCH_CENTRE_PHASES] =
{
    "TRACK", "GAP", "SEARCH", "CROSS"
};

/* Why AUTO-CENTRE failed (bench_auto_fail's reasons), sent as the index. */
#define BENCH_AUTO_FAILS (6U)
static const char * const g_auto_fail_names[BENCH_AUTO_FAILS] =
{
    "line sensor fault",
    "both front sensors see a line at START (a crossing?): start on a plain"
    " stretch",
    "not centred within 30 s",
    "pivot stalled: under 1 deg in 1.5 s (wheels stuck or slipping)",
    "no line within 70 deg either side of the START heading",
    "the sensors found the line over 50 deg apart: not one straight line"
    " (bend, crossing, bars?)"
};
 /* and x: readings with no range */
static char          g_raw[BENCH_RAW_CHARS];
static uint32_t      g_raw_number;
   /* its number in the run */
static uint32_t      g_raw_first;
    /* its first unit */
static uint32_t      g_raw_white;
static uint32_t      g_raw_sd;

/* MISSION junction moves (6 Nov). The PC sends the letter -> move map over
 * MQTT (navmap.h) before the run; no letter is fixed here. Each confirmed
 * code (bench_code_tries_step) stores its letter; the next cross-junction
 * (both front sensors on a crossing line, bench_line_black) makes that
 * letter's move, then the letter is used up:
 *   FORWARD  straight on, as before (bench_line_cross),
 *   LEFT / RIGHT / U-TURN  ADVANCE straight on the junction heading until
 *     the wheel axle is over the crossing (the front sensors are
 *     ROBOT_LINE_SENSOR_FORWARD_M ahead of it and see the crossing
 *     BENCH_CROSS_MIN_M into its 18 mm), then PIVOT in place by the
 *     wheels' heading (90 deg, or 180 for the U-turn, turned angle summed so
 *     a half turn has no short-way ambiguity), then the line follower starts
 *     afresh (bench_avoid_rejoin) on the new branch.
 * No letter stored, or no map yet: straight on, and said so.
 */
#define BENCH_NAV_ADVANCE_MM ((ROBOT_LINE_SENSOR_FORWARD_M * 1000.0) - \
                              ((BENCH_CROSS_MIN_M * 1000.0) - \
                               (BENCH_AVOID_LINE_MM / 2.0)))
#define BENCH_NAV_TURN_DEG (90.0)
#define BENCH_NAV_UTURN_DEG (180.0)

/* A junction met while the card's decode is not finished (14 Nov:
 * the fit, Decoder 2, was "?" at every junction that day; it takes ~1.1 s a
 * card):
 * the car stops on it, has what is stored decoded at once (bench_nav_force:
 * no more white will come while it stands), and decides once nothing is
 * left to decode, or after BENCH_NAV_WAIT_MS with what is in. Not when two
 * decoders already agree: the third cannot change a majority of three.
 */
#define BENCH_NAV_WAIT_MS (3000U)

/* A turn is not a junction (22 Nov: the sharp right turn ~0.5 m after
 * card 1 was taken for a junction, and card Y's U-turn was made there; the
 * 19 Nov runs show the same "junction" ~0.5 m after the card). At that turn
 * both front sensors were dark for ~20 mm, as at a crossing (25 mm), so the
 * dark itself cannot tell them apart; what follows can. Past a crossing the
 * line goes on (the car's own trace: dark again 35-45 mm on); past the turn
 * there is no line ahead (both white from 37 mm on for 125 mm, then the
 * search found it to the right). So both dark starts a PROBE: straight on
 * the heading of the moment, and the junction counts (its letter's move
 * made, its advance from the dark's start) only when a front sensor sees the
 * line again between BENCH_NAV_PROBE_FROM_MM and BENCH_NAV_PROBE_TO_MM past
 * where it went dark (darker than BENCH_NAV_LINE_DARK, or the two together
 * past BENCH_NAV_LINE_SUM: the line between them). Not seen by then: a
 * turn, said so, the letter kept for the next junction, and the line
 * follower takes over (its search finds the turn's branch). A dark band
 * BENCH_NAV_GUARD_MM after a junction is the same crossing (22 Nov 18:18:
 * one junction counted twice, 58 mm apart).
 */
#define BENCH_NAV_PROBE_FROM_MM (45.0)
#define BENCH_NAV_PROBE_TO_MM (90.0)
#define BENCH_NAV_LINE_DARK (0.5)
#define BENCH_NAV_LINE_SUM (0.4)
#define BENCH_NAV_GUARD_MM (100)

/** @brief True for the line tests. */
bool bench_is_line (bench_mode_t mode)
{
    return (BENCH_MODE_LINE == mode) || (BENCH_MODE_LINE_FAST == mode) ||
           (BENCH_MODE_LINE_MAX == mode) || (BENCH_MODE_LINE_STOP == mode) ||
           (BENCH_MODE_AVOID == mode) ||
           (BENCH_MODE_AUTO == mode) || bench_is_centre(mode);
}

/** @brief True for the LINE centre tests. */
bool bench_is_centre (bench_mode_t mode)
{
    return (BENCH_MODE_CENTRE == mode) || (BENCH_MODE_CENTRE_MAX == mode) ||
           (NULL != bench_smooth_set(mode));
}

/** @brief The LINE smooth setting of a mode (BARCODE and MISSION steer as
 * LINE smooth A, BARCODE slower), or NULL for other tests.
 */
const bench_smooth_t * bench_smooth_set (bench_mode_t mode)
{
    const bench_smooth_t * p_set = NULL;

    if ((mode >= BENCH_MODE_SMOOTH_A) && (mode <= BENCH_MODE_SMOOTH_C))
    {
        p_set = &g_smooth[(uint32_t)mode - (uint32_t)BENCH_MODE_SMOOTH_A];
    }
    else if ((BENCH_MODE_BARCODE == mode) || (BENCH_MODE_MISSION == mode))
    {
        p_set = &g_smooth[0];
    }
    else
    {
        /* Not a LINE smooth steering test. */
    }
    return p_set;
}

/** @brief A line test's straight-line speed in m/s; 0 for other tests. */
double bench_line_speed (bench_mode_t mode)
{
    double speed = 0.0;

    if ((BENCH_MODE_LINE == mode) || (BENCH_MODE_AVOID == mode))
    {
        speed = ROBOT_LINE_SPEED_MPS;
    }
    else if ((BENCH_MODE_LINE_FAST == mode) ||
             (BENCH_MODE_LINE_STOP == mode))
    {
        speed = BENCH_LINE_FAST_MPS;
    }
    else if ((BENCH_MODE_LINE_MAX == mode) ||
             (BENCH_MODE_CENTRE_MAX == mode))
    {
        speed = ROBOT_MAX_SPEED_MPS;
    }
    else if (BENCH_MODE_CENTRE == mode)
    {
        speed = BENCH_CENTRE_MPS;
    }
    else if (BENCH_MODE_BARCODE == mode)
    {
        speed = BENCH_CODE_MPS;
    }
    else if (BENCH_MODE_MISSION == mode)
    {
        speed = BENCH_MISSION_MPS;
    }
    else if (NULL != bench_smooth_set(mode))
    {
        speed = bench_smooth_set(mode)->mps;
    }
    else
    {
        /* Not a line test. */
    }
    return speed;
}

/** @brief Line test: the robot's line calibration (app_cfg_line), and the
 * first sample must be TRACKING, as FOLLOW requires; else say why not.
 * LINE centre also starts SEARCHING (the line between the sensors) and
 * leaves loss to bench_centre_steer, as does AUTO-CENTRE. AVOID uses no line
 * sensors: it only arms the front sonar and drives straight, so it starts
 * without a line sample.
 */
bool bench_line_start (void)
{
    line_ctrl_config_t config;
    line_ctrl_output_t output;
    uint16_t           raw[LINE_CTRL_CHANNELS] = {0U, 0U, 0U};
    bool               b_valid = false;
    bool               b_centre = bench_is_centre(g_mode) ||
                                  (BENCH_MODE_AUTO == g_mode);

    if (BENCH_MODE_AVOID == g_mode)
    {
        (void)memset(&g_line, 0, sizeof(g_line));
        (void)memset(&g_avoid, 0, sizeof(g_avoid)); /* yaw_home = 0 = start */
        bench_front_start();                        /* front sonar, centred */
        return true;
    }
    (void)memset(&g_line, 0, sizeof(g_line));
    (void)memset(&g_centre, 0, sizeof(g_centre));
    (void)memset(&g_cal, 0, sizeof(g_cal));
    bench_rec_reset();
    bench_nav_start();
    bench_auto_start();
    (void)memset(&output, 0, sizeof(output));
    app_cfg_line(&config);
    config.lost_timeout_ms = b_centre ? BENCH_CENTRE_LOST_MS :
                                        BENCH_LINE_LOST_MS;
    line_ctrl_init(&g_line_state);
    b_valid = line_ctrl_derive(&config, &g_line_runtime) &&
              board_read_ir(raw) &&
              line_ctrl_step(&g_line_runtime, &g_line_state, raw,
                             (uint32_t)(board_time_us() / 1000U), &output);
    /* The straight barcode image starts wherever the car is set down. */
    if (b_valid && ((LINE_CTRL_TRACKING == output.status) ||
                    (b_centre && (LINE_CTRL_SEARCHING == output.status)) ||
                    bench_straight_code(g_mode)))
    {
        if ((BENCH_MODE_LINE_STOP == g_mode) ||
            ((BENCH_MODE_MISSION == g_mode) && (!bench_straight_code(g_mode))))
        {
            bench_front_start();
        }
        if (BENCH_MODE_MISSION == g_mode)
        {
            /* phase 0 = STRAIGHT; yaw_home/last_centre are set at detection. */
            (void)memset(&g_avoid, 0, sizeof(g_avoid));
            g_avoid_queued = 0U;
            g_avoid_dropped = 0U;
            g_pose_due_us = 0U;
        }
        if ((BENCH_MODE_BARCODE == g_mode) || (BENCH_MODE_MISSION == g_mode))
        {
            return bench_code_start();
        }
        return true;
    }
    bench_text("LINE refused: line ");
    bench_text(b_valid ?
               g_line_names[(uint32_t)output.status % BENCH_LINE_STATES] :
               "FAULT");
    bench_text(", IR L ");
    bench_dec((int32_t)raw[0]);
    bench_text(" R ");
    bench_dec((int32_t)raw[1]);
    bench_text(" B ");
    bench_dec((int32_t)raw[2]);
    bench_text("; put the front sensors on the line");
    bench_flush();
    return false;
}

/** @brief Line test, before each motion_step: one IR sample through
 * line_ctrl, then the robot's FOLLOW steering (robot_app_vehicle_input and
 * vehicle_follow) while TRACKING. While SEARCHING it renews the last
 * command, where the robot halts; before the first command it halts. A
 * junction is crossed straight on (bench_line_cross). A lost line, a fault
 * or the distance or time limit ends the test; the wheels then hold a halt.
 * LINE centre steers by bench_centre_steer on every sample instead, and
 * ends on its own lost line. The AUTO-CENTRE tests (bench_has_auto) run
 * bench_auto_step on the samples first, until it hands over or fails.
 */
void bench_line_step (uint64_t now_us, motion_request_t * p_request)
{
    line_ctrl_output_t output;
    bench_line_row_t   sample;
    uint16_t           raw[LINE_CTRL_CHANNELS] = {0U, 0U, 0U};
    uint64_t           t_us = now_us - g_drive.step_us;
    int32_t            centre = ((g_distance[BENCH_LEFT] -
                                  g_drive.start[BENCH_LEFT]) +
                                 (g_distance[BENCH_RIGHT] -
                                  g_drive.start[BENCH_RIGHT])) / 2;
    double             steering = 0.0;
    double             yaw = 0.0;
    double             yaw_max = ROBOT_TURN_RATE_RPS;
    double             forward = 0.0;
    bool               b_valid = false;
    bool               b_range = false;
    bool               b_centre = bench_is_centre(g_drive.mode);
    bool               b_lost = false;
    uint32_t           channel;

    (void)memset(p_request, 0, sizeof(*p_request));
    p_request->kind = MOTION_HALT;
    if (((BENCH_MODE_LINE_STOP == g_drive.mode) ||
         (BENCH_MODE_AVOID == g_drive.mode) ||
         (BENCH_MODE_MISSION == g_drive.mode)) &&
        (!bench_straight_code(g_drive.mode)))
    {
        /* Also after the stop, for the range 1 s later. */
        b_range = bench_front_step(now_us);
    }
    if (g_drive.b_done)
    {
        return;
    }
    /* AVOID uses no line sensors: odometry holds the heading and the front
     * sonar finds the obstacle, so every phase drives from bench_avoid_drive,
     * before line_ctrl ever runs.
     */
    if (BENCH_MODE_AVOID == g_drive.mode)
    {
        bench_avoid_drive(now_us, b_range, centre, p_request);
        return;
    }
    /* MISSION follows the line until the sonar finds an obstacle; while the
     * go-around runs (phase left STRAIGHT) the trapezoid drives instead of
     * line_ctrl, then it hands back (bench_avoid_rejoin) and the line is
     * reacquired.
     */
    if ((BENCH_MODE_MISSION == g_drive.mode) &&
        (BENCH_AVOID_STRAIGHT != g_avoid.phase))
    {
        bench_avoid_drive(now_us, b_range, centre, p_request);
        bench_avoid_live(now_us);
        if (BENCH_AVOID_STRAIGHT == g_avoid.phase)
        {
            bench_avoid_rejoin(now_us, centre);
        }
        return;
    }
    /* MISSION: a junction move (advance, then pivot) drives by the wheels'
     * heading, not the line, until it hands back (bench_nav_drive).
     */
    if ((BENCH_MODE_MISSION == g_drive.mode) &&
        ((uint8_t)BENCH_NAV_IDLE != g_nav.phase))
    {
        bench_nav_drive(now_us, centre, p_request);
        bench_avoid_live(now_us);
        return;
    }
    (void)memset(&output, 0, sizeof(output));
    b_valid = board_read_ir(raw);
    if (b_valid && (BENCH_MODE_MISSION == g_drive.mode))
    {
        bench_cal_sample(raw, bench_mm(centre));
    }
    b_valid = line_ctrl_step(&g_line_runtime, &g_line_state, raw,
                             (uint32_t)(now_us / 1000U), &output) && b_valid;
    if (g_auto.b_active)
    {
        /* AUTO-CENTRE first; the line test starts where it hands over. */
        if (!b_valid)
        {
            bench_auto_fail(now_us, "line sensor fault");
            return;
        }
        (void)memcpy(g_auto.raw, raw, sizeof(g_auto.raw));
        bench_auto_step(now_us, &output, p_request);
        return;
    }
    if (!b_valid)
    {
        output.status = LINE_CTRL_FAULT;
    }
    else if (b_centre)
    {
        steering = bench_centre_steer(&output, centre, now_us, &b_lost);
        for (channel = 0U; channel < 2U; ++channel)
        {
            if (raw[channel] > g_centre.raw_max[channel])
            {
                g_centre.raw_max[channel] = raw[channel];
            }
        }
    }
    else
    {
        /* FOLLOW's steering, below. */
    }
    if (b_valid && ((BENCH_MODE_BARCODE == g_drive.mode) ||
                    (BENCH_MODE_MISSION == g_drive.mode)))
    {
        bench_code_sample(g_distance[BENCH_LEFT] - g_drive.start[BENCH_LEFT],
                          g_distance[BENCH_RIGHT] - g_drive.start[BENCH_RIGHT],
                          raw[LINE_CTRL_BARCODE_CHANNEL]);
    }
    if (BENCH_MODE_MISSION == g_drive.mode)
    {
        uint64_t part_us = board_time_us();

        bench_code_tries_step();
        bench_slowest(&g_drive.slow_tries_us, part_us);
        g_live_front[0] = raw[0];
        g_live_front[1] = raw[1];
        part_us = board_time_us();
        bench_live_step(now_us, (uint32_t)output.status, bench_mm(centre),
                        raw[LINE_CTRL_BARCODE_CHANNEL]);
        bench_slowest(&g_drive.slow_live_us, part_us);
        bench_nav_say();   /* a stored code or a new map, queued to go out */
    }
    (void)memset(&sample, 0, sizeof(sample));
    sample.centre_mm = (int16_t)bench_mm(centre);
    sample.heading_ddeg =
        (int16_t)bench_round(g_drive.yaw * 1800.0 / CORE_PI);
    (void)memcpy(sample.raw, raw, sizeof(sample.raw));
    sample.front_mm = g_front.last_mm;
    sample.steering_q7 = b_centre ?
        (int8_t)bench_round(steering * (double)LINE_CTRL_STEERING_ONE) :
        output.steering_q7;
    sample.status = (uint8_t)output.status;
    sample.phase = g_centre.phase;
    (void)memcpy(sample.wheel_mms, g_line.wheel_mms, sizeof(sample.wheel_mms));
    (void)memcpy(sample.duty_pm, g_line.duty_pm, sizeof(sample.duty_pm));
    if ((g_line.rows < BENCH_LINE_ROWS) &&
        (t_us >= ((uint64_t)g_line.rows *
                  (b_centre ? BENCH_CENTRE_TRACE_US : BENCH_LINE_TRACE_US))))
    {
        g_line.row[g_line.rows] = sample;
        ++g_line.rows;
    }
    g_line.end = sample;
    g_line.end_ms = (uint32_t)(t_us / 1000U);
    if (bench_straight_code(g_drive.mode))
    {
        /* Straight barcode image: the samples above read the cards; no line
         * end, obstacle or limit applies. Straight on the START heading
         * until STOP; an IR fault (no barcode reading) ends it.
         */
        if (!b_valid)
        {
            bench_line_end(now_us, "stopped: IR sensor fault");
            return;
        }
        bench_line_moving();
        bench_avoid_hold(0.0, &g_drive.request);
        *p_request = g_drive.request;
        return;
    }
    /* MISSION: an obstacle within reach starts the trapezoid go-around; the
     * home heading and offset are taken from here and bench_avoid_drive takes
     * over until it is back on the line. Checked before the line-end tests.
     * BENCH_AVOID_CONFIRM valid readings in a row must be in reach, so one
     * stray echo does not start it. */
    if ((BENCH_MODE_MISSION == g_drive.mode) && b_range &&
        (BENCH_AVOID_STRAIGHT == g_avoid.phase))
    {
        g_avoid.close = ((0U != g_front.last_mm) &&
                         (g_front.last_mm <= BENCH_AVOID_TURN_MM)) ?
                        (g_avoid.close + 1U) : 0U;
    }
    if ((BENCH_MODE_MISSION == g_drive.mode) && b_range &&
        (BENCH_AVOID_STRAIGHT == g_avoid.phase) &&
        (g_avoid.close >= BENCH_AVOID_CONFIRM))
    {
        g_avoid.yaw_home = g_drive.heading; /* the line heading at detection */
        bench_avoid_begin(now_us, centre);
        bench_line(BK_AV_OBSTACLE);   /* sonar 0-4095 mm */
        bench_text("MISSION: obstacle at ");
        bench_u((uint32_t)g_front.last_mm, 12U);
        bench_text(" mm, stopped to measure it");
        bench_avoid_say();
        bench_avoid_ir_say();
        bench_avoid_drive(now_us, b_range, centre, p_request);
        return;
    }
    if ((BENCH_MODE_LINE_STOP == g_drive.mode) && b_range &&
        (!g_front.b_robot) &&
        (g_front.last_mm <= ROBOT_FRONT_STOP_DISTANCE_MM))
    {
        g_front.b_robot = true;
        g_front.robot_mm = g_front.last_mm;
        g_front.robot_centre_mm = sample.centre_mm;
    }
    /* Vehicle checks the obstacle before the line, as here. */
    if ((BENCH_MODE_LINE_STOP == g_drive.mode) && b_range &&
        (g_front.last_mm <= BENCH_LINE_STOP_MM))
    {
        g_front.b_detected = true;
        g_front.detect_mm = g_front.last_mm;
        g_front.detect_centre_mm = sample.centre_mm;
        bench_line_end(now_us, "stopped: obstacle ahead");
        return;
    }
    if (b_centre && (g_centre.all_dark_m >= BENCH_SMOOTH_ALL_DARK_M))
    {
        bench_line_end(now_us,
                       "stopped: all three sensors dark (lifted, or off"
                       " the mat)");
        return;
    }
    /* MISSION: a new junction makes the stored letter's move; FORWARD (or
     * none) goes on straight below, as before. With the card still decoding
     * the car first stands on it until the decode is done (bench_nav_wait).
     * A wait, once begun, lasts after both sensors have left the tape: when
     * it started 13 mm into the 18 mm tape, on 19 Nov the
     * stop was asked for during the last ~5 mm (~50 ms) alone, the car drove
     * on unslowed, and the move was made at the next dark patch (A's U-turn
     * at 1631 mm for the junction at 1307 mm). The move is chosen at the
     * junction's own position, so a turn's advance runs from the crossing.
     */
    if ((BENCH_MODE_MISSION == g_drive.mode) && g_nav.b_probe)
    {
        /* Straight on past the dark until it is a junction or a turn. */
        if (bench_nav_probe(now_us, centre, p_request))
        {
            return;
        }
    }
    else if ((BENCH_MODE_MISSION == g_drive.mode) && (!g_line.b_crossing) &&
             (bench_line_black(&output) || g_nav.b_waiting) &&
             (g_nav.b_waiting || (bench_mm(centre) >= g_nav.guard_to_mm)))
    {
        int32_t at = g_nav.b_waiting ? g_nav.wait_centre : centre;

        if (bench_nav_wait(now_us, centre, p_request))
        {
            return;
        }
        g_nav.b_probe = true;
        g_nav.probe_from = at;
        g_nav.probe_heading = g_drive.heading;
        if (bench_nav_probe(now_us, centre, p_request))
        {
            return;
        }
    }
    if (bench_line_cross(&output, (int32_t)sample.centre_mm, p_request))
    {
        return;
    }
    if (bench_line_black(&output))
    {
        bench_line_end(now_us, "stopped: both sensors black past a junction");
        return;
    }
    if (b_lost || (LINE_CTRL_LOST == output.status))
    {
        bench_line_end(now_us, "stopped: line lost");
        return;
    }
    /* LINE smooth: line_ctrl's JUNCTION can come before bench_centre_steer
     * calls it a crossing (it needs the car going straight); it goes on.
     */
    if ((LINE_CTRL_SEARCHING != output.status) &&
        (LINE_CTRL_TRACKING != output.status) &&
        (!(b_centre && (LINE_CTRL_JUNCTION == output.status))))
    {
        bench_line_end(now_us, "stopped: line sensor fault");
        return;
    }
    /* MISSION (the presentation run, 21 Nov) has no distance or time limit:
     * it runs until it is lifted (bench_lift_check), STOP, an obstacle stop,
     * a lost line or a fault. The test modes keep their limits.
     */
    if ((BENCH_MODE_MISSION != g_drive.mode) &&
        (fabs((double)sample.centre_mm) >=
         (((BENCH_MODE_BARCODE == g_drive.mode) ? BENCH_CODE_MAX_M :
           b_centre ? BENCH_CENTRE_MAX_M : BENCH_LINE_MAX_M) * 1000.0)))
    {
        bench_line_end(now_us, "stopped at the distance limit");
        return;
    }
    if ((BENCH_MODE_MISSION != g_drive.mode) &&
        (t_us >= (b_centre ? BENCH_CENTRE_MAX_US : BENCH_LINE_MAX_US)))
    {
        bench_line_end(now_us, "stopped at the time limit");
        return;
    }
    if ((!b_centre) && (LINE_CTRL_SEARCHING == output.status))
    {
        if (g_line.b_moving)
        {
            if (!g_line.b_gap)
            {
                ++g_line.gaps;
            }
            g_line.b_gap = true;
            *p_request = g_drive.request;
        }
        return;
    }
    g_line.b_gap = false;
    bench_line_moving();
    if (!b_centre)
    {
        steering = (double)output.steering_q7 /
                   (double)LINE_CTRL_STEERING_ONE;
    }
    if ((BENCH_MODE_MISSION == g_drive.mode) && bench_code_hold())
    {
        /* Once the hold lets go (bench_hold_lets_go), FOLLOW steers for the
         * rest of that card.
         */
        if ((!g_code.b_let_go) && bench_hold_lets_go())
        {
            g_code.b_let_go = true;
            ++g_code.let_gos;
        }
    }
    else
    {
        g_code.b_let_go = false;
    }
    if ((BENCH_MODE_MISSION == g_drive.mode) && bench_code_hold() &&
        (!g_code.b_let_go))
    {
        /* Straight over a card (bench_code_hold), started on a straight: no
         * steering while the line stays between the front sensors. A front
         * sensor that reaches the line keeps FOLLOW's correction (a touch is
         * small, bench_smooth_track), so the car stays on the line. The gap
         * search the hold suppresses is not left to build up: the gap and the
         * slewed command are kept at what the car does, so it goes on
         * straight after the card instead of swinging to full lock.
         */
        if (!(((uint8_t)BENCH_PHASE_TRACK == g_centre.phase) &&
              (0 != g_centre.seen_side)))
        {
            steering = 0.0;
            g_centre.gap_m = 0.0;
            g_centre.cmd = 0.0;
        }
        if (!g_code.b_held)
        {
            g_code.b_held = true;
            g_code.held_from = bench_mm(centre);
            ++g_code.holds;
        }
    }
    else if (g_code.b_held)
    {
        g_code.b_held = false;
        g_code.held_mm += (uint32_t)((bench_mm(centre) >= g_code.held_from) ?
                                     (bench_mm(centre) - g_code.held_from) :
                                     (g_code.held_from - bench_mm(centre)));
    }
    else
    {
        /* Not over a card. */
    }
    /* Straight over the hump (bench_ramp_hold: a rise or lean open that
     * began on a straight), as over a card: no steering while the line
     * stays between the front sensors, a sensor on the line keeps its
     * correction, and a sustained bend (BENCH_SMOOTH_BEND) lets go for the
     * rest of that hump.
     */
    if ((BENCH_MODE_MISSION == g_drive.mode) && bench_ramp_hold())
    {
        if ((!g_ramp.b_hump_go) &&
            (fabs(g_centre.turn) >= BENCH_SMOOTH_BEND))
        {
            g_ramp.b_hump_go = true;
            ++g_ramp.hump_let_gos;
        }
        if (!g_ramp.b_hump_go)
        {
            if (!(((uint8_t)BENCH_PHASE_TRACK == g_centre.phase) &&
                  (0 != g_centre.seen_side)))
            {
                steering = 0.0;
                g_centre.gap_m = 0.0;
                g_centre.cmd = 0.0;
            }
            if (!g_ramp.b_hump_held)
            {
                g_ramp.b_hump_held = true;
                g_ramp.hump_held_from = bench_mm(centre);
                ++g_ramp.hump_holds;
            }
        }
    }
    else
    {
        g_ramp.b_hump_go = false;
    }
    if (g_ramp.b_hump_held &&
        ((!bench_ramp_hold()) || g_ramp.b_hump_go))
    {
        g_ramp.b_hump_held = false;
        g_ramp.hump_held_mm +=
            (uint32_t)((bench_mm(centre) >= g_ramp.hump_held_from) ?
                       (bench_mm(centre) - g_ramp.hump_held_from) :
                       (g_ramp.hump_held_from - bench_mm(centre)));
    }
    yaw = steering * ROBOT_LINE_GAIN_RPS;
    forward = bench_line_speed(g_drive.mode) * (1.0 - (0.5 * fabs(steering)));
    if (NULL != bench_smooth_set(g_drive.mode))
    {
        /* Keep the inner wheel at BENCH_SMOOTH_INNER_MPS or more, where its
         * PID still drives it, instead of near standstill.
         */
        yaw = steering * BENCH_SMOOTH_YAW_RPS;
        forward = bench_line_speed(g_drive.mode) *
                  (1.0 - (BENCH_SMOOTH_SLOW * fabs(steering)));
        forward = fmin(forward, bench_line_speed(g_drive.mode) -
                                (fmax(0.0, bench_line_speed(g_drive.mode) -
                                           BENCH_SMOOTH_TURN_MPS) *
                                 fabs(steering)));
        /* The turn first: slow down until the outer wheel is within the
         * motion limit (with a margin), so a faster set turns as sharply as
         * 100 mm/s did.
         */
        forward = fmin(forward, (BENCH_SMOOTH_WHEEL_MPS - 0.002) -
                                (fabs(yaw) * 0.5 * ROBOT_TRACK_WIDTH_M));
        yaw_max = (forward - BENCH_SMOOTH_INNER_MPS) /
                  (0.5 * ROBOT_TRACK_WIDTH_M);
        yaw_max = fmin(yaw_max, BENCH_SMOOTH_YAW_RPS);
        yaw_max = fmax(0.0, yaw_max);
    }
    if (yaw > yaw_max)
    {
        yaw = yaw_max;
    }
    if (yaw < -yaw_max)
    {
        yaw = -yaw_max;
    }
    g_drive.request.forward_mps = forward;
    /* Optical positive-right becomes differential-drive positive-left yaw. */
    g_drive.request.yaw_rps = -yaw;
    *p_request = g_drive.request;
}

/** @brief Line test: a junction (a track crossing) is crossed straight on at
 * the line speed, the way the car was going, then the line followed again.
 * Line loss is ignored while crossing; a sensor fault is not.
 * @return true while crossing (the request is set), else false.
 */
bool bench_line_cross (const line_ctrl_output_t * p_output,
                              int32_t centre_mm, motion_request_t * p_request)
{
    int32_t travelled = 0;
    bool    b_black = bench_line_black(p_output);

    if (b_black && (!g_line.b_crossing))
    {
        g_line.b_crossing = true;
        g_line.cross_from_mm = centre_mm;
        ++g_line.junctions;
    }
    if (!g_line.b_crossing)
    {
        return false;
    }
    travelled = centre_mm - g_line.cross_from_mm;
    if ((LINE_CTRL_FAULT != p_output->status) &&
        (LINE_CTRL_INHIBITED != p_output->status) &&
        ((travelled < BENCH_LINE_CROSS_MM) ||
         (b_black && (travelled < BENCH_LINE_CROSS_MAX_MM))))
    {
        bench_line_moving();
        g_drive.request.forward_mps = bench_line_speed(g_drive.mode);
        g_drive.request.yaw_rps = 0.0;
        *p_request = g_drive.request;
        return true;
    }
    g_line.b_crossing = false;
    return false;
}

/** @brief Line test: number the velocity primitive once, at the first move. */
void bench_line_moving (void)
{
    if (!g_line.b_moving)
    {
        /* 1..255 then 1 again, as vehicle_request() numbers primitives. */
        g_drive.request.command_id = (uint8_t)
            (((uint32_t)g_drive.request.command_id % BENCH_ID_LAST) + 1U);
        g_line.b_moving = true;
    }
}

/** @brief Line test: note the end, as bench_move_tick() notes completion. */
void bench_line_end (uint64_t now_us, const char * p_reason)
{
    uint32_t wheel;

    g_line.p_reason = p_reason;
    g_drive.b_done = true;
    g_drive.done_s = (double)(now_us - g_drive.step_us) / 1000000.0;
    g_drive.step_us = now_us;
    g_drive.stop_yaw = g_drive.yaw;
    for (wheel = 0U; wheel < BENCH_WHEELS; ++wheel)
    {
        g_drive.stop[wheel] = g_distance[wheel] - g_drive.start[wheel];
    }
}

/** @brief Line test: gaps, the 100 ms trace and the sample that ended it. */
void bench_line_trace (void)
{
    uint32_t row;

    bench_text(" junctions crossed straight on ");
    bench_dec((int32_t)g_line.junctions);
    bench_flush();
    if (bench_is_centre(g_drive.mode))
    {
        bench_text(" trace t ms, d centre mm, IR raw, s centring steer q7"
                   " (+ right), line, h heading deg (+ left), c phase");
        bench_flush();
        bench_text(" w measured wheel speed mm/s L R, u applied duty per"
                   " mille L R (all line tests)");
    }
    else
    {
        bench_text(" gaps crossed on the last command (TRACKING to"
                   " SEARCHING) ");
        bench_dec((int32_t)g_line.gaps);
        bench_text("; trace t ms, d centre mm, IR raw, s steer q7 (+ line"
                   " right), line, h heading deg (+ left)");
    }
    bench_flush();
    if (BENCH_MODE_LINE_STOP == g_drive.mode)
    {
        bench_text(" f front sonar mm (none: no VALID range yet or the"
                   " latest not VALID)");
        bench_flush();
    }
    for (row = 0U; row < g_line.rows; ++row)
    {
        bench_line_print(" ", row * ((bench_is_centre(g_drive.mode) ?
                                      BENCH_CENTRE_TRACE_US :
                                      BENCH_LINE_TRACE_US) / 1000U),
                         &g_line.row[row]);
    }
    bench_line_print(" end ", g_line.end_ms, &g_line.end);
}

/** @brief One line-test sample. */
void bench_line_print (const char * p_label, uint32_t t_ms,
                              const bench_line_row_t * p_row)
{
    bench_text(p_label);
    bench_dec((int32_t)t_ms);
    bench_text(" d ");
    bench_signed((int32_t)p_row->centre_mm);
    bench_text(" L ");
    bench_dec((int32_t)p_row->raw[0]);
    bench_text(" R ");
    bench_dec((int32_t)p_row->raw[1]);
    bench_text(" B ");
    bench_dec((int32_t)p_row->raw[2]);
    bench_text(" s ");
    bench_signed((int32_t)p_row->steering_q7);
    bench_text(" ");
    bench_text(g_line_names[(uint32_t)p_row->status % BENCH_LINE_STATES]);
    bench_text(" h ");
    bench_fixed((int32_t)p_row->heading_ddeg, 1U);
    if (bench_is_centre(g_drive.mode))
    {
        bench_text(" c ");
        bench_text(g_centre_names[(uint32_t)p_row->phase %
                                  BENCH_CENTRE_PHASES]);
    }
    bench_text(" w ");
    bench_signed((int32_t)p_row->wheel_mms[BENCH_LEFT]);
    bench_text(" ");
    bench_signed((int32_t)p_row->wheel_mms[BENCH_RIGHT]);
    bench_text(" u ");
    bench_signed((int32_t)p_row->duty_pm[BENCH_LEFT]);
    bench_text(" ");
    bench_signed((int32_t)p_row->duty_pm[BENCH_RIGHT]);
    if (BENCH_MODE_LINE_STOP == g_drive.mode)
    {
        bench_text(" f ");
        if (0U == p_row->front_mm)
        {
            bench_text("none");
        }
        else
        {
            bench_dec((int32_t)p_row->front_mm);
        }
    }
    bench_flush();
}

/** @brief LINE centre, one sample: steering -1..1, + toward the right as
 * line_ctrl's, from the calibrated darkness d0 (left) and d1 (right).
 * - A sensor sees the line: BENCH_CENTRE_KP * (d1 - d0). It is small while
 *   the line only touches a sensor's edge, so the car corrects gently and
 *   runs straight with the line centred, where FOLLOW swings between full
 *   left and full right. In a curve the line moves further under the
 *   inner sensor until the steering matches the curve.
 * - Both white: the line lies between the sensors. Straight on for
 *   BENCH_CENTRE_GAP_M, then BENCH_CENTRE_SEARCH toward the side it was
 *   last seen (it may have left a sensor on the outside in a tight curve).
 * - line_ctrl's JUNCTION or both at least BENCH_CENTRE_CROSS: a crossing
 *   line (bench_line_cross drives over it). Straight on until
 *   BENCH_CENTRE_CLEAR_M past it, keeping the side from before it.
 * *p_lost after BENCH_CENTRE_LOST_US of searching.
 * A kinematic simulation (sensors 18-45 mm apart and 60 or 120 mm ahead of
 * the axle, 4-10 mm spots, an 18 mm line, 17-20 cm curves, 120 and
 * 150 mm/s) kept this law on the line in 62 of 64 cases against 24 for
 * FOLLOW's, at 76% of the set speed against 60%, at full steering 11% of
 * the time against 51%. Holding full steering once a sensor had been fully
 * dark was no more robust at 120 mm/s and bang-bang again.
 */
double bench_centre_steer (const line_ctrl_output_t * p_output,
                                  int32_t centre, uint64_t now_us,
                                  bool * p_lost)
{
    double  metres_per_count = (2.0 * CORE_PI * ROBOT_WHEEL_RADIUS_M) /
                               (double)ROBOT_ENCODER_COUNTS_PER_REV;
    double  left = (double)p_output->darkness_q12[0] /
                   (double)LINE_CTRL_DARKNESS_ONE;
    double  right = (double)p_output->darkness_q12[1] /
                    (double)LINE_CTRL_DARKNESS_ONE;
    double  error = right - left;
    double  lighter = (left < right) ? left : right;
    double  darker = (left < right) ? right : left;
    double  travel = fabs((double)(centre - g_centre.last_counts)) *
                     metres_per_count;
    double  steering = 0.0;
    double  step = 0.0;
    bool    b_timed = false;
    bool    b_outward = false;
    bool    b_straight = true;
    const bench_smooth_t * p_smooth = bench_smooth_set(g_drive.mode);
    bool    b_edge = (0 != BENCH_EDGE_FOLLOW) && (NULL != p_smooth);
    double  raw_left = left;
    double  raw_right = right;
    uint32_t channel;

    *p_lost = false;
    gb_rec_keep = false;
    g_centre.last_counts = centre;
    if (NULL != p_smooth)
    {
        g_centre.path_m += travel;
        g_centre.raw[0] = raw_left;
        g_centre.raw[1] = raw_right;
        g_centre.side_dark = (double)p_output->darkness_q12[2] /
                             (double)LINE_CTRL_DARKNESS_ONE;
        for (channel = 0U; channel < 2U; ++channel)
        {
            if (((0U == channel) ? raw_left : raw_right) >= BENCH_EDGE_CROSS)
            {
                g_centre.edge_m[channel] += travel;
            }
            else
            {
                g_centre.edge_m[channel] = 0.0;
            }
        }
        /* Barcode bars are too narrow to count (bench_smooth_gate). */
        left = bench_smooth_gate(0U, left, travel);
        right = bench_smooth_gate(1U, right, travel);
        error = right - left;
        lighter = (left < right) ? left : right;
        darker = (left < right) ? right : left;
        g_centre.turn += (g_centre.cmd - g_centre.turn) *
                         fmin(1.0, travel / BENCH_SMOOTH_TURN_M);
        /* A junction: both sensors dark (BENCH_CROSS_MIN_M) while the car
         * is going straight (BENCH_STRAIGHT_*); turning, never.
         */
        b_straight = bench_centre_straight();
        g_centre.b_straight = b_straight;
        g_centre.b_both_dark = b_edge ?
            ((g_centre.edge_m[0] >= BENCH_CROSS_MIN_M) &&
             (g_centre.edge_m[1] >= BENCH_CROSS_MIN_M) && b_straight) :
            ((g_centre.dark_m[0] >= BENCH_CROSS_MIN_M) &&
             (g_centre.dark_m[1] >= BENCH_CROSS_MIN_M) && b_straight);
        if (b_edge)
        {
            /* LINE edge steers and finds the line on the raw darkness. */
            error = raw_right - raw_left;
            lighter = fmin(raw_left, raw_right);
            darker = fmax(raw_left, raw_right);
        }
        if ((g_centre.dark_m[0] > 0.0) && (g_centre.dark_m[1] > 0.0) &&
            (((double)p_output->darkness_q12[2] /
              (double)LINE_CTRL_DARKNESS_ONE) >= BENCH_SMOOTH_DARK))
        {
            g_centre.all_dark_m += travel;
        }
        else
        {
            g_centre.all_dark_m = 0.0;
        }
    }
    if ((NULL != p_smooth) ? g_centre.b_both_dark :
        ((LINE_CTRL_JUNCTION == p_output->status) ||
         (lighter >= BENCH_CENTRE_CROSS)))
    {
        if (!g_centre.b_crossing)
        {
            ++g_centre.crossings;
        }
        g_centre.seen_side = 0;
        g_centre.mag = 0.0;
        g_centre.b_crossing = true;
        g_centre.clear_m = 0.0;
        g_centre.gap_m = 0.0;
        g_centre.b_searching = false;
        g_centre.b_seen = true;
        g_centre.phase = (uint8_t)BENCH_PHASE_CROSSING;
    }
    else if (g_centre.b_crossing && (g_centre.clear_m < BENCH_CENTRE_CLEAR_M))
    {
        g_centre.clear_m += travel;
        g_centre.phase = (uint8_t)BENCH_PHASE_CROSSING;
    }
    else if (darker >= BENCH_CENTRE_SEEN)
    {
        g_centre.b_crossing = false;
        g_centre.b_seen = true;
        g_centre.b_searching = false;
        g_centre.gap_m = 0.0;
        if (fabs(error) >= BENCH_CENTRE_SIDE)
        {
            g_centre.side = (error > 0.0) ? 1 : -1;
        }
        if (b_edge)
        {
            /* LINE edge: the left sensor on the line's left edge. seen_side
             * and mag as bench_smooth_track keeps them, for the card hold
             * (it keeps this correction) and the lost-outward test.
             */
            steering = fmax(-1.0, fmin(1.0, BENCH_EDGE_KP *
                                            (BENCH_EDGE_TARGET - raw_left)));
            g_centre.seen_side = (steering >= 0.0) ? 1 : -1;
            g_centre.mag = fabs(steering);
        }
        else
        {
            steering = (NULL != p_smooth) ?
                       bench_smooth_track(p_smooth, error, travel) :
                       (BENCH_CENTRE_KP * error);
        }
        g_centre.phase = (uint8_t)BENCH_PHASE_TRACK;
    }
    else
    {
        g_centre.b_crossing = false;
        if (g_centre.b_seen)
        {
            ++g_centre.gaps;
        }
        g_centre.b_seen = false;
        g_centre.gap_m += travel;
        /* LINE smooth: lost while steering hard toward the line, or in a
         * sustained bend (BENCH_SMOOTH_TURN_M), it went out past that sensor;
         * search at once, not straight on.
         */
        b_outward = (NULL != p_smooth) &&
                    ((g_centre.mag >= BENCH_SMOOTH_OUTWARD) ||
                     (fabs(g_centre.turn) >= BENCH_SMOOTH_BEND));
        g_centre.seen_side = 0;
        if (NULL != p_smooth)
        {
            /* Back toward the side last seen, growing with travel
             * (BENCH_SMOOTH_SEARCH_PER_M); all of it when lost outward.
             */
            steering = b_outward ? 1.0 :
                       fmin(1.0, g_centre.gap_m * BENCH_SMOOTH_SEARCH_PER_M);
            b_timed = steering >= 0.5;
            g_centre.phase = (uint8_t)(b_timed ? BENCH_PHASE_SEARCH :
                                                 BENCH_PHASE_GAP);
            steering *= (double)g_centre.side;
        }
        else if ((g_centre.gap_m < BENCH_CENTRE_GAP_M) && (!b_outward))
        {
            /* Straight on. Holding the turn here instead, with other
             * changes, steered at full lock 59% of the time against 20%
             * (29 Oct).
             */
            g_centre.phase = (uint8_t)BENCH_PHASE_GAP;
        }
        else
        {
            steering = (double)g_centre.side * BENCH_CENTRE_SEARCH;
            g_centre.phase = (uint8_t)BENCH_PHASE_SEARCH;
            b_timed = true;
        }
    }
    if (b_timed)
    {
        if (!g_centre.b_searching)
        {
            g_centre.b_searching = true;
            g_centre.search_us = now_us;
            ++g_centre.searches;
        }
        if ((uint32_t)((now_us - g_centre.search_us) / 1000U) >
            g_centre.search_max_ms)
        {
            g_centre.search_max_ms =
                (uint32_t)((now_us - g_centre.search_us) / 1000U);
        }
        *p_lost = (now_us - g_centre.search_us) >=
                  ((NULL != p_smooth) ? BENCH_SMOOTH_LOST_US :
                                        BENCH_CENTRE_LOST_US);
    }
    steering = fmax(-1.0, fmin(1.0, steering));
    if (NULL != p_smooth)
    {
        /* At most p_smooth->slew per second: no full-left to full-right
         * jumps. The first sample (last_us 0) takes the steering as it is.
         */
        step = p_smooth->slew *
               ((double)(now_us - g_centre.last_us) / 1000000.0);
        g_centre.cmd += fmax(-step, fmin(step, steering - g_centre.cmd));
        steering = g_centre.cmd;
    }
    g_centre.last_us = now_us;
    ++g_centre.samples;
    ++g_centre.in_phase[g_centre.phase % BENCH_CENTRE_PHASES];
    if ((fabs(steering) * ROBOT_LINE_GAIN_RPS) >= ROBOT_TURN_RATE_RPS)
    {
        ++g_centre.full_steer;
    }
    if (BENCH_MODE_MISSION == g_drive.mode)
    {
        bench_rec_sample(now_us, raw_left, raw_right, left, right,
                         b_straight);
    }
    return steering;
}

/** @brief LINE smooth: a sensor's darkness, or 0 until it has been at least
 * BENCH_SMOOTH_DARK for BENCH_SMOOTH_DARK_M of travel; dark_m also decides
 * crossings (BENCH_CROSS_MIN_M).
 */
double bench_smooth_gate (uint32_t channel, double darkness,
                                 double travel)
{
    if (darkness >= BENCH_SMOOTH_DARK)
    {
        g_centre.dark_m[channel] += travel;
    }
    else
    {
        g_centre.dark_m[channel] = 0.0;
    }
    return (g_centre.dark_m[channel] >= BENCH_SMOOTH_DARK_M) ? darkness : 0.0;
}

/** @brief Going straight: the wheels' heading, sampled every
 * BENCH_STRAIGHT_STEP_M of travel, turned less than BENCH_STRAIGHT_RAD since
 * the oldest of the last BENCH_STRAIGHT_SLOTS samples (the first sample
 * while fewer are in). True before any.
 */
bool bench_centre_straight (void)
{
    uint32_t oldest = 0U;

    while ((g_centre.path_m - g_centre.ring_at_m) >= BENCH_STRAIGHT_STEP_M)
    {
        g_centre.yaw_ring[g_centre.ring_head] = g_drive.yaw;
        g_centre.ring_head = (g_centre.ring_head + 1U) % BENCH_STRAIGHT_SLOTS;
        g_centre.ring_at_m += BENCH_STRAIGHT_STEP_M;
        if (g_centre.ring_n < BENCH_STRAIGHT_SLOTS)
        {
            ++g_centre.ring_n;
        }
    }
    if (0U == g_centre.ring_n)
    {
        return true;
    }
    oldest = (g_centre.ring_n < BENCH_STRAIGHT_SLOTS) ? 0U :
             g_centre.ring_head;
    return fabs(bench_avoid_wrap(g_drive.yaw - g_centre.yaw_ring[oldest])) <
           BENCH_STRAIGHT_RAD;
}

/** @brief Line test: both front sensors see a line (a junction). LINE smooth
 * takes its bar-proof view (bench_smooth_gate), the others line_ctrl's.
 */
bool bench_line_black (const line_ctrl_output_t * p_output)
{
    return (NULL != bench_smooth_set(g_drive.mode)) ? g_centre.b_both_dark :
           (LINE_CTRL_JUNCTION == p_output->status);
}

/** @brief LINE smooth, a sensor sees the line: steering toward it starts at
 * p_set->start and grows by p_set->ramp per metre of travel while that side
 * keeps seeing it, so a touch on a straight is a small correction and a bend,
 * which keeps the line under the inner sensor, gets more. Per metre, not per
 * second, so the car takes the same path at any speed: per second, at
 * 180 mm/s it needed 100 mm to reach full steering and ran wide off a bend
 * (29 Oct).
 * @return steering -1..1, + toward the right.
 */
double bench_smooth_track (const bench_smooth_t * p_set, double error,
                                  double travel)
{
    int32_t side = (error > 0.0) ? 1 : -1;

    if ((fabs(error) < BENCH_CENTRE_SIDE) && (0 != g_centre.seen_side) &&
        (fabs(g_centre.turn) >= BENCH_SMOOTH_BEND))
    {
        /* Both about as dark mid-turn: the line runs under both in the
         * bend; keep turning the same way.
         */
        side = g_centre.seen_side;
        gb_rec_keep = true;
    }
    else if (fabs(error) < BENCH_CENTRE_SIDE)
    {
        /* Both about as dark: centred on the line. */
        g_centre.seen_side = 0;
        g_centre.mag = 0.0;
        return 0.0;
    }
    else
    {
        /* One side darker: steer toward it. */
    }
    if (side != g_centre.seen_side)
    {
        g_centre.seen_side = side;
        g_centre.on_m = 0.0;
    }
    else
    {
        g_centre.on_m += travel;
    }
    g_centre.mag = fmin(1.0, p_set->start + (p_set->ramp * g_centre.on_m));
    return (double)side * g_centre.mag;
}

/** @brief LINE centre: mean speed, time per phase, events, darkest raw. */
void bench_centre_results (void)
{
    double   seconds = g_drive.b_done ? g_drive.done_s :
        ((double)(board_time_us() - g_drive.step_us) / 1000000.0);
    int32_t  centre = g_drive.b_done ?
        ((g_drive.stop[BENCH_LEFT] + g_drive.stop[BENCH_RIGHT]) / 2) :
        (((g_distance[BENCH_LEFT] - g_drive.start[BENCH_LEFT]) +
          (g_distance[BENCH_RIGHT] - g_drive.start[BENCH_RIGHT])) / 2);
    uint32_t samples = (0U == g_centre.samples) ? 1U : g_centre.samples;
    uint32_t phase;

    /* Frames: path +-1048575 mm (21 bits), time 0-6553.5 s, mean speed
     * +-511 mm/s, shares 0-100 % (7 bits), run counts 0-1023, the longest
     * search 0-65535 ms, ADC 0-4095.
     */
    bench_line(BK_CENTRE);
    bench_text(" centre: ");
    bench_i(bench_mm(centre), 21U);
    bench_text(" mm in ");
    bench_ux((uint32_t)bench_round(seconds * 10.0), 1U, 16U);
    bench_text(" s, mean ");
    bench_i((seconds > 0.0) ?
            bench_round((double)bench_mm(centre) / seconds) : 0, 10U);
    bench_text(" mm/s; time % per phase");
    for (phase = 0U; phase < BENCH_CENTRE_PHASES; ++phase)
    {
        bench_text(" ");
        bench_text(g_centre_names[phase]);
        bench_text(" ");
        bench_u((uint32_t)bench_round((100.0 *
                                       (double)g_centre.in_phase[phase]) /
                                      (double)samples), 7U);
    }
    bench_text(", full steer ");
    bench_u((uint32_t)bench_round((100.0 * (double)g_centre.full_steer) /
                                  (double)samples), 7U);
    bench_flush();
    bench_line(BK_CENTRE_GAPS);
    bench_text(" gaps ");
    bench_u(g_centre.gaps, 16U);   /* ~10/s in the sim: 36000 in 1 h */
    bench_text(", crossings ");
    bench_u(g_centre.crossings, 10U);
    bench_text(", searches ");
    bench_u(g_centre.searches, 12U);   /* 43 in 2 min on the car */
    bench_text(" (longest ");
    bench_u(g_centre.search_max_ms, 16U);
    bench_text(" ms), darkest raw L ");
    bench_u((uint32_t)g_centre.raw_max[0], 12U);
    bench_text(" R ");
    bench_u((uint32_t)g_centre.raw_max[1], 12U);
    bench_flush();
}

/** @brief True for MISSION on the straight barcode image
 * (BENCH_STRAIGHT_CODE): straight on, no line, no sonar, until STOP.
 */
bool bench_straight_code (bench_mode_t mode)
{
    return (0 != BENCH_STRAIGHT_CODE) && (BENCH_MODE_MISSION == mode);
}

/** @brief AUTO-CENTRE: armed for the selected test (bench_line_start). */
void bench_auto_start (void)
{
    (void)memset(&g_auto, 0, sizeof(g_auto));
    g_auto.b_active = bench_has_auto(g_mode);
}

/** @brief AUTO-CENTRE, one sample: pivot on the spot until each front
 * sensor finds the line, then to the heading midway between, then hand
 * over to the line test (AUTO-CENTRE alone ends there).
 * - A sensor counts only while it turns toward the line from between the
 *   sensors: the right one while the car turns left, the left one while it
 *   turns right. Each then finds the line's near edge, and a sample late,
 *   by the same amount on both sides, so the middle heading puts the line
 *   midway between the sensors however far ahead of the axle they sit and
 *   however wide apart. With the axle off the line, the car's heading then
 *   differs from the line's a little; the line test corrects that.
 * - It turns left first, or right if the right sensor already sees the
 *   line, and after a find turns back for the other sensor. A sweep that
 *   reaches BENCH_AUTO_SWEEP_RAD from the START heading with nothing found
 *   turns back once.
 * - It halts between pivots and measures how far the car runs on; it halts
 *   that far (at least BENCH_AUTO_LEAD_RAD) short of the middle, settles,
 *   and corrects up to BENCH_AUTO_CORRECTIONS times while more than
 *   BENCH_AUTO_TOLERANCE_RAD off.
 * - It fails, halted, if both sensors see a line at START, a sweep reaches
 *   its limit a second time or after a find, the finds are more than
 *   BENCH_AUTO_SPAN_RAD apart, a pivot stalls, or BENCH_AUTO_MAX_US passes.
 */
void bench_auto_step (uint64_t now_us,
                             const line_ctrl_output_t * p_output,
                             motion_request_t * p_request)
{
    bool b_left = ((double)p_output->darkness_q12[0] /
                   (double)LINE_CTRL_DARKNESS_ONE) >= BENCH_SMOOTH_DARK;
    bool b_right = ((double)p_output->darkness_q12[1] /
                    (double)LINE_CTRL_DARKNESS_ONE) >= BENCH_SMOOTH_DARK;

    if (0U == g_auto.start_us)
    {
        /* The first sample. */
        g_auto.start_us = now_us;
        g_auto.start_rad = g_drive.yaw;
        if (b_left && b_right)
        {
            bench_auto_fail(now_us, "both front sensors see a line at START"
                            " (a crossing?): start on a plain stretch");
            return;
        }
        g_auto.turn = b_right ? -1 : 1;
        bench_auto_phase(now_us, BENCH_AUTO_SEEK);
    }
    if ((now_us - g_auto.start_us) >= BENCH_AUTO_MAX_US)
    {
        bench_auto_fail(now_us, "not centred within 30 s");
        return;
    }
    if ((BENCH_AUTO_SEEK == g_auto.phase) ||
        (BENCH_AUTO_MIDDLE == g_auto.phase))
    {
        if (fabs(g_drive.yaw - g_auto.mark_rad) >= BENCH_AUTO_STALL_RAD)
        {
            g_auto.mark_rad = g_drive.yaw;
            g_auto.mark_us = now_us;
        }
        else if ((now_us - g_auto.mark_us) >= BENCH_AUTO_STALL_US)
        {
            bench_auto_fail(now_us, "pivot stalled: under 1 deg in 1.5 s"
                            " (wheels stuck or slipping)");
            return;
        }
        else
        {
            /* Turning, or still starting to. */
        }
    }
    switch ((bench_auto_phase_t)g_auto.phase)
    {
        case BENCH_AUTO_SEEK:
            bench_auto_seek(now_us, p_output);
        break;
        case BENCH_AUTO_PAUSE:
            if ((now_us - g_auto.phase_us) < BENCH_AUTO_PAUSE_US)
            {
                break;
            }
            /* How far the last pivot ran on after its halt. */
            g_auto.coast_rad = fabs(g_drive.yaw - g_auto.mark_rad);
            if (BENCH_AUTO_MIDDLE == g_auto.next)
            {
                g_auto.turn = (g_auto.middle_rad >= g_drive.yaw) ? 1 : -1;
            }
            bench_auto_phase(now_us, (bench_auto_phase_t)g_auto.next);
        break;
        case BENCH_AUTO_MIDDLE:
            bench_auto_middle(now_us, p_request);
        break;
        default:
            bench_auto_settle(now_us);
        break;
    }
    if (g_auto.b_active && (BENCH_AUTO_SEEK == g_auto.phase))
    {
        /* Pivot on the spot; + yaw turns left. */
        g_drive.request.kind = MOTION_VELOCITY;
        g_drive.request.forward_mps = 0.0;
        g_drive.request.yaw_rps = (double)g_auto.turn * BENCH_AUTO_YAW_RPS;
        *p_request = g_drive.request;
    }
}

/** @brief AUTO-CENTRE, pivoting: the sought sensor (the right one turning
 * left, the left one turning right) finds the line when it goes dark after
 * being white in this sweep and stays dark for BENCH_AUTO_CONFIRM_RAD; the
 * heading where it went dark is kept. The sweep limit turns it back once.
 */
void bench_auto_seek (uint64_t now_us,
                             const line_ctrl_output_t * p_output)
{
    uint32_t channel = (g_auto.turn > 0) ? 1U : 0U;
    double   darkness = (double)p_output->darkness_q12[channel] /
                        (double)LINE_CTRL_DARKNESS_ONE;
    double   turned = (double)g_auto.turn * (g_drive.yaw - g_auto.start_rad);

    if (darkness < BENCH_AUTO_WHITE)
    {
        g_auto.b_white = true;
    }
    if (g_auto.b_white && (darkness >= BENCH_SMOOTH_DARK))
    {
        if (!g_auto.b_candidate)
        {
            g_auto.b_candidate = true;
            g_auto.candidate_rad = g_drive.yaw;
        }
        else if (fabs(g_drive.yaw - g_auto.candidate_rad) >=
                 BENCH_AUTO_CONFIRM_RAD)
        {
            g_auto.b_found[channel] = true;
            g_auto.found_rad[channel] = g_auto.candidate_rad;
            bench_auto_found(now_us);
            return;
        }
        else
        {
            /* Dark, not yet for BENCH_AUTO_CONFIRM_RAD. */
        }
    }
    else
    {
        g_auto.b_candidate = false;
    }
    if (turned < BENCH_AUTO_SWEEP_RAD)
    {
        return;
    }
    if (g_auto.b_found[0] || g_auto.b_found[1])
    {
        bench_auto_fail(now_us, (1U == channel) ?
                        "the right sensor found no line turning left"
                        " (70 deg limit)" :
                        "the left sensor found no line turning right"
                        " (70 deg limit)");
    }
    else if (g_auto.b_limit)
    {
        bench_auto_fail(now_us, "no line within 70 deg either side of the"
                        " START heading");
    }
    else
    {
        g_auto.b_limit = true;
        g_auto.turn = -g_auto.turn;
        g_auto.next = (uint8_t)BENCH_AUTO_SEEK;
        bench_auto_phase(now_us, BENCH_AUTO_PAUSE);
    }
}

/** @brief AUTO-CENTRE, a sensor found the line: turn back for the other, or
 * with both found, check them and go to the middle.
 */
void bench_auto_found (uint64_t now_us)
{
    if (!(g_auto.b_found[0] && g_auto.b_found[1]))
    {
        g_auto.turn = -g_auto.turn;
        g_auto.next = (uint8_t)BENCH_AUTO_SEEK;
        bench_auto_phase(now_us, BENCH_AUTO_PAUSE);
        return;
    }
    if (fabs(g_auto.found_rad[1] - g_auto.found_rad[0]) > BENCH_AUTO_SPAN_RAD)
    {
        bench_auto_fail(now_us, "the sensors found the line over 50 deg"
                        " apart: not one straight line (bend, crossing,"
                        " bars?)");
        return;
    }
    g_auto.middle_rad = 0.5 * (g_auto.found_rad[0] + g_auto.found_rad[1]);
    g_auto.next = (uint8_t)BENCH_AUTO_MIDDLE;
    bench_auto_phase(now_us, BENCH_AUTO_PAUSE);
}

/** @brief AUTO-CENTRE, pivoting to the middle: halt short of it by the
 * last measured run-on (at least BENCH_AUTO_LEAD_RAD), or at once if past it.
 */
void bench_auto_middle (uint64_t now_us, motion_request_t * p_request)
{
    if (((double)g_auto.turn * (g_auto.middle_rad - g_drive.yaw)) <=
        fmax(BENCH_AUTO_LEAD_RAD, g_auto.coast_rad))
    {
        bench_auto_phase(now_us, BENCH_AUTO_SETTLE);
        return;
    }
    g_drive.request.kind = MOTION_VELOCITY;
    g_drive.request.forward_mps = 0.0;
    g_drive.request.yaw_rps = (double)g_auto.turn * BENCH_AUTO_YAW_RPS;
    *p_request = g_drive.request;
}

/** @brief AUTO-CENTRE, halted: after BENCH_AUTO_SETTLE_US and both wheels
 * still, correct if still off the middle, else hand over to the line test
 * from rest with a fresh line_ctrl, its distance and time from here. The
 * line then lies between the sensors, and LINE smooth knows its side only
 * once a sensor has seen it: on a straight it would search straight on and
 * call the line lost (host simulation). It is given the side the line is
 * nearer, from the heading left over against the middle.
 */
void bench_auto_settle (uint64_t now_us)
{
    double   error = g_auto.middle_rad - g_drive.yaw;
    uint32_t wheel;

    if ((now_us - g_auto.phase_us) < BENCH_AUTO_SETTLE_US)
    {
        return;
    }
    for (wheel = 0U; wheel < BENCH_WHEELS; ++wheel)
    {
        if ((g_line.wheel_mms[wheel] > BENCH_AUTO_STILL_MMS) ||
            (g_line.wheel_mms[wheel] < -BENCH_AUTO_STILL_MMS))
        {
            return;
        }
    }
    if ((fabs(error) > BENCH_AUTO_TOLERANCE_RAD) &&
        (g_auto.corrections < BENCH_AUTO_CORRECTIONS))
    {
        ++g_auto.corrections;
        g_auto.turn = (error >= 0.0) ? 1 : -1;
        bench_auto_phase(now_us, BENCH_AUTO_MIDDLE);
        return;
    }
    g_auto.b_active = false;
    g_auto.done_ms = (uint32_t)((now_us - g_auto.start_us) / 1000U);
    g_auto.end_rad = g_drive.yaw;
    if (BENCH_MODE_AUTO == g_drive.mode)
    {
        bench_line_end(now_us, "centred");
        return;
    }
    line_ctrl_init(&g_line_state);
    g_drive.step_us = now_us;
    for (wheel = 0U; wheel < BENCH_WHEELS; ++wheel)
    {
        g_drive.start[wheel] = g_distance[wheel];
    }
    /* Turned left of the middle: the line is nearer the right sensor. */
    g_centre.side = (g_drive.yaw > g_auto.middle_rad) ? 1 : -1;
}

/** @brief AUTO-CENTRE: enter a step. Each pivot is a fresh primitive: after
 * a halt the controller refuses the old ID.
 */
void bench_auto_phase (uint64_t now_us, bench_auto_phase_t phase)
{
    g_auto.phase = (uint8_t)phase;
    g_auto.phase_us = now_us;
    g_auto.mark_rad = g_drive.yaw;
    g_auto.mark_us = now_us;
    g_auto.b_white = false;
    g_auto.b_candidate = false;
    if (BENCH_AUTO_SEEK == phase)
    {
        ++g_auto.sweeps;
    }
    if ((BENCH_AUTO_SEEK == phase) || (BENCH_AUTO_MIDDLE == phase))
    {
        g_drive.request.command_id = (uint8_t)
            (((uint32_t)g_drive.request.command_id % BENCH_ID_LAST) + 1U);
    }
}

/** @brief AUTO-CENTRE failed: halt and end the test with the reason. */
void bench_auto_fail (uint64_t now_us, const char * p_reason)
{
    g_auto.b_active = false;
    g_auto.b_failed = true;
    g_auto.p_fail = p_reason;
    g_auto.done_ms = (0U == g_auto.start_us) ? 0U :
                     (uint32_t)((now_us - g_auto.start_us) / 1000U);
    g_auto.end_rad = g_drive.yaw;
    bench_line_end(now_us, "stopped: AUTO-CENTRE failed");
}

/** @brief AUTO-CENTRE: what it found and where it left the car; headings
 * from the START heading, + left.
 */
void bench_auto_results (void)
{
    double end_rad = g_auto.end_rad;

    if (g_auto.b_active)
    {
        /* STOP during the centring. */
        end_rad = g_drive.yaw;
        g_auto.done_ms = (0U == g_auto.start_us) ? 0U :
            (uint32_t)((board_time_us() - g_auto.start_us) / 1000U);
    }
    /* Frames: the outcome 2 bits, time 0-102.3 s, counts 0-255, angles as
     * bench_deg, why g_auto_fail_names (3 bits), ADC 0-4095.
     */
    bench_line(BK_AUTO);
    bench_code(g_auto.b_failed ? 0U : g_auto.b_active ? 1U : 2U, 2U);
    bench_text(g_auto.b_failed ? " AUTO-CENTRE FAILED after " :
               g_auto.b_active ? " AUTO-CENTRE stopped after " :
                                 " AUTO-CENTRE done in ");
    bench_ux((uint32_t)bench_round((double)g_auto.done_ms / 100.0), 1U, 10U);
    bench_text(" s, sweeps ");
    bench_u(g_auto.sweeps, 8U);
    bench_text(", corrections ");
    bench_u(g_auto.corrections, 8U);
    bench_text(", run-on after a halt ");
    bench_deg(g_auto.coast_rad);
    bench_text(" deg");
    bench_flush();
    if (g_auto.b_failed)
    {
        bench_line(BK_AUTO_WHY);
        bench_text("  why: ");
        bench_name_of(g_auto_fail_names, BENCH_AUTO_FAILS, g_auto.p_fail, 3U);
        bench_flush();
    }
    bench_line(BK_AUTO_FOUND);
    bench_text("  right sensor found the line turning left at ");
    if (bench_flag(g_auto.b_found[1]))
    {
        bench_deg(g_auto.found_rad[1] - g_auto.start_rad);
        bench_text(" deg");
    }
    else
    {
        bench_text("none");
    }
    bench_text(", left turning right at ");
    if (bench_flag(g_auto.b_found[0]))
    {
        bench_deg(g_auto.found_rad[0] - g_auto.start_rad);
        bench_text(" deg");
    }
    else
    {
        bench_text("none");
    }
    bench_flush();
    bench_line(BK_AUTO_END);
    if (bench_flag(g_auto.b_found[0] && g_auto.b_found[1]))
    {
        bench_text("  middle ");
        bench_deg(g_auto.middle_rad - g_auto.start_rad);
        bench_text(" deg, heading at the end ");
        bench_deg(end_rad - g_auto.start_rad);
        bench_text(" deg (off the middle ");
        bench_deg(end_rad - g_auto.middle_rad);
        bench_text(")");
    }
    else
    {
        bench_text("  heading at the end ");
        bench_deg(end_rad - g_auto.start_rad);
        bench_text(" deg");
    }
    bench_text(", front IR L ");
    bench_u((uint32_t)g_auto.raw[0], 12U);
    bench_text(" R ");
    bench_u((uint32_t)g_auto.raw[1], 12U);
    bench_flush();
}

/** @brief MISSION, a cut that made a group: its samples, one character per
 * unit (a unit without a reading as BENCH_RAW_GAP), to go out while the car
 * drives (bench_raw_text). Skipped while the last cut is still going out.
 */
void bench_raw_keep (void)
{
    const bench_code_sample_t * p_sample = &g_code.sample[0];
    uint32_t                    count = g_code.count;
    uint32_t                    length = 0U;
    uint32_t                    index;
    uint32_t                    gap;
    int32_t                     level;

    ++g_raw_cuts;
    if ((0U != g_raw_length) || (0U == count))
    {
        return;
    }
    for (index = 0U; (index < count) && (length < BENCH_RAW_CHARS); ++index)
    {
        /* Units only increase in the store (bench_code_sample). */
        gap = (0U == index) ? 1U :
              ((uint32_t)p_sample[index].unit -
               (uint32_t)p_sample[index - 1U].unit);
        while ((gap > 1U) && (length < BENCH_RAW_CHARS))
        {
            g_raw[length] = BENCH_RAW_GAP;
            ++length;
            --gap;
        }
        level = ((int32_t)p_sample[index].raw - (int32_t)g_code.cut_white +
                 BENCH_RAW_BELOW) / BENCH_RAW_STEP;
        if (level > BENCH_RAW_KNEE)
        {
            level = BENCH_RAW_KNEE +
                    ((((int32_t)p_sample[index].raw -
                       (int32_t)g_code.cut_white + BENCH_RAW_BELOW) -
                      (BENCH_RAW_KNEE * BENCH_RAW_STEP)) / BENCH_RAW_HIGH);
        }
        if (level < 0)
        {
            level = 0;
        }
        if (level > (BENCH_RAW_TOP - BENCH_RAW_ZERO))
        {
            level = BENCH_RAW_TOP - BENCH_RAW_ZERO;
        }
        if (length < BENCH_RAW_CHARS)
        {
            g_raw[length] = (char)(BENCH_RAW_ZERO + level);
            ++length;
        }
    }
    g_raw_number = g_raw_cuts;
    g_raw_first = p_sample[0].unit;
    g_raw_white = g_code.cut_white;
    g_raw_sd = g_code.cut_sd;
    g_raw_sent = 0U;
    g_raw_length = length;
    gb_raw_head = true;
}

/** @brief MISSION: the next line of the cut going out into g_text, its
 * header first.
 * @return false when no cut waits.
 */
bool bench_raw_text (void)
{
    uint32_t chunk;
    uint32_t index;

    if (0U == g_raw_length)
    {
        return false;
    }
    /* Frame: cut number 0-1023 a run; first unit 0-8388607 (23 bits, 2 km
     * of side path); units 0-BENCH_RAW_CHARS 4096 (13 bits); white and sd
     * ADC 0-4095. Rows: up to BENCH_RAW_LINE characters, each '!'..'~' as
     * 7 bits (c - '!').
     */
    if (gb_raw_head)
    {
        gb_raw_head = false;
        bench_line(BK_RAW_HEAD);
        bench_text(" RAW ");
        bench_u(g_raw_number, 10U);
        bench_text(": u");
        bench_u(g_raw_first, 23U);
        bench_text(" (side ");
        bench_code_unit_mm(g_raw_first);
        bench_text(" mm), ");
        bench_u(g_raw_length, 13U);
        bench_text(" units, white ");
        bench_u(g_raw_white, 12U);
        bench_text(" sd ");
        bench_u(g_raw_sd, 12U);
        bench_text("; raw = white - 10 + 2 (c - '#') below 'c', else white"
                   " + 118 + 140 (c - 'c'); '!' none");
        return true;
    }
    bench_line(BK_RAW_ROW);
    bench_text(" R ");
    bench_u(g_raw_number, 10U);
    bench_text(": ");
    chunk = g_raw_length - g_raw_sent;
    if (chunk > BENCH_RAW_LINE)
    {
        chunk = BENCH_RAW_LINE;
    }
    if ((g_length + chunk + 2U) > BENCH_TEXT_BYTES)
    {
        chunk = BENCH_TEXT_BYTES - 2U - g_length;
    }
    bench_code(chunk, 7U);
    for (index = 0U; index < chunk; ++index)
    {
        bench_code((uint32_t)(uint8_t)g_raw[g_raw_sent + index] -
                   (uint32_t)BENCH_RAW_GAP, 7U);
    }
    (void)memcpy(&g_text[g_length], &g_raw[g_raw_sent], (size_t)chunk);
    g_length += chunk;
    g_text[g_length] = '\0';
    g_raw_sent += chunk;
    if (g_raw_sent >= g_raw_length)
    {
        g_raw_length = 0U;
        ++g_raw_done;
    }
    return true;
}

/** @brief MISSION, each line sample: the START calibration's extremes, over
 * AUTO-CENTRE and the first BENCH_CAL_M of line.
 */
void bench_cal_sample (const uint16_t * p_raw, int32_t centre_mm)
{
    uint32_t channel;

    if (g_cal.b_done)
    {
        return;
    }
    for (channel = 0U; channel < LINE_CTRL_CHANNELS; ++channel)
    {
        if ((0U == g_cal.count) || (p_raw[channel] < g_cal.low[channel]))
        {
            g_cal.low[channel] = p_raw[channel];
        }
        if ((0U == g_cal.count) || (p_raw[channel] > g_cal.high[channel]))
        {
            g_cal.high[channel] = p_raw[channel];
        }
    }
    ++g_cal.count;
    if ((!g_auto.b_active) && (centre_mm >= bench_round(BENCH_CAL_M * 1000.0)))
    {
        g_cal.b_done = true;
        g_cal.done_mm = centre_mm;
    }
}

/** @brief MISSION live: the START calibration, once. */
bool bench_cal_text (void)
{
    static const char * const names[LINE_CTRL_CHANNELS] =
    {
        " CAL L ", ", R ", ", B "
    };
    static const uint16_t white[LINE_CTRL_CHANNELS] =
    {
        ROBOT_IR_LEFT_WHITE, ROBOT_IR_RIGHT_WHITE, ROBOT_IR_BARCODE_WHITE
    };
    static const uint16_t black[LINE_CTRL_CHANNELS] =
    {
        ROBOT_IR_LEFT_BLACK, ROBOT_IR_RIGHT_BLACK, ROBOT_IR_BARCODE_BLACK
    };
    uint32_t channel;

    if ((!g_cal.b_done) || g_cal.b_sent)
    {
        return false;
    }
    g_cal.b_sent = true;
    /* Frame: ADC 0-4095 (12 bits each), the path +-1048575 mm (21). */
    bench_line(BK_CAL);
    for (channel = 0U; channel < LINE_CTRL_CHANNELS; ++channel)
    {
        bench_text(names[channel]);
        bench_u((uint32_t)g_cal.low[channel], 12U);
        bench_text("-");
        bench_u((uint32_t)g_cal.high[channel], 12U);
        bench_text(" (set ");
        bench_u((uint32_t)white[channel], 12U);
        bench_text("-");
        bench_u((uint32_t)black[channel], 12U);
        bench_text(")");
    }
    bench_text(" over AUTO-CENTRE and ");
    bench_i(g_cal.done_mm, 21U);
    bench_text(" mm");
    return true;
}

/** @brief MISSION start: no letter is carried over from the last run; the
 * map (from MQTT) stays.
 */
void bench_nav_start (void)
{
    g_nav.letter = '\0';
    g_nav.b_say_code = false;
    g_nav.b_card = false;
    g_nav.reported = 0U;
    (void)memset(g_nav.read, 0, sizeof(g_nav.read));
    g_nav.junction_mm = INT32_MIN;
    g_nav.phase = (uint8_t)BENCH_NAV_IDLE;
    g_nav.junctions = 0U;
    g_nav.moves = 0U;
    g_nav.b_waiting = false;
    g_nav.wait_us = 0U;
    g_nav.waits = 0U;
    g_nav.b_probe = false;
    g_nav.turns = 0U;
    g_nav.guard_to_mm = INT32_MIN;
    gb_fit_force = false;
}

/** @brief Main loop: a new letter -> move map from MQTT (navmap.h, already
 * validated by the radio) replaces the old one. Idle, its line goes out at
 * once; mid-run it is queued from the line step (bench_nav_say).
 */
void bench_nav_poll_map (void)
{
    uint8_t bytes[NAVMAP_LENGTH];
    char    letters[NAVMAP_MOVES];

    /* The robot app's map queue always exists (the host sim fills it). */
    if ((gb_mqtt || (0 != MISSION_ROBOT)) &&
        net_mqtt_ipc_receive_map(bytes) && navmap_unpack(bytes, letters))
    {
        (void)memcpy(g_nav.letters, letters, sizeof(g_nav.letters));
        g_nav.b_mapped = true;
        g_nav.b_say_map = true;
        bench_nav_backup(bytes);
    }
    if (g_nav.b_say_map && (!g_drive.b_running) && (!g_sweep.b_running) &&
        (!g_scan.b_running) && (0U == g_length))
    {
        g_nav.b_say_map = false;
        bench_nav_say_map(BK_MAP);
        bench_flush();
    }
}

/** @brief The map as one line of text (not flushed). */
void bench_nav_say_map (bench_kind_t kind)
{
    static const char * const p_names[NAVMAP_MOVES] =
    {
        "forward ", ", U-turn ", ", left ", ", right "
    };
    uint32_t move;

    /* Frame: each letter A-Z as 5 bits, as navmap.h sends them. */
    bench_line(kind);
    bench_text("MAP set:");
    for (move = 0U; move < (uint32_t)NAVMAP_MOVES; ++move)
    {
        bench_text((0U == move) ? " " : "");
        bench_text(p_names[move]);
        bench_char(g_nav.letters[move], 'A', 5U);
    }
}

/* 6 Nov: on the car the map in g_nav was found cleared after START (MAP set
 * at 15:41:04, "no map yet" at 15:41:19, no reboot; not seen in the host sim
 * under ASan, so something MQTT-only writes over it). The map's 3 bytes are
 * also kept here with their bitwise inverse; bench_nav_map_check restores
 * g_nav from them and logs what it found, to find the writer.
 */
static uint8_t g_nav_backup[NAVMAP_LENGTH];
static uint8_t g_nav_backup_inv[NAVMAP_LENGTH];
static uint8_t g_nav_seen[8];
      /* g_nav's first bytes when restored */
static bool    gb_nav_restored;
    /* a restore is still to be said */
static uint32_t g_nav_restores;

/** @brief Keep a checked copy of a received map. */
void bench_nav_backup (const uint8_t * p_bytes)
{
    uint32_t index;

    for (index = 0U; index < NAVMAP_LENGTH; ++index)
    {
        g_nav_backup[index] = p_bytes[index];
        g_nav_backup_inv[index] = (uint8_t)(~p_bytes[index]);
    }
}

/** @brief True if g_nav holds a valid map; if it does not but the backup
 * checks out, g_nav is restored from it first (and the restore said).
 */
static bool bench_nav_map_check (void)
{
    uint8_t  bytes[NAVMAP_LENGTH];
    char     letters[NAVMAP_MOVES];
    uint32_t index;
    bool     b_backup = true;

    if (g_nav.b_mapped && navmap_pack(g_nav.letters, bytes))
    {
        return true;
    }
    for (index = 0U; index < NAVMAP_LENGTH; ++index)
    {
        b_backup = b_backup && (0xFFU == (uint8_t)(g_nav_backup[index] ^
                                                   g_nav_backup_inv[index]));
    }
    if ((!b_backup) || (!navmap_unpack(g_nav_backup, letters)))
    {
        return false;
    }
    (void)memcpy(g_nav_seen, &g_nav, sizeof(g_nav_seen));
    (void)memcpy(g_nav.letters, letters, sizeof(g_nav.letters));
    g_nav.b_mapped = true;
    gb_nav_restored = true;
    ++g_nav_restores;
    return true;
}

/** @brief A byte as two hex digits. */
static void bench_nav_hex (uint8_t value)
{
    static const char digits[] = "0123456789abcdef";
    char text[3];

    text[0] = digits[(value >> 4) & 0x0FU];
    text[1] = digits[value & 0x0FU];
    text[2] = '\0';
    bench_code((uint32_t)value, 8U);   /* frame: the byte, 8 bits */
    bench_text(text);
}

/** @brief The move mapped to letter, or -1 (no map, or not in it). */
int32_t bench_nav_move_of (char letter)
{
    uint32_t move;

    if (!bench_nav_map_check())
    {
        return -1;
    }
    for (move = 0U; move < (uint32_t)NAVMAP_MOVES; ++move)
    {
        if (g_nav.letters[move] == letter)
        {
            return (int32_t)move;
        }
    }
    return -1;
}

/** @brief A confirmed Decoder 1 code (bench_code_tries_step), at once: its
 * CODE line, which joins the vote, may go out later.
 */
void bench_nav_code (const bench_code_found_t * p_found)
{
    bench_nav_report(BENCH_VOTE_DECODER1, bench_vote_mm(p_found->from_units),
                     bench_vote_mm(p_found->to_units),
                     p_found->b_reverse ? p_found->reverse : p_found->text);
}

/** @brief One decoder's result for a card (bench_vote_add, or Decoder 1 at
 * once): p_code NULL or not a single framed A-Z is a report with no read.
 * A path past the card held starts a new card; a card wholly before the
 * last junction, or wholly before the card held, is late and ignored. The
 * card's
 * letter is decided again at each report (bench_nav_decide).
 */
void bench_nav_report (uint32_t who, int32_t from_mm, int32_t to_mm,
                              const char * p_code)
{
    char    letter = '\0';
    int32_t swap;

    if ((who >= 3U) || (BENCH_MODE_MISSION != g_drive.mode))
    {
        return;
    }
    if (to_mm < from_mm)
    {
        swap = from_mm;
        from_mm = to_mm;
        to_mm = swap;
    }
    if (to_mm < g_nav.junction_mm)
    {
        return;
    }
    if (g_nav.b_card && (to_mm < g_nav.card_from))
    {
        /* Late, for a card before the one held (19 Nov 15:59: junction 1
         * missed, D0 read Q on the second card, then Decoder 2's late F for the
         * first card started a card over it and the U-turn was made at Q's
         * junction). Cards come card, junction, card, junction
         * (19 Nov): the newer card is the one the next junction is for.
         */
        return;
    }
    if ((NULL != p_code) && ('*' == p_code[0]) && (p_code[1] >= 'A') &&
        (p_code[1] <= 'Z') && ('*' == p_code[2]) && ('\0' == p_code[3]))
    {
        letter = p_code[1];
    }
    if (g_nav.b_card && ('\0' == letter) && ('\0' != g_nav.letter) &&
        (from_mm > g_nav.card_to))
    {
        /* A no read beside a card that holds a letter is noise or a line
         * (14 Nov 23:58: the junction's tape under the side sensor, cut
         * while the car stood on it, wiped the card's Y and the U-turn was
         * not made). Only a read starts a new card over it.
         */
        return;
    }
    if ((!g_nav.b_card) || (from_mm > g_nav.card_to))
    {
        g_nav.b_card = true;
        g_nav.card_from = from_mm;
        g_nav.card_to = to_mm;
        g_nav.reported = 0U;
        (void)memset(g_nav.read, 0, sizeof(g_nav.read));
    }
    else
    {
        g_nav.card_from = (from_mm < g_nav.card_from) ? from_mm :
                                                        g_nav.card_from;
        g_nav.card_to = (to_mm > g_nav.card_to) ? to_mm : g_nav.card_to;
    }
    g_nav.reported |= (uint32_t)1U << who;
    if (('\0' != letter) || ('\0' == g_nav.read[who]))
    {
        g_nav.read[who] = letter;   /* a read never undone by a no read */
    }
    g_nav.letter = bench_nav_decide(NULL, NULL);
    g_nav.b_say_code = true;
}

/** @brief The card's letter by the vote's rule among the decoders that
 * have read it so far: all agree, or a majority; else (a split, or no
 * read) none. *p_votes and *p_readers, if given: the winning count and the
 * readers.
 */
char bench_nav_decide (uint32_t * p_votes, uint32_t * p_readers)
{
    uint32_t who;
    uint32_t other;
    uint32_t votes;
    uint32_t best_votes = 0U;
    uint32_t readers = 0U;
    char     best = '\0';

    for (who = 0U; who < 3U; ++who)
    {
        if ('\0' == g_nav.read[who])
        {
            continue;
        }
        ++readers;
        votes = 0U;
        for (other = 0U; other < 3U; ++other)
        {
            if (g_nav.read[other] == g_nav.read[who])
            {
                ++votes;
            }
        }
        if (votes > best_votes)
        {
            best_votes = votes;
            best = g_nav.read[who];
        }
    }
    if (NULL != p_votes)
    {
        *p_votes = best_votes;
    }
    if (NULL != p_readers)
    {
        *p_readers = readers;
    }
    return ((0U != readers) &&
            ((best_votes == readers) || ((2U * best_votes) > readers))) ?
           best : '\0';
}

/** @brief The card's reads as text: "Decoder 1 A, Decoder 2 -, D0 A"
 * (- no read, ? not in yet).
 */
void bench_nav_reads (void)
{
    static const char * const p_who[3] =
    {
        "Decoder 1 ", ", Decoder 2 ", ", D0 "
    };
    char     one[2] = {'\0', '\0'};
    uint32_t who;

    /* Frame: each decoder 2 bits (0 not in, 1 no read, 2 a letter, then it
     * as a Code 39 character).
     */
    for (who = 0U; who < 3U; ++who)
    {
        bench_text(p_who[who]);
        if (0U == (g_nav.reported & ((uint32_t)1U << who)))
        {
            bench_code(0U, 2U);
            bench_text("?");
        }
        else if ('\0' == g_nav.read[who])
        {
            bench_code(1U, 2U);
            bench_text("-");
        }
        else
        {
            bench_code(2U, 2U);
            one[0] = g_nav.read[who];
            bench_str(one, 1U);
        }
    }
}

/** @brief MISSION line step, after the live step: queue a stored code's or
 * a new map's line (bench_avoid_say sends it live).
 */
void bench_nav_say (void)
{
    static const char * const p_moves[NAVMAP_MOVES] =
    {
        "FORWARD", "U-TURN", "LEFT", "RIGHT"
    };
    char    one[2] = {'\0', '\0'};
    int32_t move;

    if (0U != g_length)
    {
        return;
    }
    if (gb_nav_restored)
    {
        uint32_t index;

        gb_nav_restored = false;
        bench_line(BK_NAV_RESTORED);   /* restores 0-255, 8 bytes */
        bench_text("NAV: map restored from its backup (restore ");
        bench_u(g_nav_restores, 8U);
        bench_text("); g_nav began");
        for (index = 0U; index < sizeof(g_nav_seen); ++index)
        {
            bench_text(" ");
            bench_nav_hex(g_nav_seen[index]);
        }
        bench_avoid_say();
        return;
    }
    if (g_nav.b_say_map)
    {
        g_nav.b_say_map = false;
        bench_nav_say_map(BK_NAV_MAP);   /* queued: a leading space */
        bench_avoid_say();
        return;
    }
    if (!g_nav.b_say_code)
    {
        return;
    }
    g_nav.b_say_code = false;
    move = bench_nav_move_of(g_nav.letter);
    one[0] = g_nav.letter;
    /* Frame: the card's side path 0-2097151 mm (21 bits each), the reads,
     * a letter (Code 39 character) and its move (2 bits).
     */
    bench_line(BK_NAV_CARD);
    bench_text("NAV: card ");
    bench_u((uint32_t)g_nav.card_from, 21U);
    bench_text("-");
    bench_u((uint32_t)g_nav.card_to, 21U);
    bench_text(" mm (");
    bench_nav_reads();
    bench_text("): ");
    if (!bench_flag('\0' != g_nav.letter))
    {
        bench_text("no code yet (no read, or the decoders disagree)");
    }
    else
    {
        bench_str(one, 1U);
        bench_text(", at the next junction: ");
        if (bench_flag(move >= 0))
        {
            bench_name(p_moves, (uint32_t)move, 2U);
        }
        else
        {
            bench_text(bench_flag(g_nav.b_mapped) ?
                       "not in the map, straight on" :
                       "no map yet, straight on");
        }
    }
    bench_avoid_say();
}

/** @brief MISSION: a new cross-junction under both front sensors. The
 * stored letter's move: LEFT, RIGHT or U-TURN start (true, *p_request set);
 * FORWARD, no letter or no mapping go on straight (false: bench_line_cross
 * drives over it, as before). A letter used here is gone.
 */
bool bench_nav_junction (uint64_t now_us, int32_t centre,
                                motion_request_t * p_request)
{
    static const char * const p_moves[NAVMAP_MOVES] =
    {
        "FORWARD", "U-TURN", "LEFT", "RIGHT"
    };
    char     one[2] = {'\0', '\0'};
    uint32_t votes = 0U;
    uint32_t readers = 0U;
    char     letter = bench_nav_decide(&votes, &readers);
    int32_t  move = bench_nav_move_of(letter);
    bool     b_card = g_nav.b_card;

    ++g_nav.junctions;
    /* Frame: junctions 0-255 a run, centre path +-1048575 mm (21 bits), the
     * reads, letter and move as in NAV card, why straight on 2 bits, the
     * advance 0-511 mm.
     */
    bench_line(BK_NAV_JUNCTION);
    bench_text("NAV: junction ");
    bench_u(g_nav.junctions, 10U);
    bench_text(" at ");
    bench_i(bench_mm(centre), 21U);
    bench_text(" mm: ");
    if (bench_flag(b_card))
    {
        bench_text("card (");
        bench_nav_reads();
        bench_text(") ");
    }
    /* The card is used up here, whatever it held. */
    g_nav.b_card = false;
    g_nav.reported = 0U;
    (void)memset(g_nav.read, 0, sizeof(g_nav.read));
    g_nav.letter = '\0';
    g_nav.b_say_code = false;
    g_nav.junction_mm = bench_mm(centre);
    if (!bench_flag('\0' != letter))
    {
        bench_code((!b_card) ? 0U : (0U == readers) ? 1U : 2U, 2U);
        bench_text((!b_card) ? "no code stored, straight on" :
                   (0U == readers) ? "no read, straight on" :
                                     "decoders disagree, straight on");
        bench_avoid_say();
        return false;
    }
    one[0] = letter;
    bench_text("code ");
    bench_str(one, 1U);
    bench_text(bench_flag(votes != readers) ? " by majority" : "");
    if (!bench_flag(move >= 0))
    {
        bench_text(bench_flag(g_nav.b_mapped) ?
                   " not in the map, straight on" :
                   ", no map yet, straight on");
        bench_avoid_say();
        return false;
    }
    bench_text(" -> ");
    bench_name(p_moves, (uint32_t)move, 2U);
    if ((int32_t)NAVMAP_FORWARD == move)
    {
        bench_avoid_say();
        return false;
    }
    bench_text(", advancing ");
    bench_u((uint32_t)bench_round(BENCH_NAV_ADVANCE_MM), 9U);
    bench_text(" mm to turn on the crossing");
    bench_avoid_say();
    g_nav.move = (uint8_t)move;
    g_nav.phase = (uint8_t)BENCH_NAV_ADVANCE;
    g_nav.from_mm = bench_mm(centre);
    g_nav.hold = g_drive.heading;
    bench_nav_drive(now_us, centre, p_request);
    return true;
}

/** @brief MISSION, both front sensors gone dark (BENCH_NAV_PROBE_*): straight
 * on the heading held, until the line is seen again past the dark (a
 * junction: bench_nav_junction from where it went dark) or is not by
 * BENCH_NAV_PROBE_TO_MM (a turn: the letter kept, the line follower takes
 * over). True while probing, or when the junction's move has started
 * (*p_request set either way); false when the line follower goes on.
 */
bool bench_nav_probe (uint64_t now_us, int32_t centre,
                             motion_request_t * p_request)
{
    double past = (double)(bench_mm(centre) - bench_mm(g_nav.probe_from));
    double left = g_centre.raw[0];
    double right = g_centre.raw[1];
    bool   b_line = (fmax(left, right) >= BENCH_NAV_LINE_DARK) ||
                    ((left + right) >= BENCH_NAV_LINE_SUM);

    if ((past >= BENCH_NAV_PROBE_FROM_MM) && b_line)
    {
        g_nav.b_probe = false;
        g_nav.guard_to_mm = bench_mm(g_nav.probe_from) + BENCH_NAV_GUARD_MM;
        return bench_nav_junction(now_us, g_nav.probe_from, p_request);
    }
    if (past >= BENCH_NAV_PROBE_TO_MM)
    {
        g_nav.b_probe = false;
        ++g_nav.turns;
        /* Frame: the dark's centre path +-1048575 mm (21 bits), the probe
         * 0-511 mm, the reads (the letter is still stored).
         */
        bench_line(BK_NAV_TURN);
        bench_text("NAV: no line ");
        bench_u((uint32_t)bench_round(past), 9U);
        bench_text(" mm past the dark at ");
        bench_i(bench_mm(g_nav.probe_from), 21U);
        bench_text(" mm: a turn, not a junction; the card (");
        bench_nav_reads();
        bench_text(") waits for the next junction");
        bench_avoid_say();
        return false;
    }
    (void)memset(p_request, 0, sizeof(*p_request));
    p_request->kind = MOTION_VELOCITY;
    p_request->command_id = g_drive.request.command_id;
    bench_avoid_hold(g_nav.probe_heading, p_request);
    g_drive.request = *p_request;
    return true;
}

/** @brief MISSION junction move, each tick: ADVANCE on the junction heading
 * until the axle is over the crossing, PIVOT in place until the turn summed
 * from the heading reaches 90 deg (180 for the U-turn, to the left), then
 * hand back to the line follower afresh (bench_avoid_rejoin).
 */
void bench_nav_drive (uint64_t now_us, int32_t centre,
                             motion_request_t * p_request)
{
    double target_deg = ((uint8_t)NAVMAP_UTURN == g_nav.move) ?
                        BENCH_NAV_UTURN_DEG : BENCH_NAV_TURN_DEG;
    double sign = ((uint8_t)NAVMAP_RIGHT == g_nav.move) ? -1.0 : 1.0;
    double target = sign * target_deg * CORE_PI / 180.0;
    double tol = BENCH_AVOID_PIVOT_TOL_DEG * CORE_PI / 180.0;

    (void)memset(p_request, 0, sizeof(*p_request));
    p_request->kind = MOTION_VELOCITY;
    p_request->command_id = g_drive.request.command_id;
    if ((uint8_t)BENCH_NAV_ADVANCE == g_nav.phase)
    {
        bench_avoid_hold(g_nav.hold, p_request);
        if ((double)(bench_mm(centre) - g_nav.from_mm) >= BENCH_NAV_ADVANCE_MM)
        {
            p_request->forward_mps = 0.0;
            p_request->yaw_rps = 0.0;
            g_nav.phase = (uint8_t)BENCH_NAV_PIVOT;
            g_nav.turned = 0.0;
            g_nav.last_heading = g_drive.heading;
        }
    }
    else
    {
        g_nav.turned += bench_avoid_wrap(g_drive.heading - g_nav.last_heading);
        g_nav.last_heading = g_drive.heading;
        if ((sign * (target - g_nav.turned)) > tol)
        {
            p_request->yaw_rps = sign * ROBOT_TURN_RATE_RPS;
        }
        else
        {
            g_nav.phase = (uint8_t)BENCH_NAV_IDLE;
            ++g_nav.moves;
            bench_avoid_rejoin(now_us, centre);
            bench_line(BK_NAV_TURNED);   /* +-511 deg, 10 bits */
            bench_text("NAV: turned ");
            bench_s(bench_round(g_nav.turned * 180.0 / CORE_PI), 10U);
            bench_text(" deg on the crossing, following the line");
            bench_avoid_say();
        }
    }
    g_drive.request = *p_request;
}

/** @brief MISSION: true while a decoder still has work on what the side
 * sensor has crossed: Decoder 1 a group open (waiting for its white), its
 * tries running or a result not yet sent; D0 a group open or one waiting
 * to be sent (its vote goes in as it is sent); Decoder 2 a card binned, the fit
 * running, or a line waiting to be sent.
 */
bool bench_nav_pending (bool b_open_too)
{
    uint32_t index;

    if ((b_open_too && (g_code.b_open || g_d0.b_open)) || g_code.b_trying ||
        (0U != g_d0.queued) || g_fit_bin.b_used ||
        (BENCH_FIT_IDLE != g_fit.phase) || (0U != g_fit_line_count))
    {
        return true;
    }
    for (index = 0U; index < g_code.founds; ++index)
    {
        if (!g_code.found[index].b_sent)
        {
            return true;
        }
    }
    return false;
}

/** @brief MISSION, stopped at a junction: what is stored decoded now, as
 * if the white after it had come (the car stands, so it will not): Decoder 1's
 * open group cut (bench_code_tries_begin), D0's closed, and the binned card
 * let into the fit (gb_fit_force).
 */
void bench_nav_force (void)
{
    gb_fit_force = true;
    if (g_code.b_stream && g_code.b_open && (!g_code.b_trying) &&
        (0U != g_code.count) && (g_code.unit_m > 0.0))
    {
        bench_code_tries_begin((uint32_t)bench_round(
            (BENCH_CODE_QUIET_MM / 1000.0) / g_code.unit_m));
    }
    bench_d0_close();
}

/** @brief MISSION, a junction under both front sensors, before its move is
 * chosen (bench_nav_junction): true while the car is to stand on it for the
 * card's decode (*p_request a stop). It starts only while a decoder still
 * has work (bench_nav_pending) and no two decoders agree yet; it ends when
 * none has, or after BENCH_NAV_WAIT_MS; then false, and the move is chosen
 * from what is in. The line step goes on meanwhile: the decode steps and the
 * live lines that hand each decoder's result to the vote.
 */
bool bench_nav_wait (uint64_t now_us, int32_t centre,
                            motion_request_t * p_request)
{
    uint32_t votes = 0U;
    uint32_t readers = 0U;
    uint32_t waited_ms;

    if (!g_nav.b_waiting)
    {
        (void)bench_nav_decide(&votes, &readers);
        if ((votes >= 2U) || (!bench_nav_pending(true)))
        {
            return false;
        }
        g_nav.b_waiting = true;
        g_nav.wait_us = now_us;
        g_nav.wait_centre = centre;
        ++g_nav.waits;
        bench_line(BK_NAV_WAIT);   /* centre path 21 bits, the reads */
        bench_text("NAV: junction at ");
        bench_i(bench_mm(centre), 21U);
        bench_text(" mm, the card is still decoding (");
        bench_nav_reads();
        bench_text("): stopped to wait for it");
        bench_avoid_say();
        /* Once: what is open is cut now. Groups that open while the car
         * stands are noise or the junction's tape under the side sensor;
         * cutting them every tick kept Decoder 1 busy and the fit from
         * starting (14 Nov 23:58: Decoder 2 read the Y after the 3 s ran out).
         */
        bench_nav_force();
    }
    waited_ms = (uint32_t)((now_us - g_nav.wait_us) / 1000U);
    (void)bench_nav_decide(&votes, &readers);
    if ((votes < 2U) && bench_nav_pending(false) &&
        (waited_ms < BENCH_NAV_WAIT_MS))
    {
        (void)memset(p_request, 0, sizeof(*p_request));
        p_request->kind = MOTION_VELOCITY;
        p_request->command_id = g_drive.request.command_id;
        g_drive.request = *p_request;
        return true;
    }
    g_nav.b_waiting = false;
    gb_fit_force = false;
    bench_line(BK_NAV_WAITED);   /* 0-65535 ms, why 2 bits, the reads */
    bench_text("NAV: waited ");
    bench_u(waited_ms, 16U);
    bench_code((votes >= 2U) ? 0U : bench_nav_pending(false) ? 1U : 2U, 2U);
    bench_text((votes >= 2U) ? " ms, two decoders agree (" :
               bench_nav_pending(false) ?
               " ms, still decoding: deciding on (" : " ms, decoding done (");
    bench_nav_reads();
    bench_text(")");
    bench_avoid_say();
    return false;
}

/** @brief Line task: one IR sample into the ring (the oldest goes if it is
 * full: the Vehicle task was held up for over 60 ms).
 */
void mission_ir_push (const uint16_t raw[3], bool b_valid,
                      uint16_t left_total, uint16_t right_total)
{
    mission_ir_t sample;
    uint32_t     saved;
    uint32_t     next;

    sample.left_raw = (uint32_t)raw[0] & 0xFFFU;
    sample.right_raw = (uint32_t)raw[1] & 0xFFFU;
    sample.side_raw = (uint32_t)raw[2] & 0xFFFU;
    sample.b_valid = b_valid;
    sample.left_total = left_total;
    sample.right_total = right_total;
    saved = app_main_lock();
    next = (g_ir_head + 1U) % MISSION_IR_RING;
    if (next == g_ir_tail)
    {
        g_ir_tail = (uint8_t)((g_ir_tail + 1U) % MISSION_IR_RING);
        if (UINT8_MAX != g_ir_dropped)
        {
            ++g_ir_dropped;
        }
    }
    g_ir_ring[g_ir_head] = sample;
    g_ir_head = (uint8_t)next;
    app_main_unlock(saved);
}

/** @brief The IR samples the Line task took since the last step: all but
 * the newest into the barcode store while a run reads cards (the bench's
 * 1 ms polls); the newest is this step's reading (board_read_ir).
 */
void mission_ir_drain (void)
{
    mission_ir_t sample;
    uint16_t     raw[LINE_CTRL_CHANNELS];
    uint32_t     saved;
    bool         b_have;

    for (;;)
    {
        saved = app_main_lock();
        b_have = g_ir_tail != g_ir_head;
        if (b_have)
        {
            sample = g_ir_ring[g_ir_tail];
            g_ir_tail = (uint8_t)((g_ir_tail + 1U) % MISSION_IR_RING);
        }
        app_main_unlock(saved);
        if (!b_have)
        {
            return;
        }
        /* The previous newest was not this step's: a poll, as on the bench
         * (a held sample was polled once already).
         */
        if (gb_ir_now && gb_ir_fresh && bench_code_polling())
        {
            bench_code_poll_at(g_ir_now, g_ir_now_total);
        }
        if (sample.b_valid)
        {
            raw[0] = (uint16_t)sample.left_raw;
            raw[1] = (uint16_t)sample.right_raw;
            raw[2] = (uint16_t)sample.side_raw;
            (void)memcpy(g_ir_now, raw, sizeof(g_ir_now));
            g_ir_now_total[BENCH_LEFT] = (uint16_t)sample.left_total;
            g_ir_now_total[BENCH_RIGHT] = (uint16_t)sample.right_total;
            gb_ir_now = true;
            gb_ir_fresh = true;
            g_ir_failed = 0U;
        }
        else
        {
            /* Noise control: a failed read keeps the last good sample, up
             * to MISSION_IR_HOLD failed reads in a row (the Line task's
             * OPTICAL_SAMPLE needs as many).
             */
            gb_ir_fresh = false;
            if (g_ir_failed < UINT8_MAX)
            {
                ++g_ir_failed;
            }
            gb_ir_now = gb_ir_now && (g_ir_failed < MISSION_IR_HOLD);
        }
    }
}

/* End of file. */

