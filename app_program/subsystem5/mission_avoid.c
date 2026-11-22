/** @file mission_avoid.c
 * @brief The mission: subsystem 5: the front sonar and the obstacle go-
 * around. Split out of the one file by tools/reorg/split_c.py (22 Nov 2026);
 * what the parts share is in mission_private.h.
 */
#include "mission_private.h"
#include "mission_avoid.h"
#define board_ultrasonic_trigger  mission_sonar_trigger
#define board_ultrasonic_service  mission_sonar_service
#define board_ultrasonic_result   mission_sonar_result
#define board_servo_us            mission_servo_us
#define BENCH_SERVO_CENTRE_US (1500U)
#define BENCH_SERVO_FINE_US (10U)
#define BENCH_SERVO_COARSE_US (100U)
#define BENCH_SCAN_MAX_US (20000000U)   /* robot_app_scan_cycle's limit */
#define BENCH_SCAN_PER_ROW (8U)         /* beams per printed line */
#define BENCH_SCAN_STEP_DEG (2)         /* scanner.c's fine beam grid */
#define BENCH_SCAN_SIDES (3U)
#define BENCH_AVOID_PAUSE_MS (500U)    /* standstill before measuring */
#define BENCH_AVOID_SONAR_MM (70.0)    /* sonar ahead of the pivot (wheel axle
                                        * centre): the front sensors are
                                        * 130 mm ahead (ruler, 29 Oct) and
                                        * the sonar's front 60 mm behind
                                        * their front end (3 Nov) */
#define BENCH_AVOID_DEPTH_MM (150.0)   /* obstacle depth, at most (given:
                                        * 15 cm); hits this far behind the
                                        * face count */
#define BENCH_AVOID_MEASURE_DEG (80)   /* furthest each way the car turns */

/* readings past the edge to end a side */
#define BENCH_AVOID_MISSES (2U)
#define BENCH_AVOID_SPREAD_MM (20U)
#define BENCH_AVOID_CONE_DEG (15.0)    /* HC-SR04 measuring angle (datasheet):
                                        * the cone's half-angle, taken off
                                        * each hit's angle */
#define BENCH_AVOID_ANGLE_DEG (90)     /* out/back turns: a full right/left */
#define BENCH_AVOID_CLEAR_MM (70.0)    /* gap between car and obstacle */
#define BENCH_AVOID_HOLD_KP (4.0)      /* straight/diagonal heading-hold gain */
#define BENCH_AVOID_CAR_WIDTH_MM (160.0)/* 13 cm body + 3 cm QR sensor */
#define BENCH_AVOID_BACKUP_MM (150.0)  /* reverse this far to see more */
#define BENCH_AVOID_BACKUPS (2U)       /* reverses at most, then plan anyway */
#define BENCH_AVOID_REVERSE_MPS (0.06) /* reversing speed */
#define BENCH_AVOID_CREEP_MPS (0.06)   /* creeping speed: ~4 mm a reading */
#define BENCH_AVOID_CREEP_GAP_MM (20.0)/* closest the creep lets the front
                                        * corner come to the face: the face
                                        * is known from one range and the
                                        * corner from odometry, and the car
                                        * runs on a tick before it stops */
#define BENCH_AVOID_FACE_MM (40.0)     /* a creep echo at most this far behind
                                        * the face is the face (one turned
                                        * ~15 deg, a bottle's curve); from
                                        * further back (the box's side or
                                        * back, something else) the beam has
                                        * passed the face's edge */

/* 14 Nov: both creeps of the day ended at once with no echo at all ("edge
 * 45 mm out (last echo none)"): 45 mm is only where the 40 deg creep pivot
 * puts the sonar (70 x sin 40), already past the edge of a 105 mm box near
 * the line, so the edge was never measured (the dashboard then read it as
 * 87 mm). A creep with no echo off the face now turns
 * BENCH_AVOID_CREEP_STEP_DEG
 * less and creeps again (30, 20, 10 deg: the sonar 35, 24, 12 mm out), so it
 * starts inside the edge and finds it square on. With none even at
 * BENCH_AVOID_CREEP_MIN_DEG it goes on as before, from the first start's
 * offset (the widest, safe for the way round), the edge marked unseen.
 * The servo turns the sonar back by the creep's angle, and it reaches only
 * ROBOT_SCAN_HALF_ANGLE_DEG (the calibrated ends; the sonar's board meets
 * the connectors near 50, and at the right end its cable once pulled it out
 * of its holder). So the creep starts BENCH_AVOID_CREEP_START_DEG, 10 deg
 * inside that (14 Nov): the pivot's tolerance can still be followed,
 * and the far-side creep (servo to the right end) stays off the stop.
 */
/* A creep's edge reads past the real one: square on, the sonar still hears
 * the corner a little after it has passed it. 14 Nov, both edges by creep,
 * the 105 mm box, 4 runs: 141, 122, 141, 143 mm, no clear tie to the range
 * (195-312 mm), so 16 mm an edge comes off what is REPORTED (the WIDTH line
 * and OBST). The way round keeps the raw edge: the overreach is its margin.
 * The turning sweep the same runs: 101, 71, 81, 104 (wider spread). One box
 * only: [CALIBRATE] with another width before trusting it further.
 */
#define BENCH_AVOID_CREEP_OVER_MM (16.0)
#define BENCH_AVOID_CREEP_STEP_DEG (10)
#define BENCH_AVOID_CREEP_MIN_DEG (10)
#define BENCH_AVOID_GUARD_MM ((BENCH_AVOID_FRONT_MM - BENCH_AVOID_SONAR_MM) + \
                              BENCH_AVOID_CLEAR_MM)
#define BENCH_AVOID_EXTENDS (2U)       /* STEPIN retries past a blocked path */

/* Turn check (3 Nov). The car has no gyro: a pivot's angle is the encoders',
 * and on the floor the 90 deg turns never came out at 90 (car tests 3 Nov:
 * "back on course", then the line lost). A turn that ends on the line is
 * checked against it and finishes on it, MISSION only (the line's
 * calibration):
 *   - MEASURE_R swings the front sensors back across the line the car
 *     stopped on: the heading that centres the line resets yaw_home, the
 *     heading every later turn aims at, before the go-around starts.
 *   - HOMEPIVOT, the last turn back onto course, stops where a front sensor
 *     crosses the line, pivoting on past the wheels' target up to
 *     BENCH_TURN_CHECK_DEG to find it; with no line there it looks on (a
 *     bend, a junction or a short turn: BENCH_AVOID_SEARCH_DEG), and with
 *     none anywhere it goes by the wheels, as before.
 * A front sensor sweeping across the line (BENCH_AVOID_FRONT_MM out) is dark
 * for about BENCH_TURN_LINE_RAD of turn. A dark stretch of half to twice
 * that whose start was seen is the line; its middle, plus the sensor's
 * bearing (half ROBOT_LINE_SENSOR_SPACING_M at BENCH_AVOID_FRONT_MM: + for
 * the left sensor), is the heading that puts the line midway between the
 * sensors. Barcode bars start 29 mm off the line centre (spec), 12.9 deg off
 * it at the sensors (asin(29 / 130)) with the axle on the line: both checks
 * take only crossings within BENCH_TURN_CHECK_DEG of where the wheels put
 * the line. 30 deg took bars for the line (creep sim, cards at the
 * rejoin). A bigger turn error than that is left to the wheels and the
 * line follower, as before.
 */
#define BENCH_TURN_CHECK_DEG (10.0) /* inside the bars' 12.9 deg */

/* The go-around's line checks class a front sensor dark from half its span
 * (this run's lowest to highest, bench_avoid_ir_setup), where the tape's
 * edge is under the middle of the sensor's spot, so a dark stretch is as
 * long as the tape is wide; light again below BENCH_AVOID_IR_OFF. 75 %
 * (line_ctrl's) missed a moving crossing that peaked at 60-75 % (3 Nov).
 * STEPIN takes a crossing as the line when both sensors' middles lie within
 * the sensor spacing x tan(BENCH_AVOID_SQUARE_DEG): the car is then under
 * that angle to the line, and a card's bar run along its length does not
 * pass (3 Nov).
 */
#define BENCH_AVOID_IR_ON (0.5)
#define BENCH_AVOID_IR_OFF (0.3)
#define BENCH_AVOID_SQUARE_DEG (30.0)
#define BENCH_AVOID_CROSS_LOGS (3U) /* STEPIN stretches not taken, logged */

/* HOMEPIVOT with no line within BENCH_TURN_CHECK_DEG of its target (car
 * videos 3 Nov: an obstacle 10-15 cm before a junction and before a bend put
 * the way back past them; the car came in along the junction's side branch
 * and along the bend's far leg, and the quarter turn home faced white floor):
 * the car turns on, up to BENCH_AVOID_SEARCH_DEG past the target, for the
 * line, or back to a crossing it swept over on the way to the target,
 * whichever is nearer the target. With the axle on the track, the track
 * through that point crosses the front sensors' circle wherever it goes; a
 * quarter turn either side of the line heading covers a bend or a junction
 * branch, ahead or toward the side the car came from. Lines up to
 * BENCH_TURN_CHECK_DEG beyond that quarter turn are taken, and the pivot
 * runs on until a crossing there has ended (two line widths of turn). A
 * front sensor still dark when STEPIN ends means the car came in along a
 * line (a bend's far leg or a junction's branch; creep sim, a bend 5 cm past
 * the box): its heading is a candidate too, taken back to if nothing nearer
 * the target turns up. A turn that comes out short by more than
 * BENCH_TURN_CHECK_DEG (creep sim: 15 % short turns) ends on the line the
 * same way.
 */
#define BENCH_AVOID_SEARCH_DEG (90.0)

/* MISSION sends the go-around's lines live (bench_avoid_say): a flush would
 * wait on the radio between control steps (SENSOR_FAULT). Up to
 * BENCH_AVOID_LINES wait to go out; more are counted, dropped. For the
 * dashboard's map it also sends the obstacle (OBST), the map rows the sonar
 * hit (M) and, every BENCH_POSE_US, the car's odometry pose (P).
 */
#define BENCH_AVOID_LINES (32U)

/* 144: the NAV junction line, with the decoders named "Decoder 1" and
 * "Decoder 2" (19 Nov), runs to ~130 characters; 112 cut it short.
 */
#define BENCH_AVOID_LINE_BYTES (144U)

/* MISSION also sends every beam of the measuring sweeps (bench_avoid_beam:
 * the car's angle from the line heading in whole deg, + left, the range in
 * mm, 0 for no echo), BENCH_BEAMS_PER_LINE to a BEAMS line, only when the
 * link is free (after the queued lines and a due pose), so the cone and the
 * sonar's place can be fitted from one run past a box of known size. At
 * most BENCH_BEAMS per obstacle are kept; more are counted.
 */
#define BENCH_BEAMS (96U)
#define BENCH_BEAMS_PER_LINE (10U)
#define BENCH_BEAMS_SAME_MM (5)       /* a BEAMS line's beams, back within */
#define BENCH_MAP_CELL_MM (20.0)       /* map grid */
#define BENCH_MAP_X_MIN_MM (-400.0)    /* nearest row, behind the first stop */
#define BENCH_MAP_MID (20)             /* column of the line (y = 0) */

/** Obstacle map cells (bench_avoid_mark). */
typedef enum
{
    BENCH_MAP_UNKNOWN = 0,    /* no beam reached it */
    BENCH_MAP_FREE,           /* a beam passed through */
    BENCH_MAP_OBSTACLE,       /* a hit within the obstacle's depth */
    BENCH_MAP_OTHER           /* a hit further back: something else */
} bench_map_cell_t;

/* Indexed by scanner_side_t. */
static const char * const g_side_names[BENCH_SCAN_SIDES] =
{
    "NONE", "LEFT", "RIGHT"
};
bench_cal_t   g_cal;
char          g_text[BENCH_TEXT_BYTES];
uint32_t      g_length;
  /* MISSION's live line */
uint32_t      g_live_length;
uint32_t      g_live_sent;

/* The frame being built beside g_text, and the live line's frame. */
uint8_t          g_frame[BENCH_FRAME_BYTES];
bitpack_writer_t g_frame_w;
   /* MISSION status: front IR raw now */
uint16_t      g_live_front_min;

  /* MISSION status F: nearest sonar
                                         * range since the last, 0: none */
uint32_t      g_live_front_none;
bool          gb_watchdog;
static uint16_t      g_servo_us;
          /* 0 until the first command */
static uint16_t      g_servo_step = BENCH_SERVO_FINE_US;
uint32_t      g_send_tries = BENCH_SEND_TRIES;
                 /* P 3, I 15 */
line_ctrl_runtime_t g_line_runtime;
line_ctrl_state_t   g_line_state;
bench_line_t        g_line;
bench_front_t       g_front;
static core_ultrasonic_config_t g_front_range;
static scanner_state_t     g_scanner;

/* The queued lines' text: only where it is shown (offline and the sim);
 * the MQTT frames build sends the frames alone (22 Nov: 4.6 kB back for
 * the 10 s recorder within the 64 KiB heap reserve).
 */
#define BENCH_AVOID_TEXT (!(BENCH_FRAMES && ROBOT_MQTT))
#if BENCH_AVOID_TEXT
static char g_avoid_lines[BENCH_AVOID_LINES][BENCH_AVOID_LINE_BYTES];
#endif

/* Each queued line's frame (bench_avoid_say) and its size, 0: none. */
static uint8_t  g_avoid_frames[BENCH_AVOID_LINES][BENCH_FRAME_BYTES];
static uint32_t g_avoid_frame_size[BENCH_AVOID_LINES];
uint32_t            g_avoid_queued;

   /* lines in g_avoid_lines */
/* lines the queue had no room for */
uint32_t            g_avoid_dropped;
    /* MISSION: next P line */
static int16_t             g_beam_deg[BENCH_BEAMS];
  /* sweep beams: angle */
static uint16_t            g_beam_mm[BENCH_BEAMS];
   /* range, 0 no echo */
static int16_t             g_beam_back[BENCH_BEAMS];
 /* back_mm then */
static uint32_t            g_beams;
          /* beams kept, this obstacle */
static uint32_t            g_beams_sent;
     /* of those, sent */
static uint32_t            g_beams_dropped;
  /* beams past BENCH_BEAMS */
bench_centre_t      g_centre;

/** @brief Step the scan servo within the bench limits (robot_config.h of
 * the bench copy): back off as soon as it buzzes at a mechanical stop.
 */
void bench_servo (uint8_t command)
{
    uint32_t pulse = (0U == g_servo_us) ? BENCH_SERVO_CENTRE_US : g_servo_us;

    if (BENCH_CMD_UTURN == command)
    {
        g_servo_step = (BENCH_SERVO_FINE_US == g_servo_step) ?
                       (uint16_t)BENCH_SERVO_COARSE_US :
                       (uint16_t)BENCH_SERVO_FINE_US;
    }
    else if (BENCH_CMD_STRAIGHT == command)
    {
        pulse = BENCH_SERVO_CENTRE_US;
    }
    else if (BENCH_CMD_LEFT == command)
    {
        pulse = (pulse >= (ROBOT_SERVO_MIN_US + g_servo_step)) ?
                (pulse - g_servo_step) : ROBOT_SERVO_MIN_US;
    }
    else
    {
        pulse = ((pulse + g_servo_step) <= ROBOT_SERVO_MAX_US) ?
                (pulse + g_servo_step) : ROBOT_SERVO_MAX_US;
    }
    if ((BENCH_CMD_UTURN != command) || (0U != g_servo_us))
    {
        if (board_servo_us((uint16_t)pulse))
        {
            g_servo_us = (uint16_t)pulse;
        }
    }
    bench_text("SERVO ");
    if (0U == g_servo_us)
    {
        bench_text("off");
    }
    else
    {
        bench_dec((int32_t)g_servo_us);
        bench_text(" us");
    }
    bench_text(", step ");
    bench_dec((int32_t)g_servo_step);
    bench_text(" us");
    bench_flush();
}

/** @brief LINE + stop: the robot's sonar settings (app_cfg_scanner), servo
 * centred, a stale echo consumed; the first trigger waits for the servo.
 */
void bench_front_start (void)
{
    scanner_config_t config;
    uint32_t         pulse_us = 0U;
    bool             b_valid = false;

    (void)memset(&g_front, 0, sizeof(g_front));
    app_cfg_scanner(&config);
    g_front_range = config.range;
    (void)bench_scan_servo(0);
    (void)board_ultrasonic_result(&pulse_us, &b_valid);
    g_front.ready_us = board_time_us() +
                       ((uint64_t)ROBOT_SERVO_SETTLE_MS * 1000U);
}

/** @brief LINE + stop, every tick: the front polling of robot_app_scan_step
 * (consume the echo, then trigger again as soon as the board allows), on
 * the robot's ceil-mm range. True for a new VALID range, in last_mm.
 * Readings are counted only while driving.
 */
bool bench_front_step (uint64_t now_us)
{
    core_range_sample_t range = {CORE_RANGE_UNKNOWN, 0.0};
    core_range_packed_t packed;
    uint32_t            pulse_us = 0U;
    bool                b_valid = false;
    bool                b_new = false;

    (void)memset(&packed, 0, sizeof(packed));
    board_ultrasonic_service();
    if (board_ultrasonic_result(&pulse_us, &b_valid) && g_front.b_pending)
    {
        g_front.b_pending = false;
        (void)core_ultrasonic_range(&g_front_range, b_valid,
                                    (double)pulse_us / 1000000.0, &range);
        core_range_pack(&range, &packed);
        if (!g_drive.b_done)
        {
            ++g_front.readings;
        }
        if (CORE_RANGE_VALID == packed.state)
        {
            g_front.last_mm = (uint16_t)packed.distance_mm;
            if ((0U == g_live_front_min) ||
                (g_front.last_mm < g_live_front_min))
            {
                g_live_front_min = g_front.last_mm;
            }
            b_new = true;
        }
        else
        {
            g_front.last_mm = 0U;
            ++g_live_front_none;
            if (!g_drive.b_done)
            {
                if (0U == g_front.invalid)
                {
                    g_front.first_invalid_ms =
                        (uint32_t)((now_us - g_drive.step_us) / 1000U);
                }
                ++g_front.invalid;
            }
        }
    }
    if ((!g_front.b_pending) && (now_us >= g_front.ready_us) &&
        board_ultrasonic_trigger())
    {
        g_front.b_pending = true;
    }
    return b_new;
}

/** @brief LINE + stop: the range that stopped the car, the range and the
 * travel 1 s later, and the readings that would have halted the robot.
 */
void bench_front_results (void)
{
    int32_t centre = bench_mm(((g_distance[BENCH_LEFT] -
                                g_drive.start[BENCH_LEFT]) +
                               (g_distance[BENCH_RIGHT] -
                                g_drive.start[BENCH_RIGHT])) / 2);

    bench_text(" front sonar: ");
    if (g_front.b_detected)
    {
        bench_text("obstacle at ");
        bench_dec((int32_t)g_front.detect_mm);
        bench_text(" mm after ");
        bench_dec((int32_t)g_front.detect_centre_mm);
        bench_text(" mm of line; 1 s later it reads ");
    }
    else
    {
        bench_text("no obstacle within ");
        bench_dec((int32_t)BENCH_LINE_STOP_MM);
        bench_text(" mm; latest ");
    }
    if (0U == g_front.last_mm)
    {
        bench_text("none");
    }
    else
    {
        bench_u((uint32_t)g_front.last_mm, 12U);
        bench_text(" mm");
    }
    if (g_front.b_detected)
    {
        bench_text(", car rolled ");
        bench_dec(centre - (int32_t)g_front.detect_centre_mm);
        bench_text(" mm after detection");
    }
    bench_flush();
    bench_text(" robot rule (");
    bench_dec((int32_t)ROBOT_FRONT_STOP_DISTANCE_MM);
    bench_text(" mm): ");
    if (g_front.b_robot)
    {
        bench_text("would have stopped at ");
        bench_dec((int32_t)g_front.robot_mm);
        bench_text(" mm after ");
        bench_dec((int32_t)g_front.robot_centre_mm);
        bench_text(" mm of line");
    }
    else
    {
        bench_text("never fired");
    }
    bench_flush();
    bench_text(" readings while driving ");
    bench_dec((int32_t)g_front.readings);
    bench_text(", not VALID ");
    bench_dec((int32_t)g_front.invalid);
    if (0U != g_front.invalid)
    {
        bench_text(" (first at ");
        bench_fixed(bench_round((double)g_front.first_invalid_ms / 100.0),
                    1U);
        bench_text(" s: the robot would halt there, RANGE_WAIT)");
    }
    bench_flush();
}

/** @brief Hold a straight heading: forward at the line speed with a gentle
 * proportional yaw toward the target heading (+ left), clamped to the turn
 * rate. No line sensors; the odometry heading is the only feedback.
 */
void bench_avoid_hold (double target, motion_request_t * p_request)
{
    double yaw = BENCH_AVOID_HOLD_KP *
                 bench_avoid_wrap(target - g_drive.heading); /* + left */

    if (yaw > ROBOT_TURN_RATE_RPS)
    {
        yaw = ROBOT_TURN_RATE_RPS;
    }
    else if (yaw < -ROBOT_TURN_RATE_RPS)
    {
        yaw = -ROBOT_TURN_RATE_RPS;
    }
    else
    {
        /* Within the turn rate; no clamp. */
    }
    p_request->forward_mps = bench_line_speed(g_drive.mode);
    p_request->yaw_rps = yaw;
}

/** @brief AVOID, all phases: a mapped rectangle around the obstacle (no line
 * sensors; odometry heading and wheel-integrated offsets).
 *
 * STRAIGHT holds the home heading until the sonar reads an obstacle within
 * BENCH_AVOID_TURN_MM. PAUSE stops, MEASURE_L and MEASURE_R pivot the car to
 * each edge, mapping every beam (bench_avoid_edge), and bench_avoid_plan
 * picks the way: with an edge unseen RECENTRE pivots back and REVERSE backs
 * up to measure again (the only reversing); else CREEPTURN turns part way to
 * the chosen side and CREEP drives out on that diagonal, the sonar square to
 * the face, until the echo is lost: that side's edge (bench_avoid_creep).
 * OUT turns on to the full angle, the servo tracking the object (it pans
 * opposite the turn) from CREEPTURN on. STEPOUT drives out until the car is
 * side_mm to that side of the line, ALIGN pivots back to the home heading and
 * ALONG runs parallel until the pivot is past the obstacle's back by half the
 * car + the clearance. BACKPIVOT pivots toward the line, STEPIN drives back
 * until the sideways offset reaches zero (on the line), and HOMEPIVOT turns
 * back to the home heading. Velocity only.
 */
void bench_avoid_drive (uint64_t now_us, bool b_range, int32_t centre,
                               motion_request_t * p_request)
{
    double  angle_rad = (double)BENCH_AVOID_ANGLE_DEG * CORE_PI / 180.0;
    double  creep_rad = (double)g_avoid.creep_deg * CORE_PI / 180.0;
    /* + left, right -ve, the short way round (bench_avoid_wrap) */
    double  turned = bench_avoid_wrap(g_drive.heading - g_avoid.yaw_home);
    double  side_rad = (double)g_avoid.side * angle_rad; /* + toward the side */
    double  half = BENCH_AVOID_CAR_WIDTH_MM / 2.0;
    int32_t servo_deg;

    (void)memset(p_request, 0, sizeof(*p_request));
    p_request->kind = MOTION_VELOCITY;
    p_request->command_id = g_drive.request.command_id;
    /* Integrate the offsets from the first stop while the go-around runs:
     * sideways from the line, and along it (back_mm, + behind).
     */
    if (BENCH_AVOID_STRAIGHT != g_avoid.phase)
    {
        double fwd_mm = (double)(centre - g_avoid.last_centre) *
                        BENCH_COUNT_M * 1000.0;

        /* + left heading moves left, so right offset falls; hence the minus. */
        g_avoid.right_mm -= fwd_mm * sin(turned);
        g_avoid.back_mm -= fwd_mm * cos(turned);
    }
    g_avoid.last_centre = centre;
    /* Track the object with the servo only while the creep measures its edge
     * (CREEPTURN, CREEP): square to the face at the creep's angle. Once the
     * edge is found (OUT, STEPOUT) nothing reads the sonar, and the depth is
     * given, not measured (14 Nov), so it is centred then.
     */
    if ((BENCH_AVOID_CREEPTURN == g_avoid.phase) ||
        (BENCH_AVOID_CREEP == g_avoid.phase) ||
        (BENCH_AVOID_CREEPBACK == g_avoid.phase))
    {
        /* opposite the turn */
        servo_deg = bench_round(-turned * 180.0 / CORE_PI);
        if (servo_deg > ROBOT_SCAN_HALF_ANGLE_DEG)
        {
            servo_deg = ROBOT_SCAN_HALF_ANGLE_DEG;
        }
        else if (servo_deg < -ROBOT_SCAN_HALF_ANGLE_DEG)
        {
            servo_deg = -ROBOT_SCAN_HALF_ANGLE_DEG;
        }
        else
        {
            /* Within the servo's reach. */
        }
        (void)bench_scan_servo(servo_deg);
    }
    if (BENCH_AVOID_STRAIGHT == g_avoid.phase)
    {
        bench_avoid_hold(g_avoid.yaw_home, p_request);
        if (b_range && (0U != g_front.last_mm) &&
            (g_front.last_mm <= BENCH_AVOID_TURN_MM))
        {
            bench_avoid_begin(now_us, centre);
            p_request->forward_mps = 0.0;
            p_request->yaw_rps = 0.0;
            bench_line(BK_AV_OBSTACLE_AVOID);
            bench_text("AVOID: obstacle at ");
            bench_u((uint32_t)g_front.last_mm, 12U);
            bench_text(" mm, stopped to measure it");
            bench_avoid_say();
        }
    }
    else if (BENCH_AVOID_PAUSE == g_avoid.phase)
    {
        if ((now_us - g_avoid.pause_us) >=
            ((uint64_t)BENCH_AVOID_PAUSE_MS * 1000U))
        {
            g_avoid.phase = BENCH_AVOID_MEASURE_L;
            g_avoid.readings = g_front.readings;
            g_avoid.misses = 0U;
            g_avoid.med_n = 0U;
            g_avoid.med_skips = 0U;
            g_avoid.b_hit = false;
            bench_line(BK_AV_MEASURE);
            bench_text("AVOID: measuring, turning left to its edge");
            bench_avoid_say();
        }
    }
    else if (BENCH_AVOID_MEASURE_L == g_avoid.phase)
    {
        p_request->yaw_rps = ROBOT_TURN_RATE_RPS; /* pivot left in place */
        if (bench_avoid_edge(turned))
        {
            g_avoid.b_left_open = g_avoid.b_hit;
            g_avoid.phase = BENCH_AVOID_MEASURE_R;
            g_avoid.turn_sum_rad = 0.0;
            g_avoid.turn_finds = 0U;
            bench_avoid_turn_begin(g_avoid.yaw_home);
            g_avoid.misses = 0U;
            g_avoid.med_n = 0U;
            g_avoid.b_hit = false;
            p_request->yaw_rps = 0.0;
            bench_line(BK_AV_LEFT_EDGE);   /* +-4095 mm, +-255 deg */
            bench_text("AVOID: left edge ");
            bench_i(bench_round(g_avoid.left_mm), 13U);
            bench_text(" mm at ");
            bench_i(bench_round(turned * 180.0 / CORE_PI), 9U);
            bench_text(bench_flag(g_avoid.b_left_open) ?
                       " deg (still on it at the turn limit: unseen),"
                       " turning right" :
                       " deg, turning right to the other edge");
            bench_avoid_say();
        }
    }
    else if (BENCH_AVOID_MEASURE_R == g_avoid.phase)
    {
        double line_rad = 0.0;

        p_request->yaw_rps = -ROBOT_TURN_RATE_RPS; /* pivot right in place */
        if (bench_avoid_turn_line(g_avoid.yaw_home,
                                  BENCH_TURN_CHECK_DEG * CORE_PI / 180.0,
                                  &line_rad))
        {
            g_avoid.turn_sum_rad += bench_avoid_wrap(line_rad -
                                                     g_avoid.yaw_home);
            ++g_avoid.turn_finds;
        }
        if (turned > 0.0)
        {
            /* Back across the left half: those beams are already mapped. */
            g_avoid.readings = g_front.readings;
            g_avoid.misses = 0U;
            g_avoid.med_n = 0U;
        }
        else if (bench_avoid_edge(-turned))
        {
            g_avoid.b_right_open = g_avoid.b_hit;
            p_request->yaw_rps = 0.0;
            bench_avoid_home_fix();
            bench_avoid_plan(now_us, p_request);
        }
        else
        {
            /* Still on the obstacle. */
        }
    }
    else if (BENCH_AVOID_RECENTRE == g_avoid.phase)
    {
        if (bench_avoid_pivot(g_avoid.yaw_home, p_request))
        {
            g_avoid.phase = BENCH_AVOID_REVERSE;
            g_avoid.back_from_mm = g_avoid.back_mm;
        }
    }
    else if (BENCH_AVOID_REVERSE == g_avoid.phase)
    {
        /* Heading hold steers by yaw rate, the same backwards. */
        bench_avoid_hold(g_avoid.yaw_home, p_request);
        p_request->forward_mps = -BENCH_AVOID_REVERSE_MPS;
        if ((g_avoid.back_mm - g_avoid.back_from_mm) >= BENCH_AVOID_BACKUP_MM)
        {
            p_request->forward_mps = 0.0;
            p_request->yaw_rps = 0.0;
            bench_line(BK_AV_BACKED);   /* +-4095 mm */
            bench_text("AVOID: backed up to ");
            bench_i(bench_round(g_avoid.back_mm), 13U);
            if (bench_flag(g_avoid.b_creeping))
            {
                /* The side is chosen; only its edge was out of reach. */
                g_avoid.phase = BENCH_AVOID_CREEPTURN;
                bench_text(" mm behind the first stop, creeping out again");
            }
            else
            {
                g_avoid.phase = BENCH_AVOID_PAUSE;
                g_avoid.pause_us = now_us;
                bench_text(" mm behind the first stop, measuring again");
            }
            bench_avoid_say();
        }
    }
    else if (BENCH_AVOID_CREEPTURN == g_avoid.phase)
    {
        if (bench_avoid_pivot(g_avoid.yaw_home -
                              ((double)g_avoid.creep_side * creep_rad),
                              p_request))
        {
            g_avoid.phase = BENCH_AVOID_CREEP;
            g_avoid.b_creeping = true;
            g_avoid.readings = g_front.readings;
            g_avoid.misses = 0U;
            bench_line(BK_AV_CREEP);
            bench_text(bench_flag(g_avoid.creep_side > 0) ?
                       "AVOID: creeping out to find its right edge, the sonar"
                       " square to its face" :
                       "AVOID: creeping out to find its left edge, the sonar"
                       " square to its face");
            bench_avoid_say();
        }
    }
    else if (BENCH_AVOID_CREEPBACK == g_avoid.phase)
    {
        /* Back along the far creep's diagonal (the heading hold works
         * backwards too) until the car is as far out as when it began, then
         * creep to the way round's edge. At most BENCH_AVOID_BACKUP_MM back,
         * in case the heading drifted.
         */
        bench_avoid_hold(g_avoid.yaw_home -
                         ((double)g_avoid.creep_side * creep_rad), p_request);
        p_request->forward_mps = -BENCH_AVOID_CREEP_MPS;
        if ((((double)g_avoid.creep_side *
              (g_avoid.right_mm - g_avoid.far_start_mm)) <= 0.0) ||
            ((g_avoid.back_mm - g_avoid.far_back_mm) >= BENCH_AVOID_BACKUP_MM))
        {
            p_request->forward_mps = 0.0;
            p_request->yaw_rps = 0.0;
            g_avoid.creep_side = g_avoid.side;
            g_avoid.b_far = false;
            g_avoid.backups = g_avoid.far_backups;
            g_avoid.creep_deg = BENCH_AVOID_CREEP_START_DEG;
            g_avoid.creep_hits = 0U;
            g_avoid.creep_none_mm = 0.0;
            g_avoid.misses = 0U;
            g_avoid.phase = BENCH_AVOID_CREEPTURN;
            bench_line(BK_AV_FAR_BACK);   /* 0-4095 mm, 0-127 deg */
            bench_text("AVOID: back from the far edge (");
            bench_u((uint32_t)bench_round(fabs(g_avoid.right_mm -
                                       g_avoid.far_start_mm)), 12U);
            bench_text(" mm off the start), turning ");
            bench_u((uint32_t)BENCH_AVOID_CREEP_START_DEG, 7U);
            bench_text(" deg to creep out to the way round's edge");
            bench_avoid_say();
        }
    }
    else if (BENCH_AVOID_CREEP == g_avoid.phase)
    {
        bench_avoid_creep(now_us, turned, p_request);
    }
    else if (BENCH_AVOID_OUT == g_avoid.phase)
    {
        if (bench_avoid_pivot(g_avoid.yaw_home - side_rad, p_request))
        {
            g_avoid.phase = BENCH_AVOID_STEPOUT;
            bench_line(BK_AV_STEP_OUT);
            bench_text(bench_flag(g_avoid.side > 0) ?
                       "AVOID: stepping out to the right" :
                                            "AVOID: stepping out to the left");
            bench_avoid_say();
        }
    }
    else if (BENCH_AVOID_STEPOUT == g_avoid.phase)
    {
        bench_avoid_hold(g_avoid.yaw_home - side_rad, p_request);
        if (((double)g_avoid.side * g_avoid.right_mm) >= g_avoid.side_mm)
        {
            g_avoid.phase = BENCH_AVOID_ALIGN;
            p_request->forward_mps = 0.0;
            p_request->yaw_rps = 0.0;
            (void)bench_scan_servo(0);
            bench_line(BK_AV_CLEAR);
            bench_text("AVOID: clear of its edge, turning back to the line"
                       " heading");
            bench_avoid_say();
        }
    }
    else if (BENCH_AVOID_ALIGN == g_avoid.phase)
    {
        if (bench_avoid_pivot(g_avoid.yaw_home, p_request))
        {
            g_avoid.phase = BENCH_AVOID_ALONG;
            g_avoid.guard_hits = 0U;
            bench_line(BK_AV_ALONG);   /* +-4095 mm */
            bench_text("AVOID: alongside the object for ");
            bench_i(bench_round(g_avoid.face_mm + BENCH_AVOID_DEPTH_MM +
                                  half + BENCH_AVOID_CLEAR_MM +
                                  g_avoid.extra_mm + g_avoid.back_mm), 13U);
            bench_text(" mm");
            bench_avoid_say();
        }
    }
    else if (BENCH_AVOID_ALONG == g_avoid.phase)
    {
        /* parallel to the line */
        bench_avoid_hold(g_avoid.yaw_home, p_request);
        /* By position, not distance from here: the pivot ends half the car +
         * the clearance past the obstacle's back, wherever the creep left it.
         */
        if (bench_avoid_guard(b_range, p_request))
        {
            bench_line(BK_AV_AHEAD);   /* sonar 0-4095 mm */
            bench_text("AVOID: something ahead at ");
            bench_u((uint32_t)g_front.last_mm, 12U);
            bench_text(" mm alongside the object, stopped");
            bench_avoid_say();
            bench_line_end(now_us, "stopped: the way along was blocked");
            (void)memset(p_request, 0, sizeof(*p_request));
            p_request->kind = MOTION_HALT;
        }
        else if ((-g_avoid.back_mm) >= (g_avoid.face_mm + BENCH_AVOID_DEPTH_MM +
                                        half + BENCH_AVOID_CLEAR_MM +
                                        g_avoid.extra_mm))
        {
            g_avoid.phase = BENCH_AVOID_BACKPIVOT;
            g_avoid.guard_hits = 0U;
            p_request->forward_mps = 0.0;
            p_request->yaw_rps = 0.0;
            bench_line(BK_AV_PAST);
            bench_text("AVOID: past the object, stepping back to the line");
            bench_avoid_say();
        }
    }
    else if (BENCH_AVOID_BACKPIVOT == g_avoid.phase)
    {
        if (bench_avoid_guard(b_range, p_request))
        {
            bench_avoid_blocked(now_us, half, p_request);
        }
        else if (!bench_avoid_pivot(g_avoid.yaw_home + side_rad, p_request))
        {
            /* Still turning toward the line. */
        }
        else if (g_avoid.b_blocked)
        {
            /* Clear now, blocked at the last look: its end lies between the
             * two, possibly beside the flank. Run the margin past it first.
             */
            g_avoid.b_blocked = false;
            g_avoid.extra_mm += half + BENCH_AVOID_CLEAR_MM;
            g_avoid.phase = BENCH_AVOID_ALIGN;
            p_request->yaw_rps = 0.0;
            bench_line(BK_AV_WAY_CLEAR);   /* +-4095 mm */
            bench_text("AVOID: the way back looks clear, going on ");
            bench_i(bench_round(half + BENCH_AVOID_CLEAR_MM), 13U);
            bench_text(" mm more to clear the object's unseen end");
            bench_avoid_say();
        }
        else
        {
            g_avoid.phase = BENCH_AVOID_STEPIN;
            g_avoid.guard_hits = 0U;
            g_avoid.ir_peak[0] = 0U;
            g_avoid.ir_peak[1] = 0U;
            g_avoid.b_dark_on = true;
            g_avoid.b_dark_known = false;
            g_avoid.ir_dark[0] = true;
            g_avoid.ir_dark[1] = true;
            g_avoid.sx_known[0] = false;
            g_avoid.sx_known[1] = false;
            g_avoid.sx_done[0] = false;
            g_avoid.sx_done[1] = false;
            g_avoid.cross_logs = 0U;
            bench_line(BK_AV_RETURN);
            bench_text("AVOID: returning to the line");
            bench_avoid_say();
        }
    }
    else if (BENCH_AVOID_STEPIN == g_avoid.phase)
    {
        bench_avoid_hold(g_avoid.yaw_home + side_rad, p_request);
        if (bench_avoid_guard(b_range, p_request))
        {
            bench_avoid_blocked(now_us, half, p_request);
        }
        else
        {
            (void)bench_avoid_line_fix();
        }
        if ((BENCH_AVOID_STEPIN == g_avoid.phase) && (!g_drive.b_done) &&
            (((double)g_avoid.side * g_avoid.right_mm) <= 0.0))
        {
            double aim = 0.0;

            g_avoid.phase = BENCH_AVOID_HOMEPIVOT;
            p_request->forward_mps = 0.0;
            p_request->yaw_rps = 0.0;
            if (g_avoid.b_line_err)
            {
                /* Square to the line now, turned line_err off it: the line
                 * heading is a quarter turn back from here, less that. */
                aim = bench_avoid_wrap(g_drive.heading - side_rad -
                                       g_avoid.line_err_rad -
                                       g_avoid.yaw_home);
                g_avoid.yaw_home = bench_avoid_wrap(g_avoid.yaw_home + aim);
            }
            bench_line(BK_AV_ON_LINE);
            bench_text("AVOID: on the line, straightening");
            bench_avoid_peak_say();
            if (bench_flag(g_avoid.b_line_err))
            {
                bench_text(", line heading moved ");
                bench_deg(aim);
            }
            bench_avoid_say();
            bench_avoid_turn_begin(g_avoid.yaw_home);
            if ((!g_avoid.b_line_fix) &&
                (g_avoid.ir_dark[0] || g_avoid.ir_dark[1]))
            {
                /* Came in along a line (a bend's far leg or a junction's
                 * branch): this heading is a candidate. */
                g_avoid.b_turn_seen = true;
                g_avoid.b_turn_along = true;
                g_avoid.turn_seen_rad = bench_avoid_wrap(g_drive.heading);
                bench_line(BK_AV_ALONG_LINE);
                bench_text("AVOID: came in along a line (");
                bench_text(bench_flag(g_avoid.ir_dark[0]) ? "L" : "");
                bench_text(bench_flag(g_avoid.ir_dark[1]) ? "R" : "");
                bench_text(" dark): kept as a candidate");
                bench_avoid_say();
            }
        }
    }
    else /* BENCH_AVOID_HOMEPIVOT */
    {
        if (bench_avoid_turn_check(g_avoid.yaw_home, p_request))
        {
            if (g_avoid.b_turn_found)
            {
                /* The line as the sensors found it (a bend or a junction
                 * branch may lie far off the original heading). */
                g_avoid.yaw_home = g_avoid.turn_line_rad;
            }
            g_avoid.phase = BENCH_AVOID_STRAIGHT;
            (void)bench_scan_servo(0);
            bench_avoid_hold(g_avoid.yaw_home, p_request);
            bench_line(BK_AV_ON_COURSE);
            bench_text("AVOID: back on course, driving straight");
            bench_avoid_say();
        }
    }
    g_drive.request = *p_request;
}

/** @brief AVOID: stopped in front of the obstacle; the go-around starts
 * from here (yaw_home is the caller's). The map is cleared and fixed here,
 * and this range, the first beam, gives the obstacle's face.
 */
void bench_avoid_begin (uint64_t now_us, int32_t centre)
{
    double angle = bench_avoid_wrap(g_drive.heading - g_avoid.yaw_home);

    g_avoid.phase = BENCH_AVOID_PAUSE;
    g_avoid.right_mm = 0.0;
    g_avoid.last_centre = centre;
    g_avoid.side = 1;
    g_avoid.left_mm = 0.0;
    g_avoid.right_edge_mm = 0.0;
    g_avoid.back_mm = 0.0;
    g_avoid.back_from_mm = 0.0;
    g_avoid.x0_mm = g_drive.x_m * 1000.0;
    g_avoid.y0_mm = g_drive.y_m * 1000.0;
    g_avoid.pause_us = now_us;
    g_avoid.misses = 0U;
    g_avoid.close = 0U;
    g_avoid.backups = 0U;
    g_avoid.b_hit = false;
    g_avoid.b_left_open = false;
    g_avoid.b_right_open = false;
    g_avoid.b_creeping = false;
    g_avoid.miss_mm = 0.0;
    g_avoid.hit_mm = 0.0;
    g_avoid.hit_range = 0U;
    g_avoid.creep_hits = 0U;
    g_avoid.creep_deg = BENCH_AVOID_CREEP_START_DEG;
    g_avoid.creep_none_mm = 0.0;
    g_avoid.creep_side = 1;
    g_avoid.b_far = false;
    g_avoid.far_start_mm = 0.0;
    g_avoid.far_back_mm = 0.0;
    g_avoid.far_backups = 0U;
    g_avoid.b_left_creep = false;
    g_avoid.b_right_creep = false;
    g_avoid.b_line_fix = false;
    g_avoid.b_dark_on = false;
    g_avoid.b_dark_known = false;
    g_avoid.ir_dark[0] = false;
    g_avoid.ir_dark[1] = false;
    g_avoid.dark_from = 0.0;
    g_avoid.b_line_err = false;
    g_avoid.line_err_rad = 0.0;
    g_avoid.guard_hits = 0U;
    g_avoid.extends = 0U;
    g_avoid.extra_mm = 0.0;
    g_avoid.b_blocked = false;
    g_avoid.turn_sum_rad = 0.0;
    g_avoid.turn_finds = 0U;
    bench_avoid_turn_begin(g_avoid.yaw_home);
    bench_avoid_ir_setup();
    (void)memset(g_avoid.map, 0, sizeof(g_avoid.map));
    g_beams = 0U;
    g_beams_sent = 0U;
    g_beams_dropped = 0U;
    g_avoid.face_mm = (BENCH_AVOID_SONAR_MM + (double)g_front.last_mm) *
                      cos(angle);
    bench_avoid_keep_beam(angle, g_front.last_mm);
    (void)bench_avoid_beam(angle, angle, g_front.last_mm);
}

/** @brief AVOID, ALONG and STEPIN: the path guard. True once
 * BENCH_AVOID_CONFIRM new VALID echoes in a row are within
 * BENCH_AVOID_GUARD_MM (the servo is centred from ALIGN on, so the sonar
 * faces along the path); the move is zeroed then.
 */
bool bench_avoid_guard (bool b_range, motion_request_t * p_request)
{
    if (!b_range)
    {
        return false;
    }
    g_avoid.guard_hits = ((0U != g_front.last_mm) &&
                          ((double)g_front.last_mm <= BENCH_AVOID_GUARD_MM)) ?
                         (g_avoid.guard_hits + 1U) : 0U;
    if (g_avoid.guard_hits < BENCH_AVOID_CONFIRM)
    {
        return false;
    }
    g_avoid.guard_hits = 0U;
    p_request->forward_mps = 0.0;
    p_request->yaw_rps = 0.0;
    return true;
}

/** @brief AVOID, BACKPIVOT and STEPIN: the guard saw something in the way
 * back to the line. Turn back parallel (ALIGN) and run on by half the car +
 * BENCH_AVOID_CLEAR_MM, or stop once BENCH_AVOID_EXTENDS retries are spent.
 */
void bench_avoid_blocked (uint64_t now_us, double half,
                                 motion_request_t * p_request)
{
    double step_mm = half + BENCH_AVOID_CLEAR_MM;

    g_avoid.b_blocked = true;
    if (g_avoid.extends < BENCH_AVOID_EXTENDS)
    {
        ++g_avoid.extends;
        g_avoid.extra_mm += step_mm;
        g_avoid.phase = BENCH_AVOID_ALIGN;
        bench_line(BK_AV_STILL_WAY);   /* 0-4095 mm, +-4095 mm */
        bench_text("AVOID: the object is still in the way at ");
        bench_u((uint32_t)g_front.last_mm, 12U);
        bench_text(" mm, going on along it by ");
        bench_i(bench_round(step_mm), 13U);
        bench_text(" mm");
        bench_avoid_say();
    }
    else
    {
        bench_line(BK_AV_BLOCKED);
        bench_text("AVOID: the way back to the line is still blocked at ");
        bench_u((uint32_t)g_front.last_mm, 12U);
        bench_text(" mm, stopped");
        bench_avoid_say();
        bench_line_end(now_us, "stopped: the way back to the line was blocked");
        (void)memset(p_request, 0, sizeof(*p_request));
        p_request->kind = MOTION_HALT;
    }
}

/** @brief MISSION, STEPIN: the line fix. Each front sensor's class follows
 * the go-around's hysteresis (bench_avoid_ir_setup; in the band it keeps the
 * last class), and its dark stretches are timed in travel toward the line.
 * A stretch counts when its start was seen in this STEPIN (STEPIN begins as
 * if inside one) and it lasted BENCH_AVOID_LINE_MM / 2 to 1.5 x
 * BENCH_AVOID_LINE_MM. When both sensors have one, their middles within the
 * sensor spacing x tan(BENCH_AVOID_SQUARE_DEG) of each other, that is the
 * line, wherever it lies (3 Nov: the go-around came back 30 mm and more from
 * where the wheels put the line, outside the old BENCH_AVOID_LINE_WIN_MM):
 * the sideways offset is reset from the middles' mean, the pivot then
 * BENCH_AVOID_FRONT_MM short of the line, and the car's angle to the line
 * kept from their difference for HOMEPIVOT. The right sensor crossing first
 * means the car is turned left of square (+): atan(difference / spacing).
 * A card's bar beside the line is run along its length on the way in: dark
 * far longer than the tape is wide, or under one sensor only. Stretches not
 * taken are logged (BENCH_AVOID_CROSS_LOGS). Once per go-around. True when
 * it fixed.
 */
bool bench_avoid_line_fix (void)
{
    static const char * const names[2] = {"L", "R"};
    uint16_t raw[LINE_CTRL_CHANNELS] = {0U, 0U, 0U};
    double   expect = (double)g_avoid.side * g_avoid.right_mm;
    double   spacing = ROBOT_LINE_SENSOR_SPACING_M * 1000.0;
    double   apart_max = spacing *
                         tan(BENCH_AVOID_SQUARE_DEG * CORE_PI / 180.0);
    double   apart;
    double   mid_mm;
    uint32_t channel;

    if ((BENCH_MODE_MISSION != g_drive.mode) || g_avoid.b_line_fix ||
        (!board_read_ir(raw)))
    {
        return false;
    }
    bench_avoid_ir_peak(raw);
    for (channel = 0U; channel < 2U; ++channel)
    {
        line_ctrl_class_t cls = line_ctrl_classify(&g_avoid.ir_hyst[channel],
                                                   raw[channel]);

        if ((LINE_CTRL_DARK == cls) && (!g_avoid.ir_dark[channel]))
        {
            g_avoid.ir_dark[channel] = true;
            g_avoid.sx_known[channel] = true;
            g_avoid.sx_from[channel] = expect;
        }
        else if ((LINE_CTRL_LIGHT == cls) && g_avoid.ir_dark[channel])
        {
            double width = g_avoid.sx_from[channel] - expect;

            g_avoid.ir_dark[channel] = false;
            if (g_avoid.sx_known[channel] &&
                (width >= (BENCH_AVOID_LINE_MM / 2.0)) &&
                (width <= (1.5 * BENCH_AVOID_LINE_MM)))
            {
                g_avoid.sx_done[channel] = true;
                g_avoid.sx_mid[channel] = (g_avoid.sx_from[channel] +
                                           expect) / 2.0;
                g_avoid.sx_width[channel] = width;
            }
            else if (g_avoid.sx_known[channel] &&
                     (g_avoid.cross_logs < BENCH_AVOID_CROSS_LOGS))
            {
                ++g_avoid.cross_logs;
                bench_line(BK_AV_WAY_IN);   /* +-4095 mm each */
                bench_text("AVOID: way in, ");
                bench_name(names, channel, 1U);
                bench_text(" dark ");
                bench_code(0U, 1U);   /* 0: not tape-wide */
                bench_i(bench_round(width), 13U);
                bench_text(" mm at ");
                bench_i(bench_round(((g_avoid.sx_from[channel] + expect) /
                                       2.0) - BENCH_AVOID_FRONT_MM), 13U);
                bench_text(" mm off the wheels' line: not tape-wide");
                bench_avoid_say();
            }
            else
            {
                /* Start unseen, or the log is full. */
            }
            g_avoid.sx_known[channel] = false;
        }
        else
        {
            /* No change, in the band, or invalid: keep the class. */
        }
        if (g_avoid.sx_done[channel] &&
            ((g_avoid.sx_mid[channel] - expect) >
             ((1.5 * BENCH_AVOID_LINE_MM) + apart_max)))
        {
            /* The other sensor never crossed it: not the line. */
            g_avoid.sx_done[channel] = false;
            if (g_avoid.cross_logs < BENCH_AVOID_CROSS_LOGS)
            {
                ++g_avoid.cross_logs;
                bench_line(BK_AV_WAY_IN);   /* +-4095 mm each */
                bench_text("AVOID: way in, ");
                bench_name(names, channel, 1U);
                bench_text(" dark ");
                bench_code(1U, 1U);   /* 1: that sensor only */
                bench_i(bench_round(g_avoid.sx_width[channel]), 13U);
                bench_text(" mm at ");
                bench_i(bench_round(g_avoid.sx_mid[channel] -
                                      BENCH_AVOID_FRONT_MM), 13U);
                bench_text(" mm off the wheels' line: that sensor only");
                bench_avoid_say();
            }
        }
    }
    if (!(g_avoid.sx_done[0] && g_avoid.sx_done[1]))
    {
        return false;
    }
    apart = g_avoid.sx_mid[1] - g_avoid.sx_mid[0]; /* +: right crossed first */
    mid_mm = (g_avoid.sx_mid[0] + g_avoid.sx_mid[1]) / 2.0;
    if (fabs(apart) > apart_max)
    {
        g_avoid.sx_done[0] = false;
        g_avoid.sx_done[1] = false;
        if (g_avoid.cross_logs < BENCH_AVOID_CROSS_LOGS)
        {
            ++g_avoid.cross_logs;
            bench_line(BK_AV_WAY_IN_BOTH);   /* +-4095 mm each */
            bench_text("AVOID: way in, L and R dark ");
            bench_i(bench_round(fabs(apart)), 13U);
            bench_text(" mm apart at ");
            bench_i(bench_round(mid_mm - BENCH_AVOID_FRONT_MM), 13U);
            bench_text(" mm off the wheels' line: over 30 deg");
            bench_avoid_say();
        }
        return false;
    }
    g_avoid.b_line_fix = true;
    g_avoid.b_line_err = true;
    g_avoid.line_err_rad = atan2(apart, spacing);
    g_avoid.right_mm = (double)g_avoid.side *
                       (expect - (mid_mm - BENCH_AVOID_FRONT_MM));
    bench_line(BK_AV_CROSSED);   /* +-4095 mm, a bench_deg angle */
    bench_text("AVOID: line crossed ");
    bench_i(bench_round(mid_mm - BENCH_AVOID_FRONT_MM), 13U);
    bench_text(" mm off the wheels' guess, car at ");
    bench_deg(g_avoid.line_err_rad);
    bench_text(" deg to it (dark L ");
    bench_i(bench_round(g_avoid.sx_width[0]), 13U);
    bench_text(" R ");
    bench_i(bench_round(g_avoid.sx_width[1]), 13U);
    bench_text(" mm)");
    bench_avoid_say();
    return true;
}

/** @brief MISSION: the go-around is back on course (phase STRAIGHT again)
 * and the line is followed from the next step, as after AUTO-CENTRE.
 * line_ctrl was not stepped while the car went round, so its next step would
 * see a gap far past its maximum_gap_ms and fault: the car stopped with "line
 * sensor fault" one step after "back on course" (2 Nov). It starts afresh.
 * LINE centre's per-sample history starts afresh too, or its first sample
 * would take the whole go-around as travel along the line (a gap, a crossing
 * or all three dark, at once): the encoder centre and time it last saw, its
 * gap, crossing and dark travel, and its rate-limited steering, now straight
 * (the car faces the line heading). The run counters stay, and so does the
 * side the line was last seen on: where the car lands after the go-around is
 * known no better.
 */
void bench_avoid_rejoin (uint64_t now_us, int32_t centre)
{
    line_ctrl_init(&g_line_state);
    g_centre.last_counts = centre;
    g_centre.last_us = now_us;
    g_centre.gap_m = 0.0;
    g_centre.clear_m = 0.0;
    g_centre.b_seen = false;
    g_centre.b_crossing = false;
    g_centre.b_searching = false;
    g_centre.seen_side = 0;
    g_centre.on_m = 0.0;
    g_centre.mag = 0.0;
    g_centre.cmd = 0.0;
    g_centre.dark_m[0] = 0.0;
    g_centre.dark_m[1] = 0.0;
    g_centre.edge_m[0] = 0.0;
    g_centre.edge_m[1] = 0.0;
    g_centre.ring_n = 0U;
    g_centre.ring_at_m = g_centre.path_m;
    g_centre.all_dark_m = 0.0;
    g_centre.turn = 0.0;
    g_centre.b_both_dark = false;
    g_line.b_crossing = false;
}

/** @brief AVOID: an angle difference (rad) the short way round, -pi..pi.
 * The pose heading is kept within -pi..pi (core.c), so a go-around met
 * facing more than 90 deg from START has targets past +-pi (yaw_home +-
 * 90 deg): unwrapped, a pivot toward one never got within its tolerance
 * and the car spun in circles at the obstacle, and the 80 deg measuring
 * limit never came (3 Nov). Every heading difference here goes through it.
 */
double bench_avoid_wrap (double angle)
{
    return remainder(angle, 2.0 * CORE_PI);
}

/** @brief AVOID: pivot in place toward target (rad, + left) at the turn
 * rate. True, with no turn requested, once within BENCH_AVOID_PIVOT_TOL_DEG;
 * one tick's turn is under twice that, so it settles without hunting.
 */
bool bench_avoid_pivot (double target, motion_request_t * p_request)
{
    double error = bench_avoid_wrap(target - g_drive.heading);
    double tol = BENCH_AVOID_PIVOT_TOL_DEG * CORE_PI / 180.0;

    if (error > tol)
    {
        p_request->yaw_rps = ROBOT_TURN_RATE_RPS;
        return false;
    }
    if (error < -tol)
    {
        p_request->yaw_rps = -ROBOT_TURN_RATE_RPS;
        return false;
    }
    return true;
}

/** @brief AVOID HOMEPIVOT: pivot toward target as bench_avoid_pivot, but
 * the way that stays within half a turn of home: the error is taken between
 * the two headings as measured from home, not the short way round, so the
 * car never swings through facing back the way it came, toward the obstacle
 * (creep sim, a bend 5 cm past the box: the short way back to a candidate
 * swept the car's front within 1 mm of it).
 */
bool bench_avoid_pivot_from (double home, double target,
                                    motion_request_t * p_request)
{
    double error = bench_avoid_wrap(target - home) -
                   bench_avoid_wrap(g_drive.heading - home);
    double tol = BENCH_AVOID_PIVOT_TOL_DEG * CORE_PI / 180.0;

    if (error > tol)
    {
        p_request->yaw_rps = ROBOT_TURN_RATE_RPS;
        return false;
    }
    if (error < -tol)
    {
        p_request->yaw_rps = -ROBOT_TURN_RATE_RPS;
        return false;
    }
    return true;
}

/** @brief AVOID turn check: a pivot toward target starts. Each front sensor
 * starts as if inside a dark stretch whose start went unseen (what is under
 * it now is never taken for the line), and the turn's direction is kept.
 */
void bench_avoid_turn_begin (double target)
{
    uint32_t channel;

    for (channel = 0U; channel < 2U; ++channel)
    {
        g_avoid.turn_dark[channel] = true;
        g_avoid.turn_known[channel] = false;
        g_avoid.turn_from[channel] = g_drive.heading;
    }
    g_avoid.turn_dir =
        (bench_avoid_wrap(target - g_drive.heading) >= 0.0) ? 1 : -1;
    g_avoid.b_turn_found = false;
    g_avoid.b_turn_given_up = false;
    g_avoid.turn_line_rad = target;
    g_avoid.b_turn_search = false;
    g_avoid.b_turn_seen = false;
    g_avoid.turn_seen_rad = target;
    g_avoid.b_turn_along = false;
    g_avoid.turn_width_rad = 0.0;
    g_avoid.ir_peak[0] = 0U;
    g_avoid.ir_peak[1] = 0U;
}

/** @brief AVOID turn check, each tick of a pivot: each front sensor's class
 * follows the go-around's hysteresis (bench_avoid_ir_setup; a reading in
 * the band keeps the last class). When a sensor clears after a dark stretch
 * whose start was
 * seen, of half to twice BENCH_TURN_LINE_RAD of turn, the heading that puts
 * the line midway between the sensors is its middle plus the sensor's
 * bearing; within window of target that is the line: true, the heading in
 * *p_line_rad. One further out, within BENCH_AVOID_SEARCH_DEG, is kept if it
 * is the nearest the target so far (turn_seen_rad, for HOMEPIVOT's search).
 * MISSION only.
 */
bool bench_avoid_turn_line (double target, double window,
                                   double * p_line_rad)
{
    uint16_t raw[LINE_CTRL_CHANNELS] = {0U, 0U, 0U};
    double   bearing = atan2(ROBOT_LINE_SENSOR_SPACING_M * 500.0,
                             BENCH_AVOID_FRONT_MM); /* half the spacing, mm */
    bool     b_found = false;
    uint32_t channel;

    if ((BENCH_MODE_MISSION != g_drive.mode) || (!board_read_ir(raw)))
    {
        return false;
    }
    bench_avoid_ir_peak(raw);
    for (channel = 0U; channel < 2U; ++channel)
    {
        line_ctrl_class_t cls = line_ctrl_classify(&g_avoid.ir_hyst[channel],
                                                   raw[channel]);
        double swept;
        double line_rad;

        if ((LINE_CTRL_DARK == cls) && (!g_avoid.turn_dark[channel]))
        {
            g_avoid.turn_dark[channel] = true;
            g_avoid.turn_known[channel] = true;
            g_avoid.turn_from[channel] = g_drive.heading;
        }
        else if ((LINE_CTRL_LIGHT == cls) && g_avoid.turn_dark[channel])
        {
            g_avoid.turn_dark[channel] = false;
            swept = bench_avoid_wrap(g_drive.heading -
                                     g_avoid.turn_from[channel]);
            /* Channel 0 is the left sensor: its bearing is + (left). */
            line_rad = g_avoid.turn_from[channel] + (swept / 2.0) +
                       ((0U == channel) ? bearing : -bearing);
            if (g_avoid.turn_known[channel] && (!b_found) &&
                (fabs(swept) >= (0.5 * BENCH_TURN_LINE_RAD)) &&
                (fabs(swept) <= (2.0 * BENCH_TURN_LINE_RAD)))
            {
                double off = fabs(bench_avoid_wrap(line_rad - target));

                if (off <= window)
                {
                    b_found = true;
                    g_avoid.turn_width_rad = fabs(swept);
                    *p_line_rad = bench_avoid_wrap(line_rad);
                }
                else if ((off <= ((BENCH_AVOID_SEARCH_DEG +
                                   BENCH_TURN_CHECK_DEG) * CORE_PI / 180.0)) &&
                         ((!g_avoid.b_turn_seen) ||
                          (off < fabs(bench_avoid_wrap(g_avoid.turn_seen_rad -
                                                       target)))))
                {
                    g_avoid.b_turn_seen = true;
                    g_avoid.turn_seen_rad = bench_avoid_wrap(line_rad);
                    g_avoid.b_turn_along = false;
                }
                else
                {
                    /* Further from the target than one already kept. */
                }
            }
            g_avoid.turn_known[channel] = false;
        }
        else
        {
            /* No change, in the band, or invalid: keep the class. */
        }
    }
    return b_found;
}

/** @brief AVOID HOMEPIVOT: pivot toward target (the line heading by the
 * wheels) and on past it up to BENCH_TURN_CHECK_DEG until a front sensor
 * crosses the line (bench_avoid_turn_line), then onto the heading that
 * centres the line between the sensors. With no line in that reach, on past
 * it for lines up to BENCH_AVOID_SEARCH_DEG + BENCH_TURN_CHECK_DEG off (two
 * line widths more, so a crossing there ends), but not beyond a candidate
 * already kept (turn_seen_rad: swept on the way to the target, or the line
 * STEPIN ended on): the line nearer the target wins. With no line at all,
 * back to target by the wheels. Outside MISSION a plain pivot. True when
 * done.
 */
bool bench_avoid_turn_check (double target,
                                    motion_request_t * p_request)
{
    double window = BENCH_TURN_CHECK_DEG * CORE_PI / 180.0;
    double reach = (BENCH_AVOID_SEARCH_DEG + BENCH_TURN_CHECK_DEG) *
                   CORE_PI / 180.0;
    double line_rad = 0.0;
    double limit = reach + (2.0 * BENCH_TURN_LINE_RAD);
    double past;

    if ((BENCH_MODE_MISSION != g_drive.mode) || g_avoid.b_turn_given_up)
    {
        return bench_avoid_pivot(target, p_request);
    }
    if (g_avoid.b_turn_found)
    {
        return bench_avoid_pivot_from(target, g_avoid.turn_line_rad,
                                      p_request);
    }
    if (bench_avoid_turn_line(target, g_avoid.b_turn_search ? reach : window,
                              &line_rad))
    {
        g_avoid.b_turn_found = true;
        g_avoid.turn_line_rad = line_rad;
        bench_line(BK_AV_TURN_LINE);   /* bench_deg angles */
        bench_text("AVOID: turn check: line ");
        bench_deg(bench_avoid_wrap(line_rad - target));
        bench_text(" deg off the wheels' turn (dark ");
        bench_deg(g_avoid.turn_width_rad);
        bench_text("), turning onto it");
        bench_mag_say();
        bench_avoid_say();
        return bench_avoid_pivot_from(target, line_rad, p_request);
    }
    /* - while short of target, + once past it, in the turn's direction. */
    past = -(double)g_avoid.turn_dir *
           bench_avoid_wrap(target - g_drive.heading);
    if ((!g_avoid.b_turn_search) && (past >= window))
    {
        g_avoid.b_turn_search = true;
        bench_line(BK_AV_TURN_NONE);   /* 0-255 deg */
        bench_text("AVOID: turn check: no line within ");
        bench_code(0U, 1U);   /* 0: looking on */
        bench_u((uint32_t)BENCH_TURN_CHECK_DEG, 8U);
        bench_text(" deg, looking on");
        bench_avoid_peak_say();
        bench_avoid_say();
    }
    if (g_avoid.b_turn_seen)
    {
        limit = fmin(limit, fabs(bench_avoid_wrap(g_avoid.turn_seen_rad -
                                                  target)) +
                            (2.0 * BENCH_TURN_LINE_RAD));
    }
    if (g_avoid.b_turn_search && (past >= limit))
    {
        if (g_avoid.b_turn_seen)
        {
            g_avoid.b_turn_found = true;
            g_avoid.turn_line_rad = g_avoid.turn_seen_rad;
            bench_line(BK_AV_TURN_BACK);   /* a bench_deg angle */
            bench_text(bench_flag(g_avoid.b_turn_along) ?
                       "AVOID: turn check: back to the line it came in"
                       " along, " :
                       "AVOID: turn check: back to the line crossed at ");
            bench_deg(bench_avoid_wrap(g_avoid.turn_seen_rad - target));
            bench_text(" deg off the wheels' turn");
            bench_mag_say();
            bench_avoid_say();
            return bench_avoid_pivot_from(target, g_avoid.turn_seen_rad,
                                          p_request);
        }
        g_avoid.b_turn_given_up = true;
        bench_line(BK_AV_TURN_NONE);   /* 0-255 deg */
        bench_text("AVOID: turn check: no line within ");
        bench_code(1U, 1U);   /* 1: given up, by the wheels */
        bench_u((uint32_t)(BENCH_AVOID_SEARCH_DEG + BENCH_TURN_CHECK_DEG), 8U);
        bench_text(" deg, by the wheels");
        bench_avoid_peak_say();
        bench_mag_say();
        bench_avoid_say();
        return bench_avoid_pivot(target, p_request);
    }
    p_request->yaw_rps = (double)g_avoid.turn_dir * ROBOT_TURN_RATE_RPS;
    return false;
}

/** @brief AVOID, end of MEASURE_R: the front sensors crossed the line the
 * car stopped on (bench_avoid_turn_line, within BENCH_TURN_CHECK_DEG of
 * yaw_home); yaw_home moves to the mean heading found, so every later turn
 * aims at the line as the sensors saw it, not as the wheels counted it.
 * MISSION only.
 */
void bench_avoid_home_fix (void)
{
    double change;

    if (BENCH_MODE_MISSION != g_drive.mode)
    {
        return; /* no line calibration: nothing was looked for */
    }
    if (0U == g_avoid.turn_finds)
    {
        bench_line(BK_AV_HOME_NONE);
        bench_text("AVOID: measuring turns checked: line not seen, heading"
                   " by the wheels");
        bench_mag_say();
        bench_avoid_say();
        return;
    }
    change = g_avoid.turn_sum_rad / (double)g_avoid.turn_finds;
    g_avoid.yaw_home = bench_avoid_wrap(g_avoid.yaw_home + change);
    bench_line(BK_AV_HOME);   /* a bench_deg angle, 0-255 finds */
    bench_text("AVOID: measuring turns checked: line ");
    bench_deg(change);
    bench_text(" deg off the wheels' heading (");
    bench_u(g_avoid.turn_finds, 8U);
    bench_text(" seen), reset");
    bench_mag_say();
    bench_avoid_say();
}

/** @brief AVOID, MISSION, at each obstacle: the line checks' dark/light
 * thresholds per front sensor, from this run's own readings (the START
 * calibration's lowest and highest, g_cal, over AUTO-CENTRE and the line so
 * far) at BENCH_AVOID_IR_ON and BENCH_AVOID_IR_OFF of the span. A sensor
 * whose readings span under a quarter of its configured white-to-black (it
 * has not crossed the tape) uses the configured white and black. The
 * configured black (sensor alone on tape, 2 Nov) reads higher than the tape
 * passing under the moving car.
 */
void bench_avoid_ir_setup (void)
{
    static const uint16_t white[2] = {ROBOT_IR_LEFT_WHITE,
                                      ROBOT_IR_RIGHT_WHITE};
    static const uint16_t black[2] = {ROBOT_IR_LEFT_BLACK,
                                      ROBOT_IR_RIGHT_BLACK};
    line_ctrl_hysteresis_t own;
    uint32_t               channel;

    for (channel = 0U; channel < 2U; ++channel)
    {
        uint32_t span = (black[channel] > white[channel]) ?
                        (uint32_t)(black[channel] - white[channel]) :
                        (uint32_t)(white[channel] - black[channel]);

        g_avoid.ir_own[channel] = false;
        if (!line_ctrl_hysteresis(white[channel], black[channel],
                                  BENCH_AVOID_IR_ON, BENCH_AVOID_IR_OFF,
                                  &g_avoid.ir_hyst[channel]))
        {
            g_avoid.ir_hyst[channel] =
                g_line_runtime.channel[channel].hysteresis;
        }
        if ((BENCH_MODE_MISSION == g_drive.mode) && (0U != g_cal.count) &&
            (g_cal.high[channel] > g_cal.low[channel]) &&
            ((uint32_t)(g_cal.high[channel] - g_cal.low[channel]) >=
             (span / 4U)) &&
            line_ctrl_hysteresis(g_cal.low[channel], g_cal.high[channel],
                                 BENCH_AVOID_IR_ON, BENCH_AVOID_IR_OFF, &own))
        {
            g_avoid.ir_hyst[channel] = own;
            g_avoid.ir_own[channel] = true;
        }
    }
}

/** @brief AVOID, MISSION: the line checks' dark thresholds, one line. */
void bench_avoid_ir_say (void)
{
    static const char * const names[2] = {"L ", " R "};
    uint32_t channel;

    if (BENCH_MODE_MISSION != g_drive.mode)
    {
        return;
    }
    bench_line(BK_AV_IR);   /* ADC 0-4095 */
    bench_text("AVOID: line checks: dark from ");
    for (channel = 0U; channel < 2U; ++channel)
    {
        bench_text(names[channel]);
        bench_u((uint32_t)g_avoid.ir_hyst[channel].on_adc, 12U);
        bench_text(bench_flag(g_avoid.ir_own[channel]) ? " (run " :
                                                         " (configured, run ");
        bench_u((uint32_t)g_cal.low[channel], 12U);
        bench_text("-");
        bench_u((uint32_t)g_cal.high[channel], 12U);
        bench_text(")");
    }
    bench_avoid_say();
}

/** @brief AVOID: each front sensor's highest reading this phase. */
void bench_avoid_ir_peak (const uint16_t * p_raw)
{
    uint32_t channel;

    for (channel = 0U; channel < 2U; ++channel)
    {
        if (p_raw[channel] > g_avoid.ir_peak[channel])
        {
            g_avoid.ir_peak[channel] = p_raw[channel];
        }
    }
}

/** @brief AVOID: " (IR peak L x R y)" onto the text being built. */
void bench_avoid_peak_say (void)
{
    bench_text(" (IR peak L ");
    bench_u((uint32_t)g_avoid.ir_peak[0], 12U);
    bench_text(" R ");
    bench_u((uint32_t)g_avoid.ir_peak[1], 12U);
    bench_text(")");
}

/** @brief AVOID sweeps: the median of the BENCH_AVOID_MEDIAN readings held
 * (0 = no echo, so two lost echoes of three are a miss), in *p_mm. False if
 * the readings that echoed spread more than BENCH_AVOID_SPREAD_MM around it
 * (a stray echo beside a good one: skipped, neither hit nor miss).
 */
bool bench_avoid_median (uint16_t * p_mm)
{
    uint16_t sorted[BENCH_AVOID_MEDIAN];
    uint16_t swap;
    uint32_t first = 0U;
    uint32_t i;
    uint32_t j;

    for (i = 0U; i < BENCH_AVOID_MEDIAN; ++i)
    {
        sorted[i] = g_avoid.med_mm[i];
    }
    for (i = 1U; i < BENCH_AVOID_MEDIAN; ++i)
    {
        for (j = i; (j > 0U) && (sorted[j - 1U] > sorted[j]); --j)
        {
            swap = sorted[j];
            sorted[j] = sorted[j - 1U];
            sorted[j - 1U] = swap;
        }
    }
    *p_mm = sorted[BENCH_AVOID_MEDIAN / 2U];
    if (0U == *p_mm)
    {
        return true;
    }
    while (0U == sorted[first])
    {
        ++first;
    }
    return ((uint32_t)(sorted[BENCH_AVOID_MEDIAN - 1U] - sorted[first]) <=
            BENCH_AVOID_SPREAD_MM);
}

/** @brief AVOID sweeps: true if even the nearest echo of the readings held
 * lands behind the obstacle (further than BENCH_AVOID_DEPTH_MM + a map cell
 * past the face, as bench_avoid_beam places a hit, at the middle reading's
 * angle). A spread window like that is the far wall seen past the edge, not
 * a stray echo off the face: 14 Nov, the far echoes (1.0-1.5 m) spread more
 * than BENCH_AVOID_SPREAD_MM, so every window past the left edge was skipped,
 * the sweep ran to its limit "still on it", and the backups and the sweeps
 * from 150-300 mm further off then read the 105 mm box as 173, 194 and
 * 409+ mm wide.
 */
bool bench_avoid_window_beyond (void)
{
    double   cone = BENCH_AVOID_CONE_DEG * CORE_PI / 180.0;
    double   angle = g_avoid.med_rad[BENCH_AVOID_MEDIAN / 2U];
    double   hit_a = (angle >= 0.0) ? fmax(0.0, angle - cone) :
                                      fmin(0.0, angle + cone);
    uint16_t nearest = 0U;
    uint32_t i;

    for (i = 0U; i < BENCH_AVOID_MEDIAN; ++i)
    {
        if ((0U != g_avoid.med_mm[i]) &&
            ((0U == nearest) || (g_avoid.med_mm[i] < nearest)))
        {
            nearest = g_avoid.med_mm[i];
        }
    }
    if (0U == nearest)
    {
        return true;   /* No echo at all (bench_avoid_median takes that). */
    }
    return ((BENCH_AVOID_SONAR_MM * cos(angle)) - g_avoid.back_mm +
            ((double)nearest * cos(hit_a))) >
           (g_avoid.face_mm + BENCH_AVOID_DEPTH_MM + BENCH_MAP_CELL_MM);
}

/** @brief AVOID measuring, each tick: a new front reading joins the last
 * BENCH_AVOID_MEDIAN; their median (bench_avoid_median), at the middle
 * reading's angle, is mapped (bench_avoid_beam) and either hits the
 * obstacle or has slipped past its edge. outward is the turn toward the side
 * being measured (rad). True once BENCH_AVOID_MISSES medians in a row
 * missed, or at the turn limit; b_hit then says whether the sweep ended
 * still on the obstacle (edge unseen).
 */
bool bench_avoid_edge (double outward)
{
    double   limit_rad = (double)BENCH_AVOID_MEASURE_DEG * CORE_PI / 180.0;
    double   angle = bench_avoid_wrap(g_drive.heading - g_avoid.yaw_home);
    uint16_t median_mm = 0U;
    uint32_t i;

    if (g_front.readings != g_avoid.readings)
    {
        g_avoid.readings = g_front.readings;
        bench_avoid_keep_beam(angle, g_front.last_mm);
        if (g_avoid.med_n >= BENCH_AVOID_MEDIAN)
        {
            for (i = 1U; i < BENCH_AVOID_MEDIAN; ++i)
            {
                g_avoid.med_mm[i - 1U] = g_avoid.med_mm[i];
                g_avoid.med_rad[i - 1U] = g_avoid.med_rad[i];
            }
            g_avoid.med_n = BENCH_AVOID_MEDIAN - 1U;
        }
        g_avoid.med_mm[g_avoid.med_n] = g_front.last_mm;
        g_avoid.med_rad[g_avoid.med_n] = angle;
        ++g_avoid.med_n;
        if (g_avoid.med_n < BENCH_AVOID_MEDIAN)
        {
            /* Not enough readings yet for a median. */
        }
        else if (!bench_avoid_median(&median_mm))
        {
            if (bench_avoid_window_beyond())
            {
                /* Spread, but all from behind the obstacle: past its edge. */
                ++g_avoid.misses;
                g_avoid.b_hit = false;
            }
            else
            {
                ++g_avoid.med_skips;
            }
        }
        else if (bench_avoid_beam(g_avoid.med_rad[BENCH_AVOID_MEDIAN / 2U],
                                  g_avoid.med_rad[BENCH_AVOID_MEDIAN / 2U],
                                  median_mm))
        {
            g_avoid.misses = 0U;
            g_avoid.b_hit = true;
        }
        else
        {
            ++g_avoid.misses;
            g_avoid.b_hit = false;
        }
    }
    return (g_avoid.misses >= BENCH_AVOID_MISSES) || (outward >= limit_rad);
}

/** @brief AVOID CREEP, each tick: hold the creep's diagonal (creep_deg, from
 * BENCH_AVOID_CREEP_START_DEG, off the line heading toward creep_side) at
 * BENCH_AVOID_CREEP_MPS, the servo holding the sonar along the line heading,
 * square to the face. Each new reading is mapped (a beam along the line
 * heading from the sonar's place, bench_avoid_beam) and either echoes off
 * the face (from within BENCH_AVOID_FACE_MM of its distance) or not.
 * BENCH_AVOID_MISSES lost echoes in a row end it: the edge
 * is where the sonar was at the first of them (bench_avoid_creep_found).
 * Square on, a flat face echoes while the sonar is in front of it, so that is
 * the edge or a little past it (a corner's own echo), never short of it: the
 * car does not step out too little. The beam's cone is not taken off: the
 * sweeps (2 Nov) lost a box face before the cone reached its edge.
 * At the creep's angle the car's front corner on the line side comes closest
 * to the face; it stops BENCH_AVOID_CREEP_GAP_MM short and waits for the next
 * reading. A lost echo there still counts toward the edge; an echo off the
 * face means the edge is out of reach from here (the width cannot be
 * measured): back up and creep again, at most BENCH_AVOID_BACKUPS reverses in
 * all, then stop the run.
 */
void bench_avoid_creep (uint64_t now_us, double turned,
                               motion_request_t * p_request)
{
    double half = BENCH_AVOID_CAR_WIDTH_MM / 2.0;
    double creep_rad = (double)g_avoid.creep_deg * CORE_PI / 180.0;
    double side = (double)g_avoid.creep_side;
    /* The front corner furthest ahead, ahead of the first stop. */
    double corner_x = (-g_avoid.back_mm) +
                      (BENCH_AVOID_FRONT_MM * cos(turned)) +
                      (half * fabs(sin(turned)));
    /* The sonar's offset from the line toward the side (+ left turned). */
    double sonar_out = (side * g_avoid.right_mm) -
                       (side * BENCH_AVOID_SONAR_MM * sin(turned));
    bool   b_room = ((g_avoid.face_mm - corner_x) > BENCH_AVOID_CREEP_GAP_MM);
    bool   b_hit = false;

    bench_avoid_hold(g_avoid.yaw_home - (side * creep_rad), p_request);
    p_request->forward_mps = b_room ? BENCH_AVOID_CREEP_MPS : 0.0;
    if (g_front.readings == g_avoid.readings)
    {
        return;
    }
    g_avoid.readings = g_front.readings;
    /* Mapped as any beam; the face only if the echo is from about the
     * face's distance (BENCH_AVOID_FACE_MM): the beam runs along the line,
     * so the echo's place ahead is the sonar's x + the range.
     */
    b_hit = bench_avoid_beam(turned, 0.0, g_front.last_mm) &&
            (((BENCH_AVOID_SONAR_MM * cos(turned)) - g_avoid.back_mm +
              (double)g_front.last_mm) <=
             (g_avoid.face_mm + BENCH_AVOID_FACE_MM));
    if (b_hit)
    {
        g_avoid.misses = 0U;
        g_avoid.hit_mm = sonar_out;
        g_avoid.hit_range = g_front.last_mm;
        ++g_avoid.creep_hits;
    }
    else
    {
        if (0U == g_avoid.misses)
        {
            g_avoid.miss_mm = sonar_out;
        }
        ++g_avoid.misses;
    }
    if ((g_avoid.misses >= BENCH_AVOID_MISSES) && (0U == g_avoid.creep_hits) &&
        ((g_avoid.creep_deg - BENCH_AVOID_CREEP_STEP_DEG) >=
         BENCH_AVOID_CREEP_MIN_DEG))
    {
        /* No echo off the face at all: the sonar started past its edge.
         * Turn BENCH_AVOID_CREEP_STEP_DEG less (the sonar nearer the line)
         * and creep again.
         */
        p_request->forward_mps = 0.0;
        p_request->yaw_rps = 0.0;
        g_avoid.creep_none_mm = fmax(g_avoid.creep_none_mm, g_avoid.miss_mm);
        bench_line(BK_AV_NO_ECHO);   /* +-255 deg, +-4095 mm */
        bench_text("AVOID: no echo off its face creeping at ");
        bench_i(g_avoid.creep_deg, 9U);
        bench_text(" deg (the sonar ");
        bench_i(bench_round(g_avoid.miss_mm), 13U);
        bench_text(" mm out is past its edge), creeping at ");
        g_avoid.creep_deg -= BENCH_AVOID_CREEP_STEP_DEG;
        bench_i(g_avoid.creep_deg, 9U);
        bench_text(" deg");
        bench_avoid_say();
        g_avoid.misses = 0U;
        g_avoid.phase = BENCH_AVOID_CREEPTURN;
        return;
    }
    if (g_avoid.misses >= BENCH_AVOID_MISSES)
    {
        if (g_avoid.b_far)
        {
            bench_avoid_far_found(p_request, false, 0.0);
        }
        else
        {
            bench_avoid_creep_found(now_us, p_request);
        }
        return;
    }
    if (b_room || (!b_hit))
    {
        return;
    }
    /* Still on the face with no room to creep on. */
    p_request->forward_mps = 0.0;
    p_request->yaw_rps = 0.0;
    if (g_avoid.backups < BENCH_AVOID_BACKUPS)
    {
        ++g_avoid.backups;
        g_avoid.phase = BENCH_AVOID_RECENTRE;
        bench_line(BK_AV_ON_FACE);   /* +-4095 mm, counts 0-15 */
        bench_text("AVOID: still on its face ");
        bench_i(bench_round(sonar_out), 13U);
        bench_text(" mm out, no room to creep on: backing up ");
        bench_i(bench_round(BENCH_AVOID_BACKUP_MM), 13U);
        bench_text(" mm (");
        bench_u(g_avoid.backups, 4U);
        bench_text(" of ");
        bench_u((uint32_t)BENCH_AVOID_BACKUPS, 4U);
        bench_text(")");
        bench_avoid_say();
        return;
    }
    if (g_avoid.b_far)
    {
        /* The far edge only sizes the width: go on round, its edge at least
         * this far out.
         */
        bench_avoid_far_found(p_request, true, sonar_out);
        return;
    }
    if (g_avoid.side > 0)
    {
        g_avoid.right_edge_mm = fmax(g_avoid.right_edge_mm, sonar_out);
        g_avoid.b_right_open = true;
    }
    else
    {
        g_avoid.left_mm = fmax(g_avoid.left_mm, sonar_out);
        g_avoid.b_left_open = true;
    }
    bench_line(BK_AV_PAST_REACH);   /* +-4095 mm */
    bench_text("AVOID: its edge is past ");
    bench_i(bench_round(sonar_out), 13U);
    bench_text(" mm out and the reverses are spent, stopped");
    bench_avoid_say();
    g_avoid.side = 0;
    g_avoid.side_mm = 0.0;
    bench_avoid_obst_text();
    bench_avoid_map_print();
    bench_line_end(now_us, "stopped: the obstacle's edge was out of reach");
    (void)memset(p_request, 0, sizeof(*p_request));
    p_request->kind = MOTION_HALT;
}

/** @brief AVOID CREEP, the far side's (b_far): its edge is where the sonar
 * was at the first lost echo, as bench_avoid_creep_found takes it (the
 * widest start, unseen, with no echo at any angle; b_past with past_mm when
 * the reverses ran out still on the face: at least that, unseen). It only
 * sizes the obstacle; CREEPBACK then takes the car back along the creep.
 */
void bench_avoid_far_found (motion_request_t * p_request, bool b_past,
                                   double past_mm)
{
    bool   b_unseen = b_past || (0U == g_avoid.creep_hits);
    double edge = fmax(0.0, b_past ? past_mm :
                       ((0U == g_avoid.creep_hits) ?
                        fmax(g_avoid.creep_none_mm, g_avoid.miss_mm) :
                        g_avoid.miss_mm));

    p_request->forward_mps = 0.0;
    p_request->yaw_rps = 0.0;
    if (g_avoid.creep_side > 0)
    {
        g_avoid.right_edge_mm = edge;
        g_avoid.b_right_open = b_unseen;
        g_avoid.b_right_creep = !b_unseen;
    }
    else
    {
        g_avoid.left_mm = edge;
        g_avoid.b_left_open = b_unseen;
        g_avoid.b_left_creep = !b_unseen;
    }
    bench_line(BK_AV_FAR_EDGE);   /* +-4095 mm, sonar 0-4095 mm */
    bench_text(bench_flag(g_avoid.creep_side > 0) ? "AVOID: far (right) edge " :
                                          "AVOID: far (left) edge ");
    bench_i(bench_round(edge), 13U);
    bench_text(bench_flag(b_unseen) ? " mm out or more (last echo " :
                                      " mm out (last echo ");
    if (bench_flag(0U == g_avoid.creep_hits))
    {
        bench_text("none");
    }
    else
    {
        bench_i(bench_round(g_avoid.hit_mm), 13U);
        bench_text(" mm out, ");
        bench_u((uint32_t)g_avoid.hit_range, 12U);
        bench_text(" mm");
    }
    bench_text("), backing to the creep's start");
    bench_avoid_say();
    g_avoid.far_back_mm = g_avoid.back_mm;
    g_avoid.phase = BENCH_AVOID_CREEPBACK;
}

/** @brief AVOID CREEP: the echo is lost, so the chosen side's edge is where
 * the sonar was at the first lost echo. Out = that edge + half the car + the
 * clearance; along (reported) from the pivot to half the car + the clearance
 * past the obstacle's back. The side is checked clear again with the new out
 * (else the run stops, as in bench_avoid_plan); then the obstacle and the map
 * go out and OUT turns on to the full angle.
 */
void bench_avoid_creep_found (uint64_t now_us,
                                     motion_request_t * p_request)
{
    double half = BENCH_AVOID_CAR_WIDTH_MM / 2.0;
    /* No echo at any creep angle: the widest start, as before the retries
     * (safe for the way round), and the edge is marked unseen.
     */
    bool   b_unseen = (0U == g_avoid.creep_hits);
    double edge = fmax(0.0, b_unseen ? fmax(g_avoid.creep_none_mm,
                                            g_avoid.miss_mm) :
                                       g_avoid.miss_mm);

    p_request->forward_mps = 0.0;
    p_request->yaw_rps = 0.0;
    if (g_avoid.side > 0)
    {
        g_avoid.right_edge_mm = edge;
        g_avoid.b_right_open = b_unseen;
        g_avoid.b_right_creep = !b_unseen;
    }
    else
    {
        g_avoid.left_mm = edge;
        g_avoid.b_left_open = b_unseen;
        g_avoid.b_left_creep = !b_unseen;
    }
    g_avoid.side_mm = edge + half + BENCH_AVOID_CLEAR_MM;
    g_avoid.along_mm = (g_avoid.face_mm + g_avoid.back_mm) +
                       BENCH_AVOID_DEPTH_MM + half + BENCH_AVOID_CLEAR_MM;
    bench_line(BK_AV_EDGE);   /* +-4095 mm, sonar 0-4095 mm */
    bench_text("AVOID: edge ");
    bench_i(bench_round(edge), 13U);
    bench_text(" mm out (last echo ");
    if (bench_flag(0U == g_avoid.creep_hits))
    {
        bench_text("none");
    }
    else
    {
        bench_i(bench_round(g_avoid.hit_mm), 13U);
        bench_text(" mm out, ");
        bench_u((uint32_t)g_avoid.hit_range, 12U);
        bench_text(" mm");
    }
    bench_text("): out ");
    bench_i(bench_round(g_avoid.side_mm), 13U);
    bench_text(" mm, along ");
    bench_i(bench_round(g_avoid.along_mm), 13U);
    bench_text(" mm");
    bench_avoid_say();
    bench_line(BK_AV_WIDTH);   /* +-4095 mm each */
    bench_text("AVOID: WIDTH ");
    bench_i(bench_round(bench_avoid_shown(-1) + bench_avoid_shown(1)), 13U);
    bench_text(bench_flag(g_avoid.b_left_open || g_avoid.b_right_open) ?
               " mm or more by the creeps (left " : " mm by the creeps (left ");
    bench_i(bench_round(bench_avoid_shown(-1)), 13U);
    bench_text(", right ");
    bench_i(bench_round(bench_avoid_shown(1)), 13U);
    bench_text("; raw ");
    bench_i(bench_round(g_avoid.left_mm + g_avoid.right_edge_mm), 13U);
    bench_text(", less ");
    bench_i(bench_round(BENCH_AVOID_CREEP_OVER_MM), 13U);
    bench_text(" an edge)");
    bench_avoid_say();
    if (!bench_avoid_side_clear(g_avoid.side, g_avoid.side_mm))
    {
        bench_line(BK_AV_OTHER);
        bench_text("AVOID: something else in the way that far out, stopped");
        bench_avoid_say();
        g_avoid.side = 0;
        g_avoid.side_mm = 0.0;
        bench_avoid_obst_text();
        bench_avoid_map_print();
        bench_line_end(now_us, "stopped: no clear way round the obstacle");
        (void)memset(p_request, 0, sizeof(*p_request));
        p_request->kind = MOTION_HALT;
        return;
    }
    g_avoid.phase = BENCH_AVOID_OUT;
    (void)bench_scan_servo(0);   /* Edge found: the sonar back to centre */
    bench_avoid_obst_text();
    bench_avoid_map_print();
}

/** @brief AVOID: keep a sweep beam for BEAMS (bench_beams_text): the car's
 * angle from the line heading (rad, + left), the range (0 no echo) and how
 * far the pivot was behind the first stop.
 */
void bench_avoid_keep_beam (double angle, uint16_t range_mm)
{
    if (g_beams < BENCH_BEAMS)
    {
        g_beam_deg[g_beams] = (int16_t)bench_round(angle * 180.0 / CORE_PI);
        g_beam_mm[g_beams] = range_mm;
        g_beam_back[g_beams] = (int16_t)bench_round(g_avoid.back_mm);
        ++g_beams;
    }
    else
    {
        ++g_beams_dropped;
    }
}

/** @brief AVOID: one beam into the map, from the sonar of a car at car_angle
 * (rad from the line heading, + left), pointing at beam_angle (the car's
 * angle in the sweeps, the servo centred; the line heading in the creep),
 * with range_mm (0: no echo, nothing known). The cells its centre crossed
 * are free. The hit is the nearest point in the cone: at the beam's angle
 * less BENCH_AVOID_CONE_DEG toward the line heading (0 within the cone);
 * it is the obstacle if no more than BENCH_AVOID_DEPTH_MM behind the face,
 * widening the measured extents, else something else. True for an obstacle
 * hit.
 */
bool bench_avoid_beam (double car_angle, double beam_angle,
                              uint16_t range_mm)
{
    double cone = BENCH_AVOID_CONE_DEG * CORE_PI / 180.0;
    double hit_a = (beam_angle >= 0.0) ? fmax(0.0, beam_angle - cone) :
                                         fmin(0.0, beam_angle + cone);
    double cos_a = cos(beam_angle);
    double sin_a = sin(beam_angle);
    double sonar_x = (BENCH_AVOID_SONAR_MM * cos(car_angle)) -
                     g_avoid.back_mm;
    double sonar_y = (BENCH_AVOID_SONAR_MM * sin(car_angle)) -
                     g_avoid.right_mm;
    double step = BENCH_MAP_CELL_MM / 2.0;
    double reach = (double)range_mm;
    double x_far = fmax(fabs(BENCH_MAP_X_MIN_MM - sonar_x),
                        fabs((BENCH_MAP_X_MIN_MM +
                              ((double)BENCH_MAP_ROWS * BENCH_MAP_CELL_MM)) -
                             sonar_x));
    double y_far = (((double)BENCH_MAP_MID + 0.5) * BENCH_MAP_CELL_MM) +
                   fabs(sonar_y);
    double limit;
    double travel;
    double x_mm;
    double y_mm;
    bool   b_obstacle;

    if (0U == range_mm)
    {
        return false;
    }
    /* Every point further than the farthest map corner is off the map, so
     * the walk stops there: walking a long echo to its end in soft doubles
     * took one control step past the motion core's limit (SENSOR_FAULT).
     * The cells marked are the same.
     */
    limit = fmin(reach - step, sqrt((x_far * x_far) + (y_far * y_far)) + step);
    for (travel = 0.0; travel < limit; travel += step)
    {
        bench_avoid_mark(sonar_x + (travel * cos_a),
                         sonar_y + (travel * sin_a), BENCH_MAP_FREE);
    }
    x_mm = sonar_x + (reach * cos(hit_a));
    y_mm = sonar_y + (reach * sin(hit_a));
    /* A map cell of slack: the obstacle's own back edge echoes right at the
     * given depth, and a few mm of noise made it "something else", which
     * then blocked the way round (sim, 3 Nov).
     */
    b_obstacle = (x_mm <= (g_avoid.face_mm + BENCH_AVOID_DEPTH_MM +
                           BENCH_MAP_CELL_MM));
    /* The far side's creep only finds that edge: its echoes from past the
     * obstacle are not mapped as "something else" (14 Nov: one 80 mm left
     * of the line, 430-470 mm ahead, lay in the strip the car sweeps
     * crossing back over the line, and stopped three runs "no clear way
     * round" after both edges had been found).
     */
    if (b_obstacle || (!g_avoid.b_far))
    {
        bench_avoid_mark(x_mm, y_mm, b_obstacle ? BENCH_MAP_OBSTACLE :
                                                  BENCH_MAP_OTHER);
    }
    if (b_obstacle)
    {
        g_avoid.left_mm = fmax(g_avoid.left_mm, y_mm);
        g_avoid.right_edge_mm = fmax(g_avoid.right_edge_mm, -y_mm);
    }
    return b_obstacle;
}

/** @brief AVOID: mark the map cell at (x_mm ahead, y_mm left) of the first
 * stop. A hit replaces unknown or free (the first hit in a cell stays); free
 * only fills unknown. Points off the map are dropped.
 */
void bench_avoid_mark (double x_mm, double y_mm, uint8_t code)
{
    double   row = floor((x_mm - BENCH_MAP_X_MIN_MM) / BENCH_MAP_CELL_MM);
    double   col = floor((y_mm / BENCH_MAP_CELL_MM) + 0.5) +
                   (double)BENCH_MAP_MID;
    uint8_t * p_cell;

    if ((!isfinite(row)) || (!isfinite(col)) || (row < 0.0) ||
        (row >= (double)BENCH_MAP_ROWS) || (col < 0.0) ||
        (col >= (double)BENCH_MAP_COLS))
    {
        return;
    }
    /* Both are whole numbers within the map, so the casts are exact. */
    p_cell = &g_avoid.map[(uint32_t)row][(uint32_t)col];
    if (BENCH_MAP_FREE == code)
    {
        if (BENCH_MAP_UNKNOWN == *p_cell)
        {
            *p_cell = BENCH_MAP_FREE;
        }
    }
    else if (*p_cell < BENCH_MAP_OBSTACLE)
    {
        *p_cell = code;
    }
    else
    {
        /* Already a hit. */
    }
}

/** @brief AVOID: true if no "something else" hit lies where the car would
 * sweep going round on side (+1 right, -1 left) out_mm from the line: from
 * half a car behind the pivot to half a car past the parallel leg's end, and
 * from half a car on the far side of the line to the outer edge of the car.
 * The obstacle's own cells are left out; unknown cells count as clear.
 */
bool bench_avoid_side_clear (int32_t side, double out_mm)
{
    double   half = BENCH_AVOID_CAR_WIDTH_MM / 2.0;
    double   pivot_x = -g_avoid.back_mm;
    double   x_low = pivot_x - half;
    double   x_high = pivot_x + g_avoid.along_mm + half;
    double   y_out = -(double)side * (out_mm + half); /* + left */
    double   y_low = (side > 0) ? y_out : -half;
    double   y_high = (side > 0) ? half : y_out;
    /* The cells whose centre lies in the box, as whole-number bounds worked
     * out once (row centre x = X_MIN + (row + 0.5) cell, column centre
     * y = (col - MID) cell): the loop only reads cells, no soft doubles.
     */
    double   row_lo = fmax(0.0, ceil(((x_low - BENCH_MAP_X_MIN_MM) /
                                      BENCH_MAP_CELL_MM) - 0.5));
    double   row_hi = fmin((double)BENCH_MAP_ROWS - 1.0,
                           floor(((x_high - BENCH_MAP_X_MIN_MM) /
                                  BENCH_MAP_CELL_MM) - 0.5));
    double   col_lo = fmax(0.0, ceil((y_low / BENCH_MAP_CELL_MM) +
                                     (double)BENCH_MAP_MID));
    double   col_hi = fmin((double)BENCH_MAP_COLS - 1.0,
                           floor((y_high / BENCH_MAP_CELL_MM) +
                                 (double)BENCH_MAP_MID));
    uint32_t row;
    uint32_t col;

    if ((!isfinite(row_lo)) || (!isfinite(row_hi)) || (!isfinite(col_lo)) ||
        (!isfinite(col_hi)) || (row_lo > row_hi) || (col_lo > col_hi))
    {
        return true;
    }
    /* Whole numbers within the map, so the casts are exact. */
    for (row = (uint32_t)row_lo; row <= (uint32_t)row_hi; ++row)
    {
        for (col = (uint32_t)col_lo; col <= (uint32_t)col_hi; ++col)
        {
            if (BENCH_MAP_OTHER == g_avoid.map[row][col])
            {
                return false;
            }
        }
    }
    return true;
}

/** @brief AVOID: both sweeps done. Report the measured width; size both
 * ways round (out = that side's extent + half the car + the clearance;
 * along = the face from the pivot + the deepest obstacle + half the car,
 * which lies across the line once it turns back, + the clearance) and take
 * the shorter of the clear sides with a seen edge, or stop the run if there
 * is none. With an edge unseen it first backs up to see more, at most
 * BENCH_AVOID_BACKUPS times (the only reversing: the width could not be
 * measured). With a side taken, CREEPTURN starts the creep that measures
 * its edge; the obstacle for the map (bench_avoid_obst_text) and the map
 * rows go out once that is found (or here, if the run stops).
 */
void bench_avoid_plan (uint64_t now_us, motion_request_t * p_request)
{
    double half = BENCH_AVOID_CAR_WIDTH_MM / 2.0;
    double out_right = fmax(0.0, g_avoid.right_edge_mm + half +
                                 BENCH_AVOID_CLEAR_MM);
    double out_left = fmax(0.0, g_avoid.left_mm + half +
                                BENCH_AVOID_CLEAR_MM);
    bool   b_right = false;
    bool   b_left = false;

    bench_line(BK_AV_FACE);   /* +-4095 mm each, skips 0-255 */
    bench_text("AVOID: face ");
    bench_i(bench_round(g_avoid.face_mm + g_avoid.back_mm), 13U);
    bench_text(" mm ahead of the pivot; obstacle left ");
    bench_i(bench_round(g_avoid.left_mm), 13U);
    bench_text(bench_flag(g_avoid.b_left_open) ? " mm (edge unseen), right " :
                                     " mm, right ");
    bench_i(bench_round(g_avoid.right_edge_mm), 13U);
    bench_text(bench_flag(g_avoid.b_right_open) ? " mm (edge unseen); WIDTH ~" :
                                      " mm; WIDTH ~");
    bench_i(bench_round(g_avoid.left_mm + g_avoid.right_edge_mm), 13U);
    bench_text(bench_flag(g_avoid.b_left_open || g_avoid.b_right_open) ?
               " mm or more" : " mm");
    bench_text(" (median of 3, ");
    bench_u(g_avoid.med_skips, 8U);
    bench_text(" spread skipped)");
    bench_avoid_say();
    if ((g_avoid.b_left_open || g_avoid.b_right_open) &&
        (g_avoid.backups < BENCH_AVOID_BACKUPS))
    {
        ++g_avoid.backups;
        g_avoid.phase = BENCH_AVOID_RECENTRE;
        bench_line(BK_AV_PAST_SWEEP);   /* +-4095 mm, counts 0-15 */
        bench_text("AVOID: an edge is past the sweep, backing up ");
        bench_i(bench_round(BENCH_AVOID_BACKUP_MM), 13U);
        bench_text(" mm to see more (");
        bench_u(g_avoid.backups, 4U);
        bench_text(" of ");
        bench_u((uint32_t)BENCH_AVOID_BACKUPS, 4U);
        bench_text(")");
        bench_avoid_say();
        return;
    }
    g_avoid.along_mm = (g_avoid.face_mm + g_avoid.back_mm) +
                       BENCH_AVOID_DEPTH_MM + half + BENCH_AVOID_CLEAR_MM;
    b_right = (!g_avoid.b_right_open) &&
              bench_avoid_side_clear(1, out_right);
    b_left = (!g_avoid.b_left_open) &&
             bench_avoid_side_clear(-1, out_left);
    bench_line(BK_AV_PATH);   /* +-4095 mm each, 2 bits a side's state */
    bench_text("AVOID: path right: out ");
    bench_i(bench_round(out_right), 13U);
    bench_text(" mm, total ");
    bench_i(bench_round((2.0 * out_right) + g_avoid.along_mm), 13U);
    bench_code(b_right ? 0U : (g_avoid.b_right_open ? 1U : 2U), 2U);
    bench_text(b_right ? " mm, clear" :
               (g_avoid.b_right_open ? " mm, edge unseen" : " mm, blocked"));
    bench_text("; left: out ");
    bench_i(bench_round(out_left), 13U);
    bench_text(" mm, total ");
    bench_i(bench_round((2.0 * out_left) + g_avoid.along_mm), 13U);
    bench_code(b_left ? 0U : (g_avoid.b_left_open ? 1U : 2U), 2U);
    bench_text(b_left ? " mm, clear" :
               (g_avoid.b_left_open ? " mm, edge unseen" : " mm, blocked"));
    bench_avoid_say();
    if ((!b_right) && (!b_left))
    {
        bench_line(BK_AV_NO_WAY);
        bench_text("AVOID: no clear way round, stopped");
        bench_avoid_say();
        g_avoid.side = 0;
        g_avoid.side_mm = 0.0;
        bench_avoid_obst_text();
        bench_avoid_map_print();
        bench_line_end(now_us, "stopped: no clear way round the obstacle");
        (void)memset(p_request, 0, sizeof(*p_request));
        p_request->kind = MOTION_HALT;
        return;
    }
    /* Shorter path wins; the right on a tie, as before. */
    if (b_right && ((!b_left) || (out_right <= out_left)))
    {
        g_avoid.side = 1;
        g_avoid.side_mm = out_right;
    }
    else
    {
        g_avoid.side = -1;
        g_avoid.side_mm = out_left;
    }
    /* The sweeps chose the side. A sweep cannot place an edge (14 Nov: the
     * 105 mm box read 131, its far side 80 mm against ~52), so a creep
     * measures each edge square on: the far side's first (back along it
     * after, CREEPBACK), then the way round's, which sends the obstacle and
     * the map (bench_avoid_creep_found).
     */
    g_avoid.phase = BENCH_AVOID_CREEPTURN;
    g_avoid.creep_side = -g_avoid.side;
    g_avoid.b_far = true;
    g_avoid.far_start_mm = g_avoid.right_mm;
    g_avoid.far_backups = g_avoid.backups;
    g_avoid.creep_deg = BENCH_AVOID_CREEP_START_DEG;
    g_avoid.creep_hits = 0U;
    g_avoid.creep_none_mm = 0.0;
    g_avoid.misses = 0U;
    bench_line(BK_AV_GOING);   /* +-4095 mm, 0-127 deg */
    bench_text(bench_flag(g_avoid.side > 0) ? "AVOID: going round the right" :
                                    "AVOID: going round the left");
    bench_text(" (sweep out ");
    bench_i(bench_round(g_avoid.side_mm), 13U);
    bench_text(" mm); first turning ");
    bench_u((uint32_t)BENCH_AVOID_CREEP_START_DEG, 7U);
    bench_text(bench_flag(g_avoid.side > 0) ?
               " deg to creep out to its left edge" :
               " deg to creep out to its right edge");
    bench_avoid_say();
}

/** @brief AVOID: the obstacle for the dashboard's map, one line: OBST at the
 * odometry pose of the first stop (x y mm, h deg: the map's origin and its
 * x axis), the face ahead of it, the measured extents left and right of the
 * line (? for an edge unseen), the depth allowed for (BENCH_AVOID_DEPTH_MM,
 * given), the side taken (R, L or none), out and along (from the pivot) and
 * back (the pivot behind the first stop after any reverse and the creep; -
 * once ahead of it).
 */
/** @brief AVOID: an edge as reported (side +1 right, -1 left): a seen creep
 * edge less BENCH_AVOID_CREEP_OVER_MM (not under 0), else as measured. The
 * way round uses the raw edge.
 */
double bench_avoid_shown (int32_t side)
{
    double edge = (side > 0) ? g_avoid.right_edge_mm : g_avoid.left_mm;
    bool   b_creep = (side > 0) ? g_avoid.b_right_creep : g_avoid.b_left_creep;

    return b_creep ? fmax(0.0, edge - BENCH_AVOID_CREEP_OVER_MM) : edge;
}

void bench_avoid_obst_text (void)
{
    /* Frame: x, y +-65535 mm (17 bits, as P lines), h +-204.7 deg, the
     * rest +-4095 mm, the side 2 bits (0 none, 1 R, 2 L).
     */
    bench_line(BK_AV_OBST);
    bench_text("OBST at ");
    bench_i(bench_round(g_avoid.x0_mm), 17U);
    bench_text(" ");
    bench_i(bench_round(g_avoid.y0_mm), 17U);
    bench_text(" h ");
    bench_x(bench_round(g_avoid.yaw_home * 1800.0 / CORE_PI), 1U, 12U);
    bench_text(" face ");
    bench_i(bench_round(g_avoid.face_mm), 13U);
    bench_text(" left ");
    bench_i(bench_round(bench_avoid_shown(-1)), 13U);
    bench_text(bench_flag(g_avoid.b_left_open) ? "? right " : " right ");
    bench_i(bench_round(bench_avoid_shown(1)), 13U);
    bench_text(bench_flag(g_avoid.b_right_open) ? "? depth " : " depth ");
    bench_i(bench_round(BENCH_AVOID_DEPTH_MM), 13U);
    bench_code((g_avoid.side > 0) ? 1U : ((g_avoid.side < 0) ? 2U : 0U), 2U);
    bench_text((g_avoid.side > 0) ? " go R" :
               ((g_avoid.side < 0) ? " go L" : " go none"));
    bench_text(" out ");
    bench_i(bench_round(g_avoid.side_mm), 13U);
    bench_text(" along ");
    bench_i(bench_round(g_avoid.along_mm), 13U);
    bench_text(" back ");
    bench_i(bench_round(g_avoid.back_mm), 13U);
    bench_avoid_say();
}

/** @brief AVOID: the map, one line per BENCH_MAP_CELL_MM row, the left at
 * the left: # obstacle, + something else, . free, - unknown, | the line where
 * no beam hit, C the car's pivot. AVOID alone sends every row a beam reached,
 * far rows first (ahead is up). MISSION sends live only the rows with a hit,
 * nearest first, as M lines (the row's x, mm ahead of the first stop, then
 * its cells), so the link carries a few lines, not the whole map.
 */
void bench_avoid_map_print (void)
{
    char     line[BENCH_MAP_COLS + 1U];
    int32_t  car_row = bench_round(floor((-g_avoid.back_mm -
                                          BENCH_MAP_X_MIN_MM) /
                                         BENCH_MAP_CELL_MM));
    int32_t  car_col = bench_round(floor((-g_avoid.right_mm /
                                          BENCH_MAP_CELL_MM) + 0.5)) +
                       BENCH_MAP_MID;
    uint32_t first = BENCH_MAP_ROWS;
    uint32_t last = 0U;
    uint32_t row;
    uint32_t col;

    for (row = 0U; row < BENCH_MAP_ROWS; ++row)
    {
        for (col = 0U; col < BENCH_MAP_COLS; ++col)
        {
            if (BENCH_MAP_UNKNOWN != g_avoid.map[row][col])
            {
                first = (row < first) ? row : first;
                last = (row > last) ? row : last;
            }
        }
    }
    /* Frames (MISSION): the cell 0-31 mm, row x +-1023 mm (11 bits), the
     * BENCH_MAP_COLS marks 3 bits each ("-#+.|C").
     */
    if (first > last)
    {
        bench_line(BK_AVOID_MAP_NONE);
        bench_text("AVOID MAP: no beams");
        bench_avoid_say();
        return;
    }
    if (BENCH_MODE_MISSION == g_drive.mode)
    {
        bench_line(BK_AVOID_MAP_HEAD);
        bench_text("AVOID MAP ");
        bench_u((uint32_t)bench_round(BENCH_MAP_CELL_MM), 5U);
        bench_text(" mm cells, rows with a hit as M x cells, left at left:"
                   " # obstacle, + other, . free, - unknown, | line");
        bench_avoid_say();
        for (row = first; row <= last; ++row)
        {
            if (bench_avoid_map_row(row, car_row, car_col, line))
            {
                bench_line(BK_AVOID_MAP_ROW);
                bench_text("M ");
                bench_i(bench_round(BENCH_MAP_X_MIN_MM +
                                    (((double)row + 0.5) *
                                     BENCH_MAP_CELL_MM)), 11U);
                bench_text(" ");
                bench_marks(line, "-#+.|C", 3U);
                bench_avoid_say();
            }
        }
        return;
    }
    if ((car_row >= 0) && ((uint32_t)car_row < first))
    {
        first = (uint32_t)car_row;
    }
    bench_text("AVOID MAP ");
    bench_dec(bench_round(BENCH_MAP_CELL_MM));
    bench_text(" mm cells, ahead up, left at left: # obstacle, + other,"
               " . free, - unknown, | line, C car");
    bench_avoid_say();
    row = last + 1U;
    while (row > first)
    {
        --row;
        (void)bench_avoid_map_row(row, car_row, car_col, line);
        bench_text(line);
        bench_avoid_say();
    }
}

/** @brief AVOID: one map row as text, left (+y, high columns) to right, in
 * p_line (BENCH_MAP_COLS + 1 bytes). True if a beam hit a cell of it.
 */
bool bench_avoid_map_row (uint32_t row, int32_t car_row,
                                 int32_t car_col, char * p_line)
{
    uint32_t index;
    uint32_t col;
    uint8_t  cell;
    char     mark;
    bool     b_hit = false;

    for (index = 0U; index < BENCH_MAP_COLS; ++index)
    {
        col = (BENCH_MAP_COLS - 1U) - index;
        cell = g_avoid.map[row][col];
        if (BENCH_MAP_OBSTACLE == cell)
        {
            mark = '#';
            b_hit = true;
        }
        else if (BENCH_MAP_OTHER == cell)
        {
            mark = '+';
            b_hit = true;
        }
        else if ((uint32_t)BENCH_MAP_MID == col)
        {
            mark = '|';
        }
        else if (BENCH_MAP_FREE == cell)
        {
            mark = '.';
        }
        else
        {
            mark = '-';
        }
        if (((int32_t)row == car_row) && ((int32_t)col == car_col))
        {
            mark = 'C';
        }
        p_line[index] = mark;
    }
    p_line[BENCH_MAP_COLS] = '\0';
    return b_hit;
}

/** @brief The go-around's line built in g_text: MISSION queues it to go out
 * live (bench_avoid_text), never waiting on the radio mid-run; a full queue
 * drops it (counted). AVOID alone sends it at once.
 */
void bench_avoid_say (void)
{
    uint32_t length = g_length;
    uint32_t size = 0U;   /* the line's frame, queued with its text */

    if (BENCH_MODE_MISSION != g_drive.mode)
    {
        bench_flush();
        return;
    }
    if ((BK_NONE != g_frame_kind) && !gb_frame_text)
    {
        size = (uint32_t)bitpack_writer_size(&g_frame_w);
    }
    g_frame_kind = BK_NONE;
    gb_frame_text = false;
    if (g_avoid_queued >= BENCH_AVOID_LINES)
    {
        ++g_avoid_dropped;
    }
    else
    {
        if (length >= BENCH_AVOID_LINE_BYTES)
        {
            length = BENCH_AVOID_LINE_BYTES - 1U;
        }
#if BENCH_AVOID_TEXT
        (void)memcpy(g_avoid_lines[g_avoid_queued], g_text, length);
        g_avoid_lines[g_avoid_queued][length] = '\0';
#else
        (void)length;
#endif
        (void)memcpy(g_avoid_frames[g_avoid_queued], g_frame, size);
        g_avoid_frame_size[g_avoid_queued] = size;
        ++g_avoid_queued;
    }
    g_length = 0U;
    g_text[0] = '\0';
}

/** @brief MISSION: the oldest queued go-around line as text (not flushed).
 * True if there was one.
 */
bool bench_avoid_text (void)
{
    uint32_t index;

    if (0U == g_avoid_queued)
    {
        return false;
    }
#if BENCH_AVOID_TEXT
    bench_text(" ");
    bench_text(g_avoid_lines[0]);
#endif
    /* Its frame becomes the line's, whole (the kind's template has the
     * leading space); none: the line has no kind and is not sent.
     */
    if (0U != g_avoid_frame_size[0])
    {
        (void)memcpy(g_frame, g_avoid_frames[0], g_avoid_frame_size[0]);
        g_frame_w.p_data = g_frame;
        g_frame_w.capacity = sizeof(g_frame);
        g_frame_w.bit = 8U * g_avoid_frame_size[0];
        g_frame_w.b_valid = true;
        g_frame_kind = (uint32_t)g_frame[2] &
                       bitpack_mask(BENCH_FRAME_KIND_BITS);
        gb_frame_text = false;
    }
    --g_avoid_queued;
    for (index = 0U; index < g_avoid_queued; ++index)
    {
#if BENCH_AVOID_TEXT
        (void)memcpy(g_avoid_lines[index], g_avoid_lines[index + 1U],
                     BENCH_AVOID_LINE_BYTES);
#endif
        (void)memcpy(g_avoid_frames[index], g_avoid_frames[index + 1U],
                     BENCH_FRAME_BYTES);
        g_avoid_frame_size[index] = g_avoid_frame_size[index + 1U];
    }
    return true;
}

/** @brief MISSION, each step of the go-around (the line step returns before
 * bench_live_step): the live line pushed on, then the next queued line, else
 * a pose line when due, else the next sweep beams.
 */
void bench_avoid_live (uint64_t now_us)
{
    bench_live_push();
    if ((g_live_sent >= g_live_length) && (0U == g_length) &&
        (bench_avoid_text() || bench_pose_text(now_us) ||
         bench_beams_text()))
    {
        bench_live_take();
    }
}

/** @brief MISSION: the next sweep beams not yet sent, up to
 * BENCH_BEAMS_PER_LINE taken from about the same place (the pivot back mm
 * behind the first stop, within BENCH_BEAMS_SAME_MM: the offset is now
 * integrated in every phase, so a pivot's wheel scrub moves it a mm or two),
 * as one line (not flushed): BEAMS back mm: deg/mm ... (deg from the line
 * heading, + left; mm 0 for no echo). True if there were any.
 */
bool bench_beams_text (void)
{
    uint32_t count = 0U;
    int32_t  back;

    if (g_beams_sent >= g_beams)
    {
        return false;
    }
    back = (int32_t)g_beam_back[g_beams_sent];
    /* Frame: back +-2047 mm (12 bits), the beams' count 0-15 first, each
     * +-255 deg (9 bits) and 0-4095 mm, dropped 0-1023.
     */
    while (((g_beams_sent + count) < g_beams) &&
           (count < BENCH_BEAMS_PER_LINE) &&
           (((int32_t)g_beam_back[g_beams_sent + count] - back) <=
            BENCH_BEAMS_SAME_MM) &&
           ((back - (int32_t)g_beam_back[g_beams_sent + count]) <=
            BENCH_BEAMS_SAME_MM))
    {
        ++count;
    }
    bench_line(BK_BEAMS);
    bench_text(" BEAMS back ");
    bench_i(back, 12U);
    bench_text(":");
    bench_code(count, 4U);
    for (; 0U != count; --count)
    {
        bench_text(" ");
        bench_i((int32_t)g_beam_deg[g_beams_sent], 9U);
        bench_text("/");
        bench_u((uint32_t)g_beam_mm[g_beams_sent], 12U);
        ++g_beams_sent;
    }
    if (bench_flag((g_beams_sent >= g_beams) && (0U != g_beams_dropped)))
    {
        bench_text(" (+");
        bench_u(g_beams_dropped, 10U);
        bench_text(" not kept)");
    }
    return true;
}

/** @brief SCAN test: the robot's scanner (app_cfg_scanner), motors braked,
 * a stale echo consumed. The scan starts after the text, so no settle
 * window is spent sending.
 */
void bench_scan_start (void)
{
    scanner_config_t config;
    uint32_t         pulse_us = 0U;
    bool             b_valid = false;

    board_motor_brake();
    (void)memset(&g_scan, 0, sizeof(g_scan));
    app_cfg_scanner(&config);
    if (!scanner_init(&g_scanner, &config))
    {
        bench_text("SCAN refused: scanner_init");
        bench_flush();
        return;
    }
    if ((!gb_watchdog) && board_watchdog_start(BENCH_WATCHDOG_MS))
    {
        gb_watchdog = true;
    }
    bench_text("SCAN start: car still, box ahead; STOP aborts");
    bench_flush();
    (void)board_ultrasonic_result(&pulse_us, &b_valid);
    g_scan.start_us = board_time_us();
    g_scan.last_us = g_scan.start_us;
    g_scan.b_running = scanner_start(&g_scanner,
                                     (uint32_t)(g_scan.start_us / 1000U),
                                     true);
    if (!g_scan.b_running)
    {
        bench_text("SCAN refused: scanner_start");
        bench_flush();
    }
}

/** @brief One Scan-task period as robot_app_scan_step() runs it during a
 * scan: echo timeout, completed echo, scanner step, then servo and trigger.
 * The robot faults SCAN when the servo or the trigger is refused; here the
 * refusals are counted and the scan goes on, so every beam is seen.
 */
void bench_scan_tick (uint8_t command)
{
    scanner_echo_t   echo;
    scanner_output_t output;
    uint64_t         now_us = board_time_us();
    uint32_t         dt_us = (uint32_t)(now_us - g_scan.last_us);
    uint32_t         pulse_us = 0U;
    bool             b_valid = false;
    bool             b_ok = false;

    g_scan.last_us = now_us;
    if (dt_us > g_scan.dt_max_us)
    {
        g_scan.dt_max_us = dt_us;
    }
    if (BENCH_CMD_STOP == command)
    {
        bench_scan_finish("aborted");
        return;
    }
    board_ultrasonic_service();
    (void)memset(&echo, 0, sizeof(echo));
    echo.b_ready = board_ultrasonic_result(&pulse_us, &b_valid);
    echo.pulse_us = b_valid ? pulse_us : 0U;
    b_ok = scanner_step(&g_scanner, (uint32_t)(now_us / 1000U), true, &echo,
                        &output);
    if (output.b_set_servo && (!bench_scan_servo(output.angle_deg)))
    {
        ++g_scan.servo_refused;
    }
    if (output.b_trigger && (!board_ultrasonic_trigger()))
    {
        ++g_scan.trigger_refused;
    }
    if (!b_ok)
    {
        bench_scan_finish("stopped: scanner_step failed");
    }
    else if (output.b_done)
    {
        bench_scan_finish(g_scanner.result.b_aborted ?
                          "aborted by the scanner" : "complete");
    }
    else if ((now_us - g_scan.start_us) >= BENCH_SCAN_MAX_US)
    {
        bench_scan_finish("stopped at the robot's 20 s limit");
    }
    else
    {
        /* Still scanning. */
    }
}

/** @brief robot_app_servo() with the robot's own endpoints: the bench copy
 * widens ROBOT_SERVO_MIN_US/MAX_US for S5-U06. Beam angle, + left.
 */
bool bench_scan_servo (int32_t angle_deg)
{
    int32_t physical_deg = angle_deg * ROBOT_SERVO_ANGLE_SIGN;
    int32_t span_us = (physical_deg < 0) ?
        (int32_t)(ROBOT_SERVO_CENTRE_US - BENCH_ROBOT_SERVO_MIN_US) :
        (int32_t)(BENCH_ROBOT_SERVO_MAX_US - ROBOT_SERVO_CENTRE_US);

    if ((angle_deg < -ROBOT_SCAN_HALF_ANGLE_DEG) ||
        (angle_deg > ROBOT_SCAN_HALF_ANGLE_DEG))
    {
        return false;
    }
    return board_servo_us((uint16_t)((int32_t)ROBOT_SERVO_CENTRE_US +
                                     ((physical_deg * span_us) /
                                      ROBOT_SCAN_HALF_ANGLE_DEG)));
}

/** @brief Centre the servo, then print the scan; nothing moves meanwhile.
 */
void bench_scan_finish (const char * p_reason)
{
    const scanner_result_t * p_result = &g_scanner.result;

    g_scan.b_running = false;
    g_scan.done_ms = (uint32_t)((board_time_us() - g_scan.start_us) / 1000U);
    (void)bench_scan_servo(0);
    g_send_tries = BENCH_TABLE_SEND_TRIES;
    bench_text("SCAN ");
    bench_text(p_reason);
    bench_text(" after ");
    bench_fixed(bench_round((double)g_scan.done_ms / 100.0), 1U);
    bench_text(" s: ");
    bench_dec((int32_t)g_scanner.sample_count);
    bench_text(" beams, dt max ");
    bench_fixed(bench_round((double)g_scan.dt_max_us / 100.0), 1U);
    bench_text(" ms, refused trigger ");
    bench_dec((int32_t)g_scan.trigger_refused);
    bench_text(" servo ");
    bench_dec((int32_t)g_scan.servo_refused);
    bench_flush();
    if (SCANNER_COMPLETE == g_scanner.phase)
    {
        bench_text(" mm: nearest ");
        bench_scan_mm(p_result->nearest_distance_m);
        bench_text(", width ");
        bench_scan_mm(p_result->apparent_width_m);
        bench_text(", clear L ");
        bench_scan_mm(p_result->left_clearance_m);
        bench_text(" R ");
        bench_scan_mm(p_result->right_clearance_m);
        bench_text(", side ");
        bench_text(g_side_names[(uint32_t)p_result->preferred_side %
                                BENCH_SCAN_SIDES]);
        bench_text(", beams valid ");
        bench_dec((int32_t)p_result->valid_samples);
        bench_text(" unknown ");
        bench_dec((int32_t)p_result->unknown_samples);
        bench_flush();
        bench_scan_verdict();
    }
    bench_scan_beams();
    g_send_tries = BENCH_SEND_TRIES;
}

/** @brief What Vehicle would do with this profile: robot_app_bypass_input()
 * on the packed result, then vehicle_scan_valid() with app_cfg_vehicle()'s
 * limits. ROBOT_CORRIDOR_CONFIRMED is left out: it stays 0 until
 * commissioning, and with it 0 every scan faults.
 */
void bench_scan_verdict (void)
{
    scanner_shared_t shared;
    scanner_result_t seen;
    vehicle_config_t vehicle;
    double           offset_m = 0.0;
    double           length_m = 0.0;

    (void)memset(&shared, 0, sizeof(shared));
    (void)memset(&vehicle, 0, sizeof(vehicle));
    scanner_shared_pack(&g_scanner.result, &shared);
    scanner_shared_unpack(&shared, &seen);
    app_cfg_vehicle(&vehicle, true);
    offset_m = (seen.apparent_width_m / 2.0) + (ROBOT_ROBOT_WIDTH_M / 2.0) +
               ROBOT_OBSTACLE_MARGIN_M;
    length_m = seen.nearest_distance_m + ROBOT_MAX_OBSTACLE_DEPTH_M +
               ROBOT_ROBOT_LENGTH_M + ROBOT_OBSTACLE_MARGIN_M;
    bench_text(" robot");
    if ((0U != g_scan.trigger_refused) || (0U != g_scan.servo_refused))
    {
        bench_text(": SCAN FAULT, a trigger or servo command was refused");
    }
    else if (0U != seen.unknown_samples)
    {
        bench_text(": FAULT, a beam with no echo or beyond 4 m makes the"
                   " profile invalid");
    }
    else if (!seen.b_obstacle_seen)
    {
        bench_text(": FAULT, no beam within ");
        bench_scan_mm(g_scanner.config.obstacle_distance_m);
        bench_text(" mm");
    }
    else if (SCANNER_SIDE_NONE == seen.preferred_side)
    {
        bench_text(": FAULT, neither side clear by ");
        bench_scan_mm(g_scanner.config.required_clearance_m);
        bench_text(" mm");
    }
    else if ((!isfinite(offset_m)) || (offset_m <= 0.0) ||
             (offset_m > vehicle.max_bypass_offset_m) ||
             (!isfinite(length_m)) || (length_m <= 0.0) ||
             (length_m > vehicle.max_bypass_length_m))
    {
        bench_text(": FAULT, detour beyond the offset or length limit");
    }
    else
    {
        bench_text(" with CORRIDOR set: BYPASS ");
        bench_text(g_side_names[(uint32_t)seen.preferred_side %
                                BENCH_SCAN_SIDES]);
    }
    bench_text("; offset ");
    bench_scan_mm(offset_m);
    bench_text(" length ");
    bench_scan_mm(length_m);
    bench_text(" mm");
    bench_flush();
}

/** @brief Every sampled beam, the newest at each angle. */
void bench_scan_beams (void)
{
    scanner_sample_t sample;
    int32_t          angle;
    uint32_t         printed = 0U;

    bench_text(" beams deg:mm, + left, f fine pass, none = no echo,"
               " far = beyond 4 m");
    bench_flush();
    for (angle = -ROBOT_SCAN_HALF_ANGLE_DEG;
         angle <= ROBOT_SCAN_HALF_ANGLE_DEG; angle += BENCH_SCAN_STEP_DEG)
    {
        if (scanner_sample(&g_scanner, angle, &sample))
        {
            bench_text(" ");
            bench_signed(angle);
            bench_text(":");
            if (CORE_RANGE_VALID == sample.range.state)
            {
                bench_dec(bench_round(sample.range.distance_m * 1000.0));
            }
            else
            {
                bench_text((CORE_RANGE_OUT_OF_RANGE == sample.range.state) ?
                           "far" : "none");
            }
            if (sample.b_fine)
            {
                bench_text("f");
            }
            ++printed;
            if (0U == (printed % BENCH_SCAN_PER_ROW))
            {
                bench_flush();
            }
        }
    }
    if (0U != (printed % BENCH_SCAN_PER_ROW))
    {
        bench_flush();
    }
}

/** @brief Whole millimetres of a distance in metres; "none" if not finite.
 */
void bench_scan_mm (double metres)
{
    if (isfinite(metres))
    {
        bench_dec(bench_round(metres * 1000.0));
    }
    else
    {
        bench_text("none");
    }
}

/* End of file. */

