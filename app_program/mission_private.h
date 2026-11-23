/** @file mission_private.h
 * @brief Private to the mission's parts (mission.c and its subsystem files):
 * what more than one part uses. Split out of the one file by
 * tools/reorg/split_c.py (22 Nov 2026).
 */
#ifndef MISSION_PRIVATE_H
#define MISSION_PRIVATE_H

/** @file mission.c
 * @brief The MISSION (line, barcode cards, junctions, hump, obstacle) and
 * the bench tests it grew from. Two builds of this one file:
 * - The robot app (default, 22 Nov 2026): the Vehicle task's controller
 *   (robot_app.c, every 5 ms). MISSION only; the sensors and outputs stay
 *   with their owner tasks (the "robot app's owners" section below), the
 *   fault mask, STOP and the watchdog with the Safety task.
 * - The bench image (BENCH_IMAGE, firmware/bench/build.sh): bench_loop, as
 *   described below (the file was firmware/bench/bench_main.c until 22 Nov).
 *
 * BENCH IMAGE, NOT THE ROBOT FIRMWARE: the measurements the mission
 * firmware cannot make (TESTING.md section 5). No robot task starts.
 *
 * - S2-U07 duty sweep: both wheels together, forward then backward, up
 *   through g_duties and back down. Each step settles 0.5 s, then counts
 *   0.5 s. The table and a summary (start and keep-turning duty, motor sign,
 *   top speed) follow the sweep. THE CAR MUST BE ON ITS STAND WITH BOTH
 *   WHEELS OFF THE GROUND: the sweep reaches 60% duty.
 * - S3-U03 IR and S4-U03/U04 accelerometer: raw values once a second
 *   (mean and min-max), sensor axes, no bias, scale or axis mapping.
 * - S2-U06 roll test and S3-U06: wheel distance since RESET, once a second.
 * - S5-U06 servo: pulse stepping from the command bytes below.
 *
 * The motion image (build.sh ... motion) replaces the sweep and the servo
 * commands with tests of the robot's own motion controller (motion.c and
 * app_cfg_motion(), at the robot's duty limit), run with one of g_gains:
 * - S2-U08 PID steps, ON THE STAND: each wheel from rest to 0.08 m/s, to
 *   0.15 m/s, then the pivot-turn speed, 4 s each. Per wheel: rise and
 *   settling time, peak, steady mean and ripple, duty, and a 2 s trace.
 * - S2-U09 1 m drive and S2-U10 90/180 degree pivots, ON THE FLOOR: encoder
 *   distance and odometry heading when the primitive completes and 1 s
 *   later. Measure the real distance or angle by hand.
 * - LINE follow, ON THE FLOOR, front sensors on the line: the robot's
 *   line_ctrl (app_cfg_line() calibration) and its FOLLOW steering law
 *   while TRACKING. Unlike the robot, which halts while SEARCHING, it keeps
 *   its last command through short white gaps, and line_ctrl calls the
 *   line lost after BENCH_LINE_LOST_MS instead of 100 ms. A junction (a
 *   track crossing) is crossed straight on at the line speed, then the line
 *   is followed again. It runs until a lost line, 5 m or 60 s, or both
 *   sensors stay black 150 mm past a junction. Then a 250 ms trace of
 *   distance, raw IR, steering, line state and heading. Three tests: at
 *   ROBOT_LINE_SPEED_MPS (80 mm/s), 120 mm/s and ROBOT_MAX_SPEED_MPS.
 * - LINE + obstacle stop, ON THE FLOOR, an obstacle somewhere on the line:
 *   the 120 mm/s line test plus the robot's front sonar (servo centred,
 *   polled as robot_app_scan_step does). A VALID range of at most
 *   BENCH_LINE_STOP_MM halts the car within 10 cm of the obstacle; where
 *   the robot's own rule (ROBOT_FRONT_STOP_DISTANCE_MM, FOLLOW to SCAN)
 *   would have halted is recorded. A range that is not VALID would halt
 *   the robot (RANGE_WAIT); here it is counted and the test goes on.
 * - SCAN, ON THE FLOOR, CAR STILL, a box ahead: one robot obstacle scan
 *   (scanner.c, app_cfg_scanner(), the robot's servo endpoints), then its
 *   time, the profile, every beam and what the robot's Vehicle would do
 *   with it (robot_app_bypass_input and vehicle_scan_valid). The bench
 *   loop runs every ~7 ms where the robot's Scan task runs every 2 ms, so
 *   the scan takes a little longer here.
 * - LINE centre, ON THE TRACK, line between or under the front sensors:
 *   proportional centring instead of FOLLOW's full steering whenever one
 *   sensor sees the line (bench_centre_steer). Straight on with the line
 *   between the sensors, through crossings and onward until STOP, a lost
 *   line, BENCH_CENTRE_MAX_M or 120 s. At 120 mm/s and ROBOT_MAX_SPEED_MPS.
 * - LINE smooth A-C, ON THE TRACK: LINE centre against the zig-zag. A
 *   sensor that sees the line starts a small correction that grows while
 *   it keeps seeing it (bench_smooth_track); steering changes at a bounded
 *   rate; the inner wheel keeps at least BENCH_SMOOTH_INNER_MPS; a line
 *   lost under hard steering is searched for at once. Settings g_smooth.
 *   Every line test's trace also shows wheel speeds and duties.
 * - BARCODE read, ON THE TRACK, a printed code beside the line on the car's
 *   LEFT (the side sensor's side): LINE smooth A's steering (g_smooth[0])
 *   at BENCH_CODE_MPS, ending as it does but at BENCH_CODE_MAX_M, that also
 *   reads the side sensor every kernel tick (1 ms, bench_code_wait) and
 *   stores the mean of its readings per robot decoder unit
 *   (ROBOT_APP_BARCODE_UNIT_HC) of the sensor's own path (the centre path
 *   less ROBOT_BARCODE_LATERAL_OFFSET_M times the heading change).
 *   Afterwards: raw min/max, the bar and space widths of each dark group, a
 *   Code 39 decode of those widths alone (no codebook: any Code 39 text, read
 *   either way), again on this run's own thresholds if the robot's missed
 *   it, then on the card's own levels (bench_code_card: edges halfway
 *   between each bar's peak and the spaces beside it, the line left out),
 *   and a 1 mm raw trace of the whole run. Bars that never reach the robot's
 *   DARK threshold are reported as read white (an IR-invisible print, or the
 *   sensor beside the bars).
 * - AUTO-CENTRE, ON THE TRACK, front sensors near the line: pivots on the
 *   spot until each front sensor finds the line, then to the heading midway
 *   between (bench_auto_step). LINE smooth A-C, BARCODE and MISSION run it
 *   first, then follow the line from there; the AUTO-CENTRE test runs it
 *   alone.
 * - MISSION, ON THE TRACK, runs until STOP: LINE smooth A's steering at
 *   BENCH_MISSION_MPS (straight on at crossings) that reads barcodes as it
 *   drives. The side sensor is
 *   read as in BARCODE, but the store streams (bench_code_stream): once
 *   BENCH_CODE_QUIET_MM of white follows a dark mark, the darkness pass
 *   decodes what is stored, quietly, and keeps only that white. While it
 *   drives it sends each group as it is decoded (CODE when framed, else
 *   GROUP), the samples of each cut that made a group (RAW, bench_raw_text)
 *   and a status line every second (bench_live_step); after the stop, only
 *   the totals.
 * - AVOID, ON THE FLOOR, no line: the car drives straight (odometry holds the
 *   heading) at ROBOT_LINE_SPEED_MPS until the front sonar reads an obstacle
 *   within BENCH_AVOID_TURN_MM; then it stops, turns the whole car left and
 *   right to map the obstacle with the sonar (its edges and width, sent as
 *   map rows), backs up BENCH_AVOID_BACKUP_MM and maps again if an edge is
 *   out of sight (the only reversing: the width could not be measured), and
 *   goes round the shorter clear side with a seen edge (with no such side
 *   it stops): a part turn and a slow creep out, the sonar square to the
 *   face, until the echo is lost (that side's real edge), then a full turn
 *   out past that edge + BENCH_AVOID_CLEAR_MM, PARALLEL until past its back
 *   (BENCH_AVOID_DEPTH_MM at most), back to the line and onto the original
 *   course (the offsets are tracked from the wheels, so it lands back on
 *   course). Runs until STOP.
 * Its commands: 0xAA START runs MISSION with gain set 4, 0xA9 STOP stops it.
 * MISSION follows the line, reads barcodes and the ramp, and goes around an
 * obstacle (as above) when the front sonar finds one, then reacquires the
 * line. The other tests stay in the source but START runs only
 * MISSION (g_mode); standalone AVOID remains for isolated obstacle testing.
 *
 * Counts are signed with ROBOT_*_ENCODER_SIGN, so + is forward.
 *
 * Input: the car's START button runs the sweep; its STOP button aborts it
 * and brakes. The MQTT build also takes one command byte per message
 * (the receive path passes only the mission bytes 0xA9-0xAF):
 *   0xA9 STOP      abort the sweep and brake
 *   0xAA START     run the sweep (about 2.5 min)
 *   0xAB RESET     zero the wheel distance
 *   0xAC LEFT      servo pulse down one step
 *   0xAD RIGHT     servo pulse up one step
 *   0xAE STRAIGHT  servo to 1500 us
 *   0xAF UTURN     servo step 10 us <-> 100 us
 * Output: lines. MQTT build: one minimum-bit frame per line on the telemetry
 * topic (BENCH_FRAMES, Change Requirement 1; FRAMES.md), decoded back to the
 * line by tools/dashboard/bench_codec.py and the dashboard. Offline build:
 * UART0 on GP0, 115200 8N1 (the sonar is not used), each frame in hex ("#F")
 * and then its line. Console prints mask interrupts and can lose encoder
 * edges, so
 * the sweep table is printed only after the motors stop; read wheel
 * distance on the MQTT build.
 */
/* math.h and string.h bring the toolchain's size_t: skip the kernel's own
 * typedef, as lib/liblwip/sysdepend/pico_rp2040/sys_arch.c does.
 */
#define PROHIBIT_DEF_SIZE_T
#include "app_cfg.h"
#include "app_main.h"
#include "barcode.h"
#include "bitpack.h"
#include "board.h"
#include "line_ctrl.h"
#include "mission.h"
#include "motion.h"
#include "navmap.h"
#include "net_mqtt_ipc.h"
#include "robot_app.h"
#include "robot_config.h"
#include "scanner.h"
#include "terrain.h"
#include "vehicle.h"
#include <math.h>
#include <string.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#define BENCH_ROBOT_MAX_DUTY ROBOT_MAX_DUTY
#define BENCH_ROBOT_SERVO_MIN_US ROBOT_SERVO_MIN_US
#define BENCH_ROBOT_SERVO_MAX_US ROBOT_SERVO_MAX_US
#if !defined(MISSION_SIM)
#define MISSION_PRINTF mission_no_console
#else
#define MISSION_PRINTF tm_printf
#endif
#if !defined(BENCH_PIVOT_TEST)
#define BENCH_PIVOT_TEST (0)
#endif
#if !defined(BENCH_STRAIGHT_CODE)
#define BENCH_STRAIGHT_CODE (0)
#endif
#if !defined(BENCH_ROBOT_MAX_DUTY)
#error "BENCH_ROBOT_MAX_DUTY (the robot's ROBOT_MAX_DUTY) comes from build.sh"
#endif
#if !defined(BENCH_ROBOT_SERVO_MIN_US) || !defined(BENCH_ROBOT_SERVO_MAX_US)
#error "BENCH_ROBOT_SERVO_MIN_US/MAX_US (robot endpoints) come from build.sh"
#endif

/* One accelerometer sample, sensor axes, mg (16-bit chip words), and when
 * the IMU task read it (board_time_us, low 32 bits).
 */
typedef struct
{
    uint32_t x       : 16;
    uint32_t y       : 16;
    uint32_t z       : 16;
    uint32_t read_us : 32;
} mission_accel_t;
extern mission_accel_t  g_acc_ring[MISSION_ACCEL_RING];
extern uint8_t          g_acc_head;
extern uint8_t          g_acc_tail;
extern uint16_t         g_ir_now[3];
extern bool             gb_ir_now;
extern uint16_t         g_ir_now_total[2];
extern mission_input_t  g_in;
extern mission_output_t g_out;
extern uint8_t          g_motion_id;
#define BENCH_WATCHDOG_MS (1000U)
#define BENCH_TEXT_BYTES (160U)

/* Change Requirement 1 (Tribunal 1, minimum-bit data), 22 Nov: with
 * BENCH_FRAMES 1 nothing goes over MQTT as text. Each line still builds its
 * text in g_text (task-private: the offline console and the sim's check),
 * and alongside it one binary frame, which is all the radio sends:
 *   byte 0  BENCH_FRAME_HEAD 0xBD: telemetry.h's 2-bit version 1, the
 *           frame type 3 the robot reserves, then the pattern 1011b, so a
 *           frame is never printable text and never a robot frame;
 *   byte 1  seq, 8 bits, counted at sending (a gap is a lost frame);
 *   then    the line's kind (BENCH_FRAME_KIND_BITS) and its fields in order,
 *           each in the bits its range needs (bitpack.h stream: LSB first,
 *           little-endian, zero padding). Unsigned fields saturate; signed
 *           ones are two's complement and saturate at +/-(2^(n-1)-1), the
 *           most negative code unused (bitpack.h); both count g_frame_sat.
 * The constant text is not sent: tools/dashboard/bench_codec.py holds each
 * kind's template and rebuilds the same line. A line with no kind, or that
 * prints a number with no width (bench_dec and the like), is never sent:
 * it is counted (g_frame_unsent) and only the console shows it ("#T").
 * BENCH_FRAMES 0 is the old text build (debugging).
 */
/* one MQTT message, NET_MQTT_IPC_TX_MAX */
#define BENCH_FRAME_BYTES (64U)
#if ROBOT_MQTT
_Static_assert(BENCH_FRAME_BYTES <= NET_MQTT_IPC_TX_MAX,
               "a frame must fit one telemetry slot (build.sh sets "
               "NET_MQTT_IPC_BENCH)");
#endif
#define BENCH_FRAME_KIND_BITS (7U)

/* The kinds, in order: bench_codec.py's KINDS must list the same names in
 * the same order (its self-test reads this enum). Append only.
 */
typedef enum
{
    BK_NONE = 0,     /* no frame: the line is never sent */
    BK_LIVE,         /* MISSION status line, every second */
    BK_POSE,         /* " P x y heading [C x y z]" */
    BK_REC_HEAD,     /* "REC: last n samples ..." */
    BK_REC_ROW,      /* "REC k:" and 1-3 samples */
    BK_REC_END,      /* "REC end" */
    BK_CODE,         /* " CODE ..." or " GROUP ..." (Decoder 1) */
    BK_FIT,          /* " Decoder 2 ..." per card */
    BK_D0,           /* " D0 ..." per group */
    BK_D0_WIDTHS,    /* " D0 widths u: ..." */
    BK_FIT_TOTALS,   /* " Decoder 2: n read, ..." */
    BK_CAR,          /* " CAR ..." the vote per card */
    BK_CAR_TOTALS,   /* " CAR: n codes ..." */
    BK_RAW_HEAD,     /* " RAW n: ..." a cut's samples follow */
    BK_RAW_ROW,      /* " R n: ..." the samples, 7 bits each */
    BK_MAP,          /* "MAP set: ..." while idle */
    BK_NAV_MAP,      /* the same, queued in MISSION (leading space) */
    BK_NAV_RESTORED, /* " NAV: map restored ..." */
    BK_NAV_CARD,     /* " NAV: card ..." */
    BK_NAV_JUNCTION, /* " NAV: junction n at ..." */
    BK_NAV_TURNED,   /* " NAV: turned ..." */
    BK_NAV_WAIT,     /* " NAV: junction at ..., still decoding" */
    BK_NAV_WAITED,   /* " NAV: waited ..." */
    BK_RAMP_OFF,     /* "RAMP off: ..." at START */
    BK_RAMP_FLAT,    /* "RAMP flat reference: ..." at START */
    BK_RAMP,         /* " RAMP n: peak ..." a rise */
    BK_SIDE,         /* " SIDE n: ..." a one-wheel hump, or left out */
    BK_TERRAIN,      /* " TERRAIN hump ..." the robot module */
    BK_CAL,          /* " CAL L ..." the START calibration */
    BK_RAMP_HIGHEST, /* end report: " ramp: highest ..." */
    BK_RAMP_OPEN,    /* " ramp: open at the stop ..." */
    BK_RAMP_RISES,   /* " ramp: rises ..." */
    BK_RAMP_LEANS,   /* " ramp: most side lean ..." */
    BK_RAMP_HELD,    /* " ramp: held straight over humps ..." */
    BK_RAMP_OUT,     /* " ramp: left out at a turn ..." */
    BK_RAMP_WILD,    /* " ramp: left out as lifted ..." */
    BK_LIFT,         /* " lift: ..." */
    BK_RAMP_ROLL,    /* " ramp: most roll ..." */
    BK_DRIVE_START,  /* "DRIVE start: ..." */
    BK_STRAIGHT,     /* "STRAIGHT barcode: ..." (that image only) */
    BK_AUTO_FIRST,   /* "AUTO-CENTRE first: ..." */
    BK_END,          /* "MISSION stopped: ... dt max ..." */
    BK_CENTRE,       /* end report: " centre: ..." */
    BK_CENTRE_GAPS,  /* " gaps ..., crossings ..." */
    BK_AVOID_DROPPED, /* " obstacle go-around: n lines dropped" */
    BK_D0_TOTALS,    /* " D0: n changes ..." */
    BK_BARCODES,     /* " barcodes: n readings ..." */
    /* The go-around (queued: a leading space when sent), its map, sweeps. */
    BK_AV_OBSTACLE,
    BK_AV_OBSTACLE_AVOID,
    BK_AV_MEASURE,
    BK_AV_LEFT_EDGE,
    BK_AV_BACKED,
    BK_AV_CREEP,
    BK_AV_FAR_BACK,
    BK_AV_STEP_OUT,
    BK_AV_CLEAR,
    BK_AV_ALONG,
    BK_AV_AHEAD,
    BK_AV_PAST,
    BK_AV_WAY_CLEAR,
    BK_AV_RETURN,
    BK_AV_ON_LINE,
    BK_AV_ALONG_LINE,
    BK_AV_ON_COURSE,
    BK_AV_STILL_WAY,
    BK_AV_BLOCKED,
    BK_AV_WAY_IN,
    BK_AV_WAY_IN_BOTH,
    BK_AV_CROSSED,
    BK_AV_TURN_LINE,
    BK_AV_TURN_NONE,
    BK_AV_TURN_BACK,
    BK_AV_HOME_NONE,
    BK_AV_HOME,
    BK_AV_IR,
    BK_AV_NO_ECHO,
    BK_AV_ON_FACE,
    BK_AV_PAST_REACH,
    BK_AV_FAR_EDGE,
    BK_AV_EDGE,
    BK_AV_WIDTH,
    BK_AV_OTHER,
    BK_AV_FACE,
    BK_AV_PAST_SWEEP,
    BK_AV_PATH,
    BK_AV_NO_WAY,
    BK_AV_GOING,
    BK_AV_OBST,
    BK_AVOID_MAP_NONE,
    BK_AVOID_MAP_HEAD,
    BK_AVOID_MAP_ROW,
    BK_BEAMS,
    /* Idle, boot, commands, the lift stop and AUTO-CENTRE. */
    BK_BOOT,
    BK_ENC_ZERO,
    BK_IR,
    BK_ACC,
    BK_MAG,
    BK_ENC,
    BK_LIFT_STOP,
    BK_AUTO,
    BK_AUTO_WHY,
    BK_AUTO_FOUND,
    BK_AUTO_END,
    BK_NAV_TURN,     /* " NAV: no line ... a turn, not a junction" */
    BK_SLOW,         /* " slowest step at ..." (end report) */
    BK_TASKS,        /* " tasks, longest step ..." (robot app, end report) */
    BK_IMU,          /* " IMU since START: ..." (robot app, end report) */
    BK_OWNERS,       /* " owner health ..." (robot app, end report) */
    BK_CPU,          /* " tasks, own CPU ..." (robot app, end report) */
    BK_KINDS
} bench_kind_t;
#define BENCH_SEND_TRIES (25U)        /* about 0.17 s per frame while idle */
#define BENCH_TABLE_SEND_TRIES (450U) /* about 3 s: the motors are stopped */
#define BENCH_WHEELS (2U)
#define BENCH_FRONT_IR (2U)     /* the left and right line sensors */
#define BENCH_NAV_DECODERS (3U) /* Decoder 1, Decoder 2 and D0 */
#define BENCH_LEFT (0U)
#define BENCH_RIGHT (1U)

/* Motion image. */
#define BENCH_PID_STEPS (3U)
#define BENCH_TRACE_ROWS (20U)          /* the first 2 s of each step */
#define BENCH_LINE_ROWS (241U)          /* 0..60 s every BENCH_LINE_TRACE_US */
#define BENCH_LINE_STATES (6U)

/* LINE + stop: within 10 cm of the obstacle (the 120 mm/s line test rolled
 * about 3 mm after its halt on 28 Oct; the sonar reads about 7 mm long).
 */
#define BENCH_LINE_STOP_MM (100U)

/* Obstacle avoid (MISSION, and BENCH_MODE_AVOID with no line): stop in front
 * of the obstacle, map it with the front sonar, plan the shorter clear way
 * round and drive it (odometry heading + distance; no line sensors). Drive
 * until the front sonar reads an obstacle within BENCH_AVOID_TURN_MM
 * (BENCH_AVOID_CONFIRM readings in a row in MISSION), then:
 *   PAUSE     stop for BENCH_AVOID_PAUSE_MS,
 *   MEASURE_L pivot the whole car left (sonar facing ahead) until the beam
 *             slips past the obstacle's left edge or the turn limit,
 *   MEASURE_R pivot right through the line heading to the right edge;
 *             then the map is sent and the path planned (bench_avoid_plan),
 *   RECENTRE  an edge never found (the width cannot be measured from this
 *             close): pivot back to the line heading,
 *   REVERSE   back straight up BENCH_AVOID_BACKUP_MM and measure again, at
 *             most BENCH_AVOID_BACKUPS times (the only reversing, allowed
 *             when the width cannot be measured); after that a side whose
 *             edge stayed unseen is not taken, and with neither side it
 *             stops,
 *   CREEPTURN pivot BENCH_AVOID_CREEP_START_DEG toward the side being
 *             measured, the servo turning the sonar back the other way:
 *             square to the face again. First the FAR side (not the way
 *             round), then CREEPBACK reverses along that creep to its start
 *             and the chosen side's creep follows (14 Nov: both edges by
 *             creep, as the sweeps cannot place one),
 *   CREEP     drive slowly out on that diagonal until the echo is lost,
 *             BENCH_AVOID_MISSES readings in a row: the edge is where the
 *             sonar was at the first lost echo. A flat face echoes only
 *             while the sonar is square to it, so the sweeps cannot find a
 *             box's edge (every box read about the same width close up, and
 *             the car stepped out about as far round each, 3 Nov); square
 *             on, the echo stops as the sonar passes the edge. If the front
 *             corner would come within BENCH_AVOID_CREEP_GAP_MM of the face
 *             first, the edge is out of reach from here: RECENTRE and
 *             REVERSE as above, then creep again (no new sweeps), or stop
 *             once the reverses are spent,
 *   OUT       pivot on to a full BENCH_AVOID_ANGLE_DEG toward that side,
 *   STEPOUT   drive out until the car clears the edge + half the car +
 *             BENCH_AVOID_CLEAR_MM,
 *   ALIGN     pivot back to the line heading,
 *   ALONG     run PARALLEL until the whole car is past the obstacle's back:
 *             the pivot BENCH_AVOID_DEPTH_MM + half the car +
 *             BENCH_AVOID_CLEAR_MM past the face, by the forward offset
 *             (the creep and any reverse moved the car along the line),
 *   BACKPIVOT pivot toward the line, STEPIN drive back onto it, HOMEPIVOT
 *             turn onto the original heading.
 * The sideways and forward offsets are tracked from the wheels, so it comes
 * back ON the line.
 * The map: every beam of the sweeps lands in a BENCH_MAP_CELL_MM grid fixed at
 * the first stop (x ahead along the line, y + left). With the pivot BACK mm
 * behind the first stop and RIGHT mm right of the line, a beam at car angle t
 * with range r hits x = (SONAR + r) cos t - BACK, y = (SONAR + r) sin t -
 * RIGHT, and the cells before it are free. The echo comes from the nearest
 * point inside the sonar's cone, not the beam's centre: past the edge of a
 * face across the line the box still echoes until the centre is
 * BENCH_AVOID_CONE_DEG further round (this doubled a 105 mm box to 224 mm),
 * so the hit is placed at t less the cone's half-angle, toward the line
 * heading (from the sonar: r at that angle). A hit no more than
 * BENCH_AVOID_DEPTH_MM (+ a map cell) behind the face is the obstacle; one
 * further back is
 * something else (a wall), and a side with one in the car's way is not taken.
 * Of the clear sides with a found edge, the shorter path (2 x out + along)
 * wins; the creep then measures that side's edge, and the creep's beams
 * (square on, along the line) go on the map too. A flat face turned past
 * ~25-40 deg echoes nothing (read as an edge), and cells no beam reached are
 * not checked. All distances tune on the car.
 */
#define BENCH_AVOID_TURN_MM (130U)     /* stop and scan this far from the
                                        * obstacle (13 cm: 20 cm
                                        * made the way round too wide) */

/* Width sweeps (6 Nov): each decision is the median of the last
 * BENCH_AVOID_MEDIAN readings, placed at the middle one's angle (about 2.4
 * deg a reading at ROBOT_TURN_RATE_RPS), so one stray or lost echo no longer
 * moves an edge (single readings put a 12 cm box at 83-157 mm). Readings
 * that echo but spread more than BENCH_AVOID_SPREAD_MM are skipped, neither
 * hit nor miss. The creep still takes single readings.
 */
#define BENCH_AVOID_MEDIAN (3U)
#define BENCH_AVOID_PIVOT_TOL_DEG (1.5)/* a pivot is done this close */
#define BENCH_AVOID_CONFIRM (2U)       /* MISSION: readings in a row in reach */
#define BENCH_AVOID_FRONT_MM (130.0)   /* the car's front (the front sensors'
                                        * front end) ahead of the pivot
                                        * (ruler, 29 Oct) */
#define BENCH_AVOID_CREEP_START_DEG (30)
#if ((BENCH_AVOID_CREEP_START_DEG + 5) > ROBOT_SCAN_HALF_ANGLE_DEG)
#error "BENCH_AVOID_CREEP_START_DEG leaves the servo no margin to its end"
#endif
#define BENCH_AVOID_LINE_MM (18.0)     /* track line width (spec) */
#define BENCH_TURN_LINE_RAD (BENCH_AVOID_LINE_MM / BENCH_AVOID_FRONT_MM)
#define BENCH_MAP_ROWS (60U)           /* -400..+800 mm along the line: the
                                        * go-around and two back-ups */
#define BENCH_MAP_COLS (41U)           /* -410..+410 mm across, + left */
#define BENCH_CENTRE_PHASES (4U)

/* Faster sets (29 Oct, 180 mm/s target): the motion wheel limit is raised
 * from ROBOT_MAX_SPEED_MPS for LINE smooth alone, and the speed falls toward
 * BENCH_SMOOTH_TURN_MPS as the steering grows, about the full-steering speed
 * that went round the track at 110-130 mm/s. At 130 mm/s the straight-line
 * duty was about 19% of the 35% allowed.
 */
#define BENCH_SMOOTH_WHEEL_MPS (0.20)

/* MISSION's straight-line speed, smooth A's steering. 120 mm/s on 30 Oct to
 * see whether the small card's narrow bars, blurred together at 180, read
 * apart slower (about 2.4 readings per decoder unit instead of 1.6).
 */
/* 6 Nov: 80 -> 100 mm/s, raised for more speed */
#define BENCH_MISSION_MPS (0.10)

/* Both sensors dark while steering at least this much: the line crossing
 * under both in a bend (29 Oct, twice), not a crossing line: keep turning.
 */
#define BENCH_SMOOTH_BEND (0.4)
#if !defined(BENCH_EDGE_FOLLOW)
#define BENCH_EDGE_FOLLOW (0)
#endif
#define BENCH_STRAIGHT_SLOTS (30U)
#define BENCH_CODE_SAMPLES (2560U)
#define BENCH_CODE_RUNS (160U)
#define BENCH_CODE_CHARS (44U)
#define BENCH_CODE_TEXT (16U)
#define BENCH_CODE_LINES (8U)
#define BENCH_CODE_EXTREMA (480U)
#define BENCH_CODE_FOUND (12U)

/* MISSION also tries groups that end at BENCH_CODE_QUIET_MM of white, the
 * Code 39 quiet zone (10 narrow widths) of the spec's 3 mm narrow bar, and
 * cuts the stream there: at 180 mm/s on 30 Oct both cards split at 20-23 mm
 * of white inside them.
 */
#define BENCH_CODE_QUIET_MM (30.0)
#define BENCH_CODE_GAPS (2U)
#define BENCH_CODE_SWINGS (7U)  /* TRY_MIN..TRY_MAX */
#define BENCH_CODE_TRIES (BENCH_CODE_GAPS * BENCH_CODE_SWINGS)
#define BENCH_CODE_RULES (2U)
#define BENCH_CODE_ALL_TRIES (BENCH_CODE_RULES * BENCH_CODE_TRIES)
#define BENCH_CODE_SIG (40U)    /* a try's framed texts, end to end */

/* MISSION also reads the barcode module's digital D0 (board_barcode_d0, GP7,
 * from 2 Nov 2026) at every side sensor reading and keeps each change at the
 * unit of the sensor's own path (bench_d0_sample). The level seen less is
 * dark (white dominates a run). Changes from the first dark one to the last
 * one before BENCH_CODE_QUIET_MM of light form a D0 group; each group is
 * decoded as Code 39 with the 3 widest of every 9 elements wide (no
 * threshold to tune) and sent as two live lines, the result and its element
 * widths (bench_d0_text). One of the car's three barcode votes (CAR lines,
 * bench_vote_add at the end of the file).
 * A group keeps BENCH_D0_EDGES changes (15 bars, *X*, are 30), and
 * BENCH_D0_GROUPS closed groups wait to be sent; more are counted, dropped.
 */
#define BENCH_D0_EDGES (72U)
#define BENCH_D0_GROUPS (4U)
#define BENCH_RAMP_CELLS (44U)
#define BENCH_RAMP_HOLD_SAMPLES (128U) /* 100 Hz at 42 mm/s: 72 per 30 mm */
#define BENCH_COUNT_M (2.0 * CORE_PI * ROBOT_WHEEL_RADIUS_M / \
                       (double)ROBOT_ENCODER_COUNTS_PER_REV)
#define BENCH_CMD_STOP (0xA9U)
#define BENCH_CMD_LEFT (0xACU)
#define BENCH_CMD_STRAIGHT (0xAEU)
#define BENCH_CMD_UTURN (0xAFU)
extern const uint8_t g_duties[35];
#define BENCH_DUTIES ((uint32_t)(sizeof(g_duties) / sizeof(g_duties[0])))

/* Up and down, forward then backward. */
#define BENCH_STEPS (4U * BENCH_DUTIES)

typedef struct
{
    bool     b_running;
    uint32_t step;        /* index into g_steps */
    bool     b_rest;
    bool     b_counting;
    uint64_t step_us;     /* start of the current step or rest */
    uint64_t count_us;    /* start of the counted half */
    uint16_t start[BENCH_WHEELS];
    bool     b_errors;
} bench_sweep_t;

typedef struct
{
    uint32_t count;
    int32_t  sum[3];
    int16_t  minimum[3];
    int16_t  maximum[3];
} bench_accel_t;

/** MISSION: one rise above the floor behind (bench_ramp_step). */
typedef struct
{
    double  peak;                  /* above the floor, m */
    double  roll;                  /* at the peak */
    double  up;                    /* steepest nose up and down in it */
    double  down;
    double  heading;               /* g_drive.heading where it opened */
    double  turned;                /* most heading change from there, rad */
    int32_t from_mm;               /* centre path where it opened */
    int32_t peak_mm;
    int32_t peak_ds;               /* t of the peak, 0.1 s */
    bool    b_straight;            /* opened following a straight line
                                    * (bench_ramp_on_straight) */
} bench_rise_t;

/** MISSION: one lean to a side (bench_ramp_lean). */
typedef struct
{
    double  peak;                  /* side height at the most, + left up */
    double  pitch;                 /* at the most */
    double  length;                /* path while open, m */
    double  heading;               /* g_drive.heading where it opened */
    double  turned;                /* most heading change from there, rad */
    double  held;                 /* most tilted mean over BENCH_RAMP_HOLD_M,
                                    * signed as peak: the height it reports */
    int32_t from_mm;
    int32_t peak_mm;
    int32_t peak_ds;               /* t of the most, 0.1 s */
    int32_t held_mm;               /* where that hold ended */
    bool    b_straight;            /* opened following a straight line
                                    * (bench_ramp_on_straight) */
} bench_lean_t;

/** MISSION: IR channels' extremes for the START calibration. */
typedef struct
{
    uint16_t low[LINE_CTRL_CHANNELS];
    uint16_t high[LINE_CTRL_CHANNELS];
    uint32_t count;
    int32_t  done_mm;
    bool     b_done;
    bool     b_sent;
} bench_cal_t;

/** MISSION: the ramp, on the robot's terrain module. */
typedef struct
{
    terrain_state_t  state;
    terrain_output_t output;
    bench_accel_t    flat;         /* the last idle second (bench_report) */
    bench_rise_t     rise;         /* the open rise */
    bench_rise_t     sent;         /* the last closed, to go out */
    bench_rise_t     best;         /* the highest of the run */
    double           floor[BENCH_RAMP_CELLS]; /* level each cell behind */
    uint64_t         last_us;      /* the last fresh sample */
    uint64_t         window_us;
    int32_t          path;         /* left + right counts at it */
    int32_t          window_path;
    int32_t          cell_path;    /* left + right counts at the last cell */
    double           window_mps;
    double           raw_pitch;    /* last valid filtered pitch and roll */
    double           raw_roll;
    double           pitch_offset; /* the car's own (bench_ramp_offset) */
    double           roll_offset;
    double           travelled;    /* path since START, m */
    double           pitch;        /* raw less the offset */
    double           roll;
    bench_lean_t     lean;         /* the open lean */
    bench_lean_t     lean_sent;    /* the last closed, to go out */
    bench_lean_t     lean_best;    /* the most of the run */
    double           speed;        /* encoder speed, filtered */
    double           level;        /* height along the path since START */
    double           above;        /* level over the floor behind */
    double           left;         /* run's most left-up and right-up roll */
    double           right;
    double           terrain_peak; /* the robot module's last closed hump */
    int32_t          left_mm;
    int32_t          right_mm;
    uint32_t         cells;        /* floor cells filled */
    uint32_t         cell;         /* the next to write */
    uint32_t         rises;        /* rises closed */
    uint32_t         best_rise;
    uint32_t         refused;      /* samples without a valid tilt */
    uint32_t         samples;
    bool             b_ready;
    bool             b_measured;   /* reference from the idle second */
    bool             b_rising;     /* a rise is open */
    bool             b_send;       /* a closed rise waits to go out */
    bool             b_terrain;    /* a closed module hump waits */
    bool             b_terrain_valid;
    bool             b_leaning;    /* a lean is open */
    bool             b_lean_send;  /* a closed lean waits to go out */
    uint32_t         leans;        /* leans closed, BENCH_RAMP_LEAN_M long */
    uint32_t         best_lean;
    uint32_t         jolts;        /* leans closed shorter than that */
    double           hold_side[BENCH_RAMP_HOLD_SAMPLES]; /* the open lean's */
    double           hold_at[BENCH_RAMP_HOLD_SAMPLES];   /* recent samples */
    uint32_t         hold_count;   /* side height and travelled, m */
    uint32_t         hold_next;
    uint32_t         rise_turns;   /* rises left out: at a turn */
    uint32_t         lean_turns;   /* leans left out: at a turn */
    uint32_t         lean_steep;   /* leans left out: pitched too far */
    uint32_t         rise_level;   /* rises left out: no wheel up */
    uint32_t         lean_level;   /* leans left out: no wheel up */
    uint32_t         lean_wild;    /* leans and rises left out: beyond any
                                    * hump (BENCH_RAMP_PEAK_MAX_M) */
    double           lift_g[3];    /* lift stop: gravity's direction, slow */
    double           lift_gsize;   /* and its size, m/s^2, slower */
    uint32_t         lift_gn;      /* samples in them so far (to
                                    * BENCH_LIFT_START_N: plain mean) */
    double           lift_up;      /* upward speed, m/s, leaking */
    double           lift_height;  /* height change, m, leaking */
    double           lift_height_most; /* the run's most height, m */
    int8_t           lift_dir[2];  /* each motor's last direction -1, 0, +1 */
    uint64_t         lift_settle_us; /* speed and height held at 0 until */
    uint32_t         lift_settles; /* motor direction changes seen */
    double           lift_up_most; /* and most upward speed, m/s */
    bool             b_lifted;     /* the lift stop ended the run */
    const char *     p_lean_out;     /* why lean_sent was left out, else NULL */
    bool             b_hump_held;  /* steering held over the open hump */
    bool             b_hump_go;    /* that hold let go (a bend) */
    int32_t          hump_held_from; /* centre path where the hold began */
    uint32_t         hump_holds;   /* holds this run */
    uint32_t         hump_held_mm; /* path held straight over humps */
    uint32_t         hump_let_gos; /* holds let go for a bend */
} bench_ramp_t;

typedef enum
{
    BENCH_MODE_PID = 0,   /* S2-U08, on the stand */
    BENCH_MODE_DISTANCE,  /* S2-U09, on the floor */
    BENCH_MODE_LEFT_90,   /* S2-U10, on the floor */
    BENCH_MODE_RIGHT_90,
    BENCH_MODE_TURN_180,
    BENCH_MODE_LINE,      /* line follow at ROBOT_LINE_SPEED_MPS, floor */
    BENCH_MODE_LINE_FAST, /* at BENCH_LINE_FAST_MPS */
    BENCH_MODE_LINE_MAX,  /* at ROBOT_MAX_SPEED_MPS */
    BENCH_MODE_LINE_STOP, /* at BENCH_LINE_FAST_MPS, stops for an obstacle */
    BENCH_MODE_SCAN,     /* obstacle scan, car still on the floor */
    BENCH_MODE_CENTRE,   /* line centring at BENCH_CENTRE_MPS, track */
    BENCH_MODE_CENTRE_MAX, /* at ROBOT_MAX_SPEED_MPS */
    BENCH_MODE_SMOOTH_A, /* LINE smooth, g_smooth[0] */
    BENCH_MODE_SMOOTH_B,
    BENCH_MODE_SMOOTH_C,
    BENCH_MODE_BARCODE,  /* LINE smooth A's steering at BENCH_CODE_MPS */
    BENCH_MODE_AUTO,     /* AUTO-CENTRE alone, no line follow */
    BENCH_MODE_MISSION,  /* LINE smooth A, barcodes read while driving */
    BENCH_MODE_AVOID,    /* line-follow, then go around an obstacle */
    BENCH_MODES
} bench_mode_t;

/** One wheel over one PID step. Speeds are the controller's own 50 ms
 * feedback, in m/s toward the target; the steady window is the last 2 s.
 */
typedef struct
{
    double   target;          /* m/s, + forward */
    int32_t  rise_ms;         /* first at 90% of the target, -1: never */
    int32_t  out_ms;          /* last outside the 10% band, -1: never */
    double   peak;
    double   low;             /* steady window */
    double   high;
    int32_t  counts;          /* steady window, toward the target */
    double   duty_s;          /* steady window: |duty| x seconds */
    double   steady_s;
    uint32_t steady_ticks;
    uint32_t saturated;       /* ticks at the duty limit */
    uint32_t stall_us;        /* now: |duty| >= ROBOT_STALL_DUTY, no count */
    uint32_t stall_max_us;
    int16_t  trace_mm_s[BENCH_TRACE_ROWS];
    int16_t  trace_permille[BENCH_TRACE_ROWS];
} bench_wheel_t;

typedef struct
{
    bool             b_running;
    bool             b_rest;          /* PID: braked between steps */
    bool             b_done;          /* drive tests: primitive complete */
    bench_mode_t     mode;
    uint32_t         step;
    uint32_t         traced;          /* trace rows of this step */
    uint64_t         step_us;         /* start of the step, rest or 1 s on */
    uint64_t         last_us;         /* previous tick */
    uint32_t         dt_max_us;
    /* MISSION: centre path at the slowest step */
    int32_t          dt_max_mm;
    /* and each part's slowest: Decoder 1 tries */
    uint32_t         slow_tries_us;
    uint32_t         slow_fit_us;   /* Decoder 2's fit slice */
    uint32_t         slow_live_us;  /* the live lines (frames) */
    uint32_t         errors;          /* g_encoder_errors at the start */
    double           done_s;
    double           yaw;             /* latest odometry heading */
    double           heading;         /* the real heading: the wheels' turn
                                       * x BENCH_YAW_SCALE (go-around and
                                       * pivot test) */
    double           pose_yaw;        /* the odometry heading last tick */
    double           x_m;             /* and position (the map's P lines) */
    double           y_m;
    double           stop_yaw;
    int32_t          start[BENCH_WHEELS];  /* g_distance at the start */
    int32_t          stop[BENCH_WHEELS];
    motion_request_t request;
    bench_wheel_t    wheel[BENCH_PID_STEPS][BENCH_WHEELS];
} bench_drive_t;

/** One line-test sample. */
typedef struct
{
    int16_t  centre_mm;                /* encoder centre travel since START */
    int16_t  heading_ddeg;             /* odometry, 0.1 degree, + left */
    uint16_t raw[LINE_CTRL_CHANNELS];  /* IR left, right, barcode */
    uint16_t front_mm;                 /* LINE + stop: latest VALID, 0 none */
    int8_t   steering_q7;              /* + line evidence to the right */
    uint8_t  status;                   /* line_ctrl_status_t */
    uint8_t  phase;                    /* LINE centre: bench_centre_phase_t */
    int16_t  wheel_mms[BENCH_WHEELS];  /* measured wheel speed, mm/s */
    int16_t  duty_pm[BENCH_WHEELS];    /* applied duty, per mille */
} bench_line_row_t;

typedef struct
{
    bool             b_moving;   /* a velocity primitive holds the wheels */
    bool             b_gap;      /* SEARCHING on the last command */
    bool             b_crossing; /* straight on past a junction */
    int32_t          cross_from_mm; /* centre travel where it was seen */
    uint32_t         gaps;       /* TRACKING to SEARCHING */
    uint32_t         junctions;  /* crossed straight on */
    int16_t          wheel_mms[BENCH_WHEELS]; /* latest motion_step output */
    int16_t          duty_pm[BENCH_WHEELS];
    uint32_t         rows;
    uint32_t         end_ms;
    const char *     p_reason;   /* why the test ended */
    bench_line_row_t end;        /* the sample that ended it */
    bench_line_row_t row[BENCH_LINE_ROWS];
} bench_line_t;

/** LINE + stop: the front sonar while the line test runs. */
typedef struct
{
    bool     b_pending;        /* a trigger awaits its echo */
    bool     b_detected;       /* an obstacle stopped the test */
    bool     b_robot;          /* the robot's rule would have halted */
    uint16_t last_mm;          /* latest VALID range (ceil mm), 0: none */
    uint16_t detect_mm;
    int16_t  detect_centre_mm; /* centre travel at detection */
    uint16_t robot_mm;
    int16_t  robot_centre_mm;
    uint32_t readings;         /* while driving */
    uint32_t invalid;          /* while driving, not VALID */
    uint32_t first_invalid_ms; /* since START */
    uint64_t ready_us;         /* servo settled at the centre */
} bench_front_t;

/** One SCAN test: the robot's scanner state is g_scanner. */
typedef struct
{
    bool         b_running;
    uint64_t     start_us;
    uint64_t     last_us;
    uint32_t     dt_max_us;
    uint32_t     done_ms;
    uint32_t     trigger_refused;  /* the robot faults SCAN on any of these */
    uint32_t     servo_refused;
    const char * p_reason;
} bench_scan_t;

/** Obstacle avoid phases (MISSION and BENCH_MODE_AVOID). No line sensors:
 * odometry holds the heading and tracks the sideways offset. The run maps the
 * obstacle, backs up to see more if an edge is out of sight (the only
 * reversing), drives a rectangle round the clearer side with a seen edge and
 * returns onto the original line.
 */
typedef enum
{
    BENCH_AVOID_STRAIGHT = 0, /* drive straight on the line, watching ahead */
    BENCH_AVOID_PAUSE,        /* stopped in front of the obstacle */
    BENCH_AVOID_MEASURE_L,    /* pivot left to the obstacle's left edge */
    BENCH_AVOID_MEASURE_R,    /* pivot right to its right edge */
    BENCH_AVOID_RECENTRE,     /* an edge unseen: pivot back to the line */
    BENCH_AVOID_REVERSE,      /* back straight up, then measure again */
    BENCH_AVOID_CREEPTURN,    /* pivot part way, sonar square to the face */
    BENCH_AVOID_CREEP,        /* creep out until the echo is lost: the edge */
    BENCH_AVOID_OUT,          /* pivot in place to the chosen side */
    BENCH_AVOID_STEPOUT,      /* drive out to the side clearance */
    BENCH_AVOID_ALIGN,        /* pivot in place back to the line heading */
    BENCH_AVOID_ALONG,        /* run parallel past the object */
    BENCH_AVOID_BACKPIVOT,    /* pivot in place toward the line */
    BENCH_AVOID_STEPIN,       /* drive back until on the line */
    BENCH_AVOID_HOMEPIVOT,    /* pivot back to the home heading, on course */
    BENCH_AVOID_CREEPBACK     /* far edge found: back along that creep */
} bench_avoid_phase_t;

/** Obstacle avoid working state. yaw_home is the original heading, held on the
 * straight and parallel legs; the out/back legs hold yaw_home -/+ side x the
 * angle. right_mm is the car's sideways offset from the original line
 * (+ right) and back_mm how far the pivot is behind the first stop (- once
 * ahead of it), both from the wheels, so the return lands back on the line
 * and the map stays fixed at the first stop.
 */
typedef struct
{
    uint8_t  phase;       /* bench_avoid_phase_t */
    /* original heading, held straight/parallel (+ left) */
    double   yaw_home;
    double   right_mm;    /* sideways offset from the line, + right */
    int32_t  last_centre; /* centre count last tick, for the offset integrals */
    int32_t  side;        /* chosen side: +1 right, -1 left */
    double   side_mm;     /* out-leg offset, from the measured edge */
    double   along_mm;    /* parallel-leg length, from the measured face */
    double   face_mm;     /* obstacle face, map x (ahead of the first stop) */
    double   left_mm;     /* obstacle's extent left of the line, measured */
    double   right_edge_mm; /* and right of it */
    double   back_mm;     /* pivot this far behind the first stop (- ahead) */
    double   back_from_mm; /* back_mm when this reverse began */
    double   miss_mm;     /* creep: the sonar's offset toward the side at the
                           * first of the lost echoes in a row */
    double   hit_mm;      /* and at the last echo off the face */
    uint16_t hit_range;   /* that echo's range */
    uint32_t creep_hits;  /* creep echoes off the face, this obstacle */
    int32_t  creep_deg;   /* creep angle off the line heading now */
    int32_t  creep_side;  /* side this creep measures: +1 right, -1 left */
    bool     b_far;       /* it is the far side's (not the way round's) */
    double   far_start_mm; /* right_mm when the far creep began */
    double   far_back_mm; /* back_mm when CREEPBACK began */
    bool     b_left_creep; /* that edge was measured by a creep (seen) */
    bool     b_right_creep;
    uint32_t far_backups; /* backups at the plan: the far creep's own
                           * reverses do not use up the way round's */
    double   creep_none_mm; /* widest first-miss offset of the creeps that
                             * had no echo at all */
    double   x0_mm;       /* odometry pose at the first stop (the map's */
    double   y0_mm;       /* origin, axes along yaw_home), for OBST */
    uint64_t pause_us;    /* when the car stopped in front */
    uint32_t readings;    /* g_front.readings last seen, for new beams */
    uint32_t misses;      /* readings in a row past the edge */
    uint16_t med_mm[BENCH_AVOID_MEDIAN]; /* sweeps: last readings, oldest */
    double   med_rad[BENCH_AVOID_MEDIAN]; /* first, and the car angle of each */
    uint32_t med_n;       /* how many of them are held */
    uint32_t med_skips;   /* sweeps: medians skipped as too spread */
    uint32_t close;       /* MISSION: readings in a row within reach */
    uint32_t backups;     /* reverses made for this obstacle */
    bool     b_hit;       /* the last beam hit the obstacle */
    bool     b_left_open; /* left sweep ended on the obstacle: edge unseen */
    bool     b_right_open; /* and the right */
    bool     b_creeping;  /* a reverse goes back to the creep, not the sweeps */
    bool     b_line_fix;  /* STEPIN: the front sensors found the line */
    bool     b_dark_on;   /* STEPIN: both front sensors dark now */
    bool     b_dark_known; /* and that stretch's start was seen */
    bool     b_ir_dark[2];  /* STEPIN: front sensors' class, with hysteresis */
    /* STEPIN: offset toward the line when both went dark */
    double   dark_from;
    uint32_t guard_hits;  /* ALONG/STEPIN: echoes in a row within the guard */
    uint32_t extends;     /* STEPIN: times the path past it was blocked */
    double   extra_mm;    /* ALONG runs this much further (the extends) */
    bool     b_blocked;   /* the way back was blocked at the last look */
    bool     b_turn_dark[2]; /* turn check: front sensors' class */
    bool     b_turn_known[2]; /* and that dark stretch's start was seen */
    double   turn_from[2]; /* heading where it went dark */
    double   turn_width_rad; /* the line's dark stretch, last found */
    int32_t  turn_dir;    /* HOMEPIVOT: +1 turning left, -1 right */
    bool     b_turn_found; /* HOMEPIVOT: line found, turning onto it */
    bool     b_turn_given_up; /* HOMEPIVOT: no line, by the wheels */
    double   turn_line_rad; /* HOMEPIVOT: the heading onto the line */
    bool     b_turn_search; /* HOMEPIVOT: past the window, looking on */
    bool     b_turn_seen; /* HOMEPIVOT: a crossing outside the window */
    double   turn_seen_rad; /* the one nearest the target */
    bool     b_turn_along; /* that is the line STEPIN ended on */
    double   turn_sum_rad; /* MEASURE_R: line headings found, from yaw_home */
    uint32_t turn_finds;  /* and how many */
    line_ctrl_hysteresis_t ir_hyst[2]; /* line checks: front sensors'
                                        * dark/light, this run's readings */
    bool     b_ir_own[2];   /* from this run's readings, else configured */
    uint16_t ir_peak[2];  /* highest front reading this phase */
    bool     b_sx_known[2]; /* STEPIN: this sensor's dark stretch began here */
    bool     b_sx_done[2];  /* and a line-wide one has ended, not yet paired */
    double   sx_from[2];  /* travel left (expect) where it went dark */
    double   sx_mid[2];   /* the ended stretch's middle (expect) */
    double   sx_width[2]; /* and length, mm */
    uint32_t cross_logs;  /* STEPIN: stretches not taken, logged */
    bool     b_line_err;  /* STEPIN: the car's angle to the line is known */
    double   line_err_rad; /* + the car turned left of square to the line */
    uint8_t  map[BENCH_MAP_ROWS][BENCH_MAP_COLS]; /* bench_map_cell_t */
} bench_avoid_t;

/** What LINE centre steered on, per sample. */
typedef enum
{
    BENCH_PHASE_TRACK = 0,  /* a sensor sees the line: KP * (d1 - d0) */
    BENCH_PHASE_GAP,        /* both white, line between them: straight */
    BENCH_PHASE_SEARCH,     /* both white too long: toward the last side */
    BENCH_PHASE_CROSSING    /* both dark: a crossing line, straight on */
} bench_centre_phase_t;

/** LINE centre: the line's side and history, and run counters. */
typedef struct
{
    int32_t  side;           /* +1 line last seen right, -1 left, 0 never */
    double   gap_m;          /* travel with both sensors white */
    double   clear_m;        /* travel off a crossing line */
    int32_t  last_counts;    /* encoder centre at the previous sample */
    bool     b_seen;         /* the previous sample saw the line */
    bool     b_crossing;
    bool     b_searching;
    uint64_t search_us;      /* start of the current search */
    uint8_t  phase;          /* bench_centre_phase_t */
    uint32_t samples;
    uint32_t in_phase[BENCH_CENTRE_PHASES];
    uint32_t full_steer;     /* samples at the yaw limit */
    uint32_t gaps;           /* seen to both white */
    uint32_t crossings;
    uint32_t searches;
    uint32_t search_max_ms;
    uint16_t raw_max[2];     /* darkest left and right raw seen */
    int32_t  seen_side;      /* LINE smooth: the side seeing it now, 0 none */
    double   on_m;           /* LINE smooth: travel since */
    double   mag;            /* LINE smooth: steering size, kept in a gap */
    double   cmd;            /* LINE smooth: the rate-limited steering */
    double   dark_m[2];      /* LINE smooth: travel dark, left and right */
    bool     b_straight;     /* the last step's bench_centre_straight */
    double   edge_m[2];      /* LINE edge: travel at BENCH_EDGE_CROSS */
    double   all_dark_m;     /* LINE smooth: travel with all three dark */
    double   turn;           /* LINE smooth: cmd averaged over travel */
    bool     b_both_dark;    /* LINE smooth: both see a line: a crossing */
    double   raw[2];         /* LINE smooth: front darkness 0-1, this step */
    double   side_dark;      /* and the side sensor's (the REC rows) */
    uint64_t last_us;        /* the previous sample */
    /* Junctions (BENCH_STRAIGHT_*): travel so far and the heading history
     * by travel.
     */
    double   path_m;
    double   yaw_ring[BENCH_STRAIGHT_SLOTS];
    double   ring_at_m;      /* travel at the newest slot */
    uint32_t ring_head;      /* the next slot to write (the oldest when full) */
    uint32_t ring_n;         /* slots filled */
} bench_centre_t;

/** LINE smooth: one steering setting. */
typedef struct
{
    double mps;    /* straight-line speed */
    double start;  /* steering when a sensor first sees the line */
    double ramp;   /* steering added per metre while it stays there */
    double slew;   /* steering change per second, at most */
} bench_smooth_t;

/** BARCODE read: the side sensor over one unit of its own path. */
typedef struct
{
    /* 32 bits (22 Nov): uint16 units (0.32 mm) ran out at 21 m, and the
     * store then took nothing more ("STORE FULL"); the run has no limit now.
     */
    uint32_t unit;   /* decoder units of side sensor path since the start */
    uint32_t centre; /* decoder units of centre path where it began */
    uint16_t raw;    /* mean of the readings in the unit */
} bench_code_sample_t;

/** One run of a hysteresis class (BAND keeps the last). */
typedef struct
{
    bool     b_black;
    uint16_t start;  /* sample index */
    uint16_t units;  /* width in decoder units */
    uint16_t peak;   /* darkest raw (bar) or lightest raw (space) */
    uint32_t dark;   /* darkness pass, bar: raw above its valleys x units;
                      * space: units it reads white (below the bar floor) */
} bench_code_run_t;

/** MISSION: one framed text of a cut's tries and the votes that framed it
 * (bit per swing, darkness then peaks: bench_code_try_agree). */
typedef struct
{
    char     text[BENCH_CODE_SIG];
    uint32_t swings;
    uint32_t first;  /* the first try that framed it */
} bench_code_sig_t;

/** MISSION: a group of at least one Code 39 character's elements. */
typedef struct
{
    uint32_t from_units;  /* side sensor path since the start */
    uint32_t to_units;
    uint32_t bars;
    uint32_t half_sds;    /* the swing it was decoded at */
    uint32_t gap_mm;      /* and the white that ended it */
    bool     b_framed;    /* *...* as read or reversed, confirmed */
    bool     b_single;    /* framed at one swing only: not a code */
    bool     b_reverse;   /* the frame read end to start */
    bool     b_sent;      /* its live line went out */
    bool     b_partial;   /* framed by one * only (bench_code_found_window) */
    bool     b_peaks;     /* a code with the peak rule's vote in it */
    uint32_t spec;        /* the first frame the spec checks refused, why */
    char     refused[BENCH_CODE_TEXT];  /* and its text */
    char     text[BENCH_CODE_TEXT];     /* as read */
    char     reverse[BENCH_CODE_TEXT];  /* reversed, if not framed as read */
    char     chars[BENCH_CODE_TEXT];    /* unframed: characters as read */
    char     chars_reverse[BENCH_CODE_TEXT]; /* and read end to start */
} bench_code_found_t;

typedef struct
{
    bool                   b_full;    /* more units than BENCH_CODE_SAMPLES */
    bool                   b_runs_full;
    uint32_t               count;
    uint32_t               runs;
    uint16_t               minimum;
    uint16_t               maximum;
    uint32_t               narrow_units;  /* last Code 39 decode */
    uint32_t               narrow_count;
    uint32_t               wide_units;
    uint32_t               wide_count;
    double                 unit_m;
    uint32_t               readings;      /* side sensor reads, all units */
    uint64_t               first_us;      /* first and latest read */
    uint64_t               last_us;
    int32_t                pending_unit;  /* the unit being read */
    uint32_t               pending_centre;
    uint32_t               pending_sum;
    uint32_t               pending_count;
    bool                   b_dark_rule;   /* bench_code39: darkness pass */
    bool                   b_stream;      /* MISSION: bench_code_stream */
    bool                   b_open;        /* a mark since the last pass */
    uint32_t               last_dark;     /* its unit */
    bool                   b_dark;        /* the last sample at the floor */
    uint32_t               dark_from;     /* the unit its dark run began */
    bool                   b_tape;        /* a run past BENCH_CODE_LINE_M */
    bool                   b_bar;         /* a bar's darkness since open */
    bool                   b_held;        /* FOLLOW held straight now */
    int32_t                held_from;     /* centre path at the hold, mm */
    uint32_t               holds;         /* straight holds over marks */
    uint32_t               held_mm;       /* and their path */
    bool                   b_let_go;      /* the hold let go for this group */
    uint32_t               let_gos;       /* groups where it let go */
    uint32_t               stream_floor;  /* raw that opens a group */
    uint32_t               stream_low;    /* and keeps it open */
    uint32_t               since_floor;   /* samples since it was set */
    uint32_t               groups_seen;   /* of a character or more */
    uint32_t               founds;
    uint32_t               founds_done;   /* sent groups dropped, this run */
    uint32_t               codes_done;    /* of them, framed codes */
    bench_code_found_t     found[BENCH_CODE_FOUND];
    bool                   b_trying;      /* a cut being decoded */
    uint32_t               cut_count;     /* its samples */
    uint32_t               cut_gap;       /* units kept after it */
    uint32_t               try_next;
    uint32_t               try_most;      /* framed by the best try */
    uint32_t               try_best;
    uint32_t               try_founds;    /* founds and groups_seen before */
    uint32_t               try_seen;
    bool                   b_agreed;      /* BENCH_CODE_AGREE swings agree */
    bool                   b_partial_ok;  /* the second round: one * do */
    uint32_t               sigs;          /* distinct framed texts tried */
    bench_code_sig_t       sig[BENCH_CODE_ALL_TRIES];
    uint32_t               cut_white;
    uint32_t               cut_sd;
    uint32_t               try_half_sds;
    uint32_t               try_gap_mm;
    uint32_t               try_rule;      /* BENCH_CODE_RULE_DARK or PEAKS */
    bool                   b_peak_agreed; /* the agreement had a peak vote */
    double                 code_lead;     /* white before the group, units */
    double                 code_trail;    /* and after; < 0: no bar there */
    uint32_t               spec_widths;   /* frames the spec checks refused */
    uint32_t               spec_gaps;
    uint32_t               spec_quiet;
    line_ctrl_hysteresis_t robot;     /* the robot decoder's DARK and LIGHT */
    uint16_t               white;     /* side sensor calibration */
    uint16_t               black;
    uint32_t               lines;     /* tape-dark spans, sample indexes */
    uint16_t               line_first[BENCH_CODE_LINES];
    uint16_t               line_last[BENCH_CODE_LINES];
    uint32_t               extremes;  /* turning points, a valley first */
    uint16_t               extreme[BENCH_CODE_EXTREMA];
    bench_code_sample_t    sample[BENCH_CODE_SAMPLES];
    bench_code_run_t       run[BENCH_CODE_RUNS];
} bench_code_t;

/** MISSION: one D0 group (bench_d0_sample), its changes as path units. */
typedef struct
{
    uint32_t edge[BENCH_D0_EDGES];  /* the first is dark, then alternating
                                     * (32 bits: 16 ran out at 21 m) */
    uint32_t edges;                 /* changes counted, past the store too */
} bench_d0_group_t;

/** MISSION: the barcode module's D0 over a run. */
typedef struct
{
    bench_d0_group_t group[BENCH_D0_GROUPS];  /* closed, waiting to be sent */
    bench_d0_group_t open;
    uint32_t         queued;     /* closed groups in group[] */
    uint32_t         high;       /* readings high */
    uint32_t         low;        /* and low */
    uint32_t         changes;    /* level changes this run */
    uint32_t         groups;     /* groups closed */
    uint32_t         reads;      /* framed codes */
    uint32_t         dropped;    /* closed while the queue was full */
    uint32_t         shorts;     /* closed under BENCH_D0_BARS_MIN bars */
    uint32_t         unit;       /* the last reading's unit */
    uint32_t         light_from; /* unit the light now began */
    uint16_t         quiet;      /* BENCH_CODE_QUIET_MM in units */
    bool             b_started;
    bool             b_level;    /* the last reading */
    bool             b_open;
    bool             b_widths;   /* group[0]'s result went out: widths next */
} bench_d0_t;

/** AUTO-CENTRE steps. */
typedef enum
{
    BENCH_AUTO_SEEK = 0,  /* pivot until the sought sensor finds the line */
    BENCH_AUTO_PAUSE,     /* halted, then the next pivot */
    BENCH_AUTO_MIDDLE,    /* pivot to the heading between the two finds */
    BENCH_AUTO_SETTLE     /* halted until still, then done */
} bench_auto_phase_t;

/** AUTO-CENTRE: headings are the odometry's, rad, + left (CCW); index 0 is
 * the left sensor, 1 the right.
 */
typedef struct
{
    bool         b_active;       /* centring, before the line test */
    bool         b_failed;
    bool         b_white;        /* the sought sensor was white this sweep */
    bool         b_candidate;    /* dark since candidate_rad */
    bool         b_limit;        /* a sweep reached its limit, nothing found */
    bool         b_found[2];
    uint8_t      phase;          /* bench_auto_phase_t */
    uint8_t      next;           /* after a pause: SEEK or MIDDLE */
    int32_t      turn;           /* +1 left: seek the right sensor; -1 */
    double       candidate_rad;
    double       found_rad[2];   /* left found turning right, right turning
                                  * left */
    double       start_rad;
    double       middle_rad;
    double       end_rad;        /* where it handed over or failed */
    double       coast_rad;      /* run-on after the last halt */
    double       mark_rad;       /* the stall check; in a pause, the halt */
    uint64_t     mark_us;
    uint64_t     start_us;
    uint64_t     phase_us;
    uint32_t     sweeps;
    uint32_t     corrections;
    uint32_t     done_ms;
    uint16_t     raw[2];         /* front IR at the end */
    const char * p_fail;
} bench_auto_t;
extern const char g_code39_chars[BENCH_CODE_CHARS + 1U];
extern const char * const g_line_names[BENCH_LINE_STATES];
extern uint32_t      g_steps_done;
extern bench_sweep_t g_sweep;
extern bench_ramp_t  g_ramp;
extern bench_cal_t   g_cal;
extern char          g_text[BENCH_TEXT_BYTES];
extern uint32_t      g_length;
extern uint32_t      g_live_length;
extern uint32_t      g_live_sent;
extern uint8_t          g_frame[BENCH_FRAME_BYTES];
extern bitpack_writer_t g_frame_w;
extern uint32_t         g_frame_kind;
extern bool             gb_frame_text;
extern uint64_t      g_live_due_us;
extern uint16_t      g_live_side_max;
extern uint16_t      g_live_front[2];
extern uint16_t      g_live_front_min;
extern uint32_t      g_live_front_none;
extern uint32_t      g_raw_length;
extern uint32_t      g_raw_sent;
extern uint32_t      g_raw_cuts;
extern uint32_t      g_raw_done;
extern bool          gb_raw_head;
extern bench_d0_t    g_d0;
extern bool          gb_mqtt;
extern bool          gb_watchdog;
extern bool          gb_accel;
extern int32_t       g_distance[BENCH_WHEELS];
extern uint16_t      g_last_total[BENCH_WHEELS];
extern uint32_t      g_encoder_errors;
extern uint32_t      g_send_tries;
extern int32_t       g_stall_pct;
extern int32_t       g_delta[BENCH_WHEELS];
extern bench_drive_t g_drive;
extern bench_mode_t  g_mode;
extern line_ctrl_runtime_t g_line_runtime;
extern line_ctrl_state_t   g_line_state;
extern bench_line_t        g_line;
extern bench_front_t       g_front;
extern bench_scan_t        g_scan;
extern bench_avoid_t       g_avoid;
typedef enum
{
    BENCH_NAV_IDLE = 0,  /* line following; a junction may start a move */
    BENCH_NAV_ADVANCE,   /* straight on until the axle is over the crossing */
    BENCH_NAV_PIVOT      /* turning in place onto the new branch */
} bench_nav_phase_t;
typedef struct
{
    char     letters[NAVMAP_MOVES]; /* the map's letter per move (MQTT) */
    bool     b_mapped;     /* a map has arrived since boot */
    bool     b_say_map;    /* its line is still to go out (mid-run) */
    char     letter;       /* the card's letter by vote, '\0' none */
    bool     b_card;       /* a card has reads waiting for a junction */
    int32_t  card_from;    /* its side sensor path (mm), as the vote's */
    int32_t  card_to;
    /* per decoder (Decoder 1, Decoder 2, D0): its letter, '\0' no read
     * or not in yet
     */
    char     read[BENCH_NAV_DECODERS];
    uint32_t reported;     /* a bit per decoder that reported this card */
    int32_t  junction_mm;  /* encoder centre at the last junction: reports
                            * for cards before it are late, ignored */
    bool     b_say_code;   /* its line is still to go out */
    uint8_t  phase;        /* bench_nav_phase_t */
    uint8_t  move;         /* navmap_move_t being made */
    int32_t  from_mm;      /* encoder centre at the junction (mm) */
    double   hold;         /* heading held while advancing (rad, + left) */
    double   turned;       /* PIVOT: turn so far (rad, + left) */
    double   last_heading; /* PIVOT: heading last tick */
    uint32_t junctions;    /* junctions met this run */
    uint32_t moves;        /* turns made this run */
    bool     b_waiting;    /* stopped at a junction for the decode */
    uint64_t wait_us;      /* since when */
    int32_t  wait_centre;  /* encoder centre where that junction was seen */
    uint32_t waits;        /* junctions waited at this run */
    bool     b_probe;      /* both dark: straight on, looking for the line */
    int32_t  probe_from;   /* encoder centre (counts) where it went dark */
    double   probe_heading; /* heading held meanwhile (rad, + left) */
    uint32_t turns;        /* dark bands that were turns this run */
    int32_t  guard_to_mm;  /* no new junction before this centre (mm) */
} bench_nav_t;
extern bench_nav_t         g_nav;
extern uint32_t            g_avoid_queued;
extern uint32_t            g_avoid_dropped;
extern uint64_t            g_pose_due_us;
extern bench_centre_t      g_centre;
extern bench_code_t        g_code;
extern bench_auto_t        g_auto;

void bench_tick (void);
uint8_t bench_command (void);
void bench_track (void);
void bench_idle (uint8_t command);
void bench_summary (void);
bool bench_has_auto (bench_mode_t mode);
void bench_deg (double radians);
void bench_sample (void);
void bench_report (void);
int32_t bench_mm (int32_t counts);
int32_t bench_round (double value);
void bench_slowest (uint32_t * p_most, uint64_t start_us);

/* The car's barcode answer: the three decoders' vote, at the end of the
 * file. */
#define BENCH_VOTE_DECODER1 (0U)
extern bool gb_rec_keep;

bool bench_rtos_start (const char ** pp_what, int32_t * p_err);
void bench_control_task (INT stacd, void * exinf);
void bench_safety_task (INT stacd, void * exinf);
void bench_safety_cyc (void * exinf);
uint8_t bench_poll_command (void);
void bench_run_loop (void);
#define BENCH_FIT_BINS (640U)       /* 1 mm bins: at most 640 mm decoded */
#define BENCH_FIT_CANDS (78U)       /* 39 characters, both directions */
#define BENCH_FIT_SHORT (10U)
#define BENCH_FIT_TOP (6U)          /* largest humps, six of them tried (8
                                     * tried offline: more wrong reads) */

typedef enum
{
    BENCH_FIT_IDLE = 0,
    BENCH_FIT_HUMPS_PHASE,
    BENCH_FIT_LSQ,
    BENCH_FIT_SHAPE,
    BENCH_FIT_CANDS_PHASE,
    BENCH_FIT_FINE
} bench_fit_phase_t;

typedef enum
{
    BENCH_FIT_WHY_FIT = 0,
    BENCH_FIT_WHY_MISFIT,  /* six humps, no character's wide bars */
    BENCH_FIT_WHY_LONG,    /* dark longer than a card */
    BENCH_FIT_WHY_BUSY     /* two cards already waiting or being fitted */
} bench_fit_why_t;

typedef struct
{
    float    sse;
    float    m;
    float    sigma;
    float    x0;
    float    c0;
} bench_fit_best_t;

typedef struct
{
    bench_fit_phase_t phase;
    /* the run's counts */
    uint32_t reads;
    uint32_t no_reads;
    uint32_t skipped;
    /* the card being fitted, from the bins */
    uint32_t joins;
    uint32_t bins;                 /* bins in the window */
    uint32_t valid;                /* bins with readings */
    float    x_first;              /* mm since the start at bin 0 */
    int32_t  white;
    int32_t  sd;
    float    dark_from;            /* mm, first and last dark reading */
    float    dark_to;
    int16_t  vf[BENCH_FIT_BINS];   /* every bin, gaps filled */
    uint16_t valid_bin[BENCH_FIT_BINS];
    int16_t  valid_v[BENCH_FIT_BINS];   /* reading - white */
    double   sv;                   /* sums over the valid bins */
    double   svv;
    /* stage 1 */
    uint32_t humps;
    float    top_pos[BENCH_FIT_TOP]; /* the largest humps' centres, mm */
    uint32_t top_n;
    uint32_t mask;                 /* six of them: bits of top_pos */
    float    pos[6];
    float    pos_mean;
    uint32_t c;                    /* next candidate */
    float    rel[BENCH_FIT_CANDS]; /* rms / module, the best choice */
    float    rms[BENCH_FIT_CANDS];
    float    x0s[BENCH_FIT_CANDS];
    float    ms[BENCH_FIT_CANDS];
    uint8_t  short_n;
    uint8_t  short_cand[BENCH_FIT_SHORT];
    float    s1_rms;
    float    s1_rel;
    /* stage 2 */
    uint32_t k;                    /* short-list index */
    uint32_t tpl;                  /* template index in the phase */
    uint32_t xi;                   /* start index */
    bool     b_rendered;
    float    tpl_m;
    float    tpl_sigma;
    uint32_t tpl_points;
    bench_fit_best_t best[BENCH_FIT_SHORT];
    bench_fit_best_t shape;        /* the pass's spot and dead zone */
    uint32_t shape_n;              /* candidates of the closest class */
    bench_fit_best_t centre;       /* FINE's centre */
    /* cost */
    uint32_t cpu_us;
    uint32_t steps;
    uint32_t work;                 /* bin tests, all dead zones */
    /* the result, for its Decoder 2 line */
    bench_fit_why_t why;
    bool     b_read;
    char     text_char;
    bool     b_text_rev;
    char     text_other;
    bool     b_text_other_rev;
    float    text_margin;
    float    text_m;
    float    text_sigma;
    float    text_c0;
} bench_fit_t;
extern bench_fit_t g_fit;
extern uint32_t    g_fit_line_count;

 /* pushed in this step's run */
/* The card being binned (g_fit_sum, g_fit_cnt) while the last is fitted. */
typedef struct
{
    bool     b_used;
    uint32_t u0;                   /* decoder units at bin 0 */
    uint32_t k16;                  /* bins per unit, 16.16 */
    uint32_t first_u;              /* first and last dark reading, units */
    uint32_t last_u;
    uint32_t binned_u;             /* last sample binned, units */
    uint32_t joins;
    int32_t  white;
    int32_t  sd;
} bench_fit_bin_t;
extern bench_fit_bin_t g_fit_bin;
extern bool        gb_fit_force;
extern bool        gb_fit_boost;
#if defined(PICO_ON_DEVICE) && (1 == PICO_ON_DEVICE)
#undef CLOCKS_BASE
#undef RESETS_BASE
#undef IO_BANK0_BASE
#undef IO_QSPI_BASE
#undef PADS_BANK0_BASE
#undef XOSC_BASE
#undef PLL_SYS_BASE
#undef PLL_USB_BASE
#undef PWM_BASE
#undef TIMER_BASE
#undef SIO_BASE
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "hardware/i2c.h"
#pragma GCC diagnostic pop
#endif

void bench_untyped (void);

#include "mission_board.h"
#include "mission_frames.h"
#include "mission_drive.h"
#include "mission_line.h"
#include "mission_barcode.h"
#include "mission_terrain.h"
#include "mission_compass.h"
#include "mission_avoid.h"

#endif /* MISSION_PRIVATE_H */

/* End of file. */

