/** @file bench_main.c
 * @brief BENCH IMAGE, NOT THE ROBOT FIRMWARE: the measurements the mission
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
 * Output: plain text. MQTT build: the telemetry topic, read it with
 * mosquitto_sub -N. Offline build: UART0 on GP0, 115200 8N1 (the sonar is
 * not used). Console prints mask interrupts and can lose encoder edges, so
 * the sweep table is printed only after the motors stop; read wheel
 * distance on the MQTT build.
 */
/* math.h and string.h bring the toolchain's size_t: skip the kernel's own
 * typedef, as lib/liblwip/sysdepend/pico_rp2040/sys_arch.c does.
 */
#define PROHIBIT_DEF_SIZE_T
#include "app_cfg.h"
#include "barcode.h"
#include "bench.h"
#include "board.h"
#include "line_ctrl.h"
#include "motion.h"
#include "net_mqtt_ipc.h"
#include "robot_config.h"
#include "scanner.h"
#include "terrain.h"
#include <math.h>
#include <string.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

/* build.sh writes these into the bench copy of robot_config.h. */
#ifndef BENCH_MOTION
#define BENCH_MOTION (0)
#endif
/* Pivot-test image (S2-U10), built from a copy with BENCH_PIVOT_TEST 1:
 * START turns 90 deg on the spot by the encoders, at the go-around's turn
 * rate (LEFT 90 / RIGHT 90), left on starts 1-4 and right on starts 5-8,
 * then again. Four make a full circle, so the real heading error shows
 * against tape on the floor. The mission images keep it 0.
 */
#ifndef BENCH_PIVOT_TEST
#define BENCH_PIVOT_TEST (0)
#endif
#ifndef BENCH_ROBOT_MAX_DUTY
#error "BENCH_ROBOT_MAX_DUTY (the robot's ROBOT_MAX_DUTY) comes from build.sh"
#endif
#if !defined(BENCH_ROBOT_SERVO_MIN_US) || !defined(BENCH_ROBOT_SERVO_MAX_US)
#error "BENCH_ROBOT_SERVO_MIN_US/MAX_US (robot endpoints) come from build.sh"
#endif

/* A 5 ms delay plus the sensor reads takes about 6.75 ms, so every window
 * is timed with board_time_us(), not counted in ticks.
 */
#define BENCH_TICK_MS (5U)
#define BENCH_REPORT_US (1000000U)  /* sensor report period */
#define BENCH_SETTLE_US (500000U)   /* at each duty before counting */
#define BENCH_MEASURE_US (500000U)  /* then counted */
#define BENCH_REST_US (1000000U)    /* braked between directions */
#define BENCH_BUTTON_TICKS (4U)     /* debounce on START */
/* counts/s (about 8 mm/s): the wheel turns. Twitches of a sticking wheel
 * stay below it; the slowest real turning on 27 Oct was about 160.
 */
#define BENCH_MOVING_RATE (100)
#define BENCH_RATE_MAX (32767)
#define BENCH_WATCHDOG_MS (1000U)
#define BENCH_TEXT_BYTES (160U)
#define BENCH_SEND_TRIES (25U)        /* about 0.17 s per frame while idle */
#define BENCH_TABLE_SEND_TRIES (450U) /* about 3 s: the motors are stopped */
#define BENCH_SERVO_CENTRE_US (1500U)
#define BENCH_SERVO_FINE_US (10U)
#define BENCH_SERVO_COARSE_US (100U)
#define BENCH_STALL_MARGIN_PCT (2)
#define BENCH_WHEELS (2U)
#define BENCH_LEFT (0U)
#define BENCH_RIGHT (1U)

/* Motion image. */
#define BENCH_PID_STEPS (3U)
#define BENCH_PID_STEP_US (4000000U)    /* each speed step */
#define BENCH_PID_STEADY_US (2000000U)  /* its last 2 s: mean and ripple */
#define BENCH_TRACE_US (100000U)
#define BENCH_TRACE_ROWS (20U)          /* the first 2 s of each step */
#define BENCH_RISE (0.90)               /* rise: 90% of the target */
#define BENCH_BAND (0.10)               /* settled: within 10% of it */
#define BENCH_DRIVE_M (1.0)             /* S2-U09 */
#define BENCH_AFTER_US (1000000U)       /* drive tests report again 1 s on */
#define BENCH_STATUS_NAMES (7U)
#define BENCH_LINE_MAX_M (5.0)          /* line test: track runs cross junctions */
/* Straight on past a junction: at least BENCH_LINE_CROSS_MM from where it was
 * seen (the 18 mm track line, then clear of it), and on while both sensors
 * stay black, up to BENCH_LINE_CROSS_MAX_MM.
 */
#define BENCH_LINE_CROSS_MM (50)
#define BENCH_LINE_CROSS_MAX_MM (150)
#define BENCH_LINE_MAX_US (60000000U)
#define BENCH_LINE_TRACE_US (250000U)
#define BENCH_LINE_ROWS (241U)          /* 0..60 s every BENCH_LINE_TRACE_US */
/* LINE centre and smooth run twice as long, for two laps (29 Oct), with half
 * as many trace rows per second: the trace takes as long to send.
 */
#define BENCH_CENTRE_MAX_US (120000000U)
#define BENCH_CENTRE_TRACE_US (500000U)
/* Faster line tests; with the FOLLOW law (half speed at full steering, yaw
 * clamped to ROBOT_TURN_RATE_RPS) no wheel exceeds ROBOT_MAX_SPEED_MPS.
 */
#define BENCH_LINE_FAST_MPS (0.12)
#define BENCH_LINE_STATES (6U)
#define BENCH_LINE_LOST_MS (500U)       /* white gaps of the uneven tape */
#define BENCH_ID_LAST (255U)
/* LINE + stop: within 10 cm of the obstacle (the 120 mm/s line test rolled
 * about 3 mm after its halt on 28 Oct; the sonar reads about 7 mm long).
 */
#define BENCH_LINE_STOP_MM (100U)
#define BENCH_SCAN_MAX_US (20000000U)   /* robot_app_scan_cycle's limit */
#define BENCH_SCAN_PER_ROW (8U)         /* beams per printed line */
#define BENCH_SCAN_STEP_DEG (2)         /* scanner.c's fine beam grid */
#define BENCH_SCAN_SIDES (3U)
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
 *   CREEPTURN pivot ROBOT_SCAN_HALF_ANGLE_DEG toward the chosen side, the
 *             servo turning the sonar back the other way: square to the
 *             face again,
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
#define BENCH_AVOID_MISSES (2U)        /* readings past the edge to end a side */
#define BENCH_AVOID_CONE_DEG (15.0)    /* HC-SR04 measuring angle (datasheet):
                                        * the cone's half-angle, taken off
                                        * each hit's angle */
#define BENCH_AVOID_ANGLE_DEG (90)     /* out/back turns: a full right/left */
#define BENCH_AVOID_CLEAR_MM (70.0)    /* gap between car and obstacle */
#define BENCH_AVOID_HOLD_KP (4.0)      /* straight/diagonal heading-hold gain */
#define BENCH_AVOID_CAR_WIDTH_MM (160.0)/* 13 cm body + 3 cm QR sensor */
#define BENCH_AVOID_PIVOT_TOL_DEG (1.5)/* a pivot is done this close */
#define BENCH_AVOID_CONFIRM (2U)       /* MISSION: readings in a row in reach */
#define BENCH_AVOID_BACKUP_MM (150.0)  /* reverse this far to see more */
#define BENCH_AVOID_BACKUPS (2U)       /* reverses at most, then plan anyway */
#define BENCH_AVOID_REVERSE_MPS (0.06) /* reversing speed */
#define BENCH_AVOID_CREEP_MPS (0.06)   /* creeping speed: ~4 mm a reading */
#define BENCH_AVOID_FRONT_MM (130.0)   /* the car's front (the front sensors'
                                        * front end) ahead of the pivot
                                        * (ruler, 29 Oct) */
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
/* MISSION sends the go-around's lines live (bench_avoid_say): a flush would
 * wait on the radio between control steps (SENSOR_FAULT). Up to
 * BENCH_AVOID_LINES wait to go out; more are counted, dropped. For the
 * dashboard's map it also sends the obstacle (OBST), the map rows the sonar
 * hit (M) and, every BENCH_POSE_US, the car's odometry pose (P).
 */
#define BENCH_AVOID_LINES (32U)
#define BENCH_AVOID_LINE_BYTES (112U)
#define BENCH_POSE_US (500000U)
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
#define BENCH_MAP_ROWS (60U)           /* -400..+800 mm along the line: the
                                        * go-around and two back-ups */
#define BENCH_MAP_COLS (41U)           /* -410..+410 mm across, + left */
#define BENCH_MAP_MID (20)             /* column of the line (y = 0) */
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
#define BENCH_CENTRE_PHASES (4U)
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
/* Faster sets (29 Oct, 180 mm/s target): the motion wheel limit is raised
 * from ROBOT_MAX_SPEED_MPS for LINE smooth alone, and the speed falls toward
 * BENCH_SMOOTH_TURN_MPS as the steering grows, about the full-steering speed
 * that went round the track at 110-130 mm/s. At 130 mm/s the straight-line
 * duty was about 19% of the 35% allowed.
 */
#define BENCH_SMOOTH_WHEEL_MPS (0.20)
#define BENCH_SMOOTH_TURN_MPS (0.085)
/* MISSION's straight-line speed, smooth A's steering. 120 mm/s on 30 Oct to
 * see whether the small card's narrow bars, blurred together at 180, read
 * apart slower (about 2.4 readings per decoder unit instead of 1.6).
 */
#define BENCH_MISSION_MPS (0.08)
#define BENCH_SMOOTH_YAW_RPS (0.95)     /* yaw at full steering, and its cap */
/* Both sensors dark while steering at least this much: the line crossing
 * under both in a bend (29 Oct, twice), not a crossing line: keep turning.
 */
#define BENCH_SMOOTH_BEND (0.4)
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
 * BENCH_SMOOTH_DARK_M of travel at BENCH_SMOOTH_DARK or darker, and both
 * make a crossing only after BENCH_SMOOTH_CROSS_M, wider than any bar. A
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
#define BENCH_SMOOTH_CROSS_M (0.013)
/* BARCODE read. A sample per robot decoder unit, 8 half-counts (robot_app.c
 * ROBOT_APP_BARCODE_UNIT_HC, 0.32 mm): 2560 units cover the 0.8 m limit.
 * Each is the mean of the side sensor's readings in that unit, one per
 * kernel tick (CNF_TIMER_PERIOD, 1 ms): a cyclic handler wakes the bench
 * task, the fastest it can read and still wait. It runs at priority 1, the
 * radio at 9, so a busy loop between ticks would stop MQTT. A wake-up that
 * never comes ends the sleep after BENCH_CODE_POLL_TMO_MS.
 */
#define BENCH_CODE_POLL_MS (1U)
#define BENCH_CODE_POLL_TMO_MS (1)
#define BENCH_CODE_MPS (0.08)
#define BENCH_CODE_MAX_M (0.8)
#define BENCH_CODE_UNIT_HC (8)
#define BENCH_CODE_SAMPLES (2560U)
#define BENCH_CODE_RUNS (160U)
#define BENCH_CODE_GROUPS (6U)          /* dark groups reported */
#define BENCH_CODE_RUNS_PER_ROW (20U)
#define BENCH_CODE_TRACE_PER_ROW (25U)
#define BENCH_CODE_TRACE_MAX_MM (900)
#define BENCH_CODE_GROUP_GAP_MM (20.0)  /* white that separates dark groups */
#define BENCH_CODE_CONTRAST (1000U)     /* raw max - min worth a second pass */
#define BENCH_CODE_OWN_ON (0.55)
#define BENCH_CODE_OWN_OFF (0.40)
#define BENCH_CODE_CHARS (44U)
#define BENCH_CODE_ELEMENTS (9U)        /* per Code 39 character */
#define BENCH_CODE_TEXT (16U)
#define BENCH_CODE_FRAMED (3U)  /* '*', a character, '*': the least framed */
/* One-* partial frames (bench_code_found_window), off: replayed on the 30 Oct
 * 21:51 cuts, the big *A* card's second pass read D* end to start, a wrong
 * code (the same group read *A in order with the windows tried the other
 * way round).
 */
#define BENCH_CODE_PARTIAL (0)
/* The card's own levels: a print can read far lighter than the tape the
 * side sensor is calibrated on (29 Oct track card: a wide bar 2085 still,
 * the tape 4043). Shares of the calibrated white-to-tape span or of the
 * card's own contrast; the line length is the spec's (bars at most 9 mm,
 * the line 18 mm).
 */
#define BENCH_CODE_FLOOR (0.10)    /* a bar peaks this far above white */
#define BENCH_CODE_NOISE (0.025)   /* a smaller swing is noise */
#define BENCH_CODE_SWING (0.20)    /* of the card's contrast: bar to space */
#define BENCH_CODE_LINE_M (0.013)  /* tape-dark this long is a line */
#define BENCH_CODE_LINES (8U)
#define BENCH_CODE_EXTREMA (480U)
/* Fourth pass, darkness (bench_code_dark): white is the run's commonest
 * side sensor reading and its noise comes from the readings below it (a
 * mark only reads darker, so that half is white alone): the level
 * BENCH_CODE_ONE_SD_SHARE of the way up it lies one standard deviation
 * under white (2 x 15.87 % of a normal). Turning points need
 * BENCH_CODE_DARK_SDS of those. The spec's 3 mm bars read narrower than the
 * side sensor's view and its 9 mm bars hardly wider, but far darker, so a
 * bar's darkness summed across it tells wide from narrow.
 */
#define BENCH_CODE_ADC_LEVELS (4096U)
#define BENCH_CODE_ONE_SD_SHARE (0.3173)
#define BENCH_CODE_DARK_SDS (4U)
#define BENCH_CODE_DARK_PER_ROW (10U)
/* MISSION: the store streams (bench_code_stream). The floor that opens a
 * group, white + BENCH_CODE_DARK_SDS sd, is renewed every
 * BENCH_CODE_FLOOR_EVERY stored samples; white with no group open is cut
 * back to the last BENCH_CODE_QUIET_MM once BENCH_CODE_STREAM_HOLD
 * samples are stored. The first BENCH_CODE_FOUND groups of a character or
 * more are kept for the report.
 */
#define BENCH_CODE_FLOOR_EVERY (64U)
#define BENCH_CODE_STREAM_HOLD (512U)
#define BENCH_CODE_FOUND (12U)
/* MISSION decodes each cut at turning-point swings of BENCH_CODE_TRY_MIN to
 * BENCH_CODE_TRY_MAX half sds (bench_code_try): the noise is a whole
 * number of ADC steps from the run so far (3 or 4 on 30 Oct), and a sheet
 * that is not flat splits bars at small swings while large ones lose the
 * 3 mm bars.
 */
#define BENCH_CODE_TRY_MIN (6U)
#define BENCH_CODE_TRY_MAX (12U)
/* MISSION also tries groups that end at BENCH_CODE_QUIET_MM of white, the
 * Code 39 quiet zone (10 narrow widths) of the spec's 3 mm narrow bar, and
 * cuts the stream there: at 180 mm/s on 30 Oct both cards split at 20-23 mm
 * of white inside them.
 */
#define BENCH_CODE_QUIET_MM (30.0)
#define BENCH_CODE_GAPS (2U)
#define BENCH_CODE_SWINGS (7U)  /* TRY_MIN..TRY_MAX */
#define BENCH_CODE_TRIES (BENCH_CODE_GAPS * BENCH_CODE_SWINGS)
/* A cut's framed text is a code only when tries at BENCH_CODE_AGREE
 * different swings frame the same text (bench_code_tries_step): on 30 Oct
 * at 180 mm/s one swing alone read the expanded card *F*. Good 80 mm/s
 * reads frame at 4 to 7 swings.
 */
#define BENCH_CODE_AGREE (2U)
/* A second opinion (bench_code39_dark): when the darkness tries frame a text
 * but do not agree, the same tries again with each bar judged by its peak
 * and each space by its valley, which, unlike darkness, do not depend on the
 * valleys beside a bar. Each rule and swing is one vote; a code needs
 * BENCH_CODE_AGREE votes, one of them darkness. Widths were no second
 * opinion: 31 Oct on the recorded cuts and traces they framed only the big
 * cards and missed the normal-size *Y*, whose bars read 2 to 5 mm wide.
 */
#define BENCH_CODE_RULE_DARK (0U)
#define BENCH_CODE_RULE_PEAKS (1U)
#define BENCH_CODE_RULES (2U)
#define BENCH_CODE_ALL_TRIES (BENCH_CODE_RULES * BENCH_CODE_TRIES)
#define BENCH_CODE_DARK_VOTES ((1U << BENCH_CODE_SWINGS) - 1U)
/* Code 39 spec checks on a framed text (bench_code39_spec), from the spec
 * alone: every character is 6 narrow and 3 wide elements, wide 2 to 3
 * narrow, so 12 to 15 narrow widths; the gap between characters is at most
 * 3 narrow widths and the quiet zone at least 10. Measured from the
 * frame's own character widths, whole characters, which the sensor's blur
 * shortens alike (its single elements it does not: 31 Oct normal cards
 * read wide bars 4 to 5 mm, narrow 2 to 4, so no element ratio is checked).
 */
#define BENCH_CODE_CHAR_MIN_X (12.0)  /* narrow widths per character, 2:1 */
#define BENCH_CODE_CHAR_MAX_X (15.0)  /* and at 3:1 */
#define BENCH_CODE_GAP_MAX_X (3.0)
#define BENCH_CODE_QUIET_X (10.0)
#define BENCH_SPEC_OK (0U)            /* bench_code39_spec results */
#define BENCH_SPEC_WIDTHS (1U)
#define BENCH_SPEC_GAPS (2U)
#define BENCH_SPEC_QUIET (3U)
#define BENCH_SPEC_RESULTS (4U)
#define BENCH_CODE_SIG (40U)    /* a try's framed texts, end to end */
/* MISSION sends while it drives (bench_live_step): one line at a time through
 * g_live, pushed frame by frame as the MQTT queue takes them, never waiting;
 * each group's result as it is decoded, else a status line every
 * BENCH_LIVE_US. After the stop only a few lines of totals: the link takes
 * about one 20-byte frame per 200 ms, so a raw dump or trace there held the
 * car for minutes (30 Oct).
 */
#define BENCH_LIVE_US (1000000U)
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
#define BENCH_RAW_LINE (120U)
#define BENCH_RAW_ZERO ('#')
#define BENCH_RAW_TOP ('~')
#define BENCH_RAW_GAP ('!')
#define BENCH_RAW_BELOW (10)
#define BENCH_RAW_STEP (2)
#define BENCH_RAW_KNEE (64)    /* 'c': white + 118 */
#define BENCH_RAW_HIGH (140)   /* '~': white + 3898 and above */
#define BENCH_LIVE_RAW_US (3000000U)
/* MISSION also reads the barcode module's digital D0 (board_barcode_d0, GP7,
 * from 2 Nov 2026) at every side sensor reading and keeps each change at the
 * unit of the sensor's own path (bench_d0_sample). The level seen less is
 * dark (white dominates a run). Changes from the first dark one to the last
 * one before BENCH_CODE_QUIET_MM of light form a D0 group; each group is
 * decoded as Code 39 with the 3 widest of every 9 elements wide (no
 * threshold to tune) and sent as two live lines, the result and its element
 * widths (bench_d0_text). Report only: the car's answer stays the CODE lines.
 * A group keeps BENCH_D0_EDGES changes (15 bars, *X*, are 30), and
 * BENCH_D0_GROUPS closed groups wait to be sent; more are counted, dropped.
 */
#define BENCH_D0_EDGES (72U)
#define BENCH_D0_GROUPS (4U)
#define BENCH_D0_WIDTHS (36U)  /* widths on one live line (160 bytes) */
#define BENCH_D0_BARS_MIN (3U) /* fewer: a line crossed, counted, not sent */
/* MISSION also measures the ramp (bench_ramp_step): the robot's terrain
 * module on each fresh accelerometer sample, fed as robot_app_imu_step feeds
 * it, against a flat reference from the car's last idle second before START
 * (the stored ROBOT_IMU_REFERENCE_* with fewer than BENCH_RAMP_REF_MIN
 * samples, or any axis moving more than BENCH_RAMP_STILL_MG). The module's
 * hump starts at 4.6 deg of pitch, and the track's 8 mm ramp tilts the car
 * about 4 deg (its axle-to-castor base), so on 30 Oct only steering jolts
 * opened humps. The bench therefore keeps the height along the whole path on
 * the same filtered pitch (the last held over a refused sample) and measures
 * it above the lowest level of the last BENCH_RAMP_CELLS cells of
 * BENCH_RAMP_CELL_M behind (two car lengths, ROBOT_ROBOT_LENGTH_M: off any
 * ramp left behind, and too short for a pitch bias to build up). A rise
 * opens at the module's minimum hump above that floor and closes under half
 * of it. Each rise goes out live (RAMP), so do the module's own humps
 * (TERRAIN); the status line carries pitch, roll and the height above floor.
 */
#define BENCH_RAMP_REF_MIN (5U)
#define BENCH_RAMP_STILL_MG (50)
#define BENCH_RAMP_CELL_M (0.010)
#define BENCH_RAMP_CELLS (44U)
#define BENCH_RAMP_OFFSET_M (4.0 * ROBOT_ROBOT_LENGTH_M)
/* One wheel over the ramp rolls the car and barely pitches it (course
 * observation, 30 Oct): the bench also keeps the side height,
 * ROBOT_TRACK_WIDTH_M times sin(roll), + the left wheel up. A lean opens at
 * the module's minimum hump, closes under half of it, and counts only if it
 * lasted BENCH_RAMP_LEAN_M of path: a jolt passes in a few mm. The car's own
 * acceleration (encoder speed, filtered at the module's filter time) comes
 * off the forward axis first, so a speed change does not read as a slope.
 */
#define BENCH_RAMP_LEAN_M (ROBOT_WHEEL_RADIUS_M)
/* START calibration: each IR channel's
 * lowest and highest reading over AUTO-CENTRE's pivots and the first
 * BENCH_CAL_M of line, sent once (CAL) beside the configured white and
 * black. Measured only: the front spans scale darkness, so applied they
 * would change the tuned steering strength.
 */
#define BENCH_CAL_M (4.0 * ROBOT_ROBOT_LENGTH_M)
#define BENCH_RAMP_WINDOW_US (200000U)  /* robot_app's speed window */
#define BENCH_MG_TO_MPS2 (0.00980665)
#define BENCH_COUNT_M (2.0 * CORE_PI * ROBOT_WHEEL_RADIUS_M / \
                       (double)ROBOT_ENCODER_COUNTS_PER_REV)
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

/* Mission command bytes (mqttcmd.h), given bench meanings. */
#define BENCH_CMD_NONE (0x00U)
#define BENCH_CMD_STOP (0xA9U)
#define BENCH_CMD_START (0xAAU)
#define BENCH_CMD_RESET (0xABU)
#define BENCH_CMD_LEFT (0xACU)
#define BENCH_CMD_RIGHT (0xADU)
#define BENCH_CMD_STRAIGHT (0xAEU)
#define BENCH_CMD_UTURN (0xAFU)

/* Duty in percent: 1% steps where a wheel starts, coarse above. */
static const uint8_t g_duties[] =
{
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U, 13U, 14U, 15U,
    16U, 17U, 18U, 19U, 20U, 21U, 22U, 23U, 24U, 25U, 26U, 27U, 28U, 29U,
    30U, 35U, 40U, 50U, 60U
};
#define BENCH_DUTIES ((uint32_t)(sizeof(g_duties) / sizeof(g_duties[0])))
/* Up and down, forward then backward. */
#define BENCH_STEPS (4U * BENCH_DUTIES)

typedef struct
{
    int8_t  duty_pct;     /* signed: + forward */
    bool    b_down;       /* measured on the way down */
    bool    b_errors;     /* the encoder ISR rejected an edge or saturated */
    int16_t rate[BENCH_WHEELS];  /* counts/s, + forward */
} bench_step_t;

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
    uint32_t sum[3];
    uint16_t minimum[3];
    uint16_t maximum[3];
} bench_ir_t;

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
    int32_t from_mm;               /* centre path where it opened */
    int32_t peak_mm;
    int32_t peak_ds;               /* t of the peak, 0.1 s */
} bench_rise_t;

/** MISSION: one lean to a side (bench_ramp_lean). */
typedef struct
{
    double  peak;                  /* side height at the most, + left up */
    double  pitch;                 /* at the most */
    double  length;                /* path while open, m */
    int32_t from_mm;
    int32_t peak_mm;
    int32_t peak_ds;               /* t of the most, 0.1 s */
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

typedef struct
{
    double proportional;
    double integral;
} bench_gains_t;

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
    uint32_t         errors;          /* g_encoder_errors at the start */
    double           done_s;
    double           yaw;             /* latest odometry heading */
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
    BENCH_AVOID_HOMEPIVOT     /* pivot back to the home heading, on course */
} bench_avoid_phase_t;

/** Obstacle map cells (bench_avoid_mark). */
typedef enum
{
    BENCH_MAP_UNKNOWN = 0,    /* no beam reached it */
    BENCH_MAP_FREE,           /* a beam passed through */
    BENCH_MAP_OBSTACLE,       /* a hit within the obstacle's depth */
    BENCH_MAP_OTHER           /* a hit further back: something else */
} bench_map_cell_t;

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
    double   yaw_home;    /* original heading, held straight/parallel (+ left) */
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
    double   x0_mm;       /* odometry pose at the first stop (the map's */
    double   y0_mm;       /* origin, axes along yaw_home), for OBST */
    uint64_t pause_us;    /* when the car stopped in front */
    uint32_t readings;    /* g_front.readings last seen, for new beams */
    uint32_t misses;      /* readings in a row past the edge */
    uint32_t close;       /* MISSION: readings in a row within reach */
    uint32_t backups;     /* reverses made for this obstacle */
    bool     b_hit;       /* the last beam hit the obstacle */
    bool     b_left_open; /* left sweep ended on the obstacle: edge unseen */
    bool     b_right_open; /* and the right */
    bool     b_creeping;  /* a reverse goes back to the creep, not the sweeps */
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
    double   all_dark_m;     /* LINE smooth: travel with all three dark */
    double   turn;           /* LINE smooth: cmd averaged over travel */
    bool     b_both_dark;    /* LINE smooth: both see a line: a crossing */
    uint64_t last_us;        /* the previous sample */
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
    uint16_t unit;   /* decoder units of side sensor path since the start */
    uint16_t centre; /* decoder units of centre path where it began */
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
    uint16_t               pending_centre;
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
    uint16_t edge[BENCH_D0_EDGES];  /* the first is dark, then alternating */
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
    uint16_t         unit;       /* the last reading's unit */
    uint16_t         light_from; /* unit the light now began */
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
    "AVOID an obstacle and return to course, 80 mm/s (floor, no line, until STOP)"
};

/* Code 39: nine elements bar, space, ... bar, first element in bit 8, 1 for
 * wide (Barcode Sample: *A* and *Z*). Indexed as g_code39_chars.
 */
static const char g_code39_chars[BENCH_CODE_CHARS + 1U] =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. *$/+%";
static const uint16_t g_code39_patterns[BENCH_CODE_CHARS] =
{
    0x034U, 0x121U, 0x061U, 0x160U, 0x031U, 0x130U, 0x070U, 0x025U,
    0x124U, 0x064U, 0x109U, 0x049U, 0x148U, 0x019U, 0x118U, 0x058U,
    0x00DU, 0x10CU, 0x04CU, 0x01CU, 0x103U, 0x043U, 0x142U, 0x013U,
    0x112U, 0x052U, 0x007U, 0x106U, 0x046U, 0x016U, 0x181U, 0x0C1U,
    0x1C0U, 0x091U, 0x190U, 0x0D0U, 0x085U, 0x184U, 0x0C4U, 0x094U,
    0x0A8U, 0x0A2U, 0x08AU, 0x02AU
};

/* Why bench_code39_spec refused a frame, by its result. */
static const char * const g_spec_names[BENCH_SPEC_RESULTS] =
{
    "", "character widths", "gap between characters", "quiet zone"
};

/* The robot decoder's DARK/LIGHT fractions (barcode.h); its codebook is
 * empty here and never used.
 */
static const barcode_config_t g_code_limits = { BARCODE_DEFAULT_LIMITS };

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

/* Indexed by line_ctrl_status_t. */
static const char * const g_line_names[BENCH_LINE_STATES] =
{
    "INHIBITED", "TRACKING", "SEARCHING", "JUNCTION", "LOST", "FAULT"
};

/* Indexed by scanner_side_t. */
static const char * const g_side_names[BENCH_SCAN_SIDES] =
{
    "NONE", "LEFT", "RIGHT"
};

/* Indexed by motion_status_t. */
static const char * const g_status_names[BENCH_STATUS_NAMES] =
{
    "OK", "INVALID", "REJECTED", "EXPIRED", "TIMED_OUT", "SENSOR_FAULT",
    "STOPPED"
};

static bench_step_t  g_steps[BENCH_STEPS];
static uint32_t      g_steps_done;
static bench_sweep_t g_sweep;
static bench_ir_t    g_ir;
static bench_accel_t g_accel;
static bench_ramp_t  g_ramp;
static bench_cal_t   g_cal;
static char          g_text[BENCH_TEXT_BYTES];
static uint32_t      g_length;
static char          g_live[BENCH_TEXT_BYTES];  /* MISSION's live line */
static uint32_t      g_live_length;
static uint32_t      g_live_sent;
static uint64_t      g_live_due_us;
static uint16_t      g_live_side_max;
static uint16_t      g_live_front_min;  /* MISSION status F: nearest sonar
                                         * range since the last, 0: none */
static uint32_t      g_live_front_none; /* and x: readings with no range */
static char          g_raw[BENCH_RAW_CHARS];  /* MISSION's cut going out */
static uint32_t      g_raw_length;
static uint32_t      g_raw_sent;
static uint32_t      g_raw_number;   /* its number in the run */
static uint32_t      g_raw_first;    /* its first unit */
static uint32_t      g_raw_white;
static uint32_t      g_raw_sd;
static uint32_t      g_raw_cuts;     /* cuts with a group this run */
static uint32_t      g_raw_done;     /* of them, sent in full */
static bool          gb_raw_head;
static bench_d0_t    g_d0;           /* MISSION: the barcode D0 */
static bool          gb_mqtt;
static bool          gb_watchdog;
static bool          gb_accel;
static uint16_t      g_servo_us;          /* 0 until the first command */
static uint16_t      g_servo_step = BENCH_SERVO_FINE_US;
static int32_t       g_distance[BENCH_WHEELS];
static uint16_t      g_last_total[BENCH_WHEELS];
static uint32_t      g_encoder_errors;
static uint32_t      g_start_ticks;
static uint64_t      g_report_us;
static uint32_t      g_send_tries = BENCH_SEND_TRIES;
static int32_t       g_stall_pct;   /* largest steady duty of the sweep */
static int32_t       g_delta[BENCH_WHEELS];  /* this tick, + forward */
static bench_drive_t g_drive;
static motion_t      g_motion;
static bench_mode_t  g_mode = BENCH_MODE_MISSION;  /* what START runs */
static uint32_t      g_pivot_starts;  /* BENCH_PIVOT_TEST: STARTs so far */
static const uint32_t g_gain = 4U;                 /* P 3, I 15 */
static line_ctrl_runtime_t g_line_runtime;
static line_ctrl_state_t   g_line_state;
static bench_line_t        g_line;
static bench_front_t       g_front;
static core_ultrasonic_config_t g_front_range;
static scanner_state_t     g_scanner;
static bench_scan_t        g_scan;
static bench_avoid_t       g_avoid;
static char g_avoid_lines[BENCH_AVOID_LINES][BENCH_AVOID_LINE_BYTES];
static uint32_t            g_avoid_queued;   /* lines in g_avoid_lines */
static uint32_t            g_avoid_dropped;  /* lines the queue had no room for */
static uint64_t            g_pose_due_us;    /* MISSION: next P line */
static int16_t             g_beam_deg[BENCH_BEAMS];  /* sweep beams: angle */
static uint16_t            g_beam_mm[BENCH_BEAMS];   /* range, 0 no echo */
static int16_t             g_beam_back[BENCH_BEAMS]; /* back_mm then */
static uint32_t            g_beams;          /* beams kept, this obstacle */
static uint32_t            g_beams_sent;     /* of those, sent */
static uint32_t            g_beams_dropped;  /* beams past BENCH_BEAMS */
static bench_centre_t      g_centre;
static bench_code_t        g_code;
static uint16_t            g_code_levels[BENCH_CODE_ADC_LEVELS];
static ID                  g_code_cyc;    /* 1 ms wake-up, once created */
static ID                  g_bench_tid;   /* the task bench_loop runs in */
static bool                gb_code_poll;  /* g_code_cyc started */
static bench_auto_t        g_auto;

static void bench_tick (void);
static uint8_t bench_command (void);
static void bench_track (void);
static void bench_idle (uint8_t command);
static void bench_servo (uint8_t command);
static void bench_sweep_start (void);
static void bench_sweep_tick (uint8_t command);
static void bench_sweep_finish (const char * p_reason);
static void bench_step_place (uint32_t step, int8_t * p_duty_pct,
                              bool * p_down);
static void bench_summary (void);
static void bench_wheel_summary (int32_t direction, uint32_t wheel);
static void bench_drive_start (void);
static void bench_drive_request (uint32_t step);
static void bench_drive_tick (uint8_t command);
static void bench_pid_tick (uint64_t now_us, uint32_t dt_us,
                            const motion_output_t * p_output);
static void bench_pid_sample (uint32_t wheel, double speed, double duty,
                              uint64_t t_us, uint32_t dt_us);
static void bench_move_tick (uint64_t now_us,
                             const motion_output_t * p_output);
static void bench_drive_finish (const char * p_reason);
static void bench_pid_results (const char * p_reason);
static void bench_pid_wheel (const bench_wheel_t * p_wheel, uint32_t wheel);
static void bench_move_results (const char * p_reason);
static void bench_move_line (const char * p_label, const int32_t * p_counts,
                             double yaw);
static bool bench_is_line (bench_mode_t mode);
static double bench_line_speed (bench_mode_t mode);
static bool bench_line_start (void);
static void bench_line_step (uint64_t now_us, motion_request_t * p_request);
static bool bench_line_cross (const line_ctrl_output_t * p_output,
                              int32_t centre_mm, motion_request_t * p_request);
static void bench_line_moving (void);
static void bench_line_end (uint64_t now_us, const char * p_reason);
static void bench_line_trace (void);
static void bench_line_print (const char * p_label, uint32_t t_ms,
                              const bench_line_row_t * p_row);
static bool bench_is_centre (bench_mode_t mode);
static const bench_smooth_t * bench_smooth_set (bench_mode_t mode);
static double bench_smooth_track (const bench_smooth_t * p_set, double error,
                                  double travel);
static double bench_smooth_gate (uint32_t channel, double darkness,
                                 double travel);
static bool bench_line_black (const line_ctrl_output_t * p_output);
static double bench_centre_steer (const line_ctrl_output_t * p_output,
                                  int32_t centre, uint64_t now_us,
                                  bool * p_lost);
static void bench_centre_results (void);
static bool bench_has_auto (bench_mode_t mode);
static void bench_auto_start (void);
static void bench_auto_step (uint64_t now_us,
                             const line_ctrl_output_t * p_output,
                             motion_request_t * p_request);
static void bench_auto_seek (uint64_t now_us,
                             const line_ctrl_output_t * p_output);
static void bench_auto_found (uint64_t now_us);
static void bench_auto_middle (uint64_t now_us, motion_request_t * p_request);
static void bench_auto_settle (uint64_t now_us);
static void bench_auto_phase (uint64_t now_us, bench_auto_phase_t phase);
static void bench_auto_fail (uint64_t now_us, const char * p_reason);
static void bench_auto_results (void);
static void bench_deg (double radians);
static void bench_front_start (void);
static bool bench_front_step (uint64_t now_us);
static void bench_front_results (void);
static void bench_avoid_drive (uint64_t now_us, bool b_range, int32_t centre,
                               motion_request_t * p_request);
static void bench_avoid_begin (uint64_t now_us, int32_t centre);
static void bench_avoid_rejoin (uint64_t now_us, int32_t centre);
static double bench_avoid_wrap (double angle);
static bool bench_avoid_pivot (double target, motion_request_t * p_request);
static bool bench_avoid_edge (double outward);
static void bench_avoid_creep (uint64_t now_us, double turned,
                               motion_request_t * p_request);
static void bench_avoid_creep_found (uint64_t now_us,
                                     motion_request_t * p_request);
static void bench_avoid_keep_beam (double angle, uint16_t range_mm);
static bool bench_avoid_beam (double car_angle, double beam_angle,
                              uint16_t range_mm);
static void bench_avoid_mark (double x_mm, double y_mm, uint8_t code);
static bool bench_avoid_side_clear (int32_t side, double out_mm);
static void bench_avoid_plan (uint64_t now_us, motion_request_t * p_request);
static void bench_avoid_map_print (void);
static bool bench_avoid_map_row (uint32_t row, int32_t car_row,
                                 int32_t car_col, char * p_line);
static void bench_avoid_obst_text (void);
static void bench_avoid_say (void);
static bool bench_avoid_text (void);
static void bench_avoid_live (uint64_t now_us);
static bool bench_beams_text (void);
static bool bench_pose_text (uint64_t now_us);
static void bench_scan_start (void);
static void bench_scan_tick (uint8_t command);
static bool bench_scan_servo (int32_t angle_deg);
static void bench_scan_finish (const char * p_reason);
static void bench_scan_verdict (void);
static void bench_scan_beams (void);
static void bench_scan_mm (double metres);
static bool bench_code_start (void);
static void bench_code_cyc (void * p_exinf);
static bool bench_code_polling (void);
static void bench_code_wait (void);
static void bench_code_poll (void);
static void bench_code_sample (int32_t left, int32_t right, uint16_t raw);
static void bench_code_store (void);
static void bench_code_stream (void);
static void bench_code_keep (uint32_t gap);
static void bench_code_found_add (uint32_t runs);
static void bench_code_found_window (bench_code_found_t * p_found,
                                     uint32_t last);
static void bench_code_found_rest (bench_code_found_t * p_found,
                                   uint32_t last);
static void bench_code_found_chars (bench_code_found_t * p_found,
                                    uint32_t last);
static bool bench_code_found_text (void);
static bool bench_code_hold (void);
static bool bench_hold_lets_go (void);
static void bench_d0_sample (int32_t unit, bool b_level);
static void bench_d0_close (void);
static bool bench_d0_decode (const bench_d0_group_t * p_group, bool b_reverse,
                             char * p_text);
static bool bench_d0_text (void);
static void bench_live_step (uint64_t now_us, uint32_t status,
                             int32_t centre_mm, uint16_t side);
static void bench_live_take (void);
static void bench_live_push (void);
static void bench_live_finish (void);
static void bench_raw_keep (void);
static bool bench_raw_text (void);
static void bench_ramp_start (void);
static void bench_ramp_axes (const double * p_mg, double * p_mps2);
static void bench_ramp_step (uint64_t now_us,
                             const motion_output_t * p_output);
static void bench_ramp_offset (double ds);
static void bench_ramp_lean (uint64_t now_us, double ds, int32_t centre_mm);
static void bench_ramp_lean_line (const char * p_head,
                                  const bench_lean_t * p_lean);
static void bench_cal_sample (const uint16_t * p_raw, int32_t centre_mm);
static bool bench_cal_text (void);
static void bench_ramp_rise (uint64_t now_us, int32_t path,
                             int32_t centre_mm);
static bool bench_ramp_text (void);
static void bench_ramp_status (void);
static void bench_ramp_deg (double rad);
static void bench_ramp_side (double roll);
static void bench_ramp_results (void);
static void bench_code_mission_results (void);
static void bench_code_results (void);
static void bench_code_card (void);
static void bench_code_card_group (uint32_t group, uint32_t runs,
                                   bool * p_decoded);
static void bench_code_card_lines (void);
static bool bench_code_in_line (uint32_t index);
static void bench_code_card_extrema (uint16_t swing);
static double bench_code_edge (uint32_t peak, uint32_t valley, double level,
                               uint32_t * p_index);
static uint32_t bench_code_pass (const line_ctrl_hysteresis_t * p_hysteresis,
                                 bool * p_decoded);
static void bench_code_runs (const line_ctrl_hysteresis_t * p_hysteresis);
static bool bench_code_group (uint32_t first, uint32_t last);
static void bench_code_widths (uint32_t first, uint32_t last);
static void bench_code_dark (bool b_quiet, uint32_t half_sds, double gap_mm);
static void bench_code_dark_tries (void);
static void bench_code_tries_begin (uint32_t gap_units);
static void bench_code_try (uint32_t index);
static bool bench_code_try_agree (uint32_t index);
static void bench_code_tries_step (void);
static void bench_code_dark_end (uint32_t group, uint32_t runs,
                                 bool * p_decoded, bool b_quiet);
static uint32_t bench_code_white (uint32_t * p_sd);
static uint32_t bench_code_darkness (uint32_t before, uint32_t after);
static uint32_t bench_code_whiteness (uint32_t from, uint32_t to,
                                      uint32_t floor_adc);
static void bench_code_dark_row (uint32_t first, uint32_t last);
static uint32_t bench_code39_dark (uint32_t first, uint32_t last,
                                   uint32_t base, bool b_reverse);
static uint32_t bench_code39_spec (uint32_t first, uint32_t length,
                                   uint32_t last);
static bool bench_code39 (uint32_t first, uint32_t last);
static bool bench_code39_text (uint32_t first, uint32_t last, bool b_reverse,
                               char * p_text);
static uint32_t bench_code_units (uint32_t first, uint32_t last,
                                  uint32_t index, bool b_reverse);
static void bench_code_trace (void);
static void bench_code_unit_mm (uint32_t units);
static void bench_gains_text (void);
static void bench_sample (void);
static void bench_report (void);
static int32_t bench_signed_delta (uint16_t newer, uint16_t older,
                                   uint32_t wheel);
static int32_t bench_mm (int32_t counts);
static int32_t bench_round (double value);
static void bench_text (const char * p_text);
static void bench_dec (int32_t value);
static void bench_signed (int32_t value);
static void bench_fixed (int32_t value, uint32_t places);
static void bench_flush (void);
/* TRIAL (barcode_lab): the model-fit decoder, at the end of the file. */
static void bench_fit_reset (void);
static void bench_fit_begin (void);
static void bench_fit_run (void);
static bool bench_fit_complete (bool b_bin);
static bool bench_fit_text (void);
static void bench_fit_totals (void);

void bench_loop (void)
{
    uint16_t samples[3] = {0U, 0U, 0U};
    bool     b_board = false;
    uint8_t  command = BENCH_CMD_NONE;

    (void)tm_printf((const UB *)"\nBENCH IMAGE: not the robot firmware;"
                    " wheels must be off the ground.\n");
    b_board = board_init();
    /* An accelerometer failure still leaves the other measurements. */
    if ((!b_board) && (!board_read_ir(samples)))
    {
        (void)tm_printf((const UB *)"board_init refused: %s\n",
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
    board_encoders_discard();
    if (0 != BENCH_MOTION)
    {
        bench_text("BENCH MOTION ready: START runs MISSION (line-follow +"
                   " barcode + obstacle go-around), STOP stops. Accel ");
    }
    else
    {
        bench_text("BENCH ready: START = duty sweep, STOP = abort. Accel ");
    }
    bench_text(gb_accel ? "ok" : "FAILED");
    bench_flush();
    if (0 != BENCH_MOTION)
    {
        /* The sonar faces ahead from boot, for the obstacle tests. */
        (void)bench_scan_servo(0);
    }
    g_report_us = board_time_us();
    for (;;)
    {
        bench_tick();
        bench_track();
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

/** @brief One 5 ms period, read by the side sensor every 1 ms while a
 * BARCODE run follows the line. The watchdog runs from the first sweep on;
 * feeding does nothing before it starts.
 */
static void bench_tick (void)
{
    if (bench_code_polling())
    {
        bench_code_wait();
    }
    else
    {
        if (gb_code_poll)
        {
            (void)tk_stp_cyc(g_code_cyc);
            gb_code_poll = false;
        }
        (void)tk_dly_tsk((RELTIM)BENCH_TICK_MS);
    }
    board_watchdog_feed();
}

/** @brief One command per tick: STOP first, then MQTT, then START. */
static uint8_t bench_command (void)
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

/** @brief Wheel distance from the 16-bit totals; consume the ISR deltas so
 * their error bits mean rejected edges or saturation in this period only.
 */
static void bench_track (void)
{
    uint16_t total[BENCH_WHEELS] = {0U, 0U};
    int32_t  left = 0;
    int32_t  right = 0;
    uint32_t errors = 0U;
    uint32_t wheel;

    board_read_encoders(&left, &right, &errors);
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
static void bench_idle (uint8_t command)
{
    uint64_t now_us = 0U;

    switch (command)
    {
        case BENCH_CMD_START:
            if (0 != BENCH_MOTION)
            {
                if (0 != BENCH_PIVOT_TEST)
                {
                    g_mode = (0U == ((g_pivot_starts / 4U) % 2U)) ?
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
            board_motor_brake();
        break;
        case BENCH_CMD_RESET:
            g_distance[BENCH_LEFT] = 0;
            g_distance[BENCH_RIGHT] = 0;
            g_encoder_errors = 0U;
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

/** @brief Step the scan servo within the bench limits (robot_config.h of
 * the bench copy): back off as soon as it buzzes at a mechanical stop.
 */
static void bench_servo (uint8_t command)
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

/** @brief Begin the sweep from rest; refuses while STOP is held. */
static void bench_sweep_start (void)
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
static void bench_sweep_tick (uint8_t command)
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
static void bench_sweep_finish (const char * p_reason)
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
static void bench_step_place (uint32_t step, int8_t * p_duty_pct,
                              bool * p_down)
{
    uint32_t within = step % (2U * BENCH_DUTIES);
    uint32_t index = (within < BENCH_DUTIES) ? within :
                     ((2U * BENCH_DUTIES) - 1U - within);
    int32_t  pct = (int32_t)g_duties[index];

    *p_down = within >= BENCH_DUTIES;
    *p_duty_pct = (int8_t)((step < (2U * BENCH_DUTIES)) ? pct : -pct);
}

/** @brief Per wheel and direction, then the stall-duty lower bound. */
static void bench_summary (void)
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

/** @brief First duty that moves the wheel at all, the duty from which every
 * higher step turns it (steady: the wheels stick at low duty and can twitch
 * below it), keep-turning duty (lowest step down that still turns), top
 * speed and the sign at the top duty. ROBOT_STALL_DUTY must not be below
 * the largest steady duty, or a slow start latches a stall fault. The sign
 * verdict trusts the encoder signs: check those first by pushing the car
 * forward on the floor (both ENC distances must rise).
 */
static void bench_wheel_summary (int32_t direction, uint32_t wheel)
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
static void bench_drive_start (void)
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
    app_cfg_motion(&config);
    config.wheel_pid.proportional_gain = g_gains[g_gain].proportional;
    config.wheel_pid.integral_gain = g_gains[g_gain].integral;
    /* The bench copy raises ROBOT_MAX_DUTY for the sweep: use the robot's. */
    config.wheel_pid.output_min = -BENCH_ROBOT_MAX_DUTY;
    config.wheel_pid.output_max = BENCH_ROBOT_MAX_DUTY;
    if (NULL != bench_smooth_set(g_mode))
    {
        config.max_wheel_mps = BENCH_SMOOTH_WHEEL_MPS;
    }
    status = motion_init(&g_motion, &config);
    if (MOTION_OK != status)
    {
        bench_text("MOTION refused: motion_init ");
        bench_text(g_status_names[(uint32_t)status % BENCH_STATUS_NAMES]);
        bench_flush();
        return;
    }
    if ((!gb_watchdog) && board_watchdog_start(BENCH_WATCHDOG_MS))
    {
        gb_watchdog = true;
    }
    (void)memset(&g_drive, 0, sizeof(g_drive));
    g_drive.mode = g_mode;
    g_drive.errors = g_encoder_errors;
    for (wheel = 0U; wheel < BENCH_WHEELS; ++wheel)
    {
        g_drive.start[wheel] = g_distance[wheel];
    }
    if (BENCH_MODE_MISSION == g_mode)
    {
        bench_ramp_start();
    }
    bench_text((BENCH_MODE_PID == g_mode) ?
               "PID start: about 15 s, wheels must be off the ground;"
               " STOP aborts" :
               "DRIVE start: car on the floor, path clear; STOP aborts");
    bench_flush();
    if (bench_has_auto(g_mode))
    {
        bench_text("AUTO-CENTRE first: pivots on the spot, up to 70 deg"
                   " each way, until both front sensors find the line");
        bench_flush();
    }
    /* The text above can outlast line_ctrl's 50 ms sample gap. */
    line_ctrl_init(&g_line_state);
    bench_drive_request(0U);
    g_drive.b_running = true;
    g_drive.step_us = board_time_us();
    g_drive.last_us = g_drive.step_us;
}

/** @brief The request of one step, with a fresh command ID (a new ID resets
 * the controller, as for every new primitive on the robot).
 */
static void bench_drive_request (uint32_t step)
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
static void bench_drive_tick (uint8_t command)
{
    motion_output_t  output;
    motion_request_t halt;
    motion_request_t line;
    motion_status_t  status = MOTION_OK;
    uint64_t         now_us = board_time_us();
    uint32_t         dt_us = (uint32_t)(now_us - g_drive.last_us);
    uint32_t         now_ms = (uint32_t)(now_us / 1000U);

    g_drive.last_us = now_us;
    if (dt_us > g_drive.dt_max_us)
    {
        g_drive.dt_max_us = dt_us;
    }
    if (BENCH_CMD_STOP == command)
    {
        motion_stop(&g_motion, MOTION_STOPPED);
        bench_drive_finish("aborted");
        return;
    }
    if (bench_is_line(g_drive.mode))
    {
        bench_line_step(now_us, &line);
        status = motion_submit(&g_motion, &line, now_ms);
    }
    else if (g_drive.b_rest)
    {
        (void)memset(&halt, 0, sizeof(halt));
        halt.kind = MOTION_HALT;
        status = motion_submit(&g_motion, &halt, now_ms);
    }
    else if (!g_drive.b_done)
    {
        status = motion_submit(&g_motion, &g_drive.request, now_ms);
    }
    else
    {
        /* Complete: the controller holds zero duty, as on the robot. */
    }
    if (MOTION_OK != status)
    {
        bench_drive_finish(g_status_names[(uint32_t)status %
                                          BENCH_STATUS_NAMES]);
        return;
    }
    (void)motion_step(&g_motion, now_ms, (double)dt_us / 1000000.0,
                      g_delta[BENCH_LEFT], g_delta[BENCH_RIGHT], &output);
    if (g_drive.b_rest)
    {
        board_motor_brake();
    }
    else
    {
        board_motor_duty(output.left_duty, output.right_duty);
    }
    g_drive.yaw = output.pose.yaw_rad;
    g_drive.x_m = output.pose.x_m;
    g_drive.y_m = output.pose.y_m;
    if (BENCH_MODE_MISSION == g_drive.mode)
    {
        bench_ramp_step(now_us, &output);
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
static void bench_pid_tick (uint64_t now_us, uint32_t dt_us,
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
static void bench_pid_sample (uint32_t wheel, double speed, double duty,
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
static void bench_move_tick (uint64_t now_us,
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
static void bench_drive_finish (const char * p_reason)
{
    board_motor_brake();
    board_encoders_discard();
    g_drive.b_running = false;
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
static void bench_pid_results (const char * p_reason)
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
static void bench_pid_wheel (const bench_wheel_t * p_wheel, uint32_t wheel)
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
static void bench_move_results (const char * p_reason)
{
    int32_t  now[BENCH_WHEELS];
    uint32_t wheel;

    for (wheel = 0U; wheel < BENCH_WHEELS; ++wheel)
    {
        now[wheel] = g_distance[wheel] - g_drive.start[wheel];
    }
    bench_text((BENCH_MODE_MISSION == g_drive.mode) ? "MISSION" :
                                                      g_mode_names[g_drive.mode]);
    bench_text(" ");
    bench_text(p_reason);
    if (g_drive.b_done)
    {
        bench_text((bench_is_line(g_drive.mode)) ? " after " :
                                                       ", complete after ");
        bench_fixed(bench_round(g_drive.done_s * 10.0), 1U);
        bench_text(" s");
    }
    bench_text(": ");
    if (BENCH_MODE_MISSION != g_drive.mode)
    {
        bench_gains_text();
        bench_text(", ");
    }
    bench_text("dt max ");
    bench_fixed(bench_round((double)g_drive.dt_max_us / 100.0), 1U);
    bench_text(" ms");
    if (g_encoder_errors != g_drive.errors)
    {
        bench_text(", ENCODER ERRORS");
    }
    bench_flush();
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
static void bench_move_line (const char * p_label, const int32_t * p_counts,
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

/** @brief True for the line tests. */
static bool bench_is_line (bench_mode_t mode)
{
    return (BENCH_MODE_LINE == mode) || (BENCH_MODE_LINE_FAST == mode) ||
           (BENCH_MODE_LINE_MAX == mode) || (BENCH_MODE_LINE_STOP == mode) ||
           (BENCH_MODE_AVOID == mode) ||
           (BENCH_MODE_AUTO == mode) || bench_is_centre(mode);
}

/** @brief True for the LINE centre tests. */
static bool bench_is_centre (bench_mode_t mode)
{
    return (BENCH_MODE_CENTRE == mode) || (BENCH_MODE_CENTRE_MAX == mode) ||
           (NULL != bench_smooth_set(mode));
}

/** @brief The LINE smooth setting of a mode (BARCODE and MISSION steer as
 * LINE smooth A, BARCODE slower), or NULL for other tests.
 */
static const bench_smooth_t * bench_smooth_set (bench_mode_t mode)
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
static double bench_line_speed (bench_mode_t mode)
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
static bool bench_line_start (void)
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
    if (b_valid && ((LINE_CTRL_TRACKING == output.status) ||
                    (b_centre && (LINE_CTRL_SEARCHING == output.status))))
    {
        if ((BENCH_MODE_LINE_STOP == g_mode) || (BENCH_MODE_MISSION == g_mode))
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
static void bench_line_step (uint64_t now_us, motion_request_t * p_request)
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
    if ((BENCH_MODE_LINE_STOP == g_drive.mode) ||
        (BENCH_MODE_AVOID == g_drive.mode) ||
        (BENCH_MODE_MISSION == g_drive.mode))
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
        bench_code_tries_step();
        bench_live_step(now_us, (uint32_t)output.status, bench_mm(centre),
                        raw[LINE_CTRL_BARCODE_CHANNEL]);
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
        g_avoid.yaw_home = g_drive.yaw; /* the line heading at detection */
        bench_avoid_begin(now_us, centre);
        bench_text("MISSION: obstacle at ");
        bench_dec((int32_t)g_front.last_mm);
        bench_text(" mm, stopped to measure it");
        bench_avoid_say();
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
                       "stopped: all three sensors dark (lifted, or off the mat)");
        return;
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
    /* LINE smooth: line_ctrl's JUNCTION comes before both sensors have seen
     * BENCH_SMOOTH_CROSS_M of it; bench_centre_steer goes on until then.
     */
    if ((LINE_CTRL_SEARCHING != output.status) &&
        (LINE_CTRL_TRACKING != output.status) &&
        (!(b_centre && (LINE_CTRL_JUNCTION == output.status))))
    {
        bench_line_end(now_us, "stopped: line sensor fault");
        return;
    }
    if (fabs((double)sample.centre_mm) >=
        (((BENCH_MODE_BARCODE == g_drive.mode) ? BENCH_CODE_MAX_M :
          b_centre ? BENCH_CENTRE_MAX_M : BENCH_LINE_MAX_M) * 1000.0))
    {
        bench_line_end(now_us, "stopped at the distance limit");
        return;
    }
    if (t_us >= (b_centre ? BENCH_CENTRE_MAX_US : BENCH_LINE_MAX_US))
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
        /* Straight over a card (bench_code_hold) while the line stays
         * centred on a straight.
         */
        steering = 0.0;
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
static bool bench_line_cross (const line_ctrl_output_t * p_output,
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
static void bench_line_moving (void)
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
static void bench_line_end (uint64_t now_us, const char * p_reason)
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
static void bench_line_trace (void)
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
static void bench_line_print (const char * p_label, uint32_t t_ms,
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
static double bench_centre_steer (const line_ctrl_output_t * p_output,
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
    const bench_smooth_t * p_smooth = bench_smooth_set(g_drive.mode);

    *p_lost = false;
    g_centre.last_counts = centre;
    if (NULL != p_smooth)
    {
        /* Barcode bars are too narrow to count (bench_smooth_gate). */
        left = bench_smooth_gate(0U, left, travel);
        right = bench_smooth_gate(1U, right, travel);
        error = right - left;
        lighter = (left < right) ? left : right;
        darker = (left < right) ? right : left;
        g_centre.turn += (g_centre.cmd - g_centre.turn) *
                         fmin(1.0, travel / BENCH_SMOOTH_TURN_M);
        g_centre.b_both_dark =
            (g_centre.dark_m[0] >= BENCH_SMOOTH_CROSS_M) &&
            (g_centre.dark_m[1] >= BENCH_SMOOTH_CROSS_M) &&
            (fabs(g_centre.turn) < BENCH_SMOOTH_BEND);
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
        steering = (NULL != p_smooth) ?
                   bench_smooth_track(p_smooth, error, travel) :
                   (BENCH_CENTRE_KP * error);
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
    return steering;
}

/** @brief LINE smooth: a sensor's darkness, or 0 until it has been at least
 * BENCH_SMOOTH_DARK for BENCH_SMOOTH_DARK_M of travel; dark_m also decides
 * crossings (BENCH_SMOOTH_CROSS_M).
 */
static double bench_smooth_gate (uint32_t channel, double darkness,
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

/** @brief Line test: both front sensors see a line (a junction). LINE smooth
 * takes its bar-proof view (bench_smooth_gate), the others line_ctrl's.
 */
static bool bench_line_black (const line_ctrl_output_t * p_output)
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
static double bench_smooth_track (const bench_smooth_t * p_set, double error,
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
static void bench_centre_results (void)
{
    double   seconds = g_drive.b_done ? g_drive.done_s :
        ((double)(board_time_us() - g_drive.step_us) / 1000000.0);
    int32_t  centre = g_drive.b_done ?
        ((g_drive.stop[BENCH_LEFT] + g_drive.stop[BENCH_RIGHT]) / 2) :
        (((g_distance[BENCH_LEFT] - g_drive.start[BENCH_LEFT]) +
          (g_distance[BENCH_RIGHT] - g_drive.start[BENCH_RIGHT])) / 2);
    uint32_t samples = (0U == g_centre.samples) ? 1U : g_centre.samples;
    uint32_t phase;

    bench_text(" centre: ");
    bench_dec(bench_mm(centre));
    bench_text(" mm in ");
    bench_fixed(bench_round(seconds * 10.0), 1U);
    bench_text(" s, mean ");
    bench_dec((seconds > 0.0) ?
              bench_round((double)bench_mm(centre) / seconds) : 0);
    bench_text(" mm/s; time % per phase");
    for (phase = 0U; phase < BENCH_CENTRE_PHASES; ++phase)
    {
        bench_text(" ");
        bench_text(g_centre_names[phase]);
        bench_text(" ");
        bench_dec(bench_round((100.0 * (double)g_centre.in_phase[phase]) /
                              (double)samples));
    }
    bench_text(", full steer ");
    bench_dec(bench_round((100.0 * (double)g_centre.full_steer) /
                          (double)samples));
    bench_flush();
    bench_text(" gaps ");
    bench_dec((int32_t)g_centre.gaps);
    bench_text(", crossings ");
    bench_dec((int32_t)g_centre.crossings);
    bench_text(", searches ");
    bench_dec((int32_t)g_centre.searches);
    bench_text(" (longest ");
    bench_dec((int32_t)g_centre.search_max_ms);
    bench_text(" ms), darkest raw L ");
    bench_dec((int32_t)g_centre.raw_max[0]);
    bench_text(" R ");
    bench_dec((int32_t)g_centre.raw_max[1]);
    bench_flush();
}

/** @brief True for the tests that AUTO-CENTRE first: LINE smooth A-C,
 * BARCODE, and AUTO-CENTRE alone.
 */
static bool bench_has_auto (bench_mode_t mode)
{
    return (BENCH_MODE_AUTO == mode) || (NULL != bench_smooth_set(mode));
}

/** @brief AUTO-CENTRE: armed for the selected test (bench_line_start). */
static void bench_auto_start (void)
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
static void bench_auto_step (uint64_t now_us,
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
static void bench_auto_seek (uint64_t now_us,
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
static void bench_auto_found (uint64_t now_us)
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
static void bench_auto_middle (uint64_t now_us, motion_request_t * p_request)
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
static void bench_auto_settle (uint64_t now_us)
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
static void bench_auto_phase (uint64_t now_us, bench_auto_phase_t phase)
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
static void bench_auto_fail (uint64_t now_us, const char * p_reason)
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
static void bench_auto_results (void)
{
    double end_rad = g_auto.end_rad;

    if (g_auto.b_active)
    {
        /* STOP during the centring. */
        end_rad = g_drive.yaw;
        g_auto.done_ms = (0U == g_auto.start_us) ? 0U :
            (uint32_t)((board_time_us() - g_auto.start_us) / 1000U);
    }
    bench_text(g_auto.b_failed ? " AUTO-CENTRE FAILED after " :
               g_auto.b_active ? " AUTO-CENTRE stopped after " :
                                 " AUTO-CENTRE done in ");
    bench_fixed(bench_round((double)g_auto.done_ms / 100.0), 1U);
    bench_text(" s, sweeps ");
    bench_dec((int32_t)g_auto.sweeps);
    bench_text(", corrections ");
    bench_dec((int32_t)g_auto.corrections);
    bench_text(", run-on after a halt ");
    bench_deg(g_auto.coast_rad);
    bench_text(" deg");
    bench_flush();
    if (g_auto.b_failed)
    {
        bench_text("  why: ");
        bench_text(g_auto.p_fail);
        bench_flush();
    }
    bench_text("  right sensor found the line turning left at ");
    if (g_auto.b_found[1])
    {
        bench_deg(g_auto.found_rad[1] - g_auto.start_rad);
        bench_text(" deg");
    }
    else
    {
        bench_text("none");
    }
    bench_text(", left turning right at ");
    if (g_auto.b_found[0])
    {
        bench_deg(g_auto.found_rad[0] - g_auto.start_rad);
        bench_text(" deg");
    }
    else
    {
        bench_text("none");
    }
    bench_flush();
    if (g_auto.b_found[0] && g_auto.b_found[1])
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
    bench_dec((int32_t)g_auto.raw[0]);
    bench_text(" R ");
    bench_dec((int32_t)g_auto.raw[1]);
    bench_flush();
}

/** @brief Radians as signed degrees to 0.1, without a unit. */
static void bench_deg (double radians)
{
    if (radians >= 0.0)
    {
        bench_text("+");
    }
    bench_fixed(bench_round(radians * 1800.0 / CORE_PI), 1U);
}

/** @brief LINE + stop: the robot's sonar settings (app_cfg_scanner), servo
 * centred, a stale echo consumed; the first trigger waits for the servo.
 */
static void bench_front_start (void)
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
static bool bench_front_step (uint64_t now_us)
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
static void bench_front_results (void)
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
        bench_dec((int32_t)g_front.last_mm);
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
static void bench_avoid_hold (double target, motion_request_t * p_request)
{
    double yaw = BENCH_AVOID_HOLD_KP *
                 bench_avoid_wrap(target - g_drive.yaw); /* + left */

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
static void bench_avoid_drive (uint64_t now_us, bool b_range, int32_t centre,
                               motion_request_t * p_request)
{
    double  angle_rad = (double)BENCH_AVOID_ANGLE_DEG * CORE_PI / 180.0;
    double  creep_rad = (double)ROBOT_SCAN_HALF_ANGLE_DEG * CORE_PI / 180.0;
    /* + left, right -ve, the short way round (bench_avoid_wrap) */
    double  turned = bench_avoid_wrap(g_drive.yaw - g_avoid.yaw_home);
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
    /* Track the object with the servo while it is still ahead (CREEPTURN to
     * STEPOUT): square to the face at the creep's angle.
     */
    if ((BENCH_AVOID_CREEPTURN == g_avoid.phase) ||
        (BENCH_AVOID_CREEP == g_avoid.phase) ||
        (BENCH_AVOID_OUT == g_avoid.phase) ||
        (BENCH_AVOID_STEPOUT == g_avoid.phase))
    {
        servo_deg = bench_round(-turned * 180.0 / CORE_PI); /* opposite the turn */
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
            bench_text("AVOID: obstacle at ");
            bench_dec((int32_t)g_front.last_mm);
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
            g_avoid.b_hit = false;
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
            g_avoid.misses = 0U;
            g_avoid.b_hit = false;
            p_request->yaw_rps = 0.0;
            bench_text("AVOID: left edge ");
            bench_dec(bench_round(g_avoid.left_mm));
            bench_text(" mm at ");
            bench_dec(bench_round(turned * 180.0 / CORE_PI));
            bench_text(g_avoid.b_left_open ?
                       " deg (still on it at the turn limit: unseen),"
                       " turning right" :
                       " deg, turning right to the other edge");
            bench_avoid_say();
        }
    }
    else if (BENCH_AVOID_MEASURE_R == g_avoid.phase)
    {
        p_request->yaw_rps = -ROBOT_TURN_RATE_RPS; /* pivot right in place */
        if (turned > 0.0)
        {
            /* Back across the left half: those beams are already mapped. */
            g_avoid.readings = g_front.readings;
            g_avoid.misses = 0U;
        }
        else if (bench_avoid_edge(-turned))
        {
            g_avoid.b_right_open = g_avoid.b_hit;
            p_request->yaw_rps = 0.0;
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
            bench_text("AVOID: backed up to ");
            bench_dec(bench_round(g_avoid.back_mm));
            if (g_avoid.b_creeping)
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
                              ((double)g_avoid.side * creep_rad), p_request))
        {
            g_avoid.phase = BENCH_AVOID_CREEP;
            g_avoid.b_creeping = true;
            g_avoid.readings = g_front.readings;
            g_avoid.misses = 0U;
            bench_text("AVOID: creeping out to find its edge, the sonar"
                       " square to its face");
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
            bench_text((g_avoid.side > 0) ? "AVOID: stepping out to the right" :
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
            bench_text("AVOID: alongside the object for ");
            bench_dec(bench_round(g_avoid.face_mm + BENCH_AVOID_DEPTH_MM +
                                  half + BENCH_AVOID_CLEAR_MM +
                                  g_avoid.back_mm));
            bench_text(" mm");
            bench_avoid_say();
        }
    }
    else if (BENCH_AVOID_ALONG == g_avoid.phase)
    {
        bench_avoid_hold(g_avoid.yaw_home, p_request); /* parallel to the line */
        /* By position, not distance from here: the pivot ends half the car +
         * the clearance past the obstacle's back, wherever the creep left it.
         */
        if ((-g_avoid.back_mm) >= (g_avoid.face_mm + BENCH_AVOID_DEPTH_MM +
                                   half + BENCH_AVOID_CLEAR_MM))
        {
            g_avoid.phase = BENCH_AVOID_BACKPIVOT;
            p_request->forward_mps = 0.0;
            p_request->yaw_rps = 0.0;
            bench_text("AVOID: past the object, stepping back to the line");
            bench_avoid_say();
        }
    }
    else if (BENCH_AVOID_BACKPIVOT == g_avoid.phase)
    {
        if (bench_avoid_pivot(g_avoid.yaw_home + side_rad, p_request))
        {
            g_avoid.phase = BENCH_AVOID_STEPIN;
            bench_text("AVOID: returning to the line");
            bench_avoid_say();
        }
    }
    else if (BENCH_AVOID_STEPIN == g_avoid.phase)
    {
        bench_avoid_hold(g_avoid.yaw_home + side_rad, p_request);
        if (((double)g_avoid.side * g_avoid.right_mm) <= 0.0)
        {
            g_avoid.phase = BENCH_AVOID_HOMEPIVOT;
            p_request->forward_mps = 0.0;
            p_request->yaw_rps = 0.0;
            bench_text("AVOID: on the line, straightening");
            bench_avoid_say();
        }
    }
    else /* BENCH_AVOID_HOMEPIVOT */
    {
        if (bench_avoid_pivot(g_avoid.yaw_home, p_request))
        {
            g_avoid.phase = BENCH_AVOID_STRAIGHT;
            (void)bench_scan_servo(0);
            bench_avoid_hold(g_avoid.yaw_home, p_request);
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
static void bench_avoid_begin (uint64_t now_us, int32_t centre)
{
    double angle = bench_avoid_wrap(g_drive.yaw - g_avoid.yaw_home);

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
    (void)memset(g_avoid.map, 0, sizeof(g_avoid.map));
    g_beams = 0U;
    g_beams_sent = 0U;
    g_beams_dropped = 0U;
    g_avoid.face_mm = (BENCH_AVOID_SONAR_MM + (double)g_front.last_mm) *
                      cos(angle);
    bench_avoid_keep_beam(angle, g_front.last_mm);
    (void)bench_avoid_beam(angle, angle, g_front.last_mm);
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
static void bench_avoid_rejoin (uint64_t now_us, int32_t centre)
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
static double bench_avoid_wrap (double angle)
{
    return remainder(angle, 2.0 * CORE_PI);
}

/** @brief AVOID: pivot in place toward target (rad, + left) at the turn
 * rate. True, with no turn requested, once within BENCH_AVOID_PIVOT_TOL_DEG;
 * one tick's turn is under twice that, so it settles without hunting.
 */
static bool bench_avoid_pivot (double target, motion_request_t * p_request)
{
    double error = bench_avoid_wrap(target - g_drive.yaw);
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

/** @brief AVOID measuring, each tick: a new front reading is mapped
 * (bench_avoid_beam) and either hits the obstacle or has slipped past its
 * edge. outward is the turn toward the side being measured (rad). True once
 * BENCH_AVOID_MISSES readings in a row missed, or at the turn limit; b_hit
 * then says whether the sweep ended still on the obstacle (edge unseen).
 */
static bool bench_avoid_edge (double outward)
{
    double limit_rad = (double)BENCH_AVOID_MEASURE_DEG * CORE_PI / 180.0;
    double angle = bench_avoid_wrap(g_drive.yaw - g_avoid.yaw_home);

    if (g_front.readings != g_avoid.readings)
    {
        g_avoid.readings = g_front.readings;
        bench_avoid_keep_beam(angle, g_front.last_mm);
        if (bench_avoid_beam(angle, angle, g_front.last_mm))
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

/** @brief AVOID CREEP, each tick: hold the creep's diagonal
 * (ROBOT_SCAN_HALF_ANGLE_DEG off the line heading toward the chosen side) at
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
static void bench_avoid_creep (uint64_t now_us, double turned,
                               motion_request_t * p_request)
{
    double half = BENCH_AVOID_CAR_WIDTH_MM / 2.0;
    double creep_rad = (double)ROBOT_SCAN_HALF_ANGLE_DEG * CORE_PI / 180.0;
    double side = (double)g_avoid.side;
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
    if (g_avoid.misses >= BENCH_AVOID_MISSES)
    {
        bench_avoid_creep_found(now_us, p_request);
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
        bench_text("AVOID: still on its face ");
        bench_dec(bench_round(sonar_out));
        bench_text(" mm out, no room to creep on: backing up ");
        bench_dec(bench_round(BENCH_AVOID_BACKUP_MM));
        bench_text(" mm (");
        bench_dec((int32_t)g_avoid.backups);
        bench_text(" of ");
        bench_dec((int32_t)BENCH_AVOID_BACKUPS);
        bench_text(")");
        bench_avoid_say();
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
    bench_text("AVOID: its edge is past ");
    bench_dec(bench_round(sonar_out));
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

/** @brief AVOID CREEP: the echo is lost, so the chosen side's edge is where
 * the sonar was at the first lost echo. Out = that edge + half the car + the
 * clearance; along (reported) from the pivot to half the car + the clearance
 * past the obstacle's back. The side is checked clear again with the new out
 * (else the run stops, as in bench_avoid_plan); then the obstacle and the map
 * go out and OUT turns on to the full angle.
 */
static void bench_avoid_creep_found (uint64_t now_us,
                                     motion_request_t * p_request)
{
    double half = BENCH_AVOID_CAR_WIDTH_MM / 2.0;
    double edge = fmax(0.0, g_avoid.miss_mm);

    p_request->forward_mps = 0.0;
    p_request->yaw_rps = 0.0;
    if (g_avoid.side > 0)
    {
        g_avoid.right_edge_mm = edge;
    }
    else
    {
        g_avoid.left_mm = edge;
    }
    g_avoid.side_mm = edge + half + BENCH_AVOID_CLEAR_MM;
    g_avoid.along_mm = (g_avoid.face_mm + g_avoid.back_mm) +
                       BENCH_AVOID_DEPTH_MM + half + BENCH_AVOID_CLEAR_MM;
    bench_text("AVOID: edge ");
    bench_dec(bench_round(edge));
    bench_text(" mm out (last echo ");
    if (0U == g_avoid.creep_hits)
    {
        bench_text("none");
    }
    else
    {
        bench_dec(bench_round(g_avoid.hit_mm));
        bench_text(" mm out, ");
        bench_dec((int32_t)g_avoid.hit_range);
        bench_text(" mm");
    }
    bench_text("): out ");
    bench_dec(bench_round(g_avoid.side_mm));
    bench_text(" mm, along ");
    bench_dec(bench_round(g_avoid.along_mm));
    bench_text(" mm");
    bench_avoid_say();
    if (!bench_avoid_side_clear(g_avoid.side, g_avoid.side_mm))
    {
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
    bench_avoid_obst_text();
    bench_avoid_map_print();
}

/** @brief AVOID: keep a sweep beam for BEAMS (bench_beams_text): the car's
 * angle from the line heading (rad, + left), the range (0 no echo) and how
 * far the pivot was behind the first stop.
 */
static void bench_avoid_keep_beam (double angle, uint16_t range_mm)
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
static bool bench_avoid_beam (double car_angle, double beam_angle,
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
    bench_avoid_mark(x_mm, y_mm, b_obstacle ? BENCH_MAP_OBSTACLE :
                                              BENCH_MAP_OTHER);
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
static void bench_avoid_mark (double x_mm, double y_mm, uint8_t code)
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
static bool bench_avoid_side_clear (int32_t side, double out_mm)
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
static void bench_avoid_plan (uint64_t now_us, motion_request_t * p_request)
{
    double half = BENCH_AVOID_CAR_WIDTH_MM / 2.0;
    double out_right = fmax(0.0, g_avoid.right_edge_mm + half +
                                 BENCH_AVOID_CLEAR_MM);
    double out_left = fmax(0.0, g_avoid.left_mm + half +
                                BENCH_AVOID_CLEAR_MM);
    bool   b_right = false;
    bool   b_left = false;

    bench_text("AVOID: face ");
    bench_dec(bench_round(g_avoid.face_mm + g_avoid.back_mm));
    bench_text(" mm ahead of the pivot; obstacle left ");
    bench_dec(bench_round(g_avoid.left_mm));
    bench_text(g_avoid.b_left_open ? " mm (edge unseen), right " :
                                     " mm, right ");
    bench_dec(bench_round(g_avoid.right_edge_mm));
    bench_text(g_avoid.b_right_open ? " mm (edge unseen); WIDTH ~" :
                                      " mm; WIDTH ~");
    bench_dec(bench_round(g_avoid.left_mm + g_avoid.right_edge_mm));
    bench_text((g_avoid.b_left_open || g_avoid.b_right_open) ?
               " mm or more" : " mm");
    bench_avoid_say();
    if ((g_avoid.b_left_open || g_avoid.b_right_open) &&
        (g_avoid.backups < BENCH_AVOID_BACKUPS))
    {
        ++g_avoid.backups;
        g_avoid.phase = BENCH_AVOID_RECENTRE;
        bench_text("AVOID: an edge is past the sweep, backing up ");
        bench_dec(bench_round(BENCH_AVOID_BACKUP_MM));
        bench_text(" mm to see more (");
        bench_dec((int32_t)g_avoid.backups);
        bench_text(" of ");
        bench_dec((int32_t)BENCH_AVOID_BACKUPS);
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
    bench_text("AVOID: path right: out ");
    bench_dec(bench_round(out_right));
    bench_text(" mm, total ");
    bench_dec(bench_round((2.0 * out_right) + g_avoid.along_mm));
    bench_text(b_right ? " mm, clear" :
               (g_avoid.b_right_open ? " mm, edge unseen" : " mm, blocked"));
    bench_text("; left: out ");
    bench_dec(bench_round(out_left));
    bench_text(" mm, total ");
    bench_dec(bench_round((2.0 * out_left) + g_avoid.along_mm));
    bench_text(b_left ? " mm, clear" :
               (g_avoid.b_left_open ? " mm, edge unseen" : " mm, blocked"));
    bench_avoid_say();
    if ((!b_right) && (!b_left))
    {
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
    /* The sweeps chose the side; the creep measures its edge, then sends
     * the obstacle and the map (bench_avoid_creep_found).
     */
    g_avoid.phase = BENCH_AVOID_CREEPTURN;
    bench_text((g_avoid.side > 0) ? "AVOID: going round the right" :
                                    "AVOID: going round the left");
    bench_text(" (sweep out ");
    bench_dec(bench_round(g_avoid.side_mm));
    bench_text(" mm), turning ");
    bench_dec((int32_t)ROBOT_SCAN_HALF_ANGLE_DEG);
    bench_text(" deg to creep out to its edge");
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
static void bench_avoid_obst_text (void)
{
    bench_text("OBST at ");
    bench_dec(bench_round(g_avoid.x0_mm));
    bench_text(" ");
    bench_dec(bench_round(g_avoid.y0_mm));
    bench_text(" h ");
    bench_fixed(bench_round(g_avoid.yaw_home * 1800.0 / CORE_PI), 1U);
    bench_text(" face ");
    bench_dec(bench_round(g_avoid.face_mm));
    bench_text(" left ");
    bench_dec(bench_round(g_avoid.left_mm));
    bench_text(g_avoid.b_left_open ? "? right " : " right ");
    bench_dec(bench_round(g_avoid.right_edge_mm));
    bench_text(g_avoid.b_right_open ? "? depth " : " depth ");
    bench_dec(bench_round(BENCH_AVOID_DEPTH_MM));
    bench_text((g_avoid.side > 0) ? " go R" :
               ((g_avoid.side < 0) ? " go L" : " go none"));
    bench_text(" out ");
    bench_dec(bench_round(g_avoid.side_mm));
    bench_text(" along ");
    bench_dec(bench_round(g_avoid.along_mm));
    bench_text(" back ");
    bench_dec(bench_round(g_avoid.back_mm));
    bench_avoid_say();
}

/** @brief AVOID: the map, one line per BENCH_MAP_CELL_MM row, the left at
 * the left: # obstacle, + something else, . free, - unknown, | the line where
 * no beam hit, C the car's pivot. AVOID alone sends every row a beam reached,
 * far rows first (ahead is up). MISSION sends live only the rows with a hit,
 * nearest first, as M lines (the row's x, mm ahead of the first stop, then
 * its cells), so the link carries a few lines, not the whole map.
 */
static void bench_avoid_map_print (void)
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
    if (first > last)
    {
        bench_text("AVOID MAP: no beams");
        bench_avoid_say();
        return;
    }
    if (BENCH_MODE_MISSION == g_drive.mode)
    {
        bench_text("AVOID MAP ");
        bench_dec(bench_round(BENCH_MAP_CELL_MM));
        bench_text(" mm cells, rows with a hit as M x cells, left at left:"
                   " # obstacle, + other, . free, - unknown, | line");
        bench_avoid_say();
        for (row = first; row <= last; ++row)
        {
            if (bench_avoid_map_row(row, car_row, car_col, line))
            {
                bench_text("M ");
                bench_dec(bench_round(BENCH_MAP_X_MIN_MM +
                                      (((double)row + 0.5) *
                                       BENCH_MAP_CELL_MM)));
                bench_text(" ");
                bench_text(line);
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
static bool bench_avoid_map_row (uint32_t row, int32_t car_row,
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
static void bench_avoid_say (void)
{
    uint32_t length = g_length;

    if (BENCH_MODE_MISSION != g_drive.mode)
    {
        bench_flush();
        return;
    }
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
        (void)memcpy(g_avoid_lines[g_avoid_queued], g_text, length);
        g_avoid_lines[g_avoid_queued][length] = '\0';
        ++g_avoid_queued;
    }
    g_length = 0U;
    g_text[0] = '\0';
}

/** @brief MISSION: the oldest queued go-around line as text (not flushed).
 * True if there was one.
 */
static bool bench_avoid_text (void)
{
    uint32_t index;

    if (0U == g_avoid_queued)
    {
        return false;
    }
    bench_text(" ");
    bench_text(g_avoid_lines[0]);
    --g_avoid_queued;
    for (index = 0U; index < g_avoid_queued; ++index)
    {
        (void)memcpy(g_avoid_lines[index], g_avoid_lines[index + 1U],
                     BENCH_AVOID_LINE_BYTES);
    }
    return true;
}

/** @brief MISSION, each step of the go-around (the line step returns before
 * bench_live_step): the live line pushed on, then the next queued line, else
 * a pose line when due, else the next sweep beams.
 */
static void bench_avoid_live (uint64_t now_us)
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
static bool bench_beams_text (void)
{
    uint32_t count = 0U;
    int32_t  back;

    if (g_beams_sent >= g_beams)
    {
        return false;
    }
    back = (int32_t)g_beam_back[g_beams_sent];
    bench_text(" BEAMS back ");
    bench_dec(back);
    bench_text(":");
    while ((g_beams_sent < g_beams) && (count < BENCH_BEAMS_PER_LINE) &&
           (((int32_t)g_beam_back[g_beams_sent] - back) <=
            BENCH_BEAMS_SAME_MM) &&
           ((back - (int32_t)g_beam_back[g_beams_sent]) <=
            BENCH_BEAMS_SAME_MM))
    {
        bench_text(" ");
        bench_dec((int32_t)g_beam_deg[g_beams_sent]);
        bench_text("/");
        bench_dec((int32_t)g_beam_mm[g_beams_sent]);
        ++g_beams_sent;
        ++count;
    }
    if ((g_beams_sent >= g_beams) && (0U != g_beams_dropped))
    {
        bench_text(" (+");
        bench_dec((int32_t)g_beams_dropped);
        bench_text(" not kept)");
    }
    return true;
}

/** @brief MISSION: every BENCH_POSE_US, the car's odometry pose as one line
 * (not flushed) for the dashboard's map: P x y mm and heading deg, + left,
 * from the run's start. True if it was due.
 */
static bool bench_pose_text (uint64_t now_us)
{
    if (now_us < g_pose_due_us)
    {
        return false;
    }
    g_pose_due_us = now_us + BENCH_POSE_US;
    bench_text(" P ");
    bench_dec(bench_round(g_drive.x_m * 1000.0));
    bench_text(" ");
    bench_dec(bench_round(g_drive.y_m * 1000.0));
    bench_text(" ");
    bench_dec(bench_round(g_drive.yaw * 180.0 / CORE_PI));
    return true;
}

/** @brief SCAN test: the robot's scanner (app_cfg_scanner), motors braked,
 * a stale echo consumed. The scan starts after the text, so no settle
 * window is spent sending.
 */
static void bench_scan_start (void)
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
static void bench_scan_tick (uint8_t command)
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
static bool bench_scan_servo (int32_t angle_deg)
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
static void bench_scan_finish (const char * p_reason)
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
static void bench_scan_verdict (void)
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
static void bench_scan_beams (void)
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
static void bench_scan_mm (double metres)
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

/** @brief BARCODE read: the store emptied, the 1 ms wake-up created once
 * (bench_code_wait), the decoder unit in metres and the robot decoder's
 * DARK/LIGHT thresholds (g_code_limits) on the side sensor's calibration
 * (app_cfg_line). No codebook: the decode reads any Code 39 text from the
 * widths alone. False, with the reason, if the calibration gives no
 * thresholds or the kernel no wake-up.
 */
static bool bench_code_start (void)
{
    line_ctrl_config_t line;
    T_CCYC             cyc;

    (void)memset(&g_code, 0, sizeof(g_code));
    (void)memset(g_code_levels, 0, sizeof(g_code_levels));
    bench_fit_reset();  /* TRIAL */
    g_code.b_stream = (BENCH_MODE_MISSION == g_mode);
    g_live_length = 0U;
    g_live_sent = 0U;
    g_live_due_us = 0U;
    g_live_side_max = 0U;
    g_live_front_min = 0U;
    g_live_front_none = 0U;
    g_raw_length = 0U;
    g_raw_sent = 0U;
    g_raw_cuts = 0U;
    g_raw_done = 0U;
    gb_raw_head = false;
    if (g_code_cyc <= 0)
    {
        (void)memset(&cyc, 0, sizeof(cyc));
        cyc.cycatr = TA_HLNG;
        cyc.cychdr = (FP)bench_code_cyc;
        cyc.cyctim = (RELTIM)BENCH_CODE_POLL_MS;
        g_bench_tid = tk_get_tid();
        g_code_cyc = tk_cre_cyc(&cyc);
        if (g_code_cyc <= 0)
        {
            bench_text("BARCODE refused: no 1 ms wake-up, tk_cre_cyc ");
            bench_signed((int32_t)g_code_cyc);
            bench_flush();
            return false;
        }
    }
    app_cfg_line(&line);
    g_code.white = line.white_adc[LINE_CTRL_BARCODE_CHANNEL];
    g_code.black = line.black_adc[LINE_CTRL_BARCODE_CHANNEL];
    g_code.unit_m = ((double)BENCH_CODE_UNIT_HC * CORE_PI *
                     ROBOT_WHEEL_RADIUS_M) /
                    (double)ROBOT_ENCODER_COUNTS_PER_REV;
    (void)memset(&g_d0, 0, sizeof(g_d0));
    g_d0.quiet = (uint16_t)bench_round((BENCH_CODE_QUIET_MM / 1000.0) /
                                       g_code.unit_m);
    if (line_ctrl_hysteresis(line.white_adc[LINE_CTRL_BARCODE_CHANNEL],
                             line.black_adc[LINE_CTRL_BARCODE_CHANNEL],
                             g_code_limits.black_on, g_code_limits.black_off,
                             &g_code.robot))
    {
        return true;
    }
    bench_text("BARCODE refused: the side sensor calibration gives no robot"
               " DARK/LIGHT thresholds");
    bench_flush();
    return false;
}

/** @brief BARCODE read: the 1 ms cyclic handler wakes the bench task. */
static void bench_code_cyc (void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk(g_bench_tid);
}

/** @brief True while a BARCODE or MISSION run follows the line (after
 * AUTO-CENTRE, before the stop): bench_tick then reads the side sensor every
 * 1 ms.
 */
static bool bench_code_polling (void)
{
    return (g_code_cyc > 0) && g_drive.b_running && (!g_drive.b_done) &&
           (!g_auto.b_active) && ((BENCH_MODE_BARCODE == g_drive.mode) ||
                                  (BENCH_MODE_MISSION == g_drive.mode));
}

/** @brief BARCODE read: bench_tick's wait, at least BENCH_TICK_MS, as
 * kernel ticks with the side sensor read at each (bench_code_poll). The
 * wake-ups queued during the last step are dropped first, so the reads
 * keep 1 ms apart.
 */
static void bench_code_wait (void)
{
    uint64_t start_us = board_time_us();

    if (!gb_code_poll)
    {
        gb_code_poll = (E_OK == tk_sta_cyc(g_code_cyc));
    }
    (void)tk_can_wup(TSK_SELF);
    do
    {
        (void)tk_slp_tsk((TMO)BENCH_CODE_POLL_TMO_MS);
        bench_code_poll();
    } while ((board_time_us() - start_us) <
             ((uint64_t)BENCH_TICK_MS * 1000U));
}

/** @brief BARCODE read between steps: the side sensor now, at the wheel
 * distances of the last step (bench_track) plus the counts since.
 */
static void bench_code_poll (void)
{
    uint16_t raw[LINE_CTRL_CHANNELS] = {0U, 0U, 0U};
    uint16_t total[BENCH_WHEELS] = {0U, 0U};
    int32_t  now[BENCH_WHEELS] = {0, 0};
    uint32_t wheel;

    if (!board_read_ir(raw))
    {
        return;
    }
    board_encoder_totals(&total[BENCH_LEFT], &total[BENCH_RIGHT]);
    for (wheel = 0U; wheel < BENCH_WHEELS; ++wheel)
    {
        now[wheel] = (g_distance[wheel] - g_drive.start[wheel]) +
                     bench_signed_delta(total[wheel], g_last_total[wheel],
                                        wheel);
    }
    bench_code_sample(now[BENCH_LEFT], now[BENCH_RIGHT],
                      raw[LINE_CTRL_BARCODE_CHANNEL]);
}

/** @brief BARCODE read, every side sensor reading: add it to the mean of
 * the decoder unit its own path is in; entering a later unit stores the
 * last (forward only). Wheel counts since the start. The sensor sits
 * ROBOT_BARCODE_LATERAL_OFFSET_M left of the centre line, so it travels the
 * centre path less that offset times the heading change (+ left, the right
 * wheel ahead): in half-counts, left plus right less 2 * offset / track *
 * (right - left). Its offset ahead of the axle moves it along the bars, not
 * across them.
 */
static void bench_code_sample (int32_t left, int32_t right, uint16_t raw)
{
    int32_t  centre_hc = left + right;
    int32_t  unit = bench_round((double)centre_hc -
                                ((2.0 * ROBOT_BARCODE_LATERAL_OFFSET_M /
                                  ROBOT_TRACK_WIDTH_M) *
                                 (double)(right - left))) /
                    BENCH_CODE_UNIT_HC;
    uint64_t now_us = board_time_us();

    if (0U == g_code.readings)
    {
        g_code.first_us = now_us;
    }
    g_code.last_us = now_us;
    ++g_code.readings;
    if (g_code.b_stream)
    {
        bench_d0_sample(unit, board_barcode_d0());
    }
    if ((unit < 0) ||
        ((0U != g_code.pending_count) && (unit < g_code.pending_unit)))
    {
        return;
    }
    if ((0U != g_code.pending_count) && (unit == g_code.pending_unit))
    {
        g_code.pending_sum += raw;
        ++g_code.pending_count;
        return;
    }
    bench_code_store();
    g_code.pending_unit = unit;
    g_code.pending_centre =
        (uint16_t)(((centre_hc > 0) ? centre_hc : 0) / BENCH_CODE_UNIT_HC);
    g_code.pending_sum = raw;
    g_code.pending_count = 1U;
}

/** @brief BARCODE read: the unit being read stored as the mean of its
 * readings, rounded, and counted in the run's reading levels
 * (g_code_levels, for bench_code_white); MISSION then streams it.
 */
static void bench_code_store (void)
{
    bench_code_sample_t * p_sample;
    bool                  b_stored = false;

    if (0U == g_code.pending_count)
    {
        return;
    }
    if ((g_code.count >= BENCH_CODE_SAMPLES) ||
        (g_code.pending_unit > (int32_t)UINT16_MAX))
    {
        g_code.b_full = true;
    }
    else
    {
        p_sample = &g_code.sample[g_code.count];
        p_sample->unit = (uint16_t)g_code.pending_unit;
        p_sample->centre = g_code.pending_centre;
        p_sample->raw = (uint16_t)((g_code.pending_sum +
                                    (g_code.pending_count / 2U)) /
                                   g_code.pending_count);
        ++g_code_levels[(uint32_t)p_sample->raw &
                        (BENCH_CODE_ADC_LEVELS - 1U)];
        ++g_code.count;
        b_stored = true;
    }
    g_code.pending_count = 0U;
    if (b_stored && g_code.b_stream)
    {
        bench_code_stream();
    }
}

/** @brief MISSION, after each stored sample: the floor that opens a group
 * renewed now and then (bench_code_white); a sample at or above it opens or
 * extends one, as does one above the smallest swing tried
 * (BENCH_CODE_TRY_MIN half sds) while open. BENCH_CODE_QUIET_MM of white
 * after the group's last such sample, or a nearly full store, cuts: what is
 * stored is decoded over the next line steps (bench_code_tries_begin), then
 * only the last gap of samples is kept, as with white alone at
 * BENCH_CODE_STREAM_HOLD samples.
 * No text here: bench_flush may wait in bench_tick, which reads again.
 */
static void bench_code_stream (void)
{
    const bench_code_sample_t * p_last = &g_code.sample[g_code.count - 1U];
    uint32_t sd = 1U;
    uint32_t white;
    uint32_t gap = (uint32_t)bench_round((BENCH_CODE_QUIET_MM / 1000.0) /
                                         g_code.unit_m);
    bool     b_cut;

    if (g_code.b_trying)
    {
        /* Stored after the cut being decoded; looked at once it is done. */
        return;
    }
    if ((0U == g_code.stream_floor) ||
        (g_code.since_floor >= BENCH_CODE_FLOOR_EVERY))
    {
        white = bench_code_white(&sd);
        g_code.stream_floor = white + (BENCH_CODE_DARK_SDS * sd);
        g_code.stream_low = white + (((BENCH_CODE_TRY_MIN * sd) + 1U) / 2U);
        g_code.since_floor = 0U;
    }
    ++g_code.since_floor;
    /* Open at the usual floor; the smallest swing tried keeps it open. */
    if (((uint32_t)p_last->raw >= g_code.stream_floor) ||
        (g_code.b_open && ((uint32_t)p_last->raw >= g_code.stream_low)))
    {
        g_code.last_dark = p_last->unit;
        g_code.b_open = true;
    }
    /* Tape-dark (the robot's DARK) longer than any bar (the spec's 9 mm) is
     * tape across the side sensor, not a card: it ends the straight hold
     * (bench_code_hold). Not the opening floor: a card's spaces stay far
     * above white's noise (31 Oct: 1500-1900), so a whole card is one run
     * there; its bars are tape-dark 3-6 mm each.
     */
    if (g_code.robot.b_dark_high &&
        ((uint32_t)p_last->raw >= (uint32_t)g_code.robot.on_adc))
    {
        if (!g_code.b_dark)
        {
            g_code.dark_from = p_last->unit;
            g_code.b_dark = true;
        }
        if (((double)(p_last->unit - g_code.dark_from) * g_code.unit_m) >
            BENCH_CODE_LINE_M)
        {
            g_code.b_tape = true;
        }
    }
    else
    {
        g_code.b_dark = false;
    }
    /* A printed bar peaks BENCH_CODE_FLOOR of the calibrated span above
     * white; the floor that opens a group is only white's noise, so the
     * hold waits for a bar.
     */
    if (g_code.b_open && (g_code.black > g_code.white) &&
        ((uint32_t)p_last->raw >=
         ((uint32_t)g_code.white +
          (uint32_t)bench_round((double)((uint32_t)g_code.black -
                                         (uint32_t)g_code.white) *
                                BENCH_CODE_FLOOR))))
    {
        g_code.b_bar = true;
    }
    /* A full store leaves BENCH_CODE_STREAM_HOLD samples for the decode. */
    b_cut = g_code.b_open ?
            ((((uint32_t)p_last->unit) >= (g_code.last_dark + gap)) ||
             (g_code.count >= (BENCH_CODE_SAMPLES - BENCH_CODE_STREAM_HOLD))) :
            (g_code.count >= BENCH_CODE_STREAM_HOLD);
    if (b_cut && g_code.b_open)
    {
        bench_code_tries_begin(gap);
    }
    else if (b_cut)
    {
        bench_code_keep(gap);
    }
    else
    {
        /* Still storing. */
    }
}

/** @brief MISSION: the store cut back to its last gap units of samples, and
 * to those after the last mark decoded (b_open still set), so no edge of it
 * is left to start the next group.
 */
static void bench_code_keep (uint32_t gap)
{
    uint32_t last = g_code.sample[g_code.count - 1U].unit;
    uint32_t first = 0U;

    while (((first + 1U) < g_code.count) &&
           ((((uint32_t)g_code.sample[first].unit + gap) < last) ||
            (g_code.b_open &&
             ((uint32_t)g_code.sample[first].unit <= g_code.last_dark))))
    {
        ++first;
    }
    (void)memmove(&g_code.sample[0], &g_code.sample[first],
                  (size_t)(g_code.count - first) * sizeof(g_code.sample[0]));
    g_code.count -= first;
}

/** @brief MISSION, a group from the quiet darkness pass: kept if it has a
 * Code 39 character's elements or more, and decoded as read, else
 * reversed, when its count fits Code 39 (framed only with a character
 * between the *s).
 */
static void bench_code_found_add (uint32_t runs)
{
    bench_code_found_t * p_found;
    uint32_t             elements = runs - 1U;
    uint32_t             last = runs - 2U;
    uint32_t             length;

    if ((runs < 2U) || (elements < BENCH_CODE_ELEMENTS))
    {
        return;
    }
    ++g_code.groups_seen;
    if (g_code.founds >= BENCH_CODE_FOUND)
    {
        return;
    }
    p_found = &g_code.found[g_code.founds];
    ++g_code.founds;
    (void)memset(p_found, 0, sizeof(*p_found));
    p_found->from_units = g_code.sample[g_code.run[0].start].unit;
    p_found->to_units = (uint32_t)g_code.sample[g_code.run[last].start].unit +
                        g_code.run[last].units;
    p_found->bars = (elements / 2U) + 1U;
    p_found->half_sds = g_code.try_half_sds;
    p_found->gap_mm = g_code.try_gap_mm;
    if ((0U != ((elements + 1U) % (BENCH_CODE_ELEMENTS + 1U))) ||
        (((elements + 1U) / (BENCH_CODE_ELEMENTS + 1U)) >= BENCH_CODE_TEXT))
    {
        bench_code_found_rest(p_found, last);
        return;
    }
    length = (elements + 1U) / (BENCH_CODE_ELEMENTS + 1U);
    if (length < BENCH_CODE_FRAMED)
    {
        (void)bench_code39_text(0U, last, false, p_found->text);
        (void)bench_code39_text(0U, last, true, p_found->reverse);
        bench_code_found_rest(p_found, last);
        return;
    }
    p_found->b_framed = bench_code39_text(0U, last, false, p_found->text) &&
                        ('*' == p_found->text[0]) &&
                        ('*' == p_found->text[length - 1U]);
    if (!p_found->b_framed)
    {
        p_found->b_reverse =
            bench_code39_text(0U, last, true, p_found->reverse) &&
            ('*' == p_found->reverse[0]) &&
            ('*' == p_found->reverse[length - 1U]);
        p_found->b_framed = p_found->b_reverse;
    }
    if (p_found->b_framed)
    {
        p_found->spec = bench_code39_spec(0U, length, last);
        if (BENCH_SPEC_OK != p_found->spec)
        {
            (void)memcpy(p_found->refused, p_found->b_reverse ?
                                           p_found->reverse : p_found->text,
                         sizeof(p_found->refused));
            p_found->b_framed = false;
            p_found->b_reverse = false;
        }
    }
    if (!p_found->b_framed)
    {
        bench_code_found_rest(p_found, last);
    }
}

/** @brief MISSION, a group whose count or frame fails: a framed window
 * (bench_code_found_window), else every character it holds
 * (bench_code_found_chars).
 */
static void bench_code_found_rest (bench_code_found_t * p_found,
                                   uint32_t last)
{
    bench_code_found_window(p_found, last);
    if (!p_found->b_framed)
    {
        bench_code_found_chars(p_found, last);
    }
}

/** @brief MISSION, a group that framed nothing: every character it holds,
 * as read and read end to start. From the first bar, a whole character
 * that decodes is kept and the next is tried past it; else the next bar is
 * tried. Characters read end to start are listed in that order.
 */
static void bench_code_found_chars (bench_code_found_t * p_found,
                                    uint32_t last)
{
    char     text[BENCH_CODE_TEXT];
    char *   p_out;
    char     swap;
    uint32_t count;
    uint32_t start;
    uint32_t index;
    uint32_t pass;
    bool     b_reverse;

    for (pass = 0U; pass < 2U; ++pass)
    {
        b_reverse = (1U == pass);
        p_out = b_reverse ? p_found->chars_reverse : p_found->chars;
        count = 0U;
        start = 0U;
        while (((start + (BENCH_CODE_ELEMENTS - 1U)) <= last) &&
               ((count + 1U) < BENCH_CODE_TEXT))
        {
            if (bench_code39_text(start, start + (BENCH_CODE_ELEMENTS - 1U),
                                  b_reverse, text))
            {
                p_out[count] = text[0];
                ++count;
                start += BENCH_CODE_ELEMENTS + 1U;
            }
            else
            {
                start += 2U;
            }
        }
        p_out[count] = '\0';
        for (index = 0U; b_reverse && ((2U * index) + 1U < count); ++index)
        {
            swap = p_out[index];
            p_out[index] = p_out[count - 1U - index];
            p_out[count - 1U - index] = swap;
        }
    }
}

/** @brief MISSION, a group whose count or frame fails: the longest window
 * of whole characters inside it, from a bar, at least BENCH_CODE_FRAMED
 * long, that reads * ... * as read or reversed. A mark beside a card then
 * cannot hide its code (30 Oct: a faint one 21 mm past *A*'s stop made 16
 * bars). Every character in the window must pass the try's rule, so a
 * window across stray bars does not frame, and the window the spec checks
 * (bench_code39_spec), its quiet zones the spaces beside it.
 */
static void bench_code_found_window (bench_code_found_t * p_found,
                                     uint32_t last)
{
    char     text[BENCH_CODE_TEXT];
    uint32_t length;
    uint32_t span;
    uint32_t start;
    uint32_t pass;
    uint32_t spec;
    bool     b_reverse;

    for (length = ((last + 2U) / (BENCH_CODE_ELEMENTS + 1U));
         length >= BENCH_CODE_FRAMED; --length)
    {
        if (length >= BENCH_CODE_TEXT)
        {
            continue;
        }
        /* A window's last element, from its first: 9 each, 1 between. */
        span = (length * (BENCH_CODE_ELEMENTS + 1U)) - 2U;
        for (start = 0U; (start + span) <= last; start += 2U)
        {
            for (pass = 0U; pass < 2U; ++pass)
            {
                b_reverse = (1U == pass);
                if (bench_code39_text(start, start + span, b_reverse, text) &&
                    ('*' == text[0]) && ('*' == text[length - 1U]))
                {
                    spec = bench_code39_spec(start, length, last);
                    if (BENCH_SPEC_OK != spec)
                    {
                        if (BENCH_SPEC_OK == p_found->spec)
                        {
                            p_found->spec = spec;
                            (void)memcpy(p_found->refused, text,
                                         sizeof(p_found->refused));
                        }
                        continue;
                    }
                    (void)memcpy(b_reverse ? p_found->reverse :
                                             p_found->text,
                                 text, sizeof(text));
                    p_found->b_reverse = b_reverse;
                    p_found->b_framed = true;
                    return;
                }
            }
        }
    }
    /* One * only: the card's first or
     * last character lost to line following, so 2 characters, exactly one
     * of them *. Only in a cut's second round of tries, after no whole
     * frame agreed; sent as a partial, the swings must still agree on it.
     */
    if (!g_code.b_partial_ok)
    {
        return;
    }
    span = (2U * (BENCH_CODE_ELEMENTS + 1U)) - 2U;
    for (start = 0U; (start + span) <= last; start += 2U)
    {
        for (pass = 0U; pass < 2U; ++pass)
        {
            b_reverse = (1U == pass);
            if (bench_code39_text(start, start + span, b_reverse, text) &&
                (('*' == text[0]) != ('*' == text[1])))
            {
                (void)memcpy(b_reverse ? p_found->reverse : p_found->text,
                             text, sizeof(text));
                p_found->b_reverse = b_reverse;
                p_found->b_framed = true;
                p_found->b_partial = true;
                return;
            }
        }
    }
}

/** @brief MISSION, from the line step: the next group not yet sent, as one
 * line of text (not flushed): CODE for a framed one, else GROUP. True if
 * there was one.
 */
static bool bench_code_found_text (void)
{
    bench_code_found_t * p_found;
    uint32_t             index;

    for (index = 0U; index < g_code.founds; ++index)
    {
        p_found = &g_code.found[index];
        if (p_found->b_sent)
        {
            continue;
        }
        p_found->b_sent = true;
        if (p_found->b_framed)
        {
            bench_text(" CODE ");
            bench_text(p_found->b_reverse ? p_found->reverse :
                                            p_found->text);
            bench_text(p_found->b_reverse ? " read end to start" :
                                            " read in order");
            bench_text(p_found->b_partial ? ", PARTIAL: one * only" : "");
            bench_text(p_found->b_peaks ? ", peaks agree" : "");
        }
        else
        {
            bench_text(" GROUP ");
            bench_dec((int32_t)p_found->bars);
            bench_text(" bars, ");
            if (p_found->b_single)
            {
                bench_text("one swing only, not a code: ");
                bench_text(p_found->b_reverse ? p_found->reverse :
                                                p_found->text);
                bench_text(p_found->b_reverse ? " end to start" : "");
            }
            else if (BENCH_SPEC_OK != p_found->spec)
            {
                bench_text("frame ");
                bench_text(p_found->refused);
                bench_text(" refused by the spec (");
                bench_text(g_spec_names[p_found->spec % BENCH_SPEC_RESULTS]);
                bench_text(")");
            }
            else if ('\0' != p_found->text[0])
            {
                bench_text("no frame: ");
                bench_text(p_found->text);
                bench_text(" / ");
                bench_text(p_found->reverse);
            }
            else
            {
                bench_text("not a Code 39 count");
            }
            if (!p_found->b_single)
            {
                bench_text(", characters ");
                bench_text(('\0' != p_found->chars[0]) ? p_found->chars :
                                                         "none");
                bench_text(" / ");
                bench_text(('\0' != p_found->chars_reverse[0]) ?
                           p_found->chars_reverse : "none");
            }
        }
        bench_text(" at ");
        bench_code_unit_mm(p_found->from_units);
        bench_text("-");
        bench_code_unit_mm(p_found->to_units);
        bench_text(" mm (swing ");
        bench_fixed((int32_t)(5U * p_found->half_sds), 1U);
        bench_text(" sd, gap ");
        bench_dec((int32_t)p_found->gap_mm);
        bench_text(" mm)");
        return true;
    }
    return false;
}

/** @brief MISSION, from the line step: live lines while it drives, never
 * waiting on the radio. The line going out is pushed on; when it is out,
 * the next group's line (bench_code_found_text), else the next line of a
 * cut's samples (bench_raw_text) until the status is due, else every
 * BENCH_LIVE_US (BENCH_LIVE_RAW_US while samples wait) a status line: time,
 * centre path, line state, side sensor now and its darkest since the last
 * status, groups and codes so far.
 */
static void bench_live_step (uint64_t now_us, uint32_t status,
                             int32_t centre_mm, uint16_t side)
{
    uint32_t index;
    uint32_t codes = 0U;

    if (side > g_live_side_max)
    {
        g_live_side_max = side;
    }
    bench_live_push();
    if ((g_live_sent < g_live_length) || (0U != g_length))
    {
        return;
    }
    if (bench_avoid_text() || bench_code_found_text() || bench_fit_text() ||
        bench_d0_text() || bench_pose_text(now_us) || bench_ramp_text() ||
        bench_cal_text())
    {
        bench_live_take();
        return;
    }
    if ((now_us < g_live_due_us) && bench_raw_text())
    {
        bench_live_take();
        return;
    }
    if (now_us < g_live_due_us)
    {
        return;
    }
    g_live_due_us = now_us + ((0U != g_raw_length) ? BENCH_LIVE_RAW_US :
                                                     BENCH_LIVE_US);
    for (index = 0U; index < g_code.founds; ++index)
    {
        if (g_code.found[index].b_framed)
        {
            ++codes;
        }
    }
    bench_text(" t ");
    bench_fixed((int32_t)((now_us - g_drive.step_us) / 100000U), 1U);
    bench_text(" s d ");
    bench_dec(centre_mm);
    bench_text(" mm ");
    bench_text(g_line_names[status % BENCH_LINE_STATES]);
    bench_text(" B ");
    bench_dec((int32_t)side);
    bench_text(" max ");
    bench_dec((int32_t)g_live_side_max);
    bench_text(" groups ");
    bench_dec((int32_t)g_code.groups_seen);
    bench_text(" codes ");
    bench_dec((int32_t)codes);
    bench_text(" F ");
    bench_dec((int32_t)g_live_front_min);
    bench_text(" x");
    bench_dec((int32_t)g_live_front_none);
    bench_ramp_status();
    g_live_side_max = 0U;
    g_live_front_min = 0U;
    g_live_front_none = 0U;
    bench_live_take();
}

/** @brief MISSION, after the stop: any open group decoded and sent like the
 * live ones, then one line of totals. Every other group went out live.
 */
static void bench_code_mission_results (void)
{
    uint32_t index;
    uint32_t codes = 0U;
    uint32_t gap_us = 0U;

    bench_code_store();
    /* TRIAL: the fit under way finished first, then the last cut, then
     * the card the fit was binning (the last cut may have joined it).
     */
    while (bench_fit_complete(false))
    {
        if (bench_fit_text())
        {
            bench_flush();
        }
    }
    if (g_code.b_trying || (g_code.b_open && (0U != g_code.count)))
    {
        bench_code_dark_tries();
    }
    while (bench_fit_complete(true))
    {
        if (bench_fit_text())
        {
            bench_flush();
        }
    }
    while (bench_code_found_text())
    {
        bench_flush();
    }
    bench_fit_totals();
    while (bench_avoid_text())
    {
        bench_flush();
    }
    if (0U != g_avoid_dropped)
    {
        bench_text(" obstacle go-around: ");
        bench_dec((int32_t)g_avoid_dropped);
        bench_text(" lines dropped (queue full)");
        bench_flush();
    }
    bench_d0_close();
    while (bench_d0_text())
    {
        bench_flush();
    }
    bench_text(" D0: ");
    bench_dec((int32_t)g_d0.changes);
    bench_text(" changes, ");
    bench_dec((int32_t)g_d0.groups);
    bench_text(" groups (");
    bench_dec((int32_t)g_d0.shorts);
    bench_text(" under ");
    bench_dec((int32_t)BENCH_D0_BARS_MIN);
    bench_text(" bars), ");
    bench_dec((int32_t)g_d0.reads);
    bench_text(" read, ");
    bench_dec((int32_t)g_d0.dropped);
    bench_text(" dropped; high ");
    bench_dec((int32_t)g_d0.high);
    bench_text(", low ");
    bench_dec((int32_t)g_d0.low);
    bench_text(" readings, dark is ");
    bench_text((g_d0.high <= g_d0.low) ? "high" : "low");
    bench_flush();
    for (index = 0U; index < g_code.founds; ++index)
    {
        if (g_code.found[index].b_framed)
        {
            ++codes;
        }
    }
    if (g_code.readings > 1U)
    {
        gap_us = (uint32_t)((g_code.last_us - g_code.first_us) /
                            (uint64_t)(g_code.readings - 1U));
    }
    bench_text(" barcodes: ");
    bench_dec((int32_t)g_code.readings);
    bench_text(" readings, one per ");
    bench_fixed((int32_t)(gap_us / 10U), 2U);
    bench_text(" ms; groups ");
    bench_dec((int32_t)g_code.groups_seen);
    bench_text(", codes ");
    bench_dec((int32_t)codes);
    bench_text("; held straight ");
    bench_dec((int32_t)g_code.holds);
    bench_text(" times, ");
    bench_dec((int32_t)g_code.held_mm);
    bench_text(" mm, let go ");
    bench_dec((int32_t)g_code.let_gos);
    if (g_code.groups_seen > g_code.founds)
    {
        bench_text(", the first ");
        bench_dec((int32_t)g_code.founds);
        bench_text(" kept");
    }
    if (g_code.b_full)
    {
        bench_text("; STORE FULL");
    }
    /* Counted over every try, so one card can count several times. */
    bench_text("; spec refused frames: widths ");
    bench_dec((int32_t)g_code.spec_widths);
    bench_text(", gaps ");
    bench_dec((int32_t)g_code.spec_gaps);
    bench_text(", quiet ");
    bench_dec((int32_t)g_code.spec_quiet);
    /* Samples still waiting are dropped: nothing long after the stop. */
    g_raw_length = 0U;
    bench_text("; raw sent for ");
    bench_dec((int32_t)g_raw_done);
    bench_text(" of ");
    bench_dec((int32_t)g_raw_cuts);
    bench_text(" cuts");
    bench_flush();
}

/** @brief MISSION, every side sensor reading: D0 at the unit of the
 * sensor's own path (forward only: a unit behind the last counts as the
 * last). The level seen less over the run so far is dark. A change to dark
 * opens a group or adds to it; a change to light ends a bar. Light lasting
 * BENCH_CODE_QUIET_MM closes the group (bench_d0_close).
 */
static void bench_d0_sample (int32_t unit, bool b_level)
{
    bench_d0_group_t * p_open = &g_d0.open;
    uint16_t           at;
    bool               b_dark_high;

    if (unit < 0)
    {
        return;
    }
    at = (unit > (int32_t)UINT16_MAX) ? (uint16_t)UINT16_MAX :
                                        (uint16_t)unit;
    if (g_d0.b_started && (at < g_d0.unit))
    {
        at = g_d0.unit;
    }
    g_d0.unit = at;
    if (b_level)
    {
        ++g_d0.high;
    }
    else
    {
        ++g_d0.low;
    }
    b_dark_high = (g_d0.high <= g_d0.low);
    if (!g_d0.b_started)
    {
        g_d0.b_started = true;
        g_d0.b_level = b_level;
        g_d0.light_from = at;
        return;
    }
    if (b_level != g_d0.b_level)
    {
        g_d0.b_level = b_level;
        ++g_d0.changes;
        if (b_level == b_dark_high)
        {
            if (!g_d0.b_open)
            {
                (void)memset(p_open, 0, sizeof(*p_open));
                g_d0.b_open = true;
            }
        }
        else
        {
            g_d0.light_from = at;
        }
        if (g_d0.b_open)
        {
            if (p_open->edges < BENCH_D0_EDGES)
            {
                p_open->edge[p_open->edges] = at;
            }
            ++p_open->edges;
        }
    }
    if (g_d0.b_open && (b_level != b_dark_high) &&
        ((uint32_t)(at - g_d0.light_from) >= (uint32_t)g_d0.quiet))
    {
        bench_d0_close();
    }
}

/** @brief MISSION: the open D0 group closed (a bar still dark at the stop
 * ends at the last reading). Under BENCH_D0_BARS_MIN bars it is only
 * counted; else it waits to be sent, or is dropped if BENCH_D0_GROUPS wait.
 */
static void bench_d0_close (void)
{
    bench_d0_group_t * p_open = &g_d0.open;

    if (!g_d0.b_open)
    {
        return;
    }
    g_d0.b_open = false;
    if (0U != (p_open->edges & 1U))
    {
        if (p_open->edges < BENCH_D0_EDGES)
        {
            p_open->edge[p_open->edges] = g_d0.unit;
        }
        ++p_open->edges;
    }
    ++g_d0.groups;
    if (p_open->edges < (2U * BENCH_D0_BARS_MIN))
    {
        ++g_d0.shorts;
    }
    else if (g_d0.queued >= BENCH_D0_GROUPS)
    {
        ++g_d0.dropped;
    }
    else
    {
        g_d0.group[g_d0.queued] = *p_open;
        ++g_d0.queued;
    }
}

/** @brief MISSION: a closed D0 group as Code 39, from either end. Elements
 * are the widths between changes (bar, space, ...); every character is 9
 * of them and a gap, so a code is a multiple of 10 less one. In each
 * character the 3 widest are wide, and the narrowest wide must be wider
 * than the widest narrow, else the character is '?'. p_text (at least
 * BENCH_CODE_TEXT) gets the characters, or "" when the count is not Code
 * 39. True for a frame: *, at least one character, *, none '?'.
 */
static bool bench_d0_decode (const bench_d0_group_t * p_group, bool b_reverse,
                             char * p_text)
{
    uint32_t units[BENCH_CODE_ELEMENTS];
    uint32_t elements = p_group->edges - 1U;
    uint32_t length;
    uint32_t character;
    uint32_t element;
    uint32_t index;
    uint32_t wide;
    uint32_t widest;
    uint32_t narrowest_wide;
    uint32_t widest_narrow;
    uint32_t pattern;
    bool     b_all = true;

    p_text[0] = '\0';
    if ((p_group->edges < 2U) || (p_group->edges > BENCH_D0_EDGES) ||
        (0U != ((elements + 1U) % (BENCH_CODE_ELEMENTS + 1U))))
    {
        return false;
    }
    length = (elements + 1U) / (BENCH_CODE_ELEMENTS + 1U);
    if (length >= BENCH_CODE_TEXT)
    {
        return false;
    }
    for (character = 0U; character < length; ++character)
    {
        for (element = 0U; element < BENCH_CODE_ELEMENTS; ++element)
        {
            index = (character * (BENCH_CODE_ELEMENTS + 1U)) + element;
            if (b_reverse)
            {
                index = elements - 1U - index;
            }
            units[element] = (uint32_t)p_group->edge[index + 1U] -
                             (uint32_t)p_group->edge[index];
        }
        pattern = 0U;
        for (wide = 0U; wide < 3U; ++wide)
        {
            widest = BENCH_CODE_ELEMENTS;
            for (element = 0U; element < BENCH_CODE_ELEMENTS; ++element)
            {
                if ((0U == (pattern & (0x100U >> element))) &&
                    ((BENCH_CODE_ELEMENTS == widest) ||
                     (units[element] > units[widest])))
                {
                    widest = element;
                }
            }
            pattern |= 0x100U >> widest;
        }
        narrowest_wide = UINT32_MAX;
        widest_narrow = 0U;
        for (element = 0U; element < BENCH_CODE_ELEMENTS; ++element)
        {
            if (0U != (pattern & (0x100U >> element)))
            {
                if (units[element] < narrowest_wide)
                {
                    narrowest_wide = units[element];
                }
            }
            else if (units[element] > widest_narrow)
            {
                widest_narrow = units[element];
            }
        }
        index = 0U;
        while ((index < BENCH_CODE_CHARS) &&
               (g_code39_patterns[index] != pattern))
        {
            ++index;
        }
        if ((index < BENCH_CODE_CHARS) && (narrowest_wide > widest_narrow))
        {
            p_text[character] = g_code39_chars[index];
        }
        else
        {
            p_text[character] = '?';
            b_all = false;
        }
    }
    p_text[length] = '\0';
    return b_all && (length >= BENCH_CODE_FRAMED) && ('*' == p_text[0]) &&
           ('*' == p_text[length - 1U]);
}

/** @brief MISSION, from the line step and after the stop: the oldest closed
 * D0 group as two lines of text (not flushed), D0 then D0 widths: its code
 * or why not, where it was, then its element widths in decoder units. True
 * if there was a line.
 */
static bool bench_d0_text (void)
{
    const bench_d0_group_t * p_group = &g_d0.group[0];
    char                     text[BENCH_CODE_TEXT];
    char                     reverse[BENCH_CODE_TEXT];
    uint32_t                 stored;
    uint32_t                 index;
    bool                     b_order;
    bool                     b_reverse;

    if (0U == g_d0.queued)
    {
        return false;
    }
    stored = (p_group->edges < BENCH_D0_EDGES) ? p_group->edges :
                                                 BENCH_D0_EDGES;
    if (!g_d0.b_widths)
    {
        b_order = bench_d0_decode(p_group, false, text);
        b_reverse = bench_d0_decode(p_group, true, reverse);
        bench_text(" D0 ");
        if (b_order || b_reverse)
        {
            ++g_d0.reads;
            bench_text(b_order ? text : reverse);
            bench_text(b_order ? " read in order" : " read end to start");
        }
        else
        {
            bench_dec((int32_t)(p_group->edges / 2U));
            bench_text(" bars, ");
            if (p_group->edges > BENCH_D0_EDGES)
            {
                bench_text("too many to keep");
            }
            else if ('\0' != text[0])
            {
                bench_text("no frame: ");
                bench_text(text);
                bench_text(" / ");
                bench_text(reverse);
            }
            else
            {
                bench_text("not a Code 39 count");
            }
        }
        bench_text(" at ");
        bench_code_unit_mm(p_group->edge[0]);
        bench_text("-");
        bench_code_unit_mm(p_group->edge[stored - 1U]);
        bench_text(" mm");
        g_d0.b_widths = true;
        return true;
    }
    bench_text(" D0 widths u:");
    for (index = 1U; (index < stored) && (index <= BENCH_D0_WIDTHS); ++index)
    {
        bench_text(" ");
        bench_dec((int32_t)p_group->edge[index] -
                  (int32_t)p_group->edge[index - 1U]);
    }
    g_d0.b_widths = false;
    --g_d0.queued;
    for (index = 0U; index < g_d0.queued; ++index)
    {
        g_d0.group[index] = g_d0.group[index + 1U];
    }
    return true;
}

/** @brief MISSION, a cut that made a group: its samples, one character per
 * unit (a unit without a reading as BENCH_RAW_GAP), to go out while the car
 * drives (bench_raw_text). Skipped while the last cut is still going out.
 */
static void bench_raw_keep (void)
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
static bool bench_raw_text (void)
{
    uint32_t chunk;

    if (0U == g_raw_length)
    {
        return false;
    }
    if (gb_raw_head)
    {
        gb_raw_head = false;
        bench_text(" RAW ");
        bench_dec((int32_t)g_raw_number);
        bench_text(": u");
        bench_dec((int32_t)g_raw_first);
        bench_text(" (side ");
        bench_code_unit_mm(g_raw_first);
        bench_text(" mm), ");
        bench_dec((int32_t)g_raw_length);
        bench_text(" units, white ");
        bench_dec((int32_t)g_raw_white);
        bench_text(" sd ");
        bench_dec((int32_t)g_raw_sd);
        bench_text("; raw = white - 10 + 2 (c - '#') below 'c', else white"
                   " + 118 + 140 (c - 'c'); '!' none");
        return true;
    }
    bench_text(" R ");
    bench_dec((int32_t)g_raw_number);
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

/** @brief MISSION start: the terrain module, configured as on the robot,
 * with a flat reference from the car's last idle second when it held still
 * (else the stored one), and the run's ramp figures cleared.
 */
static void bench_ramp_start (void)
{
    terrain_config_t config;
    bench_accel_t    flat = g_ramp.flat;
    double           mg[3] = {0.0, 0.0, 0.0};
    double           axes[3] = {0.0, 0.0, 0.0};
    bool             b_still = (flat.count >= BENCH_RAMP_REF_MIN);
    uint32_t         axis;

    (void)memset(&g_ramp, 0, sizeof(g_ramp));
    g_ramp.flat = flat;
    for (axis = 0U; axis < 3U; ++axis)
    {
        b_still = b_still && (((int32_t)flat.maximum[axis] -
                               (int32_t)flat.minimum[axis]) <=
                              BENCH_RAMP_STILL_MG);
    }
    if (b_still)
    {
        for (axis = 0U; axis < 3U; ++axis)
        {
            mg[axis] = (double)flat.sum[axis] / (double)flat.count;
        }
        bench_ramp_axes(mg, axes);
    }
    else
    {
        axes[0] = ROBOT_IMU_REFERENCE_X_MG * BENCH_MG_TO_MPS2;
        axes[1] = ROBOT_IMU_REFERENCE_Y_MG * BENCH_MG_TO_MPS2;
        axes[2] = ROBOT_IMU_REFERENCE_Z_MG * BENCH_MG_TO_MPS2;
    }
    app_cfg_terrain(&config);
    /* The robot also waits on ROBOT_CALIBRATION_CONFIRMED (IR, course);
     * the accelerometer fit and mount themselves are measured (S4-U03/U04).
     */
    config.b_calibrated = true;
    g_ramp.b_ready = gb_accel && terrain_init(&g_ramp.state, &config) &&
                     terrain_reference(&g_ramp.state, axes[0], axes[1],
                                       axes[2], true);
    g_ramp.b_measured = b_still;
    g_ramp.last_us = board_time_us();
    g_ramp.window_us = g_ramp.last_us;
    g_ramp.path = g_distance[BENCH_LEFT] + g_distance[BENCH_RIGHT];
    g_ramp.window_path = g_ramp.path;
    g_ramp.cell_path = g_ramp.path;
    g_ramp.cells = 1U;             /* the start, at level 0 */
    g_ramp.cell = 1U;
    if (!g_ramp.b_ready)
    {
        bench_text(gb_accel ? "RAMP off: the reference was refused" :
                              "RAMP off: no accelerometer");
        bench_flush();
        return;
    }
    bench_text(b_still ? "RAMP flat reference: the last still second (n " :
                         "RAMP flat reference: stored (car not still, n ");
    bench_dec((int32_t)flat.count);
    bench_text("), start pitch ");
    bench_ramp_deg(atan2(axes[0], hypot(axes[1], axes[2])));
    bench_text(" roll ");
    bench_ramp_deg(atan2(axes[1], axes[2]));
    bench_text(" deg");
    bench_flush();
}

/** @brief Sensor mg to vehicle axes in m/s^2, as robot_app_imu_step. */
static void bench_ramp_axes (const double * p_mg, double * p_mps2)
{
    double axes[3];

    axes[0] = (p_mg[0] - ROBOT_ACCEL_BIAS_X_MG) * ROBOT_ACCEL_SCALE_X *
              BENCH_MG_TO_MPS2;
    axes[1] = (p_mg[1] - ROBOT_ACCEL_BIAS_Y_MG) * ROBOT_ACCEL_SCALE_Y *
              BENCH_MG_TO_MPS2;
    axes[2] = (p_mg[2] - ROBOT_ACCEL_BIAS_Z_MG) * ROBOT_ACCEL_SCALE_Z *
              BENCH_MG_TO_MPS2;
    p_mps2[0] = axes[ROBOT_IMU_FORWARD_AXIS] * ROBOT_IMU_FORWARD_SIGN;
    p_mps2[1] = axes[ROBOT_IMU_LEFT_AXIS] * ROBOT_IMU_LEFT_SIGN;
    p_mps2[2] = axes[ROBOT_IMU_UP_AXIS] * ROBOT_IMU_UP_SIGN;
}

/** @brief MISSION, each drive step: a fresh accelerometer sample (100 Hz)
 * into the terrain module, then the bench hump on its pitch. The wheel path
 * since the last sample is the centre's, forward only (the car never
 * reverses); roll + is the left side up.
 */
static void bench_ramp_step (uint64_t now_us,
                             const motion_output_t * p_output)
{
    const terrain_config_t * p_config = &g_ramp.state.config;
    terrain_input_t input;
    int16_t         milligravity[3] = {0, 0, 0};
    double          mg[3];
    double          axes[3];
    double          speed = 0.0;
    double          ds = 0.0;
    double          change = 0.0;
    int32_t         path = g_distance[BENCH_LEFT] + g_distance[BENCH_RIGHT];
    int32_t         centre_mm = bench_mm(((g_distance[BENCH_LEFT] -
                                           g_drive.start[BENCH_LEFT]) +
                                          (g_distance[BENCH_RIGHT] -
                                           g_drive.start[BENCH_RIGHT])) / 2);
    uint32_t        axis;

    if ((!g_ramp.b_ready) ||
        (BOARD_ACCEL_FRESH != board_read_accel(milligravity)))
    {
        return;
    }
    for (axis = 0U; axis < 3U; ++axis)
    {
        mg[axis] = (double)milligravity[axis];
    }
    bench_ramp_axes(mg, axes);
    (void)memset(&input, 0, sizeof(input));
    input.accel_x_mps2 = axes[0];
    input.accel_y_mps2 = axes[1];
    input.accel_z_mps2 = axes[2];
    input.signed_distance_m = (double)(path - g_ramp.path) *
                              (BENCH_COUNT_M / 2.0);
    input.encoder_yaw_rate_rps = (p_output->right_speed_mps -
                                  p_output->left_speed_mps) /
                                 ROBOT_TRACK_WIDTH_M;
    input.dt_s = (double)(now_us - g_ramp.last_us) / 1000000.0;
    input.b_sensor_valid = true;
    input.b_encoder_valid = true;
    speed = (p_output->left_speed_mps + p_output->right_speed_mps) / 2.0;
    if ((now_us - g_ramp.window_us) >= BENCH_RAMP_WINDOW_US)
    {
        g_ramp.window_mps = ((double)(path - g_ramp.window_path) *
                             (BENCH_COUNT_M / 2.0)) /
                            ((double)(now_us - g_ramp.window_us) / 1000000.0);
        g_ramp.window_path = path;
        g_ramp.window_us = now_us;
    }
    /* robot_app_imu_step's quasi-static test, its hypotheses too. */
    input.b_quasistatic = p_output->b_stationary ||
                          ((fabs(speed - g_ramp.window_mps) < 0.15) &&
                           (fabs(input.accel_x_mps2) < 4.0));
    g_ramp.path = path;
    g_ramp.last_us = now_us;
    ++g_ramp.samples;
    if ((input.dt_s > 0.0) && (input.dt_s <= p_config->maximum_dt_s))
    {
        /* The car's own acceleration: the filtered speed's change. */
        change = ((input.signed_distance_m / input.dt_s) - g_ramp.speed) *
                 (input.dt_s / (p_config->filter_time_s + input.dt_s));
        g_ramp.speed += change;
        input.accel_x_mps2 -= change / input.dt_s;
    }
    (void)terrain_step(&g_ramp.state, &input, &g_ramp.output);
    if (g_ramp.output.b_hump_completed &&
        (g_ramp.output.b_last_hump_valid ||
         (g_ramp.output.hump_peak_m >= p_config->minimum_hump_m)))
    {
        g_ramp.b_terrain = true;
        g_ramp.b_terrain_valid = g_ramp.output.b_last_hump_valid;
        g_ramp.terrain_peak = g_ramp.output.hump_peak_m;
    }
    if (g_ramp.output.b_tilt_valid)
    {
        g_ramp.raw_pitch = g_ramp.output.pitch_rad;
        g_ramp.raw_roll = g_ramp.output.roll_rad;
    }
    else
    {
        ++g_ramp.refused;
    }
    ds = (input.signed_distance_m > 0.0) ? input.signed_distance_m : 0.0;
    bench_ramp_offset(ds);
    if (g_ramp.roll > g_ramp.left)
    {
        g_ramp.left = g_ramp.roll;
        g_ramp.left_mm = centre_mm;
    }
    if (g_ramp.roll < g_ramp.right)
    {
        g_ramp.right = g_ramp.roll;
        g_ramp.right_mm = centre_mm;
    }
    g_ramp.level += ds * sin(g_ramp.pitch);
    bench_ramp_rise(now_us, path, centre_mm);
    bench_ramp_lean(now_us, ds, centre_mm);
}

/** @brief MISSION, each accelerometer sample: the side height, and the lean
 * it makes (BENCH_RAMP_LEAN_M).
 */
static void bench_ramp_lean (uint64_t now_us, double ds, int32_t centre_mm)
{
    const double minimum = g_ramp.state.config.minimum_hump_m;
    double       side = ROBOT_TRACK_WIDTH_M * sin(g_ramp.roll);

    if ((!g_ramp.b_leaning) && (fabs(side) >= minimum))
    {
        g_ramp.b_leaning = true;
        (void)memset(&g_ramp.lean, 0, sizeof(g_ramp.lean));
        g_ramp.lean.from_mm = centre_mm;
    }
    if (!g_ramp.b_leaning)
    {
        return;
    }
    g_ramp.lean.length += ds;
    if (fabs(side) > fabs(g_ramp.lean.peak))
    {
        g_ramp.lean.peak = side;
        g_ramp.lean.pitch = g_ramp.pitch;
        g_ramp.lean.peak_mm = centre_mm;
        g_ramp.lean.peak_ds = (int32_t)((now_us - g_drive.step_us) /
                                        100000U);
    }
    if (fabs(side) >= (0.5 * minimum))
    {
        return;
    }
    g_ramp.b_leaning = false;
    if (g_ramp.lean.length < BENCH_RAMP_LEAN_M)
    {
        ++g_ramp.jolts;
        return;
    }
    ++g_ramp.leans;
    g_ramp.lean_sent = g_ramp.lean;
    g_ramp.b_lean_send = true;
    if (fabs(g_ramp.lean.peak) > fabs(g_ramp.lean_best.peak))
    {
        g_ramp.lean_best = g_ramp.lean;
        g_ramp.best_lean = g_ramp.leans;
    }
}

/** @brief A lean as text (not flushed): side, height, roll, where, how long.
 */
static void bench_ramp_lean_line (const char * p_head,
                                  const bench_lean_t * p_lean)
{
    bench_text(p_head);
    bench_text((p_lean->peak >= 0.0) ? "left wheel up " : "right wheel up ");
    bench_dec(bench_round(fabs(p_lean->peak) * 1000.0));
    bench_text(" mm (roll ");
    bench_ramp_deg(asin(p_lean->peak / ROBOT_TRACK_WIDTH_M));
    bench_text(" deg, pitch ");
    bench_ramp_deg(p_lean->pitch);
    bench_text(") at ");
    bench_dec(p_lean->peak_mm);
    bench_text(" mm, t ");
    bench_fixed(p_lean->peak_ds, 1U);
    bench_text(" s, over ");
    bench_dec(bench_round(p_lean->length * 1000.0));
    bench_text(" mm");
}

/** @brief MISSION, each line sample: the START calibration's extremes, over
 * AUTO-CENTRE and the first BENCH_CAL_M of line.
 */
static void bench_cal_sample (const uint16_t * p_raw, int32_t centre_mm)
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
static bool bench_cal_text (void)
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
    for (channel = 0U; channel < LINE_CTRL_CHANNELS; ++channel)
    {
        bench_text(names[channel]);
        bench_dec((int32_t)g_cal.low[channel]);
        bench_text("-");
        bench_dec((int32_t)g_cal.high[channel]);
        bench_text(" (set ");
        bench_dec((int32_t)white[channel]);
        bench_text("-");
        bench_dec((int32_t)black[channel]);
        bench_text(")");
    }
    bench_text(" over AUTO-CENTRE and ");
    bench_dec(g_cal.done_mm);
    bench_text(" mm");
    return true;
}

/** @brief The car's own pitch and roll offset, their mean over the path so
 * far up to the last BENCH_RAMP_OFFSET_M, taken off both: a START on a
 * tilted spot, the stored reference when the car was moving there, a mount
 * that has shifted (30 Oct: +2.4 deg nose up against the stored floor, which
 * the old hump integrated into 63 mm). A ramp tilts the car up, then as
 * much down, so it barely moves the mean.
 */
static void bench_ramp_offset (double ds)
{
    double share = 0.0;

    if (ds > 0.0)
    {
        g_ramp.travelled += ds;
        share = ds / fmin(g_ramp.travelled, BENCH_RAMP_OFFSET_M);
        g_ramp.pitch_offset += share * (g_ramp.raw_pitch -
                                        g_ramp.pitch_offset);
        g_ramp.roll_offset += share * (g_ramp.raw_roll - g_ramp.roll_offset);
    }
    g_ramp.pitch = g_ramp.raw_pitch - g_ramp.pitch_offset;
    g_ramp.roll = g_ramp.raw_roll - g_ramp.roll_offset;
}

/** @brief MISSION, each accelerometer sample: the level above the floor
 * behind, and the rise it makes.
 */
static void bench_ramp_rise (uint64_t now_us, int32_t path, int32_t centre_mm)
{
    const double floor_cell = BENCH_RAMP_CELL_M / (BENCH_COUNT_M / 2.0);
    const double minimum = g_ramp.state.config.minimum_hump_m;
    double       floor = g_ramp.level;
    uint32_t     index;

    if ((double)(path - g_ramp.cell_path) >= floor_cell)
    {
        g_ramp.cell_path = path;
        g_ramp.floor[g_ramp.cell] = g_ramp.level;
        g_ramp.cell = (g_ramp.cell + 1U) % BENCH_RAMP_CELLS;
        if (g_ramp.cells < BENCH_RAMP_CELLS)
        {
            ++g_ramp.cells;
        }
    }
    for (index = 0U; index < g_ramp.cells; ++index)
    {
        floor = fmin(floor, g_ramp.floor[index]);
    }
    g_ramp.above = g_ramp.level - floor;
    if ((!g_ramp.b_rising) && (g_ramp.above >= minimum))
    {
        g_ramp.b_rising = true;
        (void)memset(&g_ramp.rise, 0, sizeof(g_ramp.rise));
        g_ramp.rise.from_mm = centre_mm;
        g_ramp.rise.up = g_ramp.pitch;
        g_ramp.rise.down = g_ramp.pitch;
    }
    if (!g_ramp.b_rising)
    {
        return;
    }
    if (g_ramp.above > g_ramp.rise.peak)
    {
        g_ramp.rise.peak = g_ramp.above;
        g_ramp.rise.roll = g_ramp.roll;
        g_ramp.rise.peak_mm = centre_mm;
        g_ramp.rise.peak_ds = (int32_t)((now_us - g_drive.step_us) /
                                        100000U);
    }
    g_ramp.rise.up = fmax(g_ramp.rise.up, g_ramp.pitch);
    g_ramp.rise.down = fmin(g_ramp.rise.down, g_ramp.pitch);
    if (g_ramp.above < (0.5 * minimum))
    {
        g_ramp.b_rising = false;
        ++g_ramp.rises;
        g_ramp.sent = g_ramp.rise;
        g_ramp.b_send = true;
        if (g_ramp.rise.peak > g_ramp.best.peak)
        {
            g_ramp.best = g_ramp.rise;
            g_ramp.best_rise = g_ramp.rises;
        }
    }
}

/** @brief MISSION live: a closed hump, the bench's or the module's. */
static bool bench_ramp_text (void)
{
    if (g_ramp.b_send)
    {
        g_ramp.b_send = false;
        bench_text(" RAMP ");
        bench_dec((int32_t)g_ramp.rises);
        bench_text(": peak ");
        bench_dec(bench_round(g_ramp.sent.peak * 1000.0));
        bench_text(" mm at ");
        bench_dec(g_ramp.sent.peak_mm);
        bench_text(" mm (t ");
        bench_fixed(g_ramp.sent.peak_ds, 1U);
        bench_text(" s), roll ");
        bench_ramp_deg(g_ramp.sent.roll);
        bench_ramp_side(g_ramp.sent.roll);
        bench_text("; pitch up ");
        bench_ramp_deg(g_ramp.sent.up);
        bench_text(" down ");
        bench_ramp_deg(g_ramp.sent.down);
        bench_text(" deg; from ");
        bench_dec(g_ramp.sent.from_mm);
        bench_text(" mm");
        return true;
    }
    if (g_ramp.b_lean_send)
    {
        g_ramp.b_lean_send = false;
        bench_text(" SIDE ");
        bench_dec((int32_t)g_ramp.leans);
        bench_ramp_lean_line(": ", &g_ramp.lean_sent);
        return true;
    }
    if (g_ramp.b_terrain)
    {
        g_ramp.b_terrain = false;
        bench_text(g_ramp.b_terrain_valid ? " TERRAIN hump counted: peak " :
                   " TERRAIN hump not counted (turn, jolt or not level at"
                   " the end): peak ");
        bench_dec(bench_round(g_ramp.terrain_peak * 1000.0));
        bench_text(" mm");
        return true;
    }
    return false;
}

/** @brief MISSION status line: pitch, roll, and the height in a hump. */
static void bench_ramp_status (void)
{
    if (!g_ramp.b_ready)
    {
        return;
    }
    bench_text(" pitch ");
    bench_ramp_deg(g_ramp.pitch);
    bench_text(" roll ");
    bench_ramp_deg(g_ramp.roll);
    bench_text(" h ");
    bench_dec(bench_round(g_ramp.above * 1000.0));
}

/** @brief An angle in degrees, signed, one decimal. */
static void bench_ramp_deg (double rad)
{
    int32_t tenths = bench_round(rad * (1800.0 / CORE_PI));

    if (tenths >= 0)
    {
        bench_text("+");
    }
    bench_fixed(tenths, 1U);
}

/** @brief Which wheel is higher at this roll, and by how much. */
static void bench_ramp_side (double roll)
{
    int32_t mm = bench_round(fabs(sin(roll)) * ROBOT_TRACK_WIDTH_M * 1000.0);

    bench_text((roll >= 0.0) ? " deg (left wheel " : " deg (right wheel ");
    bench_dec(mm);
    bench_text(" mm higher)");
}

/** @brief MISSION, after the stop: humps not yet sent, then the run's
 * highest point (an open hump counts), its side, and the most roll.
 */
static void bench_ramp_results (void)
{
    if (!g_ramp.b_ready)
    {
        return;
    }
    while (bench_ramp_text())
    {
        bench_flush();
    }
    if (g_ramp.b_rising && (g_ramp.rise.peak > g_ramp.best.peak))
    {
        g_ramp.best = g_ramp.rise;
        g_ramp.best_rise = g_ramp.rises + 1U;
    }
    bench_text(" ramp: highest ");
    bench_dec(bench_round(g_ramp.best.peak * 1000.0));
    bench_text(" mm");
    if (g_ramp.best.peak > 0.0)
    {
        bench_text(" (rise ");
        bench_dec((int32_t)g_ramp.best_rise);
        bench_text((g_ramp.best_rise > g_ramp.rises) ?
                   ", still open at the stop) at " : ") at ");
        bench_dec(g_ramp.best.peak_mm);
        bench_text(" mm, t ");
        bench_fixed(g_ramp.best.peak_ds, 1U);
        bench_text(" s, roll ");
        bench_ramp_deg(g_ramp.best.roll);
        bench_ramp_side(g_ramp.best.roll);
    }
    bench_flush();
    bench_text(" ramp: rises ");
    bench_dec((int32_t)g_ramp.rises);
    bench_text(" of 5 mm or more, robot module ");
    bench_dec((int32_t)g_ramp.output.completed_humps);
    bench_text(" counted, max ");
    bench_dec((int32_t)terrain_height_mm(g_ramp.output.run_maximum_m));
    bench_text(" mm; car's own offset pitch ");
    bench_ramp_deg(g_ramp.pitch_offset);
    bench_text(" roll ");
    bench_ramp_deg(g_ramp.roll_offset);
    bench_text(" deg");
    bench_flush();
    if (g_ramp.b_leaning && (g_ramp.lean.length >= BENCH_RAMP_LEAN_M) &&
        (fabs(g_ramp.lean.peak) > fabs(g_ramp.lean_best.peak)))
    {
        g_ramp.lean_best = g_ramp.lean;
        g_ramp.best_lean = g_ramp.leans + 1U;
    }
    if (0U != g_ramp.best_lean)
    {
        bench_ramp_lean_line(" ramp: most side lean ", &g_ramp.lean_best);
    }
    else
    {
        bench_text(" ramp: no side lean of 5 mm");
    }
    bench_text("; leans ");
    bench_dec((int32_t)g_ramp.leans);
    bench_text(", jolts ");
    bench_dec((int32_t)g_ramp.jolts);
    bench_flush();
    bench_text(" ramp: most roll left ");
    bench_ramp_deg(g_ramp.left);
    bench_text(" at ");
    bench_dec(g_ramp.left_mm);
    bench_text(" mm, right ");
    bench_ramp_deg(g_ramp.right);
    bench_text(" at ");
    bench_dec(g_ramp.right_mm);
    bench_text(" mm; samples ");
    bench_dec((int32_t)g_ramp.samples);
    bench_text(", no tilt ");
    bench_dec((int32_t)g_ramp.refused);
    bench_flush();
}

/** @brief BARCODE read, after the stop: raw range and thresholds, each dark
 * group's widths and Code 39 decode, again on this run's own thresholds
 * if the robot's decoded nothing, then the trace.
 */
static void bench_code_results (void)
{
    line_ctrl_hysteresis_t own;
    uint32_t               index;
    uint32_t               groups = 0U;
    uint32_t               peak = 0U;
    bool                   b_decoded = false;
    uint32_t               gap_us = 0U;

    bench_code_store();
    if (0U == g_code.count)
    {
        bench_text(" BARCODE: no side sensor samples (no forward travel)");
        bench_flush();
        return;
    }
    g_code.minimum = g_code.sample[0].raw;
    g_code.maximum = g_code.sample[0].raw;
    for (index = 1U; index < g_code.count; ++index)
    {
        if (g_code.sample[index].raw < g_code.minimum)
        {
            g_code.minimum = g_code.sample[index].raw;
        }
        if (g_code.sample[index].raw > g_code.maximum)
        {
            g_code.maximum = g_code.sample[index].raw;
            peak = index;
        }
    }
    bench_text(" BARCODE: ");
    bench_dec((int32_t)g_code.count);
    bench_text(" side sensor samples over ");
    bench_code_unit_mm(g_code.sample[g_code.count - 1U].unit);
    bench_text(" mm, raw min ");
    bench_dec((int32_t)g_code.minimum);
    bench_text(" max ");
    bench_dec((int32_t)g_code.maximum);
    bench_text(" at ");
    bench_code_unit_mm(g_code.sample[peak].unit);
    bench_text(" mm; robot DARK ");
    bench_text(g_code.robot.b_dark_high ? ">= " : "<= ");
    bench_dec((int32_t)g_code.robot.on_adc);
    bench_text(", LIGHT ");
    bench_text(g_code.robot.b_dark_high ? "<= " : ">= ");
    bench_dec((int32_t)g_code.robot.off_adc);
    if (g_code.b_full)
    {
        bench_text(", STORE FULL");
    }
    bench_flush();
    if (g_code.readings > 1U)
    {
        gap_us = (uint32_t)((g_code.last_us - g_code.first_us) /
                            (uint64_t)(g_code.readings - 1U));
    }
    bench_text(" each sample the mean of its unit's readings: ");
    bench_dec((int32_t)g_code.readings);
    bench_text(" readings, one per ");
    bench_fixed((int32_t)(gap_us / 10U), 2U);
    bench_text(" ms on average (1 ms between steps)");
    bench_flush();
    groups = bench_code_pass(&g_code.robot, &b_decoded);
    if ((0U == groups) &&
        (g_code.maximum < g_code.robot.off_adc))
    {
        bench_text(" NO BAR READ: never above LIGHT; the bars read white:"
                   " an IR-invisible print (dye ink), or the sensor ran"
                   " beside the bars");
        bench_flush();
    }
    else if (0U == groups)
    {
        bench_text(" NO BAR READ: the bars peak above LIGHT but below DARK:"
                   " too faint for the robot's thresholds");
        bench_flush();
    }
    else
    {
        /* The groups are shown. */
    }
    /* Contrast the robot's thresholds missed: decode with this run's own,
     * to show what the code says and the thresholds it would need.
     */
    if ((!b_decoded) &&
        (((uint32_t)g_code.maximum - (uint32_t)g_code.minimum) >=
         BENCH_CODE_CONTRAST) &&
        line_ctrl_hysteresis(g_code.minimum, g_code.maximum,
                             BENCH_CODE_OWN_ON, BENCH_CODE_OWN_OFF, &own))
    {
        bench_text(" again with this run's own DARK >= ");
        bench_dec((int32_t)own.on_adc);
        bench_text(", LIGHT <= ");
        bench_dec((int32_t)own.off_adc);
        bench_text(" (55%/40% of raw min-max; the robot uses 75%/55% of"
                   " the tape):");
        bench_flush();
        (void)bench_code_pass(&own, &b_decoded);
    }
    bench_code_card();
    bench_code_dark(false, 2U * BENCH_CODE_DARK_SDS, BENCH_CODE_GROUP_GAP_MM);
    bench_code_trace();
}

/** @brief One pass over the samples with a hysteresis: each dark group's
 * widths and Code 39 decode. The number of dark groups; *p_decoded is set
 * when a group decoded as a Code 39 frame.
 */
static uint32_t bench_code_pass (const line_ctrl_hysteresis_t * p_hysteresis,
                                 bool * p_decoded)
{
    uint32_t index = 0U;
    uint32_t first = 0U;
    uint32_t last = 0U;
    uint32_t groups = 0U;
    uint32_t gap_units = (uint32_t)((BENCH_CODE_GROUP_GAP_MM / 1000.0) /
                                    g_code.unit_m);

    bench_code_runs(p_hysteresis);
    while (index < g_code.runs)
    {
        if (!g_code.run[index].b_black)
        {
            ++index;
            continue;
        }
        /* A dark group: bars joined by spaces narrower than the gap. */
        first = index;
        last = index;
        while (((last + 2U) < g_code.runs) &&
               (g_code.run[last + 1U].units < gap_units) &&
               g_code.run[last + 2U].b_black)
        {
            last += 2U;
        }
        ++groups;
        if (groups <= BENCH_CODE_GROUPS)
        {
            bench_text(" group ");
            bench_dec((int32_t)groups);
            if (bench_code_group(first, last))
            {
                *p_decoded = true;
            }
        }
        index = last + 1U;
    }
    if (groups > BENCH_CODE_GROUPS)
    {
        bench_text(" ");
        bench_dec((int32_t)(groups - BENCH_CODE_GROUPS));
        bench_text(" more dark groups not shown");
        bench_flush();
    }
    if (g_code.b_runs_full)
    {
        bench_text(" RUN STORE FULL: runs after the first ");
        bench_dec((int32_t)BENCH_CODE_RUNS);
        bench_text(" are not shown");
        bench_flush();
    }
    return groups;
}

/** @brief Runs of a hysteresis class; BAND keeps the last class. A run's
 * width ends where the next run starts (or one unit past the last sample).
 */
static void bench_code_runs (const line_ctrl_hysteresis_t * p_hysteresis)
{
    bench_code_run_t * p_run = NULL;
    line_ctrl_class_t  darkness;
    uint32_t           index;
    uint32_t           end;
    bool               b_black = false;

    g_code.runs = 0U;
    g_code.b_runs_full = false;
    for (index = 0U; index < g_code.count; ++index)
    {
        darkness = line_ctrl_classify(p_hysteresis,
                                      g_code.sample[index].raw);
        if (LINE_CTRL_DARK == darkness)
        {
            b_black = true;
        }
        else if (LINE_CTRL_LIGHT == darkness)
        {
            b_black = false;
        }
        else
        {
            /* BAND or INVALID: keep the last class. */
        }
        if ((NULL == p_run) || (b_black != p_run->b_black))
        {
            if (g_code.runs >= BENCH_CODE_RUNS)
            {
                g_code.b_runs_full = true;
                break;
            }
            p_run = &g_code.run[g_code.runs];
            ++g_code.runs;
            p_run->b_black = b_black;
            p_run->start = (uint16_t)index;
            p_run->peak = g_code.sample[index].raw;
        }
        else if (b_black ? (g_code.sample[index].raw > p_run->peak) :
                           (g_code.sample[index].raw < p_run->peak))
        {
            p_run->peak = g_code.sample[index].raw;
        }
        else
        {
            /* Not a new peak. */
        }
    }
    for (index = 0U; index < g_code.runs; ++index)
    {
        p_run = &g_code.run[index];
        end = ((index + 1U) < g_code.runs) ?
              g_code.sample[g_code.run[index + 1U].start].unit :
              (g_code.sample[g_code.count - 1U].unit + 1U);
        p_run->units = (uint16_t)(end - g_code.sample[p_run->start].unit);
    }
}

/** @brief Third pass, on the card's own levels. Bars are the side sensor's
 * peaks at least BENCH_CODE_FLOOR of the calibrated span above white, other
 * than tape-dark spans of a line's length. Each edge is where the reading
 * crosses halfway between the bar's peak and the valley of the space beside
 * it, so a print lighter than the tape, or a bar narrower than the sensor's
 * spot, still gives its edges. Bars less than BENCH_CODE_GROUP_GAP_MM apart
 * form a group, decoded as Code 39.
 */
static void bench_code_card (void)
{
    uint32_t span;
    uint32_t floor_adc;
    uint32_t noise;
    uint32_t swing;
    uint32_t darkest = 0U;
    uint32_t index;
    uint32_t point;
    uint32_t before;
    uint32_t after;
    uint32_t start = 0U;
    uint32_t end = 0U;
    uint32_t runs = 0U;
    uint32_t groups = 0U;
    uint32_t space_raw = 0xFFFFU;
    double   rise;
    double   fall;
    double   last_fall = 0.0;
    double   gap_units = (BENCH_CODE_GROUP_GAP_MM / 1000.0) / g_code.unit_m;
    bool     b_decoded = false;

    if ((!g_code.robot.b_dark_high) || (g_code.black <= g_code.white))
    {
        bench_text(" card levels: not run, the side sensor reads dark low");
        bench_flush();
        return;
    }
    span = (uint32_t)g_code.black - (uint32_t)g_code.white;
    floor_adc = (uint32_t)g_code.white +
                (uint32_t)bench_round((double)span * BENCH_CODE_FLOOR);
    noise = (uint32_t)bench_round((double)span * BENCH_CODE_NOISE);
    bench_code_card_lines();
    for (index = 0U; index < g_code.count; ++index)
    {
        if ((!bench_code_in_line(index)) &&
            (g_code.sample[index].raw > darkest))
        {
            darkest = g_code.sample[index].raw;
        }
    }
    swing = (darkest > g_code.minimum) ?
            (uint32_t)bench_round((double)(darkest - g_code.minimum) *
                                  BENCH_CODE_SWING) : 0U;
    if (swing < noise)
    {
        swing = noise;
    }
    bench_code_card_extrema((uint16_t)swing);
    bench_text(" card levels: bars peak >= ");
    bench_dec((int32_t)floor_adc);
    bench_text(", darkest off the line ");
    bench_dec((int32_t)darkest);
    bench_text(", bar-space swing >= ");
    bench_dec((int32_t)swing);
    bench_text(", tape-dark lines ");
    bench_dec((int32_t)g_code.lines);
    bench_text(", turning points ");
    bench_dec((int32_t)g_code.extremes);
    if (g_code.extremes >= BENCH_CODE_EXTREMA)
    {
        bench_text(" (FULL)");
    }
    bench_flush();
    if (darkest < floor_adc)
    {
        bench_text(" card levels: no bar, nothing off the line peaks"
                   " above the floor");
        bench_flush();
        return;
    }
    /* Peaks are the odd turning points; each needs a valley after it. */
    for (point = 1U; (point + 1U) < g_code.extremes; point += 2U)
    {
        index = g_code.extreme[point];
        before = g_code.extreme[point - 1U];
        after = g_code.extreme[point + 1U];
        if (g_code.sample[before].raw < space_raw)
        {
            space_raw = g_code.sample[before].raw;
        }
        if (bench_code_in_line(index) ||
            (g_code.sample[index].raw < floor_adc))
        {
            if (bench_code_in_line(index) && (0U != runs))
            {
                /* A line ends the group. */
                ++groups;
                bench_code_card_group(groups, runs, &b_decoded);
                runs = 0U;
            }
            continue;
        }
        rise = bench_code_edge(index, before,
                               0.5 * ((double)g_code.sample[before].raw +
                                      (double)g_code.sample[index].raw),
                               &start);
        fall = bench_code_edge(index, after,
                               0.5 * ((double)g_code.sample[index].raw +
                                      (double)g_code.sample[after].raw),
                               &end);
        if ((0U != runs) && ((rise - last_fall) >= gap_units))
        {
            ++groups;
            bench_code_card_group(groups, runs, &b_decoded);
            runs = 0U;
        }
        if ((0U != runs) && ((runs + 2U) <= BENCH_CODE_RUNS))
        {
            g_code.run[runs - 1U].units =
                (uint16_t)bench_round(fmax(0.0, rise - last_fall));
            g_code.run[runs - 1U].peak = (uint16_t)space_raw;
        }
        if ((runs + 2U) <= BENCH_CODE_RUNS)
        {
            g_code.run[runs].b_black = true;
            g_code.run[runs].start = (uint16_t)start;
            g_code.run[runs].units =
                (uint16_t)bench_round(fmax(0.0, fall - rise));
            g_code.run[runs].peak = g_code.sample[index].raw;
            /* The space after it, closed by the next bar. */
            g_code.run[runs + 1U].b_black = false;
            g_code.run[runs + 1U].start = (uint16_t)end;
            runs += 2U;
        }
        last_fall = fall;
        space_raw = 0xFFFFU;
    }
    if (0U != runs)
    {
        ++groups;
        bench_code_card_group(groups, runs, &b_decoded);
    }
    if (0U == groups)
    {
        bench_text(" card levels: no bar with a space either side");
        bench_flush();
    }
    else if (groups > BENCH_CODE_GROUPS)
    {
        bench_text(" ");
        bench_dec((int32_t)(groups - BENCH_CODE_GROUPS));
        bench_text(" more card groups not shown");
        bench_flush();
    }
    else
    {
        /* The groups are shown. */
    }
}

/** @brief Fourth pass, darkness, on the run's own white and noise
 * (bench_code_white). Bars are the peaks at least the swing above white,
 * other than tape-dark lines; edges are halfway to the valleys either side,
 * as in the card pass. Each bar also keeps its darkness: the readings above
 * the higher of its two valleys, summed across it (bench_code_darkness);
 * each space how far it reads white, below the bars' floor
 * (bench_code_whiteness): blur from two dark bars widens a narrow space at
 * halfway but keeps it off white. Groups as in the card pass, decoded on
 * those two (bench_code39_dark). Turning points need half_sds half noise
 * sds; gap_mm of white ends a group. Quiet (MISSION), white and noise are
 * the cut's (bench_code_dark_tries).
 */
static void bench_code_dark (bool b_quiet, uint32_t half_sds, double gap_mm)
{
    uint32_t sd = 1U;
    uint32_t white;
    uint32_t swing;
    uint32_t floor_adc;
    uint32_t index;
    uint32_t point;
    uint32_t before;
    uint32_t after;
    uint32_t start = 0U;
    uint32_t end = 0U;
    uint32_t runs = 0U;
    uint32_t groups = 0U;
    uint32_t space_raw = 0xFFFFU;
    double   rise;
    double   fall;
    double   last_fall = 0.0;
    double   gap_units = (gap_mm / 1000.0) / g_code.unit_m;
    double   lead = -1.0;      /* white before the group; < 0: no bar */
    bool     b_decoded = false;
    bool     b_bar_before = false;  /* since the cut's start or a line */
    uint32_t last_peak = 0U;

    if (!g_code.robot.b_dark_high)
    {
        if (!b_quiet)
        {
            bench_text(" darkness: not run, the side sensor reads dark low");
            bench_flush();
        }
        return;
    }
    if (b_quiet)
    {
        white = g_code.cut_white;
        sd = g_code.cut_sd;
    }
    else
    {
        white = bench_code_white(&sd);
    }
    swing = ((half_sds * sd) + 1U) / 2U;
    floor_adc = white + swing;
    bench_code_card_extrema((uint16_t)swing);
    if (!b_quiet)
    {
        bench_text(" darkness: white ");
        bench_dec((int32_t)white);
        bench_text(" (commonest), noise sd ");
        bench_dec((int32_t)sd);
        bench_text(" (from below white), bars peak >= ");
        bench_dec((int32_t)floor_adc);
        bench_text(", swing >= ");
        bench_dec((int32_t)swing);
        bench_text(", turning points ");
        bench_dec((int32_t)g_code.extremes);
        if (g_code.extremes >= BENCH_CODE_EXTREMA)
        {
            bench_text(" (FULL)");
        }
        bench_flush();
    }
    g_code.b_dark_rule = true;
    /* Peaks are the odd turning points; each needs a valley after it. */
    for (point = 1U; (point + 1U) < g_code.extremes; point += 2U)
    {
        index = g_code.extreme[point];
        before = g_code.extreme[point - 1U];
        after = g_code.extreme[point + 1U];
        if (g_code.sample[before].raw < space_raw)
        {
            space_raw = g_code.sample[before].raw;
        }
        if (bench_code_in_line(index) ||
            (g_code.sample[index].raw < floor_adc))
        {
            if (bench_code_in_line(index) && (0U != runs))
            {
                /* A line ends the group. */
                ++groups;
                g_code.code_lead = lead;
                g_code.code_trail = -1.0;
                bench_code_dark_end(groups, runs, &b_decoded, b_quiet);
                runs = 0U;
            }
            if (bench_code_in_line(index))
            {
                b_bar_before = false;
            }
            continue;
        }
        rise = bench_code_edge(index, before,
                               0.5 * ((double)g_code.sample[before].raw +
                                      (double)g_code.sample[index].raw),
                               &start);
        fall = bench_code_edge(index, after,
                               0.5 * ((double)g_code.sample[index].raw +
                                      (double)g_code.sample[after].raw),
                               &end);
        if ((0U != runs) && ((rise - last_fall) >= gap_units))
        {
            ++groups;
            g_code.code_lead = lead;
            g_code.code_trail = rise - last_fall;
            bench_code_dark_end(groups, runs, &b_decoded, b_quiet);
            runs = 0U;
        }
        if (0U == runs)
        {
            lead = b_bar_before ? (rise - last_fall) : -1.0;
        }
        if ((0U != runs) && ((runs + 2U) <= BENCH_CODE_RUNS))
        {
            g_code.run[runs - 1U].units =
                (uint16_t)bench_round(fmax(0.0, rise - last_fall));
            g_code.run[runs - 1U].peak = (uint16_t)space_raw;
            g_code.run[runs - 1U].dark =
                bench_code_whiteness(last_peak, index, floor_adc);
        }
        if ((runs + 2U) <= BENCH_CODE_RUNS)
        {
            g_code.run[runs].b_black = true;
            g_code.run[runs].start = (uint16_t)start;
            g_code.run[runs].units =
                (uint16_t)bench_round(fmax(0.0, fall - rise));
            g_code.run[runs].peak = g_code.sample[index].raw;
            g_code.run[runs].dark = bench_code_darkness(before, after);
            /* The space after it, closed by the next bar. */
            g_code.run[runs + 1U].b_black = false;
            g_code.run[runs + 1U].start = (uint16_t)end;
            g_code.run[runs + 1U].dark = 0U;
            runs += 2U;
        }
        last_fall = fall;
        last_peak = index;
        space_raw = 0xFFFFU;
        b_bar_before = true;
    }
    if (0U != runs)
    {
        ++groups;
        g_code.code_lead = lead;
        g_code.code_trail = -1.0;
        bench_code_dark_end(groups, runs, &b_decoded, b_quiet);
    }
    g_code.b_dark_rule = false;
    if (b_quiet)
    {
        /* MISSION: the groups went to bench_code_found_add. */
    }
    else if (0U == groups)
    {
        bench_text(" darkness: no bar with a space either side");
        bench_flush();
    }
    else if (groups > BENCH_CODE_GROUPS)
    {
        bench_text(" ");
        bench_dec((int32_t)(groups - BENCH_CODE_GROUPS));
        bench_text(" more darkness groups not shown");
        bench_flush();
    }
    else
    {
        /* The groups are shown. */
    }
}

/** @brief A darkness group's end: shown and decoded (bench_code_card_group),
 * or, quiet (MISSION), kept by bench_code_found_add.
 */
static void bench_code_dark_end (uint32_t group, uint32_t runs,
                                 bool * p_decoded, bool b_quiet)
{
    if (b_quiet)
    {
        bench_code_found_add(runs);
    }
    else
    {
        bench_code_card_group(group, runs, p_decoded);
    }
}

/** @brief MISSION, a cut (bench_code_stream): its samples frozen at
 * cut_count (later ones are stored after them), tape lines, white and noise
 * found once; its tries then run one per line step (bench_code_tries_step),
 * so the drive loop waits for one darkness pass at most (all of them held
 * it 37 ms on 30 Oct and the encoders faulted).
 */
static void bench_code_tries_begin (uint32_t gap_units)
{
    g_code.b_trying = true;
    g_code.try_next = 0U;
    g_code.try_most = 0U;
    g_code.try_best = 0U;
    g_code.try_founds = g_code.founds;
    g_code.try_seen = g_code.groups_seen;
    g_code.b_agreed = false;
    g_code.b_partial_ok = false;
    g_code.b_peak_agreed = false;
    g_code.try_rule = BENCH_CODE_RULE_DARK;
    g_code.sigs = 0U;
    g_code.cut_count = g_code.count;
    g_code.cut_gap = gap_units;
    bench_code_card_lines();
    g_code.cut_white = bench_code_white(&g_code.cut_sd);
}

/** @brief MISSION, one darkness pass of a cut, quiet: try index's rule
 * (BENCH_CODE_TRIES darkness tries, then as many peak tries), group gap
 * (BENCH_CODE_QUIET_MM, then BENCH_CODE_GROUP_GAP_MM) and swing, in half
 * sds: BENCH_CODE_DARK_SDS first, then outward from it within
 * BENCH_CODE_TRY_MIN..BENCH_CODE_TRY_MAX.
 */
static void bench_code_try (uint32_t index)
{
    static const double gaps_mm[BENCH_CODE_GAPS] =
    {
        BENCH_CODE_QUIET_MM, BENCH_CODE_GROUP_GAP_MM
    };
    static const uint8_t half_sds[BENCH_CODE_SWINGS] =
    {
        8U, 9U, 7U, 10U, 6U, 11U, 12U
    };
    uint32_t local = index % BENCH_CODE_TRIES;

    g_code.try_rule = (index < BENCH_CODE_TRIES) ? BENCH_CODE_RULE_DARK :
                                                   BENCH_CODE_RULE_PEAKS;
    g_code.try_half_sds = half_sds[local % BENCH_CODE_SWINGS];
    g_code.try_gap_mm = (uint32_t)bench_round(
                            gaps_mm[local / BENCH_CODE_SWINGS]);
    bench_code_dark(true, g_code.try_half_sds,
                    gaps_mm[local / BENCH_CODE_SWINGS]);
    g_code.try_rule = BENCH_CODE_RULE_DARK;
}

/** @brief MISSION, after a try: its framed texts end to end (as read, or
 * reversed with a < after), counted as the vote of its rule and swing for
 * that text; true once BENCH_CODE_AGREE different votes framed the same
 * text, one of them darkness, the first of them then in try_best (a
 * darkness try: the peak tries run after them all).
 */
static bool bench_code_try_agree (uint32_t index)
{
    char               text[BENCH_CODE_SIG];
    bench_code_sig_t * p_sig = NULL;
    uint32_t           found;
    uint32_t           sig;
    uint32_t           swings = 0U;
    uint32_t           bit;
    size_t             length = 0U;

    text[0] = '\0';
    for (found = g_code.try_founds; found < g_code.founds; ++found)
    {
        if (g_code.found[found].b_framed)
        {
            (void)strncat(text, g_code.found[found].b_reverse ?
                                g_code.found[found].reverse :
                                g_code.found[found].text,
                          (BENCH_CODE_SIG - 1U) - length);
            length = strlen(text);
            (void)strncat(text, g_code.found[found].b_reverse ? "<|" : "|",
                          (BENCH_CODE_SIG - 1U) - length);
            length = strlen(text);
        }
    }
    if (0U == length)
    {
        return false;
    }
    for (sig = 0U; (sig < g_code.sigs) && (NULL == p_sig); ++sig)
    {
        if (0 == strcmp(g_code.sig[sig].text, text))
        {
            p_sig = &g_code.sig[sig];
        }
    }
    if (NULL == p_sig)
    {
        p_sig = &g_code.sig[g_code.sigs];
        ++g_code.sigs;
        (void)memcpy(p_sig->text, text, sizeof(text));
        p_sig->swings = 0U;
        p_sig->first = index;
    }
    p_sig->swings |= (uint32_t)1U <<
                     (((index / BENCH_CODE_TRIES) * BENCH_CODE_SWINGS) +
                      (index % BENCH_CODE_SWINGS));
    for (bit = 0U; bit < (BENCH_CODE_RULES * BENCH_CODE_SWINGS); ++bit)
    {
        swings += (p_sig->swings >> bit) & 1U;
    }
    if ((swings < BENCH_CODE_AGREE) ||
        (0U == (p_sig->swings & BENCH_CODE_DARK_VOTES)))
    {
        return false;
    }
    g_code.try_best = p_sig->first;
    g_code.b_peak_agreed = (0U != (p_sig->swings >> BENCH_CODE_SWINGS));
    return true;
}

/** @brief MISSION, from the line step: one step of a cut's decode, on its
 * frozen samples. The next try (bench_code_try), its groups dropped after
 * counting the framed ones and their texts' votes (bench_code_try_agree);
 * once BENCH_CODE_AGREE votes agree, or all tries are done, the agreed try
 * (else the first darkness try to frame the most) runs again to keep its
 * groups (bench_code_found_add), framed ones without agreement kept as
 * framed at one swing only, not codes, and the store is cut back
 * (bench_code_keep). The peak tries run only when the darkness tries framed
 * a text without agreeing: alone they can make no code.
 */
static void bench_code_tries_step (void)
{
    uint32_t count = g_code.count;
    uint32_t framed = 0U;
    uint32_t index;

    bench_fit_run();   /* TRIAL: the fit, a slice of each step */
    if (!g_code.b_trying)
    {
        return;
    }
    g_code.count = g_code.cut_count;
    if (g_code.try_next < BENCH_CODE_ALL_TRIES)
    {
        bench_code_try(g_code.try_next);
        for (index = g_code.try_founds; index < g_code.founds; ++index)
        {
            if (g_code.found[index].b_framed)
            {
                ++framed;
            }
        }
        if ((framed > g_code.try_most) &&
            (g_code.try_next < BENCH_CODE_TRIES))
        {
            g_code.try_most = framed;
            g_code.try_best = g_code.try_next;
        }
        if (bench_code_try_agree(g_code.try_next))
        {
            g_code.b_agreed = true;
            g_code.try_next = BENCH_CODE_ALL_TRIES;
        }
        else
        {
            ++g_code.try_next;
            if ((BENCH_CODE_TRIES == g_code.try_next) && (0U == g_code.sigs))
            {
                g_code.try_next = BENCH_CODE_ALL_TRIES;
            }
        }
        if ((0 != BENCH_CODE_PARTIAL) &&
            (BENCH_CODE_ALL_TRIES == g_code.try_next) && (!g_code.b_agreed) &&
            (!g_code.b_partial_ok))
        {
            /* No whole frame agreed: the tries again, one * now framing. */
            g_code.b_partial_ok = true;
            g_code.try_next = 0U;
            g_code.try_most = 0U;
            g_code.try_best = 0U;
            g_code.sigs = 0U;
        }
        g_code.founds = g_code.try_founds;
        g_code.groups_seen = g_code.try_seen;
        g_code.count = count;
        return;
    }
    bench_code_try(g_code.try_best);
    for (index = g_code.try_founds; index < g_code.founds; ++index)
    {
        if (g_code.found[index].b_framed && (!g_code.b_agreed))
        {
            g_code.found[index].b_framed = false;
            g_code.found[index].b_single = true;
        }
        g_code.found[index].b_peaks = g_code.found[index].b_framed &&
                                      g_code.b_peak_agreed;
    }
    if (g_code.groups_seen != g_code.try_seen)
    {
        bench_raw_keep();
    }
    g_code.count = count;
    bench_fit_begin(); /* TRIAL: the cut binned for the fit, before the keep */
    bench_code_keep(g_code.cut_gap);
    g_code.b_open = false;
    g_code.b_trying = false;
    g_code.b_tape = false;
    g_code.b_bar = false;
}

/** @brief MISSION: true while the side sensor crosses a card, from its
 * first mark until the group is cut, BENCH_CODE_QUIET_MM past the last.
 * FOLLOW then drives straight, so the bars cross the sensor square and the
 * front sensors (ROBOT_LINE_SENSOR_FORWARD_M; from 2 Nov 2026 ahead of the
 * side sensor, ROBOT_BARCODE_FORWARD_OFFSET_M) do not steer on the card's
 * bars. It starts at the first printed bar (b_bar), not
 * at noise; tape-long dark (b_tape) is a line, not a card: no hold, so
 * bends are still followed.
 */
static bool bench_code_hold (void)
{
    return g_code.b_stream && g_code.b_open && g_code.b_bar &&
           (!g_code.b_trying) && (!g_code.b_tape);
}

/** @brief MISSION: true when holding straight over a card would take the car
 * off the line, by LINE smooth's own tests (bench_centre_steer): a sustained
 * bend (g_centre.turn, BENCH_SMOOTH_BEND), a search, or a front sensor
 * naming the line's side (seen_side, BENCH_CENTRE_SIDE), i.e. the line has
 * drifted off-centre. Both front sensors white or about as dark is the line
 * centred: the hold stays. On 2 Nov the hold zeroed the steering in bends and
 * searches (5 holds, about 330 mm, in 0.9 m runs) and the car lost the line.
 */
static bool bench_hold_lets_go (void)
{
    return (fabs(g_centre.turn) >= BENCH_SMOOTH_BEND) ||
           ((uint8_t)BENCH_PHASE_SEARCH == g_centre.phase) ||
           (((uint8_t)BENCH_PHASE_TRACK == g_centre.phase) &&
            (0 != g_centre.seen_side));
}

/** @brief MISSION, after the stop: a cut being decoded, or the open group,
 * decoded to the end at once.
 */
static void bench_code_dark_tries (void)
{
    if (!g_code.b_trying)
    {
        bench_code_tries_begin((uint32_t)bench_round(
            (BENCH_CODE_QUIET_MM / 1000.0) / g_code.unit_m));
    }
    while (g_code.b_trying)
    {
        bench_code_tries_step();
    }
}

/** @brief The run's white: its commonest stored side sensor reading
 * (g_code_levels, every sample since the start). *p_sd is that white's
 * noise, one standard deviation, from the readings below it alone: white
 * less the level BENCH_CODE_ONE_SD_SHARE of the way up them (at least 1).
 */
static uint32_t bench_code_white (uint32_t * p_sd)
{
    uint32_t level;
    uint32_t white = 0U;
    uint32_t below = 0U;
    uint32_t counted = 0U;
    double   target;

    for (level = 1U; level < BENCH_CODE_ADC_LEVELS; ++level)
    {
        if (g_code_levels[level] > g_code_levels[white])
        {
            white = level;
        }
    }
    for (level = 0U; level < white; ++level)
    {
        below += g_code_levels[level];
    }
    target = BENCH_CODE_ONE_SD_SHARE *
             ((double)below + (0.5 * (double)g_code_levels[white]));
    level = 0U;
    while ((level < white) &&
           ((double)(counted + g_code_levels[level]) < target))
    {
        counted += g_code_levels[level];
        ++level;
    }
    *p_sd = (white > level) ? (white - level) : 1U;
    return white;
}

/** @brief A bar's darkness: from valley before to valley after, the
 * readings above the higher of the two, times the units each spans.
 */
static uint32_t bench_code_darkness (uint32_t before, uint32_t after)
{
    uint32_t base = g_code.sample[before].raw;
    uint32_t dark = 0U;
    uint32_t index;

    if (g_code.sample[after].raw > base)
    {
        base = g_code.sample[after].raw;
    }
    for (index = before; index < after; ++index)
    {
        if (g_code.sample[index].raw > base)
        {
            dark += ((uint32_t)g_code.sample[index].raw - base) *
                    ((uint32_t)g_code.sample[index + 1U].unit -
                     (uint32_t)g_code.sample[index].unit);
        }
    }
    return dark;
}

/** @brief A space's whiteness: the units from bar peak from to bar peak to
 * that read below floor_adc.
 */
static uint32_t bench_code_whiteness (uint32_t from, uint32_t to,
                                      uint32_t floor_adc)
{
    uint32_t white = 0U;
    uint32_t index;

    for (index = from; index < to; ++index)
    {
        if ((uint32_t)g_code.sample[index].raw < floor_adc)
        {
            white += (uint32_t)g_code.sample[index + 1U].unit -
                     (uint32_t)g_code.sample[index].unit;
        }
    }
    return white;
}

/** @brief Darkness pass, in rows: each bar's darkness, d in raw x mm, and
 * each space's whiteness, w in mm.
 */
static void bench_code_dark_row (uint32_t first, uint32_t last)
{
    uint32_t index;
    uint32_t column = 0U;

    bench_text("  darkness:");
    for (index = first; index <= last; ++index)
    {
        if (g_code.run[index].b_black)
        {
            bench_text(" d");
            bench_dec(bench_round((double)g_code.run[index].dark *
                                  g_code.unit_m * 1000.0));
        }
        else
        {
            bench_text(" w");
            bench_fixed(bench_round((double)g_code.run[index].dark *
                                    g_code.unit_m * 10000.0), 1U);
        }
        ++column;
        if ((BENCH_CODE_DARK_PER_ROW == column) && (index < last))
        {
            bench_flush();
            bench_text("  ");
            column = 0U;
        }
    }
    bench_flush();
}

/** @brief A card group of runs bar, space, ... bar, space (the last space
 * open): shown and decoded as runs 0..runs - 2.
 */
static void bench_code_card_group (uint32_t group, uint32_t runs,
                                   bool * p_decoded)
{
    if (group > BENCH_CODE_GROUPS)
    {
        return;
    }
    bench_text(g_code.b_dark_rule ? " darkness group " : " card group ");
    bench_dec((int32_t)group);
    if (bench_code_group(0U, runs - 2U))
    {
        *p_decoded = true;
    }
}

/** @brief Tape-dark spans (the robot's DARK) at least BENCH_CODE_LINE_M of
 * sensor path long: a line under the side sensor, not a bar.
 */
static void bench_code_card_lines (void)
{
    uint32_t index;
    uint32_t first = 0U;
    bool     b_dark = false;
    bool     b_now;

    g_code.lines = 0U;
    for (index = 0U; index <= g_code.count; ++index)
    {
        b_now = (index < g_code.count) &&
                (g_code.sample[index].raw >= g_code.robot.on_adc);
        if (b_now && (!b_dark))
        {
            first = index;
        }
        else if ((!b_now) && b_dark &&
                 (((double)(g_code.sample[index - 1U].unit -
                            g_code.sample[first].unit) * g_code.unit_m) >=
                  BENCH_CODE_LINE_M) &&
                 (g_code.lines < BENCH_CODE_LINES))
        {
            g_code.line_first[g_code.lines] = (uint16_t)first;
            g_code.line_last[g_code.lines] = (uint16_t)(index - 1U);
            ++g_code.lines;
        }
        else
        {
            /* Inside a span, or between. */
        }
        b_dark = b_now;
    }
}

/** @brief True if sample index lies in a tape-dark line span. */
static bool bench_code_in_line (uint32_t index)
{
    uint32_t line;

    for (line = 0U; line < g_code.lines; ++line)
    {
        if ((index >= g_code.line_first[line]) &&
            (index <= g_code.line_last[line]))
        {
            return true;
        }
    }
    return false;
}

/** @brief Turning points of the side sensor, a valley first: a valley ends
 * when the reading rises swing above it, a peak when it falls swing below.
 */
static void bench_code_card_extrema (uint16_t swing)
{
    uint32_t index;
    uint32_t turn = 0U;
    uint32_t raw;
    uint32_t held;
    bool     b_peak = false;

    g_code.extremes = 0U;
    for (index = 1U; index < g_code.count; ++index)
    {
        raw = g_code.sample[index].raw;
        held = g_code.sample[turn].raw;
        if (b_peak ? (raw > held) : (raw < held))
        {
            turn = index;
        }
        else if (b_peak ? ((raw + swing) <= held) : (raw >= (held + swing)))
        {
            if (g_code.extremes >= BENCH_CODE_EXTREMA)
            {
                return;
            }
            g_code.extreme[g_code.extremes] = (uint16_t)turn;
            ++g_code.extremes;
            turn = index;
            b_peak = !b_peak;
        }
        else
        {
            /* Within swing of the held turning point. */
        }
    }
    if ((0U != g_code.count) && (g_code.extremes < BENCH_CODE_EXTREMA))
    {
        g_code.extreme[g_code.extremes] = (uint16_t)turn;
        ++g_code.extremes;
    }
}

/** @brief From a peak toward a valley, where the reading first falls below
 * level, in decoder units interpolated between the two samples either side;
 * *p_index is the sample on the valley's side of it.
 */
static double bench_code_edge (uint32_t peak, uint32_t valley, double level,
                               uint32_t * p_index)
{
    uint32_t index = peak;
    uint32_t inner = peak;
    double   inner_raw;
    double   outer_raw;
    double   share;

    while (index != valley)
    {
        inner = index;
        index = (valley > peak) ? (index + 1U) : (index - 1U);
        if ((double)g_code.sample[index].raw < level)
        {
            break;
        }
    }
    *p_index = index;
    inner_raw = (double)g_code.sample[inner].raw;
    outer_raw = (double)g_code.sample[index].raw;
    share = (inner_raw > outer_raw) ?
            ((inner_raw - level) / (inner_raw - outer_raw)) : 0.0;
    return (double)g_code.sample[inner].unit +
           (share * ((double)g_code.sample[index].unit -
                     (double)g_code.sample[inner].unit));
}

/** @brief One dark group (runs first..last, bars at both ends): where, how
 * many bars, the weakest bar and space, widths and the Code 39 decode.
 */
static bool bench_code_group (uint32_t first, uint32_t last)
{
    uint32_t index;
    uint32_t weakest_bar = 0xFFFFU;
    uint32_t weakest_space = 0U;

    for (index = first; index <= last; ++index)
    {
        if (g_code.run[index].b_black &&
            (g_code.run[index].peak < weakest_bar))
        {
            weakest_bar = g_code.run[index].peak;
        }
        if ((!g_code.run[index].b_black) &&
            (g_code.run[index].peak > weakest_space))
        {
            weakest_space = g_code.run[index].peak;
        }
    }
    bench_text(" at ");
    bench_code_unit_mm(g_code.sample[g_code.run[first].start].unit);
    bench_text(" to ");
    bench_code_unit_mm(g_code.sample[g_code.run[last].start].unit +
                       g_code.run[last].units);
    bench_text(" mm: ");
    bench_dec((int32_t)(((last - first) / 2U) + 1U));
    bench_text(" bars, ");
    bench_dec((int32_t)((last - first) + 1U));
    bench_text(" runs; weakest bar peak raw ");
    bench_dec((int32_t)weakest_bar);
    if (last > first)
    {
        bench_text(", weakest space raw ");
        bench_dec((int32_t)weakest_space);
    }
    bench_flush();
    bench_code_widths(first, last);
    if (g_code.b_dark_rule)
    {
        bench_code_dark_row(first, last);
    }
    return bench_code39(first, last);
}

/** @brief The group's run widths in mm, b bar and w space, in rows. */
static void bench_code_widths (uint32_t first, uint32_t last)
{
    uint32_t index;
    uint32_t column = 0U;

    bench_text("  widths mm:");
    for (index = first; index <= last; ++index)
    {
        bench_text(g_code.run[index].b_black ? " b" : " w");
        bench_fixed(bench_round((double)g_code.run[index].units *
                                g_code.unit_m * 10000.0), 1U);
        ++column;
        if ((BENCH_CODE_RUNS_PER_ROW == column) && (index < last))
        {
            bench_flush();
            bench_text("  ");
            column = 0U;
        }
    }
    bench_flush();
}

/** @brief Code 39 decode of a group, as read, else reversed (the car may
 * have passed the code end to start); a frame must start and end with *,
 * with a character between.
 */
static bool bench_code39 (uint32_t first, uint32_t last)
{
    char     forward[BENCH_CODE_TEXT];
    char     reverse[BENCH_CODE_TEXT];
    uint32_t runs = (last - first) + 1U;
    bool     b_forward = false;
    bool     b_reverse = false;
    uint32_t length = 0U;

    bench_text("  Code 39: ");
    if ((0U != ((runs + 1U) % (BENCH_CODE_ELEMENTS + 1U))) ||
        (((runs + 1U) / (BENCH_CODE_ELEMENTS + 1U)) >= BENCH_CODE_TEXT))
    {
        bench_text("no, ");
        bench_dec((int32_t)runs);
        bench_text(" runs (a code has 10 x characters - 1, at most ");
        bench_dec((int32_t)(BENCH_CODE_TEXT - 1U));
        bench_text(" characters)");
        bench_flush();
        return false;
    }
    length = (runs + 1U) / (BENCH_CODE_ELEMENTS + 1U);
    b_forward = bench_code39_text(first, last, false, forward) &&
                (length >= BENCH_CODE_FRAMED) &&
                ('*' == forward[0]) && ('*' == forward[length - 1U]);
    if (!b_forward)
    {
        b_reverse = bench_code39_text(first, last, true, reverse) &&
                    (length >= BENCH_CODE_FRAMED) &&
                    ('*' == reverse[0]) && ('*' == reverse[length - 1U]);
        if (!b_reverse)
        {
            bench_text("no *...* frame: as read ");
            bench_text(forward);
            bench_text(", reversed ");
            bench_text(reverse);
            bench_text(" (? not a Code 39 character)");
            bench_flush();
            return false;
        }
    }
    bench_text(b_forward ? forward : reverse);
    bench_text(b_forward ? " read in order" : " read end to start");
    if ((0U != g_code.narrow_count) && (0U != g_code.wide_count))
    {
        bench_text(", narrow mean ");
        bench_fixed(bench_round(((double)g_code.narrow_units /
                                 (double)g_code.narrow_count) *
                                g_code.unit_m * 10000.0), 1U);
        bench_text(" mm, wide mean ");
        bench_fixed(bench_round(((double)g_code.wide_units /
                                 (double)g_code.wide_count) *
                                g_code.unit_m * 10000.0), 1U);
        bench_text(" mm");
    }
    bench_flush();
    return true;
}

/** @brief Darkness pass: one character's pattern, its 2 darkest of 5 bars
 * wide and its whitest of 4 spaces, judged apart so blur that thins bars
 * and widens spaces cannot mix the two. 0 (no character) unless each wide
 * element is clear of every narrow one of its kind. Code 39's $ / + % (3
 * wide spaces, no wide bar) are not read. In MISSION's peak tries
 * (BENCH_CODE_RULE_PEAKS) the bars are judged by their peaks and the spaces
 * by their valleys, the lowest wide, instead.
 */
static uint32_t bench_code39_dark (uint32_t first, uint32_t last,
                                   uint32_t base, bool b_reverse)
{
    uint32_t value[BENCH_CODE_ELEMENTS];
    uint32_t pattern = 0U;
    uint32_t element;
    uint32_t kind;
    uint32_t pick;
    uint32_t widest;
    uint32_t narrowest_wide;
    uint32_t widest_narrow;
    uint32_t run;

    for (element = 0U; element < BENCH_CODE_ELEMENTS; ++element)
    {
        run = b_reverse ? (last - (base + element)) : (first + base + element);
        if (BENCH_CODE_RULE_PEAKS != g_code.try_rule)
        {
            value[element] = g_code.run[run].dark;
        }
        else if (0U == (element % 2U))
        {
            value[element] = (uint32_t)g_code.run[run].peak;
        }
        else
        {
            value[element] = 0xFFFFU - (uint32_t)g_code.run[run].peak;
        }
    }
    /* Kind 0: the bars (even elements), 2 wide; kind 1: the spaces, 1. */
    for (kind = 0U; kind < 2U; ++kind)
    {
        for (pick = 0U; pick < (2U - kind); ++pick)
        {
            widest = BENCH_CODE_ELEMENTS;
            for (element = kind; element < BENCH_CODE_ELEMENTS;
                 element += 2U)
            {
                if ((0U == (pattern & (0x100U >> element))) &&
                    ((BENCH_CODE_ELEMENTS == widest) ||
                     (value[element] > value[widest])))
                {
                    widest = element;
                }
            }
            pattern |= 0x100U >> widest;
        }
        narrowest_wide = UINT32_MAX;
        widest_narrow = 0U;
        for (element = kind; element < BENCH_CODE_ELEMENTS; element += 2U)
        {
            if (0U != (pattern & (0x100U >> element)))
            {
                narrowest_wide = (value[element] < narrowest_wide) ?
                                 value[element] : narrowest_wide;
            }
            else
            {
                widest_narrow = (value[element] > widest_narrow) ?
                                value[element] : widest_narrow;
            }
        }
        if (narrowest_wide <= widest_narrow)
        {
            return 0U;
        }
    }
    return pattern;
}

/** @brief MISSION, the Code 39 spec checks of a frame of length characters
 * from run first of a group ending at run last (BENCH_CODE_CHAR_MIN_X): each
 * character within one narrow width of the frame's median character, each
 * gap between characters at most BENCH_CODE_GAP_MAX_X narrow widths, and a
 * bar before or after it at least BENCH_CODE_QUIET_X narrow widths away.
 * The narrow width is the median character over BENCH_CODE_CHAR_MIN_X for
 * the first two, the widest the spec allows, and over BENCH_CODE_CHAR_MAX_X
 * for the quiet zone, the narrowest: each check as lenient as the spec. A
 * space narrower than that narrowest width is no Code 39 space but a speck
 * in the bar beside it (31 Oct: 0.3 mm splitting *Z*'s first bar), so the
 * quiet zone is measured past it; at the group's ends the group's own
 * (code_lead, code_trail; < 0 when no bar bounds it). BENCH_SPEC_OK, else
 * the check that refused it (counted).
 */
static uint32_t bench_code39_spec (uint32_t first, uint32_t length,
                                   uint32_t last)
{
    uint32_t span[BENCH_CODE_TEXT];
    uint32_t sorted[BENCH_CODE_TEXT];
    uint32_t character;
    uint32_t element;
    uint32_t pick;
    uint32_t swap;
    uint32_t base;
    double   median;
    double   narrow;
    double   narrowest;
    double   quiet;
    double   lead;
    double   trail;
    bool     b_side;
    uint32_t result = BENCH_SPEC_OK;

    if ((0U == length) || (length >= BENCH_CODE_TEXT))
    {
        return BENCH_SPEC_OK;
    }
    for (character = 0U; character < length; ++character)
    {
        base = first + (character * (BENCH_CODE_ELEMENTS + 1U));
        span[character] = 0U;
        for (element = 0U; element < BENCH_CODE_ELEMENTS; ++element)
        {
            span[character] += g_code.run[base + element].units;
        }
        sorted[character] = span[character];
        for (pick = character;
             (pick > 0U) && (sorted[pick - 1U] > sorted[pick]); --pick)
        {
            swap = sorted[pick];
            sorted[pick] = sorted[pick - 1U];
            sorted[pick - 1U] = swap;
        }
    }
    median = (0U != (length % 2U)) ?
             (double)sorted[length / 2U] :
             (0.5 * ((double)sorted[(length / 2U) - 1U] +
                     (double)sorted[length / 2U]));
    narrow = median / BENCH_CODE_CHAR_MIN_X;
    narrowest = median / BENCH_CODE_CHAR_MAX_X;
    quiet = BENCH_CODE_QUIET_X * narrowest;
    /* The white before the frame's first bar, past specks. */
    lead = g_code.code_lead;
    base = first;
    b_side = false;
    while ((0U != base) && (!b_side))
    {
        if ((double)g_code.run[base - 1U].units >= narrowest)
        {
            lead = (double)g_code.run[base - 1U].units;
            b_side = true;
        }
        else
        {
            base -= 2U;
        }
    }
    /* And after its last bar. */
    trail = g_code.code_trail;
    base = first + ((length * (BENCH_CODE_ELEMENTS + 1U)) - 2U);
    b_side = false;
    while ((base < last) && (!b_side))
    {
        if ((double)g_code.run[base + 1U].units >= narrowest)
        {
            trail = (double)g_code.run[base + 1U].units;
            b_side = true;
        }
        else
        {
            base += 2U;
        }
    }
    for (character = 0U;
         (character < length) && (BENCH_SPEC_OK == result); ++character)
    {
        if (fabs((double)span[character] - median) > narrow)
        {
            result = BENCH_SPEC_WIDTHS;
            ++g_code.spec_widths;
        }
    }
    for (character = 0U;
         ((character + 1U) < length) && (BENCH_SPEC_OK == result);
         ++character)
    {
        base = first + (character * (BENCH_CODE_ELEMENTS + 1U));
        if ((double)g_code.run[base + BENCH_CODE_ELEMENTS].units >
            (BENCH_CODE_GAP_MAX_X * narrow))
        {
            result = BENCH_SPEC_GAPS;
            ++g_code.spec_gaps;
        }
    }
    if ((BENCH_SPEC_OK == result) &&
        (((lead >= 0.0) && (lead < quiet)) ||
         ((trail >= 0.0) && (trail < quiet))))
    {
        result = BENCH_SPEC_QUIET;
        ++g_code.spec_quiet;
    }
    return result;
}

/** @brief Characters of a group, the 3 widest of each 9 elements wide (the
 * darkness pass: bench_code39_dark); '?' where that is not a Code 39
 * pattern or a wide element is not wider than every narrow one. True if
 * every character decoded.
 */
static bool bench_code39_text (uint32_t first, uint32_t last, bool b_reverse,
                               char * p_text)
{
    uint32_t length = (((last - first) + 1U) + 1U) /
                      (BENCH_CODE_ELEMENTS + 1U);
    uint32_t character;
    uint32_t element;
    uint32_t pick;
    uint32_t widest;
    uint32_t pattern;
    uint32_t index;
    uint32_t units[BENCH_CODE_ELEMENTS];
    uint32_t narrowest_wide;
    uint32_t widest_narrow;
    bool     b_all = true;

    g_code.narrow_units = 0U;
    g_code.narrow_count = 0U;
    g_code.wide_units = 0U;
    g_code.wide_count = 0U;
    for (character = 0U; character < length; ++character)
    {
        for (element = 0U; element < BENCH_CODE_ELEMENTS; ++element)
        {
            units[element] = bench_code_units(first, last,
                (character * (BENCH_CODE_ELEMENTS + 1U)) + element,
                b_reverse);
        }
        pattern = 0U;
        if (g_code.b_dark_rule)
        {
            pattern = bench_code39_dark(first, last,
                                        character * (BENCH_CODE_ELEMENTS + 1U),
                                        b_reverse);
        }
        for (pick = 0U; (pick < 3U) && (!g_code.b_dark_rule); ++pick)
        {
            widest = BENCH_CODE_ELEMENTS;
            for (element = 0U; element < BENCH_CODE_ELEMENTS; ++element)
            {
                if ((0U == (pattern & (0x100U >> element))) &&
                    ((BENCH_CODE_ELEMENTS == widest) ||
                     (units[element] > units[widest])))
                {
                    widest = element;
                }
            }
            pattern |= 0x100U >> widest;
        }
        narrowest_wide = UINT32_MAX;
        widest_narrow = 0U;
        for (element = 0U; element < BENCH_CODE_ELEMENTS; ++element)
        {
            if (0U != (pattern & (0x100U >> element)))
            {
                g_code.wide_units += units[element];
                ++g_code.wide_count;
                if (units[element] < narrowest_wide)
                {
                    narrowest_wide = units[element];
                }
            }
            else
            {
                g_code.narrow_units += units[element];
                ++g_code.narrow_count;
                if (units[element] > widest_narrow)
                {
                    widest_narrow = units[element];
                }
            }
        }
        index = 0U;
        while ((index < BENCH_CODE_CHARS) &&
               (g_code39_patterns[index] != pattern))
        {
            ++index;
        }
        if ((index < BENCH_CODE_CHARS) &&
            (g_code.b_dark_rule || (narrowest_wide > widest_narrow)))
        {
            p_text[character] = g_code39_chars[index];
        }
        else
        {
            p_text[character] = '?';
            b_all = false;
        }
    }
    p_text[length] = '\0';
    return b_all;
}

/** @brief Width of element index of a group, counted from either end. */
static uint32_t bench_code_units (uint32_t first, uint32_t last,
                                  uint32_t index, bool b_reverse)
{
    return g_code.run[b_reverse ? (last - index) : (first + index)].units;
}

/** @brief The side sensor every 1 mm of its own path (the first sample in
 * each mm) over the whole run, at most BENCH_CODE_TRACE_MAX_MM; each row
 * starts with that mm and the centre path's mm there.
 */
static void bench_code_trace (void)
{
    uint32_t index = 0U;
    uint32_t column = 0U;
    int32_t  to_mm;
    int32_t  mm;

    to_mm = bench_round((double)g_code.sample[g_code.count - 1U].unit *
                        g_code.unit_m * 1000.0);
    if (to_mm > BENCH_CODE_TRACE_MAX_MM)
    {
        to_mm = BENCH_CODE_TRACE_MAX_MM;
    }
    bench_text(" side sensor raw every 1 mm of its own path (high is dark)"
               " from 0 to ");
    bench_dec(to_mm);
    bench_text(" mm; row: sensor mm c centre mm:");
    bench_flush();
    for (mm = 0; mm <= to_mm; ++mm)
    {
        while ((index < g_code.count) &&
               (((double)g_code.sample[index].unit * g_code.unit_m *
                 1000.0) < (double)mm))
        {
            ++index;
        }
        if (index >= g_code.count)
        {
            break;
        }
        if (0U == column)
        {
            bench_text("  ");
            bench_dec(mm);
            bench_text(" c");
            bench_code_unit_mm(g_code.sample[index].centre);
            bench_text(":");
        }
        bench_text(" ");
        bench_dec((int32_t)g_code.sample[index].raw);
        ++column;
        if (BENCH_CODE_TRACE_PER_ROW == column)
        {
            bench_flush();
            column = 0U;
        }
    }
    if (0U != column)
    {
        bench_flush();
    }
}

/** @brief Decoder units as whole millimetres. */
static void bench_code_unit_mm (uint32_t units)
{
    bench_dec(bench_round((double)units * g_code.unit_m * 1000.0));
}

/** @brief The selected gain set, without a line end. */
static void bench_gains_text (void)
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

/** @brief IR every tick; the 100 Hz accelerometer when a new set is ready.
 */
static void bench_sample (void)
{
    uint16_t samples[3] = {0U, 0U, 0U};
    int16_t  milligravity[3] = {0, 0, 0};
    uint32_t axis;

    if (board_read_ir(samples))
    {
        for (axis = 0U; axis < 3U; ++axis)
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
        (BOARD_ACCEL_FRESH == board_read_accel(milligravity)))
    {
        for (axis = 0U; axis < 3U; ++axis)
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

/** @brief Once a second: IR, accelerometer and wheel distance lines. */
static void bench_report (void)
{
    static const char * const ir_names[3] = {"IR L ", " R ", " B "};
    static const char * const axis_names[3] = {" x ", " y ", " z "};
    uint32_t axis;

    for (axis = 0U; (axis < 3U) && (0U != g_ir.count); ++axis)
    {
        bench_text(ir_names[axis]);
        bench_dec((int32_t)(g_ir.sum[axis] / g_ir.count));
        bench_text(" ");
        bench_dec((int32_t)g_ir.minimum[axis]);
        bench_text("-");
        bench_dec((int32_t)g_ir.maximum[axis]);
    }
    bench_flush();
    bench_text("ACC mg n ");
    bench_dec((int32_t)g_accel.count);
    for (axis = 0U; (axis < 3U) && (0U != g_accel.count); ++axis)
    {
        bench_text(axis_names[axis]);
        bench_signed(g_accel.sum[axis] / (int32_t)g_accel.count);
        bench_text(" ");
        bench_signed((int32_t)g_accel.minimum[axis]);
        bench_text("..");
        bench_signed((int32_t)g_accel.maximum[axis]);
    }
    bench_flush();
    bench_text("ENC counts L ");
    bench_signed(g_distance[BENCH_LEFT]);
    bench_text(" R ");
    bench_signed(g_distance[BENCH_RIGHT]);
    bench_text(" mm L ");
    bench_signed(bench_mm(g_distance[BENCH_LEFT]));
    bench_text(" R ");
    bench_signed(bench_mm(g_distance[BENCH_RIGHT]));
    bench_text(" err ");
    bench_dec((int32_t)g_encoder_errors);
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

/** @brief Signed modulo-2^16 difference, with the wheel's encoder sign. */
static int32_t bench_signed_delta (uint16_t newer, uint16_t older,
                                   uint32_t wheel)
{
    int32_t difference = (int32_t)(uint16_t)(newer - older);

    if (difference >= 0x8000)
    {
        difference -= 0x10000;
    }
    return difference * ((BENCH_LEFT == wheel) ? ROBOT_LEFT_ENCODER_SIGN :
                                                 ROBOT_RIGHT_ENCODER_SIGN);
}

/** @brief Wheel travel in whole millimetres for a signed count. */
static int32_t bench_mm (int32_t counts)
{
    double mm = (double)counts * 2.0 * CORE_PI * ROBOT_WHEEL_RADIUS_M *
                1000.0 / (double)ROBOT_ENCODER_COUNTS_PER_REV;

    return (int32_t)((mm >= 0.0) ? (mm + 0.5) : (mm - 0.5));
}

/** @brief Nearest whole number, halves away from zero. */
static int32_t bench_round (double value)
{
    return (int32_t)((value >= 0.0) ? (value + 0.5) : (value - 0.5));
}

static void bench_text (const char * p_text)
{
    while (('\0' != *p_text) && ((g_length + 2U) < BENCH_TEXT_BYTES))
    {
        g_text[g_length] = *p_text;
        ++g_length;
        ++p_text;
    }
    g_text[g_length] = '\0';
}

/** @brief Signed decimal; unsigned negation also covers INT32_MIN. */
static void bench_dec (int32_t value)
{
    char     digits[12];
    uint32_t index = sizeof(digits) - 1U;
    uint32_t magnitude = (value < 0) ? (0U - (uint32_t)value) :
                         (uint32_t)value;

    digits[index] = '\0';
    do
    {
        --index;
        digits[index] = (char)('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while ((0U != magnitude) && (index > 1U));
    if (value < 0)
    {
        --index;
        digits[index] = '-';
    }
    bench_text(&digits[index]);
}

/** @brief Decimal with an explicit sign. */
static void bench_signed (int32_t value)
{
    if (value >= 0)
    {
        bench_text("+");
    }
    bench_dec(value);
}

/** @brief value / 10^places (1 or 2) with that many decimals. */
static void bench_fixed (int32_t value, uint32_t places)
{
    uint32_t scale = (1U == places) ? 10U : 100U;
    uint32_t magnitude = (value < 0) ? (0U - (uint32_t)value) :
                         (uint32_t)value;
    uint32_t fraction = magnitude % scale;

    if (value < 0)
    {
        bench_text("-");
    }
    bench_dec((int32_t)(magnitude / scale));
    bench_text(".");
    if ((100U == scale) && (fraction < 10U))
    {
        bench_text("0");
    }
    bench_dec((int32_t)fraction);
}

/** @brief MISSION: the line built in g_text becomes the live line (g_live),
 * its first frames pushed at once.
 */
static void bench_live_take (void)
{
    (void)memcpy(g_live, g_text, g_length);
    g_live[g_length] = '\n';
    g_live_length = g_length + 1U;
    g_live_sent = 0U;
    g_length = 0U;
    bench_live_push();
}

/** @brief MISSION: as many frames of the live line as the MQTT queue takes
 * now, without waiting. Offline: all of it to UART0.
 */
static void bench_live_push (void)
{
#if ROBOT_MQTT
    uint32_t chunk;

    while (gb_mqtt && (g_live_sent < g_live_length))
    {
        chunk = g_live_length - g_live_sent;
        if (chunk > NET_MQTT_IPC_TX_MAX)
        {
            chunk = NET_MQTT_IPC_TX_MAX;
        }
        if (!net_mqtt_ipc_send((const uint8_t *)&g_live[g_live_sent], chunk))
        {
            return;
        }
        g_live_sent += chunk;
    }
#else
    if (g_live_sent < g_live_length)
    {
        g_live[g_live_length] = '\0';
        (void)tm_printf((const UB *)"%s", g_live);
        g_live_sent = g_live_length;
    }
#endif
}

/** @brief Before any other line: the live line's last frames, with the
 * waits of bench_flush, so lines never interleave; then no live line.
 */
static void bench_live_finish (void)
{
    uint32_t tries = 0U;

    bench_live_push();
    while ((g_live_sent < g_live_length) && (tries < g_send_tries))
    {
        bench_tick();
        bench_live_push();
        ++tries;
    }
    g_live_length = 0U;
    g_live_sent = 0U;
}

/** @brief Send one line, after any live line. MQTT: 20-byte frames, bounded
 * waits that keep the watchdog fed; a line the queue cannot take is
 * dropped. Offline: UART0.
 */
static void bench_flush (void)
{
#if ROBOT_MQTT
    uint32_t offset = 0U;
    uint32_t chunk;
    uint32_t tries;
#endif

    if (0U != g_live_length)
    {
        bench_live_finish();
    }
    g_text[g_length] = '\n';
    ++g_length;
#if ROBOT_MQTT
    while (gb_mqtt && (offset < g_length))
    {
        chunk = g_length - offset;
        if (chunk > NET_MQTT_IPC_TX_MAX)
        {
            chunk = NET_MQTT_IPC_TX_MAX;
        }
        tries = 0U;
        while (!net_mqtt_ipc_send((const uint8_t *)&g_text[offset], chunk))
        {
            ++tries;
            if (tries >= g_send_tries)
            {
                offset = g_length;
                break;
            }
            bench_tick();
        }
        if (offset < g_length)
        {
            offset += chunk;
        }
    }
#else
    g_text[g_length] = '\0';
    (void)tm_printf((const UB *)"%s", g_text);
#endif
    board_watchdog_feed();
    g_length = 0U;
    g_text[0] = '\0';
}

/* ==== TRIAL 2 Nov 2026: model-fit barcode decoder ("NEW" lines) ==========
 * Offline work in testing/barcode_lab (README.md and TRIAL_DECODER.md
 * there). Each cut the darkness tries finish is copied here as 1 mm bins
 * (bench_fit_begin), so the store and the tries go on at once. Cuts whose
 * dark is less than BENCH_FIT_JOIN_MM apart are joined first (the stream
 * cuts at 30 mm of white, and a blurred small card whose narrow bars vanish
 * has up to 45 mm between wide bars); the fit then runs over the next line
 * steps, at most BENCH_FIT_BUDGET_US of each, and sends its own reading as
 * one NEW line. It changes nothing the darkness tries do or report (CODE /
 * GROUP lines), nor the driving.
 *
 * Stage 1: every Code 39 symbol *X* has six wide bars (X any character
 * with two wide bars: all but $ / + %). Six of the BENCH_FIT_TOP humps with
 * the most darkness area are taken as them, every choice tried (a line or a
 * sheet's edge beside the card can be large too); their centres are fitted
 * (start and module, least squares) to each candidate's six wide-bar
 * centres, both directions; the BENCH_FIT_SHORT closest candidates (rms per
 * module) go on.
 * Stage 2: each short-listed candidate's bars are blurred by a Gaussian
 * spot (sd sigma), a dead zone c0 applied (the reading moves only once c0
 * of the spot is dark) and the result scaled to the cut's white and black
 * by least squares; the misfit is the sum of squares left. Spot and dead
 * zone are the sensor's, one for the pass: fitted over the candidates of
 * the closest wide-bar class (SHAPE), then every candidate fitted near its
 * stage-1 module and start with them a little either way (CANDS), then
 * module and start refined (FINE).
 * The best fit is reported when the best other character's misfit is at
 * least BENCH_FIT_MARGIN times larger, else "no read" with both.
 * Constants: the Code 39 table (g_code39_patterns) and its 1:3 ratio, the
 * spec's 3 mm narrow width for the join, and search ranges wide enough for
 * the sensor's spot and dead zone (the 30 Oct big-card fit gave sigma
 * 3.3-3.8 mm, c0 0.45-0.55). No letter, card size or track position.
 */
#define BENCH_FIT_JOIN_MM (54.0)    /* 15 modules between wide bars + 3, at 3 mm */
#define BENCH_FIT_BINS (640U)       /* 1 mm bins: at most 640 mm decoded */
#define BENCH_FIT_PAD_MM (30.0)     /* white kept either side of the dark */
#define BENCH_FIT_LONGEST_MM (420.0) /* longer dark: not one card */
#define BENCH_FIT_TPL_Q (4)         /* template points per mm */
#define BENCH_FIT_TPL (1800U)       /* template points: 450 mm */
#define BENCH_FIT_TPL_PAD_MM (20.0) /* template white before the first edge */
#define BENCH_FIT_PHI_N (1024U)     /* normal CDF table over -4..+4 sd */
#define BENCH_FIT_ONE (4096)        /* coverage 1.0 */
#define BENCH_FIT_S_ONE (1024)      /* response 1.0 */
#define BENCH_FIT_PHI_SLICE (6U)    /* normal CDF entries (one erf each, ~0.2 ms
                                     * on the M0+) built per line step */
#define BENCH_FIT_CHAR_SLICE (4U)   /* characters' tables built per line step */
#define BENCH_FIT_CANDS (78U)       /* 39 characters, both directions */
#define BENCH_FIT_EDGES (30U)       /* 15 bars of *X* */
#define BENCH_FIT_MODULES (47.0f)   /* modules of *X* */
#define BENCH_FIT_SHORT (10U)
#define BENCH_FIT_HUMPS (48U)
#define BENCH_FIT_LSQ_SLICE (26U)   /* stage-1 candidates per slice */
#define BENCH_FIT_TOP (6U)          /* largest humps, six of them tried (8
                                     * tried offline: more wrong reads) */
#define BENCH_FIT_BUDGET_US (1500U)
#define BENCH_FIT_MARGIN (1.3f)
#define BENCH_FIT_MISFIT (0.35f)    /* stage-1 rms limit, in modules */
#define BENCH_FIT_REL_PCT (15)      /* turning-point swing: % of the level */
#define BENCH_FIT_SDS (6)           /* ... or this many white noise sds */
#define BENCH_FIT_C0S (6U)
#define BENCH_FIT_X0S (13U)         /* start: -3..+3 mm by 0.5 */
#define BENCH_FIT_SHAPE_X0S (7U)    /* start: -1.5..+1.5 mm by 0.5 */
#define BENCH_FIT_FINE_X0S (7U)     /* start: -0.75..+0.75 mm by 0.25 */
#define BENCH_FIT_SHAPE_TPLS (18U)  /* 3 modules x 6 spots */
#define BENCH_FIT_CANDS_TPLS (21U)  /* 7 modules x 3 spots */
#define BENCH_FIT_FINE_TPLS (5U)
#define BENCH_FIT_LINES (4U)        /* NEW lines waiting for the link */

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
    /* the result, for its NEW line */
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

static bench_fit_t g_fit;
/* A NEW line waiting to go out: the fit goes on whatever the link does. */
typedef struct
{
    bench_fit_why_t why;
    bool     b_read;
    bool     b_rev;
    bool     b_other_rev;
    char     ch;
    char     other;
    float    margin;
    float    m;
    float    sigma;
    float    c0;
    float    rms;
    float    from;
    float    to;
    uint32_t cpu_us;
    uint32_t steps;
    uint32_t work;
    uint32_t joins;
} bench_fit_line_t;
static bench_fit_line_t g_fit_lines[BENCH_FIT_LINES];
static uint32_t    g_fit_line_first;   /* oldest waiting */
static uint32_t    g_fit_line_count;
static uint32_t    g_fit_lines_lost;   /* none free: the line dropped */
static bench_fit_line_t * gp_fit_line_new; /* pushed in this step's run */
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
static bench_fit_bin_t g_fit_bin;
static int32_t     g_fit_sum[BENCH_FIT_BINS];
static uint16_t    g_fit_cnt[BENCH_FIT_BINS];
static int16_t     g_fit_tpl[BENCH_FIT_TPL];
static uint16_t    g_fit_phi[BENCH_FIT_PHI_N + 1U];
static char        g_fit_char[BENCH_FIT_CANDS];
static bool        gb_fit_rev[BENCH_FIT_CANDS];
static uint8_t     g_fit_edge[BENCH_FIT_CANDS][BENCH_FIT_EDGES]; /* modules */
static float       g_fit_wide_dev[BENCH_FIT_CANDS][6]; /* centre - mean */
static float       g_fit_wide_mean[BENCH_FIT_CANDS];
static float       g_fit_wide_inv[BENCH_FIT_CANDS];    /* 1 / sum dev^2 */
static bool        gb_fit_tables;

static const float g_fit_shape_m[3] = { 0.985f, 1.0f, 1.015f };
static const float g_fit_shape_sigma[6] = { 1.5f, 2.1f, 2.7f, 3.3f, 3.9f, 4.5f };
static const float g_fit_shape_c0[BENCH_FIT_C0S] =
{
    0.15f, 0.25f, 0.35f, 0.45f, 0.55f, 0.65f
};

/** @brief The candidates from the Code 39 table: each character with two
 * wide bars inside *s, as read and reversed; bar edges in modules (narrow
 * 1, wide 3, 1 between characters) and the six wide bars' centres; the
 * normal CDF table for the templates.
 */
static bool bench_fit_tables (void)
{
    static uint32_t phi_i;     /* built so far, resumed by the next call */
    static uint32_t next_c;
    static uint32_t next_n;
    uint32_t c;
    uint32_t n = next_n;
    uint32_t i;
    uint32_t star = 0U;
    uint32_t r;
    uint32_t t;
    double   z;

    if (gb_fit_tables)
    {
        return true;
    }
    if (phi_i <= (BENCH_FIT_PHI_N / 2U))
    {
        /* The lower half only: Phi(-z) = 1 - Phi(z). */
        for (t = 0U; (t < BENCH_FIT_PHI_SLICE) && (phi_i <= (BENCH_FIT_PHI_N / 2U)); ++t, ++phi_i)
        {
            z = -4.0 + (8.0 * (double)phi_i / (double)BENCH_FIT_PHI_N);
            g_fit_phi[phi_i] = (uint16_t)bench_round(
                (double)BENCH_FIT_ONE * 0.5 * (1.0 + erf(z / sqrt(2.0))));
            g_fit_phi[BENCH_FIT_PHI_N - phi_i] = (uint16_t)(BENCH_FIT_ONE - g_fit_phi[phi_i]);
        }
        return false;
    }
    for (c = 0U; c < BENCH_CODE_CHARS; ++c)
    {
        if ('*' == g_code39_chars[c])
        {
            star = c;
        }
    }
    for (c = next_c, t = 0U;
         (c < BENCH_CODE_CHARS) && ((n + 1U) < BENCH_FIT_CANDS) && (t < BENCH_FIT_CHAR_SLICE);
         ++c, ++t)
    {
        uint16_t pat[3];
        uint8_t  width[29];
        uint8_t  bar[29];
        uint32_t e = 0U;
        uint32_t ch;

        if (NULL != strchr("*$/+%", g_code39_chars[c]))
        {
            continue;
        }
        pat[0] = g_code39_patterns[star];
        pat[1] = g_code39_patterns[c];
        pat[2] = g_code39_patterns[star];
        for (ch = 0U; ch < 3U; ++ch)
        {
            for (i = 0U; i < 9U; ++i)
            {
                width[e] = (0U != ((pat[ch] >> (8U - i)) & 1U)) ? 3U : 1U;
                bar[e] = (0U == (i % 2U)) ? 1U : 0U;
                ++e;
            }
            if (ch < 2U)
            {
                width[e] = 1U;
                bar[e] = 0U;
                ++e;
            }
        }
        for (r = 0U; r < 2U; ++r)
        {
            uint32_t pos = 0U;
            uint32_t edge = 0U;
            uint32_t wide = 0U;
            float    centre[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
            float    sum = 0.0f;
            float    sq = 0.0f;

            g_fit_char[n] = g_code39_chars[c];
            gb_fit_rev[n] = (1U == r);
            for (i = 0U; i < e; ++i)
            {
                uint32_t k = (0U == r) ? i : (e - 1U - i);

                if ((0U != bar[k]) && (edge < BENCH_FIT_EDGES))
                {
                    g_fit_edge[n][edge] = (uint8_t)pos;
                    g_fit_edge[n][edge + 1U] = (uint8_t)(pos + width[k]);
                    edge += 2U;
                    if ((3U == width[k]) && (wide < 6U))
                    {
                        centre[wide] = (float)pos + 1.5f;
                        sum += centre[wide];
                        ++wide;
                    }
                }
                pos += width[k];
            }
            g_fit_wide_mean[n] = sum / 6.0f;
            for (i = 0U; i < 6U; ++i)
            {
                g_fit_wide_dev[n][i] = centre[i] - g_fit_wide_mean[n];
                sq += g_fit_wide_dev[n][i] * g_fit_wide_dev[n][i];
            }
            g_fit_wide_inv[n] = (sq > 0.0f) ? (1.0f / sq) : 0.0f;
            ++n;
        }
    }
    next_c = c;
    next_n = n;
    gb_fit_tables = (c >= BENCH_CODE_CHARS) || ((n + 1U) >= BENCH_FIT_CANDS);
    return gb_fit_tables;
}

/** @brief One template: candidate cand's bars at module m mm, blurred by a
 * Gaussian of sd sigma mm, as coverage 0..BENCH_FIT_ONE every 1/TPL_Q mm
 * from BENCH_FIT_TPL_PAD_MM before the first edge. Bars are filled, then
 * each edge's blur added over +-4 sd: for a bar lo..hi the exact coverage is
 * the fill plus g(u - lo) - g(u - hi), g(d) = Phi(d / sigma) - [d >= 0].
 */
static bool bench_fit_render (uint32_t cand, float m, float sigma)
{
    uint32_t points = (uint32_t)(((BENCH_FIT_MODULES * m) +
                                  (2.0f * (float)BENCH_FIT_TPL_PAD_MM)) *
                                 (float)BENCH_FIT_TPL_Q) + 1U;
    uint32_t b;
    int32_t  p;
    int32_t  reach = (int32_t)(4.0f * sigma * (float)BENCH_FIT_TPL_Q) + 1;
    int32_t  step_q = (int32_t)((((float)BENCH_FIT_PHI_N / 8.0f) /
                                 (sigma * (float)BENCH_FIT_TPL_Q)) * 65536.0f);
    int32_t  pe[BENCH_FIT_EDGES];

    if (points > BENCH_FIT_TPL)
    {
        return false;
    }
    for (b = 0U; b < BENCH_FIT_EDGES; ++b)
    {
        pe[b] = (int32_t)ceilf((((float)g_fit_edge[cand][b] * m) +
                                (float)BENCH_FIT_TPL_PAD_MM) *
                               (float)BENCH_FIT_TPL_Q);
    }
    (void)memset(g_fit_tpl, 0, (size_t)points * sizeof(g_fit_tpl[0]));
    for (b = 0U; b < BENCH_FIT_EDGES; b += 2U)
    {
        for (p = pe[b]; (p < pe[b + 1U]) && (p < (int32_t)points); ++p)
        {
            g_fit_tpl[p] = (int16_t)(g_fit_tpl[p] + BENCH_FIT_ONE);
        }
    }
    for (b = 0U; b < BENCH_FIT_EDGES; ++b)
    {
        float   edge = ((float)g_fit_edge[cand][b] * m) + (float)BENCH_FIT_TPL_PAD_MM;
        int32_t sign = (0U == (b % 2U)) ? 1 : -1;
        int32_t p0 = pe[b] - reach;
        int32_t p1 = pe[b] + reach;
        /* table index at p: ((p / Q - edge) / sigma + 4) * N / 8, 16.16 */
        int32_t kq = (int32_t)((((((float)p0 / (float)BENCH_FIT_TPL_Q) - edge) /
                                  sigma) + 4.0f) *
                                ((float)BENCH_FIT_PHI_N / 8.0f) * 65536.0f);

        for (p = p0; p <= p1; ++p, kq += step_q)
        {
            int32_t k = (kq + 32768) / 65536;
            int32_t g;

            if ((p < 0) || (p >= (int32_t)points))
            {
                continue;
            }
            if (k < 0)
            {
                k = 0;
            }
            if (k > (int32_t)BENCH_FIT_PHI_N)
            {
                k = (int32_t)BENCH_FIT_PHI_N;
            }
            g = (int32_t)g_fit_phi[k] - ((p >= pe[b]) ? BENCH_FIT_ONE : 0);
            g_fit_tpl[p] = (int16_t)(g_fit_tpl[p] + (sign * g));
        }
    }
    for (p = 0; p < (int32_t)points; ++p)
    {
        if (g_fit_tpl[p] < 0)
        {
            g_fit_tpl[p] = 0;
        }
        if (g_fit_tpl[p] > BENCH_FIT_ONE)
        {
            g_fit_tpl[p] = BENCH_FIT_ONE;
        }
    }
    g_fit.tpl_points = points;
    g_fit.tpl_m = m;
    g_fit.tpl_sigma = sigma;
    return true;
}

/** @brief The rendered template placed with its first edge at x0 mm, each
 * dead zone in p_c0 tried: the least-squares misfit (white and black free)
 * of the binned cut; the best into *p_best when lower. The template is
 * looked up once per bin, keeping the bins it covers past the smallest dead
 * zone (the rest add nothing), then one tight pass per dead zone: on the
 * M0+ the sums stay in registers.
 */
static void bench_fit_eval (float x0, const float * p_c0, uint32_t c0s,
                            bench_fit_best_t * p_best)
{
    static int16_t cov[BENCH_FIT_BINS];   /* template at the kept bins */
    static int16_t val[BENCH_FIT_BINS];   /* their readings - white */
    uint32_t j;
    uint32_t i;
    uint32_t kept = 0U;
    uint32_t ss[BENCH_FIT_C0S];
    uint32_t sss[BENCH_FIT_C0S];
    int64_t  ssv[BENCH_FIT_C0S];
    int32_t  c0q[BENCH_FIT_C0S];
    int32_t  inv[BENCH_FIT_C0S];
    int32_t  c0_min = BENCH_FIT_ONE;
    /* template index of bin i's centre: idx0 + Q * i */
    int32_t  idx0 = (int32_t)floorf(((((g_fit.x_first + 0.5f) - x0) +
                                      (float)BENCH_FIT_TPL_PAD_MM) *
                                     (float)BENCH_FIT_TPL_Q) + 0.5f);
    double   n = (double)g_fit.valid;
    double   svv_c = g_fit.svv - ((g_fit.sv * g_fit.sv) / n);

    for (j = 0U; j < c0s; ++j)
    {
        ss[j] = 0U;
        sss[j] = 0U;
        ssv[j] = 0;
        c0q[j] = (int32_t)(p_c0[j] * (float)BENCH_FIT_ONE);
        inv[j] = ((int32_t)BENCH_FIT_S_ONE * 4096) / (BENCH_FIT_ONE - c0q[j]);
        c0_min = (c0q[j] < c0_min) ? c0q[j] : c0_min;
    }
    for (i = 0U; i < g_fit.valid; ++i)
    {
        int32_t idx = idx0 + ((int32_t)BENCH_FIT_TPL_Q * (int32_t)g_fit.valid_bin[i]);

        if ((idx >= 0) && (idx < (int32_t)g_fit.tpl_points) &&
            ((int32_t)g_fit_tpl[idx] > c0_min))
        {
            cov[kept] = g_fit_tpl[idx];
            val[kept] = g_fit.valid_v[i];
            ++kept;
        }
    }
    for (j = 0U; j < c0s; ++j)
    {
        const int32_t c0 = c0q[j];
        const uint32_t k = (uint32_t)inv[j];
        uint32_t a = 0U;
        uint32_t b = 0U;
        int64_t  d = 0;
        uint32_t i0;

        /* s * v is at most 1024 * 4095: 256 of them fit an int32 */
        for (i0 = 0U; i0 < kept; i0 += 256U)
        {
            uint32_t i1 = ((i0 + 256U) < kept) ? (i0 + 256U) : kept;
            int32_t  d32 = 0;

            for (i = i0; i < i1; ++i)
            {
                int32_t c = (int32_t)cov[i];

                if (c > c0)
                {
                    int32_t s = (int32_t)(((uint32_t)(c - c0) * k) >> 12);

                    if (s > BENCH_FIT_S_ONE)
                    {
                        s = BENCH_FIT_S_ONE;
                    }
                    a += (uint32_t)s;
                    b += (uint32_t)(s * s);
                    d32 += s * (int32_t)val[i];
                }
            }
            d += (int64_t)d32;
        }
        ss[j] = a;
        sss[j] = b;
        ssv[j] = d;
    }
    g_fit.work += g_fit.valid + (kept * c0s);
    for (j = 0U; j < c0s; ++j)
    {
        double sss_c = (double)sss[j] - (((double)ss[j] * (double)ss[j]) / n);
        double ssv_c = (double)ssv[j] - (((double)ss[j] * g_fit.sv) / n);
        double sse = svv_c;

        if ((sss_c > 1.0) && (ssv_c > 0.0))
        {
            sse = svv_c - ((ssv_c * ssv_c) / sss_c);
        }
        if ((float)sse < p_best->sse)
        {
            p_best->sse = (float)sse;
            p_best->m = g_fit.tpl_m;
            p_best->sigma = g_fit.tpl_sigma;
            p_best->x0 = x0;
            p_best->c0 = p_c0[j];
        }
    }
}

/** @brief Stage 1's humps, on the bins (gaps filled): turning points with a
 * swing of BENCH_FIT_SDS white sds or BENCH_FIT_REL_PCT of the level above
 * white, whichever is more (dark readings are noisier, and a fixed swing
 * splits a wide bar's flat top); each hump's centre (above half its
 * height) and darkness area; the six largest areas' centres, by position.
 */
static void bench_fit_humps (void)
{
    const int16_t * p_v = g_fit.vf;
    uint32_t bins = g_fit.bins;
    /* static: the bench task's stack is 4 KB (APP_MAIN_STACK_BYTES) */
    static uint16_t ext[(BENCH_FIT_HUMPS * 2U) + 2U];
    static bool     b_peak[(BENCH_FIT_HUMPS * 2U) + 2U];
    static float    hc[BENCH_FIT_HUMPS];
    static int32_t  ha[BENCH_FIT_HUMPS];
    static bool     b_used[BENCH_FIT_HUMPS];
    uint32_t exts = 0U;
    uint32_t i;
    uint32_t hi = 0U;
    uint32_t lo = 0U;
    int32_t  mode = 0;
    uint32_t humps = 0U;
    int32_t  sw = BENCH_FIT_SDS * g_fit.sd;
    int32_t  white = g_fit.white;

#define BENCH_FIT_NEED(level) \
    ((((((int32_t)(level) - white) * BENCH_FIT_REL_PCT) / 100) > sw) ? \
     ((((int32_t)(level) - white) * BENCH_FIT_REL_PCT) / 100) : sw)
    for (i = 1U; (i < bins) && ((exts + 2U) < (BENCH_FIT_HUMPS * 2U)); ++i)
    {
        int32_t v = (int32_t)p_v[i];

        if (1 == mode)
        {
            if (v > (int32_t)p_v[hi])
            {
                hi = i;
            }
            else if (((int32_t)p_v[hi] - v) >= BENCH_FIT_NEED(p_v[hi]))
            {
                ext[exts] = (uint16_t)hi;
                b_peak[exts] = true;
                ++exts;
                mode = -1;
                lo = i;
            }
            else
            {
                /* Still at the top. */
            }
        }
        else
        {
            if (v < (int32_t)p_v[lo])
            {
                lo = i;
            }
            else if ((v - (int32_t)p_v[lo]) >= BENCH_FIT_NEED(v))
            {
                ext[exts] = (uint16_t)lo;
                b_peak[exts] = false;
                ++exts;
                mode = 1;
                hi = i;
            }
            else
            {
                /* Still at the bottom. */
            }
        }
    }
#undef BENCH_FIT_NEED
    if (1 == mode)
    {
        ext[exts] = (uint16_t)hi;
        b_peak[exts] = true;
        ++exts;
    }
    ext[exts] = (uint16_t)(bins - 1U);
    b_peak[exts] = false;
    ++exts;
    for (i = 0U; (i < exts) && (humps < BENCH_FIT_HUMPS); ++i)
    {
        uint32_t a;
        uint32_t b;
        uint32_t p;
        int32_t  base;
        int32_t  half;
        int32_t  w_sum = 0;
        int64_t  wx_sum = 0;      /* weights x (2 bin + 1) */
        int32_t  area = 0;

        if ((!b_peak[i]) || ((int32_t)p_v[ext[i]] < (white + sw)))
        {
            continue;
        }
        a = (i > 0U) ? ext[i - 1U] : 0U;
        b = ((i + 1U) < exts) ? ext[i + 1U] : (bins - 1U);
        base = ((int32_t)p_v[a] > (int32_t)p_v[b]) ? (int32_t)p_v[a] : (int32_t)p_v[b];
        half = base + (((int32_t)p_v[ext[i]] - base) / 2);
        for (p = a; p <= b; ++p)
        {
            int32_t w = (int32_t)p_v[p] - half;

            if (w > 0)
            {
                w_sum += w;
                wx_sum += (int64_t)w * (int64_t)((2U * p) + 1U);
            }
            if ((int32_t)p_v[p] > white)
            {
                area += (int32_t)p_v[p] - white;
            }
        }
        if (w_sum <= 0)
        {
            continue;
        }
        hc[humps] = g_fit.x_first + (0.5f * (float)wx_sum / (float)w_sum);
        ha[humps] = area;
        ++humps;
    }
    g_fit.humps = humps;
    if (humps < 6U)
    {
        return;
    }
    {
        uint32_t t;
        uint32_t top = (humps < BENCH_FIT_TOP) ? humps : BENCH_FIT_TOP;

        (void)memset(b_used, 0, sizeof(b_used));
        for (t = 0U; t < top; ++t)
        {
            uint32_t best = BENCH_FIT_HUMPS;

            for (i = 0U; i < humps; ++i)
            {
                if ((!b_used[i]) && ((BENCH_FIT_HUMPS == best) || (ha[i] > ha[best])))
                {
                    best = i;
                }
            }
            b_used[best] = true;
            g_fit.top_pos[t] = hc[best];
        }
        for (t = 1U; t < top; ++t)
        {
            float    v = g_fit.top_pos[t];
            uint32_t u = t;

            while ((u > 0U) && (g_fit.top_pos[u - 1U] > v))
            {
                g_fit.top_pos[u] = g_fit.top_pos[u - 1U];
                --u;
            }
            g_fit.top_pos[u] = v;
        }
        g_fit.top_n = top;
    }
}

/** @brief Stage 1's next choice of six of the largest humps (a bit mask of
 * top_n bits, six set), their centres in pos[]. False when all are done.
 */
static bool bench_fit_subset (void)
{
    uint32_t t;
    uint32_t n = 0U;

    do
    {
        ++g_fit.mask;
        n = 0U;
        for (t = 0U; t < g_fit.top_n; ++t)
        {
            n += (g_fit.mask >> t) & 1U;
        }
    } while ((6U != n) && (g_fit.mask < (1UL << g_fit.top_n)));
    if (g_fit.mask >= (1UL << g_fit.top_n))
    {
        return false;
    }
    n = 0U;
    g_fit.pos_mean = 0.0f;
    for (t = 0U; t < g_fit.top_n; ++t)
    {
        if (0U != ((g_fit.mask >> t) & 1U))
        {
            g_fit.pos[n] = g_fit.top_pos[t];
            g_fit.pos_mean += g_fit.top_pos[t] / 6.0f;
            ++n;
        }
    }
    return true;
}

/** @brief Stage 1's fit of the six centres to candidate c's: module and
 * start by least squares; kept when its rms per module is the candidate's
 * lowest yet (choices of humps compared fairly whatever their module).
 */
static void bench_fit_lsq (uint32_t c)
{
    float    scp = 0.0f;
    float    r2 = 0.0f;
    float    m;
    float    rms;
    uint32_t i;

    for (i = 0U; i < 6U; ++i)
    {
        scp += g_fit_wide_dev[c][i] * (g_fit.pos[i] - g_fit.pos_mean);
    }
    m = scp * g_fit_wide_inv[c];
    if (m <= 0.0f)
    {
        return;
    }
    for (i = 0U; i < 6U; ++i)
    {
        float d = (g_fit.pos[i] - g_fit.pos_mean) - (m * g_fit_wide_dev[c][i]);

        r2 += d * d;
    }
    rms = sqrtf(r2 / 6.0f);
    if ((rms / m) < g_fit.rel[c])
    {
        g_fit.rel[c] = rms / m;
        g_fit.rms[c] = rms;
        g_fit.ms[c] = m;
        g_fit.x0s[c] = g_fit.pos_mean - (m * g_fit_wide_mean[c]);
    }
}

/** @brief Stage 1's short list: the BENCH_FIT_SHORT closest candidates
 * (stage 2 then fits the whole binned window: narrowing it to where they
 * can lie was tried offline and changed nothing).
 */
static void bench_fit_short (void)
{
    bool     b_used[BENCH_FIT_CANDS];
    uint32_t t;
    uint32_t c;

    (void)memset(b_used, 0, sizeof(b_used));
    g_fit.short_n = 0U;
    for (t = 0U; t < BENCH_FIT_SHORT; ++t)
    {
        uint32_t best = BENCH_FIT_CANDS;

        for (c = 0U; c < BENCH_FIT_CANDS; ++c)
        {
            if ((!b_used[c]) &&
                ((BENCH_FIT_CANDS == best) || (g_fit.rel[c] < g_fit.rel[best])))
            {
                best = c;
            }
        }
        b_used[best] = true;
        g_fit.short_cand[t] = (uint8_t)best;
        ++g_fit.short_n;
    }
    g_fit.s1_rel = g_fit.rel[g_fit.short_cand[0]];
    g_fit.s1_rms = g_fit.rms[g_fit.short_cand[0]];
}

/** @brief A NEW line for the cut, with its position and cost, queued. */
static void bench_fit_say (bench_fit_why_t why, float from, float to)
{
    bench_fit_line_t * p_line;

    if (g_fit_line_count >= BENCH_FIT_LINES)
    {
        ++g_fit_lines_lost;
        return;
    }
    p_line = &g_fit_lines[(g_fit_line_first + g_fit_line_count) % BENCH_FIT_LINES];
    ++g_fit_line_count;
    p_line->why = why;
    p_line->b_read = g_fit.b_read;
    p_line->ch = g_fit.text_char;
    p_line->b_rev = g_fit.b_text_rev;
    p_line->other = g_fit.text_other;
    p_line->b_other_rev = g_fit.b_text_other_rev;
    p_line->margin = g_fit.text_margin;
    p_line->m = g_fit.text_m;
    p_line->sigma = g_fit.text_sigma;
    p_line->c0 = g_fit.text_c0;
    p_line->rms = g_fit.s1_rms;
    p_line->from = from;
    p_line->to = to;
    p_line->cpu_us = g_fit.cpu_us;
    p_line->steps = g_fit.steps;
    p_line->work = g_fit.work;
    p_line->joins = g_fit.joins;
    gp_fit_line_new = p_line;
}

/** @brief The samples of the cut just decoded (0..cut_count) after
 * binned_u, tape-dark lines left out, added to the binning slot.
 */
static void bench_fit_add (void)
{
    uint32_t index;
    uint32_t i;

    for (index = 0U; index < g_code.cut_count; ++index)
    {
        uint32_t unit = g_code.sample[index].unit;

        if ((unit < g_fit_bin.u0) ||
            ((0U != g_fit_bin.binned_u) && (unit <= g_fit_bin.binned_u)) ||
            bench_code_in_line(index))
        {
            continue;
        }
        i = ((unit - g_fit_bin.u0) * g_fit_bin.k16) >> 16;
        if (i < BENCH_FIT_BINS)
        {
            g_fit_sum[i] += (int32_t)g_code.sample[index].raw;
            ++g_fit_cnt[i];
        }
    }
    if (0U != g_code.cut_count)
    {
        uint32_t last = g_code.sample[g_code.cut_count - 1U].unit;

        if (last > g_fit_bin.binned_u)
        {
            g_fit_bin.binned_u = last;
        }
    }
}

/** @brief The binning slot moved to the fit: the window (the dark +-
 * BENCH_FIT_PAD_MM) as bin means, with their sums; the slot is free again.
 * False when there is nothing to fit.
 */
static bool bench_fit_prepare (void)
{
    float    unit_mm = (float)(g_code.unit_m * 1000.0);
    uint32_t pad_u = (uint32_t)bench_round((BENCH_FIT_PAD_MM / 1000.0) / g_code.unit_m);
    uint32_t i;
    int64_t  svv = 0;
    int32_t  sv = 0;

    g_fit_bin.b_used = false;
    g_fit.white = g_fit_bin.white;
    g_fit.sd = g_fit_bin.sd;
    g_fit.joins = g_fit_bin.joins;
    g_fit.cpu_us = 0U;
    g_fit.steps = 0U;
    g_fit.work = 0U;
    g_fit.s1_rms = 0.0f;
    g_fit.dark_from = (float)g_fit_bin.first_u * unit_mm;
    g_fit.dark_to = (float)g_fit_bin.last_u * unit_mm;
    if ((g_fit.dark_to - g_fit.dark_from) > (float)BENCH_FIT_LONGEST_MM)
    {
        ++g_fit.no_reads;
        bench_fit_say(BENCH_FIT_WHY_LONG, g_fit.dark_from, g_fit.dark_to);
        return false;
    }
    g_fit.x_first = (float)g_fit_bin.u0 * unit_mm;
    g_fit.bins = (((g_fit_bin.last_u + pad_u - g_fit_bin.u0) * g_fit_bin.k16) >> 16) + 1U;
    if (g_fit.bins > BENCH_FIT_BINS)
    {
        g_fit.bins = BENCH_FIT_BINS;
    }
    g_fit.valid = 0U;
    for (i = 0U; i < g_fit.bins; ++i)
    {
        if (0U != g_fit_cnt[i])
        {
            int32_t v = (g_fit_sum[i] + ((int32_t)g_fit_cnt[i] / 2)) /
                        (int32_t)g_fit_cnt[i];
            int32_t d = v - g_fit.white;

            g_fit.vf[i] = (int16_t)v;
            g_fit.valid_bin[g_fit.valid] = (uint16_t)i;
            g_fit.valid_v[g_fit.valid] = (int16_t)d;
            sv += d;
            svv += (int64_t)d * (int64_t)d;
            ++g_fit.valid;
        }
        else
        {
            g_fit.vf[i] = (i > 0U) ? g_fit.vf[i - 1U] : (int16_t)g_fit.white;
        }
    }
    g_fit.sv = (double)sv;
    g_fit.svv = (double)svv;
    if (g_fit.valid < 50U)
    {
        return false;
    }
    g_fit.phase = BENCH_FIT_HUMPS_PHASE;
    return true;
}

/** @brief MISSION, a cut the darkness tries have just finished (samples
 * 0..cut_count, before bench_code_keep): its readings (tape-dark lines left
 * out) added to 1 mm bins, so the fit can run while the store goes on. A
 * cut whose dark starts within BENCH_FIT_JOIN_MM of the last one's joins
 * it; a binned card goes to the fit once that much white has followed
 * (bench_fit_run), or at once when the next card comes and the fit is
 * free. Integer work only; the fit itself is sliced.
 */
static void bench_fit_begin (void)
{
    float    unit_mm = (float)(g_code.unit_m * 1000.0);
    uint32_t join_u = (uint32_t)bench_round((BENCH_FIT_JOIN_MM / 1000.0) / g_code.unit_m);
    uint32_t pad_u = (uint32_t)bench_round((BENCH_FIT_PAD_MM / 1000.0) / g_code.unit_m);
    uint32_t floor_adc;
    uint32_t first_u = 0U;
    uint32_t last_u = 0U;
    uint32_t index;
    bool     b_dark = false;
    int32_t  white = (int32_t)g_code.cut_white;
    int32_t  sd = (0U != g_code.cut_sd) ? (int32_t)g_code.cut_sd : 1;

    floor_adc = (uint32_t)(white + (BENCH_FIT_SDS * sd));
    for (index = 0U; index < g_code.cut_count; ++index)
    {
        if (((uint32_t)g_code.sample[index].raw < floor_adc) ||
            bench_code_in_line(index))
        {
            continue;
        }
        if (!b_dark)
        {
            first_u = g_code.sample[index].unit;
            b_dark = true;
        }
        last_u = g_code.sample[index].unit;
    }
    if (!b_dark)
    {
        return;
    }
    if (g_fit_bin.b_used && (first_u <= (g_fit_bin.last_u + join_u)))
    {
        /* The same card, cut at the stream's 30 mm of white: joined. */
        if (last_u > g_fit_bin.last_u)
        {
            g_fit_bin.last_u = last_u;
        }
        ++g_fit_bin.joins;
        bench_fit_add();
        return;
    }
    if (g_fit_bin.b_used && (BENCH_FIT_IDLE == g_fit.phase) && gb_fit_tables)
    {
        (void)bench_fit_prepare();   /* a new card: the binned one is whole */
    }
    if (g_fit_bin.b_used)
    {
        ++g_fit.skipped;
        bench_fit_say(BENCH_FIT_WHY_BUSY, (float)first_u * unit_mm,
                      (float)last_u * unit_mm);
        return;
    }
    g_fit_bin.b_used = true;
    g_fit_bin.white = white;
    g_fit_bin.sd = sd;
    g_fit_bin.first_u = first_u;
    g_fit_bin.last_u = last_u;
    g_fit_bin.binned_u = 0U;
    g_fit_bin.joins = 0U;
    g_fit_bin.u0 = (first_u > pad_u) ? (first_u - pad_u) : 0U;
    g_fit_bin.k16 = (uint32_t)bench_round((double)unit_mm * 65536.0);
    (void)memset(g_fit_sum, 0, sizeof(g_fit_sum));
    (void)memset(g_fit_cnt, 0, sizeof(g_fit_cnt));
    bench_fit_add();
}

/** @brief The result: best short-listed candidate, the best other character
 * and their misfit ratio, into the NEW line.
 */
static void bench_fit_finish (void)
{
    uint32_t k;
    uint32_t best = 0U;
    uint32_t other = BENCH_FIT_SHORT;

    for (k = 1U; k < g_fit.short_n; ++k)
    {
        if (g_fit.best[k].sse < g_fit.best[best].sse)
        {
            best = k;
        }
    }
    for (k = 0U; k < g_fit.short_n; ++k)
    {
        if ((g_fit_char[g_fit.short_cand[k]] != g_fit_char[g_fit.short_cand[best]]) &&
            ((BENCH_FIT_SHORT == other) || (g_fit.best[k].sse < g_fit.best[other].sse)))
        {
            other = k;
        }
    }
    g_fit.text_char = g_fit_char[g_fit.short_cand[best]];
    g_fit.b_text_rev = gb_fit_rev[g_fit.short_cand[best]];
    g_fit.text_m = g_fit.best[best].m;
    g_fit.text_sigma = g_fit.best[best].sigma;
    g_fit.text_c0 = g_fit.best[best].c0;
    if (BENCH_FIT_SHORT == other)
    {
        g_fit.text_other = '?';
        g_fit.b_text_other_rev = false;
        g_fit.text_margin = 99.0f;
    }
    else
    {
        g_fit.text_other = g_fit_char[g_fit.short_cand[other]];
        g_fit.b_text_other_rev = gb_fit_rev[g_fit.short_cand[other]];
        g_fit.text_margin = g_fit.best[other].sse / fmaxf(g_fit.best[best].sse, 1.0f);
    }
    g_fit.b_read = (g_fit.text_margin >= BENCH_FIT_MARGIN);
    if (g_fit.b_read)
    {
        ++g_fit.reads;
    }
    else
    {
        ++g_fit.no_reads;
    }
    bench_fit_say(BENCH_FIT_WHY_FIT, g_fit.best[best].x0,
                  g_fit.best[best].x0 + (BENCH_FIT_MODULES * g_fit.best[best].m));
    g_fit.phase = BENCH_FIT_IDLE;
}

/** @brief One slice of the fit: the humps, BENCH_FIT_LSQ_SLICE stage-1
 * candidates, or one template rendered, or one start position of it (all
 * its dead zones). Returns false once the fit is over.
 */
static bool bench_fit_slice (void)
{
    float    c0s[3] = { 0.0f, 0.0f, 0.0f };
    const float * p_c0 = c0s;
    uint32_t c0n;
    uint32_t cand;
    float    m;
    float    sigma;
    float    x0;
    uint32_t tpls;
    uint32_t x0n;
    uint32_t t;
    bench_fit_best_t * p_best;

    switch (g_fit.phase)
    {
        case BENCH_FIT_HUMPS_PHASE:
            bench_fit_humps();
            if (g_fit.humps < 6U)
            {
                /* Not a card at this blur: the GROUP lines show it. */
                g_fit.phase = BENCH_FIT_IDLE;
                return false;
            }
            for (t = 0U; t < BENCH_FIT_CANDS; ++t)
            {
                g_fit.rel[t] = 1.0e9f;
                g_fit.rms[t] = 1.0e9f;
                g_fit.ms[t] = 0.0f;
                g_fit.x0s[t] = 0.0f;
            }
            g_fit.mask = 0U;
            (void)bench_fit_subset();
            g_fit.c = 0U;
            g_fit.phase = BENCH_FIT_LSQ;
            return true;
        case BENCH_FIT_LSQ:
            /* Every choice of six of the largest humps, every candidate. */
            for (t = 0U; (t < BENCH_FIT_LSQ_SLICE) && (g_fit.c < BENCH_FIT_CANDS); ++t)
            {
                bench_fit_lsq(g_fit.c);
                ++g_fit.c;
            }
            if (g_fit.c < BENCH_FIT_CANDS)
            {
                return true;
            }
            g_fit.c = 0U;
            if (bench_fit_subset())
            {
                return true;
            }
            bench_fit_short();
            if (g_fit.s1_rel > BENCH_FIT_MISFIT)
            {
                ++g_fit.no_reads;
                bench_fit_say(BENCH_FIT_WHY_MISFIT, g_fit.dark_from, g_fit.dark_to);
                g_fit.phase = BENCH_FIT_IDLE;
                return false;
            }
            g_fit.k = 0U;
            g_fit.tpl = 0U;
            g_fit.xi = 0U;
            g_fit.b_rendered = false;
            g_fit.shape.sse = 3.0e38f;
            for (t = 0U; t < BENCH_FIT_SHORT; ++t)
            {
                g_fit.best[t].sse = 3.0e38f;
            }
            /* The closest wide-bar class: candidates whose wide bars lie
             * where the closest one's do (the same fit to the humps).
             */
            g_fit.shape_n = 1U;
            while ((g_fit.shape_n < g_fit.short_n) &&
                   (g_fit.rel[g_fit.short_cand[g_fit.shape_n]] <=
                    (g_fit.s1_rel * 1.0001f)))
            {
                ++g_fit.shape_n;
            }
            g_fit.phase = BENCH_FIT_SHAPE;
            return true;
        case BENCH_FIT_SHAPE:
            /* One spot and dead zone for the pass (the sensor's, the same
             * whatever the card says), fitted over every candidate of the
             * closest class: the stage-1 order inside a class is arbitrary,
             * and a wrong character's best spot blurs away the bars it
             * lacks.
             */
            cand = g_fit.short_cand[g_fit.k];
            tpls = BENCH_FIT_SHAPE_TPLS;
            m = g_fit.ms[cand] * g_fit_shape_m[g_fit.tpl % 3U];
            sigma = g_fit_shape_sigma[g_fit.tpl / 3U];
            x0n = BENCH_FIT_SHAPE_X0S;
            x0 = g_fit.x0s[cand] - 1.5f + (0.5f * (float)g_fit.xi);
            p_c0 = g_fit_shape_c0;
            c0n = BENCH_FIT_C0S;
            p_best = &g_fit.shape;
            break;
        case BENCH_FIT_CANDS_PHASE:
            /* Every candidate near its stage-1 module and start, the
             * pass's spot and dead zone a little either way.
             */
            cand = g_fit.short_cand[g_fit.k];
            tpls = BENCH_FIT_CANDS_TPLS;
            m = g_fit.ms[cand] * (0.97f + (0.01f * (float)(g_fit.tpl % 7U)));
            sigma = g_fit.shape.sigma * ((0U == (g_fit.tpl / 7U)) ? 0.85f :
                                         ((1U == (g_fit.tpl / 7U)) ? 1.0f : 1.15f));
            x0n = BENCH_FIT_X0S;
            x0 = g_fit.x0s[cand] - 3.0f + (0.5f * (float)g_fit.xi);
            c0s[0] = fmaxf(0.0f, g_fit.shape.c0 - 0.07f);
            c0s[1] = g_fit.shape.c0;
            c0s[2] = fminf(0.8f, g_fit.shape.c0 + 0.07f);
            c0n = 3U;
            p_best = &g_fit.best[g_fit.k];
            break;
        case BENCH_FIT_FINE:
            /* Around the candidate's best so far, fixed when it starts. */
            if ((0U == g_fit.tpl) && (0U == g_fit.xi) && (!g_fit.b_rendered))
            {
                g_fit.centre = g_fit.best[g_fit.k];
            }
            cand = g_fit.short_cand[g_fit.k];
            tpls = BENCH_FIT_FINE_TPLS;
            m = g_fit.centre.m * (0.99f + (0.005f * (float)g_fit.tpl));
            sigma = g_fit.centre.sigma;
            x0n = BENCH_FIT_FINE_X0S;
            x0 = g_fit.centre.x0 - 0.75f + (0.25f * (float)g_fit.xi);
            c0s[0] = g_fit.centre.c0;
            c0n = 1U;
            p_best = &g_fit.best[g_fit.k];
            if (g_fit.centre.sse >= 3.0e38f)
            {
                g_fit.xi = x0n;  /* nothing fitted for it: skipped */
                g_fit.b_rendered = true;
            }
            break;
        default:
            g_fit.phase = BENCH_FIT_IDLE;
            return false;
    }
    if (!g_fit.b_rendered)
    {
        if (!bench_fit_render(cand, m, sigma))
        {
            g_fit.xi = x0n;      /* longer than the template store: skipped */
        }
        g_fit.b_rendered = true;
    }
    else if (g_fit.xi < x0n)
    {
        bench_fit_eval(x0, p_c0, c0n, p_best);
        ++g_fit.xi;
    }
    else
    {
        /* Skipped. */
    }
    if (g_fit.xi < x0n)
    {
        return true;
    }
    g_fit.xi = 0U;
    g_fit.b_rendered = false;
    ++g_fit.tpl;
    if (g_fit.tpl < tpls)
    {
        return true;
    }
    g_fit.tpl = 0U;
    ++g_fit.k;
    if (BENCH_FIT_SHAPE == g_fit.phase)
    {
        if (g_fit.k < g_fit.shape_n)
        {
            return true;
        }
        g_fit.k = 0U;
        if (g_fit.shape.sse >= 3.0e38f)
        {
            /* Every template longer than the store: no spot to go on. */
            ++g_fit.no_reads;
            bench_fit_say(BENCH_FIT_WHY_MISFIT, g_fit.dark_from, g_fit.dark_to);
            g_fit.phase = BENCH_FIT_IDLE;
            return false;
        }
        g_fit.phase = BENCH_FIT_CANDS_PHASE;
        return true;
    }
    if (g_fit.k < g_fit.short_n)
    {
        return true;
    }
    g_fit.k = 0U;
    if (BENCH_FIT_CANDS_PHASE == g_fit.phase)
    {
        g_fit.phase = BENCH_FIT_FINE;
        return true;
    }
    bench_fit_finish();
    return false;
}

/** @brief MISSION, every line step: the binned card to the fit once
 * BENCH_FIT_JOIN_MM of white has followed its last dark with no group open
 * and the fit is free; then slices of the fit until
 * BENCH_FIT_BUDGET_US has gone in this step (at least one slice).
 */
static void bench_fit_run (void)
{
    uint32_t join_u = (uint32_t)bench_round((BENCH_FIT_JOIN_MM / 1000.0) / g_code.unit_m);
    uint32_t newest = (0U != g_code.count) ? g_code.sample[g_code.count - 1U].unit : 0U;
    uint64_t t0;
    uint32_t used = 0U;
    bool     b_more;

    if (!gb_fit_tables)
    {
        (void)bench_fit_tables();   /* a share of the tables each step, from the first */
    }
    if ((!gb_fit_tables) && (BENCH_FIT_IDLE == g_fit.phase))
    {
        return;
    }
    if (g_fit_bin.b_used && (BENCH_FIT_IDLE == g_fit.phase) &&
        (!g_code.b_open) && (newest >= (g_fit_bin.last_u + join_u)))
    {
        (void)bench_fit_prepare();
    }
    if (BENCH_FIT_IDLE == g_fit.phase)
    {
        return;
    }
    ++g_fit.steps;
    gp_fit_line_new = NULL;
    do
    {
        t0 = board_time_us();
        b_more = bench_fit_slice();
        used += (uint32_t)(board_time_us() - t0);
    } while (b_more && (used < BENCH_FIT_BUDGET_US));
    g_fit.cpu_us += used;
    if ((!b_more) && (NULL != gp_fit_line_new))
    {
        gp_fit_line_new->cpu_us = g_fit.cpu_us;   /* with the last slice */
    }
}

/** @brief MISSION, after the stop: the fit under way, else (b_bin) the
 * binned card, finished at once. True while there is something to say or
 * do: called until false, its NEW line sent each time.
 */
static bool bench_fit_complete (bool b_bin)
{
    while ((!gb_fit_tables) && (b_bin ? g_fit_bin.b_used : false))
    {
        (void)bench_fit_tables();   /* nothing else runs after the stop */
    }
    if ((BENCH_FIT_IDLE == g_fit.phase) &&
        ((!b_bin) || (!g_fit_bin.b_used) || (!bench_fit_prepare())))
    {
        return (0U != g_fit_line_count);
    }
    while ((BENCH_FIT_IDLE != g_fit.phase) && bench_fit_slice())
    {
        /* Nothing else runs after the stop. */
    }
    return true;
}

/** @brief One character, as text. */
static void bench_fit_char (char ch)
{
    char text[2];

    text[0] = ch;
    text[1] = '\0';
    bench_text(text);
}

/** @brief The NEW line, once per fitted cut (at most 150 characters:
 * BENCH_TEXT_BYTES). "rev": read end to start.
 */
static bool bench_fit_text (void)
{
    const bench_fit_line_t * p_line;

    if (0U == g_fit_line_count)
    {
        return false;
    }
    p_line = &g_fit_lines[g_fit_line_first];
    g_fit_line_first = (g_fit_line_first + 1U) % BENCH_FIT_LINES;
    --g_fit_line_count;
    switch (p_line->why)
    {
        case BENCH_FIT_WHY_LONG:
            bench_text(" NEW no read: dark for ");
            bench_dec((int32_t)(p_line->to - p_line->from));
            bench_text(" mm, longer than a card");
            break;
        case BENCH_FIT_WHY_MISFIT:
            bench_text(" NEW no read: the 6 widest bars fit no character (");
            bench_fixed((int32_t)(10.0f * p_line->rms), 1U);
            bench_text(" mm off)");
            break;
        case BENCH_FIT_WHY_BUSY:
            bench_text(" NEW skipped: still fitting the last card");
            break;
        default:
            bench_text(p_line->b_read ? " NEW *" : " NEW no read: best *");
            bench_fit_char(p_line->ch);
            bench_text(p_line->b_rev ? "* rev" : "*");
            bench_text(", margin ");
            bench_fixed((int32_t)(100.0f * fminf(p_line->margin, 99.0f)), 2U);
            bench_text(" over *");
            bench_fit_char(p_line->other);
            bench_text(p_line->b_other_rev ? "* rev" : "*");
            bench_text("; module ");
            bench_fixed((int32_t)(100.0f * p_line->m), 2U);
            bench_text(" mm, spot ");
            bench_fixed((int32_t)(10.0f * p_line->sigma), 1U);
            bench_text(" mm, dead ");
            bench_fixed((int32_t)(100.0f * p_line->c0), 2U);
            break;
    }
    bench_text("; at ");
    bench_dec((int32_t)p_line->from);
    bench_text("-");
    bench_dec((int32_t)p_line->to);
    bench_text(" mm");
    if (BENCH_FIT_WHY_FIT == p_line->why)
    {
        bench_text("; ");
        bench_dec((int32_t)(p_line->cpu_us / 1000U));
        bench_text(" ms, ");
        bench_dec((int32_t)p_line->steps);
        bench_text(" steps, ");
        bench_dec((int32_t)(p_line->work / 1000U));
        bench_text("k tests");
        if (0U != p_line->joins)
        {
            bench_text(", cuts joined");
        }
    }
    return true;
}

/** @brief MISSION, at the start: no fit, no counts (the tables are kept). */
static void bench_fit_reset (void)
{
    (void)memset(&g_fit, 0, sizeof(g_fit));
    (void)memset(&g_fit_bin, 0, sizeof(g_fit_bin));
    g_fit_line_first = 0U;
    g_fit_line_count = 0U;
    g_fit_lines_lost = 0U;
    gp_fit_line_new = NULL;
}

/** @brief MISSION, after the stop: the NEW decoder's counts. */
static void bench_fit_totals (void)
{
    bench_text(" NEW decoder: ");
    bench_dec((int32_t)g_fit.reads);
    bench_text(" read, ");
    bench_dec((int32_t)g_fit.no_reads);
    bench_text(" no read, ");
    bench_dec((int32_t)g_fit.skipped);
    bench_text(" skipped, ");
    bench_dec((int32_t)g_fit_lines_lost);
    bench_text(" lines lost (margin needed ");
    bench_fixed((int32_t)(100.0f * BENCH_FIT_MARGIN), 2U);
    bench_text(")");
    bench_flush();
}

/* End of file. */
