/** @file mission_terrain.c
 * @brief The mission: subsystem 4: the hump (ramp), side leans and the lift
 * stop. Split out of the one file by tools/reorg/split_c.py (22 Nov 2026);
 * what the parts share is in mission_private.h.
 */
#include "mission_private.h"
#include "mission_terrain.h"

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
#define BENCH_RAMP_OFFSET_M (4.0 * ROBOT_ROBOT_LENGTH_M)

/* One wheel over the ramp rolls the car and barely pitches it (course
 * observation, 30 Oct): the bench also keeps the side height,
 * BENCH_RAMP_LEVER_M times sin(roll), + the left wheel up. A lean opens at
 * the module's minimum hump, closes under half of it, and counts only if it
 * lasted BENCH_RAMP_LEAN_M of path: a jolt passes in a few mm. The car's own
 * acceleration (encoder speed, filtered at the module's filter time) comes
 * off the forward axis first, so a speed change does not read as a slope.
 */
#define BENCH_RAMP_LEAN_M (ROBOT_WHEEL_RADIUS_M)

/* 9 Nov, on the track's one-sided 10 mm hump: the car read leans of 28 and
 * 31 mm in and just after a 90 deg corner (the heading swung 44 deg through
 * one, pitch +8 and +11 deg at both) and an 8 mm rise while searching for a
 * lost line. No challenge on the track is at a turn (9 Nov), so a rise
 * or lean that turned the car more than BENCH_RAMP_TURN_RAD while open is
 * left out, and so is a lean pitched beyond BENCH_RAMP_LEAN_PITCH_RAD at its
 * most: one wheel 10 mm up on the 0.12 m axle-to-castor base pitches the car
 * about 2.4 deg. Both are counted in the end report and each goes out live
 * as "SIDE left out (why)". The pitch limit was 6 deg until a crossing of
 * the real hump was left out by it (17:02, at 54 mm/s with encoder errors);
 * 10 still leaves out the +11 deg reading, and the +8 one was at a turn.
 */
/* 9 Nov: one wheel over the same 10 mm hump read 17 and 18 mm at the most
 * (roll -7.4, -7.9 deg against asin(10/130) = 4.4), with the car's own lean
 * already off (roll -0.3..+0.1 just before it). The car rolls more than
 * its wheel track predicts, so the single peak overstates a lean's height.
 * So a lean's height is BENCH_RAMP_LEVER_M x sin of the most tilted
 * mean roll over BENCH_RAMP_HOLD_M of path (the peak still goes out beside
 * it), and the lever is set from a run over the 1 cm hump.
 * [CALIBRATE] BENCH_RAMP_LEVER_M = 0.010 / sin(held roll on the 1 cm hump).
 * Calibrated 9 Nov 17:05-17:06, two clean passes (85-88 mm/s) over the 1 cm
 * hump, right wheel: 30 mm means -4.2 and -3.9 deg -> 10 and 9 mm with the
 * track as lever (single peaks -6.4, -6.6 deg -> 15 mm). The track works
 * as the lever, no shorter one: the excess was the spike as the wheel
 * climbs, which the mean takes out.
 * Recalibrated 19 Nov: three confirmed passes over the same 1 cm
 * hump (20:17 13 mm at -5.6 deg, 20:35 14 mm at -6.0, and 20:36 12 mm at
 * +5.3 on the way back after the U-turn) read 12-14 mm with the 130 mm
 * track: the car rolls more on it now. 0.010 / sin(5.63 deg) = 0.102 m puts
 * them at 10.0, 10.7 and 9.4 mm. Single peaks (16-22 mm) stay above that.
 */
#define BENCH_RAMP_LEVER_M (0.102)
#define BENCH_RAMP_HOLD_M (0.030)
#define BENCH_RAMP_TURN_RAD (20.0 * CORE_PI / 180.0)

/* 19 Nov: the hump is only ever on a straight, and the car is not to
 * steer over it, as over a barcode card. A rise or lean counts only if it
 * opened while the car followed a straight line (bench_ramp_on_straight:
 * the junctions' going-straight test, no go-around, junction move or
 * AUTO-CENTRE); one that did not is left out ("not on a straight"). While
 * such a hump is open the line step holds the steering straight
 * (bench_ramp_hold), as the card hold does: a front sensor on the line keeps
 * its correction, and a sustained bend lets go.
 */
#define BENCH_RAMP_LEAN_PITCH_RAD (10.0 * CORE_PI / 180.0)

/* No wheel-speed turn-rate gate (tried 14 Nov, removed the same day): a rise
 * or lean with a peak wheel-difference turn rate over 0.30 rad/s was left out
 * "at a turn", and it threw out the real 1 cm hump: right wheel up 11 mm,
 * roll -4.8 deg, pitch -3.8, while the heading held -1..0 deg. Only the
 * right wheel climbs (and its encoder glitches there), so the wheel speeds
 * part on a straight. The net-heading gate (BENCH_RAMP_TURN_RAD) and the
 * one-wheel rule (BENCH_RAMP_WHEEL_MIN_M) stay.
 */
/* A rise or lean counts only with one wheel, EITHER one, up by at least
 * this much: for a rise BENCH_RAMP_LEVER_M x sin(roll at its peak), for a
 * lean its height (bench_lean_height). Until 19 Nov it had to be the RIGHT
 * wheel (14 Nov), but after the junction U-turn the car drives the
 * track the other way and the same hump is under its LEFT wheel: the 20:36
 * pass (12 mm, roll +5.3) was left out as "left wheel up" (19 Nov:
 * the which-wheel rule is dropped). The 14 Nov ghosts read level (a 5 mm
 * rise at roll +1.3 deg, about 2 mm of side; a 2 mm rise at -0.8 deg), so a
 * level rise is still left out ("no wheel 3 mm up").
 */
#define BENCH_RAMP_WHEEL_MIN_M (0.003)

/* No hump in the first BENCH_RAMP_SETTLE_M of line following after START
 * (19 Nov: none is there). The car pulls away from AUTO-CENTRE's
 * pivots and its tilt reference, taken standing (pitch +3, roll -1.4 to
 * -2 deg on 19 Nov), is still settling to the driving one (about 0): the
 * 20:27 and 20:34 runs counted humps of 9, 6, 15 and 9 mm at 49-506 mm.
 * 22 Nov: the hump is at the start of the run (one challenge at a
 * time, the hump where card 1 would be), and it was never seen: the car
 * rolled a steady -3 to -5 deg, right wheel up, for 70-85 mm 116-187 mm and
 * 389-473 mm after START, inside the 0.6 m, and no lean opened (the sim
 * missed the 10 mm hump at 0.1-0.4 m too). So only the pull-away is left
 * out now; the 19 Nov readings at 49-506 mm may well have been this hump.
 */
#define BENCH_RAMP_SETTLE_M (0.05)

/* The car's own tilt (bench_ramp_offset) is the mean over the path so far,
 * up to BENCH_RAMP_OFFSET_M. 22 Nov: just after START that mean was mostly
 * the hump itself, so the hump took itself off (about a third, at 190 mm).
 * The mean now spans at least BENCH_RAMP_OFFSET_MIN_M from the start (the
 * still-second reference counts for the rest), and it learns nothing while
 * a side lean is open.
 */
#define BENCH_RAMP_OFFSET_MIN_M (0.2)

/* A lean or rise whose single peak puts one wheel more than this up (roll
 * over ~18 deg) is not the 1 cm hump: the car was lifted, knocked or the
 * accelerometer misread. The real hump's peaks: 12-20 mm (9 and 14 Nov);
 * 14 Nov 23:56 a lean peaking at 130 mm (roll -86 deg) while the car stood
 * at a junction counted as "SIDE 1: right wheel up 7 mm". Shown live as
 * "SIDE left out (lifted or knocked)".
 */
#define BENCH_RAMP_PEAK_MAX_M (0.040)

/* Lift stop (14 Nov: the IMU's collision detection is dropped; the
 * only requirement left is that lifting the robot at the end of the demo
 * stops the motors; no lift or overturn classification). MISSION, every
 * accelerometer sample (bench_lift_check): the height change only
 * (15 Nov). Gravity is followed slowly (BENCH_LIFT_GRAVITY_S), direction and
 * size, so the sensor's own offset is not taken as motion; the acceleration
 * along it less it, each sample held to +-BENCH_LIFT_CLAMP_MPS2, integrates
 * to an upward speed and that to a height, both leaking back to 0
 * (BENCH_LIFT_LEAK_S, BENCH_LIFT_HEIGHT_LEAK_S). Lifted once the height is
 * past BENCH_LIFT_HEIGHT_M: a demo's lift is 15 cm or more; a jolt on the
 * floor lasts too short to make centimetres. Then the run ends at once,
 * motors braked.
 * Earlier tries, both stopped untouched runs as reversing began (14-15 Nov):
 * the total acceleration over gravity (1.5 m/s^2 for 60 ms; raw peaks that
 * run +7.6 m/s^2), then the upward speed (0.2 m/s; read 0.23, all three as
 * the motors reversed into "backing up"). That jolt reads as a one-sided
 * upward speed that would integrate to ~12 cm, so for BENCH_LIFT_SETTLE_MS
 * after either motor changes direction or starts from rest the speed and
 * height are held at 0. A lift does not change the motors' direction.
 * Modelled offline: a 19 cm lift in 0.5 s reads 13.7 cm, a slow
 * 15 cm one 9.6 cm, a bump that comes back down 0.7 cm.
 * [CALIBRATE] from the end report's "lift:" line (the run's most height).
 */
#define BENCH_LIFT_GRAVITY_S (0.5)
#define BENCH_LIFT_LEAK_S (1.0)
#define BENCH_LIFT_HEIGHT_LEAK_S (2.0)
#define BENCH_LIFT_CLAMP_MPS2 (3.0)

/* Robot app (22 Nov: raised, as turns can trip it): 150 mm, about
 * twice the most height a run that was not lifted reached (77 mm, bench 19
 * Nov; 68 mm robot app 22 Nov). That night six 80 mm stops came with the car
 * on the floor (REC rows: line following, roll and pitch within 5 deg). A
 * hand lift's height grows fast, so it costs a lift only a fraction of a
 * second more. The end report's "lift:" line keeps the run's most height.
 */
#define BENCH_LIFT_HEIGHT_M (0.15)
#define BENCH_LIFT_SETTLE_MS (300U)

/* Gravity's SIZE is its own slow mean of each sample's size (not the
 * low-passed vector's length: vectors of mixed direction, a jolt or bad
 * samples, average to a shorter one, and every normal sample after then reads
 * as an upward push that integrates to centimetres; the sim's bad-readings
 * case did exactly that). Samples further than BENCH_LIFT_GSIZE_GATE_MPS2
 * from it are left out of it.
 */
#define BENCH_LIFT_GSIZE_S (10.0)

/* Both start from the still second before START (its mean vector); without
 * one, from the plain mean of the first this many samples (0.5 s), with no
 * integration before (a single first sample's error integrated to 50 mm in
 * the sim's plain run).
 */
#define BENCH_LIFT_START_N (50U)
#define BENCH_LIFT_GSIZE_GATE_MPS2 (1.0)
#define BENCH_LIFT_DUTY_ZERO (0.02) /* |duty| under this: that motor off */
#define BENCH_RAMP_WINDOW_US (200000U)  /* robot_app's speed window */
#define BENCH_MG_TO_MPS2 (0.00980665)

/* Why a SIDE lean was left out (g_ramp.p_lean_out points into this). */
#define BENCH_LEAN_OUTS (5U)
static const char * const g_lean_out_names[BENCH_LEAN_OUTS] =
{
    "at a turn", "not on a straight", "pitched", "lifted or knocked",
    "no wheel 3 mm up"
};

/** @brief MISSION start: the terrain module, configured as on the robot,
 * with a flat reference from the car's last idle second when it held still
 * (else the stored one), and the run's ramp figures cleared.
 */
void bench_ramp_start (void)
{
    terrain_config_t config;
    bench_accel_t    flat = g_ramp.flat;
    double           mg[3] = {0.0, 0.0, 0.0};
    double           axes[3] = {0.0, 0.0, 0.0};
    bool             b_still = (flat.count >= BENCH_RAMP_REF_MIN);
    uint32_t         axis;

    (void)memset(&g_ramp, 0, sizeof(g_ramp));
    g_ramp.flat = flat;
    for (axis = 0U; axis < ROBOT_IMU_AXES; ++axis)
    {
        b_still = b_still && (((int32_t)flat.maximum[axis] -
                               (int32_t)flat.minimum[axis]) <=
                              BENCH_RAMP_STILL_MG);
    }
    if (b_still)
    {
        for (axis = 0U; axis < ROBOT_IMU_AXES; ++axis)
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
    /* The lift stop's gravity, direction and size, from the same still
     * second (else from its first BENCH_LIFT_START_N samples).
     */
    if (b_still)
    {
        for (axis = 0U; axis < ROBOT_IMU_AXES; ++axis)
        {
            g_ramp.lift_g[axis] = axes[axis];
        }
        g_ramp.lift_gsize = hypot(axes[0], hypot(axes[1], axes[2]));
        g_ramp.lift_gn = BENCH_LIFT_START_N;
    }
    g_ramp.last_us = board_time_us();
    g_ramp.window_us = g_ramp.last_us;
    g_ramp.path = g_distance[BENCH_LEFT] + g_distance[BENCH_RIGHT];
    g_ramp.window_path = g_ramp.path;
    g_ramp.cell_path = g_ramp.path;
    g_ramp.cells = 1U;             /* the start, at level 0 */
    g_ramp.cell = 1U;
    if (!g_ramp.b_ready)
    {
        bench_line(BK_RAMP_OFF);
        bench_text(bench_flag(gb_accel) ?
                   "RAMP off: the reference was refused" :
                   "RAMP off: no accelerometer");
        bench_flush();
        return;
    }
    /* Frame: samples 0-1023, angles as bench_ramp_deg. */
    bench_line(BK_RAMP_FLAT);
    bench_text(bench_flag(b_still) ?
               "RAMP flat reference: the last still second (n " :
               "RAMP flat reference: stored (car not still, n ");
    bench_u(flat.count, 10U);
    bench_text("), start pitch ");
    bench_ramp_deg(atan2(axes[0], hypot(axes[1], axes[2])));
    bench_text(" roll ");
    bench_ramp_deg(atan2(axes[1], axes[2]));
    bench_text(" deg");
    bench_flush();
}

/** @brief Sensor mg to vehicle axes in m/s^2, as robot_app_imu_step. */
void bench_ramp_axes (const double * p_mg, double * p_mps2)
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

/** @brief MISSION, each accelerometer sample (axes in m/s^2, x forward,
 * y left, z up; dt_us since the last): the lift stop, on the height change
 * alone. Gravity low-passed (BENCH_LIFT_GRAVITY_S); the acceleration along
 * it less its size, clamped, integrated to an upward speed, that to a
 * height, both leaking; lifted past BENCH_LIFT_HEIGHT_M. Both are held at 0
 * for BENCH_LIFT_SETTLE_MS once a motor changes direction (p_output's
 * duties). Ends the run at once, motors braked. The run's most height and
 * speed are kept.
 */
void bench_lift_check (uint64_t now_us, const double * p_axes,
                              uint64_t dt_us,
                              const motion_output_t * p_output)
{
    const double gravity = g_ramp.state.config.gravity_mps2;
    double       dt = (double)((dt_us > 50000U) ? 50000U : dt_us) / 1000000.0;
    double       duty[2] = {p_output->left_duty, p_output->right_duty};
    double       g_norm;
    double       size;
    double       vertical;
    double       share;
    uint32_t     axis;
    uint32_t     wheel;
    int8_t       dir;

    if ((!isfinite(p_axes[0])) || (!isfinite(p_axes[1])) ||
        (!isfinite(p_axes[2])) || g_drive.b_done || (!g_drive.b_running))
    {
        return;
    }
    size = hypot(p_axes[0], hypot(p_axes[1], p_axes[2]));
    if (g_ramp.lift_gn < BENCH_LIFT_START_N)
    {
        /* No still second before START: a plain mean first, no lift yet. */
        ++g_ramp.lift_gn;
        for (axis = 0U; axis < ROBOT_IMU_AXES; ++axis)
        {
            g_ramp.lift_g[axis] += (p_axes[axis] - g_ramp.lift_g[axis]) /
                                   (double)g_ramp.lift_gn;
        }
        g_ramp.lift_gsize += (size - g_ramp.lift_gsize) /
                             (double)g_ramp.lift_gn;
        return;
    }
    share = dt / (BENCH_LIFT_GRAVITY_S + dt);
    for (axis = 0U; axis < ROBOT_IMU_AXES; ++axis)
    {
        g_ramp.lift_g[axis] += share * (p_axes[axis] - g_ramp.lift_g[axis]);
    }
    if (fabs(size - g_ramp.lift_gsize) < BENCH_LIFT_GSIZE_GATE_MPS2)
    {
        g_ramp.lift_gsize += (dt / (BENCH_LIFT_GSIZE_S + dt)) *
                             (size - g_ramp.lift_gsize);
    }
    g_norm = hypot(g_ramp.lift_g[0], hypot(g_ramp.lift_g[1], g_ramp.lift_g[2]));
    if ((!isfinite(g_norm)) || (g_norm < (0.5 * gravity)))
    {
        return;
    }
    /* Along gravity's direction, less gravity's measured size (not the
     * nominal: the sensor's own offset, rest ~9.74 m/s^2, would integrate to
     * centimetres; not the low-passed vector's length: see
     * BENCH_LIFT_GSIZE_S).
     */
    vertical = (((p_axes[0] * g_ramp.lift_g[0]) +
                 (p_axes[1] * g_ramp.lift_g[1]) +
                 (p_axes[2] * g_ramp.lift_g[2])) / g_norm) - g_ramp.lift_gsize;
    vertical = fmax(-BENCH_LIFT_CLAMP_MPS2,
                    fmin(BENCH_LIFT_CLAMP_MPS2, vertical));
    /* A motor changing direction (or starting) jolts the car: hold the
     * integration at 0 until that has passed.
     */
    for (wheel = 0U; wheel < BENCH_WHEELS; ++wheel)
    {
        dir = (duty[wheel] > BENCH_LIFT_DUTY_ZERO) ? (int8_t)1 :
              ((duty[wheel] < -BENCH_LIFT_DUTY_ZERO) ? (int8_t)-1 :
                                                       (int8_t)0);
        if ((dir != g_ramp.lift_dir[wheel]) && (0 != dir))
        {
            g_ramp.lift_settle_us = now_us +
                                    ((uint64_t)BENCH_LIFT_SETTLE_MS * 1000U);
            ++g_ramp.lift_settles;
        }
        g_ramp.lift_dir[wheel] = dir;
    }
    if (now_us < g_ramp.lift_settle_us)
    {
        g_ramp.lift_up = 0.0;
        g_ramp.lift_height = 0.0;
        return;
    }
    g_ramp.lift_up += (vertical * dt) -
                      (g_ramp.lift_up * (dt / BENCH_LIFT_LEAK_S));
    g_ramp.lift_height += (g_ramp.lift_up * dt) -
                          (g_ramp.lift_height *
                           (dt / BENCH_LIFT_HEIGHT_LEAK_S));
    g_ramp.lift_up_most = fmax(g_ramp.lift_up_most, g_ramp.lift_up);
    g_ramp.lift_height_most = fmax(g_ramp.lift_height_most,
                                   g_ramp.lift_height);
    if (g_ramp.lift_height <= BENCH_LIFT_HEIGHT_M)
    {
        return;
    }
    g_ramp.b_lifted = true;
    mission_motor_brake();
    bench_line(BK_LIFT_STOP);   /* 0-4095 mm */
    bench_text("LIFT: raised ");
    bench_u((uint32_t)bench_round(g_ramp.lift_height * 1000.0), 12U);
    bench_text(" mm: motors stopped");
    bench_avoid_say();
    bench_line_end(now_us, "stopped: lifted (IMU)");
}

/** @brief The time of the accelerometer sample just read (bench_ramp_step):
 * now_us on the bench, which reads the sensor itself. The robot app's IMU
 * task queues its samples, so each is taken at its own read time (22 Nov: at
 * the step's time, the lift and hump integrals were bunched after a long
 * step and lagged). A read before the last sample's time counts as none
 * elapsed.
 */
static uint64_t bench_ramp_time (uint64_t now_us)
{
    uint64_t sample_us = mission_accel_sample_us(now_us);

    return (sample_us < g_ramp.last_us) ? g_ramp.last_us : sample_us;
}

/** @brief MISSION, each drive step: a fresh accelerometer sample (100 Hz)
 * into the terrain module, then the bench hump on its pitch. The wheel path
 * since the last sample is the centre's, forward only (the car never
 * reverses); roll + is the left side up.
 */
void bench_ramp_step (uint64_t now_us,
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
        (BOARD_ACCEL_FRESH != mission_read_accel(milligravity)))
    {
        return;
    }
    now_us = bench_ramp_time(now_us);
    for (axis = 0U; axis < ROBOT_IMU_AXES; ++axis)
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
    bench_lift_check(now_us, axes, now_us - g_ramp.last_us, p_output);
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
void bench_ramp_lean (uint64_t now_us, double ds, int32_t centre_mm)
{
    const double minimum = g_ramp.state.config.minimum_hump_m;
    double       side = BENCH_RAMP_LEVER_M * sin(g_ramp.roll);
    double       sum = 0.0;
    uint32_t     count = 0U;
    uint32_t     index;
    bool         b_spans = false;

    if ((!g_ramp.b_leaning) && (fabs(side) >= minimum) &&
        (g_centre.path_m >= BENCH_RAMP_SETTLE_M))
    {
        g_ramp.b_leaning = true;
        (void)memset(&g_ramp.lean, 0, sizeof(g_ramp.lean));
        g_ramp.lean.from_mm = centre_mm;
        g_ramp.lean.heading = g_drive.heading;
        g_ramp.lean.b_straight = bench_ramp_on_straight();
        g_ramp.hold_count = 0U;
        g_ramp.hold_next = 0U;
    }
    if (!g_ramp.b_leaning)
    {
        return;
    }
    g_ramp.lean.length += ds;
    g_ramp.lean.turned = fmax(g_ramp.lean.turned,
                              fabs(bench_avoid_wrap(g_drive.heading -
                                                    g_ramp.lean.heading)));
    /* The mean side height over the last BENCH_RAMP_HOLD_M of path (all the
     * kept samples when they span less, at a crawl). */
    g_ramp.hold_side[g_ramp.hold_next] = side;
    g_ramp.hold_at[g_ramp.hold_next] = g_ramp.travelled;
    g_ramp.hold_next = (g_ramp.hold_next + 1U) % BENCH_RAMP_HOLD_SAMPLES;
    if (g_ramp.hold_count < BENCH_RAMP_HOLD_SAMPLES)
    {
        ++g_ramp.hold_count;
    }
    for (index = 0U; index < g_ramp.hold_count; ++index)
    {
        if ((g_ramp.travelled - g_ramp.hold_at[index]) <= BENCH_RAMP_HOLD_M)
        {
            sum += g_ramp.hold_side[index];
            ++count;
        }
        else
        {
            b_spans = true;
        }
    }
    if ((b_spans || (BENCH_RAMP_HOLD_SAMPLES == g_ramp.hold_count)) &&
        (0U != count) && (fabs(sum / (double)count) > fabs(g_ramp.lean.held)))
    {
        g_ramp.lean.held = sum / (double)count;
        g_ramp.lean.held_mm = centre_mm;
    }
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
    if ((!g_ramp.lean.b_straight) ||
        (g_ramp.lean.turned > BENCH_RAMP_TURN_RAD))
    {
        ++g_ramp.lean_turns;       /* At a turn: no hump is there */
        g_ramp.lean_sent = g_ramp.lean;
        g_ramp.p_lean_out = g_lean_out_names[g_ramp.lean.b_straight ? 0U : 1U];
        g_ramp.b_lean_send = true; /* Out, but shown */
        return;
    }
    if (fabs(g_ramp.lean.pitch) > BENCH_RAMP_LEAN_PITCH_RAD)
    {
        ++g_ramp.lean_steep;       /* Not one wheel on a hump */
        g_ramp.lean_sent = g_ramp.lean;
        g_ramp.p_lean_out = g_lean_out_names[2];
        g_ramp.b_lean_send = true;
        return;
    }
    if (fabs(g_ramp.lean.peak) > BENCH_RAMP_PEAK_MAX_M)
    {
        ++g_ramp.lean_wild;        /* Far beyond any hump on the track */
        g_ramp.lean_sent = g_ramp.lean;
        g_ramp.p_lean_out = g_lean_out_names[3];
        g_ramp.b_lean_send = true;
        return;
    }
    if (!bench_lean_wheel_up(&g_ramp.lean))
    {
        ++g_ramp.lean_level;       /* Level: no wheel on a hump */
        g_ramp.lean_sent = g_ramp.lean;
        g_ramp.p_lean_out = g_lean_out_names[4];
        g_ramp.b_lean_send = true;
        return;
    }
    ++g_ramp.leans;
    g_ramp.p_lean_out = NULL;
    g_ramp.lean_sent = g_ramp.lean;
    g_ramp.b_lean_send = true;
    if (fabs(bench_lean_height(&g_ramp.lean)) >
        fabs(bench_lean_height(&g_ramp.lean_best)))
    {
        g_ramp.lean_best = g_ramp.lean;
        g_ramp.best_lean = g_ramp.leans;
    }
}

/** @brief A lean's height: its most tilted BENCH_RAMP_HOLD_M mean (the peak
 * if it never spanned that), signed + left up.
 */
double bench_lean_height (const bench_lean_t * p_lean)
{
    return (0.0 != p_lean->held) ? p_lean->held : p_lean->peak;
}

/** @brief True if a lean has a wheel, either one, up by at least
 * BENCH_RAMP_WHEEL_MIN_M (heights are + left up).
 */
bool bench_lean_wheel_up (const bench_lean_t * p_lean)
{
    return fabs(bench_lean_height(p_lean)) >= BENCH_RAMP_WHEEL_MIN_M;
}

/** @brief True if a rise had a wheel, either one, up by at least
 * BENCH_RAMP_WHEEL_MIN_M at its peak (roll + left up).
 */
bool bench_rise_wheel_up (const bench_rise_t * p_rise)
{
    return fabs(BENCH_RAMP_LEVER_M * sin(p_rise->roll)) >=
           BENCH_RAMP_WHEEL_MIN_M;
}

/** @brief A lean as text (not flushed): side, height (the held mean, its
 * roll), the single peak beside it, where, how long.
 */
void bench_ramp_lean_line (const char * p_head,
                                  const bench_lean_t * p_lean)
{
    double height = bench_lean_height(p_lean);

    /* Frame: heights 0-511 mm, the path as the live line, time 0-6553.5 s,
     * the length 0-65535 mm. asin's argument held to +-1: a lifted car's
     * "lean" is far past the lever (it printed a NaN's garbage before).
     */
    bench_text(p_head);
    bench_text(bench_flag(height >= 0.0) ? "left wheel up " :
                                           "right wheel up ");
    bench_u((uint32_t)bench_round(fabs(height) * 1000.0), 9U);
    bench_text(" mm (30 mm mean, roll ");
    bench_ramp_deg(asin(fmax(-1.0, fmin(1.0, height / BENCH_RAMP_LEVER_M))));
    bench_text(" deg; peak ");
    bench_u((uint32_t)bench_round(fabs(p_lean->peak) * 1000.0), 9U);
    bench_text(" mm, roll ");
    bench_ramp_deg(asin(fmax(-1.0, fmin(1.0,
                                        p_lean->peak / BENCH_RAMP_LEVER_M))));
    bench_text(" deg, pitch ");
    bench_ramp_deg(p_lean->pitch);
    bench_text(") at ");
    bench_i((0.0 != p_lean->held) ? p_lean->held_mm : p_lean->peak_mm, 21U);
    bench_text(" mm, t ");
    bench_ux((uint32_t)p_lean->peak_ds, 1U, 16U);
    bench_text(" s, over ");
    bench_u((uint32_t)bench_round(p_lean->length * 1000.0), 16U);
    bench_text(" mm");
}

/** @brief The car's own pitch and roll offset, their mean over the path so
 * far up to the last BENCH_RAMP_OFFSET_M, taken off both: a START on a
 * tilted spot, the stored reference when the car was moving there, a mount
 * that has shifted (30 Oct: +2.4 deg nose up against the stored floor, which
 * the old hump integrated into 63 mm). A ramp tilts the car up, then as
 * much down, so it barely moves the mean.
 */
void bench_ramp_offset (double ds)
{
    double share = 0.0;

    if (ds > 0.0)
    {
        g_ramp.travelled += ds;
        share = ds / fmin(fmax(g_ramp.travelled, BENCH_RAMP_OFFSET_MIN_M),
                          BENCH_RAMP_OFFSET_M);
        g_ramp.pitch_offset += share * (g_ramp.raw_pitch -
                                        g_ramp.pitch_offset);
        if (!g_ramp.b_leaning)
        {
            /* The side lean's roll is not the car's own (22 Nov). */
            g_ramp.roll_offset += share * (g_ramp.raw_roll -
                                           g_ramp.roll_offset);
        }
    }
    g_ramp.pitch = g_ramp.raw_pitch - g_ramp.pitch_offset;
    g_ramp.roll = g_ramp.raw_roll - g_ramp.roll_offset;
}

/** @brief MISSION, each accelerometer sample: the level above the floor
 * behind, and the rise it makes.
 */
void bench_ramp_rise (uint64_t now_us, int32_t path, int32_t centre_mm)
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
    if ((!g_ramp.b_rising) && (g_ramp.above >= minimum) &&
        (g_centre.path_m >= BENCH_RAMP_SETTLE_M))
    {
        g_ramp.b_rising = true;
        (void)memset(&g_ramp.rise, 0, sizeof(g_ramp.rise));
        g_ramp.rise.from_mm = centre_mm;
        g_ramp.rise.up = g_ramp.pitch;
        g_ramp.rise.down = g_ramp.pitch;
        g_ramp.rise.heading = g_drive.heading;
        g_ramp.rise.b_straight = bench_ramp_on_straight();
    }
    if (!g_ramp.b_rising)
    {
        return;
    }
    g_ramp.rise.turned = fmax(g_ramp.rise.turned,
                              fabs(bench_avoid_wrap(g_drive.heading -
                                                    g_ramp.rise.heading)));
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
        if ((!g_ramp.rise.b_straight) ||
            (g_ramp.rise.turned > BENCH_RAMP_TURN_RAD))
        {
            ++g_ramp.rise_turns;   /* At a turn: no hump is there */
            return;
        }
        if (fabs(ROBOT_TRACK_WIDTH_M * sin(g_ramp.rise.roll)) >
            BENCH_RAMP_PEAK_MAX_M)
        {
            ++g_ramp.lean_wild;    /* Far beyond any hump on the track */
            return;
        }
        if (!bench_rise_wheel_up(&g_ramp.rise))
        {
            ++g_ramp.rise_level;   /* Level: no wheel on a hump */
            return;
        }
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

/** @brief MISSION: the hump is only ever on a straight (19 Nov). True
 * while the car follows the line (no go-around, junction move or
 * AUTO-CENTRE) and is going straight: the wheels' heading turned less than
 * BENCH_STRAIGHT_RAD over the last BENCH_STRAIGHT_SLOTS x
 * BENCH_STRAIGHT_STEP_M of travel, the junctions' test (bench_centre_straight,
 * kept by the last line step). A rise or lean that opens otherwise is left
 * out ("not on a straight").
 */
bool bench_ramp_on_straight (void)
{
    return (BENCH_MODE_MISSION == g_drive.mode) && g_drive.b_running &&
           (!g_drive.b_done) && (!g_auto.b_active) &&
           (BENCH_AVOID_STRAIGHT == g_avoid.phase) &&
           ((uint8_t)BENCH_NAV_IDLE == g_nav.phase) && g_centre.b_straight;
}

/** @brief MISSION: a hump is open that began on a straight, so the line
 * step holds the steering straight over it, as over a card (19 Nov:
 * held straight the same way as over a card; bench_line_step).
 */
bool bench_ramp_hold (void)
{
    /* Only once it looks like the hump: a lean that has lasted
     * BENCH_RAMP_LEAN_M (a jolt passes in a few mm), or a rise with a wheel
     * up now. Any open rise or lean held it at first: 40 holds, 762 mm, in
     * the 20:34 run with 2 hump passes (19 Nov).
     */
    return (g_ramp.b_rising && g_ramp.rise.b_straight &&
            (fabs(BENCH_RAMP_LEVER_M * sin(g_ramp.roll)) >=
             BENCH_RAMP_WHEEL_MIN_M)) ||
           (g_ramp.b_leaning && g_ramp.lean.b_straight &&
            (g_ramp.lean.length >= BENCH_RAMP_LEAN_M));
}

/** @brief MISSION live: a closed hump, the bench's or the module's. */
bool bench_ramp_text (void)
{
    /* Frame: humps 0-1023 a run (a 1 h run); heights +-511 mm (10 bits);
     * centre path
     * +-1048575 mm (21 bits); time 0-6553.5 s; angles as bench_ramp_deg.
     */
    if (g_ramp.b_send)
    {
        g_ramp.b_send = false;
        bench_line(BK_RAMP);
        bench_text(" RAMP ");
        bench_u(g_ramp.rises, 10U);
        bench_text(": peak ");
        bench_i(bench_round(g_ramp.sent.peak * 1000.0), 10U);
        bench_text(" mm at ");
        bench_i(g_ramp.sent.peak_mm, 21U);
        bench_text(" mm (t ");
        bench_ux((uint32_t)g_ramp.sent.peak_ds, 1U, 16U);
        bench_text(" s), roll ");
        bench_ramp_deg(g_ramp.sent.roll);
        bench_ramp_side(g_ramp.sent.roll);
        bench_text("; pitch up ");
        bench_ramp_deg(g_ramp.sent.up);
        bench_text(" down ");
        bench_ramp_deg(g_ramp.sent.down);
        bench_text(" deg; from ");
        bench_i(g_ramp.sent.from_mm, 21U);
        bench_text(" mm");
        return true;
    }
    if (g_ramp.b_lean_send)
    {
        g_ramp.b_lean_send = false;
        bench_line(BK_SIDE);
        if (bench_flag(NULL != g_ramp.p_lean_out))  /* shown, not counted */
        {
            uint32_t why = 0U;

            while (((why + 1U) < BENCH_LEAN_OUTS) &&
                   (g_lean_out_names[why] != g_ramp.p_lean_out))
            {
                ++why;
            }
            bench_text(" SIDE left out (");
            bench_name(g_lean_out_names, why, 3U);
            bench_ramp_lean_line("): ", &g_ramp.lean_sent);
            return true;
        }
        bench_text(" SIDE ");
        bench_u(g_ramp.leans, 10U);
        bench_ramp_lean_line(": ", &g_ramp.lean_sent);
        return true;
    }
    if (g_ramp.b_terrain)
    {
        g_ramp.b_terrain = false;
        bench_line(BK_TERRAIN);
        bench_text(bench_flag(g_ramp.b_terrain_valid) ?
                   " TERRAIN hump counted: peak " :
                   " TERRAIN hump not counted (turn, jolt or not level at"
                   " the end): peak ");
        bench_i(bench_round(g_ramp.terrain_peak * 1000.0), 10U);
        bench_text(" mm");
        return true;
    }
    return false;
}

/** @brief MISSION status line: pitch, roll, and the height in a hump. */
void bench_ramp_status (void)
{
    /* Angles as bench_ramp_deg (12 bits), height +-511 mm (10 bits). */
    if (!bench_flag(g_ramp.b_ready))
    {
        return;
    }
    bench_text(" pitch ");
    bench_ramp_deg(g_ramp.pitch);
    bench_text(" roll ");
    bench_ramp_deg(g_ramp.roll);
    bench_text(" h ");
    bench_i(bench_round(g_ramp.above * 1000.0), 10U);
}

/** @brief An angle in degrees, signed, one decimal. */
void bench_ramp_deg (double rad)
{
    int32_t tenths = bench_round(rad * (1800.0 / CORE_PI));

    /* Frame: +-204.7 deg in tenths, 12 bits (an IMU angle is +-180). */
    if (tenths >= 0)
    {
        bench_text("+");
    }
    bench_x(tenths, 1U, 12U);
}

/** @brief Which wheel is higher at this roll, and by how much. */
void bench_ramp_side (double roll)
{
    int32_t mm = bench_round(fabs(sin(roll)) * ROBOT_TRACK_WIDTH_M * 1000.0);

    /* Frame: the side, then 0-255 mm (the track is 0.12 m wide). */
    bench_text(bench_flag(roll >= 0.0) ? " deg (left wheel " :
                                         " deg (right wheel ");
    bench_u((uint32_t)mm, 8U);
    bench_text(" mm higher)");
}

/** @brief MISSION, after the stop: humps not yet sent, then the run's
 * highest closed hump, its side, and the most roll. A rise still open at
 * the stop never came back down, so it is shown apart and not counted
 * (14 Nov: one opened 1.8 s before STOP read as "highest 12 mm", level
 * roll, no hump there). A lean still open counts only past the same tests
 * as a closed one.
 */
void bench_ramp_results (void)
{
    /* Frames: run counts 0-1023 (a 1 h run, as the live lines); heights
     * +-511 mm; centre path +-1048575 mm (21 bits); time 0-6553.5 s; held
     * over humps 0-65535 mm; lift height and speed 0-4095 mm, mm/s; motor
     * direction changes 0-65535; IMU samples 0-524287 (19 bits, over an
     * hour at 100 Hz); angles as bench_ramp_deg.
     */
    if (!g_ramp.b_ready)
    {
        return;
    }
    while (bench_ramp_text())
    {
        bench_flush();
    }
    bench_line(BK_RAMP_HIGHEST);
    bench_text(" ramp: highest ");
    bench_i(bench_round(g_ramp.best.peak * 1000.0), 10U);
    bench_text(" mm");
    if (bench_flag(g_ramp.best.peak > 0.0))
    {
        bench_text(" (rise ");
        bench_u(g_ramp.best_rise, 10U);
        bench_text(") at ");
        bench_i(g_ramp.best.peak_mm, 21U);
        bench_text(" mm, t ");
        bench_ux((uint32_t)g_ramp.best.peak_ds, 1U, 16U);
        bench_text(" s, roll ");
        bench_ramp_deg(g_ramp.best.roll);
        bench_ramp_side(g_ramp.best.roll);
    }
    bench_flush();
    if (g_ramp.b_rising && (g_ramp.rise.peak > 0.0))
    {
        bench_line(BK_RAMP_OPEN);
        bench_text(" ramp: open at the stop, not counted: peak ");
        bench_i(bench_round(g_ramp.rise.peak * 1000.0), 10U);
        bench_text(" mm at ");
        bench_i(g_ramp.rise.peak_mm, 21U);
        bench_text(" mm, t ");
        bench_ux((uint32_t)g_ramp.rise.peak_ds, 1U, 16U);
        bench_text(" s, roll ");
        bench_ramp_deg(g_ramp.rise.roll);
        bench_ramp_side(g_ramp.rise.roll);
        bench_flush();
    }
    bench_line(BK_RAMP_RISES);
    bench_text(" ramp: rises ");
    bench_u(g_ramp.rises, 10U);
    bench_text(" of 5 mm or more, robot module ");
    bench_u((uint32_t)g_ramp.output.completed_humps, 10U);
    bench_text(" counted, max ");
    bench_u((uint32_t)terrain_height_mm(g_ramp.output.run_maximum_m), 9U);
    bench_text(" mm; car's own offset pitch ");
    bench_ramp_deg(g_ramp.pitch_offset);
    bench_text(" roll ");
    bench_ramp_deg(g_ramp.roll_offset);
    bench_text(" deg");
    bench_flush();
    if (g_ramp.b_leaning && (g_ramp.lean.length >= BENCH_RAMP_LEAN_M) &&
        (fabs(bench_lean_height(&g_ramp.lean)) >
         fabs(bench_lean_height(&g_ramp.lean_best))) &&
        g_ramp.lean.b_straight &&
        (g_ramp.lean.turned <= BENCH_RAMP_TURN_RAD) &&
        (fabs(g_ramp.lean.pitch) <= BENCH_RAMP_LEAN_PITCH_RAD) &&
        (fabs(g_ramp.lean.peak) <= BENCH_RAMP_PEAK_MAX_M) &&
        bench_lean_wheel_up(&g_ramp.lean))
    {
        g_ramp.lean_best = g_ramp.lean;
        g_ramp.best_lean = g_ramp.leans + 1U;
    }
    bench_line(BK_RAMP_LEANS);
    if (bench_flag(0U != g_ramp.best_lean))
    {
        bench_ramp_lean_line(" ramp: most side lean ", &g_ramp.lean_best);
    }
    else
    {
        bench_text(" ramp: no side lean of 5 mm");
    }
    bench_text("; leans ");
    bench_u(g_ramp.leans, 10U);
    bench_text(", jolts ");
    bench_u(g_ramp.jolts, 12U);   /* 73 in 2 min on the car */
    bench_flush();
    bench_line(BK_RAMP_HELD);
    bench_text(" ramp: held straight over humps ");
    bench_u(g_ramp.hump_holds, 10U);
    bench_text(" times, ");
    bench_u((uint32_t)g_ramp.hump_held_mm, 16U);
    bench_text(" mm, let go for a bend ");
    bench_u(g_ramp.hump_let_gos, 10U);
    bench_flush();
    bench_line(BK_RAMP_OUT);
    bench_text(" ramp: left out at a turn or not on a straight: rises ");
    bench_u(g_ramp.rise_turns, 10U);
    bench_text(", leans ");
    bench_u(g_ramp.lean_turns, 10U);
    bench_text("; leans pitched over 10 deg ");
    bench_u(g_ramp.lean_steep, 10U);
    bench_text("; no wheel 3 mm up (level): rises ");
    bench_u(g_ramp.rise_level, 10U);
    bench_text(", leans ");
    bench_u(g_ramp.lean_level, 10U);
    bench_flush();
    bench_line(BK_RAMP_WILD);
    bench_text(" ramp: left out as lifted or knocked (over ");
    bench_u((uint32_t)bench_round(BENCH_RAMP_PEAK_MAX_M * 1000.0), 9U);
    bench_text(" mm up): ");
    bench_u(g_ramp.lean_wild, 10U);
    bench_flush();
    bench_line(BK_LIFT);
    bench_text(bench_flag(g_ramp.b_lifted) ?
               " lift: STOPPED the run; most height " :
               " lift: not lifted; most height ");
    bench_u((uint32_t)bench_round(g_ramp.lift_height_most * 1000.0), 12U);
    bench_text(" mm (stop at ");
    bench_u((uint32_t)bench_round(BENCH_LIFT_HEIGHT_M * 1000.0), 12U);
    bench_text("), most upward speed ");
    bench_u((uint32_t)bench_round(g_ramp.lift_up_most * 1000.0), 12U);
    bench_text(" mm/s; motor direction changes ");
    bench_u(g_ramp.lift_settles, 16U);
    bench_flush();
    bench_line(BK_RAMP_ROLL);
    bench_text(" ramp: most roll left ");
    bench_ramp_deg(g_ramp.left);
    bench_text(" at ");
    bench_i(g_ramp.left_mm, 21U);
    bench_text(" mm, right ");
    bench_ramp_deg(g_ramp.right);
    bench_text(" at ");
    bench_i(g_ramp.right_mm, 21U);
    bench_text(" mm; samples ");
    bench_u(g_ramp.samples, 19U);
    bench_text(", no tilt ");
    bench_u(g_ramp.refused, 19U);
    bench_flush();
}

/** @brief MISSION: true when holding straight over a card would take the car
 * off the line, by LINE smooth's own tests (bench_centre_steer).
 * - Before the hold starts, only on a straight: a sustained bend
 *   (g_centre.turn, BENCH_SMOOTH_BEND) or a search lets go at once. On 2 Nov
 *   the hold zeroed the steering in bends and searches (5 holds, about
 *   330 mm, in 0.9 m runs) and the car lost the line. A front sensor on the
 *   line (seen_side) no longer stops it: the hold keeps FOLLOW's correction
 *   for it (4 Nov: the car zig-zags on straights, so a sensor is often on
 *   the line as a card starts, and then it never held at all).
 * - While holding (bench_line_step keeps the steering at 0 while the line is
 *   between the front sensors and lets a sensor that reaches the line steer
 *   back, as FOLLOW does): only a sustained bend built up by those
 *   corrections (g_centre.turn) lets go. The gap timer's SEARCH does not:
 *   both sensors white is the line still between them (4 Nov: every hold let
 *   go after 5-48 mm, at the 50 mm gap timeout, and the search steering
 *   swung the side sensor across the rest of the card).
 */
bool bench_hold_lets_go (void)
{
    if (g_code.b_held)
    {
        return fabs(g_centre.turn) >= BENCH_SMOOTH_BEND;
    }
    return (fabs(g_centre.turn) >= BENCH_SMOOTH_BEND) ||
           ((uint8_t)BENCH_PHASE_SEARCH == g_centre.phase);
}

/** @brief IMU task: one fresh accelerometer sample into the ring (the
 * oldest goes if it is full).
 */
void mission_accel_push (const int16_t milligravity[3], uint32_t read_us)
{
    mission_accel_t sample;
    uint32_t        saved;
    uint32_t        next;

    sample.x = (uint32_t)(uint16_t)milligravity[0];
    sample.y = (uint32_t)(uint16_t)milligravity[1];
    sample.z = (uint32_t)(uint16_t)milligravity[2];
    sample.read_us = read_us;
    saved = app_main_lock();
    next = (g_acc_head + 1U) % MISSION_ACCEL_RING;
    if (next == g_acc_tail)
    {
        g_acc_tail = (uint8_t)((g_acc_tail + 1U) % MISSION_ACCEL_RING);
    }
    g_acc_ring[g_acc_head] = sample;
    g_acc_head = (uint8_t)next;
    app_main_unlock(saved);
}

/* End of file. */

