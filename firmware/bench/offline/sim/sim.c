/* Host harness: the copy's bench_main.c in this translation unit, driven
 * tick by tick (bench_drive_tick) against a simulated car on a track patch:
 * an 18 mm line along x, optional crossing line and Code 39 bars beside the
 * line on the car's left. Wheels: duty to speed with static and kinetic
 * thresholds (about 8.2 mm/s per 1% duty, S2-U07) and a 60 ms lag; the
 * robot's own motion.c PID, line_ctrl.c and app_cfg.c run unchanged.
 */
#include "bench_main.c"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#define SIM_DT (0.007)
#define SIM_LINE_HALF (0.009)
#define SIM_BARS_FROM (0.029)   /* 20 mm beyond the line edge */
#define SIM_BARS_TO (0.200)

typedef struct
{
    double ahead;      /* front sensors ahead of the axle, m */
    double spacing;    /* front lens centres apart, m */
    double spot;       /* sensor spot, m square */
    double b_ahead;    /* side sensor ahead of the axle */
    double b_left;     /* side sensor left of the car's centre line */
    double b_spot;     /* side sensor spot, m square */
    double card;       /* bar darkness, share of the tape's */
    double speck;      /* white fleck mid-bar, m wide (0 none) */
    bool   b_line;
    bool   b_cross;
    double cross_x;
    bool   b_bars;
    bool   b_stuck;
    double scrub;      /* true yaw / wheel-difference yaw */
    double d_static;   /* duty that starts a wheel */
    double d_kinetic;  /* duty that keeps it turning; speed above it */
    double noise;      /* raw ADC noise, +/- */
    double coast_tau;  /* wheel run-down at zero duty, s */
    bool   b_sbend;    /* the line is the S-bend path (sbend_build) */
} world_t;

static world_t  W;
static double   g_sim_t;
static double   g_x, g_y, g_th;
static double   g_wv[2], g_wduty[2], g_wcount[2];
static bool     gb_brake;
static bool     gb_quiet;
static uint32_t g_rng = 12345U;
static double   g_bar_edge[200];
static int      g_bar_black[200];
static int      g_bars;

/* ---- kernel, board and network stand-ins ---- */
ER tk_dly_tsk (RELTIM dlytim)
{
    (void)dlytim;
    return 0;
}
ID tk_cre_cyc (const T_CCYC * pk_ccyc) { (void)pk_ccyc; return 1; }
ER tk_sta_cyc (ID cycid) { (void)cycid; return 0; }
ER tk_stp_cyc (ID cycid) { (void)cycid; return 0; }
ID tk_get_tid (void) { return 1; }
ER tk_wup_tsk (ID tskid) { (void)tskid; return 0; }
ER tk_slp_tsk (TMO tmout) { (void)tmout; return 0; }
INT tk_can_wup (ID tskid) { (void)tskid; return 0; }
/* The tasks are never started here (no bench_loop): no object exists. */
ID tk_cre_tsk (const T_CTSK * p) { (void)p; return 0; }
ER tk_sta_tsk (ID id, INT stacd) { (void)id; (void)stacd; return E_TMOUT; }
ID tk_cre_flg (const T_CFLG * p) { (void)p; return 0; }
ER tk_set_flg (ID id, UINT ptn) { (void)id; (void)ptn; return E_TMOUT; }
ER tk_wai_flg (ID id, UINT ptn, UINT mode, UINT * p_ptn, TMO tmo)
{
    (void)id; (void)ptn; (void)mode; (void)tmo;
    *p_ptn = 0U;
    return E_TMOUT;
}
ER tk_ref_flg (ID id, T_RFLG * p) { (void)id; p->flgptn = 0U; return E_TMOUT; }
ID tk_cre_mbf (const T_CMBF * p) { (void)p; return 0; }
ER tk_snd_mbf (ID id, const void * m, INT n, TMO tmo)
{
    (void)id; (void)m; (void)n; (void)tmo;
    return E_TMOUT;
}
INT tk_rcv_mbf (ID id, void * m, TMO tmo)
{
    (void)id; (void)m; (void)tmo;
    return E_TMOUT;
}
ER tk_ref_mbf (ID id, T_RMBF * p) { (void)id; (void)p; return E_TMOUT; }
ID tk_cre_mtx (const T_CMTX * p) { (void)p; return 0; }
ER tk_loc_mtx (ID id, TMO tmo) { (void)id; (void)tmo; return E_TMOUT; }
ER tk_unl_mtx (ID id) { (void)id; return E_TMOUT; }
ID tk_cre_sem (const T_CSEM * p) { (void)p; return 0; }
ER tk_sig_sem (ID id, INT n) { (void)id; (void)n; return E_TMOUT; }
ER tk_wai_sem (ID id, INT n, TMO tmo)
{
    (void)id; (void)n; (void)tmo;
    return E_TMOUT;
}

INT tm_printf (const UB * format, ...)
{
    va_list ap;
    int     n = 0;

    if (!gb_quiet)
    {
        va_start(ap, format);
        n = vprintf((const char *)format, ap);
        va_end(ap);
    }
    return n;
}

bool board_watchdog_start (uint32_t timeout_ms) { (void)timeout_ms; return true; }
void board_watchdog_feed (void) {}
bool board_init (void) { return true; }
const char * board_init_diagnostic (void) { return "sim"; }
void board_console_restore (void) {}
uint64_t board_time_us (void)
{
    return 1000000ULL + (uint64_t)(g_sim_t * 1000000.0 + 0.5);
}
void board_read_encoders (int32_t * p_left, int32_t * p_right,
                          uint32_t * p_errors)
{
    *p_left = 0;
    *p_right = 0;
    *p_errors = 0U;
}
void board_encoders_discard (void) {}
void board_encoder_totals (uint16_t * p_left, uint16_t * p_right)
{
    *p_left = 0U;
    *p_right = 0U;
}
/* Sim ramp on the centre path: up LEN at a steady slope to H, TOP flat,
 * down LEN; ROLL deg (+ left up) while on it. 100 Hz like the chip. */
static double   g_hump_from = 1.0e9;
static double   g_hump_roll;
static uint64_t g_accel_us;
static double sim_noise (void);
static double g_hump_h, g_hump_len, g_hump_top;
/* Ground height at rel metres past the ramp's foot: up, top, down. */
static double sim_ground (double rel)
{
    double top = g_hump_len + g_hump_top;

    if ((rel <= 0.0) || (rel >= (top + g_hump_len)))
    {
        return 0.0;
    }
    if (rel < g_hump_len)
    {
        return g_hump_h * rel / g_hump_len;
    }
    if (rel < top)
    {
        return g_hump_h;
    }
    return g_hump_h * (top + g_hump_len - rel) / g_hump_len;
}
board_accel_status_t board_read_accel (int16_t milligravity[3])
{
    double m = (2.0 * CORE_PI * ROBOT_WHEEL_RADIUS_M) /
               (double)ROBOT_ENCODER_COUNTS_PER_REV;
    double s = 0.5 * (double)(g_distance[0] + g_distance[1]) * m;
    double slope = asin(fmin(1.0, g_hump_h / g_hump_len));
    double th = 0.0;
    double ph = 0.0;
    double fwd, left, up;
    double rel = s - g_hump_from;
    uint64_t now = board_time_us();

    if ((now - g_accel_us) < 10000U)
    {
        return BOARD_ACCEL_PENDING;
    }
    g_accel_us = now;
    (void)slope;
    th = atan((sim_ground(rel) - sim_ground(rel - 0.12)) / 0.12) +
         (0.2 * CORE_PI / 180.0) +
         ((getenv("SIM_PITCH_OFF") ? atof(getenv("SIM_PITCH_OFF")) : 0.0) *
          CORE_PI / 180.0);
    if ((now % 700000U) < 30000U)
    {
        th += ((now / 700000U) % 2U) ? 0.17 : -0.17;
    }
    if ((rel >= 0.0) && (rel < (2.0 * g_hump_len + g_hump_top)))
    {
        ph = g_hump_roll * CORE_PI / 180.0;
    }
    ph += (getenv("SIM_ROLL_OFF") ? atof(getenv("SIM_ROLL_OFF")) : 0.0) *
          CORE_PI / 180.0;
    fwd = 1000.0 * sin(th) + ROBOT_IMU_REFERENCE_X_MG + 6.0 * sim_noise() / W.noise;
    left = 1000.0 * cos(th) * sin(ph) + ROBOT_IMU_REFERENCE_Y_MG +
           6.0 * sim_noise() / W.noise;
    up = 1000.0 * cos(th) * cos(ph) + 6.0 * sim_noise() / W.noise;
    /* SIM_LIFT_AT s: a pick-up, +300 mg up for 200 ms (the lift stop). */
    if ((NULL != getenv("SIM_LIFT_AT")) &&
        (g_sim_t >= atof(getenv("SIM_LIFT_AT"))) &&
        (g_sim_t < (atof(getenv("SIM_LIFT_AT")) + 0.2)))
    {
        up += 300.0;
    }
    /* SIM_JERK_AT s: a motor jolt, +700 mg forward for 60 ms (not a lift). */
    if ((NULL != getenv("SIM_JERK_AT")) &&
        (g_sim_t >= atof(getenv("SIM_JERK_AT"))) &&
        (g_sim_t < (atof(getenv("SIM_JERK_AT")) + 0.06)))
    {
        fwd += 700.0;
    }
    /* SIM_SHAKE_AT s: a reversal's shake, zero-mean +-1.5 g forward and
     * +-0.8 g up, alternating every 10 ms sample, for 150 ms (not a lift). */
    if ((NULL != getenv("SIM_SHAKE_AT")) &&
        (g_sim_t >= atof(getenv("SIM_SHAKE_AT"))) &&
        (g_sim_t < (atof(getenv("SIM_SHAKE_AT")) + 0.15)))
    {
        double sign = (((now / 10000U) % 2U) != 0U) ? 1.0 : -1.0;

        fwd += sign * 1500.0;
        up += sign * 800.0;
    }
    /* SIM_GLITCH_AT s: bad readings, the car "on its side" for 30 ms. */
    if ((NULL != getenv("SIM_GLITCH_AT")) &&
        (g_sim_t >= atof(getenv("SIM_GLITCH_AT"))) &&
        (g_sim_t < (atof(getenv("SIM_GLITCH_AT")) + 0.03)))
    {
        left = -1000.0;
        up = 60.0;
    }
    /* vehicle -> fitted sensor (forward -y, left +x, up +z) -> raw */
    milligravity[0] = (int16_t)lround(left / ROBOT_ACCEL_SCALE_X +
                                      ROBOT_ACCEL_BIAS_X_MG);
    milligravity[1] = (int16_t)lround(-fwd / ROBOT_ACCEL_SCALE_Y +
                                      ROBOT_ACCEL_BIAS_Y_MG);
    milligravity[2] = (int16_t)lround(up / ROBOT_ACCEL_SCALE_Z +
                                      ROBOT_ACCEL_BIAS_Z_MG);
    return BOARD_ACCEL_FRESH;
}
bool board_ultrasonic_trigger (void) { return false; }
void board_ultrasonic_service (void) {}
void board_ultrasonic_notify (void (* p_notify)(void)) { (void)p_notify; }
bool board_ultrasonic_result (uint32_t * p_pulse_us, bool * p_valid)
{
    *p_pulse_us = 0U;
    *p_valid = false;
    return false;
}
bool board_servo_us (uint16_t pulse_us) { (void)pulse_us; return true; }
void board_motor_duty (core_real_t left_duty, core_real_t right_duty)
{
    g_wduty[0] = left_duty;
    g_wduty[1] = right_duty;
    gb_brake = false;
}
void board_motor_brake (void)
{
    g_wduty[0] = 0.0;
    g_wduty[1] = 0.0;
    gb_brake = true;
}
bool board_stop_pressed (void) { return false; }
bool board_start_pressed (void) { return false; }
/* Barcode D0 not simulated: always low, so no D0 groups. */
bool board_barcode_d0 (void) { return false; }

/* ---- world ---- */
static double sim_noise (void)
{
    g_rng = (g_rng * 1103515245U) + 12345U;
    return ((double)((g_rng >> 8) % 2001U) / 1000.0 - 1.0) * W.noise;
}

static void bars_build (const char * p_text, double narrow, double wide,
                        double x0)
{
    double pos = x0;
    int    k = 0;
    size_t i;
    int    e;
    int    idx;

    for (i = 0U; '\0' != p_text[i]; ++i)
    {
        idx = 0;
        while (g_code39_chars[idx] != p_text[i])
        {
            ++idx;
        }
        if (0U != i)
        {
            g_bar_edge[k] = pos;
            g_bar_black[k++] = 0;
            pos += narrow;
        }
        for (e = 0; e < 9; ++e)
        {
            g_bar_edge[k] = pos;
            g_bar_black[k++] = !(e & 1);
            pos += (0U != (g_code39_patterns[idx] & (0x100U >> e))) ? wide :
                                                                       narrow;
        }
    }
    g_bar_edge[k] = pos;
    g_bar_black[k] = 0;
    g_bars = k;
}

/* S-bend (track half B, 19 Nov): points every 2 mm along the line's centre:
 * straight on x, a 90 deg right bend, a straight, a 90 deg left bend, then
 * straight on again.
 */
#define SIM_SB_STEP (0.002)
#define SIM_SB_MAX (6000)
static double g_sb_x[SIM_SB_MAX];
static double g_sb_y[SIM_SB_MAX];
static int    g_sb_n;

static void sbend_leg (double * p_x, double * p_y, double * p_th,
                       double length, double curvature)
{
    double s;

    for (s = 0.0; (s < length) && (g_sb_n < SIM_SB_MAX); s += SIM_SB_STEP)
    {
        g_sb_x[g_sb_n] = *p_x;
        g_sb_y[g_sb_n] = *p_y;
        ++g_sb_n;
        *p_th += curvature * SIM_SB_STEP;
        *p_x += SIM_SB_STEP * cos(*p_th);
        *p_y += SIM_SB_STEP * sin(*p_th);
    }
}

/** The dome (track, 19 Nov 19:30 sketch; the car lost the line at its second
 * bend four runs running): from x = -0.3, 0.7 m straight, a 70 deg LEFT bend
 * of radius r1, 350 mm up the diagonal, a RIGHT kink of `kink` deg (radius
 * r1), the dome: a RIGHT arc of radius R down to -70 deg, 350 mm down, a
 * 70 deg LEFT bend
 * of radius r1 back to heading 0, then 3 m straight. The last straight's y
 * is kept for the on-course check.
 */
static double g_dome_end_y;
static double g_dome_end_x;
static void dome_build (double r1, double big_r, double kink_deg)
{
    double x = -0.3;
    double y = 0.0;
    double th = 0.0;
    double k = kink_deg * CORE_PI / 180.0;
    double d70 = 70.0 * CORE_PI / 180.0;

    g_sb_n = 0;
    sbend_leg(&x, &y, &th, 0.7, 0.0);
    sbend_leg(&x, &y, &th, d70 * r1, 1.0 / r1);
    sbend_leg(&x, &y, &th, 0.35, 0.0);
    sbend_leg(&x, &y, &th, k * r1, -1.0 / r1);
    sbend_leg(&x, &y, &th, ((2.0 * d70) - k) * big_r, -1.0 / big_r);
    sbend_leg(&x, &y, &th, 0.35, 0.0);
    sbend_leg(&x, &y, &th, d70 * r1, 1.0 / r1);
    g_dome_end_x = x;
    g_dome_end_y = y;
    sbend_leg(&x, &y, &th, 3.0, 0.0);
}

/** From x = -0.3: straight to `before`, right bend of radius r, `between`
 * straight, left bend of radius r, 1.5 m straight. */
static void sbend_build (double before, double r, double between)
{
    double x = -0.3;
    double y = 0.0;
    double th = 0.0;

    g_sb_n = 0;
    sbend_leg(&x, &y, &th, before + 0.3, 0.0);
    sbend_leg(&x, &y, &th, 0.5 * CORE_PI * r, -1.0 / r);
    sbend_leg(&x, &y, &th, between, 0.0);
    sbend_leg(&x, &y, &th, 0.5 * CORE_PI * r, 1.0 / r);
    sbend_leg(&x, &y, &th, 3.0, 0.0);
}

/** On the S-bend line: within half its width of a centre point (checked
 * coarsely first, every 10th point). */
static bool sbend_on (double x, double y)
{
    const double near = SIM_LINE_HALF * SIM_LINE_HALF;
    double dx;
    double dy;
    int    i;
    int    j;

    for (i = 0; i < g_sb_n; i += 10)
    {
        dx = x - g_sb_x[i];
        dy = y - g_sb_y[i];
        if (((dx * dx) + (dy * dy)) > (0.04 * 0.04))
        {
            continue;
        }
        for (j = (i > 10) ? (i - 10) : 0; (j < (i + 10)) && (j < g_sb_n); ++j)
        {
            dx = x - g_sb_x[j];
            dy = y - g_sb_y[j];
            if (((dx * dx) + (dy * dy)) <= near)
            {
                return true;
            }
        }
    }
    return false;
}

/* S-bend smoothness: steering swings (side to side past +-0.3), the front
 * sensors' midpoint off the line centre (RMS and most), samples at full lock.
 */
static uint32_t g_sm_swings;
static int      g_sm_last_sign;
static double   g_sm_sq;
static double   g_sm_most;
static uint32_t g_sm_n;
static uint32_t g_sm_full;
static double   g_sm_worst_t;

static void sbend_measure (double cmd)
{
    double mx = g_x + (W.ahead * cos(g_th));
    double my = g_y + (W.ahead * sin(g_th));
    double best = 1.0e9;
    double d;
    int    i;
    int    sign = (cmd > 0.3) ? 1 : ((cmd < -0.3) ? -1 : 0);

    if ((0 != sign) && (0 != g_sm_last_sign) && (sign != g_sm_last_sign))
    {
        ++g_sm_swings;
    }
    if (0 != sign)
    {
        g_sm_last_sign = sign;
    }
    if (fabs(cmd) >= 0.99)
    {
        ++g_sm_full;
    }
    for (i = 0; i < g_sb_n; ++i)
    {
        d = ((mx - g_sb_x[i]) * (mx - g_sb_x[i])) +
            ((my - g_sb_y[i]) * (my - g_sb_y[i]));
        best = (d < best) ? d : best;
    }
    g_sm_sq += best;
    if (sqrt(best) > g_sm_most)
    {
        g_sm_most = sqrt(best);
        g_sm_worst_t = g_sim_t;
    }
    ++g_sm_n;
}

/** Darkness at (x, y): the tape 1, a bar W.card (a speck in its middle 0). */
static double shade_at (double x, double y)
{
    int    j;
    double mid;

    if (W.b_sbend)
    {
        if (sbend_on(x, y))
        {
            return 1.0;
        }
    }
    else if (W.b_line && (fabs(y) <= SIM_LINE_HALF) && (x > -1.0) &&
             (x < 5.0))
    {
        return 1.0;
    }
    /* SIM_CROSS_DEG: the crossing line turned this far off square to the
     * track (a car meeting a junction skewed).
     */
    if (W.b_cross &&
        (fabs(((x - W.cross_x) *
               cos((getenv("SIM_CROSS_DEG") ? atof(getenv("SIM_CROSS_DEG")) :
                    0.0) * CORE_PI / 180.0)) +
              (y * sin((getenv("SIM_CROSS_DEG") ?
                        atof(getenv("SIM_CROSS_DEG")) : 0.0) *
                       CORE_PI / 180.0))) <= SIM_LINE_HALF))
    {
        return 1.0;
    }
    if (W.b_bars && (y >= SIM_BARS_FROM) && (y <= SIM_BARS_TO))
    {
        for (j = 0; j < g_bars; ++j)
        {
            if ((x >= g_bar_edge[j]) && (x < g_bar_edge[j + 1]))
            {
                if (0 == g_bar_black[j])
                {
                    return 0.0;
                }
                mid = 0.5 * (g_bar_edge[j] + g_bar_edge[j + 1]);
                return (fabs(x - mid) < 0.5 * W.speck) ? 0.0 : W.card;
            }
        }
    }
    return 0.0;
}

static bool black_at (double x, double y)
{
    return shade_at(x, y) > 0.5;
}

/** Mean darkness of a spot x spot square at car-frame (forward, left),
 * 9 x 9 points.
 */
static double cover_spot (double forward, double left, double spot)
{
    double cx = g_x + (forward * cos(g_th)) - (left * sin(g_th));
    double cy = g_y + (forward * sin(g_th)) + (left * cos(g_th));
    int    i;
    int    j;
    double n = 0.0;

    for (i = 0; i < 9; ++i)
    {
        for (j = 0; j < 9; ++j)
        {
            n += shade_at(cx + spot * ((double)i / 8.0 - 0.5),
                          cy + spot * ((double)j / 8.0 - 0.5));
        }
    }
    return n / 81.0;
}

static double cover (double forward, double left)
{
    return cover_spot(forward, left, W.spot);
}

static uint16_t raw_of (double fraction, uint16_t white, uint16_t black)
{
    double raw = (double)white + ((double)black - (double)white) * fraction +
                 sim_noise();

    raw = fmax(0.0, fmin(4095.0, raw));
    return (uint16_t)raw;
}

bool board_read_ir (uint16_t samples[3])
{
    samples[0] = raw_of(cover(W.ahead, 0.5 * W.spacing),
                        ROBOT_IR_LEFT_WHITE, ROBOT_IR_LEFT_BLACK);
    samples[1] = raw_of(cover(W.ahead, -0.5 * W.spacing),
                        ROBOT_IR_RIGHT_WHITE, ROBOT_IR_RIGHT_BLACK);
    samples[2] = raw_of(cover_spot(W.b_ahead, W.b_left, W.b_spot),
                        ROBOT_IR_BARCODE_WHITE, ROBOT_IR_BARCODE_BLACK);
    return true;
}

static void sim_step (double dt)
{
    double metres_per_count = (2.0 * CORE_PI * ROBOT_WHEEL_RADIUS_M) /
                              (double)ROBOT_ENCODER_COUNTS_PER_REV;
    double target;
    double magnitude;
    double v;
    double omega;
    int32_t counts;
    int    w;

    for (w = 0; w < 2; ++w)
    {
        magnitude = fabs(g_wduty[w]);
        target = 0.0;
        if ((!W.b_stuck) && (!gb_brake) &&
            ((magnitude >= W.d_static) ||
             ((fabs(g_wv[w]) > 0.002) && (magnitude >= W.d_kinetic))))
        {
            target = copysign((magnitude - W.d_kinetic) * 0.82, g_wduty[w]);
        }
        g_wv[w] += (target - g_wv[w]) *
                   fmin(1.0, dt / (gb_brake ? 0.02 :
                                   ((0.0 == g_wduty[w]) ? W.coast_tau : 0.06)));
        if (W.b_stuck)
        {
            g_wv[w] = 0.0;
        }
        g_wcount[w] += g_wv[w] * dt / metres_per_count;
        counts = (int32_t)g_wcount[w];
        g_wcount[w] -= (double)counts;
        g_delta[w] = counts;
        g_distance[w] += counts;
    }
    v = 0.5 * (g_wv[0] + g_wv[1]);
    omega = (g_wv[1] - g_wv[0]) / ROBOT_TRACK_WIDTH_M * W.scrub;
    g_th += omega * dt;
    g_x += v * cos(g_th) * dt;
    g_y += v * sin(g_th) * dt;
}

static void world_default (void)
{
    memset(&W, 0, sizeof(W));
    W.ahead = 0.080;
    W.spacing = 0.030;
    W.spot = 0.006;
    W.b_ahead = 0.080;
    W.b_left = 0.045;
    W.b_line = true;
    W.scrub = 1.0;
    W.d_static = 0.14;
    W.d_kinetic = 0.10;
    W.noise = 25.0;
    W.coast_tau = 0.06;
    W.b_spot = 0.002;
    W.card = 1.0;
}

/** The car as measured 29 Oct: front sensors 130 mm ahead, 18 mm apart (over
 * the tape edges); a card printed at half the tape's darkness (a still wide
 * bar 2085, the tape 4043). The side sensor was 160 mm ahead and 71.5 mm
 * left; from 2 Nov it is 70 mm ahead and 80 mm left (robot_config.h).
 */
static void world_measured (double spot_mm)
{
    world_default();
    W.ahead = 0.130;
    W.spacing = 0.018;
    W.b_ahead = 0.070;
    W.b_left = 0.080;
    W.b_spot = spot_mm / 1000.0;
    W.card = 0.49;
}

/** One test from START at pose (x, y, heading deg); STOP at stop_s if > 0. */
static void run_mode (bench_mode_t mode, double x, double y, double th_deg,
                      double stop_s, double limit_s)
{
    bool b_stop_sent = false;

    g_sim_t = 0.010;
    g_x = x;
    g_y = y;
    g_th = th_deg * CORE_PI / 180.0;
    memset(g_wv, 0, sizeof(g_wv));
    memset(g_wduty, 0, sizeof(g_wduty));
    memset(g_wcount, 0, sizeof(g_wcount));
    memset(g_distance, 0, sizeof(g_distance));
    memset(g_delta, 0, sizeof(g_delta));
    gb_brake = false;
    g_mode = mode;
    bench_drive_start();
    while (g_drive.b_running && (g_sim_t < limit_s))
    {
        g_sim_t += SIM_DT;
        sim_step(SIM_DT);
        if ((stop_s > 0.0) && (!b_stop_sent) && (g_sim_t >= stop_s))
        {
            b_stop_sent = true;
            bench_drive_tick(BENCH_CMD_STOP);
        }
        else
        {
            bench_drive_tick(BENCH_CMD_NONE);
        }
        /* From the first line-following sample (not AUTO-CENTRE's pivots). */
        if (W.b_sbend && g_drive.b_running && (0U != g_centre.samples))
        {
            sbend_measure(g_centre.cmd);
            /* SIM_LOG: every 50 ms the true pose, both front covers, the
             * phase, steering, last side and averaged turn (S-bend losses).
             */
            if ((NULL != getenv("SIM_LOG")) &&
                (0 == ((int)(g_sim_t * 1000.0 + 0.5) % 50)))
            {
                printf("LOG t %5.2f x %4.0f y %4.0f th %+4.0f L %.2f R %.2f"
                       " ph %u cmd %+.2f side %+d seen %+d turn %+.2f\n",
                       g_sim_t, g_x * 1000.0, g_y * 1000.0,
                       g_th * 180.0 / CORE_PI,
                       cover(W.ahead, 0.5 * W.spacing),
                       cover(W.ahead, -0.5 * W.spacing),
                       (unsigned)g_centre.phase, g_centre.cmd,
                       (int)g_centre.side, (int)g_centre.seen_side,
                       g_centre.turn);
            }
        }
    }
    if (g_drive.b_running)
    {
        printf("!!! still running at the sim limit\n");
    }
}

/** Heading error of the true pose from the one that centres the line between
 * the sensors (their midpoint on the line), in degrees; *p_mid_mm that
 * midpoint's distance from the line centre.
 */
static double centred_error_deg (double * p_mid_mm)
{
    double mid = g_y + (W.ahead * sin(g_th));
    double ideal = asin(fmax(-1.0, fmin(1.0, -g_y / W.ahead)));

    *p_mid_mm = mid * 1000.0;
    return (g_th - ideal) * 180.0 / CORE_PI;
}

static void auto_grid (void)
{
    static const double aheads[] = {0.050, 0.080, 0.120};
    static const double spacings[] = {0.020, 0.026, 0.034, 0.044};
    static const double headings[] = {-40.0, -25.0, -12.0, -5.0, 0.0, 5.0,
                                      12.0, 25.0, 40.0};
    static const double offsets[] = {-0.012, 0.0, 0.012};
    static const double scrubs[] = {1.0, 0.85};
    size_t ia, is, ih, io, ic;
    int    runs = 0;
    int    good = 0;
    double worst_deg = 0.0;
    double worst_mm = 0.0;
    double worst_s = 0.0;
    double mid_mm;
    double err;

    printf("===== AUTO-CENTRE grid: ahead 50/80/120 mm, spacing 20/26/34/44 mm,"
           " start -40..+40 deg, axle -12/0/+12 mm off the line, scrub 1/0.85\n");
    for (ia = 0U; ia < 3U; ++ia)
    for (is = 0U; is < 4U; ++is)
    {
        int cell_good = 0;
        int cell_runs = 0;
        double cell_worst = 0.0;

        for (ih = 0U; ih < 9U; ++ih)
        for (io = 0U; io < 3U; ++io)
        for (ic = 0U; ic < 2U; ++ic)
        {
            world_default();
            W.ahead = aheads[ia];
            W.spacing = spacings[is];
            W.scrub = scrubs[ic];
            gb_quiet = true;
            run_mode(BENCH_MODE_AUTO, 0.3, offsets[io], headings[ih], 0.0,
                     60.0);
            gb_quiet = false;
            err = centred_error_deg(&mid_mm);
            ++runs;
            ++cell_runs;
            if ((!g_auto.b_failed) && (fabs(mid_mm) <= 3.0))
            {
                ++good;
                ++cell_good;
            }
            else
            {
                printf("  MISS ahead %.0f spacing %.0f start %+.0f deg axle"
                       " %+.0f mm scrub %.2f: %s, error %+.1f deg, midpoint"
                       " %+.1f mm\n", aheads[ia] * 1000.0,
                       spacings[is] * 1000.0, headings[ih],
                       offsets[io] * 1000.0, scrubs[ic],
                       g_auto.b_failed ? g_auto.p_fail : "done", err,
                       mid_mm);
            }
            if (fabs(err) > cell_worst)
            {
                cell_worst = fabs(err);
            }
            if (fabs(err) > worst_deg)
            {
                worst_deg = fabs(err);
            }
            if (fabs(mid_mm) > worst_mm)
            {
                worst_mm = fabs(mid_mm);
            }
            if ((double)g_auto.done_ms / 1000.0 > worst_s)
            {
                worst_s = (double)g_auto.done_ms / 1000.0;
            }
        }
        printf("  ahead %3.0f mm spacing %2.0f mm: %2d/%2d within 3 mm, worst"
               " heading error %.1f deg\n", aheads[ia] * 1000.0,
               spacings[is] * 1000.0, cell_good, cell_runs, cell_worst);
    }
    printf("  TOTAL %d/%d centred, sensor midpoint within 3 mm of the line; worst %.1f deg, worst"
           " midpoint %.1f mm, longest %.1f s\n", good, runs, worst_deg,
           worst_mm, worst_s);
}

static void auto_case (const char * p_name, double y, double th_deg)
{
    double mid_mm;
    double err;

    printf("===== %s\n", p_name);
    run_mode(BENCH_MODE_AUTO, 0.3, y, th_deg, 0.0, 60.0);
    err = centred_error_deg(&mid_mm);
    printf("  -> true heading %+.1f deg, error %+.1f deg, midpoint %+.1f mm,"
           " %s\n", g_th * 180.0 / CORE_PI, err, mid_mm,
           g_auto.b_failed ? "FAILED" : "done");
}

/* ---- barcode decode on synthetic strips (bench_code_* directly) ---- */
static void code_case (const char * p_name, const char * p_text,
                       double narrow, double wide, bool b_reverse, int dark,
                       int light, double blur, double length)
{
    double hc_mm = (CORE_PI * ROBOT_WHEEL_RADIUS_M * 1000.0) /
                   (double)ROBOT_ENCODER_COUNTS_PER_REV;
    double x;
    double c;
    int    s;
    int    raw;
    double edge_lo;
    double edge_hi;

    printf("===== BARCODE decode: %s\n", p_name);
    world_default();
    W.b_bars = (NULL != p_text);
    if (W.b_bars)
    {
        bars_build(p_text, narrow, wide, 100.0);
        if (b_reverse)
        {
            /* Mirror the strip within its own span. */
            double edges[200];
            int    blacks[200];

            edge_lo = g_bar_edge[0];
            edge_hi = g_bar_edge[g_bars];
            for (s = 0; s <= g_bars; ++s)
            {
                edges[s] = edge_lo + edge_hi - g_bar_edge[g_bars - s];
            }
            for (s = 0; s < g_bars; ++s)
            {
                blacks[s] = g_bar_black[g_bars - 1 - s];
            }
            memcpy(g_bar_edge, edges, sizeof(edges));
            memcpy(g_bar_black, blacks, sizeof(blacks));
            g_bar_black[g_bars] = 0;
        }
    }
    if (!bench_code_start())
    {
        printf("start failed\n");
        return;
    }
    for (x = 0.0; x < length; x += 0.54)
    {
        c = 0.0;
        for (s = 0; s < 9; ++s)
        {
            /* The strip is in mm here; y 0.05 is on the bars. */
            c += black_at(x + blur * ((double)s - 4.0) / 8.0, 0.05) ? 1.0 :
                                                                       0.0;
        }
        c /= 9.0;
        raw = (int)((double)light + (double)(dark - light) * c +
                    sim_noise() * 2.0);
        raw = (raw < 0) ? 0 : ((raw > 4095) ? 4095 : raw);
        bench_code_sample((int32_t)(0.5 * x / hc_mm),
                          (int32_t)(0.5 * x / hc_mm), (uint16_t)raw);
    }
    bench_code_results();
}

/** BARCODE end to end with the measured car: a card beside a straight line
 * from x0, a crossing line at cross (0 none), START at heading th_deg.
 */
static void code_drive (const char * p_name, const char * p_text,
                        double narrow, double wide, double spot_mm,
                        double card, double speck_mm, double cross,
                        double th_deg)
{
    world_measured(spot_mm);
    W.card = card;
    W.speck = speck_mm / 1000.0;
    W.b_bars = true;
    bars_build(p_text, narrow / 1000.0, wide / 1000.0, 0.10 + W.b_ahead);
    if (cross > 0.0)
    {
        W.b_cross = true;
        W.cross_x = cross;
    }
    printf("===== BARCODE drive: %s: %s %.1f/%.1f mm, spot %.0f mm, card"
           " %.2f, speck %.1f mm, cross %s, start %+.0f deg\n", p_name,
           p_text, narrow, wide, spot_mm, card, speck_mm,
           (cross > 0.0) ? "yes" : "no", th_deg);
    run_mode(BENCH_MODE_BARCODE, 0.0, 0.0, th_deg, 0.0, 60.0);
}

/* ---- a recorded 1 mm trace ("  mm c centre: raw ...") through the passes */
static void code_trace (const char * p_path)
{
    double hc_mm = (CORE_PI * ROBOT_WHEEL_RADIUS_M * 1000.0) /
                   (double)ROBOT_ENCODER_COUNTS_PER_REV;
    FILE * p_file = fopen(p_path, "r");
    char   line[2048];
    char * p_at;
    char * p_end;
    long   base;
    long   value;
    int    column;
    double x;

    if (NULL == p_file)
    {
        printf("no trace %s\n", p_path);
        return;
    }
    printf("===== BARCODE trace: %s\n", p_path);
    world_default();
    if (!bench_code_start())
    {
        printf("start failed\n");
        fclose(p_file);
        return;
    }
    while (NULL != fgets(line, sizeof(line), p_file))
    {
        base = strtol(line, &p_end, 10);
        p_at = strchr(p_end, ':');
        if ((p_end == line) || (NULL == p_at))
        {
            continue;
        }
        p_at += 1;
        for (column = 0; ; ++column)
        {
            value = strtol(p_at, &p_end, 10);
            if (p_end == p_at)
            {
                break;
            }
            p_at = p_end;
            x = (double)(base + column);
            bench_code_sample((int32_t)(0.5 * x / hc_mm),
                              (int32_t)(0.5 * x / hc_mm), (uint16_t)value);
        }
    }
    fclose(p_file);
    bench_code_results();
}

/* ---- MISSION's streaming store fed with recorded 1 mm traces, one after
 * another along the path (each shifted 1000 mm past the last) */
static void code_stream (int count, char ** p_paths)
{
    double hc_mm = (CORE_PI * ROBOT_WHEEL_RADIUS_M * 1000.0) /
                   (double)ROBOT_ENCODER_COUNTS_PER_REV;
    FILE * p_file;
    char   line[2048];
    char * p_at;
    char * p_end;
    long   base;
    long   value;
    long   top = 0;
    long   shift = 0;
    int    column;
    int    path;
    double x;

    printf("===== MISSION stream:");
    for (path = 0; path < count; ++path)
    {
        printf(" %s", p_paths[path]);
    }
    printf("\n");
    world_default();
    g_mode = BENCH_MODE_MISSION;
    if (!bench_code_start())
    {
        printf("start failed\n");
        return;
    }
    for (path = 0; path < count; ++path)
    {
        p_file = fopen(p_paths[path], "r");
        if (NULL == p_file)
        {
            printf("no trace %s\n", p_paths[path]);
            continue;
        }
        while (NULL != fgets(line, sizeof(line), p_file))
        {
            base = strtol(line, &p_end, 10);
            p_at = strchr(p_end, ':');
            if ((p_end == line) || (NULL == p_at))
            {
                continue;
            }
            p_at += 1;
            for (column = 0; ; ++column)
            {
                value = strtol(p_at, &p_end, 10);
                if (p_end == p_at)
                {
                    break;
                }
                p_at = p_end;
                x = (double)(shift + base + column);
                top = (long)x;
                bench_code_sample((int32_t)(0.5 * x / hc_mm),
                                  (int32_t)(0.5 * x / hc_mm),
                                  (uint16_t)value);
                bench_code_tries_step(); if (bench_code_found_text()) { bench_flush(); } while (bench_raw_text()) { bench_flush(); }
            }
        }
        fclose(p_file);
        shift = top + 1000;
    }
    bench_code_mission_results();
}

/* ---- MISSION's raw dump (" raw k of n: ..." then "  u<unit>: means")
 * from a run log, replayed unit by unit through the streaming store; with
 * a dump number, only that cut, then the BARCODE darkness pass on it. */
static void code_dump (const char * p_path, int only)
{
    FILE * p_file = fopen(p_path, "r");
    char   line[4096];
    char * p_at;
    char * p_end;
    long   unit;
    long   value;
    int    dump = 0;
    int    fed = 0;

    if (NULL == p_file)
    {
        printf("no log %s\n", p_path);
        return;
    }
    printf("===== MISSION dump replay: %s (cut %d)\n", p_path, only);
    world_default();
    g_mode = BENCH_MODE_MISSION;
    if (!bench_code_start())
    {
        printf("start failed\n");
        fclose(p_file);
        return;
    }
    if (0 != only)
    {
        g_code.b_stream = false;
    }
    while (NULL != fgets(line, sizeof(line), p_file))
    {
        p_at = strstr(line, " raw ");
        if ((NULL != p_at) && (NULL != strstr(line, " units of ")))
        {
            dump = (int)strtol(p_at + 5, NULL, 10);
            continue;
        }
        p_at = strstr(line, "  u");
        if ((NULL == p_at) || ((0 != only) && (dump != only)))
        {
            continue;
        }
        unit = strtol(p_at + 3, &p_end, 10);
        if ((p_end == (p_at + 3)) || (':' != *p_end))
        {
            continue;
        }
        p_at = p_end + 1;
        for (;;)
        {
            value = strtol(p_at, &p_end, 10);
            if (p_end == p_at)
            {
                break;
            }
            p_at = p_end;
            bench_code_sample((int32_t)(4 * unit + 1), (int32_t)(4 * unit + 1),
                              (uint16_t)value);
            if (0 == only)
            {
                bench_code_tries_step(); if (bench_code_found_text()) { bench_flush(); } while (bench_raw_text()) { bench_flush(); }
            }
            ++unit;
            ++fed;
        }
    }
    fclose(p_file);
    printf("fed %d units\n", fed);
    if (0 == only)
    {
        bench_code_mission_results();
    }
    else
    {
        bench_code_store();
        bench_code_dark(false, 2U * BENCH_CODE_DARK_SDS, BENCH_CODE_GROUP_GAP_MM);
        bench_code_dark(false, 2U * BENCH_CODE_DARK_SDS, BENCH_CODE_QUIET_MM);
    }
}

#ifdef BENCH_CODE_RULE_PEAKS
#define SIM_PEAKS0 ((0U != g_code.founds) ? (int)g_code.found[0].b_peaks : 0)
#else
#define SIM_PEAKS0 (0)
#endif
/* ---- MISSION over synthetic cards: one line per run, then a summary.
 * Each run: a card of 3 mm / 9 mm elements beside the line, the measured
 * car, a spot, a print darkness, a start heading, ADC noise and specks.
 * Prints card, settings, then the confirmed and one-swing texts. */
static void code_batch (void)
{
    static const char * const cards[] =
    {
        "*Z*", "*A*", "*P*", "*Y*", "*N*", "*L*", "*8*", "*F*"
    };
    static const double spots[] = { 4.0, 5.0, 6.0 };
    static const double inks[] = { 0.49, 0.30 };
    static const double ths[] = { 0.0, 6.0 };
    static const double noises[] = { 25.0, 60.0 };
    static const double specks[] = { 0.0, 0.6 };
    char     codes[256];
    char     singles[256];
    char     text[BENCH_CODE_TEXT + 2];
    unsigned c, s, k, t, n, p;
    uint32_t i;
    int      right = 0, wrong = 0, single_right = 0, single_wrong = 0;
    int      none = 0, runs = 0;
    bool     b_right;
    bool     b_wrong;
    bool     b_single;

    gb_quiet = true;
    for (c = 0U; c < sizeof(cards) / sizeof(cards[0]); ++c)
    for (s = 0U; s < sizeof(spots) / sizeof(spots[0]); ++s)
    for (k = 0U; k < sizeof(inks) / sizeof(inks[0]); ++k)
    for (t = 0U; t < sizeof(ths) / sizeof(ths[0]); ++t)
    for (n = 0U; n < sizeof(noises) / sizeof(noises[0]); ++n)
    for (p = 0U; p < sizeof(specks) / sizeof(specks[0]); ++p)
    {
        world_measured(spots[s]);
        W.card = inks[k];
        W.noise = noises[n];
        W.speck = specks[p] / 1000.0;
        W.b_bars = true;
        g_rng = 12345U + (c * 977U) + (s * 131U) + (k * 31U) + (t * 7U) +
                (n * 3U) + p;
        bars_build(cards[c], 0.003, 0.009, 0.10 + W.b_ahead);
        run_mode(BENCH_MODE_MISSION, 0.0, 0.0, ths[t], 9.0, 30.0);
        codes[0] = '\0';
        singles[0] = '\0';
        b_right = false;
        b_wrong = false;
        b_single = false;
        for (i = 0U; i < g_code.founds; ++i)
        {
            const bench_code_found_t * f = &g_code.found[i];
            if (!(f->b_framed || f->b_single))
            {
                continue;
            }
            snprintf(text, sizeof(text), "%s ",
                     f->b_reverse ? f->reverse : f->text);
            if (f->b_framed)
            {
                strncat(codes, text, sizeof(codes) - strlen(codes) - 1U);
                if (0 == strncmp(text, cards[c], strlen(cards[c])))
                {
                    b_right = true;
                }
                else
                {
                    b_wrong = true;
                }
            }
            else
            {
                strncat(singles, text, sizeof(singles) - strlen(singles) - 1U);
                b_single = true;
                if (0 == strncmp(text, cards[c], strlen(cards[c])))
                {
                    ++single_right;
                }
                else
                {
                    ++single_wrong;
                }
            }
        }
        right += b_right ? 1 : 0;
        wrong += b_wrong ? 1 : 0;
        none += (!b_right && !b_wrong && !b_single) ? 1 : 0;
        ++runs;
        printf("RUN %s spot %.0f ink %.2f th %+.0f noise %.0f speck %.1f |"
               " codes %s| single %s| peaks %d\n", cards[c], spots[s],
               inks[k], ths[t], noises[n], specks[p], codes[0] ? codes : "- ",
               singles[0] ? singles : "- ",
               SIM_PEAKS0);
    }
    printf("SUMMARY runs %d: right %d, WRONG %d, one-swing right %d, one-swing"
           " wrong %d, nothing %d\n", runs, right, wrong, single_right,
           single_wrong, none);
}

static int sim_old_main (void);

int main (int argc, char ** argv)
{
    if ((argc > 1) && (0 == strcmp(argv[1], "old")))
    {
        return sim_old_main();
    }
    if ((argc > 1) && (0 == strcmp(argv[1], "batch")))
    {
        code_batch();
        return 0;
    }
    if ((argc > 2) && (0 == strcmp(argv[1], "dump")))
    {
        code_dump(argv[2], (argc > 3) ? atoi(argv[3]) : 0);
        return 0;
    }
    if ((argc > 2) && (0 == strcmp(argv[1], "stream")))
    {
        code_stream(argc - 2, &argv[2]);
        return 0;
    }
    if ((argc > 5) && (0 == strcmp(argv[1], "ramp")))
    {
        world_measured(4.0);
        g_hump_h = atof(argv[2]) / 1000.0;
        g_hump_len = atof(argv[3]) / 1000.0;
        g_hump_top = atof(argv[4]) / 1000.0;
        g_hump_roll = atof(argv[5]);
        g_hump_from = getenv("SIM_FROM") ? atof(getenv("SIM_FROM")) : 0.6;
        gb_accel = true;
        printf("===== RAMP: %.0f mm up over %.0f mm, top %.0f mm, roll %+.1f"
               " deg, from %.0f mm\n", g_hump_h * 1000.0, g_hump_len * 1000.0,
               g_hump_top * 1000.0, g_hump_roll, g_hump_from * 1000.0);
        run_mode(BENCH_MODE_MISSION, 0.0, 0.0, 6.0, 16.0, 22.0);
        return 0;
    }
    if ((argc > 1) && (0 == strcmp(argv[1], "junction")))
    {
        /* MISSION: a card beside the line, then a crossing at cross m
         * (default 0.50: its end ~130 mm before the front sensors reach it),
         * for the junction's wait on the card's decode (bench_nav_wait).
         */
        double cross = (argc > 2) ? atof(argv[2]) : 0.50;

        world_measured(4.0);
        W.b_bars = true;
        bars_build("*A*", 0.003, 0.009, 0.10 + W.b_ahead);
        W.b_cross = true;
        W.cross_x = cross;
        printf("===== JUNCTION: *A* 3/9 mm card, crossing at %.0f mm\n",
               cross * 1000.0);
        /* SIM_START_DEG: start heading (default 4), SIM_START_Y_MM: start
         * offset across the line (default 0), so the car can meet the
         * crossing still correcting.
         */
        run_mode(BENCH_MODE_MISSION, 0.0,
                 (getenv("SIM_START_Y_MM") ? atof(getenv("SIM_START_Y_MM")) :
                  0.0) / 1000.0,
                 getenv("SIM_START_DEG") ? atof(getenv("SIM_START_DEG")) : 4.0,
                 14.0, 22.0);
        return 0;
    }
    if ((argc > 1) && (0 == strcmp(argv[1], "sbend")))
    {
        /* MISSION on an S-bend (track half B, 19 Nov): 0.4 m straight, a
         * 90 deg right bend of r mm, between mm straight, a 90 deg left
         * bend, then straight. STOP at 28 s; on the last straight the car
         * ends at y = -(2 r + between), heading 0.
         */
        double r = ((argc > 2) ? atof(argv[2]) : 150.0) / 1000.0;
        double between = ((argc > 3) ? atof(argv[3]) : 150.0) / 1000.0;

        world_measured(4.0);
        W.b_sbend = true;
        /* SIM_SCRUB: true yaw over the wheels' (tyres slipping in a turn). */
        W.scrub = getenv("SIM_SCRUB") ? atof(getenv("SIM_SCRUB")) : 1.0;
        sbend_build(0.4, r, between);
        printf("===== SBEND: right then left bend of %.0f mm radius, %.0f mm"
               " straight between\n", r * 1000.0, between * 1000.0);
        run_mode(BENCH_MODE_MISSION, 0.0, 0.0, 0.0, 28.0, 34.0);
        printf("SBEND r %.0f between %.0f: end t %.1f s at x %.0f y %.0f mm,"
               " heading %+.0f deg; %s\n", r * 1000.0, between * 1000.0,
               g_sim_t, g_x * 1000.0, g_y * 1000.0, g_th * 180.0 / CORE_PI,
               ((g_y < -(2.0 * r + between) + 0.03) &&
                (g_y > -(2.0 * r + between) - 0.03) &&
                (g_x > 0.4 + 2.0 * r) && (fabs(g_th) < 0.5)) ?
               "ON THE LAST STRAIGHT" : "OFF THE COURSE");
        printf("SMOOTH swings %u, off the line rms %.1f mm most %.1f mm, "
               "full lock %.0f%% worst_at %.2f s\n", g_sm_swings,
               1000.0 * sqrt(g_sm_sq / (double)((0U != g_sm_n) ? g_sm_n : 1U)),
               1000.0 * g_sm_most,
               100.0 * (double)g_sm_full /
               (double)((0U != g_sm_n) ? g_sm_n : 1U), g_sm_worst_t);
        return 0;
    }
    if ((argc > 1) && (0 == strcmp(argv[1], "dome")))
    {
        /* MISSION on the dome (dome_build): r1 mm, R mm, kink deg. */
        double r1 = ((argc > 2) ? atof(argv[2]) : 60.0) / 1000.0;
        double big_r = ((argc > 3) ? atof(argv[3]) : 250.0) / 1000.0;
        double kink = (argc > 4) ? atof(argv[4]) : 20.0;

        world_measured(4.0);
        W.b_sbend = true;
        W.scrub = getenv("SIM_SCRUB") ? atof(getenv("SIM_SCRUB")) : 1.0;
        dome_build(r1, big_r, kink);
        printf("===== DOME: bends r1 %.0f mm, dome R %.0f mm, kink %.0f deg\n",
               r1 * 1000.0, big_r * 1000.0, kink);
        run_mode(BENCH_MODE_MISSION, 0.0, 0.0, 0.0, 34.0, 40.0);
        printf("DOME r1 %.0f R %.0f kink %.0f: end t %.1f s at x %.0f y %.0f mm,"
               " heading %+.0f deg; %s\n", r1 * 1000.0, big_r * 1000.0, kink,
               g_sim_t, g_x * 1000.0, g_y * 1000.0, g_th * 180.0 / CORE_PI,
               ((fabs(g_y - g_dome_end_y) < 0.03) && (g_x > g_dome_end_x) &&
                (fabs(g_th) < 0.5)) ? "ON THE LAST STRAIGHT" :
                                      "OFF THE COURSE");
        return 0;
    }
    if ((argc > 2) && (0 == strcmp(argv[1], "trace")))
    {
        code_trace(argv[2]);
        return 0;
    }
    code_drive("as on the track", "*A*", 3.0, 9.0, 4.0, 0.49, 0.0, 0.0,
               4.0);
    code_drive("narrow print", "*Q7-*", 2.0, 6.0, 4.0, 0.49, 0.0, 0.0, 4.0);
    code_drive("wide spot", "*Q7-*", 2.0, 6.0, 6.0, 0.49, 0.0, 0.0, -6.0);
    code_drive("specks in bars", "*A*", 3.0, 9.0, 4.0, 0.49, 1.0, 0.0, 4.0);
    code_drive("then a crossing", "*A*", 3.0, 9.0, 4.0, 0.49, 0.0, 0.62,
               4.0);
    code_drive("tape-dark print", "*A*", 3.0, 9.0, 4.0, 1.0, 0.0, 0.0, 4.0);
    code_drive("faint print", "*A*", 3.0, 9.0, 4.0, 0.25, 0.0, 0.0, 4.0);
    code_drive("ratio 2", "*TEST*", 2.5, 5.0, 4.0, 0.49, 0.0, 0.0, 6.0);
    code_drive("spec print, 6 mm spot", "*A*", 3.0, 9.0, 6.0, 0.49, 0.0, 0.0,
               -6.0);
    code_drive("spec print, 8 mm spot", "*A*", 3.0, 9.0, 8.0, 0.49, 0.0, 0.0,
               4.0);
    code_drive("five characters", "*Q7-*", 3.0, 9.0, 4.0, 0.49, 0.0, 0.0,
               -6.0);
    code_drive("five, 6 mm spot", "*Q7-*", 3.0, 9.0, 6.0, 0.49, 0.0, 0.62,
               6.0);
    return 0;
}

static int sim_old_main (void)
{
    world_default();
    auto_case("AUTO: line between the sensors, car on it", 0.0, 0.0);
    auto_case("AUTO: car turned 20 deg left (line right of R)", 0.0, 20.0);
    auto_case("AUTO: car turned 20 deg right (line left of L)", 0.0, -20.0);
    auto_case("AUTO: R on the line at START (turns right first)", 0.0,
              atan(0.5 * W.spacing / W.ahead) * 180.0 / CORE_PI);
    auto_case("AUTO: axle 15 mm left of the line, 8 deg", 0.015, 8.0);
    auto_case("AUTO: 55 deg off (near the 70 deg window)", 0.0, 55.0);
    auto_case("AUTO: 80 deg off (outside the window)", 0.0, 80.0);
    W.b_line = false;
    auto_case("AUTO: no line at all", 0.0, 0.0);
    world_default();
    W.b_stuck = true;
    auto_case("AUTO: wheels stuck", 0.0, 10.0);
    world_default();
    W.b_cross = true;
    W.cross_x = 0.3 + W.ahead;
    auto_case("AUTO: a crossing line under both sensors at START", 0.0, 0.0);
    world_default();
    W.b_bars = true;
    bars_build("*A*", 0.003, 0.009, 0.3 + W.ahead - 0.02);
    auto_case("AUTO: Code 39 bars beside the line at the sensors, 10 deg",
              0.0, 10.0);
    world_default();
    W.d_static = 0.22;
    W.d_kinetic = 0.18;
    auto_case("AUTO: stiff wheels (start 22% duty)", 0.0, 15.0);
    world_default();
    W.coast_tau = 0.15;
    auto_case("AUTO: long run-on after a halt (150 ms)", 0.0, -15.0);
    world_default();
    W.coast_tau = 0.30;
    auto_case("AUTO: very long run-on after a halt (300 ms)", 0.01, 18.0);
    world_default();
    printf("===== LINE smooth A, STOP during the centring (at 1 s)\n");
    run_mode(BENCH_MODE_SMOOTH_A, 0.3, 0.0, 20.0, 1.0, 60.0);
    world_default();
    W.b_line = false;
    printf("===== LINE smooth B, no line: the centring fails, no line test\n");
    run_mode(BENCH_MODE_SMOOTH_B, 0.3, 0.0, 0.0, 0.0, 60.0);
    printf("===== 0xAF presses from boot (PID is 0): LINE smooth A %d, B %d,"
           " C %d, BARCODE %d, AUTO-CENTRE %d; %d tests\n",
           (int)BENCH_MODE_SMOOTH_A, (int)BENCH_MODE_SMOOTH_B,
           (int)BENCH_MODE_SMOOTH_C, (int)BENCH_MODE_BARCODE,
           (int)BENCH_MODE_AUTO, (int)BENCH_MODES);
    auto_grid();

    world_default();
    printf("===== LINE smooth A after AUTO-CENTRE, straight line, car already"
           " centred and aligned, STOP at 15 s\n");
    run_mode(BENCH_MODE_SMOOTH_A, 0.0, 0.0, 0.0, 15.0, 60.0);
    {
        double mid_mm;
        double err = centred_error_deg(&mid_mm);

        printf("  -> x %.0f mm, sensor midpoint %+.1f mm off the line,"
               " heading %+.1f deg (error %+.1f)\n", g_x * 1000.0, mid_mm,
               g_th * 180.0 / CORE_PI, err);
    }

    world_default();
    printf("===== LINE smooth A after AUTO-CENTRE, straight line, start 15 deg,"
           " axle 5 mm off, STOP at 25 s\n");
    run_mode(BENCH_MODE_SMOOTH_A, 0.0, 0.005, 15.0, 25.0, 60.0);
    {
        double mid_mm;
        double err = centred_error_deg(&mid_mm);

        printf("  -> x %.0f mm, sensor midpoint %+.1f mm off the line,"
               " heading %+.1f deg (error %+.1f)\n", g_x * 1000.0, mid_mm,
               g_th * 180.0 / CORE_PI, err);
    }

    world_default();
    W.b_bars = true;
    bars_build("*A*", 0.003, 0.009, 0.30);
    printf("===== BARCODE end to end: *A* beside the line from x 300 mm,"
           " start -10 deg\n");
    run_mode(BENCH_MODE_BARCODE, 0.0, 0.0, -10.0, 0.0, 60.0);

    world_default();
    W.spacing = 0.020;
    W.b_bars = true;
    bars_build("*A*", 0.003, 0.009, 0.30);
    printf("===== BARCODE end to end: *A*, sensors 20 mm apart, start -10"
           " deg\n");
    run_mode(BENCH_MODE_BARCODE, 0.0, 0.0, -10.0, 0.0, 60.0);

    world_default();
    W.spacing = 0.020;
    W.b_bars = true;
    bars_build("*Q7-*", 0.003, 0.009, 0.25);
    printf("===== BARCODE end to end: *Q7-* (no codebook), start +6 deg\n");
    gb_quiet = false;
    run_mode(BENCH_MODE_BARCODE, 0.0, 0.003, 6.0, 0.0, 60.0);

    code_case("A forward 3/9", "*A*", 3, 9, false, 3900, 150, 2.0, 400);
    code_case("A reversed", "*A*", 3, 9, true, 3900, 150, 2.0, 400);
    code_case("Z at 2x scale", "*Z*", 6, 18, false, 3900, 150, 2.0, 500);
    code_case("TEST, 6 characters", "*TEST*", 3, 9, false, 3900, 150, 2.0,
              500);
    code_case("ink invisible to IR", "*A*", 3, 9, false, 300, 150, 2.0, 400);
    code_case("weak toner (between LIGHT and DARK)", "*A*", 3, 9, false,
              2700, 150, 2.0, 400);
    code_case("narrow print 2.6/7.8, blur 3", "*A*", 2.6, 7.8, false, 3600,
              200, 3.0, 400);
    return 0;
}
