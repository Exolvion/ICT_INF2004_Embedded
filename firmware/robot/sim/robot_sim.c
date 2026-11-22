/* Host sim of the ROBOT APP running the mission (22 Nov 2026): robot_app.c's
 * eight task steps on a 1 ms scheduler (Safety, Line and Scan every 2 ms,
 * Motion, Vehicle and Comm every 5 ms, IMU every 10 ms, Telemetry every
 * 200 ms, in that priority order within a millisecond), with mission.c built
 * as the Vehicle task's controller (MISSION_SIM: each mission line printed
 * as "#F <frame hex>" and its text, for bench_codec.py check). The car and
 * track are the bench sim's model (firmware/bench/offline/sim/sim.c): an
 * 18 mm line along x, an optional crossing, Code 39 bars on the car's left,
 * a hump on the path, a box for the sonar; wheels from duty with static and
 * kinetic thresholds and a 60 ms lag. Here the encoders count into 16-bit
 * totals and deltas as the board's do, and START and STOP come over the
 * simulated MQTT (remote START, as the team runs it).
 * Lines starting "##" are the sim's own notes (state, faults, end).
 */
#include "robot_app.h"
#include "app_cfg.h"
#include "app_main.h"
#include "board.h"
#include "core.h"
#include "mission.h"
#include "mqttcmd.h"
#include "navmap.h"
#include "net_mqtt.h"
#include "net_mqtt_ipc.h"
#include "robot_config.h"
#include "scanner.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#define SIM_LINE_HALF (0.009)
#define SIM_BARS_FROM (0.029)   /* 20 mm beyond the line edge */
#define SIM_BARS_TO (0.200)
#define SIM_SONAR_AHEAD (0.08)
#define SIM_BEAM_DEG (8.0)

typedef struct
{
    double ahead;      /* front sensors ahead of the axle, m */
    double spacing;    /* front lens centres apart, m */
    double spot;       /* sensor spot, m square */
    double b_ahead;    /* side sensor ahead of the axle */
    double b_left;     /* side sensor left of the car's centre line */
    double b_spot;     /* side sensor spot, m square */
    double card;       /* bar darkness, share of the tape's */
    bool   b_line;
    bool   b_cross;
    double cross_x;
    bool   b_bars;
    double scrub;      /* true yaw / wheel-difference yaw */
    double d_static;   /* duty that starts a wheel */
    double d_kinetic;  /* duty that keeps it turning; speed above it */
    double noise;      /* raw ADC noise, +/- */
    double coast_tau;  /* wheel run-down at zero duty, s */
} world_t;

static world_t  W;
static uint64_t g_now_us = 1000000U;
static double   g_x, g_y, g_th;
static double   g_wv[2], g_wduty[2], g_wcount[2];
static int64_t  g_count[2];          /* wheel counts, + forward */
static int64_t  g_read[2];           /* counts at the last delta read */
static bool     gb_brake;
static uint32_t g_rng = 12345U;
static double   g_bar_edge[200];
static int      g_bar_black[200];
static int      g_bars;
static double   g_line_end = 5.0;
static bool     gb_corner;
static double   g_corner_x = 0.6;
static double   g_corner_cross_y = -0.5;
static bool     gb_box;
static double   g_box_x, g_box_y, g_box_hx, g_box_hy;
static uint16_t g_servo_pulse = ROBOT_SERVO_CENTRE_US;
static bool     gb_echo;
static uint32_t g_echo_us;
static bool     gb_echo_valid;
static double   g_hump_from = 1.0e9;
static double   g_hump_roll;
static double   g_hump_h, g_hump_len = 0.1, g_hump_top;
static uint64_t g_accel_us;
static uint32_t g_accel_failed;      /* SIM_FAIL_AT reads failed so far */
static uint32_t g_enc_failed;        /* SIM_ENC_ERR_AT reads so far */
static uint32_t g_ir_failed;         /* SIM_IR_FAIL_AT reads so far */
static uint32_t g_accel_calls;       /* SIM_FAIL_SCATTER count */
static uint32_t g_enc_calls;         /* SIM_ENC_SCATTER count */
static uint32_t g_ir_calls;          /* SIM_IR_SCATTER count */
static uint8_t  g_cmd_queue[8];
static uint32_t g_cmd_count;
static uint8_t  g_map_bytes[NAVMAP_LENGTH];
static bool     gb_map;
static uint32_t g_robot_frames;
static uint32_t g_robot_bytes;

static const char g_chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. *$/+%";
static const uint16_t g_patterns[44] =
{
    0x034U, 0x121U, 0x061U, 0x160U, 0x031U, 0x130U, 0x070U, 0x025U,
    0x124U, 0x064U, 0x109U, 0x049U, 0x148U, 0x019U, 0x118U, 0x058U,
    0x00DU, 0x10CU, 0x04CU, 0x01CU, 0x103U, 0x043U, 0x142U, 0x013U,
    0x112U, 0x052U, 0x007U, 0x106U, 0x046U, 0x016U, 0x181U, 0x0C1U,
    0x1C0U, 0x091U, 0x190U, 0x0D0U, 0x085U, 0x184U, 0x0C4U, 0x094U,
    0x0A8U, 0x0A2U, 0x08AU, 0x02AU
};

static double sim_t (void)
{
    return (double)(g_now_us - 1000000U) / 1000000.0;
}

/** A fault burst for noise control tests: true for the first <n> calls
 * (env p_n, default 1) from the time in env p_at on; *p_done counts them.
 */
static bool sim_burst (const char * p_at, const char * p_n, uint32_t * p_done)
{
    uint32_t n = (NULL != getenv(p_n)) ? (uint32_t)atoi(getenv(p_n)) : 1U;

    if ((NULL == getenv(p_at)) || (sim_t() < atof(getenv(p_at))) ||
        (*p_done >= n))
    {
        return false;
    }
    ++*p_done;
    return true;
}

/** Scattered noise: true on every k-th call (env p_k) once a run is under
 * way (after 1.6 s); *p_calls counts the calls.
 */
static bool sim_scatter (const char * p_k, uint32_t * p_calls)
{
    uint32_t k = (NULL != getenv(p_k)) ? (uint32_t)atoi(getenv(p_k)) : 0U;

    if ((0U == k) || (sim_t() < 1.6))
    {
        return false;
    }
    ++*p_calls;
    return 0U == (*p_calls % k);
}

static double sim_noise (void)
{
    g_rng = (g_rng * 1103515245U) + 12345U;
    return ((double)((g_rng >> 8) % 2001U) / 1000.0 - 1.0) * W.noise;
}

/* ---- kernel, console and network stand-ins ---- */
ER tk_dly_tsk (RELTIM dlytim) { (void)dlytim; return 0; }
ID tk_cre_cyc (const T_CCYC * pk_ccyc) { (void)pk_ccyc; return 1; }
ER tk_sta_cyc (ID cycid) { (void)cycid; return 0; }
ER tk_stp_cyc (ID cycid) { (void)cycid; return 0; }
ID tk_get_tid (void) { return 1; }
ER tk_wup_tsk (ID tskid) { (void)tskid; return 0; }
ER tk_slp_tsk (TMO tmout) { (void)tmout; return 0; }
INT tk_can_wup (ID tskid) { (void)tskid; return 0; }
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

INT tm_printf (const UB * format, ...)
{
    va_list ap;
    int     n;

    va_start(ap, format);
    n = vprintf((const char *)format, ap);
    va_end(ap);
    return n;
}

void app_main_delay (uint32_t delay_ms) { (void)delay_ms; }
uint32_t app_main_lock (void) { return 0U; }
void app_main_unlock (uint32_t saved_mask) { (void)saved_mask; }
bool app_main_install_gpio_irq (void (* p_handler)(uint32_t))
{
    (void)p_handler;
    return true;
}

bool net_mqtt_start (void) { return true; }
bool net_mqtt_send (const uint8_t * p_frame, size_t length)
{
    size_t index;

    ++g_robot_frames;
    g_robot_bytes += (uint32_t)length;
    if (NULL != getenv("SIM_R"))
    {
        printf("## R %.3f ", sim_t());
        for (index = 0U; index < length; ++index)
        {
            printf("%02x", p_frame[index]);
        }
        printf("\n");
    }
    return true;
}
bool net_mqtt_receive (uint8_t * p_command)
{
    uint32_t index;

    if (0U == g_cmd_count)
    {
        return false;
    }
    *p_command = g_cmd_queue[0];
    for (index = 1U; index < g_cmd_count; ++index)
    {
        g_cmd_queue[index - 1U] = g_cmd_queue[index];
    }
    --g_cmd_count;
    return true;
}
bool net_mqtt_online (void) { return true; }
uint8_t net_mqtt_rejected (void) { return 0U; }
void net_mqtt_poll (struct netif * p_interface) { (void)p_interface; }
bool net_mqtt_ipc_receive_map (uint8_t * p_map)
{
    if (!gb_map)
    {
        return false;
    }
    gb_map = false;
    memcpy(p_map, g_map_bytes, NAVMAP_LENGTH);
    return true;
}
bool net_mqtt_ipc_send (const uint8_t * p_frame, uint32_t length)
{
    (void)p_frame;
    (void)length;
    return true;
}
bool net_mqtt_ipc_receive (uint8_t * p_command)
{
    (void)p_command;
    return false;
}
bool net_mqtt_ipc_start (void) { return true; }

static void sim_command (mqttcmd_code_t code)
{
    if (g_cmd_count < sizeof(g_cmd_queue))
    {
        g_cmd_queue[g_cmd_count] = mqttcmd_byte(code);
        ++g_cmd_count;
    }
}

/* ---- board stand-ins ---- */
bool board_watchdog_start (uint32_t timeout_ms) { (void)timeout_ms; return true; }
void board_watchdog_feed (void) {}
bool board_watchdog_rebooted (void) { return false; }
bool board_init (void) { return true; }
void board_boot_timebase (void) {}
void board_boot_brake (void) {}
const char * board_init_diagnostic (void) { return "sim"; }
void board_console_restore (void) {}
uint64_t board_time_us (void) { return g_now_us; }

/* Raw counts as the encoders give them: + forward times the fitted sign. */
static int64_t sim_raw (int wheel)
{
    return g_count[wheel] * ((0 == wheel) ? ROBOT_LEFT_ENCODER_SIGN :
                                            ROBOT_RIGHT_ENCODER_SIGN);
}
void board_read_encoders (int32_t * p_left, int32_t * p_right,
                          uint32_t * p_errors)
{
    *p_left = (int32_t)(sim_raw(0) - g_read[0]);
    *p_right = (int32_t)(sim_raw(1) - g_read[1]);
    g_read[0] = sim_raw(0);
    g_read[1] = sim_raw(1);
    *p_errors = 0U;
    /* SIM_ENC_ERR_AT s (SIM_ENC_ERR_N reads in a row, default 1): a
     * left-wheel edge error.
     */
    if (sim_burst("SIM_ENC_ERR_AT", "SIM_ENC_ERR_N", &g_enc_failed))
    {
        *p_errors = BOARD_ENCODER_ERROR_LEFT;
        printf("## %.3f encoder error\n", sim_t());
    }
    /* SIM_ENC_SCATTER k: every k-th read reports an edge error, all run. */
    if (sim_scatter("SIM_ENC_SCATTER", &g_enc_calls))
    {
        *p_errors = BOARD_ENCODER_ERROR_LEFT;
    }
}
void board_encoders_discard (void)
{
    g_read[0] = sim_raw(0);
    g_read[1] = sim_raw(1);
}
void board_encoder_totals (uint16_t * p_left, uint16_t * p_right)
{
    *p_left = (uint16_t)(uint64_t)sim_raw(0);
    *p_right = (uint16_t)(uint64_t)sim_raw(1);
}

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
    double s = 0.5 * (double)(g_count[0] + g_count[1]) * m;
    double th;
    double ph = 0.0;
    double fwd;
    double left;
    double up;
    double rel = s - g_hump_from;
    double t = sim_t();

    if ((g_now_us - g_accel_us) < 10000U)
    {
        return BOARD_ACCEL_PENDING;
    }
    g_accel_us = g_now_us;
    /* SIM_FAIL_AT s (SIM_FAIL_N reads in a row, default 1): the read fails
     * on the bus.
     */
    if (sim_burst("SIM_FAIL_AT", "SIM_FAIL_N", &g_accel_failed))
    {
        printf("## %.3f accelerometer read fails\n", t);
        return BOARD_ACCEL_INVALID;
    }
    /* SIM_FAIL_SCATTER k: every k-th fresh read fails, all run. */
    if (sim_scatter("SIM_FAIL_SCATTER", &g_accel_calls))
    {
        return BOARD_ACCEL_INVALID;
    }
    th = atan((sim_ground(rel) - sim_ground(rel - 0.12)) / 0.12) +
         (0.2 * CORE_PI / 180.0);
    if ((rel >= 0.0) && (rel < (2.0 * g_hump_len + g_hump_top)))
    {
        ph = g_hump_roll * CORE_PI / 180.0;
    }
    fwd = 1000.0 * sin(th) + ROBOT_IMU_REFERENCE_X_MG + 6.0 * sim_noise() / W.noise;
    left = 1000.0 * cos(th) * sin(ph) + ROBOT_IMU_REFERENCE_Y_MG +
           6.0 * sim_noise() / W.noise;
    up = 1000.0 * cos(th) * cos(ph) + 6.0 * sim_noise() / W.noise;
    /* SIM_LIFT_AT s: a pick-up, +300 mg up for 200 ms (the lift stop). */
    if ((NULL != getenv("SIM_LIFT_AT")) && (t >= atof(getenv("SIM_LIFT_AT"))) &&
        (t < (atof(getenv("SIM_LIFT_AT")) + 0.2)))
    {
        up += 300.0;
    }
    /* SIM_JERK_AT s: a motor jolt, +700 mg forward for 60 ms. */
    if ((NULL != getenv("SIM_JERK_AT")) && (t >= atof(getenv("SIM_JERK_AT"))) &&
        (t < (atof(getenv("SIM_JERK_AT")) + 0.06)))
    {
        fwd += 700.0;
    }
    /* SIM_SHAKE_AT s: a reversal's shake, zero-mean +-1.5 g forward and
     * +-0.8 g up, alternating every 10 ms sample, for 150 ms.
     */
    if ((NULL != getenv("SIM_SHAKE_AT")) &&
        (t >= atof(getenv("SIM_SHAKE_AT"))) &&
        (t < (atof(getenv("SIM_SHAKE_AT")) + 0.15)))
    {
        double sign = (((g_now_us / 10000U) % 2U) != 0U) ? 1.0 : -1.0;

        fwd += sign * 1500.0;
        up += sign * 800.0;
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

/** Distance along a ray from (ox, oy) at angle a to the box, or -1. */
static double box_ray (double ox, double oy, double a)
{
    double dx = cos(a), dy = sin(a);
    double t0 = -1e9, t1 = 1e9;
    double lo[2] = {g_box_x - g_box_hx, g_box_y - g_box_hy};
    double hi[2] = {g_box_x + g_box_hx, g_box_y + g_box_hy};
    double o[2] = {ox, oy}, d[2] = {dx, dy};
    int    axis;

    for (axis = 0; axis < 2; ++axis)
    {
        double ta;
        double tb;

        if (fabs(d[axis]) < 1e-12)
        {
            if ((o[axis] < lo[axis]) || (o[axis] > hi[axis]))
            {
                return -1.0;
            }
            continue;
        }
        ta = (lo[axis] - o[axis]) / d[axis];
        tb = (hi[axis] - o[axis]) / d[axis];
        if (ta > tb) { double s = ta; ta = tb; tb = s; }
        if (ta > t0) { t0 = ta; }
        if (tb < t1) { t1 = tb; }
    }
    return ((t0 <= t1) && (t0 > 0.0)) ? t0 : -1.0;
}

bool board_ultrasonic_trigger (void)
{
    scanner_config_t config;
    double physical, angle, ox, oy, best = -1.0, d;
    int    ray;

    if (!gb_box)
    {
        return false;
    }
    physical = ((double)g_servo_pulse - (double)ROBOT_SERVO_CENTRE_US) /
               ((g_servo_pulse < ROBOT_SERVO_CENTRE_US) ?
                (double)(ROBOT_SERVO_CENTRE_US - ROBOT_SERVO_MIN_US) :
                (double)(ROBOT_SERVO_MAX_US - ROBOT_SERVO_CENTRE_US)) *
               (double)ROBOT_SCAN_HALF_ANGLE_DEG;
    angle = g_th + (physical * (double)ROBOT_SERVO_ANGLE_SIGN) * CORE_PI / 180.0;
    ox = g_x + SIM_SONAR_AHEAD * cos(g_th);
    oy = g_y + SIM_SONAR_AHEAD * sin(g_th);
    for (ray = -2; ray <= 2; ++ray)
    {
        d = box_ray(ox, oy, angle + (ray * SIM_BEAM_DEG / 2.0) * CORE_PI / 180.0);
        if ((d > 0.0) && ((best < 0.0) || (d < best)))
        {
            best = d;
        }
    }
    app_cfg_scanner(&config);
    gb_echo = true;
    gb_echo_valid = (best > 0.0) && (best < 4.0);
    g_echo_us = gb_echo_valid ?
        (uint32_t)lround(2.0 * best / config.range.sound_speed_m_per_s * 1e6) :
        0U;
    return true;
}
void board_ultrasonic_service (void) {}
void board_ultrasonic_notify (void (* p_notify)(void)) { (void)p_notify; }
bool board_ultrasonic_result (uint32_t * p_pulse_us, bool * p_valid)
{
    *p_pulse_us = g_echo_us;
    *p_valid = gb_echo_valid;
    if (!gb_echo)
    {
        return false;
    }
    gb_echo = false;
    return true;
}
bool board_servo_us (uint16_t pulse_us)
{
    g_servo_pulse = pulse_us;
    return true;
}
bool board_motor_prepare (core_real_t left_duty, core_real_t right_duty,
                          board_motor_command_t * p_command)
{
    memset(p_command, 0, sizeof(*p_command));
    if ((!isfinite(left_duty)) || (!isfinite(right_duty)) ||
        (fabs(left_duty) > 1.0) || (fabs(right_duty) > 1.0))
    {
        return false;
    }
    p_command->left_level = ((uint32_t)(fmin(fabs(left_duty), ROBOT_MAX_DUTY)
                                        * (ROBOT_PWM_WRAP + 1U))) & 0x1FFFU;
    p_command->right_level = ((uint32_t)(fmin(fabs(right_duty),
                                              ROBOT_MAX_DUTY)
                                         * (ROBOT_PWM_WRAP + 1U))) & 0x1FFFU;
    p_command->left_direction =
        BOARD_DIRECTION_ENCODE((left_duty < 0.0) ? -1 : 1);
    p_command->right_direction =
        BOARD_DIRECTION_ENCODE((right_duty < 0.0) ? -1 : 1);
    p_command->b_valid = true;
    return true;
}
void board_motor_commit (const board_motor_command_t * p_command)
{
    g_wduty[0] = ((double)p_command->left_level / (ROBOT_PWM_WRAP + 1U)) *
                 board_direction_decode(p_command->left_direction);
    g_wduty[1] = ((double)p_command->right_level / (ROBOT_PWM_WRAP + 1U)) *
                 board_direction_decode(p_command->right_direction);
    gb_brake = false;
}
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

/* ---- world ---- */
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
        while (g_chars[idx] != p_text[i])
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
            pos += (0U != (g_patterns[idx] & (0x100U >> e))) ? wide : narrow;
        }
    }
    g_bar_edge[k] = pos;
    g_bar_black[k] = 0;
    g_bars = k;
}

/** Darkness at (x, y): the tape 1, a bar W.card. */
static double shade_at (double x, double y)
{
    int j;

    if (gb_corner)
    {
        if (((fabs(y) <= SIM_LINE_HALF) && (x > -1.0) &&
             (x <= g_corner_x + SIM_LINE_HALF)) ||
            ((fabs(x - g_corner_x) <= SIM_LINE_HALF) && (y <= SIM_LINE_HALF) &&
             (y > -3.0)) ||
            ((fabs(y - g_corner_cross_y) <= SIM_LINE_HALF) &&
             (fabs(x - g_corner_x) <= 0.3)))
        {
            return 1.0;
        }
    }
    else if (W.b_line && (fabs(y) <= SIM_LINE_HALF) && (x > -1.0) &&
             (x < g_line_end))
    {
        return 1.0;
    }
    if (W.b_cross && (fabs(x - W.cross_x) <= SIM_LINE_HALF))
    {
        return 1.0;
    }
    if (W.b_bars && (y >= SIM_BARS_FROM) && (y <= SIM_BARS_TO))
    {
        for (j = 0; j < g_bars; ++j)
        {
            if ((x >= g_bar_edge[j]) && (x < g_bar_edge[j + 1]))
            {
                return (0 == g_bar_black[j]) ? 0.0 : W.card;
            }
        }
    }
    return 0.0;
}

/** Mean darkness of a spot x spot square at car-frame (forward, left). */
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

bool board_barcode_d0 (void)
{
    return (NULL != getenv("SIM_D0")) &&
           (cover_spot(W.b_ahead, W.b_left, W.b_spot) > 0.5);
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
    samples[0] = raw_of(cover_spot(W.ahead, 0.5 * W.spacing, W.spot),
                        ROBOT_IR_LEFT_WHITE, ROBOT_IR_LEFT_BLACK);
    samples[1] = raw_of(cover_spot(W.ahead, -0.5 * W.spacing, W.spot),
                        ROBOT_IR_RIGHT_WHITE, ROBOT_IR_RIGHT_BLACK);
    samples[2] = raw_of(cover_spot(W.b_ahead, W.b_left, W.b_spot),
                        ROBOT_IR_BARCODE_WHITE, ROBOT_IR_BARCODE_BLACK);
    /* SIM_LIFT_AT s: in the air from then on, the IR sees no surface (the
     * car's REC rows, 22 Nov: all three 100 % dark once lifted).
     */
    if ((NULL != getenv("SIM_LIFT_AT")) && (NULL == getenv("SIM_LIFT_NO_IR")) &&
        (sim_t() >= atof(getenv("SIM_LIFT_AT"))))
    {
        samples[0] = ROBOT_IR_LEFT_BLACK;
        samples[1] = ROBOT_IR_RIGHT_BLACK;
        samples[2] = ROBOT_IR_BARCODE_BLACK;
    }
    /* SIM_IR_FAIL_AT s (SIM_IR_FAIL_N reads in a row, default 1): the ADC
     * read fails.
     */
    if (sim_burst("SIM_IR_FAIL_AT", "SIM_IR_FAIL_N", &g_ir_failed))
    {
        printf("## %.3f IR read fails\n", sim_t());
        return false;
    }
    /* SIM_IR_SCATTER k: every k-th read fails, all run. */
    if (sim_scatter("SIM_IR_SCATTER", &g_ir_calls))
    {
        return false;
    }
    return true;
}

/** The wheels over dt: duty to speed with thresholds and a lag, counts. */
static void sim_step (double dt)
{
    double metres_per_count = (2.0 * CORE_PI * ROBOT_WHEEL_RADIUS_M) /
                              (double)ROBOT_ENCODER_COUNTS_PER_REV;
    double target;
    double magnitude;
    double v;
    double omega;
    int64_t counts;
    int    w;

    for (w = 0; w < 2; ++w)
    {
        magnitude = fabs(g_wduty[w]);
        target = 0.0;
        if ((!gb_brake) &&
            ((magnitude >= W.d_static) ||
             ((fabs(g_wv[w]) > 0.002) && (magnitude >= W.d_kinetic))))
        {
            target = copysign((magnitude - W.d_kinetic) * 0.82, g_wduty[w]);
        }
        g_wv[w] += (target - g_wv[w]) *
                   fmin(1.0, dt / (gb_brake ? 0.02 :
                                   ((0.0 == g_wduty[w]) ? W.coast_tau : 0.06)));
        g_wcount[w] += g_wv[w] * dt / metres_per_count;
        counts = (int64_t)g_wcount[w];
        g_wcount[w] -= (double)counts;
        g_count[w] += counts;
    }
    v = 0.5 * (g_wv[0] + g_wv[1]);
    omega = (g_wv[1] - g_wv[0]) / ROBOT_TRACK_WIDTH_M * W.scrub;
    g_th += omega * dt;
    g_x += v * cos(g_th) * dt;
    g_y += v * sin(g_th) * dt;
}

/** The car as measured (bench sim world_measured, 4 mm side spot). */
static void world_measured (void)
{
    memset(&W, 0, sizeof(W));
    W.ahead = 0.130;
    W.spacing = 0.018;
    W.spot = 0.006;
    W.b_ahead = 0.070;
    W.b_left = 0.080;
    W.b_spot = 0.004;
    W.b_line = true;
    W.scrub = 1.0;
    W.d_static = 0.14;
    W.d_kinetic = 0.10;
    W.noise = 25.0;
    W.coast_tau = 0.06;
    W.card = 0.49;
}

/* ---- the scheduler ---- */
static const char * const g_state_names[] =
{
    "INIT", "IDLE", "FOLLOW", "NAVIGATE", "SCAN", "BYPASS", "REACQUIRE",
    "FAULT", "RANGE_WAIT"
};
static const char * const g_fault_names[] =
{
    "NONE", "LOCAL_STOP", "REMOTE_STOP", "OWNER_TIMEOUT", "TILT",
    "COLLISION", "ENCODER", "MOTION", "OPTICAL", "SUPERVISOR", "IMU", "SCAN",
    "LEFT_STALL", "RIGHT_STALL", "MOTOR_OUTPUT", "TILT_UNAVAILABLE"
};

/* SIM_TIME=<us>: host time of every Vehicle step (the mission), each step
 * over <us> printed as "## t vehicle step N us (host)", the longest at the
 * END. The host is far faster than the car's Cortex-M0+ (no FPU: every
 * double in software), so read it as where the work is, not how long the
 * car takes. Off: no output, nothing changed.
 */
static double   g_time_over_us = -1.0;
static double   g_time_worst_us;
static double   g_time_worst_t;

/** Host microseconds, monotonic. */
static double sim_host_us (void)
{
    struct timespec now;

    (void)clock_gettime(CLOCK_MONOTONIC, &now);
    return ((double)now.tv_sec * 1e6) + ((double)now.tv_nsec / 1e3);
}

/** The Vehicle step, timed when SIM_TIME is set. */
static void sim_vehicle_step (uint32_t now_ms)
{
    double start_us;
    double used_us;

    if (g_time_over_us < 0.0)
    {
        robot_app_vehicle_step(now_ms);
        return;
    }
    start_us = sim_host_us();
    robot_app_vehicle_step(now_ms);
    used_us = sim_host_us() - start_us;
    if (used_us > g_time_worst_us)
    {
        g_time_worst_us = used_us;
        g_time_worst_t = sim_t();
    }
    if (used_us > g_time_over_us)
    {
        printf("## %.3f vehicle step %.0f us (host)\n", sim_t(), used_us);
    }
}

/** One millisecond: the car, then every task step due, by priority. */
static void sim_ms (uint32_t ms)
{
    uint32_t now_ms;

    g_now_us += 1000U;
    sim_step(0.001);
    now_ms = (uint32_t)(g_now_us / 1000U);
    if (0U == (ms % 2U))
    {
        robot_app_safety_step(now_ms);
    }
    if (0U == (ms % 5U))
    {
        robot_app_motion_step(now_ms);
    }
    if (0U == (ms % 2U))
    {
        robot_app_line_step(now_ms);
    }
    /* SIM_VSTALL_MS k: the Vehicle task busy for k ms of every 200 (a long
     * decode on the car), so the IMU task's samples queue up meanwhile.
     */
    if ((1U == (ms % 5U)) &&
        ((NULL == getenv("SIM_VSTALL_MS")) ||
         ((ms % 200U) >= (uint32_t)atoi(getenv("SIM_VSTALL_MS")))))
    {
        sim_vehicle_step(now_ms);
    }
    if (3U == (ms % 10U))
    {
        robot_app_imu_step(now_ms);
    }
    if (1U == (ms % 2U))
    {
        robot_app_scan_step(now_ms);
    }
    if (2U == (ms % 5U))
    {
        robot_app_comm_step(now_ms);
    }
    if (7U == (ms % 200U))
    {
        robot_app_telemetry_step(now_ms);
    }
}

/** START over MQTT at start_s, STOP at stop_s (if > 0); runs until the
 * mission's run has ended and 3 s more have passed (its report), or limit_s.
 */
static void sim_run (double x, double y, double th_deg, double start_s,
                     double stop_s, double limit_s)
{
    robot_app_diagnostic_t diag;
    uint16_t last_mask = 0U;
    uint32_t last_code = 0U;
    uint32_t last_state = 99U;
    uint32_t ms;
    uint32_t end_ms = 0U;
    bool     b_started = false;
    bool     b_stopped = false;
    bool     b_reset_sent = false;
    bool     b_start2_sent = false;
    bool     b_ran = false;
    const mission_output_t * p_out;

    g_x = x;
    g_y = y;
    g_th = th_deg * CORE_PI / 180.0;
    if (!robot_app_init())
    {
        printf("## robot_app_init refused\n");
    }
    for (ms = 0U; ms < (uint32_t)(limit_s * 1000.0); ++ms)
    {
        if ((!b_started) && (sim_t() >= start_s))
        {
            b_started = true;
            sim_command(MQTTCMD_START);
            printf("## %.3f START sent\n", sim_t());
        }
        if ((stop_s > 0.0) && (!b_stopped) && (sim_t() >= stop_s))
        {
            b_stopped = true;
            sim_command(MQTTCMD_STOP);
            printf("## %.3f STOP sent\n", sim_t());
        }
        /* SIM_RESET_AT / SIM_START2_AT s: RESET, and a second START. */
        if ((NULL != getenv("SIM_RESET_AT")) && (!b_reset_sent) &&
            (sim_t() >= atof(getenv("SIM_RESET_AT"))))
        {
            b_reset_sent = true;
            sim_command(MQTTCMD_RESET);
            printf("## %.3f RESET sent\n", sim_t());
        }
        if ((NULL != getenv("SIM_START2_AT")) && (!b_start2_sent) &&
            (sim_t() >= atof(getenv("SIM_START2_AT"))))
        {
            b_start2_sent = true;
            sim_command(MQTTCMD_START);
            printf("## %.3f START sent again\n", sim_t());
        }
        sim_ms(ms);
        p_out = mission_output();
        if (p_out->state != last_state)
        {
            last_state = p_out->state;
            printf("## %.3f state %s at x %.0f y %.0f mm\n", sim_t(),
                   g_state_names[last_state % 9U], g_x * 1000.0,
                   g_y * 1000.0);
        }
        robot_app_diagnostic(&diag);
        /* A new first cause (a stop) or its clearing shows too. */
        if ((diag.mask != last_mask) || (diag.code != last_code))
        {
            last_mask = diag.mask;
            last_code = diag.code;
            printf("## %.3f fault mask 0x%04x first %s stop %u\n", sim_t(),
                   (unsigned)diag.mask, g_fault_names[diag.code & 0xFU],
                   (unsigned)diag.stop_source);
        }
        if (p_out->b_running)
        {
            /* A second run (SIM_START2_AT) restarts the 3 s end wait. */
            b_ran = true;
            end_ms = 0U;
        }
        else if (b_ran && (0U == end_ms))
        {
            end_ms = ms + 3000U;
        }
        if ((0U != end_ms) && (ms >= end_ms))
        {
            break;
        }
    }
    robot_app_diagnostic(&diag);
    printf("## END t %.1f s at x %.0f y %.0f mm heading %+.0f deg; first %s,"
           " mask 0x%04x; robot frames %u (%u B)%s\n", sim_t(), g_x * 1000.0,
           g_y * 1000.0, g_th * 180.0 / CORE_PI,
           g_fault_names[diag.code & 0xFU], (unsigned)diag.mask,
           (unsigned)g_robot_frames, (unsigned)g_robot_bytes,
           b_ran ? "" : "; NO RUN STARTED");
    if (g_time_over_us >= 0.0)
    {
        printf("## longest vehicle step %.0f us (host) at %.3f s\n",
               g_time_worst_us, g_time_worst_t);
    }
}

int main (int argc, char ** argv)
{
    const char * p_case = (argc > 1) ? argv[1] : "straight";

    if (NULL != getenv("SIM_TIME"))
    {
        g_time_over_us = atof(getenv("SIM_TIME"));
    }
    world_measured();
    if (0 == strcmp(p_case, "straight"))
    {
        printf("===== ROBOT STRAIGHT: the line, STOP at 8 s\n");
        sim_run(0.0, 0.0, 4.0, 1.5, 8.0, 15.0);
    }
    else if (0 == strcmp(p_case, "junction"))
    {
        double cross = (argc > 2) ? atof(argv[2]) : 0.50;

        W.b_bars = true;
        bars_build("*A*", 0.003, 0.009, 0.10 + W.b_ahead);
        W.b_cross = true;
        W.cross_x = cross;
        printf("===== ROBOT JUNCTION: *A* card, crossing at %.0f mm\n",
               cross * 1000.0);
        sim_run(0.0, 0.0, 4.0, 1.5, 14.0, 24.0);
    }
    else if (0 == strcmp(p_case, "corner"))
    {
        static const char letters[NAVMAP_MOVES] = {'Q', 'Z', 'A', 'F'};

        gb_corner = true;
        g_corner_x = (argc > 2) ? atof(argv[2]) : 0.6;
        g_corner_cross_y = -((argc > 3) ? atof(argv[3]) : 0.5);
        W.b_bars = true;
        bars_build("*A*", 0.003, 0.009, 0.10 + W.b_ahead);
        gb_map = navmap_pack(letters, g_map_bytes);
        printf("===== ROBOT CORNER: *A* card, 90 deg right turn at %.0f mm,"
               " crossing %.0f mm down; map forward Q, U-turn Z, left A,"
               " right F\n", g_corner_x * 1000.0, -g_corner_cross_y * 1000.0);
        sim_run(0.0, 0.0, 4.0, 1.5, 25.0, 34.0);
    }
    else if (0 == strcmp(p_case, "long"))
    {
        double at = (argc > 2) ? atof(argv[2]) : 22.0;

        W.b_bars = true;
        bars_build("*A*", 0.003, 0.009, at);
        W.b_cross = true;
        W.cross_x = at + 0.4;
        g_line_end = at + 3.0;
        printf("===== ROBOT LONG: *A* card at %.1f m, crossing at %.1f m\n",
               at, at + 0.4);
        sim_run(0.0, 0.0, 4.0, 1.5, (at + 0.8) / 0.08 + 20.0,
                (at + 0.8) / 0.08 + 30.0);
    }
    else if ((0 == strcmp(p_case, "ramp")) && (argc > 5))
    {
        g_hump_h = atof(argv[2]) / 1000.0;
        g_hump_len = atof(argv[3]) / 1000.0;
        g_hump_top = atof(argv[4]) / 1000.0;
        g_hump_roll = atof(argv[5]);
        g_hump_from = getenv("SIM_FROM") ? atof(getenv("SIM_FROM")) : 0.6;
        printf("===== ROBOT RAMP: %.0f mm up over %.0f mm, top %.0f mm, roll"
               " %+.1f deg, from %.0f mm\n", g_hump_h * 1000.0,
               g_hump_len * 1000.0, g_hump_top * 1000.0, g_hump_roll,
               g_hump_from * 1000.0);
        sim_run(0.0, 0.0, 6.0, 1.5, 16.0, 24.0);
    }
    else if (0 == strcmp(p_case, "obstacle"))
    {
        gb_box = true;
        g_box_x = ((argc > 2) ? atof(argv[2]) : 900.0) / 1000.0;
        g_box_y = 0.0;
        g_box_hy = ((argc > 3) ? atof(argv[3]) : 100.0) / 2000.0;
        g_box_hx = ((argc > 4) ? atof(argv[4]) : 100.0) / 2000.0;
        printf("===== ROBOT OBSTACLE: %.0f x %.0f mm box on the line at %.0f"
               " mm\n", g_box_hy * 2000.0, g_box_hx * 2000.0,
               g_box_x * 1000.0);
        sim_run(0.0, 0.0, 4.0, 1.5, 70.0, 80.0);
    }
    else
    {
        printf("usage: robot_sim straight | junction [cross_m] | corner"
               " [corner_m cross_m] | long [at_m] | ramp h len top roll |"
               " obstacle [x w d]\n");
        return 2;
    }
    return 0;
}
