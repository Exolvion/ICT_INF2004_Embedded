/** @file mission_board.c
 * @brief The mission: the robot app's owners as the mission's board calls
 * (rings, requests, the Motion snapshot). Split out of the one file by
 * tools/reorg/split_c.py (22 Nov 2026); what the parts share is in
 * mission_private.h.
 */
#define MISSION_BOARD_OWNER /* the real board calls here */
#include "mission_private.h"
#include "mission_board.h"
#if !defined(MISSION_SIM)

/** @brief No console in the robot app: a print goes nowhere. */
INT mission_no_console (const UB * p_format, ...)
{
    (void)p_format;
    return 0;
}
#endif
       /* full ring, saturating */
mission_accel_t  g_acc_ring[MISSION_ACCEL_RING];
uint8_t          g_acc_head;
         /* IMU task writes */
uint8_t          g_acc_tail;
         /* Vehicle task reads */
static uint32_t         g_acc_read_us;
      /* read time of the last taken */
uint16_t         g_ir_now[3];
        /* the step's own reading */
bool             gb_ir_now;
              /* the requests, published */
static bool             gb_sonar_waiting;
   /* a trigger not yet echoed */
uint8_t          g_motion_id = 1U;

   /* last velocity ID submitted */

/** @brief The step's own IR reading: the newest sample the Line task took
 * (mission_ir_drain). False if that read failed or none came yet.
 */
bool mission_read_ir (uint16_t samples[3])
{
    samples[0] = g_ir_now[0];
    samples[1] = g_ir_now[1];
    samples[2] = g_ir_now[2];
    return gb_ir_now;
}

/** @brief The oldest accelerometer sample the IMU task pushed. */
board_accel_status_t mission_read_accel (int16_t milligravity[3])
{
    mission_accel_t sample;
    uint32_t        saved = app_main_lock();
    bool            b_fresh = g_acc_tail != g_acc_head;

    if (b_fresh)
    {
        sample = g_acc_ring[g_acc_tail];
        g_acc_tail = (uint8_t)((g_acc_tail + 1U) % MISSION_ACCEL_RING);
    }
    app_main_unlock(saved);
    if (!b_fresh)
    {
        return BOARD_ACCEL_PENDING;
    }
    /* The fields hold the chip's 16-bit two's complement words. */
    milligravity[0] = (int16_t)(uint16_t)sample.x;
    milligravity[1] = (int16_t)(uint16_t)sample.y;
    milligravity[2] = (int16_t)(uint16_t)sample.z;
    g_acc_read_us = (uint32_t)sample.read_us;
    return BOARD_ACCEL_FRESH;
}

/** @brief True while the IMU task has samples queued for the mission. */
bool mission_accel_waiting (void)
{
    uint32_t saved = app_main_lock();
    bool     b_waiting = g_acc_tail != g_acc_head;

    app_main_unlock(saved);
    return b_waiting;
}

/** @brief When the IMU task read the sample mission_read_accel last gave,
 * on now_us's 64-bit clock (it is less than 2^32 us old).
 */
uint64_t mission_accel_sample_us (uint64_t now_us)
{
    return now_us - (uint64_t)(uint32_t)((uint32_t)now_us - g_acc_read_us);
}

/** @brief Ask the Scan task for one trigger; false while one is out (an
 * echo that never came is given up after MISSION_SONAR_LOST_US).
 */
#define MISSION_SONAR_LOST_US (200000U)
static uint64_t g_sonar_asked_us;

bool mission_sonar_trigger (void)
{
    uint64_t now_us = board_time_us();

    if (gb_sonar_waiting &&
        ((now_us - g_sonar_asked_us) < MISSION_SONAR_LOST_US))
    {
        return false;
    }
    g_out.trigger_seq = (g_out.trigger_seq + 1U) & MISSION_SEQ_MASK;
    gb_sonar_waiting = true;
    g_sonar_asked_us = now_us;
    return true;
}

/** @brief The Scan task services the sonar. */
void mission_sonar_service (void)
{
}

/** @brief The echo of the last trigger, once, when the Scan task has it. */
bool mission_sonar_result (uint32_t * p_pulse_us, bool * p_valid)
{
    if ((!gb_sonar_waiting) || (g_in.sonar_seq != g_out.trigger_seq))
    {
        return false;
    }
    gb_sonar_waiting = false;
    *p_pulse_us = g_in.sonar_pulse_us;
    *p_valid = g_in.b_sonar_valid;
    return true;
}

/** @brief Ask the Scan task for this servo pulse. */
bool mission_servo_us (uint16_t pulse_us)
{
    g_out.servo_us = (uint32_t)pulse_us & 0xFFFU;
    g_out.servo_seq = (g_out.servo_seq + 1U) & MISSION_SEQ_MASK;
    return true;
}

/** @brief Pack the request for the Motion task (published after the step,
 * or at once in a wait).
 */
void mission_publish (const motion_request_t * p_request)
{
    motion_request_pack(p_request, &g_out.request);
    if (MOTION_HALT != p_request->kind)
    {
        g_motion_id = p_request->command_id;
    }
}

/** @brief A HALT for the Motion task. */
void mission_publish_halt (void)
{
    motion_request_t halt;

    (void)memset(&halt, 0, sizeof(halt));
    halt.kind = MOTION_HALT;
    halt.command_id = g_motion_id;
    mission_publish(&halt);
}

/** @brief Brake now (the Motion task's next step applies the HALT). */
void mission_motor_brake (void)
{
    board_motor_brake();
    mission_publish_halt();
}

/** @brief Only the bench's duty sweep sets duties directly: not here. */
void mission_motor_duty (core_real_t left_duty, core_real_t right_duty)
{
    (void)left_duty;
    (void)right_duty;
}

/** @brief The Motion task reads the encoder deltas and their error bits. */
void mission_read_encoders (int32_t * p_left, int32_t * p_right,
                                   uint32_t * p_errors)
{
    *p_left = 0;
    *p_right = 0;
    *p_errors = 0U;
}

/** @brief The Motion task owns the deltas. */
void mission_encoders_discard (void)
{
}

/** @brief The Safety task runs the watchdog. */
bool mission_watchdog_start (uint32_t timeout_ms)
{
    (void)timeout_ms;
    return true;
}

/** @brief A long computation: stay alive (mission_hook_alive). */
void mission_watchdog_feed (void)
{
    mission_hook_alive(false);
}

/* The Motion task's pose when the run started: the bench's motion_init
 * started every run at x = y = 0, heading 0, and the mission relies on it.
 */
static core_pose_t g_origin;

/** @brief Wrap metres into the packed pose's +/-32.768 m. */
static double mission_wrap_m (double metres)
{
    while (metres > 32.768)
    {
        metres -= 65.536;
    }
    while (metres < -32.768)
    {
        metres += 65.536;
    }
    return metres;
}

/** @brief Radians into +/-pi. */
static double mission_wrap_rad (double angle)
{
    while (angle > CORE_PI)
    {
        angle -= 2.0 * CORE_PI;
    }
    while (angle < -CORE_PI)
    {
        angle += 2.0 * CORE_PI;
    }
    return angle;
}

/** @brief Note the Motion task's pose as this run's origin. */
void mission_origin (void)
{
    motion_output_t output;

    motion_snapshot_unpack(&g_in.motion, &output);
    g_origin = output.pose;
}

/** @brief The Motion task's latest output as motion_step gave it to the
 * bench: the snapshot (pose from this run's origin) and the duties.
 */
void mission_motion_output (motion_output_t * p_output)
{
    double dx;
    double dy;
    double c = cos(g_origin.yaw_rad);
    double s = sin(g_origin.yaw_rad);

    motion_snapshot_unpack(&g_in.motion, p_output);
    dx = mission_wrap_m(p_output->pose.x_m - g_origin.x_m);
    dy = mission_wrap_m(p_output->pose.y_m - g_origin.y_m);
    p_output->pose.x_m = (c * dx) + (s * dy);
    p_output->pose.y_m = (c * dy) - (s * dx);
    p_output->pose.yaw_rad = mission_wrap_rad(p_output->pose.yaw_rad -
                                              g_origin.yaw_rad);
    p_output->left_duty = (double)g_in.left_duty_pm / 1000.0;
    p_output->right_duty = (double)g_in.right_duty_pm / 1000.0;
}

/* End of file. */

