/** @file mission.h
 * @brief The mission (mission.c): line following, barcode cards, junction
 * moves, the hump, the obstacle go-around and the end report.
 *
 * In the robot app it is the Vehicle task's controller (robot_app.c): every
 * sensor and output stays with its owner task (Line: IR, IMU: accelerometer,
 * Scan: sonar and servo, Motion: wheels). The owners hand their samples in
 * (mission_ir_push, mission_accel_push, mission_input_t); the mission hands
 * its motion request, servo pulse and sonar trigger back (mission_output_t).
 * The bench image (firmware/bench/build.sh, BENCH_IMAGE) builds the same file
 * as the old bench loop instead (bench_loop).
 */
#ifndef MISSION_H
#define MISSION_H

#include "motion.h"
#include <stdbool.h>
#include <stdint.h>

/** Ring depths of the owner samples (Line every 2 ms, IMU every 10 ms; the
 * Vehicle task drains both every 5 ms, and in a wait every 5 ms too).
 */
#define MISSION_IR_RING (32U)
#define MISSION_ACCEL_RING (8U)
/** Sequence numbers of the servo and sonar hand-over, 4 bits each. */
#define MISSION_SEQ_MASK (0xFU)

/** One command for the mission's step. */
typedef enum
{
    MISSION_CMD_NONE = 0,
    MISSION_CMD_START,
    MISSION_CMD_STOP
} mission_command_t;

/** The Vehicle task's copy of the other owners' data for one step. */
typedef struct
{
    motion_snapshot_t motion;      /* Motion's latest snapshot */
    int32_t           left_duty_pm;  /* signed duty, per mille */
    int32_t           right_duty_pm;
    uint32_t          sonar_seq;   /* the trigger Scan last completed */
    uint32_t          sonar_pulse_us;
    bool              b_sonar_valid;
    mission_command_t command;
} mission_input_t;

/** What the mission asks of the owners after a step (or during a wait). */
typedef struct
{
    motion_request_packed_t request;   /* to Motion; renewed every step */
    uint32_t servo_us    : 12;         /* servo pulse to set, 0 none */
    uint32_t servo_seq   : 4;          /* a new pulse: seq + 1 */
    uint32_t trigger_seq : 4;          /* a new sonar trigger: seq + 1 */
    uint32_t state       : 4;          /* vehicle_state_t of the mission */
    bool     b_running   : 1;          /* a run is under way */
} mission_output_t;

/** Robot app: once, before the tasks start (after the Motion and sensor
 * set-up). Sends the BOOT line and centres the servo.
 */
void mission_init (void);

/** Robot app: the mission's wheel controller settings (PID gains, step
 * limit, wheel speed cap) over app_cfg_motion()'s, for the Motion task.
 */
void mission_motion_config (motion_config_t * p_config);

/** Line task: one IR sample (raw ADC, 12 bits) and the wheels' 16-bit
 * encoder totals when it was taken. A full ring drops the oldest.
 */
void mission_ir_push (const uint16_t raw[3], bool b_valid,
                      uint16_t left_total, uint16_t right_total);

/** IMU task: one fresh accelerometer sample, sensor axes in mg, and when
 * it was read (board_time_us, low 32 bits): the mission's lift and hump
 * integrals take each sample at that time.
 */
void mission_accel_push (const int16_t milligravity[3], uint32_t read_us);

/** IMU task, every 100 ms: read the compass (the LSM303DLHC magnetometer,
 * on the IMU's I2C bus) for the mission's report. The first call switches
 * it on. Device builds only; the host simulations read none.
 */
void mission_mag_poll (void);

/** Vehicle task: one mission step on this step's input. */
void mission_step (const mission_input_t * p_input);

/** The mission's current requests (valid after mission_init). */
const mission_output_t * mission_output (void);

/** Robot app (robot_app.c) for the mission: publish mission_output() at
 * once, mark the Vehicle task alive and, if b_wait, sleep one 5 ms period
 * (a send waiting for room); else give the lower tasks 1 ms every 10 ms
 * (a long decode after the stop).
 */
void mission_hook_alive (bool b_wait);

/** Bench image only (BENCH_IMAGE): the bench loop; never returns. */
void bench_loop (void);

#endif /* MISSION_H */

/* End of file. */

