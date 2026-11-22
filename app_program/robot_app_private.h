/** @file robot_app_private.h
 * @brief Private to the robot app's parts (robot_app.c and its subsystem
 * files): what more than one part uses. Split out of the one file by
 * tools/reorg/split_c.py (22 Nov 2026).
 */
#ifndef ROBOT_APP_PRIVATE_H
#define ROBOT_APP_PRIVATE_H

/** @file robot_app.c
 * @brief Single-core RTOS application: sensor owners and one motion writer.
 * Shared records are copied inside bounded interrupt-masked regions. No I2C,
 * formatting, scan, network or floating-point computation occurs while locked.
 * Shared data uses minimum-width fields grouped by producing task (see
 * robot_app_shared_t). Every write to g_shared, including each bit-field
 * store, happens inside app_main_lock(): a bit-field store rewrites its whole
 * storage unit and the Cortex-M0+ has no exclusive load/store, so PRIMASK is
 * the only safe read-modify-write. Only robot_app_latch_locked(), the
 * status helper, the vehicle inhibit and the approved reset write the fault
 * word.
 * BARR-NUMERIC-01: direct double uses the core.h binary64 precision contract.
 */
#include "robot_app.h"
#include "app_main.h"
#include "app_cfg.h"
#include "bitpack.h"
#include "board.h"
#include "course_cfg.h"
#include "mission.h"
#include "mqttcmd.h"
#include "net_mqtt.h"
#include "robot_config.h"
#include <math.h>
#include <stddef.h>
#include "telemetry.h"
#include <string.h>
#if !defined(ROBOT_APP_MISSION)
#define ROBOT_APP_MISSION 1
#endif

#define ROBOT_APP_MG_TO_MPS2 (0.00980665)
#define ROBOT_APP_SERIAL_HALF (UINT32_C(2147483648))

/* Noise control (the team's rule, 22 Nov: one bad sample is noise). Every
 * sample-driven stop keeps a level (robot_app_noise): +2 for a bad sample,
 * -1 for a good one; it latches only on a bad sample that brings the level
 * to its limit. A bad sample now and then (under one in three) never
 * latches; ROBOT_APP_NOISE_LIMIT(n) latches on n bad samples in a row. The
 * original supervisor (ROBOT_APP_MISSION 0) keeps its specified latch on
 * the first bad sample.
 */
#define ROBOT_APP_NOISE_UP (2U)
#if ROBOT_APP_MISSION
#define ROBOT_APP_NOISE_LIMIT(n) (ROBOT_APP_NOISE_UP * (n))
#endif
#if !(ROBOT_APP_MISSION)
#define ROBOT_APP_NOISE_LIMIT(n) (1U)
#endif
#define ROBOT_APP_ANGLE_BITS (12U)
#define ROBOT_APP_STEER_BITS (8U)
#define ROBOT_APP_DONE_MASK (3U)

/* Signed duty in per mille: +/-511 holds the +/-350 duty limit. */
#define ROBOT_APP_DUTY_BITS (10U)

/* Half-counts per barcode decoder unit. The decoder stores run widths in
 * 8 bits (at most 255 units): at 2533 counts per turn one half-count is
 * 0.040 mm, so 8 of them (0.32 mm, near the 0.28 mm it was designed with
 * at 360 counts) keep the 60 mm widest-run limit at 186 units.
 */
#define ROBOT_APP_BARCODE_UNIT_HC (8U)

/** Fault record: written only under the lock (see file comment). A reset
 * is approved only if no stop was raised after the Vehicle snapshot that
 * authorised it. Two flags encode that exactly: every stop sets
 * b_stop_since_snapshot and clears b_reset_clean; Vehicle clears the first
 * at its snapshot and sets b_reset_clean = !b_stop_since_snapshot when it
 * publishes the authorisation; Motion reads b_reset_clean live. A counter
 * could alias: a starved Motion task leaves the window open until the
 * 1 s watchdog, long enough for hundreds of soft stops.
 */
typedef struct
{
    uint32_t          mask                  : 16;
    robot_app_fault_t first                 : 4;
    uint32_t          stop_source           : 2;
    bool              b_hard                : 1;
    bool              b_inhibit             : 1;
    bool              b_stop_latched        : 1;
    bool              b_stop_since_snapshot : 1;
    bool              b_reset_clean         : 1;
} robot_app_faults_t;

/** Liveness stamps of the six owners that gate the watchdog; Safety and
 * Telemetry stamps had no reader and are not kept.
 */
typedef enum
{
    ROBOT_APP_HEALTH_MOTION = 0,
    ROBOT_APP_HEALTH_LINE,
    ROBOT_APP_HEALTH_VEHICLE,
    ROBOT_APP_HEALTH_IMU,
    ROBOT_APP_HEALTH_SCAN,
    ROBOT_APP_HEALTH_COMM,
    ROBOT_APP_HEALTH_COUNT
} robot_app_health_t;

/** Vehicle task (P4) word and the packed request it publishes. Motion
 * clears b_reset_motion and b_reset_hard; the button handler in this task
 * sets b_start and b_reset. b_reset_hard (mission: a RESET) lets that reset
 * clear a hard fault too. request_version wraps: two reads are at most 5 ms
 * apart.
 */
typedef struct
{
    motion_request_packed_t request;
    vehicle_state_t         state           : 4;
    vehicle_scan_t          scan_request    : 2;
    uint32_t                request_version : 4;
    uint32_t                run_id          : 8;
    bool                    b_reset_motion  : 1;
    bool                    b_reset_hard    : 1;
    bool                    b_start         : 1;
    bool                    b_reset         : 1;
} robot_app_vehicle_t;

/** Line task (P3) words. The junction edge and navigation are events that
 * Vehicle clears; Vehicle also resets the barcode action and symbol at run
 * start. Steering is Q7 two's complement; bu/br saturate at 255.
 */
typedef struct
{
    line_ctrl_status_t status           : 3;
    uint32_t           steering_q7      : 8;
    bool               b_junction_edge  : 1;
    vehicle_nav_t      navigation       : 3;
    barcode_action_t   barcode_action   : 3;
    uint32_t           barcode_symbol   : 4;
    uint32_t           barcode_unknown  : 8;
    uint32_t           barcode_rejected : 8;
} robot_app_line_t;

/** Scan task (P6) front reading: the pose at its trigger, the packed range
 * and two completion counters. Scan is their only writer; Vehicle compares
 * them with the values it last acted on.
 */
typedef struct
{
    core_pose_packed_t  pose;
    core_range_packed_t range;
    uint32_t            coarse_done : 2;
    uint32_t            fine_done   : 2;
} robot_app_front_t;

/** The mission's sonar and servo hand-over (ROBOT_APP_MISSION). Vehicle
 * asks (a new servo_seq: set servo_us; a new trigger_seq: one trigger);
 * Scan answers with the trigger it completed and that echo. 41 bits.
 */
typedef struct
{
    uint32_t servo_us    : 12;
    uint32_t servo_seq   : 4;
    uint32_t trigger_seq : 4;
    uint32_t done_seq    : 4;
    uint32_t pulse_us    : 15;
    bool     b_valid     : 1;
} robot_app_sonar_t;

typedef struct
{
    uint32_t            health[ROBOT_APP_HEALTH_COUNT];
    uint32_t            line_ms;
    uint32_t            imu_ms;
    uint32_t            tilt_ms;
    uint32_t            range_ms;
    uint32_t            path_ms;
    uint32_t            previous_path_ms;
    uint32_t            first_fault_ms;
    robot_app_faults_t  faults;
    robot_app_vehicle_t vehicle;
    robot_app_line_t    line;
    motion_snapshot_t   motion;
    terrain_shared_t    terrain;
    scanner_shared_t    scan;
    robot_app_front_t   front;
    /* Motion publishes each signed centre path, in wrapping half-counts
     * (left + right counts), with its release stamp and keeps the previous
     * pair, in one critical section (robot_app_commit).
     */
    uint16_t            path_hc;
    uint16_t            previous_path_hc;
    uint8_t             ignored_commands;
    uint8_t             dropped_telemetry;
    /* Motion's commanded duties, signed per mille (the mission's lift check
     * needs their direction), and the sonar hand-over.
     */
    uint32_t            left_duty_pm  : ROBOT_APP_DUTY_BITS;
    uint32_t            right_duty_pm : ROBOT_APP_DUTY_BITS;
    robot_app_sonar_t   sonar;
} robot_app_shared_t;

/** Vehicle task flags and the completion counters it last acted on. */
typedef struct
{
    bool     b_button_down    : 1;
    bool     b_button_latched : 1;
    bool     b_line_lock      : 1;
    uint32_t coarse_seen      : 2;
    uint32_t fine_seen        : 2;
} robot_app_vehicle_flags_t;

/** What one IMU step adds to the evidence (robot_app_imu_note). */
typedef enum
{
    ROBOT_APP_IMU_GAP = 0,     /* a fresh sample: the gap since the last */
    ROBOT_APP_IMU_READ,        /* any read: its wall time */
    ROBOT_APP_IMU_FAILED,
    ROBOT_APP_IMU_SATURATED,
    ROBOT_APP_IMU_LATE,
    ROBOT_APP_IMU_REFUSED
} robot_app_imu_event_t;
extern uint32_t g_timing_window;
extern robot_app_shared_t g_shared;
extern motion_t g_motion;
extern line_ctrl_runtime_t g_line_ctrl_runtime;
extern line_ctrl_state_t g_line_ctrl;
extern barcode_runtime_t g_barcode_runtime;
extern barcode_state_t g_barcode;
extern terrain_state_t g_terrain;
extern scanner_state_t g_scanner;
#if !ROBOT_APP_MISSION
extern core_pose_packed_t g_route_anchor;
#endif
extern uint64_t g_motion_us;
extern double g_half_count_m;
extern uint32_t g_scan_centred_ms;
extern uint32_t g_stationary_since_ms;
extern uint32_t g_imu_window_ms;
extern uint16_t g_line_left_total;
extern uint16_t g_line_right_total;
extern robot_app_vehicle_flags_t g_vehicle_flags;
extern bool gb_optical_valid;

void robot_app_copy (robot_app_shared_t * p_copy);
void robot_app_latch_locked (bool b_hard, robot_app_fault_t cause,
                                    uint32_t mask, uint32_t source);
void robot_app_fault_set (bool b_hard, robot_app_bit_t bit);
void robot_app_stop_set (uint32_t source);
void robot_app_status_set (robot_app_bit_t bit, bool b_active);
bool robot_app_active (vehicle_state_t state);
bool robot_app_tilt_exceeded (const terrain_shared_t * p_terrain);
#if !ROBOT_APP_MISSION
bool robot_app_ready (const robot_app_shared_t * p_copy,
                             uint32_t now_ms);
#endif
bool robot_app_noise (uint8_t * p_level, bool b_bad, uint32_t limit);
void robot_app_owner_note (const uint32_t health[ROBOT_APP_HEALTH_COUNT],
                                  uint32_t now_ms);
bool robot_app_sensors_init (void);
void robot_app_health (robot_app_health_t index, uint32_t now_ms);
void robot_app_periodic (void (* p_step)(uint32_t), uint32_t period_ms,
                                uint32_t index);
uint32_t robot_app_now (void);
void robot_app_button (uint32_t now_ms, vehicle_state_t state);
#if ROBOT_APP_MISSION
void robot_app_mission_input (const robot_app_shared_t * p_copy);
void robot_app_mission_publish (uint32_t now_ms);
#endif
#if !(ROBOT_APP_MISSION)
void robot_app_vehicle_input (const robot_app_shared_t * p_copy,
                                     uint32_t now_ms,
                                     vehicle_input_t * p_input);
void robot_app_bypass_input (const robot_app_shared_t * p_copy,
                                    vehicle_input_t * p_input);
#endif
uint32_t robot_app_age (uint32_t now_ms, uint32_t stamp_ms);

#include "robot_comm.h"
#include "robot_motion.h"
#include "robot_line.h"
#include "robot_imu.h"
#include "robot_scan.h"

#endif /* ROBOT_APP_PRIVATE_H */

/* End of file. */

