/** @file scanner.h
 * @brief Bounded nonblocking servo/range acquisition and apparent profile.
 *
 * Zero degrees is forward; positive angles point left. Angles describe the
 * calibrated sensor beam, not raw servo commands. Direct double follows the
 * project numeric deviation D-NUMERIC. State and output belong to one task.
 */
#ifndef SCANNER_H
#define SCANNER_H

#include <stdbool.h>
#include <stdint.h>
#include "core.h"

#define SCANNER_MAX_SAMPLES (80U)

typedef enum
{
    SCANNER_IDLE = 0,
    SCANNER_POSITION,
    SCANNER_SETTLE,
    SCANNER_WAIT_ECHO,
    SCANNER_COMPLETE,
    SCANNER_ABORTED
} scanner_phase_t;

typedef enum
{
    SCANNER_SIDE_NONE = 0,
    SCANNER_SIDE_LEFT,
    SCANNER_SIDE_RIGHT
} scanner_side_t;

typedef struct
{
    core_ultrasonic_config_t range;
    double obstacle_distance_m;
    double robot_half_width_m;
    double uncertainty_margin_m;
    double required_clearance_m;
    uint32_t settle_ms;
    uint32_t trigger_interval_ms;
    uint32_t echo_timeout_ms;
    bool b_calibrated;
} scanner_config_t;

typedef struct
{
    bool     b_ready;
    uint32_t pulse_us;
} scanner_echo_t;

typedef struct
{
    int32_t             angle_deg;
    core_range_sample_t range;
    bool                b_fine;
} scanner_sample_t;

typedef struct
{
    double nearest_distance_m;
    double apparent_width_m;
    double left_clearance_m;
    double right_clearance_m;
    scanner_side_t preferred_side;
    uint32_t       valid_samples;
    uint32_t       unknown_samples;
    bool           b_profile_valid;
    bool           b_obstacle_seen;
    bool           b_left_observed;
    bool           b_right_observed;
    bool           b_aborted;
} scanner_result_t;

typedef struct
{
    bool    b_set_servo;
    int32_t angle_deg;
    bool    b_trigger;
    bool    b_done;
} scanner_output_t;

typedef struct
{
    scanner_config_t config;
    scanner_sample_t samples[SCANNER_MAX_SAMPLES];
    scanner_result_t result;
    scanner_phase_t  phase;
    uint32_t         sample_count;
    uint32_t         phase_started_ms;
    uint32_t         last_trigger_ms;
    int32_t          angle_deg;
    int32_t          fine_end_deg;
    bool             b_initialized;
    bool             b_triggered;
    bool             b_fine;
} scanner_state_t;

/** Set proposed 150 ms settle, 65 ms interval and 30 ms timeout defaults. */
void scanner_default_config (scanner_config_t * p_config);

/** Validate configuration and reset state. False leaves state unchanged. */
bool scanner_init (scanner_state_t * p_state,
                   const scanner_config_t * p_config);

/** Begin a -60..+60 degree coarse scan; calibrated and stationary required. */
bool scanner_start (scanner_state_t * p_state, uint32_t now_ms,
                    bool b_stationary);

/**
 * Advance acquisition without blocking. Caller supplies a completed echo only
 * for the current trigger; the board must reject noise/duplicate/stale edges.
 * A trigger output requests a board-generated >=10 us pulse. Polling need not
 * itself have microsecond resolution. Millisecond timestamps wrap safely when
 * consecutive calls are less than 2^31 ms apart. Moving aborts the entire scan.
 * The fine pass uses 2 degree spacing around coarse obstacle/unknown regions.
 * Completion recenters the servo; the board must still wait for it to settle.
 * Missing and out-of-range observations never certify clear space.
 * Clearances are sampled radial free-distance margins, not swept-path proof;
 * preferred_side is a planning hint that requires monitored motion and recheck.
 */
bool scanner_step (scanner_state_t * p_state, uint32_t now_ms,
                   bool b_stationary, const scanner_echo_t * p_echo,
                   scanner_output_t * p_output);

#endif /* SCANNER_H */

/** End of file. */

