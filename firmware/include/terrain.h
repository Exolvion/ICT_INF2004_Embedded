/** @file terrain.h
 * @brief Quasi-static accelerometer terrain and encoder motion estimates.
 *
 * Sensor axes: x forward, y left, z up; stationary flat z is positive gravity.
 * Pitch is positive nose-up. No gyroscope signal is used or synthesized.
 * Direct double use follows project numeric deviation D-NUMERIC.
 */
#ifndef TERRAIN_H
#define TERRAIN_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    TERRAIN_INVALID = 0,
    TERRAIN_STATIONARY,
    TERRAIN_STEADY,
    TERRAIN_ACCELERATING,
    TERRAIN_BRAKING,
    TERRAIN_TURNING,
    TERRAIN_CLIMBING,
    TERRAIN_DESCENDING,
    TERRAIN_COLLISION_CANDIDATE
} terrain_motion_t;

typedef struct
{
    double gravity_mps2;
    double gravity_tolerance_mps2;
    double filter_time_s;
    double maximum_dt_s;
    double slope_enter_rad;
    double slope_exit_rad;
    double flat_confirm_s;
    double minimum_hump_m;
    double maximum_hump_m;
    double closure_tolerance_m;
    double stationary_speed_mps;
    double acceleration_threshold_mps2;
    double collision_deceleration_mps2;
    double turning_threshold_rps;
    bool   b_calibrated;
} terrain_config_t;

typedef struct
{
    double accel_x_mps2;
    double accel_y_mps2;
    double accel_z_mps2;
    double signed_distance_m;
    double encoder_yaw_rate_rps;
    double dt_s;
    bool   b_sensor_valid;
    bool   b_encoder_valid;
    bool   b_quasistatic;
} terrain_input_t;

typedef struct
{
    double           pitch_rad;
    double           roll_rad;
    double           encoder_yaw_rate_rps;
    double           height_estimate_m;
    double           hump_peak_m;
    double           last_hump_peak_m;
    double           run_maximum_m;
    terrain_motion_t motion;
    uint32_t         completed_humps;
    bool             b_tilt_valid;
    bool             b_height_valid;
    bool             b_hump_active;
    bool             b_hump_completed;
    bool             b_last_hump_valid;
    bool             b_collision_candidate;
} terrain_output_t;

typedef struct
{
    terrain_config_t config;
    terrain_output_t output;
    double           pitch_reference_rad;
    double           roll_reference_rad;
    double           previous_speed_mps;
    double           filtered_pitch_rad;
    double           filtered_roll_rad;
    double           flat_time_s;
    bool             b_initialized;
    bool             b_reference_valid;
    bool             b_filter_valid;
    bool             b_speed_valid;
    bool             b_active_valid;
    bool             b_descent_seen;
} terrain_state_t;

/** Populate proposed thresholds; calibration remains false. Null is ignored. */
void terrain_default_config (terrain_config_t * p_config);

/** Validate configuration and reset all state. False leaves state unchanged. */
bool terrain_init (terrain_state_t * p_state,
                   const terrain_config_t * p_config);

/**
 * Capture mounting reference from a known-flat stationary mean acceleration.
 * Caller must supply a previously averaged, calibrated sample and stationary
 * evidence; one arbitrary live sample is not a calibration procedure.
 * Refuses active humps, non-gravity samples and uncalibrated configuration.
 */
bool terrain_reference (terrain_state_t * p_state,
                        double accel_x_mps2,
                        double accel_y_mps2,
                        double accel_z_mps2,
                        bool b_stationary);

/**
 * Advance one sample. Invalid samples explicitly invalidate current estimates
 * and the active hump, while preserving previously valid completed run maxima.
 * Height integrates signed path * sin(filtered pitch), a chassis-path estimate.
 * A complete hump needs uphill onset, a forward descent outside the flat band,
 * then flat dwell and height closure. Flat crests leave the event open.
 * This does not establish physical ground height. Caller serializes all calls.
 */
bool terrain_step (terrain_state_t * p_state,
                   const terrain_input_t * p_input,
                   terrain_output_t * p_output);

#endif /* TERRAIN_H */

/** End of file. */

