/**
 * @file barcode.h
 * @brief Configurable spatial run decoder; no course symbology is assumed.
 * @details Frames start/end with black and require white quiet zones.
 * Encoder distance increases in the intended scan direction, in metres.
 * This explicit framing model itself needs course-owner confirmation.
 * Direct double use is a proposed project BARR-C 5.4 deviation.
 */
#ifndef BARCODE_H
#define BARCODE_H

#include <stdbool.h>
#include <stdint.h>

#define BARCODE_MAX_SYMBOLS (16U)
#define BARCODE_MAX_RUNS    (32U)
#define BARCODE_MIN_SCALE   (0.5)
#define BARCODE_MAX_SCALE   (2.0)

typedef enum
{
    BARCODE_ACTION_UNKNOWN = 0,
    BARCODE_ACTION_LEFT,
    BARCODE_ACTION_RIGHT,
    BARCODE_ACTION_STRAIGHT,
    BARCODE_ACTION_UTURN
} barcode_action_t;

typedef enum
{
    BARCODE_EVENT_NONE = 0,
    BARCODE_EVENT_DECODED,
    BARCODE_EVENT_UNKNOWN,
    BARCODE_EVENT_REJECTED,
    BARCODE_EVENT_INHIBITED
} barcode_event_t;

typedef enum
{
    BARCODE_REASON_NONE = 0,
    BARCODE_REASON_INPUT,
    BARCODE_REASON_REVERSE,
    BARCODE_REASON_DISTANCE,
    BARCODE_REASON_TIMEOUT,
    BARCODE_REASON_OVERFLOW,
    BARCODE_REASON_WIDTH,
    BARCODE_REASON_NO_MATCH,
    BARCODE_REASON_AMBIGUOUS,
    BARCODE_REASON_DUPLICATE
} barcode_reason_t;

typedef struct
{
    double           width_m[BARCODE_MAX_RUNS];
    uint8_t          run_count;
    char             label;
    barcode_action_t action;
} barcode_symbol_t;

typedef struct
{
    barcode_symbol_t symbols[BARCODE_MAX_SYMBOLS];
    double           black_on;
    double           black_off;
    double           minimum_run_m;
    double           maximum_run_m;
    double           quiet_zone_m;
    double           maximum_frame_m;
    double           maximum_step_m;
    double           reverse_tolerance_m;
    double           absolute_tolerance_m;
    double           relative_tolerance;
    double           minimum_scale;
    double           maximum_scale;
    double           duplicate_distance_m;
    uint32_t         frame_timeout_ms;
    uint32_t         maximum_gap_ms;
    uint8_t          symbol_count;
    bool             b_calibration_confirmed;
    bool             b_codebook_confirmed;
    bool             b_scale_tolerant;
} barcode_config_t;

typedef struct
{
    double   widths_m[BARCODE_MAX_RUNS];
    double   high_water_m;
    double   run_start_m;
    double   candidate_start_m;
    double   frame_start_m;
    double   last_decode_m;
    uint32_t previous_ms;
    uint32_t frame_start_ms;
    uint8_t  run_count;
    char     last_label;
    bool     b_started;
    bool     b_current_black;
    bool     b_candidate;
    bool     b_candidate_black;
    bool     b_frame;
    bool     b_armed;
    bool     b_last_decode;
} barcode_state_t;

typedef struct
{
    barcode_event_t  event;
    barcode_reason_t reason;
    barcode_action_t action;
    char             label;
    uint8_t          run_count;
    double           frame_end_m;
} barcode_output_t;

/** @brief Set unconfirmed illustrative limits and an empty codebook. */
void barcode_defaults (barcode_config_t * p_config);

/** @brief Check structural validity; confirmation gates are independent. */
bool barcode_config_valid (const barcode_config_t * p_config);

/** @brief Reset history; NULL is ignored. Call before the first sample. */
void barcode_init (barcode_state_t * p_state);

/**
 * @brief Sample calibrated darkness and monotonically forward distance.
 * @param darkness Finite calibrated [0,1], from the sidebarcode sensor.
 * @param distance_m Finite cumulative encoder path coordinate, metres.
 * @param now_ms Monotonic wrapping milliseconds with bounded sample gaps.
 * @return False for invalid pointer/configuration/sample; true otherwise.
 * @details Output is initialized on every call. A decoded event occurs
 * once after a trailing quiet zone, with action UNKNOWN on every other
 * result. Reverse motion, stale input and overflow clear the frame and
 * require a fresh quiet zone. Quiet-zone/repeat distances, thresholds,
 * widths and timing limits must be confirmed for the actual course.
 * maximum_frame_m covers the first black edge through the final black edge;
 * it excludes leading/trailing quiet zones and edge-confirmation travel.
 * The default compares physical widths directly. Optional b_scale_tolerant
 * fits one observed/template total-width ratio in [minimum_scale,
 * maximum_scale], then requires every run to match that scaled template.
 * Scale bounds must lie in [BARCODE_MIN_SCALE, BARCODE_MAX_SCALE]; all
 * scaled template widths must remain within the physical acquisition limits.
 * Absolute/relative tolerances apply to each scaled run, not to scale bounds.
 * This does not recover unobserved runs or relax maximum_step_m, quiet zones,
 * minimum_run_m, framing, direction, or the two confirmation gates.
 * Call from one owner; reset if configuration changes during a scan.
 */
bool barcode_step (const barcode_config_t * p_config,
                   barcode_state_t * p_state, double darkness,
                   double distance_m, uint32_t now_ms,
                   barcode_output_t * p_output);

#endif /* BARCODE_H */

/* End of file. */

