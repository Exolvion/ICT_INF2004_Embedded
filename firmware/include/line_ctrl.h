/**
 * @file line_ctrl.h
 * @brief Two line-edge channels plus a calibrated sidebarcode channel.
 * @details ADC units are counts; darkness is 0 white to 1 black. This
 * module assumes the physical optical arrangement has been verified.
 * Direct double use is a proposed project BARR-C 5.4 deviation.
 */
#ifndef LINE_CTRL_H
#define LINE_CTRL_H

#include <stdbool.h>
#include <stdint.h>

#define LINE_CTRL_CHANNELS (3U)

typedef enum
{
    LINE_CTRL_INHIBITED = 0,
    LINE_CTRL_TRACKING,
    LINE_CTRL_SEARCHING,
    LINE_CTRL_JUNCTION,
    LINE_CTRL_LOST,
    LINE_CTRL_FAULT
} line_ctrl_status_t;

typedef struct
{
    uint16_t white_adc[LINE_CTRL_CHANNELS];
    uint16_t black_adc[LINE_CTRL_CHANNELS];
    uint16_t minimum_contrast_adc;
    uint16_t maximum_adc;
    double   black_on;
    double   black_off;
    double   minimum_signal;
    uint32_t junction_debounce_ms;
    uint32_t lost_timeout_ms;
    uint32_t maximum_gap_ms;
    bool     b_calibration_confirmed;
} line_ctrl_config_t;

typedef struct
{
    uint32_t previous_ms;
    uint32_t lost_since_ms;
    uint32_t junction_since_ms;
    bool     b_started;
    bool     b_losing;
    bool     b_junction_pending;
    bool     b_junction_latched;
    bool     b_black[LINE_CTRL_CHANNELS];
} line_ctrl_state_t;

typedef struct
{
    double             darkness[LINE_CTRL_CHANNELS];
    double             confidence;
    double             steering;
    line_ctrl_status_t status;
    bool               b_junction_edge;
    bool               b_barcode_black;
} line_ctrl_output_t;

/** @brief Set unconfirmed illustrative thresholds; NULL is ignored. */
void line_ctrl_defaults (line_ctrl_config_t * p_config);

/** @brief Validate structure/ranges; confirmation is a separate gate. */
bool line_ctrl_config_valid (const line_ctrl_config_t * p_config);

/** @brief Clear history; NULL is ignored. Call before the first sample. */
void line_ctrl_init (line_ctrl_state_t * p_state);

/**
 * @brief Process a calibrated sample without allocation.
 * @param now_ms Monotonic wrapping milliseconds; gap must stay bounded.
 * @param p_raw Three ADC counts: left, right, sidebarcode.
 * @param p_output Positive steering means line evidence to the right.
 * @return False for NULL, invalid configuration, ADC or sample timing.
 * @details Valid but unconfirmed calibration returns true and INHIBITED.
 * A fault clears history. SEARCHING is uncertain and is not permission
 * to drive. JUNCTION is ambiguous both-black evidence, not a turn order.
 */
bool line_ctrl_step (const line_ctrl_config_t * p_config,
                     line_ctrl_state_t * p_state,
                     const uint16_t * p_raw, uint32_t now_ms,
                     line_ctrl_output_t * p_output);

#endif /* LINE_CTRL_H */

/* End of file. */

