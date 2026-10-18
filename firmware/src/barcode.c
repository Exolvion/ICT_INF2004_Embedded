/**
 * @file barcode.c
 * @brief Bounded distance-domain run acquisition and explicit codebook match.
 */
#include "barcode.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#if DBL_MANT_DIG < 53
#error "barcode requires at least 53 double mantissa bits"
#endif

static bool barcode_limits_valid (const barcode_config_t * p_config);
static void barcode_reject (barcode_state_t * p_state,
                            barcode_reason_t reason,
                            barcode_output_t * p_output);
static void barcode_transition (const barcode_config_t * p_config,
                                barcode_state_t * p_state,
                                uint32_t now_ms,
                                barcode_output_t * p_output);
static void barcode_acquire (const barcode_config_t * p_config,
                             barcode_state_t * p_state, double darkness,
                             uint32_t now_ms,
                             barcode_output_t * p_output);
static void barcode_finish (const barcode_config_t * p_config,
                            barcode_state_t * p_state,
                            barcode_output_t * p_output);
static bool barcode_symbol_matches (const barcode_config_t * p_config,
                                    const barcode_state_t * p_state,
                                    const barcode_symbol_t * p_symbol);

/** @brief Defaults are engineering hypotheses and do not enable decoding. */
void barcode_defaults (barcode_config_t * p_config)
{
    if (NULL == p_config)
    {
        return;
    }
    memset(p_config, 0, sizeof(*p_config));
    p_config->black_on             = 0.75;
    p_config->black_off            = 0.55;
    p_config->minimum_run_m        = 0.001;
    p_config->maximum_run_m        = 0.060;
    p_config->quiet_zone_m         = 0.015;
    p_config->maximum_frame_m      = 0.250;
    p_config->maximum_step_m       = 0.0002;
    p_config->reverse_tolerance_m  = 0.0005;
    p_config->absolute_tolerance_m = 0.0003;
    p_config->relative_tolerance   = 0.15;
    p_config->minimum_scale        = 1.0;
    p_config->maximum_scale        = 1.0;
    p_config->duplicate_distance_m = 0.200;
    p_config->frame_timeout_ms     = 10000U;
    p_config->maximum_gap_ms       = 50U;
}

/** @brief Reject malformed, impossible or non-finite configured patterns. */
bool barcode_config_valid (const barcode_config_t * p_config)
{
    uint32_t symbol;
    uint32_t run;
    double   total;
    double   minimum_scale;
    double   maximum_scale;
    const barcode_symbol_t * p_entry;

    if ((NULL == p_config) || (!barcode_limits_valid(p_config)))
    {
        return false;
    }
    minimum_scale = p_config->b_scale_tolerant ? p_config->minimum_scale : 1.0;
    maximum_scale = p_config->b_scale_tolerant ? p_config->maximum_scale : 1.0;
    for (symbol = 0U; symbol < p_config->symbol_count; ++symbol)
    {
        p_entry = &p_config->symbols[symbol];
        if ((0U == p_entry->run_count) ||
            (p_entry->run_count > BARCODE_MAX_RUNS) ||
            (0U == (p_entry->run_count % 2U)) ||
            ('\0' == p_entry->label) ||
            (p_entry->action < BARCODE_ACTION_LEFT) ||
            (p_entry->action > BARCODE_ACTION_UTURN))
        {
            return false;
        }
        total = 0.0;
        for (run = 0U; run < p_entry->run_count; ++run)
        {
            if ((!isfinite(p_entry->width_m[run])) ||
                (p_entry->width_m[run] < p_config->minimum_run_m) ||
                (p_entry->width_m[run] > p_config->maximum_run_m) ||
                ((p_entry->width_m[run] * minimum_scale) <
                 p_config->minimum_run_m) ||
                ((p_entry->width_m[run] * maximum_scale) >
                 p_config->maximum_run_m) ||
                ((1U == (run % 2U)) &&
                 ((p_entry->width_m[run] * maximum_scale) >=
                  p_config->quiet_zone_m)))
            {
                return false;
            }
            total += p_entry->width_m[run];
        }
        if ((!isfinite(total)) ||
            ((total * maximum_scale) > p_config->maximum_frame_m))
        {
            return false;
        }
    }
    return true;
}

/** @brief Clear fixed-size state; no heap or global ownership is needed. */
void barcode_init (barcode_state_t * p_state)
{
    if (p_state != NULL)
    {
        memset(p_state, 0, sizeof(*p_state));
    }
}

/** @brief Validate and clock one spatial acquisition sample. */
bool barcode_step (const barcode_config_t * p_config,
                   barcode_state_t * p_state, double darkness,
                   double distance_m, uint32_t now_ms,
                   barcode_output_t * p_output)
{
    double advance;

    if (NULL == p_output)
    {
        return false;
    }
    memset(p_output, 0, sizeof(*p_output));
    if ((NULL == p_state) || (!barcode_config_valid(p_config)) ||
        (!isfinite(darkness)) || (!isfinite(distance_m)) ||
        (darkness < 0.0) || (darkness > 1.0))
    {
        barcode_init(p_state);
        p_output->event  = BARCODE_EVENT_REJECTED;
        p_output->reason = BARCODE_REASON_INPUT;
        return false;
    }
    if ((!p_config->b_calibration_confirmed) ||
        (!p_config->b_codebook_confirmed) ||
        (0U == p_config->symbol_count))
    {
        barcode_init(p_state);
        p_output->event = BARCODE_EVENT_INHIBITED;
        return true;
    }
    if (!p_state->b_started)
    {
        p_state->b_started       = true;
        p_state->high_water_m    = distance_m;
        p_state->run_start_m     = distance_m;
        p_state->previous_ms     = now_ms;
        p_state->b_current_black = (darkness >= p_config->black_on);
    }
    advance = distance_m - p_state->high_water_m;
    if ((!isfinite(advance)) || (advance > p_config->maximum_step_m) ||
        (advance < -p_config->reverse_tolerance_m))
    {
        barcode_reject(p_state,
            (advance < -p_config->reverse_tolerance_m) ?
            BARCODE_REASON_REVERSE : BARCODE_REASON_DISTANCE, p_output);
        p_state->b_started = false;
        return true;
    }
    if (((now_ms - p_state->previous_ms) > p_config->maximum_gap_ms) ||
        ((p_state->b_frame) && ((now_ms - p_state->frame_start_ms) >
                              p_config->frame_timeout_ms)))
    {
        barcode_reject(p_state, BARCODE_REASON_TIMEOUT, p_output);
        p_state->b_started = false;
        return true;
    }
    p_state->previous_ms = now_ms;
    if (advance > 0.0)
    {
        p_state->high_water_m = distance_m;
    }
    barcode_acquire(p_config, p_state, darkness, now_ms, p_output);
    return true;
}

/** @brief Bound every numeric setting to avoid invalid state arithmetic. */
static bool barcode_limits_valid (const barcode_config_t * p_config)
{
    const double limits[] =
    {
        p_config->black_on, p_config->black_off,
        p_config->minimum_run_m, p_config->maximum_run_m,
        p_config->quiet_zone_m, p_config->maximum_frame_m,
        p_config->maximum_step_m, p_config->reverse_tolerance_m,
        p_config->absolute_tolerance_m, p_config->relative_tolerance,
        p_config->duplicate_distance_m
    };
    size_t index;
    const size_t count = sizeof(limits) / sizeof(limits[0]);

    for (index = 0U; index < count; ++index)
    {
        if ((!isfinite(limits[index])) || (limits[index] < 0.0))
        {
            return false;
        }
    }
    if ((p_config->b_scale_tolerant) &&
        ((!isfinite(p_config->minimum_scale)) ||
         (!isfinite(p_config->maximum_scale)) ||
         (p_config->minimum_scale < BARCODE_MIN_SCALE) ||
         (p_config->maximum_scale > BARCODE_MAX_SCALE) ||
         (p_config->minimum_scale > p_config->maximum_scale)))
    {
        return false;
    }
    return (p_config->black_on <= 1.0) &&
           (p_config->black_off < p_config->black_on) &&
           (p_config->minimum_run_m > 0.0) &&
           (p_config->minimum_run_m < p_config->quiet_zone_m) &&
           (p_config->maximum_run_m >= p_config->minimum_run_m) &&
           (p_config->maximum_run_m < p_config->maximum_frame_m) &&
           (p_config->maximum_frame_m <= 100.0) &&
           (p_config->quiet_zone_m < p_config->maximum_frame_m) &&
           (p_config->maximum_step_m > 0.0) &&
           (p_config->maximum_step_m <=
            (0.20 * p_config->minimum_run_m)) &&
           (p_config->reverse_tolerance_m < p_config->minimum_run_m) &&
           (p_config->absolute_tolerance_m < p_config->minimum_run_m) &&
           (p_config->relative_tolerance < 0.5) &&
           (p_config->duplicate_distance_m <= 100.0) &&
           (p_config->frame_timeout_ms > 0U) &&
           (p_config->frame_timeout_ms <= 3600000U) &&
           (p_config->maximum_gap_ms > 0U) &&
           (p_config->maximum_gap_ms <= 60000U) &&
           (p_config->symbol_count <= BARCODE_MAX_SYMBOLS);
}

/** @brief Discard uncertain frame while retaining duplicate history. */
static void barcode_reject (barcode_state_t * p_state,
                            barcode_reason_t reason,
                            barcode_output_t * p_output)
{
    p_output->event     = BARCODE_EVENT_REJECTED;
    p_output->reason    = reason;
    p_output->run_count = p_state->run_count;
    p_state->b_frame    = false;
    p_state->b_armed    = false;
    p_state->b_candidate = false;
    p_state->run_count   = 0U;
    p_state->run_start_m = p_state->high_water_m;
}

/** @brief Commit an edge only after one-quarter minimum-run persistence. */
static void barcode_transition (const barcode_config_t * p_config,
                                barcode_state_t * p_state,
                                uint32_t now_ms,
                                barcode_output_t * p_output)
{
    double width = p_state->candidate_start_m - p_state->run_start_m;

    if (p_state->b_frame)
    {
        if (p_state->run_count >= BARCODE_MAX_RUNS)
        {
            barcode_reject(p_state, BARCODE_REASON_OVERFLOW, p_output);
        }
        else if ((width < p_config->minimum_run_m) ||
                 (width > p_config->maximum_run_m))
        {
            barcode_reject(p_state, BARCODE_REASON_WIDTH, p_output);
        }
        else
        {
            p_state->widths_m[p_state->run_count] = width;
            ++p_state->run_count;
        }
    }
    else if ((p_state->b_candidate_black) &&
             ((p_state->b_armed) || (width >= p_config->quiet_zone_m)))
    {
        p_state->b_frame        = true;
        p_state->b_armed        = false;
        p_state->run_count      = 0U;
        p_state->frame_start_m  = p_state->candidate_start_m;
        p_state->frame_start_ms = now_ms;
    }
    else
    {
        /* A black element without a leading quiet zone is ignored. */
    }
    p_state->b_current_black = p_state->b_candidate_black;
    p_state->b_candidate     = false;
    p_state->run_start_m     = p_state->candidate_start_m;
}

/** @brief Hysteretic edges reject short spatial noise; quiet white closes. */
static void barcode_acquire (const barcode_config_t * p_config,
                             barcode_state_t * p_state, double darkness,
                             uint32_t now_ms,
                             barcode_output_t * p_output)
{
    bool b_black = p_state->b_current_black;
    double span;
    double observed_end_m;

    if (darkness >= p_config->black_on)
    {
        b_black = true;
    }
    else if (darkness <= p_config->black_off)
    {
        b_black = false;
    }
    else
    {
        /* Keep the stable classification inside the hysteresis band. */
    }
    if (b_black != p_state->b_current_black)
    {
        if (!p_state->b_candidate)
        {
            p_state->b_candidate       = true;
            p_state->b_candidate_black = b_black;
            p_state->candidate_start_m = p_state->high_water_m;
        }
        if ((p_state->high_water_m - p_state->candidate_start_m) >=
            (0.25 * p_config->minimum_run_m))
        {
            barcode_transition(p_config, p_state, now_ms, p_output);
        }
    }
    else
    {
        p_state->b_candidate = false;
    }
    /* Edge persistence and trailing quiet are outside the encoded width. */
    observed_end_m = p_state->b_candidate ? p_state->candidate_start_m
                                          : p_state->high_water_m;
    span = observed_end_m - p_state->run_start_m;
    if (!p_state->b_current_black)
    {
        observed_end_m = p_state->run_start_m;
    }
    if ((p_state->b_frame) &&
        ((observed_end_m - p_state->frame_start_m) >
         p_config->maximum_frame_m))
    {
        barcode_reject(p_state, BARCODE_REASON_OVERFLOW, p_output);
    }
    else if ((!p_state->b_current_black) && (!p_state->b_candidate) &&
             (span >= p_config->quiet_zone_m))
    {
        if (p_state->b_frame)
        {
            barcode_finish(p_config, p_state, p_output);
        }
        p_state->b_armed = true;
    }
    else if ((p_state->b_frame) && (p_state->b_current_black) &&
             (span > p_config->maximum_run_m))
    {
        barcode_reject(p_state, BARCODE_REASON_WIDTH, p_output);
    }
    else
    {
        /* Incomplete frames remain pending until bounded termination. */
    }
}

/** @brief Require a unique matching configured pattern and reject repeats. */
static void barcode_finish (const barcode_config_t * p_config,
                            barcode_state_t * p_state,
                            barcode_output_t * p_output)
{
    uint32_t symbol;
    uint32_t matches = 0U;
    const barcode_symbol_t * p_match = NULL;

    p_output->run_count   = p_state->run_count;
    p_output->frame_end_m = p_state->run_start_m;
    p_output->event       = BARCODE_EVENT_UNKNOWN;
    p_output->reason      = BARCODE_REASON_NO_MATCH;
    for (symbol = 0U; symbol < p_config->symbol_count; ++symbol)
    {
        if (barcode_symbol_matches(p_config, p_state,
                                    &p_config->symbols[symbol]))
        {
            ++matches;
            p_match = &p_config->symbols[symbol];
        }
    }
    if (matches > 1U)
    {
        p_output->reason = BARCODE_REASON_AMBIGUOUS;
    }
    else if (1U == matches)
    {
        if ((p_state->b_last_decode) &&
            (p_match->label == p_state->last_label) &&
            (fabs(p_state->run_start_m - p_state->last_decode_m) <
             p_config->duplicate_distance_m))
        {
            p_output->event  = BARCODE_EVENT_REJECTED;
            p_output->reason = BARCODE_REASON_DUPLICATE;
        }
        else
        {
            p_output->event    = BARCODE_EVENT_DECODED;
            p_output->reason   = BARCODE_REASON_NONE;
            p_output->action   = p_match->action;
            p_output->label    = p_match->label;
            p_state->last_label    = p_match->label;
            p_state->last_decode_m = p_state->run_start_m;
            p_state->b_last_decode = true;
        }
    }
    else
    {
        /* An unmatched pattern never acquires a motion action. */
    }
    p_state->b_frame   = false;
    p_state->run_count = 0U;
}

/** @brief Match all observed runs with optional bounded uniform scaling. */
static bool barcode_symbol_matches (const barcode_config_t * p_config,
                                    const barcode_state_t * p_state,
                                    const barcode_symbol_t * p_symbol)
{
    uint32_t run;
    double   tolerance;
    double   scale = 1.0;
    double   observed_total = 0.0;
    double   template_total = 0.0;
    double   expected;

    if (p_symbol->run_count != p_state->run_count)
    {
        return false;
    }
    if (p_config->b_scale_tolerant)
    {
        for (run = 0U; run < p_symbol->run_count; ++run)
        {
            observed_total += p_state->widths_m[run];
            template_total += p_symbol->width_m[run];
        }
        scale = observed_total / template_total;
        if ((!isfinite(scale)) || (scale < p_config->minimum_scale) ||
            (scale > p_config->maximum_scale))
        {
            return false;
        }
    }
    for (run = 0U; run < p_symbol->run_count; ++run)
    {
        expected = scale * p_symbol->width_m[run];
        tolerance = p_config->absolute_tolerance_m +
                    (p_config->relative_tolerance * expected);
        if (fabs(p_state->widths_m[run] - expected) > tolerance)
        {
            return false;
        }
    }
    return true;
}

/* End of file. */

