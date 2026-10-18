/**
 * @file line_ctrl.c
 * @brief Calibrated optical evidence and bounded event timing.
 */
#include "line_ctrl.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#if DBL_MANT_DIG < 53
#error "line_ctrl requires at least 53 double mantissa bits"
#endif

static double line_ctrl_clamp (double value);
static void line_ctrl_lost (const line_ctrl_config_t * p_config,
                            line_ctrl_state_t * p_state, uint32_t now_ms,
                            line_ctrl_output_t * p_output);
static void line_ctrl_events (const line_ctrl_config_t * p_config,
                              line_ctrl_state_t * p_state,
                              uint32_t now_ms,
                              line_ctrl_output_t * p_output);

/** @brief Set proposed thresholds; all calibration starts unconfirmed. */
void line_ctrl_defaults (line_ctrl_config_t * p_config)
{
    uint32_t channel;

    if (NULL == p_config)
    {
        return;
    }
    memset(p_config, 0, sizeof(*p_config));
    for (channel = 0U; channel < LINE_CTRL_CHANNELS; ++channel)
    {
        p_config->white_adc[channel] = 4095U;
        p_config->black_adc[channel] = 0U;
    }
    p_config->minimum_contrast_adc = 200U;
    p_config->maximum_adc          = 4095U;
    p_config->black_on             = 0.75;
    p_config->black_off            = 0.55;
    p_config->minimum_signal       = 0.15;
    p_config->junction_debounce_ms = 20U;
    p_config->lost_timeout_ms      = 100U;
    p_config->maximum_gap_ms       = 50U;
}

/** @brief Validate finite thresholds and all three calibration spans. */
bool line_ctrl_config_valid (const line_ctrl_config_t * p_config)
{
    uint32_t channel;
    double   contrast;

    if (NULL == p_config)
    {
        return false;
    }
    if ((0U == p_config->minimum_contrast_adc) ||
        (0U == p_config->maximum_adc) ||
        (0U == p_config->junction_debounce_ms) ||
        (0U == p_config->lost_timeout_ms) ||
        (0U == p_config->maximum_gap_ms) ||
        (p_config->maximum_gap_ms > 60000U) ||
        (p_config->lost_timeout_ms > 60000U) ||
        (p_config->junction_debounce_ms > 60000U) ||
        (!isfinite(p_config->black_on)) ||
        (!isfinite(p_config->black_off)) ||
        (!isfinite(p_config->minimum_signal)))
    {
        return false;
    }
    if ((p_config->black_on > 1.0) ||
        (p_config->black_off < 0.0) ||
        (p_config->black_off >= p_config->black_on) ||
        (p_config->minimum_signal <= 0.0) ||
        (p_config->minimum_signal >= 1.0))
    {
        return false;
    }
    for (channel = 0U; channel < LINE_CTRL_CHANNELS; ++channel)
    {
        if ((p_config->white_adc[channel] > p_config->maximum_adc) ||
            (p_config->black_adc[channel] > p_config->maximum_adc))
        {
            return false;
        }
        /* uint16_t values are represented exactly by double. */
        contrast = (double) p_config->black_adc[channel] -
                   (double) p_config->white_adc[channel];
        if (fabs(contrast) < p_config->minimum_contrast_adc)
        {
            return false;
        }
    }
    return true;
}

/** @brief Reset all ownership-local state to the unstarted condition. */
void line_ctrl_init (line_ctrl_state_t * p_state)
{
    if (p_state != NULL)
    {
        memset(p_state, 0, sizeof(*p_state));
    }
}

/** @brief Normalize channels and derive evidence; see header contract. */
bool line_ctrl_step (const line_ctrl_config_t * p_config,
                     line_ctrl_state_t * p_state,
                     const uint16_t * p_raw, uint32_t now_ms,
                     line_ctrl_output_t * p_output)
{
    uint32_t channel;
    double   span;
    double   signal;

    if (NULL == p_output)
    {
        return false;
    }
    memset(p_output, 0, sizeof(*p_output));
    p_output->status = LINE_CTRL_FAULT;
    if ((NULL == p_state) || (NULL == p_raw) ||
        (!line_ctrl_config_valid(p_config)))
    {
        line_ctrl_init(p_state);
        return false;
    }
    if (!p_config->b_calibration_confirmed)
    {
        line_ctrl_init(p_state);
        p_output->status = LINE_CTRL_INHIBITED;
        return true;
    }
    if ((p_state->b_started) &&
        ((now_ms - p_state->previous_ms) > p_config->maximum_gap_ms))
    {
        line_ctrl_init(p_state);
        return false;
    }
    for (channel = 0U; channel < LINE_CTRL_CHANNELS; ++channel)
    {
        if (p_raw[channel] > p_config->maximum_adc)
        {
            line_ctrl_init(p_state);
            return false;
        }
        /* ADC values are uint16_t and exactly representable in double. */
        span = (double) p_config->black_adc[channel] -
               (double) p_config->white_adc[channel];
        p_output->darkness[channel] = line_ctrl_clamp(
            ((double) p_raw[channel] - p_config->white_adc[channel]) /
            span);
        if (p_output->darkness[channel] >= p_config->black_on)
        {
            p_state->b_black[channel] = true;
        }
        else if (p_output->darkness[channel] <= p_config->black_off)
        {
            p_state->b_black[channel] = false;
        }
        else
        {
            /* Hysteresis retains the previous classification. */
        }
    }
    p_state->b_started   = true;
    p_state->previous_ms = now_ms;
    signal = p_output->darkness[0] + p_output->darkness[1];
    p_output->confidence = line_ctrl_clamp(signal);
    if (signal >= p_config->minimum_signal)
    {
        p_output->steering = (p_output->darkness[1] -
                              p_output->darkness[0]) / signal;
    }
    p_output->b_barcode_black = p_state->b_black[2];
    line_ctrl_events(p_config, p_state, now_ms, p_output);
    return true;
}

/** @brief Clamp finite calibrated darkness to the closed unit interval. */
static double line_ctrl_clamp (double value)
{
    if (value < 0.0)
    {
        return 0.0;
    }
    if (value > 1.0)
    {
        return 1.0;
    }
    return value;
}

/** @brief Keep uncertain evidence separate from timed-out line loss. */
static void line_ctrl_lost (const line_ctrl_config_t * p_config,
                            line_ctrl_state_t * p_state, uint32_t now_ms,
                            line_ctrl_output_t * p_output)
{
    p_output->status = LINE_CTRL_TRACKING;
    if (p_output->confidence < p_config->minimum_signal)
    {
        if (!p_state->b_losing)
        {
            p_state->b_losing      = true;
            p_state->lost_since_ms = now_ms;
        }
        p_output->status = LINE_CTRL_SEARCHING;
        if ((now_ms - p_state->lost_since_ms) >= p_config->lost_timeout_ms)
        {
            p_output->status = LINE_CTRL_LOST;
        }
    }
    else
    {
        p_state->b_losing = false;
    }
}

/** @brief Debounce ambiguous junctions and uncertain/lost line evidence. */
static void line_ctrl_events (const line_ctrl_config_t * p_config,
                              line_ctrl_state_t * p_state,
                              uint32_t now_ms,
                              line_ctrl_output_t * p_output)
{
    if ((p_state->b_black[0]) && (p_state->b_black[1]))
    {
        p_state->b_losing = false;
        if (!p_state->b_junction_pending)
        {
            p_state->b_junction_pending  = true;
            p_state->junction_since_ms = now_ms;
        }
        p_output->status = LINE_CTRL_SEARCHING;
        if ((now_ms - p_state->junction_since_ms) >=
            p_config->junction_debounce_ms)
        {
            p_output->status = LINE_CTRL_JUNCTION;
            p_output->b_junction_edge = !p_state->b_junction_latched;
            p_state->b_junction_latched = true;
        }
    }
    else
    {
        p_state->b_junction_pending = false;
        p_state->b_junction_latched = false;
        line_ctrl_lost(p_config, p_state, now_ms, p_output);
    }
}

/* End of file. */

