/**
 * @file test_line_main.c
 * @brief Optical and configurable barcode sequence tests, not track proof.
 * @details The integer line and barcode decisions are checked against
 * copies of the removed double code kept here as references: ADC hysteresis
 * on every 12-bit code, the minimum signal on every production raw pair,
 * randomised line sequences in lockstep, and every barcode fixture replayed
 * through the legacy double decoder in lockstep. One fixture step is one
 * half-count; binary step sizes keep the legacy arithmetic exact.
 */
#include "test_line_main.h"

#include "barcode.h"
#include "line_ctrl.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_LINE_MAIN_EVENT_COUNT   (5U)
#define TEST_LINE_MAIN_REASON_COUNT  (11U)
/* Binary fixture quantum near 0.1 mm: one step is one half-count. */
#define TEST_LINE_MAIN_STEP_M        (1.0 / 8192.0)
#define TEST_LINE_MAIN_COARSE_M      (1.0 / 2048.0)
#define TEST_LINE_MAIN_WHITE_STEPS   (200U)
#define TEST_LINE_MAIN_BLACK_STEPS   (40U)
#define TEST_LINE_MAIN_WIDE_STEPS    (80U)
#define TEST_LINE_MAIN_LONG_STEPS    (120U)
#define TEST_LINE_MAIN_NOISE_STEPS   (2U)
#define TEST_LINE_MAIN_JITTER_STEPS  (2)
#define TEST_LINE_MAIN_REVERSE_STEPS (20)
/* test_line_main_symbol() with a WIDE final bar: 200 + 40 + 40 + 80 + 200. */
#define TEST_LINE_MAIN_SYMBOL_STEPS  (560)
#define TEST_LINE_MAIN_RUN_PAIRS     (18U)
#define TEST_LINE_MAIN_PAUSE_STEPS   (8U)
#define TEST_LINE_MAIN_SAMPLE_RUNS   (29U)
#define TEST_LINE_MAIN_SAMPLE_WHITE  (1288U)
#define TEST_LINE_MAIN_SAMPLE_STEP   (1.0 / 65536.0)
/* 42 steps per narrow element (was 100): 2.0 * 3 * 42 = 252 <= 255. */
#define TEST_LINE_MAIN_SAMPLE_UNIT   (42.0 * TEST_LINE_MAIN_SAMPLE_STEP)
#define TEST_LINE_MAIN_ADC_COUNT     (4096U)
#define TEST_LINE_MAIN_PI            (3.14159265358979323846)
#define TEST_LINE_MAIN_RADIUS_M      (0.0325)
#define TEST_LINE_MAIN_COUNTS        (360.0)
#define TEST_LINE_MAIN_MARGIN_HC     (1.0e-6)
#define TEST_LINE_MAIN_CALIBRATIONS  (1000U)
#define TEST_LINE_MAIN_QUOTIENTS     (100000U)
#define TEST_LINE_MAIN_SEQUENCE      (200000U)
#define TEST_LINE_MAIN_SEED          (20261020U)
#define TEST_LINE_MAIN_TESTED_RATIO  (33554432.0)

/** Removed double barcode decoder state: the equivalence reference. */
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
    uint32_t run_count;
    char     last_label;
    bool     b_started;
    bool     b_current_black;
    bool     b_candidate;
    bool     b_candidate_black;
    bool     b_frame;
    bool     b_armed;
    bool     b_last_decode;
} test_line_main_legacy_t;

/** Removed double barcode output, with the label as a character. */
typedef struct
{
    barcode_event_t  event;
    barcode_reason_t reason;
    barcode_action_t action;
    char             label;
    uint32_t         run_count;
} test_line_main_verdict_t;

/** Removed double line_ctrl history: the sequence reference. */
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
} test_line_main_old_t;

/** Removed double line_ctrl output. */
typedef struct
{
    double             darkness[LINE_CTRL_CHANNELS];
    double             confidence;
    double             steering;
    line_ctrl_status_t status;
    bool               b_junction_edge;
    bool               b_barcode_black;
} test_line_main_old_output_t;

/** Disagreements of one raw-pair grid with the legacy compares. */
typedef struct
{
    uint32_t signal;    /* minimum-signal decisions that differ */
    uint32_t beyond;    /* ... with N / (S0 S1) beyond 4 ulp of it */
    uint32_t steering;  /* Q7 codes off the exact half-away rounding */
} test_line_main_grid_t;

typedef struct
{
    barcode_config_t         config;
    line_ctrl_config_t       line;
    barcode_runtime_t        runtime;
    barcode_state_t          state;
    barcode_output_t         output;
    test_line_main_legacy_t  legacy;
    test_line_main_verdict_t verdict;
    double                   step_m;
    int32_t                  position;
    uint32_t                 origin;
    uint32_t                 now_ms;
    uint32_t                 events[TEST_LINE_MAIN_EVENT_COUNT];
    uint32_t                 reasons[TEST_LINE_MAIN_REASON_COUNT];
    uint32_t                 last_run_count;
    uint32_t                 last_symbol;
    barcode_action_t         last_action;
    bool                     b_jitter;
} test_line_main_fixture_t;

static uint32_t g_checks;
static uint32_t g_failures;

static void test_line_main_check (bool b_condition, const char * p_label);
static uint32_t test_line_main_random (uint32_t * p_seed);
static double test_line_main_unit (uint32_t * p_seed);
static double test_line_main_legacy_darkness (uint32_t raw,
                                              uint16_t white_adc,
                                              uint16_t black_adc);
static int64_t test_line_main_numerator (uint32_t raw, uint16_t white_adc,
                                         uint16_t black_adc);
static bool test_line_main_steering_ok (int8_t steering_q7,
                                        double steering);
static bool test_line_main_rounding_ok (int8_t steering_q7,
                                        int64_t difference, int64_t sum);
static void test_line_main_line (void);
static void test_line_main_line_faults (void);
static uint32_t test_line_main_class_errors (uint16_t white_adc,
                                             uint16_t black_adc,
                                             double black_on,
                                             double black_off);
static void test_line_main_hysteresis (void);
static void test_line_main_signal_grid (const line_ctrl_config_t * p_config,
                                        uint32_t stride,
                                        test_line_main_grid_t * p_grid);
static void test_line_main_signal (void);
static bool test_line_main_old_step (const line_ctrl_config_t * p_config,
                                     test_line_main_old_t * p_state,
                                     const uint16_t * p_raw, uint32_t now_ms,
                                     test_line_main_old_output_t * p_output);
static void test_line_main_old_events (const line_ctrl_config_t * p_config,
                                       test_line_main_old_t * p_state,
                                       uint32_t now_ms,
                                       test_line_main_old_output_t * p_output);
static uint16_t test_line_main_zone_raw (const line_ctrl_config_t * p_config,
                                         const line_ctrl_runtime_t * p_runtime,
                                         uint32_t channel, uint32_t zone,
                                         uint32_t * p_seed);
static bool test_line_main_darkness_ok (
    const line_ctrl_output_t * p_output,
    const test_line_main_old_output_t * p_old);
static uint32_t test_line_main_sequence (const line_ctrl_config_t * p_config,
                                         uint32_t seed_value);
static void test_line_main_sequences (void);
static int32_t test_line_main_side (uint32_t count, double unit,
                                    double value);
static uint32_t test_line_main_bracket (double value, double unit);
static void test_line_main_quotient (void);
static bool test_line_main_margin_ok (double value, double unit);
static double test_line_main_half_count (void);
static void test_line_main_production (void);
static void test_line_main_classify (void);
static bool test_line_main_legacy_step (const barcode_config_t * p_config,
                                        test_line_main_legacy_t * p_state,
                                        double darkness, double distance_m,
                                        uint32_t now_ms,
                                        test_line_main_verdict_t * p_output);
static void test_line_main_legacy_reject (test_line_main_legacy_t * p_state,
                                          barcode_reason_t reason,
                                          test_line_main_verdict_t * p_output);
static void test_line_main_legacy_transition (
    const barcode_config_t * p_config, test_line_main_legacy_t * p_state,
    uint32_t now_ms, test_line_main_verdict_t * p_output);
static void test_line_main_legacy_acquire (const barcode_config_t * p_config,
                                           test_line_main_legacy_t * p_state,
                                           double darkness, uint32_t now_ms,
                                           test_line_main_verdict_t * p_output);
static void test_line_main_legacy_finish (const barcode_config_t * p_config,
                                          test_line_main_legacy_t * p_state,
                                          test_line_main_verdict_t * p_output);
static bool test_line_main_legacy_matches (
    const barcode_config_t * p_config,
    const test_line_main_legacy_t * p_state,
    const barcode_symbol_t * p_symbol);
static void test_line_main_line_config (line_ctrl_config_t * p_line);
static void test_line_main_prepare (test_line_main_fixture_t * p_test);
static bool test_line_main_derive (test_line_main_fixture_t * p_test);
static double test_line_main_level (const barcode_config_t * p_config,
                                    line_ctrl_class_t darkness);
static uint32_t test_line_main_lowest (const barcode_config_t * p_config,
                                       char label);
static bool test_line_main_sample (test_line_main_fixture_t * p_test,
                                   line_ctrl_class_t darkness,
                                   int32_t position);
static void test_line_main_feed (test_line_main_fixture_t * p_test,
                                 line_ctrl_class_t darkness, uint32_t steps);
static void test_line_main_symbol (test_line_main_fixture_t * p_test,
                                   uint32_t final_steps);
static void test_line_main_gate (void);
static void test_line_main_decode (void);
static void test_line_main_no_match (void);
static void test_line_main_faults (void);
static void test_line_main_overflow (void);
static void test_line_main_duplicate (void);
static void test_line_main_limits (void);
static void test_line_main_boundaries (void);
static void test_line_main_widths (void);
static void test_line_main_wrap (void);
static void test_line_main_labels (void);
static void test_line_main_sample_prepare (test_line_main_fixture_t * p_test);
static void test_line_main_sample_frame (test_line_main_fixture_t * p_test,
                                        const barcode_symbol_t * p_symbol,
                                        double scale, bool b_reverse,
                                        bool b_noise);
static void test_line_main_scale_decode (void);
static void test_line_main_scale_unknown (void);
static void test_line_main_scale_physical (void);
static void test_line_main_scale_bounds (void);
static barcode_event_t test_line_main_frame (
    test_line_main_fixture_t * p_test, const uint32_t * p_widths,
    uint32_t count);
static void test_line_main_window_case (double width_m, double absolute_m,
                                        uint32_t low, uint32_t high);
static void test_line_main_window_neighbours (void);
static void test_line_main_window_unit (void);
static void test_line_main_reentry (void);
static void test_line_main_scale_edges (void);
static void test_line_main_timing (void);

/** @brief Run deterministic fixtures; nonzero exit reports any failure. */
int main (void)
{
    test_line_main_line();
    test_line_main_line_faults();
    test_line_main_hysteresis();
    test_line_main_signal();
    test_line_main_sequences();
    test_line_main_quotient();
    test_line_main_production();
    test_line_main_classify();
    test_line_main_gate();
    test_line_main_decode();
    test_line_main_no_match();
    test_line_main_faults();
    test_line_main_overflow();
    test_line_main_duplicate();
    test_line_main_limits();
    test_line_main_boundaries();
    test_line_main_widths();
    test_line_main_wrap();
    test_line_main_labels();
    test_line_main_scale_decode();
    test_line_main_scale_unknown();
    test_line_main_scale_physical();
    test_line_main_scale_bounds();
    test_line_main_window_neighbours();
    test_line_main_window_unit();
    test_line_main_reentry();
    test_line_main_scale_edges();
    test_line_main_timing();
    if (printf("Line/barcode: %u checks, %u failures\n",
               g_checks, g_failures) < 0)
    {
        return EXIT_FAILURE;
    }
    return (0U == g_failures) ? EXIT_SUCCESS : EXIT_FAILURE;
}

/** @brief Report a named assertion without preprocessor side effects. */
static void test_line_main_check (bool b_condition, const char * p_label)
{
    ++g_checks;
    if (!b_condition)
    {
        ++g_failures;
        if (fprintf(stderr, "FAIL: %s\n", p_label) < 0)
        {
            ++g_failures;
        }
    }
}

/** @brief Deterministic 32-bit linear congruential generator. */
static uint32_t test_line_main_random (uint32_t * p_seed)
{
    *p_seed = (*p_seed * 1664525U) + 1013904223U;
    return *p_seed;
}

/** @brief Uniform double in [0, 1) from 24 random bits. */
static double test_line_main_unit (uint32_t * p_seed)
{
    return (double) (test_line_main_random(p_seed) >> 8U) / 16777216.0;
}

/** @brief Removed line_ctrl.c darkness: clamp((raw - white) / span). */
static double test_line_main_legacy_darkness (uint32_t raw,
                                              uint16_t white_adc,
                                              uint16_t black_adc)
{
    /* ADC values are exact in double, as in the removed code. */
    double span = (double) black_adc - (double) white_adc;
    double value = ((double) raw - white_adc) / span;

    if (value < 0.0)
    {
        return 0.0;
    }
    return (value > 1.0) ? 1.0 : value;
}

/** @brief Test-side numerator clamp((raw - white) * polarity, 0, span). */
static int64_t test_line_main_numerator (uint32_t raw, uint16_t white_adc,
                                         uint16_t black_adc)
{
    int64_t span = (int64_t) black_adc - (int64_t) white_adc;
    int64_t value = (int64_t) raw - (int64_t) white_adc;

    if (span < 0)
    {
        span = -span;
        value = -value;
    }
    if (value < 0)
    {
        value = 0;
    }
    return (value > span) ? span : value;
}

/** @brief No sign inversion, zero stays zero, within half a Q7 code. */
static bool test_line_main_steering_ok (int8_t steering_q7, double steering)
{
    if (0.0 == steering)
    {
        return 0 == steering_q7;
    }
    if ((0 != steering_q7) && ((steering_q7 > 0) != (steering > 0.0)))
    {
        return false;
    }
    return fabs((double) steering_q7 - (127.0 * steering)) <= (0.5 + 1.0e-9);
}

/** @brief q7 = round(127 * difference / sum), exact ties away from zero. */
static bool test_line_main_rounding_ok (int8_t steering_q7,
                                        int64_t difference, int64_t sum)
{
    int64_t error = (254 * difference) - (2 * sum * (int64_t) steering_q7);
    int64_t size = (error < 0) ? -error : error;

    if (size != sum)
    {
        return size < sum;
    }
    return (difference > 0) ? (error < 0) : (error > 0);
}

/** @brief Check normalization, polarity, hysteresis and temporal evidence. */
static void test_line_main_line (void)
{
    line_ctrl_config_t  config;
    line_ctrl_runtime_t runtime;
    line_ctrl_state_t   state;
    line_ctrl_output_t  output;
    uint16_t raw[LINE_CTRL_CHANNELS] = {2048U, 2048U, 0U};

    line_ctrl_defaults(&config);
    line_ctrl_init(&state);
    test_line_main_check(line_ctrl_config_valid(&config), "line defaults");
    /* test_motion_main.c derives the NI-MOT-1 detection travel from it. */
    test_line_main_check(20U == config.junction_debounce_ms,
                         "20 ms production junction debounce");
    test_line_main_check(line_ctrl_derive(&config, &runtime),
                         "line defaults derive");
    test_line_main_check(line_ctrl_step(&runtime, &state, raw, 0U, &output),
                         "unconfirmed line is valid but inhibited");
    test_line_main_check(LINE_CTRL_INHIBITED == output.status, "line gate");
    config.b_calibration_confirmed = true;
    test_line_main_check(line_ctrl_derive(&config, &runtime),
                         "confirmed line derives");
    test_line_main_check(line_ctrl_step(&runtime, &state, raw, 0U, &output),
                         "balanced line sample");
    test_line_main_check((0 == output.steering_q7) &&
                         (LINE_CTRL_TRACKING == output.status),
                         "balanced edges follow");
    raw[0] = 4095U;
    raw[1] = 0U;
    test_line_main_check(line_ctrl_step(&runtime, &state, raw, 5U, &output),
                         "right line sample");
    /* The removed steering > 0.99 check is q7 >= 126 (0.99 * 127). */
    test_line_main_check(output.steering_q7 >= 126, "rightward correction");
    test_line_main_check((LINE_CTRL_STEERING_ONE == output.steering_q7) &&
                         (0U == output.darkness_q12[0]) &&
                         (LINE_CTRL_DARKNESS_ONE == output.darkness_q12[1]),
                         "full contrast is Q7 127 and Q12 0..4095");
    raw[0] = 0U;
    test_line_main_check(line_ctrl_step(&runtime, &state, raw, 10U, &output),
                         "junction starts");
    test_line_main_check(LINE_CTRL_SEARCHING == output.status,
                         "both black initially ambiguous");
    test_line_main_check(line_ctrl_step(&runtime, &state, raw, 29U, &output),
                         "junction almost debounced");
    test_line_main_check(!output.b_junction_edge, "no early junction");
    test_line_main_check(line_ctrl_step(&runtime, &state, raw, 30U, &output),
                         "junction debounce boundary");
    test_line_main_check(output.b_junction_edge &&
                         (LINE_CTRL_JUNCTION == output.status),
                         "single junction event");
    test_line_main_check(line_ctrl_step(&runtime, &state, raw, 40U, &output),
                         "junction remains");
    test_line_main_check(!output.b_junction_edge, "junction not repeated");
    raw[0] = 4095U;
    raw[1] = 4095U;
    test_line_main_check(line_ctrl_step(&runtime, &state, raw, 50U, &output),
                         "first all white");
    test_line_main_check(LINE_CTRL_SEARCHING == output.status,
                         "line loss starts uncertain");
    test_line_main_check(line_ctrl_step(&runtime, &state, raw, 90U, &output),
                         "line loss maintained");
    test_line_main_check(line_ctrl_step(&runtime, &state, raw, 130U,
                                        &output),
                         "line loss still before timeout");
    test_line_main_check(LINE_CTRL_SEARCHING == output.status,
                         "no premature line loss");
    test_line_main_check(line_ctrl_step(&runtime, &state, raw, 150U,
                                        &output),
                         "line loss timeout sample");
    test_line_main_check((LINE_CTRL_LOST == output.status) &&
                         (0 == output.steering_q7),
                         "lost has zero demand");
}

/** @brief Check polarity inversion, ADC limits, wrap and stale rejection. */
static void test_line_main_line_faults (void)
{
    line_ctrl_config_t     config;
    line_ctrl_runtime_t    runtime;
    line_ctrl_state_t      state;
    line_ctrl_output_t     output;
    line_ctrl_hysteresis_t hysteresis;
    uint16_t raw[LINE_CTRL_CHANNELS] = {4095U, 4095U, 4095U};

    line_ctrl_defaults(&config);
    line_ctrl_init(&state);
    config.b_calibration_confirmed = true;
    config.white_adc[2] = 0U;
    config.black_adc[2] = 4095U;
    test_line_main_check(line_ctrl_derive(&config, &runtime),
                         "inverted sidebarcode channel derives");
    test_line_main_check(line_ctrl_step(&runtime, &state, raw,
                         UINT32_MAX - 5U, &output), "line near timer wrap");
    test_line_main_check(output.b_barcode_black, "channel polarity varies");
    raw[2] = 2600U;
    test_line_main_check(line_ctrl_step(&runtime, &state, raw, 10U,
                                        &output), "timer wrap accepted");
    test_line_main_check(output.b_barcode_black, "hysteresis stays black");
    raw[2] = 2000U;
    test_line_main_check(line_ctrl_step(&runtime, &state, raw, 12U,
                                        &output), "hysteresis low sample");
    test_line_main_check(!output.b_barcode_black, "hysteresis turns white");
    test_line_main_check(!line_ctrl_step(&runtime, &state, raw, 100U,
                                         &output), "stale sample rejected");
    test_line_main_check(LINE_CTRL_FAULT == output.status, "stale fault");
    raw[0] = UINT16_MAX;
    test_line_main_check(!line_ctrl_step(&runtime, &state, raw, 101U,
                                         &output),
                         "ADC out of configured range");
    config.black_on = NAN;
    test_line_main_check(!line_ctrl_config_valid(&config), "NaN threshold");
    test_line_main_check((!line_ctrl_derive(&config, &runtime)) &&
                         (!runtime.b_valid),
                         "NaN threshold cannot derive");
    test_line_main_check(!line_ctrl_step(&runtime, &state, raw, 0U,
                                         &output),
                         "underived runtime rejected");
    line_ctrl_defaults(&config);
    config.maximum_adc = 4096U;
    test_line_main_check(line_ctrl_config_valid(&config) &&
                         (!line_ctrl_derive(&config, &runtime)),
                         "derive alone caps maximum_adc at 12 bits");
    line_ctrl_defaults(&config);
    config.black_adc[1] = config.white_adc[1];
    test_line_main_check(!line_ctrl_config_valid(&config), "zero contrast");
    test_line_main_check(!line_ctrl_derive(&config, &runtime),
                         "zero contrast cannot derive");
    test_line_main_check(!line_ctrl_step(NULL, &state, raw, 0U, &output),
                         "NULL line runtime");
    line_ctrl_defaults(&config);
    test_line_main_check(line_ctrl_derive(&config, &runtime) &&
                         (!line_ctrl_step(&runtime, NULL, raw, 0U, &output)),
                         "NULL line state");
    test_line_main_check((!line_ctrl_derive(NULL, &runtime)) &&
                         (!line_ctrl_derive(&config, NULL)),
                         "NULL derive arguments");
    test_line_main_check((!line_ctrl_hysteresis(100U, 100U, 0.75, 0.55,
                                                &hysteresis)) &&
                         (!line_ctrl_hysteresis(4096U, 0U, 0.75, 0.55,
                                                &hysteresis)) &&
                         (!line_ctrl_hysteresis(0U, 4095U, 0.55, 0.75,
                                                &hysteresis)) &&
                         (!line_ctrl_hysteresis(0U, 4095U, 1.5, 0.55,
                                                &hysteresis)) &&
                         (!line_ctrl_hysteresis(0U, 4095U, 0.75, -0.5,
                                                &hysteresis)) &&
                         (!line_ctrl_hysteresis(0U, 4095U, 0.75, 0.55,
                                                NULL)),
                         "invalid hysteresis inputs rejected");
    test_line_main_check((LINE_CTRL_INVALID ==
                          line_ctrl_classify(NULL, 0U)) &&
                         (LINE_CTRL_INVALID ==
                          line_ctrl_classify(&runtime.channel[0].hysteresis,
                                             4096U)),
                         "classes outside the 12-bit ADC are invalid");
}

/** @brief Codes 0..4095 whose class differs from the removed compares;
 * UINT32_MAX when the pair cannot be derived.
 */
static uint32_t test_line_main_class_errors (uint16_t white_adc,
                                             uint16_t black_adc,
                                             double black_on,
                                             double black_off)
{
    line_ctrl_hysteresis_t hysteresis;
    line_ctrl_class_t      expected;
    uint32_t raw;
    uint32_t errors = 0U;
    double   darkness;

    if (!line_ctrl_hysteresis(white_adc, black_adc, black_on, black_off,
                              &hysteresis))
    {
        return UINT32_MAX;
    }
    for (raw = 0U; raw < TEST_LINE_MAIN_ADC_COUNT; ++raw)
    {
        darkness = test_line_main_legacy_darkness(raw, white_adc, black_adc);
        expected = LINE_CTRL_BAND;
        if (darkness >= black_on)
        {
            expected = LINE_CTRL_DARK;
        }
        else if (darkness <= black_off)
        {
            expected = LINE_CTRL_LIGHT;
        }
        else
        {
            /* Inside the band the removed code kept its classification. */
        }
        /* raw < 4096 fits uint16_t. */
        if (expected != line_ctrl_classify(&hysteresis, (uint16_t) raw))
        {
            ++errors;
        }
    }
    return errors;
}

/** @brief Exhaustive hysteresis equivalence for fixed and random pairs. */
static void test_line_main_hysteresis (void)
{
    const uint16_t levels[][2] =
    {
        {3500U, 500U}, {4095U, 0U}, {0U, 4095U}, {3000U, 800U},
        {200U, 3900U}, {1U, 0U}, {0U, 1U}, {4095U, 4094U}, {2048U, 2047U}
    };
    const double pairs[][2] =
    {
        {0.75, 0.55}, {nextafter(0.75, 2.0), 0.55},
        {nextafter(0.75, 0.0), 0.55}, {0.75, nextafter(0.55, 1.0)},
        {0.75, nextafter(0.55, 0.0)}, {1.0, 0.0},
        {nextafter(1.0, 0.0), nextafter(0.0, 1.0)}, {0.8, 0.4}, {0.5, 0.25}
    };
    line_ctrl_hysteresis_t hysteresis;
    size_t   level;
    size_t   pair;
    uint32_t trial;
    uint32_t errors = 0U;
    uint32_t seed = TEST_LINE_MAIN_SEED;
    uint16_t white;
    uint16_t black;
    double   first;
    double   second;

    for (level = 0U; level < (sizeof(levels) / sizeof(levels[0])); ++level)
    {
        for (pair = 0U; pair < (sizeof(pairs) / sizeof(pairs[0])); ++pair)
        {
            test_line_main_check(0U == test_line_main_class_errors(
                levels[level][0], levels[level][1], pairs[pair][0],
                pairs[pair][1]),
                "hysteresis equals the removed compares on 4096 codes");
        }
    }
    for (trial = 0U; trial < TEST_LINE_MAIN_CALIBRATIONS; ++trial)
    {
        /* Masked to 12 bits, so both fit uint16_t. */
        white = (uint16_t) (test_line_main_random(&seed) & 0xFFFU);
        black = (uint16_t) (test_line_main_random(&seed) & 0xFFFU);
        black = (white == black) ? (uint16_t) (black ^ 1U) : black;
        first = test_line_main_unit(&seed);
        second = test_line_main_unit(&seed);
        if (first != second)
        {
            errors += test_line_main_class_errors(white, black,
                fmax(first, second), fmin(first, second));
        }
    }
    test_line_main_check(0U == errors,
                         "random calibrations equal the removed compares");
    /* Raw 1250 is darkness 0.75 exactly; a Q12 compare would miss it. */
    test_line_main_check(line_ctrl_hysteresis(3500U, 500U, 0.75, 0.55,
                                              &hysteresis) &&
                         (1250U == hysteresis.on_adc) &&
                         (1850U == hysteresis.off_adc) &&
                         (!hysteresis.b_dark_high),
                         "production thresholds 1250 dark, 1850 light");
}

/** @brief Minimum signal and Q7 steering for raw pairs on a grid. */
static void test_line_main_signal_grid (const line_ctrl_config_t * p_config,
                                        uint32_t stride,
                                        test_line_main_grid_t * p_grid)
{
    double   left_darkness[TEST_LINE_MAIN_ADC_COUNT];
    double   right_darkness[TEST_LINE_MAIN_ADC_COUNT];
    int64_t  left_count[TEST_LINE_MAIN_ADC_COUNT];
    int64_t  right_count[TEST_LINE_MAIN_ADC_COUNT];
    /* The numerator at the black level is the span |black - white|. */
    int64_t  left_span = test_line_main_numerator(p_config->black_adc[0],
        p_config->white_adc[0], p_config->black_adc[0]);
    int64_t  right_span = test_line_main_numerator(p_config->black_adc[1],
        p_config->white_adc[1], p_config->black_adc[1]);
    double   bound = 4.0 * (nextafter(p_config->minimum_signal, 1.0) -
                            p_config->minimum_signal);
    line_ctrl_runtime_t runtime;
    uint16_t raw[LINE_CTRL_CHANNELS] = {0U, 0U, 0U};
    uint32_t left;
    uint32_t right;
    int64_t  sum;
    double   total;
    int8_t   steering_q7 = 0;
    bool     b_signal;

    memset(p_grid, 0, sizeof(*p_grid));
    if (!line_ctrl_derive(p_config, &runtime))
    {
        p_grid->signal = UINT32_MAX;
        return;
    }
    for (left = 0U; left < TEST_LINE_MAIN_ADC_COUNT; ++left)
    {
        left_darkness[left] = test_line_main_legacy_darkness(left,
            p_config->white_adc[0], p_config->black_adc[0]);
        right_darkness[left] = test_line_main_legacy_darkness(left,
            p_config->white_adc[1], p_config->black_adc[1]);
        left_count[left] = test_line_main_numerator(left,
            p_config->white_adc[0], p_config->black_adc[0]);
        right_count[left] = test_line_main_numerator(left,
            p_config->white_adc[1], p_config->black_adc[1]);
    }
    for (left = 0U; left < TEST_LINE_MAIN_ADC_COUNT; left += stride)
    {
        raw[0] = (uint16_t) left; /* below 4096 */
        for (right = 0U; right < TEST_LINE_MAIN_ADC_COUNT; right += stride)
        {
            raw[1] = (uint16_t) right; /* below 4096 */
            b_signal = line_ctrl_steer(&runtime, raw, &steering_q7);
            total = left_darkness[left] + right_darkness[right];
            /* N / (S0 S1) is the exact value of the removed d0 + d1. */
            sum = (left_count[left] * right_span) +
                  (right_count[right] * left_span);
            if (b_signal != (total >= p_config->minimum_signal))
            {
                ++p_grid->signal;
                if (fabs(((double) sum / (double) (left_span * right_span)) -
                         p_config->minimum_signal) > bound)
                {
                    ++p_grid->beyond;
                }
            }
            if ((!test_line_main_steering_ok(steering_q7, b_signal ?
                    ((right_darkness[right] - left_darkness[left]) / total)
                    : 0.0)) ||
                (b_signal && (!test_line_main_rounding_ok(steering_q7,
                    (right_count[right] * left_span) -
                    (left_count[left] * right_span), sum))))
            {
                ++p_grid->steering;
            }
        }
    }
}

/** @brief Minimum signal and steering equal the removed compares. */
static void test_line_main_signal (void)
{
    line_ctrl_config_t    config;
    line_ctrl_runtime_t   runtime;
    test_line_main_grid_t grid;
    double                threshold;

    test_line_main_line_config(&config);
    test_line_main_check(line_ctrl_derive(&config, &runtime) &&
                         (1350000U == runtime.signal_threshold),
                         "production T = ceil(0.15 * 3000^2) = 1,350,000");
    test_line_main_signal_grid(&config, 1U, &grid);
    test_line_main_check(0U == grid.signal,
                         "production minimum signal equals the removed "
                         "compare on all 4096^2 raw pairs");
    test_line_main_check(0U == grid.steering,
                         "production Q7 steering: exact half-away rounding");
    line_ctrl_defaults(&config);
    test_line_main_check(line_ctrl_derive(&config, &runtime) &&
                         (2515354U == runtime.signal_threshold),
                         "default T = ceil(0.15 * 4095^2) = 2,515,354");
    test_line_main_signal_grid(&config, 1U, &grid);
    test_line_main_check((0U == grid.signal) && (0U == grid.steering),
                         "default calibration equals the removed compares");
    config.white_adc[0] = 3900U;
    config.black_adc[0] = 100U;
    config.white_adc[1] = 2500U;
    config.black_adc[1] = 1200U;
    test_line_main_signal_grid(&config, 3U, &grid);
    test_line_main_check((0U == grid.signal) && (0U == grid.steering),
                         "asymmetric spans equal the removed compares");
    config.white_adc[1] = 600U;
    config.black_adc[1] = 3800U;
    config.minimum_signal = 0.5;
    test_line_main_signal_grid(&config, 3U, &grid);
    test_line_main_check((0U == grid.signal) && (0U == grid.steering),
                         "mixed polarity equals the removed compares");
    test_line_main_line_config(&config);
    for (threshold = 0.0; threshold < 2.0; threshold += 1.0)
    {
        /* nextafter neighbours of the production threshold 0.15. */
        config.minimum_signal = nextafter(0.15, threshold);
        test_line_main_signal_grid(&config, 1U, &grid);
        test_line_main_check((0U == grid.beyond) && (0U == grid.steering),
                             "neighbour thresholds differ only within 4 ulp");
    }
    /* fl(0.1 * 1000^2) is exactly 100000, but the double 0.1 is above one
     * tenth: the exact ceiling is 100001.
     */
    config.white_adc[0] = 3500U;
    config.black_adc[0] = 2500U;
    config.white_adc[1] = 3500U;
    config.black_adc[1] = 2500U;
    config.minimum_signal = 0.1;
    test_line_main_check(line_ctrl_derive(&config, &runtime) &&
                         (100001U == runtime.signal_threshold) &&
                         (fma(0.1, 1.0e6, -100001.0) < 0.0) &&
                         (fma(0.1, 1.0e6, -100000.0) > 0.0),
                         "exact T when the rounded product is an integer");
    test_line_main_signal_grid(&config, 1U, &grid);
    test_line_main_check((0U == grid.beyond) && (0U == grid.steering),
                         "integer product differs only within 4 ulp");
}

/** @brief Removed line_ctrl_step() on doubles: the sequence reference. */
static bool test_line_main_old_step (const line_ctrl_config_t * p_config,
                                     test_line_main_old_t * p_state,
                                     const uint16_t * p_raw, uint32_t now_ms,
                                     test_line_main_old_output_t * p_output)
{
    uint32_t channel;
    double   sum;

    memset(p_output, 0, sizeof(*p_output));
    p_output->status = LINE_CTRL_FAULT;
    if (!line_ctrl_config_valid(p_config))
    {
        memset(p_state, 0, sizeof(*p_state));
        return false;
    }
    if (!p_config->b_calibration_confirmed)
    {
        memset(p_state, 0, sizeof(*p_state));
        p_output->status = LINE_CTRL_INHIBITED;
        return true;
    }
    if ((p_state->b_started) &&
        ((now_ms - p_state->previous_ms) > p_config->maximum_gap_ms))
    {
        memset(p_state, 0, sizeof(*p_state));
        return false;
    }
    for (channel = 0U; channel < LINE_CTRL_CHANNELS; ++channel)
    {
        if (p_raw[channel] > p_config->maximum_adc)
        {
            memset(p_state, 0, sizeof(*p_state));
            return false;
        }
        p_output->darkness[channel] = test_line_main_legacy_darkness(
            p_raw[channel], p_config->white_adc[channel],
            p_config->black_adc[channel]);
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
    sum = p_output->darkness[0] + p_output->darkness[1];
    p_output->confidence = (sum > 1.0) ? 1.0 : sum;
    if (sum >= p_config->minimum_signal)
    {
        p_output->steering = (p_output->darkness[1] -
                              p_output->darkness[0]) / sum;
    }
    p_output->b_barcode_black = p_state->b_black[2];
    test_line_main_old_events(p_config, p_state, now_ms, p_output);
    return true;
}

/** @brief Removed junction debounce and line-loss timing. */
static void test_line_main_old_events (const line_ctrl_config_t * p_config,
                                       test_line_main_old_t * p_state,
                                       uint32_t now_ms,
                                       test_line_main_old_output_t * p_output)
{
    if ((p_state->b_black[0]) && (p_state->b_black[1]))
    {
        p_state->b_losing = false;
        if (!p_state->b_junction_pending)
        {
            p_state->b_junction_pending = true;
            p_state->junction_since_ms  = now_ms;
        }
        p_output->status = LINE_CTRL_SEARCHING;
        if ((now_ms - p_state->junction_since_ms) >=
            p_config->junction_debounce_ms)
        {
            p_output->status = LINE_CTRL_JUNCTION;
            p_output->b_junction_edge = !p_state->b_junction_latched;
            p_state->b_junction_latched = true;
        }
        return;
    }
    p_state->b_junction_pending = false;
    p_state->b_junction_latched = false;
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

/** @brief Raw code in zone 0 black, 1 white, 2 near on, 3 near off,
 * 4 uniform or 5 above the ADC range.
 */
static uint16_t test_line_main_zone_raw (const line_ctrl_config_t * p_config,
                                         const line_ctrl_runtime_t * p_runtime,
                                         uint32_t channel, uint32_t zone,
                                         uint32_t * p_seed)
{
    uint32_t noise = test_line_main_random(p_seed) >> 16U;
    int32_t  value;

    if (4U == zone)
    {
        return (uint16_t) (noise & 0xFFFU); /* 12 bits */
    }
    if (5U == zone)
    {
        return (uint16_t) (4096U + (noise & 3U)); /* below 4100 */
    }
    value = (int32_t) p_config->black_adc[channel];
    if (1U == zone)
    {
        value = (int32_t) p_config->white_adc[channel];
    }
    else if (2U == zone)
    {
        value = (int32_t) p_runtime->channel[channel].hysteresis.on_adc;
    }
    else if (3U == zone)
    {
        value = (int32_t) p_runtime->channel[channel].hysteresis.off_adc;
    }
    else
    {
        /* Zone 0 is centred on the black level. */
    }
    /* noise % 7 is at most 6, so the sum stays within int32_t. */
    value += (int32_t) (noise % 7U) - 3;
    value = (value < 0) ? 0 : ((value > 4095) ? 4095 : value);
    return (uint16_t) value; /* clamped to 0..4095 */
}

/** @brief Q12 darkness is the nearest code of the removed darkness. */
static bool test_line_main_darkness_ok (
    const line_ctrl_output_t * p_output,
    const test_line_main_old_output_t * p_old)
{
    uint32_t channel;

    for (channel = 0U; channel < LINE_CTRL_CHANNELS; ++channel)
    {
        if (fabs((double) p_output->darkness_q12[channel] -
                 (4095.0 * p_old->darkness[channel])) > (0.5 + 1.0e-9))
        {
            return false;
        }
    }
    return true;
}

/** @brief Random line sequence in lockstep with the removed double code;
 * returns the samples that differ (UINT32_MAX if it cannot derive).
 */
static uint32_t test_line_main_sequence (const line_ctrl_config_t * p_config,
                                         uint32_t seed_value)
{
    line_ctrl_runtime_t         runtime;
    line_ctrl_state_t           state;
    line_ctrl_output_t          output;
    test_line_main_old_t        old_state;
    test_line_main_old_output_t old_output;
    uint16_t raw[LINE_CTRL_CHANNELS] = {0U, 0U, 0U};
    uint32_t zones[LINE_CTRL_CHANNELS] = {0U, 0U, 0U};
    uint32_t seed = seed_value;
    uint32_t now_ms = UINT32_MAX - 2000U;
    uint32_t hold = 0U;
    uint32_t sample;
    uint32_t channel;
    uint32_t errors = 0U;
    bool     b_new;
    bool     b_old;

    if (!line_ctrl_derive(p_config, &runtime))
    {
        return UINT32_MAX;
    }
    line_ctrl_init(&state);
    memset(&old_state, 0, sizeof(old_state));
    for (sample = 0U; sample < TEST_LINE_MAIN_SEQUENCE; ++sample)
    {
        if (0U == hold)
        {
            hold = 1U + (test_line_main_random(&seed) % 80U);
            for (channel = 0U; channel < LINE_CTRL_CHANNELS; ++channel)
            {
                zones[channel] = test_line_main_random(&seed) % 64U;
                zones[channel] = (0U == zones[channel]) ? 5U :
                    ((zones[channel] < 10U) ? 4U : (zones[channel] % 4U));
            }
        }
        --hold;
        /* Mostly 1-3 ms apart; a rare 60 ms gap is a stale sample. */
        now_ms += (0U == (test_line_main_random(&seed) % 500U)) ? 60U :
                  (1U + (test_line_main_random(&seed) % 3U));
        for (channel = 0U; channel < LINE_CTRL_CHANNELS; ++channel)
        {
            raw[channel] = test_line_main_zone_raw(p_config, &runtime,
                                                   channel, zones[channel],
                                                   &seed);
        }
        b_new = line_ctrl_step(&runtime, &state, raw, now_ms, &output);
        b_old = test_line_main_old_step(p_config, &old_state, raw, now_ms,
                                        &old_output);
        if ((b_new != b_old) || (output.status != old_output.status) ||
            (output.b_junction_edge != old_output.b_junction_edge) ||
            (output.b_barcode_black != old_output.b_barcode_black) ||
            (!test_line_main_steering_ok(output.steering_q7,
                                         old_output.steering)) ||
            (b_new && (!test_line_main_darkness_ok(&output, &old_output))))
        {
            ++errors;
        }
    }
    return errors;
}

/** @brief Line state machine in lockstep for several calibrations. */
static void test_line_main_sequences (void)
{
    line_ctrl_config_t config;

    test_line_main_line_config(&config);
    test_line_main_check(0U == test_line_main_sequence(&config, 1U),
                         "production line sequence equals the removed code");
    line_ctrl_defaults(&config);
    config.b_calibration_confirmed = true;
    config.white_adc[1] = 600U;
    config.black_adc[1] = 3800U;
    config.white_adc[2] = 0U;
    config.black_adc[2] = 4095U;
    test_line_main_check(0U == test_line_main_sequence(&config, 2U),
                         "mixed polarity sequence equals the removed code");
    config.minimum_signal = 0.6;
    config.junction_debounce_ms = 7U;
    config.lost_timeout_ms = 33U;
    config.maximum_adc = 4000U;
    config.white_adc[0] = 3900U;
    config.black_adc[2] = 3950U;
    test_line_main_check(0U == test_line_main_sequence(&config, 3U),
                         "other limits sequence equals the removed code");
    config.b_calibration_confirmed = false;
    test_line_main_check(0U == test_line_main_sequence(&config, 4U),
                         "inhibited sequence equals the removed code");
}

/** @brief Host oracle: sign of the exact count * unit - value (fma). */
static int32_t test_line_main_side (uint32_t count, double unit,
                                    double value)
{
    double residual = fma((double) count, unit, -value);

    if (residual > 0.0)
    {
        return 1;
    }
    return (residual < 0.0) ? -1 : 0;
}

/** @brief 0 when floor/ceil of value / unit bracket it exactly, else 1;
 * quotients of 2^25 or more are outside this check.
 */
static uint32_t test_line_main_bracket (double value, double unit)
{
    uint32_t lower = 0U;
    uint32_t upper = 0U;
    bool     b_ok;

    if (!((value / unit) < TEST_LINE_MAIN_TESTED_RATIO))
    {
        return 0U;
    }
    b_ok = line_ctrl_quotient(value, unit, false, &lower) &&
           line_ctrl_quotient(value, unit, true, &upper) &&
           (test_line_main_side(lower, unit, value) <= 0) &&
           (test_line_main_side(lower + 1U, unit, value) > 0) &&
           (test_line_main_side(upper, unit, value) >= 0) &&
           ((0U == upper) ||
            (test_line_main_side(upper - 1U, unit, value) < 0));
    return b_ok ? 0U : 1U;
}

/** @brief Exact rational floor/ceiling against a correctly rounded fma. */
static void test_line_main_quotient (void)
{
    const double units[] =
    {
        TEST_LINE_MAIN_STEP_M, 0.0001, 0.0002836, 1.0e-6, 0.37, 3.0e-4,
        TEST_LINE_MAIN_SAMPLE_STEP, 0.000283616
    };
    const uint32_t counts[] =
    {
        1U, 2U, 3U, 4U, 10U, 53U, 211U, 255U, 256U, 706U, 881U, 4096U,
        65535U, 1000000U, 33554431U
    };
    const double values[] =
    {
        0.0, 0.001, 0.015, 0.060, 0.250, 0.0002, 0.0005, 0.0003, 0.200,
        0.0099, 0.0141, 0.012, 0.00025, 0.0025, 100.0, 5.0e-324
    };
    size_t   unit;
    size_t   index;
    uint32_t trial;
    uint32_t errors = 0U;
    uint32_t result = 0U;
    uint32_t seed = TEST_LINE_MAIN_SEED;
    double   step;
    double   exact;

    for (unit = 0U; unit < (sizeof(units) / sizeof(units[0])); ++unit)
    {
        for (index = 0U; index < (sizeof(counts) / sizeof(counts[0]));
             ++index)
        {
            exact = (double) counts[index] * units[unit];
            errors += test_line_main_bracket(exact, units[unit]) +
                      test_line_main_bracket(nextafter(exact, 0.0),
                                             units[unit]) +
                      test_line_main_bracket(nextafter(exact, 1.0e9),
                                             units[unit]);
        }
        for (index = 0U; index < (sizeof(values) / sizeof(values[0]));
             ++index)
        {
            errors += test_line_main_bracket(values[index], units[unit]);
        }
    }
    for (trial = 0U; trial < TEST_LINE_MAIN_QUOTIENTS; ++trial)
    {
        step = 1.0e-6 + (test_line_main_unit(&seed) * 1.0e-3);
        exact = (double) (test_line_main_random(&seed) % 1048576U) * step;
        errors += test_line_main_bracket(exact, step) +
                  test_line_main_bracket(nextafter(exact, 0.0), step) +
                  test_line_main_bracket(nextafter(exact, 1.0e9), step);
    }
    test_line_main_check(0U == errors,
                         "quotient floor/ceiling bracket the exact value");
    test_line_main_check((!line_ctrl_quotient(NAN, 1.0, false, &result)) &&
                         (!line_ctrl_quotient(-1.0, 1.0, false, &result)) &&
                         (!line_ctrl_quotient(INFINITY, 1.0, true, &result)) &&
                         (!line_ctrl_quotient(1.0, 0.0, true, &result)) &&
                         (!line_ctrl_quotient(1.0, -1.0, true, &result)) &&
                         (!line_ctrl_quotient(1.0, NAN, true, &result)) &&
                         (!line_ctrl_quotient(1.0, INFINITY, true, &result)) &&
                         (!line_ctrl_quotient(1.0, 1.0, true, NULL)),
                         "invalid quotient arguments rejected");
    test_line_main_check(line_ctrl_quotient(67108863.0, 1.0, false,
                                            &result) &&
                         (67108863U == result) &&
                         (!line_ctrl_quotient(67108863.5, 1.0, true,
                                              &result)) &&
                         (!line_ctrl_quotient(67108864.0, 1.0, false,
                                              &result)),
                         "quotients stop below 2^26");
}

/** @brief value / unit lies at least 1e-6 from every integer. */
static bool test_line_main_margin_ok (double value, double unit)
{
    double ratio = value / unit;

    return fabs(ratio - floor(ratio + 0.5)) >= TEST_LINE_MAIN_MARGIN_HC;
}

/** @brief robot_app.c metres per half-count: pi * (R + R) / (2 * counts). */
static double test_line_main_half_count (void)
{
    return (TEST_LINE_MAIN_PI *
            (TEST_LINE_MAIN_RADIUS_M + TEST_LINE_MAIN_RADIUS_M)) /
           (2.0 * TEST_LINE_MAIN_COUNTS);
}

/** @brief Production half-count thresholds and their integer margins. */
static void test_line_main_production (void)
{
    barcode_config_t   config;
    line_ctrl_config_t line;
    barcode_runtime_t  runtime;
    const double unit_m = test_line_main_half_count();
    const double width_m = 0.012;
    const double tolerance_m = 0.0003 + (0.15 * width_m);
    uint32_t lower = 0U;
    uint32_t upper = 0U;

    barcode_defaults(&config);
    test_line_main_line_config(&line);
    test_line_main_check(barcode_derive(&config, unit_m, &line, &runtime),
                         "production defaults derive");
    test_line_main_check((0U == runtime.maximum_step_hc) &&
                         (1U == runtime.reverse_hc) &&
                         (4U == runtime.minimum_run_hc) &&
                         (211U == runtime.maximum_run_hc) &&
                         (1U == runtime.persistence_hc),
                         "production runs 0/1/4/211/1 half-counts");
    test_line_main_check((53U == runtime.quiet_zone_hc) &&
                         (881U == runtime.maximum_frame_hc) &&
                         (706U == runtime.duplicate_hc),
                         "production quiet/frame/duplicate 53/881/706");
    test_line_main_check(
        test_line_main_margin_ok(config.maximum_step_m, unit_m) &&
        test_line_main_margin_ok(config.reverse_tolerance_m, unit_m) &&
        test_line_main_margin_ok(config.minimum_run_m, unit_m) &&
        test_line_main_margin_ok(config.maximum_run_m, unit_m) &&
        test_line_main_margin_ok(0.25 * config.minimum_run_m, unit_m) &&
        test_line_main_margin_ok(config.quiet_zone_m, unit_m) &&
        test_line_main_margin_ok(config.maximum_frame_m, unit_m) &&
        test_line_main_margin_ok(config.duplicate_distance_m, unit_m),
        "production limits lie >= 1e-6 half-count from integers");
    test_line_main_check((1250U == runtime.hysteresis.on_adc) &&
                         (1850U == runtime.hysteresis.off_adc) &&
                         (!runtime.hysteresis.b_dark_high),
                         "production sidebarcode thresholds 1250/1850");
    /* apphost_test_main.c course_cfg fixture: 12 mm bar, 10 mm runs. */
    config.minimum_run_m = 0.010;
    config.maximum_step_m = 0.002;
    test_line_main_check(barcode_derive(&config, unit_m, &line, &runtime) &&
                         (36U == runtime.minimum_run_hc) &&
                         (7U == runtime.maximum_step_hc) &&
                         (9U == runtime.persistence_hc),
                         "apphost fixture runs 36/7/9 half-counts");
    test_line_main_check(line_ctrl_quotient(width_m - tolerance_m, unit_m,
                                            true, &lower) &&
                         line_ctrl_quotient(width_m + tolerance_m, unit_m,
                                            false, &upper) &&
                         (35U == lower) && (49U == upper),
                         "12 mm fixture window [35, 49] half-counts");
    test_line_main_check(
        test_line_main_margin_ok(config.minimum_run_m, unit_m) &&
        test_line_main_margin_ok(config.maximum_step_m, unit_m) &&
        test_line_main_margin_ok(0.25 * config.minimum_run_m, unit_m) &&
        test_line_main_margin_ok(width_m - tolerance_m, unit_m) &&
        test_line_main_margin_ok(width_m + tolerance_m, unit_m),
        "fixture limits lie >= 1e-6 half-count from integers");
}

/** @brief Sidebarcode classes: own thresholds, gates and invalid input. */
static void test_line_main_classify (void)
{
    barcode_config_t   config;
    line_ctrl_config_t line;
    barcode_runtime_t  runtime;
    line_ctrl_class_t  expected;
    const double unit_m = test_line_main_half_count();
    uint32_t raw;
    uint32_t errors = 0U;
    double   darkness;

    barcode_defaults(&config);
    config.black_on = 0.8;
    config.black_off = 0.4;
    test_line_main_line_config(&line);
    line.white_adc[2] = 300U;
    line.black_adc[2] = 3700U;
    test_line_main_check(barcode_derive(&config, unit_m, &line, &runtime),
                         "barcode hysteresis derives from channel 2");
    for (raw = 0U; raw < TEST_LINE_MAIN_ADC_COUNT; ++raw)
    {
        darkness = test_line_main_legacy_darkness(raw, 300U, 3700U);
        expected = (darkness >= 0.8) ? LINE_CTRL_DARK :
                   ((darkness <= 0.4) ? LINE_CTRL_LIGHT : LINE_CTRL_BAND);
        /* raw < 4096 fits uint16_t. */
        if (expected != barcode_classify(&runtime, (uint16_t) raw))
        {
            ++errors;
        }
    }
    test_line_main_check(0U == errors,
                         "sidebarcode class equals the removed barcode "
                         "compares on 4096 codes");
    test_line_main_check(LINE_CTRL_INVALID ==
                         barcode_classify(&runtime, 4096U),
                         "codes above 4095 are invalid");
    line.maximum_adc = 3800U;
    test_line_main_check(barcode_derive(&config, unit_m, &line, &runtime) &&
                         (LINE_CTRL_DARK ==
                          barcode_classify(&runtime, 3800U)) &&
                         (LINE_CTRL_INVALID ==
                          barcode_classify(&runtime, 3801U)),
                         "configured ADC maximum bounds the class");
    line.b_calibration_confirmed = false;
    test_line_main_check(barcode_derive(&config, unit_m, &line, &runtime) &&
                         (LINE_CTRL_LIGHT ==
                          barcode_classify(&runtime, 3700U)),
                         "inhibited line reads light (darkness 0)");
    test_line_main_check(LINE_CTRL_INVALID == barcode_classify(NULL, 0U),
                         "NULL runtime class is invalid");
    memset(&runtime, 0, sizeof(runtime));
    test_line_main_check(LINE_CTRL_INVALID ==
                         barcode_classify(&runtime, 0U),
                         "underived runtime class is invalid");
    test_line_main_line_config(&line);
    test_line_main_check(
        (!barcode_derive(&config, 0.0, &line, &runtime)) &&
        (!barcode_derive(&config, -unit_m, &line, &runtime)) &&
        (!barcode_derive(&config, NAN, &line, &runtime)) &&
        (!barcode_derive(&config, INFINITY, &line, &runtime)),
        "unusable half-count unit rejected");
    test_line_main_check((!barcode_derive(NULL, unit_m, &line, &runtime)) &&
                         (!barcode_derive(&config, unit_m, NULL, &runtime)) &&
                         (!barcode_derive(&config, unit_m, &line, NULL)),
                         "NULL derive arguments rejected");
    line.maximum_adc = 4096U;
    test_line_main_check(!barcode_derive(&config, unit_m, &line, &runtime),
                         "13-bit ADC maximum rejected");
    test_line_main_line_config(&line);
    line.black_on = NAN;
    test_line_main_check(!barcode_derive(&config, unit_m, &line, &runtime),
                         "invalid line calibration rejected");
    test_line_main_line_config(&line);
    config.black_on = 0.3;
    test_line_main_check(!barcode_derive(&config, unit_m, &line, &runtime),
                         "barcode thresholds are validated");
}

/** @brief Removed barcode_step() on doubles, kept as the reference. */
static bool test_line_main_legacy_step (const barcode_config_t * p_config,
                                        test_line_main_legacy_t * p_state,
                                        double darkness, double distance_m,
                                        uint32_t now_ms,
                                        test_line_main_verdict_t * p_output)
{
    double advance;

    memset(p_output, 0, sizeof(*p_output));
    if ((!barcode_config_valid(p_config)) || (!isfinite(darkness)) ||
        (!isfinite(distance_m)) || (darkness < 0.0) || (darkness > 1.0))
    {
        memset(p_state, 0, sizeof(*p_state));
        p_output->event  = BARCODE_EVENT_REJECTED;
        p_output->reason = BARCODE_REASON_INPUT;
        return false;
    }
    if ((!p_config->b_calibration_confirmed) ||
        (!p_config->b_codebook_confirmed) ||
        (0U == p_config->symbol_count))
    {
        memset(p_state, 0, sizeof(*p_state));
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
        test_line_main_legacy_reject(p_state,
            (advance < -p_config->reverse_tolerance_m) ?
            BARCODE_REASON_REVERSE : BARCODE_REASON_DISTANCE, p_output);
        p_state->b_started = false;
        return true;
    }
    if (((now_ms - p_state->previous_ms) > p_config->maximum_gap_ms) ||
        ((p_state->b_frame) && ((now_ms - p_state->frame_start_ms) >
                              p_config->frame_timeout_ms)))
    {
        test_line_main_legacy_reject(p_state, BARCODE_REASON_TIMEOUT,
                                     p_output);
        p_state->b_started = false;
        return true;
    }
    p_state->previous_ms = now_ms;
    if (advance > 0.0)
    {
        p_state->high_water_m = distance_m;
    }
    test_line_main_legacy_acquire(p_config, p_state, darkness, now_ms,
                                  p_output);
    return true;
}

/** @brief Removed barcode_reject() on doubles. */
static void test_line_main_legacy_reject (test_line_main_legacy_t * p_state,
                                          barcode_reason_t reason,
                                          test_line_main_verdict_t * p_output)
{
    p_output->event      = BARCODE_EVENT_REJECTED;
    p_output->reason     = reason;
    p_output->run_count  = p_state->run_count;
    p_state->b_frame     = false;
    p_state->b_armed     = false;
    p_state->b_candidate = false;
    p_state->run_count   = 0U;
    p_state->run_start_m = p_state->high_water_m;
}

/** @brief Removed barcode_transition() on doubles. */
static void test_line_main_legacy_transition (
    const barcode_config_t * p_config, test_line_main_legacy_t * p_state,
    uint32_t now_ms, test_line_main_verdict_t * p_output)
{
    double width = p_state->candidate_start_m - p_state->run_start_m;

    if (p_state->b_frame)
    {
        if (p_state->run_count >= BARCODE_MAX_RUNS)
        {
            test_line_main_legacy_reject(p_state, BARCODE_REASON_OVERFLOW,
                                         p_output);
        }
        else if ((width < p_config->minimum_run_m) ||
                 (width > p_config->maximum_run_m))
        {
            test_line_main_legacy_reject(p_state, BARCODE_REASON_WIDTH,
                                         p_output);
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

/** @brief Removed barcode_acquire() on doubles. */
static void test_line_main_legacy_acquire (const barcode_config_t * p_config,
                                           test_line_main_legacy_t * p_state,
                                           double darkness, uint32_t now_ms,
                                           test_line_main_verdict_t * p_output)
{
    bool   b_black = p_state->b_current_black;
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
            test_line_main_legacy_transition(p_config, p_state, now_ms,
                                             p_output);
        }
    }
    else
    {
        p_state->b_candidate = false;
    }
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
        test_line_main_legacy_reject(p_state, BARCODE_REASON_OVERFLOW,
                                     p_output);
    }
    else if ((!p_state->b_current_black) && (!p_state->b_candidate) &&
             (span >= p_config->quiet_zone_m))
    {
        if (p_state->b_frame)
        {
            test_line_main_legacy_finish(p_config, p_state, p_output);
        }
        p_state->b_armed = true;
    }
    else if ((p_state->b_frame) && (p_state->b_current_black) &&
             (span > p_config->maximum_run_m))
    {
        test_line_main_legacy_reject(p_state, BARCODE_REASON_WIDTH,
                                     p_output);
    }
    else
    {
        /* Incomplete frames remain pending until bounded termination. */
    }
}

/** @brief Removed barcode_finish() on doubles, labels as characters. */
static void test_line_main_legacy_finish (const barcode_config_t * p_config,
                                          test_line_main_legacy_t * p_state,
                                          test_line_main_verdict_t * p_output)
{
    uint32_t symbol;
    uint32_t matches = 0U;
    const barcode_symbol_t * p_match = NULL;

    p_output->run_count = p_state->run_count;
    p_output->event     = BARCODE_EVENT_UNKNOWN;
    p_output->reason    = BARCODE_REASON_NO_MATCH;
    for (symbol = 0U; symbol < p_config->symbol_count; ++symbol)
    {
        if (test_line_main_legacy_matches(p_config, p_state,
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
    else if ((1U == matches) && (NULL != p_match))
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
            p_output->event        = BARCODE_EVENT_DECODED;
            p_output->reason       = BARCODE_REASON_NONE;
            p_output->action       = p_match->action;
            p_output->label        = p_match->label;
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

/** @brief Removed barcode_symbol_matches() on doubles. */
static bool test_line_main_legacy_matches (
    const barcode_config_t * p_config,
    const test_line_main_legacy_t * p_state,
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

/** @brief Production line calibration (3500 white, 500 black), confirmed. */
static void test_line_main_line_config (line_ctrl_config_t * p_line)
{
    uint32_t channel;

    line_ctrl_defaults(p_line);
    for (channel = 0U; channel < LINE_CTRL_CHANNELS; ++channel)
    {
        p_line->white_adc[channel] = 3500U;
        p_line->black_adc[channel] = 500U;
    }
    p_line->b_calibration_confirmed = true;
}

/** @brief Configure a synthetic three-run fixture, not a course symbol. */
static void test_line_main_prepare (test_line_main_fixture_t * p_test)
{
    const double step = TEST_LINE_MAIN_STEP_M;

    memset(p_test, 0, sizeof(*p_test));
    p_test->step_m = step;
    test_line_main_line_config(&p_test->line);
    barcode_defaults(&p_test->config);
    barcode_init(&p_test->state);
    /* The default limits in 0.1 mm fixture steps; 250 steps (not 600)
     * keeps every stored width within uint8_t.
     */
    p_test->config.minimum_run_m           = 10.0 * step;
    p_test->config.maximum_run_m           = 250.0 * step;
    p_test->config.quiet_zone_m            = 150.0 * step;
    p_test->config.maximum_frame_m         = 2500.0 * step;
    p_test->config.maximum_step_m          = 2.0 * step;
    p_test->config.reverse_tolerance_m     = 5.0 * step;
    p_test->config.absolute_tolerance_m    = 3.0 * step;
    p_test->config.duplicate_distance_m    = 2000.0 * step;
    p_test->config.b_calibration_confirmed = true;
    p_test->config.b_codebook_confirmed    = true;
    p_test->config.symbol_count            = 1U;
    p_test->config.symbols[0].run_count    = 3U;
    p_test->config.symbols[0].width_m[0]   = 40.0 * step;
    p_test->config.symbols[0].width_m[1]   = 40.0 * step;
    p_test->config.symbols[0].width_m[2]   = 80.0 * step;
    p_test->config.symbols[0].label        = 'T';
    p_test->config.symbols[0].action       = BARCODE_ACTION_LEFT;
    test_line_main_check(test_line_main_derive(p_test),
                         "synthetic fixture derives");
}

/** @brief Convert the authored fixture at one half-count per step. */
static bool test_line_main_derive (test_line_main_fixture_t * p_test)
{
    return barcode_derive(&p_test->config, p_test->step_m, &p_test->line,
                          &p_test->runtime);
}

/** @brief Darkness the removed decoder saw for a class (NaN if invalid). */
static double test_line_main_level (const barcode_config_t * p_config,
                                    line_ctrl_class_t darkness)
{
    double level = NAN;

    if (LINE_CTRL_LIGHT == darkness)
    {
        level = 0.0;
    }
    else if (LINE_CTRL_DARK == darkness)
    {
        level = 1.0;
    }
    else if (LINE_CTRL_BAND == darkness)
    {
        level = 0.5 * (p_config->black_on + p_config->black_off);
    }
    else
    {
        /* INVALID: the removed decoder rejected non-finite darkness. */
    }
    return level;
}

/** @brief Lowest codebook index with this label (the canonical index). */
static uint32_t test_line_main_lowest (const barcode_config_t * p_config,
                                       char label)
{
    uint32_t index = 0U;

    while ((index < p_config->symbol_count) &&
           (p_config->symbols[index].label != label))
    {
        ++index;
    }
    return index;
}

/** @brief One sample through both decoders; any difference fails. */
static bool test_line_main_sample (test_line_main_fixture_t * p_test,
                                   line_ctrl_class_t darkness,
                                   int32_t position)
{
    /* The decoder sees the signed position modulo 2^32. */
    uint32_t coordinate = p_test->origin + (uint32_t) position;
    bool b_result = barcode_step(&p_test->runtime, &p_test->state, darkness,
                                 coordinate, p_test->now_ms,
                                 &p_test->output);
    bool b_legacy = test_line_main_legacy_step(&p_test->config,
        &p_test->legacy, test_line_main_level(&p_test->config, darkness),
        (double) position * p_test->step_m, p_test->now_ms,
        &p_test->verdict);
    bool b_same = (b_result == b_legacy) &&
        (p_test->output.event == p_test->verdict.event) &&
        (p_test->output.reason == p_test->verdict.reason) &&
        (p_test->output.action == p_test->verdict.action) &&
        (p_test->output.run_count == p_test->verdict.run_count) &&
        (p_test->state.b_frame == p_test->legacy.b_frame) &&
        ((BARCODE_EVENT_DECODED != p_test->output.event) ||
         (p_test->output.symbol ==
          test_line_main_lowest(&p_test->config, p_test->verdict.label)));

    test_line_main_check(b_same, "half-count decoder equals the removed one");
    return b_result;
}

/** @brief Feed distance samples and retain non-idle event totals. */
static void test_line_main_feed (test_line_main_fixture_t * p_test,
                                 line_ctrl_class_t darkness, uint32_t steps)
{
    uint32_t sample;
    int32_t  position;
    bool     b_result;

    for (sample = 0U; sample < steps; ++sample)
    {
        ++p_test->position;
        ++p_test->now_ms;
        position = p_test->position;
        if (p_test->b_jitter && (6U == (sample % 7U)))
        {
            /* Every seventh sample glitches two steps back (0.15 mm was
             * 1.5 steps): one step behind the high water, then a two-step
             * catch-up. Sample 6, not 0, so the first sample of the
             * fixture is never the glitch.
             */
            position -= TEST_LINE_MAIN_JITTER_STEPS;
        }
        b_result = test_line_main_sample(p_test, darkness, position);
        test_line_main_check(b_result, "barcode sample structurally valid");
        if (BARCODE_EVENT_NONE != p_test->output.event)
        {
            ++p_test->events[p_test->output.event];
            ++p_test->reasons[p_test->output.reason];
            p_test->last_run_count = p_test->output.run_count;
            if (BARCODE_EVENT_DECODED != p_test->output.event)
            {
                test_line_main_check(BARCODE_ACTION_UNKNOWN ==
                                     p_test->output.action,
                                     "nondecode event carries no action");
            }
        }
        if (BARCODE_EVENT_DECODED == p_test->output.event)
        {
            p_test->last_action = p_test->output.action;
            p_test->last_symbol = p_test->output.symbol;
        }
    }
}

/** @brief Generate one synthetic framed pattern with a selectable last run. */
static void test_line_main_symbol (test_line_main_fixture_t * p_test,
                                   uint32_t final_steps)
{
    test_line_main_feed(p_test, LINE_CTRL_LIGHT, TEST_LINE_MAIN_WHITE_STEPS);
    test_line_main_feed(p_test, LINE_CTRL_DARK, TEST_LINE_MAIN_BLACK_STEPS);
    test_line_main_feed(p_test, LINE_CTRL_LIGHT, TEST_LINE_MAIN_BLACK_STEPS);
    test_line_main_feed(p_test, LINE_CTRL_DARK, final_steps);
    test_line_main_feed(p_test, LINE_CTRL_LIGHT, TEST_LINE_MAIN_WHITE_STEPS);
}

/** @brief Empty, unconfirmed and malformed configurations cannot decode. */
static void test_line_main_gate (void)
{
    test_line_main_fixture_t test;
    const double step = TEST_LINE_MAIN_STEP_M;

    test_line_main_prepare(&test);
    barcode_defaults(&test.config);
    test_line_main_check(barcode_config_valid(&test.config),
                         "barcode defaults structurally valid");
    /* 60 mm is 491 fixture steps: wider than a uint8_t width. */
    test_line_main_check((!test_line_main_derive(&test)) &&
                         (!test.runtime.b_valid),
                         "default maximum run exceeds 255 fixture steps");
    test.step_m = TEST_LINE_MAIN_COARSE_M;
    test_line_main_check(test_line_main_derive(&test),
                         "defaults derive at a 0.49 mm half-count");
    test_line_main_check(test_line_main_sample(&test, LINE_CTRL_LIGHT, 0),
                         "empty codebook call");
    test_line_main_check(BARCODE_EVENT_INHIBITED == test.output.event,
                         "empty codebook inhibited");
    test_line_main_prepare(&test);
    test.config.b_codebook_confirmed = false;
    test_line_main_check(test_line_main_derive(&test) &&
                         test_line_main_sample(&test, LINE_CTRL_LIGHT, 0),
                         "unconfirmed codebook call");
    test_line_main_check(BARCODE_EVENT_INHIBITED == test.output.event,
                         "unconfirmed codebook inhibited");
    test_line_main_prepare(&test);
    test.config.b_calibration_confirmed = false;
    test_line_main_check(test_line_main_derive(&test) &&
                         test_line_main_sample(&test, LINE_CTRL_LIGHT, 0),
                         "uncalibrated barcode call");
    test_line_main_check(BARCODE_EVENT_INHIBITED == test.output.event,
                         "uncalibrated barcode inhibited");
    test.config.symbols[0].run_count = 2U;
    test_line_main_check((!barcode_config_valid(&test.config)) &&
                         (!test_line_main_derive(&test)),
                         "even run count is invalid framing");
    test_line_main_prepare(&test);
    test.config.symbols[0].width_m[1] = test.config.quiet_zone_m;
    test_line_main_check(!barcode_config_valid(&test.config),
                         "internal white must be below quiet zone");
    test_line_main_prepare(&test);
    test.config.minimum_run_m = 10.0 * step;
    test.config.maximum_step_m = 2.0 * step;
    test_line_main_check(barcode_config_valid(&test.config),
                         "five samples across minimum run accepted");
    test.config.maximum_step_m = 2.5 * step;
    test_line_main_check(!barcode_config_valid(&test.config),
                         "four samples across minimum run rejected");
    test_line_main_prepare(&test);
    test.config.relative_tolerance = NAN;
    test_line_main_check(!barcode_config_valid(&test.config),
                         "NaN tolerance invalid");
}

/** @brief Every navigation action and distance jitter decode explicitly. */
static void test_line_main_decode (void)
{
    test_line_main_fixture_t test;
    const barcode_action_t actions[] =
    {
        BARCODE_ACTION_LEFT, BARCODE_ACTION_RIGHT,
        BARCODE_ACTION_STRAIGHT, BARCODE_ACTION_UTURN
    };
    size_t action;
    const size_t action_count = sizeof(actions) / sizeof(actions[0]);

    for (action = 0U; action < action_count; ++action)
    {
        test_line_main_prepare(&test);
        test.config.symbols[0].action = actions[action];
        test_line_main_symbol(&test, TEST_LINE_MAIN_WIDE_STEPS);
        test_line_main_check(1U == test.events[BARCODE_EVENT_DECODED],
                             "single decode per synthetic barcode");
        test_line_main_check(actions[action] == test.last_action,
                             "configured action preserved");
        test_line_main_check((0U == test.last_symbol) &&
                             ('T' == test.config.symbols[0].label),
                             "configured label as its codebook index");
        test_line_main_check(0U == test.events[BARCODE_EVENT_REJECTED],
                             "clean frame has no rejection");
    }
    test_line_main_prepare(&test);
    test.b_jitter = true;
    test.config.minimum_run_m = 12.5 * TEST_LINE_MAIN_STEP_M;
    test.config.maximum_step_m = 2.5 * TEST_LINE_MAIN_STEP_M;
    test_line_main_check(test_line_main_derive(&test),
                         "jitter fixture derives");
    test_line_main_symbol(&test, TEST_LINE_MAIN_WIDE_STEPS);
    test_line_main_check(1U == test.events[BARCODE_EVENT_DECODED],
                         "sub-tolerance odometry jitter");
    test_line_main_prepare(&test);
    test_line_main_feed(&test, LINE_CTRL_LIGHT, TEST_LINE_MAIN_WHITE_STEPS);
    test_line_main_feed(&test, LINE_CTRL_DARK, TEST_LINE_MAIN_NOISE_STEPS);
    test_line_main_symbol(&test, TEST_LINE_MAIN_WIDE_STEPS);
    test_line_main_check(1U == test.events[BARCODE_EVENT_DECODED],
                         "short black speck suppressed");
    test_line_main_check(0U == test.events[BARCODE_EVENT_UNKNOWN],
                         "speck creates no false unknown frame");
}

/** @brief Reject nonmatching or multiply matching configured patterns. */
static void test_line_main_no_match (void)
{
    test_line_main_fixture_t test;

    test_line_main_prepare(&test);
    test_line_main_symbol(&test, TEST_LINE_MAIN_LONG_STEPS);
    test_line_main_check(1U == test.events[BARCODE_EVENT_UNKNOWN],
                         "wrong width unknown");
    test_line_main_check(0U == test.events[BARCODE_EVENT_DECODED],
                         "unknown has no action");
    test_line_main_prepare(&test);
    test.config.symbol_count = 2U;
    test.config.symbols[1] = test.config.symbols[0];
    test.config.symbols[1].action = BARCODE_ACTION_RIGHT;
    test_line_main_check(test_line_main_derive(&test),
                         "two-symbol codebook derives");
    test_line_main_symbol(&test, TEST_LINE_MAIN_WIDE_STEPS);
    test_line_main_check(1U == test.reasons[BARCODE_REASON_AMBIGUOUS],
                         "ambiguous patterns rejected");
    test_line_main_check(0U == test.events[BARCODE_EVENT_DECODED],
                         "ambiguous pattern cannot direct motion");
    test_line_main_prepare(&test);
    test_line_main_feed(&test, LINE_CTRL_DARK, TEST_LINE_MAIN_BLACK_STEPS);
    test_line_main_feed(&test, LINE_CTRL_LIGHT, TEST_LINE_MAIN_BLACK_STEPS);
    test_line_main_feed(&test, LINE_CTRL_DARK, TEST_LINE_MAIN_WIDE_STEPS);
    test_line_main_feed(&test, LINE_CTRL_LIGHT, TEST_LINE_MAIN_WHITE_STEPS);
    test_line_main_check(0U == test.events[BARCODE_EVENT_DECODED],
                         "missing leading quiet zone ignored");
    /* A one-run 40-step template against a 40/40/80 frame: the first run
     * matches, but the run counts differ.
     */
    test_line_main_prepare(&test);
    test.config.symbols[0].run_count = 1U;
    test_line_main_check(test_line_main_derive(&test),
                         "one-run template derives");
    test_line_main_symbol(&test, TEST_LINE_MAIN_WIDE_STEPS);
    test_line_main_check((1U == test.reasons[BARCODE_REASON_NO_MATCH]) &&
                         (0U == test.events[BARCODE_EVENT_DECODED]),
                         "matching first run plus extra runs is unknown");
}

/** @brief Reverse movement, stale data and invalid samples clear frames. */
static void test_line_main_faults (void)
{
    test_line_main_fixture_t test;
    uint32_t pause;

    test_line_main_prepare(&test);
    test_line_main_feed(&test, LINE_CTRL_LIGHT, TEST_LINE_MAIN_WHITE_STEPS);
    test_line_main_feed(&test, LINE_CTRL_DARK, TEST_LINE_MAIN_BLACK_STEPS);
    ++test.now_ms;
    test_line_main_check(test_line_main_sample(&test, LINE_CTRL_DARK,
                         test.position - TEST_LINE_MAIN_REVERSE_STEPS),
                         "reverse sample handled");
    test_line_main_check(BARCODE_REASON_REVERSE == test.output.reason,
                         "reverse rejects frame");
    test_line_main_check(!test.state.b_frame, "reverse clears collection");
    test_line_main_prepare(&test);
    test_line_main_feed(&test, LINE_CTRL_LIGHT, TEST_LINE_MAIN_WHITE_STEPS);
    test_line_main_feed(&test, LINE_CTRL_DARK, TEST_LINE_MAIN_BLACK_STEPS);
    test.now_ms += test.config.maximum_gap_ms + 1U;
    test_line_main_check(test_line_main_sample(&test, LINE_CTRL_DARK,
                                               test.position),
                         "sample gap handled");
    test_line_main_check(BARCODE_REASON_TIMEOUT == test.output.reason,
                         "sample gap rejects frame");
    test_line_main_prepare(&test);
    test.config.frame_timeout_ms = 40U;
    test_line_main_check(test_line_main_derive(&test),
                         "short frame timeout derives");
    test_line_main_feed(&test, LINE_CTRL_LIGHT, TEST_LINE_MAIN_WHITE_STEPS);
    test_line_main_feed(&test, LINE_CTRL_DARK, TEST_LINE_MAIN_BLACK_STEPS);
    for (pause = 0U; pause < TEST_LINE_MAIN_PAUSE_STEPS; ++pause)
    {
        test.now_ms += 10U;
        test_line_main_check(test_line_main_sample(&test, LINE_CTRL_DARK,
                                                   test.position),
                             "stationary frame sample");
        if (BARCODE_REASON_TIMEOUT == test.output.reason)
        {
            ++test.reasons[BARCODE_REASON_TIMEOUT];
        }
    }
    test_line_main_check(1U == test.reasons[BARCODE_REASON_TIMEOUT],
                         "stationary frame times out exactly once");
    test_line_main_check(!test_line_main_sample(&test, LINE_CTRL_INVALID, 0),
                         "invalid darkness class rejected");
    test_line_main_check((BARCODE_EVENT_REJECTED == test.output.event) &&
                         (BARCODE_REASON_INPUT == test.output.reason),
                         "invalid class is an input rejection");
    test_line_main_check(!barcode_step(&test.runtime, NULL, LINE_CTRL_LIGHT,
                                       0U, 0U, &test.output),
                         "NULL state rejected");
    test_line_main_check(!barcode_step(&test.runtime, &test.state,
                                       LINE_CTRL_LIGHT, 0U, 0U, NULL),
                         "NULL output rejected");
    test_line_main_check(!barcode_step(NULL, &test.state, LINE_CTRL_LIGHT,
                                       0U, 0U, &test.output),
                         "NULL runtime rejected");
    memset(&test.runtime, 0, sizeof(test.runtime));
    test_line_main_check(!barcode_step(&test.runtime, &test.state,
                                       LINE_CTRL_LIGHT, 0U, 0U,
                                       &test.output),
                         "underived runtime rejected");
    test_line_main_check(BARCODE_ACTION_UNKNOWN == test.output.action,
                         "invalid input has no navigation action");
}

/** @brief Excess run count cannot write beyond fixed storage. */
static void test_line_main_overflow (void)
{
    test_line_main_fixture_t test;
    uint32_t pair;

    test_line_main_prepare(&test);
    test_line_main_feed(&test, LINE_CTRL_LIGHT, TEST_LINE_MAIN_WHITE_STEPS);
    for (pair = 0U; pair < TEST_LINE_MAIN_RUN_PAIRS; ++pair)
    {
        test_line_main_feed(&test, LINE_CTRL_DARK,
                            TEST_LINE_MAIN_BLACK_STEPS);
        test_line_main_feed(&test, LINE_CTRL_LIGHT,
                            TEST_LINE_MAIN_BLACK_STEPS);
    }
    test_line_main_feed(&test, LINE_CTRL_LIGHT, TEST_LINE_MAIN_WHITE_STEPS);
    test_line_main_check(1U == test.reasons[BARCODE_REASON_OVERFLOW],
                         "33rd run produces bounded overflow rejection");
    test_line_main_check(BARCODE_MAX_RUNS == test.last_run_count,
                         "overflow reports 32 runs in the 6-bit field");
    test_line_main_check(0U == test.events[BARCODE_EVENT_DECODED],
                         "overflow does not produce action");
    test_line_main_symbol(&test, TEST_LINE_MAIN_WIDE_STEPS);
    test_line_main_check(1U == test.events[BARCODE_EVENT_DECODED],
                         "fresh quiet zone recovers after overflow");
}

/** @brief Repeat guard suppresses close duplicate identities. */
static void test_line_main_duplicate (void)
{
    test_line_main_fixture_t test;

    test_line_main_prepare(&test);
    test_line_main_symbol(&test, TEST_LINE_MAIN_WIDE_STEPS);
    test_line_main_symbol(&test, TEST_LINE_MAIN_WIDE_STEPS);
    test_line_main_check(1U == test.events[BARCODE_EVENT_DECODED],
                         "duplicate does not dispatch a second action");
    test_line_main_check(1U == test.reasons[BARCODE_REASON_DUPLICATE],
                         "near duplicate is explicit");
    test.config.duplicate_distance_m = 100.0 * TEST_LINE_MAIN_STEP_M;
    test_line_main_check(test_line_main_derive(&test),
                         "shorter duplicate distance derives");
    test_line_main_symbol(&test, TEST_LINE_MAIN_WIDE_STEPS);
    test_line_main_check(2U == test.events[BARCODE_EVENT_DECODED],
                         "configured separation permits later symbol");
}

/** @brief Exact binary distances isolate framing and persistence boundaries. */
static void test_line_main_limits (void)
{
    test_line_main_fixture_t test;
    uint32_t scenario;
    const double step_m = 1.0 / 8192.0;

    for (scenario = 0U; scenario < 6U; ++scenario)
    {
        test_line_main_prepare(&test);
        test.step_m = step_m;
        test.position = -1;
        test.config.minimum_run_m = 10.0 * step_m;
        test.config.maximum_step_m = 2.0 * step_m;
        test.config.maximum_run_m =
            ((0U == scenario) || (3U == scenario) ? 64.0 : 96.0) * step_m;
        test.config.quiet_zone_m = 128.0 * step_m;
        test.config.maximum_frame_m =
            ((1U == scenario) || (4U == scenario) ? 160.0 : 512.0) * step_m;
        test.config.symbols[0].width_m[0] = 64.0 * step_m;
        test.config.symbols[0].width_m[1] = 32.0 * step_m;
        test.config.symbols[0].width_m[2] = 64.0 * step_m;
        test_line_main_check(barcode_config_valid(&test.config) &&
                             test_line_main_derive(&test),
                             "exact-boundary codebook structurally valid");
        test_line_main_feed(&test, LINE_CTRL_LIGHT, (2U == scenario) ? 128U
                            : ((5U == scenario) ? 127U : 136U));
        test_line_main_feed(&test, LINE_CTRL_DARK, 64U);
        test_line_main_feed(&test, LINE_CTRL_LIGHT, 32U);
        test_line_main_feed(&test, LINE_CTRL_DARK,
                            ((3U == scenario) || (4U == scenario)) ? 65U : 64U);
        test_line_main_feed(&test, LINE_CTRL_LIGHT, 136U);
        if (scenario < 3U)
        {
            test_line_main_check(1U == test.events[BARCODE_EVENT_DECODED],
                (0U == scenario) ? "maximum run excludes edge confirmation"
                : ((1U == scenario) ? "maximum frame excludes trailing quiet"
                   : "minimum leading quiet zone accepted at black edge"));
            test_line_main_check(0U == test.events[BARCODE_EVENT_REJECTED],
                                 "valid boundary frame never rejected");
        }
        else
        {
            test_line_main_check(0U == test.events[BARCODE_EVENT_DECODED],
                "beyond run/frame or below quiet boundary cannot decode");
            if (scenario < 5U)
            {
                test_line_main_check(1U == test.reasons[(3U == scenario)
                    ? BARCODE_REASON_WIDTH : BARCODE_REASON_OVERFLOW],
                    "one-step run/frame excess rejected explicitly");
            }
        }
    }
}

/** @brief One half-count either side of each per-sample threshold. */
static void test_line_main_boundaries (void)
{
    test_line_main_fixture_t test;
    const uint32_t widths[] = {30U, 31U, 49U, 50U};
    uint32_t value;
    size_t   index;

    for (value = 5U; value <= 6U; ++value)
    {
        test_line_main_prepare(&test);
        test_line_main_feed(&test, LINE_CTRL_LIGHT,
                            TEST_LINE_MAIN_WHITE_STEPS);
        ++test.now_ms;
        /* value <= 6 fits int32_t. */
        test_line_main_check(test_line_main_sample(&test, LINE_CTRL_LIGHT,
                             test.position - (int32_t) value) &&
                             ((5U == value) ==
                              (BARCODE_EVENT_NONE == test.output.event)),
                             "reverse: 5 steps back kept, 6 rejected");
    }
    for (value = 2U; value <= 3U; ++value)
    {
        test_line_main_prepare(&test);
        test_line_main_feed(&test, LINE_CTRL_LIGHT,
                            TEST_LINE_MAIN_WHITE_STEPS);
        ++test.now_ms;
        /* value <= 3 fits int32_t. */
        test_line_main_check(test_line_main_sample(&test, LINE_CTRL_LIGHT,
                             test.position + (int32_t) value) &&
                             ((2U == value) ==
                              (BARCODE_EVENT_NONE == test.output.event)),
                             "distance: 2 steps ahead kept, 3 rejected");
    }
    for (value = 560U; value <= 561U; ++value)
    {
        /* Consecutive symbols end exactly 560 steps apart. */
        test_line_main_prepare(&test);
        test.config.duplicate_distance_m = (double) value *
                                           TEST_LINE_MAIN_STEP_M;
        test_line_main_check(test_line_main_derive(&test),
                             "boundary duplicate distance derives");
        test_line_main_symbol(&test, TEST_LINE_MAIN_WIDE_STEPS);
        test_line_main_symbol(&test, TEST_LINE_MAIN_WIDE_STEPS);
        test_line_main_check((560U == value) ?
                             (2U == test.events[BARCODE_EVENT_DECODED]) :
                             (1U == test.reasons[BARCODE_REASON_DUPLICATE]),
                             "duplicate: |D| < 560 steps excludes 560");
    }
    for (index = 0U; index < (sizeof(widths) / sizeof(widths[0])); ++index)
    {
        /* 40-step runs: T = 3 + 0.15 * 40 = 9, window [31, 49]. */
        test_line_main_prepare(&test);
        test_line_main_feed(&test, LINE_CTRL_LIGHT,
                            TEST_LINE_MAIN_WHITE_STEPS);
        test_line_main_feed(&test, LINE_CTRL_DARK, widths[index]);
        test_line_main_feed(&test, LINE_CTRL_LIGHT,
                            TEST_LINE_MAIN_BLACK_STEPS);
        test_line_main_feed(&test, LINE_CTRL_DARK, TEST_LINE_MAIN_WIDE_STEPS);
        test_line_main_feed(&test, LINE_CTRL_LIGHT,
                            TEST_LINE_MAIN_WHITE_STEPS);
        test_line_main_check((((31U <= widths[index]) &&
                               (widths[index] <= 49U)) ? 1U : 0U) ==
                             test.events[BARCODE_EVENT_DECODED],
                             "strict window [31, 49] is inclusive");
    }
    for (value = 9U; value <= 10U; ++value)
    {
        test_line_main_prepare(&test);
        test_line_main_feed(&test, LINE_CTRL_LIGHT,
                            TEST_LINE_MAIN_WHITE_STEPS);
        test_line_main_feed(&test, LINE_CTRL_DARK, value);
        test_line_main_feed(&test, LINE_CTRL_LIGHT,
                            TEST_LINE_MAIN_WHITE_STEPS);
        test_line_main_check(((9U == value) ? 1U : 0U) ==
                             test.reasons[BARCODE_REASON_WIDTH],
                             "minimum run: 10 steps stored, 9 rejected");
    }
}

/** @brief A 255 half-count run is stored exactly; 256 is a width fault. */
static void test_line_main_widths (void)
{
    test_line_main_fixture_t test;
    const double step = TEST_LINE_MAIN_STEP_M;

    test_line_main_prepare(&test);
    test.config.maximum_run_m = 256.0 * step;
    test_line_main_check(!test_line_main_derive(&test),
                         "maximum run of 256 half-counts cannot derive");
    test.config.maximum_run_m = nextafter(256.0 * step, 0.0);
    test_line_main_check(test_line_main_derive(&test) &&
                         (255U == test.runtime.maximum_run_hc),
                         "a maximum just below 256 floors to 255");
    test.config.maximum_run_m = 255.0 * step;
    test.config.symbols[0].run_count = 1U;
    test.config.symbols[0].width_m[0] = 255.0 * step;
    test_line_main_check(test_line_main_derive(&test),
                         "255 half-count template derives");
    test_line_main_feed(&test, LINE_CTRL_LIGHT, TEST_LINE_MAIN_WHITE_STEPS);
    test_line_main_feed(&test, LINE_CTRL_DARK, 255U);
    test_line_main_feed(&test, LINE_CTRL_LIGHT, TEST_LINE_MAIN_WHITE_STEPS);
    test_line_main_check((1U == test.events[BARCODE_EVENT_DECODED]) &&
                         (0U == test.events[BARCODE_EVENT_REJECTED]),
                         "widest uint8_t run stored and decoded");
    test_line_main_feed(&test, LINE_CTRL_DARK, 256U);
    test_line_main_feed(&test, LINE_CTRL_LIGHT, TEST_LINE_MAIN_WHITE_STEPS);
    test_line_main_check(1U == test.reasons[BARCODE_REASON_WIDTH],
                         "256 half-counts rejected as width");
}

/** @brief Wrapping coordinates: a frame and its duplicate across 2^32. */
static void test_line_main_wrap (void)
{
    test_line_main_fixture_t test;

    test_line_main_prepare(&test);
    /* Fixture position 300 is the decoder's 2^32 - 1 to 0 step, inside
     * the last bar of the first frame.
     */
    test.origin = UINT32_MAX - 299U;
    test_line_main_symbol(&test, TEST_LINE_MAIN_WIDE_STEPS);
    test_line_main_check(1U == test.events[BARCODE_EVENT_DECODED],
                         "frame across the 2^32 wrap decodes");
    test_line_main_check(0U == test.events[BARCODE_EVENT_REJECTED],
                         "wrap is not a reverse or distance fault");
    test_line_main_symbol(&test, TEST_LINE_MAIN_WIDE_STEPS);
    test_line_main_check(1U == test.reasons[BARCODE_REASON_DUPLICATE],
                         "duplicate distance measured across the wrap");
}

/** @brief Duplicate characters share the lowest index; index 15 fits. */
static void test_line_main_labels (void)
{
    test_line_main_fixture_t test;
    barcode_symbol_t base;
    const double step = TEST_LINE_MAIN_STEP_M;
    uint32_t symbol;
    uint32_t run;

    test_line_main_prepare(&test);
    test.config.symbol_count = 3U;
    test.config.symbols[1] = test.config.symbols[0];
    test.config.symbols[1].label = 'A';
    test.config.symbols[1].width_m[2] = 160.0 * step;
    test.config.symbols[2] = test.config.symbols[0];
    test.config.symbols[2].width_m[2] = 120.0 * step;
    test.config.symbols[2].action = BARCODE_ACTION_RIGHT;
    test_line_main_check(test_line_main_derive(&test),
                         "duplicate-label codebook derives");
    test_line_main_symbol(&test, TEST_LINE_MAIN_LONG_STEPS);
    test_line_main_check((1U == test.events[BARCODE_EVENT_DECODED]) &&
                         (BARCODE_ACTION_RIGHT == test.last_action) &&
                         (0U == test.last_symbol),
                         "duplicate label reports its lowest index");
    test_line_main_symbol(&test, TEST_LINE_MAIN_WIDE_STEPS);
    test_line_main_check(1U == test.reasons[BARCODE_REASON_DUPLICATE],
                         "same character stays a duplicate");
    test_line_main_symbol(&test, 160U);
    test_line_main_check((2U == test.events[BARCODE_EVENT_DECODED]) &&
                         (1U == test.last_symbol),
                         "different character decodes nearby");
    test_line_main_prepare(&test);
    base = test.config.symbols[0];
    test.config.symbol_count = BARCODE_MAX_SYMBOLS;
    for (symbol = 0U; symbol < BARCODE_MAX_SYMBOLS; ++symbol)
    {
        test.config.symbols[symbol] = base;
        /* 'a' + 0..15 stays within char. */
        test.config.symbols[symbol].label = (char) ('a' + (int) symbol);
        if (symbol < (BARCODE_MAX_SYMBOLS - 1U))
        {
            test.config.symbols[symbol].run_count = 5U;
            for (run = 0U; run < 5U; ++run)
            {
                test.config.symbols[symbol].width_m[run] = 40.0 * step;
            }
        }
    }
    test_line_main_check(test_line_main_derive(&test),
                         "sixteen-symbol codebook derives");
    test_line_main_symbol(&test, TEST_LINE_MAIN_WIDE_STEPS);
    test_line_main_check((1U == test.events[BARCODE_EVENT_DECODED]) &&
                         ((BARCODE_MAX_SYMBOLS - 1U) == test.last_symbol),
                         "index 15 fits the 4-bit symbol field");
}

/**
 * @brief PDF vector A/Z run fixtures; no course command mapping is claimed.
 * @details Barcode Sample.pdf pp1-4, vector extraction recorded in source
 * review notes. The two 9-run end groups and narrow separators are included.
 * Three-to-one widths are evidenced; the metre unit, tolerances, scale range
 * and LEFT/RIGHT actions below are synthetic test choices, not course facts.
 */
static void test_line_main_sample_prepare (test_line_main_fixture_t * p_test)
{
    const char * const patterns[] =
    {
        "nwnnwnwnn" "n" "wnnnnwnnw" "n" "nwnnwnwnn",
        "nwnnwnwnn" "n" "nwwnwnnnn" "n" "nwnnwnwnn"
    };
    uint32_t symbol;
    uint32_t run;

    test_line_main_prepare(p_test);
    p_test->step_m = TEST_LINE_MAIN_SAMPLE_STEP;
    p_test->position = -1;
    p_test->config.minimum_run_m = 20.0 * p_test->step_m;
    /* Stored widths are uint8_t half-counts. */
    p_test->config.maximum_run_m = 255.0 * p_test->step_m;
    p_test->config.maximum_step_m = 4.0 * p_test->step_m;
    p_test->config.reverse_tolerance_m = p_test->step_m;
    p_test->config.quiet_zone_m = 1280.0 * p_test->step_m;
    p_test->config.absolute_tolerance_m = 2.0 * p_test->step_m;
    p_test->config.relative_tolerance = 0.03;
    p_test->config.minimum_scale = 0.8;
    p_test->config.maximum_scale = 1.2;
    p_test->config.b_scale_tolerant = true;
    p_test->config.symbol_count = 2U;
    for (symbol = 0U; symbol < 2U; ++symbol)
    {
        p_test->config.symbols[symbol].run_count =
            TEST_LINE_MAIN_SAMPLE_RUNS;
        p_test->config.symbols[symbol].label = (0U == symbol) ? 'A' : 'Z';
        p_test->config.symbols[symbol].action = (0U == symbol)
            ? BARCODE_ACTION_LEFT : BARCODE_ACTION_RIGHT;
        for (run = 0U; run < TEST_LINE_MAIN_SAMPLE_RUNS; ++run)
        {
            p_test->config.symbols[symbol].width_m[run] =
                (('w' == patterns[symbol][run]) ? 3.0 : 1.0) *
                TEST_LINE_MAIN_SAMPLE_UNIT;
        }
    }
    test_line_main_check(barcode_config_valid(&p_test->config) &&
                         test_line_main_derive(p_test),
                         "PDF vector fixture configuration valid");
}

/** @brief Sample each physical run; noise stays below persistence limits. */
static void test_line_main_sample_frame (test_line_main_fixture_t * p_test,
                                        const barcode_symbol_t * p_symbol,
                                        double scale, bool b_reverse,
                                        bool b_noise)
{
    uint32_t run;
    uint32_t index;
    uint32_t steps;
    line_ctrl_class_t darkness;

    test_line_main_feed(p_test, LINE_CTRL_LIGHT, TEST_LINE_MAIN_SAMPLE_WHITE);
    if (b_noise)
    {
        test_line_main_feed(p_test, LINE_CTRL_DARK,
                            TEST_LINE_MAIN_NOISE_STEPS);
        test_line_main_feed(p_test, LINE_CTRL_LIGHT,
                            TEST_LINE_MAIN_SAMPLE_WHITE);
    }
    for (run = 0U; run < p_symbol->run_count; ++run)
    {
        index = b_reverse ? ((uint32_t)p_symbol->run_count - 1U - run) : run;
        /* Fixture widths are a few hundred steps: lround fits uint32_t. */
        steps = (uint32_t)lround(p_symbol->width_m[index] * scale /
                                p_test->step_m);
        if (b_noise)
        {
            steps = (0U == (run % 2U)) ? (steps + 1U) : (steps - 1U);
        }
        darkness = (0U == (run % 2U)) ? LINE_CTRL_DARK : LINE_CTRL_LIGHT;
        test_line_main_feed(p_test, darkness, steps);
    }
    test_line_main_feed(p_test, LINE_CTRL_LIGHT, TEST_LINE_MAIN_SAMPLE_WHITE);
}

/** @brief Uniform scale changes need explicit opt-in; action is configured. */
static void test_line_main_scale_decode (void)
{
    test_line_main_fixture_t test;
    const double scales[] = {0.8, 0.9, 1.0, 1.1, 1.2};
    uint32_t symbol;
    size_t scale;
    bool b_expected;

    test_line_main_prepare(&test);
    test_line_main_check((!test.config.b_scale_tolerant) &&
                         (1.0 == test.config.minimum_scale) &&
                         (1.0 == test.config.maximum_scale),
                         "defaults retain strict physical-width matching");
    for (symbol = 0U; symbol < 2U; ++symbol)
    {
        for (scale = 0U; scale < (sizeof(scales) / sizeof(scales[0])); ++scale)
        {
            test_line_main_sample_prepare(&test);
            test_line_main_sample_frame(&test, &test.config.symbols[symbol],
                                         scales[scale], false, false);
            test_line_main_check(1U == test.events[BARCODE_EVENT_DECODED],
                                 "scaled source A/Z frame decodes once");
            test_line_main_check(symbol == test.last_symbol,
                                 "source vector label preserved");
            test_line_main_check(test.config.symbols[symbol].action ==
                                 test.last_action,
                                 "synthetic fixture action preserved");
            test_line_main_check(0U == test.events[BARCODE_EVENT_REJECTED],
                                 "scaled frame meets physical limits");
            test_line_main_sample_prepare(&test);
            test.config.b_scale_tolerant = false;
            test_line_main_check(test_line_main_derive(&test),
                                 "strict A/Z fixture derives");
            test_line_main_sample_frame(&test, &test.config.symbols[symbol],
                                         scales[scale], false, false);
            b_expected = (1.0 == scales[scale]);
            test_line_main_check((b_expected ? 1U : 0U) ==
                                 test.events[BARCODE_EVENT_DECODED],
                                 "strict mode retains original tolerance");
        }
    }
    test_line_main_sample_prepare(&test);
    test_line_main_sample_frame(&test, &test.config.symbols[0],
                                 1.0, false, true);
    test_line_main_check(1U == test.events[BARCODE_EVENT_DECODED],
                         "bounded edge noise and short speck tolerated");
    test_line_main_check(0U == test.events[BARCODE_EVENT_UNKNOWN],
                         "short speck produces no extra unknown frame");
}

/** @brief Reversed, malformed, distorted or ambiguous frames stay unknown. */
static void test_line_main_scale_unknown (void)
{
    test_line_main_fixture_t test;
    barcode_symbol_t observed;
    uint32_t scenario;
    uint32_t run;
    double temporary;

    for (scenario = 0U; scenario < 6U; ++scenario)
    {
        test_line_main_sample_prepare(&test);
        observed = test.config.symbols[0];
        if (1U == scenario)
        {
            /* Distort the leading framing group without changing its total. */
            temporary = observed.width_m[0];
            observed.width_m[0] = observed.width_m[1];
            observed.width_m[1] = temporary;
        }
        else if (2U == scenario)
        {
            /* Missing runs are not inferred by the scale matcher. */
            observed.run_count = TEST_LINE_MAIN_SAMPLE_RUNS - 2U;
        }
        else if (3U == scenario)
        {
            observed.width_m[12] *= 1.5;
        }
        else if (4U == scenario)
        {
            test.config.symbol_count = 3U;
            test.config.symbols[2] = test.config.symbols[0];
            test.config.symbols[2].label = 'X';
            test.config.symbols[2].action = BARCODE_ACTION_UTURN;
            for (run = 0U; run < observed.run_count; ++run)
            {
                test.config.symbols[2].width_m[run] *= 1.1;
            }
            test_line_main_check(test_line_main_derive(&test),
                                 "ambiguous scaled codebook derives");
        }
        else if (5U == scenario)
        {
            /* Sustained black noise splits one wide white into three runs. */
            memmove(&observed.width_m[4], &observed.width_m[2],
                    27U * sizeof(observed.width_m[0]));
            observed.width_m[1] = TEST_LINE_MAIN_SAMPLE_UNIT;
            observed.width_m[2] = TEST_LINE_MAIN_SAMPLE_UNIT;
            observed.width_m[3] = TEST_LINE_MAIN_SAMPLE_UNIT;
            observed.run_count = TEST_LINE_MAIN_SAMPLE_RUNS + 2U;
        }
        else
        {
            /* The source run sequence is sampled backwards below. */
        }
        test_line_main_sample_frame(&test, &observed,
                                     1.0, 0U == scenario, false);
        test_line_main_check(1U == test.events[BARCODE_EVENT_UNKNOWN],
                             "invalid or ambiguous scale frame is unknown");
        test_line_main_check(0U == test.events[BARCODE_EVENT_DECODED],
                             "no guessed motion action from uncertain frame");
        test_line_main_check(BARCODE_ACTION_UNKNOWN == test.last_action,
                             "unknown frame never assigns an action");
        test_line_main_check(1U == test.reasons[(4U == scenario)
                             ? BARCODE_REASON_AMBIGUOUS
                             : BARCODE_REASON_NO_MATCH],
                             "ambiguity is distinct from malformed pattern");
    }
}

/** @brief Scale does not bypass direction, sampled distance or run limits. */
static void test_line_main_scale_physical (void)
{
    test_line_main_fixture_t test;
    const double scales[] = {0.79, 1.21};
    size_t scale;

    for (scale = 0U; scale < (sizeof(scales) / sizeof(scales[0])); ++scale)
    {
        test_line_main_sample_prepare(&test);
        test_line_main_sample_frame(&test, &test.config.symbols[0],
                                     scales[scale], false, false);
        test_line_main_check(1U == test.reasons[BARCODE_REASON_NO_MATCH],
                             "scale estimate outside explicit bounds rejected");
        test_line_main_check(0U == test.events[BARCODE_EVENT_DECODED],
                             "residual tolerance cannot enlarge scale bounds");
    }
    test_line_main_sample_prepare(&test);
    /* Narrow runs of 0.25 * 42 = 10.5 steps (0.1 * 100 = 10 before) stay
     * between the 5-step edge persistence and the 20-step minimum run.
     */
    test_line_main_sample_frame(&test, &test.config.symbols[0],
                                 0.25, false, false);
    test_line_main_check(test.reasons[BARCODE_REASON_WIDTH] > 0U,
                         "undersized physical runs rejected before matching");
    test_line_main_check(0U == test.events[BARCODE_EVENT_DECODED],
                         "uniform scale cannot rescue undersized runs");
    test_line_main_sample_prepare(&test);
    test_line_main_feed(&test, LINE_CTRL_LIGHT, TEST_LINE_MAIN_SAMPLE_WHITE);
    test_line_main_feed(&test, LINE_CTRL_DARK, 20U);
    ++test.now_ms;
    test_line_main_check(test_line_main_sample(&test, LINE_CTRL_DARK,
                                               test.position + 5),
                         "excessive spatial sample gap handled");
    test_line_main_check(BARCODE_REASON_DISTANCE == test.output.reason,
                         "scale matcher preserves spatial sampling gate");
    test_line_main_check(BARCODE_ACTION_UNKNOWN == test.output.action,
                         "sampling rejection carries no action");
    test_line_main_sample_prepare(&test);
    test_line_main_feed(&test, LINE_CTRL_LIGHT, TEST_LINE_MAIN_SAMPLE_WHITE);
    test_line_main_feed(&test, LINE_CTRL_DARK, 20U);
    ++test.now_ms;
    test_line_main_check(test_line_main_sample(&test, LINE_CTRL_DARK,
                                               test.position - 2),
                         "reverse distance during scaled scan handled");
    test_line_main_check(BARCODE_REASON_REVERSE == test.output.reason,
                         "scale matching preserves forward scan requirement");
}

/** @brief Reject invalid scale settings and impossible scaled templates. */
static void test_line_main_scale_bounds (void)
{
    test_line_main_fixture_t test;
    const double bad_bounds[][2] =
    {
        {NAN, 1.2}, {0.8, INFINITY}, {0.49, 1.2},
        {0.8, 2.01}, {1.2, 0.8}, {0.0, 0.0}
    };
    size_t index;
    uint32_t run;
    double total = 0.0;

    for (index = 0U; index < (sizeof(bad_bounds) / sizeof(bad_bounds[0]));
         ++index)
    {
        test_line_main_sample_prepare(&test);
        test.config.minimum_scale = bad_bounds[index][0];
        test.config.maximum_scale = bad_bounds[index][1];
        test_line_main_check(!barcode_config_valid(&test.config),
                             "invalid optional scale interval rejected");
    }
    test_line_main_sample_prepare(&test);
    test.config.minimum_scale = BARCODE_MIN_SCALE;
    test.config.maximum_scale = BARCODE_MAX_SCALE;
    test_line_main_check(barcode_config_valid(&test.config) &&
                         test_line_main_derive(&test),
                         "hard scale bounds are inclusive");
    test_line_main_sample_prepare(&test);
    test.config.minimum_run_m = 0.9 * TEST_LINE_MAIN_SAMPLE_UNIT;
    test_line_main_check(!barcode_config_valid(&test.config),
                         "smallest scaled run must meet physical minimum");
    test_line_main_sample_prepare(&test);
    test.config.maximum_run_m = 3.3 * TEST_LINE_MAIN_SAMPLE_UNIT;
    test_line_main_check(!barcode_config_valid(&test.config),
                         "largest scaled run must meet physical maximum");
    test_line_main_sample_prepare(&test);
    test.config.quiet_zone_m = 3.3 * TEST_LINE_MAIN_SAMPLE_UNIT;
    test_line_main_check(!barcode_config_valid(&test.config),
                         "scaled internal white cannot become quiet zone");
    test_line_main_sample_prepare(&test);
    for (run = 0U; run < test.config.symbols[0].run_count; ++run)
    {
        total += test.config.symbols[0].width_m[run];
    }
    test.config.maximum_frame_m = 1.1 * total;
    test_line_main_check(!barcode_config_valid(&test.config),
                         "largest scaled frame must meet physical maximum");
    test_line_main_sample_prepare(&test);
    test.config.maximum_step_m = 5.0 * test.step_m;
    test_line_main_check(!barcode_config_valid(&test.config),
                         "scale config cannot weaken five-sample minimum");
}

/** @brief Quiet, alternating runs (black first) of the given widths in
 * steps, quiet. Returns DECODED or UNKNOWN when that single frame result
 * occurred without any rejection, else REJECTED.
 */
static barcode_event_t test_line_main_frame (
    test_line_main_fixture_t * p_test, const uint32_t * p_widths,
    uint32_t count)
{
    uint32_t run;
    uint32_t decoded;
    uint32_t unknown;

    test_line_main_feed(p_test, LINE_CTRL_LIGHT, TEST_LINE_MAIN_WHITE_STEPS);
    for (run = 0U; run < count; ++run)
    {
        test_line_main_feed(p_test, (0U == (run % 2U)) ? LINE_CTRL_DARK
                                                      : LINE_CTRL_LIGHT,
                            p_widths[run]);
    }
    test_line_main_feed(p_test, LINE_CTRL_LIGHT, TEST_LINE_MAIN_WHITE_STEPS);
    decoded = p_test->events[BARCODE_EVENT_DECODED];
    unknown = p_test->events[BARCODE_EVENT_UNKNOWN];
    if ((0U != p_test->events[BARCODE_EVENT_REJECTED]) ||
        (1U != (decoded + unknown)))
    {
        return BARCODE_EVENT_REJECTED;
    }
    return (1U == decoded) ? BARCODE_EVENT_DECODED : BARCODE_EVENT_UNKNOWN;
}

/** @brief Bars of low - 1, low, high and high + 1 steps against a one-run
 * template with an absolute tolerance only (rel 0, 14-step minimum run):
 * low and high decode and their neighbours stay unknown, in lockstep.
 */
static void test_line_main_window_case (double width_m, double absolute_m,
                                        uint32_t low, uint32_t high)
{
    test_line_main_fixture_t test;
    const uint32_t widths[4] = {low - 1U, low, high, high + 1U};
    uint32_t index;
    barcode_event_t expected;

    for (index = 0U; index < 4U; ++index)
    {
        test_line_main_prepare(&test);
        test.config.minimum_run_m = 14.0 * test.step_m;
        test.config.absolute_tolerance_m = absolute_m;
        test.config.relative_tolerance = 0.0;
        test.config.symbols[0].run_count = 1U;
        test.config.symbols[0].width_m[0] = width_m;
        test_line_main_check(test_line_main_derive(&test),
                             "one-run window fixture derives");
        expected = ((1U == index) || (2U == index)) ? BARCODE_EVENT_DECODED
                                                    : BARCODE_EVENT_UNKNOWN;
        test_line_main_check(expected ==
                             test_line_main_frame(&test, &widths[index], 1U),
                             "strict window edges equal the removed predicate");
    }
}

/** @brief nextafter() neighbours of the template width and of the absolute
 * tolerance (q = 2^-13 m keeps the removed predicate exact) move each
 * window edge by one step. The last case is the review reproduction: 32
 * steps with a tolerance just below 13 steps rejects a 19-step bar.
 */
static void test_line_main_window_neighbours (void)
{
    const double step = TEST_LINE_MAIN_STEP_M;

    test_line_main_window_case(40.0 * step, 9.0 * step, 31U, 49U);
    test_line_main_window_case(40.0 * step, nextafter(9.0 * step, 0.0),
                               32U, 48U);
    test_line_main_window_case(40.0 * step,
                               nextafter(9.0 * step, INFINITY), 31U, 49U);
    test_line_main_window_case(nextafter(40.0 * step, INFINITY), 9.0 * step,
                               32U, 49U);
    test_line_main_window_case(nextafter(40.0 * step, 0.0), 9.0 * step,
                               31U, 48U);
    test_line_main_window_case(32.0 * step, nextafter(13.0 * step, 0.0),
                               20U, 44U);
}

/** @brief Production half-count (pi * 32.5 mm / 360) with the apphost 12 mm
 * bar, 0.3 mm + 15 %: window [35, 49] through barcode_step in lockstep.
 * A 9 mm minimum run (32 half-counts) lets 34 and 35 reach the matcher.
 */
static void test_line_main_window_unit (void)
{
    test_line_main_fixture_t test;
    const uint32_t widths[] = {34U, 35U, 49U, 50U};
    const double width_m = 0.012;
    const double tolerance_m = 0.0003 + (0.15 * width_m);
    size_t index;
    barcode_event_t expected;

    for (index = 0U; index < (sizeof(widths) / sizeof(widths[0])); ++index)
    {
        test_line_main_prepare(&test);
        test.step_m = test_line_main_half_count();
        barcode_defaults(&test.config);
        test.config.b_calibration_confirmed = true;
        test.config.b_codebook_confirmed    = true;
        test.config.minimum_run_m           = 0.009;
        test.config.maximum_step_m          = 0.0015;
        test.config.symbol_count            = 1U;
        test.config.symbols[0].run_count    = 1U;
        test.config.symbols[0].width_m[0]   = width_m;
        test.config.symbols[0].label        = 'T';
        test.config.symbols[0].action       = BARCODE_ACTION_STRAIGHT;
        test_line_main_check(test_line_main_derive(&test) &&
                             (32U == test.runtime.minimum_run_hc) &&
                             (5U == test.runtime.maximum_step_hc) &&
                             (8U == test.runtime.persistence_hc),
                             "production window fixture runs 32/5/8");
        expected = ((35U <= widths[index]) && (widths[index] <= 49U)) ?
                   BARCODE_EVENT_DECODED : BARCODE_EVENT_UNKNOWN;
        test_line_main_check(expected ==
                             test_line_main_frame(&test, &widths[index], 1U),
                             "production strict window [35, 49] decodes");
    }
    test_line_main_check(
        test_line_main_margin_ok(test.config.minimum_run_m, test.step_m) &&
        test_line_main_margin_ok(0.25 * test.config.minimum_run_m,
                                 test.step_m) &&
        test_line_main_margin_ok(test.config.maximum_run_m, test.step_m) &&
        test_line_main_margin_ok(test.config.maximum_step_m, test.step_m) &&
        test_line_main_margin_ok(test.config.reverse_tolerance_m,
                                 test.step_m) &&
        test_line_main_margin_ok(test.config.quiet_zone_m, test.step_m) &&
        test_line_main_margin_ok(test.config.maximum_frame_m, test.step_m) &&
        test_line_main_margin_ok(width_m - tolerance_m, test.step_m) &&
        test_line_main_margin_ok(width_m + tolerance_m, test.step_m),
        "production window limits lie >= 1e-6 half-count from integers");
}

/** @brief Re-entry behind the last decode after a REVERSE rejection: the
 * duplicate guard |separation| < 2000 steps holds on the negative side.
 */
static void test_line_main_reentry (void)
{
    test_line_main_fixture_t test;
    const int32_t separations[] = {-1999, -2000, -2001};
    size_t index;
    double decode_m;
    bool   b_duplicate;

    for (index = 0U; index < (sizeof(separations) / sizeof(separations[0]));
         ++index)
    {
        test_line_main_prepare(&test);
        test_line_main_symbol(&test, TEST_LINE_MAIN_WIDE_STEPS);
        decode_m = test.legacy.last_decode_m;
        /* Repeat the symbol (SYMBOL - separation) steps back: its first
         * sample is a REVERSE rejection that keeps the decode history.
         */
        test.position -= TEST_LINE_MAIN_SYMBOL_STEPS - separations[index];
        test_line_main_symbol(&test, TEST_LINE_MAIN_WIDE_STEPS);
        test_line_main_check((1U == test.reasons[BARCODE_REASON_REVERSE]) &&
                             ((test.legacy.run_start_m - decode_m) ==
                              ((double) separations[index] * test.step_m)),
                             "re-entry lands at the intended separation");
        b_duplicate = (-1999 == separations[index]);
        test_line_main_check(b_duplicate ?
            ((1U == test.events[BARCODE_EVENT_DECODED]) &&
             (1U == test.reasons[BARCODE_REASON_DUPLICATE])) :
            ((2U == test.events[BARCODE_EVENT_DECODED]) &&
             (0U == test.reasons[BARCODE_REASON_DUPLICATE])),
            "duplicate behind: |D| < 2000 steps excludes 2000");
    }
}

/** @brief Scale mode edges: a fitted scale exactly on either bound is kept,
 * and a run residual exactly equal to the tolerance still matches.
 */
static void test_line_main_scale_edges (void)
{
    test_line_main_fixture_t test;
    const uint32_t bars[] = {19U, 20U, 80U, 81U};
    const uint32_t runs[2][3] = {{43U, 40U, 77U}, {44U, 40U, 76U}};
    size_t index;
    barcode_event_t expected;

    for (index = 0U; index < (sizeof(bars) / sizeof(bars[0])); ++index)
    {
        test_line_main_prepare(&test);
        test.config.b_scale_tolerant = true;
        test.config.minimum_scale = BARCODE_MIN_SCALE;
        test.config.maximum_scale = BARCODE_MAX_SCALE;
        test.config.symbols[0].run_count = 1U;
        test_line_main_check(test_line_main_derive(&test),
                             "one-run scale fixture derives");
        /* The fitted scale is bar / 40: exactly 0.5 and 2.0 at 20 and 80. */
        expected = ((1U == index) || (2U == index)) ? BARCODE_EVENT_DECODED
                                                    : BARCODE_EVENT_UNKNOWN;
        test_line_main_check(expected ==
                             test_line_main_frame(&test, &bars[index], 1U),
                             "scale exactly on a bound is accepted");
    }
    for (index = 0U; index < 2U; ++index)
    {
        test_line_main_prepare(&test);
        test.config.b_scale_tolerant = true;
        test.config.minimum_scale = BARCODE_MIN_SCALE;
        test.config.maximum_scale = BARCODE_MAX_SCALE;
        test.config.relative_tolerance = 0.0;
        test_line_main_check(test_line_main_derive(&test),
                             "three-run scale fixture derives");
        /* Totals stay 160 steps (scale 1.0); residuals of 3 and 4 steps
         * meet the 3-step absolute tolerance.
         */
        expected = (0U == index) ? BARCODE_EVENT_DECODED
                                 : BARCODE_EVENT_UNKNOWN;
        test_line_main_check(expected ==
                             test_line_main_frame(&test, runs[index], 3U),
                             "scaled residual equal to tolerance matches");
    }
}

/** @brief A sample gap of exactly maximum_gap_ms and a frame age of exactly
 * frame_timeout_ms are kept; one millisecond more times out.
 */
static void test_line_main_timing (void)
{
    test_line_main_fixture_t test;
    uint32_t extra;
    uint32_t start_ms;
    bool     b_kept;

    for (extra = 0U; extra <= 1U; ++extra)
    {
        test_line_main_prepare(&test);
        test_line_main_feed(&test, LINE_CTRL_LIGHT,
                            TEST_LINE_MAIN_WHITE_STEPS);
        test_line_main_feed(&test, LINE_CTRL_DARK, TEST_LINE_MAIN_BLACK_STEPS);
        test.now_ms += test.config.maximum_gap_ms + extra;
        test_line_main_check(test.legacy.b_frame &&
            test_line_main_sample(&test, LINE_CTRL_DARK, test.position) &&
            ((0U == extra) ? (BARCODE_EVENT_NONE == test.output.event)
                           : (BARCODE_REASON_TIMEOUT == test.output.reason)),
            "gap: maximum_gap_ms kept, one ms more times out");
    }
    for (extra = 0U; extra <= 1U; ++extra)
    {
        test_line_main_prepare(&test);
        test.config.frame_timeout_ms = 100U;
        test_line_main_check(test_line_main_derive(&test),
                             "100 ms frame timeout derives");
        test_line_main_feed(&test, LINE_CTRL_LIGHT,
                            TEST_LINE_MAIN_WHITE_STEPS);
        test_line_main_feed(&test, LINE_CTRL_DARK, TEST_LINE_MAIN_BLACK_STEPS);
        start_ms = test.legacy.frame_start_ms;
        /* Stationary samples within the 50 ms gap age the open frame. */
        test.now_ms = start_ms + 50U;
        b_kept = test.legacy.b_frame &&
                 test_line_main_sample(&test, LINE_CTRL_DARK, test.position) &&
                 (BARCODE_EVENT_NONE == test.output.event);
        test.now_ms = start_ms + 99U;
        b_kept = b_kept &&
                 test_line_main_sample(&test, LINE_CTRL_DARK, test.position) &&
                 (BARCODE_EVENT_NONE == test.output.event);
        test.now_ms = start_ms + 100U + extra;
        test_line_main_check(b_kept &&
            test_line_main_sample(&test, LINE_CTRL_DARK, test.position) &&
            ((0U == extra) ? (BARCODE_EVENT_NONE == test.output.event)
                           : (BARCODE_REASON_TIMEOUT == test.output.reason)),
            "frame age: frame_timeout_ms kept, one ms more times out");
    }
}

/* End of file. */

