/**
 * @file test_line_main.c
 * @brief Optical and configurable barcode sequence tests, not track proof.
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

#define TEST_LINE_MAIN_EVENT_COUNT  (5U)
#define TEST_LINE_MAIN_REASON_COUNT (11U)
#define TEST_LINE_MAIN_STEP_M       (0.0001)
#define TEST_LINE_MAIN_WHITE_STEPS  (200U)
#define TEST_LINE_MAIN_BLACK_STEPS  (40U)
#define TEST_LINE_MAIN_WIDE_STEPS   (80U)
#define TEST_LINE_MAIN_LONG_STEPS   (120U)
#define TEST_LINE_MAIN_NOISE_STEPS  (2U)
#define TEST_LINE_MAIN_RUN_PAIRS    (18U)
#define TEST_LINE_MAIN_PAUSE_STEPS  (8U)
#define TEST_LINE_MAIN_SAMPLE_RUNS  (29U)
#define TEST_LINE_MAIN_SAMPLE_WHITE (1288U)
#define TEST_LINE_MAIN_SAMPLE_STEP  (1.0 / 65536.0)
#define TEST_LINE_MAIN_SAMPLE_UNIT  (100.0 * TEST_LINE_MAIN_SAMPLE_STEP)

typedef struct
{
    barcode_config_t config;
    barcode_state_t  state;
    barcode_output_t output;
    double           position_m;
    double           step_m;
    uint32_t         now_ms;
    uint32_t         events[TEST_LINE_MAIN_EVENT_COUNT];
    uint32_t         reasons[TEST_LINE_MAIN_REASON_COUNT];
    barcode_action_t last_action;
    char             last_label;
    bool             b_jitter;
} test_line_main_fixture_t;

static uint32_t g_checks;
static uint32_t g_failures;

static void test_line_main_check (bool b_condition, const char * p_label);
static void test_line_main_line (void);
static void test_line_main_line_faults (void);
static void test_line_main_prepare (test_line_main_fixture_t * p_test);
static void test_line_main_feed (test_line_main_fixture_t * p_test,
                                 double darkness, uint32_t steps);
static void test_line_main_symbol (test_line_main_fixture_t * p_test,
                                   uint32_t final_steps);
static void test_line_main_gate (void);
static void test_line_main_decode (void);
static void test_line_main_no_match (void);
static void test_line_main_faults (void);
static void test_line_main_overflow (void);
static void test_line_main_duplicate (void);
static void test_line_main_limits (void);
static void test_line_main_sample_prepare (test_line_main_fixture_t * p_test);
static void test_line_main_sample_frame (test_line_main_fixture_t * p_test,
                                        const barcode_symbol_t * p_symbol,
                                        double scale, bool b_reverse,
                                        bool b_noise);
static void test_line_main_scale_decode (void);
static void test_line_main_scale_unknown (void);
static void test_line_main_scale_physical (void);
static void test_line_main_scale_bounds (void);

/** @brief Run deterministic fixtures; nonzero exit reports any failure. */
int main (void)
{
    test_line_main_line();
    test_line_main_line_faults();
    test_line_main_gate();
    test_line_main_decode();
    test_line_main_no_match();
    test_line_main_faults();
    test_line_main_overflow();
    test_line_main_duplicate();
    test_line_main_limits();
    test_line_main_scale_decode();
    test_line_main_scale_unknown();
    test_line_main_scale_physical();
    test_line_main_scale_bounds();
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

/** @brief Check normalization, polarity, hysteresis and temporal evidence. */
static void test_line_main_line (void)
{
    line_ctrl_config_t config;
    line_ctrl_state_t  state;
    line_ctrl_output_t output;
    uint16_t raw[LINE_CTRL_CHANNELS] = {2048U, 2048U, 0U};

    line_ctrl_defaults(&config);
    line_ctrl_init(&state);
    test_line_main_check(line_ctrl_config_valid(&config), "line defaults");
    /* test_motion_main.c derives the NI-MOT-1 detection travel from it. */
    test_line_main_check(20U == config.junction_debounce_ms,
                         "20 ms production junction debounce");
    test_line_main_check(line_ctrl_step(&config, &state, raw, 0U, &output),
                         "unconfirmed line is valid but inhibited");
    test_line_main_check(LINE_CTRL_INHIBITED == output.status, "line gate");
    config.b_calibration_confirmed = true;
    test_line_main_check(line_ctrl_step(&config, &state, raw, 0U, &output),
                         "balanced line sample");
    test_line_main_check((fabs(output.steering) < 0.001) &&
                         (LINE_CTRL_TRACKING == output.status),
                         "balanced edges follow");
    raw[0] = 4095U;
    raw[1] = 0U;
    test_line_main_check(line_ctrl_step(&config, &state, raw, 5U, &output),
                         "right line sample");
    test_line_main_check(output.steering > 0.99, "rightward correction");
    raw[0] = 0U;
    test_line_main_check(line_ctrl_step(&config, &state, raw, 10U, &output),
                         "junction starts");
    test_line_main_check(LINE_CTRL_SEARCHING == output.status,
                         "both black initially ambiguous");
    test_line_main_check(line_ctrl_step(&config, &state, raw, 29U, &output),
                         "junction almost debounced");
    test_line_main_check(!output.b_junction_edge, "no early junction");
    test_line_main_check(line_ctrl_step(&config, &state, raw, 30U, &output),
                         "junction debounce boundary");
    test_line_main_check(output.b_junction_edge &&
                         (LINE_CTRL_JUNCTION == output.status),
                         "single junction event");
    test_line_main_check(line_ctrl_step(&config, &state, raw, 40U, &output),
                         "junction remains");
    test_line_main_check(!output.b_junction_edge, "junction not repeated");
    raw[0] = 4095U;
    raw[1] = 4095U;
    test_line_main_check(line_ctrl_step(&config, &state, raw, 50U, &output),
                         "first all white");
    test_line_main_check(LINE_CTRL_SEARCHING == output.status,
                         "line loss starts uncertain");
    test_line_main_check(line_ctrl_step(&config, &state, raw, 90U, &output),
                         "line loss maintained");
    test_line_main_check(line_ctrl_step(&config, &state, raw, 130U, &output),
                         "line loss still before timeout");
    test_line_main_check(LINE_CTRL_SEARCHING == output.status,
                         "no premature line loss");
    test_line_main_check(line_ctrl_step(&config, &state, raw, 150U, &output),
                         "line loss timeout sample");
    test_line_main_check((LINE_CTRL_LOST == output.status) &&
                         (fabs(output.steering) < 0.001),
                         "lost has zero demand");
}

/** @brief Check polarity inversion, ADC limits, wrap and stale rejection. */
static void test_line_main_line_faults (void)
{
    line_ctrl_config_t config;
    line_ctrl_state_t  state;
    line_ctrl_output_t output;
    uint16_t raw[LINE_CTRL_CHANNELS] = {4095U, 4095U, 4095U};

    line_ctrl_defaults(&config);
    line_ctrl_init(&state);
    config.b_calibration_confirmed = true;
    config.white_adc[2] = 0U;
    config.black_adc[2] = 4095U;
    test_line_main_check(line_ctrl_step(&config, &state, raw,
                         UINT32_MAX - 5U, &output), "line near timer wrap");
    test_line_main_check(output.b_barcode_black, "channel polarity varies");
    raw[2] = 2600U;
    test_line_main_check(line_ctrl_step(&config, &state, raw, 10U, &output),
                         "timer wrap accepted");
    test_line_main_check(output.b_barcode_black, "hysteresis stays black");
    raw[2] = 2000U;
    test_line_main_check(line_ctrl_step(&config, &state, raw, 12U, &output),
                         "hysteresis low sample");
    test_line_main_check(!output.b_barcode_black, "hysteresis turns white");
    test_line_main_check(!line_ctrl_step(&config, &state, raw, 100U, &output),
                         "stale sample rejected");
    test_line_main_check(LINE_CTRL_FAULT == output.status, "stale fault");
    raw[0] = UINT16_MAX;
    test_line_main_check(!line_ctrl_step(&config, &state, raw, 101U, &output),
                         "ADC out of configured range");
    config.black_on = NAN;
    test_line_main_check(!line_ctrl_config_valid(&config), "NaN threshold");
    line_ctrl_defaults(&config);
    config.black_adc[1] = config.white_adc[1];
    test_line_main_check(!line_ctrl_config_valid(&config), "zero contrast");
    test_line_main_check(!line_ctrl_step(NULL, &state, raw, 0U, &output),
                         "NULL line configuration");
    test_line_main_check(!line_ctrl_step(&config, NULL, raw, 0U, &output),
                         "NULL line state");
}

/** @brief Configure a synthetic three-run fixture, not a course symbol. */
static void test_line_main_prepare (test_line_main_fixture_t * p_test)
{
    memset(p_test, 0, sizeof(*p_test));
    p_test->step_m = TEST_LINE_MAIN_STEP_M;
    barcode_defaults(&p_test->config);
    barcode_init(&p_test->state);
    p_test->config.b_calibration_confirmed = true;
    p_test->config.b_codebook_confirmed    = true;
    p_test->config.symbol_count           = 1U;
    p_test->config.symbols[0].run_count    = 3U;
    p_test->config.symbols[0].width_m[0]   = 0.004;
    p_test->config.symbols[0].width_m[1]   = 0.004;
    p_test->config.symbols[0].width_m[2]   = 0.008;
    p_test->config.symbols[0].label        = 'T';
    p_test->config.symbols[0].action       = BARCODE_ACTION_LEFT;
}

/** @brief Feed physical distance samples and retain non-idle event totals. */
static void test_line_main_feed (test_line_main_fixture_t * p_test,
                                 double darkness, uint32_t steps)
{
    uint32_t sample;
    double   position;
    bool     b_result;

    for (sample = 0U; sample < steps; ++sample)
    {
        p_test->position_m += p_test->step_m;
        ++p_test->now_ms;
        position = p_test->position_m;
        if (p_test->b_jitter && (0U == (sample % 7U)))
        {
            position -= 0.00015;
        }
        b_result = barcode_step(&p_test->config, &p_test->state, darkness,
                                 position, p_test->now_ms, &p_test->output);
        test_line_main_check(b_result, "barcode sample structurally valid");
        if (BARCODE_EVENT_NONE != p_test->output.event)
        {
            ++p_test->events[p_test->output.event];
            ++p_test->reasons[p_test->output.reason];
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
            p_test->last_label  = p_test->output.label;
        }
    }
}

/** @brief Generate one synthetic framed pattern with a selectable last run. */
static void test_line_main_symbol (test_line_main_fixture_t * p_test,
                                   uint32_t final_steps)
{
    test_line_main_feed(p_test, 0.0, TEST_LINE_MAIN_WHITE_STEPS);
    test_line_main_feed(p_test, 1.0, TEST_LINE_MAIN_BLACK_STEPS);
    test_line_main_feed(p_test, 0.0, TEST_LINE_MAIN_BLACK_STEPS);
    test_line_main_feed(p_test, 1.0, final_steps);
    test_line_main_feed(p_test, 0.0, TEST_LINE_MAIN_WHITE_STEPS);
}

/** @brief Empty, unconfirmed and malformed configurations cannot decode. */
static void test_line_main_gate (void)
{
    test_line_main_fixture_t test;

    test_line_main_prepare(&test);
    barcode_defaults(&test.config);
    test_line_main_check(barcode_config_valid(&test.config),
                         "barcode defaults structurally valid");
    test_line_main_check(barcode_step(&test.config, &test.state, 0.0,
                         0.0, 0U, &test.output), "empty codebook call");
    test_line_main_check(BARCODE_EVENT_INHIBITED == test.output.event,
                         "empty codebook inhibited");
    test_line_main_prepare(&test);
    test.config.b_codebook_confirmed = false;
    test_line_main_check(barcode_step(&test.config, &test.state, 0.0,
                         0.0, 0U, &test.output), "unconfirmed codebook call");
    test_line_main_check(BARCODE_EVENT_INHIBITED == test.output.event,
                         "unconfirmed codebook inhibited");
    test_line_main_prepare(&test);
    test.config.b_calibration_confirmed = false;
    test_line_main_check(barcode_step(&test.config, &test.state, 0.0,
                         0.0, 0U, &test.output), "uncalibrated barcode call");
    test_line_main_check(BARCODE_EVENT_INHIBITED == test.output.event,
                         "uncalibrated barcode inhibited");
    test.config.symbols[0].run_count = 2U;
    test_line_main_check(!barcode_config_valid(&test.config),
                         "even run count is invalid framing");
    test_line_main_prepare(&test);
    test.config.symbols[0].width_m[1] = test.config.quiet_zone_m;
    test_line_main_check(!barcode_config_valid(&test.config),
                         "internal white must be below quiet zone");
    test_line_main_prepare(&test);
    test.config.minimum_run_m = 0.001;
    test.config.maximum_step_m = 0.0002;
    test_line_main_check(barcode_config_valid(&test.config),
                         "five samples across minimum run accepted");
    test.config.maximum_step_m = 0.00025;
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
        test_line_main_check('T' == test.last_label, "configured label");
        test_line_main_check(0U == test.events[BARCODE_EVENT_REJECTED],
                             "clean frame has no rejection");
    }
    test_line_main_prepare(&test);
    test.b_jitter = true;
    test.config.minimum_run_m = 0.00125;
    test.config.maximum_step_m = 0.00025;
    test_line_main_symbol(&test, TEST_LINE_MAIN_WIDE_STEPS);
    test_line_main_check(1U == test.events[BARCODE_EVENT_DECODED],
                         "sub-tolerance odometry jitter");
    test_line_main_prepare(&test);
    test_line_main_feed(&test, 0.0, TEST_LINE_MAIN_WHITE_STEPS);
    test_line_main_feed(&test, 1.0, TEST_LINE_MAIN_NOISE_STEPS);
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
    test_line_main_symbol(&test, TEST_LINE_MAIN_WIDE_STEPS);
    test_line_main_check(1U == test.reasons[BARCODE_REASON_AMBIGUOUS],
                         "ambiguous patterns rejected");
    test_line_main_check(0U == test.events[BARCODE_EVENT_DECODED],
                         "ambiguous pattern cannot direct motion");
    test_line_main_prepare(&test);
    test_line_main_feed(&test, 1.0, TEST_LINE_MAIN_BLACK_STEPS);
    test_line_main_feed(&test, 0.0, TEST_LINE_MAIN_BLACK_STEPS);
    test_line_main_feed(&test, 1.0, TEST_LINE_MAIN_WIDE_STEPS);
    test_line_main_feed(&test, 0.0, TEST_LINE_MAIN_WHITE_STEPS);
    test_line_main_check(0U == test.events[BARCODE_EVENT_DECODED],
                         "missing leading quiet zone ignored");
}

/** @brief Reverse movement, stale data and invalid samples clear frames. */
static void test_line_main_faults (void)
{
    test_line_main_fixture_t test;
    uint32_t pause;

    test_line_main_prepare(&test);
    test_line_main_feed(&test, 0.0, TEST_LINE_MAIN_WHITE_STEPS);
    test_line_main_feed(&test, 1.0, TEST_LINE_MAIN_BLACK_STEPS);
    test_line_main_check(barcode_step(&test.config, &test.state, 1.0,
                         test.position_m - 0.002, ++test.now_ms,
                         &test.output), "reverse sample handled");
    test_line_main_check(BARCODE_REASON_REVERSE == test.output.reason,
                         "reverse rejects frame");
    test_line_main_check(!test.state.b_frame, "reverse clears collection");
    test_line_main_prepare(&test);
    test_line_main_feed(&test, 0.0, TEST_LINE_MAIN_WHITE_STEPS);
    test_line_main_feed(&test, 1.0, TEST_LINE_MAIN_BLACK_STEPS);
    test.now_ms += test.config.maximum_gap_ms + 1U;
    test_line_main_check(barcode_step(&test.config, &test.state, 1.0,
                         test.position_m, test.now_ms, &test.output),
                         "sample gap handled");
    test_line_main_check(BARCODE_REASON_TIMEOUT == test.output.reason,
                         "sample gap rejects frame");
    test_line_main_prepare(&test);
    test.config.frame_timeout_ms = 40U;
    test_line_main_feed(&test, 0.0, TEST_LINE_MAIN_WHITE_STEPS);
    test_line_main_feed(&test, 1.0, TEST_LINE_MAIN_BLACK_STEPS);
    for (pause = 0U; pause < TEST_LINE_MAIN_PAUSE_STEPS; ++pause)
    {
        test.now_ms += 10U;
        test_line_main_check(barcode_step(&test.config, &test.state, 1.0,
                             test.position_m, test.now_ms, &test.output),
                             "stationary frame sample");
        if (BARCODE_REASON_TIMEOUT == test.output.reason)
        {
            ++test.reasons[BARCODE_REASON_TIMEOUT];
        }
    }
    test_line_main_check(1U == test.reasons[BARCODE_REASON_TIMEOUT],
                         "stationary frame times out exactly once");
    test_line_main_check(!barcode_step(&test.config, &test.state, NAN,
                         0.0, 0U, &test.output), "NaN darkness rejected");
    test_line_main_check(!barcode_step(&test.config, &test.state, 0.0,
                         INFINITY, 0U, &test.output),
                         "infinite encoder distance rejected");
    test_line_main_check(!barcode_step(NULL, &test.state, 0.0,
                         0.0, 0U, &test.output), "NULL config rejected");
    test_line_main_check(BARCODE_ACTION_UNKNOWN == test.output.action,
                         "invalid input has no navigation action");
}

/** @brief Excess run count cannot write beyond fixed storage. */
static void test_line_main_overflow (void)
{
    test_line_main_fixture_t test;
    uint32_t pair;

    test_line_main_prepare(&test);
    test_line_main_feed(&test, 0.0, TEST_LINE_MAIN_WHITE_STEPS);
    for (pair = 0U; pair < TEST_LINE_MAIN_RUN_PAIRS; ++pair)
    {
        test_line_main_feed(&test, 1.0, TEST_LINE_MAIN_BLACK_STEPS);
        test_line_main_feed(&test, 0.0, TEST_LINE_MAIN_BLACK_STEPS);
    }
    test_line_main_feed(&test, 0.0, TEST_LINE_MAIN_WHITE_STEPS);
    test_line_main_check(1U == test.reasons[BARCODE_REASON_OVERFLOW],
                         "33rd run produces bounded overflow rejection");
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
    test.config.duplicate_distance_m = 0.01;
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
        test.position_m = -step_m;
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
        test_line_main_check(barcode_config_valid(&test.config),
                             "exact-boundary codebook structurally valid");
        test_line_main_feed(&test, 0.0, (2U == scenario) ? 128U
                                       : ((5U == scenario) ? 127U : 136U));
        test_line_main_feed(&test, 1.0, 64U);
        test_line_main_feed(&test, 0.0, 32U);
        test_line_main_feed(&test, 1.0,
                            ((3U == scenario) || (4U == scenario)) ? 65U : 64U);
        test_line_main_feed(&test, 0.0, 136U);
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
    p_test->position_m = -p_test->step_m;
    p_test->config.minimum_run_m = 20.0 * p_test->step_m;
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
    test_line_main_check(barcode_config_valid(&p_test->config),
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
    double darkness;

    test_line_main_feed(p_test, 0.0, TEST_LINE_MAIN_SAMPLE_WHITE);
    if (b_noise)
    {
        test_line_main_feed(p_test, 1.0, TEST_LINE_MAIN_NOISE_STEPS);
        test_line_main_feed(p_test, 0.0, TEST_LINE_MAIN_SAMPLE_WHITE);
    }
    for (run = 0U; run < p_symbol->run_count; ++run)
    {
        index = b_reverse ? ((uint32_t)p_symbol->run_count - 1U - run) : run;
        steps = (uint32_t)lround(p_symbol->width_m[index] * scale /
                                p_test->step_m);
        if (b_noise)
        {
            steps = (0U == (run % 2U)) ? (steps + 1U) : (steps - 1U);
        }
        darkness = (0U == (run % 2U)) ? 1.0 : 0.0;
        test_line_main_feed(p_test, darkness, steps);
    }
    test_line_main_feed(p_test, 0.0, TEST_LINE_MAIN_SAMPLE_WHITE);
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
            test_line_main_check(test.config.symbols[symbol].label ==
                                 test.last_label,
                                 "source vector label preserved");
            test_line_main_check(test.config.symbols[symbol].action ==
                                 test.last_action,
                                 "synthetic fixture action preserved");
            test_line_main_check(0U == test.events[BARCODE_EVENT_REJECTED],
                                 "scaled frame meets physical limits");
            test_line_main_sample_prepare(&test);
            test.config.b_scale_tolerant = false;
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
    test_line_main_sample_frame(&test, &test.config.symbols[0],
                                 0.1, false, false);
    test_line_main_check(test.reasons[BARCODE_REASON_WIDTH] > 0U,
                         "undersized physical runs rejected before matching");
    test_line_main_check(0U == test.events[BARCODE_EVENT_DECODED],
                         "uniform scale cannot rescue undersized runs");
    test_line_main_sample_prepare(&test);
    test_line_main_feed(&test, 0.0, TEST_LINE_MAIN_SAMPLE_WHITE);
    test_line_main_feed(&test, 1.0, 20U);
    test_line_main_check(barcode_step(&test.config, &test.state, 1.0,
                         test.position_m + (5.0 * test.step_m),
                         ++test.now_ms, &test.output),
                         "excessive spatial sample gap handled");
    test_line_main_check(BARCODE_REASON_DISTANCE == test.output.reason,
                         "scale matcher preserves spatial sampling gate");
    test_line_main_check(BARCODE_ACTION_UNKNOWN == test.output.action,
                         "sampling rejection carries no action");
    test_line_main_sample_prepare(&test);
    test_line_main_feed(&test, 0.0, TEST_LINE_MAIN_SAMPLE_WHITE);
    test_line_main_feed(&test, 1.0, 20U);
    test_line_main_check(barcode_step(&test.config, &test.state, 1.0,
                         test.position_m - (2.0 * test.step_m),
                         ++test.now_ms, &test.output),
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
    test_line_main_check(barcode_config_valid(&test.config),
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

/* End of file. */

