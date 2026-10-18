/**
 * @file test_telemetry_main.c
 * @brief Exact-schema, integer-extreme and capacity boundary tests.
 * NOTE: main preserves the required hosted-C entry name (BARR-C 6.1.i).
 */
#include "test_telemetry_main.h"
#include "telemetry.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t checks;
static uint32_t failures;

static const telemetry_perception_t perception_max =
{
    .seq = UINT32_MAX, .now_ms = UINT32_MAX, .run_id = UINT32_MAX,
    .cal_version = UINT32_MAX, .barcode_action = INT32_MIN,
    .barcode_label = INT32_MIN, .yaw_mrad_s = INT32_MIN,
    .motion_class = INT32_MIN, .scan_width_mm = INT32_MIN,
    .scan_nearest_mm = INT32_MIN, .scan_side = INT32_MIN,
    .scan_valid = UINT32_MAX, .scan_unknown = UINT32_MAX,
    .b_tilt_valid = false, .b_hump_active = false, .b_scan_valid = false
};


static void test_telemetry_check (bool b_result, const char * p_label);
static void test_telemetry_fixture (void);
static void test_telemetry_extremes (void);
static void test_telemetry_bounds (void);
static void test_perception_fixture (void);
static void test_perception_extremes (void);
static void test_perception_bounds (void);
static void test_diagnostic_fixture (void);
static void test_diagnostic_extremes (void);
static void test_diagnostic_bounds (void);

/** @brief Return a failing exit status for any violated formatter contract. */
int main (void)
{
    int result;

    test_telemetry_fixture();
    test_telemetry_extremes();
    test_telemetry_bounds();
    test_perception_fixture();
    test_perception_extremes();
    test_perception_bounds();
    test_diagnostic_fixture();
    test_diagnostic_extremes();
    test_diagnostic_bounds();
    result = printf("telemetry: %u checks, %u failures\n", checks, failures);
    if ((result < 0) || (failures > 0U))
    {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

/** @brief Record assertions while keeping every scenario runnable. */
static void test_telemetry_check (bool b_result, const char * p_label)
{
    checks++;
    if (!b_result)
    {
        int result;

        failures++;
        result = fprintf(stderr, "FAIL: %s\n", p_label);
        /* A diagnostic write failure does not erase the failed assertion. */
        (void)result;
    }
}

/** @brief Compare all fields and mixed Boolean values with exact JSON. */
static void test_telemetry_fixture (void)
{
    const telemetry_status_t status =
    {
        .seq = 12U, .now_ms = 3456U, .dropped = 7U, .ignored = 8U,
        .range_age_ms = 9U, .state = 2, .vleft_mm_s = -123,
        .vright_mm_s = 456, .pitch_mrad = -7, .peak_mm = 8, .range_mm = 90,
        .b_stop = true, .b_hardfault = false,
        .b_heightvalid = true, .b_rangevalid = false
    };
    const char expected[] =
        "{\"seq\":12,\"ms\":3456,\"drop\":7,\"ign\":8,\"age\":9,"
        "\"st\":2,\"vl\":-123,\"vr\":456,\"pt\":-7,\"pk\":8,\"rg\":90,"
        "\"stop\":true,\"fault\":false,\"hv\":true,\"rv\":false}";
    char text[TELEMETRY_BUFFER_SIZE];
    size_t length = telemetry_format(&status, text, sizeof(text));

    test_telemetry_check(length == (sizeof(expected) - 1U),
                          "fixture byte length excludes NUL");
    test_telemetry_check(0 == strcmp(text, expected),
                          "all keys, units, numbers and Booleans match");
    test_telemetry_check('\0' == text[length], "success terminates text");
}

/** @brief All longest values fit, including INT32_MIN without overflow. */
static void test_telemetry_extremes (void)
{
    telemetry_status_t status =
    {
        .seq = UINT32_MAX, .now_ms = UINT32_MAX, .dropped = UINT32_MAX,
        .ignored = UINT32_MAX, .range_age_ms = UINT32_MAX,
        .state = INT32_MIN, .vleft_mm_s = INT32_MIN,
        .vright_mm_s = INT32_MIN, .pitch_mrad = INT32_MIN,
        .peak_mm = INT32_MIN, .range_mm = INT32_MIN,
        .b_stop = false, .b_hardfault = false,
        .b_heightvalid = false, .b_rangevalid = false
    };
    const char expected[] =
        "{\"seq\":4294967295,\"ms\":4294967295,\"drop\":4294967295,"
        "\"ign\":4294967295,\"age\":4294967295,\"st\":-2147483648,"
        "\"vl\":-2147483648,\"vr\":-2147483648,\"pt\":-2147483648,"
        "\"pk\":-2147483648,\"rg\":-2147483648,\"stop\":false,"
        "\"fault\":false,\"hv\":false,\"rv\":false}";
    char text[TELEMETRY_BUFFER_SIZE];
    size_t length = telemetry_format(&status, text, sizeof(text));

    test_telemetry_check(length > 0U, "all worst-width fields fit");
    test_telemetry_check(length < TELEMETRY_BUFFER_SIZE,
                          "worst schema leaves room for NUL");
    test_telemetry_check(length == (sizeof(expected) - 1U),
                          "worst schema exact length");
    test_telemetry_check(0 == strcmp(text, expected),
                          "UINT32_MAX and INT32_MIN exact decimal strings");
    status.state = INT32_MAX;
    status.vleft_mm_s = -1;
    status.vright_mm_s = 0;
    status.pitch_mrad = 1;
    status.peak_mm = 10;
    status.range_mm = 100;
    length = telemetry_format(&status, text, sizeof(text));
    test_telemetry_check(length > 0U, "mixed sign and decimal boundaries fit");
    test_telemetry_check(NULL != strstr(text,
        "\"st\":2147483647,\"vl\":-1,\"vr\":0,\"pt\":1,\"pk\":10,\"rg\":100"),
        "signed maximum, negative one, zero and powers of ten");
}

/** @brief Every undersized capacity clears output and preserves canaries. */
static void test_telemetry_bounds (void)
{
    const telemetry_status_t status = {0};
    char reference[TELEMETRY_BUFFER_SIZE];
    char text[TELEMETRY_BUFFER_SIZE];
    size_t capacity;
    size_t required = telemetry_format(&status, reference, sizeof(reference));
    size_t length;

    test_telemetry_check(required > 0U, "zero-valued snapshot is valid");
    for (capacity = 1U; capacity <= required; capacity++)
    {
        memset(text, 'X', sizeof(text));
        length = telemetry_format(&status, text, capacity);
        test_telemetry_check(0U == length, "short capacity reports failure");
        test_telemetry_check('\0' == text[0], "failure clears first byte");
        test_telemetry_check('X' == text[capacity], "no write past capacity");
    }
    memset(text, 'X', sizeof(text));
    length = telemetry_format(&status, text, required + 1U);
    test_telemetry_check(length == required, "exact capacity succeeds");
    test_telemetry_check(0 == strcmp(text, reference), "exact capacity JSON");
    test_telemetry_check('X' == text[required + 1U], "exact-size canary");
    text[0] = 'X';
    test_telemetry_check(0U == telemetry_format(&status, text, 0U),
                          "zero capacity rejected");
    test_telemetry_check('X' == text[0], "zero capacity performs no write");
    test_telemetry_check(0U == telemetry_format(&status, NULL, sizeof(text)),
                          "NULL destination rejected");
    test_telemetry_check(0U == telemetry_format(NULL, text, sizeof(text)),
                          "NULL snapshot rejected");
    test_telemetry_check('\0' == text[0], "NULL snapshot clears destination");
}

/** @brief Verify exact perception keys, field order and physical units. */
static void test_perception_fixture (void)
{
    const telemetry_perception_t status =
    {
        .seq = 5U, .now_ms = 1000U, .run_id = 2U, .cal_version = 3U,
        .barcode_action = 1, .barcode_label = 65, .yaw_mrad_s = -123,
        .motion_class = 4, .scan_width_mm = 56, .scan_nearest_mm = 78,
        .scan_side = -1, .scan_valid = 9U, .scan_unknown = 10U,
        .b_tilt_valid = true, .b_hump_active = false, .b_scan_valid = true
    };
    const char expected[] =
        "{\"seq\":5,\"ms\":1000,\"run\":2,\"cal\":3,\"ba\":1,\"bl\":65,"
        "\"yr\":-123,\"mc\":4,\"sw\":56,\"sn\":78,\"ss\":-1,\"sv\":9,"
        "\"su\":10,\"tv\":true,\"ha\":false,\"sc\":true}";
    char text[TELEMETRY_BUFFER_SIZE];
    size_t length = telemetry_format_perception(&status, text, sizeof(text));

    test_telemetry_check(length == (sizeof(expected) - 1U),
                          "perception fixture excludes final NUL");
    test_telemetry_check(0 == strcmp(text, expected),
                          "perception fields and native Booleans match");
    test_telemetry_check('\0' == text[length], "perception has final NUL");
}

/** @brief Every longest representation fits within the 256-byte limit. */
static void test_perception_extremes (void)
{
    const char expected[] =
        "{\"seq\":4294967295,\"ms\":4294967295,\"run\":4294967295,"
        "\"cal\":4294967295,\"ba\":-2147483648,\"bl\":-2147483648,"
        "\"yr\":-2147483648,\"mc\":-2147483648,\"sw\":-2147483648,"
        "\"sn\":-2147483648,\"ss\":-2147483648,\"sv\":4294967295,"
        "\"su\":4294967295,\"tv\":false,\"ha\":false,\"sc\":false}";
    char text[TELEMETRY_BUFFER_SIZE];
    size_t length;

    length = telemetry_format_perception(&perception_max, text, sizeof(text));
    test_telemetry_check(252U == length, "perception longest frame is252");
    test_telemetry_check(length < TELEMETRY_BUFFER_SIZE,
                          "perception longest frame has NUL capacity");
    test_telemetry_check(length == (sizeof(expected) - 1U),
                          "perception longest exact fixture size");
    test_telemetry_check(0 == strcmp(text, expected),
                          "all perception UINT32_MAX/INT32_MIN fields exact");
}

/** @brief Every insufficient size is rejected, without crossing its bound. */
static void test_perception_bounds (void)
{
    char reference[TELEMETRY_BUFFER_SIZE];
    char text[TELEMETRY_BUFFER_SIZE];
    size_t capacity;
    size_t required;
    size_t length;

    required = telemetry_format_perception(&perception_max, reference,
                                             sizeof(reference));
    for (capacity = 1U; capacity <= required; capacity++)
    {
        memset(text, 'X', sizeof(text));
        length = telemetry_format_perception(&perception_max, text, capacity);
        test_telemetry_check(0U == length, "short perception size fails");
        test_telemetry_check('\0' == text[0], "failed perception is empty");
        test_telemetry_check('X' == text[capacity], "perception bound canary");
    }
    memset(text, 'X', sizeof(text));
    length = telemetry_format_perception(&perception_max, text, required + 1U);
    test_telemetry_check(length == required, "perception exact capacity");
    test_telemetry_check(0 == strcmp(text, reference), "perception exact JSON");
    test_telemetry_check('X' == text[required + 1U], "perception exact canary");
    text[0] = 'X';
    length = telemetry_format_perception(&perception_max, text, 0U);
    test_telemetry_check(0U == length, "zero perception capacity fails");
    test_telemetry_check('X' == text[0], "zero perception capacity no write");
    length = telemetry_format_perception(&perception_max, NULL, sizeof(text));
    test_telemetry_check(0U == length, "NULL perception output fails");
    length = telemetry_format_perception(NULL, text, sizeof(text));
    test_telemetry_check(0U == length, "NULL perception snapshot fails");
    test_telemetry_check('\0' == text[0], "NULL perception empties text");
}

/** @brief Diagnostic keys retain event time separately from frame time. */
static void test_diagnostic_fixture (void)
{
    telemetry_diagnostic_t status =
    {
        .seq = 6U, .now_ms = 1234U, .run_id = 2U,
        .fault_code = 5, .first_fault_ms = 900U,
        .barcode_unknown = 2U, .barcode_rejected = 3U,
        .mission_peak_mm = 19, .b_mission_valid = true
    };
    const char expected[] =
        "{\"seq\":6,\"ms\":1234,\"run\":2,\"fc\":5,\"fm\":900,"
        "\"bu\":2,\"br\":3,\"mp\":19,\"mv\":true}";
    char text[TELEMETRY_BUFFER_SIZE];
    size_t length = telemetry_format_diagnostic(&status, text, sizeof(text));

    test_telemetry_check(length == (sizeof(expected) - 1U),
                          "diagnostic fixture length excludes NUL");
    test_telemetry_check(0 == strcmp(text, expected),
                          "diagnostic frame and first-fault times differ");
    test_telemetry_check('\0' == text[length], "diagnostic terminates text");
    status.fault_code = 0;
    status.first_fault_ms = 0U;
    length = telemetry_format_diagnostic(&status, text, sizeof(text));
    test_telemetry_check(length > 0U, "cleared diagnostic formats");
    test_telemetry_check(NULL != strstr(text, "\"fc\":0,\"fm\":0,"),
                          "cleared diagnostic latch is explicit");
}

/** @brief Longest numeric values fit with event time at unsigned maximum. */
static void test_diagnostic_extremes (void)
{
    telemetry_diagnostic_t status =
    {
        .seq = UINT32_MAX, .now_ms = UINT32_MAX, .run_id = UINT32_MAX,
        .fault_code = INT32_MIN, .first_fault_ms = UINT32_MAX,
        .barcode_unknown = UINT32_MAX, .barcode_rejected = UINT32_MAX,
        .mission_peak_mm = INT32_MIN, .b_mission_valid = false
    };
    const char expected[] =
        "{\"seq\":4294967295,\"ms\":4294967295,\"run\":4294967295,"
        "\"fc\":-2147483648,\"fm\":4294967295,"
        "\"bu\":4294967295,\"br\":4294967295,\"mp\":-2147483648,\"mv\":false}";
    char text[TELEMETRY_BUFFER_SIZE];
    size_t length = telemetry_format_diagnostic(&status, text, sizeof(text));

    test_telemetry_check(144U == length, "diagnostic longest frame is 144");
    test_telemetry_check(length == (sizeof(expected) - 1U),
                          "diagnostic longest exact fixture size");
    test_telemetry_check(0 == strcmp(text, expected),
                          "diagnostic signed minimum and unsigned maximum");
    status.fault_code = INT32_MAX;
    length = telemetry_format_diagnostic(&status, text, sizeof(text));
    test_telemetry_check(length > 0U, "diagnostic signed maximum formats");
    test_telemetry_check(NULL != strstr(text, "\"fc\":2147483647,"),
                          "diagnostic signed maximum remains exact");
}

/** @brief Insufficient diagnostic buffers fail and preserve each canary. */
static void test_diagnostic_bounds (void)
{
    const telemetry_diagnostic_t status =
    {
        .seq = UINT32_MAX, .now_ms = UINT32_MAX, .run_id = UINT32_MAX,
        .fault_code = INT32_MIN, .first_fault_ms = UINT32_MAX,
        .barcode_unknown = UINT32_MAX, .barcode_rejected = UINT32_MAX,
        .mission_peak_mm = INT32_MIN, .b_mission_valid = false
    };
    char reference[TELEMETRY_BUFFER_SIZE];
    char text[TELEMETRY_BUFFER_SIZE];
    size_t required = telemetry_format_diagnostic(&status, reference,
                                                  sizeof(reference));
    size_t capacity;
    size_t length;

    test_telemetry_check(144U == required, "diagnostic bound fixture valid");
    for (capacity = 1U; capacity <= required; capacity++)
    {
        memset(text, 'X', sizeof(text));
        length = telemetry_format_diagnostic(&status, text, capacity);
        test_telemetry_check(0U == length, "short diagnostic size fails");
        test_telemetry_check('\0' == text[0], "failed diagnostic is empty");
        test_telemetry_check('X' == text[capacity], "diagnostic bound canary");
    }
    memset(text, 'X', sizeof(text));
    length = telemetry_format_diagnostic(&status, text, required + 1U);
    test_telemetry_check(length == required, "diagnostic exact capacity");
    test_telemetry_check(0 == strcmp(text, reference), "diagnostic exact JSON");
    test_telemetry_check('X' == text[required + 1U], "diagnostic exact canary");
    text[0] = 'X';
    length = telemetry_format_diagnostic(&status, text, 0U);
    test_telemetry_check(0U == length, "zero diagnostic capacity fails");
    test_telemetry_check('X' == text[0], "zero diagnostic capacity no write");
    length = telemetry_format_diagnostic(&status, NULL, sizeof(text));
    test_telemetry_check(0U == length, "NULL diagnostic output fails");
    length = telemetry_format_diagnostic(NULL, text, sizeof(text));
    test_telemetry_check(0U == length, "NULL diagnostic snapshot fails");
    test_telemetry_check('\0' == text[0], "NULL diagnostic empties text");
}

/* End of file. */

