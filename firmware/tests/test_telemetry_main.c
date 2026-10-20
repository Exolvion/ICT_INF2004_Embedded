/**
 * @file test_telemetry_main.c
 * @brief Golden bytes shared with scripts/telemetry_codec.py, saturation,
 * invalid codes, decoder rejection and randomized round trips.
 * NOTE: main preserves the required hosted-C entry name (BARR-C 6.1.i).
 */
#include "test_telemetry_main.h"
#include "telemetry.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_TELEMETRY_ROUNDS (20000U)

static uint32_t g_checks;
static uint32_t g_failures;
static uint32_t g_seed = 0x5EEDU;

/* The same inputs and bytes are in scripts/test_telemetry_codec.py. */
static const telemetry_status_t g_status_golden =
{
    .now_ms = 0x12345678U, .range_age_ms = 1023U, .vleft_mm_s = -123,
    .vright_mm_s = 511, .pitch_mrad = TELEMETRY_INVALID, .peak_mm = 300U,
    .range_mm = 4095U, .seq = 125U, .dropped = 3U, .ignored = 255U,
    .state = 2U, .b_stop = true, .b_hardfault = false,
    .b_heightvalid = true, .b_rangevalid = false
};
static const uint8_t g_status_bytes[TELEMETRY_STATUS_BYTES] =
{
    0xD1U, 0x87U, 0x67U, 0x45U, 0x23U, 0x31U, 0xF0U, 0xFFU, 0xBFU, 0x14U,
    0xFEU, 0x1FU, 0x00U, 0xB2U, 0xFCU, 0xFFU, 0x02U
};
static const telemetry_perception_t g_perception_golden =
{
    .now_ms = 0x89ABCDEFU, .yaw_mrad_s = -4095, .scan_width_mm = 1234U,
    .scan_nearest_mm = 4000U, .seq = 125U, .run_id = 7U, .cal_version = 1U,
    .barcode_action = 4U, .barcode_symbol = 15U, .motion_class = 8U,
    .scan_side = 2U, .scan_valid = 61U, .scan_unknown = 0U,
    .b_tilt_valid = true, .b_hump_active = false, .b_scan_valid = true,
    .b_obstacle_seen = true
};
static const uint8_t g_perception_bytes[TELEMETRY_PERCEPTION_BYTES] =
{
    0xD5U, 0xF7U, 0xDEU, 0xBCU, 0x9AU, 0x78U, 0x10U, 0xC0U, 0x0FU, 0x80U,
    0x28U, 0x4DU, 0xA0U, 0x6FU, 0x0FU, 0x34U
};
static const telemetry_diagnostic_t g_diagnostic_golden =
{
    .now_ms = 1U, .first_fault_ms = 0xFFFFFFFFU, .fault_mask = 0x8001U,
    .mission_peak_mm = 511U, .seq = 0U, .run_id = 255U, .fault_code = 15U,
    .stop_source = 3U, .barcode_unknown = 0U, .barcode_rejected = 200U,
    .b_mission_valid = true
};
static const uint8_t g_diagnostic_bytes[TELEMETRY_DIAGNOSTIC_BYTES] =
{
    0x09U, 0x10U, 0x00U, 0x00U, 0x00U, 0xF0U, 0xFFU, 0xFFU, 0xFFU, 0xFFU,
    0xFFU, 0x00U, 0xC8U, 0xFFU, 0x07U, 0x00U, 0x0EU
};

static void test_telemetry_check (bool b_result, const char * p_label);
static uint32_t test_telemetry_random (void);
static int16_t test_telemetry_signed (uint32_t width);
static void test_telemetry_golden (void);
static void test_telemetry_saturation (void);
static void test_telemetry_rejection (void);
static void test_telemetry_capacity (void);
static size_t test_telemetry_encode_golden (telemetry_frame_t type,
                                            uint8_t * p_frame,
                                            size_t capacity);
static void test_telemetry_status_rounds (void);
static void test_telemetry_perception_rounds (void);
static void test_telemetry_diagnostic_rounds (void);

/** @brief Return a failing exit status for any violated codec contract. */
int main (void)
{
    int result;

    test_telemetry_golden();
    test_telemetry_saturation();
    test_telemetry_rejection();
    test_telemetry_capacity();
    test_telemetry_status_rounds();
    test_telemetry_perception_rounds();
    test_telemetry_diagnostic_rounds();
    result = printf("telemetry: %u checks, %u failures\n", g_checks,
                    g_failures);
    if ((result < 0) || (g_failures > 0U))
    {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

/** @brief Record every assertion without aborting subsequent scenarios. */
static void test_telemetry_check (bool b_result, const char * p_label)
{
    g_checks++;
    if (!b_result)
    {
        int result;

        g_failures++;
        result = fprintf(stderr, "FAIL: %s\n", p_label);
        /* Failure is already recorded even if diagnostic output fails. */
        (void)result;
    }
}

/** @brief Deterministic LCG keeps the randomized rounds reproducible. */
static uint32_t test_telemetry_random (void)
{
    g_seed = (g_seed * 1664525U) + 1013904223U;
    return g_seed;
}

/** @brief Random in-range value of a symmetric signed field, or invalid. */
static int16_t test_telemetry_signed (uint32_t width)
{
    uint32_t limit = (1U << (width - 1U)) - 1U;
    uint32_t draw = test_telemetry_random() % ((2U * limit) + 2U);

    if (draw > (2U * limit))
    {
        return TELEMETRY_INVALID;
    }
    /* draw - limit lies in +/-4095, exactly representable in int16_t. */
    return (int16_t)((int32_t)draw - (int32_t)limit);
}

/** @brief Every frame matches the Python reference bytes exactly. */
static void test_telemetry_golden (void)
{
    uint8_t frame[TELEMETRY_FRAME_MAX + 3U];
    telemetry_status_t status;
    telemetry_perception_t perception;
    telemetry_diagnostic_t diagnostic;
    telemetry_frame_t type = TELEMETRY_FRAME_DIAGNOSTIC;

    memset(frame, 0xA5, sizeof(frame));
    memset(&status, 0, sizeof(status));
    memset(&perception, 0, sizeof(perception));
    memset(&diagnostic, 0, sizeof(diagnostic));
    test_telemetry_check(TELEMETRY_STATUS_BYTES ==
                         telemetry_encode_status(&g_status_golden, frame,
                                                 sizeof(frame)),
                         "status is 17 bytes");
    test_telemetry_check(0 == memcmp(frame, g_status_bytes,
                                     TELEMETRY_STATUS_BYTES),
                         "status golden bytes");
    test_telemetry_check(0xA5U == frame[TELEMETRY_STATUS_BYTES],
                         "status writes nothing past its length");
    test_telemetry_check(telemetry_frame_type(frame, TELEMETRY_STATUS_BYTES,
                                              &type) &&
                         (TELEMETRY_FRAME_STATUS == type), "status type");
    test_telemetry_check(telemetry_decode_status(g_status_bytes,
                         TELEMETRY_STATUS_BYTES, &status) &&
                         (0 == memcmp(&status, &g_status_golden,
                                      sizeof(status))),
                         "status decodes to its inputs");
    test_telemetry_check(TELEMETRY_PERCEPTION_BYTES ==
                         telemetry_encode_perception(&g_perception_golden,
                             frame, sizeof(frame)) &&
                         (0 == memcmp(frame, g_perception_bytes,
                                      TELEMETRY_PERCEPTION_BYTES)),
                         "perception golden bytes");
    test_telemetry_check(telemetry_frame_type(frame,
                         TELEMETRY_PERCEPTION_BYTES, &type) &&
                         (TELEMETRY_FRAME_PERCEPTION == type),
                         "perception type");
    test_telemetry_check(telemetry_decode_perception(g_perception_bytes,
                         TELEMETRY_PERCEPTION_BYTES, &perception) &&
                         (0 == memcmp(&perception, &g_perception_golden,
                                      sizeof(perception))),
                         "perception decodes to its inputs");
    test_telemetry_check(TELEMETRY_DIAGNOSTIC_BYTES ==
                         telemetry_encode_diagnostic(&g_diagnostic_golden,
                             frame, sizeof(frame)) &&
                         (0 == memcmp(frame, g_diagnostic_bytes,
                                      TELEMETRY_DIAGNOSTIC_BYTES)),
                         "diagnostic golden bytes");
    test_telemetry_check(telemetry_decode_diagnostic(g_diagnostic_bytes,
                         TELEMETRY_DIAGNOSTIC_BYTES, &diagnostic) &&
                         (0 == memcmp(&diagnostic, &g_diagnostic_golden,
                                      sizeof(diagnostic))),
                         "diagnostic decodes to its inputs");
    test_telemetry_check(0U == (TELEMETRY_SEQUENCE_MODULO % 5U),
                         "sequence wrap keeps the every-fifth rule");
}

/** @brief Out-of-range inputs saturate; invalid survives a round trip. */
static void test_telemetry_saturation (void)
{
    uint8_t frame[TELEMETRY_FRAME_MAX];
    telemetry_status_t status = g_status_golden;
    telemetry_status_t back;
    telemetry_perception_t perception = g_perception_golden;
    telemetry_perception_t perception_back;
    telemetry_diagnostic_t diagnostic = g_diagnostic_golden;
    telemetry_diagnostic_t diagnostic_back;

    status.range_age_ms = 60000U;
    status.state = 200U;
    status.vleft_mm_s = 600;
    status.vright_mm_s = -32767;
    status.pitch_mrad = 3000;
    status.peak_mm = 700U;
    status.range_mm = 5000U;
    test_telemetry_check(
        (TELEMETRY_STATUS_BYTES ==
         telemetry_encode_status(&status, frame, sizeof(frame))) &&
        telemetry_decode_status(frame, TELEMETRY_STATUS_BYTES, &back) &&
        (1023U == back.range_age_ms) && (15U == back.state) &&
        (511 == back.vleft_mm_s) && (-511 == back.vright_mm_s) &&
        (2047 == back.pitch_mrad) && (511U == back.peak_mm) &&
        (4095U == back.range_mm), "status fields saturate, never wrap");
    status.pitch_mrad = -2048;
    test_telemetry_check(
        (TELEMETRY_STATUS_BYTES ==
         telemetry_encode_status(&status, frame, sizeof(frame))) &&
        telemetry_decode_status(frame, TELEMETRY_STATUS_BYTES, &back) &&
        (-2047 == back.pitch_mrad), "a real -2048 mrad is not invalid");
    perception.yaw_mrad_s = 9000;
    perception.scan_valid = 99U;
    perception.barcode_action = 9U;
    test_telemetry_check(
        (TELEMETRY_PERCEPTION_BYTES == telemetry_encode_perception(
            &perception, frame, sizeof(frame))) &&
        telemetry_decode_perception(frame, TELEMETRY_PERCEPTION_BYTES,
                                    &perception_back) &&
        (4095 == perception_back.yaw_mrad_s) &&
        (63U == perception_back.scan_valid) &&
        (7U == perception_back.barcode_action),
        "perception fields saturate");
    diagnostic.mission_peak_mm = 1000U;
    diagnostic.stop_source = 9U;
    test_telemetry_check(
        (TELEMETRY_DIAGNOSTIC_BYTES == telemetry_encode_diagnostic(
            &diagnostic, frame, sizeof(frame))) &&
        telemetry_decode_diagnostic(frame, TELEMETRY_DIAGNOSTIC_BYTES,
                                    &diagnostic_back) &&
        (511U == diagnostic_back.mission_peak_mm) &&
        (3U == diagnostic_back.stop_source),
        "diagnostic fields saturate");
}

/** @brief Wrong length, version, type, padding or pointers are refused. */
static void test_telemetry_rejection (void)
{
    uint8_t frame[TELEMETRY_FRAME_MAX];
    telemetry_status_t status = g_status_golden;
    telemetry_perception_t perception;
    telemetry_frame_t type = TELEMETRY_FRAME_STATUS;

    memset(frame, 0x5A, sizeof(frame));
    test_telemetry_check((0U == telemetry_encode_status(&g_status_golden,
                          frame, TELEMETRY_STATUS_BYTES - 1U)) &&
                         (0x5AU == frame[0]),
                         "short capacity writes nothing");
    test_telemetry_check(0U == telemetry_encode_status(NULL, frame,
                                                       sizeof(frame)),
                         "NULL status refused");
    test_telemetry_check(0U == telemetry_encode_perception(
                             &g_perception_golden, NULL, 16U),
                         "NULL frame refused");
    memcpy(frame, g_status_bytes, TELEMETRY_STATUS_BYTES);
    test_telemetry_check(!telemetry_decode_status(frame,
                         TELEMETRY_STATUS_BYTES - 1U, &status),
                         "short status refused");
    frame[0] = (uint8_t)((frame[0] & 0xFCU) | 0x02U);
    test_telemetry_check((!telemetry_decode_status(frame,
                          TELEMETRY_STATUS_BYTES, &status)) &&
                         (!telemetry_frame_type(frame,
                          TELEMETRY_STATUS_BYTES, &type)),
                         "other version refused");
    memcpy(frame, g_status_bytes, TELEMETRY_STATUS_BYTES);
    frame[TELEMETRY_STATUS_BYTES - 1U] |= 0x80U;
    test_telemetry_check(!telemetry_decode_status(frame,
                         TELEMETRY_STATUS_BYTES, &status),
                         "nonzero padding refused");
    test_telemetry_check(0 == memcmp(&status, &g_status_golden,
                                     sizeof(status)),
                         "refused decode leaves output unchanged");
    test_telemetry_check(!telemetry_decode_perception(g_status_bytes,
                         TELEMETRY_PERCEPTION_BYTES, &perception),
                         "status bytes are not a perception frame");
    frame[0] = 0x0DU;
    test_telemetry_check(!telemetry_frame_type(frame, 1U, &type),
                         "reserved frame type refused");
    test_telemetry_check(!telemetry_frame_type(frame, 0U, &type),
                         "empty frame refused");
}

/** @brief Every capacity below a frame's size is refused with nothing
 * written; the exact size gives the golden frame and nothing past it.
 */
static void test_telemetry_capacity (void)
{
    static const telemetry_frame_t types[3] =
    {
        TELEMETRY_FRAME_STATUS, TELEMETRY_FRAME_PERCEPTION,
        TELEMETRY_FRAME_DIAGNOSTIC
    };
    static const size_t sizes[3] =
    {
        TELEMETRY_STATUS_BYTES, TELEMETRY_PERCEPTION_BYTES,
        TELEMETRY_DIAGNOSTIC_BYTES
    };
    static const uint8_t * const goldens[3] =
    {
        g_status_bytes, g_perception_bytes, g_diagnostic_bytes
    };
    uint8_t frame[TELEMETRY_FRAME_MAX + 1U];
    size_t kind = 0U;
    size_t capacity = 0U;
    size_t index = 0U;
    bool b_untouched = true;

    for (kind = 0U; kind < 3U; kind++)
    {
        for (capacity = 0U; capacity < sizes[kind]; capacity++)
        {
            memset(frame, 0x5A, sizeof(frame));
            b_untouched = 0U == test_telemetry_encode_golden(types[kind],
                                                             frame,
                                                             capacity);
            for (index = 0U; index < sizeof(frame); index++)
            {
                b_untouched = b_untouched && (0x5AU == frame[index]);
            }
            test_telemetry_check(b_untouched,
                                 "short capacity refused, nothing written");
        }
        memset(frame, 0x5A, sizeof(frame));
        test_telemetry_check((sizes[kind] ==
                              test_telemetry_encode_golden(types[kind], frame,
                                                           sizes[kind])) &&
                             (0 == memcmp(frame, goldens[kind],
                                          sizes[kind])) &&
                             (0x5AU == frame[sizes[kind]]),
                             "exact capacity writes the frame only");
    }
}

/** @brief Encode the golden record of one frame type. */
static size_t test_telemetry_encode_golden (telemetry_frame_t type,
                                            uint8_t * p_frame,
                                            size_t capacity)
{
    size_t length = 0U;

    switch (type)
    {
        case TELEMETRY_FRAME_STATUS:
            length = telemetry_encode_status(&g_status_golden, p_frame,
                                             capacity);
        break;
        case TELEMETRY_FRAME_PERCEPTION:
            length = telemetry_encode_perception(&g_perception_golden,
                                                 p_frame, capacity);
        break;
        case TELEMETRY_FRAME_DIAGNOSTIC:
            length = telemetry_encode_diagnostic(&g_diagnostic_golden,
                                                 p_frame, capacity);
        break;
        default:
            length = 0U;
        break;
    }
    return length;
}

/** @brief Random in-range status values decode to themselves. */
static void test_telemetry_status_rounds (void)
{
    uint8_t frame[TELEMETRY_FRAME_MAX];
    telemetry_status_t status;
    telemetry_status_t back;
    uint32_t round;
    bool b_equal = true;

    for (round = 0U; round < TEST_TELEMETRY_ROUNDS; round++)
    {
        memset(&status, 0, sizeof(status));
        memset(&back, 0, sizeof(back));
        status.now_ms = test_telemetry_random();
        status.range_age_ms = (uint16_t)(test_telemetry_random() % 1024U);
        status.vleft_mm_s = test_telemetry_signed(10U);
        status.vright_mm_s = test_telemetry_signed(10U);
        status.pitch_mrad = test_telemetry_signed(12U);
        status.peak_mm = (uint16_t)(test_telemetry_random() % 512U);
        status.range_mm = (uint16_t)(test_telemetry_random() % 4096U);
        status.seq = (uint8_t)(test_telemetry_random() % 250U);
        status.dropped = (uint8_t)test_telemetry_random();
        status.ignored = (uint8_t)test_telemetry_random();
        status.state = (uint8_t)(test_telemetry_random() % 16U);
        status.b_stop = 0U != (test_telemetry_random() & 1U);
        status.b_hardfault = 0U != (test_telemetry_random() & 1U);
        status.b_heightvalid = 0U != (test_telemetry_random() & 1U);
        status.b_rangevalid = 0U != (test_telemetry_random() & 1U);
        b_equal = b_equal &&
            (TELEMETRY_STATUS_BYTES ==
             telemetry_encode_status(&status, frame, sizeof(frame))) &&
            telemetry_decode_status(frame, TELEMETRY_STATUS_BYTES, &back) &&
            (0 == memcmp(&status, &back, sizeof(status)));
    }
    test_telemetry_check(b_equal, "status random round trips");
}

/** @brief Random in-range perception values decode to themselves. */
static void test_telemetry_perception_rounds (void)
{
    uint8_t frame[TELEMETRY_FRAME_MAX];
    telemetry_perception_t status;
    telemetry_perception_t back;
    uint32_t round;
    bool b_equal = true;

    for (round = 0U; round < TEST_TELEMETRY_ROUNDS; round++)
    {
        memset(&status, 0, sizeof(status));
        memset(&back, 0, sizeof(back));
        status.now_ms = test_telemetry_random();
        status.yaw_mrad_s = test_telemetry_signed(13U);
        status.scan_width_mm = (uint16_t)(test_telemetry_random() % 4096U);
        status.scan_nearest_mm =
            (uint16_t)(test_telemetry_random() % 4096U);
        status.seq = (uint8_t)(test_telemetry_random() % 250U);
        status.run_id = (uint8_t)test_telemetry_random();
        status.cal_version = (uint8_t)test_telemetry_random();
        status.barcode_action = (uint8_t)(test_telemetry_random() % 8U);
        status.barcode_symbol = (uint8_t)(test_telemetry_random() % 16U);
        status.motion_class = (uint8_t)(test_telemetry_random() % 16U);
        status.scan_side = (uint8_t)(test_telemetry_random() % 4U);
        status.scan_valid = (uint8_t)(test_telemetry_random() % 64U);
        status.scan_unknown = (uint8_t)(test_telemetry_random() % 64U);
        status.b_tilt_valid = 0U != (test_telemetry_random() & 1U);
        status.b_hump_active = 0U != (test_telemetry_random() & 1U);
        status.b_scan_valid = 0U != (test_telemetry_random() & 1U);
        status.b_obstacle_seen = 0U != (test_telemetry_random() & 1U);
        b_equal = b_equal &&
            (TELEMETRY_PERCEPTION_BYTES ==
             telemetry_encode_perception(&status, frame, sizeof(frame))) &&
            telemetry_decode_perception(frame, TELEMETRY_PERCEPTION_BYTES,
                                        &back) &&
            (0 == memcmp(&status, &back, sizeof(status)));
    }
    test_telemetry_check(b_equal, "perception random round trips");
}

/** @brief Random in-range diagnostic values decode to themselves. */
static void test_telemetry_diagnostic_rounds (void)
{
    uint8_t frame[TELEMETRY_FRAME_MAX];
    telemetry_diagnostic_t status;
    telemetry_diagnostic_t back;
    uint32_t round;
    bool b_equal = true;

    for (round = 0U; round < TEST_TELEMETRY_ROUNDS; round++)
    {
        memset(&status, 0, sizeof(status));
        memset(&back, 0, sizeof(back));
        status.now_ms = test_telemetry_random();
        status.first_fault_ms = test_telemetry_random();
        status.fault_mask = (uint16_t)test_telemetry_random();
        status.mission_peak_mm = (uint16_t)(test_telemetry_random() % 512U);
        status.seq = (uint8_t)(test_telemetry_random() % 250U);
        status.run_id = (uint8_t)test_telemetry_random();
        status.fault_code = (uint8_t)(test_telemetry_random() % 16U);
        status.stop_source = (uint8_t)(test_telemetry_random() % 4U);
        status.barcode_unknown = (uint8_t)test_telemetry_random();
        status.barcode_rejected = (uint8_t)test_telemetry_random();
        status.b_mission_valid = 0U != (test_telemetry_random() & 1U);
        b_equal = b_equal &&
            (TELEMETRY_DIAGNOSTIC_BYTES ==
             telemetry_encode_diagnostic(&status, frame, sizeof(frame))) &&
            telemetry_decode_diagnostic(frame, TELEMETRY_DIAGNOSTIC_BYTES,
                                        &back) &&
            (0 == memcmp(&status, &back, sizeof(status)));
    }
    test_telemetry_check(b_equal, "diagnostic random round trips");
}

/* End of file. */

