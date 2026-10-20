/**
 * @file test_bitpack_main.c
 * @brief Masks, saturation, sign fields, threshold-preserving rounding and
 * the LSB-first stream, including a fixed golden byte vector.
 * NOTE: main is the externally required hosted-C entry point (BARR 6.1.i).
 */
#include "test_bitpack_main.h"
#include "bitpack.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_BITPACK_FRAME (64U)
#define TEST_BITPACK_ROUNDS (2000U)

static uint32_t g_checks;
static uint32_t g_failures;
static uint32_t g_seed = 0x2004U;

static void test_bitpack_check (bool b_condition, const char * p_label);
static uint32_t test_bitpack_random (void);
static void test_bitpack_masks (void);
static void test_bitpack_signed (void);
static void test_bitpack_quantise (void);
static void test_bitpack_thresholds (void);
static bool test_bitpack_rules (double value, double limit,
                                int32_t threshold);
static void test_bitpack_golden (void);
static void test_bitpack_stream (void);
static void test_bitpack_limits (void);

/** @brief Run every scenario; a nonzero exit reports any failure. */
int main (void)
{
    test_bitpack_masks();
    test_bitpack_signed();
    test_bitpack_quantise();
    test_bitpack_thresholds();
    test_bitpack_golden();
    test_bitpack_stream();
    test_bitpack_limits();
    if (printf("bitpack: %u checks, %u failures\n", g_checks,
               g_failures) < 0)
    {
        return EXIT_FAILURE;
    }
    return (0U == g_failures) ? EXIT_SUCCESS : EXIT_FAILURE;
}

/** @brief Record one named assertion without aborting the run. */
static void test_bitpack_check (bool b_condition, const char * p_label)
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

/** @brief Deterministic 32-bit LCG keeps every run reproducible. */
static uint32_t test_bitpack_random (void)
{
    g_seed = (g_seed * 1664525U) + 1013904223U;
    return g_seed;
}

/** @brief Mask edges, saturation and the reserved signed code. */
static void test_bitpack_masks (void)
{
    test_bitpack_check(0U == bitpack_mask(0U), "width 0 mask");
    test_bitpack_check(1U == bitpack_mask(1U), "width 1 mask");
    test_bitpack_check(0xFFFU == bitpack_mask(12U), "width 12 mask");
    test_bitpack_check(0x7FFFFFFFU == bitpack_mask(31U), "width 31 mask");
    test_bitpack_check(UINT32_MAX == bitpack_mask(32U), "width 32 mask");
    test_bitpack_check(UINT32_MAX == bitpack_mask(40U), "wide mask");
    test_bitpack_check(255U == bitpack_saturate(1000U, 8U),
                       "saturate above field");
    test_bitpack_check(17U == bitpack_saturate(17U, 8U), "keep in field");
    test_bitpack_check(511U == bitpack_saturate(UINT32_MAX, 9U),
                       "saturate maximum");
    test_bitpack_check(0x800U == bitpack_invalid(12U), "12-bit reserved");
    test_bitpack_check(0x80000000U == bitpack_invalid(32U),
                       "32-bit reserved");
    test_bitpack_check(0U == bitpack_invalid(1U), "no 1-bit signed field");
    test_bitpack_check(2047 == bitpack_signed_max(12U), "12-bit limit");
    test_bitpack_check(INT32_MAX == bitpack_signed_max(32U), "32-bit limit");
    test_bitpack_check(0 == bitpack_signed_max(33U), "33-bit rejected");
}

/** @brief Exhaustive round trips for narrow widths and wide edge values. */
static void test_bitpack_signed (void)
{
    uint32_t width;
    int32_t  value;
    int32_t  limit;
    bool     b_round_trip = true;
    bool     b_reserved_unused = true;

    for (width = 2U; width <= 16U; width++)
    {
        limit = bitpack_signed_max(width);
        for (value = -limit - 3; value <= (limit + 3); value++)
        {
            uint32_t field = bitpack_signed(value, width);
            int32_t  expected = (value > limit) ? limit :
                                ((value < -limit) ? -limit : value);

            b_round_trip = b_round_trip &&
                           (bitpack_extend(field, width) == expected) &&
                           (field <= bitpack_mask(width));
            b_reserved_unused = b_reserved_unused &&
                                (field != bitpack_invalid(width));
        }
        b_round_trip = b_round_trip &&
            (bitpack_extend(bitpack_invalid(width), width) == (-limit - 1));
    }
    test_bitpack_check(b_round_trip, "narrow signed round trips");
    test_bitpack_check(b_reserved_unused, "reserved code never encoded");
    test_bitpack_check(-INT32_MAX ==
                       bitpack_extend(bitpack_signed(INT32_MIN, 32U), 32U),
                       "INT32_MIN saturates to symmetric minimum");
    test_bitpack_check(INT32_MIN == bitpack_extend(0x80000000U, 32U),
                       "32-bit reserved extends to INT32_MIN");
    test_bitpack_check(-1 == bitpack_extend(0xFFFFFFFFU, 32U),
                       "32-bit minus one");
    test_bitpack_check(-1 == bitpack_extend(0xFFFFU, 12U),
                       "high garbage bits ignored");
    test_bitpack_check(0U == bitpack_signed(5, 1U), "1-bit signed refused");
    test_bitpack_check(0 == bitpack_extend(1U, 0U), "0-bit extend refused");
}

/** @brief Rounding rules, clamping and refusal of non-finite input. */
static void test_bitpack_quantise (void)
{
    int32_t result = 99;

    test_bitpack_check(bitpack_quantise(0.2504, 1000.0, BITPACK_NEAREST,
                                        0, 4095, &result) &&
                       (250 == result), "nearest");
    test_bitpack_check(bitpack_quantise(0.2504, 1000.0, BITPACK_CEIL,
                                        0, 4095, &result) &&
                       (251 == result), "ceil");
    test_bitpack_check(bitpack_quantise(0.2504, 1000.0, BITPACK_FLOOR,
                                        0, 4095, &result) &&
                       (250 == result), "floor");
    test_bitpack_check(bitpack_quantise(-0.6001, 1000.0, BITPACK_AWAY,
                                        -2047, 2047, &result) &&
                       (-601 == result), "away from zero, negative");
    test_bitpack_check(bitpack_quantise(0.6001, 1000.0, BITPACK_AWAY,
                                        -2047, 2047, &result) &&
                       (601 == result), "away from zero, positive");
    test_bitpack_check(bitpack_quantise(-0.0005, 1000.0, BITPACK_NEAREST,
                                        -2047, 2047, &result) &&
                       (-1 == result), "ties away from zero");
    test_bitpack_check(bitpack_quantise(9.0, 1000.0, BITPACK_CEIL,
                                        0, 4095, &result) &&
                       (4095 == result), "clamp maximum");
    test_bitpack_check(bitpack_quantise(-9.0, 1000.0, BITPACK_FLOOR,
                                        -2047, 2047, &result) &&
                       (-2047 == result), "clamp minimum");
    test_bitpack_check(bitpack_quantise(1.0e300, 1.0e300, BITPACK_CEIL,
                                        0, 9, &result) == false,
                       "overflowing product refused");
    result = 7;
    test_bitpack_check((!bitpack_quantise(NAN, 1000.0, BITPACK_CEIL,
                                          0, 9, &result)) && (7 == result),
                       "NaN refused, output unchanged");
    test_bitpack_check((!bitpack_quantise(INFINITY, 1000.0, BITPACK_AWAY,
                                          -9, 9, &result)) &&
                       (!bitpack_quantise(-INFINITY, 1000.0, BITPACK_FLOOR,
                                          -9, 9, &result)) &&
                       (!bitpack_quantise(-1.0e300, 1.0e300, BITPACK_NEAREST,
                                          -9, 9, &result)) && (7 == result),
                       "+/-Inf and -1e300 x 1e300 refused, output unchanged");
    test_bitpack_check(bitpack_quantise(1.0e15, 1.0, BITPACK_CEIL, 0, 9,
                                        &result) && (9 == result) &&
                       bitpack_quantise(-1.0e15, 1.0, BITPACK_AWAY, -9, 9,
                                        &result) && (-9 == result),
                       "huge finite values clamp without a wide cast");
    test_bitpack_check(!bitpack_quantise(1.0, 0.0, BITPACK_CEIL, 0, 9,
                                         &result), "zero scale refused");
    test_bitpack_check(!bitpack_quantise(1.0, 1.0, BITPACK_CEIL, 9, 0,
                                         &result), "inverted bounds refused");
    test_bitpack_check(!bitpack_quantise(1.0, 1.0, BITPACK_CEIL, 0, 9,
                                         NULL), "NULL refused");
}

/** @brief Each rule keeps its consumer's ORIGINAL comparison, written as the
 * firmware writes it (unscaled metres or radians against the decimal
 * literal), on a dense sweep and on +/-400 ulps around each threshold.
 */
static void test_bitpack_thresholds (void)
{
    static const double limits[] = {0.25, 0.60, 0.65};
    static const int32_t thresholds[] = {250, 600, 650};
    uint32_t index;
    int32_t  step;
    bool     b_ceil = true;
    bool     b_floor = true;
    bool     b_away = true;

    for (index = 0U; index < 3U; index++)
    {
        double below = limits[index];
        double above = limits[index];

        for (step = -2000; step <= 2000; step++)
        {
            double value = limits[index] + ((double)step * 1.0e-6);

            b_ceil = b_ceil && test_bitpack_rules(value, limits[index],
                                                  thresholds[index]);
        }
        for (step = 0; step < 400; step++)
        {
            below = nextafter(below, 0.0);
            above = nextafter(above, 1.0);
            b_floor = b_floor &&
                      test_bitpack_rules(below, limits[index],
                                         thresholds[index]) &&
                      test_bitpack_rules(above, limits[index],
                                         thresholds[index]);
        }
        b_away = b_away && test_bitpack_rules(limits[index], limits[index],
                                              thresholds[index]);
    }
    test_bitpack_check(b_ceil, "rules keep the original decisions, sweep");
    test_bitpack_check(b_floor, "rules keep the original decisions, ulps");
    test_bitpack_check(b_away, "rules keep the original decisions, exact");
}

/** @brief CEIL: x <= limit; FLOOR: x >= limit; AWAY: |x| > limit for x and
 * -x; each quantised decision must equal the unscaled double decision.
 */
static bool test_bitpack_rules (double value, double limit,
                                int32_t threshold)
{
    int32_t q_ceil = 0;
    int32_t q_floor = 0;
    int32_t q_away = 0;
    int32_t q_negative = 0;
    bool b_ok = bitpack_quantise(value, 1000.0, BITPACK_CEIL, 0, 4095,
                                 &q_ceil) &&
                bitpack_quantise(value, 1000.0, BITPACK_FLOOR, 0, 4095,
                                 &q_floor) &&
                bitpack_quantise(value, 1000.0, BITPACK_AWAY, -2047, 2047,
                                 &q_away) &&
                bitpack_quantise(-value, 1000.0, BITPACK_AWAY, -2047, 2047,
                                 &q_negative);

    return b_ok &&
           ((q_ceil <= threshold) == (value <= limit)) &&
           ((q_floor >= threshold) == (value >= limit)) &&
           ((abs(q_away) > threshold) == (fabs(value) > limit)) &&
           ((abs(q_negative) > threshold) == (fabs(-value) > limit));
}

/** @brief Fixed vector shared with the Python decoder's golden tests. */
static void test_bitpack_golden (void)
{
    static const uint8_t expected[6] = {0xFAU, 0x7FU, 0xEFU, 0xBEU, 0xADU,
                                        0xDEU};
    uint8_t frame[8];
    bitpack_writer_t writer;
    bitpack_reader_t reader;
    uint32_t index;
    bool b_bytes = true;

    bitpack_writer_init(&writer, frame, sizeof(frame));
    bitpack_write(&writer, 0xAU, 4U);
    bitpack_write(&writer, 0x3U, 2U);
    bitpack_write(&writer, 0x1FFU, 9U);
    bitpack_write(&writer, 0U, 1U);
    bitpack_write(&writer, 0xDEADBEEFU, 32U);
    test_bitpack_check(6U == bitpack_writer_size(&writer), "golden size");
    for (index = 0U; index < 6U; index++)
    {
        b_bytes = b_bytes && (expected[index] == frame[index]);
    }
    test_bitpack_check(b_bytes, "golden LSB-first little-endian bytes");
    bitpack_reader_init(&reader, frame, 6U);
    test_bitpack_check(0xAU == bitpack_read(&reader, 4U), "golden field 1");
    test_bitpack_check(0x3U == bitpack_read(&reader, 2U), "golden field 2");
    test_bitpack_check(0x1FFU == bitpack_read(&reader, 9U),
                       "golden field 3");
    test_bitpack_check(0U == bitpack_read(&reader, 1U), "golden field 4");
    test_bitpack_check(0xDEADBEEFU == bitpack_read(&reader, 32U),
                       "golden field 5");
    test_bitpack_check(bitpack_reader_complete(&reader), "golden complete");
    test_bitpack_check(0U == bitpack_read(&reader, 1U), "read past end");
    test_bitpack_check(!bitpack_reader_complete(&reader),
                       "past end invalidates");
}

/** @brief Random widths and values round-trip within the frame capacity. */
static void test_bitpack_stream (void)
{
    uint8_t  frame[TEST_BITPACK_FRAME];
    uint32_t widths[TEST_BITPACK_FRAME * 8U];
    uint32_t values[TEST_BITPACK_FRAME * 8U];
    uint32_t round;
    uint32_t count;
    uint32_t index;
    uint32_t used;
    bool     b_equal = true;
    bitpack_writer_t writer;
    bitpack_reader_t reader;

    for (round = 0U; round < TEST_BITPACK_ROUNDS; round++)
    {
        bitpack_writer_init(&writer, frame, sizeof(frame));
        count = 0U;
        used = 0U;
        while (count < (TEST_BITPACK_FRAME * 8U))
        {
            uint32_t width = (test_bitpack_random() % 32U) + 1U;

            if ((used + width) > (TEST_BITPACK_FRAME * 8U))
            {
                break;
            }
            widths[count] = width;
            values[count] = test_bitpack_random() & bitpack_mask(width);
            bitpack_write(&writer, values[count], width);
            used += width;
            ++count;
        }
        bitpack_reader_init(&reader, frame, bitpack_writer_size(&writer));
        for (index = 0U; index < count; index++)
        {
            b_equal = b_equal &&
                      (values[index] == bitpack_read(&reader, widths[index]));
        }
        b_equal = b_equal && bitpack_reader_complete(&reader);
    }
    test_bitpack_check(b_equal, "random stream round trips");
}

/** @brief Oversized values, full buffers, truncation and stale padding. */
static void test_bitpack_limits (void)
{
    uint8_t frame[2];
    bitpack_writer_t writer;
    bitpack_reader_t reader;

    bitpack_writer_init(&writer, frame, sizeof(frame));
    bitpack_write(&writer, 0x10U, 4U);
    test_bitpack_check((!writer.b_valid) &&
                       (0U == bitpack_writer_size(&writer)),
                       "value wider than field invalidates");
    bitpack_writer_init(&writer, frame, sizeof(frame));
    bitpack_write(&writer, 0x7FFFU, 15U);
    bitpack_write(&writer, 1U, 1U);
    test_bitpack_check(2U == bitpack_writer_size(&writer), "exact fill");
    bitpack_write(&writer, 1U, 1U);
    test_bitpack_check(!writer.b_valid, "overflow invalidates");
    bitpack_writer_init(&writer, NULL, 4U);
    test_bitpack_check(!writer.b_valid, "NULL buffer refused");
    frame[0] = 0x05U;
    frame[1] = 0x80U;
    bitpack_reader_init(&reader, frame, 2U);
    test_bitpack_check(5U == bitpack_read(&reader, 9U), "9-bit read");
    test_bitpack_check(!bitpack_reader_complete(&reader),
                       "nonzero padding rejected");
    bitpack_reader_init(&reader, frame, 1U);
    (void)bitpack_read(&reader, 9U);
    test_bitpack_check(!reader.b_valid, "truncated frame rejected");
    bitpack_reader_init(&reader, NULL, 0U);
    test_bitpack_check(bitpack_reader_complete(&reader), "empty frame");
    test_bitpack_check(0U == bitpack_read(&reader, 0U) && (!reader.b_valid),
                       "zero width read refused");
}

/* End of file. */

