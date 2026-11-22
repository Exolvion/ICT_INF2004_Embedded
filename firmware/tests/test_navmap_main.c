/**
 * @file test_navmap_main.c
 * @brief Validate the 3-byte letter -> junction move map (navmap.h): the
 * golden bytes the PC tool (tools/broker/send_cmd.js) sends, every letter
 * A-Z in every slot, rejection of bad patterns, letters past Z and repeats,
 * and fragment assembly.
 * NOTE: main is the externally required hosted-C entry point (BARR 6.1.i).
 */
#include "test_navmap_main.h"
#include "navmap.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t checks;
static uint32_t failures;

static void navmap_test_check (bool b_result, const char * p_label);
static void navmap_test_golden (void);
static void navmap_test_all_letters (void);
static void navmap_test_invalid (void);
static void navmap_test_fragments (void);

/** @brief Return failure if any scenario's contract is violated. */
int main (void)
{
    int result;

    navmap_test_golden();
    navmap_test_all_letters();
    navmap_test_invalid();
    navmap_test_fragments();
    result = printf("navmap: %u checks, %u failures\n", checks, failures);
    if ((result < 0) || (failures > 0U))
    {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

/** @brief Record every assertion without aborting subsequent scenarios. */
static void navmap_test_check (bool b_result, const char * p_label)
{
    checks++;
    if (!b_result)
    {
        int result;

        failures++;
        result = fprintf(stderr, "FAIL: %s\n", p_label);
        /* Failure remains recorded if diagnostics cannot be written. */
        (void)result;
    }
}

/** @brief F=A U=Z L=B R=C is 0xB10720: bytes 20 07 B1 (send_cmd.js). */
static void navmap_test_golden (void)
{
    const char    letters[NAVMAP_MOVES] = {'A', 'Z', 'B', 'C'};
    const uint8_t golden[NAVMAP_LENGTH] = {0x20U, 0x07U, 0xB1U};
    uint8_t       bytes[NAVMAP_LENGTH] = {0U, 0U, 0U};
    char          back[NAVMAP_MOVES] = {'\0', '\0', '\0', '\0'};

    navmap_test_check(navmap_pack(letters, bytes), "golden packs");
    navmap_test_check(0 == memcmp(bytes, golden, sizeof(golden)),
                      "golden bytes 20 07 B1");
    navmap_test_check(navmap_unpack(golden, back), "golden unpacks");
    navmap_test_check(0 == memcmp(back, letters, sizeof(letters)),
                      "golden letters A Z B C");
}

/** @brief Every letter in every slot round-trips (the others distinct). */
static void navmap_test_all_letters (void)
{
    char     letters[NAVMAP_MOVES];
    char     back[NAVMAP_MOVES];
    uint8_t  bytes[NAVMAP_LENGTH];
    uint32_t slot;
    uint32_t code;
    uint32_t other;
    uint32_t next;
    bool     b_ok = true;

    for (slot = 0U; slot < (uint32_t)NAVMAP_MOVES; ++slot)
    {
        for (code = 0U; code < NAVMAP_LETTERS; ++code)
        {
            letters[slot] = (char)('A' + (int)code);
            next = (code + 1U) % NAVMAP_LETTERS;
            for (other = 0U; other < (uint32_t)NAVMAP_MOVES; ++other)
            {
                if (other != slot)
                {
                    letters[other] = (char)('A' + (int)next);
                    next = (next + 1U) % NAVMAP_LETTERS;
                }
            }
            (void)memset(back, 0, sizeof(back));
            b_ok = b_ok && navmap_pack(letters, bytes) &&
                   navmap_unpack(bytes, back) &&
                   (0 == memcmp(back, letters, sizeof(letters)));
        }
    }
    navmap_test_check(b_ok, "every letter A-Z round-trips in every slot");
}

/** @brief Bad pattern, letter past Z, a repeat, a non-letter: refused. */
static void navmap_test_invalid (void)
{
    const char    repeat[NAVMAP_MOVES] = {'A', 'B', 'A', 'C'};
    const char    lower[NAVMAP_MOVES] = {'a', 'B', 'C', 'D'};
    const char    digit[NAVMAP_MOVES] = {'1', 'B', 'C', 'D'};
    const uint8_t bad_pattern[NAVMAP_LENGTH] = {0x20U, 0x07U, 0xA1U};
    const uint8_t past_z[NAVMAP_LENGTH] = {0x1AU, 0x07U, 0xB1U}; /* 26 */
    const uint8_t same[NAVMAP_LENGTH] = {0x00U, 0x00U, 0xB0U};   /* AAAA */
    const uint8_t zero[NAVMAP_LENGTH] = {0x00U, 0x00U, 0x00U};
    uint8_t       bytes[NAVMAP_LENGTH] = {0x55U, 0x55U, 0x55U};
    char          back[NAVMAP_MOVES] = {'x', 'x', 'x', 'x'};

    navmap_test_check(!navmap_pack(repeat, bytes), "repeat not packed");
    navmap_test_check(!navmap_pack(lower, bytes), "lower case not packed");
    navmap_test_check(!navmap_pack(digit, bytes), "digit not packed");
    navmap_test_check((0x55U == bytes[0]) && (0x55U == bytes[2]),
                      "refused pack leaves the bytes");
    navmap_test_check(!navmap_unpack(bad_pattern, back), "bad pattern");
    navmap_test_check(!navmap_unpack(past_z, back), "letter past Z");
    navmap_test_check(!navmap_unpack(same, back), "four equal letters");
    navmap_test_check(!navmap_unpack(zero, back), "all zero");
    navmap_test_check('x' == back[0], "refused unpack leaves the letters");
    navmap_test_check(!navmap_pack(NULL, bytes), "null letters");
    navmap_test_check(!navmap_unpack(NULL, back), "null bytes");
}

/** @brief Assembly: whole, split, wrong length, wrong topic, overrun. */
static void navmap_test_fragments (void)
{
    const uint8_t golden[NAVMAP_LENGTH] = {0x20U, 0x07U, 0xB1U};
    const uint8_t four[4] = {0x20U, 0x07U, 0xB1U, 0x00U};
    navmap_rx_t   rx;

    navmap_begin(&rx, NAVMAP_LENGTH, true);
    navmap_test_check(navmap_feed(&rx, golden, NAVMAP_LENGTH, true),
                      "whole map accepted");
    navmap_test_check(0 == memcmp(rx.bytes, golden, sizeof(golden)),
                      "whole map bytes kept");
    navmap_test_check(!navmap_feed(&rx, golden, NAVMAP_LENGTH, true),
                      "no repeat after completion");

    navmap_begin(&rx, NAVMAP_LENGTH, true);
    navmap_test_check(!navmap_feed(&rx, golden, 1U, false), "split part 1");
    navmap_test_check(!navmap_feed(&rx, &golden[1], 1U, false),
                      "split part 2");
    navmap_test_check(navmap_feed(&rx, &golden[2], 1U, true),
                      "split map accepted");

    navmap_begin(&rx, NAVMAP_LENGTH, false);
    navmap_test_check(!navmap_feed(&rx, golden, NAVMAP_LENGTH, true),
                      "wrong topic refused");
    navmap_begin(&rx, 1U, true);
    navmap_test_check(!navmap_feed(&rx, golden, 1U, true),
                      "one-byte command is not a map");
    navmap_begin(&rx, 4U, true);
    navmap_test_check(!navmap_feed(&rx, four, 4U, true),
                      "four bytes refused");
    navmap_begin(&rx, NAVMAP_LENGTH, true);
    navmap_test_check(!navmap_feed(&rx, four, 4U, true),
                      "overrun of a declared 3 refused");
    navmap_begin(&rx, NAVMAP_LENGTH, true);
    navmap_test_check(!navmap_feed(&rx, golden, 2U, true),
                      "short final refused");
    navmap_begin(&rx, NAVMAP_LENGTH, true);
    navmap_test_check(!navmap_feed(&rx, NULL, 3U, true), "null data");
    navmap_test_check(!navmap_feed(NULL, golden, 3U, true), "null state");
}

/* End of file. */

