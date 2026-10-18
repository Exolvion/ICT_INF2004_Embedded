/**
 * @file test_mqtt_main.c
 * @brief Validate complete tokens, fragment boundaries and invalid input.
 * NOTE: main is the externally required hosted-C entry point (BARR 6.1.i).
 */
#include "test_mqtt_main.h"
#include "mqttcmd.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t checks;
static uint32_t failures;

static void mqttcmd_test_check (bool b_result, const char * p_label);
static void mqttcmd_test_tokens (void);
static void mqttcmd_test_invalid (void);

/** @brief Return failure if any scenario's contract is violated. */
int main (void)
{
    int result;

    mqttcmd_test_tokens();
    mqttcmd_test_invalid();
    result = printf("mqttcmd: %u checks, %u failures\n", checks, failures);
    if ((result < 0) || (failures > 0U))
    {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

/** @brief Record every assertion without aborting subsequent scenarios. */
static void mqttcmd_test_check (bool b_result, const char * p_label)
{
    checks++;
    if (!b_result)
    {
        int result;

        failures++;
        result = fprintf(stderr, "FAIL: %s\n", p_label);
        /* Failure is already recorded even if diagnostic output fails. */
        (void)result;
    }
}

/** @brief Every split of every supported token, including empty fragments. */
static void mqttcmd_test_tokens (void)
{
    static const char * const tokens[] =
    {
        "START", "STOP", "RESET", "LEFT", "RIGHT", "STRAIGHT", "UTURN"
    };
    size_t index;
    size_t split;
    size_t count = sizeof(tokens) / sizeof(tokens[0]);

    for (index = 0U; index < count; index++)
    {
        size_t length = strlen(tokens[index]);

        mqttcmd_test_check(mqttcmd_valid(tokens[index], length),
                            "exact token accepted");
        for (split = 0U; split <= length; split++)
        {
            mqttcmd_t state;
            bool b_done;

            /* Fixture lengths and split offsets are all at most 8. */
            mqttcmd_begin(&state, (uint32_t)length, true);
            b_done = mqttcmd_feed(&state, (const uint8_t *)tokens[index],
                                   (uint32_t)split, false);
            mqttcmd_test_check(!b_done, "non-final fragment is incomplete");
            /* char bytes are read-only through a uint8_t alias; length <=8. */
            b_done = mqttcmd_feed(&state,
                                   (const uint8_t *)&tokens[index][split],
                                   (uint32_t)(length - split), true);
            mqttcmd_test_check(b_done, "valid final fragment completes");
            mqttcmd_test_check(0 == strcmp(state.text, tokens[index]),
                                "assembled bytes equal fixture");
            b_done = mqttcmd_feed(&state, NULL, 0U, true);
            mqttcmd_test_check(!b_done, "completed command cannot replay");
        }
    }
}

/** @brief Reject malformed metadata, truncated/oversized and unknown bytes. */
static void mqttcmd_test_invalid (void)
{
    static const char * const invalid[] =
    {
        "", "start", "STOP ", " STOP", "STOP\n", "STOPX", "GO", "U-TURN"
    };
    mqttcmd_t state;
    size_t index;
    size_t count = sizeof(invalid) / sizeof(invalid[0]);
    const uint8_t stop[] = {'S', 'T', 'O', 'P'};
    const uint8_t embedded[] = {'S', 'T', 'O', 'P', 0U};
    bool b_done;

    for (index = 0U; index < count; index++)
    {
        mqttcmd_test_check(!mqttcmd_valid(invalid[index],
                                         strlen(invalid[index])),
                            "invalid token rejected");
    }
    mqttcmd_test_check(!mqttcmd_valid(NULL, 4U), "NULL token rejected");
    mqttcmd_test_check(!mqttcmd_valid("STOP", SIZE_MAX),
                        "huge length rejected before reading");
    mqttcmd_begin(NULL, 4U, true);
    mqttcmd_test_check(!mqttcmd_feed(NULL, stop, 4U, true),
                        "NULL state rejected");
    mqttcmd_begin(&state, 4U, false);
    mqttcmd_test_check(!mqttcmd_feed(&state, stop, 4U, true),
                        "wrong topic/retained metadata rejected");
    mqttcmd_begin(&state, UINT32_MAX, true);
    mqttcmd_test_check(!mqttcmd_feed(&state, stop, 4U, true),
                        "oversized declared length rejected");
    mqttcmd_begin(&state, 4U, true);
    mqttcmd_test_check(!mqttcmd_feed(&state, NULL, 4U, true),
                        "NULL nonempty data rejected");
    mqttcmd_begin(&state, 4U, true);
    mqttcmd_test_check(!mqttcmd_feed(&state, stop, 3U, true),
                        "truncated final fragment rejected");
    mqttcmd_begin(&state, 4U, true);
    mqttcmd_test_check(!mqttcmd_feed(&state, stop, UINT32_MAX, true),
                        "oversized fragment rejected before copy");
    mqttcmd_begin(&state, 5U, true);
    mqttcmd_test_check(!mqttcmd_feed(&state, embedded, 5U, true),
                        "embedded NUL rejected");
    mqttcmd_begin(&state, 0U, true);
    mqttcmd_test_check(!mqttcmd_feed(&state, NULL, 0U, true),
                        "empty publication rejected");
    mqttcmd_begin(&state, 4U, true);
    b_done = mqttcmd_feed(&state, stop, 2U, false);
    mqttcmd_test_check(!b_done, "partial old message waits");
    mqttcmd_begin(&state, 4U, true);
    mqttcmd_test_check(mqttcmd_feed(&state, stop, 4U, true),
                        "new publication clears old fragments");
}

/* End of file. */

