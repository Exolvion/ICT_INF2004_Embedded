/**
 * @file test_mqtt_main.c
 * @brief Validate the one-byte command format, fragment boundaries and
 * rejection of every byte outside the seven valid commands.
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
static void mqttcmd_test_codes (void);
static void mqttcmd_test_fragments (void);
static void mqttcmd_test_invalid (void);

/** @brief Return failure if any scenario's contract is violated. */
int main (void)
{
    int result;

    mqttcmd_test_codes();
    mqttcmd_test_fragments();
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

/** @brief All 256 byte values, each checked: exactly 0xA9..0xAF decode,
 * each to the code whose byte it is; every other byte is refused.
 */
static void mqttcmd_test_codes (void)
{
    uint32_t value;
    uint32_t valid = 0U;

    for (value = 0U; value <= UINT8_MAX; value++)
    {
        /* Loop bound keeps value within the uint8_t range. */
        mqttcmd_code_t code = mqttcmd_code((uint8_t)value);
        bool b_expected = (value >= 0xA9U) && (value <= 0xAFU);

        if (MQTTCMD_NONE != code)
        {
            ++valid;
        }
        mqttcmd_test_check(b_expected ?
                           (mqttcmd_byte(code) == (uint8_t)value) :
                           (MQTTCMD_NONE == code),
                           "each byte decodes or is refused as specified");
    }
    mqttcmd_test_check(7U == valid, "exactly seven valid command bytes");
    mqttcmd_test_check(MQTTCMD_STOP_BYTE == mqttcmd_byte(MQTTCMD_STOP),
                       "STOP byte is 0xA9");
    mqttcmd_test_check(MQTTCMD_STOP == mqttcmd_code(0xA9U), "0xA9 is STOP");
    mqttcmd_test_check(MQTTCMD_UTURN == mqttcmd_code(0xAFU), "0xAF is UTURN");
    mqttcmd_test_check(0U == mqttcmd_byte(MQTTCMD_NONE), "NONE has no byte");
    mqttcmd_test_check(MQTTCMD_NONE == mqttcmd_code(0xA8U),
                       "pattern with code 0 rejected");
    mqttcmd_test_check(MQTTCMD_NONE == mqttcmd_code(0x00U), "zero rejected");
    mqttcmd_test_check(MQTTCMD_NONE == mqttcmd_code(0xFFU),
                       "all ones rejected");
    mqttcmd_test_check(MQTTCMD_NONE == mqttcmd_code((uint8_t)'S'),
                       "first byte of the old text STOP rejected");
}

/** @brief Every valid byte, with and without empty surrounding fragments. */
static void mqttcmd_test_fragments (void)
{
    uint32_t code;
    mqttcmd_t state;
    uint8_t byte;
    bool b_done;

    for (code = (uint32_t)MQTTCMD_STOP; code <= (uint32_t)MQTTCMD_UTURN;
         code++)
    {
        /* code is 1..7, a declared mqttcmd_code_t enumerator. */
        byte = mqttcmd_byte((mqttcmd_code_t)code);
        mqttcmd_begin(&state, 1U, true);
        mqttcmd_test_check(mqttcmd_feed(&state, &byte, 1U, true) &&
                           (byte == state.command),
                           "single fragment completes");
        mqttcmd_test_check(!mqttcmd_feed(&state, NULL, 0U, true),
                           "completed command cannot replay");
        mqttcmd_begin(&state, 1U, true);
        b_done = mqttcmd_feed(&state, NULL, 0U, false);
        mqttcmd_test_check(!b_done, "empty leading fragment waits");
        b_done = mqttcmd_feed(&state, &byte, 1U, false);
        mqttcmd_test_check(!b_done, "non-final fragment is incomplete");
        mqttcmd_test_check(mqttcmd_feed(&state, NULL, 0U, true),
                           "empty final fragment completes");
    }
}

/** @brief Reject malformed metadata, wrong lengths and invalid bytes. */
static void mqttcmd_test_invalid (void)
{
    static const uint8_t stop = MQTTCMD_STOP_BYTE;
    static const uint8_t text_stop[] = {'S', 'T', 'O', 'P'};
    static const uint8_t invalid = 0x29U;
    mqttcmd_t state;

    mqttcmd_begin(NULL, 1U, true);
    mqttcmd_test_check(!mqttcmd_feed(NULL, &stop, 1U, true),
                       "NULL state rejected");
    mqttcmd_begin(&state, 1U, false);
    mqttcmd_test_check(!mqttcmd_feed(&state, &stop, 1U, true),
                       "wrong topic/retained metadata rejected");
    mqttcmd_begin(&state, 4U, true);
    mqttcmd_test_check(!mqttcmd_feed(&state, text_stop, 4U, true),
                       "old four-byte text STOP rejected");
    mqttcmd_begin(&state, UINT32_MAX, true);
    mqttcmd_test_check(!mqttcmd_feed(&state, &stop, 1U, true),
                       "oversized declared length rejected");
    mqttcmd_begin(&state, 0U, true);
    mqttcmd_test_check(!mqttcmd_feed(&state, NULL, 0U, true),
                       "empty publication rejected");
    mqttcmd_begin(&state, 1U, true);
    mqttcmd_test_check(!mqttcmd_feed(&state, NULL, 1U, true),
                       "NULL nonempty data rejected");
    mqttcmd_begin(&state, 1U, true);
    mqttcmd_test_check(!mqttcmd_feed(&state, text_stop, 2U, true),
                       "fragment longer than the payload rejected");
    mqttcmd_begin(&state, 1U, true);
    (void)mqttcmd_feed(&state, &stop, 1U, false);
    mqttcmd_test_check(!mqttcmd_feed(&state, &stop, 1U, true),
                       "second byte after the payload rejected");
    mqttcmd_begin(&state, 1U, true);
    mqttcmd_test_check(!mqttcmd_feed(&state, NULL, 0U, true),
                       "missing byte rejected");
    mqttcmd_begin(&state, 1U, true);
    mqttcmd_test_check(!mqttcmd_feed(&state, &invalid, 1U, true),
                       "byte without the pattern rejected");
    mqttcmd_begin(&state, 1U, true);
    (void)mqttcmd_feed(&state, &invalid, 1U, false);
    mqttcmd_begin(&state, 1U, true);
    mqttcmd_test_check(mqttcmd_feed(&state, &stop, 1U, true),
                       "new publication clears the old byte");
}

/* End of file. */

