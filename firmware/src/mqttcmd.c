/**
 * @file mqttcmd.c
 * @brief Exact ASCII commands; no dynamic allocation or implicit trimming.
 * NOTE: Early error returns preserve shallow validation (BARR-C 6.2.c).
 */
#include "mqttcmd.h"

#include <string.h>

/** @brief Exact command vocabulary, checked by byte count and byte value. */
bool mqttcmd_valid (const char * p_text, size_t length)
{
    static const char * const commands[] =
    {
        "START", "STOP", "RESET", "LEFT", "RIGHT", "STRAIGHT", "UTURN"
    };
    size_t index;
    size_t count = sizeof(commands) / sizeof(commands[0]);

    if ((NULL == p_text) || (0U == length) ||
        (length > MQTTCMD_MAX_LENGTH))
    {
        return false;
    }
    for (index = 0U; index < count; index++)
    {
        if ((length == strlen(commands[index])) &&
            (0 == memcmp(p_text, commands[index], length)))
        {
            return true;
        }
    }
    return false;
}

/** @brief Clear old bytes so rejected publications cannot reuse state. */
void mqttcmd_begin (mqttcmd_t * p_state, uint32_t length,
                    bool b_topic_matches)
{
    if (NULL == p_state)
    {
        return;
    }
    memset(p_state, 0, sizeof(*p_state));
    p_state->expected    = length;
    p_state->b_accepting = b_topic_matches && (length > 0U) &&
                           (length <= MQTTCMD_MAX_LENGTH);
}

/** @brief Validate fragment bounds before copying and consuming a command. */
bool mqttcmd_feed (mqttcmd_t * p_state, const uint8_t * p_data,
                   uint32_t length, bool b_last)
{
    if ((NULL == p_state) || (!p_state->b_accepting))
    {
        return false;
    }
    if (((NULL == p_data) && (length > 0U)) ||
        (p_state->used > p_state->expected) ||
        (length > (p_state->expected - p_state->used)))
    {
        p_state->b_accepting = false;
        return false;
    }
    if (length > 0U)
    {
        memcpy(&p_state->text[p_state->used], p_data, length);
        p_state->used += length;
    }
    if (!b_last)
    {
        return false;
    }
    p_state->b_accepting = false;
    if (p_state->used != p_state->expected)
    {
        return false;
    }
    p_state->text[p_state->used] = '\0';
    return mqttcmd_valid(p_state->text, p_state->used);
}

/* End of file. */

