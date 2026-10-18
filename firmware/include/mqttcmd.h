/**
 * @file mqttcmd.h
 * @brief Bounded, exact-token MQTT command assembly independent of lwIP.
 */
#ifndef MQTTCMD_H
#define MQTTCMD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MQTTCMD_MAX_LENGTH 32U

typedef struct
{
    char     text[MQTTCMD_MAX_LENGTH + 1U];
    uint32_t expected;
    uint32_t used;
    bool     b_accepting;
} mqttcmd_t;

/** Reject NULL, empty, whitespace, NUL, suffixes and unsupported tokens. */
bool mqttcmd_valid (const char * p_text, size_t length);

/** Reset assembly; accept only a matching topic and 1..32 payload bytes. */
void mqttcmd_begin (mqttcmd_t * p_state, uint32_t length,
                    bool b_topic_matches);

/**
 * Append a fragment. Return true only for one exact complete valid command.
 * Invalid lengths/pointers permanently reject the current message. A final
 * empty fragment is allowed only after all declared bytes were delivered.
 * On success text is NUL terminated; a later feed cannot repeat the command.
 */
bool mqttcmd_feed (mqttcmd_t * p_state, const uint8_t * p_data,
                   uint32_t length, bool b_last);

#endif /* MQTTCMD_H */

/* End of file. */

