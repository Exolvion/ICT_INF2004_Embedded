/**
 * @file navmap.h
 * @brief Barcode letter -> junction move map, a 3-byte binary MQTT message,
 * independent of lwIP.
 * @details The instructor names, just before the demo, which four letters
 * (A-Z) mean the four moves; the PC sends that map to the car on the command
 * topic, so no letter is fixed in the firmware. Minimum-bit (Change
 * Requirement 1): 26 letters need 5 bits, so the four take 20 bits, in move
 * order (bits 0-4 FORWARD, 5-9 U-TURN, 10-14 LEFT, 15-19 RIGHT, each 0 = A
 * .. 25 = Z), and bits 20-23 hold the fixed pattern NAVMAP_PATTERN: 24 bits,
 * sent least significant byte first. A one-byte command (mqttcmd.h) can never
 * be a map, nor a map a command: the lengths differ. A map is valid only
 * with the pattern, every letter in A-Z and the four letters all different.
 */
#ifndef NAVMAP_H
#define NAVMAP_H

#include <stdbool.h>
#include <stdint.h>

#define NAVMAP_LENGTH        (3U)
#define NAVMAP_LETTERS       (26U)   /* A-Z */
#define NAVMAP_LETTER_BITS   (5U)
#define NAVMAP_PATTERN       (0xBU)  /* bits 20-23: 1011b */
#define NAVMAP_PATTERN_SHIFT (20U)

/** The four moves, in their order in the message. */
typedef enum
{
    NAVMAP_FORWARD = 0,
    NAVMAP_UTURN,
    NAVMAP_LEFT,
    NAVMAP_RIGHT,
    NAVMAP_MOVES
} navmap_move_t;

/** One publication being assembled. */
typedef struct
{
    uint8_t bytes[NAVMAP_LENGTH];
    uint8_t count;
    bool    b_accepting;
} navmap_rx_t;

/** @brief Pack four letters ('A'-'Z', move order) into the 3-byte message.
 * False (p_bytes untouched) unless all four are A-Z and different.
 */
bool navmap_pack (const char p_letters[NAVMAP_MOVES],
                  uint8_t p_bytes[NAVMAP_LENGTH]);

/** @brief Unpack a 3-byte message into four letters ('A'-'Z', move order).
 * False (p_letters untouched) for a wrong pattern, a letter past Z or a
 * letter used twice.
 */
bool navmap_unpack (const uint8_t p_bytes[NAVMAP_LENGTH],
                    char p_letters[NAVMAP_MOVES]);

/** @brief Reset assembly; accept only a matching topic and NAVMAP_LENGTH. */
void navmap_begin (navmap_rx_t * p_state, uint32_t length,
                   bool b_topic_matches);

/**
 * Append a fragment. True only when the last fragment completes exactly
 * NAVMAP_LENGTH bytes that unpack to a valid map; p_state->bytes then holds
 * it. Too many bytes or a bad pointer reject the message for good.
 */
bool navmap_feed (navmap_rx_t * p_state, const uint8_t * p_data,
                  uint32_t length, bool b_last);

#endif /* NAVMAP_H */

/* End of file. */

