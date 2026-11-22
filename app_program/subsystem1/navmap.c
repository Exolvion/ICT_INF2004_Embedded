/**
 * @file navmap.c
 * @brief Barcode letter -> junction move map message (navmap.h).
 */
#include "navmap.h"

#include <stddef.h>

#define NAVMAP_LETTER_MASK  ((1UL << NAVMAP_LETTER_BITS) - 1UL)
#define NAVMAP_PATTERN_MASK (0xFUL)

/** @brief True if every letter is 'A'-'Z' and none repeats. */
static bool navmap_letters_ok (const char p_letters[NAVMAP_MOVES])
{
    uint32_t move;
    uint32_t other;

    for (move = 0U; move < (uint32_t)NAVMAP_MOVES; ++move)
    {
        if ((p_letters[move] < 'A') || (p_letters[move] > 'Z'))
        {
            return false;
        }
        for (other = 0U; other < move; ++other)
        {
            if (p_letters[other] == p_letters[move])
            {
                return false;
            }
        }
    }
    return true;
}

bool navmap_pack (const char p_letters[NAVMAP_MOVES],
                  uint8_t p_bytes[NAVMAP_LENGTH])
{
    uint32_t word = (uint32_t)NAVMAP_PATTERN << NAVMAP_PATTERN_SHIFT;
    uint32_t move;
    uint32_t index;

    if ((NULL == p_letters) || (NULL == p_bytes) ||
        (!navmap_letters_ok(p_letters)))
    {
        return false;
    }
    for (move = 0U; move < (uint32_t)NAVMAP_MOVES; ++move)
    {
        /* Checked 'A'-'Z' above: 0..25, fits in 5 bits. */
        word |= ((uint32_t)(p_letters[move] - 'A')) <<
                (move * NAVMAP_LETTER_BITS);
    }
    for (index = 0U; index < NAVMAP_LENGTH; ++index)
    {
        p_bytes[index] = (uint8_t)((word >> (8U * index)) & 0xFFU);
    }
    return true;
}

bool navmap_unpack (const uint8_t p_bytes[NAVMAP_LENGTH],
                    char p_letters[NAVMAP_MOVES])
{
    char     letters[NAVMAP_MOVES];
    uint32_t word = 0U;
    uint32_t code;
    uint32_t index;

    if ((NULL == p_bytes) || (NULL == p_letters))
    {
        return false;
    }
    for (index = 0U; index < NAVMAP_LENGTH; ++index)
    {
        word |= ((uint32_t)p_bytes[index]) << (8U * index);
    }
    if (((word >> NAVMAP_PATTERN_SHIFT) & NAVMAP_PATTERN_MASK) !=
        NAVMAP_PATTERN)
    {
        return false;
    }
    for (index = 0U; index < (uint32_t)NAVMAP_MOVES; ++index)
    {
        code = (word >> (index * NAVMAP_LETTER_BITS)) & NAVMAP_LETTER_MASK;
        if (code >= NAVMAP_LETTERS)
        {
            return false;
        }
        /* 0..25: 'A'..'Z'. */
        letters[index] = (char)('A' + (int32_t)code);
    }
    if (!navmap_letters_ok(letters))
    {
        return false;
    }
    for (index = 0U; index < (uint32_t)NAVMAP_MOVES; ++index)
    {
        p_letters[index] = letters[index];
    }
    return true;
}

void navmap_begin (navmap_rx_t * p_state, uint32_t length,
                   bool b_topic_matches)
{
    if (NULL == p_state)
    {
        return;
    }
    p_state->count = 0U;
    p_state->b_accepting = b_topic_matches && (NAVMAP_LENGTH == length);
}

bool navmap_feed (navmap_rx_t * p_state, const uint8_t * p_data,
                  uint32_t length, bool b_last)
{
    char     letters[NAVMAP_MOVES];
    uint32_t index;

    if (NULL == p_state)
    {
        return false;
    }
    if (!p_state->b_accepting)
    {
        return false;
    }
    if (((NULL == p_data) && (0U != length)) ||
        (length > (NAVMAP_LENGTH - (uint32_t)p_state->count)))
    {
        p_state->b_accepting = false;
        return false;
    }
    for (index = 0U; index < length; ++index)
    {
        p_state->bytes[p_state->count] = p_data[index];
        ++p_state->count;
    }
    if (!b_last)
    {
        return false;
    }
    p_state->b_accepting = false;
    return (NAVMAP_LENGTH == (uint32_t)p_state->count) &&
           navmap_unpack(p_state->bytes, letters);
}

/* End of file. */
