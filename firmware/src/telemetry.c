/**
 * @file telemetry.c
 * @brief Integer JSON writer with explicit bounds and signed-min handling.
 * NOTE: Early returns preserve shallow validation (BARR-C 6.2.c).
 */
#include "telemetry.h"

#define TELEMETRY_DECIMAL_DIGITS 10U
#define TELEMETRY_DECIMAL_BASE 10U

typedef struct
{
    char * p_text;
    size_t capacity;
    size_t used;
    bool   b_valid;
} telemetry_writer_t;

static void telemetry_character (telemetry_writer_t * p_writer,
                                  char character);
static void telemetry_literal (telemetry_writer_t * p_writer,
                                const char * p_literal);
static void telemetry_unsigned (telemetry_writer_t * p_writer,
                                 uint32_t value);
static void telemetry_signed (telemetry_writer_t * p_writer, int32_t value);
static void telemetry_boolean (telemetry_writer_t * p_writer, bool b_value);

/** @brief A fixed schema and bounded decimal conversions form valid JSON. */
size_t telemetry_format (const telemetry_status_t * p_status,
                         char * p_text, size_t capacity)
{
    telemetry_writer_t writer;

    if ((NULL == p_text) || (0U == capacity))
    {
        return 0U;
    }
    p_text[0] = '\0';
    if (NULL == p_status)
    {
        return 0U;
    }
    writer.p_text   = p_text;
    writer.capacity = capacity;
    writer.used     = 0U;
    writer.b_valid  = true;
    telemetry_literal(&writer, "{\"seq\":");
    telemetry_unsigned(&writer, p_status->seq);
    telemetry_literal(&writer, ",\"ms\":");
    telemetry_unsigned(&writer, p_status->now_ms);
    telemetry_literal(&writer, ",\"drop\":");
    telemetry_unsigned(&writer, p_status->dropped);
    telemetry_literal(&writer, ",\"ign\":");
    telemetry_unsigned(&writer, p_status->ignored);
    telemetry_literal(&writer, ",\"age\":");
    telemetry_unsigned(&writer, p_status->range_age_ms);
    telemetry_literal(&writer, ",\"st\":");
    telemetry_signed(&writer, p_status->state);
    telemetry_literal(&writer, ",\"vl\":");
    telemetry_signed(&writer, p_status->vleft_mm_s);
    telemetry_literal(&writer, ",\"vr\":");
    telemetry_signed(&writer, p_status->vright_mm_s);
    telemetry_literal(&writer, ",\"pt\":");
    telemetry_signed(&writer, p_status->pitch_mrad);
    telemetry_literal(&writer, ",\"pk\":");
    telemetry_signed(&writer, p_status->peak_mm);
    telemetry_literal(&writer, ",\"rg\":");
    telemetry_signed(&writer, p_status->range_mm);
    telemetry_literal(&writer, ",\"stop\":");
    telemetry_boolean(&writer, p_status->b_stop);
    telemetry_literal(&writer, ",\"fault\":");
    telemetry_boolean(&writer, p_status->b_hardfault);
    telemetry_literal(&writer, ",\"hv\":");
    telemetry_boolean(&writer, p_status->b_heightvalid);
    telemetry_literal(&writer, ",\"rv\":");
    telemetry_boolean(&writer, p_status->b_rangevalid);
    telemetry_character(&writer, '}');
    if (!writer.b_valid)
    {
        p_text[0] = '\0';
        return 0U;
    }
    p_text[writer.used] = '\0';
    return writer.used;
}

/** @brief Perception schema uses at most 252 bytes for every integer value. */
size_t telemetry_format_perception (
    const telemetry_perception_t * p_status, char * p_text, size_t capacity)
{
    telemetry_writer_t writer;

    if ((NULL == p_text) || (0U == capacity))
    {
        return 0U;
    }
    p_text[0] = '\0';
    if (NULL == p_status)
    {
        return 0U;
    }
    writer.p_text   = p_text;
    writer.capacity = capacity;
    writer.used     = 0U;
    writer.b_valid  = true;
    telemetry_literal(&writer, "{\"seq\":");
    telemetry_unsigned(&writer, p_status->seq);
    telemetry_literal(&writer, ",\"ms\":");
    telemetry_unsigned(&writer, p_status->now_ms);
    telemetry_literal(&writer, ",\"run\":");
    telemetry_unsigned(&writer, p_status->run_id);
    telemetry_literal(&writer, ",\"cal\":");
    telemetry_unsigned(&writer, p_status->cal_version);
    telemetry_literal(&writer, ",\"ba\":");
    telemetry_signed(&writer, p_status->barcode_action);
    telemetry_literal(&writer, ",\"bl\":");
    telemetry_signed(&writer, p_status->barcode_label);
    telemetry_literal(&writer, ",\"yr\":");
    telemetry_signed(&writer, p_status->yaw_mrad_s);
    telemetry_literal(&writer, ",\"mc\":");
    telemetry_signed(&writer, p_status->motion_class);
    telemetry_literal(&writer, ",\"sw\":");
    telemetry_signed(&writer, p_status->scan_width_mm);
    telemetry_literal(&writer, ",\"sn\":");
    telemetry_signed(&writer, p_status->scan_nearest_mm);
    telemetry_literal(&writer, ",\"ss\":");
    telemetry_signed(&writer, p_status->scan_side);
    telemetry_literal(&writer, ",\"sv\":");
    telemetry_unsigned(&writer, p_status->scan_valid);
    telemetry_literal(&writer, ",\"su\":");
    telemetry_unsigned(&writer, p_status->scan_unknown);
    telemetry_literal(&writer, ",\"tv\":");
    telemetry_boolean(&writer, p_status->b_tilt_valid);
    telemetry_literal(&writer, ",\"ha\":");
    telemetry_boolean(&writer, p_status->b_hump_active);
    telemetry_literal(&writer, ",\"sc\":");
    telemetry_boolean(&writer, p_status->b_scan_valid);
    telemetry_character(&writer, '}');
    if (!writer.b_valid)
    {
        p_text[0] = '\0';
        return 0U;
    }
    p_text[writer.used] = '\0';
    return writer.used;
}

/** @brief Keep diagnostic fields within their own small, bounded schema. */
size_t telemetry_format_diagnostic (
    const telemetry_diagnostic_t * p_status, char * p_text, size_t capacity)
{
    telemetry_writer_t writer;

    if ((NULL == p_text) || (0U == capacity))
    {
        return 0U;
    }
    p_text[0] = '\0';
    if (NULL == p_status)
    {
        return 0U;
    }
    writer.p_text   = p_text;
    writer.capacity = capacity;
    writer.used     = 0U;
    writer.b_valid  = true;
    telemetry_literal(&writer, "{\"seq\":");
    telemetry_unsigned(&writer, p_status->seq);
    telemetry_literal(&writer, ",\"ms\":");
    telemetry_unsigned(&writer, p_status->now_ms);
    telemetry_literal(&writer, ",\"run\":");
    telemetry_unsigned(&writer, p_status->run_id);
    telemetry_literal(&writer, ",\"fc\":");
    telemetry_signed(&writer, p_status->fault_code);
    telemetry_literal(&writer, ",\"fm\":");
    telemetry_unsigned(&writer, p_status->first_fault_ms);
    telemetry_literal(&writer, ",\"bu\":");
    telemetry_unsigned(&writer, p_status->barcode_unknown);
    telemetry_literal(&writer, ",\"br\":");
    telemetry_unsigned(&writer, p_status->barcode_rejected);
    telemetry_literal(&writer, ",\"mp\":");
    telemetry_signed(&writer, p_status->mission_peak_mm);
    telemetry_literal(&writer, ",\"mv\":");
    telemetry_boolean(&writer, p_status->b_mission_valid);
    telemetry_character(&writer, '}');
    if (!writer.b_valid)
    {
        p_text[0] = '\0';
        return 0U;
    }
    p_text[writer.used] = '\0';
    return writer.used;
}

/** @brief Reserve one byte for NUL; capacity is positive by construction. */
static void telemetry_character (telemetry_writer_t * p_writer,
                                  char character)
{
    if (!p_writer->b_valid)
    {
        return;
    }
    if (p_writer->used >= (p_writer->capacity - 1U))
    {
        p_writer->b_valid = false;
        return;
    }
    p_writer->p_text[p_writer->used] = character;
    p_writer->used++;
}

/** @brief Append an internal fixed literal, never untrusted string data. */
static void telemetry_literal (telemetry_writer_t * p_writer,
                                const char * p_literal)
{
    size_t index = 0U;

    while ('\0' != p_literal[index])
    {
        telemetry_character(p_writer, p_literal[index]);
        index++;
    }
}

/** @brief uint32_t needs at most ten decimal digits, including zero. */
static void telemetry_unsigned (telemetry_writer_t * p_writer,
                                 uint32_t value)
{
    char digits[TELEMETRY_DECIMAL_DIGITS];
    size_t count;
    uint32_t remainder;

    for (count = 0U; count < TELEMETRY_DECIMAL_DIGITS; count++)
    {
        remainder = value % TELEMETRY_DECIMAL_BASE;
        /* Remainder is 0..9; '0'..'9' are contiguous representable chars. */
        digits[count] = (char)('0' + remainder);
        value /= TELEMETRY_DECIMAL_BASE;
        if (0U == value)
        {
            count++;
            break;
        }
    }
    while (count > 0U)
    {
        count--;
        telemetry_character(p_writer, digits[count]);
    }
}

/** @brief Convert INT32_MIN without signed negation or signed overflow. */
static void telemetry_signed (telemetry_writer_t * p_writer, int32_t value)
{
    uint32_t magnitude;

    /* C99 signed-to-unsigned conversion is modulo 2^32 for uint32_t. */
    magnitude = (uint32_t)value;
    if (value < 0)
    {
        telemetry_character(p_writer, '-');
        /* Unsigned subtraction is defined modulo 2^32, even for INT32_MIN. */
        magnitude = 0U - magnitude;
    }
    telemetry_unsigned(p_writer, magnitude);
}

/** @brief Emit JSON Boolean literals, independent of C bool storage. */
static void telemetry_boolean (telemetry_writer_t * p_writer, bool b_value)
{
    if (b_value)
    {
        telemetry_literal(p_writer, "true");
    }
    else
    {
        telemetry_literal(p_writer, "false");
    }
}

/* End of file. */

