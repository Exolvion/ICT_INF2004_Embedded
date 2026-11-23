/** @file mission_frames.c
 * @brief The mission: subsystem 1: the mission's minimum-bit frames, text,
 * the line recorder and live lines. Split out of the one file by
 * tools/reorg/split_c.py (22 Nov 2026); what the parts share is in
 * mission_private.h.
 */
#include "mission_private.h"
#include "mission_frames.h"
#define BENCH_FRAME_HEAD (0xBDU)

/* MISSION sends while it drives (bench_live_step): one line at a time through
 * g_live, pushed frame by frame as the MQTT queue takes them, never waiting;
 * each group's result as it is decoded, else a status line every
 * BENCH_LIVE_US. After the stop only a few lines of totals: the link takes
 * about one 20-byte frame per 200 ms, so a raw dump or trace there held the
 * car for minutes (30 Oct).
 */
#define BENCH_LIVE_US (1000000U)
#define BENCH_LIVE_RAW_US (3000000U)

/* Indexed by line_ctrl_status_t. */
const char * const g_line_names[BENCH_LINE_STATES] =
{
    "INHIBITED", "TRACKING", "SEARCHING", "JUNCTION", "LOST", "FAULT"
};
static char          g_live[BENCH_TEXT_BYTES];
   /* a number printed with no width */
static uint8_t          g_frame_seq;
     /* the next frame's seq */
static uint8_t          g_live_frame[BENCH_FRAME_BYTES];
static uint32_t         g_live_frame_size;
 /* bytes, 0: no frame */
static uint32_t         g_frame_sent;
    /* frames this boot */
static uint32_t         g_frame_unsent;
  /* lines with no frame, dropped */
static uint32_t         g_frame_bad;
     /* frames over BENCH_FRAME_BYTES */
static uint32_t         g_frame_sat;
uint16_t      g_live_front[2];
bool gb_rec_keep;

/** @brief MISSION, from the line step: live lines while it drives, never
 * waiting on the radio. The line going out is pushed on; when it is out,
 * the next group's line (bench_code_found_text), else the next line of a
 * cut's samples (bench_raw_text) until the status is due, else every
 * BENCH_LIVE_US (BENCH_LIVE_RAW_US while samples wait) a status line: time,
 * centre path, line state, side sensor now and its darkest since the last
 * status, groups and codes so far.
 */
void bench_live_step (uint64_t now_us, uint32_t status,
                             int32_t centre_mm, uint16_t side)
{
    uint32_t index;
    uint32_t codes = 0U;

    if (side > g_live_side_max)
    {
        g_live_side_max = side;
    }
    bench_live_push();
    if ((g_live_sent < g_live_length) || (0U != g_length))
    {
        return;
    }
    if (bench_avoid_text() || bench_code_found_text() || bench_fit_text() ||
        bench_d0_text() || bench_vote_text() || bench_pose_text(now_us) ||
        bench_ramp_text() ||
        bench_cal_text())
    {
        bench_live_take();
        return;
    }
    if ((now_us < g_live_due_us) && bench_raw_text())
    {
        bench_live_take();
        return;
    }
    if (now_us < g_live_due_us)
    {
        return;
    }
    g_live_due_us = now_us + ((0U != g_raw_length) ? BENCH_LIVE_RAW_US :
                                                     BENCH_LIVE_US);
    codes = g_code.codes_done;   /* groups already dropped (sent) */
    for (index = 0U; index < g_code.founds; ++index)
    {
        if (g_code.found[index].b_framed)
        {
            ++codes;
        }
    }
    /* Minimum bits: time 0.1 s to 6553.5 s (no run limit now: 109 min);
     * centre path +-1048575 mm (over 2 h at the robot's top speed); line
     * state 6 names; ADC 0-4095; groups 0-1023 a run; codes up to
     * 1023 a run; sonar 0-4095 mm; no-echo readings since the last
     * status 0-65535 (none goes out during a go-around: 2903 in 40 s, sim).
     */
    bench_line(BK_LIVE);
    bench_text(" t ");
    bench_ux((uint32_t)((now_us - g_drive.step_us) / 100000U), 1U, 16U);
    bench_text(" s d ");
    bench_i(centre_mm, 21U);
    bench_text(" mm ");
    bench_name(g_line_names, status % BENCH_LINE_STATES, 3U);
    bench_text(" B ");
    bench_u((uint32_t)side, 12U);
    bench_text(" max ");
    bench_u((uint32_t)g_live_side_max, 12U);
    /* The front sensors now (22 Nov: the dashboard's live sensor chart had
     * only the idle report, never anything while the car drove).
     */
    bench_text(" L ");
    bench_u((uint32_t)g_live_front[0], 12U);
    bench_text(" R ");
    bench_u((uint32_t)g_live_front[1], 12U);
    bench_text(" groups ");
    bench_u(g_code.groups_seen, 10U);
    bench_text(" codes ");
    bench_u(codes, 10U);
    bench_text(" F ");
    bench_u((uint32_t)g_live_front_min, 12U);
    bench_text(" x");
    bench_u(g_live_front_none, 16U);
    bench_ramp_status();
    g_live_side_max = 0U;
    g_live_front_min = 0U;
    g_live_front_none = 0U;
    bench_live_take();
}

/** @brief Signed modulo-2^16 difference, with the wheel's encoder sign. */
int32_t bench_signed_delta (uint16_t newer, uint16_t older,
                                   uint32_t wheel)
{
    int32_t difference = (int32_t)(uint16_t)(newer - older);

    if (difference >= 0x8000)
    {
        difference -= 0x10000;
    }
    return difference * ((BENCH_LEFT == wheel) ? ROBOT_LEFT_ENCODER_SIGN :
                                                 ROBOT_RIGHT_ENCODER_SIGN);
}

/** @brief Append p_text to g_text, as much as fits while leaving room for
 * the terminator; g_text is always terminated.
 */
void bench_text (const char * p_text)
{
    while (('\0' != *p_text) && ((g_length + 2U) < BENCH_TEXT_BYTES))
    {
        g_text[g_length] = *p_text;
        ++g_length;
        ++p_text;
    }
    g_text[g_length] = '\0';
}

/** @brief Signed decimal into g_text; unsigned negation also covers
 * INT32_MIN.
 */
static void bench_dec_text (int32_t value)
{
    char     digits[12];
    uint32_t index = sizeof(digits) - 1U;
    uint32_t magnitude = (value < 0) ? (0U - (uint32_t)value) :
                         (uint32_t)value;

    digits[index] = '\0';
    do
    {
        --index;
        digits[index] = (char)('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while ((0U != magnitude) && (index > 1U));
    if (value < 0)
    {
        --index;
        digits[index] = '-';
    }
    bench_text(&digits[index]);
}

/** @brief Decimal with an explicit sign, into g_text. */
static void bench_signed_text (int32_t value)
{
    if (value >= 0)
    {
        bench_text("+");
    }
    bench_dec_text(value);
}

/** @brief value / 10^places (1 or 2) with that many decimals, into g_text. */
static void bench_fixed_text (int32_t value, uint32_t places)
{
    uint32_t scale = (1U == places) ? 10U : 100U;
    uint32_t magnitude = (value < 0) ? (0U - (uint32_t)value) :
                         (uint32_t)value;
    uint32_t fraction = magnitude % scale;

    if (value < 0)
    {
        bench_text("-");
    }
    bench_dec_text((int32_t)(magnitude / scale));
    bench_text(".");
    if ((100U == scale) && (fraction < 10U))
    {
        bench_text("0");
    }
    bench_dec_text((int32_t)fraction);
}

/** @brief Signed decimal, text only (a line using it is never sent). */
void bench_dec (int32_t value)
{
    bench_untyped();
    bench_dec_text(value);
}

/** @brief Decimal with an explicit sign, text only. */
void bench_signed (int32_t value)
{
    bench_untyped();
    bench_signed_text(value);
}

/** @brief value / 10^places with that many decimals, text only. */
void bench_fixed (int32_t value, uint32_t places)
{
    bench_untyped();
    bench_fixed_text(value, places);
}

/** @brief A new line of this kind: its frame's header (seq set at
 * sending) and kind. Text build: nothing.
 */
void bench_line (bench_kind_t kind)
{
    bitpack_writer_init(&g_frame_w, g_frame, sizeof(g_frame));
    bitpack_write(&g_frame_w, BENCH_FRAME_HEAD, 8U);
    bitpack_write(&g_frame_w, 0U, 8U);
    bitpack_write(&g_frame_w, (uint32_t)kind, BENCH_FRAME_KIND_BITS);
    g_frame_kind = (uint32_t)kind;
    gb_frame_text = false;
}

/** @brief One field of the line's frame, already in its bits. */
void bench_frame_put (uint32_t field, uint32_t bits)
{
    if (BK_NONE != g_frame_kind)
    {
        bitpack_write(&g_frame_w, field, bits);
    }
}

/** @brief value in bits unsigned bits, saturated (counted). */
uint32_t bench_frame_unsigned (uint32_t value, uint32_t bits)
{
    uint32_t largest = bitpack_mask(bits);

    if (value > largest)
    {
        ++g_frame_sat;
        return largest;
    }
    return value;
}

/** @brief value as a bits-bit two's complement field, saturated at
 * +/-(2^(bits-1)-1) (counted); the most negative code is never made.
 */
static uint32_t bench_frame_signed (int32_t value, uint32_t bits)
{
    int32_t largest = bitpack_signed_max(bits);

    if ((value > largest) || (value < -largest))
    {
        ++g_frame_sat;
    }
    return bitpack_signed(value, bits);
}

/** @brief value as a bits-bit unsigned frame field (saturated, counted)
 * and as decimal text.
 */
void bench_u (uint32_t value, uint32_t bits)
{
    bench_frame_put(bench_frame_unsigned(value, bits), bits);
    bench_dec_text((int32_t)value);
}

void bench_i (int32_t value, uint32_t bits)
{
    bench_frame_put(bench_frame_signed(value, bits), bits);
    bench_dec_text(value);
}

void bench_s (int32_t value, uint32_t bits)
{
    bench_frame_put(bench_frame_signed(value, bits), bits);
    bench_signed_text(value);
}

void bench_x (int32_t value, uint32_t places, uint32_t bits)
{
    bench_frame_put(bench_frame_signed(value, bits), bits);
    bench_fixed_text(value, places);
}

/** @brief As bench_u, but the text is value / 10^places with that many
 * decimals.
 */
void bench_ux (uint32_t value, uint32_t places, uint32_t bits)
{
    bench_frame_put(bench_frame_unsigned(value, bits), bits);
    bench_fixed_text((int32_t)value, places);
}

bool bench_flag (bool b_value)
{
    bench_frame_put(b_value ? 1U : 0U, 1U);
    return b_value;
}

void bench_name (const char * const * pp_names, uint32_t index,
                        uint32_t bits)
{
    bench_frame_put(bench_frame_unsigned(index, bits), bits);
    bench_text(pp_names[index]);
}

/** @brief Decoded text: its length (length_bits), then 6 bits a character,
 * the index in g_code39_chars (44 characters), BENCH_CODE_CHARS + 1 for
 * '?' (an unread character). Printed as it is.
 */
void bench_str (const char * p_text, uint32_t length_bits)
{
    uint32_t length = 0U;
    uint32_t index;
    uint32_t code;

    while ((length < bitpack_mask(length_bits)) && ('\0' != p_text[length]))
    {
        ++length;
    }
    bench_code(length, length_bits);
    for (index = 0U; index < length; ++index)
    {
        code = BENCH_CODE_CHARS;
        while ((code > 0U) && (g_code39_chars[code - 1U] != p_text[index]))
        {
            --code;
        }
        /* code: 1 + its index, or 0: not a Code 39 character ('?'). */
        bench_code((0U == code) ? (BENCH_CODE_CHARS + 1U) : (code - 1U), 6U);
    }
    bench_text(p_text);
}

/** @brief p_text as its index in pp_names (bits); a text not in the table
 * has no code, so the line is not sent (bench_untyped).
 */
void bench_name_of (const char * const * pp_names, uint32_t count,
                           const char * p_text, uint32_t bits)
{
    uint32_t index = 0U;

    while ((index < count) && (0 != strcmp(pp_names[index], p_text)))
    {
        ++index;
    }
    if (index < count)
    {
        bench_name(pp_names, index, bits);
        return;
    }
    bench_untyped();
    bench_text(p_text);
}

/** @brief One character from a known range: ch - base in bits bits. */
void bench_char (char ch, char base, uint32_t bits)
{
    char text[2];

    text[0] = ch;
    text[1] = '\0';
    bench_code((uint32_t)(uint8_t)ch - (uint32_t)(uint8_t)base, bits);
    bench_text(text);
}

/** @brief End the line's frame: its size in bytes, or 0 when the line has
 * no frame (no kind, or a number with no width: counted in
 * g_frame_unsent) or overflowed BENCH_FRAME_BYTES (g_frame_bad). Either way
 * the next line starts with no kind.
 */
static uint32_t bench_frame_close (void)
{
    uint32_t size = 0U;

    if ((BK_NONE == g_frame_kind) || gb_frame_text)
    {
        ++g_frame_unsent;
    }
    else
    {
        size = (uint32_t)bitpack_writer_size(&g_frame_w);
        if (0U == size)
        {
            ++g_frame_bad;
        }
    }
    g_frame_kind = BK_NONE;
    gb_frame_text = false;
    return size;
}
#if ROBOT_MQTT

/** @brief Queue one frame for the radio with the next seq; false if the
 * queue is full (the caller may try again: the seq is not used up).
 */
static bool bench_frame_send (uint8_t * p_frame, uint32_t size)
{
    p_frame[1] = g_frame_seq;
    if (!net_mqtt_ipc_send(p_frame, size))
    {
        return false;
    }
    ++g_frame_seq;
    ++g_frame_sent;
    return true;
}
#endif
#if !ROBOT_MQTT

/** @brief Offline and the sim: the frame in hex ("#F"), then its text, so a
 * run checks every frame against the line it stands for
 * (bench_codec.py check).
 */
static void bench_frame_print (uint8_t * p_frame, uint32_t size,
                               const char * p_text)
{
    static const char p_hex[] = "0123456789abcdef";
    char     line[(2U * BENCH_FRAME_BYTES) + 6U];
    uint32_t index;

    if (0U == size)
    {
        (void)MISSION_PRINTF((const UB *)"#T %s", p_text);
        return;
    }
    p_frame[1] = g_frame_seq;
    ++g_frame_seq;
    ++g_frame_sent;
    line[0] = '#';
    line[1] = 'F';
    line[2] = ' ';
    for (index = 0U; index < size; ++index)
    {
        line[3U + (2U * index)] = p_hex[p_frame[index] >> 4];
        line[4U + (2U * index)] = p_hex[p_frame[index] & 0x0FU];
    }
    line[3U + (2U * size)] = '\n';
    line[4U + (2U * size)] = '\0';
    (void)MISSION_PRINTF((const UB *)"%s%s", line, p_text);
}
#endif

/** @brief MISSION: the line built in g_text becomes the live line (g_live),
 * its first frames pushed at once.
 */
void bench_live_take (void)
{
    g_live_frame_size = bench_frame_close();
    (void)memcpy(g_live_frame, g_frame, g_live_frame_size);
    (void)memcpy(g_live, g_text, g_length);
    g_live[g_length] = '\n';
    g_live_length = g_length + 1U;
    g_live_sent = 0U;
    g_length = 0U;
#if ROBOT_MQTT
    if (0U == g_live_frame_size)
    {
        g_live_length = 0U;     /* no frame: nothing to send */
    }
#endif
    bench_live_push();
}

/** @brief MISSION: as much of the live line as the MQTT queue takes now,
 * without waiting (a frame, or text frames in the text build). Offline:
 * all of it to UART0.
 */
void bench_live_push (void)
{
#if ROBOT_MQTT
    if (gb_mqtt && (g_live_sent < g_live_length) &&
        bench_frame_send(g_live_frame, g_live_frame_size))
    {
        g_live_sent = g_live_length;
    }
#else
    if (g_live_sent < g_live_length)
    {
        g_live[g_live_length] = '\0';
        bench_frame_print(g_live_frame, g_live_frame_size, g_live);
        g_live_sent = g_live_length;
    }
#endif
}

/** @brief Before any other line: the live line's last frames, with the
 * waits of bench_flush, so lines never interleave; then no live line.
 */
void bench_live_finish (void)
{
    uint32_t tries = 0U;

    bench_live_push();
    while ((g_live_sent < g_live_length) && (tries < g_send_tries))
    {
        bench_tick();
        bench_live_push();
        ++tries;
    }
    g_live_length = 0U;
    g_live_sent = 0U;
}

/** @brief Send one line, after any live line, with bounded waits that keep
 * the watchdog fed; a line the queue cannot take is dropped. MQTT: its frame
 * (text build: the text in NET_MQTT_IPC_TX_MAX pieces). Offline: UART0.
 */
void bench_flush (void)
{
#if ROBOT_MQTT
    uint32_t tries;
#endif
    uint32_t size;

    if (0U != g_live_length)
    {
        bench_live_finish();
    }
    size = bench_frame_close();
    g_text[g_length] = '\n';
    ++g_length;
#if ROBOT_MQTT
    tries = 0U;
    while (gb_mqtt && (0U != size) && !bench_frame_send(g_frame, size))
    {
        ++tries;
        if (tries >= g_send_tries)
        {
            break;
        }
        bench_tick();
    }
#else
    g_text[g_length] = '\0';
    bench_frame_print(g_frame, size, g_text);
#endif
    mission_watchdog_feed();
    g_length = 0U;
    g_text[0] = '\0';
}

/* ==== MISSION line recorder (19 Nov 2026) ================================
 * The car lost the line at the same place four runs running (the dome's
 * first right bend, after a left one: it turned LEFT there, then spun right
 * searching), and nothing it sends while driving says what the front
 * sensors and the steering did. While LINE smooth steers (bench_centre_steer)
 * a sample every BENCH_REC_US goes into a ring of the last BENCH_REC_N
 * (5 s); after the stop they go out as REC lines, oldest first. Recording
 * only: nothing here changes the driving. Per sample, comma-separated:
 *   d   path (wheels' centre, mm)       h  heading (deg, + left, as P lines)
 *   L R front darkness, raw (%)         l r  as the law sees them (gated, %)
 *   p   phase 0 TRACK 1 GAP 2 SEARCH 3 CROSS
 *   c   steering after the slew (%, + right)
 *   t   steering averaged over 40 mm (%, the bend test's g_centre.turn)
 *   s   side the line was last seen (+1 right, -1 left)
 *   n   side seeing it now (seen_side)
 *   f   flags: 1 junction (both dark), 2 going straight, 4 the keep-turning
 *       rule acted (both about as dark mid-turn: turn on the same way),
 *       8 a side lean open, 16 a rise open, 32 the steering held straight
 *       over a hump, 64 a junction probe (BENCH_NAV_PROBE_*)
 *   B   side sensor darkness (%)
 *   r   roll, q pitch (0.1 deg, + left up / nose up), as the ramp uses them
 * 22 Nov: B, r, q and flags 8-64 added and the ring doubled to 10 s (500,
 * 12 kB): the hump was missed (12 leans under 33 mm, "jolts") and read
 * 6 mm, and a turn was taken for a junction; the 1 s status line could not
 * show either.
 */
#define BENCH_REC_N (500U)
#define BENCH_REC_US (20000U)
#define BENCH_REC_PER_LINE (3U)   /* a frame holds 3 (about 49 B) */

typedef struct
{
    int32_t d_mm;       /* was int16: wrapped past 32.7 m, no run limit now */
    int16_t h_deg;
    uint8_t raw_l;
    uint8_t raw_r;
    uint8_t gate_l;
    uint8_t gate_r;
    uint8_t phase;
    int8_t  cmd;
    int8_t  turn;
    int8_t  side;
    int8_t  seen;
    uint8_t flags;
    uint8_t side_pct;   /* B */
    int16_t roll_dd;    /* r, 0.1 deg */
    int16_t pitch_dd;   /* q, 0.1 deg */
} bench_rec_t;

static bench_rec_t g_rec[BENCH_REC_N];
static uint32_t    g_rec_head;
   /* the next slot to write */
static uint32_t    g_rec_n;
      /* slots filled */
static uint64_t    g_rec_us;

     /* the last sample's time */

/** @brief A value 0..1 as a whole percentage, 0..100. */
static uint8_t bench_rec_pct (double value)
{
    return (uint8_t)bench_round(100.0 * fmax(0.0, fmin(1.0, value)));
}

/** @brief A value -1..1 as a whole signed percentage. */
static int8_t bench_rec_spct (double value)
{
    return (int8_t)bench_round(100.0 * fmax(-1.0, fmin(1.0, value)));
}

/** @brief MISSION start: an empty ring. */
void bench_rec_reset (void)
{
    g_rec_head = 0U;
    g_rec_n = 0U;
    g_rec_us = 0U;
    gb_rec_keep = false;
}

/** @brief One sample every BENCH_REC_US, from bench_centre_steer's step. */
void bench_rec_sample (uint64_t now_us, double raw_left,
                              double raw_right, double left, double right,
                              bool b_straight)
{
    bench_rec_t * p_rec;

    if ((0U != g_rec_n) && ((now_us - g_rec_us) < BENCH_REC_US))
    {
        return;
    }
    g_rec_us = now_us;
    p_rec = &g_rec[g_rec_head];
    p_rec->d_mm = bench_round(g_centre.path_m * 1000.0);
    p_rec->h_deg = (int16_t)bench_round(g_drive.yaw * 180.0 / CORE_PI);
    p_rec->raw_l = bench_rec_pct(raw_left);
    p_rec->raw_r = bench_rec_pct(raw_right);
    p_rec->gate_l = bench_rec_pct(left);
    p_rec->gate_r = bench_rec_pct(right);
    p_rec->phase = g_centre.phase;
    p_rec->cmd = bench_rec_spct(g_centre.cmd);
    p_rec->turn = bench_rec_spct(g_centre.turn);
    p_rec->side = (int8_t)g_centre.side;
    p_rec->seen = (int8_t)g_centre.seen_side;
    p_rec->flags = (uint8_t)((g_centre.b_both_dark ? 1U : 0U) |
                             (b_straight ? 2U : 0U) |
                             (gb_rec_keep ? 4U : 0U) |
                             (g_ramp.b_leaning ? 8U : 0U) |
                             (g_ramp.b_rising ? 16U : 0U) |
                             (g_ramp.b_hump_held ? 32U : 0U) |
                             (g_nav.b_probe ? 64U : 0U));
    p_rec->side_pct = bench_rec_pct(g_centre.side_dark);
    p_rec->roll_dd = (int16_t)(g_ramp.b_ready ?
                               bench_round(g_ramp.roll * 1800.0 / CORE_PI) : 0);
    p_rec->pitch_dd = (int16_t)(g_ramp.b_ready ?
                                bench_round(g_ramp.pitch * 1800.0 /
                                            CORE_PI) : 0);
    g_rec_head = (g_rec_head + 1U) % BENCH_REC_N;
    if (g_rec_n < BENCH_REC_N)
    {
        ++g_rec_n;
    }
}

/** @brief After a MISSION stop: the ring as REC lines, oldest first.
 * Frames: d +-1048575 mm (21 bits, the run has no length limit), h +-180
 * deg (9), the four darkness 0-100 % (7 each), phase 0-3 (2), steering
 * +-100 % (8 each), sides -1..+1 (2 each), flags 0-127 (7), B 0-100 % (7),
 * roll and pitch +-204.7 deg in tenths (12 each): 118 bits a sample, at
 * most BENCH_REC_PER_LINE (3) a frame, their count first (2 bits); row
 * numbers 0-255 (8 bits: 500 samples are 167 rows).
 */
void bench_rec_text (void)
{
    const bench_rec_t * p_rec;
    uint32_t            first = (g_rec_head + BENCH_REC_N - g_rec_n) %
                                BENCH_REC_N;
    uint32_t            index;
    uint32_t            line = 0U;
    uint32_t            count;

    if (0U == g_rec_n)
    {
        return;
    }
    bench_line(BK_REC_HEAD);
    bench_text("REC: last ");
    bench_u(g_rec_n, 9U);
    bench_text(" samples, one per 20 ms, before the stop:"
               " d,h,L,R,l,r,p,c,t,s,n,f,B,r,q");
    bench_flush();
    for (index = 0U; index < g_rec_n; ++index)
    {
        if (0U == (index % BENCH_REC_PER_LINE))
        {
            count = g_rec_n - index;
            if (count > BENCH_REC_PER_LINE)
            {
                count = BENCH_REC_PER_LINE;
            }
            bench_line(BK_REC_ROW);
            bench_text("REC ");
            bench_u(line, 8U);
            bench_text(":");
            bench_code(count, 2U);
            ++line;
        }
        p_rec = &g_rec[(first + index) % BENCH_REC_N];
        bench_text(" ");
        bench_i(p_rec->d_mm, 21U);
        bench_text(",");
        bench_i((int32_t)p_rec->h_deg, 9U);
        bench_text(",");
        bench_u((uint32_t)p_rec->raw_l, 7U);
        bench_text(",");
        bench_u((uint32_t)p_rec->raw_r, 7U);
        bench_text(",");
        bench_u((uint32_t)p_rec->gate_l, 7U);
        bench_text(",");
        bench_u((uint32_t)p_rec->gate_r, 7U);
        bench_text(",");
        bench_u((uint32_t)p_rec->phase, 2U);
        bench_text(",");
        bench_i((int32_t)p_rec->cmd, 8U);
        bench_text(",");
        bench_i((int32_t)p_rec->turn, 8U);
        bench_text(",");
        bench_i((int32_t)p_rec->side, 2U);
        bench_text(",");
        bench_i((int32_t)p_rec->seen, 2U);
        bench_text(",");
        bench_u((uint32_t)p_rec->flags, 7U);
        bench_text(",");
        bench_u((uint32_t)p_rec->side_pct, 7U);
        bench_text(",");
        bench_i((int32_t)p_rec->roll_dd, 12U);
        bench_text(",");
        bench_i((int32_t)p_rec->pitch_dd, 12U);
        if ((BENCH_REC_PER_LINE - 1U) == (index % BENCH_REC_PER_LINE))
        {
            bench_flush();
        }
    }
    if (0U != g_length)
    {
        bench_flush();
    }
    bench_line(BK_REC_END);
    bench_text("REC end");
    bench_flush();
}

/** @brief End report, robot app (SYS-12 evidence): each task's longest step
 * (us, 0-65535) and skipped periods (0-4095) since this run's START. Frame:
 * the eight pairs in app_main.c's task order. A step still under way is not
 * in it yet (the Vehicle task's own, which sends this line). Then the IMU
 * task's evidence (robot_app_imu_evidence_t): failed reads (and the last
 * one's wall time: near ROBOT_I2C_TIMEOUT_US is a timeout, short a refused
 * transfer), saturated reads, the longest gap between fresh samples, the
 * slowest read, and the samples terrain refused, late (past its 50 ms
 * bound) or otherwise.
 */
void bench_tasks_text (void)
{
    static const char * const p_names[ROBOT_APP_TASKS] =
    {
        "Safety ", ", Motion ", ", Line ", ", Vehicle ", ", IMU ",
        ", Scan ", ", Comm ", ", Telemetry "
    };
    static const char * const p_owners[ROBOT_APP_OWNERS] =
    {
        "Motion ", " ms, Line ", " ms, Vehicle ", " ms, IMU ", " ms, Scan ",
        " ms, Comm "
    };
    robot_app_task_time_t      timing[ROBOT_APP_TASKS];
    robot_app_imu_evidence_t   imu;
    robot_app_owner_evidence_t owners;
    uint32_t                   index;
    uint32_t                   window_ms;
    uint32_t                   share;
    uint32_t                   mean_us;

    /* Each owner's longest health gap: over its gate is OWNER_TIMEOUT. */
    robot_app_owner_evidence(&owners);
    bench_line(BK_OWNERS);
    bench_text(" owner health, longest gap since START: ");
    for (index = 0U; index < ROBOT_APP_OWNERS; ++index)
    {
        bench_text(p_owners[index]);
        bench_u(owners.longest_ms[index], 16U);
    }
    bench_text(" ms");
    bench_flush();
    robot_app_imu_evidence(&imu);
    bench_line(BK_IMU);
    bench_text(" IMU since START: failed reads ");
    bench_u(imu.failed, 8U);
    bench_text(" (the last took ");
    bench_u(imu.failed_us, 16U);
    bench_text(" us), saturated ");
    bench_u(imu.saturated, 8U);
    bench_text(", longest gap ");
    bench_u(imu.longest_ms, 16U);
    bench_text(" ms, slowest read ");
    bench_u(imu.slowest_us, 16U);
    bench_text(" us; terrain refused ");
    bench_u(imu.late, 8U);
    bench_text(" late, ");
    bench_u(imu.refused, 8U);
    bench_text(" other");
    bench_flush();
    robot_app_timing(timing);
    bench_line(BK_TASKS);
    bench_text(" tasks, longest step us/skipped periods: ");
    for (index = 0U; index < ROBOT_APP_TASKS; ++index)
    {
        bench_text(p_names[index]);
        bench_u((timing[index].worst_us > 65535U) ? 65535U :
                timing[index].worst_us, 16U);
        bench_text("/");
        bench_u((timing[index].skipped > 4095U) ? 4095U :
                timing[index].skipped, 12U);
    }
    bench_flush();
    /* Where the CPU went (23 Nov): each task's own time (other tasks' steps
     * inside its steps taken out) as a share of the window, and its mean
     * step. What the eight leave is the Wi-Fi task, interrupts outside the
     * steps, the kernel and idle.
     */
    window_ms = robot_app_timing_window_ms();
    bench_line(BK_CPU);
    bench_text(" tasks, own CPU per mille/mean step us: ");
    for (index = 0U; index < ROBOT_APP_TASKS; ++index)
    {
        share = (0U != window_ms) ? (timing[index].own_us / window_ms) : 0U;
        mean_us = (0U != timing[index].steps) ?
                  (timing[index].own_us / timing[index].steps) : 0U;
        bench_text(p_names[index]);
        bench_u((share > 1023U) ? 1023U : share, 10U);
        bench_text("/");
        bench_u((mean_us > 65535U) ? 65535U : mean_us, 16U);
    }
    bench_text("; over ");
    bench_u((window_ms > 1048575U) ? 1048575U : window_ms, 20U);
    bench_text(" ms");
    bench_flush();
}

/* End of file. */

