/** @file mission_barcode.c
 * @brief The mission: subsystem 3: the barcode store and decoders (Decoder
 * 1, Decoder 2, D0) and their vote. Split out of the one file by
 * tools/reorg/split_c.py (22 Nov 2026); what the parts share is in
 * mission_private.h.
 */
#include "mission_private.h"
#include "mission_barcode.h"
#if defined(MISSION_SIM)
#include <stdlib.h>   /* getenv: the sim's SIM_FIT_SLICE_US */
#endif

/* BARCODE read. A sample per robot decoder unit, 8 half-counts (robot_app.c
 * ROBOT_APP_BARCODE_UNIT_HC, 0.32 mm): 2560 units cover the 0.8 m limit.
 * Each is the mean of the side sensor's readings in that unit, one per
 * kernel tick (CNF_TIMER_PERIOD, 1 ms): a cyclic handler wakes the bench
 * task, the fastest it can read and still wait. It runs at priority 1, the
 * radio at 9, so a busy loop between ticks would stop MQTT. A wake-up that
 * never comes ends the sleep after BENCH_CODE_POLL_TMO_MS.
 */
#define BENCH_CODE_POLL_MS (1U)
#define BENCH_CODE_UNIT_HC (8)
#define BENCH_CODE_GROUPS (6U)          /* dark groups reported */
#define BENCH_CODE_RUNS_PER_ROW (20U)
#define BENCH_CODE_TRACE_PER_ROW (25U)
#define BENCH_CODE_TRACE_MAX_MM (900)
#define BENCH_CODE_GROUP_GAP_MM (20.0)  /* white that separates dark groups */
#define BENCH_CODE_CONTRAST (1000U)     /* raw max - min worth a second pass */
#define BENCH_CODE_OWN_ON (0.55)
#define BENCH_CODE_OWN_OFF (0.40)
#define BENCH_CODE_ELEMENTS (9U)        /* per Code 39 character */
#define BENCH_CODE_FRAMED (3U)  /* '*', a character, '*': the least framed */
#define BENCH_CODE_WIDE (3U)        /* wide elements per character */
#define BENCH_CODE_DIRECTIONS (2U)  /* read forward, then reversed */
#define BENCH_CODE_KINDS (2U)       /* bars, then spaces */
#define BENCH_FIT_WIDE (6U)  /* wide bars in '*', X, '*': two each */

/* One-* partial frames (bench_code_found_window), off: replayed on the 30 Oct
 * 21:51 cuts, the big *A* card's second pass read D* end to start, a wrong
 * code (the same group read *A in order with the windows tried the other
 * way round).
 */
#define BENCH_CODE_PARTIAL (0)

/* The card's own levels: a print can read far lighter than the tape the
 * side sensor is calibrated on (29 Oct track card: a wide bar 2085 still,
 * the tape 4043). Shares of the calibrated white-to-tape span or of the
 * card's own contrast; the line length is the spec's (bars at most 9 mm,
 * the line 18 mm).
 */
#define BENCH_CODE_FLOOR (0.10)    /* a bar peaks this far above white */
#define BENCH_CODE_NOISE (0.025)   /* a smaller swing is noise */
#define BENCH_CODE_SWING (0.20)    /* of the card's contrast: bar to space */
#define BENCH_CODE_LINE_M (0.013)  /* tape-dark this long is a line */

/* Fourth pass, darkness (bench_code_dark): white is the run's commonest
 * side sensor reading and its noise comes from the readings below it (a
 * mark only reads darker, so that half is white alone): the level
 * BENCH_CODE_ONE_SD_SHARE of the way up it lies one standard deviation
 * under white (2 x 15.87 % of a normal). Turning points need
 * BENCH_CODE_DARK_SDS of those. The spec's 3 mm bars read narrower than the
 * side sensor's view and its 9 mm bars hardly wider, but far darker, so a
 * bar's darkness summed across it tells wide from narrow.
 */
#define BENCH_CODE_ADC_LEVELS (4096U)
#define BENCH_CODE_ONE_SD_SHARE (0.3173)
#define BENCH_CODE_DARK_SDS (4U)
#define BENCH_CODE_DARK_PER_ROW (10U)

/* MISSION: the store streams (bench_code_stream). The floor that opens a
 * group, white + BENCH_CODE_DARK_SDS sd, is renewed every
 * BENCH_CODE_FLOOR_EVERY stored samples; white with no group open is cut
 * back to the last BENCH_CODE_QUIET_MM once BENCH_CODE_STREAM_HOLD
 * samples are stored. The first BENCH_CODE_FOUND groups of a character or
 * more are kept for the report.
 */
#define BENCH_CODE_FLOOR_EVERY (64U)
#define BENCH_CODE_STREAM_HOLD (512U)

/* MISSION decodes each cut at turning-point swings of BENCH_CODE_TRY_MIN to
 * BENCH_CODE_TRY_MAX half sds (bench_code_try): the noise is a whole
 * number of ADC steps from the run so far (3 or 4 on 30 Oct), and a sheet
 * that is not flat splits bars at small swings while large ones lose the
 * 3 mm bars.
 */
#define BENCH_CODE_TRY_MIN (6U)

/* A cut's framed text is a code only when tries at BENCH_CODE_AGREE
 * different swings frame the same text (bench_code_tries_step): on 30 Oct
 * at 180 mm/s one swing alone read the expanded card *F*. Good 80 mm/s
 * reads frame at 4 to 7 swings.
 */
#define BENCH_CODE_AGREE (2U)

/* A second opinion (bench_code39_dark): when the darkness tries frame a text
 * but do not agree, the same tries again with each bar judged by its peak
 * and each space by its valley, which, unlike darkness, do not depend on the
 * valleys beside a bar. Each rule and swing is one vote; a code needs
 * BENCH_CODE_AGREE votes, one of them darkness. Widths were no second
 * opinion: 31 Oct on the recorded cuts and traces they framed only the big
 * cards and missed the normal-size *Y*, whose bars read 2 to 5 mm wide.
 */
#define BENCH_CODE_RULE_DARK (0U)
#define BENCH_CODE_RULE_PEAKS (1U)
#define BENCH_CODE_DARK_VOTES ((1U << BENCH_CODE_SWINGS) - 1U)

/* Code 39 spec checks on a framed text (bench_code39_spec), from the spec
 * alone: every character is 6 narrow and 3 wide elements, wide 2 to 3
 * narrow, so 12 to 15 narrow widths; the gap between characters is at most
 * 3 narrow widths and the quiet zone at least 10. Measured from the
 * frame's own character widths, whole characters, which the sensor's blur
 * shortens alike (its single elements it does not: 31 Oct normal cards
 * read wide bars 4 to 5 mm, narrow 2 to 4, so no element ratio is checked).
 */
#define BENCH_CODE_CHAR_MIN_X (12.0)  /* narrow widths per character, 2:1 */
#define BENCH_CODE_CHAR_MAX_X (15.0)  /* and at 3:1 */
#define BENCH_CODE_GAP_MAX_X (3.0)
#define BENCH_CODE_QUIET_X (10.0)
#define BENCH_SPEC_OK (0U)            /* bench_code39_spec results */
#define BENCH_SPEC_WIDTHS (1U)
#define BENCH_SPEC_GAPS (2U)
#define BENCH_SPEC_QUIET (3U)
#define BENCH_SPEC_RESULTS (4U)
#define BENCH_D0_WIDTHS (36U)  /* widths on one live line (160 bytes) */
#define BENCH_D0_BARS_MIN (3U) /* fewer: a line crossed, counted, not sent */

/* Code 39: nine elements bar, space, ... bar, first element in bit 8, 1 for
 * wide (Barcode Sample: *A* and *Z*). Indexed as g_code39_chars.
 */
const char g_code39_chars[BENCH_CODE_CHARS + 1U] =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. *$/+%";
static const uint16_t g_code39_patterns[BENCH_CODE_CHARS] =
{
    0x034U, 0x121U, 0x061U, 0x160U, 0x031U, 0x130U, 0x070U, 0x025U,
    0x124U, 0x064U, 0x109U, 0x049U, 0x148U, 0x019U, 0x118U, 0x058U,
    0x00DU, 0x10CU, 0x04CU, 0x01CU, 0x103U, 0x043U, 0x142U, 0x013U,
    0x112U, 0x052U, 0x007U, 0x106U, 0x046U, 0x016U, 0x181U, 0x0C1U,
    0x1C0U, 0x091U, 0x190U, 0x0D0U, 0x085U, 0x184U, 0x0C4U, 0x094U,
    0x0A8U, 0x0A2U, 0x08AU, 0x02AU
};

/* Why bench_code39_spec refused a frame, by its result. */
static const char * const g_spec_names[BENCH_SPEC_RESULTS] =
{
    "", "character widths", "gap between characters", "quiet zone"
};

/* The robot decoder's DARK/LIGHT fractions (barcode.h); its codebook is
 * empty here and never used.
 */
static const barcode_config_t g_code_limits = { BARCODE_DEFAULT_LIMITS };
     /* fields saturated */
uint64_t      g_live_due_us;
uint16_t      g_live_side_max;
  /* MISSION's cut going out */
uint32_t      g_raw_length;
uint32_t      g_raw_sent;
uint32_t      g_raw_cuts;
     /* cuts with a group this run */
uint32_t      g_raw_done;
     /* of them, sent in full */
bool          gb_raw_head;
bench_d0_t    g_d0;
bench_code_t        g_code;
static uint16_t            g_code_levels[BENCH_CODE_ADC_LEVELS];
static ID                  g_code_cyc;
    /* 1 ms wake-up, once created */
static ID                  g_bench_tid;
   /* the task bench_loop runs in */
bench_auto_t        g_auto;
#define BENCH_VOTE_DECODER2 (1U)
#define BENCH_VOTE_D0      (2U)

/** @brief BARCODE read: the store emptied, the 1 ms wake-up created once
 * (bench_code_wait), the decoder unit in metres and the robot decoder's
 * DARK/LIGHT thresholds (g_code_limits) on the side sensor's calibration
 * (app_cfg_line). No codebook: the decode reads any Code 39 text from the
 * widths alone. False, with the reason, if the calibration gives no
 * thresholds or the kernel no wake-up.
 */
bool bench_code_start (void)
{
    line_ctrl_config_t line;
    T_CCYC             cyc;

    (void)memset(&g_code, 0, sizeof(g_code));
    (void)memset(g_code_levels, 0, sizeof(g_code_levels));
    bench_fit_reset();  /* TRIAL */
    bench_vote_reset();
    g_code.b_stream = (BENCH_MODE_MISSION == g_mode);
    g_live_length = 0U;
    g_live_sent = 0U;
    g_live_due_us = 0U;
    g_live_side_max = 0U;
    g_live_front_min = 0U;
    g_live_front_none = 0U;
    g_raw_length = 0U;
    g_raw_sent = 0U;
    g_raw_cuts = 0U;
    g_raw_done = 0U;
    gb_raw_head = false;
    if (g_code_cyc <= 0)
    {
        (void)memset(&cyc, 0, sizeof(cyc));
        cyc.cycatr = TA_HLNG;
        cyc.cychdr = (FP)bench_code_cyc;
        cyc.cyctim = (RELTIM)BENCH_CODE_POLL_MS;
        g_bench_tid = tk_get_tid();
        g_code_cyc = tk_cre_cyc(&cyc);
        if (g_code_cyc <= 0)
        {
            bench_text("BARCODE refused: no 1 ms wake-up, tk_cre_cyc ");
            bench_signed((int32_t)g_code_cyc);
            bench_flush();
            return false;
        }
    }
    app_cfg_line(&line);
    g_code.white = line.white_adc[LINE_CTRL_BARCODE_CHANNEL];
    g_code.black = line.black_adc[LINE_CTRL_BARCODE_CHANNEL];
    g_code.unit_m = ((double)BENCH_CODE_UNIT_HC * CORE_PI *
                     ROBOT_WHEEL_RADIUS_M) /
                    (double)ROBOT_ENCODER_COUNTS_PER_REV;
    (void)memset(&g_d0, 0, sizeof(g_d0));
    g_d0.quiet = (uint16_t)bench_round((BENCH_CODE_QUIET_MM / 1000.0) /
                                       g_code.unit_m);
    if (line_ctrl_hysteresis(line.white_adc[LINE_CTRL_BARCODE_CHANNEL],
                             line.black_adc[LINE_CTRL_BARCODE_CHANNEL],
                             g_code_limits.black_on, g_code_limits.black_off,
                             &g_code.robot))
    {
        return true;
    }
    bench_text("BARCODE refused: the side sensor calibration gives no robot"
               " DARK/LIGHT thresholds");
    bench_flush();
    return false;
}

/** @brief BARCODE read: the 1 ms cyclic handler wakes the bench task. */
void bench_code_cyc (void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk(g_bench_tid);
}

/** @brief True while a BARCODE or MISSION run follows the line (after
 * AUTO-CENTRE, before the stop): bench_tick then reads the side sensor every
 * 1 ms.
 */
bool bench_code_polling (void)
{
    return (g_code_cyc > 0) && g_drive.b_running && (!g_drive.b_done) &&
           (!g_auto.b_active) && ((BENCH_MODE_BARCODE == g_drive.mode) ||
                                  (BENCH_MODE_MISSION == g_drive.mode));
}

/** @brief BARCODE read: bench_tick's wait, at least BENCH_TICK_MS, as
 * kernel ticks with the side sensor read at each (bench_code_poll). The
 * wake-ups queued during the last step are dropped first, so the reads
 * keep 1 ms apart.
 */

/** @brief One between-steps reading into the barcode store: raw taken when
 * the wheels' totals were total (the bench's 1 ms polls; the robot app's
 * Line task samples, mission_ir_drain).
 */
void bench_code_poll_at (const uint16_t raw[LINE_CTRL_CHANNELS],
                                const uint16_t total[BENCH_WHEELS])
{
    int32_t  now[BENCH_WHEELS] = {0, 0};
    uint32_t wheel;

    for (wheel = 0U; wheel < BENCH_WHEELS; ++wheel)
    {
        now[wheel] = (g_distance[wheel] - g_drive.start[wheel]) +
                     bench_signed_delta(total[wheel], g_last_total[wheel],
                                        wheel);
    }
    bench_code_sample(now[BENCH_LEFT], now[BENCH_RIGHT],
                      raw[LINE_CTRL_BARCODE_CHANNEL]);
}

/** @brief BARCODE read, every side sensor reading: add it to the mean of
 * the decoder unit its own path is in; entering a later unit stores the
 * last (forward only). Wheel counts since the start. The sensor sits
 * ROBOT_BARCODE_LATERAL_OFFSET_M left of the centre line, so it travels the
 * centre path less that offset times the heading change (+ left, the right
 * wheel ahead): in half-counts, left plus right less 2 * offset / track *
 * (right - left). Its offset ahead of the axle moves it along the bars, not
 * across them.
 */
void bench_code_sample (int32_t left, int32_t right, uint16_t raw)
{
    int32_t  centre_hc = left + right;
    int32_t  unit = bench_round((double)centre_hc -
                                ((2.0 * ROBOT_BARCODE_LATERAL_OFFSET_M /
                                  ROBOT_TRACK_WIDTH_M) *
                                 (double)(right - left))) /
                    BENCH_CODE_UNIT_HC;
    uint64_t now_us = board_time_us();

    if (0U == g_code.readings)
    {
        g_code.first_us = now_us;
    }
    g_code.last_us = now_us;
    ++g_code.readings;
    if (g_code.b_stream)
    {
        bench_d0_sample(unit, board_barcode_d0());
    }
    if ((unit < 0) ||
        ((0U != g_code.pending_count) && (unit < g_code.pending_unit)))
    {
        return;
    }
    if ((0U != g_code.pending_count) && (unit == g_code.pending_unit))
    {
        g_code.pending_sum += raw;
        ++g_code.pending_count;
        return;
    }
    bench_code_store();
    g_code.pending_unit = unit;
    g_code.pending_centre =
        (uint32_t)(((centre_hc > 0) ? centre_hc : 0) / BENCH_CODE_UNIT_HC);
    g_code.pending_sum = raw;
    g_code.pending_count = 1U;
}

/** @brief BARCODE read: the unit being read stored as the mean of its
 * readings, rounded, and counted in the run's reading levels
 * (g_code_levels, for bench_code_white); MISSION then streams it.
 */
void bench_code_store (void)
{
    bench_code_sample_t * p_sample;
    bool                  b_stored = false;

    if (0U == g_code.pending_count)
    {
        return;
    }
    if (g_code.count >= BENCH_CODE_SAMPLES)
    {
        g_code.b_full = true;
    }
    else
    {
        p_sample = &g_code.sample[g_code.count];
        p_sample->unit = (uint32_t)g_code.pending_unit;
        p_sample->centre = g_code.pending_centre;
        p_sample->raw = (uint16_t)((g_code.pending_sum +
                                    (g_code.pending_count / 2U)) /
                                   g_code.pending_count);
        ++g_code_levels[(uint32_t)p_sample->raw &
                        (BENCH_CODE_ADC_LEVELS - 1U)];
        ++g_code.count;
        b_stored = true;
    }
    g_code.pending_count = 0U;
    if (b_stored && g_code.b_stream)
    {
        bench_code_stream();
    }
}

/** @brief MISSION, after each stored sample: the floor that opens a group
 * renewed now and then (bench_code_white); a sample at or above it opens or
 * extends one, as does one above the smallest swing tried
 * (BENCH_CODE_TRY_MIN half sds) while open. BENCH_CODE_QUIET_MM of white
 * after the group's last such sample, or a nearly full store, cuts: what is
 * stored is decoded over the next line steps (bench_code_tries_begin), then
 * only the last gap of samples is kept, as with white alone at
 * BENCH_CODE_STREAM_HOLD samples.
 * No text here: bench_flush may wait in bench_tick, which reads again.
 */
void bench_code_stream (void)
{
    const bench_code_sample_t * p_last = &g_code.sample[g_code.count - 1U];
    uint32_t sd = 1U;
    uint32_t white;
    uint32_t gap = (uint32_t)bench_round((BENCH_CODE_QUIET_MM / 1000.0) /
                                         g_code.unit_m);
    bool     b_cut;

    if (g_code.b_trying)
    {
        /* Stored after the cut being decoded; looked at once it is done. */
        return;
    }
    if ((0U == g_code.stream_floor) ||
        (g_code.since_floor >= BENCH_CODE_FLOOR_EVERY))
    {
        white = bench_code_white(&sd);
        g_code.stream_floor = white + (BENCH_CODE_DARK_SDS * sd);
        g_code.stream_low = white + (((BENCH_CODE_TRY_MIN * sd) + 1U) / 2U);
        g_code.since_floor = 0U;
    }
    ++g_code.since_floor;
    /* Open at the usual floor; the smallest swing tried keeps it open. */
    if (((uint32_t)p_last->raw >= g_code.stream_floor) ||
        (g_code.b_open && ((uint32_t)p_last->raw >= g_code.stream_low)))
    {
        g_code.last_dark = p_last->unit;
        g_code.b_open = true;
    }
    /* Tape-dark (the robot's DARK) longer than any bar (the spec's 9 mm) is
     * tape across the side sensor, not a card: it ends the straight hold
     * (bench_code_hold). Not the opening floor: a card's spaces stay far
     * above white's noise (31 Oct: 1500-1900), so a whole card is one run
     * there; its bars are tape-dark 3-6 mm each.
     */
    if (g_code.robot.b_dark_high &&
        ((uint32_t)p_last->raw >= (uint32_t)g_code.robot.on_adc))
    {
        if (!g_code.b_dark)
        {
            g_code.dark_from = p_last->unit;
            g_code.b_dark = true;
        }
        if (((double)(p_last->unit - g_code.dark_from) * g_code.unit_m) >
            BENCH_CODE_LINE_M)
        {
            g_code.b_tape = true;
        }
    }
    else
    {
        g_code.b_dark = false;
    }
    /* A printed bar peaks BENCH_CODE_FLOOR of the calibrated span above
     * white; the floor that opens a group is only white's noise, so the
     * hold waits for a bar.
     */
    if (g_code.b_open && (g_code.black > g_code.white) &&
        ((uint32_t)p_last->raw >=
         ((uint32_t)g_code.white +
          (uint32_t)bench_round((double)((uint32_t)g_code.black -
                                         (uint32_t)g_code.white) *
                                BENCH_CODE_FLOOR))))
    {
        g_code.b_bar = true;
    }
    /* A full store leaves BENCH_CODE_STREAM_HOLD samples for the decode. */
    b_cut = g_code.b_open ?
            ((((uint32_t)p_last->unit) >= (g_code.last_dark + gap)) ||
             (g_code.count >= (BENCH_CODE_SAMPLES - BENCH_CODE_STREAM_HOLD))) :
            (g_code.count >= BENCH_CODE_STREAM_HOLD);
    if (b_cut && g_code.b_open)
    {
        bench_code_tries_begin(gap);
    }
    else if (b_cut)
    {
        bench_code_keep(gap);
    }
    else
    {
        /* Still storing. */
    }
}

/** @brief MISSION: the store cut back to its last gap units of samples, and
 * to those after the last mark decoded (b_open still set), so no edge of it
 * is left to start the next group.
 */
void bench_code_keep (uint32_t gap)
{
    uint32_t last = g_code.sample[g_code.count - 1U].unit;
    uint32_t first = 0U;

    while (((first + 1U) < g_code.count) &&
           ((((uint32_t)g_code.sample[first].unit + gap) < last) ||
            (g_code.b_open &&
             ((uint32_t)g_code.sample[first].unit <= g_code.last_dark))))
    {
        ++first;
    }
    (void)memmove(&g_code.sample[0], &g_code.sample[first],
                  (size_t)(g_code.count - first) * sizeof(g_code.sample[0]));
    g_code.count -= first;
}

/** @brief MISSION, a group from the quiet darkness pass: kept if it has a
 * Code 39 character's elements or more, and decoded as read, else
 * reversed, when its count fits Code 39 (framed only with a character
 * between the *s).
 */
void bench_code_found_add (uint32_t runs)
{
    bench_code_found_t * p_found;
    uint32_t             elements = runs - 1U;
    uint32_t             last = runs - 2U;
    uint32_t             length;

    if ((runs < 2U) || (elements < BENCH_CODE_ELEMENTS))
    {
        return;
    }
    ++g_code.groups_seen;
    if (g_code.founds >= BENCH_CODE_FOUND)
    {
        return;
    }
    p_found = &g_code.found[g_code.founds];
    ++g_code.founds;
    (void)memset(p_found, 0, sizeof(*p_found));
    p_found->from_units = g_code.sample[g_code.run[0].start].unit;
    p_found->to_units = (uint32_t)g_code.sample[g_code.run[last].start].unit +
                        g_code.run[last].units;
    p_found->bars = (elements / 2U) + 1U;
    p_found->half_sds = g_code.try_half_sds;
    p_found->gap_mm = g_code.try_gap_mm;
    if ((0U != ((elements + 1U) % (BENCH_CODE_ELEMENTS + 1U))) ||
        (((elements + 1U) / (BENCH_CODE_ELEMENTS + 1U)) >= BENCH_CODE_TEXT))
    {
        bench_code_found_rest(p_found, last);
        return;
    }
    length = (elements + 1U) / (BENCH_CODE_ELEMENTS + 1U);
    if (length < BENCH_CODE_FRAMED)
    {
        (void)bench_code39_text(0U, last, false, p_found->text);
        (void)bench_code39_text(0U, last, true, p_found->reverse);
        bench_code_found_rest(p_found, last);
        return;
    }
    p_found->b_framed = bench_code39_text(0U, last, false, p_found->text) &&
                        ('*' == p_found->text[0]) &&
                        ('*' == p_found->text[length - 1U]);
    if (!p_found->b_framed)
    {
        p_found->b_reverse =
            bench_code39_text(0U, last, true, p_found->reverse) &&
            ('*' == p_found->reverse[0]) &&
            ('*' == p_found->reverse[length - 1U]);
        p_found->b_framed = p_found->b_reverse;
    }
    if (p_found->b_framed)
    {
        p_found->spec = bench_code39_spec(0U, length, last);
        if (BENCH_SPEC_OK != p_found->spec)
        {
            (void)memcpy(p_found->refused, p_found->b_reverse ?
                                           p_found->reverse : p_found->text,
                         sizeof(p_found->refused));
            p_found->b_framed = false;
            p_found->b_reverse = false;
        }
    }
    if (!p_found->b_framed)
    {
        bench_code_found_rest(p_found, last);
    }
}

/** @brief MISSION, a group whose count or frame fails: a framed window
 * (bench_code_found_window), else every character it holds
 * (bench_code_found_chars).
 */
void bench_code_found_rest (bench_code_found_t * p_found,
                                   uint32_t last)
{
    bench_code_found_window(p_found, last);
    if (!p_found->b_framed)
    {
        bench_code_found_chars(p_found, last);
    }
}

/** @brief MISSION, a group that framed nothing: every character it holds,
 * as read and read end to start. From the first bar, a whole character
 * that decodes is kept and the next is tried past it; else the next bar is
 * tried. Characters read end to start are listed in that order.
 */
void bench_code_found_chars (bench_code_found_t * p_found,
                                    uint32_t last)
{
    char     text[BENCH_CODE_TEXT];
    char *   p_out;
    char     swap;
    uint32_t count;
    uint32_t start;
    uint32_t index;
    uint32_t pass;
    bool     b_reverse;

    for (pass = 0U; pass < BENCH_CODE_DIRECTIONS; ++pass)
    {
        b_reverse = (1U == pass);
        p_out = b_reverse ? p_found->chars_reverse : p_found->chars;
        count = 0U;
        start = 0U;
        while (((start + (BENCH_CODE_ELEMENTS - 1U)) <= last) &&
               ((count + 1U) < BENCH_CODE_TEXT))
        {
            if (bench_code39_text(start, start + (BENCH_CODE_ELEMENTS - 1U),
                                  b_reverse, text))
            {
                p_out[count] = text[0];
                ++count;
                start += BENCH_CODE_ELEMENTS + 1U;
            }
            else
            {
                start += 2U;
            }
        }
        p_out[count] = '\0';
        for (index = 0U; b_reverse && ((2U * index) + 1U < count); ++index)
        {
            swap = p_out[index];
            p_out[index] = p_out[count - 1U - index];
            p_out[count - 1U - index] = swap;
        }
    }
}

/** @brief MISSION, a group whose count or frame fails: the longest window
 * of whole characters inside it, from a bar, at least BENCH_CODE_FRAMED
 * long, that reads * ... * as read or reversed. A mark beside a card then
 * cannot hide its code (30 Oct: a faint one 21 mm past *A*'s stop made 16
 * bars). Every character in the window must pass the try's rule, so a
 * window across stray bars does not frame, and the window the spec checks
 * (bench_code39_spec), its quiet zones the spaces beside it.
 */
void bench_code_found_window (bench_code_found_t * p_found,
                                     uint32_t last)
{
    char     text[BENCH_CODE_TEXT];
    uint32_t length;
    uint32_t span;
    uint32_t start;
    uint32_t pass;
    uint32_t spec;
    bool     b_reverse;

    for (length = ((last + 2U) / (BENCH_CODE_ELEMENTS + 1U));
         length >= BENCH_CODE_FRAMED; --length)
    {
        if (length >= BENCH_CODE_TEXT)
        {
            continue;
        }
        /* A window's last element, from its first: 9 each, 1 between. */
        span = (length * (BENCH_CODE_ELEMENTS + 1U)) - 2U;
        for (start = 0U; (start + span) <= last; start += 2U)
        {
            for (pass = 0U; pass < BENCH_CODE_DIRECTIONS; ++pass)
            {
                b_reverse = (1U == pass);
                if (bench_code39_text(start, start + span, b_reverse, text) &&
                    ('*' == text[0]) && ('*' == text[length - 1U]))
                {
                    spec = bench_code39_spec(start, length, last);
                    if (BENCH_SPEC_OK != spec)
                    {
                        if (BENCH_SPEC_OK == p_found->spec)
                        {
                            p_found->spec = spec;
                            (void)memcpy(p_found->refused, text,
                                         sizeof(p_found->refused));
                        }
                        continue;
                    }
                    (void)memcpy(b_reverse ? p_found->reverse :
                                             p_found->text,
                                 text, sizeof(text));
                    p_found->b_reverse = b_reverse;
                    p_found->b_framed = true;
                    return;
                }
            }
        }
    }
    /* One * only: the card's first or
     * last character lost to line following, so 2 characters, exactly one
     * of them *. Only in a cut's second round of tries, after no whole
     * frame agreed; sent as a partial, the swings must still agree on it.
     */
    if (!g_code.b_partial_ok)
    {
        return;
    }
    span = (2U * (BENCH_CODE_ELEMENTS + 1U)) - 2U;
    for (start = 0U; (start + span) <= last; start += 2U)
    {
        for (pass = 0U; pass < BENCH_CODE_DIRECTIONS; ++pass)
        {
            b_reverse = (1U == pass);
            if (bench_code39_text(start, start + span, b_reverse, text) &&
                (('*' == text[0]) != ('*' == text[1])))
            {
                (void)memcpy(b_reverse ? p_found->reverse : p_found->text,
                             text, sizeof(text));
                p_found->b_reverse = b_reverse;
                p_found->b_framed = true;
                p_found->b_partial = true;
                return;
            }
        }
    }
}

/** @brief MISSION, from the line step: the next group not yet sent, as one
 * line of text (not flushed): CODE for a framed one, else GROUP. True if
 * there was one.
 */
bool bench_code_found_text (void)
{
    bench_code_found_t * p_found;
    uint32_t             index;

    for (index = 0U; index < g_code.founds; ++index)
    {
        p_found = &g_code.found[index];
        if (p_found->b_sent)
        {
            continue;
        }
        p_found->b_sent = true;
        /* Frame: texts BENCH_CODE_TEXT - 1 = 15 characters at most (4-bit
         * length), bars 0-255, the swing 0-51.1 sd, the gap 0-255 mm.
         */
        bench_line(BK_CODE);
        if (bench_flag(p_found->b_framed))
        {
            bench_text(" CODE ");
            bench_str(p_found->b_reverse ? p_found->reverse :
                                           p_found->text, 4U);
            bench_text(bench_flag(p_found->b_reverse) ?
                       " read end to start" : " read in order");
            bench_text(bench_flag(p_found->b_partial) ?
                       ", PARTIAL: one * only" : "");
            bench_text(bench_flag(p_found->b_peaks) ? ", peaks agree" : "");
        }
        else
        {
            bench_text(" GROUP ");
            bench_u(p_found->bars, 8U);
            bench_text(" bars, ");
            /* Which GROUP wording, 2 bits: 0 one swing, 1 refused, 2 no
             * frame, 3 not a count.
             */
            if (p_found->b_single)
            {
                bench_code(0U, 2U);
                bench_text("one swing only, not a code: ");
                bench_str(p_found->b_reverse ? p_found->reverse :
                                               p_found->text, 4U);
                bench_text(bench_flag(p_found->b_reverse) ? " end to start" :
                                                            "");
            }
            else if (BENCH_SPEC_OK != p_found->spec)
            {
                bench_code(1U, 2U);
                bench_text("frame ");
                bench_str(p_found->refused, 4U);
                bench_text(" refused by the spec (");
                bench_name(g_spec_names, p_found->spec % BENCH_SPEC_RESULTS,
                           2U);
                bench_text(")");
            }
            else if ('\0' != p_found->text[0])
            {
                bench_code(2U, 2U);
                bench_text("no frame: ");
                bench_str(p_found->text, 4U);
                bench_text(" / ");
                bench_str(p_found->reverse, 4U);
            }
            else
            {
                bench_code(3U, 2U);
                bench_text("not a Code 39 count");
            }
            if (!p_found->b_single)
            {
                bench_text(", characters ");
                bench_str(p_found->chars, 4U);
                bench_text(('\0' != p_found->chars[0]) ? "" : "none");
                bench_text(" / ");
                bench_str(p_found->chars_reverse, 4U);
                bench_text(('\0' != p_found->chars_reverse[0]) ? "" : "none");
            }
        }
        bench_vote_add(BENCH_VOTE_DECODER1, bench_vote_mm(p_found->from_units),
                       bench_vote_mm(p_found->to_units),
                       (!p_found->b_framed) ? NULL :
                       (p_found->b_reverse ? p_found->reverse : p_found->text));
        bench_text(" at ");
        bench_code_unit_mm(p_found->from_units);
        bench_text("-");
        bench_code_unit_mm(p_found->to_units);
        bench_text(" mm (swing ");
        bench_ux(5U * p_found->half_sds, 1U, 9U);
        bench_text(" sd, gap ");
        bench_u(p_found->gap_mm, 8U);
        bench_text(" mm)");
        return true;
    }
    return false;
}

/** @brief MISSION, after the stop: any open group decoded and sent like the
 * live ones, then one line of totals. Every other group went out live.
 */
void bench_code_mission_results (void)
{
    uint32_t index;
    uint32_t codes = 0U;
    uint32_t gap_us = 0U;

    bench_code_store();
    /* TRIAL: the fit under way finished first, then the last cut, then
     * the card the fit was binning (the last cut may have joined it).
     */
    while (bench_fit_complete(false))
    {
        if (bench_fit_text())
        {
            bench_flush();
        }
    }
    if (g_code.b_trying || (g_code.b_open && (0U != g_code.count)))
    {
        bench_code_dark_tries();
    }
    while (bench_fit_complete(true))
    {
        if (bench_fit_text())
        {
            bench_flush();
        }
    }
    while (bench_code_found_text())
    {
        bench_flush();
    }
    bench_fit_totals();
    while (bench_avoid_text())
    {
        bench_flush();
    }
    if (0U != g_avoid_dropped)
    {
        bench_line(BK_AVOID_DROPPED);   /* 0-1023 lines */
        bench_text(" obstacle go-around: ");
        bench_u(g_avoid_dropped, 10U);
        bench_text(" lines dropped (queue full)");
        bench_flush();
    }
    bench_d0_close();
    while (bench_d0_text())
    {
        bench_flush();
    }
    bench_vote_finish();
    while (bench_vote_text())
    {
        bench_flush();
    }
    bench_vote_totals();
    /* Frame: changes and readings 0-4194303 (22 bits: over 1 h of 1 ms
     * readings), groups 0-4095, counts 0-1023, the bar minimum 0-15.
     */
    bench_line(BK_D0_TOTALS);
    bench_text(" D0: ");
    bench_u(g_d0.changes, 22U);
    bench_text(" changes, ");
    bench_u(g_d0.groups, 12U);
    bench_text(" groups (");
    bench_u(g_d0.shorts, 12U);
    bench_text(" under ");
    bench_u(BENCH_D0_BARS_MIN, 4U);
    bench_text(" bars), ");
    bench_u(g_d0.reads, 10U);
    bench_text(" read, ");
    bench_u(g_d0.dropped, 10U);
    bench_text(" dropped; high ");
    bench_u(g_d0.high, 22U);
    bench_text(", low ");
    bench_u(g_d0.low, 22U);
    bench_text(" readings, dark is ");
    bench_text(bench_flag(g_d0.high <= g_d0.low) ? "high" : "low");
    bench_flush();
    codes = g_code.codes_done;   /* groups already dropped (sent) */
    for (index = 0U; index < g_code.founds; ++index)
    {
        if (g_code.found[index].b_framed)
        {
            ++codes;
        }
    }
    if (g_code.readings > 1U)
    {
        gap_us = (uint32_t)((g_code.last_us - g_code.first_us) /
                            (uint64_t)(g_code.readings - 1U));
    }
    /* Frame: readings 22 bits, the period 0-163.83 ms, counts as above,
     * held 0-65535 mm, groups kept 0-15, spec refusals 0-4095 (every try).
     */
    bench_line(BK_BARCODES);
    bench_text(" barcodes: ");
    bench_u(g_code.readings, 22U);
    bench_text(" readings, one per ");
    bench_ux(gap_us / 10U, 2U, 14U);
    bench_text(" ms; groups ");
    bench_u(g_code.groups_seen, 10U);
    bench_text(", codes ");
    bench_u(codes, 10U);
    bench_text("; held straight ");
    bench_u(g_code.holds, 10U);
    bench_text(" times, ");
    bench_u((uint32_t)g_code.held_mm, 16U);
    bench_text(" mm, let go ");
    bench_u(g_code.let_gos, 10U);
    if (bench_flag(g_code.groups_seen > (g_code.founds + g_code.founds_done)))
    {
        bench_text(", the first ");
        bench_u(g_code.founds + g_code.founds_done, 10U);
        bench_text(" kept");
    }
    if (bench_flag(g_code.b_full))
    {
        bench_text("; STORE FULL");
    }
    /* Counted over every try, so one card can count several times. */
    bench_text("; spec refused frames: widths ");
    bench_u(g_code.spec_widths, 12U);
    bench_text(", gaps ");
    bench_u(g_code.spec_gaps, 12U);
    bench_text(", quiet ");
    bench_u(g_code.spec_quiet, 12U);
    /* Samples still waiting are dropped: nothing long after the stop. */
    g_raw_length = 0U;
    bench_text("; raw sent for ");
    bench_u(g_raw_done, 10U);
    bench_text(" of ");
    bench_u(g_raw_cuts, 10U);
    bench_text(" cuts");
    bench_flush();
}

/** @brief MISSION, every side sensor reading: D0 at the unit of the
 * sensor's own path (forward only: a unit behind the last counts as the
 * last). The level seen less over the run so far is dark. A change to dark
 * opens a group or adds to it; a change to light ends a bar. Light lasting
 * BENCH_CODE_QUIET_MM closes the group (bench_d0_close).
 */
void bench_d0_sample (int32_t unit, bool b_level)
{
    bench_d0_group_t * p_open = &g_d0.open;
    uint32_t           at;
    bool               b_dark_high;

    if (unit < 0)
    {
        return;
    }
    at = (uint32_t)unit;
    if (g_d0.b_started && (at < g_d0.unit))
    {
        at = g_d0.unit;
    }
    g_d0.unit = at;
    if (b_level)
    {
        ++g_d0.high;
    }
    else
    {
        ++g_d0.low;
    }
    b_dark_high = (g_d0.high <= g_d0.low);
    if (!g_d0.b_started)
    {
        g_d0.b_started = true;
        g_d0.b_level = b_level;
        g_d0.light_from = at;
        return;
    }
    if (b_level != g_d0.b_level)
    {
        g_d0.b_level = b_level;
        ++g_d0.changes;
        if (b_level == b_dark_high)
        {
            if (!g_d0.b_open)
            {
                (void)memset(p_open, 0, sizeof(*p_open));
                g_d0.b_open = true;
            }
        }
        else
        {
            g_d0.light_from = at;
        }
        if (g_d0.b_open)
        {
            if (p_open->edges < BENCH_D0_EDGES)
            {
                p_open->edge[p_open->edges] = at;
            }
            ++p_open->edges;
        }
    }
    if (g_d0.b_open && (b_level != b_dark_high) &&
        ((uint32_t)(at - g_d0.light_from) >= (uint32_t)g_d0.quiet))
    {
        bench_d0_close();
    }
}

/** @brief MISSION: the open D0 group closed (a bar still dark at the stop
 * ends at the last reading). Under BENCH_D0_BARS_MIN bars it is only
 * counted; else it waits to be sent, or is dropped if BENCH_D0_GROUPS wait.
 */
void bench_d0_close (void)
{
    bench_d0_group_t * p_open = &g_d0.open;

    if (!g_d0.b_open)
    {
        return;
    }
    g_d0.b_open = false;
    if (0U != (p_open->edges & 1U))
    {
        if (p_open->edges < BENCH_D0_EDGES)
        {
            p_open->edge[p_open->edges] = g_d0.unit;
        }
        ++p_open->edges;
    }
    ++g_d0.groups;
    if (p_open->edges < (2U * BENCH_D0_BARS_MIN))
    {
        ++g_d0.shorts;
    }
    else if (g_d0.queued >= BENCH_D0_GROUPS)
    {
        ++g_d0.dropped;
    }
    else
    {
        g_d0.group[g_d0.queued] = *p_open;
        ++g_d0.queued;
    }
}

/** @brief MISSION: a closed D0 group as Code 39, from either end. Elements
 * are the widths between changes (bar, space, ...); every character is 9
 * of them and a gap, so a code is a multiple of 10 less one. In each
 * character the 3 widest are wide, and the narrowest wide must be wider
 * than the widest narrow, else the character is '?'. p_text (at least
 * BENCH_CODE_TEXT) gets the characters, or "" when the count is not Code
 * 39. True for a frame: *, at least one character, *, none '?'.
 */
bool bench_d0_decode (const bench_d0_group_t * p_group, bool b_reverse,
                             char * p_text)
{
    uint32_t units[BENCH_CODE_ELEMENTS];
    uint32_t elements = p_group->edges - 1U;
    uint32_t length;
    uint32_t character;
    uint32_t element;
    uint32_t index;
    uint32_t wide;
    uint32_t widest;
    uint32_t narrowest_wide;
    uint32_t widest_narrow;
    uint32_t pattern;
    bool     b_all = true;

    p_text[0] = '\0';
    if ((p_group->edges < 2U) || (p_group->edges > BENCH_D0_EDGES) ||
        (0U != ((elements + 1U) % (BENCH_CODE_ELEMENTS + 1U))))
    {
        return false;
    }
    length = (elements + 1U) / (BENCH_CODE_ELEMENTS + 1U);
    if (length >= BENCH_CODE_TEXT)
    {
        return false;
    }
    for (character = 0U; character < length; ++character)
    {
        for (element = 0U; element < BENCH_CODE_ELEMENTS; ++element)
        {
            index = (character * (BENCH_CODE_ELEMENTS + 1U)) + element;
            if (b_reverse)
            {
                index = elements - 1U - index;
            }
            units[element] = (uint32_t)p_group->edge[index + 1U] -
                             (uint32_t)p_group->edge[index];
        }
        pattern = 0U;
        for (wide = 0U; wide < BENCH_CODE_WIDE; ++wide)
        {
            widest = BENCH_CODE_ELEMENTS;
            for (element = 0U; element < BENCH_CODE_ELEMENTS; ++element)
            {
                if ((0U == (pattern & (0x100U >> element))) &&
                    ((BENCH_CODE_ELEMENTS == widest) ||
                     (units[element] > units[widest])))
                {
                    widest = element;
                }
            }
            pattern |= 0x100U >> widest;
        }
        narrowest_wide = UINT32_MAX;
        widest_narrow = 0U;
        for (element = 0U; element < BENCH_CODE_ELEMENTS; ++element)
        {
            if (0U != (pattern & (0x100U >> element)))
            {
                if (units[element] < narrowest_wide)
                {
                    narrowest_wide = units[element];
                }
            }
            else if (units[element] > widest_narrow)
            {
                widest_narrow = units[element];
            }
            else
            {
                /* A narrow element no wider than the widest narrow so far. */
            }
        }
        index = 0U;
        while ((index < BENCH_CODE_CHARS) &&
               (g_code39_patterns[index] != pattern))
        {
            ++index;
        }
        if ((index < BENCH_CODE_CHARS) && (narrowest_wide > widest_narrow))
        {
            p_text[character] = g_code39_chars[index];
        }
        else
        {
            p_text[character] = '?';
            b_all = false;
        }
    }
    p_text[length] = '\0';
    return b_all && (length >= BENCH_CODE_FRAMED) && ('*' == p_text[0]) &&
           ('*' == p_text[length - 1U]);
}

/** @brief MISSION, from the line step and after the stop: the oldest closed
 * D0 group as two lines of text (not flushed), D0 then D0 widths: its code
 * or why not, where it was, then its element widths in decoder units. True
 * if there was a line.
 */
bool bench_d0_text (void)
{
    const bench_d0_group_t * p_group = &g_d0.group[0];
    char                     text[BENCH_CODE_TEXT];
    char                     reverse[BENCH_CODE_TEXT];
    uint32_t                 stored;
    uint32_t                 index;
    bool                     b_order;
    bool                     b_reverse;

    if (0U == g_d0.queued)
    {
        return false;
    }
    stored = (p_group->edges < BENCH_D0_EDGES) ? p_group->edges :
                                                 BENCH_D0_EDGES;
    if (!g_d0.b_widths)
    {
        b_order = bench_d0_decode(p_group, false, text);
        b_reverse = bench_d0_decode(p_group, true, reverse);
        bench_vote_add(BENCH_VOTE_D0, bench_vote_mm((uint32_t)p_group->edge[0]),
                       bench_vote_mm((uint32_t)p_group->edge[stored - 1U]),
                       b_order ? text : (b_reverse ? reverse : NULL));
        /* Frame: texts 15 characters at most (4-bit length), bars 0-1023,
         * why-not 2 bits (0 too many, 1 no frame, 2 not a count).
         */
        bench_line(BK_D0);
        bench_text(" D0 ");
        if (bench_flag(b_order || b_reverse))
        {
            ++g_d0.reads;
            bench_str(b_order ? text : reverse, 4U);
            bench_text(bench_flag(b_order) ? " read in order" :
                                             " read end to start");
        }
        else
        {
            bench_u(p_group->edges / 2U, 10U);
            bench_text(" bars, ");
            if (p_group->edges > BENCH_D0_EDGES)
            {
                bench_code(0U, 2U);
                bench_text("too many to keep");
            }
            else if ('\0' != text[0])
            {
                bench_code(1U, 2U);
                bench_text("no frame: ");
                bench_str(text, 4U);
                bench_text(" / ");
                bench_str(reverse, 4U);
            }
            else
            {
                bench_code(2U, 2U);
                bench_text("not a Code 39 count");
            }
        }
        bench_text(" at ");
        bench_code_unit_mm(p_group->edge[0]);
        bench_text("-");
        bench_code_unit_mm(p_group->edge[stored - 1U]);
        bench_text(" mm");
        g_d0.b_widths = true;
        return true;
    }
    /* Frame: the count 0-BENCH_D0_WIDTHS (36, 6 bits), each width 0-511
     * units (9 bits; a group ends at BENCH_CODE_QUIET_MM of white).
     */
    bench_line(BK_D0_WIDTHS);
    bench_text(" D0 widths u:");
    bench_code(((stored > BENCH_D0_WIDTHS) ? BENCH_D0_WIDTHS :
                (stored - 1U)), 6U);
    for (index = 1U; (index < stored) && (index <= BENCH_D0_WIDTHS); ++index)
    {
        bench_text(" ");
        bench_u((uint32_t)p_group->edge[index] -
                (uint32_t)p_group->edge[index - 1U], 9U);
    }
    g_d0.b_widths = false;
    --g_d0.queued;
    for (index = 0U; index < g_d0.queued; ++index)
    {
        g_d0.group[index] = g_d0.group[index + 1U];
    }
    return true;
}

/** @brief BARCODE read, after the stop: raw range and thresholds, each dark
 * group's widths and Code 39 decode, again on this run's own thresholds
 * if the robot's decoded nothing, then the trace.
 */
void bench_code_results (void)
{
    line_ctrl_hysteresis_t own;
    uint32_t               index;
    uint32_t               groups = 0U;
    uint32_t               peak = 0U;
    bool                   b_decoded = false;
    uint32_t               gap_us = 0U;

    bench_code_store();
    if (0U == g_code.count)
    {
        bench_text(" BARCODE: no side sensor samples (no forward travel)");
        bench_flush();
        return;
    }
    g_code.minimum = g_code.sample[0].raw;
    g_code.maximum = g_code.sample[0].raw;
    for (index = 1U; index < g_code.count; ++index)
    {
        if (g_code.sample[index].raw < g_code.minimum)
        {
            g_code.minimum = g_code.sample[index].raw;
        }
        if (g_code.sample[index].raw > g_code.maximum)
        {
            g_code.maximum = g_code.sample[index].raw;
            peak = index;
        }
    }
    bench_text(" BARCODE: ");
    bench_dec((int32_t)g_code.count);
    bench_text(" side sensor samples over ");
    bench_code_unit_mm(g_code.sample[g_code.count - 1U].unit);
    bench_text(" mm, raw min ");
    bench_dec((int32_t)g_code.minimum);
    bench_text(" max ");
    bench_dec((int32_t)g_code.maximum);
    bench_text(" at ");
    bench_code_unit_mm(g_code.sample[peak].unit);
    bench_text(" mm; robot DARK ");
    bench_text(g_code.robot.b_dark_high ? ">= " : "<= ");
    bench_dec((int32_t)g_code.robot.on_adc);
    bench_text(", LIGHT ");
    bench_text(g_code.robot.b_dark_high ? "<= " : ">= ");
    bench_dec((int32_t)g_code.robot.off_adc);
    if (g_code.b_full)
    {
        bench_text(", STORE FULL");
    }
    bench_flush();
    if (g_code.readings > 1U)
    {
        gap_us = (uint32_t)((g_code.last_us - g_code.first_us) /
                            (uint64_t)(g_code.readings - 1U));
    }
    bench_text(" each sample the mean of its unit's readings: ");
    bench_dec((int32_t)g_code.readings);
    bench_text(" readings, one per ");
    bench_fixed((int32_t)(gap_us / 10U), 2U);
    bench_text(" ms on average (1 ms between steps)");
    bench_flush();
    groups = bench_code_pass(&g_code.robot, &b_decoded);
    if ((0U == groups) &&
        (g_code.maximum < g_code.robot.off_adc))
    {
        bench_text(" NO BAR READ: never above LIGHT; the bars read white:"
                   " an IR-invisible print (dye ink), or the sensor ran"
                   " beside the bars");
        bench_flush();
    }
    else if (0U == groups)
    {
        bench_text(" NO BAR READ: the bars peak above LIGHT but below DARK:"
                   " too faint for the robot's thresholds");
        bench_flush();
    }
    else
    {
        /* The groups are shown. */
    }
    /* Contrast the robot's thresholds missed: decode with this run's own,
     * to show what the code says and the thresholds it would need.
     */
    if ((!b_decoded) &&
        (((uint32_t)g_code.maximum - (uint32_t)g_code.minimum) >=
         BENCH_CODE_CONTRAST) &&
        line_ctrl_hysteresis(g_code.minimum, g_code.maximum,
                             BENCH_CODE_OWN_ON, BENCH_CODE_OWN_OFF, &own))
    {
        bench_text(" again with this run's own DARK >= ");
        bench_dec((int32_t)own.on_adc);
        bench_text(", LIGHT <= ");
        bench_dec((int32_t)own.off_adc);
        bench_text(" (55%/40% of raw min-max; the robot uses 75%/55% of"
                   " the tape):");
        bench_flush();
        (void)bench_code_pass(&own, &b_decoded);
    }
    bench_code_card();
    bench_code_dark(false, 2U * BENCH_CODE_DARK_SDS, BENCH_CODE_GROUP_GAP_MM);
    bench_code_trace();
}

/** @brief One pass over the samples with a hysteresis: each dark group's
 * widths and Code 39 decode. The number of dark groups; *p_decoded is set
 * when a group decoded as a Code 39 frame.
 */
uint32_t bench_code_pass (const line_ctrl_hysteresis_t * p_hysteresis,
                                 bool * p_decoded)
{
    uint32_t index = 0U;
    uint32_t first = 0U;
    uint32_t last = 0U;
    uint32_t groups = 0U;
    uint32_t gap_units = (uint32_t)((BENCH_CODE_GROUP_GAP_MM / 1000.0) /
                                    g_code.unit_m);

    bench_code_runs(p_hysteresis);
    while (index < g_code.runs)
    {
        if (!g_code.run[index].b_black)
        {
            ++index;
            continue;
        }
        /* A dark group: bars joined by spaces narrower than the gap. */
        first = index;
        last = index;
        while (((last + 2U) < g_code.runs) &&
               (g_code.run[last + 1U].units < gap_units) &&
               g_code.run[last + 2U].b_black)
        {
            last += 2U;
        }
        ++groups;
        if (groups <= BENCH_CODE_GROUPS)
        {
            bench_text(" group ");
            bench_dec((int32_t)groups);
            if (bench_code_group(first, last))
            {
                *p_decoded = true;
            }
        }
        index = last + 1U;
    }
    if (groups > BENCH_CODE_GROUPS)
    {
        bench_text(" ");
        bench_dec((int32_t)(groups - BENCH_CODE_GROUPS));
        bench_text(" more dark groups not shown");
        bench_flush();
    }
    if (g_code.b_runs_full)
    {
        bench_text(" RUN STORE FULL: runs after the first ");
        bench_dec((int32_t)BENCH_CODE_RUNS);
        bench_text(" are not shown");
        bench_flush();
    }
    return groups;
}

/** @brief Runs of a hysteresis class; BAND keeps the last class. A run's
 * width ends where the next run starts (or one unit past the last sample).
 */
void bench_code_runs (const line_ctrl_hysteresis_t * p_hysteresis)
{
    bench_code_run_t * p_run = NULL;
    line_ctrl_class_t  darkness;
    uint32_t           index;
    uint32_t           end;
    bool               b_black = false;

    g_code.runs = 0U;
    g_code.b_runs_full = false;
    for (index = 0U; index < g_code.count; ++index)
    {
        darkness = line_ctrl_classify(p_hysteresis,
                                      g_code.sample[index].raw);
        if (LINE_CTRL_DARK == darkness)
        {
            b_black = true;
        }
        else if (LINE_CTRL_LIGHT == darkness)
        {
            b_black = false;
        }
        else
        {
            /* BAND or INVALID: keep the last class. */
        }
        if ((NULL == p_run) || (b_black != p_run->b_black))
        {
            if (g_code.runs >= BENCH_CODE_RUNS)
            {
                g_code.b_runs_full = true;
                break;
            }
            p_run = &g_code.run[g_code.runs];
            ++g_code.runs;
            p_run->b_black = b_black;
            p_run->start = (uint16_t)index;
            p_run->peak = g_code.sample[index].raw;
        }
        else if (b_black ? (g_code.sample[index].raw > p_run->peak) :
                           (g_code.sample[index].raw < p_run->peak))
        {
            p_run->peak = g_code.sample[index].raw;
        }
        else
        {
            /* Not a new peak. */
        }
    }
    for (index = 0U; index < g_code.runs; ++index)
    {
        p_run = &g_code.run[index];
        end = ((index + 1U) < g_code.runs) ?
              g_code.sample[g_code.run[index + 1U].start].unit :
              (g_code.sample[g_code.count - 1U].unit + 1U);
        p_run->units = (uint16_t)(end - g_code.sample[p_run->start].unit);
    }
}

/** @brief Third pass, on the card's own levels. Bars are the side sensor's
 * peaks at least BENCH_CODE_FLOOR of the calibrated span above white, other
 * than tape-dark spans of a line's length. Each edge is where the reading
 * crosses halfway between the bar's peak and the valley of the space beside
 * it, so a print lighter than the tape, or a bar narrower than the sensor's
 * spot, still gives its edges. Bars less than BENCH_CODE_GROUP_GAP_MM apart
 * form a group, decoded as Code 39.
 */
void bench_code_card (void)
{
    uint32_t span;
    uint32_t floor_adc;
    uint32_t noise;
    uint32_t swing;
    uint32_t darkest = 0U;
    uint32_t index;
    uint32_t point;
    uint32_t before;
    uint32_t after;
    uint32_t start = 0U;
    uint32_t end = 0U;
    uint32_t runs = 0U;
    uint32_t groups = 0U;
    uint32_t space_raw = 0xFFFFU;
    double   rise;
    double   fall;
    double   last_fall = 0.0;
    double   gap_units = (BENCH_CODE_GROUP_GAP_MM / 1000.0) / g_code.unit_m;
    bool     b_decoded = false;

    if ((!g_code.robot.b_dark_high) || (g_code.black <= g_code.white))
    {
        bench_text(" card levels: not run, the side sensor reads dark low");
        bench_flush();
        return;
    }
    span = (uint32_t)g_code.black - (uint32_t)g_code.white;
    floor_adc = (uint32_t)g_code.white +
                (uint32_t)bench_round((double)span * BENCH_CODE_FLOOR);
    noise = (uint32_t)bench_round((double)span * BENCH_CODE_NOISE);
    bench_code_card_lines();
    for (index = 0U; index < g_code.count; ++index)
    {
        if ((!bench_code_in_line(index)) &&
            (g_code.sample[index].raw > darkest))
        {
            darkest = g_code.sample[index].raw;
        }
    }
    swing = (darkest > g_code.minimum) ?
            (uint32_t)bench_round((double)(darkest - g_code.minimum) *
                                  BENCH_CODE_SWING) : 0U;
    if (swing < noise)
    {
        swing = noise;
    }
    bench_code_card_extrema((uint16_t)swing);
    bench_text(" card levels: bars peak >= ");
    bench_dec((int32_t)floor_adc);
    bench_text(", darkest off the line ");
    bench_dec((int32_t)darkest);
    bench_text(", bar-space swing >= ");
    bench_dec((int32_t)swing);
    bench_text(", tape-dark lines ");
    bench_dec((int32_t)g_code.lines);
    bench_text(", turning points ");
    bench_dec((int32_t)g_code.extremes);
    if (g_code.extremes >= BENCH_CODE_EXTREMA)
    {
        bench_text(" (FULL)");
    }
    bench_flush();
    if (darkest < floor_adc)
    {
        bench_text(" card levels: no bar, nothing off the line peaks"
                   " above the floor");
        bench_flush();
        return;
    }
    /* Peaks are the odd turning points; each needs a valley after it. */
    for (point = 1U; (point + 1U) < g_code.extremes; point += 2U)
    {
        index = g_code.extreme[point];
        before = g_code.extreme[point - 1U];
        after = g_code.extreme[point + 1U];
        if (g_code.sample[before].raw < space_raw)
        {
            space_raw = g_code.sample[before].raw;
        }
        if (bench_code_in_line(index) ||
            (g_code.sample[index].raw < floor_adc))
        {
            if (bench_code_in_line(index) && (0U != runs))
            {
                /* A line ends the group. */
                ++groups;
                bench_code_card_group(groups, runs, &b_decoded);
                runs = 0U;
            }
            continue;
        }
        rise = bench_code_edge(index, before,
                               0.5 * ((double)g_code.sample[before].raw +
                                      (double)g_code.sample[index].raw),
                               &start);
        fall = bench_code_edge(index, after,
                               0.5 * ((double)g_code.sample[index].raw +
                                      (double)g_code.sample[after].raw),
                               &end);
        if ((0U != runs) && ((rise - last_fall) >= gap_units))
        {
            ++groups;
            bench_code_card_group(groups, runs, &b_decoded);
            runs = 0U;
        }
        if ((0U != runs) && ((runs + 2U) <= BENCH_CODE_RUNS))
        {
            g_code.run[runs - 1U].units =
                (uint16_t)bench_round(fmax(0.0, rise - last_fall));
            g_code.run[runs - 1U].peak = (uint16_t)space_raw;
        }
        if ((runs + 2U) <= BENCH_CODE_RUNS)
        {
            g_code.run[runs].b_black = true;
            g_code.run[runs].start = (uint16_t)start;
            g_code.run[runs].units =
                (uint16_t)bench_round(fmax(0.0, fall - rise));
            g_code.run[runs].peak = g_code.sample[index].raw;
            /* The space after it, closed by the next bar. */
            g_code.run[runs + 1U].b_black = false;
            g_code.run[runs + 1U].start = (uint16_t)end;
            runs += 2U;
        }
        last_fall = fall;
        space_raw = 0xFFFFU;
    }
    if (0U != runs)
    {
        ++groups;
        bench_code_card_group(groups, runs, &b_decoded);
    }
    if (0U == groups)
    {
        bench_text(" card levels: no bar with a space either side");
        bench_flush();
    }
    else if (groups > BENCH_CODE_GROUPS)
    {
        bench_text(" ");
        bench_dec((int32_t)(groups - BENCH_CODE_GROUPS));
        bench_text(" more card groups not shown");
        bench_flush();
    }
    else
    {
        /* The groups are shown. */
    }
}

/** @brief Fourth pass, darkness, on the run's own white and noise
 * (bench_code_white). Bars are the peaks at least the swing above white,
 * other than tape-dark lines; edges are halfway to the valleys either side,
 * as in the card pass. Each bar also keeps its darkness: the readings above
 * the higher of its two valleys, summed across it (bench_code_darkness);
 * each space how far it reads white, below the bars' floor
 * (bench_code_whiteness): blur from two dark bars widens a narrow space at
 * halfway but keeps it off white. Groups as in the card pass, decoded on
 * those two (bench_code39_dark). Turning points need half_sds half noise
 * sds; gap_mm of white ends a group. Quiet (MISSION), white and noise are
 * the cut's (bench_code_dark_tries).
 */
void bench_code_dark (bool b_quiet, uint32_t half_sds, double gap_mm)
{
    uint32_t sd = 1U;
    uint32_t white;
    uint32_t swing;
    uint32_t floor_adc;
    uint32_t index;
    uint32_t point;
    uint32_t before;
    uint32_t after;
    uint32_t start = 0U;
    uint32_t end = 0U;
    uint32_t runs = 0U;
    uint32_t groups = 0U;
    uint32_t space_raw = 0xFFFFU;
    double   rise;
    double   fall;
    double   last_fall = 0.0;
    double   gap_units = (gap_mm / 1000.0) / g_code.unit_m;
    double   lead = -1.0;      /* white before the group; < 0: no bar */
    bool     b_decoded = false;
    bool     b_bar_before = false;  /* since the cut's start or a line */
    uint32_t last_peak = 0U;

    if (!g_code.robot.b_dark_high)
    {
        if (!b_quiet)
        {
            bench_text(" darkness: not run, the side sensor reads dark low");
            bench_flush();
        }
        return;
    }
    if (b_quiet)
    {
        white = g_code.cut_white;
        sd = g_code.cut_sd;
    }
    else
    {
        white = bench_code_white(&sd);
    }
    swing = ((half_sds * sd) + 1U) / 2U;
    floor_adc = white + swing;
    bench_code_card_extrema((uint16_t)swing);
    if (!b_quiet)
    {
        bench_text(" darkness: white ");
        bench_dec((int32_t)white);
        bench_text(" (commonest), noise sd ");
        bench_dec((int32_t)sd);
        bench_text(" (from below white), bars peak >= ");
        bench_dec((int32_t)floor_adc);
        bench_text(", swing >= ");
        bench_dec((int32_t)swing);
        bench_text(", turning points ");
        bench_dec((int32_t)g_code.extremes);
        if (g_code.extremes >= BENCH_CODE_EXTREMA)
        {
            bench_text(" (FULL)");
        }
        bench_flush();
    }
    g_code.b_dark_rule = true;
    /* Peaks are the odd turning points; each needs a valley after it. */
    for (point = 1U; (point + 1U) < g_code.extremes; point += 2U)
    {
        index = g_code.extreme[point];
        before = g_code.extreme[point - 1U];
        after = g_code.extreme[point + 1U];
        if (g_code.sample[before].raw < space_raw)
        {
            space_raw = g_code.sample[before].raw;
        }
        if (bench_code_in_line(index) ||
            (g_code.sample[index].raw < floor_adc))
        {
            if (bench_code_in_line(index) && (0U != runs))
            {
                /* A line ends the group. */
                ++groups;
                g_code.code_lead = lead;
                g_code.code_trail = -1.0;
                bench_code_dark_end(groups, runs, &b_decoded, b_quiet);
                runs = 0U;
            }
            if (bench_code_in_line(index))
            {
                b_bar_before = false;
            }
            continue;
        }
        rise = bench_code_edge(index, before,
                               0.5 * ((double)g_code.sample[before].raw +
                                      (double)g_code.sample[index].raw),
                               &start);
        fall = bench_code_edge(index, after,
                               0.5 * ((double)g_code.sample[index].raw +
                                      (double)g_code.sample[after].raw),
                               &end);
        if ((0U != runs) && ((rise - last_fall) >= gap_units))
        {
            ++groups;
            g_code.code_lead = lead;
            g_code.code_trail = rise - last_fall;
            bench_code_dark_end(groups, runs, &b_decoded, b_quiet);
            runs = 0U;
        }
        if (0U == runs)
        {
            lead = b_bar_before ? (rise - last_fall) : -1.0;
        }
        if ((0U != runs) && ((runs + 2U) <= BENCH_CODE_RUNS))
        {
            g_code.run[runs - 1U].units =
                (uint16_t)bench_round(fmax(0.0, rise - last_fall));
            g_code.run[runs - 1U].peak = (uint16_t)space_raw;
            g_code.run[runs - 1U].dark =
                bench_code_whiteness(last_peak, index, floor_adc);
        }
        if ((runs + 2U) <= BENCH_CODE_RUNS)
        {
            g_code.run[runs].b_black = true;
            g_code.run[runs].start = (uint16_t)start;
            g_code.run[runs].units =
                (uint16_t)bench_round(fmax(0.0, fall - rise));
            g_code.run[runs].peak = g_code.sample[index].raw;
            g_code.run[runs].dark = bench_code_darkness(before, after);
            /* The space after it, closed by the next bar. */
            g_code.run[runs + 1U].b_black = false;
            g_code.run[runs + 1U].start = (uint16_t)end;
            g_code.run[runs + 1U].dark = 0U;
            runs += 2U;
        }
        last_fall = fall;
        last_peak = index;
        space_raw = 0xFFFFU;
        b_bar_before = true;
    }
    if (0U != runs)
    {
        ++groups;
        g_code.code_lead = lead;
        g_code.code_trail = -1.0;
        bench_code_dark_end(groups, runs, &b_decoded, b_quiet);
    }
    g_code.b_dark_rule = false;
    if (b_quiet)
    {
        /* MISSION: the groups went to bench_code_found_add. */
    }
    else if (0U == groups)
    {
        bench_text(" darkness: no bar with a space either side");
        bench_flush();
    }
    else if (groups > BENCH_CODE_GROUPS)
    {
        bench_text(" ");
        bench_dec((int32_t)(groups - BENCH_CODE_GROUPS));
        bench_text(" more darkness groups not shown");
        bench_flush();
    }
    else
    {
        /* The groups are shown. */
    }
}

/** @brief A darkness group's end: shown and decoded (bench_code_card_group),
 * or, quiet (MISSION), kept by bench_code_found_add.
 */
void bench_code_dark_end (uint32_t group, uint32_t runs,
                                 bool * p_decoded, bool b_quiet)
{
    if (b_quiet)
    {
        bench_code_found_add(runs);
    }
    else
    {
        bench_code_card_group(group, runs, p_decoded);
    }
}

/** @brief MISSION, before a cut's tries: the groups already sent are done
 * (their line went out, their vote is in), so they give their room back.
 * Only BENCH_CODE_FOUND (12) were ever kept: with no run limit (22 Nov) a
 * run read no more cards after its 12th group (the sim's long scenario).
 * The run's totals keep counting them (founds_done, codes_done).
 */
static void bench_code_found_drop (void)
{
    uint32_t from;
    uint32_t to = 0U;

    for (from = 0U; from < g_code.founds; ++from)
    {
        if (g_code.found[from].b_sent)
        {
            ++g_code.founds_done;
            if (g_code.found[from].b_framed)
            {
                ++g_code.codes_done;
            }
            continue;
        }
        if (to != from)
        {
            g_code.found[to] = g_code.found[from];
        }
        ++to;
    }
    g_code.founds = to;
}

/** @brief MISSION, a cut (bench_code_stream): its samples frozen at
 * cut_count (later ones are stored after them), tape lines, white and noise
 * found once; its tries then run one per line step (bench_code_tries_step),
 * so the drive loop waits for one darkness pass at most (all of them held
 * it 37 ms on 30 Oct and the encoders faulted).
 */
void bench_code_tries_begin (uint32_t gap_units)
{
    bench_code_found_drop();
    g_code.b_trying = true;
    g_code.try_next = 0U;
    g_code.try_most = 0U;
    g_code.try_best = 0U;
    g_code.try_founds = g_code.founds;
    g_code.try_seen = g_code.groups_seen;
    g_code.b_agreed = false;
    g_code.b_partial_ok = false;
    g_code.b_peak_agreed = false;
    g_code.try_rule = BENCH_CODE_RULE_DARK;
    g_code.sigs = 0U;
    g_code.cut_count = g_code.count;
    g_code.cut_gap = gap_units;
    bench_code_card_lines();
    g_code.cut_white = bench_code_white(&g_code.cut_sd);
}

/** @brief MISSION, one darkness pass of a cut, quiet: try index's rule
 * (BENCH_CODE_TRIES darkness tries, then as many peak tries), group gap
 * (BENCH_CODE_QUIET_MM, then BENCH_CODE_GROUP_GAP_MM) and swing, in half
 * sds: BENCH_CODE_DARK_SDS first, then outward from it within
 * BENCH_CODE_TRY_MIN..BENCH_CODE_TRY_MAX.
 */
void bench_code_try (uint32_t index)
{
    static const double gaps_mm[BENCH_CODE_GAPS] =
    {
        BENCH_CODE_QUIET_MM, BENCH_CODE_GROUP_GAP_MM
    };
    static const uint8_t half_sds[BENCH_CODE_SWINGS] =
    {
        8U, 9U, 7U, 10U, 6U, 11U, 12U
    };
    uint32_t local = index % BENCH_CODE_TRIES;

    g_code.try_rule = (index < BENCH_CODE_TRIES) ? BENCH_CODE_RULE_DARK :
                                                   BENCH_CODE_RULE_PEAKS;
    g_code.try_half_sds = half_sds[local % BENCH_CODE_SWINGS];
    g_code.try_gap_mm = (uint32_t)bench_round(
                            gaps_mm[local / BENCH_CODE_SWINGS]);
    bench_code_dark(true, g_code.try_half_sds,
                    gaps_mm[local / BENCH_CODE_SWINGS]);
    g_code.try_rule = BENCH_CODE_RULE_DARK;
}

/** @brief MISSION, after a try: its framed texts end to end (as read, or
 * reversed with a < after), counted as the vote of its rule and swing for
 * that text; true once BENCH_CODE_AGREE different votes framed the same
 * text, one of them darkness, the first of them then in try_best (a
 * darkness try: the peak tries run after them all).
 */
bool bench_code_try_agree (uint32_t index)
{
    char               text[BENCH_CODE_SIG];
    bench_code_sig_t * p_sig = NULL;
    uint32_t           found;
    uint32_t           sig;
    uint32_t           swings = 0U;
    uint32_t           bit;
    size_t             length = 0U;

    text[0] = '\0';
    for (found = g_code.try_founds; found < g_code.founds; ++found)
    {
        if (g_code.found[found].b_framed)
        {
            (void)strncat(text, g_code.found[found].b_reverse ?
                                g_code.found[found].reverse :
                                g_code.found[found].text,
                          (BENCH_CODE_SIG - 1U) - length);
            length = strlen(text);
            (void)strncat(text, g_code.found[found].b_reverse ? "<|" : "|",
                          (BENCH_CODE_SIG - 1U) - length);
            length = strlen(text);
        }
    }
    if (0U == length)
    {
        return false;
    }
    for (sig = 0U; (sig < g_code.sigs) && (NULL == p_sig); ++sig)
    {
        if (0 == strcmp(g_code.sig[sig].text, text))
        {
            p_sig = &g_code.sig[sig];
        }
    }
    if (NULL == p_sig)
    {
        p_sig = &g_code.sig[g_code.sigs];
        ++g_code.sigs;
        (void)memcpy(p_sig->text, text, sizeof(text));
        p_sig->swings = 0U;
        p_sig->first = index;
    }
    p_sig->swings |= (uint32_t)1U <<
                     (((index / BENCH_CODE_TRIES) * BENCH_CODE_SWINGS) +
                      (index % BENCH_CODE_SWINGS));
    for (bit = 0U; bit < (BENCH_CODE_RULES * BENCH_CODE_SWINGS); ++bit)
    {
        swings += (p_sig->swings >> bit) & 1U;
    }
    if ((swings < BENCH_CODE_AGREE) ||
        (0U == (p_sig->swings & BENCH_CODE_DARK_VOTES)))
    {
        return false;
    }
    g_code.try_best = p_sig->first;
    g_code.b_peak_agreed = (0U != (p_sig->swings >> BENCH_CODE_SWINGS));
    return true;
}

/** @brief MISSION, from the line step: one step of a cut's decode, on its
 * frozen samples. The next try (bench_code_try), its groups dropped after
 * counting the framed ones and their texts' votes (bench_code_try_agree);
 * once BENCH_CODE_AGREE votes agree, or all tries are done, the agreed try
 * (else the first darkness try to frame the most) runs again to keep its
 * groups (bench_code_found_add), framed ones without agreement kept as
 * framed at one swing only, not codes, and the store is cut back
 * (bench_code_keep). The peak tries run only when the darkness tries framed
 * a text without agreeing: alone they can make no code.
 */
void bench_code_tries_step (void)
{
    uint32_t count = g_code.count;
    uint32_t framed = 0U;
    uint32_t index;

    uint64_t start_us = board_time_us();

    bench_fit_run();   /* TRIAL: the fit, a slice of each step */
    bench_slowest(&g_drive.slow_fit_us, start_us);
    if (!g_code.b_trying)
    {
        return;
    }
    g_code.count = g_code.cut_count;
    if (g_code.try_next < BENCH_CODE_ALL_TRIES)
    {
        bench_code_try(g_code.try_next);
        for (index = g_code.try_founds; index < g_code.founds; ++index)
        {
            if (g_code.found[index].b_framed)
            {
                ++framed;
            }
        }
        if ((framed > g_code.try_most) &&
            (g_code.try_next < BENCH_CODE_TRIES))
        {
            g_code.try_most = framed;
            g_code.try_best = g_code.try_next;
        }
        if (bench_code_try_agree(g_code.try_next))
        {
            g_code.b_agreed = true;
            g_code.try_next = BENCH_CODE_ALL_TRIES;
        }
        else
        {
            ++g_code.try_next;
            if ((BENCH_CODE_TRIES == g_code.try_next) && (0U == g_code.sigs))
            {
                g_code.try_next = BENCH_CODE_ALL_TRIES;
            }
        }
        if ((0 != BENCH_CODE_PARTIAL) &&
            (BENCH_CODE_ALL_TRIES == g_code.try_next) && (!g_code.b_agreed) &&
            (!g_code.b_partial_ok))
        {
            /* No whole frame agreed: the tries again, one * now framing. */
            g_code.b_partial_ok = true;
            g_code.try_next = 0U;
            g_code.try_most = 0U;
            g_code.try_best = 0U;
            g_code.sigs = 0U;
        }
        g_code.founds = g_code.try_founds;
        g_code.groups_seen = g_code.try_seen;
        g_code.count = count;
        return;
    }
    bench_code_try(g_code.try_best);
    for (index = g_code.try_founds; index < g_code.founds; ++index)
    {
        if (g_code.found[index].b_framed && (!g_code.b_agreed))
        {
            g_code.found[index].b_framed = false;
            g_code.found[index].b_single = true;
        }
        g_code.found[index].b_peaks = g_code.found[index].b_framed &&
                                      g_code.b_peak_agreed;
        if (g_code.found[index].b_framed)
        {
            /* Confirmed: its letter waits for the next junction. */
            bench_nav_code(&g_code.found[index]);
        }
    }
    if (g_code.groups_seen != g_code.try_seen)
    {
        bench_raw_keep();
    }
    g_code.count = count;
    bench_fit_begin(); /* TRIAL: the cut binned for the fit, before the keep */
    bench_code_keep(g_code.cut_gap);
    g_code.b_open = false;
    g_code.b_trying = false;
    g_code.b_tape = false;
    g_code.b_bar = false;
}

/** @brief MISSION: true while the side sensor crosses a card, from its
 * first mark until the group is cut, BENCH_CODE_QUIET_MM past the last.
 * FOLLOW then drives straight, so the bars cross the sensor square and the
 * front sensors (ROBOT_LINE_SENSOR_FORWARD_M; from 2 Nov 2026 ahead of the
 * side sensor, ROBOT_BARCODE_FORWARD_OFFSET_M) do not steer on the card's
 * bars. It starts at the first printed bar (b_bar), not
 * at noise; tape-long dark (b_tape) before the hold is a line, not a card:
 * no hold, so bends are still followed. Once the hold has started, tape-long
 * dark is a wide bar (the big cards' bars are 18 mm, as wide as the line):
 * the hold goes on (4 Nov: the 13 mm line rule ended every big-card hold).
 */
bool bench_code_hold (void)
{
    return g_code.b_stream && g_code.b_open && g_code.b_bar &&
           (!g_code.b_trying) && ((!g_code.b_tape) || g_code.b_held);
}

/** @brief MISSION, after the stop: a cut being decoded, or the open group,
 * decoded to the end at once. The watchdog (BENCH_WATCHDOG_MS, still running
 * after the stop) is fed each try step: no bench tick runs here (4 Nov: a
 * card still open at the stop reset the car before its read was sent).
 */
void bench_code_dark_tries (void)
{
    if (!g_code.b_trying)
    {
        bench_code_tries_begin((uint32_t)bench_round(
            (BENCH_CODE_QUIET_MM / 1000.0) / g_code.unit_m));
    }
    while (g_code.b_trying)
    {
        bench_code_tries_step();
        mission_watchdog_feed();
    }
}

/** @brief The run's white: its commonest stored side sensor reading
 * (g_code_levels, every sample since the start). *p_sd is that white's
 * noise, one standard deviation, from the readings below it alone: white
 * less the level BENCH_CODE_ONE_SD_SHARE of the way up them (at least 1).
 */
uint32_t bench_code_white (uint32_t * p_sd)
{
    uint32_t level;
    uint32_t white = 0U;
    uint32_t below = 0U;
    uint32_t counted = 0U;
    double   target;

    for (level = 1U; level < BENCH_CODE_ADC_LEVELS; ++level)
    {
        if (g_code_levels[level] > g_code_levels[white])
        {
            white = level;
        }
    }
    for (level = 0U; level < white; ++level)
    {
        below += g_code_levels[level];
    }
    target = BENCH_CODE_ONE_SD_SHARE *
             ((double)below + (0.5 * (double)g_code_levels[white]));
    level = 0U;
    while ((level < white) &&
           ((double)(counted + g_code_levels[level]) < target))
    {
        counted += g_code_levels[level];
        ++level;
    }
    *p_sd = (white > level) ? (white - level) : 1U;
    return white;
}

/** @brief A bar's darkness: from valley before to valley after, the
 * readings above the higher of the two, times the units each spans.
 */
uint32_t bench_code_darkness (uint32_t before, uint32_t after)
{
    uint32_t base = g_code.sample[before].raw;
    uint32_t dark = 0U;
    uint32_t index;

    if (g_code.sample[after].raw > base)
    {
        base = g_code.sample[after].raw;
    }
    for (index = before; index < after; ++index)
    {
        if (g_code.sample[index].raw > base)
        {
            dark += ((uint32_t)g_code.sample[index].raw - base) *
                    ((uint32_t)g_code.sample[index + 1U].unit -
                     (uint32_t)g_code.sample[index].unit);
        }
    }
    return dark;
}

/** @brief A space's whiteness: the units from bar peak from to bar peak to
 * that read below floor_adc.
 */
uint32_t bench_code_whiteness (uint32_t from, uint32_t to,
                                      uint32_t floor_adc)
{
    uint32_t white = 0U;
    uint32_t index;

    for (index = from; index < to; ++index)
    {
        if ((uint32_t)g_code.sample[index].raw < floor_adc)
        {
            white += (uint32_t)g_code.sample[index + 1U].unit -
                     (uint32_t)g_code.sample[index].unit;
        }
    }
    return white;
}

/** @brief Darkness pass, in rows: each bar's darkness, d in raw x mm, and
 * each space's whiteness, w in mm.
 */
void bench_code_dark_row (uint32_t first, uint32_t last)
{
    uint32_t index;
    uint32_t column = 0U;

    bench_text("  darkness:");
    for (index = first; index <= last; ++index)
    {
        if (g_code.run[index].b_black)
        {
            bench_text(" d");
            bench_dec(bench_round((double)g_code.run[index].dark *
                                  g_code.unit_m * 1000.0));
        }
        else
        {
            bench_text(" w");
            bench_fixed(bench_round((double)g_code.run[index].dark *
                                    g_code.unit_m * 10000.0), 1U);
        }
        ++column;
        if ((BENCH_CODE_DARK_PER_ROW == column) && (index < last))
        {
            bench_flush();
            bench_text("  ");
            column = 0U;
        }
    }
    bench_flush();
}

/** @brief A card group of runs bar, space, ... bar, space (the last space
 * open): shown and decoded as runs 0..runs - 2.
 */
void bench_code_card_group (uint32_t group, uint32_t runs,
                                   bool * p_decoded)
{
    if (group > BENCH_CODE_GROUPS)
    {
        return;
    }
    bench_text(g_code.b_dark_rule ? " darkness group " : " card group ");
    bench_dec((int32_t)group);
    if (bench_code_group(0U, runs - 2U))
    {
        *p_decoded = true;
    }
}

/** @brief Tape-dark spans (the robot's DARK) at least BENCH_CODE_LINE_M of
 * sensor path long: a line under the side sensor, not a bar.
 */
void bench_code_card_lines (void)
{
    uint32_t index;
    uint32_t first = 0U;
    bool     b_dark = false;
    bool     b_now;

    g_code.lines = 0U;
    for (index = 0U; index <= g_code.count; ++index)
    {
        b_now = (index < g_code.count) &&
                (g_code.sample[index].raw >= g_code.robot.on_adc);
        if (b_now && (!b_dark))
        {
            first = index;
        }
        else if ((!b_now) && b_dark &&
                 (((double)(g_code.sample[index - 1U].unit -
                            g_code.sample[first].unit) * g_code.unit_m) >=
                  BENCH_CODE_LINE_M) &&
                 (g_code.lines < BENCH_CODE_LINES))
        {
            g_code.line_first[g_code.lines] = (uint16_t)first;
            g_code.line_last[g_code.lines] = (uint16_t)(index - 1U);
            ++g_code.lines;
        }
        else
        {
            /* Inside a span, or between. */
        }
        b_dark = b_now;
    }
}

/** @brief True if sample index lies in a tape-dark line span. */
bool bench_code_in_line (uint32_t index)
{
    uint32_t line;

    for (line = 0U; line < g_code.lines; ++line)
    {
        if ((index >= g_code.line_first[line]) &&
            (index <= g_code.line_last[line]))
        {
            return true;
        }
    }
    return false;
}

/** @brief Turning points of the side sensor, a valley first: a valley ends
 * when the reading rises swing above it, a peak when it falls swing below.
 */
void bench_code_card_extrema (uint16_t swing)
{
    uint32_t index;
    uint32_t turn = 0U;
    uint32_t raw;
    uint32_t held;
    bool     b_peak = false;

    g_code.extremes = 0U;
    for (index = 1U; index < g_code.count; ++index)
    {
        raw = g_code.sample[index].raw;
        held = g_code.sample[turn].raw;
        if (b_peak ? (raw > held) : (raw < held))
        {
            turn = index;
        }
        else if (b_peak ? ((raw + swing) <= held) : (raw >= (held + swing)))
        {
            if (g_code.extremes >= BENCH_CODE_EXTREMA)
            {
                return;
            }
            g_code.extreme[g_code.extremes] = (uint16_t)turn;
            ++g_code.extremes;
            turn = index;
            b_peak = !b_peak;
        }
        else
        {
            /* Within swing of the held turning point. */
        }
    }
    if ((0U != g_code.count) && (g_code.extremes < BENCH_CODE_EXTREMA))
    {
        g_code.extreme[g_code.extremes] = (uint16_t)turn;
        ++g_code.extremes;
    }
}

/** @brief From a peak toward a valley, where the reading first falls below
 * level, in decoder units interpolated between the two samples either side;
 * *p_index is the sample on the valley's side of it.
 */
double bench_code_edge (uint32_t peak, uint32_t valley, double level,
                               uint32_t * p_index)
{
    uint32_t index = peak;
    uint32_t inner = peak;
    double   inner_raw;
    double   outer_raw;
    double   share;

    while (index != valley)
    {
        inner = index;
        index = (valley > peak) ? (index + 1U) : (index - 1U);
        if ((double)g_code.sample[index].raw < level)
        {
            break;
        }
    }
    *p_index = index;
    inner_raw = (double)g_code.sample[inner].raw;
    outer_raw = (double)g_code.sample[index].raw;
    share = (inner_raw > outer_raw) ?
            ((inner_raw - level) / (inner_raw - outer_raw)) : 0.0;
    return (double)g_code.sample[inner].unit +
           (share * ((double)g_code.sample[index].unit -
                     (double)g_code.sample[inner].unit));
}

/** @brief One dark group (runs first..last, bars at both ends): where, how
 * many bars, the weakest bar and space, widths and the Code 39 decode.
 */
bool bench_code_group (uint32_t first, uint32_t last)
{
    uint32_t index;
    uint32_t weakest_bar = 0xFFFFU;
    uint32_t weakest_space = 0U;

    for (index = first; index <= last; ++index)
    {
        if (g_code.run[index].b_black &&
            (g_code.run[index].peak < weakest_bar))
        {
            weakest_bar = g_code.run[index].peak;
        }
        if ((!g_code.run[index].b_black) &&
            (g_code.run[index].peak > weakest_space))
        {
            weakest_space = g_code.run[index].peak;
        }
    }
    bench_text(" at ");
    bench_code_unit_mm(g_code.sample[g_code.run[first].start].unit);
    bench_text(" to ");
    bench_code_unit_mm(g_code.sample[g_code.run[last].start].unit +
                       g_code.run[last].units);
    bench_text(" mm: ");
    bench_dec((int32_t)(((last - first) / 2U) + 1U));
    bench_text(" bars, ");
    bench_dec((int32_t)((last - first) + 1U));
    bench_text(" runs; weakest bar peak raw ");
    bench_dec((int32_t)weakest_bar);
    if (last > first)
    {
        bench_text(", weakest space raw ");
        bench_dec((int32_t)weakest_space);
    }
    bench_flush();
    bench_code_widths(first, last);
    if (g_code.b_dark_rule)
    {
        bench_code_dark_row(first, last);
    }
    return bench_code39(first, last);
}

/** @brief The group's run widths in mm, b bar and w space, in rows. */
void bench_code_widths (uint32_t first, uint32_t last)
{
    uint32_t index;
    uint32_t column = 0U;

    bench_text("  widths mm:");
    for (index = first; index <= last; ++index)
    {
        bench_text(g_code.run[index].b_black ? " b" : " w");
        bench_fixed(bench_round((double)g_code.run[index].units *
                                g_code.unit_m * 10000.0), 1U);
        ++column;
        if ((BENCH_CODE_RUNS_PER_ROW == column) && (index < last))
        {
            bench_flush();
            bench_text("  ");
            column = 0U;
        }
    }
    bench_flush();
}

/** @brief Code 39 decode of a group, as read, else reversed (the car may
 * have passed the code end to start); a frame must start and end with *,
 * with a character between.
 */
bool bench_code39 (uint32_t first, uint32_t last)
{
    char     forward[BENCH_CODE_TEXT];
    char     reverse[BENCH_CODE_TEXT];
    uint32_t runs = (last - first) + 1U;
    bool     b_forward = false;
    bool     b_reverse = false;
    uint32_t length = 0U;

    bench_text("  Code 39: ");
    if ((0U != ((runs + 1U) % (BENCH_CODE_ELEMENTS + 1U))) ||
        (((runs + 1U) / (BENCH_CODE_ELEMENTS + 1U)) >= BENCH_CODE_TEXT))
    {
        bench_text("no, ");
        bench_dec((int32_t)runs);
        bench_text(" runs (a code has 10 x characters - 1, at most ");
        bench_dec((int32_t)(BENCH_CODE_TEXT - 1U));
        bench_text(" characters)");
        bench_flush();
        return false;
    }
    length = (runs + 1U) / (BENCH_CODE_ELEMENTS + 1U);
    b_forward = bench_code39_text(first, last, false, forward) &&
                (length >= BENCH_CODE_FRAMED) &&
                ('*' == forward[0]) && ('*' == forward[length - 1U]);
    if (!b_forward)
    {
        b_reverse = bench_code39_text(first, last, true, reverse) &&
                    (length >= BENCH_CODE_FRAMED) &&
                    ('*' == reverse[0]) && ('*' == reverse[length - 1U]);
        if (!b_reverse)
        {
            bench_text("no *...* frame: as read ");
            bench_text(forward);
            bench_text(", reversed ");
            bench_text(reverse);
            bench_text(" (? not a Code 39 character)");
            bench_flush();
            return false;
        }
    }
    bench_text(b_forward ? forward : reverse);
    bench_text(b_forward ? " read in order" : " read end to start");
    if ((0U != g_code.narrow_count) && (0U != g_code.wide_count))
    {
        bench_text(", narrow mean ");
        bench_fixed(bench_round(((double)g_code.narrow_units /
                                 (double)g_code.narrow_count) *
                                g_code.unit_m * 10000.0), 1U);
        bench_text(" mm, wide mean ");
        bench_fixed(bench_round(((double)g_code.wide_units /
                                 (double)g_code.wide_count) *
                                g_code.unit_m * 10000.0), 1U);
        bench_text(" mm");
    }
    bench_flush();
    return true;
}

/** @brief Darkness pass: one character's pattern, its 2 darkest of 5 bars
 * wide and its whitest of 4 spaces, judged apart so blur that thins bars
 * and widens spaces cannot mix the two. 0 (no character) unless each wide
 * element is clear of every narrow one of its kind. Code 39's $ / + % (3
 * wide spaces, no wide bar) are not read. In MISSION's peak tries
 * (BENCH_CODE_RULE_PEAKS) the bars are judged by their peaks and the spaces
 * by their valleys, the lowest wide, instead.
 */
uint32_t bench_code39_dark (uint32_t first, uint32_t last,
                                   uint32_t base, bool b_reverse)
{
    uint32_t value[BENCH_CODE_ELEMENTS];
    uint32_t pattern = 0U;
    uint32_t element;
    uint32_t kind;
    uint32_t pick;
    uint32_t widest;
    uint32_t narrowest_wide;
    uint32_t widest_narrow;
    uint32_t run;

    for (element = 0U; element < BENCH_CODE_ELEMENTS; ++element)
    {
        run = b_reverse ? (last - (base + element)) : (first + base + element);
        if (BENCH_CODE_RULE_PEAKS != g_code.try_rule)
        {
            value[element] = g_code.run[run].dark;
        }
        else if (0U == (element % 2U))
        {
            value[element] = (uint32_t)g_code.run[run].peak;
        }
        else
        {
            value[element] = 0xFFFFU - (uint32_t)g_code.run[run].peak;
        }
    }
    /* Kind 0: the bars (even elements), 2 wide; kind 1: the spaces, 1. */
    for (kind = 0U; kind < BENCH_CODE_KINDS; ++kind)
    {
        for (pick = 0U; pick < (2U - kind); ++pick)
        {
            widest = BENCH_CODE_ELEMENTS;
            for (element = kind; element < BENCH_CODE_ELEMENTS;
                 element += 2U)
            {
                if ((0U == (pattern & (0x100U >> element))) &&
                    ((BENCH_CODE_ELEMENTS == widest) ||
                     (value[element] > value[widest])))
                {
                    widest = element;
                }
            }
            pattern |= 0x100U >> widest;
        }
        narrowest_wide = UINT32_MAX;
        widest_narrow = 0U;
        for (element = kind; element < BENCH_CODE_ELEMENTS; element += 2U)
        {
            if (0U != (pattern & (0x100U >> element)))
            {
                narrowest_wide = (value[element] < narrowest_wide) ?
                                 value[element] : narrowest_wide;
            }
            else
            {
                widest_narrow = (value[element] > widest_narrow) ?
                                value[element] : widest_narrow;
            }
        }
        if (narrowest_wide <= widest_narrow)
        {
            return 0U;
        }
    }
    return pattern;
}

/** @brief MISSION, the Code 39 spec checks of a frame of length characters
 * from run first of a group ending at run last (BENCH_CODE_CHAR_MIN_X): each
 * character within one narrow width of the frame's median character, each
 * gap between characters at most BENCH_CODE_GAP_MAX_X narrow widths, and a
 * bar before or after it at least BENCH_CODE_QUIET_X narrow widths away.
 * The narrow width is the median character over BENCH_CODE_CHAR_MIN_X for
 * the first two, the widest the spec allows, and over BENCH_CODE_CHAR_MAX_X
 * for the quiet zone, the narrowest: each check as lenient as the spec. A
 * space narrower than that narrowest width is no Code 39 space but a speck
 * in the bar beside it (31 Oct: 0.3 mm splitting *Z*'s first bar), so the
 * quiet zone is measured past it; at the group's ends the group's own
 * (code_lead, code_trail; < 0 when no bar bounds it). BENCH_SPEC_OK, else
 * the check that refused it (counted).
 */
uint32_t bench_code39_spec (uint32_t first, uint32_t length,
                                   uint32_t last)
{
    uint32_t span[BENCH_CODE_TEXT];
    uint32_t sorted[BENCH_CODE_TEXT];
    uint32_t character;
    uint32_t element;
    uint32_t pick;
    uint32_t swap;
    uint32_t base;
    double   median;
    double   narrow;
    double   narrowest;
    double   quiet;
    double   lead;
    double   trail;
    bool     b_side;
    uint32_t result = BENCH_SPEC_OK;

    if ((0U == length) || (length >= BENCH_CODE_TEXT))
    {
        return BENCH_SPEC_OK;
    }
    for (character = 0U; character < length; ++character)
    {
        base = first + (character * (BENCH_CODE_ELEMENTS + 1U));
        span[character] = 0U;
        for (element = 0U; element < BENCH_CODE_ELEMENTS; ++element)
        {
            span[character] += g_code.run[base + element].units;
        }
        sorted[character] = span[character];
        for (pick = character;
             (pick > 0U) && (sorted[pick - 1U] > sorted[pick]); --pick)
        {
            swap = sorted[pick];
            sorted[pick] = sorted[pick - 1U];
            sorted[pick - 1U] = swap;
        }
    }
    median = (0U != (length % 2U)) ?
             (double)sorted[length / 2U] :
             (0.5 * ((double)sorted[(length / 2U) - 1U] +
                     (double)sorted[length / 2U]));
    narrow = median / BENCH_CODE_CHAR_MIN_X;
    narrowest = median / BENCH_CODE_CHAR_MAX_X;
    quiet = BENCH_CODE_QUIET_X * narrowest;
    /* The white before the frame's first bar, past specks. */
    lead = g_code.code_lead;
    base = first;
    b_side = false;
    while ((0U != base) && (!b_side))
    {
        if ((double)g_code.run[base - 1U].units >= narrowest)
        {
            lead = (double)g_code.run[base - 1U].units;
            b_side = true;
        }
        else
        {
            base -= 2U;
        }
    }
    /* And after its last bar. */
    trail = g_code.code_trail;
    base = first + ((length * (BENCH_CODE_ELEMENTS + 1U)) - 2U);
    b_side = false;
    while ((base < last) && (!b_side))
    {
        if ((double)g_code.run[base + 1U].units >= narrowest)
        {
            trail = (double)g_code.run[base + 1U].units;
            b_side = true;
        }
        else
        {
            base += 2U;
        }
    }
    for (character = 0U;
         (character < length) && (BENCH_SPEC_OK == result); ++character)
    {
        if (fabs((double)span[character] - median) > narrow)
        {
            result = BENCH_SPEC_WIDTHS;
            ++g_code.spec_widths;
        }
    }
    for (character = 0U;
         ((character + 1U) < length) && (BENCH_SPEC_OK == result);
         ++character)
    {
        base = first + (character * (BENCH_CODE_ELEMENTS + 1U));
        if ((double)g_code.run[base + BENCH_CODE_ELEMENTS].units >
            (BENCH_CODE_GAP_MAX_X * narrow))
        {
            result = BENCH_SPEC_GAPS;
            ++g_code.spec_gaps;
        }
    }
    if ((BENCH_SPEC_OK == result) &&
        (((lead >= 0.0) && (lead < quiet)) ||
         ((trail >= 0.0) && (trail < quiet))))
    {
        result = BENCH_SPEC_QUIET;
        ++g_code.spec_quiet;
    }
    return result;
}

/** @brief Characters of a group, the 3 widest of each 9 elements wide (the
 * darkness pass: bench_code39_dark); '?' where that is not a Code 39
 * pattern or a wide element is not wider than every narrow one. True if
 * every character decoded.
 */
bool bench_code39_text (uint32_t first, uint32_t last, bool b_reverse,
                               char * p_text)
{
    uint32_t length = (((last - first) + 1U) + 1U) /
                      (BENCH_CODE_ELEMENTS + 1U);
    uint32_t character;
    uint32_t element;
    uint32_t pick;
    uint32_t widest;
    uint32_t pattern;
    uint32_t index;
    uint32_t units[BENCH_CODE_ELEMENTS];
    uint32_t narrowest_wide;
    uint32_t widest_narrow;
    bool     b_all = true;

    g_code.narrow_units = 0U;
    g_code.narrow_count = 0U;
    g_code.wide_units = 0U;
    g_code.wide_count = 0U;
    for (character = 0U; character < length; ++character)
    {
        for (element = 0U; element < BENCH_CODE_ELEMENTS; ++element)
        {
            units[element] = bench_code_units(first, last,
                (character * (BENCH_CODE_ELEMENTS + 1U)) + element,
                b_reverse);
        }
        pattern = 0U;
        if (g_code.b_dark_rule)
        {
            pattern = bench_code39_dark(first, last,
                                        character * (BENCH_CODE_ELEMENTS + 1U),
                                        b_reverse);
        }
        for (pick = 0U; (pick < BENCH_CODE_WIDE) && (!g_code.b_dark_rule);
             ++pick)
        {
            widest = BENCH_CODE_ELEMENTS;
            for (element = 0U; element < BENCH_CODE_ELEMENTS; ++element)
            {
                if ((0U == (pattern & (0x100U >> element))) &&
                    ((BENCH_CODE_ELEMENTS == widest) ||
                     (units[element] > units[widest])))
                {
                    widest = element;
                }
            }
            pattern |= 0x100U >> widest;
        }
        narrowest_wide = UINT32_MAX;
        widest_narrow = 0U;
        for (element = 0U; element < BENCH_CODE_ELEMENTS; ++element)
        {
            if (0U != (pattern & (0x100U >> element)))
            {
                g_code.wide_units += units[element];
                ++g_code.wide_count;
                if (units[element] < narrowest_wide)
                {
                    narrowest_wide = units[element];
                }
            }
            else
            {
                g_code.narrow_units += units[element];
                ++g_code.narrow_count;
                if (units[element] > widest_narrow)
                {
                    widest_narrow = units[element];
                }
            }
        }
        index = 0U;
        while ((index < BENCH_CODE_CHARS) &&
               (g_code39_patterns[index] != pattern))
        {
            ++index;
        }
        if ((index < BENCH_CODE_CHARS) &&
            (g_code.b_dark_rule || (narrowest_wide > widest_narrow)))
        {
            p_text[character] = g_code39_chars[index];
        }
        else
        {
            p_text[character] = '?';
            b_all = false;
        }
    }
    p_text[length] = '\0';
    return b_all;
}

/** @brief Width of element index of a group, counted from either end. */
uint32_t bench_code_units (uint32_t first, uint32_t last,
                                  uint32_t index, bool b_reverse)
{
    return g_code.run[b_reverse ? (last - index) : (first + index)].units;
}

/** @brief The side sensor every 1 mm of its own path (the first sample in
 * each mm) over the whole run, at most BENCH_CODE_TRACE_MAX_MM; each row
 * starts with that mm and the centre path's mm there.
 */
void bench_code_trace (void)
{
    uint32_t index = 0U;
    uint32_t column = 0U;
    int32_t  to_mm;
    int32_t  mm;

    to_mm = bench_round((double)g_code.sample[g_code.count - 1U].unit *
                        g_code.unit_m * 1000.0);
    if (to_mm > BENCH_CODE_TRACE_MAX_MM)
    {
        to_mm = BENCH_CODE_TRACE_MAX_MM;
    }
    bench_text(" side sensor raw every 1 mm of its own path (high is dark)"
               " from 0 to ");
    bench_dec(to_mm);
    bench_text(" mm; row: sensor mm c centre mm:");
    bench_flush();
    for (mm = 0; mm <= to_mm; ++mm)
    {
        while ((index < g_code.count) &&
               (((double)g_code.sample[index].unit * g_code.unit_m *
                 1000.0) < (double)mm))
        {
            ++index;
        }
        if (index >= g_code.count)
        {
            break;
        }
        if (0U == column)
        {
            bench_text("  ");
            bench_dec(mm);
            bench_text(" c");
            bench_code_unit_mm(g_code.sample[index].centre);
            bench_text(":");
        }
        bench_text(" ");
        bench_dec((int32_t)g_code.sample[index].raw);
        ++column;
        if (BENCH_CODE_TRACE_PER_ROW == column)
        {
            bench_flush();
            column = 0U;
        }
    }
    if (0U != column)
    {
        bench_flush();
    }
}

/** @brief Decoder units as whole millimetres. */
void bench_code_unit_mm (uint32_t units)
{
    /* The side sensor's path from the start, 0-2097151 mm (21 bits). */
    bench_u((uint32_t)bench_round((double)units * g_code.unit_m * 1000.0), 21U);
}

void bench_code (uint32_t value, uint32_t bits)
{
    bench_frame_put(bench_frame_unsigned(value, bits), bits);
}

/* ==== TRIAL 2 Nov 2026: model-fit barcode decoder, Decoder 2 ===========
 * Offline work in testing/barcode_lab (see the README's barcode
 * section). Each cut the darkness tries finish is copied here as 1 mm bins
 * (bench_fit_begin), so the store and the tries go on at once. Cuts whose
 * dark is less than BENCH_FIT_JOIN_MM apart are joined first (the stream
 * cuts at 30 mm of white, and a blurred small card whose narrow bars vanish
 * has up to 45 mm between wide bars); the fit then runs over the next line
 * steps, at most BENCH_FIT_BUDGET_US of each, and sends its own reading as
 * one Decoder 2 line. It changes nothing the darkness tries do or report (CODE
 * /
 * GROUP lines), nor the driving.
 *
 * Stage 1: every Code 39 symbol *X* has six wide bars (X any character
 * with two wide bars: all but $ / + %). Six of the BENCH_FIT_TOP humps with
 * the most darkness area are taken as them, every choice tried (a line or a
 * sheet's edge beside the card can be large too); their centres are fitted
 * (start and module, least squares) to each candidate's six wide-bar
 * centres, both directions; the BENCH_FIT_SHORT closest candidates (rms per
 * module) go on.
 * Stage 2: each short-listed candidate's bars are blurred by a Gaussian
 * spot (sd sigma), a dead zone c0 applied (the reading moves only once c0
 * of the spot is dark) and the result scaled to the cut's white and black
 * by least squares; the misfit is the sum of squares left. Spot and dead
 * zone are the sensor's, one for the pass: fitted over the candidates of
 * the closest wide-bar class (SHAPE), then every candidate fitted near its
 * stage-1 module and start with them a little either way (CANDS), then
 * module and start refined (FINE).
 * The best fit is reported when the best other character's misfit is at
 * least BENCH_FIT_MARGIN times larger, else "no read" with both.
 * Constants: the Code 39 table (g_code39_patterns) and its 1:3 ratio, the
 * spec's 3 mm narrow width for the join, and search ranges wide enough for
 * the sensor's spot and dead zone (the 30 Oct big-card fit gave sigma
 * 3.3-3.8 mm, c0 0.45-0.55). No letter, card size or track position.
 */
/* 15 modules between wide bars + 3, at 3 mm */
#define BENCH_FIT_JOIN_MM (54.0)
#define BENCH_FIT_PAD_MM (30.0)     /* white kept either side of the dark */
#define BENCH_FIT_LONGEST_MM (420.0) /* longer dark: not one card */
#define BENCH_FIT_TPL_Q (4)         /* template points per mm */
#define BENCH_FIT_TPL (1800U)       /* template points: 450 mm */
#define BENCH_FIT_TPL_PAD_MM (20.0) /* template white before the first edge */
#define BENCH_FIT_PHI_N (1024U)     /* normal CDF table over -4..+4 sd */
#define BENCH_FIT_ONE (4096)        /* coverage 1.0 */
#define BENCH_FIT_S_ONE (1024)      /* response 1.0 */
#define BENCH_FIT_PHI_SLICE (6U)    /* normal CDF entries (one erf each, ~0.2 ms
                                     * on the M0+) built per line step */
#define BENCH_FIT_CHAR_SLICE (4U)   /* characters' tables built per line step */
#define BENCH_FIT_EDGES (30U)       /* 15 bars of *X* */
#define BENCH_FIT_MODULES (47.0f)   /* modules of *X* */
#define BENCH_FIT_HUMPS (48U)
#define BENCH_FIT_LSQ_SLICE (26U)   /* stage-1 candidates per slice */
#define BENCH_FIT_BUDGET_US (1500U)
/* Standing at a junction with only the fit left to decode the card, and
 * neither Decoder 1 nor D0 with a letter (23 Nov: most of the CPU goes to
 * Decoder 2 then). The wheels are halted (no motion lease), so a step may
 * run long: 20 ms of fit, then the Vehicle task's 5 ms sleep, in which the
 * lower tasks (IMU, Scan, Comm, the radio) run; their 100 ms owner gates
 * see gaps near 30 ms. About three quarters of the time goes to the fit.
 */
#define BENCH_FIT_BOOST_US (20000U)
#define BENCH_FIT_MARGIN (1.3f)
#define BENCH_FIT_MISFIT (0.35f)    /* stage-1 rms limit, in modules */
#define BENCH_FIT_REL_PCT (15)      /* turning-point swing: % of the level */
#define BENCH_FIT_SDS (6)           /* ... or this many white noise sds */
#define BENCH_FIT_C0S (6U)
#define BENCH_FIT_X0S (13U)         /* start: -3..+3 mm by 0.5 */
#define BENCH_FIT_SHAPE_X0S (7U)    /* start: -1.5..+1.5 mm by 0.5 */
#define BENCH_FIT_FINE_X0S (7U)     /* start: -0.75..+0.75 mm by 0.25 */
#define BENCH_FIT_SHAPE_TPLS (18U)  /* 3 modules x 6 spots */
#define BENCH_FIT_CANDS_TPLS (21U)  /* 7 modules x 3 spots */
#define BENCH_FIT_FINE_TPLS (5U)
#define BENCH_FIT_LINES (4U)        /* Decoder 2 lines waiting for the link */

bench_fit_t g_fit;

/* A Decoder 2 line waiting to go out: the fit goes on whatever the link
 * does. */
typedef struct
{
    bench_fit_why_t why;
    bool     b_read;
    bool     b_rev;
    bool     b_other_rev;
    char     ch;
    char     other;
    float    margin;
    float    m;
    float    sigma;
    float    c0;
    float    rms;
    float    from;
    float    to;
    uint32_t cpu_us;
    uint32_t steps;
    uint32_t work;
    uint32_t joins;
} bench_fit_line_t;
static bench_fit_line_t g_fit_lines[BENCH_FIT_LINES];
static uint32_t    g_fit_line_first;
   /* oldest waiting */
uint32_t    g_fit_line_count;
static uint32_t    g_fit_lines_lost;
   /* none free: the line dropped */
static bench_fit_line_t * gp_fit_line_new;
bench_fit_bin_t g_fit_bin;
static int32_t     g_fit_sum[BENCH_FIT_BINS];
static uint16_t    g_fit_cnt[BENCH_FIT_BINS];
static int16_t     g_fit_tpl[BENCH_FIT_TPL];
static uint16_t    g_fit_phi[BENCH_FIT_PHI_N + 1U];
static char        g_fit_char[BENCH_FIT_CANDS];
static bool        gb_fit_rev[BENCH_FIT_CANDS];
static uint8_t     g_fit_edge[BENCH_FIT_CANDS][BENCH_FIT_EDGES];
 /* modules */
static float       g_fit_wide_dev[BENCH_FIT_CANDS][BENCH_FIT_WIDE];
 /* centre - mean */
static float       g_fit_wide_mean[BENCH_FIT_CANDS];
static float       g_fit_wide_inv[BENCH_FIT_CANDS];
    /* 1 / sum dev^2 */
static bool        gb_fit_tables;
bool        gb_fit_force;
bool        gb_fit_boost;   /* bench_nav_wait: BENCH_FIT_BOOST_US a step */

  /* stopped at a junction for the decode
                                   * (bench_nav_wait): the binned card goes
                                   * to the fit without the white after it */
static const float g_fit_shape_m[3] = { 0.985f, 1.0f, 1.015f };
static const float g_fit_shape_sigma[6] =
{
    1.5f, 2.1f, 2.7f, 3.3f, 3.9f, 4.5f
};
static const float g_fit_shape_c0[BENCH_FIT_C0S] =
{
    0.15f, 0.25f, 0.35f, 0.45f, 0.55f, 0.65f
};

/** @brief The candidates from the Code 39 table: each character with two
 * wide bars inside *s, as read and reversed; bar edges in modules (narrow
 * 1, wide 3, 1 between characters) and the six wide bars' centres; the
 * normal CDF table for the templates.
 */
static bool bench_fit_tables (void)
{
    static uint32_t phi_i;     /* built so far, resumed by the next call */
    static uint32_t next_c;
    static uint32_t next_n;
    uint32_t c;
    uint32_t n = next_n;
    uint32_t i;
    uint32_t star = 0U;
    uint32_t r;
    uint32_t t;
    double   z;

    if (gb_fit_tables)
    {
        return true;
    }
    if (phi_i <= (BENCH_FIT_PHI_N / 2U))
    {
        /* The lower half only: Phi(-z) = 1 - Phi(z). */
        for (t = 0U; (t < BENCH_FIT_PHI_SLICE) &&
                     (phi_i <= (BENCH_FIT_PHI_N / 2U)); ++t, ++phi_i)
        {
            z = -4.0 + (8.0 * (double)phi_i / (double)BENCH_FIT_PHI_N);
            g_fit_phi[phi_i] = (uint16_t)bench_round(
                (double)BENCH_FIT_ONE * 0.5 * (1.0 + erf(z / sqrt(2.0))));
            g_fit_phi[BENCH_FIT_PHI_N - phi_i] =
                (uint16_t)(BENCH_FIT_ONE - g_fit_phi[phi_i]);
        }
        return false;
    }
    for (c = 0U; c < BENCH_CODE_CHARS; ++c)
    {
        if ('*' == g_code39_chars[c])
        {
            star = c;
        }
    }
    for (c = next_c, t = 0U;
         (c < BENCH_CODE_CHARS) && ((n + 1U) < BENCH_FIT_CANDS) &&
         (t < BENCH_FIT_CHAR_SLICE);
         ++c, ++t)
    {
        uint16_t pat[BENCH_CODE_FRAMED];
        uint8_t  width[29];
        uint8_t  bar[29];
        uint32_t e = 0U;
        uint32_t ch;

        if (NULL != strchr("*$/+%", g_code39_chars[c]))
        {
            continue;
        }
        pat[0] = g_code39_patterns[star];
        pat[1] = g_code39_patterns[c];
        pat[2] = g_code39_patterns[star];
        for (ch = 0U; ch < BENCH_CODE_FRAMED; ++ch)
        {
            for (i = 0U; i < BENCH_CODE_ELEMENTS; ++i)
            {
                width[e] = (0U != ((pat[ch] >> (8U - i)) & 1U)) ? 3U : 1U;
                bar[e] = (0U == (i % 2U)) ? 1U : 0U;
                ++e;
            }
            if (ch < 2U)
            {
                width[e] = 1U;
                bar[e] = 0U;
                ++e;
            }
        }
        for (r = 0U; r < BENCH_CODE_DIRECTIONS; ++r)
        {
            uint32_t pos = 0U;
            uint32_t edge = 0U;
            uint32_t wide = 0U;
            float    centre[BENCH_FIT_WIDE] =
                { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
            float    sum = 0.0f;
            float    sq = 0.0f;

            g_fit_char[n] = g_code39_chars[c];
            gb_fit_rev[n] = (1U == r);
            for (i = 0U; i < e; ++i)
            {
                uint32_t k = (0U == r) ? i : (e - 1U - i);

                if ((0U != bar[k]) && (edge < BENCH_FIT_EDGES))
                {
                    g_fit_edge[n][edge] = (uint8_t)pos;
                    g_fit_edge[n][edge + 1U] = (uint8_t)(pos + width[k]);
                    edge += 2U;
                    if ((3U == width[k]) && (wide < BENCH_FIT_WIDE))
                    {
                        centre[wide] = (float)pos + 1.5f;
                        sum += centre[wide];
                        ++wide;
                    }
                }
                pos += width[k];
            }
            g_fit_wide_mean[n] = sum / 6.0f;
            for (i = 0U; i < BENCH_FIT_WIDE; ++i)
            {
                g_fit_wide_dev[n][i] = centre[i] - g_fit_wide_mean[n];
                sq += g_fit_wide_dev[n][i] * g_fit_wide_dev[n][i];
            }
            g_fit_wide_inv[n] = (sq > 0.0f) ? (1.0f / sq) : 0.0f;
            ++n;
        }
    }
    next_c = c;
    next_n = n;
    gb_fit_tables = (c >= BENCH_CODE_CHARS) || ((n + 1U) >= BENCH_FIT_CANDS);
    return gb_fit_tables;
}

/** @brief One template: candidate cand's bars at module m mm, blurred by a
 * Gaussian of sd sigma mm, as coverage 0..BENCH_FIT_ONE every 1/TPL_Q mm
 * from BENCH_FIT_TPL_PAD_MM before the first edge. Bars are filled, then
 * each edge's blur added over +-4 sd: for a bar lo..hi the exact coverage is
 * the fill plus g(u - lo) - g(u - hi), g(d) = Phi(d / sigma) - [d >= 0].
 */
static bool bench_fit_render (uint32_t cand, float m, float sigma)
{
    uint32_t points = (uint32_t)(((BENCH_FIT_MODULES * m) +
                                  (2.0f * (float)BENCH_FIT_TPL_PAD_MM)) *
                                 (float)BENCH_FIT_TPL_Q) + 1U;
    uint32_t b;
    int32_t  p;
    int32_t  reach = (int32_t)(4.0f * sigma * (float)BENCH_FIT_TPL_Q) + 1;
    int32_t  step_q = (int32_t)((((float)BENCH_FIT_PHI_N / 8.0f) /
                                 (sigma * (float)BENCH_FIT_TPL_Q)) * 65536.0f);
    int32_t  pe[BENCH_FIT_EDGES];

    if (points > BENCH_FIT_TPL)
    {
        return false;
    }
    for (b = 0U; b < BENCH_FIT_EDGES; ++b)
    {
        pe[b] = (int32_t)ceilf((((float)g_fit_edge[cand][b] * m) +
                                (float)BENCH_FIT_TPL_PAD_MM) *
                               (float)BENCH_FIT_TPL_Q);
    }
    (void)memset(g_fit_tpl, 0, (size_t)points * sizeof(g_fit_tpl[0]));
    for (b = 0U; b < BENCH_FIT_EDGES; b += 2U)
    {
        for (p = pe[b]; (p < pe[b + 1U]) && (p < (int32_t)points); ++p)
        {
            g_fit_tpl[p] = (int16_t)(g_fit_tpl[p] + BENCH_FIT_ONE);
        }
    }
    for (b = 0U; b < BENCH_FIT_EDGES; ++b)
    {
        float   edge = ((float)g_fit_edge[cand][b] * m) +
                       (float)BENCH_FIT_TPL_PAD_MM;
        int32_t sign = (0U == (b % 2U)) ? 1 : -1;
        int32_t p0 = pe[b] - reach;
        int32_t p1 = pe[b] + reach;
        /* table index at p: ((p / Q - edge) / sigma + 4) * N / 8, 16.16 */
        int32_t kq = (int32_t)((((((float)p0 / (float)BENCH_FIT_TPL_Q) - edge) /
                                  sigma) + 4.0f) *
                                ((float)BENCH_FIT_PHI_N / 8.0f) * 65536.0f);

        for (p = p0; p <= p1; ++p, kq += step_q)
        {
            int32_t k = (kq + 32768) / 65536;
            int32_t g;

            if ((p < 0) || (p >= (int32_t)points))
            {
                continue;
            }
            if (k < 0)
            {
                k = 0;
            }
            if (k > (int32_t)BENCH_FIT_PHI_N)
            {
                k = (int32_t)BENCH_FIT_PHI_N;
            }
            g = (int32_t)g_fit_phi[k] - ((p >= pe[b]) ? BENCH_FIT_ONE : 0);
            g_fit_tpl[p] = (int16_t)(g_fit_tpl[p] + (sign * g));
        }
    }
    for (p = 0; p < (int32_t)points; ++p)
    {
        if (g_fit_tpl[p] < 0)
        {
            g_fit_tpl[p] = 0;
        }
        if (g_fit_tpl[p] > BENCH_FIT_ONE)
        {
            g_fit_tpl[p] = BENCH_FIT_ONE;
        }
    }
    g_fit.tpl_points = points;
    g_fit.tpl_m = m;
    g_fit.tpl_sigma = sigma;
    return true;
}

/** @brief The rendered template placed with its first edge at x0 mm, each
 * dead zone in p_c0 tried: the least-squares misfit (white and black free)
 * of the binned cut; the best into *p_best when lower. The template is
 * looked up once per bin, keeping the bins it covers past the smallest dead
 * zone (the rest add nothing), then one tight pass per dead zone: on the
 * M0+ the sums stay in registers.
 */
static void bench_fit_eval (float x0, const float * p_c0, uint32_t c0s,
                            bench_fit_best_t * p_best)
{
    static int16_t cov[BENCH_FIT_BINS];   /* template at the kept bins */
    static int16_t val[BENCH_FIT_BINS];   /* their readings - white */
    uint32_t j;
    uint32_t i;
    uint32_t kept = 0U;
    uint32_t ss[BENCH_FIT_C0S];
    uint32_t sss[BENCH_FIT_C0S];
    int64_t  ssv[BENCH_FIT_C0S];
    int32_t  c0q[BENCH_FIT_C0S];
    int32_t  inv[BENCH_FIT_C0S];
    int32_t  c0_min = BENCH_FIT_ONE;
    /* template index of bin i's centre: idx0 + Q * i */
    int32_t  idx0 = (int32_t)floorf(((((g_fit.x_first + 0.5f) - x0) +
                                      (float)BENCH_FIT_TPL_PAD_MM) *
                                     (float)BENCH_FIT_TPL_Q) + 0.5f);
    double   n = (double)g_fit.valid;
    double   svv_c = g_fit.svv - ((g_fit.sv * g_fit.sv) / n);

    for (j = 0U; j < c0s; ++j)
    {
        ss[j] = 0U;
        sss[j] = 0U;
        ssv[j] = 0;
        c0q[j] = (int32_t)(p_c0[j] * (float)BENCH_FIT_ONE);
        inv[j] = ((int32_t)BENCH_FIT_S_ONE * 4096) / (BENCH_FIT_ONE - c0q[j]);
        c0_min = (c0q[j] < c0_min) ? c0q[j] : c0_min;
    }
    for (i = 0U; i < g_fit.valid; ++i)
    {
        int32_t idx = idx0 + ((int32_t)BENCH_FIT_TPL_Q *
                              (int32_t)g_fit.valid_bin[i]);

        if ((idx >= 0) && (idx < (int32_t)g_fit.tpl_points) &&
            ((int32_t)g_fit_tpl[idx] > c0_min))
        {
            cov[kept] = g_fit_tpl[idx];
            val[kept] = g_fit.valid_v[i];
            ++kept;
        }
    }
    for (j = 0U; j < c0s; ++j)
    {
        const int32_t c0 = c0q[j];
        const uint32_t k = (uint32_t)inv[j];
        uint32_t a = 0U;
        uint32_t b = 0U;
        int64_t  d = 0;
        uint32_t i0;

        /* s * v is at most 1024 * 4095: 256 of them fit an int32 */
        for (i0 = 0U; i0 < kept; i0 += 256U)
        {
            uint32_t i1 = ((i0 + 256U) < kept) ? (i0 + 256U) : kept;
            int32_t  d32 = 0;

            for (i = i0; i < i1; ++i)
            {
                int32_t c = (int32_t)cov[i];

                if (c > c0)
                {
                    int32_t s = (int32_t)(((uint32_t)(c - c0) * k) >> 12);

                    if (s > BENCH_FIT_S_ONE)
                    {
                        s = BENCH_FIT_S_ONE;
                    }
                    a += (uint32_t)s;
                    b += (uint32_t)(s * s);
                    d32 += s * (int32_t)val[i];
                }
            }
            d += (int64_t)d32;
        }
        ss[j] = a;
        sss[j] = b;
        ssv[j] = d;
    }
    g_fit.work += g_fit.valid + (kept * c0s);
    for (j = 0U; j < c0s; ++j)
    {
        double sss_c = (double)sss[j] - (((double)ss[j] * (double)ss[j]) / n);
        double ssv_c = (double)ssv[j] - (((double)ss[j] * g_fit.sv) / n);
        double sse = svv_c;

        if ((sss_c > 1.0) && (ssv_c > 0.0))
        {
            sse = svv_c - ((ssv_c * ssv_c) / sss_c);
        }
        if ((float)sse < p_best->sse)
        {
            p_best->sse = (float)sse;
            p_best->m = g_fit.tpl_m;
            p_best->sigma = g_fit.tpl_sigma;
            p_best->x0 = x0;
            p_best->c0 = p_c0[j];
        }
    }
}

/** @brief Stage 1's humps, on the bins (gaps filled): turning points with a
 * swing of BENCH_FIT_SDS white sds or BENCH_FIT_REL_PCT of the level above
 * white, whichever is more (dark readings are noisier, and a fixed swing
 * splits a wide bar's flat top); each hump's centre (above half its
 * height) and darkness area; the six largest areas' centres, by position.
 */
static void bench_fit_humps (void)
{
    const int16_t * p_v = g_fit.vf;
    uint32_t bins = g_fit.bins;
    /* static: the bench task's stack is 4 KB (APP_MAIN_STACK_BYTES) */
    static uint16_t ext[(BENCH_FIT_HUMPS * 2U) + 2U];
    static bool     b_peak[(BENCH_FIT_HUMPS * 2U) + 2U];
    static float    hc[BENCH_FIT_HUMPS];
    static int32_t  ha[BENCH_FIT_HUMPS];
    static bool     b_used[BENCH_FIT_HUMPS];
    uint32_t exts = 0U;
    uint32_t i;
    uint32_t hi = 0U;
    uint32_t lo = 0U;
    int32_t  mode = 0;
    uint32_t humps = 0U;
    int32_t  sw = BENCH_FIT_SDS * g_fit.sd;
    int32_t  white = g_fit.white;

#define BENCH_FIT_NEED(level) \
    ((((((int32_t)(level) - white) * BENCH_FIT_REL_PCT) / 100) > sw) ? \
     ((((int32_t)(level) - white) * BENCH_FIT_REL_PCT) / 100) : sw)
    for (i = 1U; (i < bins) && ((exts + 2U) < (BENCH_FIT_HUMPS * 2U)); ++i)
    {
        int32_t v = (int32_t)p_v[i];

        if (1 == mode)
        {
            if (v > (int32_t)p_v[hi])
            {
                hi = i;
            }
            else if (((int32_t)p_v[hi] - v) >= BENCH_FIT_NEED(p_v[hi]))
            {
                ext[exts] = (uint16_t)hi;
                b_peak[exts] = true;
                ++exts;
                mode = -1;
                lo = i;
            }
            else
            {
                /* Still at the top. */
            }
        }
        else
        {
            if (v < (int32_t)p_v[lo])
            {
                lo = i;
            }
            else if ((v - (int32_t)p_v[lo]) >= BENCH_FIT_NEED(v))
            {
                ext[exts] = (uint16_t)lo;
                b_peak[exts] = false;
                ++exts;
                mode = 1;
                hi = i;
            }
            else
            {
                /* Still at the bottom. */
            }
        }
    }
#undef BENCH_FIT_NEED
    if (1 == mode)
    {
        ext[exts] = (uint16_t)hi;
        b_peak[exts] = true;
        ++exts;
    }
    ext[exts] = (uint16_t)(bins - 1U);
    b_peak[exts] = false;
    ++exts;
    for (i = 0U; (i < exts) && (humps < BENCH_FIT_HUMPS); ++i)
    {
        uint32_t a;
        uint32_t b;
        uint32_t p;
        int32_t  base;
        int32_t  half;
        int32_t  w_sum = 0;
        int64_t  wx_sum = 0;      /* weights x (2 bin + 1) */
        int32_t  area = 0;

        if ((!b_peak[i]) || ((int32_t)p_v[ext[i]] < (white + sw)))
        {
            continue;
        }
        a = (i > 0U) ? ext[i - 1U] : 0U;
        b = ((i + 1U) < exts) ? ext[i + 1U] : (bins - 1U);
        base = ((int32_t)p_v[a] > (int32_t)p_v[b]) ? (int32_t)p_v[a] :
                                                      (int32_t)p_v[b];
        half = base + (((int32_t)p_v[ext[i]] - base) / 2);
        for (p = a; p <= b; ++p)
        {
            int32_t w = (int32_t)p_v[p] - half;

            if (w > 0)
            {
                w_sum += w;
                wx_sum += (int64_t)w * (int64_t)((2U * p) + 1U);
            }
            if ((int32_t)p_v[p] > white)
            {
                area += (int32_t)p_v[p] - white;
            }
        }
        if (w_sum <= 0)
        {
            continue;
        }
        hc[humps] = g_fit.x_first + (0.5f * (float)wx_sum / (float)w_sum);
        ha[humps] = area;
        ++humps;
    }
    g_fit.humps = humps;
    if (humps < 6U)
    {
        return;
    }
    {
        uint32_t t;
        uint32_t top = (humps < BENCH_FIT_TOP) ? humps : BENCH_FIT_TOP;

        (void)memset(b_used, 0, sizeof(b_used));
        for (t = 0U; t < top; ++t)
        {
            uint32_t best = BENCH_FIT_HUMPS;

            for (i = 0U; i < humps; ++i)
            {
                if ((!b_used[i]) &&
                    ((BENCH_FIT_HUMPS == best) || (ha[i] > ha[best])))
                {
                    best = i;
                }
            }
            b_used[best] = true;
            g_fit.top_pos[t] = hc[best];
        }
        for (t = 1U; t < top; ++t)
        {
            float    v = g_fit.top_pos[t];
            uint32_t u = t;

            while ((u > 0U) && (g_fit.top_pos[u - 1U] > v))
            {
                g_fit.top_pos[u] = g_fit.top_pos[u - 1U];
                --u;
            }
            g_fit.top_pos[u] = v;
        }
        g_fit.top_n = top;
    }
}

/** @brief Stage 1's next choice of six of the largest humps (a bit mask of
 * top_n bits, six set), their centres in pos[]. False when all are done.
 */
static bool bench_fit_subset (void)
{
    uint32_t t;
    uint32_t n = 0U;

    do
    {
        ++g_fit.mask;
        n = 0U;
        for (t = 0U; t < g_fit.top_n; ++t)
        {
            n += (g_fit.mask >> t) & 1U;
        }
    } while ((6U != n) && (g_fit.mask < (1UL << g_fit.top_n)));
    if (g_fit.mask >= (1UL << g_fit.top_n))
    {
        return false;
    }
    n = 0U;
    g_fit.pos_mean = 0.0f;
    for (t = 0U; t < g_fit.top_n; ++t)
    {
        if (0U != ((g_fit.mask >> t) & 1U))
        {
            g_fit.pos[n] = g_fit.top_pos[t];
            g_fit.pos_mean += g_fit.top_pos[t] / 6.0f;
            ++n;
        }
    }
    return true;
}

/** @brief Stage 1's fit of the six centres to candidate c's: module and
 * start by least squares; kept when its rms per module is the candidate's
 * lowest yet (choices of humps compared fairly whatever their module).
 */
static void bench_fit_lsq (uint32_t c)
{
    float    scp = 0.0f;
    float    r2 = 0.0f;
    float    m;
    float    rms;
    uint32_t i;

    for (i = 0U; i < BENCH_FIT_WIDE; ++i)
    {
        scp += g_fit_wide_dev[c][i] * (g_fit.pos[i] - g_fit.pos_mean);
    }
    m = scp * g_fit_wide_inv[c];
    if (m <= 0.0f)
    {
        return;
    }
    for (i = 0U; i < BENCH_FIT_WIDE; ++i)
    {
        float d = (g_fit.pos[i] - g_fit.pos_mean) - (m * g_fit_wide_dev[c][i]);

        r2 += d * d;
    }
    rms = sqrtf(r2 / 6.0f);
    if ((rms / m) < g_fit.rel[c])
    {
        g_fit.rel[c] = rms / m;
        g_fit.rms[c] = rms;
        g_fit.ms[c] = m;
        g_fit.x0s[c] = g_fit.pos_mean - (m * g_fit_wide_mean[c]);
    }
}

/** @brief Stage 1's short list: the BENCH_FIT_SHORT closest candidates
 * (stage 2 then fits the whole binned window: narrowing it to where they
 * can lie was tried offline and changed nothing).
 */
static void bench_fit_short (void)
{
    bool     b_used[BENCH_FIT_CANDS];
    uint32_t t;
    uint32_t c;

    (void)memset(b_used, 0, sizeof(b_used));
    g_fit.short_n = 0U;
    for (t = 0U; t < BENCH_FIT_SHORT; ++t)
    {
        uint32_t best = BENCH_FIT_CANDS;

        for (c = 0U; c < BENCH_FIT_CANDS; ++c)
        {
            if ((!b_used[c]) &&
                ((BENCH_FIT_CANDS == best) || (g_fit.rel[c] < g_fit.rel[best])))
            {
                best = c;
            }
        }
        b_used[best] = true;
        g_fit.short_cand[t] = (uint8_t)best;
        ++g_fit.short_n;
    }
    g_fit.s1_rel = g_fit.rel[g_fit.short_cand[0]];
    g_fit.s1_rms = g_fit.rms[g_fit.short_cand[0]];
}

/** @brief A Decoder 2 line for the cut, with its position and cost, queued. */
static void bench_fit_say (bench_fit_why_t why, float from, float to)
{
    bench_fit_line_t * p_line;

    if (g_fit_line_count >= BENCH_FIT_LINES)
    {
        ++g_fit_lines_lost;
        return;
    }
    p_line = &g_fit_lines[(g_fit_line_first + g_fit_line_count) %
                          BENCH_FIT_LINES];
    ++g_fit_line_count;
    p_line->why = why;
    p_line->b_read = g_fit.b_read;
    p_line->ch = g_fit.text_char;
    p_line->b_rev = g_fit.b_text_rev;
    p_line->other = g_fit.text_other;
    p_line->b_other_rev = g_fit.b_text_other_rev;
    p_line->margin = g_fit.text_margin;
    p_line->m = g_fit.text_m;
    p_line->sigma = g_fit.text_sigma;
    p_line->c0 = g_fit.text_c0;
    p_line->rms = g_fit.s1_rms;
    p_line->from = from;
    p_line->to = to;
    p_line->cpu_us = g_fit.cpu_us;
    p_line->steps = g_fit.steps;
    p_line->work = g_fit.work;
    p_line->joins = g_fit.joins;
    gp_fit_line_new = p_line;
}

/** @brief The samples of the cut just decoded (0..cut_count) after
 * binned_u, tape-dark lines left out, added to the binning slot.
 */
static void bench_fit_add (void)
{
    uint32_t index;
    uint32_t i;

    for (index = 0U; index < g_code.cut_count; ++index)
    {
        uint32_t unit = g_code.sample[index].unit;

        if ((unit < g_fit_bin.u0) ||
            ((0U != g_fit_bin.binned_u) && (unit <= g_fit_bin.binned_u)) ||
            bench_code_in_line(index))
        {
            continue;
        }
        i = ((unit - g_fit_bin.u0) * g_fit_bin.k16) >> 16;
        if (i < BENCH_FIT_BINS)
        {
            g_fit_sum[i] += (int32_t)g_code.sample[index].raw;
            ++g_fit_cnt[i];
        }
    }
    if (0U != g_code.cut_count)
    {
        uint32_t last = g_code.sample[g_code.cut_count - 1U].unit;

        if (last > g_fit_bin.binned_u)
        {
            g_fit_bin.binned_u = last;
        }
    }
}

/** @brief The binning slot moved to the fit: the window (the dark +-
 * BENCH_FIT_PAD_MM) as bin means, with their sums; the slot is free again.
 * False when there is nothing to fit.
 */
static bool bench_fit_prepare (void)
{
    float    unit_mm = (float)(g_code.unit_m * 1000.0);
    uint32_t pad_u = (uint32_t)bench_round((BENCH_FIT_PAD_MM / 1000.0) /
                                           g_code.unit_m);
    uint32_t i;
    int64_t  svv = 0;
    int32_t  sv = 0;

    g_fit_bin.b_used = false;
    g_fit.white = g_fit_bin.white;
    g_fit.sd = g_fit_bin.sd;
    g_fit.joins = g_fit_bin.joins;
    g_fit.cpu_us = 0U;
    g_fit.steps = 0U;
    g_fit.work = 0U;
    g_fit.s1_rms = 0.0f;
    g_fit.dark_from = (float)g_fit_bin.first_u * unit_mm;
    g_fit.dark_to = (float)g_fit_bin.last_u * unit_mm;
    if ((g_fit.dark_to - g_fit.dark_from) > (float)BENCH_FIT_LONGEST_MM)
    {
        ++g_fit.no_reads;
        bench_fit_say(BENCH_FIT_WHY_LONG, g_fit.dark_from, g_fit.dark_to);
        return false;
    }
    g_fit.x_first = (float)g_fit_bin.u0 * unit_mm;
    g_fit.bins = (((g_fit_bin.last_u + pad_u - g_fit_bin.u0) *
                   g_fit_bin.k16) >> 16) + 1U;
    if (g_fit.bins > BENCH_FIT_BINS)
    {
        g_fit.bins = BENCH_FIT_BINS;
    }
    g_fit.valid = 0U;
    for (i = 0U; i < g_fit.bins; ++i)
    {
        if (0U != g_fit_cnt[i])
        {
            int32_t v = (g_fit_sum[i] + ((int32_t)g_fit_cnt[i] / 2)) /
                        (int32_t)g_fit_cnt[i];
            int32_t d = v - g_fit.white;

            g_fit.vf[i] = (int16_t)v;
            g_fit.valid_bin[g_fit.valid] = (uint16_t)i;
            g_fit.valid_v[g_fit.valid] = (int16_t)d;
            sv += d;
            svv += (int64_t)d * (int64_t)d;
            ++g_fit.valid;
        }
        else
        {
            g_fit.vf[i] = (i > 0U) ? g_fit.vf[i - 1U] : (int16_t)g_fit.white;
        }
    }
    g_fit.sv = (double)sv;
    g_fit.svv = (double)svv;
    if (g_fit.valid < 50U)
    {
        return false;
    }
    g_fit.phase = BENCH_FIT_HUMPS_PHASE;
    return true;
}

/** @brief MISSION, a cut the darkness tries have just finished (samples
 * 0..cut_count, before bench_code_keep): its readings (tape-dark lines left
 * out) added to 1 mm bins, so the fit can run while the store goes on. A
 * cut whose dark starts within BENCH_FIT_JOIN_MM of the last one's joins
 * it; a binned card goes to the fit once that much white has followed
 * (bench_fit_run), or at once when the next card comes and the fit is
 * free. Integer work only; the fit itself is sliced.
 */
void bench_fit_begin (void)
{
    float    unit_mm = (float)(g_code.unit_m * 1000.0);
    uint32_t join_u = (uint32_t)bench_round((BENCH_FIT_JOIN_MM / 1000.0) /
                                            g_code.unit_m);
    uint32_t pad_u = (uint32_t)bench_round((BENCH_FIT_PAD_MM / 1000.0) /
                                           g_code.unit_m);
    uint32_t floor_adc;
    uint32_t first_u = 0U;
    uint32_t last_u = 0U;
    uint32_t index;
    bool     b_dark = false;
    int32_t  white = (int32_t)g_code.cut_white;
    int32_t  sd = (0U != g_code.cut_sd) ? (int32_t)g_code.cut_sd : 1;

    floor_adc = (uint32_t)(white + (BENCH_FIT_SDS * sd));
    for (index = 0U; index < g_code.cut_count; ++index)
    {
        if (((uint32_t)g_code.sample[index].raw < floor_adc) ||
            bench_code_in_line(index))
        {
            continue;
        }
        if (!b_dark)
        {
            first_u = g_code.sample[index].unit;
            b_dark = true;
        }
        last_u = g_code.sample[index].unit;
    }
    if (!b_dark)
    {
        return;
    }
    if (g_fit_bin.b_used && (first_u <= (g_fit_bin.last_u + join_u)))
    {
        /* The same card, cut at the stream's 30 mm of white: joined. */
        if (last_u > g_fit_bin.last_u)
        {
            g_fit_bin.last_u = last_u;
        }
        ++g_fit_bin.joins;
        bench_fit_add();
        return;
    }
    if (g_fit_bin.b_used && (BENCH_FIT_IDLE == g_fit.phase) && gb_fit_tables)
    {
        (void)bench_fit_prepare();   /* a new card: the binned one is whole */
    }
    if (g_fit_bin.b_used)
    {
        ++g_fit.skipped;
        bench_fit_say(BENCH_FIT_WHY_BUSY, (float)first_u * unit_mm,
                      (float)last_u * unit_mm);
        return;
    }
    g_fit_bin.b_used = true;
    g_fit_bin.white = white;
    g_fit_bin.sd = sd;
    g_fit_bin.first_u = first_u;
    g_fit_bin.last_u = last_u;
    g_fit_bin.binned_u = 0U;
    g_fit_bin.joins = 0U;
    g_fit_bin.u0 = (first_u > pad_u) ? (first_u - pad_u) : 0U;
    g_fit_bin.k16 = (uint32_t)bench_round((double)unit_mm * 65536.0);
    (void)memset(g_fit_sum, 0, sizeof(g_fit_sum));
    (void)memset(g_fit_cnt, 0, sizeof(g_fit_cnt));
    bench_fit_add();
}

/** @brief The result: best short-listed candidate, the best other character
 * and their misfit ratio, into the Decoder 2 line.
 */
static void bench_fit_finish (void)
{
    uint32_t k;
    uint32_t best = 0U;
    uint32_t other = BENCH_FIT_SHORT;

    for (k = 1U; k < g_fit.short_n; ++k)
    {
        if (g_fit.best[k].sse < g_fit.best[best].sse)
        {
            best = k;
        }
    }
    for (k = 0U; k < g_fit.short_n; ++k)
    {
        if ((g_fit_char[g_fit.short_cand[k]] !=
             g_fit_char[g_fit.short_cand[best]]) &&
            ((BENCH_FIT_SHORT == other) ||
             (g_fit.best[k].sse < g_fit.best[other].sse)))
        {
            other = k;
        }
    }
    g_fit.text_char = g_fit_char[g_fit.short_cand[best]];
    g_fit.b_text_rev = gb_fit_rev[g_fit.short_cand[best]];
    g_fit.text_m = g_fit.best[best].m;
    g_fit.text_sigma = g_fit.best[best].sigma;
    g_fit.text_c0 = g_fit.best[best].c0;
    if (BENCH_FIT_SHORT == other)
    {
        g_fit.text_other = '?';
        g_fit.b_text_other_rev = false;
        g_fit.text_margin = 99.0f;
    }
    else
    {
        g_fit.text_other = g_fit_char[g_fit.short_cand[other]];
        g_fit.b_text_other_rev = gb_fit_rev[g_fit.short_cand[other]];
        g_fit.text_margin = g_fit.best[other].sse /
                            fmaxf(g_fit.best[best].sse, 1.0f);
    }
    g_fit.b_read = (g_fit.text_margin >= BENCH_FIT_MARGIN);
    if (g_fit.b_read)
    {
        ++g_fit.reads;
    }
    else
    {
        ++g_fit.no_reads;
    }
    bench_fit_say(BENCH_FIT_WHY_FIT, g_fit.best[best].x0,
                  g_fit.best[best].x0 +
                  (BENCH_FIT_MODULES * g_fit.best[best].m));
    g_fit.phase = BENCH_FIT_IDLE;
}

/** @brief One slice of the fit: the humps, BENCH_FIT_LSQ_SLICE stage-1
 * candidates, or one template rendered, or one start position of it (all
 * its dead zones). Returns false once the fit is over.
 */
static bool bench_fit_slice (void)
{
    float    c0s[3] = { 0.0f, 0.0f, 0.0f };
    const float * p_c0 = c0s;
    uint32_t c0n;
    uint32_t cand;
    float    m;
    float    sigma;
    float    x0;
    uint32_t tpls;
    uint32_t x0n;
    uint32_t t;
    bench_fit_best_t * p_best;

    switch (g_fit.phase)
    {
        case BENCH_FIT_HUMPS_PHASE:
            bench_fit_humps();
            if (g_fit.humps < 6U)
            {
                /* Not a card at this blur: the GROUP lines show it. */
                g_fit.phase = BENCH_FIT_IDLE;
                return false;
            }
            for (t = 0U; t < BENCH_FIT_CANDS; ++t)
            {
                g_fit.rel[t] = 1.0e9f;
                g_fit.rms[t] = 1.0e9f;
                g_fit.ms[t] = 0.0f;
                g_fit.x0s[t] = 0.0f;
            }
            g_fit.mask = 0U;
            (void)bench_fit_subset();
            g_fit.c = 0U;
            g_fit.phase = BENCH_FIT_LSQ;
            return true;
        case BENCH_FIT_LSQ:
            /* Every choice of six of the largest humps, every candidate. */
            for (t = 0U; (t < BENCH_FIT_LSQ_SLICE) &&
                         (g_fit.c < BENCH_FIT_CANDS); ++t)
            {
                bench_fit_lsq(g_fit.c);
                ++g_fit.c;
            }
            if (g_fit.c < BENCH_FIT_CANDS)
            {
                return true;
            }
            g_fit.c = 0U;
            if (bench_fit_subset())
            {
                return true;
            }
            bench_fit_short();
            if (g_fit.s1_rel > BENCH_FIT_MISFIT)
            {
                ++g_fit.no_reads;
                bench_fit_say(BENCH_FIT_WHY_MISFIT, g_fit.dark_from,
                              g_fit.dark_to);
                g_fit.phase = BENCH_FIT_IDLE;
                return false;
            }
            g_fit.k = 0U;
            g_fit.tpl = 0U;
            g_fit.xi = 0U;
            g_fit.b_rendered = false;
            g_fit.shape.sse = 3.0e38f;
            for (t = 0U; t < BENCH_FIT_SHORT; ++t)
            {
                g_fit.best[t].sse = 3.0e38f;
            }
            /* The closest wide-bar class: candidates whose wide bars lie
             * where the closest one's do (the same fit to the humps).
             */
            g_fit.shape_n = 1U;
            while ((g_fit.shape_n < g_fit.short_n) &&
                   (g_fit.rel[g_fit.short_cand[g_fit.shape_n]] <=
                    (g_fit.s1_rel * 1.0001f)))
            {
                ++g_fit.shape_n;
            }
            g_fit.phase = BENCH_FIT_SHAPE;
            return true;
        case BENCH_FIT_SHAPE:
            /* One spot and dead zone for the pass (the sensor's, the same
             * whatever the card says), fitted over every candidate of the
             * closest class: the stage-1 order inside a class is arbitrary,
             * and a wrong character's best spot blurs away the bars it
             * lacks.
             */
            cand = g_fit.short_cand[g_fit.k];
            tpls = BENCH_FIT_SHAPE_TPLS;
            m = g_fit.ms[cand] * g_fit_shape_m[g_fit.tpl % 3U];
            sigma = g_fit_shape_sigma[g_fit.tpl / 3U];
            x0n = BENCH_FIT_SHAPE_X0S;
            x0 = g_fit.x0s[cand] - 1.5f + (0.5f * (float)g_fit.xi);
            p_c0 = g_fit_shape_c0;
            c0n = BENCH_FIT_C0S;
            p_best = &g_fit.shape;
            break;
        case BENCH_FIT_CANDS_PHASE:
            /* Every candidate near its stage-1 module and start, the
             * pass's spot and dead zone a little either way.
             */
            cand = g_fit.short_cand[g_fit.k];
            tpls = BENCH_FIT_CANDS_TPLS;
            m = g_fit.ms[cand] * (0.97f + (0.01f * (float)(g_fit.tpl % 7U)));
            sigma = g_fit.shape.sigma *
                    ((0U == (g_fit.tpl / 7U)) ? 0.85f :
                     ((1U == (g_fit.tpl / 7U)) ? 1.0f : 1.15f));
            x0n = BENCH_FIT_X0S;
            x0 = g_fit.x0s[cand] - 3.0f + (0.5f * (float)g_fit.xi);
            c0s[0] = fmaxf(0.0f, g_fit.shape.c0 - 0.07f);
            c0s[1] = g_fit.shape.c0;
            c0s[2] = fminf(0.8f, g_fit.shape.c0 + 0.07f);
            c0n = 3U;
            p_best = &g_fit.best[g_fit.k];
            break;
        case BENCH_FIT_FINE:
            /* Around the candidate's best so far, fixed when it starts. */
            if ((0U == g_fit.tpl) && (0U == g_fit.xi) && (!g_fit.b_rendered))
            {
                g_fit.centre = g_fit.best[g_fit.k];
            }
            cand = g_fit.short_cand[g_fit.k];
            tpls = BENCH_FIT_FINE_TPLS;
            m = g_fit.centre.m * (0.99f + (0.005f * (float)g_fit.tpl));
            sigma = g_fit.centre.sigma;
            x0n = BENCH_FIT_FINE_X0S;
            x0 = g_fit.centre.x0 - 0.75f + (0.25f * (float)g_fit.xi);
            c0s[0] = g_fit.centre.c0;
            c0n = 1U;
            p_best = &g_fit.best[g_fit.k];
            if (g_fit.centre.sse >= 3.0e38f)
            {
                g_fit.xi = x0n;  /* nothing fitted for it: skipped */
                g_fit.b_rendered = true;
            }
            break;
        default:
            g_fit.phase = BENCH_FIT_IDLE;
            return false;
    }
    if (!g_fit.b_rendered)
    {
        if (!bench_fit_render(cand, m, sigma))
        {
            g_fit.xi = x0n;      /* longer than the template store: skipped */
        }
        g_fit.b_rendered = true;
    }
    else if (g_fit.xi < x0n)
    {
        bench_fit_eval(x0, p_c0, c0n, p_best);
        ++g_fit.xi;
    }
    else
    {
        /* Skipped. */
    }
    if (g_fit.xi < x0n)
    {
        return true;
    }
    g_fit.xi = 0U;
    g_fit.b_rendered = false;
    ++g_fit.tpl;
    if (g_fit.tpl < tpls)
    {
        return true;
    }
    g_fit.tpl = 0U;
    ++g_fit.k;
    if (BENCH_FIT_SHAPE == g_fit.phase)
    {
        if (g_fit.k < g_fit.shape_n)
        {
            return true;
        }
        g_fit.k = 0U;
        if (g_fit.shape.sse >= 3.0e38f)
        {
            /* Every template longer than the store: no spot to go on. */
            ++g_fit.no_reads;
            bench_fit_say(BENCH_FIT_WHY_MISFIT, g_fit.dark_from, g_fit.dark_to);
            g_fit.phase = BENCH_FIT_IDLE;
            return false;
        }
        g_fit.phase = BENCH_FIT_CANDS_PHASE;
        return true;
    }
    if (g_fit.k < g_fit.short_n)
    {
        return true;
    }
    g_fit.k = 0U;
    if (BENCH_FIT_CANDS_PHASE == g_fit.phase)
    {
        g_fit.phase = BENCH_FIT_FINE;
        return true;
    }
    bench_fit_finish();
    return false;
}

/** @brief MISSION, every line step: the binned card to the fit once
 * BENCH_FIT_JOIN_MM of white has followed its last dark with no group open
 * and the fit is free; then slices of the fit until
 * BENCH_FIT_BUDGET_US has gone in this step (at least one slice), or
 * BENCH_FIT_BOOST_US while the car stands at a junction for this fit alone
 * (gb_fit_boost, bench_nav_wait).
 */
void bench_fit_run (void)
{
    uint32_t join_u = (uint32_t)bench_round((BENCH_FIT_JOIN_MM / 1000.0) /
                                            g_code.unit_m);
    uint32_t newest = (0U != g_code.count) ?
                      g_code.sample[g_code.count - 1U].unit : 0U;
    uint32_t budget = gb_fit_boost ? BENCH_FIT_BOOST_US : BENCH_FIT_BUDGET_US;
    uint64_t t0;
    uint32_t used = 0U;
    bool     b_more;
#if defined(MISSION_SIM)
    /* Sim only: its clock stands still within a step, so no budget runs out
     * and a fit ends in one step. SIM_FIT_SLICE_US n counts n us a slice,
     * so the fit spans steps as on the car (the junction wait's test).
     */
    const char * p_slice = getenv("SIM_FIT_SLICE_US");
    uint32_t sim_slice_us = (NULL != p_slice) ?
                            (uint32_t)strtoul(p_slice, NULL, 10) : 0U;
#endif

    if (!gb_fit_tables)
    {
        /* a share of the tables each step, from the first */
        (void)bench_fit_tables();
    }
    if ((!gb_fit_tables) && (BENCH_FIT_IDLE == g_fit.phase))
    {
        return;
    }
    if (g_fit_bin.b_used && (BENCH_FIT_IDLE == g_fit.phase) &&
        (gb_fit_force ||
         ((!g_code.b_open) && (newest >= (g_fit_bin.last_u + join_u)))))
    {
        (void)bench_fit_prepare();
    }
    if (BENCH_FIT_IDLE == g_fit.phase)
    {
        return;
    }
    ++g_fit.steps;
    gp_fit_line_new = NULL;
    do
    {
        t0 = board_time_us();
        b_more = bench_fit_slice();
        used += (uint32_t)(board_time_us() - t0);
#if defined(MISSION_SIM)
        used += sim_slice_us;
#endif
    } while (b_more && (used < budget));
    g_fit.cpu_us += used;
    if ((!b_more) && (NULL != gp_fit_line_new))
    {
        gp_fit_line_new->cpu_us = g_fit.cpu_us;   /* with the last slice */
    }
}

/** @brief MISSION, after the stop: the fit under way, else (b_bin) the
 * binned card, finished at once. True while there is something to say or
 * do: called until false, its Decoder 2 line sent each time. The watchdog is
 * fed
 * each table share and slice: no bench tick runs here, and a whole card's
 * fit takes longer than BENCH_WATCHDOG_MS (4 Nov: the car reset after the
 * stop and its Decoder 2 reads and totals were lost).
 */
bool bench_fit_complete (bool b_bin)
{
    while ((!gb_fit_tables) && (b_bin ? g_fit_bin.b_used : false))
    {
        (void)bench_fit_tables();   /* nothing else runs after the stop */
        mission_watchdog_feed();
    }
    if ((BENCH_FIT_IDLE == g_fit.phase) &&
        ((!b_bin) || (!g_fit_bin.b_used) || (!bench_fit_prepare())))
    {
        return (0U != g_fit_line_count);
    }
    while ((BENCH_FIT_IDLE != g_fit.phase) && bench_fit_slice())
    {
        mission_watchdog_feed();      /* nothing else runs after the stop */
    }
    mission_watchdog_feed();
    return true;
}

/** @brief One character, as text. */
static void bench_fit_char (char ch)
{
    char text[2];

    text[0] = ch;
    text[1] = '\0';
    bench_str(text, 1U);   /* frame: a 1-bit length (always 1), 6 bits */
}

/** @brief The Decoder 2 line, once per fitted cut (at most 150 characters:
 * BENCH_TEXT_BYTES). "rev": read end to start.
 */
bool bench_fit_text (void)
{
    const bench_fit_line_t * p_line;
    char                     code[4];

    if (0U == g_fit_line_count)
    {
        return false;
    }
    p_line = &g_fit_lines[g_fit_line_first];
    g_fit_line_first = (g_fit_line_first + 1U) % BENCH_FIT_LINES;
    --g_fit_line_count;
    code[0] = '*';
    code[1] = p_line->ch;
    code[2] = '*';
    code[3] = '\0';
    bench_vote_add(BENCH_VOTE_DECODER2, (int32_t)p_line->from,
                   (int32_t)p_line->to,
                   ((BENCH_FIT_WHY_FIT == p_line->why) && p_line->b_read) ?
                   code : NULL);
    /* Frame: why 2 bits (bench_fit_why_t); dark length 0-65535 mm; misfit
     * 0-102.3 mm; margin 0-99.00 (capped there); module 0-20.47 mm; spot
     * 0-25.5 mm; dead 0-1.27; path 0-2097151 mm (21 bits each); fit time
     * 0-65535 ms; steps 0-65535; tests 0-1048575 thousand.
     */
    bench_line(BK_FIT);
    bench_code((uint32_t)p_line->why, 2U);
    switch (p_line->why)
    {
        case BENCH_FIT_WHY_LONG:
            bench_text(" Decoder 2 no read: dark for ");
            bench_u((uint32_t)(p_line->to - p_line->from), 16U);
            bench_text(" mm, longer than a card");
            break;
        case BENCH_FIT_WHY_MISFIT:
            bench_text(" Decoder 2 no read: the 6 widest bars fit no"
                       " character (");
            bench_ux((uint32_t)(10.0f * p_line->rms), 1U, 10U);
            bench_text(" mm off)");
            break;
        case BENCH_FIT_WHY_BUSY:
            bench_text(" Decoder 2 skipped: still fitting the last card");
            break;
        default:
            bench_text(bench_flag(p_line->b_read) ?
                       " Decoder 2 *" : " Decoder 2 no read: best *");
            bench_fit_char(p_line->ch);
            bench_text(bench_flag(p_line->b_rev) ? "* rev" : "*");
            bench_text(", margin ");
            bench_ux((uint32_t)(100.0f * fminf(p_line->margin, 99.0f)), 2U,
                     14U);
            bench_text(" over *");
            bench_fit_char(p_line->other);
            bench_text(bench_flag(p_line->b_other_rev) ? "* rev" : "*");
            bench_text("; module ");
            bench_ux((uint32_t)(100.0f * p_line->m), 2U, 11U);
            bench_text(" mm, spot ");
            bench_ux((uint32_t)(10.0f * p_line->sigma), 1U, 8U);
            bench_text(" mm, dead ");
            bench_ux((uint32_t)(100.0f * p_line->c0), 2U, 7U);
            break;
    }
    bench_text("; at ");
    bench_u((uint32_t)p_line->from, 21U);
    bench_text("-");
    bench_u((uint32_t)p_line->to, 21U);
    bench_text(" mm");
    if (BENCH_FIT_WHY_FIT == p_line->why)
    {
        bench_text("; ");
        bench_u(p_line->cpu_us / 1000U, 16U);
        bench_text(" ms, ");
        bench_u(p_line->steps, 16U);
        bench_text(" steps, ");
        bench_u(p_line->work / 1000U, 20U);
        bench_text("k tests");
        if (bench_flag(0U != p_line->joins))
        {
            bench_text(", cuts joined");
        }
    }
    return true;
}

/** @brief MISSION, at the start: no fit, no counts (the tables are kept). */
void bench_fit_reset (void)
{
    (void)memset(&g_fit, 0, sizeof(g_fit));
    (void)memset(&g_fit_bin, 0, sizeof(g_fit_bin));
    g_fit_line_first = 0U;
    g_fit_line_count = 0U;
    g_fit_lines_lost = 0U;
    gp_fit_line_new = NULL;
}

/** @brief MISSION, after the stop: the Decoder 2 decoder's counts. */
void bench_fit_totals (void)
{
    /* Frame: counts 0-1023 a run; the margin setting 0-2.55. */
    bench_line(BK_FIT_TOTALS);
    bench_text(" Decoder 2: ");
    bench_u(g_fit.reads, 10U);
    bench_text(" read, ");
    bench_u(g_fit.no_reads, 10U);
    bench_text(" no read, ");
    bench_u(g_fit.skipped, 10U);
    bench_text(" skipped, ");
    bench_u(g_fit_lines_lost, 10U);
    bench_text(" lines lost (margin needed ");
    bench_ux((uint32_t)(100.0f * BENCH_FIT_MARGIN), 2U, 8U);
    bench_text(")");
    bench_flush();
}

/* ==== The car's barcode answer (4 Nov 2026): the three decoders' vote ====
 * Our rule, after 10 counted rounds on the tape track: any decoder
 * that reads gives the answer; if those that read disagree, the code read by
 * more than half of them wins, else no code (a wrong code is worse than
 * none). The voters: Decoder 1 (CODE lines), Decoder 2 (the TRIAL fit, a read
 * only
 * past BENCH_FIT_MARGIN) and D0. Each decoder's result joins the vote whose
 * path it overlaps when its own line goes out, final by then. Decoders
 * report in path order, so one that reports a mark beyond a vote's end will
 * not come back to it. A vote is decided when all three are in or have moved
 * past it, or at the stop: one CAR line each, then a CAR totals line. The
 * CODE, Decoder 2 and D0 lines are unchanged.
 */
#define BENCH_VOTE_CARDS   (8U)   /* votes open at once */
#define BENCH_VOTE_WHO     (3U)
#define BENCH_VOTE_ALL     (0x7U)

typedef struct
{
    int32_t  from_mm;   /* side sensor path, the union of the reports */
    int32_t  to_mm;
    uint32_t in;        /* a bit per decoder that reported */
    uint32_t done;      /* and per decoder final: reported, or moved past */
    char     code[BENCH_VOTE_WHO][BENCH_CODE_TEXT];  /* between the *s */
} bench_vote_t;

typedef struct
{
    bench_vote_t vote[BENCH_VOTE_CARDS];  /* the oldest first */
    uint32_t     count;
    bool         b_end;      /* the stop: every vote is decided */
    uint32_t     agreed;     /* codes every decoder that read agreed on */
    uint32_t     majority;   /* codes by majority */
    uint32_t     split;      /* no code: the readers disagreed */
    uint32_t     unread;     /* no code: no decoder read */
    uint32_t     lost;       /* results with no vote free */
} bench_votes_t;

static bench_votes_t g_vote;
static const char * const g_vote_names[BENCH_VOTE_WHO] =
{
    "Decoder 1", "Decoder 2", "D0"
};

/** @brief MISSION, at the start: no votes. */
void bench_vote_reset (void)
{
    (void)memset(&g_vote, 0, sizeof(g_vote));
}

/** @brief Decoder units as whole millimetres, as the lines print them. */
int32_t bench_vote_mm (uint32_t units)
{
    return bench_round((double)units * g_code.unit_m * 1000.0);
}

/** @brief A decoder's code between its *s (a partial frame has one only). */
static void bench_vote_code (char * p_dst, const char * p_src)
{
    size_t length;

    if ('*' == p_src[0])
    {
        ++p_src;
    }
    length = strlen(p_src);
    if ((length > 0U) && ('*' == p_src[length - 1U]))
    {
        --length;
    }
    if (length >= BENCH_CODE_TEXT)
    {
        length = BENCH_CODE_TEXT - 1U;
    }
    (void)memcpy(p_dst, p_src, length);
    p_dst[length] = '\0';
}

/** @brief One decoder's result for the path from_mm..to_mm: its code, or
 * NULL for no read. It joins the oldest vote it overlaps that lacks this
 * decoder (or has it with no read, when this one reads), else opens one.
 * Older votes it lies wholly past are final for this decoder.
 */
void bench_vote_add (uint32_t who, int32_t from_mm, int32_t to_mm,
                            const char * p_code)
{
    bench_vote_t * p_vote = NULL;
    bench_vote_t * p_each;
    uint32_t       bit = (uint32_t)1U << who;
    uint32_t       index;
    int32_t        swap;
    bool           b_free;

    if (to_mm < from_mm)
    {
        swap = from_mm;
        from_mm = to_mm;
        to_mm = swap;
    }
    for (index = 0U; index < g_vote.count; ++index)
    {
        p_each = &g_vote.vote[index];
        if (p_each->to_mm < from_mm)
        {
            p_each->done |= bit;
            continue;
        }
        b_free = (0U == (p_each->in & bit)) ||
                 (('\0' == p_each->code[who][0]) && (NULL != p_code));
        if ((NULL == p_vote) && (p_each->from_mm <= to_mm) && b_free)
        {
            p_vote = p_each;
        }
    }
    if (NULL == p_vote)
    {
        if (g_vote.count >= BENCH_VOTE_CARDS)
        {
            ++g_vote.lost;
            return;
        }
        p_vote = &g_vote.vote[g_vote.count];
        ++g_vote.count;
        (void)memset(p_vote, 0, sizeof(*p_vote));
        p_vote->from_mm = from_mm;
        p_vote->to_mm = to_mm;
    }
    if (from_mm < p_vote->from_mm)
    {
        p_vote->from_mm = from_mm;
    }
    if (to_mm > p_vote->to_mm)
    {
        p_vote->to_mm = to_mm;
    }
    p_vote->in |= bit;
    p_vote->done |= bit;
    if (NULL != p_code)
    {
        bench_vote_code(p_vote->code[who], p_code);
    }
    /* The junction needs the card's letter before the vote is final. */
    bench_nav_report(who, from_mm, to_mm, p_code);
}

/** @brief MISSION: the oldest vote, once decided, as one CAR line (not
 * flushed): the code, or no code and why, then each decoder's part (none:
 * it sent nothing for this path). True if there was one.
 */
bool bench_vote_text (void)
{
    const bench_vote_t * p_vote = &g_vote.vote[0];
    const char *         p_best = NULL;
    uint32_t             who;
    uint32_t             other;
    uint32_t             votes;
    uint32_t             best_votes = 0U;
    uint32_t             readers = 0U;
    uint32_t             index;

    if ((0U == g_vote.count) ||
        ((!g_vote.b_end) &&
         (BENCH_VOTE_ALL != (p_vote->done & BENCH_VOTE_ALL))))
    {
        return false;
    }
    for (who = 0U; who < BENCH_VOTE_WHO; ++who)
    {
        if ('\0' == p_vote->code[who][0])
        {
            continue;
        }
        ++readers;
        votes = 0U;
        for (other = 0U; other < BENCH_VOTE_WHO; ++other)
        {
            if (0 == strcmp(p_vote->code[who], p_vote->code[other]))
            {
                ++votes;
            }
        }
        if (votes > best_votes)
        {
            best_votes = votes;
            p_best = p_vote->code[who];
        }
    }
    /* Frame: the answer 2 bits (0 no read, 1 agreed, 2 majority, 3 split),
     * codes 15 characters at most (4-bit length), each decoder 2 bits (0
     * none, 1 no read, 2 a code), path 0-2097151 mm (21 bits).
     */
    bench_line(BK_CAR);
    bench_text(" CAR ");
    if ((0U == readers) || (NULL == p_best))
    {
        bench_code(0U, 2U);
        bench_text("no code");
        ++g_vote.unread;
    }
    else if (best_votes == readers)
    {
        bench_code(1U, 2U);
        bench_text("*");
        bench_str(p_best, 4U);
        bench_text("*");
        ++g_vote.agreed;
    }
    else if ((2U * best_votes) > readers)
    {
        bench_code(2U, 2U);
        bench_text("*");
        bench_str(p_best, 4U);
        bench_text("* by majority");
        ++g_vote.majority;
    }
    else
    {
        bench_code(3U, 2U);
        bench_text("no code, the decoders disagree");
        ++g_vote.split;
    }
    bench_text(" (");
    for (who = 0U; who < BENCH_VOTE_WHO; ++who)
    {
        bench_text((0U == who) ? "" : ", ");
        bench_text(g_vote_names[who]);
        if (0U == (p_vote->in & ((uint32_t)1U << who)))
        {
            bench_code(0U, 2U);
            bench_text(" none");
        }
        else if ('\0' == p_vote->code[who][0])
        {
            bench_code(1U, 2U);
            bench_text(" no read");
        }
        else
        {
            bench_code(2U, 2U);
            bench_text(" *");
            bench_str(p_vote->code[who], 4U);
            bench_text("*");
        }
    }
    bench_text(") at ");
    bench_u((uint32_t)p_vote->from_mm, 21U);
    bench_text("-");
    bench_u((uint32_t)p_vote->to_mm, 21U);
    bench_text(" mm");
    --g_vote.count;
    for (index = 0U; index < g_vote.count; ++index)
    {
        g_vote.vote[index] = g_vote.vote[index + 1U];
    }
    return true;
}

/** @brief MISSION, after the stop and every decoder's lines: all decided. */
void bench_vote_finish (void)
{
    g_vote.b_end = true;
}

/** @brief MISSION, after the stop: the car's answers counted. */
void bench_vote_totals (void)
{
    /* Frame: counts 0-1023 a run (the codes total 0-2047). */
    bench_line(BK_CAR_TOTALS);
    bench_text(" CAR: ");
    bench_u(g_vote.agreed + g_vote.majority, 11U);
    bench_text(" codes (");
    bench_u(g_vote.agreed, 10U);
    bench_text(" all readers agreed, ");
    bench_u(g_vote.majority, 10U);
    bench_text(" by majority), no code ");
    bench_u(g_vote.split, 10U);
    bench_text(" times as the decoders disagreed, ");
    bench_u(g_vote.unread, 10U);
    bench_text(" with no read");
    if (bench_flag(0U != g_vote.lost))
    {
        bench_text("; ");
        bench_u(g_vote.lost, 10U);
        bench_text(" results lost (no vote free)");
    }
    bench_flush();
}

/* End of file. */

