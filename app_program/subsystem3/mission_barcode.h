/** @file mission_barcode.h
 * @brief The mission: subsystem 3: the barcode store and decoders (Decoder
 * 1, Decoder 2, D0) and their vote; the calls the other parts make into
 * mission_barcode.c. Include mission_private.h first.
 */
#ifndef MISSION_BARCODE_H
#define MISSION_BARCODE_H
bool bench_code_start (void);
void bench_code_cyc (void * p_exinf);
bool bench_code_polling (void);
void bench_code_poll_at (const uint16_t raw[LINE_CTRL_CHANNELS],
                                const uint16_t total[BENCH_WHEELS]);
void bench_code_sample (int32_t left, int32_t right, uint16_t raw);
void bench_code_store (void);
void bench_code_stream (void);
void bench_code_keep (uint32_t gap);
void bench_code_found_add (uint32_t runs);
void bench_code_found_window (bench_code_found_t * p_found,
                                     uint32_t last);
void bench_code_found_rest (bench_code_found_t * p_found,
                                   uint32_t last);
void bench_code_found_chars (bench_code_found_t * p_found,
                                    uint32_t last);
bool bench_code_found_text (void);
bool bench_code_hold (void);
void bench_d0_sample (int32_t unit, bool b_level);
void bench_d0_close (void);
bool bench_d0_decode (const bench_d0_group_t * p_group, bool b_reverse,
                             char * p_text);
bool bench_d0_text (void);
void bench_code_mission_results (void);
void bench_code_results (void);
void bench_code_card (void);
void bench_code_card_group (uint32_t group, uint32_t runs,
                                   bool * p_decoded);
void bench_code_card_lines (void);
bool bench_code_in_line (uint32_t index);
void bench_code_card_extrema (uint16_t swing);
double bench_code_edge (uint32_t peak, uint32_t valley, double level,
                               uint32_t * p_index);
uint32_t bench_code_pass (const line_ctrl_hysteresis_t * p_hysteresis,
                                 bool * p_decoded);
void bench_code_runs (const line_ctrl_hysteresis_t * p_hysteresis);
bool bench_code_group (uint32_t first, uint32_t last);
void bench_code_widths (uint32_t first, uint32_t last);
void bench_code_dark (bool b_quiet, uint32_t half_sds, double gap_mm);
void bench_code_dark_tries (void);
void bench_code_tries_begin (uint32_t gap_units);
void bench_code_try (uint32_t index);
bool bench_code_try_agree (uint32_t index);
void bench_code_tries_step (void);
void bench_code_dark_end (uint32_t group, uint32_t runs,
                                 bool * p_decoded, bool b_quiet);
uint32_t bench_code_white (uint32_t * p_sd);
uint32_t bench_code_darkness (uint32_t before, uint32_t after);
uint32_t bench_code_whiteness (uint32_t from, uint32_t to,
                                      uint32_t floor_adc);
void bench_code_dark_row (uint32_t first, uint32_t last);
uint32_t bench_code39_dark (uint32_t first, uint32_t last,
                                   uint32_t base, bool b_reverse);
uint32_t bench_code39_spec (uint32_t first, uint32_t length,
                                   uint32_t last);
bool bench_code39 (uint32_t first, uint32_t last);
bool bench_code39_text (uint32_t first, uint32_t last, bool b_reverse,
                               char * p_text);
uint32_t bench_code_units (uint32_t first, uint32_t last,
                                  uint32_t index, bool b_reverse);
void bench_code_trace (void);
void bench_code_unit_mm (uint32_t units);
                 /* 1 bit, no text */
void bench_code (uint32_t value, uint32_t bits);

   /* fixed-length text, an alphabet */
                                         /* one character, ch - base */
/* TRIAL (barcode_lab): the model-fit decoder, at the end of the file. */
void bench_fit_reset (void);
void bench_fit_begin (void);
void bench_fit_run (void);
bool bench_fit_complete (bool b_bin);
bool bench_fit_text (void);
void    bench_vote_reset (void);
int32_t bench_vote_mm (uint32_t units);
void    bench_vote_add (uint32_t who, int32_t from_mm, int32_t to_mm,
                               const char * p_code);
bool    bench_vote_text (void);
void    bench_vote_finish (void);
void    bench_vote_totals (void);
void bench_fit_totals (void);

#endif /* MISSION_BARCODE_H */

/* End of file. */

