/** @file mission_frames.h
 * @brief The mission: subsystem 1: the mission's minimum-bit frames, text,
 * the line recorder and live lines; the calls the other parts make into
 * mission_frames.c. Include mission_private.h first.
 */
#ifndef MISSION_FRAMES_H
#define MISSION_FRAMES_H
void bench_tasks_text (void);
void bench_live_step (uint64_t now_us, uint32_t status,
                             int32_t centre_mm, uint16_t side);
void bench_live_take (void);
void bench_live_push (void);
void bench_live_finish (void);
int32_t bench_signed_delta (uint16_t newer, uint16_t older,
                                   uint32_t wheel);
void bench_text (const char * p_text);
void bench_dec (int32_t value);
void bench_signed (int32_t value);
void bench_fixed (int32_t value, uint32_t places);
void bench_flush (void);

/* Minimum-bit frames (BENCH_FRAMES): the line's kind, then each field in
 * the bits its range needs. Each prints as the call it replaces would.
 */
void bench_line (bench_kind_t kind);
void bench_u (uint32_t value, uint32_t bits);
   /* as bench_dec */
void bench_i (int32_t value, uint32_t bits);
    /* as bench_dec */
void bench_s (int32_t value, uint32_t bits);
    /* as bench_signed */
void bench_x (int32_t value, uint32_t places, uint32_t bits);
void bench_ux (uint32_t value, uint32_t places, uint32_t bits);

                                                       /* as bench_fixed */
bool bench_flag (bool b_value);
 /* no text */
void bench_name (const char * const * p_names, uint32_t index,
                        uint32_t bits);
                /* p_names[index] */
void bench_str (const char * p_text, uint32_t length_bits);

                                         /* Code 39 text, 6 bits a char */
void bench_name_of (const char * const * p_names, uint32_t count,
                           const char * p_text, uint32_t bits);
void bench_char (char ch, char base, uint32_t bits);

/* MISSION line recorder, at the end of the file. */
void bench_rec_reset (void);
void bench_rec_sample (uint64_t now_us, double raw_left,
                              double raw_right, double left, double right,
                              bool b_straight);
void bench_rec_text (void);

void bench_frame_put (uint32_t field, uint32_t bits);

uint32_t bench_frame_unsigned (uint32_t value, uint32_t bits);

#endif /* MISSION_FRAMES_H */

/* End of file. */

