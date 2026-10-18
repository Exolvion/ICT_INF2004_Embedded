/**
 * @file telemetry.h
 * @brief Bounded integer-only JSON snapshots; no printf or allocation.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TELEMETRY_BUFFER_SIZE 256U

typedef struct
{
    uint32_t seq;
    uint32_t now_ms;
    uint32_t dropped;
    uint32_t ignored;
    uint32_t range_age_ms;
    int32_t  state;
    int32_t  vleft_mm_s;
    int32_t  vright_mm_s;
    int32_t  pitch_mrad;
    int32_t  peak_mm;
    int32_t  range_mm;
    bool     b_stop;
    bool     b_hardfault;
    bool     b_heightvalid;
    bool     b_rangevalid;
} telemetry_status_t;

/** @brief Perception snapshot; sign and units are explicit in field names. */
typedef struct
{
    uint32_t seq;
    uint32_t now_ms;
    uint32_t run_id;
    uint32_t cal_version;
    int32_t  barcode_action;
    int32_t  barcode_label;
    int32_t  yaw_mrad_s;
    int32_t  motion_class;
    int32_t  scan_width_mm;
    int32_t  scan_nearest_mm;
    int32_t  scan_side;
    uint32_t scan_valid;
    uint32_t scan_unknown;
    bool     b_tilt_valid;
    bool     b_hump_active;
    bool     b_scan_valid;
} telemetry_perception_t;

/** @brief First-fault reason and occurrence time, separate from live state. */
typedef struct
{
    uint32_t seq;
    uint32_t now_ms;
    uint32_t run_id;
    int32_t  fault_code;
    uint32_t first_fault_ms;
    uint32_t barcode_unknown;
    uint32_t barcode_rejected;
    int32_t  mission_peak_mm;
    bool     b_mission_valid;
} telemetry_diagnostic_t;

/**
 * Format one JSON object, returning its byte count excluding the final NUL.
 * All uint32_t/int32_t values, including INT32_MIN, are supported. Compact
 * keys: seq, ms, drop, ign, age, st, vl, vr, pt, pk, rg, stop, fault, hv, rv.
 * Units match the fields; age is milliseconds. Booleans are true/false.
 * Invalid sensors keep their numeric fields. The application encodes a
 * non-finite value (NaN pitch/speed/range) as 0, so read rg only with rv,
 * and pt only with the latest perception frame's tv; this frame has no tilt
 * flag. hv is the validity of the last completed hump, not of pk: pk is the
 * run's maximum valid hump, so pk > 0 is valid even while hv is false.
 * drop counts only application enqueue/format failures. Radio-side discards
 * (offline newest-only drain, publish errors) and rejected or overflowing
 * commands are not counted; detect lost status frames from seq gaps. A lost
 * perception or diagnostic frame leaves no seq gap (both reuse the status
 * seq): detect it as a status frame whose seq is a multiple of 5 without a
 * matching ba/fc frame with the same seq.
 * Return 0 for NULL arguments, zero capacity or insufficient capacity.
 * If p_text is non-NULL and capacity>0, failure writes p_text[0]='\0'.
 * A 256-byte buffer holds every representable snapshot and the final NUL.
 * Snapshot and destination must not overlap; caller supplies an owned copy.
 */
size_t telemetry_format (const telemetry_status_t * p_status,
                         char * p_text, size_t capacity);

/**
 * Format perception JSON with the same capacity/failure contract as above.
 * Keys: seq/ms/run/cal/ba/bl/yr/mc/sw/sn/ss/sv/su/tv/ha/sc, in field order.
 * bl is the barcode label's integer ASCII code; yr is estimated milliradians
 * per second. sw/sn are millimetres. sv/su count valid/unknown scan samples.
 * Enum values ba/mc/ss use their source modules' documented numeric values.
 * tv/ha/sc are JSON Booleans for tilt validity, active hump and valid scan.
 * A non-finite yaw rate is encoded as 0 and occurs only with mc 0
 * (TERRAIN_INVALID) in the same frame; read yr only when mc is nonzero.
 * The worst representation is 252 bytes plus NUL, within the 256-byte buffer.
 * Frames are distinguished by schema: ba occurs only in perception frames.
 */
size_t telemetry_format_perception (
    const telemetry_perception_t * p_status, char * p_text, size_t capacity);

/**
 * Format diagnostic JSON with the same capacity/failure contract as above.
 * Keys: seq/ms/run/fc/fm/bu/br/mp/mv. fc is the first-fault enum value;
 * zero means no latched fault. fm is the occurrence time in milliseconds,
 * not the current frame time. An approved reset clears the application
 * latch; it is refused while a hard fault is latched (reboot and review).
 * fc distinguishes this schema from status and perception frames.
 * bu/br are saturating unknown/rejected barcode counts since power-up; br
 * also counts a partly read frame discarded when FOLLOW ends mid-frame,
 * including one opened by a dark non-barcode feature, such as a junction
 * crossbar under the side sensor as NAVIGATE starts (rule pending, T-55).
 * mp/mv are the maximum completed valid hump in this power session and its
 * validity; local reset/start preserves them. Status pk remains run-local.
 * The worst representation is 144 bytes plus NUL, within the 256-byte buffer.
 */
size_t telemetry_format_diagnostic (
    const telemetry_diagnostic_t * p_status, char * p_text, size_t capacity);

#endif /* TELEMETRY_H */

/* End of file. */

