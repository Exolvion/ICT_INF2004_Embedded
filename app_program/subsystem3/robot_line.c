/** @file robot_line.c
 * @brief The robot app: subsystem 3: the Line task (IR samples, line
 * control). Split out of the one file by tools/reorg/split_c.py (22 Nov
 * 2026); what the parts share is in robot_app_private.h.
 */
#include "robot_app_private.h"
#include "robot_line.h"
#define ROBOT_APP_IR_RUN (5U)        /* Line, 2 ms: 10 ms of failed IR reads */
#if ROBOT_APP_MISSION
static uint8_t g_noise_ir;
#endif
#if !ROBOT_APP_MISSION
static uint32_t g_line_lock_ms;
static uint32_t g_optical_hc;
#endif
bool gb_optical_valid;
#if ROBOT_APP_MISSION
void robot_app_line_step (uint32_t now_ms)
{
    robot_app_shared_t copy = {0};
    line_ctrl_output_t output = {0};
    uint16_t raw[LINE_CTRL_CHANNELS] = {0};
    uint16_t left_total = 0U;
    uint16_t right_total = 0U;
    uint32_t saved = 0U;
    bool b_read = board_read_ir(raw);
    bool b_valid = false;

    board_encoder_totals(&left_total, &right_total);
    mission_ir_push(raw, b_read, left_total, right_total);
    robot_app_copy(&copy);
    b_valid = line_ctrl_step(&g_line_ctrl_runtime, &g_line_ctrl, raw, now_ms,
                             &output) && b_read;
    if (!b_valid)
    {
        output.status = LINE_CTRL_FAULT;
    }
    saved = app_main_lock();
    g_shared.line.status = output.status;
    g_shared.line.steering_q7 = bitpack_signed(output.steering_q7,
                                               ROBOT_APP_STEER_BITS) & 0xFFU;
    if (b_read)
    {
        g_shared.line_ms = now_ms;
    }
    g_shared.health[ROBOT_APP_HEALTH_LINE] = now_ms;
    app_main_unlock(saved);
    /* Noise control: five failed reads in a row (10 ms); the mission keeps
     * the last good sample through fewer (mission_ir_drain).
     */
    if (robot_app_noise(&g_noise_ir, !b_read,
                        ROBOT_APP_NOISE_LIMIT(ROBOT_APP_IR_RUN)) &&
        robot_app_active(copy.vehicle.state))
    {
        robot_app_fault_set(true, ROBOT_APP_BIT_OPTICAL_SAMPLE);
    }
}
#endif
#if !(ROBOT_APP_MISSION)
void robot_app_line_step (uint32_t now_ms)
{
    robot_app_shared_t copy = {0};
    line_ctrl_output_t output = {0};
    barcode_output_t code = {0};
    uint16_t raw[LINE_CTRL_CHANNELS] = {0};
    uint32_t saved = 0U;
    bool b_valid = board_read_ir(raw);
    bool b_frame_lost = false;
    bool b_overrun = false;

    robot_app_copy(&copy);
    b_valid = line_ctrl_step(&g_line_ctrl_runtime, &g_line_ctrl, raw, now_ms,
                             &output) && b_valid;
    b_valid = robot_app_optical_path() && b_valid;
    if (!b_valid)
    {
        output.status = LINE_CTRL_FAULT;
    }
    if (b_valid && (VEHICLE_FOLLOW == copy.vehicle.state))
    {
        /* At the 2^32 half-count wrap (173 km) the divided coordinate
         * steps back, which only drops an open frame like any reverse.
         */
        b_valid = barcode_step(&g_barcode_runtime, &g_barcode,
            barcode_classify(&g_barcode_runtime, raw[2]),
            g_optical_hc / ROBOT_APP_BARCODE_UNIT_HC, now_ms, &code);
    }
    else
    {
        b_frame_lost = g_barcode.b_frame;
        barcode_init(&g_barcode);
    }
    saved = app_main_lock();
    g_shared.line.status = output.status;
    g_shared.line.steering_q7 = bitpack_signed(output.steering_q7,
                                               ROBOT_APP_STEER_BITS) & 0xFFU;
    if (((VEHICLE_FOLLOW == copy.vehicle.state) ||
         (VEHICLE_RANGE_WAIT == copy.vehicle.state)) && output.b_junction_edge)
    {
        /* The 2 ms producer must retain an edge for the 10 ms consumer. */
        g_shared.line.b_junction_edge = true;
    }
    if ((BARCODE_EVENT_UNKNOWN == code.event) &&
        (UINT8_MAX != g_shared.line.barcode_unknown))
    {
        g_shared.line.barcode_unknown =
            (g_shared.line.barcode_unknown + 1U) & UINT8_MAX;
    }
    if (((BARCODE_EVENT_REJECTED == code.event) || b_frame_lost) &&
        (UINT8_MAX != g_shared.line.barcode_rejected))
    {
        g_shared.line.barcode_rejected =
            (g_shared.line.barcode_rejected + 1U) & UINT8_MAX;
    }
    if (b_valid)
    {
        g_shared.line_ms = now_ms;
    }
    if (BARCODE_EVENT_DECODED == code.event)
    {
        b_overrun = VEHICLE_NAV_NONE != g_shared.line.navigation;
        if (!b_overrun)
        {
            g_shared.line.navigation = robot_app_nav(code.action);
            g_shared.line.barcode_action = code.action;
            g_shared.line.barcode_symbol = code.symbol & 0xFU;
        }
    }
    g_shared.health[ROBOT_APP_HEALTH_LINE] = now_ms;
    app_main_unlock(saved);
    if (b_overrun)
    {
        /* A second decode before Vehicle took the first loses route data. */
        robot_app_fault_set(true, ROBOT_APP_BIT_BARCODE_OVERRUN);
    }
    /* A valid optical sample with an unmatched frame is not sensor failure. */
    if ((!b_valid) && (robot_app_active(copy.vehicle.state)))
    {
        robot_app_fault_set(true, ROBOT_APP_BIT_OPTICAL_SAMPLE);
    }
}
#endif
#if !ROBOT_APP_MISSION

/** @brief Mapping is from decoded action, never from illustrative A/B/C/D. */
vehicle_nav_t robot_app_nav (barcode_action_t action)
{
    vehicle_nav_t navigation = VEHICLE_NAV_NONE;

    switch (action)
    {
        case BARCODE_ACTION_LEFT:
            navigation = VEHICLE_NAV_LEFT;
        break;
        case BARCODE_ACTION_RIGHT:
            navigation = VEHICLE_NAV_RIGHT;
        break;
        case BARCODE_ACTION_STRAIGHT:
            navigation = VEHICLE_NAV_STRAIGHT;
        break;
        case BARCODE_ACTION_UTURN:
            navigation = VEHICLE_NAV_UTURN;
        break;
        case BARCODE_ACTION_UNKNOWN:
        default:
            navigation = VEHICLE_NAV_NONE;
        break;
    }
    return navigation;
}

/** @brief Require known course corridor, heading and sustained optical lock. */
bool robot_app_original_line (const robot_app_shared_t * p_copy,
                                     uint32_t now_ms)
{
    core_pose_t anchor;
    core_real_t delta_x = 0.0;
    core_real_t delta_y = 0.0;
    core_real_t yaw_error = 0.0;
    double lateral_m = 0.0;
    bool b_matches = false;

    core_pose_unpack(&g_route_anchor, &anchor);
    core_pose_delta(&p_copy->motion.pose, &g_route_anchor, &delta_x,
                    &delta_y, &yaw_error);
    lateral_m = (-sin(anchor.yaw_rad) * delta_x) +
                (cos(anchor.yaw_rad) * delta_y);
    b_matches = (0 != ROBOT_COURSE_CONFIRMED) &&
        (LINE_CTRL_TRACKING == p_copy->line.status) &&
        (fabs(lateral_m) <= ROBOT_LINE_UNIQUE_CORRIDOR_M) &&
        (fabs(yaw_error) <= ROBOT_LINE_HEADING_TOL_RAD);
    if ((!b_matches) || (VEHICLE_REACQUIRE != p_copy->vehicle.state))
    {
        g_vehicle_flags.b_line_lock = false;
        return false;
    }
    if (!g_vehicle_flags.b_line_lock)
    {
        g_vehicle_flags.b_line_lock = true;
        g_line_lock_ms = now_ms;
    }
    return (now_ms - g_line_lock_ms) >= ROBOT_LINE_LOCK_MS;
}

/** @brief Two-millisecond optical path from independent encoder totals:
 * the signed centre path in half-counts (left + right counts), wrapping
 * modulo 2^32, so barcode widths are whole decoder units of
 * ROBOT_APP_BARCODE_UNIT_HC half-counts.
 */
bool robot_app_optical_path (void)
{
    uint16_t left_total = 0U;
    uint16_t right_total = 0U;
    int32_t left_delta = 0;
    int32_t right_delta = 0;

    board_encoder_totals(&left_total, &right_total);
    left_delta = robot_app_signed_count(
        robot_app_count_delta(left_total, g_line_left_total),
        ROBOT_LEFT_ENCODER_SIGN);
    right_delta = robot_app_signed_count(
        robot_app_count_delta(right_total, g_line_right_total),
        ROBOT_RIGHT_ENCODER_SIGN);
    g_line_left_total = left_total;
    g_line_right_total = right_total;
    /* Conversion to uint32_t is modulo 2^32: a wrapping coordinate. */
    g_optical_hc += (uint32_t)(left_delta + right_delta);
    return gb_optical_valid;
}
#endif

/** @brief Reject a codebook whose narrowest run cannot be sampled spatially.
 * The gate runs on the authored metre values, as before.
 */
bool robot_app_codebook_ready (void)
{
    const barcode_config_t * p_codebook = course_cfg_codebook();
    double quantum_m = 0.0;
    double worst_step_m = 0.0;
    double counts_per_revolution = ROBOT_ENCODER_COUNTS_PER_REV;

    if ((NULL == p_codebook) || (counts_per_revolution <= 0.0))
    {
        return false;
    }
    /* One decoder unit of centre path, the step barcode_step() sees. */
    quantum_m = ((double)ROBOT_APP_BARCODE_UNIT_HC * CORE_PI *
                 ROBOT_WHEEL_RADIUS_M) / counts_per_revolution;
    /* Qualify the largest accepted gap, not the nominal 2 ms task period.
     * The line-speed bound and encoder calibration still need bench proof.
     */
    worst_step_m = quantum_m + (ROBOT_LINE_SPEED_MPS *
        ((double)p_codebook->maximum_gap_ms / 1000.0));

    return (p_codebook->b_codebook_confirmed) &&
        (0U != p_codebook->symbol_count) &&
        (barcode_config_valid(p_codebook)) &&
        (p_codebook->maximum_step_m >= worst_step_m) &&
        (p_codebook->minimum_run_m >=
         (5.0 * p_codebook->maximum_step_m));
}

/* End of file. */

