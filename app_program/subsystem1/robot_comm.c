/** @file robot_comm.c
 * @brief The robot app: subsystem 1: MQTT commands (Comm) and the status,
 * perception and diagnostic frames (Telemetry). Split out of the one file by
 * tools/reorg/split_c.py (22 Nov 2026); what the parts share is in
 * robot_app_private.h.
 */
#include "robot_app_private.h"
#include "robot_comm.h"
#define ROBOT_APP_SPEED_BITS (10U)
#define ROBOT_APP_RATE_BITS (13U)
#define ROBOT_APP_AGE_BITS (10U)
#define ROBOT_APP_BYTE_BITS (8U)
static uint8_t g_telemetry_sequence;
static uint8_t g_rejected_seen;
static bool gb_wifi_loss;

/** @brief Receive at most one command byte: STOP, and with the mission also
 * START and RESET (README "Known deviations"). Other valid commands and
 * payloads the radio rejected as malformed are both counted as ignored. The
 * radio owner's session flag is mirrored into WIFI_LOSS through the locked
 * status helper, only when it changes.
 */
void robot_app_comm_step (uint32_t now_ms)
{
    uint8_t command = 0U;
    uint8_t rejected = net_mqtt_rejected();
    /* Modulo-256 difference of the radio's wrapping counter. */
    uint32_t ignored = (uint32_t)(uint8_t)(rejected - g_rejected_seen);
    uint32_t saved = 0U;
    bool b_wifi_loss = !net_mqtt_online();

    g_rejected_seen = rejected;
    if (net_mqtt_receive(&command))
    {
        if (MQTTCMD_STOP == mqttcmd_code(command))
        {
            robot_app_stop_set(ROBOT_APP_STOP_REMOTE);
        }
#if ROBOT_APP_MISSION
        /* Remote START and RESET (the team's deviation from FR10, which
         * keeps them local): the Vehicle task acts on them as on the button.
         */
        else if ((MQTTCMD_START == mqttcmd_code(command)) ||
                 (MQTTCMD_RESET == mqttcmd_code(command)))
        {
            saved = app_main_lock();
            if (MQTTCMD_START == mqttcmd_code(command))
            {
                g_shared.vehicle.b_start = true;
            }
            else
            {
                g_shared.vehicle.b_reset = true;
            }
            app_main_unlock(saved);
        }
#endif
        else
        {
            ++ignored;
        }
    }
    if (0U != ignored)
    {
        saved = app_main_lock();
        g_shared.ignored_commands = (uint8_t)bitpack_saturate(
            g_shared.ignored_commands + ignored, ROBOT_APP_BYTE_BITS);
        app_main_unlock(saved);
    }
    if (b_wifi_loss != gb_wifi_loss)
    {
        gb_wifi_loss = b_wifi_loss;
        robot_app_status_set(ROBOT_APP_BIT_WIFI_LOSS, b_wifi_loss);
    }
    robot_app_health(ROBOT_APP_HEALTH_COMM, now_ms);
}

/** @brief Binary frames in fixed minimum-width fields; no text formatting. */
void robot_app_telemetry_step (uint32_t now_ms)
{
    robot_app_shared_t copy = {0};
    telemetry_status_t status;
    uint8_t frame[TELEMETRY_FRAME_MAX] = {0};
    size_t length = 0U;

    robot_app_copy(&copy);
    /* Wrapping at 250 keeps "every fifth frame" true across the wrap. */
    g_telemetry_sequence = (uint8_t)((g_telemetry_sequence + 1U) %
                                     TELEMETRY_SEQUENCE_MODULO);
    memset(&status, 0, sizeof(status));
    status.seq = g_telemetry_sequence;
    status.now_ms = now_ms;
    status.dropped = copy.dropped_telemetry;
    status.ignored = copy.ignored_commands;
    /* Saturated to the 10-bit field, which fits uint16_t. */
    status.range_age_ms = (uint16_t)bitpack_saturate(
        robot_app_age(now_ms, copy.range_ms), ROBOT_APP_AGE_BITS);
    /* Enumerators 0..8 fit the 4-bit state field. */
    status.state = (uint8_t)copy.vehicle.state;
    status.vleft_mm_s = robot_app_wire_signed(copy.motion.left_mm_s,
                                              ROBOT_APP_SPEED_BITS);
    status.vright_mm_s = robot_app_wire_signed(copy.motion.right_mm_s,
                                               ROBOT_APP_SPEED_BITS);
    status.pitch_mrad = robot_app_wire_signed(copy.terrain.pitch_mrad,
                                              ROBOT_APP_ANGLE_BITS);
    /* Nine- and twelve-bit fields fit uint16_t exactly. */
    status.peak_mm = (uint16_t)copy.terrain.run_maximum_mm;
    status.range_mm = (uint16_t)copy.front.range.distance_mm;
    status.b_stop = copy.faults.b_inhibit;
    status.b_hardfault = copy.faults.b_hard;
    /* hv reports the last completed hump; pk > 0 is already a valid peak. */
    status.b_heightvalid = copy.terrain.b_last_hump_valid;
    status.b_rangevalid = CORE_RANGE_VALID == copy.front.range.state;
    length = telemetry_encode_status(&status, frame, sizeof(frame));
    if ((0U == length) || (!net_mqtt_send(frame, length)))
    {
        robot_app_count_drop();
    }
    if (0U == (g_telemetry_sequence % 5U))
    {
        robot_app_perception(&copy, now_ms);
        robot_app_diagnostic_frame(&copy, now_ms);
    }
}

/** @brief Saturating count of telemetry frames that were not queued. */
void robot_app_count_drop (void)
{
    uint32_t saved = app_main_lock();

    if (UINT8_MAX != g_shared.dropped_telemetry)
    {
        ++g_shared.dropped_telemetry;
    }
    app_main_unlock(saved);
}

/** @brief A signed shared field in wire form; the reserved code is invalid.
 */
int16_t robot_app_wire_signed (uint32_t field, uint32_t width)
{
    if (bitpack_invalid(width) == field)
    {
        return TELEMETRY_INVALID;
    }
    /* Shared signed fields are at most 13 bits, so the value fits int16_t. */
    return (int16_t)bitpack_extend(field, width);
}

/** @brief One-Hz perception summary complements five-Hz motion status. */
void robot_app_perception (const robot_app_shared_t * p_copy,
                                  uint32_t now_ms)
{
    telemetry_perception_t status;
    uint8_t frame[TELEMETRY_FRAME_MAX] = {0};
    size_t length = 0U;

    memset(&status, 0, sizeof(status));
    status.seq = g_telemetry_sequence;
    status.now_ms = now_ms;
    /* Eight-bit run counter; the calibration version saturates at 255. */
    status.run_id = (uint8_t)p_copy->vehicle.run_id;
    status.cal_version = (uint8_t)bitpack_saturate(ROBOT_CALIBRATION_VERSION,
                                                   ROBOT_APP_BYTE_BITS);
    /* Enumerators and 4-bit fields fit uint8_t exactly. */
    status.barcode_action = (uint8_t)p_copy->line.barcode_action;
    status.barcode_symbol = (uint8_t)p_copy->line.barcode_symbol;
    status.yaw_mrad_s = robot_app_wire_signed(p_copy->terrain.yaw_mrad_s,
                                              ROBOT_APP_RATE_BITS);
    status.motion_class = (uint8_t)p_copy->terrain.motion;
    /* Twelve-bit distances fit uint16_t; six-bit counts fit uint8_t. */
    status.scan_width_mm = (uint16_t)p_copy->scan.width_mm;
    status.scan_nearest_mm = (uint16_t)p_copy->scan.nearest_mm;
    status.scan_side = (uint8_t)p_copy->scan.preferred_side;
    status.scan_valid = (uint8_t)p_copy->scan.valid_samples;
    status.scan_unknown = (uint8_t)p_copy->scan.unknown_samples;
    status.b_tilt_valid = p_copy->terrain.b_tilt_valid;
    status.b_hump_active = p_copy->terrain.b_hump_active;
    status.b_scan_valid = p_copy->scan.b_profile_valid;
    status.b_obstacle_seen = p_copy->scan.b_obstacle_seen;
    length = telemetry_encode_perception(&status, frame, sizeof(frame));
    if ((0U == length) || (!net_mqtt_send(frame, length)))
    {
        robot_app_count_drop();
    }
}

/** @brief One-Hz first cause, active mask and stop sources. */
void robot_app_diagnostic_frame (const robot_app_shared_t * p_copy,
                                       uint32_t now_ms)
{
    telemetry_diagnostic_t status;
    uint8_t frame[TELEMETRY_FRAME_MAX] = {0};
    size_t length = 0U;

    memset(&status, 0, sizeof(status));
    status.seq = g_telemetry_sequence;
    status.now_ms = now_ms;
    /* Eight-, four- and two-bit fields fit their uint8_t/uint16_t wires. */
    status.run_id = (uint8_t)p_copy->vehicle.run_id;
    status.first_fault_ms = p_copy->first_fault_ms;
    status.barcode_unknown = (uint8_t)p_copy->line.barcode_unknown;
    status.barcode_rejected = (uint8_t)p_copy->line.barcode_rejected;
    status.mission_peak_mm = (uint16_t)p_copy->terrain.mission_maximum_mm;
    status.b_mission_valid = p_copy->terrain.b_mission_valid;
    status.fault_code = (uint8_t)p_copy->faults.first;
    status.fault_mask = (uint16_t)p_copy->faults.mask;
    status.stop_source = (uint8_t)p_copy->faults.stop_source;
    length = telemetry_encode_diagnostic(&status, frame, sizeof(frame));
    if ((0U == length) || (!net_mqtt_send(frame, length)))
    {
        robot_app_count_drop();
    }
}

/* End of file. */

