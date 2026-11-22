/**
 * @file net_mqtt_ipc.h
 * @brief Kernel/ISO-C boundary: fixed-width lengths avoid size_t conflicts.
 * Calls are nonblocking task-context APIs, never ISR APIs. Start once before
 * starting application or radio tasks. STOP is a separate priority channel.
 * Payloads are binary: telemetry frames of at most NET_MQTT_IPC_TX_MAX bytes
 * (telemetry.h) and one-byte commands (mqttcmd.h). This header cannot
 * include either (they use size_t), so net_mqtt.c checks the sizes and the
 * STOP byte against them at compile time.
 */
#ifndef NET_MQTT_IPC_H
#define NET_MQTT_IPC_H

#include <stdbool.h>
#include <stdint.h>

/** The robot's largest frame is 17 bytes; the bench image sends text lines
 * through the same queue, so 64 (a 4-byte multiple): a 120-character line
 * is 2 frames, not 6 (19 Nov: the text link ran at ~150 characters/s).
 */
#define NET_MQTT_IPC_TX_MAX 64U
#define NET_MQTT_IPC_RX_MAX 1U
#define NET_MQTT_IPC_STOP_BYTE 0xA9U
/** A junction map (navmap.h) is 3 bytes; net_mqtt.c checks it. */
#define NET_MQTT_IPC_MAP_MAX 3U

/** Create three bounded message buffers; clean up on partial failure. */
bool net_mqtt_ipc_start (void);

/** Queue 1..NET_MQTT_IPC_TX_MAX telemetry bytes. Return false on invalid
 * input/full queue.
 */
bool net_mqtt_ipc_send (const uint8_t * p_frame, uint32_t length);

/** Receive one command byte, STOP first. True when a byte was written. */
bool net_mqtt_ipc_receive (uint8_t * p_command);

/** Radio-owner only: dequeue one frame; capacity must be >=
 * NET_MQTT_IPC_TX_MAX bytes.
 */
uint32_t net_mqtt_ipc_take_tx (uint8_t * p_frame, uint32_t capacity);

/** Radio-owner only: enqueue one validated command byte. */
bool net_mqtt_ipc_put_rx (uint8_t command);

/** Radio-owner only: enqueue one validated 3-byte junction map. */
bool net_mqtt_ipc_put_map (const uint8_t * p_map);

/** Receive one 3-byte junction map into p_map (room for 3 bytes). True when
 * a map was written.
 */
bool net_mqtt_ipc_receive_map (uint8_t * p_map);

#endif /* NET_MQTT_IPC_H */

/* End of file. */

