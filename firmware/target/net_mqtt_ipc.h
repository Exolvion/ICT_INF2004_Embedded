/**
 * @file net_mqtt_ipc.h
 * @brief Kernel/ISO-C boundary: fixed-width lengths avoid size_t conflicts.
 * Calls are nonblocking task-context APIs, never ISR APIs. Start once before
 * starting application or radio tasks. STOP is a separate priority channel.
 */
#ifndef NET_MQTT_IPC_H
#define NET_MQTT_IPC_H

#include <stdbool.h>
#include <stdint.h>

#define NET_MQTT_IPC_TX_MAX 256U
#define NET_MQTT_IPC_RX_MAX 32U

/** Create three bounded message buffers; clean up on partial failure. */
bool net_mqtt_ipc_start (void);

/** Queue 1..256 telemetry bytes. Return false on invalid input/full queue. */
bool net_mqtt_ipc_send (const char * p_text, uint32_t length);

/** Receive STOP first; capacity must be >=33 including trailing NUL. */
uint32_t net_mqtt_ipc_receive (char * p_text, uint32_t capacity);

/** Radio-owner only: dequeue telemetry into capacity >=257, add NUL. */
uint32_t net_mqtt_ipc_take_tx (char * p_text, uint32_t capacity);

/** Radio-owner only: enqueue a validated 1..32-byte command. */
bool net_mqtt_ipc_put_rx (const char * p_text, uint32_t length);

#endif /* NET_MQTT_IPC_H */

/* End of file. */

