/**
 * @file net_mqtt.h
 * @brief ISO-C task API and radio-owner MQTT service.
 * Never include with tk/tkernel.h (the upstream size_t differs). Kernel-only
 * translation units use the equivalent net_mqtt_ipc.h fixed-width API.
 */
#ifndef NET_MQTT_H
#define NET_MQTT_H

#include <stdbool.h>
#include <stddef.h>

struct netif;

/** Create bounded queues once, before creating radio/application tasks. */
bool net_mqtt_start (void);

/** Enqueue 1..256 bytes, nonblocking; false means invalid input or full. */
bool net_mqtt_send (const char * p_text, size_t length);

/** Dequeue one exact command, STOP first. Capacity must be >=33. */
size_t net_mqtt_receive (char * p_text, size_t capacity);

/**
 * Radio-owner only, after lwip_init and cyw43 initialization. Call every
 * 2..5 ms in the same context as sys_check_timeouts, after it returns: never
 * from an lwIP callback or sys_timeout handler, because connection closes
 * deferred from lwIP callbacks run here. Never from an ISR.
 * Handles Wi-Fi association, MQTT connection, subscription and QoS0 publish.
 */
void net_mqtt_poll (struct netif * p_interface);

#endif /* NET_MQTT_H */

/* End of file. */

