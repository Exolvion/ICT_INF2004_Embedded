/**
 * @file mqttnet_test_main.c
 * @brief Real SDK types with mocked radio, MQTT transport, IPC and clock.
 * This tests adapter state handling, not the lwIP stack or physical radio.
 * NOTE: Mock symbol names must match external APIs (BARR-C 6.1.i deviation).
 */
#include "mqttnet_test_main.h"
#include "net_mqtt.h"
#include "net_mqtt_ipc.h"
#include "net_mqtt_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cyw43.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"
#include "lwip/netif.h"
#include "lwip/sys.h"

cyw43_t cyw43_state;
static mqtt_client_t mqtt_client;
static uint32_t clock_ms;
static uint32_t joins;
static uint32_t leaves;
static uint32_t connects;
static uint32_t disconnects;
static uint32_t subscribes;
static uint32_t publications;
static uint32_t commands;
static err_t publish_result = ERR_OK;
static char last_publication[NET_MQTT_IPC_TX_MAX + 1U];
static uint32_t checks;
static uint32_t failures;
#define MQTTNET_TEST_TX_DEPTH 4U
static char tx_messages[MQTTNET_TEST_TX_DEPTH][NET_MQTT_IPC_TX_MAX + 1U];
static uint32_t tx_lengths[MQTTNET_TEST_TX_DEPTH];
static uint32_t tx_head;
static uint32_t tx_count;
static char last_command[NET_MQTT_IPC_RX_MAX + 1U];
static mqtt_connection_cb_t connection_callback;
static mqtt_request_cb_t subscription_callback;
static mqtt_incoming_publish_cb_t publication_callback;
static mqtt_incoming_data_cb_t fragment_callback;

static void mqttnet_test_check (bool b_result, const char * p_label);
static void mqttnet_test_join (struct netif * p_interface);
static void mqttnet_test_commands (void);
static void mqttnet_test_frame_triple (struct netif * p_interface);
static void mqttnet_test_reconnect (struct netif * p_interface);
static void mqttnet_test_backpressure (struct netif * p_interface);
static void mqttnet_test_rollover (struct netif * p_interface);
static void mqttnet_test_suback_timeout (struct netif * p_interface);

/** @brief Exercise one complete deterministic simulated client lifecycle. */
int main (void)
{
    struct netif interface = {0};
    int result;

    mqttnet_test_check(net_mqtt_start(), "queues initialized");
    mqttnet_test_check(!net_mqtt_send(NULL, 1U), "NULL send rejected");
    mqttnet_test_check(!net_mqtt_send("x", SIZE_MAX), "huge send rejected");
    mqttnet_test_check(net_mqtt_send("{\"seq\":1}", 9U), "telemetry queued");
    mqttnet_test_join(&interface);
    mqttnet_test_commands();
    mqttnet_test_frame_triple(&interface);
    mqttnet_test_backpressure(&interface);
    mqttnet_test_reconnect(&interface);
    mqttnet_test_rollover(&interface);
    mqttnet_test_suback_timeout(&interface);
    result = printf("mqttnet mocked adapter: %u checks, %u failures\n",
                      checks, failures);
    if ((result < 0) || (failures > 0U))
    {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

/** @brief Mock monotonic milliseconds, changed only by the test thread. */
u32_t sys_now (void)
{
    return clock_ms;
}

/** @brief Mock join acceptance; link establishment is a separate event. */
int cyw43_wifi_join (cyw43_t * p_state, size_t ssid_length,
                     const uint8_t * p_ssid, size_t key_length,
                     const uint8_t * p_key, uint32_t auth,
                     const uint8_t * p_bssid, uint32_t channel)
{
    /* Fixed callback ABI has parameters irrelevant to this state test. */
    (void)p_state;
    (void)ssid_length;
    (void)p_ssid;
    (void)key_length;
    (void)p_key;
    (void)auth;
    (void)p_bssid;
    (void)channel;
    joins++;
    return 0;
}

/** @brief Count disassociation attempts without accessing hardware. */
int cyw43_wifi_leave (cyw43_t * p_state, int interface)
{
    /* Mock does not implement hardware interfaces. */
    (void)p_state;
    (void)interface;
    leaves++;
    return 0;
}

/** @brief Supply a reusable SDK-typed client; real lwIP allocates this. */
mqtt_client_t * mqtt_client_new (void)
{
    return &mqtt_client;
}

/** @brief Keep callbacks for event injection; do not open a TCP socket. */
err_t mqtt_client_connect (mqtt_client_t * p_client,
                           const ip_addr_t * p_address, u16_t port,
                           mqtt_connection_cb_t callback, void * p_context,
                           const struct mqtt_connect_client_info_t * p_info)
{
    /* Parameters are validated below; context is unused in this fixture. */
    (void)p_context;
    mqttnet_test_check((NULL != p_client) && (NULL != p_address) &&
                        (1883U == port), "connect parameters");
    mqttnet_test_check(15U == p_info->keep_alive, "keepalive configured");
    connection_callback = callback;
    connects++;
    return ERR_OK;
}

/** @brief Synchronous close with no disconnect callback, matching lwIP. */
void mqtt_disconnect (mqtt_client_t * p_client)
{
    /* No transport object exists in this fixture. */
    (void)p_client;
    disconnects++;
}

/** @brief Store incoming callbacks exactly as the SDK API requests. */
void mqtt_set_inpub_callback (mqtt_client_t * p_client,
                              mqtt_incoming_publish_cb_t publish_callback,
                              mqtt_incoming_data_cb_t data_callback,
                              void * p_context)
{
    /* Client/context are fixed by this single-client test. */
    (void)p_client;
    (void)p_context;
    publication_callback = publish_callback;
    fragment_callback    = data_callback;
}

/** @brief Record subscription; SUBACK remains a separate injected event. */
err_t mqtt_sub_unsub (mqtt_client_t * p_client, const char * p_topic,
                      u8_t qos, mqtt_request_cb_t callback,
                      void * p_context, u8_t subscribe)
{
    /* Client/context identity has already been checked at CONNECT. */
    (void)p_client;
    (void)p_context;
    mqttnet_test_check((0U == qos) && (1U == subscribe) &&
                        (0 == strcmp(p_topic, NET_MQTT_COMMAND_TOPIC)),
                        "exact QoS0 subscription");
    subscription_callback = callback;
    subscribes++;
    return ERR_OK;
}

/** @brief Count accepted publications; this does not simulate delivery. */
err_t mqtt_publish (mqtt_client_t * p_client, const char * p_topic,
                    const void * p_payload, u16_t length, u8_t qos,
                    u8_t retain, mqtt_request_cb_t callback,
                    void * p_context)
{
    /* Packet encoding and TCP delivery belong to real lwIP, outside scope. */
    (void)p_client;
    (void)p_topic;
    (void)callback;
    (void)p_context;
    mqttnet_test_check((NULL != p_payload) && (length > 0U) &&
                        (0U == qos) && (0U == retain),
                        "bounded non-retained QoS0 publication");
    publications++;
    if ((ERR_OK == publish_result) &&
        (0 == strcmp(p_topic, NET_MQTT_TELEMETRY_TOPIC)))
    {
        memcpy(last_publication, p_payload, length);
        last_publication[length] = '\0';
    }
    return publish_result;
}

/** @brief Fixture address parser; parsing itself belongs to lwIP tests. */
int ip4addr_aton (const char * p_text, ip4_addr_t * p_address)
{
    /* Fixture always uses the configured nonempty IPv4 string. */
    (void)p_text;
    p_address->addr = 1U;
    return 1;
}

/** @brief Mock bounded queue setup. */
bool net_mqtt_ipc_start (void)
{
    return true;
}

/** @brief A bounded FIFO models ordered status/perception producer pairs. */
bool net_mqtt_ipc_send (const char * p_text, uint32_t length)
{
    uint32_t tail;

    if ((NULL == p_text) || (0U == length) ||
        (length > NET_MQTT_IPC_TX_MAX) ||
        (tx_count >= MQTTNET_TEST_TX_DEPTH))
    {
        return false;
    }
    tail = (tx_head + tx_count) % MQTTNET_TEST_TX_DEPTH;
    memcpy(tx_messages[tail], p_text, length);
    tx_lengths[tail] = length;
    tx_count++;
    return true;
}

/** @brief Supply no user-consumer input; parser enqueue is observed below. */
uint32_t net_mqtt_ipc_receive (char * p_text, uint32_t capacity)
{
    /* Receive behavior is exercised separately at the kernel boundary. */
    (void)p_text;
    (void)capacity;
    return 0U;
}

/** @brief Return one FIFO sample and consume it exactly once. */
uint32_t net_mqtt_ipc_take_tx (char * p_text, uint32_t capacity)
{
    uint32_t length;

    if (0U == tx_count)
    {
        return 0U;
    }
    length = tx_lengths[tx_head];
    if ((NULL == p_text) || (capacity <= length))
    {
        return 0U;
    }
    memcpy(p_text, tx_messages[tx_head], length);
    p_text[length] = '\0';
    tx_head = (tx_head + 1U) % MQTTNET_TEST_TX_DEPTH;
    tx_count--;
    return length;
}

/** @brief Save validated commands; queue scheduling is outside this test. */
bool net_mqtt_ipc_put_rx (const char * p_text, uint32_t length)
{
    memcpy(last_command, p_text, length);
    last_command[length] = '\0';
    commands++;
    return true;
}

/** @brief Record assertions without hiding later independent failures. */
static void mqttnet_test_check (bool b_result, const char * p_label)
{
    checks++;
    if (!b_result)
    {
        int result;

        failures++;
        result = fprintf(stderr, "FAIL: %s\n", p_label);
        /* Failure remains recorded if diagnostics cannot be written. */
        (void)result;
    }
}

/** @brief Joining is not link-up; verify timeout and bounded retry wait. */
static void mqttnet_test_join (struct netif * p_interface)
{
    p_interface->flags = NETIF_FLAG_UP;
    net_mqtt_poll(p_interface);
    mqttnet_test_check((1U == joins) && (0U == connects),
                        "initial join without premature MQTT connect");
    clock_ms = 29999U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(1U == joins, "join attempt has one active window");
    clock_ms = 30000U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(1U == leaves, "join timeout disassociates");
    clock_ms = 30999U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(1U == joins, "join backoff waits");
    clock_ms = 31000U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(2U == joins, "join retry after delay");
    p_interface->flags |= NETIF_FLAG_LINK_UP;
    p_interface->ip_addr.addr = 1U;
    clock_ms = 32000U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(1U == connects, "MQTT starts after actual link/IP");
    connection_callback(&mqtt_client, NULL, MQTT_CONNECT_ACCEPTED);
    mqttnet_test_check(1U == subscribes, "CONNACK triggers subscribe");
    clock_ms = 32100U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(0U == publications, "wait for successful SUBACK");
    subscription_callback(NULL, ERR_OK);
    clock_ms = 32200U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(2U == publications, "online status and telemetry");
}

/** @brief Reject retained and wrong-topic data; assemble one valid token. */
static void mqttnet_test_commands (void)
{
    const uint8_t stop[] = {'S', 'T', 'O', 'P'};
    const uint8_t start[] = {'S', 'T', 'A', 'R', 'T'};

    mqtt_client.rx_buffer[0] = 0x31U;
    publication_callback(NULL, NET_MQTT_COMMAND_TOPIC, 4U);
    fragment_callback(NULL, stop, 4U, MQTT_DATA_FLAG_LAST);
    mqttnet_test_check(0U == commands, "retained STOP rejected");
    mqtt_client.rx_buffer[0] = 0x30U;
    publication_callback(NULL, "wrong/topic", 4U);
    fragment_callback(NULL, stop, 4U, MQTT_DATA_FLAG_LAST);
    mqttnet_test_check(0U == commands, "wrong topic rejected");
    publication_callback(NULL, NET_MQTT_COMMAND_TOPIC, 5U);
    fragment_callback(NULL, start, 2U, 0U);
    mqttnet_test_check(0U == commands, "fragment alone cannot issue command");
    fragment_callback(NULL, &start[2], 3U, MQTT_DATA_FLAG_LAST);
    mqttnet_test_check((1U == commands) &&
                        (0 == strcmp(last_command, "START")),
                        "valid fragmented command queued once");
    fragment_callback(NULL, NULL, 0U, MQTT_DATA_FLAG_LAST);
    mqttnet_test_check(1U == commands, "final callback cannot replay");
}

/** @brief Three telemetry schemas survive a single producer activation. */
static void mqttnet_test_frame_triple (struct netif * p_interface)
{
    const char status[] = "{\"seq\":2}";
    const char perception[] = "{\"seq\":2,\"ba\":0}";
    const char diagnostic[] = "{\"seq\":2,\"fc\":0,\"fm\":0}";

    mqttnet_test_check(net_mqtt_send(status, sizeof(status) - 1U),
                        "status frame queued");
    mqttnet_test_check(net_mqtt_send(perception, sizeof(perception) - 1U),
                        "perception frame queued");
    mqttnet_test_check(net_mqtt_send(diagnostic, sizeof(diagnostic) - 1U),
                        "diagnostic frame queued");
    clock_ms = 32300U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check((3U == publications) && (2U == tx_count),
                        "one FIFO frame transmitted per poll interval");
    clock_ms = 32400U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check((4U == publications) && (1U == tx_count),
                        "second distinct frame remains and transmits");
    clock_ms = 32500U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check((5U == publications) && (0U == tx_count) &&
                        (0 == strcmp(last_publication, diagnostic)),
                        "third diagnostic frame remains and transmits");
}

/** @brief Failure retry delays and connection watchdog remain bounded. */
static void mqttnet_test_reconnect (struct netif * p_interface)
{
    uint32_t original_commands = commands;
    const uint8_t stop[] = {'S', 'T', 'O', 'P'};

    mqtt_client.rx_buffer[0] = 0x30U;
    publication_callback(NULL, NET_MQTT_COMMAND_TOPIC, 4U);
    fragment_callback(NULL, stop, 2U, 0U);
    clock_ms = 33000U;
    connection_callback(&mqtt_client, NULL, MQTT_CONNECT_DISCONNECTED);
    fragment_callback(NULL, &stop[2], 2U, MQTT_DATA_FLAG_LAST);
    mqttnet_test_check(commands == original_commands,
                        "disconnect discards incomplete command");
    publication_callback(NULL, NET_MQTT_COMMAND_TOPIC, 4U);
    fragment_callback(NULL, stop, 4U, MQTT_DATA_FLAG_LAST);
    mqttnet_test_check(commands == original_commands,
                        "disconnected client cannot issue commands");
    clock_ms = 33999U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(1U == connects, "MQTT first backoff waits 1 second");
    clock_ms = 34000U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(2U == connects, "MQTT retries after 1 second");
    clock_ms = 63999U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(0U == disconnects, "connect window remains active");
    clock_ms = 64000U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(1U == disconnects, "connect timeout closes client");
    clock_ms = 65999U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(2U == connects, "second backoff waits 2 seconds");
    clock_ms = 66000U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(3U == connects, "second retry after 2 seconds");
    p_interface->flags = NETIF_FLAG_UP;
    clock_ms = 66100U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(2U == disconnects, "link loss closes MQTT attempt");
}

/** @brief Backpressure preserves one frame and the following FIFO frame. */
static void mqttnet_test_backpressure (struct netif * p_interface)
{
    uint32_t original_publications = publications;

    mqttnet_test_check(net_mqtt_send("first", 5U),
                        "backpressure first frame queued");
    mqttnet_test_check(net_mqtt_send("second", 6U),
                        "backpressure second frame queued");
    publish_result = ERR_MEM;
    clock_ms = 32600U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check((publications == original_publications + 1U) &&
                        (1U == tx_count),
                        "full transport retains first frame separately");
    clock_ms = 32699U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(publications == original_publications + 1U,
                        "backpressure does not cause busy-loop publishing");
    publish_result = ERR_OK;
    clock_ms = 32700U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check((0 == strcmp(last_publication, "first")) &&
                        (1U == tx_count), "retry transmits original frame");
    clock_ms = 32800U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check((0 == strcmp(last_publication, "second")) &&
                        (0U == tx_count), "next FIFO frame follows retry");
}

/** @brief MQTT retry elapsed-time arithmetic survives the 32-bit wrap. */
static void mqttnet_test_rollover (struct netif * p_interface)
{
    uint32_t original_connects;

    p_interface->flags = NETIF_FLAG_UP | NETIF_FLAG_LINK_UP;
    p_interface->ip_addr.addr = 1U;
    clock_ms = UINT32_MAX - 2000U;
    net_mqtt_poll(p_interface);
    connection_callback(&mqtt_client, NULL, MQTT_CONNECT_ACCEPTED);
    subscription_callback(NULL, ERR_OK);
    original_connects = connects;
    clock_ms = UINT32_MAX - 499U;
    connection_callback(&mqtt_client, NULL, MQTT_CONNECT_DISCONNECTED);
    clock_ms = 499U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(connects == original_connects,
                        "retry waits 999 ms across counter wrap");
    clock_ms = 500U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(connects == original_connects + 1U,
                        "retry starts at 1000 ms across counter wrap");
}

/** @brief lwIP reports a SUBACK timeout from its cyclic MQTT timer, which
 * re-arms itself afterwards. Closing inside that callback would leave a
 * duplicate timer, so the close must wait for the next radio-owner poll.
 */
static void mqttnet_test_suback_timeout (struct netif * p_interface)
{
    uint32_t original_disconnects = disconnects;
    uint32_t original_connects = connects;
    uint32_t original_publications = publications;

    clock_ms = 600U;
    connection_callback(&mqtt_client, NULL, MQTT_CONNECT_ACCEPTED);
    subscription_callback(NULL, ERR_TIMEOUT);
    mqttnet_test_check(disconnects == original_disconnects,
                        "SUBACK timeout does not close inside lwIP callback");
    clock_ms = 700U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(disconnects == (original_disconnects + 1U),
                        "next poll performs the deferred close");
    mqttnet_test_check(publications == original_publications,
                        "no telemetry without a successful SUBACK");
    clock_ms = 800U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(disconnects == (original_disconnects + 1U),
                        "deferred close happens only once");
    clock_ms = 2699U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(connects == original_connects,
                        "deferred close keeps the doubled retry backoff");
    clock_ms = 2700U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(connects == (original_connects + 1U),
                        "client reconnects after the deferred close");
    connection_callback(&mqtt_client, NULL, MQTT_CONNECT_ACCEPTED);
    subscription_callback(NULL, ERR_TIMEOUT);
    connection_callback(&mqtt_client, NULL, MQTT_CONNECT_DISCONNECTED);
    clock_ms = 2800U;
    net_mqtt_poll(p_interface);
    mqttnet_test_check(disconnects == (original_disconnects + 1U),
                        "retry after a closed connection clears the close");
}

/* End of file. */

