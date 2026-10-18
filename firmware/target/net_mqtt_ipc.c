/**
 * @file net_mqtt_ipc.c
 * @brief Three bounded micro T-Kernel message buffers, all calls polling.
 * Kernel headers deliberately remain isolated from ISO stddef/lwIP headers.
 * NOTE: net_mqtt_ipc shares net_mqtt's first eight filename characters;
 * this explicit adapter boundary is a proposed BARR-C 4.1.b deviation.
 */
#include "net_mqtt_ipc.h"

#include <tk/tkernel.h>

#define NET_MQTT_IPC_DEPTH 4U
#define NET_MQTT_IPC_STOP_SIZE 4U

static ID gh_tx_id   = 0;
static ID gh_rx_id   = 0;
static ID gh_stop_id = 0;

static uint32_t g_tx_storage[NET_MQTT_IPC_DEPTH *
                          ((NET_MQTT_IPC_TX_MAX / 4U) + 1U)];
static uint32_t g_rx_storage[NET_MQTT_IPC_DEPTH *
                          ((NET_MQTT_IPC_RX_MAX / 4U) + 1U)];
static uint32_t g_stop_storage[2U];

static ID net_mqtt_ipc_create (void * p_buffer, uint32_t size,
                              uint32_t maximum);
static uint32_t net_mqtt_ipc_take (ID h_buffer, char * p_text);

/** @brief Called exactly once before task creation; idempotent on success. */
bool net_mqtt_ipc_start (void)
{
    ER result;

    if (gh_tx_id > 0)
    {
        return true;
    }
    gh_tx_id = net_mqtt_ipc_create(g_tx_storage, sizeof(g_tx_storage),
                                 NET_MQTT_IPC_TX_MAX);
    if (gh_tx_id <= 0)
    {
        return false;
    }
    gh_rx_id = net_mqtt_ipc_create(g_rx_storage, sizeof(g_rx_storage),
                                 NET_MQTT_IPC_RX_MAX);
    if (gh_rx_id > 0)
    {
        gh_stop_id = net_mqtt_ipc_create(g_stop_storage, sizeof(g_stop_storage),
                                       NET_MQTT_IPC_STOP_SIZE);
    }
    if ((gh_rx_id <= 0) || (gh_stop_id <= 0))
    {
        result = tk_del_mbf(gh_tx_id);
        /* Cleanup result cannot repair creation; report start failure. */
        (void)result;
        if (gh_rx_id > 0)
        {
            result = tk_del_mbf(gh_rx_id);
            /* Same fail-closed cleanup policy as above. */
            (void)result;
        }
        gh_tx_id = 0;
        gh_rx_id = 0;
        return false;
    }
    return true;
}

/** @brief Enqueue one telemetry record without delaying control tasks. */
bool net_mqtt_ipc_send (const char * p_text, uint32_t length)
{
    if ((NULL == p_text) || (0U == length) ||
        (length > NET_MQTT_IPC_TX_MAX) || (gh_tx_id <= 0))
    {
        return false;
    }
    /* Validated length is at most 256, representable in kernel INT. */
    return E_OK == tk_snd_mbf(gh_tx_id, p_text, (INT)length, TMO_POL);
}

/** @brief Return a complete command; a pending STOP always wins. */
uint32_t net_mqtt_ipc_receive (char * p_text, uint32_t capacity)
{
    uint32_t length;

    if ((NULL == p_text) || (capacity <= NET_MQTT_IPC_RX_MAX) ||
        (gh_rx_id <= 0) || (gh_stop_id <= 0))
    {
        return 0U;
    }
    length = net_mqtt_ipc_take(gh_stop_id, p_text);
    if (0U == length)
    {
        length = net_mqtt_ipc_take(gh_rx_id, p_text);
    }
    return length;
}

/** @brief Only the radio task consumes the telemetry queue. */
uint32_t net_mqtt_ipc_take_tx (char * p_text, uint32_t capacity)
{
    if ((NULL == p_text) || (capacity <= NET_MQTT_IPC_TX_MAX) ||
        (gh_tx_id <= 0))
    {
        return 0U;
    }
    return net_mqtt_ipc_take(gh_tx_id, p_text);
}

/** @brief Route exact STOP separately; duplicates remain safely pending. */
bool net_mqtt_ipc_put_rx (const char * p_text, uint32_t length)
{
    ID h_target = gh_rx_id;
    ER result;

    if ((NULL == p_text) || (0U == length) ||
        (length > NET_MQTT_IPC_RX_MAX) || (gh_rx_id <= 0))
    {
        return false;
    }
    if ((NET_MQTT_IPC_STOP_SIZE == length) && ('S' == p_text[0]) &&
        ('T' == p_text[1]) && ('O' == p_text[2]) && ('P' == p_text[3]))
    {
        h_target = gh_stop_id;
    }
    /* Validated length is at most 32, representable in kernel INT. */
    result = tk_snd_mbf(h_target, p_text, (INT)length, TMO_POL);
    return (E_OK == result) ||
           ((h_target == gh_stop_id) && (E_TMOUT == result));
}

/** @brief Create fixed user storage; kernel copies each complete message. */
static ID net_mqtt_ipc_create (void * p_buffer, uint32_t size,
                              uint32_t maximum)
{
    T_CMBF config = {0};

    config.mbfatr = TA_TFIFO | TA_USERBUF;
    /* All callers use static buffers <=1040 bytes and messages <=256. */
    config.bufsz  = (SZ)size;
    config.maxmsz = (INT)maximum;
    config.bufptr = p_buffer;
    return tk_cre_mbf(&config);
}

/** @brief Internal callers guarantee max-message-size plus NUL capacity. */
static uint32_t net_mqtt_ipc_take (ID h_buffer, char * p_text)
{
    INT length = tk_rcv_mbf(h_buffer, p_text, TMO_POL);

    if (length <= 0)
    {
        return 0U;
    }
    p_text[length] = '\0';
    /* Positive message length is bounded by the configured maximum, 256. */
    return (uint32_t)length;
}

/* End of file. */

