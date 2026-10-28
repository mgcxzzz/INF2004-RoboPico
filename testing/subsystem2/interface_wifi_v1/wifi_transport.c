#include "wifi_transport.h"

#include <string.h>
#include <stdio.h>

#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "hardware/sync.h"

#ifndef BUDDY_WIFI_ENABLED
#define BUDDY_WIFI_ENABLED 0
#endif
#ifndef BUDDY_WIFI_LOCAL_CONFIG
#define BUDDY_WIFI_LOCAL_CONFIG 0
#endif

#if BUDDY_WIFI_ENABLED
#define WIFI_TX_DEPTH 12u
#define WIFI_RX_DEPTH 8u
#define WIFI_RX_COPY_MAX 8u
#define WIFI_MQTT_PORT 1883u
#define WIFI_KEEPALIVE_SECONDS 5u
#define WIFI_CONNECT_TIMEOUT_MS 10000u
#define WIFI_JOIN_TIMEOUT_MS 15000u
#define WIFI_POLL_INTERVAL_MS 2u
#define WIFI_FLASH_TIMEOUT_US 50000u

typedef struct {
    uint8_t topic;
    uint8_t len;
    uint8_t data[WIFI_PACKET_MAX];
} TxPacket;

typedef struct {
    uint8_t len;
    uint8_t copied[WIFI_RX_COPY_MAX];
    size_t original_len;
} RxPacket;
#endif

static spin_lock_t *g_queue_lock;
#if BUDDY_WIFI_ENABLED
static WifiTransportConfig g_network_config;
static bool g_config_is_valid;
static bool g_worker_launched;
#endif
#if BUDDY_WIFI_ENABLED
static TxPacket g_tx[WIFI_TX_DEPTH];
static unsigned g_tx_count;
static RxPacket g_rx[WIFI_RX_DEPTH];
static unsigned g_rx_head;
static unsigned g_rx_tail;
static unsigned g_rx_count;
static TxPacket g_pending;
static bool g_pending_valid;
#endif
static WifiTransportStatus g_status;
static bool g_initialized;

#if BUDDY_WIFI_ENABLED
static bool has_terminator(const char *text, size_t capacity)
{
    return text != NULL && memchr(text, '\0', capacity) != NULL;
}

static bool valid_ipv4(const char *text)
{
    unsigned octet = 0u;
    unsigned digits = 0u;
    unsigned dots = 0u;
    const char *p = text;

    if (text == NULL || *text == '\0') return false;
    for (; *p != '\0'; ++p) {
        if (*p >= '0' && *p <= '9') {
            octet = octet * 10u + (unsigned)(*p - '0');
            if (octet > 255u || ++digits > 3u) return false;
        } else if (*p == '.' && digits != 0u && dots < 3u) {
            dots++;
            digits = 0u;
            octet = 0u;
        } else {
            return false;
        }
    }
    return dots == 3u && digits != 0u;
}

static bool valid_config(const WifiTransportConfig *config)
{
    return config != NULL &&
           has_terminator(config->ssid, sizeof(config->ssid)) &&
           has_terminator(config->password, sizeof(config->password)) &&
           has_terminator(config->broker, sizeof(config->broker)) &&
           has_terminator(config->client_id, sizeof(config->client_id)) &&
           config->ssid[0] != '\0' && config->client_id[0] != '\0' &&
           valid_ipv4(config->broker);
}
#endif

bool wifi_transport_configure(const WifiTransportConfig *config)
{
#if BUDDY_WIFI_ENABLED
    if (g_initialized || !valid_config(config)) return false;
    g_network_config = *config;
    g_config_is_valid = true;
    return true;
#else
    (void)config;
    return false;
#endif
}

static uint32_t lock_queues(void)
{
    return spin_lock_blocking(g_queue_lock);
}

static void unlock_queues(uint32_t irq_state)
{
    spin_unlock(g_queue_lock, irq_state);
}

bool wifi_transport_send(unsigned topic, const uint8_t *data, size_t len)
{
#if BUDDY_WIFI_ENABLED
    unsigned i;
    uint32_t irq_state;

    if (!g_initialized || data == NULL || len == 0u || len > WIFI_PACKET_MAX ||
        topic > WIFI_TOPIC_HB) {
        return false;
    }
    irq_state = lock_queues();
    if (g_tx_count == WIFI_TX_DEPTH) {
        /* Keep command answers and heartbeat state ahead of telemetry. */
        if (topic == WIFI_TOPIC_TLM) {
            g_status.tx_dropped++;
            unlock_queues(irq_state);
            return false;
        }
        for (i = 0; i < g_tx_count; ++i) {
            if (g_tx[i].topic == WIFI_TOPIC_TLM) {
                memmove(&g_tx[i], &g_tx[i + 1u],
                        (g_tx_count - i - 1u) * sizeof(g_tx[0]));
                g_tx_count--;
                g_status.tx_dropped++;
                break;
            }
        }
        if (g_tx_count == WIFI_TX_DEPTH) {
            g_status.tx_dropped++;
            unlock_queues(irq_state);
            return false;
        }
    }
    g_tx[g_tx_count].topic = (uint8_t)topic;
    g_tx[g_tx_count].len = (uint8_t)len;
    memcpy(g_tx[g_tx_count].data, data, len);
    g_tx_count++;
    unlock_queues(irq_state);
    return true;
#else
    (void)topic;
    (void)data;
    (void)len;
    return false;
#endif
}

bool wifi_transport_receive(uint8_t data[8], size_t *original_len)
{
#if BUDDY_WIFI_ENABLED
    RxPacket packet;
    uint32_t irq_state;

    if (data == NULL || original_len == NULL || !g_initialized) {
        return false;
    }
    irq_state = lock_queues();
    if (g_rx_count == 0u) {
        unlock_queues(irq_state);
        return false;
    }
    packet = g_rx[g_rx_tail];
    g_rx_tail = (g_rx_tail + 1u) % WIFI_RX_DEPTH;
    g_rx_count--;
    unlock_queues(irq_state);
    memset(data, 0, 8u);
    memcpy(data, packet.copied, packet.len);
    *original_len = packet.original_len;
    return true;
#else
    (void)data;
    (void)original_len;
    return false;
#endif
}

WifiTransportStatus wifi_transport_status(void)
{
    WifiTransportStatus snapshot;
    uint32_t irq_state;

    if (g_queue_lock == NULL) {
        memset(&snapshot, 0, sizeof(snapshot));
        return snapshot;
    }
    irq_state = lock_queues();
    snapshot = g_status;
#if BUDDY_WIFI_ENABLED
    snapshot.tx_fill_pct = (uint8_t)((g_tx_count * 100u) / WIFI_TX_DEPTH);
#else
    snapshot.tx_fill_pct = 0u;
#endif
    unlock_queues(irq_state);
    return snapshot;
}

bool wifi_transport_flash_begin(void)
{
#if BUDDY_WIFI_ENABLED
    if (!g_initialized || !g_worker_launched) {
        return true;
    }
    if (!multicore_lockout_victim_is_initialized(1u)) return false;
    if (!multicore_lockout_start_timeout_us(WIFI_FLASH_TIMEOUT_US)) {
        return false;
    }
    return true;
#else
    return true;
#endif
}

void wifi_transport_flash_end(void)
{
#if BUDDY_WIFI_ENABLED
    if (g_initialized && g_worker_launched) {
        multicore_lockout_end_blocking();
    }
#endif
}

#if BUDDY_WIFI_ENABLED

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "pico/cyw43_arch.h"
#include "cyw43.h"
#include "wifi_link_state.h"
#if BUDDY_WIFI_LOCAL_CONFIG
#include "wifi_config.local.h"
#endif

#define WIFI_TOPIC_TLM_TEXT "is28/tlm"
#define WIFI_TOPIC_ACK_TEXT "is28/ack"
#define WIFI_TOPIC_HB_TEXT "is28/hb"
#define WIFI_TOPIC_CMD_TEXT "is28/cmd"
#define WIFI_TOPIC_LWT_TEXT "is28/lwt"

static mqtt_client_t *g_mqtt;
static ip_addr_t g_broker;
static bool g_wifi_started;
static bool g_connecting;
static bool g_mqtt_connected;
static bool g_subscribing;
static bool g_wifi_joining;
static bool g_wifi_was_up;
static bool g_rx_is_cmd;
static uint8_t g_rx_staging[WIFI_RX_COPY_MAX];
static uint8_t g_rx_staging_len;
static size_t g_rx_original_len;
static uint32_t g_retry_at;
static uint32_t g_wifi_retry_at;
static uint32_t g_wifi_join_started;
static uint32_t g_connect_started;
static uint32_t g_attempts;
static uint32_t g_wifi_attempts;
static uint32_t g_last_poll_us;
static uint32_t g_poll_gap_max_us;

static void discard_stale_tx(void)
{
    uint32_t irq_state = lock_queues();
    g_status.tx_dropped += g_tx_count;
    g_tx_count = 0u;
    if (g_pending_valid) {
        g_status.tx_dropped++;
        g_pending_valid = false;
    }
    unlock_queues(irq_state);
}

static const char *mqtt_topic(unsigned topic)
{
    switch (topic) {
    case WIFI_TOPIC_ACK:
        return WIFI_TOPIC_ACK_TEXT;
    case WIFI_TOPIC_HB:
        return WIFI_TOPIC_HB_TEXT;
    default:
        return WIFI_TOPIC_TLM_TEXT;
    }
}

static uint32_t retry_delay_ms(uint32_t attempt)
{
    if (attempt <= 1u) return 1000u;
    if (attempt == 2u) return 2000u;
    if (attempt == 3u) return 4000u;
    return 8000u;
}

static void schedule_retry(uint32_t now)
{
    if (g_attempts < UINT32_MAX) g_attempts++;
    g_retry_at = now + retry_delay_ms(g_attempts);
    g_connecting = false;
    g_mqtt_connected = false;
    g_subscribing = false;
    discard_stale_tx();
    {
        uint32_t irq_state = lock_queues();
        g_status.connected = 0u;
        g_status.subscribed = 0u;
        g_status.reconnects++;
        unlock_queues(irq_state);
    }
}

static void schedule_wifi_retry(uint32_t now)
{
    if (g_wifi_attempts < UINT32_MAX) g_wifi_attempts++;
    g_wifi_retry_at = now + retry_delay_ms(g_wifi_attempts);
    g_wifi_joining = false;
}

static void start_wifi_join(uint32_t now)
{
    const char *password = g_network_config.password[0] == '\0' ?
        NULL : g_network_config.password;
    uint32_t auth = password == NULL ? CYW43_AUTH_OPEN : CYW43_AUTH_WPA2_AES_PSK;
    int result = cyw43_arch_wifi_connect_async(g_network_config.ssid, password, auth);

    if (result == 0) {
        g_wifi_joining = true;
        g_wifi_join_started = now;
    } else {
        schedule_wifi_retry(now);
    }
}

static void incoming_publish(void *arg, const char *topic, uint32_t total_len)
{
    (void)arg;
    g_rx_is_cmd = topic != NULL && strcmp(topic, WIFI_TOPIC_CMD_TEXT) == 0;
    g_rx_staging_len = 0u;
    g_rx_original_len = (size_t)total_len;
}

static void enqueue_received_command(void)
{
    uint32_t irq_state = lock_queues();
    if (g_rx_count == WIFI_RX_DEPTH) {
        g_status.rx_dropped++;
    } else {
        RxPacket *packet = &g_rx[g_rx_head];
        packet->len = g_rx_staging_len;
        memcpy(packet->copied, g_rx_staging, g_rx_staging_len);
        packet->original_len = g_rx_original_len;
        g_rx_head = (g_rx_head + 1u) % WIFI_RX_DEPTH;
        g_rx_count++;
    }
    unlock_queues(irq_state);
}

static void incoming_data(void *arg, const uint8_t *data, uint16_t len,
                          uint8_t flags)
{
    size_t i;
    (void)arg;
    if (g_rx_is_cmd && data != NULL) {
        for (i = 0u; i < (size_t)len && g_rx_staging_len < WIFI_RX_COPY_MAX; ++i) {
            g_rx_staging[g_rx_staging_len++] = data[i];
        }
    }
    if ((flags & MQTT_DATA_FLAG_LAST) != 0u) {
        if (g_rx_is_cmd) enqueue_received_command();
        g_rx_is_cmd = false;
    }
}

static void subscribe_callback(void *arg, err_t result)
{
    (void)arg;
    g_subscribing = false;
    if (result == ERR_OK) {
        uint32_t irq_state = lock_queues();
        g_status.subscribed = 1u;
        unlock_queues(irq_state);
    } else {
        schedule_retry(to_ms_since_boot(get_absolute_time()));
    }
}

static void connection_callback(mqtt_client_t *client, void *arg,
                                mqtt_connection_status_t status)
{
    uint32_t now = to_ms_since_boot(get_absolute_time());
    (void)client;
    (void)arg;
    uint32_t status_irq=lock_queues();
    g_status.mqtt_result=(int16_t)status;
    unlock_queues(status_irq);
    g_connecting = false;
    if (status == MQTT_CONNECT_ACCEPTED) {
        err_t result;
        discard_stale_tx();
        g_attempts = 0u;
        g_mqtt_connected = true;
        mqtt_set_inpub_callback(g_mqtt, incoming_publish, incoming_data, NULL);
        g_subscribing = true;
        result = mqtt_subscribe(g_mqtt, WIFI_TOPIC_CMD_TEXT, 0u,
                                subscribe_callback, NULL);
        if (result != ERR_OK) {
            g_subscribing = false;
            schedule_retry(now);
            return;
        }
        {
            uint32_t irq_state = lock_queues();
            g_status.connected = 1u;
            unlock_queues(irq_state);
        }
    } else {
        schedule_retry(now);
    }
}

static void begin_connect(uint32_t now)
{
    struct mqtt_connect_client_info_t info;
    err_t result;
    static const uint8_t offline = 0u;
    uint32_t status_irq=lock_queues();
    g_status.mqtt_attempts++;
    unlock_queues(status_irq);

    if (g_mqtt == NULL) {
        g_mqtt = mqtt_client_new();
    if (g_mqtt == NULL || !ip4addr_aton(g_network_config.broker, ip_2_ip4(&g_broker))) {
            g_mqtt = NULL;
            schedule_retry(now);
            return;
        }
        IP_SET_TYPE(&g_broker, IPADDR_TYPE_V4);
    }
    mqtt_disconnect(g_mqtt);
    memset(&info, 0, sizeof(info));
    info.client_id = g_network_config.client_id;
    info.keep_alive = WIFI_KEEPALIVE_SECONDS;
    info.will_topic = WIFI_TOPIC_LWT_TEXT;
    info.will_msg = (const char *)&offline;
    info.will_msg_len = 1u;
    info.will_qos = 0u;
    info.will_retain = 0u;
    result = mqtt_client_connect(g_mqtt, &g_broker, WIFI_MQTT_PORT,
                                 connection_callback, NULL, &info);
    status_irq=lock_queues();
    g_status.mqtt_result=(int16_t)result;
    unlock_queues(status_irq);
    if (result == ERR_OK) {
        g_connecting = true;
        g_connect_started = now;
    } else {
        schedule_retry(now);
    }
}

static bool take_tx_packet(void)
{
    unsigned chosen = 0u;
    unsigned i;
    uint32_t irq_state = lock_queues();
    if (g_tx_count == 0u) {
        unlock_queues(irq_state);
        return false;
    }
    for (i = 0u; i < g_tx_count; ++i) {
        if (g_tx[i].topic == WIFI_TOPIC_ACK) {
            chosen = i;
            break;
        }
        if (g_tx[i].topic == WIFI_TOPIC_HB) chosen = i;
    }
    g_pending = g_tx[chosen];
    memmove(&g_tx[chosen], &g_tx[chosen + 1u],
            (g_tx_count - chosen - 1u) * sizeof(g_tx[0]));
    g_tx_count--;
    g_pending_valid = true;
    unlock_queues(irq_state);
    return true;
}

static void pump_publish(void)
{
    unsigned sent;
    for (sent = 0u; sent < 4u; ++sent) {
        err_t result;
        if (!g_pending_valid) {
            if (!take_tx_packet()) break;
        }
        result = mqtt_publish(g_mqtt, mqtt_topic(g_pending.topic),
                              (const char *)g_pending.data, g_pending.len,
                              0u, 0u, NULL, NULL);
        if (result == ERR_MEM) break;
        if (result == ERR_OK) {
            uint32_t irq_state = lock_queues();
            g_status.published++;
            g_pending_valid = false;
            unlock_queues(irq_state);
        } else {
            uint32_t irq_state = lock_queues();
            g_status.tx_dropped++;
            g_pending_valid = false;
            unlock_queues(irq_state);
        }
    }
}

static void network_core(void)
{
    uint32_t irq_state;
    uint32_t now;
    multicore_lockout_victim_init();
    if (cyw43_arch_init() != 0) {
        for (;;) tight_loop_contents();
    }
    cyw43_arch_enable_sta_mode();
    uint32_t active_pm=UINT32_MAX;
    int pm_set=cyw43_wifi_pm(&cyw43_state,CYW43_NONE_PM);
    int pm_read=cyw43_wifi_get_pm(&cyw43_state,&active_pm);
    irq_state=lock_queues();
    g_status.pm_disabled=pm_set==0 && pm_read==0 && (active_pm&15u)==0;
    unlock_queues(irq_state);
    g_wifi_started = true;
    now = to_ms_since_boot(get_absolute_time());
    start_wifi_join(now);
    irq_state = lock_queues();
    g_status.ready = 1u;
    unlock_queues(irq_state);

    for (;;) {
        uint32_t now_us;
        uint32_t gap;
        int link_status;

        cyw43_arch_poll();
        now_us = time_us_32();
        gap = g_last_poll_us == 0u ? 0u : now_us - g_last_poll_us;
        g_last_poll_us = now_us;
        if (gap > g_poll_gap_max_us) g_poll_gap_max_us = gap;
        irq_state = lock_queues();
        if (g_poll_gap_max_us > g_status.max_poll_gap_us)
            g_status.max_poll_gap_us = g_poll_gap_max_us;
        unlock_queues(irq_state);

        now = to_ms_since_boot(get_absolute_time());
        link_status = g_wifi_started ? wifi_network_link_status() : CYW43_LINK_DOWN;
        irq_state=lock_queues();
        g_status.link_status=(int8_t)link_status;
        g_status.ipv4=link_status==CYW43_LINK_UP ?
            lwip_ntohl(netif_ip4_addr(&cyw43_state.netif[CYW43_ITF_STA])->addr) : 0u;
        unlock_queues(irq_state);
        if (link_status == CYW43_LINK_UP) {
            g_wifi_was_up = true;
            g_wifi_joining = false;
            g_wifi_attempts = 0u;
            if (g_mqtt_connected && !mqtt_client_is_connected(g_mqtt)) {
                schedule_retry(now);
            }
            if (!g_mqtt_connected && !g_connecting &&
                (int32_t)(now - g_retry_at) >= 0) {
                begin_connect(now);
            } else if (g_connecting && (now - g_connect_started) > WIFI_CONNECT_TIMEOUT_MS) {
                mqtt_disconnect(g_mqtt);
                schedule_retry(now);
            } else if (g_mqtt_connected) {
                pump_publish();
            }
        } else {
            if (g_mqtt_connected || g_connecting) {
                if (g_mqtt != NULL) mqtt_disconnect(g_mqtt);
                schedule_retry(now);
            } else {
                discard_stale_tx();
            }
            if (g_wifi_was_up) {
                g_wifi_was_up = false;
                g_wifi_joining = false;
                g_wifi_retry_at = now;
            }
            if (g_wifi_joining && link_status < 0) {
                schedule_wifi_retry(now);
            } else if (g_wifi_joining &&
                       (now - g_wifi_join_started) > WIFI_JOIN_TIMEOUT_MS) {
                schedule_wifi_retry(now);
            }
            if (!g_wifi_joining && (int32_t)(now - g_wifi_retry_at) >= 0) {
                start_wifi_join(now);
            }
        }
        sleep_ms(WIFI_POLL_INTERVAL_MS);
    }
}

#endif /* BUDDY_WIFI_ENABLED */

void wifi_transport_init(void)
{
    if (g_initialized) return;
    g_queue_lock = spin_lock_instance(spin_lock_claim_unused(true));
#if BUDDY_WIFI_ENABLED
#if BUDDY_WIFI_LOCAL_CONFIG
_Static_assert(sizeof(WIFI_SSID) <= sizeof(((WifiTransportConfig *)0)->ssid), "WIFI_SSID exceeds transport config");
_Static_assert(sizeof(WIFI_PASSWORD) <= sizeof(((WifiTransportConfig *)0)->password), "WIFI_PASSWORD exceeds transport config");
_Static_assert(sizeof(MQTT_BROKER_IP) <= sizeof(((WifiTransportConfig *)0)->broker), "MQTT_BROKER_IP exceeds transport config");
_Static_assert(sizeof(MQTT_CLIENT_ID) <= sizeof(((WifiTransportConfig *)0)->client_id), "MQTT_CLIENT_ID exceeds transport config");
    if (!g_config_is_valid) {
        WifiTransportConfig local_config = {0};
        (void)snprintf(local_config.ssid, sizeof(local_config.ssid), "%s", WIFI_SSID);
        (void)snprintf(local_config.password, sizeof(local_config.password), "%s", WIFI_PASSWORD);
        (void)snprintf(local_config.broker, sizeof(local_config.broker), "%s", MQTT_BROKER_IP);
        (void)snprintf(local_config.client_id, sizeof(local_config.client_id), "%s", MQTT_CLIENT_ID);
        if (valid_config(&local_config)) {
            g_network_config = local_config;
            g_config_is_valid = true;
        }
    }
#endif
    memset(&g_status, 0, sizeof(g_status));
    g_status.mqtt_result=INT16_MIN;
    g_status.configured = g_config_is_valid ? 1u : 0u;
    g_status.ready = 0u;
    g_status.connected = 0u;
    g_status.subscribed = 0u;
    g_retry_at = 0u;
    g_initialized = true;
    if (g_config_is_valid) {
        g_worker_launched = true;
        multicore_launch_core1(network_core);
    } else {
        puts("Wi-Fi/MQTT is unconfigured. Use USB setup to provide SSID, password, laptop broker IPv4, and client ID.");
    }
#else
    memset(&g_status, 0, sizeof(g_status));
    puts("Wi-Fi/MQTT transport is not configured. Copy wifi_config.example.h to wifi_config.local.h, set Wi-Fi and laptop broker values, then rebuild with BUDDY_WIFI_ENABLED=ON.");
    g_initialized = true;
#endif
}
