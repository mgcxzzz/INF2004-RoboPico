#ifndef WIFI_TRANSPORT_H
#define WIFI_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define WIFI_PACKET_MAX 64u

enum { WIFI_TOPIC_TLM = 0, WIFI_TOPIC_ACK = 1, WIFI_TOPIC_HB = 2 };

typedef struct {
    unsigned configured : 1;
    unsigned ready : 1;
    unsigned connected : 1;
    unsigned subscribed : 1;
    unsigned pm_disabled : 1;
    uint32_t tx_dropped;
    uint32_t rx_dropped;
    uint32_t published;
    uint32_t reconnects;
    uint32_t max_poll_gap_us;
    uint8_t tx_fill_pct; /* queued ring occupancy; excludes core1's in-flight packet */
    int8_t link_status; /* SDK: 0 down,1 join,2 no IP,3 IP ready,-1 fail,-2 no SSID,-3 auth */
    int16_t mqtt_result; /* latest connect API/callback result; -32768 before an attempt */
    uint32_t ipv4; /* host-order address, copied from netif by core1 */
    uint32_t mqtt_attempts;
    uint32_t polls; /* completed core1 network poll iterations */
    uint32_t last_poll_ms; /* millis since boot at last completed poll */
    uint32_t broker_ipv4; /* host-order configured broker IPv4 */
    unsigned phase : 3; /* 0 init, 1 poll, 2 link, 3 mqtt_connect, 4 mqtt_disconnect, 5 publish, 6 sleep */
} WifiTransportStatus;

typedef struct {
    char ssid[33];
    char password[65];
    char broker[16];
    char client_id[32];
} WifiTransportConfig;

/* Optional runtime configuration, core0 only and only before init. */
bool wifi_transport_configure(const WifiTransportConfig *config);
/* Call once from core0. Starts core1 only when local Wi-Fi configuration exists. */
void wifi_transport_init(void);
/* Copies a packet into a bounded queue; never waits for the network. */
bool wifi_transport_send(unsigned topic, const uint8_t *data, size_t len);
/* Copies at most eight bytes; original_len reports the complete MQTT payload size. */
bool wifi_transport_receive(uint8_t data[8], size_t *original_len);
WifiTransportStatus wifi_transport_status(void);
/* Call on core0 before disabling IRQs for flash operations. */
bool wifi_transport_flash_begin(void);
/* Call on core0 after IRQs have been restored. */
void wifi_transport_flash_end(void);

#endif
