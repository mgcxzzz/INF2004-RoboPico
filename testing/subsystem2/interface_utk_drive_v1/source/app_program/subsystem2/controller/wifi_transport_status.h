#ifndef WIFI_TRANSPORT_STATUS_H
#define WIFI_TRANSPORT_STATUS_H

#include <stdint.h>

enum { WIFI_TOPIC_TLM = 0, WIFI_TOPIC_ACK = 1, WIFI_TOPIC_HB = 2 };

/* Network diagnostics copied across the kernel bridge. Keep this header
 * independent of stddef/T-Kernel declarations so both sides can include it. */
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
    uint8_t tx_fill_pct;
    int8_t link_status;
    int16_t mqtt_result;
    int16_t connect_api_result;
    int16_t subscribe_result;
    int16_t last_error;
    int16_t init_result;
    uint8_t mqtt_stage;
    uint8_t mqtt_sdk_state;
    uint32_t stage_since_ms;
    uint32_t stage_age_ms;
    uint32_t ipv4;
    uint32_t mqtt_attempts;
    uint32_t polls;
    uint32_t last_poll_ms;
    uint32_t broker_ipv4;
    unsigned phase : 3;
} WifiTransportStatus;

#endif
