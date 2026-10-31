#ifndef WIFI_MQTT_DIAG_H
#define WIFI_MQTT_DIAG_H
#include <stdbool.h>
#include <stdint.h>
enum {
    WIFI_STAGE_UNCONFIGURED = 0,
    WIFI_STAGE_INIT,
    WIFI_STAGE_WIFI,
    WIFI_STAGE_TCP,
    WIFI_STAGE_CONNACK,
    WIFI_STAGE_SUBACK,
    WIFI_STAGE_ONLINE,
    WIFI_STAGE_RETRY,
    WIFI_STAGE_INIT_FAILED
};
enum {
    WIFI_ERROR_NONE = 0,
    WIFI_ERROR_INIT = -1001,
    WIFI_ERROR_ADDRESS = -1002,
    WIFI_ERROR_MEMORY = -1003,
    WIFI_ERROR_TCP_TIMEOUT = -1004,
    WIFI_ERROR_CONNACK_TIMEOUT = -1005,
    WIFI_ERROR_SUBACK_TIMEOUT = -1006,
    WIFI_ERROR_SUBSCRIBE = -1007
};
static inline const char *wifi_mqtt_stage_name(unsigned s) {
    static const char *const names[] = {"unconfigured", "initialising",  "wifi_or_dhcp",
                                        "tcp_connect",  "await_connack", "await_suback",
                                        "online",       "retry_wait",    "init_failed"};
    return s < sizeof(names) / sizeof(names[0]) ? names[s] : "unknown";
}
static inline bool wifi_mqtt_due(uint32_t now, uint32_t deadline) {
    return (int32_t)(now - deadline) >= 0;
}
#endif
