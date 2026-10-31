#ifndef TEST_LWIP_MQTT_H
#define TEST_LWIP_MQTT_H
#include <stdint.h>
#include "lwip/err.h"
#include "lwip/ip_addr.h"
typedef struct mqtt_client mqtt_client_t;
typedef enum {
    MQTT_CONNECT_ACCEPTED = 0,
    MQTT_CONNECT_REFUSED_PROTOCOL_VERSION = 1,
    MQTT_CONNECT_REFUSED_IDENTIFIER = 2,
    MQTT_CONNECT_REFUSED_SERVER = 3,
    MQTT_CONNECT_REFUSED_USERNAME_PASS = 4,
    MQTT_CONNECT_REFUSED_NOT_AUTHORIZED = 5,
    MQTT_CONNECT_DISCONNECTED = 256
} mqtt_connection_status_t;
typedef void (*mqtt_connection_cb_t)(mqtt_client_t *, void *, mqtt_connection_status_t);
typedef void (*mqtt_request_cb_t)(void *, err_t);
typedef void (*mqtt_incoming_publish_cb_t)(void *, const char *, uint32_t);
typedef void (*mqtt_incoming_data_cb_t)(void *, const uint8_t *, uint16_t, uint8_t);
#define MQTT_DATA_FLAG_LAST 1u
struct mqtt_connect_client_info_t {
    const char *client_id;
    uint16_t keep_alive;
    const char *will_topic;
    const char *will_msg;
    uint8_t will_msg_len, will_qos, will_retain;
};
mqtt_client_t *mqtt_client_new(void);
err_t mqtt_client_connect(mqtt_client_t *, const ip_addr_t *, uint16_t, mqtt_connection_cb_t,
                          void *, const struct mqtt_connect_client_info_t *);
void mqtt_disconnect(mqtt_client_t *);
uint8_t mqtt_client_is_connected(mqtt_client_t *);
void mqtt_set_inpub_callback(mqtt_client_t *, mqtt_incoming_publish_cb_t, mqtt_incoming_data_cb_t,
                             void *);
err_t mqtt_subscribe(mqtt_client_t *, const char *, uint8_t, mqtt_request_cb_t, void *);
err_t mqtt_publish(mqtt_client_t *, const char *, const char *, uint16_t, uint8_t, uint8_t,
                   mqtt_request_cb_t, void *);
#endif
