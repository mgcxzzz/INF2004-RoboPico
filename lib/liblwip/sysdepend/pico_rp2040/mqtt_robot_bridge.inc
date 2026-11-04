/* Robot network checkpoint. Included only by mqtt_utk.c and executed solely
 * in the kernel's CYW43 task. Application tasks exchange copied bounded MBFs. */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/sys.h"
#include "mqtt_utk.h"
#include "../../../libwifi/sysdepend/pico_rp2040/cyw43_utk.h"
#include "../../../libwifi/sysdepend/pico_rp2040/network_settings.h"
#include "../../../../app_program/network_bridge/network_bridge.h"
#include <tm/tmonitor.h>

#if MEMP_NUM_SYS_TIMEOUT < (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 4)
#error "Reserve MQTT/application timer slots before enabling the network bridge"
#endif
static mqtt_client_t *robot_client;
static ip_addr_t robot_broker;
static uint32_t robot_retry, robot_connect_ms, robot_sub_ms, robot_polls, robot_connects;
static int16_t robot_result = INT16_MIN;
static bool robot_connecting, robot_subscribed, robot_sub_pending;
static bool robot_command_topic, robot_pending;
static uint8_t robot_rx[RN_RX_MAX], robot_copied, robot_packet[RN_PACKET_MAX];
static uint32_t robot_total, robot_received, robot_packet_ms;
static uint8_t robot_topic, robot_length;

static void robot_receive_topic(void *arg, const char *topic, u32_t length) {
    (void)arg;
    robot_command_topic = topic && strcmp(topic, "is28/cmd") == 0;
    robot_copied = 0;
    robot_received = 0;
    robot_total = length;
}
static void robot_receive_data(void *arg, const u8_t *data, u16_t length, u8_t flags) {
    (void)arg;
    if (!robot_command_topic)
        return;
    robot_received += length;
    for (unsigned n = 0; data && n < length && robot_copied < RN_RX_MAX; ++n)
        robot_rx[robot_copied++] = data[n];
    if (flags & MQTT_DATA_FLAG_LAST) {
        (void)rn_rx_push(robot_rx, robot_copied,
                         robot_received == robot_total ? robot_total : UINT32_MAX, sys_now());
        robot_command_topic = false;
    }
}
static void robot_subscribe_done(void *arg, err_t result) {
    (void)arg;
    robot_sub_pending = false;
    robot_subscribed = result == ERR_OK && mqtt_client_is_connected(robot_client);
    if (robot_subscribed)
        tm_printf((UB *)"mqtt: Operational; subscribed to is28/cmd\n");
    else
        robot_retry = sys_now() + 3000U;
}
static void robot_subscribe(void) {
    robot_sub_ms = sys_now();
    err_t result = mqtt_subscribe(robot_client, "is28/cmd", 0, robot_subscribe_done, NULL);
    robot_sub_pending = result == ERR_OK;
    if (result != ERR_OK)
        robot_retry = sys_now() + 3000U;
}
/* mqtt_connection_cb_t's first argument is mqtt_client_t*, not a generic context. */
static void robot_connected(mqtt_client_t *client, void *arg, mqtt_connection_status_t result) {
    (void)client;
    (void)arg;
    robot_result = (int16_t)result;
    robot_connecting = robot_subscribed = robot_sub_pending = false;
    if (result == MQTT_CONNECT_ACCEPTED) {
        ++robot_connects;
        mqtt_set_inpub_callback(robot_client, robot_receive_topic, robot_receive_data, NULL);
        robot_subscribe();
    } else
        robot_retry = sys_now() + 3000U;
}
void mqtt_utk_start(void) {
    if (!rn_ready())
        return; /* Startup order: no message-buffer access before creation. */
    uint32_t now = sys_now();
    T_CYW43_UTK_STATUS radio;
    cyw43_utk_get_status(&radio); /* Same kernel owner/core, never a subsystem lwIP call. */
    bool configured = utk_network_settings_available();
    if (configured) {
        (void)ip4addr_aton(utk_network_broker(), ip_2_ip4(&robot_broker));
        IP_SET_TYPE(&robot_broker, IPADDR_TYPE_V4);
    }
    if (configured && radio.link_status == 3) {
        if (!robot_client)
            robot_client = mqtt_client_new();
        if (robot_client && !mqtt_client_is_connected(robot_client)) {
            if (robot_connecting && now - robot_connect_ms >= 10000U) {
                mqtt_disconnect(robot_client);
                robot_connecting = false;
                robot_retry = now + 3000U;
            }
            if (!robot_connecting && (int32_t)(now - robot_retry) >= 0 &&
                ip4addr_aton(utk_network_broker(), ip_2_ip4(&robot_broker))) {
                IP_SET_TYPE(&robot_broker, IPADDR_TYPE_V4);
                static const char offline = 0;
                struct mqtt_connect_client_info_t info = {0};
                info.client_id = utk_network_client_id();
                info.keep_alive = 5;
                info.will_topic = "is28/lwt";
                info.will_msg = &offline;
                info.will_msg_len = 1;
                robot_connect_ms = now;
                robot_subscribed = false;
                err_t result = mqtt_client_connect(robot_client, &robot_broker, 1883,
                                                   robot_connected, NULL, &info);
                robot_connecting = result == ERR_OK;
                if (result != ERR_OK)
                    robot_retry = now + 3000U;
                tm_printf((UB *)"mqtt: connection requested broker=%s result=%d\n",
                          utk_network_broker(), result);
            }
        } else if (robot_client && !robot_subscribed) {
            if (robot_sub_pending && now - robot_sub_ms >= 5000U) {
                mqtt_disconnect(robot_client);
                robot_sub_pending = false;
                robot_retry = now + 3000U;
            } else if (!robot_sub_pending && (int32_t)(now - robot_retry) >= 0)
                robot_subscribe();
        }
    } else if (robot_client) {
        if (mqtt_client_is_connected(robot_client) || robot_connecting)
            mqtt_disconnect(robot_client);
        robot_connecting = robot_subscribed = robot_sub_pending = false;
        if (robot_pending)
            rn_publish_result(false);
        robot_pending = false;
    }
    bool connected = robot_client && mqtt_client_is_connected(robot_client);
    if (connected && robot_subscribed) {
        for (unsigned n = 0; n < 4; ++n) {
            if (!robot_pending) {
                if (!rn_tx_take(&robot_topic, robot_packet, &robot_length, now))
                    break;
                robot_packet_ms = now;
                robot_pending = true;
            }
            if (now - robot_packet_ms > (robot_topic == RN_TLM ? 250U : 2000U)) {
                rn_publish_result(false);
                robot_pending = false;
                continue;
            }
            const char *topic = robot_topic == RN_ACK  ? "is28/ack"
                                : robot_topic == RN_HB ? "is28/hb"
                                                       : "is28/tlm";
            err_t result =
                mqtt_publish(robot_client, topic, robot_packet, robot_length, 0, 0, NULL, NULL);
            if (result == ERR_MEM)
                break;
            rn_publish_result(result == ERR_OK);
            robot_pending = false;
        }
    }
    RnStatus status = {.polls = ++robot_polls,
                       .last_poll_ms = now,
                       .ipv4 = radio.link_status == 3 ? radio.ip_addr : 0,
                       .broker_ipv4 = configured ? ip4_addr_get_u32(ip_2_ip4(&robot_broker)) : 0,
                       .connects = robot_connects,
                       .mqtt_result = robot_result,
                       .link = (int8_t)radio.link_status,
                       .flags = (configured ? RN_CONFIGURED : 0) | (connected ? RN_CONNECTED : 0) |
                                (robot_subscribed ? RN_SUBSCRIBED : 0)};
    rn_status_update(&status);
}
