/* MQTT remains in the existing core0 micro T-Kernel CYW43 owner. The sole
 * core1 drive task is the only producer/consumer of controller work; all
 * crossing data is copied through drive_bridge's bounded MBFs. */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/sys.h"
#include "../../../libwifi/sysdepend/pico_rp2040/cyw43_utk.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>

#include "../../../../app_program/subsystem1/b1_net.h"
#include "../../../../app_program/subsystem1/b1_proto.h"
#include "../../../../app_program/subsystem1/drive_bridge.h"
#include "../../../../app_program/subsystem2/controller/wifi_mqtt_diag.h"

#if MEMP_NUM_SYS_TIMEOUT < (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 4)
#error "MQTT requires the reserved lwIP application timeout slots"
#endif

/* Runtime settings are loaded by the core0 CYW43 owner from the validated
 * WCF1 page. Keep this interface small to avoid pulling libc helper types
 * into the micro T-Kernel/lwIP compilation unit. */
extern bool utk_network_settings_available(void);
extern const char *utk_network_broker(void);
extern const char *utk_network_client_id(void);

#define MQTT_UTK_GAP_MS       500U
#define MQTT_UTK_LWT_TOPIC    "is28/lwt"
#define MQTT_UTK_BROKER_PORT  1883U
#define MQTT_UTK_COPY_MAX     8U

static mqtt_client_t *client;
static ip_addr_t broker_addr;
static u8_t net_state=B1_NET_DISCONNECTED;
static u8_t g_subscribed;
static u8_t g_rx_is_cmd;
static u8_t g_rx_buf[MQTT_UTK_COPY_MAX];
static u8_t g_rx_copied;
static u32_t g_rx_total;
static u8_t g_pend_topic;
static u8_t g_pend_body[DRIVE_BRIDGE_PACKET_MAX];
static u32_t g_pend_len;
static u8_t g_pend_valid;
static u32_t g_retry_at;
static u32_t g_connect_started;
static u32_t g_attempts;
static u32_t g_connections;
static u32_t g_polls;
static u32_t g_last_poll;
static u32_t g_max_poll_gap_us;
static u32_t g_stage_since;
static u8_t g_stage=WIFI_STAGE_INIT;
static s16_t g_mqtt_result=INT16_MIN;
static s16_t g_connect_api_result=INT16_MIN;
static s16_t g_subscribe_result=INT16_MIN;
static s16_t g_last_error;

static u32_t retry_delay_ms(u32_t attempts)
{
    if(attempts<=1u)return 1000u;
    if(attempts==2u)return 2000u;
    if(attempts==3u)return 4000u;
    return 8000u;
}

static const char *state_name(u8_t state)
{
    switch(state) {
    case B1_NET_DISCONNECTED:return "disconnected";
    case B1_NET_CONNECTING:return "connecting";
    case B1_NET_OPERATIONAL:return "operational";
    case B1_NET_RECONNECT_WAIT:return "reconnect-wait";
    default:return "unknown";
    }
}

static void set_stage(u8_t stage,u32_t now)
{
    if(stage!=g_stage){g_stage=stage;g_stage_since=now;}
}

static void set_state(u8_t state)
{
    if(state!=net_state) {
        net_state=state;
        tm_printf((UB*)"mqtt: %s\n",(UB*)state_name(state));
    }
}

static void wait_and_retry(u32_t now)
{
    if(g_attempts<UINT32_MAX)++g_attempts;
    g_retry_at=now+retry_delay_ms(g_attempts);
    g_subscribed=0u;
    set_state(B1_NET_RECONNECT_WAIT);
}

static const char *mqtt_topic(u8_t topic)
{
    switch(topic) {
    case WIFI_TOPIC_ACK:return B1_TOPIC_ACK;
    case WIFI_TOPIC_HB:return B1_TOPIC_HB;
    default:return B1_TOPIC_TLM;
    }
}

static void incoming_publish_cb(void *arg,const char *topic,u32_t total_len)
{
    (void)arg;
    g_rx_is_cmd=(u8_t)(topic!=NULL && strcmp(topic,B1_TOPIC_CMD)==0);
    g_rx_copied=0u;g_rx_total=total_len;
}

static void incoming_data_cb(void *arg,const u8_t *data,u16_t len,u8_t flags)
{
    (void)arg;
    if(g_rx_is_cmd && data!=NULL) {
        for(u16_t i=0u;i<len && g_rx_copied<MQTT_UTK_COPY_MAX;++i)
            g_rx_buf[g_rx_copied++]=data[i];
    }
    if((flags&MQTT_DATA_FLAG_LAST)!=0u) {
        if(g_rx_is_cmd) {
            (void)drive_bridge_rx_enqueue(g_rx_buf,g_rx_copied,g_rx_total);
            g_rx_is_cmd=0u;
        }
    }
}

static void subscribe_done_cb(void *arg,err_t result)
{
    u32_t now=sys_now();
    (void)arg;
    g_subscribe_result=(s16_t)result;
    if(result==ERR_OK && client!=NULL && mqtt_client_is_connected(client)) {
        g_subscribed=1u;g_attempts=0u;g_last_error=WIFI_ERROR_NONE;
        tm_printf((UB*)"mqtt: listening on " B1_TOPIC_CMD "\n");
    } else {
        g_subscribed=0u;g_last_error=WIFI_ERROR_SUBSCRIBE;
        tm_printf((UB*)"mqtt: subscribe failed, err=%d\n",(int)result);
        wait_and_retry(now);
    }
}

static void connection_cb(mqtt_client_t *c,void *arg,mqtt_connection_status_t status)
{
    u32_t now=sys_now();err_t result;
    (void)c;(void)arg;
    g_mqtt_result=(s16_t)status;g_subscribed=0u;
    if(status==MQTT_CONNECT_ACCEPTED) {
        if(g_connections<UINT32_MAX)++g_connections;
        set_state(B1_NET_OPERATIONAL);
        mqtt_set_inpub_callback(client,incoming_publish_cb,incoming_data_cb,NULL);
        result=mqtt_subscribe(client,B1_TOPIC_CMD,0u,subscribe_done_cb,NULL);
        g_subscribe_result=(s16_t)result;
        if(result!=ERR_OK) {
            g_last_error=WIFI_ERROR_SUBSCRIBE;
            wait_and_retry(now);
        }
    } else {
        g_last_error=(s16_t)status;
        wait_and_retry(now);
    }
}

static void start_connect(u32_t now)
{
    struct mqtt_connect_client_info_t info;
    static const u8_t offline=0u;
    err_t result;

    mqtt_disconnect(client);
    memset(&info,0,sizeof(info));
    info.client_id=utk_network_client_id();
    info.keep_alive=B1_KEEPALIVE_S;
    info.will_topic=MQTT_UTK_LWT_TOPIC;
    info.will_msg=(const char *)&offline;
    info.will_msg_len=1u;info.will_qos=0u;info.will_retain=0u;
    tm_printf((UB*)"mqtt: connecting to %s ...\n",utk_network_broker());
    result=mqtt_client_connect(client,&broker_addr,MQTT_UTK_BROKER_PORT,
                               connection_cb,NULL,&info);
    g_connect_api_result=(s16_t)result;
    if(result==ERR_OK) {
        g_connect_started=now;g_last_error=WIFI_ERROR_NONE;
        set_state(B1_NET_CONNECTING);
    } else {
        g_last_error=(s16_t)result;wait_and_retry(now);
    }
}

static void pump_publish(void)
{
    for(unsigned sent=0u;sent<B1_PUBLISH_PER_POLL;++sent) {
        err_t result;
        if(!g_pend_valid) {
            if(!drive_bridge_tx_take(&g_pend_topic,g_pend_body,&g_pend_len))break;
            g_pend_valid=1u;
        }
        result=mqtt_publish(client,mqtt_topic(g_pend_topic),(const char*)g_pend_body,
                            (u16_t)g_pend_len,0u,0u,NULL,NULL);
        if(result==ERR_MEM)break;
        drive_bridge_publish_result(result==ERR_OK);
        g_pend_valid=0u;
    }
}

static u32_t host_order_ip(u32_t raw)
{
    return ((raw&0x000000ffu)<<24)|((raw&0x0000ff00u)<<8)|
           ((raw&0x00ff0000u)>>8)|((raw&0xff000000u)>>24);
}

static u32_t configured_broker_ip(void)
{
    ip4_addr_t addr;
    const char *text=utk_network_broker();
    if(text==NULL || !ip4addr_aton(text,&addr))return 0u;
    return host_order_ip(addr.addr);
}

static void update_bridge_status(u32_t now)
{
    T_CYW43_UTK_STATUS radio;
    WifiTransportStatus status={0};
    u8_t stage;
    cyw43_utk_get_status(&radio);
    if(!utk_network_settings_available())stage=WIFI_STAGE_UNCONFIGURED;
    else if(!radio.init_complete)stage=WIFI_STAGE_INIT;
    else if(!radio.ready)stage=WIFI_STAGE_INIT_FAILED;
    else if(radio.link_status!=3)stage=WIFI_STAGE_WIFI;
    else if(g_subscribed)stage=WIFI_STAGE_ONLINE;
    else if(g_mqtt_result==MQTT_CONNECT_ACCEPTED)stage=WIFI_STAGE_SUBACK;
    else if(net_state==B1_NET_CONNECTING)stage=WIFI_STAGE_TCP;
    else stage=WIFI_STAGE_RETRY;
    set_stage(stage,now);

    status.configured=(unsigned)utk_network_settings_available();
    status.ready=radio.ready!=0u;
    status.connected=g_mqtt_result==MQTT_CONNECT_ACCEPTED && net_state==B1_NET_OPERATIONAL;
    status.subscribed=g_subscribed!=0u;
    status.link_status=(int8_t)radio.link_status;
    status.mqtt_result=g_mqtt_result;
    status.connect_api_result=g_connect_api_result;
    status.subscribe_result=g_subscribe_result;
    status.last_error=g_last_error;
    status.init_result=(s16_t)radio.init_result;
    status.mqtt_stage=g_stage;status.mqtt_sdk_state=0u;
    status.stage_since_ms=g_stage_since;status.stage_age_ms=now-g_stage_since;
    status.ipv4=host_order_ip(radio.ip_addr);
    status.broker_ipv4=configured_broker_ip();
    status.mqtt_attempts=g_attempts;status.polls=g_polls;
    status.last_poll_ms=now;status.phase=1u;
    status.reconnects=g_connections;
    status.max_poll_gap_us=g_max_poll_gap_us;
    (void)drive_bridge_status_update(&status);
}

/* Called by cyw43_utk_task() from the sole core0 Wi-Fi/lwIP owner. */
void mqtt_utk_start(void)
{
    u32_t now=sys_now();
    if(!drive_bridge_ready())return;
    if(!utk_network_settings_available()) {
        g_last_error=WIFI_ERROR_NONE;
        update_bridge_status(now);
        return;
    }
    if(client==NULL) {
        client=mqtt_client_new();
        if(client==NULL) {
            g_last_error=WIFI_ERROR_MEMORY;
            update_bridge_status(now);return;
        }
        if(!ip4addr_aton(utk_network_broker(),ip_2_ip4(&broker_addr))) {
            g_last_error=WIFI_ERROR_ADDRESS;
            update_bridge_status(now);return;
        }
        IP_SET_TYPE(&broker_addr,IPADDR_TYPE_V4);
        g_retry_at=now;
    }

    if(g_last_poll!=0u) {
        u32_t gap_ms=now-g_last_poll;
        if(gap_ms>g_max_poll_gap_us/1000u)g_max_poll_gap_us=gap_ms*1000u;
        if(gap_ms>MQTT_UTK_GAP_MS && net_state!=B1_NET_RECONNECT_WAIT) {
            mqtt_disconnect(client);g_attempts=0u;g_retry_at=now;
            g_subscribed=0u;set_state(B1_NET_RECONNECT_WAIT);
        }
    }
    g_last_poll=now;if(g_polls<UINT32_MAX)++g_polls;

    switch(net_state) {
    case B1_NET_DISCONNECTED:
    case B1_NET_RECONNECT_WAIT:
        if((s32_t)(now-g_retry_at)>=0)start_connect(now);
        break;
    case B1_NET_CONNECTING:
        if((now-g_connect_started)>B1_CONNECT_TIMEOUT_MS) {
            g_last_error=WIFI_ERROR_CONNACK_TIMEOUT;mqtt_disconnect(client);wait_and_retry(now);
        }
        break;
    case B1_NET_OPERATIONAL:
        if(!mqtt_client_is_connected(client)) {
            g_last_error=WIFI_ERROR_SUBSCRIBE;wait_and_retry(now);
        } else if(g_subscribed) {
            pump_publish();
        }
        break;
    default:break;
    }
    update_bridge_status(now);
}
