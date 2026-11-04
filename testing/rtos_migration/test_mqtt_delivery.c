/* Executes the production kernel-owner pump against deterministic lwIP/radio
 * doubles. Real MBF/priority copying is checked by test_connected_bridge.c. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "app_program/network_bridge/network_bridge.h"
#include "app_program/network_bridge/delivery_policy.h"
#define RN_BRIDGE_DEPENDENCIES_PROVIDED 1
#define LWIP_NUM_SYS_TIMEOUT_INTERNAL 7
#define MEMP_NUM_SYS_TIMEOUT 11
#define MQTT_REQ_MAX_IN_FLIGHT 16
#define MQTT_OUTPUT_RINGBUF_SIZE 1024
#define MQTT_DATA_FLAG_LAST 1
#define IPADDR_TYPE_V4 0
#define IP_SET_TYPE(p, t) ((void)(p))
#define ip_2_ip4(p) (p)
#define ip4_addr_get_u32(p) (*(p))
typedef uint8_t u8_t, UB;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int8_t err_t;
typedef uint32_t ip_addr_t;
typedef int mqtt_connection_status_t;
enum { ERR_OK = 0, ERR_MEM = -1, ERR_ABRT = -13, MQTT_CONNECT_ACCEPTED = 0 };
struct altcp_pcb {
    unsigned free;
    int nagle;
    uint16_t snd_queuelen, rto;
    uint8_t nrtx;
};
static struct {
    struct {
        uint16_t used, max, avail, err;
    } mem;
} lwip_stats;
struct mqtt_request_t {
    struct mqtt_request_t *next;
};
typedef struct {
    struct altcp_pcb *conn;
    struct mqtt_request_t req_list[16];
    struct {
        uint16_t get, put;
    } output;
    uint8_t rx_buffer[1];
    bool connected;
} mqtt_client_t;
struct mqtt_connect_client_info_t {
    const char *client_id, *will_topic, *will_msg;
    unsigned keep_alive, will_msg_len;
};
typedef struct {
    int link_status;
    uint32_t ip_addr;
} T_CYW43_UTK_STATUS;
static uint32_t now = 10000;
static mqtt_client_t client;
static struct altcp_pcb tcp = {.free = 11680};
static unsigned free_slots = 16, successes, drops, takes;
static uint8_t queued_topic;
static bool queued, allow_seen, radio_up = true;
static int8_t publish_error;
static RnStatus copied_status;
static void (*complete)(void *, err_t);
static uint32_t sys_now(void) {
    return now;
}
static void tm_printf(UB *format, ...) {
    (void)format;
}
static bool mqtt_client_is_connected(mqtt_client_t *c) {
    return c->connected;
}
static mqtt_client_t *mqtt_client_new(void) {
    return &client;
}
static unsigned altcp_sndbuf(struct altcp_pcb *p) {
    return p->free;
}
static void altcp_nagle_disable(struct altcp_pcb *p) {
    p->nagle = 1;
}
static int altcp_nagle_disabled(struct altcp_pcb *p) {
    return p->nagle;
}
static void mqtt_disconnect(mqtt_client_t *c) {
    c->connected = false;
}
static void mqtt_set_inpub_callback(mqtt_client_t *c, void (*topic)(void *, const char *, uint32_t),
                                    void (*data)(void *, const u8_t *, u16_t, u8_t), void *arg) {
    (void)c;
    (void)topic;
    (void)data;
    (void)arg;
}
static err_t mqtt_subscribe(mqtt_client_t *c, const char *t, unsigned qos,
                            void (*cb)(void *, err_t), void *arg) {
    (void)c;
    (void)t;
    (void)qos;
    (void)cb;
    (void)arg;
    return 0;
}
static err_t mqtt_client_connect(mqtt_client_t *c, const ip_addr_t *ip, unsigned port,
                                 void (*cb)(mqtt_client_t *, void *, mqtt_connection_status_t),
                                 void *arg, const struct mqtt_connect_client_info_t *info) {
    (void)c;
    (void)ip;
    (void)port;
    (void)cb;
    (void)arg;
    assert(info->keep_alive == 5);
    return 0;
}
static err_t mqtt_publish(mqtt_client_t *c, const char *topic, const uint8_t *data, unsigned len,
                          unsigned qos, unsigned retain, void (*cb)(void *, err_t), void *arg) {
    (void)c;
    (void)topic;
    (void)data;
    (void)len;
    (void)qos;
    (void)retain;
    (void)arg;
    complete = cb;
    return publish_error;
}
static void cyw43_utk_get_status(T_CYW43_UTK_STATUS *s) {
    *s = (T_CYW43_UTK_STATUS){radio_up ? 3 : 0, 123};
}
static bool utk_network_settings_available(void) {
    return true;
}
static const char *utk_network_broker(void) {
    return "192.168.1.2";
}
static const char *utk_network_client_id(void) {
    return "test";
}
static int ip4addr_aton(const char *s, ip_addr_t *out) {
    (void)s;
    *out = 42;
    return 1;
}
bool rn_ready(void) {
    return true;
}
bool rn_rx_push(const uint8_t *d, uint8_t n, uint32_t total, uint32_t ms) {
    (void)d;
    (void)n;
    (void)total;
    (void)ms;
    return true;
}
void rn_publish_result(bool ok) {
    if (ok)
        ++successes;
    else
        ++drops;
}
void rn_status_update(const RnStatus *s) {
    copied_status = *s;
}
bool rn_tx_take_budget(uint8_t *topic, uint8_t data[64], uint8_t *len, uint32_t ms,
                       bool telemetry) {
    (void)ms;
    ++takes;
    allow_seen = telemetry;
    if (!queued || (!telemetry && queued_topic == RN_TLM))
        return false;
    *topic = queued_topic;
    data[0] = 1;
    *len = 1;
    queued = false;
    return true;
}
#include "lib/liblwip/sysdepend/pico_rp2040/mqtt_robot_bridge.inc"
static void capacity(unsigned n) {
    free_slots = n;
    for (unsigned i = 0; i < 16; ++i)
        client.req_list[i].next = i < n ? &client.req_list[i] : NULL;
}
static void enqueue(uint8_t topic) {
    queued = true;
    queued_topic = topic;
}
int main(void) {
    lwip_stats.mem.used=4096;lwip_stats.mem.max=8192;lwip_stats.mem.avail=16384;lwip_stats.mem.err=7;
    tcp.rto=32767;tcp.snd_queuelen=16;tcp.nrtx=2;
    client.conn = &tcp;
    client.connected = true;
    robot_client = &client;
    capacity(16);
    robot_connected(&client, NULL, MQTT_CONNECT_ACCEPTED);
    assert(tcp.nagle && robot_sub_pending);
    robot_subscribe_done(NULL, ERR_OK);
    capacity(2);
    enqueue(RN_TLM);
    mqtt_utk_start();
    assert(queued && !allow_seen && !successes && copied_status.telemetry_wait);
    enqueue(RN_HB);
    mqtt_utk_start();
    assert(!queued && successes == 1);
    capacity(16);
    client.output.get = 950;
    client.output.put = 26; /*100 occupied, wrapped.*/
    enqueue(RN_TLM);
    mqtt_utk_start();
    assert(successes == 2 && allow_seen);
    assert(copied_status.ring_used == 100 && copied_status.request_free == 16 &&
           copied_status.nagle_off);
    assert(copied_status.tcp_rto_ms==16383500&&copied_status.heap_capacity==16384&&
           copied_status.heap_errors==7&&copied_status.tcp_retries==2);
    complete(NULL, ERR_OK);
    assert(robot_complete_ms == now);
    publish_error = ERR_MEM;
    enqueue(RN_TLM);
    mqtt_utk_start();
    assert(!robot_pending && drops == 1 && copied_status.publish_mem == 1);
    /* A failed stream publish cannot strand the next priority packet. */
    publish_error = ERR_OK;
    enqueue(RN_ACK);
    mqtt_utk_start();
    assert(successes == 3);
    publish_error = ERR_MEM;
    enqueue(RN_HB);
    mqtt_utk_start();
    assert(robot_pending);
    unsigned before = drops;
    now += 2001;
    mqtt_utk_start();
    assert(!robot_pending && drops == before + 1); /* Bounded retry, no stale heartbeat. */
    enqueue(RN_ACK);
    mqtt_utk_start();
    assert(robot_pending);
    radio_up = false;
    mqtt_utk_start();
    assert(!robot_pending && drops == before + 2);
    assert(!(copied_status.flags & RN_CONNECTED));
    assert(takes > 0 && free_slots == 16);
    puts(
        "PASS production MQTT owner: reserved headroom, wrapped ring, Nagle off, QoS0 ACK activity, ERR_MEM priority, expiry and disconnect cancellation");
    return 0;
}
