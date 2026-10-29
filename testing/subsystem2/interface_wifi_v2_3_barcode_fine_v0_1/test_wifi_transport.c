/* Host harness: compiles the real worker source against boundary-only SDK stubs. */
#define BUDDY_WIFI_ENABLED 1
#define BUDDY_WIFI_LOCAL_CONFIG 0
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "cyw43.h"
#include "lwip/apps/mqtt_priv.h"
#include "wifi_transport.h"
#include "wifi_mqtt_diag.h"

static uint32_t fake_now_ms;
static int fake_init_result, fake_link_status=CYW43_LINK_UP;
static int fake_join_result, fake_connect_result, fake_subscribe_result;
static uint8_t fake_conn_state=1u;
static int fake_new_fails, fake_accept_on_poll;
static unsigned fake_init_calls, fake_poll_calls, fake_sleep_calls;
static jmp_buf worker_exit;
static int worker_exit_armed;
static spin_lock_t fake_lock;
static mqtt_client_t fake_client;
static mqtt_connection_cb_t fake_connection_callback;
static mqtt_request_cb_t fake_subscribe_callback;
static void *fake_subscribe_arg;
cyw43_t cyw43_state;

unsigned spin_lock_claim_unused(bool required) { (void)required; return 0u; }
spin_lock_t *spin_lock_instance(unsigned n) { (void)n; return &fake_lock; }
uint32_t spin_lock_blocking(spin_lock_t *lock) { (void)lock; return 0u; }
void spin_unlock(spin_lock_t *lock, uint32_t irq_state) { (void)lock; (void)irq_state; }
void multicore_lockout_victim_init(void) {}
bool multicore_lockout_victim_is_initialized(unsigned core) { (void)core; return true; }
bool multicore_lockout_start_timeout_us(uint64_t us) { (void)us; return true; }
void multicore_lockout_end_blocking(void) {}
void multicore_launch_core1(void (*entry)(void)) { (void)entry; }
absolute_time_t get_absolute_time(void) { return (absolute_time_t)fake_now_ms*1000u; }
uint32_t to_ms_since_boot(absolute_time_t time) { return (uint32_t)(time/1000u); }
uint32_t time_us_32(void) { return fake_now_ms*1000u; }
void sleep_ms(uint32_t ms) {
    fake_sleep_calls++;
    fake_now_ms += ms;
    if (worker_exit_armed && wifi_transport_status().last_error != WIFI_ERROR_NONE)
        longjmp(worker_exit, 1);
}
int cyw43_arch_init(void) { fake_init_calls++; return fake_init_result; }
void cyw43_arch_enable_sta_mode(void) {}
void cyw43_arch_poll(void) {
    fake_poll_calls++;
    if (fake_accept_on_poll && fake_connection_callback) {
        fake_accept_on_poll=0;
        fake_client.conn_state=3u;
        fake_connection_callback(&fake_client, NULL, MQTT_CONNECT_ACCEPTED);
    }
}
int cyw43_arch_wifi_connect_async(const char *ssid,const char *pw,uint32_t auth) {
    (void)ssid; (void)pw; (void)auth; return fake_join_result;
}
int cyw43_wifi_pm(cyw43_t *state,uint32_t mode) { (void)state; (void)mode; return 0; }
int cyw43_wifi_get_pm(cyw43_t *state,uint32_t *mode) { (void)state; *mode=0; return 0; }
int cyw43_tcpip_link_status(cyw43_t *state,int itf) { (void)state; (void)itf; return fake_link_status; }
int ip4addr_aton(const char *text,ip4_addr_t *address) {
    unsigned a,b,c,d; char tail;
    if (sscanf(text,"%u.%u.%u.%u%c",&a,&b,&c,&d,&tail)!=4 || a>255 || b>255 || c>255 || d>255) return 0;
    address->addr=(a<<24)|(b<<16)|(c<<8)|d; return 1;
}
uint32_t lwip_ntohl(uint32_t value) { return value; }
const ip4_addr_t *netif_ip4_addr(const struct netif *netif) { return &netif->ip_addr; }
mqtt_client_t *mqtt_client_new(void) { return fake_new_fails ? NULL : &fake_client; }
err_t mqtt_client_connect(mqtt_client_t *client,const ip_addr_t *addr,uint16_t port,
                          mqtt_connection_cb_t cb,void *arg,
                          const struct mqtt_connect_client_info_t *info) {
    (void)addr; (void)port; (void)arg; (void)info;
    fake_connection_callback=cb;
    client->conn_state=fake_conn_state;
    return fake_connect_result;
}
void mqtt_disconnect(mqtt_client_t *client) { client->conn_state=0u; }
uint8_t mqtt_client_is_connected(mqtt_client_t *client) { return client->conn_state==3u; }
void mqtt_set_inpub_callback(mqtt_client_t *client,mqtt_incoming_publish_cb_t pub,
                             mqtt_incoming_data_cb_t data,void *arg) {
    (void)client; (void)pub; (void)data; (void)arg;
}
err_t mqtt_subscribe(mqtt_client_t *client,const char *topic,uint8_t qos,
                     mqtt_request_cb_t cb,void *arg) {
    (void)client; (void)topic; (void)qos;
    fake_subscribe_callback=cb; fake_subscribe_arg=arg; return fake_subscribe_result;
}
err_t mqtt_publish(mqtt_client_t *client,const char *topic,const char *data,uint16_t len,
                   uint8_t qos,uint8_t retain,mqtt_request_cb_t cb,void *arg) {
    (void)client; (void)topic; (void)data; (void)len; (void)qos; (void)retain; (void)cb; (void)arg;
    return ERR_OK;
}

#include "wifi_transport.c"

static void reset_fixture(void) {
    memset(&g_status,0,sizeof(g_status));
    memset(&g_network_config,0,sizeof(g_network_config));
    memset(&fake_client,0,sizeof(fake_client));
    memset(&cyw43_state,0,sizeof(cyw43_state));
    strcpy(g_network_config.ssid,"test-network");
    strcpy(g_network_config.broker,"192.0.2.2");
    strcpy(g_network_config.client_id,"host-test");
    g_config_is_valid=true; g_worker_launched=false; g_initialized=true;
    g_queue_lock=&fake_lock; g_mqtt=NULL; g_broker=(ip_addr_t){0};
    g_wifi_started=g_connecting=g_mqtt_connected=g_subscribing=false;
    g_wifi_joining=g_wifi_was_up=g_rx_is_cmd=false;
    g_rx_staging_len=0; g_rx_original_len=0; g_retry_at=g_wifi_retry_at=0;
    g_wifi_join_started=g_connect_started=g_subscribe_started=0;
    g_attempts=g_wifi_attempts=g_last_poll_us=g_poll_gap_max_us=0;
    g_tx_count=g_rx_head=g_rx_tail=g_rx_count=0; g_pending_valid=false;
    fake_now_ms=0; fake_init_result=0; fake_link_status=CYW43_LINK_UP;
    fake_join_result=fake_connect_result=fake_subscribe_result=ERR_OK;
    fake_conn_state=1u; fake_new_fails=0; fake_accept_on_poll=0;
    fake_init_calls=fake_poll_calls=fake_sleep_calls=0;
    fake_connection_callback=NULL; fake_subscribe_callback=NULL; fake_subscribe_arg=NULL;
    worker_exit_armed=0;
    g_status.mqtt_result=INT16_MIN; g_status.connect_api_result=INT16_MIN;
    g_status.subscribe_result=INT16_MIN; g_status.init_result=INT16_MIN;
}

static void test_connect_api_result_is_not_connack(void) {
    reset_fixture(); begin_connect(0u);
    assert(g_status.connect_api_result==ERR_OK);
    assert(g_status.mqtt_result==INT16_MIN);
    assert(g_connecting && !g_mqtt_connected);
    fake_client.conn_state=3u;
    fake_connection_callback(&fake_client,NULL,MQTT_CONNECT_ACCEPTED);
    assert(g_status.mqtt_result==MQTT_CONNECT_ACCEPTED);
    assert(g_status.connected && !g_status.subscribed && g_subscribing);
    assert(fake_subscribe_callback!=NULL);
    fake_subscribe_callback(fake_subscribe_arg,ERR_OK);
    assert(g_status.subscribed && g_status.connected && !g_subscribing);
    assert(g_status.subscribe_result==ERR_OK);
}

static void test_connect_setup_failures_are_explicit(void) {
    reset_fixture(); strcpy(g_network_config.broker,"bad-address");
    begin_connect(0u);
    assert(g_status.last_error==WIFI_ERROR_ADDRESS);
    assert(g_status.connect_api_result==INT16_MIN);
    reset_fixture(); fake_new_fails=1;
    begin_connect(0u);
    assert(g_status.last_error==WIFI_ERROR_MEMORY);
    assert(g_status.connect_api_result==INT16_MIN);
}

static void run_worker_until_error(void) {
    worker_exit_armed=1;
    if (setjmp(worker_exit)==0) network_core();
    worker_exit_armed=0;
}

static void test_tcp_and_connack_timeouts_are_distinct(void) {
    reset_fixture(); fake_conn_state=1u;
    run_worker_until_error();
    assert(g_status.last_error==WIFI_ERROR_TCP_TIMEOUT);
    assert(fake_sleep_calls>1000u);
    reset_fixture(); fake_conn_state=2u;
    run_worker_until_error();
    assert(g_status.last_error==WIFI_ERROR_CONNACK_TIMEOUT);
}

static void test_missing_suback_times_out_and_retries(void) {
    reset_fixture(); fake_conn_state=2u; fake_accept_on_poll=1;
    run_worker_until_error();
    assert(g_status.last_error==WIFI_ERROR_SUBACK_TIMEOUT);
    assert(!g_status.connected && !g_status.subscribed);
    assert(g_status.reconnects>0u);
    assert(fake_subscribe_callback!=NULL);
}

static void test_init_failure_is_visible_and_delayed(void) {
    reset_fixture(); fake_init_result=-7;
    run_worker_until_error();
    assert(fake_init_calls==1u);
    assert(fake_sleep_calls==1u);
    assert(g_status.init_result==-7);
    assert(g_status.last_error==WIFI_ERROR_INIT);
    assert(g_status.mqtt_stage==WIFI_STAGE_INIT_FAILED);
    assert(!g_status.ready);
}

int main(void) {
    test_connect_api_result_is_not_connack();
    test_connect_setup_failures_are_explicit();
    test_tcp_and_connack_timeouts_are_distinct();
    test_missing_suback_times_out_and_retries();
    test_init_failure_is_visible_and_delayed();
    puts("PASS real wifi_transport.c host harness: API-vs-CONNACK result, CONNACK/SUBACK callbacks, setup failures, TCP/CONNACK/SUBACK timeout paths, init-failure delay");
    return 0;
}
