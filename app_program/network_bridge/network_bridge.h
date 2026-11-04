#ifndef RTOS_NETWORK_BRIDGE_H
#define RTOS_NETWORK_BRIDGE_H
#include <stdint.h>
#include <stdbool.h>

enum { RN_PACKET_MAX = 64, RN_RX_MAX = 8, RN_TLM = 0, RN_ACK = 1, RN_HB = 2 };
enum { RN_CONFIGURED = 1, RN_CONNECTED = 2, RN_SUBSCRIBED = 4 };
/* Incoming MQTT PUBLISH must be non-retained. A broker replay is never a
 * fresh motion request; QoS/DUP bits do not change this condition. */
static inline bool rn_live_command(bool command_topic, uint8_t fixed_header) {
    return command_topic && (fixed_header & 0xf1U) == 0x30U;
}
enum { RN_URGENT_STOP = 1, RN_URGENT_ESTOP = 2, RN_URGENT_PAUSE = 4, RN_RX_BYTES = 25 };
typedef struct {
    uint32_t polls, last_poll_ms, ipv4, broker_ipv4, connects;
    uint32_t published, received, tx_dropped, rx_dropped;
    uint32_t tx_coalesced, publish_mem, publish_error, telemetry_wait;
    uint32_t last_admit_ms, last_complete_ms;
    unsigned ring_used : 11, tcp_free : 14;
    uint16_t heap_errors;
    unsigned tcp_rto_ms : 24;
    unsigned heap_used : 15, heap_max : 15, heap_capacity : 15, tcp_queue : 6;
    unsigned tcp_retries : 4;
    int16_t mqtt_result;
    int8_t publish_result;
    int8_t link;
    uint8_t flags;
    unsigned request_free : 5, nagle_off : 1;
} RnStatus;
/* No lwIP, radio or kernel typedefs cross this header. All messages are copied. */
int32_t rn_init(void);
bool rn_ready(void);
bool rn_tx_push(uint8_t topic, const uint8_t *data, uint8_t len, uint32_t now);
bool rn_tx_take(uint8_t *topic, uint8_t data[RN_PACKET_MAX], uint8_t *len, uint32_t now);
bool rn_tx_take_budget(uint8_t *topic, uint8_t data[RN_PACKET_MAX], uint8_t *len, uint32_t now,
                       bool telemetry);
bool rn_rx_push(const uint8_t *data, uint8_t copied, uint32_t original, uint32_t now);
void rn_publish_result(bool ok);
void rn_status_update(const RnStatus *status);
RnStatus rn_status(void);
uint32_t rn_mapping_snapshot(void);
uint8_t rn_next_sequence(unsigned source);
void rn_app_task(int32_t code, void *exinf);
void rn_report(void);
#ifdef APP_RTOS_IMU
/* Kernel Wi-Fi owner publishes elapsed service wall times; copied only,
 * never calls network APIs from the diagnostic/application tasks. */
void rn_service_sample(uint32_t radio_us, uint32_t timers_us, uint32_t mqtt_us);
#endif
void rn_command_result(uint8_t code, uint8_t result);
uint32_t rn_barrier(void);
unsigned rn_take_urgent(void);
bool rn_take_pause(void);
#if defined(APP_RTOS_LINE) && defined(APP_RTOS_NETWORK)
int32_t rn_drive_init(void);
bool rn_drive_enqueue(const uint8_t message[RN_RX_BYTES]);
uint32_t rn_app_last_ms(void);
void rn_drive_process(void *feedback); /* Control-core owner only. */
bool rn_drive_paused(void);
void rn_drive_cancel(void);
void rn_drive_begin(void);
bool rn_drive_online(void);
void rn_drive_telemetry(uint32_t now);
void rn_drive_set_telemetry(bool enabled);
bool rn_drive_telemetry_enabled(void);
#endif
#endif
