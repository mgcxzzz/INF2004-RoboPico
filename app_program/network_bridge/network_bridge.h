#ifndef RTOS_NETWORK_BRIDGE_H
#define RTOS_NETWORK_BRIDGE_H
#include <stdint.h>
#include <stdbool.h>

enum { RN_PACKET_MAX = 64, RN_RX_MAX = 8, RN_TLM = 0, RN_ACK = 1, RN_HB = 2 };
enum { RN_CONFIGURED = 1, RN_CONNECTED = 2, RN_SUBSCRIBED = 4 };
typedef struct {
    uint32_t polls, last_poll_ms, ipv4, broker_ipv4, connects;
    uint32_t published, received, tx_dropped, rx_dropped;
    int16_t mqtt_result;
    int8_t link;
    uint8_t flags;
} RnStatus;
/* No lwIP, radio or kernel typedefs cross this header. All messages are copied. */
int32_t rn_init(void);
bool rn_ready(void);
bool rn_tx_push(uint8_t topic, const uint8_t *data, uint8_t len, uint32_t now);
bool rn_tx_take(uint8_t *topic, uint8_t data[RN_PACKET_MAX], uint8_t *len, uint32_t now);
bool rn_rx_push(const uint8_t *data, uint8_t copied, uint32_t original, uint32_t now);
void rn_publish_result(bool ok);
void rn_status_update(const RnStatus *status);
RnStatus rn_status(void);
void rn_app_task(int32_t code, void *exinf);
void rn_report(void);
#endif
