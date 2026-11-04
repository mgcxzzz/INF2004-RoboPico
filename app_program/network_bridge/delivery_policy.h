#ifndef RTOS_DELIVERY_POLICY_H
#define RTOS_DELIVERY_POLICY_H
#include <stdbool.h>
#include <stdint.h>
/* Leave two requests and 128 output bytes for heartbeat/command responses.
 * Payloads are <=64 bytes on is28/tlm: MQTT fixed+topic overhead is 12 bytes.
 * Called only in the kernel Wi-Fi owner; no synchronization/lwIP call here. */
static inline bool rn_telemetry_room(unsigned requests, unsigned ring_free, unsigned tcp_free) {
    return requests > 2 && ring_free >= 128 + 76 && tcp_free >= 128 + 76;
}
#endif
