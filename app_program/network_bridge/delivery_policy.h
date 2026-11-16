/**
 * @file delivery_policy.h
 * @brief Delivery policy.
 * @details Application tasks exchange copied, bounded command and telemetry records.
 * The kernel Wi-Fi service owns the transport; reconnect never restarts motion.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_DELIVERY_POLICY_H
#define RTOS_DELIVERY_POLICY_H
#include <stdbool.h>
#include <stdint.h>
/* Leave two requests and 128 output bytes for heartbeat/command responses.
 * Payloads are <=64 bytes on is28/tlm: MQTT fixed+topic overhead is 12 bytes.
 * Called only in the kernel Wi-Fi owner; no synchronization/lwIP call here. */
static inline bool rn_telemetry_room (unsigned requests, unsigned ring_free,
                                      unsigned tcp_free)
{
    return requests > 2 && ring_free >= 128 + 76 && tcp_free >= 128 + 76;
}
#endif

/* End of file. */
