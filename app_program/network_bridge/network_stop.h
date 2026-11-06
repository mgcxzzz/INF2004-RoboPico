#ifndef RTOS_NETWORK_STOP_H
#define RTOS_NETWORK_STOP_H

#include <stdbool.h>
#include <stdint.h>

#include "network_bridge.h"

/* Source 15 subtype 0xC6: fixed 32-byte v0.2.1 frame (F2, len29, seq, C6,
 * version21, reserved byte5). Six full-width source fields remain u32 in native
 * RAM; age and status masks are bounded only when copied to the wire. Metadata
 * u16 @30: flags bits0..2, signed link bits3..5, failure bits6..12, valid bit13,
 * reserved bits14..15. */
typedef struct {
    unsigned valid : 1;
    unsigned flags : 3;
    signed int link : 3;
    unsigned failmask : 7;
    uint32_t stop_ms;
    uint32_t poll_age_ms;
    uint32_t app_age_ms;
    uint32_t ack_age_ms;
    uint32_t connects;
    uint32_t last_complete_ms;
} RnNetworkStopSnapshot;

enum {
    RN_STOP_FAIL_LINK_DOWN = 1U << 0,
    RN_STOP_FAIL_NOT_CONNECTED = 1U << 1,
    RN_STOP_FAIL_NOT_SUBSCRIBED = 1U << 2,
    RN_STOP_FAIL_POLL_STALE = 1U << 3,
    RN_STOP_FAIL_APP_STALE = 1U << 4,
    RN_STOP_FAIL_ACK_STALE = 1U << 5,
    RN_STOP_FAIL_ACK_BEFORE_CONNECT = 1U << 6
};

static inline uint8_t rn_network_stop_failure_mask(const RnStatus *status, uint32_t now_ms,
                                                   uint32_t app_last_ms,
                                                   uint32_t connected_since_ms) {
    uint8_t mask = 0U;
    if (status->link != 3)
        mask |= RN_STOP_FAIL_LINK_DOWN;
    if ((status->flags & RN_CONNECTED) == 0U)
        mask |= RN_STOP_FAIL_NOT_CONNECTED;
    if ((status->flags & RN_SUBSCRIBED) == 0U)
        mask |= RN_STOP_FAIL_NOT_SUBSCRIBED;
    if (status->polls == 0U || now_ms - status->last_poll_ms > 250U)
        mask |= RN_STOP_FAIL_POLL_STALE;
    if (now_ms - app_last_ms > 250U)
        mask |= RN_STOP_FAIL_APP_STALE;
    if (now_ms - status->last_complete_ms > 3000U)
        mask |= RN_STOP_FAIL_ACK_STALE;
    if ((int32_t)(status->last_complete_ms - connected_since_ms) < 0)
        mask |= RN_STOP_FAIL_ACK_BEFORE_CONNECT;
    return mask;
}

static inline RnNetworkStopSnapshot rn_network_stop_candidate(const RnStatus *status,
                                                               uint32_t now_ms,
                                                               uint32_t app_last_ms,
                                                               uint32_t connected_since_ms) {
    return (RnNetworkStopSnapshot){
        .valid = 1U,
        .flags = (unsigned)(status->flags & 7U),
        .link = status->link,
        .failmask = rn_network_stop_failure_mask(status, now_ms, app_last_ms,
                                                 connected_since_ms),
        .stop_ms = now_ms,
        .poll_age_ms = status->polls == 0U ? UINT32_MAX : now_ms - status->last_poll_ms,
        .app_age_ms = now_ms - app_last_ms,
        .ack_age_ms = now_ms - status->last_complete_ms,
        .connects = status->connects,
        .last_complete_ms = status->last_complete_ms};
}

static inline void rn_network_stop_clear(RnNetworkStopSnapshot *snapshot) {
    *snapshot = (RnNetworkStopSnapshot){0};
}

/* Latch the first failed session guard until accepted Start clears it. */
static inline bool rn_network_stop_capture_once(RnNetworkStopSnapshot *stored,
                                                const RnNetworkStopSnapshot *candidate) {
    if (stored->valid)
        return false;
    *stored = *candidate;
    stored->valid = 1U;
    return true;
}

static inline void rn_network_stop_put32(uint8_t *out, unsigned at, uint32_t value) {
    out[at] = (uint8_t)value;
    out[at + 1U] = (uint8_t)(value >> 8U);
    out[at + 2U] = (uint8_t)(value >> 16U);
    out[at + 3U] = (uint8_t)(value >> 24U);
}

static inline void rn_network_stop_encode(const RnNetworkStopSnapshot *snapshot, uint8_t seq,
                                          uint8_t out[RN_NETWORK_STOP_PACKET_BYTES]) {
    for (unsigned i = 0U; i < RN_NETWORK_STOP_PACKET_BYTES; ++i)
        out[i] = 0U;
    out[0] = 0xf2U;
    out[1] = RN_NETWORK_STOP_PACKET_BYTES - 3U;
    out[2] = seq;
    out[3] = 0xc6U;
    out[4] = 0x21U; /* v0.2.1 */
    if (!snapshot->valid)
        return;

    rn_network_stop_put32(out, 6U, snapshot->stop_ms);
    rn_network_stop_put32(out, 10U, snapshot->poll_age_ms);
    rn_network_stop_put32(out, 14U, snapshot->app_age_ms);
    rn_network_stop_put32(out, 18U, snapshot->ack_age_ms);
    rn_network_stop_put32(out, 22U, snapshot->connects);
    rn_network_stop_put32(out, 26U, snapshot->last_complete_ms);
    uint16_t metadata = (uint16_t)(snapshot->flags & 7U) |
                        (uint16_t)(((uint16_t)snapshot->link & 7U) << 3U) |
                        (uint16_t)((snapshot->failmask & 0x7fU) << 6U) | (1U << 13U);
    out[30] = (uint8_t)metadata;
    out[31] = (uint8_t)(metadata >> 8U);
}

#endif
