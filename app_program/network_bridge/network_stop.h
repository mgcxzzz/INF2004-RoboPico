/**
 * @file network_stop.h
 * @brief Network stop.
 * @details Application tasks exchange copied, bounded command and telemetry records.
 * The kernel Wi-Fi service owns the transport; reconnect never restarts motion.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_NETWORK_STOP_H
#define RTOS_NETWORK_STOP_H

#include <stdbool.h>
#include <stdint.h>

#include "network_bridge.h"
#include "src/platform/compact_signed.h"

/* Source 15 subtype 0xC6: fixed 32-byte v0.2.1 frame (F2, len29, seq, C6,
 * version21, reserved byte5). Six full-width source fields remain u32 in native
 * RAM; age and status masks are bounded only when copied to the wire. Metadata
 * u16 @30: flags bits0..2, signed link bits3..5, failure bits6..12, valid bit13,
 * reserved bits14..15. */
typedef struct
{
    unsigned valid : 1;
    unsigned flags : 3;
    unsigned link_bits : 3;
    unsigned failmask : 7;
    uint32_t stop_ms;
    uint32_t poll_age_ms;
    uint32_t app_age_ms;
    uint32_t ack_age_ms;
    uint32_t connects;
    uint32_t last_complete_ms;
} rn_network_stop_snapshot_t;

enum
{
    RN_STOP_FAIL_LINK_DOWN = 1U << 0,
    RN_STOP_FAIL_NOT_CONNECTED = 1U << 1,
    RN_STOP_FAIL_NOT_SUBSCRIBED = 1U << 2,
    RN_STOP_FAIL_POLL_STALE = 1U << 3,
    RN_STOP_FAIL_APP_STALE = 1U << 4,
    RN_STOP_FAIL_ACK_STALE = 1U << 5,
    RN_STOP_FAIL_ACK_BEFORE_CONNECT = 1U << 6
};

static inline uint8_t
rn_network_stop_failure_mask (const rn_status_t * p_status, uint32_t now_ms,
                              uint32_t app_last_ms, uint32_t connected_since_ms)
{
    uint8_t mask = 0U;
    if (p_status->link != 3)
    {
        mask |= RN_STOP_FAIL_LINK_DOWN;
    }
    if ((p_status->flags & RN_CONNECTED) == 0U)
    {
        mask |= RN_STOP_FAIL_NOT_CONNECTED;
    }
    if ((p_status->flags & RN_SUBSCRIBED) == 0U)
    {
        mask |= RN_STOP_FAIL_NOT_SUBSCRIBED;
    }
    if (p_status->polls == 0U || now_ms - p_status->last_poll_ms > 250U)
    {
        mask |= RN_STOP_FAIL_POLL_STALE;
    }
    if (now_ms - app_last_ms > 250U)
    {
        mask |= RN_STOP_FAIL_APP_STALE;
    }
    if (now_ms - p_status->last_complete_ms > 3000U)
    {
        mask |= RN_STOP_FAIL_ACK_STALE;
    }
    if ((int32_t)(p_status->last_complete_ms - connected_since_ms) < 0)
    {
        mask |= RN_STOP_FAIL_ACK_BEFORE_CONNECT;
    }
    return mask;
}

static inline rn_network_stop_snapshot_t
rn_network_stop_candidate (const rn_status_t * p_status, uint32_t now_ms,
                           uint32_t app_last_ms, uint32_t connected_since_ms)
{
    return (rn_network_stop_snapshot_t){
        .valid = 1U,
        .flags = (unsigned)(p_status->flags & 7U),
        .link_bits = compact_signed_pack(p_status->link, 3U),
        .failmask = rn_network_stop_failure_mask(p_status, now_ms, app_last_ms,
                                                 connected_since_ms),
        .stop_ms = now_ms,
        .poll_age_ms = p_status->polls == 0U ? UINT32_MAX
                                             : now_ms - p_status->last_poll_ms,
        .app_age_ms = now_ms - app_last_ms,
        .ack_age_ms = now_ms - p_status->last_complete_ms,
        .connects = p_status->connects,
        .last_complete_ms = p_status->last_complete_ms};
}

static inline void
rn_network_stop_clear (rn_network_stop_snapshot_t * p_snapshot)
{
    *p_snapshot = (rn_network_stop_snapshot_t){0};
}

/* Latch the first failed session guard until accepted Start clears it. */
static inline bool
rn_network_stop_capture_once (rn_network_stop_snapshot_t * p_stored,
                              const rn_network_stop_snapshot_t * p_candidate)
{
    if (p_stored->valid)
    {
        return false;
    }
    *p_stored = *p_candidate;
    p_stored->valid = 1U;
    return true;
}

static inline void rn_network_stop_put32 (uint8_t * p_out, unsigned at_value,
                                          uint32_t value)
{
    p_out[at_value] = (uint8_t)value;
    p_out[at_value + 1U] = (uint8_t)(value >> 8U);
    p_out[at_value + 2U] = (uint8_t)(value >> 16U);
    p_out[at_value + 3U] = (uint8_t)(value >> 24U);
}

static inline void
rn_network_stop_encode (const rn_network_stop_snapshot_t * p_snapshot,
                        uint8_t seq, uint8_t out[RN_NETWORK_STOP_PACKET_BYTES])
{
    for (unsigned index = 0U; index < RN_NETWORK_STOP_PACKET_BYTES; ++index)
    {
        out[index] = 0U;
    }
    out[0] = 0xf2U;
    out[1] = RN_NETWORK_STOP_PACKET_BYTES - 3U;
    out[2] = seq;
    out[3] = 0xc6U;
    out[4] = 0x21U; /* v0.2.1 */
    if (!p_snapshot->valid)
    {
        return;
    }

    rn_network_stop_put32(out, 6U, p_snapshot->stop_ms);
    rn_network_stop_put32(out, 10U, p_snapshot->poll_age_ms);
    rn_network_stop_put32(out, 14U, p_snapshot->app_age_ms);
    rn_network_stop_put32(out, 18U, p_snapshot->ack_age_ms);
    rn_network_stop_put32(out, 22U, p_snapshot->connects);
    rn_network_stop_put32(out, 26U, p_snapshot->last_complete_ms);
    uint16_t metadata =
        (uint16_t)(p_snapshot->flags & 7U) |
        (uint16_t)(((uint16_t)p_snapshot->link_bits & 7U) << 3U) |
        (uint16_t)((p_snapshot->failmask & 0x7fU) << 6U) | (1U << 13U);
    out[30] = (uint8_t)metadata;
    out[31] = (uint8_t)(metadata >> 8U);
}

#endif

/* End of file. */
