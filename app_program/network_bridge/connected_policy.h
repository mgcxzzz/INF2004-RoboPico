/**
 * @file connected_policy.h
 * @brief Connected policy.
 * @details Application tasks exchange copied, bounded command and telemetry records.
 * The kernel Wi-Fi service owns the transport; reconnect never restarts motion.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_CONNECTED_POLICY_H
#define RTOS_CONNECTED_POLICY_H
#include "network_bridge.h"
#include "../rtos_bringup/bringup_policy.h"
/* Separate application state from coordinator mode. Values are the existing
 * dashboard protocol, not a second motor-ownership enum. */
static inline uint8_t rn_drive_state (unsigned flags, bool paused, bool pending,
                                      unsigned reason)
{
    if (flags & BR_ESTOP)
    {
        return reason == BR_REASON_CLOCK || reason == BR_REASON_ADC ||
                       reason == BR_REASON_API
                   ? 6
                   : 5;
    }
    if (flags & BR_LATCHED)
    {
        return 6;
    }
    if (flags & BR_ACTIVE)
    {
        return paused ? 2 : 1;
    }
    if (pending)
    {
        return 0; /* Countdown is not motion. */
    }
    return reason != BR_REASON_BOOT ? 3 : 0;
}
static inline bool rn_link_usable (const rn_status_t * p_state, uint32_t now)
{
    return p_state->link == 3 &&
           (p_state->flags & (RN_CONNECTED | RN_SUBSCRIBED)) ==
               (RN_CONNECTED | RN_SUBSCRIBED) &&
           p_state->polls && now - p_state->last_poll_ms <= 250U;
}
/* Require recent successful delivery in production, not merely a polling
 * task that is alive while TCP is black-holed. HB/release metadata continue
 * when ordinary telemetry is disabled. Comparison is modulo32 safe. */
static inline bool rn_delivery_fresh (const rn_status_t * p_state, uint32_t now,
                                      uint32_t connected_since)
{
    return rn_link_usable(p_state, now) &&
           now - p_state->last_complete_ms <= 3000U &&
           (int32_t)(p_state->last_complete_ms - connected_since) >= 0;
}
static inline bool rn_request_current (uint32_t now, uint32_t received,
                                       uint32_t barrier, uint32_t current,
                                       uint32_t epoch, uint32_t connects)
{
    return now - received <= 500U && barrier == current && epoch == connects;
}
#endif

/* End of file. */
