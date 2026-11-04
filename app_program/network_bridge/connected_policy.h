#ifndef RTOS_CONNECTED_POLICY_H
#define RTOS_CONNECTED_POLICY_H
#include "network_bridge.h"
#include "../rtos_bringup/bringup_policy.h"
/* Separate application state from coordinator mode. Values are the existing
 * dashboard protocol, not a second motor-ownership enum. */
static inline uint8_t rn_drive_state(unsigned flags, bool paused, bool pending, unsigned reason) {
    if (flags & BR_ESTOP)
        return reason == BR_REASON_CLOCK || reason == BR_REASON_ADC || reason == BR_REASON_API ? 6
                                                                                               : 5;
    if (flags & BR_LATCHED)
        return 6;
    if (flags & BR_ACTIVE)
        return paused ? 2 : 1;
    if (pending)
        return 0; /* Countdown is not motion. */
    return reason != BR_REASON_BOOT ? 3 : 0;
}
static inline bool rn_link_usable(const RnStatus *s, uint32_t now) {
    return s->link == 3 &&
           (s->flags & (RN_CONNECTED | RN_SUBSCRIBED)) == (RN_CONNECTED | RN_SUBSCRIBED) &&
           s->polls && now - s->last_poll_ms <= 250U;
}
static inline bool rn_request_current(uint32_t now, uint32_t received, uint32_t barrier,
                                      uint32_t current, uint32_t epoch, uint32_t connects) {
    return now - received <= 500U && barrier == current && epoch == connects;
}
#endif
