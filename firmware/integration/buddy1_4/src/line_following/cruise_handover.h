/**
 * @file cruise_handover.h
 * @brief Cruise handover.
 * @details The control task owns mutable steering/recovery state.
 * Keep straight, scan, turn and recovery behavior distinct; no direct PWM here.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BUDDY_CRUISE_HANDOVER_H
#define BUDDY_CRUISE_HANDOVER_H
#include "src/motion/circuit_wheel.h"
#include "src/line_following/circuit_logic.h"
/* Narrow H1 change: ordinary, visible, already-forward running only. Existing
 * speed targets, gains, caps, wheel-specific slowdown/coast and STOP remain.
 * The10mm/s overspeed allowance selects this transfer; it is not a speed boost. */
static inline bool cruise_handover_eligible (unsigned profile, unsigned phase,
                                             unsigned decision,
                                             const int16_t target[2],
                                             const float speed[2],
                                             const signed_wheel_t wheel[2],
                                             bool special, bool stop)
{
    if (special || stop || (profile != 1 && profile != 3) ||
        phase != C_FOLLOW || (decision != 24 && decision != 25))
    {
        return false;
    }
    for (unsigned w = 0; w < 2; ++w)
    {
        if (target[w] < 20 || wheel[w].sign != 1 || wheel[w].was_turning ||
            !isfinite(speed[w]) || speed[w] < -8 || speed[w] > target[w] + 10)
        {
            return false;
        }
    }
    return true;
}
static inline void cruise_handover_apply (signed_wheel_t * p, bool preserve)
{
    if (!preserve)
    {
        circuit_leave_fast(p);
        return;
    }
    /* Keep the original50mm/s reference cap, but do not erase learned running
     * effort just because the cruising label changed. The ordinary controller
     * still integrates negative error and applies its existing deceleration,
     * saturation, low-inner-wheel coast and zero/sign-change resets. */
    if (p->pi.ramp > 50)
    {
        p->pi.ramp = 50;
    }
}
#endif

/* End of file. */
