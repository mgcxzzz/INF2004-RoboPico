#ifndef B5_SERVO_SLEW_H
#define B5_SERVO_SLEW_H
/* Pure servo slew scheduler (host-testable). A request only changes the
 * target; it NEVER resets the step cadence, so continuous tracking requests
 * at any poll rate cannot postpone movement (bug found in the turn logger
 * v0.1 runtime, 2026-11-16). `applied` is the position actually written to
 * the PWM, reported separately from the requested target. */
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    int applied, target, limit_dd;
    uint64_t last_step, arrived;
    unsigned step_us, settle_us, step_dd;
} B5ServoSlew;
static inline void b5_servo_slew_init(B5ServoSlew *s, int limit_dd, unsigned step_ms,
                                      unsigned speed_dd_s, unsigned settle_ms, uint64_t now) {
    s->applied = s->target = 0; s->limit_dd = limit_dd;
    s->last_step = s->arrived = now;
    s->step_us = step_ms * 1000u; s->settle_us = settle_ms * 1000u;
    s->step_dd = (speed_dd_s * step_ms) / 1000u;
}
static inline void b5_servo_slew_request(B5ServoSlew *s, int dd, uint64_t now) {
    dd = dd < -s->limit_dd ? -s->limit_dd : dd > s->limit_dd ? s->limit_dd : dd;
    if (dd == s->target) return;            /* same target: nothing changes */
    s->target = dd;
    s->arrived = now;                       /* settle restarts; the step cadence does not */
}
/* Call every poll. Returns true when `applied` changed (write the PWM). */
static inline bool b5_servo_slew_tick(B5ServoSlew *s, uint64_t now) {
    if (s->applied == s->target || now - s->last_step < s->step_us) return false;
    int diff = s->target - s->applied, step = (int)s->step_dd;
    s->applied += diff > step ? step : diff < -step ? -step : diff;
    s->last_step = s->arrived = now;
    return true;
}
static inline bool b5_servo_slew_settled(const B5ServoSlew *s, uint64_t now) {
    return s->applied == s->target && now >= s->arrived && now - s->arrived >= s->settle_us;
}
#endif
