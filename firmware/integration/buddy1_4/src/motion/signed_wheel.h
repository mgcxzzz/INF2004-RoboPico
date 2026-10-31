#ifndef JUNCTION_SIGNED_WHEEL_H
#define JUNCTION_SIGNED_WHEEL_H
#include "src/line_following/wheel_control.h"
#include "src/line_following/line_logic.h"
typedef struct {
    WheelPI pi;
    LineBreakaway kick;
    int8_t sign;
    uint64_t peak_end_us;
    bool peak_used, peak_active, was_turning;
} SignedWheel;
enum {
    J_TURN_FEEDFORWARD = 310,
    J_TURN_PWM_LIMIT = 350,
    J_TURN_PEAK_PWM = 400,
    J_TURN_PEAK_US = 150000
};
_Static_assert(J_TURN_FEEDFORWARD <= J_TURN_PWM_LIMIT && J_TURN_PWM_LIMIT <= J_TURN_PEAK_PWM &&
                   J_TURN_PEAK_PWM <= 999,
               "Invalid turn effort limits");
/* V4: turn-only31% feedforward /35% sustained limit plus one40% start pulse.
 * Peak ends at150ms or measured along-speed>=25mm/s. It never retriggers in
 * that turn. The adapter also clamps expired peaks from its independent1ms ISR.
 * Forward phases retain V11's25% ceiling. This is a provisional load estimate,
 * NOT a forced minimum output: signed PI can reduce effort for overspeed.
 * Caller must settle before reversing; zero clears PI and boost. */
static inline int32_t junction_wheel_step(SignedWheel *p, unsigned w, int16_t wanted, float speed,
                                          float dt, uint64_t now, bool turning) {
    int sign = wanted > 0 ? 1 : wanted < 0 ? -1 : 0;
    if (sign != p->sign) {
        *p = (SignedWheel){.sign = (int8_t)sign};
    }
    if (!sign) {
        *p = (SignedWheel){0};
        return 0;
    }
    float magnitude = (float)(wanted * sign), along = speed * sign;
    /* The loaded pivot and ordinary forward motion use different feedforward.
       Clear accumulated compensation in BOTH directions of that transition:
       a negative turn integral can otherwise cancel the lower forward output.
       Preserve the speed ramp, so handover does not restart acceleration. */
    if (turning != p->was_turning)
        p->pi.integral = 0;
    p->was_turning = turning;
    if (!turning) {
        p->peak_used = false;
        p->peak_active = false;
        p->peak_end_us = 0;
    } else {
        if (!p->peak_used) {
            p->peak_used = true;
            p->peak_active = true;
            p->peak_end_us = now + J_TURN_PEAK_US;
        }
        if (now >= p->peak_end_us || along >= 25 || along < -8)
            p->peak_active = false;
    }
    p->pi.ramp += clampf(magnitude - p->pi.ramp, -100.0f * dt, 100.0f * dt);
    float error = p->pi.ramp - along, ff = feedforward(w, sign, p->pi.ramp);
    if (turning)
        ff = fmaxf(ff, J_TURN_FEEDFORWARD);
    int limit = turning ? J_TURN_PWM_LIMIT : PWM_LIMIT;
    float candidate = clampf(p->pi.integral + LINE_SPEED_KI * error * dt, -LINE_SPEED_I_LIMIT,
                             LINE_SPEED_I_LIMIT);
    float u = ff + LINE_SPEED_KP * error + candidate;
    if (!((u > limit && error > 0) || (u < 0 && error < 0)))
        p->pi.integral = candidate;
    int32_t out =
        (int32_t)(clampf(ff + LINE_SPEED_KP * error + p->pi.integral, 0, (float)limit) + 0.5f);
    /* The shared breakaway helper clamps to25%. Do not let it silently clip
       the new turn PI; still use its unchanged bounded pulse for lower output. */
    int32_t boosted = line_breakaway(&p->kick, now, (int16_t)magnitude, p->pi.ramp, along,
                                     out > PWM_LIMIT ? PWM_LIMIT : out);
    if (!turning || boosted > out)
        out = boosted;
    if (turning && p->peak_active && out < J_TURN_PEAK_PWM)
        out = J_TURN_PEAK_PWM;
    return sign * out;
}
/* Integer-only clamp used both immediately before commit and in safety ISR.
 * A stale computed peak may not be re-applied after its absolute deadline. */
static inline int32_t junction_peak_cap(int32_t pwm, uint64_t until, uint64_t now) {
    int limit = now < until ? J_TURN_PEAK_PWM : J_TURN_PWM_LIMIT;
    if (pwm > limit)
        return limit;
    if (pwm < -limit)
        return -limit;
    return pwm;
}
/* Shared dual_levels also clamps at25%. Only the explicitly gated phase4
 * turn may use the bounded40% peak; other phases keep the V11 mapping. */
static inline DualLevels junction_levels(unsigned w, int32_t pwm, bool turning) {
    if (!turning)
        return dual_levels(w, pwm);
    DualLevels v = {0};
    if (w > 1)
        return v;
    if (pwm > J_TURN_PEAK_PWM)
        pwm = J_TURN_PEAK_PWM;
    if (pwm < -J_TURN_PEAK_PWM)
        pwm = -J_TURN_PEAK_PWM;
    uint16_t magnitude = (uint16_t)(pwm < 0 ? -pwm : pwm);
    bool a_side = w == 0 ? pwm > 0 : pwm < 0;
    if (a_side)
        v.a = magnitude;
    else
        v.b = magnitude;
    return v;
}
#endif
