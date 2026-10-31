#ifndef BARCODE_MANEUVER_POWER_H
#define BARCODE_MANEUVER_POWER_H
#include "src/motion/circuit_wheel.h"
enum { M_SUSTAIN = 600, M_PEAK = 750, M_PEAK_US = 180000 };
typedef struct {
    bool active;
} ManeuverPower;
/* Physical PWM units, no extra20% multiplication. This controller is used
 * only for Z turning or an active rolling line-search wheel. */
static inline int32_t maneuver_wheel_step(ManeuverPower *s, SignedWheel *p, unsigned w,
                                          int16_t wanted, float speed, float dt, uint64_t now) {
    int sign = wanted > 0 ? 1 : wanted < 0 ? -1 : 0;
    if (!sign) {
        *s = (ManeuverPower){0};
        *p = (SignedWheel){0};
        return 0;
    }
    if (!s->active || p->sign != sign) {
        float ramp = p->sign == sign ? p->pi.ramp : 0;
        *p = (SignedWheel){.sign = (int8_t)sign, .pi = {.ramp = ramp}, .was_turning = true};
        s->active = true;
    }
    dt = clampf(dt, 0, .05f);
    float goal = (float)(wanted * sign), along = speed * sign;
    p->pi.ramp += clampf(goal - p->pi.ramp, -700 * dt, 300 * dt);
    if (!p->peak_used) {
        p->peak_used = p->peak_active = true;
        p->peak_end_us = now + M_PEAK_US;
    }
    if (now >= p->peak_end_us || along >= fminf(50, .8f * goal) || along < -8)
        p->peak_active = false;
    float error = p->pi.ramp - along;
    if (error < 0 && p->pi.integral > 0)
        p->pi.integral = 0;
    if (along > goal + 10 || error < -15) {
        p->pi.integral = 0;
        return 0;
    }
    float ff = fmaxf(400, (float)effort_scale20((int32_t)(feedforward(w, sign, p->pi.ramp) + .5f)));
    float candidate = clampf(p->pi.integral + 6 * error * dt, -180, 180);
    float proposed = ff + 2.2f * error + candidate;
    if (!((proposed > M_SUSTAIN && error > 0) || (proposed < 0 && error < 0)))
        p->pi.integral = candidate;
    int32_t out = (int32_t)(clampf(ff + 2.2f * error + p->pi.integral, 0, M_SUSTAIN) + .5f);
    if (p->peak_active && out < M_PEAK)
        out = M_PEAK;
    return sign * out;
}
static inline void maneuver_leave(ManeuverPower *s, SignedWheel *p, int16_t wanted) {
    if (!s->active)
        return;
    s->active = false;
    p->pi.integral = 0;
    p->kick = (LineBreakaway){0};
    p->peak_used = p->peak_active = false;
    p->peak_end_us = 0;
    p->was_turning = false;
    float goal = fabsf((float)wanted);
    if (p->pi.ramp > goal)
        p->pi.ramp = goal;
}
static inline int32_t maneuver_output_cap(int32_t pwm, uint64_t until, uint64_t now) {
    int limit = now < until ? M_PEAK : M_SUSTAIN;
    return pwm > limit ? limit : pwm < -limit ? -limit : pwm;
}
static inline DualLevels maneuver_levels(unsigned w, int32_t pwm, bool allow_reverse) {
    DualLevels v = {0};
    if (w > 1)
        return v;
    if (!allow_reverse && pwm < 0)
        return v;
    if (pwm > M_PEAK)
        pwm = M_PEAK;
    if (pwm < -M_PEAK)
        pwm = -M_PEAK;
    uint16_t magnitude = (uint16_t)(pwm < 0 ? -pwm : pwm);
    bool a_side = w == 0 ? pwm > 0 : pwm < 0;
    if (a_side)
        v.a = magnitude;
    else
        v.b = magnitude;
    return v;
}
#endif
