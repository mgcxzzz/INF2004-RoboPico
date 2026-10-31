#ifndef JUNCTION_BOOSTED_WHEEL_H
#define JUNCTION_BOOSTED_WHEEL_H
#include "src/motion/signed_wheel.h"
#include "src/motion/effort_scale.h"
enum { J_BOOST_FORWARD = 300, J_BOOST_TURN = 420, J_BOOST_PEAK = 480 };
static inline int32_t boosted_junction_step(SignedWheel *p, unsigned w, int16_t wanted, float speed,
                                            float dt, uint64_t now, bool turning) {
    return effort_scale20(junction_wheel_step(p, w, wanted, speed, dt, now, turning));
}
/* Already-scaled PWM enters here: never multiply again from the 1ms ISR. */
static inline int32_t boosted_junction_peak_cap(int32_t pwm, uint64_t until, uint64_t now) {
    int limit = now < until ? J_BOOST_PEAK : J_BOOST_TURN;
    return pwm > limit ? limit : pwm < -limit ? -limit : pwm;
}
static inline DualLevels boosted_junction_levels(unsigned w, int32_t pwm, bool turning) {
    DualLevels v = {0};
    if (w > 1)
        return v;
    int limit = turning ? J_BOOST_PEAK : J_BOOST_FORWARD;
    if (pwm > limit)
        pwm = limit;
    if (pwm < -limit)
        pwm = -limit;
    uint16_t magnitude = (uint16_t)(pwm < 0 ? -pwm : pwm);
    bool a_side = w == 0 ? pwm > 0 : pwm < 0;
    if (a_side)
        v.a = magnitude;
    else
        v.b = magnitude;
    return v;
}
#endif
