/**
 * @file boosted_wheel.h
 * @brief Boosted wheel.
 * @details Motion helpers run under the coordinator/control ownership contract.
 * Preserve bounded outputs, target sign, stop precedence and request lifetimes.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef JUNCTION_BOOSTED_WHEEL_H
#define JUNCTION_BOOSTED_WHEEL_H
#include "src/motion/signed_wheel.h"
#include "src/motion/effort_scale.h"
enum
{
    J_BOOST_FORWARD = 300,
    J_BOOST_TURN = 420,
    J_BOOST_PEAK = 480
};
static inline int32_t boosted_junction_step (signed_wheel_t * p_data,
                                             unsigned wheel, int16_t wanted,
                                             float speed, float delta_time,
                                             uint64_t now, bool turning)
{
    return effort_scale20(junction_wheel_step(p_data, wheel, wanted, speed,
                                              delta_time, now, turning));
}
/* Already-scaled PWM enters here: never multiply again from the 1ms ISR. */
static inline int32_t boosted_junction_peak_cap (int32_t pwm, uint64_t until,
                                                 uint64_t now)
{
    int limit = now < until ? J_BOOST_PEAK : J_BOOST_TURN;
    return pwm > limit ? limit : pwm < -limit ? -limit : pwm;
}
static inline dual_levels_t boosted_junction_levels (unsigned wheel,
                                                     int32_t pwm, bool turning)
{
    dual_levels_t value = {0};
    if (wheel > 1)
    {
        return value;
    }
    int limit = turning ? J_BOOST_PEAK : J_BOOST_FORWARD;
    if (pwm > limit)
    {
        pwm = limit;
    }
    if (pwm < -limit)
    {
        pwm = -limit;
    }
    uint16_t magnitude = (uint16_t)(pwm < 0 ? -pwm : pwm);
    bool a_side = wheel == 0 ? pwm > 0 : pwm < 0;
    if (a_side)
    {
        value.a = magnitude;
    }
    else
    {
        value.b = magnitude;
    }
    return value;
}
#endif

/* End of file. */
