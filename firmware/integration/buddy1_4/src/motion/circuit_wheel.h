/**
 * @file circuit_wheel.h
 * @brief Circuit wheel.
 * @details Motion helpers run under the coordinator/control ownership contract.
 * Preserve bounded outputs, target sign, stop precedence and request lifetimes.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef CIRCUIT_WHEEL_H
#define CIRCUIT_WHEEL_H
#include "src/motion/boosted_wheel.h"
static inline unsigned circuit_forward_cap (unsigned profile)
{
    return 3 == profile ? 420 : 2 == profile ? 360 : 300;
}
/* Carry normal turn feedforward through a loaded capture correction, but do
   not grant a fresh48% start pulse. Only the selected OUTER wheel calls this.
 */
static inline void circuit_capture_prepare (signed_wheel_t * p_data)
{
    if (p_data->sign != 1)
    {
        *p_data = (signed_wheel_t){.sign = 1};
    }
    p_data->pi.integral = 0;
    p_data->peak_used = true;
    p_data->peak_active = false;
    p_data->peak_end_us = 0;
}
/* Remove extra cruise compensation before using the normal correction PI.
   Measured speed still controls output; this cannot stop inertia instantly. */
static inline void circuit_leave_fast (signed_wheel_t * p_data)
{
    p_data->pi.integral = 0;
    if (p_data->pi.ramp > 50)
    {
        p_data->pi.ramp = 50;
    }
}
static inline int32_t circuit_wheel_step (signed_wheel_t * p_data,
                                          unsigned wheel, int16_t wanted,
                                          float speed, float delta_time,
                                          uint64_t now, bool turning,
                                          unsigned profile)
{
    if (turning)
    {
        return boosted_junction_step(p_data, wheel, wanted, speed, delta_time,
                                     now, true);
    }
    if (1 == profile)
    {
        /* Shared historical controller remains unchanged. In this circuit,
           a falling correction target needs400 rather than100mm/s^2 ramp
           deceleration. Remove retained positive I while actually overspeed. */
        if ((wanted > 0) && 1 == p_data->sign && (p_data->pi.ramp > wanted))
        {
            p_data->pi.ramp =
                fmaxf((float)wanted, p_data->pi.ramp - 300.0f * delta_time);
            if ((speed > wanted + 5) && (p_data->pi.integral > 0))
            {
                p_data->pi.integral = 0;
            }
        }
        int32_t out = boosted_junction_step(p_data, wheel, wanted, speed,
                                            delta_time, now, false);
        /* A low-speed inner wheel must coast instead of continuing to push
           through a correction while still faster than its new target. */
        if (((wanted > 0) && (wanted <= 15)) && (speed > wanted + 5))
        {
            p_data->pi.integral = 0;
            return 0;
        }
        return out;
    }
    if (wanted <= 0)
    {
        *p_data = (signed_wheel_t){0};
        return 0;
    }
    if (p_data->sign != 1)
    {
        *p_data = (signed_wheel_t){.sign = 1};
    }
    if (p_data->was_turning)
    {
        p_data->pi.integral = 0;
    }
    p_data->was_turning = p_data->peak_used = p_data->peak_active = false;
    p_data->peak_end_us = 0;
    /* Faster profiles decelerate the requested speed at200mm/s^2. Actual
       motion is measured; the approach-speed guard must still pass. */
    p_data->pi.ramp += clampf(wanted - p_data->pi.ramp, -200.0f * delta_time,
                              100.0f * delta_time);
    float error = p_data->pi.ramp - speed,
          ff_value = feedforward(wheel, 1, p_data->pi.ramp);
    float limit = 3 == profile ? 350 : 300;
    float candidate =
        clampf(p_data->pi.integral + LINE_SPEED_KI * error * delta_time,
               -LINE_SPEED_I_LIMIT, LINE_SPEED_I_LIMIT);
    float proposed = ff_value + LINE_SPEED_KP * error + candidate;
    if (!(((proposed > limit) && (error > 0)) ||
          ((proposed < 0) && (error < 0))))
    {
        p_data->pi.integral = candidate;
    }
    int32_t out =
        (int32_t)(clampf(ff_value + LINE_SPEED_KP * error + p_data->pi.integral,
                         0, limit) +
                  0.5f);
    int32_t kicked = line_breakaway(&p_data->kick, now, wanted, p_data->pi.ramp,
                                    speed, out > PWM_LIMIT ? PWM_LIMIT : out);
    if (kicked > out)
    {
        out = kicked;
    }
    return effort_scale20(out);
}
static inline dual_levels_t circuit_levels (unsigned wheel, int32_t pwm,
                                            bool turning, unsigned profile)
{
    dual_levels_t value = {0};
    if (wheel > 1)
    {
        return value;
    }
    int limit = turning ? J_BOOST_PEAK : (int)circuit_forward_cap(profile);
    if (pwm > limit)
    {
        pwm = limit;
    }
    if (pwm < 0)
    {
        pwm = 0; /* circuit is forward-only, including rolling turns */
    }
    if (0 == wheel)
    {
        value.a = (uint16_t)pwm;
    }
    else
    {
        value.b = (uint16_t)pwm;
    }
    return value;
}
#endif

/* End of file. */
