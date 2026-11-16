/**
 * @file maneuver_power.h
 * @brief Maneuver power.
 * @details Motion helpers run under the coordinator/control ownership contract.
 * Preserve bounded outputs, target sign, stop precedence and request lifetimes.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BARCODE_MANEUVER_POWER_H
#define BARCODE_MANEUVER_POWER_H
#include "src/motion/circuit_wheel.h"
enum
{
    M_SUSTAIN = 600,
    M_PEAK = 750,
    M_PEAK_US = 180000
};
typedef struct
{
    bool active;
} maneuver_power_t;
/* Physical PWM units, no extra20% multiplication. This controller is used
 * only for Z turning or an active rolling line-search wheel. */
static inline int32_t maneuver_wheel_step (maneuver_power_t * p_state,
                                           signed_wheel_t * p_data,
                                           unsigned wheel, int16_t wanted,
                                           float speed, float delta_time,
                                           uint64_t now)
{
    int sign = wanted > 0 ? 1 : wanted < 0 ? -1 : 0;
    if (!sign)
    {
        *p_state = (maneuver_power_t){0};
        *p_data = (signed_wheel_t){0};
        return 0;
    }
    if (!p_state->active || p_data->sign != sign)
    {
        float ramp = p_data->sign == sign ? p_data->pi.ramp : 0;
        *p_data = (signed_wheel_t){
            .sign = (int8_t)sign, .pi = {.ramp = ramp}, .was_turning = true};
        p_state->active = true;
    }
    delta_time = clampf(delta_time, 0, .05f);
    float goal = (float)(wanted * sign), along = speed * sign;
    p_data->pi.ramp +=
        clampf(goal - p_data->pi.ramp, -700 * delta_time, 300 * delta_time);
    if (!p_data->peak_used)
    {
        p_data->peak_used = p_data->peak_active = true;
        p_data->peak_end_us = now + M_PEAK_US;
    }
    if (now >= p_data->peak_end_us || along >= fminf(50, .8f * goal) ||
        along < -8)
    {
        p_data->peak_active = false;
    }
    float error = p_data->pi.ramp - along;
    if (error < 0 && p_data->pi.integral > 0)
    {
        p_data->pi.integral = 0;
    }
    if (along > goal + 10 || error < -15)
    {
        p_data->pi.integral = 0;
        return 0;
    }
    float ff_value = fmaxf(
        400, (float)effort_scale20(
                 (int32_t)(feedforward(wheel, sign, p_data->pi.ramp) + .5f)));
    float candidate =
        clampf(p_data->pi.integral + 6 * error * delta_time, -180, 180);
    float proposed = ff_value + 2.2f * error + candidate;
    if (!((proposed > M_SUSTAIN && error > 0) || (proposed < 0 && error < 0)))
    {
        p_data->pi.integral = candidate;
    }
    int32_t out =
        (int32_t)(clampf(ff_value + 2.2f * error + p_data->pi.integral, 0,
                         M_SUSTAIN) +
                  .5f);
    if (p_data->peak_active && out < M_PEAK)
    {
        out = M_PEAK;
    }
    return sign * out;
}
static inline void maneuver_leave (maneuver_power_t * p_state,
                                   signed_wheel_t * p_data, int16_t wanted)
{
    if (!p_state->active)
    {
        return;
    }
    p_state->active = false;
    p_data->pi.integral = 0;
    p_data->kick = (line_breakaway_t){0};
    p_data->peak_used = p_data->peak_active = false;
    p_data->peak_end_us = 0;
    p_data->was_turning = false;
    float goal = fabsf((float)wanted);
    if (p_data->pi.ramp > goal)
    {
        p_data->pi.ramp = goal;
    }
}
static inline int32_t maneuver_output_cap (int32_t pwm, uint64_t until,
                                           uint64_t now)
{
    int limit = now < until ? M_PEAK : M_SUSTAIN;
    return pwm > limit ? limit : pwm < -limit ? -limit : pwm;
}
static inline dual_levels_t maneuver_levels (unsigned wheel, int32_t pwm,
                                             bool allow_reverse)
{
    dual_levels_t value = {0};
    if (wheel > 1)
    {
        return value;
    }
    if (!allow_reverse && pwm < 0)
    {
        return value;
    }
    if (pwm > M_PEAK)
    {
        pwm = M_PEAK;
    }
    if (pwm < -M_PEAK)
    {
        pwm = -M_PEAK;
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
