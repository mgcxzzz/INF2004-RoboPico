/**
 * @file signed_wheel.h
 * @brief Signed wheel.
 * @details Motion helpers run under the coordinator/control ownership contract.
 * Preserve bounded outputs, target sign, stop precedence and request lifetimes.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef JUNCTION_SIGNED_WHEEL_H
#define JUNCTION_SIGNED_WHEEL_H
#include "src/line_following/wheel_control.h"
#include "src/line_following/line_logic.h"
typedef struct
{
    wheel_pi_t pi;
    line_breakaway_t kick;
    int8_t sign;
    uint64_t peak_end_us;
    bool peak_used;
    bool peak_active;
    bool was_turning;
} signed_wheel_t;
enum
{
    J_TURN_FEEDFORWARD = 310,
    J_TURN_PWM_LIMIT = 350,
    J_TURN_PEAK_PWM = 400,
    J_TURN_PEAK_US = 150000
};
_Static_assert(J_TURN_FEEDFORWARD <= J_TURN_PWM_LIMIT &&
                   J_TURN_PWM_LIMIT <= J_TURN_PEAK_PWM &&
                   J_TURN_PEAK_PWM <= 999,
               "Invalid turn effort limits");
/* V4: turn-only31% feedforward /35% sustained limit plus one40% start pulse.
 * Peak ends at150ms or measured along-speed>=25mm/s. It never retriggers in
 * that turn. The adapter also clamps expired peaks from its independent1ms ISR.
 * Forward phases retain V11's25% ceiling. This is a provisional load estimate,
 * NOT a forced minimum output: signed PI can reduce effort for overspeed.
 * Caller must settle before reversing; zero clears PI and boost. */
static inline int32_t junction_wheel_step (signed_wheel_t * p_data,
                                           unsigned wheel, int16_t wanted,
                                           float speed, float delta_time,
                                           uint64_t now, bool turning)
{
    int sign = wanted > 0 ? 1 : wanted < 0 ? -1 : 0;
    if (sign != p_data->sign)
    {
        *p_data = (signed_wheel_t){.sign = (int8_t)sign};
    }
    if (!sign)
    {
        *p_data = (signed_wheel_t){0};
        return 0;
    }
    float magnitude = (float)(wanted * sign), along = speed * sign;
    /* The loaded pivot and ordinary forward motion use different feedforward.
       Clear accumulated compensation in BOTH directions of that transition:
       a negative turn integral can otherwise cancel the lower forward output.
       Preserve the speed ramp, so handover does not restart acceleration. */
    if (turning != p_data->was_turning)
    {
        p_data->pi.integral = 0;
    }
    p_data->was_turning = turning;
    if (!turning)
    {
        p_data->peak_used = false;
        p_data->peak_active = false;
        p_data->peak_end_us = 0;
    }
    else
    {
        if (!p_data->peak_used)
        {
            p_data->peak_used = true;
            p_data->peak_active = true;
            p_data->peak_end_us = now + J_TURN_PEAK_US;
        }
        if (now >= p_data->peak_end_us || along >= 25 || along < -8)
        {
            p_data->peak_active = false;
        }
    }
    p_data->pi.ramp += clampf(magnitude - p_data->pi.ramp, -100.0f * delta_time,
                              100.0f * delta_time);
    float error = p_data->pi.ramp - along,
          ff_value = feedforward(wheel, sign, p_data->pi.ramp);
    if (turning)
    {
        ff_value = fmaxf(ff_value, J_TURN_FEEDFORWARD);
    }
    int limit = turning ? J_TURN_PWM_LIMIT : PWM_LIMIT;
    float candidate =
        clampf(p_data->pi.integral + LINE_SPEED_KI * error * delta_time,
               -LINE_SPEED_I_LIMIT, LINE_SPEED_I_LIMIT);
    float u_value = ff_value + LINE_SPEED_KP * error + candidate;
    if (!((u_value > limit && error > 0) || (u_value < 0 && error < 0)))
    {
        p_data->pi.integral = candidate;
    }
    int32_t out =
        (int32_t)(clampf(ff_value + LINE_SPEED_KP * error + p_data->pi.integral,
                         0, (float)limit) +
                  0.5f);
    /* The shared breakaway helper clamps to25%. Do not let it silently clip
       the new turn PI; still use its unchanged bounded pulse for lower output. */
    int32_t boosted =
        line_breakaway(&p_data->kick, now, (int16_t)magnitude, p_data->pi.ramp,
                       along, out > PWM_LIMIT ? PWM_LIMIT : out);
    if (!turning || boosted > out)
    {
        out = boosted;
    }
    if (turning && p_data->peak_active && out < J_TURN_PEAK_PWM)
    {
        out = J_TURN_PEAK_PWM;
    }
    return sign * out;
}
/* Integer-only clamp used both immediately before commit and in safety ISR.
 * A stale computed peak may not be re-applied after its absolute deadline. */
static inline int32_t junction_peak_cap (int32_t pwm, uint64_t until,
                                         uint64_t now)
{
    int limit = now < until ? J_TURN_PEAK_PWM : J_TURN_PWM_LIMIT;
    if (pwm > limit)
    {
        return limit;
    }
    if (pwm < -limit)
    {
        return -limit;
    }
    return pwm;
}
/* Shared dual_levels also clamps at25%. Only the explicitly gated phase4
 * turn may use the bounded40% peak; other phases keep the V11 mapping. */
static inline dual_levels_t junction_levels (unsigned wheel, int32_t pwm,
                                             bool turning)
{
    if (!turning)
    {
        return dual_levels(wheel, pwm);
    }
    dual_levels_t value = {0};
    if (wheel > 1)
    {
        return value;
    }
    if (pwm > J_TURN_PEAK_PWM)
    {
        pwm = J_TURN_PEAK_PWM;
    }
    if (pwm < -J_TURN_PEAK_PWM)
    {
        pwm = -J_TURN_PEAK_PWM;
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
