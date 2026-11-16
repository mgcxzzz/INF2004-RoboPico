/**
 * @file ramp_profile.h
 * @brief Ramp profile.
 * @details Motion helpers run under the coordinator/control ownership contract.
 * Preserve bounded outputs, target sign, stop precedence and request lifetimes.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BARCODE_RAMP_PROFILE_H
#define BARCODE_RAMP_PROFILE_H
#include "src/barcode/barcode_nav.h"
#include "src/motion/circuit_wheel.h"

enum
{
    RAMP2_SPEED = 180,
    RAMP2_SUSTAIN = 500,
    RAMP2_BOOST = 650,
    RAMP2_QUALIFY_US = 40000,
    RAMP2_BOOST_US = 1500000,
    RAMP2_RECOVER_US = 200000
};
typedef struct
{
    uint64_t deficit_since;
    uint64_t boost_until;
    uint64_t recovered_since;
    float extra;
    bool active;
    bool deficit;
    bool boost_used;
    bool recovered;
} ramp_wheel_t;
static inline bool ramp_profile_active (unsigned profile,
                                        const circuit_t * p_character,
                                        const analog_demand_t * p_demand)
{
    return 2 == profile && C_FOLLOW == p_character->phase &&
           (!p_character->busy) && (!p_demand->stop) &&
           (!p_character->follower.core.white) &&
           ((p_demand->level[0] >= 400) || (p_demand->level[1] >= 400));
}
static inline unsigned barcode_profile_cruise (unsigned profile)
{
    return 2 == profile ? RAMP2_SPEED : (unsigned)circuit_cruise(profile);
}
/* Higher effort follows EACH wheel's own target, so an intentionally slow or
 * off inner wheel is never driven toward the outer wheel's speed. Physical
 * PWM units are used here; do not apply the historical20% multiplier again. */
static inline int32_t ramp_wheel_step (ramp_wheel_t * p_state,
                                       signed_wheel_t * p_data, unsigned wheel,
                                       int16_t wanted, float speed,
                                       float delta_time, uint64_t now)
{
    if (wanted <= 0)
    {
        *p_state = (ramp_wheel_t){0};
        *p_data = (signed_wheel_t){0};
        return 0;
    }
    if ((!p_state->active) || (p_data->sign != 1))
    {
        *p_state = (ramp_wheel_t){.active = true};
        p_data->pi.integral = 0;
        p_data->sign = 1;
        p_data->was_turning = false;
        p_data->peak_used = p_data->peak_active = false;
        p_data->peak_end_us = 0;
        p_data->kick = (line_breakaway_t){0};
    }
    delta_time = clampf(delta_time, 0, .05f);
    bool falling = p_data->pi.ramp > wanted;
    if (falling)
    {
        p_data->pi.ramp = wanted;
    }
    else
    {
        p_data->pi.ramp += fminf(wanted - p_data->pi.ramp, 400 * delta_time);
    }
    float error = p_data->pi.ramp - speed;
    bool loaded =
        ((((!falling) && (wanted >= 20)) && (p_data->pi.ramp >= 20)) &&
         (speed >= -8)) &&
        (error > fmaxf(12, .30f * p_data->pi.ramp));
    if (!loaded)
    {
        p_state->deficit = false;
    }
    else
    {
        if (!p_state->deficit)
        {
            p_state->deficit = true;
            p_state->deficit_since = now;
        }
        if ((now - p_state->deficit_since >= RAMP2_QUALIFY_US) &&
            (!p_state->boost_used))
        {
            p_state->boost_used = true;
            p_state->boost_until = now + RAMP2_BOOST_US;
        }
    }
    /* Persistent stalls cannot continually renew the high-power deadline.
     * Re-arm only after200ms genuinely close to the requested wheel speed. */
    bool recovered = (((!falling) && (p_data->pi.ramp >= 20)) &&
                      (speed >= .85f * p_data->pi.ramp)) &&
                     (fabsf(error) <= fmaxf(8, .15f * p_data->pi.ramp));
    if (!recovered)
    {
        p_state->recovered = false;
    }
    else
    {
        if (!p_state->recovered)
        {
            p_state->recovered = true;
            p_state->recovered_since = now;
        }
        if (now - p_state->recovered_since >= RAMP2_RECOVER_US)
        {
            p_state->boost_used = false;
            p_state->boost_until = 0;
        }
    }
    bool boost =
        (loaded && (p_state->boost_used)) && (now < p_state->boost_until);
    if (boost)
    {
        p_state->extra = fminf(220, p_state->extra + 2000 * delta_time);
    }
    else
    {
        p_state->extra = fmaxf(0, p_state->extra - 1600 * delta_time);
    }
    if (falling || (error <= 0))
    {
        p_state->extra = 0;
        if (p_data->pi.integral > 0)
        {
            p_data->pi.integral = 0;
        }
    }
    /* Coast immediately when a wheel is already too fast for its target.
     * No reverse braking and no positive boost through a slowdown command. */
    if ((speed > wanted + 10) || (error < -10))
    {
        p_data->pi.integral = 0;
        p_state->extra = 0;
        return 0;
    }
    float ff_value = (float)effort_scale20(
        (int32_t)(feedforward(wheel, 1, p_data->pi.ramp) + .5f));
    float limit = boost ? RAMP2_BOOST : RAMP2_SUSTAIN;
    float candidate =
        clampf(p_data->pi.integral + 10 * error * delta_time, -180, 180);
    float proposed = ff_value + 3.5f * error + candidate + p_state->extra;
    if (!(((proposed > limit) && (error > 0)) ||
          ((proposed < 0) && (error < 0))))
    {
        p_data->pi.integral = candidate;
    }
    return (int32_t)(clampf(ff_value + 3.5f * error + p_data->pi.integral +
                                p_state->extra,
                            0, limit) +
                     .5f);
}
static inline void ramp_wheel_leave (ramp_wheel_t * p_state,
                                     signed_wheel_t * p_data, int16_t wanted)
{
    if (p_state->active)
    {
        *p_state = (ramp_wheel_t){0};
        p_data->pi.integral = 0;
        p_data->kick = (line_breakaway_t){0};
        if (p_data->pi.ramp > wanted)
        {
            p_data->pi.ramp = fmaxf(wanted, 0);
        }
    }
}
/* Integer-only output envelope shared by the control commit and1ms ISR. */
static inline int32_t barcode_output_cap (int32_t pwm, bool ramp,
                                          uint64_t until, uint64_t now)
{
    if (!ramp)
    {
        return boosted_junction_peak_cap(pwm, until, now);
    }
    int limit = now < until ? RAMP2_BOOST : RAMP2_SUSTAIN;
    return pwm < 0 ? 0 : pwm > limit ? limit : pwm;
}
static inline dual_levels_t barcode_output_levels (unsigned wheel, int32_t pwm,
                                                   bool ramp, bool turning,
                                                   unsigned profile)
{
    if (!ramp)
    {
        return circuit_levels(wheel, pwm, turning, profile);
    }
    dual_levels_t value = {0};
    if (wheel > 1)
    {
        return value;
    }
    if (pwm < 0)
    {
        pwm = 0;
    }
    if (pwm > RAMP2_BOOST)
    {
        pwm = RAMP2_BOOST;
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
