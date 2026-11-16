/**
 * @file line_recovery_power.h
 * @brief Line recovery power.
 * @details Motion helpers run under the coordinator/control ownership contract.
 * Preserve bounded outputs, target sign, stop precedence and request lifetimes.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BUDDY_LINE_RECOVERY_POWER_H
#define BUDDY_LINE_RECOVERY_POWER_H
#include <math.h>
#include "src/motion/continuous_drive.h"

enum
{
    LINE_RECOVERY_PWM_LIMIT = 300
};
typedef struct
{
    continuous_drive_t drive;
    signed_wheel_t wheel;
} line_recovery_wheel_t;
static inline int32_t line_recovery_output_cap (int32_t pwm)
{
    return pwm > LINE_RECOVERY_PWM_LIMIT    ? LINE_RECOVERY_PWM_LIMIT
           : pwm < -LINE_RECOVERY_PWM_LIMIT ? -LINE_RECOVERY_PWM_LIMIT
                                            : pwm;
}

/* Signed wrapper around the established low-speed line controller. The
 * continuous controller's optional ramp assistance stays zero; its existing
 * single startup window is bounded by the recovery envelope below. */
static inline int32_t line_recovery_wheel_step (line_recovery_wheel_t * p_data,
                                                unsigned wheel, int16_t wanted,
                                                float speed, float delta_time,
                                                uint64_t now,
                                                int32_t actual_previous)
{
    if (((wheel > 1) || (!isfinite(speed))) || (!isfinite(delta_time)))
    {
        *p_data = (line_recovery_wheel_t){0};
        return 0;
    }
    int sign = wanted > 0 ? 1 : wanted < 0 ? -1 : 0;
    if (!sign)
    {
        *p_data = (line_recovery_wheel_t){0};
        return 0;
    }
    float along = speed * sign;
    if (!isfinite(along))
    {
        *p_data = (line_recovery_wheel_t){0};
        return 0;
    }
    int32_t prior = actual_previous < 0 ? -actual_previous : actual_previous;
    prior = line_recovery_output_cap(prior);
    int32_t out = continuous_drive_step(&p_data->drive, &p_data->wheel, wheel,
                                        (int16_t)(wanted * sign), along,
                                        delta_time, now, prior);
    return sign * line_recovery_output_cap(out);
}
static inline dual_levels_t line_recovery_levels (unsigned wheel, int32_t pwm)
{
    dual_levels_t value = {0};
    if (wheel > 1)
    {
        return value;
    }
    pwm = line_recovery_output_cap(pwm);
    uint16_t magnitude = (uint16_t)(pwm < 0 ? -pwm : pwm);
    bool a_side = 0 == wheel ? pwm > 0 : pwm < 0;
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
