#ifndef BUDDY_LINE_RECOVERY_POWER_H
#define BUDDY_LINE_RECOVERY_POWER_H
#include <math.h>
#include "src/motion/continuous_drive.h"

enum { LINE_RECOVERY_PWM_LIMIT = 300 };
typedef struct {
    ContinuousDrive drive;
    SignedWheel wheel;
} LineRecoveryWheel;
static inline int32_t line_recovery_output_cap(int32_t pwm) {
    return pwm > LINE_RECOVERY_PWM_LIMIT ? LINE_RECOVERY_PWM_LIMIT
           : pwm < -LINE_RECOVERY_PWM_LIMIT ? -LINE_RECOVERY_PWM_LIMIT
                                            : pwm;
}

/* Signed wrapper around the established low-speed line controller. The
 * continuous controller's optional ramp assistance stays zero; its existing
 * single startup window is bounded by the recovery envelope below. */
static inline int32_t line_recovery_wheel_step(LineRecoveryWheel *p, unsigned w, int16_t wanted,
                                               float speed, float dt, uint64_t now,
                                               int32_t actual_previous) {
    if (w > 1 || !isfinite(speed) || !isfinite(dt)) {
        *p = (LineRecoveryWheel){0};
        return 0;
    }
    int sign = wanted > 0 ? 1 : wanted < 0 ? -1 : 0;
    if (!sign) {
        *p = (LineRecoveryWheel){0};
        return 0;
    }
    float along = speed * sign;
    if (!isfinite(along)) {
        *p = (LineRecoveryWheel){0};
        return 0;
    }
    int32_t prior = actual_previous < 0 ? -actual_previous : actual_previous;
    prior = line_recovery_output_cap(prior);
    int32_t out = continuous_drive_step(&p->drive, &p->wheel, w,
                                        (int16_t)(wanted * sign), along, dt, now, prior);
    return sign * line_recovery_output_cap(out);
}
static inline DualLevels line_recovery_levels(unsigned w, int32_t pwm) {
    DualLevels v = {0};
    if (w > 1)
        return v;
    pwm = line_recovery_output_cap(pwm);
    uint16_t magnitude = (uint16_t)(pwm < 0 ? -pwm : pwm);
    bool a_side = w == 0 ? pwm > 0 : pwm < 0;
    if (a_side)
        v.a = magnitude;
    else
        v.b = magnitude;
    return v;
}
#endif
