#ifndef BUDDY5_SIGNED_CONTINUOUS_H
#define BUDDY5_SIGNED_CONTINUOUS_H

#include <math.h>
#include "src/motion/continuous_drive.h"
#include "src/motion/dual_logic.h"
#include "include/interfaces/motion_api.h"
#ifndef BUDDY5_DRIVE_I_MAX
#define BUDDY5_DRIVE_I_MAX 100
#endif
#if BUDDY5_DRIVE_I_MAX < 100 || BUDDY5_DRIVE_I_MAX > 300
#error "Buddy5 integral authority must stay within 100..300 PWM units"
#endif

/* RTOS port (rtos_buddy1_5): identical logic to the SDK v0.6.1 header; only the
 * barr-c-cleanup type names (continuous_drive_t, signed_wheel_t, dual_levels_t)
 * differ.
 * A direction adapter, not another PI implementation. The original forward
 * controller, gain/slew/420 cap and bounded startup aid are reused unchanged.
 * Reverse uses the same magnitude feedforward provisionally: reverse/pivot
 * load calibration and real heading accuracy are NOT established by this. */
typedef struct {
    continuous_drive_t drive;
    int8_t sign;
} B5SignedContinuous;

static inline bool b5_signed_selected(bool external, unsigned owner) {
    return external && owner == MOTION_BUDDY5;
}
static inline int32_t b5_signed_cap(int32_t value) {
    return value > CONTINUOUS_PWM_MAX ? CONTINUOUS_PWM_MAX :
           value < -CONTINUOUS_PWM_MAX ? -CONTINUOUS_PWM_MAX : value;
}
static inline dual_levels_t b5_signed_levels(unsigned wheel, int32_t pwm) {
    dual_levels_t result = {0};
    if (wheel > 1)
        return result;
    pwm = b5_signed_cap(pwm);
    unsigned magnitude = (unsigned)(pwm < 0 ? -pwm : pwm);
    if ((wheel == 0 && pwm > 0) || (wheel == 1 && pwm < 0))
        result.a = (uint16_t)magnitude;
    else
        result.b = (uint16_t)magnitude;
    return result;
}
static inline void b5_signed_reset(B5SignedContinuous *state, signed_wheel_t *legacy) {
    *state = (B5SignedContinuous){0};
    *legacy = (signed_wheel_t){0};
}
static inline int32_t b5_signed_step(B5SignedContinuous *state, signed_wheel_t *legacy,
                                    unsigned wheel, int16_t wanted, float measured, float dt,
                                    uint64_t now, int32_t previous) {
    if (!state || !legacy)
        return 0;
    if (wheel > 1 || wanted < -127 || wanted > 127 || !isfinite(measured) ||
        !isfinite(dt) || dt < 0 || wanted == 0) {
        b5_signed_reset(state, legacy);
        return 0; /* Stop/invalid demand never ramps through zero. */
    }
    int sign = wanted > 0 ? 1 : -1;
    if (state->sign != sign) {
        b5_signed_reset(state, legacy);
        /* Do not reverse polarity or start a fresh direction while still
         * rolling. The coordinator's existing settling/timeout gates remain. */
        if (previous != 0 || fabsf(measured) >= 5.0f)
            return 0;
        state->sign = (int8_t)sign;
    }
    if ((sign > 0 && previous < 0) || (sign < 0 && previous > 0)) {
        b5_signed_reset(state, legacy);
        return 0;
    }
    /* Private state cannot inherit Buddy4 ramp boosts or speed floors. */
    state->drive.boost_frac = state->drive.boost_cap = state->drive.boost_add = 0;
    state->drive.hold_min_speed = state->drive.hold_ki = 0;
    /* v0.2 automatic build opts into the ORIGINAL controller's independent
     * I-limit hook. No speed floor, boost, gain change or new PI equation.
     * At30mm/s, I100 trapped a loaded pivot at230/250 despite the420cap. */
    state->drive.hold_i_max = (float)BUDDY5_DRIVE_I_MAX;
    int32_t along_previous = (int32_t)((int64_t)b5_signed_cap(previous) * sign);
    int32_t magnitude = continuous_drive_step(&state->drive, legacy, wheel,
        (int16_t)(wanted * sign), measured * sign, dt, now, along_previous);
    legacy->sign = (int8_t)sign;
    return b5_signed_cap(sign * magnitude);
}
#endif
