#ifndef ROBOPICO_RAMP_DRIVE_H
#define ROBOPICO_RAMP_DRIVE_H
#include "src/motion/continuous_drive.h"
#include "src/motion/ramp_assist.h"
#include "include/interfaces/motion_api.h"
static inline void motion_apply_ramp_profile(ContinuousDrive *s, const MotionRampAssist *a) {
    float h = (float)a->hold_milli / 1000.0f;
    s->boost_cap = RA_PWM_CAP;
    s->boost_frac = s->active ? (float)a->boost_milli / 1000.0f : 0;
    s->boost_add = s->active ? (float)a->push_pwm_x100 / 100.0f : 0;
    s->hold_min_speed = h > 0 ? RA_HOLD_MIN_MM_S * h : 0;
    s->hold_i_max = h > 0 ? CONTINUOUS_I_MAX + (RA_HOLD_I_MAX - CONTINUOUS_I_MAX) * h : 0;
    s->hold_ki = h > 0 ? 1 + (RA_HOLD_KI - 1) * h : 0;
}
#endif
