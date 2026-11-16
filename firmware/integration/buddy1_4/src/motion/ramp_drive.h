/**
 * @file ramp_drive.h
 * @brief Ramp drive.
 * @details Motion helpers run under the coordinator/control ownership contract.
 * Preserve bounded outputs, target sign, stop precedence and request lifetimes.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef ROBOPICO_RAMP_DRIVE_H
#define ROBOPICO_RAMP_DRIVE_H
#include "src/motion/continuous_drive.h"
#include "src/motion/ramp_assist.h"
#include "include/interfaces/motion_api.h"
static inline void
motion_apply_ramp_profile (continuous_drive_t * p_state,
                           const motion_ramp_assist_t * p_first)
{
    float h_value = (float)p_first->hold_milli / 1000.0f;
    p_state->boost_cap = RA_PWM_CAP;
    p_state->boost_frac =
        p_state->active ? (float)p_first->boost_milli / 1000.0f : 0;
    p_state->boost_add =
        p_state->active ? (float)p_first->push_pwm_x100 / 100.0f : 0;
    p_state->hold_min_speed = h_value > 0 ? RA_HOLD_MIN_MM_S * h_value : 0;
    p_state->hold_i_max =
        h_value > 0
            ? CONTINUOUS_I_MAX + (RA_HOLD_I_MAX - CONTINUOUS_I_MAX) * h_value
            : 0;
    p_state->hold_ki = h_value > 0 ? 1 + (RA_HOLD_KI - 1) * h_value : 0;
}
#endif

/* End of file. */
