/**
 * @file continuous_follow.h
 * @brief Continuous follow.
 * @details The control task owns mutable steering/recovery state.
 * Keep straight, scan, turn and recovery behavior distinct; no direct PWM here.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BUDDY_CONTINUOUS_FOLLOW_H
#define BUDDY_CONTINUOUS_FOLLOW_H
#include "src/motion/continuous_drive.h"
#include "src/line_following/scan_smooth.h"
#include "src/line_following/continuous_steering.h"
/* Barcode state and original optical tracker run exactly once per tick.
 * Only the ordinary visible-line output adapter is replaced. */
static inline analog_demand_t continuous_follow_step_with_cruise (
    circuit_t * p_character, barcode_nav_t * p_count,
    const line_calibration_t * p_cal, const circuit_input_t * p_index,
    unsigned profile, smooth_follow_t * p_legacy,
    const barcode_analog_t * p_first, scan_smooth_t * p_state,
    continuous_steering_t * p_sample, float delta_time,
    unsigned cruise_speed_mm_s)
{
    if (profile == 2)
    {
        continuous_steering_reset(p_sample);
        return scan_smooth_step(p_character, p_count, p_cal, p_index, profile,
                                p_legacy, p_first, p_state, delta_time);
    }
    analog_demand_t demand =
        barcode_circuit_step(p_character, p_count, p_cal, p_index, profile);
    DRIVE_DIAG_RAW(demand);
    bool activity = p_first->scanning || p_count->pending ||
                    p_first->adc >= (unsigned)p_first->baseline + 16;
    if (activity)
    {
        p_state->activity_us = p_index->now;
    }
    bool scanning =
        !p_character->busy && !demand.stop &&
        (activity ||
         (p_state->scanning && p_index->now - p_state->activity_us < 600000));
    if (scanning != p_state->scanning)
    {
        double mm_value = (p_index->wheel[0] + p_index->wheel[1]) * .5;
        if (scanning)
        {
            p_state->start_mm = mm_value;
        }
        barcode_event(p_count, scanning ? B_SOFT_BEGIN : B_SOFT_END,
                      p_index->now, mm_value, mm_value - p_state->start_mm,
                      p_count->last_code, '-', p_character->busy ? 1 : 0, NULL);
    }
    p_state->scanning = scanning;
    *p_legacy = (smooth_follow_t){0};
    demand = continuous_steering_step_with_cruise(
        p_sample, demand, p_character, p_index, scanning, profile, delta_time,
        cruise_speed_mm_s);
    p_state->mean = p_sample->mean;
    p_state->half = p_sample->half;
    p_state->active = p_sample->active;
    for (unsigned wheel = 0; wheel < 2; ++wheel)
    {
        p_state->emitted[wheel] = p_sample->emitted[wheel];
    }
    DRIVE_DIAG_SOFT(demand, p_sample->active ? 1 : 0, scanning,
                    p_sample->active);
    return demand;
}

static inline analog_demand_t continuous_follow_step (
    circuit_t * p_character, barcode_nav_t * p_count,
    const line_calibration_t * p_cal, const circuit_input_t * p_index,
    unsigned profile, smooth_follow_t * p_legacy,
    const barcode_analog_t * p_first, scan_smooth_t * p_state,
    continuous_steering_t * p_sample, float delta_time)
{
    return continuous_follow_step_with_cruise(
        p_character, p_count, p_cal, p_index, profile, p_legacy, p_first,
        p_state, p_sample, delta_time, DRIVE_MODE_BASELINE_SPEED);
}
static inline bool continuous_drive_selected (unsigned profile, unsigned phase,
                                              bool busy, bool external,
                                              bool ramp, bool maneuver,
                                              bool white,
                                              const analog_demand_t * p_demand)
{
    /* `white` is retained for call-site compatibility. During the existing
  * 150 ms bridge and 2 s search confirmation, a short both-white gap may
  * still produce an ordinary forward recovery/probe demand. The final
  * decision and explicit targets distinguish those from an active search. */
    (void)white;
    if (!p_demand)
    {
        return false;
    }
    bool forward_decision =
        p_demand->decision == SYM_STRAIGHT || p_demand->decision == SYM_LEFT ||
        p_demand->decision == SYM_RIGHT ||
        p_demand->decision == SYM_RECOVER_LEFT ||
        p_demand->decision == SYM_RECOVER_RIGHT ||
        p_demand->decision == C_PROBE_LEFT ||
        p_demand->decision == C_PROBE_RIGHT || p_demand->decision == 24 ||
        p_demand->decision == 25;
    return (profile == 1 || profile == 3) && phase == C_FOLLOW && !busy &&
           !external && !ramp && !maneuver && !p_demand->stop &&
           forward_decision;
}
static inline int32_t continuous_output_cap (int32_t pwm)
{
    return pwm < 0 ? 0 : pwm > CONTINUOUS_PWM_MAX ? CONTINUOUS_PWM_MAX : pwm;
}
static inline dual_levels_t continuous_output_levels (unsigned wheel,
                                                      int32_t pwm)
{
    dual_levels_t value = {0};
    if (wheel > 1)
    {
        return value;
    }
    if (wheel == 0)
    {
        value.a = (uint16_t)continuous_output_cap(pwm);
    }
    else
    {
        value.b = (uint16_t)continuous_output_cap(pwm);
    }
    return value;
}
/* Drop private mode latches when leaving ordinary drive. Seed the existing
 * legacy PI in its native units; its special manoeuvre rules still take over. */
static inline void continuous_leave (continuous_drive_t * p_state,
                                     signed_wheel_t * p_data, unsigned wheel,
                                     int16_t wanted, float speed,
                                     int32_t previous)
{
    if (!p_state->active)
    {
        return;
    }
    if (wanted > 0 && p_data->sign == 1)
    {
        float ff_value = feedforward(wheel, 1, p_data->pi.ramp);
        p_data->pi.integral =
            clampf((float)previous / 1.2f - ff_value -
                       LINE_SPEED_KP * (p_data->pi.ramp - speed),
                   -LINE_SPEED_I_LIMIT, LINE_SPEED_I_LIMIT);
    }
    *p_state = (continuous_drive_t){0};
}
#endif

/* End of file. */
