/**
 * @file ramp_model.h
 * @brief Ramp model.
 * @details The IMU task owns sensor acquisition and calibration; readers use
 * snapshots. Ramp requests expire through the coordinator; lift safety may
 * remove output.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_RAMP_MODEL_H
#define RTOS_RAMP_MODEL_H
#include "imu_motion.h"
/* Pure policy adapter: numeric detector6 is unchanged. The returned request
 * has no effect until the shared nav coordinator approves it. */
static inline motion_ramp_assist_t
ri_ramp_request (ramp_assist_t * p_state, const b4v23_tilt_t * p_tilt,
                 uint64_t now, float delta_time, bool running, bool automatic,
                 bool following, const float speed[2])
{
    ramp_assist_input_t in_value = {
        .now_us = now,
        .dt = delta_time,
        .run_active = running,
        .auto_mode = automatic,
        .following = following,
        .tilt_seq = p_tilt->seq,
        .tilt_us = p_tilt->t_us,
        .tilt_valid = p_tilt->valid,
        .tilt_mm = p_tilt->height_mm,
        .tilt_skip = ra_tilt_skip(p_tilt->tracking, p_tilt->valid,
                                  p_tilt->quality_reasons),
        .speed_mm_s = {speed[0], speed[1]}};
    float alpha = ramp_assist_step(p_state, &in_value);
    return (motion_ramp_assist_t){
        .ttl_ms = 50,
        .boost_milli = (uint16_t)(alpha + .5f),
        .push_pwm_x100 = (uint16_t)(p_state->push_pwm * 100 + .5f),
        .hold_milli = (uint16_t)(p_state->hold_level * 1000 + .5f)};
}
#endif

/* End of file. */
