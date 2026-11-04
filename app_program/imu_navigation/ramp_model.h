#ifndef RTOS_RAMP_MODEL_H
#define RTOS_RAMP_MODEL_H
#include "imu_motion.h"
/* Pure policy adapter: numeric detector6 is unchanged. The returned request
 * has no effect until the shared nav coordinator approves it. */
static inline MotionRampAssist ri_ramp_request(RampAssist *state, const b4v23_tilt_t *tilt,
                                               uint64_t now, float dt, bool running, bool automatic,
                                               bool following, const float speed[2]) {
    RampAssistInput in = {.now_us = now,
                          .dt = dt,
                          .run_active = running,
                          .auto_mode = automatic,
                          .following = following,
                          .tilt_seq = tilt->seq,
                          .tilt_us = tilt->t_us,
                          .tilt_valid = tilt->valid,
                          .tilt_mm = tilt->height_mm,
                          .tilt_skip =
                              ra_tilt_skip(tilt->tracking, tilt->valid, tilt->quality_reasons),
                          .speed_mm_s = {speed[0], speed[1]}};
    float alpha = ramp_assist_step(state, &in);
    return (MotionRampAssist){.ttl_ms = 50,
                              .boost_milli = (uint16_t)(alpha + .5f),
                              .push_pwm_x100 = (uint16_t)(state->push_pwm * 100 + .5f),
                              .hold_milli = (uint16_t)(state->hold_level * 1000 + .5f)};
}
#endif
