/**
 * @file smooth_follow.h
 * @brief Smooth follow.
 * @details The control task owns mutable steering/recovery state.
 * Keep straight, scan, turn and recovery behavior distinct; no direct PWM here.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BARCODE_SMOOTH_FOLLOW_H
#define BARCODE_SMOOTH_FOLLOW_H
#include "src/barcode/barcode_nav.h"
#include "src/motion/ramp_profile.h"

typedef struct
{
    float target[2];
    bool active;
} smooth_follow_t;
/* This adapter changes only visible-line output targets. Raw calibrated
 * levels still own loss detection, recovery memory and junction acceptance.
 * No extra optical delay is added to the shared follower or barcode reader. */
static inline void smooth_follow_apply (smooth_follow_t * p_state,
                                        const circuit_t * p_character,
                                        float delta_time,
                                        analog_demand_t * p_demand)
{
    bool following = p_character->phase == C_FOLLOW;
    const circuit_tracker_t * p_sample = &p_character->follower;
    int error = (int)p_demand->level[0] - p_demand->level[1],
        magnitude = abs(error);
    unsigned strongest = p_demand->level[0] > p_demand->level[1]
                             ? p_demand->level[0]
                             : p_demand->level[1];
    /* A clear departure or strong near-pivot demand must retain its original bend
     * authority immediately. The same is true of every blind search/stop. */
    bool near_pivot = p_demand->target[0] <= 10 || p_demand->target[1] <= 10;
    bool eligible =
        following && !p_character->busy && !p_demand->stop &&
        !p_sample->core.white && strongest >= 200 && magnitude < 700 &&
        !(near_pivot && strongest >= 400) &&
        (p_demand->decision == SYM_STRAIGHT || p_demand->decision == SYM_LEFT ||
         p_demand->decision == SYM_RIGHT || p_demand->decision == C_FAST);
    if (!eligible)
    {
        *p_state = (smooth_follow_t){0};
        return;
    }
    float desired[2] = {(float)p_demand->target[0], (float)p_demand->target[1]};
    /* Reduce only small, well-visible balance corrections, blending back to
     * the original gain by350/1000 error. Stronger bends keep the old gain. */
    if (p_demand->level[0] >= 300 && p_demand->level[1] >= 300 &&
        magnitude < 350)
    {
        float mean = (desired[0] + desired[1]) * .5f;
        float half = (desired[1] - desired[0]) * .5f *
                     (.65f + .35f * magnitude / 350.0f);
        desired[0] = mean - half;
        desired[1] = mean + half;
    }
    if (!p_state->active)
    {
        p_state->active = true;
        p_state->target[0] = desired[0];
        p_state->target[1] = desired[1];
    }
    else
    {
        /* Leaving90/135 cruising must not carry a high-speed target into a
         * correction. Preserve the old correction ceiling immediately. */
        if ((p_state->target[0] > 55 || p_state->target[1] > 55) &&
            desired[0] <= 50 && desired[1] <= 50)
        {
            for (unsigned wheel = 0; wheel < 2; ++wheel)
            {
                p_state->target[wheel] = fminf(p_state->target[wheel], 50);
            }
        }
        delta_time = clampf(delta_time, 0, .05f);
        float urgency = clampf((magnitude - 250) / 450.0f, 0, 1);
        float tau = .060f - .040f * urgency, rate = 200 + 400 * urgency;
        float alpha = delta_time / (tau + delta_time);
        for (unsigned wheel_value = 0; wheel_value < 2; ++wheel_value)
        {
            float change =
                (desired[wheel_value] - p_state->target[wheel_value]) * alpha;
            p_state->target[wheel_value] +=
                clampf(change, -rate * delta_time, rate * delta_time);
        }
    }
    for (unsigned wheel_value_value = 0; wheel_value_value < 2;
         ++wheel_value_value)
    {
        p_demand->target[wheel_value_value] =
            (int16_t)(p_state->target[wheel_value_value] + .5f);
    }
    p_demand->decision = p_demand->target[0] < p_demand->target[1]   ? SYM_LEFT
                         : p_demand->target[0] > p_demand->target[1] ? SYM_RIGHT
                         : p_character->fast                         ? C_FAST
                                             : SYM_STRAIGHT;
}
static inline analog_demand_t
barcode_drive_step (circuit_t * p_character, barcode_nav_t * p_count,
                    const line_calibration_t * p_cal,
                    const circuit_input_t * p_index, unsigned profile,
                    smooth_follow_t * p_state, float delta_time)
{
    analog_demand_t demand =
        barcode_circuit_step(p_character, p_count, p_cal, p_index, profile);
    if (profile == 2 && p_character->fast && !demand.stop)
    {
        demand.target[0] = demand.target[1] = RAMP2_SPEED;
    }
    smooth_follow_apply(p_state, p_character, delta_time, &demand);
    return demand;
}
#endif

/* End of file. */
