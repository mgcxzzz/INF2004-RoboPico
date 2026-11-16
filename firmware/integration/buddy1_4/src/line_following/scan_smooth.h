/**
 * @file scan_smooth.h
 * @brief Scan smooth.
 * @details The control task owns mutable steering/recovery state.
 * Keep straight, scan, turn and recovery behavior distinct; no direct PWM here.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BARCODE_SCAN_SMOOTH_H
#define BARCODE_SCAN_SMOOTH_H
#include "src/barcode/barcode_analog.h"
#include "src/line_following/smooth_follow.h"
#ifndef DRIVE_DIAG_RAW
#define DRIVE_DIAG_RAW(d) ((void)0)
#define DRIVE_DIAG_SOFT(d, kind, scan, active) ((void)0)
#endif

typedef struct
{
    float mean;
    float half;
    int16_t emitted[2];
    bool active;
    bool scanning;
    uint64_t activity_us;
    double start_mm;
} scan_smooth_t;
/* Optical state, recovery, crossing and turns run on every real ADC reading.
 * This output adapter never captures a heading or freezes the follower. */
static inline analog_demand_t
scan_smooth_step (circuit_t * p_character, barcode_nav_t * p_count,
                  const line_calibration_t * p_cal,
                  const circuit_input_t * p_index, unsigned profile,
                  smooth_follow_t * p_legacy, const barcode_analog_t * p_first,
                  scan_smooth_t * p_state, float delta_time)
{
    analog_demand_t demand =
        barcode_circuit_step(p_character, p_count, p_cal, p_index, profile);
    DRIVE_DIAG_RAW(demand);
    delta_time = clampf(delta_time, 0, .05f);
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
    /* The original optical tracker decides all loss/search/stop conditions.
     * No synthetic ADCs, extra centre calibration or record of approach yaw. */
    bool following = p_character->phase == C_FOLLOW && !p_character->busy &&
                     !demand.stop && !p_character->follower.core.white;
    bool ordinary = demand.decision == SYM_STRAIGHT ||
                    demand.decision == SYM_LEFT ||
                    demand.decision == SYM_RIGHT || demand.decision == C_FAST;
    unsigned strongest =
        demand.level[0] > demand.level[1] ? demand.level[0] : demand.level[1];
    unsigned weakest =
        demand.level[0] < demand.level[1] ? demand.level[0] : demand.level[1];
    unsigned magnitude = (unsigned)abs((int)demand.level[0] - demand.level[1]);
    *p_legacy = (smooth_follow_t){0};
    if (!following || !ordinary || strongest < 200)
    {
        p_state->active = false;
        DRIVE_DIAG_SOFT(demand, 0, scanning, p_state->active);
        return demand;
    }
    /* Retain the V12 strong-bend bypass. Smoothing a clear one-sided departure
     * was a previous regression: it must never hold a crooked entry heading. */
    bool urgent = magnitude >= 650 ||
                  ((demand.target[0] <= 10 || demand.target[1] <= 10) &&
                   strongest >= 400);
    if (urgent)
    {
        p_state->mean = (demand.target[0] + demand.target[1]) * .5f;
        p_state->half = (demand.target[1] - demand.target[0]) * .5f;
        p_state->active = true;
        p_state->emitted[0] = demand.target[0];
        p_state->emitted[1] = demand.target[1];
        p_character->fast = false;
        DRIVE_DIAG_SOFT(demand, 2, scanning, p_state->active);
        return demand;
    }
    float desired_mean = (demand.target[0] + demand.target[1]) * .5f;
    float desired_half = (demand.target[1] - demand.target[0]) * .5f;
    if (weakest >= 200 && magnitude < 350)
    {
        /* Soften only the small balance response, restoring full V12 demand
         * continuously by350. Unequal brightness still guides the robot. */
        desired_half *= .45f + .55f * magnitude / 350.0f;
    }
    if (!scanning && profile == 1 && weakest >= 200 && magnitude < 350)
    {
        desired_mean += 15 * clampf((350 - (float)magnitude) / 200, 0, 1);
    }
    if (profile == 2 && p_character->fast && !scanning)
    {
        desired_mean = RAMP2_SPEED;
    }
    if (scanning && desired_mean > 45)
    {
        desired_mean = 45;
    }
    if (!p_state->active)
    {
        /* Seed from measured forward motion; the unchanged wheel controller
         * additionally ramps its reference and owns breakaway/PWM. */
        float speed = fmaxf(0, (p_index->speed[0] + p_index->speed[1]) * .5f);
        p_state->mean = fminf(desired_mean, speed);
        p_state->half = 0;
        p_state->active = true;
        p_state->emitted[0] = p_state->emitted[1] =
            (int16_t)lroundf(p_state->mean);
    }
    float urgency = clampf(((float)magnitude - 200) / 450, 0, 1);
    float tau = (scanning ? .180f : .120f) * (1 - urgency) + .025f * urgency;
    float desired_change =
        (desired_half - p_state->half) * delta_time / (tau + delta_time);
    float steer_rate = (scanning ? 100 : 140) + 460 * urgency;
    p_state->half += clampf(desired_change, -steer_rate * delta_time,
                            steer_rate * delta_time);
    /* Filter common speed as well as differential steering: alternating ADC
     * balance used to modulate total forward speed even on a straight. */
    float mean_tau = .160f * (1 - urgency) + .025f * urgency;
    float mean_change =
        (desired_mean - p_state->mean) * delta_time / (mean_tau + delta_time);
    p_state->mean += clampf(mean_change, -240 * delta_time, 80 * delta_time);
    /* Faster cruising must not leak through a departure or scan transition. */
    float ceiling = scanning                             ? 50
                    : (magnitude > 350 || weakest < 200) ? 50
                                                         : 60;
    if (profile > 1 && !scanning && p_character->fast)
    {
        ceiling = desired_mean;
    }
    p_state->mean = fminf(p_state->mean, ceiling - fabsf(p_state->half));
    for (unsigned wheel = 0; wheel < 2; ++wheel)
    {
        float target = p_state->mean + (wheel ? p_state->half : -p_state->half);
        target = fmaxf(0, target);
        /* A1mm/s Schmitt band prevents sub-count rounding chatter without
         * adding delay to the urgent/stop/search paths above. */
        float difference = target - p_state->emitted[wheel];
        if (difference >= 1)
        {
            p_state->emitted[wheel] += (int16_t)floorf(difference);
        }
        else if (difference <= -1)
        {
            p_state->emitted[wheel] -= (int16_t)floorf(-difference);
        }
        if (p_state->emitted[wheel] > ceiling)
        {
            p_state->emitted[wheel] = (int16_t)ceiling;
        }
        demand.target[wheel] = p_state->emitted[wheel];
    }
    p_character->fast =
        !scanning && (demand.target[0] > 55 || demand.target[1] > 55);
    demand.decision = scanning ? 24 : 25;
    DRIVE_DIAG_SOFT(demand, 1, scanning, p_state->active);
    return demand;
}
#endif

/* End of file. */
