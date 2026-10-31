#ifndef CONTINUOUS_STEERING_H
#define CONTINUOUS_STEERING_H

#include <math.h>
#include "src/barcode/barcode_nav.h"

/* Output-only steering adaptation. The optical tracker and barcode reader
 * continue to see the original ADC samples and make the original decisions. */
typedef struct {
    float mean, half;
    int16_t emitted[2];
    float previous_error, return_confidence, same_side_duration;
    float gain, damp_weight;
    uint8_t side : 2;
    uint8_t active : 1;
} ContinuousSteering;

static inline void continuous_steering_reset(ContinuousSteering *s) {
    *s = (ContinuousSteering){0};
}

static inline float continuous_steering_finite(float x, float fallback) {
    return isfinite(x) ? x : fallback;
}

static inline AnalogDemand continuous_steering_step(ContinuousSteering *s, AnalogDemand d,
                                                    Circuit *c, const CircuitInput *i,
                                                    bool scanning, unsigned profile, float dt) {
    bool ordinary = d.decision == SYM_STRAIGHT || d.decision == SYM_LEFT ||
                    d.decision == SYM_RIGHT || d.decision == C_FAST;
    bool eligible =
        c->phase == C_FOLLOW && !c->busy && !d.stop && !c->follower.core.white && ordinary;
    if (!eligible) {
        continuous_steering_reset(s);
        return d;
    }

    dt = continuous_steering_finite(dt, 0.0f);
    dt = clampf(dt, 0.0f, 0.05f);
    float desired_left = (float)d.target[0];
    float desired_right = (float)d.target[1];
    float desired_mean = (desired_left + desired_right) * 0.5f;
    float original_half = (desired_right - desired_left) * 0.5f;
    float error = ((float)d.level[0] - (float)d.level[1]) / 1000.0f;
    float measured_left = continuous_steering_finite(i->speed[0], 0.0f);
    float measured_right = continuous_steering_finite(i->speed[1], 0.0f);
    float measured_half = (measured_right - measured_left) * 0.5f;

    /* Retain the established profile speeds; a lower requested ceiling is
     * reached through the mean slew below. */
    float ceiling = scanning ? 45.0f : profile == 3 ? 135.0f : 55.0f;
    desired_mean = clampf(desired_mean, 0.0f, ceiling);
    original_half = clampf(original_half, -desired_mean, desired_mean);

    bool entering = !s->active;
    if (entering) {
        float measured_mean = fmaxf(0.0f, (measured_left + measured_right) * 0.5f);
        s->mean = fminf(measured_mean, ceiling);
        s->half = 0.0f;
        s->emitted[0] = s->emitted[1] = (int16_t)lroundf(s->mean);
        s->previous_error = error; /* no derivative kick on entry */
        s->return_confidence = 0.0f;
        s->same_side_duration = 0.0f;
        s->side = original_half > 0.0f ? 1u : original_half < 0.0f ? 2u : 0u;
        s->gain = 0.70f;
        s->damp_weight = 0.0f;
        s->active = true;
    }

    /* Dwell tracks the core's actual signed steering request. A residual
     * sensor imbalance inside the core's straight dead zone has no half-turn
     * authority and must not earn full gain. */
    uint8_t side = original_half > 0.0f ? 1u : original_half < 0.0f ? 2u : 0u;
    if (!entering && side != 0u && side == s->side) {
        s->same_side_duration = fminf(0.35f, s->same_side_duration + dt);
    } else if (side != s->side || side == 0u) {
        s->same_side_duration = 0.0f;
    }
    s->side = side;
    s->gain = 0.70f + 0.30f * clampf(s->same_side_duration / 0.35f, 0.0f, 1.0f);
    float desired_half = original_half * s->gain;

    /* Scale return confidence by the normalized calibrated-error fall rate.
     * The 40ms EMA smooths it; the half slew already bounds output changes. */
    float abs_error = fabsf(error);
    float fall_rate = dt > 0.0f ? fmaxf(0.0f, (fabsf(s->previous_error) - abs_error) / dt) : 0.0f;
    float confidence_target = clampf(fall_rate / 4.0f, 0.0f, 1.0f);
    if (original_half == 0.0f)
        confidence_target = 1.0f;
    s->return_confidence += (confidence_target - s->return_confidence) * dt / (0.040f + dt);
    s->return_confidence = clampf(s->return_confidence, 0.0f, 1.0f);
    s->damp_weight = s->return_confidence;
    float damping = clampf(0.35f * measured_half * s->damp_weight, -7.0f, 7.0f);
    desired_half -= damping;
    s->previous_error = error;

    /* The original bend request regains all authority after 350ms on the same
     * side. Damping fades as the optical error stops shrinking. */
    desired_half = clampf(desired_half, -desired_mean, desired_mean);
    float outer = desired_mean + fabsf(desired_half);
    if (outer > ceiling)
        desired_mean = fmaxf(0.0f, ceiling - fabsf(desired_half));

    float evidence = clampf(fabsf(error) / 0.70f, 0.0f, 1.0f);
    float steer_rate = 120.0f + 180.0f * evidence;
    float half_change = desired_half - s->half;
    s->half += clampf(half_change, -steer_rate * dt, steer_rate * dt);

    float mean_change = desired_mean - s->mean;
    float mean_rate = mean_change >= 0.0f ? 100.0f : 200.0f;
    s->mean += clampf(mean_change, -mean_rate * dt, mean_rate * dt);
    s->mean = fmaxf(0.0f, s->mean);
    s->half = clampf(s->half, -s->mean, s->mean);

    for (unsigned w = 0; w < 2; ++w) {
        float target = s->mean + (w ? s->half : -s->half);
        target = clampf(target, 0.0f, 135.0f);
        s->emitted[w] = (int16_t)lroundf(target);
        d.target[w] = s->emitted[w];
    }
    c->fast = !scanning && (d.target[0] > 55 || d.target[1] > 55);
    d.decision = scanning ? 24 : 25;
    return d;
}

#endif
