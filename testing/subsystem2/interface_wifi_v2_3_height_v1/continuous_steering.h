#ifndef CONTINUOUS_STEERING_H
#define CONTINUOUS_STEERING_H

#include <math.h>
#include "v12_2/baseline/testing/subsystem2/circuit_with_barcode/barcode_nav.h"

/* Raised-height V1: output-only adaptation. Optical decisions, white/loss
 * detection, barcode decoding and explicit manoeuvres remain upstream.
 * Short alternating requests use 45% authority, a bounded half-speed and
 * steadier mean; sustained one-sided requests regain the original bend. */
enum { HEIGHT_BEND_DWELL_MS=450 };
#define HEIGHT_INITIAL_GAIN 0.45f
#define HEIGHT_BRIEF_HALF_MAX 12.0f
#define HEIGHT_BRIEF_MEAN_MIN 35.0f
#define HEIGHT_YAW_DAMPING 0.65f
#define HEIGHT_YAW_DAMP_MAX 8.0f
typedef struct {
    float mean, half;
    int16_t emitted[2];
    float same_side_duration;
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

static inline AnalogDemand continuous_steering_step(ContinuousSteering *s,
        AnalogDemand d, Circuit *c, const CircuitInput *i, bool scanning,
        unsigned profile, float dt) {
    bool ordinary = d.decision == SYM_STRAIGHT || d.decision == SYM_LEFT ||
        d.decision == SYM_RIGHT || d.decision == C_FAST;
    bool eligible = c->phase == C_FOLLOW && !c->busy && !d.stop &&
        !c->follower.core.white && ordinary;
    if (!eligible) {
        continuous_steering_reset(s);
        return d;
    }

    dt = continuous_steering_finite(dt, 0.0f);
    dt = clampf(dt, 0.0f, 0.05f);
    float desired_left = (float)d.target[0];
    float desired_right = (float)d.target[1];
    float core_mean = (desired_left + desired_right) * 0.5f;
    float original_half = (desired_right - desired_left) * 0.5f;
    float error = ((float)d.level[0] - (float)d.level[1]) / 1000.0f;
    float measured_left = continuous_steering_finite(i->speed[0], 0.0f);
    float measured_right = continuous_steering_finite(i->speed[1], 0.0f);
    float measured_half = (measured_right - measured_left) * 0.5f;

    /* Retain the established profile speeds; a lower requested ceiling is
     * reached through the mean slew below. */
    float ceiling = scanning ? 45.0f : profile == 3 ? 135.0f : 55.0f;
    core_mean = clampf(core_mean, 0.0f, ceiling);
    original_half = clampf(original_half, -core_mean, core_mean);
    if (core_mean <= 0.0f) {
        continuous_steering_reset(s);
        return d;
    }

    bool entering = !s->active;
    if (entering) {
        float measured_mean = fmaxf(0.0f,
            (measured_left + measured_right) * 0.5f);
        s->mean = fminf(measured_mean, ceiling);
        s->half = 0.0f;
        s->emitted[0] = s->emitted[1] = (int16_t)lroundf(s->mean);
        s->same_side_duration = 0.0f;
        s->side = original_half > 0.0f ? 1u : original_half < 0.0f ? 2u : 0u;
        s->gain = HEIGHT_INITIAL_GAIN;
        s->damp_weight = 0.0f;
        s->active = true;
    }

    /* Dwell tracks the core's actual signed steering request. A residual
     * sensor imbalance inside the core's straight dead zone has no half-turn
     * authority and must not earn full gain. */
    uint8_t side = original_half > 0.0f ? 1u : original_half < 0.0f ? 2u : 0u;
    if (!entering && side != 0u && side == s->side) {
        s->same_side_duration = fminf(HEIGHT_BEND_DWELL_MS/1000.0f,
                                     s->same_side_duration + dt);
    } else if (side != s->side || side == 0u) {
        s->same_side_duration = 0.0f;
    }
    s->side = side;
    float persistence = clampf(s->same_side_duration /
                               (HEIGHT_BEND_DWELL_MS/1000.0f), 0.0f, 1.0f);
    s->gain = HEIGHT_INITIAL_GAIN + (1.0f-HEIGHT_INITIAL_GAIN)*persistence;
    float desired_half = original_half * s->gain;

    /* Oppose actual rotation exceeding the requested rotation, including
     * rotation in the old direction after an optical reversal. A steady
     * bend at/below its requested rate is not continuously weakened.
     * Existing velocity estimation is used; no added ADC/steering filter. */
    float excess_half = measured_half > 0.0f ?
        fmaxf(0.0f, measured_half-fmaxf(0.0f,desired_half)) :
        fminf(0.0f, measured_half-fminf(0.0f,desired_half));
    float damping = clampf(HEIGHT_YAW_DAMPING*excess_half,
                           -HEIGHT_YAW_DAMP_MAX, HEIGHT_YAW_DAMP_MAX);
    s->damp_weight = fabsf(damping)/HEIGHT_YAW_DAMP_MAX;
    desired_half -= damping;

    /* A brief visible-line correction should not halve the forward target.
     * Persistent bends recover the original mean AND full inner-wheel range.
     * This floor never applies to lost-line/coast/STOP/manoeuvre demands. */
    float brief_mean = fminf(ceiling,fmaxf(core_mean,HEIGHT_BRIEF_MEAN_MIN));
    float desired_mean = brief_mean+(core_mean-brief_mean)*persistence;
    float half_limit = HEIGHT_BRIEF_HALF_MAX +
        fmaxf(0.0f,desired_mean-HEIGHT_BRIEF_HALF_MAX)*persistence;
    desired_half = clampf(desired_half,-half_limit,half_limit);
    desired_half = clampf(desired_half, -desired_mean, desired_mean);
    float outer = desired_mean + fabsf(desired_half);
    if (outer > ceiling) desired_mean = fmaxf(0.0f, ceiling - fabsf(desired_half));

    float evidence = clampf(fabsf(error) / 0.70f, 0.0f, 1.0f);
    float steer_rate = 120.0f + 180.0f * evidence;
    /* Release an old correction promptly as the request shrinks/reverses.
     * Use the existing 300mm/s^2 maximum, not an extra smoothing delay. */
    if (fabsf(desired_half)<fabsf(s->half) || desired_half*s->half<0.0f)
        steer_rate=300.0f;
    float half_change = desired_half - s->half;
    s->half += clampf(half_change, -steer_rate * dt, steer_rate * dt);

    float mean_change = desired_mean - s->mean;
    float mean_rate = mean_change >= 0.0f ? 100.0f : 200.0f;
    s->mean += clampf(mean_change, -mean_rate * dt, mean_rate * dt);
    s->mean = fmaxf(0.0f, s->mean);
    s->half = clampf(s->half, -s->mean, s->mean);

    for (unsigned w=0; w<2; ++w) {
        float target = s->mean + (w ? s->half : -s->half);
        target = clampf(target, 0.0f, ceiling);
        s->emitted[w] = (int16_t)lroundf(target);
        d.target[w] = s->emitted[w];
    }
    c->fast = !scanning && (d.target[0] > 55 || d.target[1] > 55);
    d.decision = scanning ? 24 : 25;
    return d;
}

#endif
