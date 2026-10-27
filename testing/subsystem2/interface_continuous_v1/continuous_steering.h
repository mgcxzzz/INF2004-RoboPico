#ifndef CONTINUOUS_STEERING_H
#define CONTINUOUS_STEERING_H

#include "v12_2/baseline/testing/subsystem2/circuit_with_barcode/barcode_nav.h"

/* Output-only adapter for ordinary visible line following.  The barcode and
 * circuit tracker continue to receive every unmodified optical sample. */
typedef struct {
    float mean, half;
    int16_t emitted[2];
    uint8_t active:1;
} ContinuousSteering;

static inline void continuous_steering_reset(ContinuousSteering *s) {
    *s = (ContinuousSteering){0};
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

    dt = clampf(dt, 0.0f, 0.05f);
    float desired_left = (float)d.target[0];
    float desired_right = (float)d.target[1];
    float desired_mean = (desired_left + desired_right) * 0.5f;
    float desired_half = (desired_right - desired_left) * 0.5f;

    /* Profile 1 remains a modest cruise. Profile 3 keeps the established
     * cruise target, with the same continuous steering response as other
     * visible corrections. */
    float ceiling = scanning ? 45.0f : profile == 3 ? 135.0f : 55.0f;
    if (ceiling > 135.0f) ceiling = 135.0f;
    desired_mean = clampf(desired_mean, 0.0f, ceiling);
    desired_half = clampf(desired_half, -desired_mean, desired_mean);
    /* Constrain the outer wheel to the applicable ceiling while preserving
     * the original differential request and its eventual zero inner wheel. */
    float outer = desired_mean + fabsf(desired_half);
    if (outer > ceiling) desired_mean = fmaxf(0.0f, ceiling - fabsf(desired_half));

    if (!s->active) {
        /* Seed from measured motion to avoid a target step when the adapter is
         * first enabled. The ordinary wheel controller still owns its ramp. */
        float measured = fmaxf(0.0f, (i->speed[0] + i->speed[1]) * 0.5f);
        s->mean = fminf(measured, ceiling);
        s->half = 0.0f;
        s->emitted[0] = s->emitted[1] = (int16_t)lroundf(s->mean);
        s->active = true;
    }

    /* Differential slew is gradual for small errors and speeds up smoothly
     * as optical evidence grows. There is no urgent/bypass threshold. */
    float evidence = clampf((float)abs((int)d.level[0] - (int)d.level[1]) / 700.0f,
                            0.0f, 1.0f);
    float steer_rate = 120.0f + 180.0f * evidence;
    float half_change = desired_half - s->half;
    s->half += clampf(half_change, -steer_rate * dt, steer_rate * dt);

    /* Common speed has its own asymmetric acceleration limits. A falling
     * ceiling is approached at the same bounded deceleration rate. */
    float mean_change = desired_mean - s->mean;
    float mean_rate = mean_change >= 0.0f ? 100.0f : 200.0f;
    s->mean += clampf(mean_change, -mean_rate * dt, mean_rate * dt);
    s->mean = fmaxf(0.0f, s->mean);
    /* Keep the stored differential feasible at low common speed, rather than
     * hiding a negative wheel command behind the final output clamp. */
    s->half = clampf(s->half, -s->mean, s->mean);

    for (unsigned w = 0; w < 2; ++w) {
        float target = s->mean + (w ? s->half : -s->half);
        /* Apply the hard safety range only. The current profile/scan ceiling
         * shapes the state target above and is reached through the rate
         * limits, so changing ceilings cannot clip persisted wheel commands. */
        target = clampf(target, 0.0f, 135.0f);
        s->emitted[w] = (int16_t)lroundf(target);
        d.target[w] = s->emitted[w];
    }
    c->fast = !scanning && (d.target[0] > 55 || d.target[1] > 55);
    d.decision = scanning ? 24 : 25;
    return d;
}

#endif
