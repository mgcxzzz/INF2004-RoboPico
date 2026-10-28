#ifndef CONTINUOUS_DRIVE_H
#define CONTINUOUS_DRIVE_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "v12_2/baseline/testing/subsystem2/circuit_left_test/circuit_wheel.h"

enum {
    CONTINUOUS_PWM_MAX = 420,
    CONTINUOUS_I_MAX = 100,
    CONTINUOUS_START_MAX = 300,
    CONTINUOUS_START_DWELL_US = 100000,
    CONTINUOUS_START_ACTIVE_US = 150000
};

/* Ordinary forward drive state. `duty` is physical PWM permille, while the
 * reference and integral are kept in physical controller units here. */
typedef struct {
    float duty;
    float reference;
    float integral;
    uint64_t still_since_us;
    uint64_t start_since_us;
    uint8_t active:1;
    uint8_t still_timing:1;
    uint8_t start_used:1;
    uint8_t start_active:1;
} ContinuousDrive;

static inline float continuous_clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

/* The captured steady-state values at 45 mm/s were about 150 (left) and 180
 * (right). A straight line through zero gives a smooth provisional map and
 * deliberately does not impose a minimum effort at low speeds. */
static inline float continuous_feedforward(unsigned wheel, float speed) {
    if (wheel > 1 || speed <= 0.0f) return 0.0f;
    return speed * (wheel == 0 ? (150.0f / 45.0f) : (180.0f / 45.0f));
}

static inline void continuous_drive_reset(ContinuousDrive *s, SignedWheel *p) {
    if (s) *s = (ContinuousDrive){0};
    if (p) *p = (SignedWheel){0};
}

static inline int32_t continuous_drive_step(ContinuousDrive *s, SignedWheel *p,
                                             unsigned wheel, int16_t wanted,
                                             float measured, float dt,
                                             uint64_t now,
                                             int32_t previous_output) {
    if (!s || !p) return 0;
    if (wanted <= 0) {
        continuous_drive_reset(s, p);
        return 0;
    }
    if (wheel > 1 || !isfinite(measured) || !isfinite(dt) || dt < 0.0f) {
        continuous_drive_reset(s, p);
        return 0;
    }

    /* Keep an unexpectedly late loop from turning a bounded slew into a
     * one-call jump. The outer watchdog remains responsible for hard safety. */
    if (dt > 0.1f) dt = 0.1f;
    float prior = continuous_clampf((float)previous_output, 0.0f,
                                    (float)CONTINUOUS_PWM_MAX);
    float target = (float)wanted;

    if (!s->active) {
        s->active = true;
        s->duty = prior;
        s->reference = continuous_clampf(measured, 0.0f, target);
        float error = s->reference - measured;
        float ff = continuous_feedforward(wheel, s->reference);
        s->integral = continuous_clampf(prior - ff - error,
                                        -(float)CONTINUOUS_I_MAX,
                                        (float)CONTINUOUS_I_MAX);
    } else {
        /* The adapter may impose a tighter envelope after this function. Use
         * its actual prior output as the anti-windup reference. */
        s->duty = prior;
        s->integral = continuous_clampf(s->integral,
                                        -(float)CONTINUOUS_I_MAX,
                                        (float)CONTINUOUS_I_MAX);
        float delta = target - s->reference;
        s->reference += continuous_clampf(delta, -250.0f * dt, 150.0f * dt);
        s->reference = fmaxf(0.0f, s->reference);
    }

    float error = s->reference - measured;
    float ff = continuous_feedforward(wheel, s->reference);
    float base = ff + error;

    /* One bounded assistance window per positive-command segment. A stalled
     * wheel cannot re-arm it; target changes alone do not clear this latch. */
    if (!s->start_used && s->reference >= 20.0f && fabsf(measured) < 3.0f) {
        if (!s->still_timing) {
            s->still_timing = true;
            s->still_since_us = now;
        } else if (now >= s->still_since_us &&
                   now - s->still_since_us >= CONTINUOUS_START_DWELL_US) {
            s->start_used = true;
            s->start_active = true;
            s->start_since_us = now;
        }
    } else if (!s->start_used) {
        s->still_timing = false;
    }

    float requested = base + s->integral;
    if (s->start_active) {
        uint64_t age = now >= s->start_since_us ? now - s->start_since_us
                                                : CONTINUOUS_START_ACTIVE_US;
        if (age >= CONTINUOUS_START_ACTIVE_US || target < 20.0f ||
            measured >= fminf(20.0f, 0.8f * s->reference)) {
            s->start_active = false;
        } else {
            float fraction = (float)age / (float)CONTINUOUS_START_ACTIVE_US;
            float assist = s->duty + (CONTINUOUS_START_MAX - s->duty) * fraction;
            if (requested < assist) requested = assist;
        }
    }
    requested = continuous_clampf(requested, 0.0f,
                                   (float)CONTINUOUS_PWM_MAX);

    float max_step = 1200.0f * dt;
    float output = s->duty + continuous_clampf(requested - s->duty,
                                               -max_step, max_step);
    output = continuous_clampf(output, 0.0f,
                               (float)CONTINUOUS_PWM_MAX);

    /* Track the applied physical output whenever the request is rate-limited,
     * saturated, or being helped. This prevents stored I from fighting the
     * actuator envelope and gives the startup aid a smooth release. */
    float unconstrained = base + s->integral;
    bool constrained = s->start_active || fabsf(output - unconstrained) > 0.5f;
    if (s->start_active || constrained) {
        s->integral = continuous_clampf(output - base,
                                        -(float)CONTINUOUS_I_MAX,
                                        (float)CONTINUOUS_I_MAX);
    } else {
        float candidate = continuous_clampf(s->integral + 3.0f * error * dt,
                                            -(float)CONTINUOUS_I_MAX,
                                            (float)CONTINUOUS_I_MAX);
        float raw = base + candidate;
        if (!((raw > CONTINUOUS_PWM_MAX && error > 0.0f) ||
              (raw < 0.0f && error < 0.0f))) s->integral = candidate;
    }

    s->duty = output;
    /* Legacy diagnostics are defined before the driver's 1.2 effort scale. */
    p->sign = 1;
    p->pi.ramp = s->reference;
    p->pi.integral = s->integral / 1.2f;
    p->was_turning = false;
    p->peak_active = false;
    p->peak_used = false;
    p->peak_end_us = 0;
    p->kick = (LineBreakaway){0};
    return (int32_t)(output + 0.5f);
}

#endif
