/**
 * @file continuous_drive.h
 * @brief Continuous drive.
 * @details Motion helpers run under the coordinator/control ownership contract.
 * Preserve bounded outputs, target sign, stop precedence and request lifetimes.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef CONTINUOUS_DRIVE_H
#define CONTINUOUS_DRIVE_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "src/motion/circuit_wheel.h"
#include "src/motion/drive_mode.h"

enum
{
    CONTINUOUS_PWM_MAX = 420,
    CONTINUOUS_I_MAX = 100,
    CONTINUOUS_START_MAX = 300,
    CONTINUOUS_START_DWELL_US = 100000,
    CONTINUOUS_START_ACTIVE_US = 150000
};

/* Ordinary forward drive state. `duty` is physical PWM permille, while the
 * reference and integral are kept in physical controller units here. */
typedef struct
{
    float duty;
    float reference;
    float integral;
    uint64_t still_since_us;
    uint64_t start_since_us;
    uint8_t active : 1;
    uint8_t still_timing : 1;
    uint8_t start_used : 1;
    uint8_t start_active : 1;
    /* v2.3+Buddy4 ramp-assist v0.1 (only addition to this controller):
     * boost_frac raises the existing feedforward by a fraction, capped at
     * boost_cap PWM; boost_pwm reports what was applied. All three are zero
     * after any reset, so the controller is unchanged unless a caller sets
     * them. (Later ramp-assist versions add boost_add and the hold_* fields
     * below; the 420 clamp, slew and anti-windup logic stay unchanged.) */
    float boost_frac;
    float boost_cap;
    float boost_pwm;
    /* ramp-assist v0.4 stall push: absolute PWM added on top (>= 0), zero
     * after any reset. Same output clamp/slew/anti-windup as everything else.
     */
    float boost_add;
    /* ramp-assist v0.6 speed hold (replaces the v0.5 PWM floor). All zero after
     * any reset; zero = the unchanged controller.
     *  hold_min_speed: the speed target is at least this (mm/s), also when the
     *    steering asks for 0 (continuous drive is only selected without a stop
     *    demand, so 0 here is steering, not a stop);
     *  hold_i_max: integral limit while > CONTINUOUS_I_MAX (e.g. 300);
     *  hold_ki: integral gain multiplier while > 1.
     * Output clamp 0..420, slew 1200/s and anti-windup are unchanged. When the
     * hold is ramped out the integral is clamped back to the active limit. */
    float hold_min_speed;
    float hold_i_max;
    float hold_ki;
    /* Bounded continuous-drive load reserve. It is a single smooth episode
     * per persistent load event, separate from Buddy4 ramp assistance. */
    float load_pwm;
    float load_goal_pwm;
    uint64_t load_since_us;
    uint64_t load_rearm_us;
    uint8_t load_timing : 1;
    uint8_t load_active : 1;
} continuous_drive_t;

static inline float continuous_clampf (float value_x, float low, float high)
{
    return value_x < low ? low : (value_x > high ? high : value_x);
}

/* The captured steady-state values at 45 mm/s were about 150 (left) and 180
 * (right). A straight line through zero gives a smooth provisional map and
 * deliberately does not impose a minimum effort at low speeds. */
static inline float continuous_feedforward (unsigned wheel, float speed)
{
    if ((wheel > 1) || (speed <= 0.0f))
    {
        return 0.0f;
    }
    return speed * (0 == wheel ? (150.0f / 45.0f) : (180.0f / 45.0f));
}

static inline void continuous_drive_reset (continuous_drive_t * p_state,
                                           signed_wheel_t * p_data)
{
    if (p_state)
    {
        *p_state = (continuous_drive_t){0};
    }
    if (p_data)
    {
        *p_data = (signed_wheel_t){0};
    }
}

static inline void continuous_drive_load_step (continuous_drive_t * p_state,
                                               float commanded, float measured,
                                               float delta_time, uint64_t now,
                                               bool eligible)
{
    if (!eligible)
    {
        p_state->load_timing = false;
        p_state->load_active = false;
        p_state->load_goal_pwm = 0.0f;
        p_state->load_pwm = 0.0f;
        return;
    }
    if (measured < 3.0f)
    {
        /* Do not arm reserve from rest. An already-qualified reserve remains
         * bounded while a loaded wheel slows, leaving encoder stall checks in
         * charge of a persistent stop. */
        p_state->load_timing = false;
    }
    else
    {
        float desired = drive_mode_load_pwm_target(commanded, measured);
        if (p_state->load_active)
        {
            if (measured >= commanded * 0.90f)
            {
                p_state->load_active = false;
                p_state->load_timing = false;
                p_state->load_goal_pwm = 0.0f;
                p_state->load_rearm_us = now + DRIVE_MODE_LOAD_REARM_US;
            }
            else
            {
                p_state->load_goal_pwm = desired;
            }
        }
        else if (now < p_state->load_rearm_us)
        {
            p_state->load_timing = false;
        }
        else if (desired > 0.0f)
        {
            if (!p_state->load_timing)
            {
                p_state->load_timing = true;
                p_state->load_since_us = now;
            }
            else if ((now >= p_state->load_since_us) &&
                     (now - p_state->load_since_us >= DRIVE_MODE_LOAD_DWELL_US))
            {
                p_state->load_timing = false;
                p_state->load_active = true;
                p_state->load_goal_pwm = desired;
            }
        }
        else
        {
            p_state->load_timing = false;
        }
    }
    float target = p_state->load_active ? p_state->load_goal_pwm : 0.0f;
    float rate = target >= p_state->load_pwm ? 50.0f : 90.0f;
    p_state->load_pwm += continuous_clampf(
        target - p_state->load_pwm, -rate * delta_time, rate * delta_time);
    p_state->load_pwm = continuous_clampf(p_state->load_pwm, 0.0f,
                                          (float)DRIVE_MODE_LOAD_MAX_PWM);
}

static inline int32_t continuous_drive_step_policy (
    continuous_drive_t * p_state, signed_wheel_t * p_data, unsigned wheel,
    int16_t wanted, float measured, float delta_time, uint64_t now,
    int32_t previous_output, bool load_eligible)
{
    if ((!p_state) || (!p_data))
    {
        return 0;
    }
    const bool hold = p_state->hold_min_speed > 0.0f; /* ramp-assist v0.6 */
    if ((wanted < 0) || (0 == wanted && (!hold)))
    {
        continuous_drive_reset(p_state, p_data);
        return 0;
    }
    if ((((wheel > 1) || (!isfinite(measured))) || (!isfinite(delta_time))) ||
        (delta_time < 0.0f))
    {
        continuous_drive_reset(p_state, p_data);
        return 0;
    }

    /* Keep an unexpectedly late loop from turning a bounded slew into a
     * one-call jump. The outer watchdog remains responsible for hard safety. */
    if (delta_time > 0.1f)
    {
        delta_time = 0.1f;
    }
    float prior = continuous_clampf((float)previous_output, 0.0f,
                                    (float)CONTINUOUS_PWM_MAX);
    float target = (float)wanted;
    if (hold && (target < p_state->hold_min_speed))
    {
        target = p_state->hold_min_speed; /* v0.6 */
    }
    const float i_max = p_state->hold_i_max > (float)CONTINUOUS_I_MAX
                            ? p_state->hold_i_max
                            : (float)CONTINUOUS_I_MAX;
    const float ki_value = p_state->hold_ki > 1.0f ? p_state->hold_ki : 1.0f;

    if (!p_state->active)
    {
        p_state->active = true;
        p_state->duty = prior;
        p_state->reference = continuous_clampf(measured, 0.0f, target);
        float error = p_state->reference - measured;
        float ff_value = continuous_feedforward(wheel, p_state->reference);
        p_state->integral =
            continuous_clampf(prior - ff_value - error, -i_max, i_max);
    }
    else
    {
        /* The adapter may impose a tighter envelope after this function. Use
         * its actual prior output as the anti-windup reference. */
        p_state->duty = prior;
        p_state->integral = continuous_clampf(p_state->integral, -i_max, i_max);
        float delta = target - p_state->reference;
        p_state->reference +=
            continuous_clampf(delta, -250.0f * delta_time, 150.0f * delta_time);
        p_state->reference = fmaxf(0.0f, p_state->reference);
    }

    /* A zero requested wheel stays zero even if Buddy4's independent hold
     * mode is active. That hold policy is applied below; load reserve is not.
     */
    if (wanted > 0)
    {
        continuous_drive_load_step(p_state, p_state->reference, measured,
                                   delta_time, now, load_eligible);
    }
    else
    {
        p_state->load_timing = false;
        p_state->load_active = false;
        p_state->load_goal_pwm = 0.0f;
        p_state->load_pwm = 0.0f;
    }
    float error = p_state->reference - measured;
    float ff_value_value = continuous_feedforward(wheel, p_state->reference);
    p_state->boost_pwm =
        continuous_clampf(ff_value_value * p_state->boost_frac, 0.0f,
                          p_state->boost_cap) +
        (p_state->boost_add > 0.0f ? p_state->boost_add : 0.0f);
    float base =
        ff_value_value + p_state->boost_pwm + error + p_state->load_pwm;

    /* One bounded assistance window per positive-command segment. A stalled
     * wheel cannot re-arm it; target changes alone do not clear this latch. */
    if (((!p_state->start_used) && (p_state->reference >= 20.0f)) &&
        (fabsf(measured) < 3.0f))
    {
        if (!p_state->still_timing)
        {
            p_state->still_timing = true;
            p_state->still_since_us = now;
        }
        else if ((now >= p_state->still_since_us) &&
                 (now - p_state->still_since_us >= CONTINUOUS_START_DWELL_US))
        {
            p_state->start_used = true;
            p_state->start_active = true;
            p_state->start_since_us = now;
        }
    }
    else if (!p_state->start_used)
    {
        p_state->still_timing = false;
    }

    float requested = base + p_state->integral;
    if (p_state->start_active)
    {
        uint64_t age = now >= p_state->start_since_us
                           ? now - p_state->start_since_us
                           : CONTINUOUS_START_ACTIVE_US;
        if (((age >= CONTINUOUS_START_ACTIVE_US) || (target < 20.0f)) ||
            (measured >= fminf(20.0f, 0.8f * p_state->reference)))
        {
            p_state->start_active = false;
        }
        else
        {
            float fraction = (float)age / (float)CONTINUOUS_START_ACTIVE_US;
            float assist = p_state->duty +
                           (CONTINUOUS_START_MAX - p_state->duty) * fraction;
            if (requested < assist)
            {
                requested = assist;
            }
        }
    }
    requested = continuous_clampf(requested, 0.0f, (float)CONTINUOUS_PWM_MAX);

    float max_step = 1200.0f * delta_time;
    float output = p_state->duty + continuous_clampf(requested - p_state->duty,
                                                     -max_step, max_step);
    output = continuous_clampf(output, 0.0f, (float)CONTINUOUS_PWM_MAX);

    /* Track the applied physical output whenever the request is rate-limited,
     * saturated, or being helped. This prevents stored I from fighting the
     * actuator envelope and gives the startup aid a smooth release. */
    float unconstrained = base + p_state->integral;
    bool constrained =
        (p_state->start_active) || (fabsf(output - unconstrained) > 0.5f);
    if (constrained)
    {
        p_state->integral = continuous_clampf(output - base, -i_max, i_max);
    }
    else
    {
        float candidate = continuous_clampf(
            p_state->integral + 3.0f * ki_value * error * delta_time, -i_max,
            i_max);
        float raw = base + candidate;
        if (!(((raw > CONTINUOUS_PWM_MAX) && (error > 0.0f)) ||
              ((raw < 0.0f) && (error < 0.0f))))
        {
            p_state->integral = candidate;
        }
    }

    p_state->duty = output;
    /* Legacy diagnostics are defined before the driver's 1.2 effort scale. */
    p_data->sign = 1;
    p_data->pi.ramp = p_state->reference;
    p_data->pi.integral = p_state->integral / 1.2f;
    p_data->was_turning = false;
    p_data->peak_active = false;
    p_data->peak_used = false;
    p_data->peak_end_us = 0;
    p_data->kick = (line_breakaway_t){0};
    return (int32_t)(output + 0.5f);
}

/* Compatibility facade: existing users retain the pre-v0.2.5 controller.
 * The production line adapter opts into load reserve with the policy call. */
static inline int32_t
continuous_drive_step (continuous_drive_t * p_state, signed_wheel_t * p_data,
                       unsigned wheel, int16_t wanted, float measured,
                       float delta_time, uint64_t now, int32_t previous_output)
{
    return continuous_drive_step_policy(p_state, p_data, wheel, wanted,
                                        measured, delta_time, now,
                                        previous_output, false);
}

#endif

/* End of file. */
