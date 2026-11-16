/**
 * @file continuous_steering.h
 * @brief Continuous steering.
 * @details The control task owns mutable steering/recovery state.
 * Keep straight, scan, turn and recovery behavior distinct; no direct PWM here.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef CONTINUOUS_STEERING_H
#define CONTINUOUS_STEERING_H

#include <math.h>
#include "src/barcode/barcode_nav.h"
#include "src/motion/drive_mode.h"

/* Opt in for the provisional, sensor-qualified bend assist in production.
 * Default-off preserves the original Continuous V2 output path and baseline
 * host/SDK builds until the production profile explicitly enables it. */
#ifndef CONTINUOUS_BEND_ASSIST
#define CONTINUOUS_BEND_ASSIST 0
#endif

/* Provisional normalized thresholds and persistence limits. These are
 * software bounds for a first controlled trial, not hardware-tuned values. */
#define CONTINUOUS_BEND_ENTER_ERROR 0.22f
#define CONTINUOUS_BEND_ENTER_CURVATURE 0.18f
#define CONTINUOUS_BEND_EXIT_ERROR 0.14f
#define CONTINUOUS_BEND_EXIT_CURVATURE 0.10f
#define CONTINUOUS_BEND_CANDIDATE_S 0.10f
#define CONTINUOUS_BEND_CONFIRM_S 0.18f
#define CONTINUOUS_BEND_YAW_S 0.08f
#define CONTINUOUS_BEND_DROPOUT_S 0.08f
#define CONTINUOUS_BEND_BLEND_S 0.15f
#define CONTINUOUS_BEND_SPEED_SCALE 0.80f
#define CONTINUOUS_BEND_AUTHORITY_SCALE 1.10f

/* Output-only steering adaptation. The optical tracker and barcode reader
 * continue to see the original ADC samples and make the original decisions. */
typedef struct
{
    float mean;
    float half;
    int16_t emitted[2];
    float previous_error;
    float return_confidence;
    float same_side_duration;
    float gain;
    float damp_weight;
    /* Opt-in bend assist state. Durations and scales are finite, bounded
     * values; bitfields make the state visible to compact diagnostics. */
    float bend_demand_duration;
    float bend_yaw_duration;
    float bend_exit_duration;
    float bend_speed_scale;
    float bend_authority_scale;
    uint8_t side : 2;
    uint8_t active : 1;
    uint8_t bend_candidate : 1;
    uint8_t bend_confirmed : 1;
    uint8_t bend_exiting : 1;
    int8_t bend_direction : 2;
} continuous_steering_t;

static inline void continuous_steering_reset (continuous_steering_t * p_state)
{
    *p_state = (continuous_steering_t){0};
}

static inline float continuous_steering_finite (float value_x, float fallback)
{
    return isfinite(value_x) ? value_x : fallback;
}

#if CONTINUOUS_BEND_ASSIST
static inline float continuous_steering_approach (float value, float target,
                                                  float step)
{
    float delta = target - value;
    return value + clampf(delta, -step, step);
}

/* A bend may begin from optical evidence alone, but extra steering authority
 * requires persistent correctly signed encoder yaw as well. The encoder is
 * never allowed to create or retain a bend without the optical request. */
static inline void
continuous_steering_update_bend (continuous_steering_t * p_state, float error,
                                 float original_half, float desired_mean,
                                 float measured_half, float measured_mean,
                                 float delta_time)
{
    error = clampf(error, -1.0f, 1.0f);
    float curvature = fabsf(original_half) / fmaxf(desired_mean, 10.0f);
    curvature = clampf(curvature, 0.0f, 1.0f);
    int8_t error_direction = error > 0.0f ? 1 : error < 0.0f ? -1 : 0;
    int8_t request_direction = original_half > 0.0f   ? 1
                               : original_half < 0.0f ? -1
                                                      : 0;
    bool aligned = error_direction != 0 && error_direction == request_direction;
    bool enter_signal = aligned &&
                        fabsf(error) >= CONTINUOUS_BEND_ENTER_ERROR &&
                        curvature >= CONTINUOUS_BEND_ENTER_CURVATURE;
    bool hold_signal = aligned && fabsf(error) >= CONTINUOUS_BEND_EXIT_ERROR &&
                       curvature >= CONTINUOUS_BEND_EXIT_CURVATURE;

    /* Handles callers that seed the legacy state directly in a test or
     * restore it from an older zero-initialized application snapshot. */
    if (!(p_state->bend_speed_scale > 0.0f) ||
        !isfinite(p_state->bend_speed_scale))
    {
        p_state->bend_speed_scale = 1.0f;
    }
    if (!(p_state->bend_authority_scale > 0.0f) ||
        !isfinite(p_state->bend_authority_scale))
    {
        p_state->bend_authority_scale = 1.0f;
    }
    p_state->bend_speed_scale =
        clampf(p_state->bend_speed_scale, CONTINUOUS_BEND_SPEED_SCALE, 1.0f);
    p_state->bend_authority_scale = clampf(p_state->bend_authority_scale, 1.0f,
                                           CONTINUOUS_BEND_AUTHORITY_SCALE);

    bool candidate_just_started = false;
    if (!p_state->bend_candidate)
    {
        if (enter_signal)
        {
            if (p_state->bend_direction != error_direction)
            {
                p_state->bend_direction = error_direction;
                p_state->bend_demand_duration = 0.0f;
                p_state->bend_yaw_duration = 0.0f;
            }
            p_state->bend_demand_duration =
                fminf(.50f, p_state->bend_demand_duration + delta_time);
            if (p_state->bend_demand_duration >= CONTINUOUS_BEND_CANDIDATE_S)
            {
                p_state->bend_candidate = 1;
                candidate_just_started = true;
                p_state->bend_exiting = 0;
                p_state->bend_exit_duration = 0.0f;
            }
        }
        else
        {
            p_state->bend_demand_duration = 0.0f;
            p_state->bend_yaw_duration = 0.0f;
        }
    }

    if (p_state->bend_candidate)
    {
        if (hold_signal && error_direction == p_state->bend_direction)
        {
            p_state->bend_exit_duration = 0.0f;
            p_state->bend_exiting = 0;
            if (!candidate_just_started)
            {
                p_state->bend_demand_duration =
                    fminf(.50f, p_state->bend_demand_duration + delta_time);
            }

            float yaw_ratio = p_state->bend_direction * measured_half /
                              fmaxf(fabsf(measured_mean), 10.0f);
            bool correct_yaw = measured_mean >= 8.0f && yaw_ratio >= .08f;
            if (correct_yaw)
            {
                p_state->bend_yaw_duration =
                    fminf(.50f, p_state->bend_yaw_duration + delta_time);
            }
            else
            {
                p_state->bend_yaw_duration = 0.0f;
            }
            if (p_state->bend_demand_duration >= CONTINUOUS_BEND_CONFIRM_S &&
                p_state->bend_yaw_duration >= CONTINUOUS_BEND_YAW_S)
            {
                p_state->bend_confirmed = 1;
            }
        }
        else
        {
            p_state->bend_exiting = 1;
            p_state->bend_exit_duration =
                fminf(.50f, p_state->bend_exit_duration + delta_time);
            if (p_state->bend_exit_duration >= CONTINUOUS_BEND_DROPOUT_S)
            {
                p_state->bend_candidate = 0;
                p_state->bend_confirmed = 0;
                p_state->bend_demand_duration = 0.0f;
                p_state->bend_yaw_duration = 0.0f;
            }
        }
    }

    float speed_target =
        p_state->bend_candidate ? CONTINUOUS_BEND_SPEED_SCALE : 1.0f;
    float authority_target =
        p_state->bend_confirmed ? CONTINUOUS_BEND_AUTHORITY_SCALE : 1.0f;
    float blend_step = delta_time / CONTINUOUS_BEND_BLEND_S;
    p_state->bend_speed_scale = continuous_steering_approach(
        p_state->bend_speed_scale, speed_target, blend_step * .20f);
    p_state->bend_authority_scale = continuous_steering_approach(
        p_state->bend_authority_scale, authority_target, blend_step * .10f);
    p_state->bend_speed_scale =
        clampf(p_state->bend_speed_scale, CONTINUOUS_BEND_SPEED_SCALE, 1.0f);
    p_state->bend_authority_scale = clampf(p_state->bend_authority_scale, 1.0f,
                                           CONTINUOUS_BEND_AUTHORITY_SCALE);

    /* Pending sensor dwell retains its direction until qualification. Clear
     * it only after both the dwell and any exit blend have ended. */
    if (!p_state->bend_candidate && p_state->bend_demand_duration <= 0.0f &&
        p_state->bend_speed_scale >= 0.9999f &&
        p_state->bend_authority_scale <= 1.0001f)
    {
        p_state->bend_speed_scale = 1.0f;
        p_state->bend_authority_scale = 1.0f;
        p_state->bend_exiting = 0;
        p_state->bend_direction = 0;
        p_state->bend_exit_duration = 0.0f;
    }
    else if (!p_state->bend_candidate &&
             (p_state->bend_speed_scale < 0.9999f ||
              p_state->bend_authority_scale > 1.0001f))
    {
        p_state->bend_exiting = 1;
    }
}
#endif

static inline analog_demand_t continuous_steering_step_with_cruise (
    continuous_steering_t * p_state, analog_demand_t demand,
    circuit_t * p_character, const circuit_input_t * p_index, bool scanning,
    unsigned profile, float delta_time, unsigned cruise_speed_mm_s)
{
    bool ordinary = demand.decision == SYM_STRAIGHT ||
                    demand.decision == SYM_LEFT ||
                    demand.decision == SYM_RIGHT || demand.decision == C_FAST;
    bool eligible = p_character->phase == C_FOLLOW && !p_character->busy &&
                    !demand.stop && !p_character->follower.core.white &&
                    ordinary;
    if (!eligible)
    {
        continuous_steering_reset(p_state);
        return demand;
    }

    delta_time = continuous_steering_finite(delta_time, 0.0f);
    delta_time = clampf(delta_time, 0.0f, 0.05f);
    float desired_left = (float)demand.target[0];
    float desired_right = (float)demand.target[1];
    float desired_mean = (desired_left + desired_right) * 0.5f;
    float original_half = (desired_right - desired_left) * 0.5f;
    float error = ((float)demand.level[0] - (float)demand.level[1]) / 1000.0f;
    float measured_left = continuous_steering_finite(p_index->speed[0], 0.0f);
    float measured_right = continuous_steering_finite(p_index->speed[1], 0.0f);
    float measured_half = (measured_right - measured_left) * 0.5f;
#if CONTINUOUS_BEND_ASSIST
    float measured_mean = (measured_left + measured_right) * 0.5f;
#endif

    /* Runtime speed modes opt into higher ordinary cruise only. A zero or
     * near-zero inside wheel marks a correcting turn and keeps the reference
     * profile's targets unchanged. Barcode activity also retains45mm/s. */
    bool scaled_cruise = profile == 1 && !scanning && cruise_speed_mm_s > 45U &&
                         demand.target[0] > 10 && demand.target[1] > 10;
    float ceiling = scanning        ? 45.0f
                    : profile == 3  ? 135.0f
                    : scaled_cruise ? (float)cruise_speed_mm_s
                                    : 55.0f;
    desired_mean = clampf(desired_mean, 0.0f, ceiling);
    original_half = clampf(original_half, -desired_mean, desired_mean);
    if (scaled_cruise)
    {
        float scale = (float)cruise_speed_mm_s / 45.0f;
        desired_mean = clampf(desired_mean * scale, 0.0f, ceiling);
        original_half *= scale;
        original_half = clampf(original_half, -desired_mean, desired_mean);
    }

    bool entering = !p_state->active;
    if (entering)
    {
        float measured_mean =
            fmaxf(0.0f, (measured_left + measured_right) * 0.5f);
        p_state->mean = fminf(measured_mean, ceiling);
        p_state->half = 0.0f;
        p_state->emitted[0] = p_state->emitted[1] =
            (int16_t)lroundf(p_state->mean);
        p_state->previous_error = error; /* no derivative kick on entry */
        p_state->return_confidence = 0.0f;
        p_state->same_side_duration = 0.0f;
        p_state->side = original_half > 0.0f   ? 1u
                        : original_half < 0.0f ? 2u
                                               : 0u;
        p_state->gain = 0.70f;
        p_state->damp_weight = 0.0f;
        p_state->bend_speed_scale = 1.0f;
        p_state->bend_authority_scale = 1.0f;
        p_state->active = true;
    }

#if CONTINUOUS_BEND_ASSIST
    continuous_steering_update_bend(p_state, error, original_half, desired_mean,
                                    measured_half, measured_mean, delta_time);
    desired_mean *= p_state->bend_speed_scale;
#endif

    /* Dwell tracks the core's actual signed steering request. A residual
     * sensor imbalance inside the core's straight dead zone has no half-turn
     * authority and must not earn full gain. */
    uint8_t side = original_half > 0.0f ? 1u : original_half < 0.0f ? 2u : 0u;
    if (!entering && side != 0u && side == p_state->side)
    {
        p_state->same_side_duration =
            fminf(0.35f, p_state->same_side_duration + delta_time);
    }
    else if (side != p_state->side || side == 0u)
    {
        p_state->same_side_duration = 0.0f;
    }
    p_state->side = side;
    p_state->gain =
        0.70f + 0.30f * clampf(p_state->same_side_duration / 0.35f, 0.0f, 1.0f);
    float desired_half = original_half * p_state->gain;
#if CONTINUOUS_BEND_ASSIST
    desired_half *= p_state->bend_authority_scale;
#endif

    /* Scale return confidence by the normalized calibrated-error fall rate.
     * The 40ms EMA smooths it; the half slew already bounds output changes. */
    float abs_error = fabsf(error);
    float fall_rate =
        delta_time > 0.0f
            ? fmaxf(0.0f,
                    (fabsf(p_state->previous_error) - abs_error) / delta_time)
            : 0.0f;
    float confidence_target = clampf(fall_rate / 4.0f, 0.0f, 1.0f);
    if (original_half == 0.0f)
    {
        confidence_target = 1.0f;
    }
    p_state->return_confidence +=
        (confidence_target - p_state->return_confidence) * delta_time /
        (0.040f + delta_time);
    p_state->return_confidence = clampf(p_state->return_confidence, 0.0f, 1.0f);
    p_state->damp_weight = p_state->return_confidence;
    float damping =
        clampf(0.35f * measured_half * p_state->damp_weight, -7.0f, 7.0f);
    desired_half -= damping;
    p_state->previous_error = error;

    /* The original bend request regains all authority after 350ms on the same
     * side. Damping fades as the optical error stops shrinking. */
    desired_half = clampf(desired_half, -desired_mean, desired_mean);
    float outer = desired_mean + fabsf(desired_half);
    if (outer > ceiling)
    {
        desired_mean = fmaxf(0.0f, ceiling - fabsf(desired_half));
    }

    float evidence = clampf(fabsf(error) / 0.70f, 0.0f, 1.0f);
    float steer_rate = 120.0f + 180.0f * evidence;
    float half_change = desired_half - p_state->half;
    p_state->half +=
        clampf(half_change, -steer_rate * delta_time, steer_rate * delta_time);

    float mean_change = desired_mean - p_state->mean;
    float mean_rate = mean_change >= 0.0f ? 100.0f : 200.0f;
    p_state->mean +=
        clampf(mean_change, -mean_rate * delta_time, mean_rate * delta_time);
    p_state->mean = fmaxf(0.0f, p_state->mean);
    p_state->half = clampf(p_state->half, -p_state->mean, p_state->mean);

    for (unsigned wheel = 0; wheel < 2; ++wheel)
    {
        float target = p_state->mean + (wheel ? p_state->half : -p_state->half);
        target = clampf(target, 0.0f, 135.0f);
        p_state->emitted[wheel] = (int16_t)lroundf(target);
        demand.target[wheel] = p_state->emitted[wheel];
    }
    p_character->fast =
        !scanning && (demand.target[0] > 55 || demand.target[1] > 55);
    demand.decision = scanning ? 24 : 25;
    return demand;
}

/* Existing callers keep the profile's historical speeds by default. */
static inline analog_demand_t
continuous_steering_step (continuous_steering_t * p_state,
                          analog_demand_t demand, circuit_t * p_character,
                          const circuit_input_t * p_index, bool scanning,
                          unsigned profile, float delta_time)
{
    return continuous_steering_step_with_cruise(
        p_state, demand, p_character, p_index, scanning, profile, delta_time,
        DRIVE_MODE_BASELINE_SPEED);
}

#endif

/* End of file. */
