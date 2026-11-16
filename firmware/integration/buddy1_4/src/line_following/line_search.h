/**
 * @file line_search.h
 * @brief Line search.
 * @details The control task owns mutable steering/recovery state.
 * Keep straight, scan, turn and recovery behavior distinct; no direct PWM here.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BARCODE_LINE_SEARCH_H
#define BARCODE_LINE_SEARCH_H
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "src/motion/dual_logic.h"

enum
{
    LS_CONFIRM_US = 2000000,
    LS_BRIDGE_US = 150000,
    LS_ALIGN_US = 9000000,
    LS_ALIGN_MM = 360,
    LS_TOTAL_US = 5250000,
    LS_MAX_MM = 300,
    LS_SPEED = 90,
    LS_CONFIRM_SPEED = 45,
    LS_TIMEOUT = 47,
    LS_DISTANCE = 48,
    LS_ANGLE = 49,
    LS_STALL = 50,
    LS_SETTLE_TIMEOUT = 51,
    LS_SWEEPS_EXHAUSTED = 52,
    LS_INPUT = 53,
    LS_DECISION_FIRST = 19
};
enum
{
    LS_PHASE_IDLE = 0,
    LS_PHASE_SETTLE_START = 1,
    LS_PHASE_SWEEP = 2,
    LS_PHASE_SETTLE_REVERSAL = 3,
    LS_PHASE_SETTLE_FOUND = 4,
    LS_PHASE_SETTLE_RESUME = 5,
    LS_PHASE_CONFIRMING = 6,
    LS_PHASE_WHITE_WAIT = 7,
    LS_MAX_PATH_MM = 120,
    LS_MAX_ANGLE_DEG = 25,
    LS_SMALL_ANGLE_DEG = 10,
    LS_WIDE_ANGLE_DEG = 20,
    LS_STATIONARY_SPEED = 24,
    LS_STILL_US = 80000,
    LS_SETTLE_LIMIT_US = 600000,
    LS_STALL_US = 1200000,
    LS_STILL_DELTA_MM_X100 = 2
};
typedef struct
{
    uint64_t since;
    uint64_t wait_since;
    uint64_t settle_since;
    uint64_t progress_since;
    uint64_t still_since;
    uint32_t mm;
    signed first : 2;
    unsigned phase : 3;
    unsigned stage : 3;
    double origin_wheel[2];
    double last_wheel[2];
    double progress_wheel[2];
    float relative_yaw_deg;
    float target_yaw_deg;
    unsigned active : 1;
    unsigned waiting : 1;
    unsigned still_timing : 1;
    unsigned wheel_valid : 1;
    unsigned progress_valid : 1;
    unsigned found_candidate : 1;
} line_search_t;
typedef struct
{
    int16_t target[2];
    unsigned stage : 3;
    unsigned phase : 3;
    unsigned reason : 6;
    unsigned complete : 1;
} line_search_demand_t;
static inline int line_search_outer (bool active, int left, int right)
{
    return !active                  ? -1
           : left == 0 && right > 0 ? 1
           : right == 0 && left > 0 ? 0
                                    : -1;
}
/* Count a continuous both-white interval, with a150..199 hysteresis band.
 * A credible return resets this pre-search timer; once sweeping, flashes
 * cannot renew its separate time/distance budget. */
static inline bool line_search_ready (line_search_t * p_state, uint64_t now,
                                      uint16_t left, uint16_t right)
{
    if (p_state->active)
    {
        return true;
    }
    if (left >= 200 || right >= 200)
    {
        p_state->waiting = false;
        p_state->phase = LS_PHASE_IDLE;
        return false;
    }
    if (!p_state->waiting)
    {
        if (left >= 150 || right >= 150)
        {
            return false;
        }
        p_state->waiting = true;
        p_state->phase = LS_PHASE_WHITE_WAIT;
        p_state->wait_since = now;
    }
    return now - p_state->wait_since >= LS_CONFIRM_US;
}
static inline void line_search_begin (line_search_t * p_state, uint64_t now,
                                      uint32_t mm_value, int direction)
{
    *p_state = (line_search_t){.since = now,
                               .mm = mm_value,
                               .first = direction < 0 ? -1 : 1,
                               .active = true};
}
/* Forward-only arcs: coast one wheel and drive the other. Their directions
 * alternate;350/700/1050/1400/1750ms durations widen the angular sweep if
 * motion is comparable in both directions. This does not claim a stationary
 * pivot or an encoder-guaranteed angle. Time also expires with zero ticks. */
static inline line_search_demand_t
line_search_step (const line_search_t * p_state, uint64_t now,
                  uint32_t mm_value, bool confirming)
{
    line_search_demand_t demand = {0};
    uint64_t elapsed = now - p_state->since;
    if (elapsed >= LS_TOTAL_US)
    {
        demand.reason = LS_TIMEOUT;
        return demand;
    }
    if (mm_value - p_state->mm >= LS_MAX_MM)
    {
        demand.reason = LS_DISTANCE;
        return demand;
    }
    uint64_t end = 350000;
    while (demand.stage < 4 && elapsed >= end)
    {
        ++demand.stage;
        end += (uint64_t)(demand.stage + 1) * 350000;
    }
    int direction = demand.stage % 2 ? -p_state->first : p_state->first;
    demand.target[direction > 0 ? 1 : 0] =
        confirming ? LS_CONFIRM_SPEED : LS_SPEED;
    return demand;
}

/* Production-only stationary search. Encoder coordinates use the same
 * forward-positive millimetres as CircuitInput.wheel; positive `first` keeps
 * the established left-recovery convention (-left,+right). Every change of
 * wheel direction crosses a zero-target, encoder-still dwell. */
static inline float line_search_target_yaw (const line_search_t * p_state)
{
    int direction = (p_state->stage & 1U) ? -p_state->first : p_state->first;
    int degrees = p_state->stage < 2 ? LS_SMALL_ANGLE_DEG : LS_WIDE_ANGLE_DEG;
    /* heading_deg() uses left-minus-right, so this sign maps a left pivot to
       a negative observed yaw while preserving the optical recovery sign. */
    return (float)(-direction * degrees);
}
static inline void line_search_settle (line_search_t * p_state, uint8_t phase,
                                       uint64_t now)
{
    p_state->phase = phase;
    p_state->settle_since = now;
    p_state->still_timing = false;
}
static inline void line_search_stationary_begin (line_search_t * p_state,
                                                 uint64_t now,
                                                 uint32_t mm_value,
                                                 int direction,
                                                 const double wheel[2])
{
    /* With no directional memory, left is only the deterministic first leg;
       the subsequent bounded sweep covers the opposite side as well. */
    *p_state = (line_search_t){.since = now,
                               .mm = mm_value,
                               .first = direction < 0 ? -1 : 1,
                               .phase = LS_PHASE_SETTLE_START,
                               .active = true,
                               .wheel_valid = wheel && isfinite(wheel[0]) &&
                                              isfinite(wheel[1]),
                               .progress_valid = true};
    if (!p_state->wheel_valid)
    {
        return; /* The first step reports LS_INPUT without dereferencing pose. */
    }
    p_state->origin_wheel[0] = p_state->last_wheel[0] =
        p_state->progress_wheel[0] = wheel[0];
    p_state->origin_wheel[1] = p_state->last_wheel[1] =
        p_state->progress_wheel[1] = wheel[1];
    p_state->target_yaw_deg = line_search_target_yaw(p_state);
    p_state->settle_since = p_state->progress_since = now;
}
static inline bool line_search_at_target (const line_search_t * p_state)
{
    return p_state->target_yaw_deg >= 0
               ? p_state->relative_yaw_deg >= p_state->target_yaw_deg
               : p_state->relative_yaw_deg <= p_state->target_yaw_deg;
}
static inline line_search_demand_t line_search_stationary_step (
    line_search_t * p_state, uint64_t now, uint32_t mm_value,
    const double wheel[2], const float speed[2], bool candidate, bool confirmed)
{
    line_search_demand_t demand = {.stage = p_state->stage,
                                   .phase = p_state->phase};
    if (!p_state->active)
    {
        return demand;
    }
    if (!p_state->wheel_valid || !wheel || !speed || !isfinite(wheel[0]) ||
        !isfinite(wheel[1]) || !isfinite(speed[0]) || !isfinite(speed[1]))
    {
        demand.reason = LS_INPUT;
        return demand;
    }
    uint64_t elapsed = now - p_state->since;
    if (elapsed >= LS_TOTAL_US)
    {
        demand.reason = LS_TIMEOUT;
        return demand;
    }
    if (mm_value - p_state->mm >= LS_MAX_PATH_MM)
    {
        demand.reason = LS_DISTANCE;
        return demand;
    }
    p_state->relative_yaw_deg =
        heading_deg((float)(wheel[0] - p_state->origin_wheel[0]),
                    (float)(wheel[1] - p_state->origin_wheel[1]));
    if (fabsf(p_state->relative_yaw_deg) > LS_MAX_ANGLE_DEG)
    {
        demand.reason = LS_ANGLE;
        return demand;
    }
    double delta_left = wheel[0] - p_state->last_wheel[0];
    double delta_right = wheel[1] - p_state->last_wheel[1];
    bool ticked = fabs(delta_left) > LS_STILL_DELTA_MM_X100 / 100.0 ||
                  fabs(delta_right) > LS_STILL_DELTA_MM_X100 / 100.0;
    p_state->last_wheel[0] = wheel[0];
    p_state->last_wheel[1] = wheel[1];
    bool stopped = fabsf(speed[0]) < 5.0f && fabsf(speed[1]) < 5.0f && !ticked;

    if (p_state->phase == LS_PHASE_SWEEP && ticked)
    {
        p_state->progress_wheel[0] = wheel[0];
        p_state->progress_wheel[1] = wheel[1];
        p_state->progress_since = now;
    }
    else if (p_state->phase == LS_PHASE_SWEEP && p_state->progress_valid &&
             now - p_state->progress_since >= LS_STALL_US)
    {
        demand.reason = LS_STALL;
        return demand;
    }
    if (p_state->phase == LS_PHASE_SWEEP && candidate)
    {
        p_state->found_candidate = false;
        line_search_settle(p_state, LS_PHASE_CONFIRMING, now);
    }
    else if (p_state->phase == LS_PHASE_SWEEP && line_search_at_target(p_state))
    {
        line_search_settle(p_state, LS_PHASE_SETTLE_REVERSAL, now);
    }
    else if ((p_state->phase == LS_PHASE_SETTLE_START ||
              p_state->phase == LS_PHASE_SETTLE_REVERSAL ||
              p_state->phase == LS_PHASE_SETTLE_RESUME) &&
             candidate)
    {
        p_state->found_candidate = false;
        line_search_settle(
            p_state, confirmed ? LS_PHASE_SETTLE_FOUND : LS_PHASE_CONFIRMING,
            now);
    }
    else if (p_state->phase == LS_PHASE_CONFIRMING && !candidate)
    {
        p_state->found_candidate = false;
        line_search_settle(p_state, LS_PHASE_SETTLE_RESUME, now);
    }
    else if (p_state->phase == LS_PHASE_CONFIRMING && confirmed)
    {
        p_state->found_candidate = true;
        line_search_settle(p_state, LS_PHASE_SETTLE_FOUND, now);
    }
    else if (p_state->phase == LS_PHASE_SETTLE_FOUND && !candidate)
    {
        p_state->found_candidate = false;
        line_search_settle(p_state, LS_PHASE_SETTLE_RESUME, now);
    }
    else if (p_state->phase == LS_PHASE_SETTLE_RESUME && confirmed)
    {
        p_state->found_candidate = true;
        line_search_settle(p_state, LS_PHASE_SETTLE_FOUND, now);
    }

    if (p_state->phase == LS_PHASE_SETTLE_START ||
        p_state->phase == LS_PHASE_SETTLE_REVERSAL ||
        p_state->phase == LS_PHASE_SETTLE_FOUND ||
        p_state->phase == LS_PHASE_SETTLE_RESUME ||
        p_state->phase == LS_PHASE_CONFIRMING)
    {
        if (now - p_state->settle_since >= LS_SETTLE_LIMIT_US &&
            (!p_state->still_timing ||
             now - p_state->still_since < LS_STILL_US))
        {
            demand.reason = LS_SETTLE_TIMEOUT;
            return demand;
        }
        if (stopped)
        {
            if (!p_state->still_timing)
            {
                p_state->still_timing = true;
                p_state->still_since = now;
            }
            if (now - p_state->still_since >= LS_STILL_US)
            {
                if (p_state->phase == LS_PHASE_SETTLE_FOUND &&
                    p_state->found_candidate)
                {
                    demand.complete = true;
                    demand.phase = p_state->phase;
                    return demand;
                }
                if (p_state->phase == LS_PHASE_CONFIRMING)
                {
                    /* Candidate is still being qualified; remain stopped. */
                }
                else if (p_state->phase == LS_PHASE_SETTLE_START)
                {
                    p_state->target_yaw_deg = line_search_target_yaw(p_state);
                    p_state->phase = LS_PHASE_SWEEP;
                    p_state->progress_since = now;
                    p_state->progress_valid = true;
                }
                else if (line_search_at_target(p_state))
                {
                    if (++p_state->stage >= 4)
                    {
                        demand.reason = LS_SWEEPS_EXHAUSTED;
                        return demand;
                    }
                    p_state->target_yaw_deg = line_search_target_yaw(p_state);
                    p_state->phase = LS_PHASE_SWEEP;
                    p_state->progress_since = now;
                }
                else
                {
                    p_state->phase = LS_PHASE_SWEEP;
                    p_state->progress_since = now;
                }
                p_state->progress_wheel[0] = wheel[0];
                p_state->progress_wheel[1] = wheel[1];
                p_state->still_timing = false;
            }
        }
        else
        {
            p_state->still_timing = false;
        }
    }
    if (p_state->phase == LS_PHASE_SWEEP)
    {
        int direction =
            (p_state->stage & 1U) ? -p_state->first : p_state->first;
        demand.target[0] = (int16_t)(-direction * LS_STATIONARY_SPEED);
        demand.target[1] = (int16_t)(direction * LS_STATIONARY_SPEED);
    }
    demand.stage = p_state->stage;
    demand.phase = p_state->phase;
    return demand;
}
#endif

/* End of file. */
