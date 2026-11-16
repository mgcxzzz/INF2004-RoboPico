/**
 * @file circuit_recovery.h
 * @brief Circuit recovery.
 * @details The control task owns mutable steering/recovery state.
 * Keep straight, scan, turn and recovery behavior distinct; no direct PWM here.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef CIRCUIT_RECOVERY_H
#define CIRCUIT_RECOVERY_H
#include "src/line_following/symmetric_logic.h"
#if defined(CIRCUIT_SWEEP_SEARCH) || defined(CIRCUIT_STATIONARY_SEARCH) ||     \
    defined(LINE_STATIONARY_SEARCH)
#include "src/line_following/line_search.h"
#endif

enum
{
    C_LOSS_NO_DIRECTION = 42,
    C_LOSS_TIME,
    C_LOSS_DISTANCE,
    C_LOSS_STRAIGHT,
    C_LOSS_US = 2000000,
    C_LOSS_MM = 50,
    C_MEMORY_US = 1500000,
    C_WEAK_US = 30000,
    C_REVERSE_US = 80000,
    C_WEAK_REVERSE_US = 120000,
    C_PROBE_US = 30000,
    C_REACQUIRE_STRONG_US = 40000,
    C_REACQUIRE_WEAK_US = 80000,
    C_REACQUIRE_FAINT_US = 120000,
    C_BRIDGE_US = 150000,
    C_CENTRE_US = 100000,
    C_CENTRE_MEMORY_US = 400000,
    C_STRAIGHT_GAP_US = 500000,
    C_STRAIGHT_GAP_MM = 15,
    C_PROBE_LEFT = 17,
    C_PROBE_RIGHT = 18
};
typedef struct
{
    symmetric_tracker_t core;
    uint64_t weak_since;
    uint64_t reverse_since;
    uint64_t reacquire_since;
    uint64_t probe_since;
    uint64_t medium_since;
    uint64_t strong_since;
    uint64_t centre_since;
    uint64_t centre_us;
    int16_t bridge[2];
    int8_t pending;
    int8_t reverse;
    int8_t probe_direction;
    uint8_t loss_reason;
    bool weak_timing;
    bool reverse_timing;
    bool reacquire_timing;
    bool probe_timing;
    bool medium_timing;
    bool strong_timing;
    bool centre_timing;
    bool centre_valid;
    bool straight_recovery;
#if defined(CIRCUIT_SWEEP_SEARCH) || defined(CIRCUIT_STATIONARY_SEARCH) ||     \
    defined(LINE_STATIONARY_SEARCH)
    line_search_t search;
#endif
} circuit_tracker_t;
/* 0 unknown/stale,1 left,2 right,3 recently centred. This is recovery
   confidence, not a delay on the ordinary optical steering targets. */
static inline unsigned circuit_memory (const circuit_tracker_t * p_sample,
                                       uint64_t now)
{
    const symmetric_tracker_t * p_state = &p_sample->core;
    if (((p_sample->centre_valid) &&
         (now - p_sample->centre_us <= C_CENTRE_MEMORY_US)) &&
        ((!p_state->direction) ||
         (p_sample->centre_us >= p_state->direction_us)))
    {
        return 3;
    }
    return (p_state->direction) && (now - p_state->direction_us <= C_MEMORY_US)
               ? (p_state->direction > 0 ? 1 : 2)
               : 0;
}
static inline void circuit_direction (circuit_tracker_t * p_sample, int error,
                                      unsigned strongest, uint64_t now)
{
    symmetric_tracker_t * p_state = &p_sample->core;
    int magnitude = error < 0 ? -error : error;
    int8_t direction = error > 0 ? 1 : -1;
    bool strong = (magnitude >= 200) && (strongest >= 400);
    bool weak = (magnitude >= 150) && (strongest >= 200);
    if (strong && ((!p_state->direction) || (direction == p_state->direction)))
    {
        p_state->direction = direction;
        p_state->direction_us = now;
        p_sample->reverse_timing = p_sample->weak_timing = false;
        p_sample->pending = 0;
        return;
    }
    if (strong)
    {
        p_sample->weak_timing = false;
        p_sample->pending = 0;
        if ((!p_sample->reverse_timing) || (p_sample->reverse != direction))
        {
            p_sample->reverse_timing = true;
            p_sample->reverse = direction;
            p_sample->reverse_since = now;
        }
        if (now - p_sample->reverse_since >= C_REVERSE_US)
        {
            p_state->direction = direction;
            p_state->direction_us = now;
            p_sample->reverse_timing = false;
        }
        return;
    }
    p_sample->reverse_timing = false;
    /* A sustained faint return can also be real. Reverse memory only after
       120ms of consistent weak asymmetry; shorter flashes may guide a slow
       probe without committing the direction or restarting the loss budget. */
    if (weak)
    {
        if ((!p_sample->weak_timing) || (p_sample->pending != direction))
        {
            p_sample->weak_timing = true;
            p_sample->pending = direction;
            p_sample->weak_since = now;
        }
        unsigned dwell =
            (p_state->direction) && (direction != p_state->direction)
                ? C_WEAK_REVERSE_US
                : C_WEAK_US;
        if (now - p_sample->weak_since >= dwell)
        {
            p_state->direction = direction;
            p_state->direction_us = now;
        }
    }
    else
    {
        p_sample->weak_timing = false;
        p_sample->pending = 0;
    }
}
/* Absolute reflectance falls on worn tape. With a credible asymmetric faint
   return, use the left/right balance rather than treating0/243 as a tiny
   steering error. Outer35 keeps this correction slower than normal cruising. */
static inline void circuit_faint_targets (int error, const uint16_t level[2],
                                          int16_t target[2])
{
    int total = level[0] + level[1];
    symmetric_targets(error * 1000 / total, target);
    int maximum = target[0] > target[1] ? target[0] : target[1];
    target[0] = (int16_t)(target[0] * 35 / maximum);
    target[1] = (int16_t)(target[1] * 35 / maximum);
}
static inline analog_demand_t
circuit_line_demand_motion (circuit_tracker_t * p_sample,
                            const line_calibration_t * p_cal, uint16_t l_value,
                            uint16_t result, uint64_t now, uint32_t mm_value,
                            const double wheel[2], const float speed[2])
{
    (void)wheel;
    (void)speed;
    analog_demand_t demand = {.decision = SYM_STRAIGHT};
    p_sample->loss_reason = 0;
    symmetric_tracker_t * p_state = &p_sample->core;
#if defined(CIRCUIT_STATIONARY_SEARCH) || defined(LINE_STATIONARY_SEARCH)
    if ((((((!wheel) || (!speed)) || (!isfinite(wheel[0]))) ||
          (!isfinite(wheel[1]))) ||
         (!isfinite(speed[0]))) ||
        (!isfinite(speed[1])))
    {
        p_sample->loss_reason = LS_INPUT;
        demand.stop = true;
        demand.decision = AL_LOST;
        return demand;
    }
#endif
    if (!calibration_valid(p_cal))
    {
        demand.stop = true;
        demand.decision = AL_LOST;
        return demand;
    }
    demand.level[0] = line_level(l_value, p_cal->white[0], p_cal->black[0]);
    demand.level[1] = line_level(result, p_cal->white[1], p_cal->black[1]);
    int error = (int)demand.level[0] - demand.level[1],
        magnitude = error < 0 ? -error : error;
    unsigned strongest =
        demand.level[0] > demand.level[1] ? demand.level[0] : demand.level[1];
    bool low = (demand.level[0] < 150) && (demand.level[1] < 150);
    bool faint = (strongest >= 200) && (magnitude >= 150);
    int8_t observed = error > 0 ? 1 : -1;
    if (!faint)
    {
        p_sample->probe_timing = false;
    }
    else if ((!p_sample->probe_timing) ||
             (p_sample->probe_direction != observed))
    {
        p_sample->probe_timing = true;
        p_sample->probe_direction = observed;
        p_sample->probe_since = now;
    }
    circuit_direction(p_sample, error, strongest, now);
    bool centred = ((demand.level[0] >= 400) && (demand.level[1] >= 400)) &&
                   (magnitude <= 80);
    if (!centred)
    {
        p_sample->centre_timing = false;
    }
    else
    {
        if (!p_sample->centre_timing)
        {
            p_sample->centre_timing = true;
            p_sample->centre_since = now;
        }
        if (now - p_sample->centre_since >= C_CENTRE_US)
        {
            p_sample->centre_valid = true;
            p_sample->centre_us = now;
        }
    }
    if (low && (!p_state->white))
    {
        unsigned memory = circuit_memory(p_sample, now);
        p_state->white = true;
        p_state->white_since = now;
        p_state->white_mm = mm_value;
        p_state->recovery = 1 == memory ? 1 : 2 == memory ? -1 : 0;
        p_sample->straight_recovery = 3 == memory;
        p_sample->reacquire_timing = p_sample->medium_timing =
            p_sample->strong_timing = false;
    }
    if (p_state->white)
    {
        /* An isolated >=150 observation no longer ends/restarts a loss. The
           same episode and search direction persist until a credible line
           is sustained, including across weak or opposite-side flashes. */
        if ((strongest < 300) && (!faint))
        {
            p_sample->reacquire_timing = p_sample->medium_timing =
                p_sample->strong_timing = false;
        }
        else
        {
            if (!p_sample->reacquire_timing)
            {
                p_sample->reacquire_timing = true;
                p_sample->reacquire_since = now;
            }
            if (strongest < 300)
            {
                p_sample->medium_timing = false;
            }
            else if (!p_sample->medium_timing)
            {
                p_sample->medium_timing = true;
                p_sample->medium_since = now;
            }
            if (strongest < 400)
            {
                p_sample->strong_timing = false;
            }
            else if (!p_sample->strong_timing)
            {
                p_sample->strong_timing = true;
                p_sample->strong_since = now;
            }
            /* Independent bands:400-threshold flicker cannot discard an
               otherwise sustained >=300 observation. */
#if defined(CIRCUIT_STATIONARY_SEARCH) || defined(LINE_STATIONARY_SEARCH)
            bool reacquired =
                (((p_sample->strong_timing) &&
                  (now - p_sample->strong_since >= C_REACQUIRE_STRONG_US)) ||
                 ((p_sample->medium_timing) &&
                  (now - p_sample->medium_since >= C_REACQUIRE_WEAK_US))) ||
                ((p_sample->probe_timing) &&
                 (now - p_sample->probe_since >= C_REACQUIRE_FAINT_US));
            if (reacquired && (p_sample->search.active))
            {

                line_search_demand_t found = line_search_stationary_step(
                    &p_sample->search, now, mm_value, wheel, speed,
                    p_sample->reacquire_timing, true);
                if (found.reason)
                {
                    p_sample->loss_reason = found.reason;
                    demand.stop = true;
                    demand.decision = AL_LOST;
                    return demand;
                }
                if (!found.complete)
                {
                    demand.target[0] = found.target[0];
                    demand.target[1] = found.target[1];
                    demand.decision =
                        (uint8_t)(LS_DECISION_FIRST + found.stage);
                    return demand;
                }
            }
#else
            bool reacquired =
                (p_sample->strong_timing &&
                 now - p_sample->strong_since >= C_REACQUIRE_STRONG_US) ||
                (p_sample->medium_timing &&
                 now - p_sample->medium_since >= C_REACQUIRE_WEAK_US) ||
                (p_sample->probe_timing &&
                 now - p_sample->probe_since >= C_REACQUIRE_FAINT_US);
#endif
            if (reacquired)
            {
                p_state->white = false;
                p_state->recovery = 0;
                p_sample->straight_recovery = false;
                p_sample->reacquire_timing = p_sample->medium_timing =
                    p_sample->strong_timing = false;
#if defined(CIRCUIT_SWEEP_SEARCH) || defined(CIRCUIT_STATIONARY_SEARCH) ||     \
    defined(LINE_STATIONARY_SEARCH)
                p_sample->search = (line_search_t){0};
#endif
            }
        }
        if (p_state->white)
        {
            uint64_t elapsed = now - p_state->white_since;
            uint32_t travel = mm_value - p_state->white_mm;
#if defined(CIRCUIT_STATIONARY_SEARCH) || defined(LINE_STATIONARY_SEARCH)
            (void)travel;
            if (line_search_ready(&p_sample->search, now, demand.level[0],
                                  demand.level[1]))
            {
                if (!p_sample->search.active)
                {

                    line_search_stationary_begin(&p_sample->search, now,
                                                 mm_value, p_state->recovery,
                                                 wheel);
                }

                line_search_demand_t seek = line_search_stationary_step(
                    &p_sample->search, now, mm_value, wheel, speed,
                    p_sample->reacquire_timing, false);
                if (seek.reason)
                {
                    p_sample->loss_reason = seek.reason;
                    demand.stop = true;
                    demand.decision = AL_LOST;
                    return demand;
                }
                demand.target[0] = seek.target[0];
                demand.target[1] = seek.target[1];
                demand.decision = (uint8_t)(LS_DECISION_FIRST + seek.stage);
                return demand;
            }
            /* The original brief-gap bridge remains; sustained loss waits at
               zero until the two-second confirmation opens the bounded sweep.
             */
            if (elapsed >= LS_BRIDGE_US)
            {
                return demand;
            }
            if ((!p_state->recovery) && (!p_sample->straight_recovery))
            {
                return demand;
            }
#elif defined(CIRCUIT_SWEEP_SEARCH)
            (void)travel;
            /* A short worn gap still uses the proven bridge below. Once the
               gap persists, replace a one-direction timeout with alternating
               rolling searches; weak flashes do not restart this budget. */
            if (line_search_ready(&p_sample->search, now, demand.level[0],
                                  demand.level[1]))
            {
                if (!p_sample->search.active)
                {
                    line_search_begin(&p_sample->search, now, mm_value,
                                      p_state->recovery);
                }
                line_search_demand_t seek =
                    line_search_step(&p_sample->search, now, mm_value,
                                     p_sample->reacquire_timing);
                if (seek.reason)
                {
                    p_sample->loss_reason = seek.reason;
                    demand.stop = true;
                    demand.decision = AL_LOST;
                    return demand;
                }
                demand.target[0] = seek.target[0];
                demand.target[1] = seek.target[1];
                demand.decision = (uint8_t)(LS_DECISION_FIRST + seek.stage);
                return demand;
            }
            /* Barcode V12: after the short gap bridge, coast while the
               continuous-white timer runs instead of pivoting for2s. */
            if (elapsed >= LS_BRIDGE_US && strongest < 200)
            {
                return demand;
            }
            /* With no direction history, wait for the short confirmation
               rather than inventing a turn before the first sweep begins. */
            if (!p_state->recovery && !p_sample->straight_recovery)
            {
                return demand;
            }
#else
            if (!p_state->recovery && !p_sample->straight_recovery)
            {
                p_sample->loss_reason = C_LOSS_NO_DIRECTION;
            }
            else if (p_sample->straight_recovery &&
                     (elapsed >= C_STRAIGHT_GAP_US ||
                      travel >= C_STRAIGHT_GAP_MM))
            {
                p_sample->loss_reason = C_LOSS_STRAIGHT;
            }
            else if (elapsed >= C_LOSS_US)
            {
                p_sample->loss_reason = C_LOSS_TIME;
            }
            else if (travel >= C_LOSS_MM)
            {
                p_sample->loss_reason = C_LOSS_DISTANCE;
            }
            if (p_sample->loss_reason)
            {
                demand.stop = true;
                demand.decision = AL_LOST;
                return demand;
            }
#endif
            /* Unlike V5, do not blindly ignore a real weak return on the
               other sensor. Probe using reflectance balance at reduced speed,
               keeping the original loss deadline until confirmation succeeds.
             */
#if !defined(CIRCUIT_STATIONARY_SEARCH) && !defined(LINE_STATIONARY_SEARCH)
            if (faint && p_sample->probe_timing &&
                now - p_sample->probe_since >= C_PROBE_US)
            {
                circuit_faint_targets(error, demand.level, demand.target);
                demand.decision = error > 0 ? C_PROBE_LEFT : C_PROBE_RIGHT;
                return demand;
            }
#endif
            if (p_sample->straight_recovery)
            {
                demand.target[0] = demand.target[1] = 35;
                return demand;
            }
            demand.decision =
                p_state->recovery > 0 ? SYM_RECOVER_LEFT : SYM_RECOVER_RIGHT;
            demand.target[0] = p_state->recovery > 0 ? 0 : 45;
            demand.target[1] = p_state->recovery > 0 ? 45 : 0;
            /* Brief worn gaps retain the last reliable curve's curvature at
               reduced speed before beginning the usual bounded pivot search. */
            if ((elapsed < C_BRIDGE_US) &&
                (((p_state->recovery > 0) &&
                  (p_sample->bridge[0] < p_sample->bridge[1])) ||
                 ((p_state->recovery < 0) &&
                  (p_sample->bridge[0] > p_sample->bridge[1]))))
            {
                demand.target[0] = p_sample->bridge[0];
                demand.target[1] = p_sample->bridge[1];
            }
            return demand;
        }
    }
    symmetric_targets(error, demand.target);
    if (((strongest >= 200) && (strongest < 400)) && (magnitude >= 150))
    {
        circuit_faint_targets(error, demand.level, demand.target);
    }
    if (demand.target[0] < demand.target[1])
    {
        demand.decision = SYM_LEFT;
    }
    else if (demand.target[0] > demand.target[1])
    {
        demand.decision = SYM_RIGHT;
    }
    if ((strongest >= 400) && (((error > 80) && (p_state->direction > 0)) ||
                               ((error < -80) && (p_state->direction < 0))))
    {
        int maximum = demand.target[0] > demand.target[1] ? demand.target[0]
                                                          : demand.target[1];
        p_sample->bridge[0] = (int16_t)(demand.target[0] * 35 / maximum);
        p_sample->bridge[1] = (int16_t)(demand.target[1] * 35 / maximum);
    }
    return demand;
}
static inline analog_demand_t
circuit_line_demand (circuit_tracker_t * p_sample,
                     const line_calibration_t * p_cal, uint16_t l_value,
                     uint16_t result, uint64_t now, uint32_t mm_value)
{
    const double wheel[2] = {0, 0};
    const float speed[2] = {0, 0};
    return circuit_line_demand_motion(p_sample, p_cal, l_value, result, now,
                                      mm_value, wheel, speed);
}
#endif

/* End of file. */
