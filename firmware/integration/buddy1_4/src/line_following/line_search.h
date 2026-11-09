#ifndef BARCODE_LINE_SEARCH_H
#define BARCODE_LINE_SEARCH_H
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "src/motion/dual_logic.h"

enum {
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
enum {
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
typedef struct {
    uint64_t since, wait_since;
    uint64_t settle_since, progress_since, still_since;
    uint32_t mm;
    signed first : 2;
    unsigned phase : 3, stage : 3;
    double origin_wheel[2], last_wheel[2], progress_wheel[2];
    float relative_yaw_deg, target_yaw_deg;
    unsigned active : 1, waiting : 1, still_timing : 1, wheel_valid : 1, progress_valid : 1,
        found_candidate : 1;
} LineSearch;
typedef struct {
    int16_t target[2];
    unsigned stage : 3, phase : 3, reason : 6, complete : 1;
} LineSearchDemand;
static inline int line_search_outer(bool active, int left, int right) {
    return !active ? -1 : left == 0 && right > 0 ? 1 : right == 0 && left > 0 ? 0 : -1;
}
/* Count a continuous both-white interval, with a150..199 hysteresis band.
 * A credible return resets this pre-search timer; once sweeping, flashes
 * cannot renew its separate time/distance budget. */
static inline bool line_search_ready(LineSearch *s, uint64_t now, uint16_t left, uint16_t right) {
    if (s->active)
        return true;
    if (left >= 200 || right >= 200) {
        s->waiting = false;
        s->phase = LS_PHASE_IDLE;
        return false;
    }
    if (!s->waiting) {
        if (left >= 150 || right >= 150)
            return false;
        s->waiting = true;
        s->phase = LS_PHASE_WHITE_WAIT;
        s->wait_since = now;
    }
    return now - s->wait_since >= LS_CONFIRM_US;
}
static inline void line_search_begin(LineSearch *s, uint64_t now, uint32_t mm, int direction) {
    *s = (LineSearch){.since = now, .mm = mm, .first = direction < 0 ? -1 : 1, .active = true};
}
/* Forward-only arcs: coast one wheel and drive the other. Their directions
 * alternate;350/700/1050/1400/1750ms durations widen the angular sweep if
 * motion is comparable in both directions. This does not claim a stationary
 * pivot or an encoder-guaranteed angle. Time also expires with zero ticks. */
static inline LineSearchDemand line_search_step(const LineSearch *s, uint64_t now, uint32_t mm,
                                                bool confirming) {
    LineSearchDemand d = {0};
    uint64_t elapsed = now - s->since;
    if (elapsed >= LS_TOTAL_US) {
        d.reason = LS_TIMEOUT;
        return d;
    }
    if (mm - s->mm >= LS_MAX_MM) {
        d.reason = LS_DISTANCE;
        return d;
    }
    uint64_t end = 350000;
    while (d.stage < 4 && elapsed >= end) {
        ++d.stage;
        end += (uint64_t)(d.stage + 1) * 350000;
    }
    int direction = d.stage % 2 ? -s->first : s->first;
    d.target[direction > 0 ? 1 : 0] = confirming ? LS_CONFIRM_SPEED : LS_SPEED;
    return d;
}

/* Production-only stationary search. Encoder coordinates use the same
 * forward-positive millimetres as CircuitInput.wheel; positive `first` keeps
 * the established left-recovery convention (-left,+right). Every change of
 * wheel direction crosses a zero-target, encoder-still dwell. */
static inline float line_search_target_yaw(const LineSearch *s) {
    int direction = (s->stage & 1U) ? -s->first : s->first;
    int degrees = s->stage < 2 ? LS_SMALL_ANGLE_DEG : LS_WIDE_ANGLE_DEG;
    /* heading_deg() uses left-minus-right, so this sign maps a left pivot to
       a negative observed yaw while preserving the optical recovery sign. */
    return (float)(-direction * degrees);
}
static inline void line_search_settle(LineSearch *s, uint8_t phase, uint64_t now) {
    s->phase = phase;
    s->settle_since = now;
    s->still_timing = false;
}
static inline void line_search_stationary_begin(LineSearch *s, uint64_t now, uint32_t mm,
                                                int direction, const double wheel[2]) {
    /* With no directional memory, left is only the deterministic first leg;
       the subsequent bounded sweep covers the opposite side as well. */
    *s = (LineSearch){.since = now,
                      .mm = mm,
                      .first = direction < 0 ? -1 : 1,
                      .phase = LS_PHASE_SETTLE_START,
                      .active = true,
                      .wheel_valid = wheel && isfinite(wheel[0]) && isfinite(wheel[1]),
                      .progress_valid = true};
    if (!s->wheel_valid)
        return; /* The first step reports LS_INPUT without dereferencing pose. */
    s->origin_wheel[0] = s->last_wheel[0] = s->progress_wheel[0] = wheel[0];
    s->origin_wheel[1] = s->last_wheel[1] = s->progress_wheel[1] = wheel[1];
    s->target_yaw_deg = line_search_target_yaw(s);
    s->settle_since = s->progress_since = now;
}
static inline bool line_search_at_target(const LineSearch *s) {
    return s->target_yaw_deg >= 0 ? s->relative_yaw_deg >= s->target_yaw_deg
                                  : s->relative_yaw_deg <= s->target_yaw_deg;
}
static inline LineSearchDemand line_search_stationary_step(LineSearch *s, uint64_t now,
                                                           uint32_t mm, const double wheel[2],
                                                           const float speed[2], bool candidate,
                                                           bool confirmed) {
    LineSearchDemand d = {.stage = s->stage, .phase = s->phase};
    if (!s->active)
        return d;
    if (!s->wheel_valid || !wheel || !speed || !isfinite(wheel[0]) || !isfinite(wheel[1]) || !isfinite(speed[0]) ||
        !isfinite(speed[1])) {
        d.reason = LS_INPUT;
        return d;
    }
    uint64_t elapsed = now - s->since;
    if (elapsed >= LS_TOTAL_US) {
        d.reason = LS_TIMEOUT;
        return d;
    }
    if (mm - s->mm >= LS_MAX_PATH_MM) {
        d.reason = LS_DISTANCE;
        return d;
    }
    s->relative_yaw_deg = heading_deg((float)(wheel[0] - s->origin_wheel[0]),
                                      (float)(wheel[1] - s->origin_wheel[1]));
    if (fabsf(s->relative_yaw_deg) > LS_MAX_ANGLE_DEG) {
        d.reason = LS_ANGLE;
        return d;
    }
    double delta_left = wheel[0] - s->last_wheel[0];
    double delta_right = wheel[1] - s->last_wheel[1];
    bool ticked = fabs(delta_left) > LS_STILL_DELTA_MM_X100 / 100.0 ||
                  fabs(delta_right) > LS_STILL_DELTA_MM_X100 / 100.0;
    s->last_wheel[0] = wheel[0];
    s->last_wheel[1] = wheel[1];
    bool stopped = fabsf(speed[0]) < 5.0f && fabsf(speed[1]) < 5.0f && !ticked;

    if (s->phase == LS_PHASE_SWEEP && ticked) {
        s->progress_wheel[0] = wheel[0];
        s->progress_wheel[1] = wheel[1];
        s->progress_since = now;
    } else if (s->phase == LS_PHASE_SWEEP && s->progress_valid &&
               now - s->progress_since >= LS_STALL_US) {
        d.reason = LS_STALL;
        return d;
    }
    if (s->phase == LS_PHASE_SWEEP && candidate) {
        s->found_candidate = false;
        line_search_settle(s, LS_PHASE_CONFIRMING, now);
    } else if (s->phase == LS_PHASE_SWEEP && line_search_at_target(s)) {
        line_search_settle(s, LS_PHASE_SETTLE_REVERSAL, now);
    } else if ((s->phase == LS_PHASE_SETTLE_START || s->phase == LS_PHASE_SETTLE_REVERSAL ||
                s->phase == LS_PHASE_SETTLE_RESUME) && candidate) {
        s->found_candidate = false;
        line_search_settle(s, confirmed ? LS_PHASE_SETTLE_FOUND : LS_PHASE_CONFIRMING, now);
    } else if (s->phase == LS_PHASE_CONFIRMING && !candidate) {
        s->found_candidate = false;
        line_search_settle(s, LS_PHASE_SETTLE_RESUME, now);
    } else if (s->phase == LS_PHASE_CONFIRMING && confirmed) {
        s->found_candidate = true;
        line_search_settle(s, LS_PHASE_SETTLE_FOUND, now);
    } else if (s->phase == LS_PHASE_SETTLE_FOUND && !candidate) {
        s->found_candidate = false;
        line_search_settle(s, LS_PHASE_SETTLE_RESUME, now);
    } else if (s->phase == LS_PHASE_SETTLE_RESUME && confirmed) {
        s->found_candidate = true;
        line_search_settle(s, LS_PHASE_SETTLE_FOUND, now);
    }

    if (s->phase == LS_PHASE_SETTLE_START || s->phase == LS_PHASE_SETTLE_REVERSAL ||
        s->phase == LS_PHASE_SETTLE_FOUND || s->phase == LS_PHASE_SETTLE_RESUME ||
        s->phase == LS_PHASE_CONFIRMING) {
        if (now - s->settle_since >= LS_SETTLE_LIMIT_US &&
            (!s->still_timing || now - s->still_since < LS_STILL_US)) {
            d.reason = LS_SETTLE_TIMEOUT;
            return d;
        }
        if (stopped) {
            if (!s->still_timing) {
                s->still_timing = true;
                s->still_since = now;
            }
            if (now - s->still_since >= LS_STILL_US) {
                if (s->phase == LS_PHASE_SETTLE_FOUND && s->found_candidate) {
                    d.complete = true;
                    d.phase = s->phase;
                    return d;
                }
                if (s->phase == LS_PHASE_CONFIRMING) {
                    /* Candidate is still being qualified; remain stopped. */
                } else if (s->phase == LS_PHASE_SETTLE_START) {
                    s->target_yaw_deg = line_search_target_yaw(s);
                    s->phase = LS_PHASE_SWEEP;
                    s->progress_since = now;
                    s->progress_valid = true;
                } else if (line_search_at_target(s)) {
                    if (++s->stage >= 4) {
                        d.reason = LS_SWEEPS_EXHAUSTED;
                        return d;
                    }
                    s->target_yaw_deg = line_search_target_yaw(s);
                    s->phase = LS_PHASE_SWEEP;
                    s->progress_since = now;
                } else {
                    s->phase = LS_PHASE_SWEEP;
                    s->progress_since = now;
                }
                s->progress_wheel[0] = wheel[0];
                s->progress_wheel[1] = wheel[1];
                s->still_timing = false;
            }
        } else
            s->still_timing = false;
    }
    if (s->phase == LS_PHASE_SWEEP) {
        int direction = (s->stage & 1U) ? -s->first : s->first;
        d.target[0] = (int16_t)(-direction * LS_STATIONARY_SPEED);
        d.target[1] = (int16_t)(direction * LS_STATIONARY_SPEED);
    }
    d.stage = s->stage;
    d.phase = s->phase;
    return d;
}
#endif
