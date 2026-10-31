#ifndef BARCODE_LINE_SEARCH_H
#define BARCODE_LINE_SEARCH_H
#include <stdint.h>
#include <stdbool.h>

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
    LS_DECISION_FIRST = 19
};
typedef struct {
    uint64_t since, wait_since;
    uint32_t mm;
    int8_t first;
    bool active, waiting;
} LineSearch;
typedef struct {
    int16_t target[2];
    uint8_t stage, reason;
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
        return false;
    }
    if (!s->waiting) {
        if (left >= 150 || right >= 150)
            return false;
        s->waiting = true;
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
#endif
