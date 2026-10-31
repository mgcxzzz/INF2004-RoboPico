#ifndef BENCH_LOGIC_H
#define BENCH_LOGIC_H
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>

/* Hardware-independent diagnostic logic; single-core caller provides locking. */
enum { DRIVE_OFF = 0, DRIVE_A = 1, DRIVE_B = 2 };
enum { PULSE_US = 3000000, REST_US = 1000000 };
enum { STOP_NONE, STOP_GP20, STOP_GP21_RELEASE, STOP_TIMEOUT, STOP_SERIAL, STOP_USB };
typedef struct {
    uint16_t a, b;
} BenchLevels;
static inline BenchLevels bench_levels(uint8_t selected, uint8_t wheel, uint8_t drive,
                                       uint8_t duty) {
    BenchLevels v = {0, 0};
    if (selected > 1 || wheel > 1 || selected != wheel || duty > 40)
        return v;
    /* Current bench uses a 1000-count PWM period. */
    uint16_t level = (uint16_t)(10u * duty);
    if (drive == DRIVE_A)
        v.a = level;
    else if (drive == DRIVE_B)
        v.b = level;
    return v;
}
typedef struct {
    uint8_t drive;
    bool consumed;
    uint64_t deadline_us, next_start_us, started_us, last_run_us;
    uint32_t stop_count;
    uint8_t stop_reason;
} BenchControl;

static inline void bench_stop(volatile BenchControl *c, uint64_t now, uint8_t reason) {
    if (c->drive != DRIVE_OFF) {
        c->next_start_us = now + REST_US;
        c->last_run_us = now - c->started_us;
        c->stop_reason = reason;
        ++c->stop_count;
    }
    c->drive = DRIVE_OFF;
    c->consumed = true;
}
static inline void bench_tick(volatile BenchControl *c, uint64_t now, bool hold, bool stop) {
    if (stop)
        bench_stop(c, now, STOP_GP20);
    else if (!hold)
        bench_stop(c, now, STOP_GP21_RELEASE);
    else if (c->drive && now >= c->deadline_us)
        bench_stop(c, now, STOP_TIMEOUT);
    if (!hold && !stop)
        c->consumed = false;
}
static inline bool bench_start(volatile BenchControl *c, uint64_t now, bool hold, bool stop,
                               uint8_t direction) {
    if (!hold || stop || c->consumed || c->drive || now < c->next_start_us ||
        (direction != DRIVE_A && direction != DRIVE_B))
        return false;
    c->drive = direction;
    c->started_us = now;
    c->deadline_us = now + PULSE_US;
    c->consumed = true;
    return true;
}

typedef struct {
    uint8_t ab;
    int32_t position;
    uint32_t a_edges, b_edges, invalid;
} BenchEncoder;
static inline void count_up(volatile uint32_t *v) {
    if (*v != UINT32_MAX)
        ++*v;
}
static inline void bench_encoder_update(volatile BenchEncoder *e, uint8_t ab) {
    /* Positive electrical sequence: 00 -> 01 -> 11 -> 10 -> 00. */
    static const int8_t step[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
    ab &= 3u;
    uint8_t changed = e->ab ^ ab;
    if (changed & 2u)
        count_up(&e->a_edges);
    if (changed & 1u)
        count_up(&e->b_edges);
    if (changed == 3u)
        count_up(&e->invalid);
    int8_t d = step[(e->ab << 2) | ab];
    if (d > 0 && e->position < INT32_MAX)
        ++e->position;
    if (d < 0 && e->position > INT32_MIN)
        --e->position;
    e->ab = ab;
}
#endif
