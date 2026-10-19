#ifndef BENCH_LOGIC_H
#define BENCH_LOGIC_H
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>

/* Hardware-independent diagnostic logic; single-core caller provides locking. */
enum { DRIVE_OFF = 0, DRIVE_A = 1, DRIVE_B = 2 };
enum { PULSE_US = 500000, REST_US = 1000000 };
typedef struct {
    uint8_t drive;
    bool consumed;
    uint64_t deadline_us, next_start_us;
} BenchControl;

static inline void bench_stop(volatile BenchControl *c, uint64_t now) {
    if (c->drive != DRIVE_OFF) c->next_start_us = now + REST_US;
    c->drive = DRIVE_OFF;
    c->consumed = true;
}
static inline void bench_tick(volatile BenchControl *c, uint64_t now,
                              bool hold, bool stop) {
    if (stop || !hold || (c->drive && now >= c->deadline_us))
        bench_stop(c, now);
    if (!hold && !stop) c->consumed = false;
}
static inline bool bench_start(volatile BenchControl *c, uint64_t now,
                               bool hold, bool stop, uint8_t direction) {
    if (!hold || stop || c->consumed || c->drive || now < c->next_start_us ||
        (direction != DRIVE_A && direction != DRIVE_B)) return false;
    c->drive = direction;
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
    if (*v != UINT32_MAX) ++*v;
}
static inline void bench_encoder_update(volatile BenchEncoder *e, uint8_t ab) {
    /* Positive electrical sequence: 00 -> 01 -> 11 -> 10 -> 00. */
    static const int8_t step[16] = {
         0, 1,-1, 0, -1, 0, 0, 1, 1, 0, 0,-1, 0,-1, 1, 0
    };
    ab &= 3u;
    uint8_t changed = e->ab ^ ab;
    if (changed & 2u) count_up(&e->a_edges);
    if (changed & 1u) count_up(&e->b_edges);
    if (changed == 3u) count_up(&e->invalid);
    int8_t d = step[(e->ab << 2) | ab];
    if (d > 0 && e->position < INT32_MAX) ++e->position;
    if (d < 0 && e->position > INT32_MIN) --e->position;
    e->ab = ab;
}
#endif
