/**
 * @file encoder_and_guard.h
 * @brief Encoder and guard.
 * @details Motion helpers run under the coordinator/control ownership contract.
 * Preserve bounded outputs, target sign, stop precedence and request lifetimes.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BENCH_LOGIC_H
#define BENCH_LOGIC_H
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>

/* Hardware-independent diagnostic logic; single-core caller provides locking.
 */
enum
{
    DRIVE_OFF = 0,
    DRIVE_A = 1,
    DRIVE_B = 2
};
enum
{
    PULSE_US = 3000000,
    REST_US = 1000000
};
enum
{
    STOP_NONE,
    STOP_GP20,
    STOP_GP21_RELEASE,
    STOP_TIMEOUT,
    STOP_SERIAL,
    STOP_USB
};
typedef struct
{
    uint16_t a;
    uint16_t b;
} bench_levels_t;
static inline bench_levels_t bench_levels (uint8_t selected, uint8_t wheel,
                                           uint8_t drive, uint8_t duty)
{
    bench_levels_t value = {0, 0};
    if ((((selected > 1) || (wheel > 1)) || (selected != wheel)) || (duty > 40))
    {
        return value;
    }
    /* Current bench uses a 1000-count PWM period. */
    uint16_t level = (uint16_t)(10u * duty);
    if (DRIVE_A == drive)
    {
        value.a = level;
    }
    else if (DRIVE_B == drive)
    {
        value.b = level;
    }
    return value;
}
typedef struct
{
    uint8_t drive;
    bool consumed;
    uint64_t deadline_us;
    uint64_t next_start_us;
    uint64_t started_us;
    uint64_t last_run_us;
    uint32_t stop_count;
    uint8_t stop_reason;
} bench_control_t;

static inline void bench_stop (volatile bench_control_t * p_character,
                               uint64_t now, uint8_t reason)
{
    if (p_character->drive != DRIVE_OFF)
    {
        p_character->next_start_us = now + REST_US;
        p_character->last_run_us = now - p_character->started_us;
        p_character->stop_reason = reason;
        ++p_character->stop_count;
    }
    p_character->drive = DRIVE_OFF;
    p_character->consumed = true;
}
static inline void bench_tick (volatile bench_control_t * p_character,
                               uint64_t now, bool hold, bool stop)
{
    if (stop)
    {
        bench_stop(p_character, now, STOP_GP20);
    }
    else if (!hold)
    {
        bench_stop(p_character, now, STOP_GP21_RELEASE);
    }
    else if ((p_character->drive) && (now >= p_character->deadline_us))
    {
        bench_stop(p_character, now, STOP_TIMEOUT);
    }
    if ((!hold) && (!stop))
    {
        p_character->consumed = false;
    }
}
static inline bool bench_start (volatile bench_control_t * p_character,
                                uint64_t now, bool hold, bool stop,
                                uint8_t direction)
{
    if ((((((!hold) || stop) || (p_character->consumed)) ||
          (p_character->drive)) ||
         (now < p_character->next_start_us)) ||
        ((direction != DRIVE_A) && (direction != DRIVE_B)))
    {
        return false;
    }
    p_character->drive = direction;
    p_character->started_us = now;
    p_character->deadline_us = now + PULSE_US;
    p_character->consumed = true;
    return true;
}

typedef struct
{
    uint8_t ab;
    int32_t position;
    uint32_t a_edges;
    uint32_t b_edges;
    uint32_t invalid;
} bench_encoder_t;
static inline void count_up (volatile uint32_t * p_value)
{
    if (*p_value != UINT32_MAX)
    {
        ++*p_value;
    }
}
static inline void bench_encoder_update (volatile bench_encoder_t * p_event,
                                         uint8_t ab_value)
{
    /* Positive electrical sequence: 00 -> 01 -> 11 -> 10 -> 00. */
    static const int8_t step[16] = {0, 1, -1, 0,  -1, 0,  0, 1,
                                    1, 0, 0,  -1, 0,  -1, 1, 0};
    ab_value &= 3u;
    uint8_t changed = p_event->ab ^ ab_value;
    if (changed & 2u)
    {
        count_up(&p_event->a_edges);
    }
    if (changed & 1u)
    {
        count_up(&p_event->b_edges);
    }
    if (3u == changed)
    {
        count_up(&p_event->invalid);
    }
    int8_t demand = step[(p_event->ab << 2) | ab_value];
    if ((demand > 0) && (p_event->position < INT32_MAX))
    {
        ++p_event->position;
    }
    if ((demand < 0) && (p_event->position > INT32_MIN))
    {
        --p_event->position;
    }
    p_event->ab = ab_value;
}
#endif

/* End of file. */
