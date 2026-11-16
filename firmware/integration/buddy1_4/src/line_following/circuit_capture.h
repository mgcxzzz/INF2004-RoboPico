/**
 * @file circuit_capture.h
 * @brief Circuit capture.
 * @details The control task owns mutable steering/recovery state.
 * Keep straight, scan, turn and recovery behavior distinct; no direct PWM here.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef CIRCUIT_CAPTURE_H
#define CIRCUIT_CAPTURE_H
#include "src/line_following/circuit_logic.h"
enum
{
    C_CAPTURE_US = 3000000,
    C_CAPTURE_MM = 45,
    C_ROLLING_US = 100000
};
typedef struct
{
    uint64_t since;
    uint64_t rolling_since;
    double path;
    int8_t outer;
    bool window;
    bool rolling_timing;
} circuit_capture_t;
static inline void circuit_capture_begin (circuit_capture_t * p_state,
                                          uint64_t now, double path)
{
    *p_state = (circuit_capture_t){
        .since = now, .path = path, .outer = -1, .window = true};
}
/* Return outer wheel0/1, or-1 for ordinary effort. No target changes.
   Assistance is possible only just after optical confirmation of a branch,
   with a visible line and a near-pivot demand. It never assists blind recovery.
 */
static inline int circuit_capture_outer (circuit_capture_t * p_state,
                                         const circuit_input_t * p_index,
                                         unsigned phase,
                                         const analog_demand_t * p_demand)
{
    if (!p_state->window)
    {
        return -1;
    }
    if ((((p_demand->stop) || ((phase != J_ACQUIRE) && (phase != C_FOLLOW))) ||
         (p_index->now - p_state->since >= C_CAPTURE_US)) ||
        (p_index->path - p_state->path >= C_CAPTURE_MM))
    {
        p_state->window = false;
        p_state->outer = -1;
        return -1;
    }
    bool rolling =
        (((p_demand->target[0] >= 15) && (p_demand->target[1] >= 15)) &&
         (p_index->speed[0] >= 10)) &&
        (p_index->speed[1] >= 10);
    if (!rolling)
    {
        p_state->rolling_timing = false;
    }
    else
    {
        if (!p_state->rolling_timing)
        {
            p_state->rolling_timing = true;
            p_state->rolling_since = p_index->now;
        }
        if (p_index->now - p_state->rolling_since >= C_ROLLING_US)
        {
            p_state->window = false;
            p_state->outer = -1;
            return -1;
        }
    }
    unsigned outer = p_demand->target[0] > p_demand->target[1] ? 0 : 1,
             inner = 1 - outer;
    unsigned limit = p_state->outer == (int)outer
                         ? 20
                         : 12; /* effort hysteresis, not steering thresholds */
    bool visible = (p_demand->level[0] >= 400) || (p_demand->level[1] >= 400);
    if ((visible && (p_demand->target[outer] >= 30)) &&
        (p_demand->target[inner] <= (int)limit))
    {
        p_state->outer = (int8_t)outer;
        return (int)outer;
    }
    p_state->outer = -1;
    return -1;
}
#endif

/* End of file. */
