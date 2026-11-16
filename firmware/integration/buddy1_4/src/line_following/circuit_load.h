/**
 * @file circuit_load.h
 * @brief Circuit load.
 * @details The control task owns mutable steering/recovery state.
 * Keep straight, scan, turn and recovery behavior distinct; no direct PWM here.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef CIRCUIT_LOAD_H
#define CIRCUIT_LOAD_H
#include "src/line_following/circuit_logic.h"

enum
{
    C_LOAD_QUALIFY_US = 120000,
    C_LOAD_US = 2000000,
    C_LOAD_MM = 30,
    C_LOAD_COOLDOWN_US = 300000
};
typedef struct
{
    uint64_t since;
    uint64_t cooldown_until;
    double path;
    unsigned outer;
    bool timing;
    bool active;
} circuit_load_t;
/* Ordinary visible sharp corrections can need the same effort as branch
   capture. Qualify measured underspeed; never change the optical targets. */
static inline int circuit_load_outer (circuit_load_t * p_state,
                                      const circuit_input_t * p_index,
                                      unsigned phase,
                                      const analog_demand_t * p_demand,
                                      bool capture_active)
{
    unsigned outer = p_demand->target[0] > p_demand->target[1] ? 0 : 1,
             inner = 1 - outer;
    bool eligible =
        ((!p_demand->stop) && (!capture_active)) && C_FOLLOW == phase &&
        ((p_demand->level[0] >= 400) || (p_demand->level[1] >= 400)) &&
        (p_demand->target[outer] >= 30) &&
        (p_demand->target[inner] <= (p_state->active ? 20 : 12));
    if (p_state->active)
    {
        if (((((!eligible) || (outer != p_state->outer)) ||
              (p_index->now - p_state->since >= C_LOAD_US)) ||
             (p_index->path - p_state->path >= C_LOAD_MM)) ||
            (p_index->speed[outer] > p_demand->target[outer] + 5))
        {
            p_state->active = p_state->timing = false;
            p_state->cooldown_until = p_index->now + C_LOAD_COOLDOWN_US;
            return -1;
        }
        return (int)outer;
    }
    if (((!eligible) || (p_index->now < p_state->cooldown_until)) ||
        (p_index->speed[outer] >= 0.70f * p_demand->target[outer]))
    {
        p_state->timing = false;
        return -1;
    }
    if ((!p_state->timing) || (outer != p_state->outer))
    {
        p_state->timing = true;
        p_state->outer = outer;
        p_state->since = p_index->now;
    }
    if (p_index->now - p_state->since < C_LOAD_QUALIFY_US)
    {
        return -1;
    }
    p_state->active = true;
    p_state->timing = false;
    p_state->since = p_index->now;
    p_state->path = p_index->path;
    return (int)outer;
}
#endif

/* End of file. */
