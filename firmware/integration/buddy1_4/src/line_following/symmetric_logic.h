/**
 * @file symmetric_logic.h
 * @brief Symmetric logic.
 * @details The control task owns mutable steering/recovery state.
 * Keep straight, scan, turn and recovery behavior distinct; no direct PWM here.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef SYMMETRIC_LINE_LOGIC_H
#define SYMMETRIC_LINE_LOGIC_H
#include "src/line_following/line_logic.h"

/* For the NEW placement: physical left/right spots span the black strip.
 * Two-black is normal, never evidence of a crossing in this profile.
 * This cannot locate the exact centre inside a saturated two-black plateau. */
enum
{
    SYM_STRAIGHT = 11,
    SYM_LEFT,
    SYM_RIGHT,
    SYM_RECOVER_LEFT,
    SYM_RECOVER_RIGHT
};
enum
{
    SYM_RUN_US = 10000000,
    SYM_RUN_MM = 500,
    SYM_TRACE_US = 125000,
    SYM_LOSS_US = 650000,
    SYM_LOSS_MM = 25,
    SYM_MEMORY_US = 500000
};
typedef struct
{
    uint64_t direction_us;
    uint64_t white_since;
    uint32_t white_mm;
    int8_t direction;
    int8_t recovery;
    bool white;
} symmetric_tracker_t;
/* V11 adaptive forward speed. Level difference is reflectance error, not mm.
 * Piecewise interpolation keeps target changes <=1mm/s per error count:
 * |error|80:45/45,250:40/50,500:25/50,1000:0/45 (inner/outer).
 * Targets stay forward-only. Zero is coast/off through the unchanged wheel PI.
 */
static inline void symmetric_targets (int32_t error, int16_t target[2])
{
    if (error > 1000)
    {
        error = 1000;
    }
    if (error < -1000)
    {
        error = -1000;
    }
    int32_t magnitude = error < 0 ? -error : error;
    int32_t inner = 45, outer = 45;
    if ((magnitude > 80) && (magnitude <= 250))
    {
        int32_t turn = ((magnitude - 80) * 5 + 169) / 170;
        inner = 45 - turn;
        outer = 45 + turn;
    }
    else if ((magnitude > 250) && (magnitude <= 500))
    {
        inner = 40 - (magnitude - 250) * 15 / 250;
        outer = 50;
    }
    else if (magnitude > 500)
    {
        inner = (1000 - magnitude) * 25 / 500;
        /* Quantize the outer target with the inner target, so rounding cannot
           weaken curvature on a step where only the outer speed decreases. */
        outer = 45 + inner / 5;
    }
    target[0] = (int16_t)(error >= 0 ? inner : outer);
    target[1] = (int16_t)(error >= 0 ? outer : inner);
}
static inline analog_demand_t
symmetric_demand (symmetric_tracker_t * p_sample,
                  const line_calibration_t * p_character, uint16_t l_value,
                  uint16_t result, uint64_t now, uint32_t mm_value)
{
    analog_demand_t demand = {.decision = SYM_STRAIGHT};
    if (!calibration_valid(p_character))
    {
        demand.stop = true;
        demand.decision = AL_LOST;
        return demand;
    }
    demand.level[0] =
        line_level(l_value, p_character->white[0], p_character->black[0]);
    demand.level[1] =
        line_level(result, p_character->white[1], p_character->black[1]);
    if ((demand.level[0] < 150) && (demand.level[1] < 150))
    {
        if (!p_sample->white)
        {
            p_sample->white = true;
            p_sample->white_since = now;
            p_sample->white_mm = mm_value;
            p_sample->recovery =
                (p_sample->direction) &&
                        (now - p_sample->direction_us <= SYM_MEMORY_US)
                    ? p_sample->direction
                    : 0;
        }
        if (((!p_sample->recovery) ||
             (now - p_sample->white_since >= SYM_LOSS_US)) ||
            (mm_value - p_sample->white_mm >= SYM_LOSS_MM))
        {
            demand.stop = true;
            demand.decision = AL_LOST;
            return demand;
        }
        demand.decision =
            p_sample->recovery > 0 ? SYM_RECOVER_LEFT : SYM_RECOVER_RIGHT;
        demand.target[0] = p_sample->recovery > 0 ? 0 : 45;
        demand.target[1] = p_sample->recovery > 0 ? 45 : 0;
        return demand;
    }
    p_sample->white = false;
    p_sample->recovery = 0;
    int32_t error = (int32_t)demand.level[0] - demand.level[1];
    /* Only a clear, recent asymmetric observation may choose loss recovery. */
    if (((error >= 200) || (error <= -200)) &&
        ((demand.level[0] >= 400) || (demand.level[1] >= 400)))
    {
        p_sample->direction = error > 0 ? 1 : -1;
        p_sample->direction_us = now;
    }
    symmetric_targets(error, demand.target);
    if (demand.target[0] < demand.target[1])
    {
        demand.decision = SYM_LEFT;
    }
    else if (demand.target[0] > demand.target[1])
    {
        demand.decision = SYM_RIGHT;
    }
    return demand;
}
#endif

/* End of file. */
