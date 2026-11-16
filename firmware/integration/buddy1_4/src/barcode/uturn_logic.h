/**
 * @file uturn_logic.h
 * @brief Uturn logic.
 * @details Pure barcode/route logic is called by the ADC owner and emits
 * requests. Native records are not wire packets; preserve both decode
 * directions.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BARCODE_UTURN_LOGIC_H
#define BARCODE_UTURN_LOGIC_H
#include "src/line_following/circuit_logic.h"

enum
{
    U_SEGMENT_US = 8000000,
    U_TOTAL_US = 18000000,
    U_SETTLE_US = 2000000,
    U_STILL_US = 120000,
    U_FORWARD_FAST = 90,
    U_FORWARD_MID = 55,
    U_FORWARD_FINE = 35,
    U_REVERSE_FAST = 80,
    U_REVERSE_MID = 45,
    U_REVERSE_FINE = 30
};
typedef struct
{
    uint64_t since;
    uint64_t settle_since;
    float reverse_origin[2];
    uint8_t stage;
    bool started;
    bool settling;
} barcode_u_turn_t;
/* Position is signed; travelled distance must keep reverse increments too.
 * Encoder deltas are32-bit, so widen before negating INT32_MIN. */
static inline uint64_t barcode_travel_ticks (int32_t delta)
{
    return delta < 0 ? (uint64_t) - (int64_t)delta : (uint64_t)delta;
}
/* Two rolling quarter-turns with the same leftward yaw:
 * right forward/left OFF, settle, left reverse/right OFF, settle.
 * With equal wheel travel the final axle is on the original longitudinal
 * line. A single forward-only semicircle would instead be one track-width
 * beside it. Encoder geometry is provisional; optics must confirm return. */
static inline analog_demand_t
barcode_uturn_step (barcode_u_turn_t * p_u_value, junction_t * p_other_index,
                    const line_calibration_t * p_cal,
                    const junction_input_t * p_index)
{
    if (J_CENTRE == p_other_index->phase)
    {
        analog_demand_t demand = junction_step(p_other_index, p_cal, p_index);
        if (J_TURN == p_other_index->phase && (!demand.stop))
        {
            demand.target[1] = U_FORWARD_FAST;
        }
        return demand;
    }
    analog_demand_t demand_value = {0};
    for (unsigned wheel = 0; wheel < 2; ++wheel)
    {
        demand_value.level[wheel] = line_level(
            p_index->adc[wheel], p_cal->white[wheel], p_cal->black[wheel]);
    }
    if (!calibration_valid(p_cal))
    {
        j_fail(p_other_index, J_LINE_LOST);
    }
    if ((!p_u_value->started) && J_TURN == p_other_index->phase)
    {
        p_u_value->started = true;
        p_u_value->since = p_index->now;
    }
    p_other_index->turn_degrees = j_rotation(p_other_index, p_index);
    if ((p_u_value->started) && (p_index->now - p_u_value->since >= U_TOTAL_US))
    {
        j_fail(p_other_index, J_TURN_TIMEOUT);
    }
    if ((p_other_index->turn_degrees < -5) ||
        (p_other_index->turn_degrees >= 195))
    {
        j_fail(p_other_index, J_BRANCH_MISSING);
    }
    if (J_TURN == p_other_index->phase)
    {
        if (p_index->now - p_other_index->phase_us >= U_SEGMENT_US)
        {
            j_fail(p_other_index, J_TURN_TIMEOUT);
        }
        float left = p_index->wheel[0] - p_other_index->turn_origin[0],
              right = p_index->wheel[1] - p_other_index->turn_origin[1];
        bool white =
            (demand_value.level[0] < 200) && (demand_value.level[1] < 200);
        if ((white || JUNCTION_ANGLE_GATE_ONLY) &&
            (p_other_index->turn_degrees > 10))
        {
            if (!p_other_index->off_timing)
            {
                p_other_index->off_timing = true;
                p_other_index->off_us = p_index->now;
            }
            if (p_index->now - p_other_index->off_us >= 30000)
            {
                p_other_index->off_line = true;
            }
        }
        else
        {
            p_other_index->off_timing = false;
        }
        if (0 == p_u_value->stage)
        {
            if (((fabsf(left) > 12) || (right < -5)) || (right > 220))
            {
                j_fail(p_other_index, J_CENTRE_DRIFT);
            }
            if ((p_other_index->phase != J_FAULT) &&
                (p_other_index->turn_degrees >= 90))
            {
                p_u_value->stage = 1;
                p_u_value->settling = false;
                j_enter(p_other_index, J_TURN_SETTLE, p_index->now);
            }
            else
            {
                demand_value.target[1] =
                    p_other_index->turn_degrees >= 82   ? U_FORWARD_FINE
                    : p_other_index->turn_degrees >= 60 ? U_FORWARD_MID
                                                        : U_FORWARD_FAST;
            }
        }
        else if (2 == p_u_value->stage)
        {
            float reverse = p_u_value->reverse_origin[0] - p_index->wheel[0];
            float stationary = p_index->wheel[1] - p_u_value->reverse_origin[1];
            if (((fabsf(stationary) > 12) || (reverse < -5)) || (reverse > 220))
            {
                j_fail(p_other_index, J_CENTRE_DRIFT);
            }
            bool branch = ((p_other_index->off_line) &&
                           (p_other_index->turn_degrees >= 165)) &&
                          ((demand_value.level[0] >= 400) ||
                           (demand_value.level[1] >= 400));
            if (!branch)
            {
                p_other_index->line_timing = false;
            }
            else
            {
                if (!p_other_index->line_timing)
                {
                    p_other_index->line_timing = true;
                    p_other_index->line_us = p_index->now;
                }
                if ((p_index->now - p_other_index->line_us >= 30000) &&
                    (p_other_index->phase != J_FAULT))
                {
                    p_u_value->stage = 3;
                    p_u_value->settling = false;
                    int error =
                        (int)demand_value.level[0] - demand_value.level[1];
                    p_other_index->follower =
                        (symmetric_tracker_t){.direction = error > 80    ? 1
                                                           : error < -80 ? -1
                                                                         : 0,
                                              .direction_us = p_index->now};
                    j_enter(p_other_index, J_TURN_SETTLE, p_index->now);
                }
            }
            if (J_TURN == p_other_index->phase)
            {
                demand_value.target[0] =
                    -(p_other_index->turn_degrees >= 170   ? U_REVERSE_FINE
                      : p_other_index->turn_degrees >= 150 ? U_REVERSE_MID
                                                           : U_REVERSE_FAST);
            }
        }
    }
    else if (J_TURN_SETTLE == p_other_index->phase)
    {
        if (p_index->now - p_other_index->phase_us >= U_SETTLE_US)
        {
            j_fail(p_other_index, J_SETTLE_TIMEOUT);
        }
        bool still =
            (fabsf(p_index->speed[0]) < 5) && (fabsf(p_index->speed[1]) < 5);
        if (!still)
        {
            p_u_value->settling = false;
        }
        else
        {
            if (!p_u_value->settling)
            {
                p_u_value->settling = true;
                p_u_value->settle_since = p_index->now;
            }
            if ((p_index->now - p_u_value->settle_since >= U_STILL_US) &&
                (p_other_index->phase != J_FAULT))
            {
                if (1 == p_u_value->stage)
                {
                    memcpy(p_u_value->reverse_origin, p_index->wheel,
                           sizeof(p_u_value->reverse_origin));
                    p_u_value->stage = 2;
                    p_other_index->line_timing = false;
                    j_enter(p_other_index, J_TURN, p_index->now);
                }
                else if (3 == p_u_value->stage)
                {
                    p_u_value->stage = 4;
                    p_other_index->exit_origin = p_index->path;
                    j_enter(p_other_index, J_ACQUIRE, p_index->now);
                }
            }
        }
    }
    if (J_FAULT == p_other_index->phase)
    {
        demand_value.stop = true;
        demand_value.target[0] = demand_value.target[1] = 0;
    }
    return demand_value;
}
#endif

/* End of file. */
