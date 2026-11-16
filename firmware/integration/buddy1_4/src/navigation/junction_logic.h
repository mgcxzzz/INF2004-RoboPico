/**
 * @file junction_logic.h
 * @brief Junction logic.
 * @details The shared facade/coordinator arbitrates ownership and generations.
 * Acceptance is not completion; external callers never write motor registers.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef JUNCTION_LEFT_LOGIC_H
#define JUNCTION_LEFT_LOGIC_H
#include "src/line_following/symmetric_logic.h"
#include "src/motion/dual_logic.h"

/* Opt-in Step 2 experiment. Reference/SDK builds retain optical departure. */
#ifndef JUNCTION_ANGLE_GATE_ONLY
#define JUNCTION_ANGLE_GATE_ONLY 0
#endif

enum
{
    J_APPROACH = 1,
    J_CENTRE,
    J_SETTLE,
    J_TURN,
    J_TURN_SETTLE,
    J_EXIT,
    J_DONE,
    J_FAULT,
    J_ACQUIRE
};
enum
{
    J_OK = 0,
    J_COMPLETE = 20,
    J_NO_CROSS,
    J_BAD_STRIPE,
    J_CENTRE_DRIFT,
    J_SETTLE_TIMEOUT,
    J_TURN_TIMEOUT,
    J_BRANCH_MISSING,
    J_PATH_LIMIT,
    J_LINE_LOST,
    J_ALIGN_TIMEOUT
};
enum
{
    J_RUN_US = 35000000,
    J_ALIGN_US = 6000000,
    J_ALIGN_MM = 180,
    J_EXIT_US = 10000000,
    J_TRACE_US = 500000,
    J_MAX_PATH_MM = 900
};
typedef struct
{
    uint8_t white;
    uint8_t black;
    uint8_t mask;
} side_calibration_t;
static inline bool side_valid (const side_calibration_t * p_state)
{
    return p_state->mask == 3 && p_state->white <= 1 && p_state->black <= 1 &&
           p_state->white != p_state->black;
}
typedef struct
{
    uint64_t now;
    float wheel[2];
    float speed[2];
    float path;
    uint16_t adc[2];
    bool side_black;
} junction_input_t;
typedef struct
{
    uint8_t phase;
    uint8_t reason;
    bool turn_enabled;
    bool side_last;
    bool side_armed;
    bool striped;
    bool off_line;
    bool still_timing;
    bool off_timing;
    bool line_timing;
    bool rolling; /* Fits former padding: persisted record offsets unchanged. */
    uint64_t phase_us;
    uint64_t edge_us;
    uint64_t still_us;
    uint64_t off_us;
    uint64_t line_us;
    float edge_mm;
    float leading_mm;
    float trailing_mm;
    float centre_mm;
    float approach_heading;
    float turn_origin[2];
    float exit_origin;
    float turn_degrees;
    symmetric_tracker_t follower;
} junction_t;
static inline float j_mean (const junction_input_t * p_index)
{
    return (p_index->wheel[0] + p_index->wheel[1]) * 0.5f;
}
static inline void j_enter (junction_t * p_other_index, unsigned phase,
                            uint64_t now)
{
    p_other_index->phase = (uint8_t)phase;
    p_other_index->phase_us = now;
    p_other_index->still_timing = false;
}
static inline void j_begin (junction_t * p_other_index, bool turn,
                            const junction_input_t * p_index)
{
    *p_other_index = (junction_t){.turn_enabled = turn,
                                  .edge_us = p_index->now,
                                  .edge_mm = j_mean(p_index)};
    j_enter(p_other_index, J_APPROACH, p_index->now);
}
static inline void j_begin_mode (junction_t * p_other_index, unsigned mode,
                                 const junction_input_t * p_index)
{
    j_begin(p_other_index, mode != 1, p_index);
    p_other_index->rolling = mode == 3;
}
static inline float j_turn_start_offset (const junction_t * p_other_index)
{
    return p_other_index->rolling ? TRACK_MM * 0.5f : 0;
}
static inline void j_fail (junction_t * p_other_index, unsigned why)
{
    p_other_index->phase = J_FAULT;
    p_other_index->reason = (uint8_t)why;
}
static inline bool j_still (junction_t * p_other_index,
                            const junction_input_t * p_index)
{
    bool still = fabsf(p_index->speed[0]) < 5 && fabsf(p_index->speed[1]) < 5;
    if (!still)
    {
        p_other_index->still_timing = false;
        return false;
    }
    if (!p_other_index->still_timing)
    {
        p_other_index->still_timing = true;
        p_other_index->still_us = p_index->now;
    }
    return p_index->now - p_other_index->phase_us >= 300000 &&
           p_index->now - p_other_index->still_us >= 200000;
}
static inline void j_start_turn (junction_t * p_other_index,
                                 const junction_input_t * p_index)
{
    for (unsigned wheel = 0; wheel < 2; ++wheel)
    {
        p_other_index->turn_origin[wheel] = p_index->wheel[wheel];
    }
    p_other_index->off_line = false;
    p_other_index->off_timing = p_other_index->line_timing = false;
    j_enter(p_other_index, J_TURN, p_index->now);
}
static inline float j_rotation (const junction_t * p_other_index,
                                const junction_input_t * p_index)
{
    return ((p_index->wheel[1] - p_other_index->turn_origin[1]) -
            (p_index->wheel[0] - p_other_index->turn_origin[0])) /
           TRACK_MM * (180.0f / 3.14159265358979323846f);
}
/* Mode3 only: first sight of the outgoing stripe starts forward alignment.
 * exit_origin holds absolute path at acquisition entry, then mean wheel position
 * at J_EXIT entry. Reuse existing storage so saved V1..V5 record ABI is intact. */
static inline analog_demand_t j_acquire (junction_t * p_other_index,
                                         const line_calibration_t * p_character,
                                         const junction_input_t * p_index)
{
    analog_demand_t demand = symmetric_demand(
        &p_other_index->follower, p_character, p_index->adc[0], p_index->adc[1],
        p_index->now, (uint32_t)p_index->path);
    p_other_index->turn_degrees = j_rotation(p_other_index, p_index);
    if (p_other_index->turn_degrees < 60 || p_other_index->turn_degrees > 115 ||
        p_index->path - p_other_index->exit_origin >= J_ALIGN_MM)
    {
        j_fail(p_other_index, J_BRANCH_MISSING);
    }
    else if (p_index->now - p_other_index->phase_us >= J_ALIGN_US)
    {
        j_fail(p_other_index, J_ALIGN_TIMEOUT);
    }
    else if (demand.stop)
    {
        j_fail(p_other_index, J_LINE_LOST);
    }
    bool white = demand.level[0] < 150 && demand.level[1] < 150;
    if (white)
    {
        if (!p_other_index->off_timing)
        {
            p_other_index->off_timing = true;
            p_other_index->off_us = p_index->now;
        }
        if (p_index->now - p_other_index->off_us >= 150000)
        {
            j_fail(p_other_index, J_LINE_LOST);
        }
    }
    else
    {
        p_other_index->off_timing = false;
    }
    int32_t error = (int32_t)demand.level[0] - demand.level[1];
    if (white)
    {
        error = p_other_index->follower.recovery * 1000;
    }
    int32_t magnitude = error < 0 ? -error : error;
    if (magnitude > 1000)
    {
        magnitude = 1000;
    }
    int32_t amount = magnitude > 80 ? magnitude - 80 : 0;
    int16_t inner = (int16_t)(30 - amount * 10 / 920),
            outer = (int16_t)(30 + amount * 5 / 920);
    demand.target[0] = error >= 0 ? inner : outer;
    demand.target[1] = error >= 0 ? outer : inner;
    demand.decision = error > 80    ? SYM_LEFT
                      : error < -80 ? SYM_RIGHT
                                    : SYM_STRAIGHT;
    bool aligned =
        demand.level[0] >= 400 && demand.level[1] >= 400 && magnitude <= 350;
    if (aligned)
    {
        if (!p_other_index->line_timing)
        {
            p_other_index->line_timing = true;
            p_other_index->line_us = p_index->now;
        }
        if (p_other_index->phase == J_ACQUIRE &&
            p_index->now - p_other_index->line_us >= 80000)
        {
            p_other_index->exit_origin = j_mean(p_index);
            j_enter(p_other_index, J_EXIT, p_index->now);
        }
    }
    else
    {
        p_other_index->line_timing = false;
    }
    if (p_other_index->phase == J_FAULT)
    {
        demand.stop = true;
        demand.target[0] = demand.target[1] = 0;
    }
    return demand;
}
/* Cross detection is ONLY valid on a straight, with NO barcode/other side marks.
 * The side sensing point is approximately 92mm ahead of the wheel axle.
 * Record both cross-arm edges; their midpoint plus92mm is the axle target.
 * This is a provisional geometric value, not calibrated stopping compensation. */
static inline analog_demand_t
junction_step (junction_t * p_other_index,
               const line_calibration_t * p_character,
               const junction_input_t * p_index)
{
    analog_demand_t demand = {0};
    for (unsigned wheel = 0; wheel < 2; ++wheel)
    {
        demand.level[wheel] =
            line_level(p_index->adc[wheel], p_character->white[wheel],
                       p_character->black[wheel]);
    }
    if (!calibration_valid(p_character))
    {
        j_fail(p_other_index, J_LINE_LOST);
    }
    if (p_index->path >= J_MAX_PATH_MM)
    {
        j_fail(p_other_index, J_PATH_LIMIT);
    }
    float mean = j_mean(p_index);
    if (p_index->side_black != p_other_index->side_last)
    {
        p_other_index->side_last = p_index->side_black;
        p_other_index->edge_us = p_index->now;
        p_other_index->edge_mm = mean;
    }
    switch (p_other_index->phase)
    {
    case J_APPROACH:
        demand = symmetric_demand(&p_other_index->follower, p_character,
                                  p_index->adc[0], p_index->adc[1],
                                  p_index->now, (uint32_t)p_index->path);
        if (demand.stop)
        {
            j_fail(p_other_index, J_LINE_LOST);
            break;
        }
        if (!p_index->side_black &&
            p_index->now - p_other_index->edge_us >= 100000)
        {
            p_other_index->side_armed = true;
        }
        if (p_index->side_black &&
            p_index->now - p_other_index->edge_us >= 20000)
        {
            if (!p_other_index->side_armed)
            {
                j_fail(p_other_index, J_NO_CROSS);
                break;
            }
            p_other_index->leading_mm = p_other_index->edge_mm;
            p_other_index->centre_mm = p_other_index->leading_mm + 101.0f -
                                       j_turn_start_offset(p_other_index);
            p_other_index->approach_heading =
                heading_deg(p_index->wheel[0], p_index->wheel[1]);
            j_enter(p_other_index, J_CENTRE, p_index->now);
            demand.target[0] = demand.target[1] = 30;
            demand.stop = false;
        }
        else if (p_index->now - p_other_index->phase_us >= 12000000 ||
                 mean >= 450)
        {
            j_fail(p_other_index, J_NO_CROSS);
        }
        break;
    case J_CENTRE:
    {
        float drift = heading_deg(p_index->wheel[0], p_index->wheel[1]) -
                      p_other_index->approach_heading;
        if (fabsf(drift) > 12)
        {
            j_fail(p_other_index, J_CENTRE_DRIFT);
            break;
        }
        if (!p_other_index->striped && !p_index->side_black &&
            p_index->now - p_other_index->edge_us >= 20000)
        {
            p_other_index->trailing_mm = p_other_index->edge_mm;
            float width =
                p_other_index->trailing_mm - p_other_index->leading_mm;
            if (width < 6 || width > 40)
            {
                j_fail(p_other_index, J_BAD_STRIPE);
                break;
            }
            p_other_index->striped = true;
            p_other_index->centre_mm =
                (p_other_index->leading_mm + p_other_index->trailing_mm) *
                    0.5f +
                92.0f - j_turn_start_offset(p_other_index);
        }
        if ((!p_other_index->striped &&
             mean - p_other_index->leading_mm > 50) ||
            p_index->now - p_other_index->phase_us >= 6000000)
        {
            j_fail(p_other_index, J_BAD_STRIPE);
            break;
        }
        float remaining = p_other_index->centre_mm - mean;
        if (p_other_index->striped && remaining <= 1.0f)
        {
            if (p_other_index->rolling)
            {
                /* No reversal: keep the right wheel rolling through the corner
                   instead of stopping both wheels and overcoming stiction again. */
                j_start_turn(p_other_index, p_index);
                demand.target[0] = 0;
                demand.target[1] = 45;
            }
            else
            {
                j_enter(p_other_index, J_SETTLE, p_index->now);
            }
            break;
        }
        float base = remaining < 15 ? 25.0f : 30.0f;
        float correction = clampf(drift * 1.5f, -6, 6);
        demand.target[0] = (int16_t)(base - correction);
        demand.target[1] = (int16_t)(base + correction);
        break;
    }
    case J_SETTLE:
        if (j_still(p_other_index, p_index))
        {
            if (!p_other_index->turn_enabled)
            {
                p_other_index->phase = J_DONE;
                p_other_index->reason = J_COMPLETE;
                break;
            }
            j_start_turn(p_other_index, p_index);
        }
        else if (p_index->now - p_other_index->phase_us >= 2000000)
        {
            j_fail(p_other_index, J_SETTLE_TIMEOUT);
        }
        break;
    case J_TURN:
    {
        float left = -(p_index->wheel[0] - p_other_index->turn_origin[0]);
        float right = p_index->wheel[1] - p_other_index->turn_origin[1];
        p_other_index->turn_degrees = j_rotation(p_other_index, p_index);
        if (p_other_index->rolling)
        {
            /* Inner wheel is unpowered, not actively locked. Encoder drift is
               bounded; optical branch confirmation must still succeed. */
            if (fabsf(left) > 12 || right < -5)
            {
                j_fail(p_other_index, J_CENTRE_DRIFT);
                break;
            }
            if (p_index->now - p_other_index->phase_us >= 9000000u)
            {
                j_fail(p_other_index, J_TURN_TIMEOUT);
                break;
            }
        }
        else if (left < -5 || right < -5 || fabsf(left - right) > 20)
        {
            j_fail(p_other_index, J_CENTRE_DRIFT);
            break;
        }
        bool white = demand.level[0] < 200 && demand.level[1] < 200;
        if ((white || JUNCTION_ANGLE_GATE_ONLY) &&
            p_other_index->turn_degrees > 10)
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
        bool branch = p_other_index->rolling
                          ? (demand.level[0] >= 400 || demand.level[1] >= 400)
                          : (demand.level[0] >= 650 && demand.level[1] >= 650);
        bool found =
            p_other_index->off_line && p_other_index->turn_degrees >= 75 &&
            (!p_other_index->rolling || p_other_index->turn_degrees < 110) &&
            branch;
        if (found)
        {
            if (!p_other_index->line_timing)
            {
                p_other_index->line_timing = true;
                p_other_index->line_us = p_index->now;
            }
            if (p_index->now - p_other_index->line_us >= 30000)
            {
                if (p_other_index->rolling)
                {
                    p_other_index->follower = (symmetric_tracker_t){0};
                    p_other_index->exit_origin = p_index->path;
                    p_other_index->line_timing = p_other_index->off_timing =
                        false;
                    j_enter(p_other_index, J_ACQUIRE, p_index->now);
                    demand = j_acquire(p_other_index, p_character, p_index);
                }
                else
                {
                    j_enter(p_other_index, J_TURN_SETTLE, p_index->now);
                }
                break;
            }
        }
        else
        {
            p_other_index->line_timing = false;
        }
        if (p_other_index->turn_degrees >= 110 || left >= 120 ||
            right >= (p_other_index->rolling ? 240 : 120))
        {
            j_fail(p_other_index, J_BRANCH_MISSING);
            break;
        }
        if (p_index->now - p_other_index->phase_us >=
            (p_other_index->rolling ? 9000000u : 6000000u))
        {
            j_fail(p_other_index, J_TURN_TIMEOUT);
            break;
        }
        if (p_other_index->rolling)
        {
            demand.target[0] = 0;
            demand.target[1] = p_other_index->turn_degrees >= 65 ? 30 : 45;
        }
        else
        {
            int16_t speed = p_other_index->turn_degrees >= 65 ? 25 : 35;
            float sync = clampf((left - right) * 1.0f, -7, 7);
            demand.target[0] = -(int16_t)(speed - sync);
            demand.target[1] = (int16_t)(speed + sync);
        }
        break;
    }
    case J_ACQUIRE:
        demand = j_acquire(p_other_index, p_character, p_index);
        break;
    case J_TURN_SETTLE:
        if (j_still(p_other_index, p_index))
        {
            if (demand.level[0] < 400 || demand.level[1] < 400)
            {
                j_fail(p_other_index, J_BRANCH_MISSING);
                break;
            }
            p_other_index->exit_origin = mean;
            p_other_index->follower = (symmetric_tracker_t){0};
            j_enter(p_other_index, J_EXIT, p_index->now);
        }
        else if (p_index->now - p_other_index->phase_us >= 2000000)
        {
            j_fail(p_other_index, J_SETTLE_TIMEOUT);
        }
        break;
    case J_EXIT:
        /* The ten seconds begin only after outgoing-line alignment succeeds.
           Distance is not a completion condition; whole-run/loss guards remain. */
        if (p_index->now - p_other_index->phase_us >= J_EXIT_US)
        {
            p_other_index->phase = J_DONE;
            p_other_index->reason = J_COMPLETE;
            break;
        }
        demand = symmetric_demand(&p_other_index->follower, p_character,
                                  p_index->adc[0], p_index->adc[1],
                                  p_index->now, (uint32_t)p_index->path);
        if (demand.stop)
        {
            j_fail(p_other_index, J_LINE_LOST);
        }
        break;
    default:
        break;
    }
    if (p_other_index->phase == J_DONE || p_other_index->phase == J_FAULT)
    {
        demand.stop = true;
        demand.target[0] = demand.target[1] = 0;
    }
    return demand;
}
#endif

/* End of file. */
