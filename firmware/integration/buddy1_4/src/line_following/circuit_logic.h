/**
 * @file circuit_logic.h
 * @brief Circuit logic.
 * @details The control task owns mutable steering/recovery state.
 * Keep straight, scan, turn and recovery behavior distinct; no direct PWM here.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef CIRCUIT_LOGIC_H
#define CIRCUIT_LOGIC_H
#include <stdlib.h>
#include "src/navigation/junction_logic.h"
#include "src/line_following/circuit_recovery.h"

enum
{
    C_FOLLOW = 10,
    C_APPROACH_FAST = 40,
    C_BAD_PROFILE = 41,
    C_TRACE_US = 250000,
    C_GATE_US = 300000,
    C_WHITE_US = 100000,
    C_CLEAR_MM = 150,
    C_FAST = 16,
    C_CRUISE_US = 300000,
    C_JOIN_US = 150000
};
typedef struct
{
    uint64_t now;
    double wheel[2];
    double path;
    float speed[2];
    uint16_t adc[2];
    bool side_black;
} circuit_input_t;
typedef struct
{
    junction_t turn;
    circuit_tracker_t follower;
    circuit_tracker_t joining;
    bool busy;
    bool fault;
    bool completed;
    bool gate_timing;
    bool white_timing;
    bool side_last;
    bool candidate;
    bool cruise_timing;
    bool fast;
    bool join_timing;
    uint8_t reason;
    uint8_t phase;
    uint64_t gate_since;
    uint64_t white_since;
    uint64_t edge_us;
    uint64_t leg_since;
    uint64_t cruise_since;
    uint64_t join_since;
    double gate_wheel[2];
    double edge_wheel[2];
    double edge_path;
    double clear_path;
    double follow_path;
    uint32_t turns;
    uint32_t ignored;
    uint32_t follow_ms;
    uint16_t centre_ms;
    uint16_t turn_ms;
    uint16_t align_ms;
} circuit_t;
static inline int circuit_cruise (unsigned profile)
{
    return profile >= 1 && profile <= 3 ? (int)profile * 45 : 0;
}
static inline analog_demand_t circuit_follow (circuit_t * p_character,
                                              const line_calibration_t * p_cal,
                                              const circuit_input_t * p_index,
                                              unsigned profile)
{
    analog_demand_t demand = circuit_line_demand_motion(
        &p_character->follower, p_cal, p_index->adc[0], p_index->adc[1],
        p_index->now, (uint32_t)(p_index->path - p_character->follow_path),
        p_index->wheel, p_index->speed);
    /* V2: only a stable centred observation earns faster straight travel.
       All correcting and blind-recovery targets are the exact profile1 values. */
    p_character->fast = false;
    int error = (int)demand.level[0] - demand.level[1];
    bool centred = !demand.stop && demand.level[0] >= 400 &&
                   demand.level[1] >= 400 && abs(error) <= 80;
    if (profile == 1 || !centred)
    {
        p_character->cruise_timing = false;
    }
    else
    {
        if (!p_character->cruise_timing)
        {
            p_character->cruise_timing = true;
            p_character->cruise_since = p_index->now;
        }
        if (p_index->now - p_character->cruise_since >= C_CRUISE_US)
        {
            demand.target[0] = demand.target[1] =
                (int16_t)circuit_cruise(profile);
            demand.decision = C_FAST;
            p_character->fast = true;
        }
    }
    return demand;
}
/* The turn already confirmed a new branch at75..110deg, after leaving the old
   line. Use the ordinary curve follower immediately: a short outgoing straight
   must not require both sensors to stay balanced before taking its next bend.
   A visible branch must persist150ms before counting the junction as complete. */
static inline analog_demand_t circuit_join (circuit_t * p_character,
                                            const line_calibration_t * p_cal,
                                            const junction_input_t * p_index)
{
    junction_t * p_other_index = &p_character->turn;
    const double wheel[2] = {p_index->wheel[0], p_index->wheel[1]};
    analog_demand_t demand = circuit_line_demand_motion(
        &p_character->joining, p_cal, p_index->adc[0], p_index->adc[1],
        p_index->now, (uint32_t)p_index->path, wheel, p_index->speed);
    p_other_index->turn_degrees = j_rotation(p_other_index, p_index);
    uint64_t alignment_limit = J_ALIGN_US;
    float alignment_distance = J_ALIGN_MM;
#if defined(CIRCUIT_SWEEP_SEARCH) || defined(CIRCUIT_STATIONARY_SEARCH) ||     \
    defined(LINE_STATIONARY_SEARCH)
    alignment_limit =
        LS_ALIGN_US; /* Covers2s white confirmation plus a full search. */
    alignment_distance = LS_ALIGN_MM;
#endif
    if (p_index->now - p_other_index->phase_us >= alignment_limit)
    {
        j_fail(p_other_index, J_ALIGN_TIMEOUT);
    }
    else if (p_index->path - p_other_index->exit_origin >= alignment_distance)
    {
        j_fail(p_other_index, J_BRANCH_MISSING);
    }
    else if (demand.stop)
    {
        j_fail(p_other_index, p_character->joining.loss_reason
                                  ? p_character->joining.loss_reason
                                  : J_LINE_LOST);
    }
    bool visible = demand.level[0] >= 400 || demand.level[1] >= 400;
    if (!visible)
    {
        p_character->join_timing = false;
    }
    else
    {
        if (!p_character->join_timing)
        {
            p_character->join_timing = true;
            p_character->join_since = p_index->now;
        }
        if (p_other_index->phase == J_ACQUIRE &&
            p_index->now - p_character->join_since >= C_JOIN_US)
        {
            j_enter(p_other_index, J_EXIT, p_index->now);
        }
    }
    if (p_other_index->phase == J_FAULT)
    {
        demand.stop = true;
        demand.target[0] = demand.target[1] = 0;
    }
    return demand;
}
static inline void circuit_begin (circuit_t * p_character,
                                  const circuit_input_t * p_index)
{
    *p_character = (circuit_t){.phase = C_FOLLOW,
                               .side_last = p_index->side_black,
                               .leg_since = p_index->now,
                               .follow_path = p_index->path};
}
static inline void circuit_fail (circuit_t * p_character, unsigned reason)
{
    p_character->fault = true;
    p_character->fast = false;
    p_character->phase = J_FAULT;
    p_character->reason = (uint8_t)reason;
}
static inline analog_demand_t circuit_step (circuit_t * p_character,
                                            const line_calibration_t * p_cal,
                                            const circuit_input_t * p_index,
                                            unsigned profile)
{
    analog_demand_t demand = {0};
    p_character->completed = false;
    p_character->fast = false;
    if (!circuit_cruise(profile))
    {
        circuit_fail(p_character, C_BAD_PROFILE);
    }
    if (!calibration_valid(p_cal))
    {
        circuit_fail(p_character, J_LINE_LOST);
    }
    if (p_character->fault)
    {
        demand.stop = true;
        return demand;
    }
    if (!p_character->busy)
    {
        demand = circuit_follow(p_character, p_cal, p_index, profile);
        if (demand.stop)
        {
            circuit_fail(p_character, p_character->follower.loss_reason
                                          ? p_character->follower.loss_reason
                                          : J_LINE_LOST);
            return demand;
        }
        bool visible = demand.level[0] >= 400 || demand.level[1] >= 400;
        bool straight =
            visible && p_index->speed[0] >= 10 && p_index->speed[1] >= 10;
        if (!straight)
        {
            p_character->gate_timing = false;
        }
        else
        {
            if (!p_character->gate_timing)
            {
                p_character->gate_timing = true;
                p_character->gate_since = p_index->now;
                p_character->gate_wheel[0] = p_index->wheel[0];
                p_character->gate_wheel[1] = p_index->wheel[1];
            }
            float yaw = heading_deg(
                (float)(p_index->wheel[0] - p_character->gate_wheel[0]),
                (float)(p_index->wheel[1] - p_character->gate_wheel[1]));
            if (fabsf(yaw) > 3)
            {
                p_character->gate_since = p_index->now;
                p_character->gate_wheel[0] = p_index->wheel[0];
                p_character->gate_wheel[1] = p_index->wheel[1];
            }
        }
        if (!p_index->side_black)
        {
            if (!p_character->white_timing)
            {
                p_character->white_timing = true;
                p_character->white_since = p_index->now;
            }
            p_character->candidate = false;
        }
        if (p_index->side_black && !p_character->side_last)
        {
            p_character->candidate =
                p_character->white_timing &&
                p_index->now - p_character->white_since >= C_WHITE_US &&
                p_character->gate_timing &&
                p_index->now - p_character->gate_since >= C_GATE_US &&
                p_index->path >= p_character->clear_path;
            p_character->white_timing = false;
            p_character->edge_us = p_index->now;
            p_character->edge_path = p_index->path;
            p_character->edge_wheel[0] = p_index->wheel[0];
            p_character->edge_wheel[1] = p_index->wheel[1];
            if (!p_character->candidate)
            {
                ++p_character->ignored;
            }
        }
        p_character->side_last = p_index->side_black;
        if (p_character->candidate)
        {
            p_character->fast = p_character->cruise_timing = false;
            demand.target[0] = demand.target[1] =
                (int16_t)(30 +
                          (profile - 1) * 5); /* slow on first qualified edge */
            if (p_index->side_black &&
                p_index->now - p_character->edge_us >= 20000)
            {
                junction_input_t local = {.now = p_index->now,
                                          .side_black = true};
                j_begin_mode(&p_character->turn, 3, &local);
                p_character->turn.phase = J_CENTRE;
                p_character->turn.phase_us = p_index->now;
                p_character->turn.side_last = true;
                p_character->turn.side_armed = true;
                p_character->turn.edge_us = p_character->edge_us;
                p_character->turn.leading_mm = 0;
                p_character->turn.edge_mm = 0;
                p_character->turn.centre_mm =
                    101.0f - j_turn_start_offset(&p_character->turn);
                p_character->turn.approach_heading = heading_deg(
                    (float)(p_index->wheel[0] - p_character->edge_wheel[0]),
                    (float)(p_index->wheel[1] - p_character->edge_wheel[1]));
                p_character->follow_ms =
                    (uint32_t)((p_character->edge_us - p_character->leg_since) /
                               1000);
                p_character->busy = true;
                p_character->candidate = false;
                p_character->centre_ms = p_character->turn_ms =
                    p_character->align_ms = 0;
            }
        }
        if (!p_character->busy)
        {
            return demand;
        }
    }
    junction_input_t local = {
        .now = p_index->now,
        .path = (float)(p_index->path - p_character->edge_path),
        .wheel = {(float)(p_index->wheel[0] - p_character->edge_wheel[0]),
                  (float)(p_index->wheel[1] - p_character->edge_wheel[1])},
        .speed = {p_index->speed[0], p_index->speed[1]},
        .adc = {p_index->adc[0], p_index->adc[1]},
        .side_black = p_index->side_black};
    unsigned before = p_character->turn.phase;
    uint64_t phase_start = p_character->turn.phase_us;
    p_character->cruise_timing = false;
    if (before == J_ACQUIRE)
    {
        demand = circuit_join(p_character, p_cal, &local);
    }
    else
    {
        demand = junction_step(&p_character->turn, p_cal, &local);
        if (before == J_TURN && p_character->turn.phase == J_ACQUIRE)
        {
            p_character->join_timing = false;
            p_character->joining =
                (circuit_tracker_t){.core = p_character->turn.follower};
            demand = circuit_join(p_character, p_cal, &local);
        }
    }
    p_character->phase = p_character->turn.phase;
    if (before == J_CENTRE && p_character->phase == J_TURN)
    {
        p_character->centre_ms =
            (uint16_t)((p_index->now - phase_start) / 1000);
        if (fabsf(p_index->speed[0]) > 65 || fabsf(p_index->speed[1]) > 65)
        {
            circuit_fail(p_character, C_APPROACH_FAST);
        }
    }
    if (before == J_TURN && p_character->phase == J_ACQUIRE)
    {
        p_character->turn_ms = (uint16_t)((p_index->now - phase_start) / 1000);
    }
    if (before == J_ACQUIRE && p_character->phase == J_EXIT)
    {
        p_character->align_ms = (uint16_t)((p_index->now - phase_start) / 1000);
        ++p_character->turns;
        p_character->completed = true;
        p_character->busy = false;
        p_character->phase = C_FOLLOW;
        p_character->gate_timing = p_character->white_timing = false;
        p_character->candidate = false;
        p_character->side_last = p_index->side_black;
        p_character->clear_path = p_index->path + C_CLEAR_MM;
        p_character->leg_since = p_index->now;
        p_character->follower = p_character->joining;
        p_character->follow_path = p_index->path;
        demand = circuit_follow(
            p_character, p_cal, p_index,
            profile); /* resume, never the old ten-second finish */
    }
    else if (p_character->phase == J_CENTRE)
    {
        for (unsigned wheel = 0; wheel < 2; ++wheel)
        {
            if (demand.target[wheel] > 0)
            {
                demand.target[wheel] += (int16_t)((profile - 1) * 5);
            }
        }
    }
    else if (p_character->phase == J_TURN)
    {
        if (demand.target[1] > 0)
        {
            demand.target[1] =
                (int16_t)(demand.target[1] +
                          (profile - 1) * (demand.target[1] >= 45 ? 10 : 5));
        }
    }
    if (demand.stop || p_character->turn.phase == J_FAULT)
    {
        circuit_fail(p_character, p_character->reason
                                      ? p_character->reason
                                      : p_character->turn.reason);
    }
    if (p_character->fault)
    {
        demand.stop = true;
        demand.target[0] = demand.target[1] = 0;
    }
    return demand;
}
#endif

/* End of file. */
