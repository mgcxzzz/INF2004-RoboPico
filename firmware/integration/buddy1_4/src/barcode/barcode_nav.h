/**
 * @file barcode_nav.h
 * @brief Barcode nav.
 * @details Pure barcode/route logic is called by the ADC owner and emits
 * requests. Native records are not wire packets; preserve both decode
 * directions.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BARCODE_NAV_H
#define BARCODE_NAV_H
#include "src/line_following/circuit_logic.h"
#include "src/barcode/barcode_scan.h"
#ifdef BUDDY_COORDINATED_BARCODE
#include "include/interfaces/nav_api.h"
#endif
#include "src/barcode/uturn_logic.h"
#ifdef BUDDY_RUNTIME_MAPPING
#include "src/network/wifi_protocol.h"
#endif

#ifndef BARCODE_SENSOR_AHEAD_MM
#define BARCODE_SENSOR_AHEAD_MM 92
#endif
#ifndef BARCODE_CONFIRM_GATE
#define BARCODE_CONFIRM_GATE 0
#endif
enum
{
    B_CROSS_MIN_MM = 11,
    B_CROSS_MAX_MM = 30,
    B_CROSS_WHITE_MM = 4,
    B_SIDE_AHEAD_MM = BARCODE_SENSOR_AHEAD_MM,
    B_EDGE_FAILURE = 46
};
typedef struct
{
    barcode_reader_t reader;
    barcode_event_t events[B_EVENTS];
    uint32_t event_count;
    uint32_t event_head;
    uint32_t event_overwritten;
    uint32_t event_sequence;
    uint32_t decoded_a;
    uint32_t decoded_f;
    uint32_t unsupported;
    uint32_t rejected;
    uint32_t lefts;
    uint32_t rights;
    uint32_t straight_crosses;
    uint32_t decoded_z;
    uint32_t uturns;
    barcode_u_turn_t uturn;
    uint64_t start_us;
    uint64_t leading_us;
    uint64_t cross_us;
    double leading_wheel[2];
    double cross_wheel[2];
    double cross_width;
    double cross_trailing_mm;
    bool leading_gate;
    bool cross_gate;
    bool cross_ready;
    bool cross_candidate;
    bool busy;
    bool allow_cross;
    bool external_reader;
    bool external_scanning;
    bool external_cross; /* Optional independent junction sensor; reference
                            defaults false. */
    char pending;
    char action;
    char last_code;
    /* Routes capture the action and map revision at decode, not at dispatch. */
    uint32_t mapping;
    uint32_t decoded_total;
    char pending_action;
    char executing_action;
    uint8_t pending_revision;
    uint8_t executing_revision;
} barcode_nav_t;
static inline void barcode_event (barcode_nav_t * p_count, unsigned type,
                                  uint64_t now, double mm_value, double width,
                                  char code, char action, unsigned flags,
                                  const uint16_t * p_pattern)
{
    barcode_event_t * p_event = &p_count->events[p_count->event_head];
    memset(p_event, 0, sizeof(*p_event));
    p_event->ms = (now - p_count->start_us) / 1000;
    p_event->path10 = mm_value > 0 ? (uint32_t)(mm_value * 10) : 0;
    p_event->width100 = (int32_t)(width * 100);
    p_event->number = ++p_count->event_sequence;
    p_event->type = (uint8_t)type;
    p_event->code = (uint8_t)code;
    p_event->action = (uint8_t)action;
    p_event->flags = (uint8_t)flags;
    p_event->map_revision =
        (B_DECODE == type || B_ANALOG_SELECT == type)
            ? (uint8_t)(p_count->mapping >> 21)
        : code && (code == p_count->pending) ? p_count->pending_revision
        : code && (code == p_count->action)  ? p_count->executing_revision
                                            : (uint8_t)(p_count->mapping >> 21);
    if (p_pattern)
    {
        memcpy(p_event->pattern, p_pattern, sizeof(p_event->pattern));
    }
    p_count->event_head = (p_count->event_head + 1) % B_EVENTS;
    if (p_count->event_count < B_EVENTS)
    {
        ++p_count->event_count;
    }
    else
    {
        ++p_count->event_overwritten;
    }
}
static inline void barcode_nav_begin (barcode_nav_t * p_count,
                                      const circuit_input_t * p_index)
{
    *p_count = (barcode_nav_t){.start_us = p_index->now};
    barcode_reader_reset(&p_count->reader, p_index->side_black, p_index->now,
                         (p_index->wheel[0] + p_index->wheel[1]) * .5);
}
static inline void
barcode_recovery_events (barcode_nav_t * p_count, uint64_t now, double position,
                         bool was_white, bool was_waiting, bool was_search,
                         unsigned previous_decision,
                         const circuit_tracker_t * p_sample,
                         const analog_demand_t * p_demand, unsigned reason)
{
#ifdef CIRCUIT_SWEEP_SEARCH
    if (((!was_waiting) && (!was_search)) && (p_sample->search.waiting))
    {
        barcode_event(p_count, B_RECOVERY_WAIT, now, position,
                      LS_CONFIRM_US / 1000, 0, '-', 0, NULL);
    }
    if ((((p_sample->search.active) &&
          ((!was_search) || (p_demand->decision != previous_decision))) &&
         (p_demand->decision >= LS_DECISION_FIRST)) &&
        (p_demand->decision < LS_DECISION_FIRST + 5))
    {
        int speed = p_demand->target[0] > p_demand->target[1]
                        ? p_demand->target[0]
                        : p_demand->target[1];
        barcode_event(p_count, B_RECOVERY_SWEEP, now, position, speed, 0,
                      p_demand->target[1] > p_demand->target[0] ? 'L' : 'R',
                      p_demand->decision - LS_DECISION_FIRST + 1, NULL);
    }
    if (((!p_demand->stop) && was_white) && (!p_sample->core.white))
    {
        barcode_event(p_count, B_RECOVERY_FOUND, now, position, 0, 0, '-',
                      was_search ? 1 : 0, NULL);
    }
    if ((p_demand->stop) && was_search)
    {
        barcode_event(p_count, B_RECOVERY_FAIL, now, position, reason, 0, '-',
                      0, NULL);
    }
#else
    (void)p_count;
    (void)now;
    (void)position;
    (void)was_white;
    (void)was_waiting;
    (void)was_search;
    (void)previous_decision;
    (void)p_sample;
    (void)p_demand;
    (void)reason;
#endif
}
static inline char barcode_action (char code)
{
    return 'A' == code ? 'L' : 'F' == code ? 'R' : 'Z' == code ? 'U' : '-';
}
static inline char barcode_decode_action (const barcode_nav_t * p_count,
                                          char code)
{
#ifdef BUDDY_RUNTIME_MAPPING
    return wp_map_action(p_count->mapping, code);
#else
    (void)p_count;
    return barcode_action(code);
#endif
}
static inline char barcode_nav_action (const barcode_nav_t * p_count, char code)
{
#ifdef BUDDY_RUNTIME_MAPPING
    if ((code && (code == p_count->pending)) && (p_count->pending_action))
    {
        return p_count->pending_action;
    }
    if ((code && (code == p_count->action)) && (p_count->executing_action))
    {
        return p_count->executing_action;
    }
    return wp_map_action(p_count->mapping, code);
#else
    (void)p_count;
    return barcode_action(code);
#endif
}
static inline bool barcode_set_pending (barcode_nav_t * p_count, char code)
{
#ifdef BUDDY_RUNTIME_MAPPING
    char action = wp_map_action(p_count->mapping, code);
    if ('-' == action)
    {
        return false;
    }
    p_count->pending = code;
    p_count->pending_action = action;
    p_count->pending_revision = (uint8_t)(p_count->mapping >> 21);
    return true;
#else
    if (code != 'A' && code != 'F' && code != 'Z')
    {
        return false;
    }
    p_count->pending = code;
    return true;
#endif
}
static inline void barcode_queue_code (barcode_nav_t * p_count, char code)
{
#ifdef BUDDY_COORDINATED_BARCODE
    if (nav_accept_barcode(MOTION_BUDDY3, code) != MOTION_OK)
    {
        return;
    }
#endif
    p_count->last_code = code;
    ++p_count->decoded_total;
    if ('A' == code)
    {
        ++p_count->decoded_a;
    }
    else if ('F' == code)
    {
        ++p_count->decoded_f;
    }
    else if ('Z' == code)
    {
        ++p_count->decoded_z;
    }
    if (!barcode_set_pending(p_count, code))
    {
        ++p_count->unsupported;
    }
}
static inline void barcode_cross_confirm (barcode_nav_t * p_count, uint64_t now,
                                          double mm_value, bool white)
{
    if (((p_count->cross_candidate) && white) &&
        (mm_value - p_count->cross_trailing_mm >= B_CROSS_WHITE_MM))
    {
        p_count->cross_candidate = false;
        p_count->cross_ready = true;
        barcode_event(p_count, B_CROSS, now, mm_value, p_count->cross_width,
                      p_count->pending,
                      barcode_nav_action(p_count, p_count->pending), 0, NULL);
    }
}
static inline void barcode_nav_edge (barcode_nav_t * p_count, uint64_t now,
                                     const double wheel[2], bool black)
{
    double mm_value = (wheel[0] + wheel[1]) * .5;
    if (p_count->busy)
    {
        barcode_reader_reset(&p_count->reader, black, now, mm_value);
        return;
    }
    bool was_frame = p_count->reader.frame;
    if (black && (!p_count->reader.black))
    {
        /* Confirm by encoder travel, even if this next edge arrives between
           control ticks. A new bar inside4mm cancels, never consumes pending.
         */
        barcode_cross_confirm(p_count, now, mm_value, true);
        if (p_count->cross_candidate)
        {
            p_count->cross_candidate = false;
            barcode_event(p_count, B_CROSS_CANCEL, now, mm_value,
                          mm_value - p_count->cross_trailing_mm,
                          p_count->pending, 0, 0, NULL);
        }
        memcpy(p_count->leading_wheel, wheel, sizeof(p_count->leading_wheel));
        p_count->leading_us = now;
        p_count->leading_gate = p_count->allow_cross;
    }
    barcode_read_t result =
        barcode_reader_edge(&p_count->reader, black, now, mm_value);
    if (result.events & (1u << B_RUN))
    {
        barcode_event(p_count, B_RUN, now, mm_value, result.width, 0, 0,
                      result.completed_black, NULL);
    }
    if (result.events & (1u << B_FRAME_START))
    {
        barcode_event(p_count, B_FRAME_START, now, mm_value,
                      mm_value - result.start, 0, 0, p_count->reader.reverse,
                      NULL);
    }
    if (result.events & (1u << B_BAD_FRAME))
    {
        ++p_count->rejected;
        barcode_event(p_count, B_BAD_FRAME, now, mm_value,
                      mm_value - result.start, '?', 0, result.rejection,
                      result.pattern);
    }
    if (result.events & (1u << B_DECODE))
    {
        barcode_queue_code(p_count, result.code);
        barcode_event(p_count, B_DECODE, now, mm_value, mm_value - result.start,
                      result.code, barcode_decode_action(p_count, result.code),
                      result.reverse, result.pattern);
    }
    /* A complete barcode is required before queuing a route command. Frame
       context protects its bars, even when they are wider than the cross.
       Cross uses a separate isolated black run after its trailing edge. */
    if ((((((result.completed_black) && (!was_frame)) &&
           (!(result.events & (1u << B_FRAME_START)))) &&
          (p_count->reader.history_count <= 2)) &&
         (result.width >= B_CROSS_MIN_MM)) &&
        (result.width <= B_CROSS_MAX_MM))
    {
        p_count->cross_width = result.width;
        p_count->cross_us = p_count->leading_us;
        memcpy(p_count->cross_wheel, p_count->leading_wheel,
               sizeof(p_count->cross_wheel));
        p_count->cross_gate = p_count->leading_gate;
        p_count->cross_candidate = true;
        p_count->cross_trailing_mm = mm_value;
        barcode_event(p_count, B_CROSS_CANDIDATE, now, mm_value, result.width,
                      p_count->pending,
                      barcode_nav_action(p_count, p_count->pending), 0, NULL);
    }
}
static inline void barcode_mirror_tracker (circuit_tracker_t * p_sample)
{
    p_sample->core.direction = -p_sample->core.direction;
    p_sample->core.recovery = -p_sample->core.recovery;
    p_sample->pending = -p_sample->pending;
    p_sample->reverse = -p_sample->reverse;
    p_sample->probe_direction = -p_sample->probe_direction;
    int16_t value = p_sample->bridge[0];
    p_sample->bridge[0] = p_sample->bridge[1];
    p_sample->bridge[1] = value;
#ifdef CIRCUIT_SWEEP_SEARCH
    p_sample->search.first_bits =
        compact_signed_pack(-line_search_direction(&p_sample->search), 2U);
#endif
}
static inline void barcode_straight_gate (circuit_t * p_character,
                                          barcode_nav_t * p_count,
                                          const circuit_input_t * p_index,
                                          const analog_demand_t * p_demand)
{
    bool visible = (p_demand->level[0] >= 400) || (p_demand->level[1] >= 400);
#ifdef CIRCUIT_SWEEP_SEARCH
    visible = visible && (!p_character->follower.core.white);
#endif
    if (BARCODE_CONFIRM_GATE)
    {
        /* Step4: eligibility is rechecked at confirmation, not latched at
         * the first edge. Ordinary corrections need no yaw/speed dwell. */
        p_count->allow_cross =
            visible && (p_index->path >= p_character->clear_path);
        return;
    }
    bool straight =
        (visible && (p_index->speed[0] >= 10)) && (p_index->speed[1] >= 10);
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
            memcpy(p_character->gate_wheel, p_index->wheel,
                   sizeof(p_character->gate_wheel));
        }
        float yaw = heading_deg(
            (float)(p_index->wheel[0] - p_character->gate_wheel[0]),
            (float)(p_index->wheel[1] - p_character->gate_wheel[1]));
        if (fabsf(yaw) > 3)
        {
            p_character->gate_since = p_index->now;
            memcpy(p_character->gate_wheel, p_index->wheel,
                   sizeof(p_character->gate_wheel));
        }
    }
    p_count->allow_cross =
        ((p_character->gate_timing) &&
         (p_index->now - p_character->gate_since >= C_GATE_US)) &&
        (p_index->path >= p_character->clear_path);
}
enum
{
    B_GATE_APPROACH = 1,
    B_GATE_CLEARANCE = 2,
    B_GATE_LINE_LOST = 4,
    B_GATE_NOT_VISIBLE = 8,
    B_GATE_SEARCH = 16
};
static inline unsigned barcode_cross_rejection (
    const circuit_t * p_character, const barcode_nav_t * p_count,
    const circuit_input_t * p_index, const analog_demand_t * p_demand)
{
    unsigned flags =
        (!BARCODE_CONFIRM_GATE) && (!p_count->cross_gate) ? B_GATE_APPROACH : 0;
    if (p_index->path < p_character->clear_path)
    {
        flags |= B_GATE_CLEARANCE;
    }
#ifdef CIRCUIT_SWEEP_SEARCH
    if (p_character->follower.core.white)
    {
        flags |= B_GATE_LINE_LOST;
    }
    if (BARCODE_CONFIRM_GATE && (p_character->follower.search.active))
    {
        flags |= B_GATE_SEARCH;
    }
#endif
    if ((BARCODE_CONFIRM_GATE && (p_demand->level[0] < 400)) &&
        (p_demand->level[1] < 400))
    {
        flags |= B_GATE_NOT_VISIBLE;
    }
    return flags;
}
/* Right turns reuse the proven rolling-left geometry in a virtual mirror:
   swap front calibration, ADCs, wheel distances/speeds, then swap outputs.
   Real-world following/recovery state is restored after acquisition. */
static inline analog_demand_t
barcode_circuit_step (circuit_t * p_character, barcode_nav_t * p_count,
                      const line_calibration_t * p_cal,
                      const circuit_input_t * p_index, unsigned profile)
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
    double mean = (p_index->wheel[0] + p_index->wheel[1]) * .5;
    if (!p_character->busy)
    {
        if (!p_count->external_cross)
        {
            barcode_cross_confirm(p_count, p_index->now, mean,
                                  !p_index->side_black);
        }
        if ((!p_count->external_reader) &&
            (barcode_reader_expire(&p_count->reader, p_index->now, mean)))
        {
            ++p_count->rejected;
            barcode_event(p_count, B_BAD_FRAME, p_index->now, mean,
                          mean - p_count->reader.start_mm, '?', 0,
                          B_REJECT_TIMEOUT, NULL);
        }
        /* Slow scanning and the approach with a pending command; encoding
           remains distance-based even as measured speed changes. */
        bool scan = p_count->external_reader
                        ? ((p_count->external_scanning) || (p_count->pending))
                        : ((((p_count->reader.frame) ||
                             (p_count->reader.history_count)) ||
                            (p_count->reader.black)) ||
                           (p_count->pending));
        demand =
            circuit_follow(p_character, p_cal, p_index, scan ? 1 : profile);
        if (demand.stop)
        {
            circuit_fail(p_character, p_character->follower.loss_reason
                                          ? p_character->follower.loss_reason
                                          : J_LINE_LOST);
            return demand;
        }
        barcode_straight_gate(p_character, p_count, p_index, &demand);
        if (p_count->cross_ready)
        {
            p_count->cross_ready = false;
            unsigned rejection =
                barcode_cross_rejection(p_character, p_count, p_index, &demand);
            if (!p_count->pending)
            {
                ++p_count->straight_crosses;
                barcode_event(p_count, B_NO_COMMAND, p_index->now, mean,
                              p_count->cross_width, 0, 'S', 0, NULL);
            }
            else if (rejection)
            {
                ++p_character->ignored;
                barcode_event(p_count, B_GATE_REJECT, p_index->now, mean,
                              p_count->cross_width, p_count->pending, 0,
                              BARCODE_CONFIRM_GATE ? rejection : 0, NULL);
            }
            else
            {
                p_count->action = p_count->pending;
                p_count->executing_action =
                    barcode_nav_action(p_count, p_count->pending);
                p_count->executing_revision = p_count->pending_revision;
                p_count->pending = 0;
                p_count->pending_action = 0;
                p_count->pending_revision = 0;
#ifdef BUDDY_RUNTIME_MAPPING
                if ('S' == p_count->executing_action)
                {
                    ++p_count->straight_crosses;
                    barcode_event(p_count, B_DISPATCH, p_index->now, mean,
                                  p_count->cross_width, p_count->action, 'S', 0,
                                  NULL);
                    barcode_event(p_count, B_COMPLETE, p_index->now, mean, 0,
                                  p_count->action, 'S', 0, NULL);
                    p_count->action = 0;
                    p_count->executing_action = 0;
                    p_count->executing_revision = 0;
                    return demand;
                }
#endif
                p_count->busy = p_character->busy = true;
                p_count->uturn = (barcode_u_turn_t){0};
                p_character->cruise_timing = p_character->fast = false;
                p_character->edge_us = p_count->cross_us;
                memcpy(p_character->edge_wheel, p_count->cross_wheel,
                       sizeof(p_character->edge_wheel));
                p_character->edge_path =
                    p_index->path -
                    (mean -
                     (p_count->cross_wheel[0] + p_count->cross_wheel[1]) * .5);
                junction_input_t begin = {.now = p_index->now,
                                          .side_black = false};
                j_begin_mode(&p_character->turn, 3, &begin);
                p_character->turn.phase = J_CENTRE;
                p_character->turn.striped = true;
                p_character->turn.side_armed = true;
                p_character->turn.leading_mm = 0;
                p_character->turn.trailing_mm = (float)p_count->cross_width;
                p_character->turn.centre_mm =
                    (float)p_count->cross_width * .5f + B_SIDE_AHEAD_MM -
                    j_turn_start_offset(&p_character->turn);
                p_character->turn.approach_heading = 0;
                p_character->follow_ms =
                    (uint32_t)((p_character->edge_us - p_character->leg_since) /
                               1000);
                p_character->centre_ms = p_character->turn_ms =
                    p_character->align_ms = 0;
                barcode_event(p_count, B_DISPATCH, p_index->now, mean,
                              p_count->cross_width, p_count->action,
                              barcode_nav_action(p_count, p_count->action), 0,
                              NULL);
                barcode_reader_reset(&p_count->reader, p_index->side_black,
                                     p_index->now, mean);
            }
        }
        if (!p_character->busy)
        {
            return demand;
        }
    }
    bool right = 'R' == barcode_nav_action(p_count, p_count->action);
    unsigned first = right ? 1 : 0, second = 1 - first;
    line_calibration_t virtual_cal = {
        {p_cal->white[first], p_cal->white[second]},
        {p_cal->black[first], p_cal->black[second]},
        p_cal->mask};
    junction_input_t local = {
        .now = p_index->now,
        .path = (float)(p_index->path - p_character->edge_path),
        .wheel =
            {(float)(p_index->wheel[first] - p_character->edge_wheel[first]),
             (float)(p_index->wheel[second] - p_character->edge_wheel[second])},
        .speed = {p_index->speed[first], p_index->speed[second]},
        .adc = {p_index->adc[first], p_index->adc[second]},
        .side_black = p_index->side_black};
    unsigned before = p_character->turn.phase;
    uint64_t phase_start = p_character->turn.phase_us;
    p_character->cruise_timing = false;
    if (J_ACQUIRE == before)
    {
        demand = circuit_join(p_character, &virtual_cal, &local);
    }
    else
    {
        unsigned stage = p_count->uturn.stage;
        demand = 'U' == barcode_nav_action(p_count, p_count->action)
                     ? barcode_uturn_step(&p_count->uturn, &p_character->turn,
                                          &virtual_cal, &local)
                     : junction_step(&p_character->turn, &virtual_cal, &local);
        if ('U' == barcode_nav_action(p_count, p_count->action) &&
            (stage != p_count->uturn.stage))
        {
            barcode_event(p_count, B_UTURN_STAGE, p_index->now, mean,
                          p_character->turn.turn_degrees, p_count->action, 'U',
                          p_count->uturn.stage, NULL);
        }
        if ((J_TURN == before ||
             ('U' == barcode_nav_action(p_count, p_count->action) &&
              J_TURN_SETTLE == before)) &&
            J_ACQUIRE == p_character->turn.phase)
        {
            p_character->join_timing = false;
            p_character->joining =
                (circuit_tracker_t){.core = p_character->turn.follower};
            demand = circuit_join(p_character, &virtual_cal, &local);
        }
    }
    p_character->phase = p_character->turn.phase;
    if (J_CENTRE == before && J_TURN == p_character->phase)
    {
        p_character->centre_ms =
            (uint16_t)((p_index->now - phase_start) / 1000);
        if ((fabsf(p_index->speed[0]) > 65) || (fabsf(p_index->speed[1]) > 65))
        {
            circuit_fail(p_character, C_APPROACH_FAST);
        }
    }
    if ((J_TURN == before ||
         ('U' == barcode_nav_action(p_count, p_count->action) &&
          J_TURN_SETTLE == before)) &&
        J_ACQUIRE == p_character->phase)
    {
        p_character->turn_ms =
            (uint16_t)((p_index->now -
                        ('U' == barcode_nav_action(p_count, p_count->action)
                             ? p_count->uturn.since
                             : phase_start)) /
                       1000);
    }
    if (J_ACQUIRE == before && J_EXIT == p_character->phase)
    {
        p_character->align_ms = (uint16_t)((p_index->now - phase_start) / 1000);
        ++p_character->turns;
        p_character->completed = true;
        if ('U' == barcode_nav_action(p_count, p_count->action))
        {
            ++p_count->uturns;
        }
        else if (right)
        {
            ++p_count->rights;
        }
        else
        {
            ++p_count->lefts;
        }
        p_character->busy = p_count->busy = false;
        p_character->phase = C_FOLLOW;
        p_character->gate_timing = p_character->white_timing = false;
        p_count->allow_cross = false;
        p_character->clear_path = p_index->path + C_CLEAR_MM;
        p_character->leg_since = p_index->now;
        p_character->follower = p_character->joining;
        if (right)
        {
            barcode_mirror_tracker(&p_character->follower);
        }
        p_character->follow_path = p_index->path;
        barcode_event(p_count, B_COMPLETE, p_index->now, mean,
                      p_character->turn.turn_degrees, p_count->action,
                      barcode_nav_action(p_count, p_count->action), 0, NULL);
        barcode_reader_reset(&p_count->reader, p_index->side_black,
                             p_index->now, mean);
        return circuit_follow(p_character, p_cal, p_index, profile);
    }
    if (J_CENTRE == p_character->phase)
    {
        for (unsigned wheel = 0; wheel < 2; ++wheel)
        {
            if (demand.target[wheel] > 0)
            {
                demand.target[wheel] += (int16_t)((profile - 1) * 5);
            }
        }
    }
    else if ((barcode_nav_action(p_count, p_count->action) != 'U') &&
             J_TURN == p_character->phase && (demand.target[1] > 0))
    {
        demand.target[1] =
            (int16_t)(demand.target[1] +
                      (profile - 1) * (demand.target[1] >= 45 ? 10 : 5));
    }
    if (right)
    {
        int16_t speed = demand.target[0];
        demand.target[0] = demand.target[1];
        demand.target[1] = speed;
        uint16_t level = demand.level[0];
        demand.level[0] = demand.level[1];
        demand.level[1] = level;
        if (SYM_LEFT == demand.decision)
        {
            demand.decision = SYM_RIGHT;
        }
        else if (SYM_RIGHT == demand.decision)
        {
            demand.decision = SYM_LEFT;
        }
        else if (SYM_RECOVER_LEFT == demand.decision)
        {
            demand.decision = SYM_RECOVER_RIGHT;
        }
        else if (SYM_RECOVER_RIGHT == demand.decision)
        {
            demand.decision = SYM_RECOVER_LEFT;
        }
        else if (C_PROBE_LEFT == demand.decision)
        {
            demand.decision = C_PROBE_RIGHT;
        }
        else if (C_PROBE_RIGHT == demand.decision)
        {
            demand.decision = C_PROBE_LEFT;
        }
    }
    if ((demand.stop) || J_FAULT == p_character->turn.phase)
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
