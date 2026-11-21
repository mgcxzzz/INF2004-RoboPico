/**
 * @file b5_mission.c
 * @brief Buddy5 obstacle mission sequencing (see b5_mission.h).
 * @details Owner: the core1 motion task, one step per 10 ms frame right
 * after motion_update(). No blocking, no sensor or motor register access:
 * sonar/servo go through the B5SensorPort, motion through nav_api.h with one
 * Buddy5 lease. The mission uses the long width probe's edge evidence to
 * choose a side, restores the line pose, and runs the rectangular bypass.
 * @see app_program/obstacle_navigation/README.md
 */
#include "b5_mission.h"
#include "width_retry_policy.h"
#include "src/line_following/symmetric_logic.h"
#include <math.h>
#include <string.h>

/* Stationary poses about the width-reference heading. */
static const float g_align_deg[B5MI_ALIGN_STEPS] = {-2.0f, 2.0f,  -4.0f,
                                                    4.0f,  -6.0f, 6.0f};

static bool still (motion_status_t status)
{
    return (MOTION_PAUSED == status.mode) && (status.stopped);
}

static bool owns (const b5_mission_t * p_mission, motion_status_t status)
{
    return (status.session_active) && (MOTION_BUDDY5 == status.owner) &&
           (status.lease == p_mission->lease) && (0U == status.fault) &&
           (status.mode != MOTION_FAULT);
}

static unsigned line_mask (const b5_mission_t * p_mission)
{
    return symmetric_line_mask(p_mission->line[0], p_mission->line[1]);
}

/* Fresh line evidence: a sensor that saw the line at the takeover sees it
 * again (or any sensor when none was above the level at the takeover). */
static bool line_evidence (const b5_mission_t * p_mission)
{
    unsigned mask = line_mask(p_mission);
    if (0U == p_mission->takeover_mask)
    {
        return mask != 0U;
    }
    return (mask & p_mission->takeover_mask) != 0U;
}

static bool fresh_observation (b5_mission_t * p_mission)
{
    if (p_mission->used_observation == p_mission->observed)
    {
        return false;
    }
    p_mission->used_observation = p_mission->observed;
    return true;
}

static void enter (b5_mission_t * p_mission, b5mi_stage_t stage, uint64_t now)
{
    p_mission->stage = stage;
    p_mission->stage_since = now;
    p_mission->sub_since = now;
    p_mission->sub = 0U;
    p_mission->evidence_timing = false;
    p_mission->confirm = 0U;
    p_mission->close_count = 0U;
    p_mission->ping_pending = false;
    p_mission->progress_id = 0U;
}

static void servo_front (b5_mission_t * p_mission, uint64_t now)
{
    p_mission->sensor.cancel();
    p_mission->sensor.angle(0, now);
    p_mission->ping_pending = false;
}

enum
{
    B5_RETRY_STRAIGHT_GRACE = 1,
    B5_RETRY_STRAIGHT_STOPPED,
    B5_RETRY_WIDTH_GATE,
    B5_RETRY_APPROACH_ALIGN
};
enum
{
    B5_RETRY_REQUESTED,
    B5_RETRY_RECOVERED,
    B5_RETRY_HELD,
    B5_RANGE_BORDERLINE
};

static void retry_note (b5_mission_t * p_mission, unsigned kind,
                        unsigned result, unsigned reason, uint64_t now)
{
    B5Event event = {.ms = (uint32_t)(now / 1000U),
                     .code = B5V_MISSION_RETRY,
                     .detail = (uint8_t)kind,
                     .result = (uint8_t)result,
                     .reason = (uint8_t)reason,
                     .phase = (uint8_t)p_mission->stage};
    b5_auto_queue_event(&p_mission->bypass, &event);
}

/* Stopped hold with the lease kept. The module that ended has already
 * stopped; a mission move is stopped here. Never clears another fault. */
static void hold (b5_mission_t * p_mission, b5mi_reason_t reason,
                  unsigned module_reason, uint64_t now)
{
    motion_status_t status = nav_get_status();
    /* v0.6.1 stop(): also stops line following when the guard fails. */
    if ((status.session_active) && (status.mode != MOTION_FAULT) &&
        (!still(status)))
    {
        (void)nav_request_stop(MOTION_BUDDY5);
    }
    p_mission->sensor.cancel();
    if (p_mission->guard_rechecking)
    {
        retry_note(p_mission, B5_RETRY_STRAIGHT_STOPPED, B5_RETRY_HELD, reason,
                   now);
        p_mission->guard_rechecking = false;
    }
    else if (p_mission->guard_grace_since != 0U)
    {
        retry_note(p_mission, B5_RETRY_STRAIGHT_GRACE, B5_RETRY_HELD, reason,
                   now);
    }
    p_mission->guard_grace_since = 0U;
    if (p_mission->width_rechecked && !p_mission->width_started &&
        (B5MI_WIDTH == p_mission->after_range))
    {
        retry_note(p_mission, B5_RETRY_WIDTH_GATE, B5_RETRY_HELD, reason, now);
    }
    if (p_mission->approach_retried && (B5MI_ALIGN == p_mission->stage))
    {
        retry_note(p_mission, B5_RETRY_APPROACH_ALIGN, B5_RETRY_HELD, reason,
                   now);
    }
    p_mission->reason = reason;
    p_mission->module_reason = (uint8_t)module_reason;
    enter(p_mission, B5MI_HOLD, now);
}

/* Discard unsent records of a PREVIOUS run so the frozen probe can start.
 * Counted and reported (E5), never silent. */
static void drain_width (b5_mission_t * p_mission)
{
    B5WPRec record;
    while (b5_wp_peek(&p_mission->width, &record))
    {
        b5_wp_drop(&p_mission->width);
        p_mission->stale_dropped++;
    }
    if (p_mission->width.summary_pending)
    {
        p_mission->width.summary_pending = false;
        p_mission->stale_dropped++;
    }
    if (p_mission->width.result_pending)
    {
        p_mission->width.result_pending = false;
        p_mission->stale_dropped++;
    }
    while (p_mission->width.retry_sent < p_mission->width.retry_count)
    {
        p_mission->width.retry_sent++;
        p_mission->stale_dropped++;
    }
    if (p_mission->width.limits_pending) /* long probe D5 */
    {
        p_mission->width.limits_pending = false;
        p_mission->stale_dropped++;
    }
}

/* ---- Sonar -------------------------------------------------------------- */

/* One watch ping per call when the servo is settled. Returns true with a new
 * ping. Fault/timeout bookkeeping follows the v0.6.1 guard. */
static bool watch_ping (b5_mission_t * p_mission, uint64_t now,
                        b5_ping_t * p_ping)
{
    bool taken = false;
    if (!p_mission->sensor.settled(now))
    {
        return false;
    }
    if (p_mission->sensor.take(p_ping))
    {
        taken = true;
        p_mission->taken_requested = p_mission->ping_requested;
        p_mission->ping_pending = false;
        p_mission->last_ping = now;
        if (B5_PING_FAULT == p_ping->cls)
        {
            p_mission->fault_run++;
        }
        else
        {
            p_mission->fault_run = 0U;
        }
    }
    if (p_mission->sensor.request(now))
    {
        p_mission->ping_pending = true;
        p_mission->ping_requested = now;
    }
    return taken;
}

static unsigned watch_failure (const b5_mission_t * p_mission, uint64_t now)
{
    if (p_mission->fault_run >= 3U)
    {
        return B5MI_WATCH_FAULTS;
    }
    return ((now - p_mission->last_ping) > (uint64_t)300000U)
               ? B5MI_WATCH_TIMEOUT
               : 0U;
}

static void range_begin (b5_mission_t * p_mission, b5mi_stage_t after,
                         uint64_t now)
{
    p_mission->after_range = after;
    memset(&p_mission->range, 0, sizeof(p_mission->range));
    servo_front(p_mission, now);
    enter(p_mission, B5MI_RANGE, now);
}

static uint16_t median3 (const b5_ping_t ping[3])
{
    uint32_t value_a = ping[0].range_mm;
    uint32_t value_b = ping[1].range_mm;
    uint32_t value_c = ping[2].range_mm;
    uint32_t low = value_a < value_b ? value_a : value_b;
    uint32_t high = value_a < value_b ? value_b : value_a;
    uint32_t result = value_c < low ? low : (value_c > high ? high : value_c);
    return (uint16_t)result; /* 12-bit range field: fits uint16_t */
}

static void range_note (b5_mission_t * p_mission, unsigned result, uint64_t now)
{
    const b5mi_range_t * p_range = &p_mission->range;
    B5Event event = {.ms = (uint32_t)(now / 1000U),
                     .code = B5V_RANGE_BATCH,
                     .detail = (uint8_t)p_mission->after_range,
                     .result = (uint8_t)result,
                     .search = p_range->attempts};
    for (unsigned index = 0U; index < 3U; ++index)
    {
        uint16_t mm = (B5_PING_ECHO == p_range->ping[index].cls &&
                       p_range->ping[index].range_mm < B5_RANGE_FAR_MM)
                          ? (uint16_t)p_range->ping[index].range_mm
                          : 0U;
        if (index < 2U)
        {
            event.level[index] = mm;
        }
        else
        {
            event.bg[0] = mm;
        }
        event.bg[1] |=
            (uint16_t)((p_range->ping[index].cls & 3U) << (2U * index));
    }
    b5_auto_queue_event(&p_mission->bypass, &event);
}

static void range_complete (b5_mission_t * p_mission, uint64_t now)
{
    b5mi_range_t * p_range = &p_mission->range;
    bool clean = true;
    bool clear = true;
    uint32_t low = UINT32_MAX;
    uint32_t high = 0U;
    uint32_t sum = 0U;
    for (unsigned index = 0U; index < 3U; ++index)
    {
        if ((B5_PING_ECHO != p_range->ping[index].cls) ||
            (B5_PING_WHY_OK != p_range->ping[index].why) ||
            (0U == p_range->ping[index].range_mm) ||
            (p_range->ping[index].range_mm >= B5_RANGE_FAR_MM))
        {
            clean = false;
        }
        if ((B5_PING_NO_RETURN != p_range->ping[index].cls) &&
            ((B5_PING_ECHO != p_range->ping[index].cls) ||
             (p_range->ping[index].range_mm <= B5MI_CONFIRM_RANGE_MM)))
        {
            clear = false;
        }
        low = p_range->ping[index].range_mm < low
                  ? p_range->ping[index].range_mm
                  : low;
        high = p_range->ping[index].range_mm > high
                   ? p_range->ping[index].range_mm
                   : high;
        sum += p_range->ping[index].range_mm;
    }
    p_range->attempts++;
    bool guard = p_mission->guard_rechecking;
    if (guard && clear)
    {
        range_note(p_mission, B5_RETRY_RECOVERED, now);
        p_range->status = 1U;
        p_mission->sub_since = now;
        return;
    }
    bool width = guard || (B5MI_WIDTH == p_mission->after_range);
    uint16_t median_mm = width ? median3(p_range->ping) : 0U;
    bool borderline =
        guard && clean && width_ref_borderline(low, median_mm, high);
    uint32_t mean_mm = (sum + 1U) / 3U; /* round to nearest whole mm */
    /* A near echo wins over a request to creep forward. Re-sample that
     * conflict while stopped so the average cannot authorize an approach. */
    bool usable = width ? ((high - low) <= (uint32_t)B5MI_RANGE_SPREAD_MM)
                        : ((mean_mm <= B5MI_APPROACH_ACCEPT_MM) ||
                           (low > B5MI_APPROACH_STOP_MM));
    /* A settling takeover already had tracking and bounded heading change.
     * Require retained tape and a clean reference before starting the probe. */
    bool guard_ready =
        (!guard) ||
        ((((low >= B5WP_D0_MIN_MM) && (high <= B5WP_D0_MAX_MM)) ||
          borderline) &&
         (0U != p_mission->takeover_mask) && line_evidence(p_mission));
    if ((clean) && (usable) && (guard_ready))
    {
        range_note(p_mission,
                   borderline ? B5_RANGE_BORDERLINE : B5_RETRY_RECOVERED, now);
        p_range->range_mm = width ? median_mm : (uint16_t)mean_mm;
        p_range->nearest_mm = (uint16_t)low;
        p_range->farthest_mm = (uint16_t)high;
        p_range->status = 1U;
        if (guard)
        {
            retry_note(p_mission, B5_RETRY_STRAIGHT_STOPPED, B5_RETRY_RECOVERED,
                       B5MI_R_NONE, now);
            p_mission->guard_rechecking = false;
            p_mission->after_range = B5MI_WIDTH;
        }
        enter(p_mission, p_mission->after_range, now);
        return;
    }
    unsigned limit = width ? B5MI_RANGE_ATTEMPTS : B5MI_APPROACH_RANGE_ATTEMPTS;
    if (p_range->attempts >= limit)
    {
        range_note(p_mission, B5_RETRY_HELD, now);
        p_range->status = 2U;
        hold(p_mission, guard ? B5MI_R_NOT_STRAIGHT : B5MI_R_RANGE,
             guard ? p_mission->gate_reason : 0U, now);
        return;
    }
    range_note(p_mission, B5_RETRY_REQUESTED, now);
    p_range->npings = 0U;
    p_range->ping_active = false;
    p_range->retry_after_us = now + B5MI_RANGE_RETRY_US;
}

/* The width probe needs a tight starting reference. After side selection,
 * use the mean of three valid forward echoes and keep the nearest for safety.
 */
static void range_tick (b5_mission_t * p_mission, motion_status_t status,
                        uint64_t now)
{
    b5mi_range_t * p_range = &p_mission->range;
    if (!still(status))
    {
        hold(p_mission, B5MI_R_MOVE, 0U, now);
        return;
    }
    if ((now - p_mission->stage_since) > (uint64_t)B5MI_HALT_LIMIT_US)
    {
        hold(p_mission, B5MI_R_SENSOR, 0U, now);
        return;
    }
    if (now < p_range->retry_after_us)
    {
        return;
    }
    if (p_mission->guard_rechecking && (1U == p_range->status))
    {
        if ((now - p_mission->sub_since) > B5MI_EVIDENCE_WAIT_US)
        {
            hold(p_mission, B5MI_R_NO_LINE_ALIGN, 0U, now);
            return;
        }
        motion_result_t result =
            nav_resume_line(MOTION_BUDDY5, p_mission->lease, false);
        if (MOTION_OK == result)
        {
            retry_note(p_mission, B5_RETRY_STRAIGHT_STOPPED, B5_RETRY_RECOVERED,
                       B5MI_R_NONE, now);
            p_mission->guard_rechecking = false;
            p_mission->lease = 0U;
            p_mission->last_ping = now;
            p_mission->fault_run = 0U;
            enter(p_mission, B5MI_GUARD, now);
        }
        else if (MOTION_NEED_LINE != result)
        {
            hold(p_mission, B5MI_R_MOVE, (unsigned)result, now);
        }
        return;
    }
    if ((p_range->npings < 3U) && (!p_range->ping_active))
    {
        if (p_mission->sensor.settled(now) && p_mission->sensor.request(now))
        {
            p_range->ping_active = true;
            p_range->ping_since = now;
        }
        return;
    }
    if (p_range->npings < 3U)
    {
        b5_ping_t ping;
        if (p_mission->sensor.take(&ping))
        {
            p_range->ping[p_range->npings++] = ping;
            p_range->ping_active = false;
        }
        else if ((now - p_range->ping_since) >= (uint64_t)B5MI_PING_TIMEOUT_US)
        {
            p_mission->sensor.cancel();
            p_range->ping[p_range->npings++] =
                (b5_ping_t){.cls = B5_PING_FAULT, .why = 7U};
            p_range->ping_active = false;
        }
        if (p_range->npings < 3U)
        {
            return;
        }
    }
    range_complete(p_mission, now);
}

/* ---- Mission moves (alignment / approach) -------------------------------- */

static bool start_move (b5_mission_t * p_mission, bool turn, float goal,
                        unsigned speed, uint64_t now)
{
    motion_result_t result =
        turn ? nav_turn_relative(MOTION_BUDDY5, p_mission->lease, goal, speed,
                                 MOTION_IN_PLACE, &p_mission->command)
             : nav_drive_distance(MOTION_BUDDY5, p_mission->lease, goal, speed,
                                  &p_mission->command);
    if (result != MOTION_OK)
    {
        hold(p_mission, B5MI_R_MOVE, (unsigned)result, now);
        return false;
    }
    p_mission->generation = nav_get_status().generation;
    p_mission->halting_move = false;
    p_mission->progress_id = 0U;
    p_mission->sub_since = now;
    return true;
}

/* 0 running, 1 finished (completed or our own stop), -1 held. */
static int follow_move (b5_mission_t * p_mission, motion_status_t status,
                        uint64_t now, motion_command_status_t * p_command)
{
    if ((0U == p_mission->command) ||
        (!nav_get_command(p_mission->command, p_command)))
    {
        hold(p_mission, B5MI_R_MOVE, 0U, now);
        return -1;
    }
    if (MOTION_CMD_RUNNING == p_command->state)
    {
        if ((!p_mission->halting_move) &&
            (status.generation != p_mission->generation))
        {
            hold(p_mission, B5MI_R_CANCELLED, 0U, now);
            return -1;
        }
        return 0;
    }
    bool ours = (MOTION_CMD_DONE == p_command->state) ||
                ((p_mission->halting_move) &&
                 (MOTION_CMD_CANCELLED == p_command->state));
    if (!ours)
    {
        hold(p_mission, B5MI_R_MOVE, p_command->state, now);
        return -1;
    }
    if (!still(status))
    {
        return 0;
    }
    p_mission->generation = status.generation;
    p_mission->command = 0U;
    return 1;
}

/* Track completed movement in the initial width-reference frame. */
static void update_route_pose (b5_mission_t * p_mission,
                               const motion_command_status_t * p_command)
{
    float midpoint =
        (p_mission->align_heading_deg + 0.5f * p_command->rotation_deg) *
        (3.14159265358979323846f / 180.0f);
    p_mission->route_lateral_mm += p_command->progress_mm * sinf(midpoint);
    p_mission->route_forward_mm += p_command->progress_mm * cosf(midpoint);
    p_mission->align_heading_deg += p_command->rotation_deg;
}

static void halt_move (b5_mission_t * p_mission)
{
    if (!p_mission->halting_move)
    {
        (void)nav_request_stop(MOTION_BUDDY5);
        p_mission->halting_move = true;
    }
}

/* v0.6.1 progress rule applied to the mission's own moves. */
static bool progress_ok (b5_mission_t * p_mission, motion_status_t status,
                         uint64_t now)
{
    if (((status.mode != MOTION_DRIVING) && (status.mode != MOTION_TURNING)) ||
        ((0 == status.target_mm_s[0]) && (0 == status.target_mm_s[1])))
    {
        p_mission->progress_id = 0U;
        return true;
    }
    float progress = MOTION_TURNING == status.mode ? status.rotation_deg
                                                   : status.progress_mm;
    if ((p_mission->progress_id != status.command_id) ||
        (fabsf(progress - p_mission->last_progress) >= 1.0f))
    {
        p_mission->progress_id = status.command_id;
        p_mission->last_progress = progress;
        p_mission->progress_us = now;
    }
    if ((now - p_mission->progress_us) >= (uint64_t)B5MI_STALL_US)
    {
        hold(p_mission, B5MI_R_STALL, 0U, now);
        return false;
    }
    return true;
}

/* ---- Stages ---------------------------------------------------------------
 */

static void reset_results (b5_mission_t * p_mission)
{
    p_mission->width_d0 = 0U;
    p_mission->width_end = 0U;
    p_mission->width_attempted = false;
    p_mission->direction_side = B5_SIDE_NONE;
    p_mission->direction_basis = B5MI_DIR_NONE;
    p_mission->direction_decision = B5EP_DECISION_HOLD;
    p_mission->direction_end = 0U;
    p_mission->direction_states = 0U;
    p_mission->direction_attempted = false;
    p_mission->bypass_range_mm = 0U;
    p_mission->approach_legs = 0U;
    p_mission->approach_start_mm = 0U;
    p_mission->approach_retried = false;
    p_mission->approach_line_lost = false;
    p_mission->width_started = false;
    p_mission->width_rechecked = false;
    p_mission->width_policy = B5MI_WIDTH_NONE;
    p_mission->width_gate_mm = 0U;
    p_mission->steer_mm = 0.0f;
    p_mission->steer_decideg = 0;
    p_mission->steer_max_decideg = 0U;
    p_mission->steer_saturated = 0U;
    p_mission->align_steps = 0U;
    p_mission->align_for_bypass = false;
    p_mission->align_heading_deg = 0.0f;
    p_mission->route_lateral_mm = 0.0f;
    p_mission->route_forward_mm = 0.0f;
    p_mission->recenter_mm = 0.0f;
    p_mission->reason = B5MI_R_NONE;
    p_mission->module_reason = 0U;
}

static void guard_takeover (b5_mission_t * p_mission, uint16_t range_mm,
                            bool recheck, uint64_t now)
{
    uint32_t lease = 0U;
    if (MOTION_OK != nav_request_takeover(MOTION_BUDDY5, &lease))
    {
        hold(p_mission, B5MI_R_LEASE, 0U, now);
        return;
    }
    if (recheck)
    {
        retry_note(p_mission, B5_RETRY_STRAIGHT_STOPPED, B5_RETRY_REQUESTED,
                   B5MI_R_NONE, now);
    }
    else if (p_mission->guard_grace_since != 0U)
    {
        retry_note(p_mission, B5_RETRY_STRAIGHT_GRACE, B5_RETRY_RECOVERED,
                   B5MI_R_NONE, now);
    }
    p_mission->guard_grace_since = 0U;
    p_mission->guard_rechecking = recheck;
    reset_results(p_mission);
    p_mission->lease = lease;
    p_mission->trigger_mm = range_mm;
    p_mission->run++;
    p_mission->sensor.cancel();
    enter(p_mission, B5MI_HALTING, now);
}

static void guard_tick (b5_mission_t * p_mission, motion_status_t status,
                        uint64_t now)
{
    b5_ping_t ping;
    bool following =
        (MOTION_AUTO == status.mode) && (MOTION_BUDDY3 == status.owner);
    /* Straight-only takeover gate, evaluated while Buddy3 still owns the
     * motion. Losing it discards every qualifying echo counted so far. */
    uint8_t gate = b5_straight_check(&p_mission->straight, now);
    bool eligible = (B5S_STRAIGHT == gate);
    p_mission->gate_reason = gate;
    /* During a junction turn the sonar points across the track. Start a
     * fresh guard scan once Buddy3 has finished turning and rejoined. */
    if (B5S_TURN == gate)
    {
        if (p_mission->ping_pending)
        {
            p_mission->sensor.cancel();
            p_mission->ping_pending = false;
        }
        p_mission->confirm = 0U;
        p_mission->close_count = 0U;
        p_mission->guard_grace_since = 0U;
        p_mission->clear_timing = false;
        p_mission->fault_run = 0U;
        p_mission->last_ping = now;
        return;
    }
    bool taken = watch_ping(p_mission, now, &ping);
    if (!eligible)
    {
        p_mission->confirm = 0U;
    }
    if (!following)
    {
        p_mission->confirm = 0U;
        p_mission->close_count = 0U;
        return;
    }
    if (taken)
    {
        bool echo = (B5_PING_ECHO == ping.cls);
        bool close = (echo) && (ping.range_mm <= (uint32_t)B5MI_CLOSE_HOLD_MM);
        bool in_range =
            (echo) && (ping.range_mm <= (uint32_t)B5MI_CONFIRM_RANGE_MM);
        if (p_mission->guard_grace_since != 0U &&
            ((B5_PING_NO_RETURN == ping.cls) || (echo && !in_range)))
        {
            retry_note(p_mission, B5_RETRY_STRAIGHT_GRACE, B5_RETRY_RECOVERED,
                       B5MI_R_NONE, now);
            p_mission->guard_grace_since = 0U;
        }
        if (p_mission->rearm)
        {
            bool clear = (B5_PING_FAULT != ping.cls) &&
                         ((B5_PING_NO_RETURN == ping.cls) || (!in_range));
            if (!clear)
            {
                p_mission->clear_timing = false;
            }
            else if (!p_mission->clear_timing)
            {
                p_mission->clear_timing = true;
                p_mission->clear_since = now;
            }
            else if ((now - p_mission->clear_since) >=
                     (uint64_t)B5MI_REARM_CLEAR_US)
            {
                p_mission->rearm = false;
            }
            if (close)
            {
                /* v0.6.1 rule: an obstacle inside the threshold while
                 * re-arming stops the robot rather than starting a probe. */
                hold(p_mission, B5MI_R_SENSOR, 0U, now);
                return;
            }
        }
        else
        {
            /* Only a ping REQUESTED after the gate opened qualifies, so an
             * echo from before (or across) a curve never carries over. */
            bool fresh = (eligible) && (p_mission->taken_requested >=
                                        p_mission->straight.open_us);
            p_mission->close_count = close ? p_mission->close_count + 1U : 0U;
            if ((!in_range) || (!fresh))
            {
                p_mission->confirm = 0U;
            }
            else if (p_mission->confirm < (unsigned)B5MI_TRIGGER_CONFIRM)
            {
                p_mission->confirm++;
            }
            if ((!eligible) &&
                (p_mission->close_count >= (unsigned)B5MI_TRIGGER_CONFIRM))
            {
                if (B5S_SETTLING != gate)
                {
                    hold(p_mission, B5MI_R_NOT_STRAIGHT, gate, now);
                }
                else if ((ping.range_mm <= B5MI_GRACE_MIN_MM) ||
                         ((p_mission->guard_grace_since != 0U) &&
                          ((now - p_mission->guard_grace_since) >=
                           B5MI_GRACE_US)))
                {
                    guard_takeover(p_mission, (uint16_t)ping.range_mm, true,
                                   now);
                }
                else if (0U == p_mission->guard_grace_since)
                {
                    p_mission->guard_grace_since = now;
                    p_mission->guard_grace_range_mm = (uint16_t)ping.range_mm;
                    retry_note(p_mission, B5_RETRY_STRAIGHT_GRACE,
                               B5_RETRY_REQUESTED, B5MI_R_NONE, now);
                }
                return;
            }
            if ((p_mission->confirm >= (unsigned)B5MI_TRIGGER_CONFIRM) &&
                (ping.range_mm <= (uint32_t)B5MI_STRAIGHT_STOP_MM))
            {
                guard_takeover(p_mission, (uint16_t)ping.range_mm, false, now);
                return;
            }
        }
    }
    unsigned failure = watch_failure(p_mission, now);
    if (failure != 0U)
    {
        hold(p_mission, B5MI_R_SENSOR, failure, now);
    }
    else if ((B5S_SETTLING == gate) &&
             (p_mission->close_count >= B5MI_TRIGGER_CONFIRM) &&
             (p_mission->guard_grace_since != 0U) &&
             ((now - p_mission->guard_grace_since) >= B5MI_GRACE_US))
    {
        guard_takeover(p_mission, p_mission->guard_grace_range_mm, true, now);
    }
}

static void width_begin (b5_mission_t * p_mission, uint64_t now)
{
    drain_width(p_mission);
    if (!b5_wp_begin(&p_mission->width, now))
    {
        hold(p_mission, B5MI_R_WIDTH_BUSY, 0U, now);
        return;
    }
    if (p_mission->width.lease != p_mission->lease)
    {
        b5_wp_cancel(&p_mission->width, now);
        hold(p_mission, B5MI_R_LEASE, 0U, now);
        return;
    }
    p_mission->width_attempted = true;
    p_mission->width.allow_borderline_ref =
        (B5S_SETTLING == p_mission->gate_reason) &&
        (p_mission->range.nearest_mm < B5WP_D0_MIN_MM);
    p_mission->width_started = true;
    p_mission->width_policy = B5MI_WIDTH_RUNNING;
    enter(p_mission, B5MI_WIDTH, now);
}

/* Width policy (review 2026-11-19): the frozen probe is started only when a
 * fresh CLEAN triplet (three echoes within 15 mm) lies inside its own REF
 * window. A clean range outside that window is a deliberate, labelled skip
 * with no motion. Once started, any non-COMPLETE end (REFERENCE included)
 * holds; its D3 150 mm assumption is still displayed. */
static void width_decide (b5_mission_t * p_mission, uint64_t now)
{
    uint16_t range = p_mission->range.range_mm;
    p_mission->width_gate_mm = range;
    if ((range >= (uint16_t)B5WP_D0_MIN_MM) &&
        (range <= (uint16_t)B5WP_D0_MAX_MM))
    {
        width_begin(p_mission, now);
        if (p_mission->width_rechecked && (B5MI_WIDTH == p_mission->stage))
        {
            retry_note(p_mission, B5_RETRY_WIDTH_GATE, B5_RETRY_RECOVERED,
                       B5MI_R_NONE, now);
        }
        return;
    }
    if ((!p_mission->width_rechecked) &&
        (range >= B5WP_D0_MIN_MM - B5MI_WIDTH_RECHECK_MARGIN_MM) &&
        (range <= B5WP_D0_MAX_MM + B5MI_WIDTH_RECHECK_MARGIN_MM))
    {
        p_mission->width_rechecked = true;
        retry_note(p_mission, B5_RETRY_WIDTH_GATE, B5_RETRY_REQUESTED,
                   B5MI_R_NONE, now);
        range_begin(p_mission, B5MI_WIDTH, now);
        p_mission->range.retry_after_us = now + B5MI_RANGE_RETRY_US;
        return;
    }
    p_mission->width_policy = B5MI_WIDTH_SKIPPED;
    p_mission->direction_basis = B5MI_DIR_HELD;
    hold(p_mission, B5MI_R_DIRECTION_UNKNOWN, 0U, now);
}

/* The outermost stop is not revisited. Use the adjacent stop only when
 * the probe still saw the obstacle at the outermost stop and at the adjacent
 * stop in both directions. Return lateral reach outward from the axis. */
static float corroborated_reach (const B5WPSide * p_side, float sign)
{
    if ((B5WP_ST_NOT_REACHED != p_side->state) || (!p_side->return_ok) ||
        (p_side->last <= B5WP_STOP_PRE))
    {
        return 0.0f;
    }
    const B5WPStop * p_far = &p_side->stop[p_side->last];
    const B5WPStop * p_revisit = &p_side->stop[p_side->last - 1U];
    if ((!p_far->visited) || (B5WP_IN != p_far->out_cls) ||
        (!p_revisit->visited) || (!p_revisit->revisited) ||
        (B5WP_IN != p_revisit->out_cls) || (B5WP_IN != p_revisit->ret_cls))
    {
        return 0.0f;
    }
    return fminf(sign * p_revisit->out_axis, sign * p_revisit->ret_axis);
}

/* Decide using scan evidence independently of the calibrated display number.
 * Two corroborated edges bound the centre. An unresolved far edge only
 * suffices if the return pass confirms it extends well beyond the near edge. */
static bool choose_width_direction (b5_mission_t * p_mission)
{
    const B5WPSummary * p_scan = &p_mission->width.summary;
    const B5WPSide * p_left = &p_mission->width.side[0];
    const B5WPSide * p_right = &p_mission->width.side[1];
    float left_reach = corroborated_reach(p_left, -1.0f);
    float right_reach = corroborated_reach(p_right, 1.0f);
    const uint8_t left_state = B5WP_ST_BOUNDARY == p_scan->state_l
                                   ? B5EP_SIDE_CROSSED
                               : left_reach > 0.0f ? B5EP_SIDE_REMAINS
                                                   : B5EP_SIDE_UNKNOWN;
    const uint8_t right_state = B5WP_ST_BOUNDARY == p_scan->state_r
                                    ? B5EP_SIDE_CROSSED
                                : right_reach > 0.0f ? B5EP_SIDE_REMAINS
                                                     : B5EP_SIDE_UNKNOWN;
    p_mission->direction_attempted = true;
    p_mission->direction_end = B5EP_END_COMPLETE;
    p_mission->direction_states = (uint8_t)(left_state | (right_state << 4));
    const uint16_t required = B5WP_FLAG_REF_OK | B5WP_FLAG_MID_OK |
                              B5WP_FLAG_FINAL_REF_OK | B5WP_FLAG_L_RETURN_OK |
                              B5WP_FLAG_R_RETURN_OK | B5WP_FLAG_SERVO_IN_RANGE;
    if ((p_scan->lost != 0U) || ((p_scan->flags & required) != required))
    {
        return false;
    }

    unsigned side = B5_SIDE_NONE;
    if ((B5WP_WIDTH_MEASURED == p_scan->width_result) &&
        (p_scan->l_in_mm10 <= 0) && (p_scan->r_in_mm10 >= 0))
    {
        /* Edge coordinates are +RIGHT, in tenths of a millimetre.
         * The sum is twice the centre; multiply the mm thresholds by 20. */
        int centre_lower = (int)p_scan->l_far_mm10 + (int)p_scan->r_in_mm10;
        int centre_upper = (int)p_scan->l_in_mm10 + (int)p_scan->r_far_mm10;
        if (centre_lower > 20 * B5MI_DIRECTION_MARGIN_MM)
        {
            side = B5_SIDE_LEFT;
        }
        else if (centre_upper < -20 * B5MI_DIRECTION_MARGIN_MM)
        {
            side = B5_SIDE_RIGHT;
        }
    }
    else if ((B5EP_SIDE_CROSSED == left_state) && (p_left->far_x < 0.0f) &&
             (p_left->far_x < fmaxf(p_left->in_x, p_left->ret_in_x)) &&
             (fmaxf(p_left->in_x, p_left->ret_in_x) <= 0.0f) &&
             (right_reach + p_left->far_x > 2.0f * B5MI_DIRECTION_MARGIN_MM))
    {
        side = B5_SIDE_LEFT;
    }
    else if ((B5EP_SIDE_CROSSED == right_state) && (p_right->far_x > 0.0f) &&
             (p_right->far_x > fminf(p_right->in_x, p_right->ret_in_x)) &&
             (fminf(p_right->in_x, p_right->ret_in_x) >= 0.0f) &&
             (left_reach - p_right->far_x > 2.0f * B5MI_DIRECTION_MARGIN_MM))
    {
        side = B5_SIDE_RIGHT;
    }
    if (B5_SIDE_NONE == side)
    {
        p_mission->direction_side = B5MI_DEFAULT_SIDE;
        p_mission->direction_basis = B5MI_DIR_DEFAULT;
        return true;
    }

    p_mission->direction_side = (uint8_t)side;
    p_mission->direction_basis = B5MI_DIR_EVIDENCE;
    p_mission->direction_decision =
        (B5_SIDE_LEFT == side) ? B5EP_DECISION_LEFT : B5EP_DECISION_RIGHT;
    return true;
}

static void width_tick (b5_mission_t * p_mission, uint64_t now)
{
    b5_wp_tick(&p_mission->width, now);
    if (!b5_wp_ready(&p_mission->width))
    {
        return;
    }
    p_mission->width_end = (uint8_t)p_mission->width.end;
    p_mission->width_d0 = p_mission->width.d0;
    /* An attempted probe that did not complete holds, REFERENCE included:
     * END_REFERENCE alone is not evidence of a clean skip. The probe's D3
     * (150 mm PROJECT_RANGE_ASSUMPTION) remains the displayed number. */
    if (B5WP_END_COMPLETE != p_mission->width.end)
    {
        p_mission->width_policy = B5MI_WIDTH_FAILED;
        hold(p_mission, B5MI_R_WIDTH_ABORT, p_mission->width.end, now);
        return;
    }
    p_mission->width_policy = B5MI_WIDTH_MEASURED;
    if (!choose_width_direction(p_mission))
    {
        p_mission->direction_basis = B5MI_DIR_HELD;
        hold(p_mission, B5MI_R_DIRECTION_UNKNOWN, 0U, now);
        return;
    }
    p_mission->align_heading_deg = p_mission->width.heading_deg;
    p_mission->route_lateral_mm = -p_mission->width.net_x_mm;
    p_mission->route_forward_mm = p_mission->width.net_y_mm;
    p_mission->align_plane_mm = (float)p_mission->width_d0;
    servo_front(p_mission, now);
    enter(p_mission, B5MI_ALIGN, now);
}

enum
{
    ALIGN_START,
    ALIGN_HEADING,
    ALIGN_EVIDENCE,
    ALIGN_SEARCH,
    ALIGN_RECENTER_TURN,
    ALIGN_RECENTER_DRIVE
};

/* The stopped IR pattern is only usable near the original line heading. */
static bool aligned_on_line (const b5_mission_t * p_mission, bool for_bypass)
{
    return (fabsf(p_mission->align_heading_deg) <=
            (float)B5MI_ALIGN_HEADING_DEG) &&
           (for_bypass ? line_evidence(p_mission)
                       : (0U != line_mask(p_mission)));
}

static bool alignment_turn_clear (b5_mission_t * p_mission, uint64_t now)
{
    if (b5_wp_turn_front_gap(p_mission->align_plane_mm,
                             p_mission->route_forward_mm + B5WP_OVERSHOOT_MM) >=
        B5WP_FRONT_MARGIN_MM)
    {
        return true;
    }
    hold(p_mission,
         p_mission->approach_retried ? B5MI_R_NO_LINE_APPROACH
                                     : B5MI_R_NO_LINE_ALIGN,
         0U, now);
    return false;
}

static void align_tick (b5_mission_t * p_mission, motion_status_t status,
                        uint64_t now)
{
    motion_command_status_t command;
    bool observation = fresh_observation(p_mission);
    if ((now - p_mission->stage_since) > (uint64_t)B5MI_ALIGN_LIMIT_US)
    {
        hold(p_mission, B5MI_R_TIMEOUT, 0U, now);
        return;
    }
    switch (p_mission->sub)
    {
    case ALIGN_START:
    {
        if (!still(status))
        {
            return;
        }
        float residual = p_mission->align_heading_deg;
        if (fabsf(residual) * 100.0f > (float)B5MI_ALIGN_TRIM_MAX_CDEG)
        {
            hold(p_mission, B5MI_R_MOVE, 0U, now);
            return;
        }
        float drift = p_mission->route_lateral_mm;
        if ((!p_mission->approach_retried) &&
            (fabsf(drift) > (float)B5MI_ALIGN_DRIFT_MM))
        {
            /* A single short diagonal restores lateral position. Check the
             * worst turn footprint through the move plus overshoot. */
            p_mission->recenter_mm = fabsf(drift) * 1.41421356f;
            if ((p_mission->recenter_mm > (float)B5MI_ALIGN_RECENTER_MM) ||
                (b5_wp_turn_front_gap(
                     (float)p_mission->width_d0,
                     p_mission->route_forward_mm + p_mission->recenter_mm +
                         B5WP_OVERSHOOT_MM) < B5WP_FRONT_MARGIN_MM))
            {
                hold(p_mission, B5MI_R_NO_LINE_ALIGN, 0U, now);
                return;
            }
            float heading = (drift > 0.0f) ? -45.0f : 45.0f;
            if (start_move(p_mission, true, heading - residual,
                           B5MI_ALIGN_SPEED, now))
            {
                p_mission->sub = ALIGN_RECENTER_TURN;
            }
            return;
        }
        if (fabsf(residual) < 1.0f)
        {
            p_mission->sub = ALIGN_EVIDENCE;
            p_mission->sub_since = now;
            return;
        }
        if (!alignment_turn_clear(p_mission, now))
        {
            return;
        }
        if (start_move(p_mission, true, -residual, B5MI_ALIGN_SPEED, now))
        {
            p_mission->sub = ALIGN_HEADING;
        }
        return;
    }
    case ALIGN_HEADING:
    case ALIGN_SEARCH:
    case ALIGN_RECENTER_TURN:
    case ALIGN_RECENTER_DRIVE:
    {
        if (!progress_ok(p_mission, status, now))
        {
            return;
        }
        int done = follow_move(p_mission, status, now, &command);
        if (done <= 0)
        {
            return;
        }
        update_route_pose(p_mission, &command);
        if (ALIGN_RECENTER_TURN == p_mission->sub)
        {
            if ((fabsf(fabsf(p_mission->align_heading_deg) - 45.0f) > 3.0f) ||
                (p_mission->align_heading_deg * p_mission->route_lateral_mm >=
                 0.0f) ||
                (b5_wp_turn_front_gap(
                     (float)p_mission->width_d0,
                     p_mission->route_forward_mm + p_mission->recenter_mm +
                         B5WP_OVERSHOOT_MM) < B5WP_FRONT_MARGIN_MM))
            {
                hold(p_mission, B5MI_R_NO_LINE_ALIGN, 0U, now);
                return;
            }
            if (start_move(p_mission, false, p_mission->recenter_mm,
                           B5MI_ALIGN_SPEED, now))
            {
                p_mission->sub = ALIGN_RECENTER_DRIVE;
            }
            return;
        }
        if (ALIGN_RECENTER_DRIVE == p_mission->sub)
        {
            if ((!b5_wp_segment_ok(p_mission->recenter_mm,
                                   command.progress_mm)) ||
                (b5_wp_turn_front_gap((float)p_mission->width_d0,
                                      p_mission->route_forward_mm) <
                 B5WP_FRONT_MARGIN_MM))
            {
                hold(p_mission, B5MI_R_NO_LINE_ALIGN, 0U, now);
                return;
            }
            if (start_move(p_mission, true, -p_mission->align_heading_deg,
                           B5MI_ALIGN_SPEED, now))
            {
                p_mission->sub = ALIGN_HEADING;
            }
            return;
        }
        p_mission->sub = ALIGN_EVIDENCE;
        p_mission->sub_since = now;
        p_mission->evidence_timing = false;
        return;
    }
    default:
        break;
    }
    if (!still(status))
    {
        hold(p_mission, B5MI_R_MOVE, 0U, now);
        return;
    }
    if (observation)
    {
        if (!aligned_on_line(p_mission, p_mission->align_for_bypass))
        {
            p_mission->evidence_timing = false;
        }
        else if (!p_mission->evidence_timing)
        {
            p_mission->evidence_timing = true;
            p_mission->evidence_since = now;
        }
    }
    if ((p_mission->evidence_timing) &&
        ((now - p_mission->evidence_since) >= (uint64_t)B5MI_EVIDENCE_US))
    {
        if (p_mission->approach_retried)
        {
            retry_note(p_mission, B5_RETRY_APPROACH_ALIGN, B5_RETRY_RECOVERED,
                       B5MI_R_NONE, now);
        }
        range_begin(p_mission, B5MI_APPROACH, now);
        return;
    }
    if ((now - p_mission->sub_since) < (uint64_t)B5MI_EVIDENCE_WAIT_US)
    {
        return;
    }
    if (!alignment_turn_clear(p_mission, now))
    {
        return;
    }
    if (p_mission->align_steps >= (unsigned)B5MI_ALIGN_STEPS)
    {
        hold(p_mission,
             p_mission->approach_retried ? B5MI_R_NO_LINE_APPROACH
                                         : B5MI_R_NO_LINE_ALIGN,
             0U, now);
        return;
    }
    float goal =
        g_align_deg[p_mission->align_steps++] - p_mission->align_heading_deg;
    if (fabsf(goal) < 0.25f)
    {
        p_mission->sub_since = now;
        return;
    }
    /* The shared executor starts settling one degree short of its goal. */
    float turn = goal + copysignf(1.0f, goal);
    if (start_move(p_mission, true, turn, B5MI_ALIGN_SPEED, now))
    {
        p_mission->sub = ALIGN_SEARCH;
    }
}

/* Use Buddy3's symmetric wheel targets as a heading correction, scaled
 * for the slow approach. Keep the last heading through a brief white gap.
 * Buddy5 owns the distance, tape-loss and sonar stop decisions. */
static bool steer_step (b5_mission_t * p_mission, motion_status_t status,
                        bool tape_visible, uint64_t now)
{
    if ((MOTION_DRIVING != status.mode) || (now <= p_mission->steer_us))
    {
        p_mission->steer_us = now;
        return true;
    }
    float delta_time = (float)(now - p_mission->steer_us) / 1000000.0f;
    p_mission->steer_us = now;
    if (delta_time > 0.05f)
    {
        delta_time = 0.05f; /* never integrate across a stall/gap */
    }
    if (tape_visible)
    {
        int16_t target[2];
        int32_t error = (int32_t)p_mission->line[0] - p_mission->line[1];
        symmetric_targets(error, target);
        float scale = (float)B5MI_APPROACH_SPEED / (float)LINE_BASE_MM_S;
        p_mission->steer_mm +=
            (float)(target[1] - target[0]) * scale * delta_time;
    }
    float limit = (float)B5MI_STEER_MAX_DECIDEG / 10.0f * 110.0f *
                  3.14159265358979323846f / 180.0f;
    bool saturated = fabsf(p_mission->steer_mm) >= limit;
    if (p_mission->steer_mm > limit)
    {
        p_mission->steer_mm = limit;
    }
    else if (p_mission->steer_mm < -limit)
    {
        p_mission->steer_mm = -limit;
    }
    if ((saturated) && (p_mission->steer_saturated < 255U))
    {
        p_mission->steer_saturated++;
    }
    float degrees =
        p_mission->steer_mm * 180.0f / (110.0f * 3.14159265358979323846f);
    long decideg = lroundf(degrees * 10.0f);
    decideg = decideg > B5MI_STEER_MAX_DECIDEG    ? B5MI_STEER_MAX_DECIDEG
              : decideg < -B5MI_STEER_MAX_DECIDEG ? -B5MI_STEER_MAX_DECIDEG
                                                  : decideg;
    p_mission->steer_decideg = (int16_t)decideg; /* |decideg| <= 80 */
    unsigned magnitude = (unsigned)(decideg < 0 ? -decideg : decideg);
    if (magnitude > p_mission->steer_max_decideg)
    {
        p_mission->steer_max_decideg = (uint8_t)magnitude; /* <= 80 */
    }
    motion_result_t result = nav_steer_drive(MOTION_BUDDY5, p_mission->lease,
                                             (float)decideg / 10.0f);
    return (MOTION_OK == result) || (MOTION_BUSY == result);
}

static void bypass_begin (b5_mission_t * p_mission, uint64_t now);

/* Reuse the stopped alignment search once, without another diagonal drive.
 * The frontal range and the encoder model must both leave room to turn. */
static void approach_realign (b5_mission_t * p_mission, uint64_t now)
{
    if ((p_mission->approach_retried) ||
        (fabsf(p_mission->align_heading_deg) * 100.0f >
         (float)B5MI_ALIGN_TRIM_MAX_CDEG))
    {
        hold(p_mission, B5MI_R_NO_LINE_APPROACH, 0U, now);
        return;
    }
    p_mission->approach_retried = true;
    retry_note(p_mission, B5_RETRY_APPROACH_ALIGN, B5_RETRY_REQUESTED,
               B5MI_R_NONE, now);
    /* Project the stopped echo into the original forward axis. The 0.97
     * factor rounds cos(12 degrees) down over the allowed heading error. */
    float stopped_range =
        0.97f * ((float)p_mission->range.nearest_mm + B5WP_SHAFT_MM) -
        B5WP_SHAFT_MM;
    p_mission->align_plane_mm = fminf(
        p_mission->align_plane_mm, p_mission->route_forward_mm + stopped_range);
    if (!alignment_turn_clear(p_mission, now))
    {
        return;
    }
    p_mission->approach_line_lost = false;
    p_mission->align_steps = 0U;
    p_mission->align_for_bypass =
        p_mission->range.range_mm <= (uint16_t)B5MI_APPROACH_ACCEPT_MM;
    enter(p_mission, B5MI_ALIGN, now);
}

/* Entered with a fresh stationary range. sub 0 decide, 1 driving, 2 halting
 * on range, 3 halting on lost line, 4 halting on sonar failure. */
static void approach_tick (b5_mission_t * p_mission, motion_status_t status,
                           uint64_t now)
{
    motion_command_status_t command;
    if ((now - p_mission->stage_since) > (uint64_t)B5MI_APPROACH_LIMIT_US)
    {
        hold(p_mission, B5MI_R_TIMEOUT, 0U, now);
        return;
    }
    if (0U == p_mission->sub)
    {
        uint16_t range = p_mission->range.range_mm;
        bool bypass_range = range <= (uint16_t)B5MI_APPROACH_ACCEPT_MM;
        bool aligned = aligned_on_line(p_mission, bypass_range);
        if ((p_mission->approach_line_lost) || (!aligned))
        {
            approach_realign(p_mission, now);
            return;
        }
        if (0U == p_mission->approach_start_mm)
        {
            p_mission->approach_start_mm = range;
        }
        if (bypass_range)
        {
            bypass_begin(p_mission, now);
            return;
        }
        if (p_mission->approach_legs >= (unsigned)B5MI_APPROACH_MAX_LEGS)
        {
            hold(p_mission, B5MI_R_APPROACH_RANGE, 0U, now);
            return;
        }
        float goal = (float)range - (float)B5MI_APPROACH_TARGET_MM;
        if (goal > (float)B5MI_APPROACH_MAX_LEG_MM)
        {
            hold(p_mission, B5MI_R_APPROACH_RANGE, 0U, now);
            return;
        }
        goal = fminf(goal, (float)p_mission->range.nearest_mm -
                               (float)B5MI_APPROACH_TARGET_MM);
        p_mission->approach_legs++;
        p_mission->approach_last_line_mm = 0.0f;
        p_mission->steer_mm = 0.0f;
        p_mission->steer_us = now;
        p_mission->fault_run = 0U;
        p_mission->last_ping = now;
        if (start_move(p_mission, false, goal, B5MI_APPROACH_SPEED, now))
        {
            p_mission->sub = 1U;
        }
        return;
    }
    if (!progress_ok(p_mission, status, now))
    {
        return;
    }
    int done = follow_move(p_mission, status, now, &command);
    if (done < 0)
    {
        return;
    }
    if (done > 0)
    {
        update_route_pose(p_mission, &command);
        if ((3U == p_mission->sub) && p_mission->approach_retried)
        {
            hold(p_mission, B5MI_R_NO_LINE_APPROACH, 0U, now);
        }
        else if (4U == p_mission->sub)
        {
            hold(p_mission, B5MI_R_SENSOR, 0U, now);
        }
        else
        {
            range_begin(p_mission, B5MI_APPROACH, now);
        }
        return;
    }
    if (p_mission->sub != 1U)
    {
        return; /* our own stop is settling */
    }
    if (fresh_observation(p_mission))
    {
        bool startup =
            p_mission->approach_last_line_mm < (float)B5MI_APPROACH_VERIFY_MM;
        bool seen = 0U != line_mask(p_mission);
        float loss_limit = startup ? (float)B5MI_APPROACH_START_LOSS_MM
                                   : (float)B5MI_APPROACH_NO_LINE_MM;
        if (seen)
        {
            p_mission->approach_last_line_mm = status.progress_mm;
        }
        else if ((status.progress_mm - p_mission->approach_last_line_mm) >
                 loss_limit)
        {
            halt_move(p_mission);
            p_mission->approach_line_lost = true;
            p_mission->sub = 3U;
            return;
        }
        if ((p_mission->steering) &&
            (!steer_step(p_mission, status, seen, now)))
        {
            halt_move(p_mission);
            hold(p_mission, B5MI_R_MOVE, 0U, now);
            return;
        }
    }
    b5_ping_t ping;
    if (watch_ping(p_mission, now, &ping) && (B5_PING_ECHO == ping.cls) &&
        (ping.range_mm <= (uint32_t)B5MI_APPROACH_STOP_MM))
    {
        halt_move(p_mission);
        p_mission->sub = 2U;
        return;
    }
    if (watch_failure(p_mission, now) != 0U)
    {
        halt_move(p_mission);
        p_mission->sub = 4U;
    }
}

static void bypass_begin (b5_mission_t * p_mission, uint64_t now)
{
    uint16_t range = p_mission->range.range_mm;
    p_mission->bypass_range_mm = range;
    float heading0 = p_mission->align_heading_deg;
    float lateral0 = p_mission->route_lateral_mm;
    if (!b5_auto_start(&p_mission->bypass, p_mission->lease,
                       p_mission->direction_side, range,
                       p_mission->range.nearest_mm,
                       p_mission->range.farthest_mm, heading0, lateral0, now))
    {
        hold(p_mission,
             B5A_BLOCKED == p_mission->bypass.state ? B5MI_R_BYPASS_BLOCKED
                                                    : B5MI_R_BYPASS_FAULT,
             p_mission->bypass.reason, now);
        return;
    }
    enter(p_mission, B5MI_BYPASS, now);
}

static void bypass_tick (b5_mission_t * p_mission, uint64_t now)
{
    b5_auto_tick(&p_mission->bypass, now);
    if (B5A_DONE == p_mission->bypass.state)
    {
        /* Handed back: line following owns the robot again. Re-arm only
         * after the front has been clear for 1 s (v0.6.1 rule). */
        p_mission->rearm = true;
        p_mission->clear_timing = false;
        p_mission->lease = 0U;
        p_mission->fault_run = 0U;
        p_mission->last_ping = now;
        servo_front(p_mission, now);
        enter(p_mission, B5MI_GUARD, now);
        return;
    }
    if ((B5A_BLOCKED == p_mission->bypass.state) ||
        (B5A_FAULT == p_mission->bypass.state))
    {
        hold(p_mission,
             B5A_BLOCKED == p_mission->bypass.state ? B5MI_R_BYPASS_BLOCKED
                                                    : B5MI_R_BYPASS_FAULT,
             p_mission->bypass.reason, now);
    }
}

/* ---- Public ---------------------------------------------------------------
 */

void b5_mission_init (b5_mission_t * p_mission, const B5SensorPort * p_port,
                      int (*p_applied)(void))
{
    memset(p_mission, 0, sizeof(*p_mission));
    p_mission->sensor = *p_port;
    b5_wp_init(&p_mission->width, p_port);
    p_mission->width.applied = p_applied;
    b5_auto_init(&p_mission->bypass, p_port);
    p_mission->stage = B5MI_IDLE;
    p_mission->steering = true;
}

bool b5_mission_begin (b5_mission_t * p_mission, uint64_t now)
{
    if (b5_mission_active(p_mission))
    {
        return false;
    }
    if (!b5_wp_ready(&p_mission->width))
    {
        b5_wp_cancel(&p_mission->width, now);
    }
    p_mission->session = true;
    p_mission->rearm = false;
    p_mission->clear_timing = false;
    p_mission->guard_grace_since = 0U;
    p_mission->guard_rechecking = false;
    p_mission->lease = 0U;
    p_mission->fault_run = 0U;
    p_mission->last_ping = now;
    p_mission->observed = now;
    p_mission->used_observation = 0U;
    reset_results(p_mission);
    b5_straight_reset(&p_mission->straight, B5S_UNKNOWN);
    p_mission->gate_reason = B5S_UNKNOWN;
    p_mission->close_count = 0U;
    servo_front(p_mission, now);
    enter(p_mission, B5MI_GUARD, now);
    return true;
}

void b5_mission_line (b5_mission_t * p_mission,
                      const b5_line_sample_t * p_sample)
{
    b5_straight_step(&p_mission->straight, p_sample);
}

void b5_mission_guard_limit (const b5_mission_t * p_mission, int16_t target[2])
{
    if ((B5MI_GUARD != p_mission->stage) ||
        (0U == p_mission->guard_grace_since) ||
        (p_mission->close_count < B5MI_TRIGGER_CONFIRM))
    {
        return;
    }
    int32_t left = target[0];
    int32_t right = target[1];
    int32_t left_abs = (left < 0) ? -left : left;
    int32_t right_abs = (right < 0) ? -right : right;
    int32_t peak = (left_abs > right_abs) ? left_abs : right_abs;
    if (peak > B5MI_GRACE_SPEED)
    {
        /* Scale both targets together so the follower keeps steering. */
        target[0] = (int16_t)((left * B5MI_GRACE_SPEED) / peak);
        target[1] = (int16_t)((right * B5MI_GRACE_SPEED) / peak);
    }
}

void b5_mission_observe (b5_mission_t * p_mission, uint64_t now, uint16_t left,
                         uint16_t right)
{
    p_mission->observed = now;
    p_mission->line[0] = left;
    p_mission->line[1] = right;
    b5_auto_observe(&p_mission->bypass, now, left, right);
}

bool b5_mission_active (const b5_mission_t * p_mission)
{
    switch (p_mission->stage)
    {
    case B5MI_HALTING:
    case B5MI_WIDTH:
    case B5MI_ALIGN:
    case B5MI_APPROACH:
    case B5MI_RANGE:
    case B5MI_BYPASS:
    case B5MI_HOLD:
        return true;
    default:
        return false;
    }
}

void b5_mission_cancel (b5_mission_t * p_mission, uint64_t now)
{
    bool active = b5_mission_active(p_mission);
    if (!b5_wp_ready(&p_mission->width))
    {
        b5_wp_cancel(&p_mission->width, now);
    }
    if (p_mission->bypass.session)
    {
        b5_auto_cancel(&p_mission->bypass, now);
    }
    motion_status_t status = nav_get_status();
    if ((status.session_active) && (MOTION_BUDDY5 == status.owner) &&
        (status.mode != MOTION_FAULT))
    {
        (void)nav_request_stop(MOTION_BUDDY5);
    }
    servo_front(p_mission, now);
    p_mission->session = false;
    if (active || (B5MI_GUARD == p_mission->stage))
    {
        if (B5MI_HOLD != p_mission->stage)
        {
            p_mission->reason = B5MI_R_CANCELLED;
        }
        enter(p_mission, active ? B5MI_CANCELLED : B5MI_IDLE, now);
    }
}

void b5_mission_tick (b5_mission_t * p_mission, uint64_t now)
{
    if (!p_mission->session)
    {
        return;
    }
    motion_status_t status = nav_get_status();
    if (!status.session_active)
    {
        b5_mission_cancel(p_mission, now);
        return;
    }
    switch (p_mission->stage)
    {
    case B5MI_GUARD:
        guard_tick(p_mission, status, now);
        return;
    case B5MI_WIDTH:
        width_tick(p_mission, now);
        return;
    case B5MI_BYPASS:
        bypass_tick(p_mission, now);
        return;
    case B5MI_HALTING:
    case B5MI_ALIGN:
    case B5MI_APPROACH:
    case B5MI_RANGE:
        break;
    default:
        return; /* IDLE, HOLD, CANCELLED: stopped, nothing to step */
    }
    if (!owns(p_mission, status))
    {
        hold(p_mission, B5MI_R_LEASE, 0U, now);
        return;
    }
    if ((now < p_mission->observed) ||
        ((now - p_mission->observed) > (uint64_t)B5MI_OBSERVATION_MAX_US))
    {
        hold(p_mission, B5MI_R_STALE, 0U, now);
        return;
    }
    if (B5MI_HALTING == p_mission->stage)
    {
        if ((now - p_mission->stage_since) > (uint64_t)B5MI_HALT_LIMIT_US)
        {
            hold(p_mission, B5MI_R_TIMEOUT, 0U, now);
        }
        else if (still(status))
        {
            /* Decide the width policy from a fresh stationary triplet; a
             * faulty or inconsistent triplet holds (RANGE), never skips. */
            p_mission->takeover_mask = (uint8_t)line_mask(p_mission);
            range_begin(p_mission,
                        p_mission->guard_rechecking ? B5MI_GUARD : B5MI_WIDTH,
                        now);
        }
        return;
    }
    if (B5MI_ALIGN == p_mission->stage)
    {
        align_tick(p_mission, status, now);
        return;
    }
    if (B5MI_APPROACH == p_mission->stage)
    {
        approach_tick(p_mission, status, now);
        return;
    }
    range_tick(p_mission, status, now);
    if ((B5MI_WIDTH == p_mission->stage) && (!p_mission->width_started))
    {
        width_decide(p_mission, now);
    }
}

const char * b5_mission_stage_name (unsigned stage)
{
    static const char * const names[B5MI_STAGE_COUNT] = {
        "IDLE",      "GUARD",  "HALTING", "WIDTH",     "ALIGN", "APPROACH",
        "DIRECTION", "BYPASS", "HOLD",    "CANCELLED", "RANGE"};
    return stage < (unsigned)B5MI_STAGE_COUNT ? names[stage] : "INVALID";
}

const char * b5_mission_reason_name (unsigned reason)
{
    static const char * const names[B5MI_R_COUNT] = {"NONE",
                                                     "CANCELLED",
                                                     "SENSOR",
                                                     "LEASE",
                                                     "MOVE",
                                                     "TIMEOUT",
                                                     "STALE",
                                                     "STALL",
                                                     "WIDTH_ABORT",
                                                     "WIDTH_BUSY",
                                                     "NO_LINE_ALIGN",
                                                     "NO_LINE_APPROACH",
                                                     "APPROACH_RANGE",
                                                     "DIRECTION_ABORT",
                                                     "DIRECTION_UNKNOWN",
                                                     "DIRECTION_BUSY",
                                                     "RANGE",
                                                     "BYPASS_BLOCKED",
                                                     "BYPASS_FAULT",
                                                     "NOT_STRAIGHT"};
    return reason < (unsigned)B5MI_R_COUNT ? names[reason] : "INVALID";
}

/* End of file. */
